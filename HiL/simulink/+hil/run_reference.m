function result = run_reference(cell_cfg, pack_cfg, sim_cfg, params, profile)
%RUN_REFERENCE Generic discrete electrothermal 2RC pack reference plant.

[cell_cfg, ~] = hil.validate_configuration(cell_cfg);
[pack_cfg, ~] = hil.validate_configuration(pack_cfg);
[sim_cfg, ~] = hil.validate_configuration(sim_cfg);
pack_cfg = hil.derive_pack(cell_cfg, pack_cfg);
hil.validate_parameters(cell_cfg, params);
pack_cfg.initial_soc = double(sim_cfg.initial_soc);

if nargin < 5 || isempty(profile)
    profile = hil.load_profile(sim_cfg, cell_cfg, pack_cfg);
end

time = double(profile.time_s(:));
input_current = double(profile.pack_current_A(:));
ambient = double(profile.ambient_temperature_C(:));
if numel(time) ~= numel(input_current) || numel(time) ~= numel(ambient)
    error('hil:run:ProfileDimensions', 'Profile vectors must have equal lengths.');
end
sample_time = double(sim_cfg.sample_time_s);
if numel(time) < 2 || max(abs(diff(time) - sample_time)) > 1e-8
    error('hil:run:ProfileSampleTime', ...
        'Profile must be uniformly sampled at sim_cfg.sample_time_s.');
end

Ns = double(pack_cfg.series_groups);
Np = double(pack_cfg.parallel_cells);
group_params = hil.build_group_parameters(cell_cfg, pack_cfg);
soc = group_params.initial_soc;
vp1 = zeros(Ns, 1);
vp2 = zeros(Ns, 1);
core = repmat(double(pack_cfg.initial_temperature_C), Ns, 1);
surface = core;

decimation = 1;
if isfield(sim_cfg, 'output_decimation')
    decimation = max(1, round(double(sim_cfg.output_decimation)));
end
stored_indices = unique([1:decimation:numel(time), numel(time)]); %#ok<NBRAK>
stored_count = numel(stored_indices);
store_lookup = zeros(numel(time), 1);
store_lookup(stored_indices) = 1:stored_count;

result = allocate_result(stored_count, Ns, pack_cfg);
previous_pack_voltage = pack_cfg.nominal_voltage_V;

for step_index = 1:numel(time)
    pack_current = input_current(step_index);
    if isfield(profile, 'pack_power_W') && ...
            strcmpi(profile.scaling_mode, 'constant_pack_power')
        pack_current = double(profile.pack_power_W(step_index)) / ...
            max(abs(previous_pack_voltage), pack_cfg.minimum_pack_voltage_V);
    end
    if ~(isfield(profile, 'inputs_resolved') && ...
            logical(profile.inputs_resolved))
        pack_current = pack_current + double(sim_cfg.current_bias_A);
    end
    cell_current = pack_current / Np;

    r0 = lookup_table(params.R0, params, soc, core) .* ...
        group_params.r0_multiplier;
    r1 = lookup_table(params.R1, params, soc, core) .* ...
        group_params.r1_multiplier;
    c1 = lookup_table(params.C1, params, soc, core) .* ...
        group_params.c1_multiplier;
    r2 = double(params.R2) .* group_params.r2_multiplier;
    c2 = double(params.C2) .* group_params.c2_multiplier;
    ocv = lookup_ocv(params.OCV, params, soc, core);

    group_voltage = ocv - cell_current .* r0 - vp1 - vp2;
    physical = struct( ...
        'V_group', group_voltage, ...
        'SoC_group', soc, ...
        'T_core_group', core, ...
        'T_surf_group', surface, ...
        'Vp1_group', vp1, ...
        'Vp2_group', vp2);
    output = hil.expand_pack_outputs(physical, pack_cfg);
    output = hil.apply_fault_injection( ...
        output, sim_cfg.fault_injection, time(step_index), pack_cfg);
    previous_pack_voltage = output.V_pack;

    storage_index = store_lookup(step_index);
    if storage_index > 0
        result = store_output(result, storage_index, time(step_index), ...
            pack_current, ambient(step_index), output);
    end

    generated_heat = cell_current .^ 2 .* r0 + ...
        (vp1 .^ 2) ./ r1 + (vp2 .^ 2) ./ r2;
    rcs = double(params.Rcs);
    rsa = double(params.Rsa) * ...
        double(pack_cfg.cooling_boundary.Rsa_multiplier) .* ...
        group_params.rsa_multiplier;
    core_to_surface = (core - surface) ./ rcs;

    soc = min(max(soc - sample_time * cell_current ./ ...
        (3600.0 * double(params.Q_nom) .* ...
         group_params.capacity_multiplier), 0), 1);
    vp1 = vp1 + sample_time * ...
        (cell_current ./ c1 - vp1 ./ (r1 .* c1));
    vp2 = vp2 + sample_time * ...
        (cell_current ./ c2 - vp2 ./ (r2 .* c2));
    core = core + sample_time * ...
        (generated_heat - core_to_surface) / double(params.Cc);
    surface = surface + sample_time * ...
        (core_to_surface - (surface - ambient(step_index)) ./ rsa) / ...
        double(params.Cs);
end

result.engine = 'matlab_reference';
result.profile = profile;
result.profile.pack_current_A = [];
result.profile.ambient_temperature_C = [];
if isfield(result.profile, 'pack_power_W')
    result.profile.pack_power_W = [];
end
result.cell_configuration = cell_cfg;
result.pack_configuration = pack_cfg;
result.simulation_configuration = sim_cfg;
result.parameter_hash = params.configuration_hash;
result.configuration_hash = hil.configuration_hash( ...
    cell_cfg, pack_cfg, sim_cfg, params.configuration_hash);
result.group_parameters = group_params;
result.measurements = add_measurement_effects(result, sim_cfg);
end

function values = lookup_table(table_data, params, soc, temperature)
temperature = min(max(temperature, min(double(params.temp_bp))), ...
    max(double(params.temp_bp)));
soc = min(max(soc, min(double(params.soc_common))), ...
    max(double(params.soc_common)));
values = interp2(double(params.temp_bp(:).'), ...
    double(params.soc_common(:)), double(table_data), ...
    temperature, soc, 'linear');
end

function values = lookup_ocv(table_data, params, soc, temperature)
temperature = min(max(temperature, min(double(params.temp_bp_ocv))), ...
    max(double(params.temp_bp_ocv)));
soc = min(max(soc, min(double(params.soc_ocv_common))), ...
    max(double(params.soc_ocv_common)));
values = interp2(double(params.temp_bp_ocv(:).'), ...
    double(params.soc_ocv_common(:)), double(table_data), ...
    temperature, soc, 'linear');
end

function result = allocate_result(count, Ns, pack_cfg)
result = struct();
result.time_s = zeros(count, 1);
result.I_pack = zeros(count, 1);
result.T_ambient = zeros(count, 1);
result.V_pack = zeros(count, 1);
result.T_core = zeros(count, 1);
result.T_surf = zeros(count, 1);
result.SoC_true = zeros(count, 1);
result.Vp1 = zeros(count, 1);
result.Vp2 = zeros(count, 1);
result.V_group = zeros(count, Ns);
result.V_segment = zeros(count, pack_cfg.segment_count);
result.T_sensor = zeros(count, pack_cfg.temperature_sensor_count);
result.SoC_group = zeros(count, Ns);
result.V_min = zeros(count, 1);
result.V_max = zeros(count, 1);
result.T_max = zeros(count, 1);
result.T_avg = zeros(count, 1);
end

function result = store_output(result, index, time, current, ambient, output)
result.time_s(index) = time;
result.I_pack(index) = current;
result.T_ambient(index) = ambient;
result.V_pack(index) = output.V_pack;
result.T_core(index) = output.T_core;
result.T_surf(index) = output.T_surf;
result.SoC_true(index) = output.SoC_true;
result.Vp1(index) = output.Vp1;
result.Vp2(index) = output.Vp2;
result.V_group(index, :) = output.V_group(:).';
result.V_segment(index, :) = output.V_segment(:).';
result.T_sensor(index, :) = output.T_sensor(:).';
result.SoC_group(index, :) = output.SoC_group(:).';
result.V_min(index) = output.V_min;
result.V_max(index) = output.V_max;
result.T_max(index) = output.T_max;
result.T_avg(index) = output.T_avg;
end

function measured = add_measurement_effects(result, sim_cfg)
noise = sim_cfg.measurement_noise;
stream = RandStream('mt19937ar', 'Seed', double(noise.seed));
measured = struct();
measured.V_pack = result.V_pack + ...
    double(noise.voltage_std_V) * randn(stream, size(result.V_pack));
measured.I_pack = result.I_pack + ...
    double(noise.current_std_A) * randn(stream, size(result.I_pack));
measured.T_surf = result.T_surf + ...
    double(noise.temperature_std_C) * randn(stream, size(result.T_surf));
end
