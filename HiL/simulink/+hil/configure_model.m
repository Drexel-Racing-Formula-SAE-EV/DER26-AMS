function artifact = configure_model(cell_cfg, pack_cfg, sim_cfg, params, varargin)
%CONFIGURE_MODEL Materialize a topology-specific model from the clean template.

parser = inputParser();
parser.addParameter('OutputDirectory', '', @(x) ischar(x) || isstring(x));
parser.addParameter('ModelName', '', @(x) ischar(x) || isstring(x));
parser.parse(varargin{:});
options = parser.Results;

[cell_cfg, ~] = hil.validate_configuration(cell_cfg);
[pack_cfg, ~] = hil.validate_configuration(pack_cfg);
[sim_cfg, ~] = hil.validate_configuration(sim_cfg);
pack_cfg = hil.derive_pack(cell_cfg, pack_cfg);
hil.validate_parameters(cell_cfg, params);

if exist('load_system', 'file') ~= 2 || ~license('test', 'Simulink')
    error('hil:model:SimulinkRequired', ...
        'Configuring the model requires MATLAB with a Simulink license.');
end

paths = hil.project_paths();
template_file = fullfile(paths.models, 'der_accumulator_2rc_thermal_plant.slx');
if ~isfile(template_file)
    error('hil:model:TemplateMissing', 'Missing model template: %s', template_file);
end

configuration_hash = hil.configuration_hash( ...
    cell_cfg, pack_cfg, sim_cfg, params.configuration_hash);
model_name = char(options.ModelName);
if isempty(model_name)
    model_name = short_model_name( ...
        cell_cfg.id, pack_cfg.id, configuration_hash);
end
model_name = matlab.lang.makeValidName(model_name);
if strlength(string(model_name)) > 31
    error('hil:model:NameTooLong', ...
        'Generated model names are limited to 31 characters.');
end

output_directory = char(options.OutputDirectory);
if isempty(output_directory)
    output_directory = fullfile(paths.model_build_root, configuration_hash);
end
hil.ensure_directory(output_directory);
model_file = fullfile(output_directory, [model_name, '.slx']);
copyfile(template_file, model_file, 'f');

load_system(model_file);
cleanup = onCleanup(@() close_system(model_name, 0)); %#ok<NASGU>
root = model_name;

model_params = params;
model_params.SoC_init = single(sim_cfg.initial_soc);
model_params.T_init = single(pack_cfg.initial_temperature_C);
model_params.Rsa = single(double(params.Rsa) * ...
    double(pack_cfg.cooling_boundary.Rsa_multiplier));
workspace = get_param(root, 'ModelWorkspace');
names = fieldnames(model_params);
for index = 1:numel(names)
    workspace.assignin(names{index}, model_params.(names{index}));
end
workspace.assignin('Ns', double(pack_cfg.series_groups));
workspace.assignin('Np', double(pack_cfg.parallel_cells));
workspace.assignin('Nsegments', double(pack_cfg.segment_count));
workspace.assignin('Ntemp', double(pack_cfg.temperature_sensor_count));

set_param([root, '/I_pack_to_cell'], 'Gain', sprintf('1/%d', ...
    pack_cfg.parallel_cells));
set_param([root, '/V_cell_to_pack'], 'Gain', sprintf('%d', ...
    pack_cfg.series_groups));
set_param([root, '/dSoC_per_step'], 'Gain', '-1/(3600*Q_nom)');
set_param([root, '/I_to_Vp2'], 'Gain', '1/C2');
set_param([root, '/Vp2_feedback'], 'Gain', '-1/(R2*C2)');
set_param([root, '/ T_clamp'], ...
    'LowerLimit', sprintf('%.17g', cell_cfg.temperature_model_limits_C(1)), ...
    'UpperLimit', sprintf('%.17g', cell_cfg.temperature_model_limits_C(2)));

sample_time = sprintf('%.17g', sim_cfg.sample_time_s);
integrators = {
    [root, '/SoC_Integrator']
    [root, '/Vp1_Integrator1']
    [root, '/Vp2_Integrator']
    [root, '/Thermal_2Node/T_core_int']
    [root, '/Thermal_2Node/T_surf_int']
    };
for index = 1:numel(integrators)
    set_param(integrators{index}, 'SampleTime', sample_time);
end

set_param(root, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', sample_time, ...
    'StartTime', '0', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset');
if isnumeric(sim_cfg.stop_time_s)
    set_param(root, 'StopTime', sprintf('%.17g', sim_cfg.stop_time_s));
end

chart_path = [root, '/Accumulator_Output_Expansion'];
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', chart_path);
if isempty(chart)
    error('hil:model:ChartMissing', ...
        'Could not locate MATLAB Function block %s.', chart_path);
end
chart(1).Script = hil.codegen_chart_script(pack_cfg);

set_param([root, '/V_group_spec'], ...
    'Dimensions', sprintf('[%d 1]', pack_cfg.series_groups));
set_param([root, '/V_segment_spec'], ...
    'Dimensions', sprintf('[%d 1]', pack_cfg.segment_count));
set_param([root, '/T_sensor_spec'], ...
    'Dimensions', sprintf('[%d 1]', pack_cfg.temperature_sensor_count));
set_param([root, '/SoC_group_spec'], ...
    'Dimensions', sprintf('[%d 1]', pack_cfg.series_groups));

set_param(root, 'Description', sprintf( ...
    ['Configured by hil.configure_model. Cell=%s Pack=%s Hash=%s. ' ...
     'The generated model uses one representative electrothermal state.'], ...
    cell_cfg.id, pack_cfg.id, configuration_hash));
save_system(root, model_file);

manifest = struct();
manifest.schema_version = 1;
manifest.generated_utc = char(datetime('now', 'TimeZone', 'UTC', ...
    'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX'));
manifest.template_file = template_file;
manifest.template_sha256 = hil.file_sha256(template_file);
manifest.model_file = model_file;
manifest.model_name = model_name;
manifest.configuration_hash = configuration_hash;
manifest.parameter_hash = params.configuration_hash;
manifest.cell_configuration_hash = hil.configuration_hash(cell_cfg);
manifest.pack_configuration_hash = hil.configuration_hash(pack_cfg);
manifest.simulation_configuration_hash = hil.configuration_hash(sim_cfg);
manifest.cell_configuration = cell_cfg;
manifest.pack_configuration = pack_cfg;
manifest.simulation_configuration = sim_cfg;
manifest.generated_model_scope = ...
    ['One representative 2RC/two-node thermal state with configurable ' ...
     'fixed-size pack output expansion.'];
manifest.distributed_reference_scope = ...
    ['Use hil.run_reference with parameter_distributed mode for independently ' ...
     'evolved group electrical and thermal states.'];
manifest_file = fullfile(output_directory, 'model_manifest.json');
hil.write_json(manifest_file, manifest);

artifact = struct( ...
    'model_name', model_name, ...
    'model_file', model_file, ...
    'manifest_file', manifest_file, ...
    'configuration_hash', configuration_hash, ...
    'parameter_hash', params.configuration_hash, ...
    'cell_configuration_hash', manifest.cell_configuration_hash, ...
    'pack_configuration_hash', manifest.pack_configuration_hash, ...
    'simulation_configuration_hash', ...
        manifest.simulation_configuration_hash, ...
    'simulation_configuration', sim_cfg, ...
    'output_directory', output_directory, ...
    'output_names', {{ ...
        'V_pack', 'T_core', 'T_surf', 'SoC_true', ...
        'V_group', 'V_segment', 'T_sensor', 'SoC_group', ...
        'V_min', 'V_max', 'T_max', 'T_avg'}});
end

function name = short_model_name(cell_id, pack_id, hash)
identity = lower(regexprep( ...
    [char(cell_id), '_', char(pack_id)], '[^A-Za-z0-9_]', '_'));
prefix = 'der_';
suffix = ['_', lower(hash(1:8))];
available = 31 - numel(prefix) - numel(suffix);
if numel(identity) > available
    identity = identity(1:available);
end
name = matlab.lang.makeValidName([prefix, identity, suffix]);
end
