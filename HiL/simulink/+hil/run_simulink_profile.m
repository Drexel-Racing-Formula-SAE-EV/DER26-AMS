function result = run_simulink_profile( ...
    cell_cfg, pack_cfg, sim_cfg, params, profile, varargin)
%RUN_SIMULINK_PROFILE Configure and execute the generated-model template.

parser = inputParser();
parser.addParameter('ModelArtifact', struct(), @isstruct);
parser.parse(varargin{:});
artifact = parser.Results.ModelArtifact;
if isempty(fieldnames(artifact))
    artifact = hil.configure_model(cell_cfg, pack_cfg, sim_cfg, params);
end
profile = hil.resolve_profile_inputs( ...
    cell_cfg, pack_cfg, sim_cfg, params, profile);
model_name = artifact.model_name;
load_system(artifact.model_file);
cleanup = onCleanup(@() close_system(model_name, 0)); %#ok<NASGU>

external_input = [ ...
    double(profile.time_s(:)), ...
    double(profile.pack_current_A(:)), ...
    double(profile.ambient_temperature_C(:))];
simulation_input = Simulink.SimulationInput(model_name);
simulation_input = simulation_input.setExternalInput(external_input);
simulation_input = simulation_input.setModelParameter( ...
    'StopTime', sprintf('%.17g', profile.time_s(end)), ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset');
simulation_output = sim(simulation_input);
yout = simulation_output.get('yout');

result = struct();
for index = 1:numel(artifact.output_names)
    name = artifact.output_names{index};
    [time, values] = output_values(yout, name, index);
    if index == 1
        result.time_s = time;
    end
    result.(name) = values;
end
result.I_pack = interp1(profile.time_s, profile.pack_current_A, ...
    result.time_s, 'previous', 'extrap');
result.T_ambient = interp1(profile.time_s, profile.ambient_temperature_C, ...
    result.time_s, 'linear', 'extrap');
result.Vp1 = [];
result.Vp2 = [];
result.engine = 'simulink';
result.model_artifact = artifact;
result.profile = profile;
result.profile.pack_current_A = [];
result.profile.ambient_temperature_C = [];
result.cell_configuration = cell_cfg;
result.pack_configuration = hil.derive_pack(cell_cfg, pack_cfg);
result.simulation_configuration = sim_cfg;
result.parameter_hash = params.configuration_hash;
result.configuration_hash = artifact.configuration_hash;
result.measurements = measured_outputs(result, sim_cfg);
end

function [time, values] = output_values(dataset, name, index)
element = [];
if isa(dataset, 'Simulink.SimulationData.Dataset')
    try
        element = dataset.getElement(name);
    catch
        element = dataset.getElement(index);
    end
else
    error('hil:run:OutputFormat', ...
        'Expected a Simulink.SimulationData.Dataset output.');
end
if isempty(element)
    element = dataset.getElement(index);
end
series = element.Values;
time = double(series.Time(:));
values = squeeze(double(series.Data));
if isvector(values)
    values = values(:);
elseif size(values, 1) ~= numel(time) && size(values, ndims(values)) == numel(time)
    order = [ndims(values), 1:(ndims(values) - 1)];
    values = squeeze(permute(values, order));
end
end

function measured = measured_outputs(result, sim_cfg)
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
