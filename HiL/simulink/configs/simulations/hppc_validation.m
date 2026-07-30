function cfg = hppc_validation()
%HPPC_VALIDATION Synthetic pulse profile for electrical regression.

paths = hil.project_paths();
cfg = struct();
cfg.schema_version = 1;
cfg.kind = 'simulation';
cfg.id = 'hppc_validation';
cfg.sample_time_s = 0.1;
cfg.stop_time_s = 120.0;
cfg.profile = struct( ...
    'name', 'synthetic_hppc', ...
    'kind', 'synthetic_hppc', ...
    'scaling_mode', 'no_scaling', ...
    'repeat_policy', 'repeat');
cfg.ambient_temperature_C = 25.0;
cfg.initial_soc = 1.0;
cfg.measurement_noise = struct( ...
    'voltage_std_V', 0.0, ...
    'current_std_A', 0.0, ...
    'temperature_std_C', 0.0, ...
    'seed', 1001);
cfg.current_bias_A = 0.0;
cfg.fault_injection = struct('enabled', false);
cfg.output_directory = fullfile(paths.output_root, cfg.id);
cfg.engine = 'auto';
end
