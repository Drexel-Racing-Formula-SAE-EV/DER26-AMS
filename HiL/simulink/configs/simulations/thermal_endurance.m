function cfg = thermal_endurance()
%THERMAL_ENDURANCE Repeated-load screening for passive-cooling sensitivity.

cfg = us06_25c();
paths = hil.project_paths();
cfg.id = 'thermal_endurance';
cfg.stop_time_s = 'profile';
cfg.profile.repeat_policy = 'repeat_count';
cfg.profile.repeat_count = 20;
cfg.profile.scaling_mode = 'vehicle_current_replay';
cfg.ambient_temperature_C = 40.0;
cfg.initial_soc = 1.0;
cfg.output_directory = fullfile(paths.output_root, cfg.id);
cfg.engine = 'reference';
cfg.output_decimation = 10;
end
