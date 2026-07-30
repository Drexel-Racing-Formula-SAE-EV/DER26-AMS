function cfg = candidate_csv_template()
%CANDIDATE_CSV_TEMPLATE Copy and replace before candidate-cell fitting.

paths = hil.project_paths();
cfg = struct();
cfg.schema_version = 1;
cfg.kind = 'dataset';
cfg.id = 'candidate_csv_template';
cfg.is_template = true;
cfg.adapter = 'load_generic_csv';
cfg.root = fullfile(paths.data_root, 'REPLACE_ME');
cfg.fit_files = {'REPLACE_ME_pulses.csv'};
cfg.holdout_files = {'REPLACE_ME_holdout.csv'};
cfg.ocv_files = {'REPLACE_ME_ocv.csv'};
cfg.r0_file = 'REPLACE_ME_r0.csv';
cfg.current_sign = 'auto';
cfg.units = struct( ...
    'time', 's', ...
    'current', 'A', ...
    'voltage', 'V', ...
    'temperature', 'degC', ...
    'capacity', 'Ah');
cfg.source = 'REPLACE_ME';
cfg.version = 'REPLACE_ME';
cfg.test_type = 'REPLACE_ME';
cfg.surface_temperature_source = ...
    'REPLACE_ME: measured | not_available (ambient is not surface temperature)';
end
