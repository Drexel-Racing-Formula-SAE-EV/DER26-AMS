function cfg = udds_25c()
%UDDS_25C Embedded UDDS pack-current trace at 25 degC.

cfg = us06_25c();
paths = hil.project_paths();
cfg.id = 'udds_25c';
cfg.profile.name = 'udds_25c';
cfg.profile.array_name = 'udds25_i_10ma';
cfg.output_directory = fullfile(paths.output_root, cfg.id);
end
