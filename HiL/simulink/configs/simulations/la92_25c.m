function cfg = la92_25c()
%LA92_25C Embedded LA92 pack-current trace at 25 degC.

cfg = us06_25c();
paths = hil.project_paths();
cfg.id = 'la92_25c';
cfg.profile.name = 'la92_25c';
cfg.profile.array_name = 'la92_25_i_10ma';
cfg.output_directory = fullfile(paths.output_root, cfg.id);
end
