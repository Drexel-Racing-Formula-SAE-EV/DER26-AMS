% Compatibility entry point for fast topology and scaling checks.
setup_hil
validation_report = hil.validate_all( ...
    'RunThermal', false, 'RunDataset', false, 'RunSimulink', false);
disp(validation_report.report_files)
