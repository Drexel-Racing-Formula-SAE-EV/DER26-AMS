% One-command MATLAB/Simulink test entry point.
setup_hil
validation_report = hil.validate_all( ...
    'RunThermal', true, 'RunDataset', 'auto', 'RunSimulink', 'auto');
assert(validation_report.framework_passed, ...
    'One or more executed HIL framework gates failed.')
disp(validation_report.report_files)
