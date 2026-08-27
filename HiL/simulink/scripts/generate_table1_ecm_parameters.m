% Compatibility entry point for auditable parameter-table export.
setup_hil
cell_configuration = hil.config.cell('p42a');
[plant_parameters, parameter_build_report] = hil.build_parameters( ...
    cell_configuration, 'Save', false);
paths = hil.project_paths();
parameter_summary_files = hil.export_parameter_summary( ...
    plant_parameters, fullfile(paths.validation, 'generated', 'parameters'));
disp(parameter_summary_files)
