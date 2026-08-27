% Compatibility entry point: load the reviewed legacy snapshot.
setup_hil
cell_configuration = hil.config.cell('p42a');
[plant_parameters, parameter_build_report] = hil.build_parameters( ...
    cell_configuration, 'Save', true);
disp(parameter_build_report)
