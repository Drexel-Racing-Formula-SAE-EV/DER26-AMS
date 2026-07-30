% Compatibility entry point: all electrical branches are fitted together.
setup_hil
cell_configuration = hil.config.cell('p42a');
dataset_configuration = hil.config.dataset('p42a_published_hcgt');
[plant_parameters, parameter_build_report] = hil.build_parameters( ...
    cell_configuration, 'DatasetConfiguration', dataset_configuration, ...
    'ForceRefit', true, 'Save', true);
disp(parameter_build_report)
