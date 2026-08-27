% Compatibility entry point for normalized P42A HCGT validation.
setup_hil
cell_configuration = hil.config.cell('p42a');
pack_configuration = hil.config.pack('der26_75s6p');
dataset_configuration = hil.config.dataset('p42a_published_hcgt');
[plant_parameters, parameter_build_report] = hil.build_parameters( ...
    cell_configuration, 'Save', false);
dataset_validation_report = hil.validate_dataset( ...
    cell_configuration, pack_configuration, dataset_configuration, ...
    plant_parameters);
disp(dataset_validation_report)
