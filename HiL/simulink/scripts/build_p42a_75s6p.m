% Build the configured P42A/DER26 model without refitting reviewed parameters.
setup_hil
cell_configuration = hil.config.cell('p42a');
pack_configuration = hil.config.pack('der26_75s6p');
simulation_configuration = hil.config.simulation('hppc_validation');
[plant_parameters, parameter_build_report] = hil.build_parameters( ...
    cell_configuration, 'Save', false);
model_artifact = hil.configure_model( ...
    cell_configuration, pack_configuration, simulation_configuration, ...
    plant_parameters);
disp(model_artifact)
