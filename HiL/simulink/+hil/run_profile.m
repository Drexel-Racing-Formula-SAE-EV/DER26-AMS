function result = run_profile(cell_cfg, pack_cfg, sim_cfg, params)
%RUN_PROFILE Execute the configured Simulink or MATLAB reference engine.

[cell_cfg, ~] = hil.validate_configuration(cell_cfg);
[pack_cfg, ~] = hil.validate_configuration(pack_cfg);
[sim_cfg, ~] = hil.validate_configuration(sim_cfg);
pack_cfg = hil.derive_pack(cell_cfg, pack_cfg);
profile = hil.load_profile(sim_cfg, cell_cfg, pack_cfg);

engine = lower(char(sim_cfg.engine));
if strcmp(engine, 'auto')
    if exist('sim', 'file') == 2 && license('test', 'Simulink')
        engine = 'simulink';
    else
        engine = 'reference';
    end
end

switch engine
    case 'reference'
        result = hil.run_reference(cell_cfg, pack_cfg, sim_cfg, params, profile);
    case 'simulink'
        result = hil.run_simulink_profile( ...
            cell_cfg, pack_cfg, sim_cfg, params, profile);
    otherwise
        error('hil:run:Engine', 'Unsupported simulation engine "%s".', engine);
end
end
