function profile = load_profile(sim_cfg, cell_cfg, pack_cfg)
%LOAD_PROFILE Resolve the configured profile loader.

paths = hil.project_paths();
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
addpath(paths.profiles, '-begin');
profile = load_current_profile(sim_cfg.profile, sim_cfg, cell_cfg, pack_cfg);
end
