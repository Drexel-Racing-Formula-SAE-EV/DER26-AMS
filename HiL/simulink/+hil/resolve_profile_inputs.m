function profile = resolve_profile_inputs( ...
    cell_cfg, pack_cfg, sim_cfg, params, profile)
%RESOLVE_PROFILE_INPUTS Freeze bias/power semantics to a pack-current trace.

if isfield(profile, 'inputs_resolved') && logical(profile.inputs_resolved)
    return;
end

if isfield(profile, 'pack_power_W')
    reference_cfg = sim_cfg;
    reference_cfg.engine = 'reference';
    reference_cfg.output_decimation = 1;
    reference = hil.run_reference( ...
        cell_cfg, pack_cfg, reference_cfg, params, profile);
    if numel(reference.time_s) ~= numel(profile.time_s) || ...
            max(abs(double(reference.time_s(:)) - ...
            double(profile.time_s(:)))) > 1e-9
        error('hil:profile:PowerResolutionAlignment', ...
            'Power-profile resolution must retain every input sample.');
    end
    profile.pack_current_A = double(reference.I_pack(:));
    profile = rmfield(profile, 'pack_power_W');
    profile.resolution = 'reference-voltage iterative power-to-current';
else
    profile.pack_current_A = double(profile.pack_current_A(:)) + ...
        double(sim_cfg.current_bias_A);
    profile.resolution = 'configured current plus current bias';
end
profile.scaling_mode = 'resolved_current';
profile.inputs_resolved = true;
end
