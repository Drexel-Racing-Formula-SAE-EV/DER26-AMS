function profile = synthetic_hppc(sim_cfg)
%SYNTHETIC_HPPC Reproduce the embedded 60-second repeating pulse sequence.

sample_time_s = double(sim_cfg.sample_time_s);
if isnumeric(sim_cfg.stop_time_s)
    stop_time_s = double(sim_cfg.stop_time_s);
else
    stop_time_s = 60.0;
end
time_s = (0:sample_time_s:stop_time_s).';
cycle_step = mod(floor(time_s / sample_time_s), round(60 / sample_time_s));
seconds = cycle_step * sample_time_s;

current_A = zeros(size(time_s));
current_A(seconds >= 10 & seconds < 11) = 100.0;
current_A(seconds >= 15 & seconds < 16) = -40.0;
current_A(seconds >= 16 & seconds < 26) = 60.0;
current_A(seconds >= 31 & seconds < 36) = 120.0;
current_A(seconds >= 40 & seconds < 44) = -60.0;

profile = struct( ...
    'time_s', time_s, ...
    'pack_current_A', current_A, ...
    'ambient_temperature_C', ...
        repmat(double(sim_cfg.ambient_temperature_C), size(time_s)), ...
    'name', 'synthetic_hppc', ...
    'source', 'repository-defined deterministic profile', ...
    'sample_time_s', sample_time_s, ...
    'repeat_policy', sim_cfg.profile.repeat_policy, ...
    'scaling_mode', sim_cfg.profile.scaling_mode);
end
