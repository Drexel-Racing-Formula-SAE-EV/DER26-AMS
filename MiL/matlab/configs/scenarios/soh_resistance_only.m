function cfg = soh_resistance_only()
%SOH_RESISTANCE_ONLY Aggregate segment-observable R0 growth with nominal capacity.
%
% The production R0/SoH path estimates one equivalent R0 per 15-group segment.
% Apply the target resistance multiplier uniformly so a 40% growth target is
% observable rather than asking a segment estimator to localize one cell group.
cfg=mil.default_config();cfg.id='soh_resistance_only';
cfg.description='Independent aggregate 40% resistance-growth case with nominal capacity';
cfg.initial_soc=0.80;cfg.stop_time_s='profile';
step_pair=repmat([40 60],1,120);       % 240 s, >50 observable R0 transitions
step_duration=ones(size(step_pair));
cfg.profile=struct('kind','segments','name','soh_resistance_only', ...
    'duration_s',[90 step_duration 90], ...
    'current_A',[0 step_pair 0]);

overrides=repmat(struct('group_index',1,'capacity_multiplier',1.0, ...
    'r0_multiplier',1.40,'r1_multiplier',1.0,'c1_multiplier',1.0, ...
    'r2_multiplier',1.0,'c2_multiplier',1.0,'rsa_multiplier',1.0),1,75);
for k=1:75,overrides(k).group_index=k;end
cfg.plant.overrides=overrides;

cfg.sensor.current.noise_50A_std_A=0.01;cfg.sensor.current.noise_800A_std_A=0.10;
cfg.sop_oracle.enabled=false;cfg.gates.soh=true;
cfg.requirements={'SOH-RESISTANCE','SOH-INDEPENDENCE'};cfg.tier='release';
cfg.gates.soh_capacity=false;cfg.gates.soh_resistance=true;
end
