function cfg = soh_capacity_window()
%SOH_CAPACITY_WINDOW Balanced knee-to-knee capacity-observability campaign.
%
% Production capacity SoH requires both low estimator uncertainty and <=50 mV
% cell spread at a qualified rest. The licensed v2.6.7 C5 run showed that a
% distributed-capacity pack can satisfy the low-SOC sigma gate yet correctly
% fail the independent spread gate. This positive observer case therefore
% removes capacity/initial-SOC imbalance while leaving the firmware gates
% unchanged; imbalance rejection is qualified separately.
cfg=mil.default_config();cfg.id='soh_capacity_window';
cfg.description='Balanced knee-to-knee production capacity-observability campaign';
cfg.initial_soc=0.98;
cfg.profile=struct('kind','segments','name','soh_capacity_window', ...
    'duration_s',[300 2000 300 2000 500 1000 300], ...
    'current_A',[0 40 0 -30 -20 -10 0]);
cfg.stop_time_s='profile';
cfg.preflight.capacity_confidence_soc_windows=[0.045 0.105;0.975 0.985];

overrides=repmat(struct('group_index',1,'capacity_multiplier',1.0, ...
    'initial_soc_offset',0.0),1,75);
for k=1:75,overrides(k).group_index=k;end
cfg.plant.overrides=overrides;

cfg.sensor.current.noise_50A_std_A=0.01;cfg.sensor.current.noise_800A_std_A=0.1;
cfg.requirements={'SOH-CAPACITY-OBSERVABILITY'};cfg.tier='release';
cfg.sop_oracle.enabled=false;cfg.gates.soh=true;
cfg.gates.soh_capacity=true;cfg.gates.soh_resistance=false;
end
