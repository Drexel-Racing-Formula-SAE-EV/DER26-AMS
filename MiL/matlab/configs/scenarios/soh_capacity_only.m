function cfg = soh_capacity_only()
%SOH_CAPACITY_ONLY Aggregate 20 %% capacity fade with nominal resistance.
%
% Production capacity SoH is an aggregate pack observer driven by the
% capacity-weighted pack SoC. Keep all group capacity fade equal and explicitly
% zero initial SoC offsets so the low-knee positive-observability anchor does not
% intentionally violate the independent 50 mV cell-spread gate.
% Apply the fade to all 75 series groups so the
% injected truth is architecture-observable. The profile uses the same
% knee-to-knee confidence logic as C5, scaled to the 20.16 Ah faded pack:
% 1600 s at +40 A transfers 17.7778 Ah, taking 98 %% -> ~9.82 %% SoC.
cfg=mil.default_config();cfg.id='soh_capacity_only';
cfg.description='Independent aggregate 20% capacity-fade knee-to-knee case';
cfg.initial_soc=0.98;cfg.stop_time_s='profile';
cfg.profile=struct('kind','segments','name','soh_capacity_only', ...
    'duration_s',[300 1600 300 1600 400 800 300], ...
    'current_A',[0 40 0 -30 -20 -10 0]);
cfg.preflight.capacity_confidence_soc_windows=[0.045 0.105;0.975 0.985];

overrides=repmat(struct('group_index',1,'capacity_multiplier',0.80, ...
    'r0_multiplier',1.0,'r1_multiplier',1.0,'c1_multiplier',1.0, ...
    'r2_multiplier',1.0,'c2_multiplier',1.0,'rsa_multiplier',1.0, ...
    'initial_soc_offset',0.0),1,75);
for k=1:75,overrides(k).group_index=k;end
cfg.plant.overrides=overrides;

cfg.sensor.current.noise_50A_std_A=0.01;cfg.sensor.current.noise_800A_std_A=0.10;
cfg.sop_oracle.enabled=false;cfg.gates.soh=true;
cfg.requirements={'SOH-CAPACITY','SOH-INDEPENDENCE'};cfg.tier='release';
cfg.gates.soh_capacity=true;cfg.gates.soh_resistance=false;
end
