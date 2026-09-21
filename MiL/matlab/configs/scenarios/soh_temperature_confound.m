function cfg = soh_temperature_confound()
%SOH_TEMPERATURE_CONFOUND Temperature change without irreversible aging.
cfg=mil.default_config();cfg.id='soh_temperature_confound';
cfg.description='Temperature-only resistance increase must not become permanent aging';
cfg.initial_soc=0.80;cfg.initial_temperature_C=5;cfg.ambient_temperature_C=40;
cfg.stop_time_s='profile';
cfg.profile=struct('kind','segments','name','soh_temperature_confound', ...
    'duration_s',[120 240 120 240 120 60], ...
    'current_A',[0 50 0 50 0 0],'ambient_C',[5 5 25 40 40 25]);
cfg.sop_oracle.enabled=false;cfg.gates.soh=true;
cfg.gates.soh_capacity=false;cfg.gates.soh_resistance=false;cfg.gates.soh_false_aging=true;
cfg.requirements={'SOH-FALSE-AGING','SOH-TEMPERATURE-CONFOUND'};
cfg.tier='release';
end
