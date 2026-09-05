function cfg = c4_hot_weak_group()
%C4_HOT_WEAK_GROUP Hot weak-group electrothermal canonical case.
cfg=mil.default_config();cfg.id='c4_hot_weak_group';
cfg.description='C4: 600 s hot dynamic profile with weak/high-R/poor-cooling group';
cfg.initial_soc=0.60;cfg.initial_temperature_C=40;cfg.ambient_temperature_C=40;
cfg.stop_time_s=600;
% This scenario qualifies dynamic behavior, not cold-start acquisition. The
% production estimator receives a stationary precondition before scored t=0
% so the full 600 s drive profile remains unchanged while SoP authority starts
% from a legitimately acquired estimator state.
cfg.production.estimator.precondition_rest_s=30.0;
cfg.plant.overrides=struct('group_index',37,'capacity_multiplier',0.90, ...
    'r0_multiplier',1.35,'r1_multiplier',1.20,'rsa_multiplier',1.25);
cfg.sop_oracle.checkpoint_times_s=[30 120 300 480 590];
cfg.requirements={'C4-HOT-WEAK','EKF-IMBALANCE','SOP-WEAK-CELL','SOP-THERMAL'};
cfg.tier='pr';
end
