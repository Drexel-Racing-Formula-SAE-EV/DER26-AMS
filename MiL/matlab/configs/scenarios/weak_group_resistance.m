function cfg = weak_group_resistance()
%WEAK_GROUP_RESISTANCE DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='weak_group_resistance'; cfg.description='One high-resistance series group'; cfg.plant.overrides=struct('group_index',37,'r0_multiplier',1.35,'r1_multiplier',1.20); cfg.acceptance.ekf.r0_accuracy_required=true;
cfg.acceptance.ekf.r0_unobservable_drift_required=true; cfg.requirements={'EKF-R0','SOH-RESISTANCE','SOP-WEAK-CELL'}; cfg.tier='nightly';
end
