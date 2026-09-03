function cfg = weak_group_capacity()
%WEAK_GROUP_CAPACITY DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='weak_group_capacity'; cfg.description='One weak series group with 80 percent capacity'; cfg.plant.overrides=struct('group_index',37,'capacity_multiplier',0.80); cfg.requirements={'EKF-IMBALANCE','SOH-CAPACITY','SOP-WEAK-CELL'}; cfg.tier='nightly';
end
