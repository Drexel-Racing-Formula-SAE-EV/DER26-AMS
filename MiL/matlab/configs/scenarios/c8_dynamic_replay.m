function cfg = c8_dynamic_replay()
%C8_DYNAMIC_REPLAY Checked-in dynamic replay canonical case.
cfg=mil.default_config();cfg.id='c8_dynamic_replay';
cfg.description='C8: 600 s checked-in US06 dynamic estimator/SoP/fuse replay';
cfg.initial_soc=0.70;cfg.stop_time_s=600;
% This scenario qualifies dynamic behavior, not cold-start acquisition. The
% production estimator receives a stationary precondition before scored t=0
% so the full 600 s drive profile remains unchanged while SoP authority starts
% from a legitimately acquired estimator state.
cfg.production.estimator.precondition_rest_s=30.0;
cfg.sop_oracle.checkpoint_times_s=[30 120 300 480 590];
cfg.requirements={'C8-DYNAMIC','EKF-NOMINAL','SOP-HORIZONS','FUSE-CONSERVATIVE'};
cfg.tier='pr';
end
