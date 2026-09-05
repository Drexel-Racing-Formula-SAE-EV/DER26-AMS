function cfg = ekf_acquisition_segment_bias_recovery()
%EKF_ACQUISITION_SEGMENT_BIAS_RECOVERY Production acquisition fault recovery.
%
% Segment 1 receives a coherent +20 mV/cell voltage bias through the first
% fixed-basis decision. Qualification requires consensus rejection, healthy
% peer isolation, and later clean reacquisition. The normal clean-data 60 s
% convergence gate is intentionally not relaxed for this fault case.
cfg=mil.default_config();
cfg.id='ekf_acquisition_segment_bias_recovery';
cfg.description=['Production acquisition: reject coherent segment-1 voltage ', ...
    'bias, preserve healthy peers, and reacquire after clean data returns'];
cfg.initial_soc=0.60;
cfg.reference_ekf.enabled=false;
cfg.reference_ekf.initial_soc_offset=0.20;
cfg.reference_ekf.acquisition.enabled=false;
cfg.production.estimator.enabled=true;
cfg.production.estimator.precondition_rest_s=0.0;
cfg.production.sop.enabled=false;
cfg.production.soh.enabled=false;
cfg.sop_oracle.enabled=false;
cfg.fuse.enabled=false;

% Long initial rest provides the deliberately poisoned first acquisition
% opportunity and enough clean time afterwards for a retry. The later pulses
% preserve the C1 excitation/convergence character.
cfg.profile=struct('kind','segments', ...
    'duration_s',[50 10 10 10 10 55], ...
    'current_A',[0 60 0 -10 0 0], ...
    'ambient_C',[25 25 25 25 25 25], ...
    'name','acquisition_segment_bias_recovery');
cfg.stop_time_s=sum(cfg.profile.duration_s);

faults=repmat(struct('type','cell_bias','start_s',0,'end_s',20.1, ...
    'target',1,'value',0.020),1,15);
for k=1:15
    faults(k).target=k;
end
cfg.sensor.faults=faults;

cfg.acceptance.ekf.acquisition_mode=true;
cfg.acceptance.ekf.acquisition_expected=true;
cfg.acceptance.acquisition_fault_recovery.enabled=true;
cfg.acceptance.acquisition_fault_recovery.target_segment=1;
cfg.acceptance.acquisition_fault_recovery.fault_end_s=20.1;
cfg.requirements={'EKF-ACQ-FAULT-RECOVERY','EKF-CONVERGENCE','EKF-COVARIANCE','NUMERIC'};
cfg.tier='pr';
end
