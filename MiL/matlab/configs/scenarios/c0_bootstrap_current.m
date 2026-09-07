function cfg = c0_bootstrap_current()
%C0_BOOTSTRAP_CURRENT Bootstrap/precharge/sign and invalid-H qualification.
cfg=mil.default_config();cfg.id='c0_bootstrap_current';
cfg.description='C0: stable zero, precharge, +/-0.5 A sign, and invalid H-channel injection';
cfg.initial_soc=0.60;cfg.stop_time_s='profile';
cfg.production.estimator.precondition_rest_s=0.0;
cfg.profile=struct('kind','segments','name','c0_bootstrap_current', ...
    'duration_s',[20 10 20 10 10 10 10], ...
    'current_A',[0 3 0 0.5 0 -0.5 0]);
cfg.sensor.current.faults=struct('type','adc_stuck_low_800a', ...
    'start_s',50,'end_s',60,'target',0,'value',0);
cfg.sop_oracle.enabled=false;cfg.gates.ekf=false;
cfg.requirements={'C0-CURRENT-BOOTSTRAP','CURRENT-ZERO-INVALID','CURRENT-POLARITY','FAIL-CLOSED'};
cfg.tier='pr';
end
