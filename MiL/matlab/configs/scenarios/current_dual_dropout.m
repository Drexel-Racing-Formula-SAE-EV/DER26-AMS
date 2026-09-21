function cfg = current_dual_dropout()
%CURRENT_DUAL_DROPOUT Both Hall ranges unavailable during a dynamic interval.
cfg=mil.default_config();cfg.id='current_dual_dropout';
cfg.description='Simultaneous 50 A and 800 A current-channel dropout';
cfg.sensor.current.faults=[ ...
    struct('type','dropout_50a','start_s',10,'end_s',20,'target',0,'value',0), ...
    struct('type','dropout_800a','start_s',10,'end_s',20,'target',0,'value',0)];
cfg.sop_oracle.enabled=false;cfg.gates.ekf=false;
cfg.requirements={'CURRENT-DROPOUT','CURRENT-IMPLAUSIBLE','FAIL-CLOSED'};
cfg.tier='pr';
end
