function cfg = current_800a_stuck_high()
%CURRENT_800A_STUCK_HIGH High-range ADC rail fault must fail closed.
cfg=mil.default_config();cfg.id='current_800a_stuck_high';
cfg.description='800 A channel ADC stuck at high rail';
cfg.sensor.current.faults=struct('type','adc_stuck_high_800a', ...
    'start_s',10,'end_s',20,'target',0,'value',4095);
cfg.sop_oracle.enabled=false;cfg.gates.ekf=false;
cfg.requirements={'CURRENT-SATURATION','CURRENT-IMPLAUSIBLE','FAIL-CLOSED'};
cfg.tier='pr';
end
