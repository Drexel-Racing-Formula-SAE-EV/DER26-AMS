function cfg = c3_high_soc_charge_sop()
%C3_HIGH_SOC_CHARGE_SOP High-SoC regen/charge canonical case.
cfg=mil.default_config();cfg.id='c3_high_soc_charge_sop';
cfg.description='C3: high-SoC charge ramps and pulses through OV/SoC limits';
cfg.initial_soc=0.95;cfg.stop_time_s='profile';
cfg.profile=struct('kind','segments','name','c3_high_soc_charge_sop', ...
    'duration_s',[30 30 30 30 30 30],'current_A',[0 -5 -10 -12 -5 0]);
cfg.sop_oracle.checkpoint_times_s=[25 55 85 115 145 175];
cfg.requirements={'C3-HIGH-SOC-SOP','EKF-HIGH-SOC','SOP-OV','SOP-CHARGE','CURRENT-POLARITY'};
cfg.tier='pr';
end
