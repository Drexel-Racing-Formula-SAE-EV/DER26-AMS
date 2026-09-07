function cfg = c2_low_soc_sop()
%C2_LOW_SOC_SOP Low-SoC discharge/UV boundary canonical case.
cfg=mil.default_config();cfg.id='c2_low_soc_sop';
cfg.description='C2: low-SoC discharge ramps toward voltage/current boundaries';
cfg.initial_soc=0.08;cfg.stop_time_s='profile';
cfg.profile=struct('kind','segments','name','c2_low_soc_sop', ...
    'duration_s',[30 30 30 30 30 30],'current_A',[0 40 60 80 100 0]);
cfg.sop_oracle.checkpoint_times_s=[25 55 85 115 145 175];
cfg.requirements={'C2-LOW-SOC-SOP','EKF-LOW-SOC','SOP-UV','SOP-HORIZONS'};
cfg.tier='pr';
end
