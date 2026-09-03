function cfg = high_soc_regen()
%HIGH_SOC_REGEN DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='high_soc_regen'; cfg.description='High-SoC charge/regen constraint exercise'; cfg.initial_soc=0.94; cfg.profile=struct('kind','segments','name','high_soc_regen','duration_s',[10 8 8 8 8 8 10],'current_A',[0 -10 10 -10 20 -10 0]); cfg.stop_time_s='profile'; cfg.requirements={'EKF-HIGH-SOC','SOP-OV','SOP-CHARGE'}; cfg.tier='nightly'; cfg.sop_oracle.checkpoint_times_s=[5 15 30 50];
end
