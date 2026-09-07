function cfg = low_soc_dynamic()
%LOW_SOC_DYNAMIC DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='low_soc_dynamic'; cfg.description='Dynamic operation from 20 percent SoC'; cfg.initial_soc=0.20; cfg.requirements={'EKF-LOW-SOC','SOP-UV'}; cfg.tier='nightly'; cfg.sop_oracle.checkpoint_times_s=[5 20 40];
end
