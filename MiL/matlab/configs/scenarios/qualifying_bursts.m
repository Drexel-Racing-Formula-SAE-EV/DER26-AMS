function cfg = qualifying_bursts()
%QUALIFYING_BURSTS DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='qualifying_bursts'; cfg.description='Short high-power discharge/regen burst sequence'; cfg.initial_soc=0.80; cfg.profile=struct('kind','segments','name','qualifying_bursts','duration_s',[10 3 7 3 7 3 7 3 10],'current_A',[0 110 0 -10 0 110 0 -10 0]); cfg.stop_time_s='profile'; cfg.requirements={'QUALIFY-BURST','SOP-0P1S','SOP-1S'}; cfg.tier='nightly'; cfg.sop_oracle.checkpoint_times_s=[9 11 20 30 40];
end
