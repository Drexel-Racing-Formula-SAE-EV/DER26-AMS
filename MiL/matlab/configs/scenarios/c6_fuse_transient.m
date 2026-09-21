function cfg = c6_fuse_transient()
%C6_FUSE_TRANSIENT Existing deterministic autocross fuse trace through plant.
cfg=mil.default_config();cfg.id='c6_fuse_transient';
cfg.description='C6: 360 s corner-exit/autocross transient and fuse model comparison';
cfg.initial_soc=0.60;cfg.stop_time_s='profile';
cfg.profile=struct('kind','csv','name','c6_fuse_transient', ...
    'path','Tools/fuse_replay/traces/synthetic_autocross.csv');
cfg.sop_oracle.checkpoint_times_s=[60 120 240 350];
cfg.requirements={'C6-FUSE','FUSE-CONSERVATIVE','SOP-FUSE-ENVELOPE'};
cfg.tier='pr';
end
