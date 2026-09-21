function cfg = timestamp_jitter()
%TIMESTAMP_JITTER DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='timestamp_jitter'; cfg.description='Estimator timestamp jitter robustness'; cfg.sensor.timestamp_jitter_std_s=0.010; cfg.requirements={'TIMING-JITTER','EKF-DT'}; cfg.tier='nightly';
end
