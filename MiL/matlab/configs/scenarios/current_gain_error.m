function cfg = current_gain_error()
%CURRENT_GAIN_ERROR DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='current_gain_error'; cfg.description='Common +2 percent current gain error'; cfg.sensor.current.gain_50A=1.02; cfg.sensor.current.gain_800A=1.02; cfg.requirements={'CURRENT-GAIN','EKF-CURRENT-ROBUSTNESS'}; cfg.tier='nightly';
end
