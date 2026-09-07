function cfg = current_bias_positive()
%CURRENT_BIAS_POSITIVE DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='current_bias_positive'; cfg.description='+1 A common current bias accumulation into SoC'; cfg.sensor.current.bias_50A_A=1.0; cfg.sensor.current.bias_800A_A=1.0; cfg.requirements={'CURRENT-BIAS','EKF-CURRENT-ROBUSTNESS'}; cfg.tier='nightly';
end
