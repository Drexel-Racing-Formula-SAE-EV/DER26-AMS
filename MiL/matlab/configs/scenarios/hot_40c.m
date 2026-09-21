function cfg = hot_40c()
%HOT_40C DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='hot_40c'; cfg.description='Hot edge of characterized electrical LUT range'; cfg.initial_temperature_C=40; cfg.ambient_temperature_C=40; cfg.requirements={'EKF-TEMP','EKF-TEMP-CONFIDENCE','SOP-THERMAL'}; cfg.tier='nightly';
end
