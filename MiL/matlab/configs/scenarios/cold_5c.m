function cfg = cold_5c()
%COLD_5C DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='cold_5c'; cfg.description='Cold edge of characterized electrical LUT range'; cfg.initial_temperature_C=5; cfg.ambient_temperature_C=5; cfg.requirements={'EKF-TEMP','EKF-TEMP-CONFIDENCE','SOP-CHARGE-TEMP'}; cfg.tier='nightly';
end
