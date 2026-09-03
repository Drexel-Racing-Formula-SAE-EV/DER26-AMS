function cfg = temperature_bias()
%TEMPERATURE_BIAS DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='temperature_bias'; cfg.description='+5 C bias on one temperature channel'; cfg.sensor.faults=struct('type','temperature_bias','start_s',5,'end_s',60,'target',50,'value',5.0); cfg.requirements={'TEMP-BIAS','SOP-THERMAL'}; cfg.tier='nightly';
end
