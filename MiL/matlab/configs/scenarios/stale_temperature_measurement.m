function cfg = stale_temperature_measurement()
%STALE_TEMPERATURE_MEASUREMENT DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='stale_temperature_measurement'; cfg.description='Temperature measurement age exceeds 250 ms freshness limit'; cfg.sensor.faults=struct('type','temperature_stale','start_s',10,'end_s',20,'target',31,'value',1000); cfg.gates.ekf=false; cfg.requirements={'TEMP-STALE','FAIL-CLOSED'}; cfg.tier='nightly'; cfg.sop_oracle.enabled=false;
end
