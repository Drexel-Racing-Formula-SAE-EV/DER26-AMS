function cfg = stale_cell_measurement()
%STALE_CELL_MEASUREMENT DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='stale_cell_measurement'; cfg.description='Cell measurement age exceeds 250 ms freshness limit'; cfg.sensor.faults=struct('type','cell_stale','start_s',10,'end_s',20,'target',31,'value',1000); cfg.gates.ekf=false; cfg.requirements={'MEAS-STALE','FAIL-CLOSED'}; cfg.tier='pr'; cfg.sop_oracle.enabled=false;
end
