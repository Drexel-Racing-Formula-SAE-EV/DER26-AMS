function cfg = segment_pec_dropout()
%SEGMENT_PEC_DROPOUT DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='segment_pec_dropout'; cfg.description='Whole segment invalid due to PEC-invalid measurement image'; cfg.sensor.faults=struct('type','segment_pec_invalid','start_s',10,'end_s',20,'target',3,'value',0); cfg.gates.ekf=false; cfg.requirements={'ADBMS-PEC','MEAS-VALIDITY','FAIL-CLOSED'}; cfg.tier='pr'; cfg.sop_oracle.enabled=false;
end
