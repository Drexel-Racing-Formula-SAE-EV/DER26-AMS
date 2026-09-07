function cfg = cell_voltage_bias()
%CELL_VOLTAGE_BIAS DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='cell_voltage_bias'; cfg.description='Persistent +30 mV bias on one series-group voltage'; cfg.sensor.faults=struct('type','cell_bias','start_s',5,'end_s',60,'target',22,'value',0.030); cfg.requirements={'VOLTAGE-BIAS','EKF-INNOVATION'}; cfg.tier='nightly';
end
