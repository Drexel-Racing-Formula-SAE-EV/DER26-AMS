function cfg = endurance_replay()
%ENDURANCE_REPLAY DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='endurance_replay'; cfg.description='Repeated endurance-like current trace at elevated ambient'; cfg.simulation_id='thermal_endurance'; cfg.initial_soc=0.95; cfg.initial_temperature_C=35; cfg.ambient_temperature_C=40; cfg.requirements={'ENDURANCE','EKF-LONG-DURATION','SOP-THERMAL'}; cfg.tier='release'; cfg.sop_oracle.checkpoint_times_s=[60 300 600];
end
