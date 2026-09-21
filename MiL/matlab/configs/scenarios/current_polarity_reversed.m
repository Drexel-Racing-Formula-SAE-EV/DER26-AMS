function cfg = current_polarity_reversed()
%CURRENT_POLARITY_REVERSED DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='current_polarity_reversed'; cfg.description='Reversed current polarity characterization; must not be accepted as qualification'; cfg.sensor.current.polarity=-1; cfg.sensor.current.polarity_validated=false; cfg.gates.ekf=false; cfg.requirements={'CURRENT-POLARITY'}; cfg.tier='release';
end
