function cfg = current_800a_stuck_low()
%CURRENT_800A_STUCK_LOW DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='current_800a_stuck_low'; cfg.description='800 A DHAB channel electrically stuck near ground'; cfg.sensor.current.faults=struct('type','stuck_800a','start_s',5,'end_s',40,'value',-1000); cfg.gates.ekf=false; cfg.requirements={'CURRENT-IMPLAUSIBLE','FAIL-CLOSED'}; cfg.tier='pr';
end
