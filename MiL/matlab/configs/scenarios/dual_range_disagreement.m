function cfg = dual_range_disagreement()
%DUAL_RANGE_DISAGREEMENT DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='dual_range_disagreement'; cfg.description='Dual-range Hall disagreement above compare threshold'; cfg.profile=struct('kind','segments','name','dual_range_disagreement','duration_s',[5 20 5],'current_A',[0 25 0]); cfg.stop_time_s='profile'; cfg.sensor.current.faults=struct('type','bias_800a','start_s',5,'end_s',25,'value',20); cfg.gates.ekf=false; cfg.requirements={'CURRENT-DUAL-RANGE','FAIL-CLOSED'}; cfg.tier='pr'; cfg.sop_oracle.enabled=false;
end
