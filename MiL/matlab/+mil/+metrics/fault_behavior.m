function report = fault_behavior(meas,cfg)
%FAULT_BEHAVIOR Check deterministic fail-closed expectations at measurement boundary.
report=struct('checks',struct([]),'pass',true,'note', ...
    'Checks measurement-boundary behavior only; firmware authority parity is a separate production-C test.');
checks=repmat(struct( ...
    'fault_type','', ...
    'check','', ...
    'pass',false),0,1);
% Cell/segment faults that invalidate voltage images must invalidate pack image.
for k=1:numel(cfg.sensor.faults)
    f=cfg.sensor.faults(k); type=lower(string(f.type));
    active=window(meas.time_s,f);
    if ~any(active),continue;end
    should_invalidate=ismember(type,["cell_dropout","cell_invalid","pec_invalid_cell", ...
        "cell_open_wire_like","segment_pec_invalid"]);
    if should_invalidate
        ok=all(~meas.pack_voltage_valid(active));
        checks(end+1)=makecheck(char(type),'pack_voltage_fail_closed',ok); %#ok<AGROW>
    end
    if type=="cell_stale"
        target=double(f.target); threshold=double(cfg.sensor.max_cell_age_ms);
        expected=double(f.value)>threshold;
        if expected
            ok=all(~meas.cell_fresh(active,target));
            checks(end+1)=makecheck(char(type),'stale_cell_not_fresh',ok); %#ok<AGROW>
        end
    end
    if type=="temperature_stale"
        target=double(f.target); threshold=double(cfg.sensor.max_temp_age_ms);
        if double(f.value)>threshold
            ok=all(~meas.temperature_fresh(active,target));
            checks(end+1)=makecheck(char(type),'stale_temperature_not_fresh',ok); %#ok<AGROW>
        end
    end
    if type=="pack_current_dropout"
        ok=all(~meas.current_valid(active));
        checks(end+1)=makecheck(char(type),'current_fail_closed',ok); %#ok<AGROW>
    end
    if type=="sequence_repeat"
        ok=numel(unique(meas.sequence(active)))==1;
        checks(end+1)=makecheck(char(type),'sequence_frozen',ok); %#ok<AGROW>
    end
end
% Current-front-end faults are applied inside the DHAB model.
if isfield(cfg.sensor.current,'faults')
    for k=1:numel(cfg.sensor.current.faults)
        f=cfg.sensor.current.faults(k); type=lower(string(f.type)); active=window(meas.time_s,f);
        if ~any(active),continue;end
        if ismember(type,["dropout_50a","dropout_800a", ...
                "adc_stuck_low_50a","adc_stuck_high_50a", ...
                "adc_stuck_low_800a","adc_stuck_high_800a"])
            ok=all(~meas.current_valid(active));
            checks(end+1)=makecheck(char(type),'current_fail_closed',ok); %#ok<AGROW>
        end
    end
end
if isempty(checks),report.checks=struct([]);else,report.checks=checks;report.pass=all([checks.pass]);end
end
function active=window(t,f)
lo=-Inf;hi=Inf;if isfield(f,'start_s'),lo=f.start_s;end;if isfield(f,'end_s'),hi=f.end_s;end
active=t>=lo&t<=hi;
end
function c=makecheck(type,name,pass)
c=struct('fault_type',type,'check',name,'pass',logical(pass));
end
