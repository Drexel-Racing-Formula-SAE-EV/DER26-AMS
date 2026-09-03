function [meas, log] = apply_sensor_faults(meas, faults, time_s, pack_cfg)
%APPLY_SENSOR_FAULTS Deterministic measurement-bus fault injection.
log = struct('type',{},'start_s',{},'end_s',{},'target',{},'value',{});
if isempty(faults)
    return;
end
for k = 1:numel(faults)
    f = faults(k);
    type = lower(string(f.type));
    start_s = field_or(f,'start_s',-Inf);
    end_s = field_or(f,'end_s',Inf);
    active = time_s >= start_s & time_s <= end_s;
    target = field_or(f,'target',0);
    value = field_or(f,'value',0);
    switch type
        case "cell_bias"
            idx = checked_index(target,size(meas.cell_voltage_V,2),'cell');
            meas.cell_voltage_V(active,idx) = meas.cell_voltage_V(active,idx) + value;
        case "cell_stuck"
            idx = checked_index(target,size(meas.cell_voltage_V,2),'cell');
            meas.cell_voltage_V(active,idx) = value;
        case {"cell_dropout","cell_invalid","pec_invalid_cell","cell_open_wire_like"}
            idx = checked_index(target,size(meas.cell_voltage_V,2),'cell');
            meas.cell_valid(active,idx) = false;
            meas.cell_voltage_V(:,idx) = freeze_during_window( ...
                meas.cell_voltage_V(:,idx),active);
        case "segment_pec_invalid"
            seg = checked_index(target,double(pack_cfg.segment_count),'segment');
            indices = find(double(pack_cfg.group_to_segment(:)) == seg);
            meas.cell_valid(active,indices) = false;
            for idx = indices(:).'
                meas.cell_voltage_V(:,idx) = freeze_during_window( ...
                    meas.cell_voltage_V(:,idx),active);
            end
        case "cell_stale"
            idx = checked_index(target,size(meas.cell_voltage_V,2),'cell');
            age = max(0,value);
            meas.cell_age_ms(active,idx) = age;
            first = find(active,1,'first');
            if first > 1
                meas.cell_voltage_V(active,idx) = meas.cell_voltage_V(first-1,idx);
            end
        case "temperature_bias"
            idx = checked_index(target,size(meas.temperature_C,2),'temperature');
            meas.temperature_C(active,idx) = meas.temperature_C(active,idx) + value;
        case "temperature_stuck"
            idx = checked_index(target,size(meas.temperature_C,2),'temperature');
            meas.temperature_C(active,idx) = value;
        case {"temperature_dropout","temperature_invalid"}
            idx = checked_index(target,size(meas.temperature_C,2),'temperature');
            meas.temperature_valid(active,idx) = false;
            meas.temperature_C(:,idx) = freeze_during_window( ...
                meas.temperature_C(:,idx),active);
        case "temperature_stale"
            idx = checked_index(target,size(meas.temperature_C,2),'temperature');
            age = max(0,value);
            meas.temperature_age_ms(active,idx) = age;
            first = find(active,1,'first');
            if first > 1
                meas.temperature_C(active,idx) = meas.temperature_C(first-1,idx);
            end
        case "pack_current_dropout"
            meas.current_valid(active) = false;
            meas.current.valid(active) = false;
            meas.pack_current_A = freeze_during_window(meas.pack_current_A,active);
            meas.current.current_A = meas.pack_current_A;
            meas.current.reason(active) = "injected_dropout";
        case "pack_current_stuck"
            meas.pack_current_A(active) = value;
            meas.current.current_A(active) = value;
        case "pack_current_bias"
            meas.pack_current_A(active) = meas.pack_current_A(active) + value;
            meas.current.current_A(active) = meas.pack_current_A(active);
        case "pack_current_polarity"
            meas.pack_current_A(active) = -meas.pack_current_A(active);
            meas.current.current_A(active) = meas.pack_current_A(active);
        case "sequence_repeat"
            first = find(active,1,'first');
            if first > 1
                meas.sequence(active) = meas.sequence(first-1);
            else
                meas.sequence(active) = uint32(0);
            end
        case "timestamp_offset"
            meas.timestamp_s(active) = max(0,meas.timestamp_s(active) + value);
        otherwise
            error('mil:fault:UnknownType','Unknown sensor fault type %s.',type);
    end
    log(end+1) = struct('type',char(type),'start_s',start_s,'end_s',end_s, ... %#ok<AGROW>
        'target',target,'value',value);
end
end

function x = freeze_during_window(x,active)
first = find(active,1,'first');
if isempty(first)
    return;
end
if first > 1
    frozen = x(first-1);
else
    frozen = x(first);
end
x(active) = frozen;
end

function index = checked_index(value,count,description)
index = double(value);
if ~isscalar(index) || ~isfinite(index) || index < 1 || index > count || mod(index,1) ~= 0
    error('mil:fault:Index','Fault %s index out of range.',description);
end
end

function value = field_or(s,name,default)
if isfield(s,name), value=s.(name); else, value=default; end
end
