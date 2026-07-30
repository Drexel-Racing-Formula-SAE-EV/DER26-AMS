function output = apply_fault_injection(output, fault, time_s, pack_cfg)
%APPLY_FAULT_INJECTION Apply configured logical sensor/image faults.

if ~isfield(fault, 'enabled') || ~logical(fault.enabled)
    return;
end
start_time = field_or(fault, 'start_time_s', -Inf);
end_time = field_or(fault, 'end_time_s', Inf);
if time_s < start_time || time_s > end_time
    return;
end

if isfield(fault, 'group_index')
    index = checked_index(fault.group_index, numel(output.V_group), 'group');
    output.V_group(index) = output.V_group(index) + ...
        field_or(fault, 'group_voltage_offset_V', 0.0);
    if logical(field_or(fault, 'group_voltage_nan', false))
        output.V_group(index) = NaN;
    end
    output.V_segment = accumarray( ...
        double(pack_cfg.group_to_segment(:)), output.V_group(:), ...
        [pack_cfg.segment_count, 1], @sum, 0);
    output.V_min = min(output.V_group);
    output.V_max = max(output.V_group);
end

if isfield(fault, 'sensor_index')
    index = checked_index( ...
        fault.sensor_index, numel(output.T_sensor), 'temperature sensor');
    output.T_sensor(index) = output.T_sensor(index) + ...
        field_or(fault, 'temperature_offset_C', 0.0);
    if logical(field_or(fault, 'temperature_nan', false))
        output.T_sensor(index) = NaN;
    end
    output.T_max = max(output.T_sensor);
    output.T_avg = mean(output.T_sensor);
end

if isfield(fault, 'soc_group_index')
    index = checked_index( ...
        fault.soc_group_index, numel(output.SoC_group), 'SoC group');
    output.SoC_group(index) = min(max(output.SoC_group(index) + ...
        field_or(fault, 'soc_offset', 0.0), 0), 1);
end
output.V_pack = output.V_pack + field_or(fault, 'pack_voltage_offset_V', 0.0);
end

function value = field_or(source, name, default)
if isfield(source, name)
    value = source.(name);
else
    value = default;
end
end

function index = checked_index(value, count, description)
index = double(value);
if ~isscalar(index) || ~isfinite(index) || ...
        index < 1 || index > count || mod(index, 1) ~= 0
    error('hil:fault:Index', ...
        'Fault %s index must select one configured output.', description);
end
end
