function script = codegen_chart_script(pack_cfg)
%CODEGEN_CHART_SCRIPT Generate fixed-size, codegen-safe pack output expansion.
%
% The configured Simulink plant retains one representative 2RC electrothermal
% state. "parameter_distributed" can be inspected as an output-level proxy in
% Simulink, but hil.generate_code blocks that mode. Use hil.run_reference for
% independently evolved per-group states.

[pack_cfg, ~] = hil.validate_configuration(pack_cfg);
Ns = double(pack_cfg.series_groups);
Nsegments = double(pack_cfg.segment_count);
Ntemp = double(pack_cfg.temperature_sensor_count);
mode = lower(char(pack_cfg.imbalance_model.mode));

voltage_offset = zeros(Ns, 1);
soc_offset = zeros(Ns, 1);
temperature_offset = zeros(Ntemp, 1);

switch mode
    case 'uniform'
        mode_level = 0;

    case 'deterministic_spread'
        mode_level = 1;
        index = (1:Ns).';
        if Ns > 1
            voltage_offset = ...
                double(pack_cfg.imbalance_model.voltage_peak_to_peak_V) .* ...
                (mod(index * 37, Ns) ./ (Ns - 1) - 0.5);
            soc_offset = ...
                double(pack_cfg.imbalance_model.soc_peak_to_peak) .* ...
                (mod(index * 29, Ns) ./ (Ns - 1) - 0.5);
        end
        sensor_index = (1:Ntemp).';
        temperature_offset = ...
            0.5 * double(pack_cfg.imbalance_model.temperature_peak_to_peak_C) .* ...
            ((mod(sensor_index * 17, 41) - 20) ./ 20);

    case 'parameter_distributed'
        mode_level = 2;
        group = hil.build_group_parameters(struct('id', 'chart_proxy'), pack_cfg);
        voltage_offset = centered_shape(group.r0_multiplier, ...
            0.5 * double(pack_cfg.imbalance_model.voltage_peak_to_peak_V));
        soc_offset = group.initial_soc_offset;
        sensor_groups = double(pack_cfg.temp_sensor_to_group(:));
        temperature_offset = centered_shape( ...
            group.rsa_multiplier(sensor_groups), ...
            0.5 * double(pack_cfg.imbalance_model.temperature_peak_to_peak_C));

    otherwise
        error('hil:model:ImbalanceMode', ...
            'Unsupported pack output mode "%s".', mode);
end

lines = {
    'function [V_group, V_segment, T_sensor, SoC_group, V_min, V_max, T_max, T_avg] = Accumulator_Output_Expansion(V_pack, T_core, T_surf, SoC_true)'
    '%#codegen'
    sprintf('%% Generated output expansion: level %d (%s).', mode_level, mode)
    sprintf('V_group = zeros(%d,1,''single'');', Ns)
    sprintf('V_segment = zeros(%d,1,''single'');', Nsegments)
    sprintf('T_sensor = zeros(%d,1,''single'');', Ntemp)
    sprintf('SoC_group = zeros(%d,1,''single'');', Ns)
    sprintf('group_to_segment = uint16(%s);', ...
        numeric_literal(pack_cfg.group_to_segment(:)))
    sprintf('sensor_to_group = uint16(%s);', ...
        numeric_literal(pack_cfg.temp_sensor_to_group(:)))
    sprintf('sensor_core_weight = single(%s);', ...
        numeric_literal(pack_cfg.temp_sensor_core_weight(:)))
    sprintf('voltage_offset = single(%s);', numeric_literal(voltage_offset))
    sprintf('soc_offset = single(%s);', numeric_literal(soc_offset))
    sprintf('temperature_offset = single(%s);', ...
        numeric_literal(temperature_offset))
    'V_pack_s = single(V_pack);'
    'T_core_s = single(T_core);'
    'T_surf_s = single(T_surf);'
    'SoC_s = single(SoC_true);'
    sprintf('V_nom = V_pack_s / single(%d.0);', Ns)
    'sum_v = single(0.0);'
    sprintf('for i = 1:%d', Ns)
    '    V_group(i) = V_nom + voltage_offset(i);'
    '    sum_v = sum_v + V_group(i);'
    'end'
    sprintf('correction = (V_pack_s - sum_v) / single(%d.0);', Ns)
    sprintf('for i = 1:%d', Ns)
    '    V_group(i) = V_group(i) + correction;'
    '    segment_index = int32(group_to_segment(i));'
    '    V_segment(segment_index) = V_segment(segment_index) + V_group(i);'
    '    value = SoC_s + soc_offset(i);'
    '    if value > single(1.0)'
    '        value = single(1.0);'
    '    elseif value < single(0.0)'
    '        value = single(0.0);'
    '    end'
    '    SoC_group(i) = value;'
    'end'
    sprintf('for i = 1:%d', Ntemp)
    '    group_index = int32(sensor_to_group(i));'
    '    weight = sensor_core_weight(i);'
    '    T_sensor(i) = weight * T_core_s + ...'
    '        (single(1.0) - weight) * T_surf_s + temperature_offset(i);'
    'end'
    'V_min = V_group(1);'
    'V_max = V_group(1);'
    sprintf('for i = 2:%d', Ns)
    '    if V_group(i) < V_min'
    '        V_min = V_group(i);'
    '    end'
    '    if V_group(i) > V_max'
    '        V_max = V_group(i);'
    '    end'
    'end'
    'T_max = T_sensor(1);'
    'sum_t = single(0.0);'
    sprintf('for i = 1:%d', Ntemp)
    '    if T_sensor(i) > T_max'
    '        T_max = T_sensor(i);'
    '    end'
    '    sum_t = sum_t + T_sensor(i);'
    'end'
    sprintf('T_avg = sum_t / single(%d.0);', Ntemp)
    };
script = strjoin(lines, newline);
end

function output = centered_shape(values, half_range)
values = double(values(:));
values = values - mean(values);
peak = max(abs(values));
if peak > 0
    output = values * (double(half_range) / peak);
else
    output = zeros(size(values));
end
end

function value = numeric_literal(values)
values = double(values(:));
tokens = arrayfun(@(x) sprintf('%.9g', x), values, ...
    'UniformOutput', false);
value = ['[', strjoin(tokens, ';'), ']'];
end
