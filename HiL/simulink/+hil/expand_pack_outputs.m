function output = expand_pack_outputs(physical, pack_cfg)
%EXPAND_PACK_OUTPUTS Apply explicit topology/sensor mappings and summaries.

Ns = double(pack_cfg.series_groups);
Nsegments = double(pack_cfg.segment_count);
Ntemp = double(pack_cfg.temperature_sensor_count);
mode = lower(char(pack_cfg.imbalance_model.mode));

voltage = double(physical.V_group(:));
soc = double(physical.SoC_group(:));
core = double(physical.T_core_group(:));
surface = double(physical.T_surf_group(:));
if numel(voltage) ~= Ns || numel(soc) ~= Ns || ...
        numel(core) ~= Ns || numel(surface) ~= Ns
    error('hil:pack:PhysicalDimensions', ...
        'Physical group-state vectors must all have series_groups elements.');
end

switch mode
    case 'uniform'
        % No output-level variation.

    case 'deterministic_spread'
        if Ns > 1
            index = (1:Ns).';
            raw_voltage = mod(index * 37, Ns) / (Ns - 1) - 0.5;
            voltage = mean(voltage) + ...
                pack_cfg.imbalance_model.voltage_peak_to_peak_V * raw_voltage;
            voltage = voltage + ...
                (sum(physical.V_group) - sum(voltage)) / Ns;

            raw_soc = mod(index * 29, Ns) / (Ns - 1) - 0.5;
            soc = min(max(mean(soc) + ...
                pack_cfg.imbalance_model.soc_peak_to_peak * raw_soc, 0), 1);
        end

    case 'parameter_distributed'
        % Physical reference runner already evolved each group independently.

    otherwise
        error('hil:pack:ImbalanceMode', 'Unknown imbalance mode "%s".', mode);
end

segment = accumarray( ...
    double(pack_cfg.group_to_segment(:)), voltage, [Nsegments, 1], @sum, 0);

sensor_group = double(pack_cfg.temp_sensor_to_group(:));
core_weight = double(pack_cfg.temp_sensor_core_weight(:));
temperature = core_weight .* core(sensor_group) + ...
    (1.0 - core_weight) .* surface(sensor_group);

if strcmp(mode, 'deterministic_spread')
    index = (1:Ntemp).';
    raw_temperature = (mod(index * 17, 41) - 20) / 20;
    temperature = temperature + ...
        0.5 * pack_cfg.imbalance_model.temperature_peak_to_peak_C * ...
        raw_temperature;
end

output = struct();
output.V_pack = sum(voltage);
output.T_core = mean(core);
output.T_surf = mean(surface);
output.SoC_true = mean(soc);
output.V_group = voltage;
output.V_segment = segment;
output.T_sensor = temperature;
output.SoC_group = soc;
output.V_min = min(voltage);
output.V_max = max(voltage);
output.T_max = max(temperature);
output.T_avg = mean(temperature);
output.Vp1 = mean(double(physical.Vp1_group));
output.Vp2 = mean(double(physical.Vp2_group));
end
