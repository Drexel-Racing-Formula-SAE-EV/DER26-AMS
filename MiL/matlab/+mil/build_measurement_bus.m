function meas = build_measurement_bus(truth, sensor_cfg, pack_cfg)
%BUILD_MEASUREMENT_BUS Build the only bus visible to estimator/BMS algorithms.
%
% The adapter adds independent measurement effects after the physical plant.
% Faults are therefore measurement faults rather than changes to hidden truth.
N = numel(truth.time_s);
Ns = size(truth.group_voltage_V,2);
Nt = size(truth.temp_sensor_C,2);
stream = RandStream('mt19937ar','Seed',double(sensor_cfg.seed));

meas = struct();
meas.tag = 'AMS_MEASUREMENT_BUS';
meas.time_s = truth.time_s;
meas.sequence = truth.sequence;
meas.timestamp_s = truth.time_s;
if sensor_cfg.timestamp_jitter_std_s > 0
    meas.timestamp_s = meas.timestamp_s + ...
        sensor_cfg.timestamp_jitter_std_s .* randn(stream,N,1);
    meas.timestamp_s = cummax(max(meas.timestamp_s,0));
end

current_cfg = sensor_cfg.current;
meas.current = mil.current_window_model(truth.time_s,truth.pack_current_A, ...
    current_cfg,sensor_cfg.seed + 17);
meas.pack_current_A = meas.current.current_A;
meas.current_valid = meas.current.valid;
meas.current_calibrated = meas.current.calibrated;
meas.current_polarity_validated = meas.current.polarity_validated;

meas.cell_voltage_V = truth.group_voltage_V + sensor_cfg.voltage_bias_V + ...
    sensor_cfg.voltage_noise_std_V .* randn(stream,N,Ns);
if sensor_cfg.voltage_quantization_V > 0
    q = sensor_cfg.voltage_quantization_V;
    meas.cell_voltage_V = q .* round(meas.cell_voltage_V ./ q);
end
meas.cell_valid = true(N,Ns);
meas.cell_age_ms = sensor_cfg.default_cell_age_ms .* ones(N,Ns);

meas.temperature_C = truth.temp_sensor_C + sensor_cfg.temperature_bias_C + ...
    sensor_cfg.temperature_noise_std_C .* randn(stream,N,Nt);
if sensor_cfg.temperature_quantization_C > 0
    q = sensor_cfg.temperature_quantization_C;
    meas.temperature_C = q .* round(meas.temperature_C ./ q);
end
meas.temperature_valid = true(N,Nt);
meas.temperature_age_ms = sensor_cfg.default_temp_age_ms .* ones(N,Nt);

[meas, fault_log] = mil.apply_sensor_faults(meas, sensor_cfg.faults, ...
    truth.time_s, pack_cfg);
meas.fault_log = fault_log;
meas.cell_fresh = meas.cell_valid & isfinite(meas.cell_voltage_V) & ...
    meas.cell_age_ms <= double(sensor_cfg.max_cell_age_ms);
meas.temperature_fresh = meas.temperature_valid & isfinite(meas.temperature_C) & ...
    meas.temperature_age_ms <= double(sensor_cfg.max_temp_age_ms);

Nseg = double(pack_cfg.segment_count);
meas.segment_voltage_V = nan(N,Nseg);
meas.segment_cell_usable_mask = zeros(N,Nseg,'uint16');
meas.segment_max_cell_age_ms = zeros(N,Nseg);
meas.segment_surface_max_C = nan(N,Nseg);
for s = 1:Nseg
    group_mask = double(pack_cfg.group_to_segment(:)) == s;
    group_indices = find(group_mask);
    valid = meas.cell_fresh(:,group_indices);
    volt = meas.cell_voltage_V(:,group_indices);
    volt(~valid) = 0;
    meas.segment_voltage_V(:,s) = sum(volt,2);
    full = all(valid,2);
    meas.segment_voltage_V(~full,s) = NaN;
    bits = zeros(N,1,'uint16');
    for j = 1:numel(group_indices)
        bits = bitor(bits, bitshift(uint16(valid(:,j)),j-1));
    end
    meas.segment_cell_usable_mask(:,s) = bits;
    meas.segment_max_cell_age_ms(:,s) = max(meas.cell_age_ms(:,group_indices),[],2);

    sensor_groups = double(pack_cfg.temp_sensor_to_group(:));
    temp_indices = find(ismember(sensor_groups,group_indices));
    tvalid = meas.temperature_fresh(:,temp_indices);
    temp = meas.temperature_C(:,temp_indices);
    temp(~tvalid) = -Inf;
    tmax = max(temp,[],2);
    tmax(~any(tvalid,2)) = NaN;
    meas.segment_surface_max_C(:,s) = tmax;
end

meas.pack_voltage_V = sum(meas.cell_voltage_V .* meas.cell_valid,2);
meas.pack_voltage_valid = all(meas.cell_fresh,2);
meas.pack_voltage_V(~meas.pack_voltage_valid) = NaN;
meas.measurement_valid = meas.pack_voltage_valid & meas.current_valid & ...
    all(meas.segment_cell_usable_mask == uint16(32767),2) & ...
    all(isfinite(meas.segment_surface_max_C),2);
end
