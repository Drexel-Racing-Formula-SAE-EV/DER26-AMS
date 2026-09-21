function out = current_window_model(epoch_time_s,true_current_A,cfg,seed)
%CURRENT_WINDOW_MODEL Run the Hall/ADC model at 50 Hz and publish 10 Hz windows.
%
% Plant current is zero-order-held between 100 ms plant epochs. Every MiL
% measurement epoch conservatively requires all Hall samples in its window to
% be valid. The published current is the mean of the valid 20 ms samples.
t = double(epoch_time_s(:));
itruth = double(true_current_A(:));
if numel(t) ~= numel(itruth) || numel(t) < 2 || any(diff(t) <= 0)
    error('mil:current:EpochContract','Current epochs must be increasing and aligned.');
end
dt = double(cfg.sample_time_s);
sub_t = (t(1):dt:t(end)).';
if sub_t(end) < t(end)-1e-10
    sub_t(end+1,1) = t(end); %#ok<AGROW>
end
sub_i = interp1(t,itruth,sub_t,'previous','extrap');
sub_cfg = cfg;
if isfield(sub_cfg,'faults') && ~isempty(sub_cfg.faults)
    for k = 1:numel(sub_cfg.faults)
        sub_cfg.faults(k).sample_time_s = dt;
    end
end
sub = mil.current_sensor_model(sub_i,sub_cfg,seed);

N = numel(t);
out = struct();
out.true_A = itruth;
out.current_A = nan(N,1);
out.current_50A_A = nan(N,1);
out.current_800A_A = nan(N,1);
out.sensor_voltage_50A_V = nan(N,1);
out.sensor_voltage_800A_V = nan(N,1);
out.ideal_sensor_voltage_50A_V = nan(N,1);
out.ideal_sensor_voltage_800A_V = nan(N,1);
out.adc_count_50A = zeros(N,1,'uint16');
out.adc_count_800A = zeros(N,1,'uint16');
out.valid = false(N,1);
out.range = zeros(N,1,'uint8');
out.reason = strings(N,1);
out.calibrated = repmat(logical(cfg.calibrated),N,1);
out.polarity_validated = repmat(logical(cfg.polarity_validated),N,1);
out.window_sample_count = zeros(N,1,'uint8');
out.subclock = struct('time_s',sub_t,'measurement',sub);

for n = 1:N
    if n == 1
        idx = find(sub_t >= t(n)-1e-10 & sub_t <= t(n)+1e-10);
    else
        idx = find(sub_t > t(n-1)+1e-10 & sub_t <= t(n)+1e-10);
    end
    if isempty(idx)
        out.reason(n) = "no_current_samples";
        continue;
    end
    out.window_sample_count(n) = uint8(min(numel(idx),255));
    all_valid = all(sub.valid(idx));
    out.valid(n) = all_valid;
    out.current_50A_A(n) = mean(sub.current_50A_A(idx));
    out.current_800A_A(n) = mean(sub.current_800A_A(idx));
    out.sensor_voltage_50A_V(n) = mean(sub.sensor_voltage_50A_V(idx));
    out.sensor_voltage_800A_V(n) = mean(sub.sensor_voltage_800A_V(idx));
    out.ideal_sensor_voltage_50A_V(n) = mean(sub.ideal_sensor_voltage_50A_V(idx));
    out.ideal_sensor_voltage_800A_V(n) = mean(sub.ideal_sensor_voltage_800A_V(idx));
    out.adc_count_50A(n) = sub.adc_count_50A(idx(end));
    out.adc_count_800A(n) = sub.adc_count_800A(idx(end));
    out.range(n) = sub.range(idx(end));
    if all_valid
        out.current_A(n) = mean(sub.current_A(idx));
        out.reason(n) = "ok";
    else
        first_bad = idx(find(~sub.valid(idx),1,'first'));
        out.reason(n) = sub.reason(first_bad);
        out.range(n) = uint8(0);
    end
end
end
