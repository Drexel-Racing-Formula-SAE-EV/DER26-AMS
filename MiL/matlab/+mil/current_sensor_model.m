function out = current_sensor_model(true_current_A, cfg, seed)
%CURRENT_SENSOR_MODEL Behavioral model of the dual-range AMS DHAB path.
%
% This model intentionally reproduces the *measurement path* semantics
% (5 V midpoint outputs, 100k/150k divider, 12-bit ADC, range hysteresis and
% cross-channel plausibility) while allowing controlled bias/gain/noise/fault
% injection. It is not a claim about unmeasured sensor transfer tolerances.

Itrue = double(true_current_A(:));
N = numel(Itrue);
stream = RandStream('mt19937ar', 'Seed', double(seed));

ADC_MAX = 4095;
DIV_GAIN = 150000 / (100000 + 150000);
S50 = 0.040;
S800 = 0.0025;
OFFSET = double(cfg.sensor_supply_V) * 0.5;
ADC_VREF = double(cfg.adc_vref_V);
SUPPLY_SCALE = double(cfg.sensor_supply_V) / 5.0;

fault50 = zeros(N,1);
fault800 = zeros(N,1);
gain50 = double(cfg.gain_50A) * ones(N,1);
gain800 = double(cfg.gain_800A) * ones(N,1);
polarity = double(cfg.polarity) * ones(N,1);
drop50 = repmat(~logical(cfg.channel_50A_enabled),N,1);
drop800 = repmat(~logical(cfg.channel_800A_enabled),N,1);
stuck50 = nan(N,1);
stuck800 = nan(N,1);
adc_stuck50 = nan(N,1);
adc_stuck800 = nan(N,1);

if isfield(cfg, 'faults') && ~isempty(cfg.faults)
    for k = 1:numel(cfg.faults)
        f = cfg.faults(k);
        start_s = field_or(f, 'start_s', -Inf);
        end_s = field_or(f, 'end_s', Inf);
        if isfield(f, 'sample_time_s')
            t = (0:N-1).' * double(f.sample_time_s);
        else
            t = (0:N-1).';
        end
        active = t >= start_s & t <= end_s;
        type = lower(string(f.type));
        value = field_or(f, 'value', 0);
        switch type
            case "bias_50a"
                fault50(active) = fault50(active) + value;
            case "bias_800a"
                fault800(active) = fault800(active) + value;
            case "gain_50a"
                gain50(active) = gain50(active) .* value;
            case "gain_800a"
                gain800(active) = gain800(active) .* value;
            case "polarity_reverse"
                polarity(active) = -polarity(active);
            case "dropout_50a"
                drop50(active) = true;
            case "dropout_800a"
                drop800(active) = true;
            case "stuck_50a"
                stuck50(active) = value;
            case "stuck_800a"
                stuck800(active) = value;
            case "adc_stuck_low_50a"
                adc_stuck50(active) = 0;
            case "adc_stuck_high_50a"
                adc_stuck50(active) = ADC_MAX;
            case "adc_stuck_low_800a"
                adc_stuck800(active) = 0;
            case "adc_stuck_high_800a"
                adc_stuck800(active) = ADC_MAX;
            otherwise
                error('mil:current:UnknownFault', ...
                    'Unknown current fault type %s.', type);
        end
    end
end

physical50_A = polarity .* Itrue .* gain50 + double(cfg.bias_50A_A) + fault50;
physical800_A = polarity .* Itrue .* gain800 + double(cfg.bias_800A_A) + fault800;
physical50_A = physical50_A + double(cfg.noise_50A_std_A) .* randn(stream,N,1);
physical800_A = physical800_A + double(cfg.noise_800A_std_A) .* randn(stream,N,1);
physical50_A(~isnan(stuck50)) = stuck50(~isnan(stuck50));
physical800_A(~isnan(stuck800)) = stuck800(~isnan(stuck800));

v50_ideal = OFFSET + physical50_A .* (S50 * SUPPLY_SCALE);
v800_ideal = OFFSET + physical800_A .* (S800 * SUPPLY_SCALE);
% The DHAB outputs saturate near their analog rails. Production firmware
% deliberately treats 0.30/4.70 V as a clamp indication while accepting the
% channel as electrically plausible until 0.20/4.80 V. Model that physical
% saturation so the +/-50 A channel can overrange cleanly and hand off to the
% +/-800 A channel instead of creating an impossible >5 V sensor output.
v50 = min(max(v50_ideal,0.25),4.75);
v800 = min(max(v800_ideal,0.25),4.75);
adc50_v = min(max(v50 .* DIV_GAIN, 0), ADC_VREF);
adc800_v = min(max(v800 .* DIV_GAIN, 0), ADC_VREF);
count50 = round(adc50_v ./ ADC_VREF .* ADC_MAX);
count800 = round(adc800_v ./ ADC_VREF .* ADC_MAX);
count50(drop50) = 0;
count800(drop800) = 0;
count50(~isnan(adc_stuck50)) = adc_stuck50(~isnan(adc_stuck50));
count800(~isnan(adc_stuck800)) = adc_stuck800(~isnan(adc_stuck800));

% Reconstruct using the nominal firmware conversion/calibration contract.
sensor50_v_rec = (count50 ./ ADC_MAX .* ADC_VREF) ./ DIV_GAIN;
sensor800_v_rec = (count800 ./ ADC_MAX .* ADC_VREF) ./ DIV_GAIN;
meas50 = (sensor50_v_rec - OFFSET) ./ (S50 * SUPPLY_SCALE) - ...
    double(cfg.zero_cal_offset_50A_A);
meas800 = (sensor800_v_rec - OFFSET) ./ (S800 * SUPPLY_SCALE) - ...
    double(cfg.zero_cal_offset_800A_A);
meas50(abs(meas50) < 0.25) = 0;
meas800(abs(meas800) < 2.0) = 0;

valid50 = count50 >= 100 & count50 <= 3800 & ...
    sensor50_v_rec >= 0.20 & sensor50_v_rec <= 4.80;
valid800 = count800 >= 100 & count800 <= 3800 & ...
    sensor800_v_rec >= 0.20 & sensor800_v_rec <= 4.80;
clamp50 = sensor50_v_rec <= 0.30 | sensor50_v_rec >= 4.70;
clamp800 = sensor800_v_rec <= 0.30 | sensor800_v_rec >= 4.70;

selected = nan(N,1);
range = zeros(N,1); % 0 invalid, 1=50A, 2=800A
valid = false(N,1);
reason = strings(N,1);
previous_range = 0;
for n = 1:N
    if ~valid50(n) || ~valid800(n)
        reason(n) = "adc_implausible";
        continue;
    end
    low_limit = 45.0;
    if previous_range == 2
        low_limit = 38.0;
    end
    force = lower(string(cfg.force_range));
    use50 = abs(meas50(n)) <= low_limit && ~clamp50(n);
    if force == "50a"
        use50 = true;
    elseif force == "800a"
        use50 = false;
    end
    if use50
        selected(n) = meas50(n);
        range(n) = 1;
        if abs(meas50(n)) >= 10.0 && ~clamp800(n)
            agreement = 7.5 + 0.15 * abs(meas50(n));
            if abs(meas50(n) - meas800(n)) > agreement
                range(n) = 0;
                valid(n) = false;
                reason(n) = "channel_mismatch";
                previous_range = 0;
                continue;
            end
        end
    else
        if clamp800(n)
            reason(n) = "sensor_saturation";
            previous_range = 0;
            continue;
        end
        selected(n) = meas800(n);
        range(n) = 2;
    end
    valid(n) = isfinite(selected(n));
    reason(n) = "ok";
    previous_range = range(n);
end

out = struct();
out.true_A = Itrue;
out.current_A = selected;
out.current_50A_A = meas50;
out.current_800A_A = meas800;
out.sensor_voltage_50A_V = v50;
out.sensor_voltage_800A_V = v800;
out.ideal_sensor_voltage_50A_V = v50_ideal;
out.ideal_sensor_voltage_800A_V = v800_ideal;
out.adc_count_50A = uint16(max(0,min(ADC_MAX,count50)));
out.adc_count_800A = uint16(max(0,min(ADC_MAX,count800)));
out.valid = valid;
out.range = uint8(range);
out.reason = reason;
out.calibrated = repmat(logical(cfg.calibrated),N,1);
out.polarity_validated = repmat(logical(cfg.polarity_validated),N,1);
end

function value = field_or(s, name, default)
if isfield(s, name)
    value = s.(name);
else
    value = default;
end
end
