function fit = fit_r2c2(normalized_tests, fast_fit, cell_cfg)
%FIT_R2C2 Fit one scalar slow branch after subtracting the fast branch.

[cell_cfg, ~] = hil.validate_configuration(cell_cfg);
options = cell_cfg.parameter_fit_options;
temperatures = double(cell_cfg.temperature_breakpoints_C(:));
soc_regions = double(options.hcgt_soc_regions(:));
target_current = double(cell_cfg.nominal_capacity_Ah);
tolerance = options.current_tolerance_fraction * target_current;
accepted = zeros(0, 6);

for test_index = 1:numel(normalized_tests)
    test = normalized_tests(test_index);
    temperature = double(test.test_temperature_C);
    if min(abs(temperatures - temperature)) > 0.5
        continue;
    end

    time = double(test.time_s(:));
    current = double(test.cell_current_A(:));
    voltage = double(test.cell_voltage_V(:));
    pulse_mask = current > 0 & abs(current - target_current) <= tolerance;
    [starts, ends] = local_segments(pulse_mask);
    count = min(numel(starts), numel(soc_regions));

    for pulse_index = 1:count
        start_index = starts(pulse_index);
        end_index = ends(pulse_index);
        duration = time(end_index) - time(start_index);
        pulse_current = mean(current(start_index:end_index), 'omitnan');
        if duration < 1.0 || duration > 180.0 || ...
                pulse_current < 0.35 * target_current || ...
                pulse_current > 1.90 * target_current
            continue;
        end

        rest_start = end_index + 1;
        if rest_start >= numel(time)
            continue;
        end
        next_pulse = find(abs(current(rest_start:end)) > ...
            options.rest_current_threshold_A, 1, 'first');
        if isempty(next_pulse)
            rest_end = numel(time);
        else
            rest_end = rest_start + next_pulse - 3;
        end
        if rest_end <= rest_start + 20
            continue;
        end

        relaxation_time = time(rest_start:rest_end) - time(rest_start);
        relaxation_voltage = voltage(rest_start:rest_end);
        window = relaxation_time >= 0.5 & relaxation_time <= 240.0;
        relaxation_time = relaxation_time(window);
        relaxation_voltage = relaxation_voltage(window);
        if numel(relaxation_time) < 30
            continue;
        end

        tail_count = max(5, round(0.15 * numel(relaxation_voltage)));
        asymptote = median(relaxation_voltage( ...
            (end - tail_count + 1):end), 'omitnan');
        total_recovery = asymptote - relaxation_voltage;
        if max(total_recovery) < 0.003
            continue;
        end

        soc = test_pulse_soc(test, pulse_index, ...
            start_index, soc_regions);
        r1 = local_lookup(fast_fit.R1, cell_cfg, soc, temperature);
        c1 = local_lookup(fast_fit.C1, cell_cfg, soc, temperature);
        tau1 = r1 * c1;
        if ~isfinite(tau1) || tau1 < options.tau1_bounds_s(1) || ...
                tau1 > options.tau1_bounds_s(2)
            continue;
        end

        fast_end_amplitude = pulse_current * r1 * ...
            (1.0 - exp(-duration / tau1));
        slow_recovery = total_recovery - ...
            fast_end_amplitude .* exp(-relaxation_time ./ tau1);
        peak = max(slow_recovery);
        if ~isfinite(peak) || peak < 0.001
            continue;
        end

        selected = slow_recovery > max(0.0005, 0.05 * peak) & ...
            relaxation_time >= 5.0;
        fit_time = relaxation_time(selected);
        fit_recovery = slow_recovery(selected);
        if numel(fit_time) < 12
            continue;
        end
        fit_time = fit_time - fit_time(1);
        line = polyfit(fit_time, log(fit_recovery), 1);
        if line(1) >= 0
            continue;
        end

        tau2 = -1.0 / line(1);
        amplitude2 = exp(line(2));
        if tau2 < options.tau2_bounds_s(1) || tau2 > options.tau2_bounds_s(2)
            continue;
        end
        denominator = pulse_current * (1.0 - exp(-duration / tau2));
        if denominator <= 0
            continue;
        end
        r2 = amplitude2 / denominator;
        c2 = tau2 / r2;
        if r2 < options.r2_bounds_ohm(1) || r2 > options.r2_bounds_ohm(2) || ...
                c2 < options.c2_bounds_F(1) || c2 > options.c2_bounds_F(2)
            continue;
        end
        accepted(end + 1, :) = [temperature, soc, r2, c2, tau2, pulse_current]; %#ok<AGROW>
    end
end

if size(accepted, 1) < options.minimum_valid_slow_fits
    r2 = options.fallback_R2_ohm;
    c2 = options.fallback_C2_F;
    fallback_used = true;
else
    r2 = median(accepted(:, 3), 'omitnan');
    tau2 = median(accepted(:, 5), 'omitnan');
    c2 = tau2 / r2;
    fallback_used = false;
end

fit = struct();
fit.R2 = single(r2);
fit.C2 = single(c2);
fit.accepted = array2table(accepted, 'VariableNames', { ...
    'temperature_C', 'soc', 'R2_ohm', 'C2_F', 'tau2_s', 'pulse_current_A'});
fit.report = struct( ...
    'method', 'slow residual after fast-branch subtraction', ...
    'accepted_fit_count', size(accepted, 1), ...
    'fallback_used', fallback_used);
end

function value = test_pulse_soc(test, pulse_index, start_index, fallback)
value = NaN;
if isfield(test, 'pulse_soc') && isfinite(test.pulse_soc)
    value = double(test.pulse_soc);
elseif isfield(test, 'soc') && numel(test.soc) >= start_index && ...
        isfinite(test.soc(start_index))
    value = double(test.soc(start_index));
elseif pulse_index <= numel(fallback)
    value = double(fallback(pulse_index));
end
value = min(max(value, 0.0), 1.0);
end

function value = local_lookup(table_data, cell_cfg, soc, temperature)
value = interp2( ...
    double(cell_cfg.temperature_breakpoints_C(:).'), ...
    double(cell_cfg.soc_breakpoints(:)), ...
    double(table_data), ...
    temperature, soc, 'linear');
end

function [starts, ends] = local_segments(mask)
edges = diff([false; mask(:) ~= 0; false]);
starts = find(edges == 1);
ends = find(edges == -1) - 1;
end
