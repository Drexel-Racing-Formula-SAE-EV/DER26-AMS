function fit = fit_r1c1(normalized_tests, cell_cfg)
%FIT_R1C1 Fit the fast polarization branch from pulse relaxation.

[cell_cfg, ~] = hil.validate_configuration(cell_cfg);
options = cell_cfg.parameter_fit_options;
soc_regions = double(options.hcgt_soc_regions(:));
temperatures = double(cell_cfg.temperature_breakpoints_C(:));
target_current = double(cell_cfg.nominal_capacity_Ah);
tolerance = options.current_tolerance_fraction * target_current;

rows = zeros(0, 7);
row_cell_ids = {};
row_sources = {};

for test_index = 1:numel(normalized_tests)
    test = normalized_tests(test_index);
    temperature_index = find(abs(temperatures - test.test_temperature_C) <= 0.5, ...
        1, 'first');
    if isempty(temperature_index)
        continue;
    end

    time = double(test.time_s(:));
    current = double(test.cell_current_A(:));
    voltage = double(test.cell_voltage_V(:));
    pulse_mask = current > 0 & abs(current - target_current) <= tolerance;
    [starts, ends] = local_segments(pulse_mask);
    keep = false(size(starts));
    for segment_index = 1:numel(starts)
        duration = time(ends(segment_index)) - time(starts(segment_index));
        average_current = mean(current(starts(segment_index):ends(segment_index)), ...
            'omitnan');
        keep(segment_index) = duration >= 1.0 && duration <= 180.0 && ...
            average_current >= 0.35 * target_current && ...
            average_current <= 1.90 * target_current;
    end
    starts = starts(keep);
    ends = ends(keep);
    count = min(numel(starts), numel(soc_regions));

    for pulse_index = 1:count
        start_index = starts(pulse_index);
        end_index = ends(pulse_index);
        pulse_current = mean(current(start_index:end_index), 'omitnan');
        pulse_duration = time(end_index) - time(start_index);

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
        if rest_end <= rest_start + 10
            continue;
        end

        relaxation_time = time(rest_start:rest_end) - time(rest_start);
        relaxation_voltage = voltage(rest_start:rest_end);
        window = relaxation_time >= 0.5 & relaxation_time <= 120.0;
        [tau, amplitude, ok] = local_fit_exp_recovery( ...
            relaxation_time(window), relaxation_voltage(window), ...
            options.tau1_bounds_s);
        if ~ok
            continue;
        end

        denominator = pulse_current * (1.0 - exp(-pulse_duration / tau));
        if denominator <= 0
            continue;
        end
        r1 = amplitude / denominator;
        c1 = tau / r1;
        if r1 < options.r1_bounds_ohm(1) || ...
                r1 > options.r1_bounds_ohm(2) || ...
                c1 < options.c1_bounds_F(1) || c1 > options.c1_bounds_F(2)
            continue;
        end

        pulse_soc = test_pulse_soc(test, pulse_index, ...
            start_index, soc_regions);
        rows(end + 1, :) = [ ... %#ok<AGROW>
            temperature_index, pulse_index, pulse_soc, ...
            r1, c1, tau, pulse_current];
        row_cell_ids{end + 1, 1} = test.cell_id; %#ok<AGROW>
        row_sources{end + 1, 1} = test.source_file; %#ok<AGROW>
    end
end

soc_axis = double(cell_cfg.soc_breakpoints(:));
r1_grid = nan(numel(soc_axis), numel(temperatures));
c1_grid = nan(numel(soc_axis), numel(temperatures));
accepted_by_temperature = zeros(size(temperatures));

for temperature_index = 1:numel(temperatures)
    selected = rows(:, 1) == temperature_index;
    accepted_by_temperature(temperature_index) = nnz(selected);
    if nnz(selected) < options.minimum_valid_fast_fits
        r1_grid(:, temperature_index) = options.fallback_R1_ohm;
        c1_grid(:, temperature_index) = options.fallback_C1_F;
        continue;
    end

    source_soc = unique(rows(selected, 3));
    median_r1 = zeros(size(source_soc));
    median_c1 = zeros(size(source_soc));
    for index = 1:numel(source_soc)
        at_soc = selected & abs(rows(:, 3) - source_soc(index)) < 1e-9;
        median_r1(index) = median(rows(at_soc, 4), 'omitnan');
        median_c1(index) = median(rows(at_soc, 5), 'omitnan');
    end

    if numel(source_soc) < 3
        r1_grid(:, temperature_index) = options.fallback_R1_ohm;
        c1_grid(:, temperature_index) = options.fallback_C1_F;
    else
        [source_soc, order] = sort(source_soc);
        median_r1 = median_r1(order);
        median_c1 = median_c1(order);
        r1_grid(:, temperature_index) = interp1( ...
            source_soc, median_r1, soc_axis, 'linear', 'extrap');
        c1_grid(:, temperature_index) = interp1( ...
            source_soc, median_c1, soc_axis, 'linear', 'extrap');
    end
end

r1_grid = min(max(r1_grid, options.r1_bounds_ohm(1)), ...
    options.r1_bounds_ohm(2));
c1_grid = min(max(c1_grid, options.c1_bounds_F(1)), ...
    options.c1_bounds_F(2));

accepted = table();
if ~isempty(rows)
    accepted = array2table(rows, 'VariableNames', { ...
        'temperature_index', 'pulse_index', 'soc', ...
        'R1_ohm', 'C1_F', 'tau1_s', 'pulse_current_A'});
    accepted.cell_id = row_cell_ids;
    accepted.source_file = row_sources;
end

fit = struct();
fit.R1 = single(r1_grid);
fit.C1 = single(c1_grid);
fit.inv_C1 = single(1.0 ./ c1_grid);
fit.neg_inv_R1C1 = single(-1.0 ./ (r1_grid .* c1_grid));
fit.accepted = accepted;
fit.report = struct( ...
    'method', 'single-exponential pulse-relaxation recovery', ...
    'accepted_by_temperature', accepted_by_temperature, ...
    'fallback_used', accepted_by_temperature < options.minimum_valid_fast_fits);
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

function [starts, ends] = local_segments(mask)
edges = diff([false; mask(:) ~= 0; false]);
starts = find(edges == 1);
ends = find(edges == -1) - 1;
end

function [tau, amplitude, ok] = local_fit_exp_recovery(time, voltage, bounds)
tau = NaN;
amplitude = NaN;
ok = false;
time = double(time(:));
voltage = double(voltage(:));
valid = isfinite(time) & isfinite(voltage);
time = time(valid);
voltage = voltage(valid);
if numel(time) < 20
    return;
end

tail_count = max(5, round(0.15 * numel(voltage)));
asymptote = median(voltage((end - tail_count + 1):end), 'omitnan');
recovery = asymptote - voltage;
peak = max(recovery);
if ~isfinite(peak) || peak < 0.002
    return;
end
selected = recovery > max(0.0005, 0.03 * peak);
fit_time = time(selected);
fit_recovery = recovery(selected);
if numel(fit_time) < 10
    return;
end

fit_time = fit_time - fit_time(1);
line = polyfit(fit_time, log(fit_recovery), 1);
if line(1) >= 0
    return;
end
tau_candidate = -1.0 / line(1);
amplitude_candidate = exp(line(2));
if tau_candidate < bounds(1) || tau_candidate > bounds(2) || ...
        amplitude_candidate < 0.001 || amplitude_candidate > 1.0
    return;
end
tau = tau_candidate;
amplitude = amplitude_candidate;
ok = true;
end
