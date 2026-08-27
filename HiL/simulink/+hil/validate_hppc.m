function report = validate_hppc(test, result, pack_cfg, acceptance_cfg)
%VALIDATE_HPPC Report pulse onset, loaded, and relaxation errors.

if nargin < 4
    acceptance_cfg = [];
end

base = hil.validate_dynamic_profile(test, result, pack_cfg, []);
time = double(test.time_s(:));
current = double(test.cell_current_A(:));
measured = double(test.cell_voltage_V(:));
predicted = interp1(double(result.time_s(:)), ...
    double(result.V_pack(:)) / double(pack_cfg.series_groups), ...
    time, 'linear', NaN);

threshold = max(0.20, 0.10 * max(abs(current)));
active = abs(current) > threshold;
edges = diff([false; active; false]);
starts = find(edges == 1);
ends = find(edges == -1) - 1;
pulse_rows = zeros(numel(starts), 8);
all_loaded_error = [];
all_relaxation_error = [];

for pulse = 1:numel(starts)
    first = starts(pulse);
    last = ends(pulse);
    relax_last = min(numel(time), last + max(1, round(30 / median(diff(time)))));
    onset_error = predicted(first) - measured(first);
    loaded_error = predicted(first:last) - measured(first:last);
    relaxation_error = predicted((last + 1):relax_last) - ...
        measured((last + 1):relax_last);
    all_loaded_error = [ ...
        all_loaded_error; loaded_error(isfinite(loaded_error))]; %#ok<AGROW>
    all_relaxation_error = [ ...
        all_relaxation_error; ...
        relaxation_error(isfinite(relaxation_error))]; %#ok<AGROW>
    pulse_rows(pulse, :) = [ ...
        pulse, time(first), time(last), mean(current(first:last), 'omitnan'), ...
        onset_error, maximum_absolute(loaded_error), ...
        root_mean_square(loaded_error), root_mean_square(relaxation_error)];
end

report = base;
report.pulses = array2table(pulse_rows, 'VariableNames', { ...
    'pulse', 'start_time_s', 'end_time_s', 'mean_current_A', ...
    'onset_error_V', 'maximum_loaded_error_V', ...
    'loaded_rms_error_V', 'relaxation_rms_error_V'});
report.pulse_count = numel(starts);
report.loaded_voltage_error = hil.metrics(all_loaded_error);
report.relaxation_voltage_error = hil.metrics(all_relaxation_error);
report.test_class = 'hppc';
report.acceptance_evaluated = ~isempty(acceptance_cfg);
if report.acceptance_evaluated
    report.acceptance = hil.evaluate_validation_acceptance( ...
        report, acceptance_cfg, report.test_class);
    report.passed = report.acceptance.passed;
    if strcmp(report.thermal_validation.status, 'EVALUATED')
        checks = report.acceptance.checks;
        if checks.surface_temperature_available && ...
                checks.surface_temperature_rms_error && ...
                checks.surface_temperature_maximum_error
            report.thermal_validation.status = 'PASS';
        else
            report.thermal_validation.status = 'FAIL';
        end
    end
else
    report.acceptance = struct();
    report.passed = false;
end
end

function value = maximum_absolute(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(abs(values));
end
end

function value = root_mean_square(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = sqrt(mean(values .^ 2));
end
end
