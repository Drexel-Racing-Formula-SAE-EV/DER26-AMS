function report = validate_dynamic_profile( ...
    test, result, pack_cfg, acceptance_cfg)
%VALIDATE_DYNAMIC_PROFILE Compare a normalized dynamic holdout against output.

if nargin < 4
    acceptance_cfg = [];
end

time = double(test.time_s(:));
measured_voltage = double(test.cell_voltage_V(:));
model_cell_voltage = interp1(double(result.time_s(:)), ...
    double(result.V_pack(:)) / double(pack_cfg.series_groups), ...
    time, 'linear', NaN);
valid = isfinite(measured_voltage) & isfinite(model_cell_voltage);
voltage_error = model_cell_voltage(valid) - measured_voltage(valid);

report = struct();
report.source_file = test.source_file;
report.cell_id = test.cell_id;
report.test_temperature_C = test.test_temperature_C;
report.voltage_error = hil.metrics(voltage_error);
report.endpoint_voltage_error_V = endpoint_error( ...
    model_cell_voltage, measured_voltage);

temperature_source = surface_temperature_source(test);
report.surface_temperature_source = temperature_source;
if is_measured_temperature_source(temperature_source)
    measured_surface_temperature = double(test.surface_temperature_C(:));
    model_surface_temperature = interp1(double(result.time_s(:)), ...
        double(result.T_surf(:)), time, 'linear', NaN);
    temperature_valid = isfinite(measured_surface_temperature) & ...
        isfinite(model_surface_temperature);
    report.surface_temperature_error = hil.metrics( ...
        model_surface_temperature(temperature_valid) - ...
        measured_surface_temperature(temperature_valid));
    report.endpoint_surface_temperature_error_C = endpoint_error( ...
        model_surface_temperature, measured_surface_temperature);
    report.thermal_validation = struct( ...
        'status', 'EVALUATED', ...
        'reason', 'Explicit measured surface-temperature channel supplied.');
else
    report.surface_temperature_error = hil.metrics([]);
    report.endpoint_surface_temperature_error_C = NaN;
    report.thermal_validation = struct( ...
        'status', 'NOT_RUN', ...
        'reason', sprintf( ...
            ['Surface-temperature source "%s" is not explicit measured ' ...
             'cell-surface evidence.'], temperature_source));
end

soc_valid = isfinite(test.soc);
if any(soc_valid)
    model_soc = interp1(double(result.time_s(:)), ...
        double(result.SoC_true(:)), time(soc_valid), 'linear', NaN);
    report.soc_error = hil.metrics(model_soc - double(test.soc(soc_valid)));
    report.endpoint_soc_error = endpoint_error(model_soc, test.soc(soc_valid));
else
    report.soc_error = hil.metrics([]);
    report.endpoint_soc_error = NaN;
end
report.pulses = array2table(zeros(0, 8), 'VariableNames', { ...
    'pulse', 'start_time_s', 'end_time_s', 'mean_current_A', ...
    'onset_error_V', 'maximum_loaded_error_V', ...
    'loaded_rms_error_V', 'relaxation_rms_error_V'});
report.pulse_count = 0;
report.loaded_voltage_error = hil.metrics([]);
report.relaxation_voltage_error = hil.metrics([]);
report.test_class = 'dynamic';
report.acceptance_evaluated = ~isempty(acceptance_cfg);
if report.acceptance_evaluated
    report.acceptance = hil.evaluate_validation_acceptance( ...
        report, acceptance_cfg, report.test_class);
    report.passed = report.acceptance.passed;
    if strcmp(report.thermal_validation.status, 'EVALUATED')
        thermal_checks = report.acceptance.checks;
        thermal_passed = thermal_checks.surface_temperature_available && ...
            thermal_checks.surface_temperature_rms_error && ...
            thermal_checks.surface_temperature_maximum_error;
        if thermal_passed
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

function source = surface_temperature_source(test)
if isfield(test, 'surface_temperature_source') && ...
        ~isempty(test.surface_temperature_source)
    source = char(string(test.surface_temperature_source));
else
    source = 'unspecified';
end
end

function result = is_measured_temperature_source(source)
normalized = lower(strtrim(char(source)));
result = startsWith(normalized, 'measured');
end

function value = endpoint_error(model, measurement)
valid = isfinite(model) & isfinite(measurement);
index = find(valid, 1, 'last');
if isempty(index)
    value = NaN;
else
    value = double(model(index) - measurement(index));
end
end
