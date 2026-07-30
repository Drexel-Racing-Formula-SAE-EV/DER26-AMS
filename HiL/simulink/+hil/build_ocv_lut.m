function [ocv, report] = build_ocv_lut(ocv_tests, cell_cfg)
%BUILD_OCV_LUT Build OCV(SoC, temperature) from normalized capacity curves.

[cell_cfg, ~] = hil.validate_configuration(cell_cfg);
soc_axis = double(cell_cfg.ocv_soc_breakpoints(:));
temperature_axis = double(cell_cfg.temperature_breakpoints_C(:));
voltage_bounds = double(cell_cfg.parameter_fit_options.ocv_voltage_bounds_V);

ocv = nan(numel(soc_axis), numel(temperature_axis));
case_counts = zeros(size(temperature_axis));
used_sources = cell(size(temperature_axis));

for temperature_index = 1:numel(temperature_axis)
    temperature = temperature_axis(temperature_index);
    curves = [];
    sources = {};

    for test_index = 1:numel(ocv_tests)
        test = ocv_tests(test_index);
        if abs(double(test.test_temperature_C) - temperature) > 0.5
            continue;
        end

        capacity = double(test.capacity_Ah(:));
        voltage = double(test.cell_voltage_V(:));
        valid = isfinite(capacity) & isfinite(voltage);
        capacity = capacity(valid);
        voltage = voltage(valid);
        if numel(capacity) < 10
            continue;
        end

        capacity = capacity - min(capacity);
        full_capacity = max(capacity);
        if ~(isfinite(full_capacity) && full_capacity > 0)
            continue;
        end

        soc = 1.0 - capacity ./ full_capacity;
        [soc, order] = sort(soc);
        voltage = voltage(order);
        [soc, unique_index] = unique(soc, 'stable');
        voltage = voltage(unique_index);
        if numel(soc) < 4
            continue;
        end

        curve = interp1(soc, voltage, soc_axis, 'linear', 'extrap');
        curve = min(max(curve, voltage_bounds(1)), voltage_bounds(2));
        curves(:, end + 1) = curve(:); %#ok<AGROW>
        sources{end + 1, 1} = test.source_file; %#ok<AGROW>
    end

    if isempty(curves)
        error('hil:fit:MissingOCVTemperature', ...
            'No usable OCV curve was found at %.1f degC.', temperature);
    end
    ocv(:, temperature_index) = median(curves, 2, 'omitnan');
    case_counts(temperature_index) = size(curves, 2);
    used_sources{temperature_index} = unique(sources, 'stable');
end

ocv = single(ocv);
report = struct( ...
    'method', 'capacity-normalized CDT median', ...
    'soc_axis', soc_axis, ...
    'temperature_axis_C', temperature_axis, ...
    'case_counts', case_counts, ...
    'source_files', {used_sources}, ...
    'voltage_bounds_V', voltage_bounds);
end
