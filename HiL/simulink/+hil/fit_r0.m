function [r0, report] = fit_r0(r0_records, cell_cfg)
%FIT_R0 Fit R0(SoC, temperature) from normalized resistance records.

[cell_cfg, ~] = hil.validate_configuration(cell_cfg);
if ~istable(r0_records) || isempty(r0_records)
    error('hil:fit:MissingR0', 'No R0 records were supplied.');
end

required = {'temperature_C', 'soc', 'r0_ohm'};
if ~all(ismember(required, r0_records.Properties.VariableNames))
    error('hil:fit:R0Contract', ...
        'R0 table must contain temperature_C, soc, and r0_ohm.');
end

soc_axis = double(cell_cfg.soc_breakpoints(:));
temperature_axis = double(cell_cfg.temperature_breakpoints_C(:));
bounds = double(cell_cfg.parameter_fit_options.r0_bounds_ohm);
r0 = nan(numel(soc_axis), numel(temperature_axis));
case_counts = zeros(size(temperature_axis));

for temperature_index = 1:numel(temperature_axis)
    temperature = temperature_axis(temperature_index);
    mask = abs(double(r0_records.temperature_C) - temperature) <= 0.5 & ...
        isfinite(r0_records.soc) & isfinite(r0_records.r0_ohm) & ...
        r0_records.r0_ohm > 0;
    subset = r0_records(mask, :);
    if height(subset) < 3
        error('hil:fit:InsufficientR0', ...
            'Fewer than three valid R0 records at %.1f degC.', temperature);
    end

    rounded_soc = round(double(subset.soc), 6);
    [groups, soc_unique] = findgroups(rounded_soc);
    median_r0 = splitapply(@(x) median(x, 'omitnan'), ...
        double(subset.r0_ohm), groups);
    [soc_unique, order] = sort(soc_unique);
    median_r0 = median_r0(order);

    fitted = interp1(soc_unique, median_r0, soc_axis, 'linear', 'extrap');
    fitted = min(max(fitted, bounds(1)), bounds(2));
    r0(:, temperature_index) = fitted;
    case_counts(temperature_index) = height(subset);
end

r0 = single(r0);
report = struct( ...
    'method', 'median by SoC then linear interpolation', ...
    'soc_axis', soc_axis, ...
    'temperature_axis_C', temperature_axis, ...
    'case_counts', case_counts, ...
    'bounds_ohm', bounds);
end
