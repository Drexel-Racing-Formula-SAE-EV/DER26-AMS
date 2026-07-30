function limits = validation_limits_at_temperature( ...
    acceptance_cfg, test_class, temperature_C)
%VALIDATION_LIMITS_AT_TEMPERATURE Resolve one frozen limit set.

[acceptance_cfg, ~] = hil.validate_configuration(acceptance_cfg);
test_class = lower(char(test_class));
if ~ismember(test_class, {'hppc', 'dynamic'})
    error('hil:validation:TestClass', ...
        'Validation test class must be "hppc" or "dynamic".');
end

source = acceptance_cfg.(test_class);
axis = double(source.temperature_breakpoints_C(:));
if ~isfinite(temperature_C)
    temperature_C = median(axis);
end
selected_temperature_C = min(max(double(temperature_C), axis(1)), axis(end));

limits = struct();
names = fieldnames(source);
for index = 1:numel(names)
    name = names{index};
    if strcmp(name, 'temperature_breakpoints_C')
        continue;
    end
    value = source.(name);
    if isnumeric(value) && numel(value) == numel(axis)
        limits.(name) = interp1(axis, double(value(:)), ...
            selected_temperature_C, 'linear');
    else
        limits.(name) = value;
    end
end
limits.test_class = test_class;
limits.requested_temperature_C = double(temperature_C);
limits.selected_temperature_C = selected_temperature_C;
limits.temperature_clamped = ...
    abs(selected_temperature_C - double(temperature_C)) > eps;
end
