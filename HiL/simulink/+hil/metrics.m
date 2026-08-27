function result = metrics(error_values)
%METRICS Standard error metrics used by pulse and dynamic validation.

error_values = double(error_values(:));
error_values = error_values(isfinite(error_values));
if isempty(error_values)
    result = struct( ...
        'count', 0, 'rms', NaN, 'mean_bias', NaN, ...
        'bias_corrected_rms', NaN, 'maximum_absolute', NaN, ...
        'p95_absolute', NaN, 'p99_absolute', NaN);
    return;
end

absolute = sort(abs(error_values));
result = struct();
result.count = numel(error_values);
result.rms = sqrt(mean(error_values .^ 2));
result.mean_bias = mean(error_values);
result.bias_corrected_rms = sqrt(mean( ...
    (error_values - result.mean_bias) .^ 2));
result.maximum_absolute = max(absolute);
result.p95_absolute = local_percentile(absolute, 0.95);
result.p99_absolute = local_percentile(absolute, 0.99);
end

function value = local_percentile(sorted_values, fraction)
position = 1 + (numel(sorted_values) - 1) * fraction;
lower_index = floor(position);
upper_index = ceil(position);
weight = position - lower_index;
value = sorted_values(lower_index) * (1 - weight) + ...
    sorted_values(upper_index) * weight;
end
