function report = validate_pack_outputs(result, pack_cfg, varargin)
%VALIDATE_PACK_OUTPUTS Check pack image dimensions and conservation identities.

parser = inputParser();
parser.addParameter('AbsoluteTolerance', 2e-4, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter('ThrowOnFailure', true, ...
    @(x) islogical(x) && isscalar(x));
parser.parse(varargin{:});
options = parser.Results;
tolerance = double(options.AbsoluteTolerance);

[pack_cfg, ~] = hil.validate_configuration(pack_cfg);
Ns = double(pack_cfg.series_groups);
Nsegments = double(pack_cfg.segment_count);
Ntemp = double(pack_cfg.temperature_sensor_count);
sample_count = numel(result.time_s);

required = { ...
    'V_pack', 'V_group', 'V_segment', 'T_sensor', 'SoC_group', ...
    'V_min', 'V_max', 'T_max', 'T_avg'};
missing = required(~isfield_many(result, required));
if ~isempty(missing)
    error('hil:validation:MissingOutputs', ...
        'Result is missing outputs: %s', strjoin(missing, ', '));
end

checks = struct();
checks.sample_count_positive = sample_count >= 2;
checks.time_is_finite_and_increasing = ...
    all(isfinite(result.time_s)) && all(diff(result.time_s) > 0);
checks.group_dimensions = isequal(size(result.V_group), [sample_count, Ns]);
checks.segment_dimensions = ...
    isequal(size(result.V_segment), [sample_count, Nsegments]);
checks.temperature_dimensions = ...
    isequal(size(result.T_sensor), [sample_count, Ntemp]);
checks.soc_dimensions = isequal(size(result.SoC_group), [sample_count, Ns]);
checks.outputs_finite = all_finite(result, required);
checks.soc_bounded = all(result.SoC_group(:) >= -tolerance) && ...
    all(result.SoC_group(:) <= 1 + tolerance);

pack_error = double(result.V_pack(:)) - sum(double(result.V_group), 2);
checks.pack_voltage_conserved = max(abs(pack_error)) <= tolerance;

segment_expected = zeros(sample_count, Nsegments);
for segment = 1:Nsegments
    selected = pack_cfg.group_to_segment == segment;
    segment_expected(:, segment) = sum(double(result.V_group(:, selected)), 2);
end
segment_error = double(result.V_segment) - segment_expected;
checks.segment_voltage_conserved = ...
    max(abs(segment_error(:))) <= tolerance;

checks.minimum_matches = max(abs(double(result.V_min(:)) - ...
    min(double(result.V_group), [], 2))) <= tolerance;
checks.maximum_matches = max(abs(double(result.V_max(:)) - ...
    max(double(result.V_group), [], 2))) <= tolerance;
checks.temperature_maximum_matches = ...
    max(abs(double(result.T_max(:)) - ...
    max(double(result.T_sensor), [], 2))) <= tolerance;
checks.temperature_average_matches = ...
    max(abs(double(result.T_avg(:)) - ...
    mean(double(result.T_sensor), 2))) <= tolerance;

names = fieldnames(checks);
failed = names(~cellfun(@(name) checks.(name), names));
report = struct();
report.passed = isempty(failed);
report.checks = checks;
report.failed_checks = failed;
report.maximum_pack_conservation_error_V = max(abs(pack_error));
report.maximum_segment_conservation_error_V = max(abs(segment_error(:)));
report.absolute_tolerance = tolerance;
report.dimensions = struct( ...
    'samples', sample_count, 'groups', Ns, ...
    'segments', Nsegments, 'temperature_sensors', Ntemp);

if ~report.passed && options.ThrowOnFailure
    error('hil:validation:PackOutputs', ...
        'Pack-output validation failed: %s', strjoin(failed, ', '));
end
end

function present = isfield_many(value, names)
present = false(size(names));
for index = 1:numel(names)
    present(index) = isfield(value, names{index});
end
end

function valid = all_finite(result, names)
valid = true;
for index = 1:numel(names)
    valid = valid && all(isfinite(double(result.(names{index})(:))));
end
end
