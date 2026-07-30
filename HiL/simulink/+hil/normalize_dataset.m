function [tests, report] = normalize_dataset(raw, options)
%NORMALIZE_DATASET Convert external records to the internal cell-test contract.
%
% Positive current is always discharge after normalization.

if nargin < 2
    options = struct();
end
options = apply_defaults(options);

if isstruct(raw) && isscalar(raw) && isfield(raw, 'tests')
    raw = raw.tests;
end
if istable(raw)
    raw = table2struct(raw, 'ToScalar', true);
end
if iscell(raw)
    [tests, report] = normalize_cell_records(raw, options);
    return;
end
if ~isstruct(raw)
    error('hil:data:InvalidRawType', ...
        'Raw dataset must be a table, struct array, cell array, or struct with tests.');
end

empty_test = struct( ...
    'time_s', [], ...
    'cell_current_A', [], ...
    'cell_voltage_V', [], ...
    'surface_temperature_C', [], ...
    'surface_temperature_source', '', ...
    'ambient_temperature_C', [], ...
    'capacity_Ah', [], ...
    'soc', [], ...
    'initial_soc', NaN, ...
    'initial_ocv_V', NaN, ...
    'pulse_soc', NaN, ...
    'soc_source', '', ...
    'ocvsoc_metadata', [], ...
    'ocv_metadata', [], ...
    'cell_id', '', ...
    'test_temperature_C', NaN, ...
    'test_type', '', ...
    'pulse_identifier', [], ...
    'record_id', '', ...
    'source_file', '');
tests = repmat(empty_test, 0, 1);

report = struct();
report.schema_version = 1;
report.input_record_count = numel(raw);
report.accepted_record_count = 0;
report.rejected_records = {};
record_reports = cell(numel(raw), 1);

for record_index = 1:numel(raw)
    [test, item_report, accepted] = normalize_one(raw(record_index), options);
    record_reports{record_index} = item_report;
    if accepted
        tests(end + 1, 1) = test; %#ok<AGROW>
    else
        report.rejected_records{end + 1, 1} = item_report.rejection_reason; %#ok<AGROW>
    end
end

if isempty(record_reports)
    report.records = repmat(empty_report(), 0, 1);
else
    report.records = vertcat(record_reports{:});
end
report.accepted_record_count = numel(tests);
report.rejected_record_count = numel(report.rejected_records);
end

function options = apply_defaults(options)
defaults = struct( ...
    'current_sign', 'auto', ...
    'time_scale_to_s', 1.0, ...
    'current_scale_to_A', 1.0, ...
    'voltage_scale_to_V', 1.0, ...
    'temperature_scale_to_C', 1.0, ...
    'capacity_scale_to_Ah', 1.0, ...
    'target_sample_time_s', NaN, ...
    'allow_missing_current', false, ...
    'pulse_current_threshold_A', 0.20, ...
    'surface_temperature_source', '', ...
    'source_units', struct());

names = fieldnames(defaults);
for index = 1:numel(names)
    if ~isfield(options, names{index})
        options.(names{index}) = defaults.(names{index});
    end
end
end

function [test, item_report, accepted] = normalize_one(raw, options)
test = struct( ...
    'time_s', [], ...
    'cell_current_A', [], ...
    'cell_voltage_V', [], ...
    'surface_temperature_C', [], ...
    'surface_temperature_source', '', ...
    'ambient_temperature_C', [], ...
    'capacity_Ah', [], ...
    'soc', [], ...
    'initial_soc', NaN, ...
    'initial_ocv_V', NaN, ...
    'pulse_soc', NaN, ...
    'soc_source', '', ...
    'ocvsoc_metadata', [], ...
    'ocv_metadata', [], ...
    'cell_id', '', ...
    'test_temperature_C', NaN, ...
    'test_type', '', ...
    'pulse_identifier', [], ...
    'record_id', '', ...
    'source_file', '');

item_report = empty_report();
item_report.source_file = char_or_default(get_any(raw, {'source_file'}), '');
item_report.missing_fields = {};

time = vector_or_empty(get_any(raw, {'time_s', 'time', 'timestamp_s', 't'}));
current = vector_or_empty(get_any(raw, ...
    {'cell_current_A', 'current_A', 'current', 'I', 'i'}));
voltage = vector_or_empty(get_any(raw, ...
    {'cell_voltage_V', 'voltage_V', 'voltage', 'V', 'v'}));

if isempty(time)
    item_report.missing_fields{end + 1} = 'time_s';
end
if isempty(voltage)
    item_report.missing_fields{end + 1} = 'cell_voltage_V';
end
if isempty(current) && ~options.allow_missing_current
    item_report.missing_fields{end + 1} = 'cell_current_A';
end

if ~isempty(item_report.missing_fields)
    item_report.rejection_reason = sprintf( ...
        'Missing required fields: %s', strjoin(item_report.missing_fields, ', '));
    accepted = false;
    return;
end

if isempty(current)
    current = zeros(size(time));
end

n = min([numel(time), numel(current), numel(voltage)]);
if n < 2
    item_report.rejection_reason = 'Fewer than two aligned samples.';
    accepted = false;
    return;
end

time = double(time(1:n)) * options.time_scale_to_s;
current = double(current(1:n)) * options.current_scale_to_A;
voltage = double(voltage(1:n)) * options.voltage_scale_to_V;

surface = align_optional(get_any(raw, ...
    {'surface_temperature_C', 'cell_temperature_C', 'temperature_C', ...
     'temperature', 'temp'}), n, NaN) * options.temperature_scale_to_C;
ambient = align_optional(get_any(raw, ...
    {'ambient_temperature_C', 'ambient_C', 'T_amb'}), n, NaN) ...
    * options.temperature_scale_to_C;
capacity = align_optional(get_any(raw, ...
    {'capacity_Ah', 'capacity', 'discharged_capacity_Ah'}), n, NaN) ...
    * options.capacity_scale_to_Ah;
soc = align_optional(get_any(raw, {'soc', 'SoC', 'state_of_charge'}), n, NaN);
pulse_identifier = align_optional(get_any(raw, ...
    {'pulse_identifier', 'pulse_id'}), n, 0);

good = isfinite(time) & isfinite(voltage) & isfinite(current);
item_report.nan_rows_removed = n - nnz(good);
time = time(good);
current = current(good);
voltage = voltage(good);
surface = surface(good);
ambient = ambient(good);
capacity = capacity(good);
soc = soc(good);
pulse_identifier = pulse_identifier(good);

if numel(time) < 2
    item_report.rejection_reason = 'Too few finite aligned samples after cleanup.';
    accepted = false;
    return;
end

[time, order] = sort(time);
current = current(order);
voltage = voltage(order);
surface = surface(order);
ambient = ambient(order);
capacity = capacity(order);
soc = soc(order);
pulse_identifier = pulse_identifier(order);

[time, unique_index] = unique(time, 'stable');
item_report.duplicate_timestamps_removed = numel(order) - numel(unique_index);
current = current(unique_index);
voltage = voltage(unique_index);
surface = surface(unique_index);
ambient = ambient(unique_index);
capacity = capacity(unique_index);
soc = soc(unique_index);
pulse_identifier = pulse_identifier(unique_index);

item_report.time_origin_shift_s = time(1);
time = time - time(1);

[current, detected_sign] = normalize_current_sign(current, options.current_sign);
item_report.current_sign_detected = detected_sign;

if isfinite(options.target_sample_time_s) && options.target_sample_time_s > 0
    new_time = (time(1):options.target_sample_time_s:time(end)).';
    if numel(new_time) >= 2 && ...
            (numel(new_time) ~= numel(time) || ...
             max(abs(diff(time) - options.target_sample_time_s)) > 1e-9)
        current = interp1(time, current, new_time, 'previous', 'extrap');
        voltage = interp1(time, voltage, new_time, 'linear', 'extrap');
        surface = interp_optional(time, surface, new_time);
        ambient = interp_optional(time, ambient, new_time);
        capacity = interp_optional(time, capacity, new_time);
        soc = interp_optional(time, soc, new_time);
        pulse_identifier = interp1(time, pulse_identifier, ...
            new_time, 'nearest', 'extrap');
        time = new_time;
        item_report.resampled = true;
    end
end

test.time_s = time(:);
test.cell_current_A = current(:);
test.cell_voltage_V = voltage(:);
test.surface_temperature_C = surface(:);
test.surface_temperature_source = temperature_source(raw, surface, options);
test.ambient_temperature_C = ambient(:);
test.capacity_Ah = capacity(:);
test.soc = soc(:);
test.initial_soc = bounded_soc_or_nan(get_any(raw, ...
    {'initial_soc', 'initial_SoC', 'starting_soc'}));
test.initial_ocv_V = scalar_or_default(get_any(raw, ...
    {'initial_ocv_V', 'initial_ocv', 'starting_ocv_V'}), NaN);
test.pulse_soc = bounded_soc_or_nan(get_any(raw, ...
    {'pulse_soc', 'pulse_SoC'}));
test.soc_source = char_or_default(get_any(raw, ...
    {'soc_source', 'initial_soc_source'}), '');
test.ocvsoc_metadata = numeric_metadata(get_any(raw, ...
    {'ocvsoc_metadata', 'ocvsoc'}));
test.ocv_metadata = numeric_metadata(get_any(raw, ...
    {'ocv_metadata', 'ocv'}));
test.cell_id = char_or_default(get_any(raw, {'cell_id', 'cell'}), 'unknown');
test.test_temperature_C = scalar_or_default(get_any(raw, ...
    {'test_temperature_C', 'temperature_test_C', 'test_temp_C'}), ...
    temperature_fallback(surface, ambient));
test.test_type = char_or_default(get_any(raw, {'test_type', 'type'}), 'unknown');
test.pulse_identifier = pulse_identifier(:);
test.record_id = char_or_default(get_any(raw, ...
    {'record_id', 'test_id', 'window_id'}), '');
test.source_file = item_report.source_file;

item_report.temperature_available = any(isfinite(surface));
item_report.surface_temperature_source = test.surface_temperature_source;
item_report.initial_soc_available = isfinite(test.initial_soc);
item_report.initial_ocv_available = isfinite(test.initial_ocv_V);
item_report.pulse_count = count_segments(abs(current) > ...
    options.pulse_current_threshold_A);
finite_soc = soc(isfinite(soc));
if ~isempty(finite_soc)
    item_report.soc_coverage = [min(finite_soc), max(finite_soc)];
end
item_report.units_assumed = struct( ...
    'time', 's', 'current', 'A', 'voltage', 'V', ...
    'temperature', 'degC', 'capacity', 'Ah');
item_report.source_units = options.source_units;
item_report.accepted = true;
accepted = true;
end

function value = get_any(raw, aliases)
value = [];
names = fieldnames(raw);
for alias_index = 1:numel(aliases)
    match = find(strcmpi(names, aliases{alias_index}), 1, 'first');
    if ~isempty(match)
        value = raw.(names{match});
        return;
    end
end
end

function value = vector_or_empty(value)
if isempty(value) || ~(isnumeric(value) || islogical(value))
    value = [];
else
    value = value(:);
end
end

function value = align_optional(value, n, fill_value)
if isempty(value) || ~(isnumeric(value) || islogical(value))
    value = repmat(fill_value, n, 1);
elseif isscalar(value)
    value = repmat(double(value), n, 1);
else
    value = double(value(:));
    if numel(value) < n
        value(end + 1:n, 1) = fill_value;
    else
        value = value(1:n);
    end
end
end

function value = interp_optional(time, value, new_time)
finite = isfinite(value);
if nnz(finite) < 2
    value = repmat(median(value, 'omitnan'), size(new_time));
else
    value = interp1(time(finite), value(finite), new_time, 'linear', 'extrap');
end
end

function [current, detected] = normalize_current_sign(current, requested)
requested = lower(char(requested));
active = abs(current) > max(0.20, 0.05 * max(abs(current)));

switch requested
    case 'positive_discharge'
        detected = 'positive_discharge';
    case 'negative_discharge'
        current = -current;
        detected = 'negative_discharge';
    case 'auto'
        negative_count = nnz(current(active) < 0);
        positive_count = nnz(current(active) > 0);
        if negative_count > positive_count
            current = -current;
            detected = 'negative_discharge';
        else
            detected = 'positive_discharge';
        end
    otherwise
        error('hil:data:CurrentSign', 'Unsupported current sign "%s".', requested);
end
end

function count = count_segments(mask)
mask = mask(:) ~= 0;
edges = diff([false; mask; false]);
count = nnz(edges == 1);
end

function value = char_or_default(value, default)
if isempty(value)
    value = default;
elseif iscell(value)
    value = value{1};
end
value = char(string(value));
end

function value = scalar_or_default(value, default)
if isempty(value) || ~isnumeric(value)
    value = default;
else
    value = double(value(1));
end
if isempty(value) || ~isfinite(value)
    value = NaN;
end
end

function value = bounded_soc_or_nan(value)
value = scalar_or_default(value, NaN);
if isfinite(value) && value > 1.0 && value <= 100.0
    value = value / 100.0;
end
if ~isfinite(value) || value < 0.0 || value > 1.0
    value = NaN;
end
end

function value = numeric_metadata(value)
if isempty(value) || ~isnumeric(value)
    value = [];
else
    value = double(value(:));
end
end

function source = temperature_source(raw, surface, options)
source = char_or_default(get_any(raw, ...
    {'surface_temperature_source', 'temperature_source'}), '');
if ~isempty(source)
    return;
end
if isfield(options, 'surface_temperature_source') && ...
        ~isempty(options.surface_temperature_source)
    source = char(string(options.surface_temperature_source));
    return;
end
if any(isfinite(surface))
    source = 'unspecified';
else
    source = 'not_available';
end
end

function value = temperature_fallback(surface, ambient)
value = median(surface, 'omitnan');
if ~isfinite(value)
    value = median(ambient, 'omitnan');
end
end

function report = empty_report()
report = struct( ...
    'source_file', '', ...
    'missing_fields', {{}}, ...
    'nan_rows_removed', 0, ...
    'duplicate_timestamps_removed', 0, ...
    'time_origin_shift_s', NaN, ...
    'resampled', false, ...
    'current_sign_detected', '', ...
    'units_assumed', struct(), ...
    'source_units', struct(), ...
    'temperature_available', false, ...
    'surface_temperature_source', '', ...
    'initial_soc_available', false, ...
    'initial_ocv_available', false, ...
    'pulse_count', 0, ...
    'soc_coverage', [NaN, NaN], ...
    'accepted', false, ...
    'rejection_reason', '');
end

function [tests, report] = normalize_cell_records(raw, options)
test_items = cell(numel(raw), 1);
report_items = cell(numel(raw), 1);
for index = 1:numel(raw)
    [test_items{index}, report_items{index}] = ...
        hil.normalize_dataset(raw{index}, options);
end
nonempty_tests = test_items(~cellfun(@isempty, test_items));
if isempty(nonempty_tests)
    tests = struct([]);
else
    tests = vertcat(nonempty_tests{:});
end
report = struct();
report.schema_version = 1;
report.input_record_count = sum(cellfun( ...
    @(item) item.input_record_count, report_items));
report.accepted_record_count = numel(tests);
rejected_cells = cellfun( ...
    @(item) item.rejected_records, report_items, ...
    'UniformOutput', false);
if isempty(rejected_cells)
    report.rejected_records = {};
else
    report.rejected_records = vertcat(rejected_cells{:});
end
record_cells = cellfun(@(item) item.records, report_items, ...
    'UniformOutput', false);
record_cells = record_cells(~cellfun(@isempty, record_cells));
if isempty(record_cells)
    report.records = repmat(empty_report(), 0, 1);
else
    report.records = vertcat(record_cells{:});
end
report.rejected_record_count = numel(report.rejected_records);
end
