function report = validate_dataset( ...
    cell_cfg, pack_cfg, dataset_cfg, params, acceptance_cfg)
%VALIDATE_DATASET Execute normalized cell tests through the reference plant.

if nargin < 5 || isempty(acceptance_cfg)
    acceptance_name = cell_cfg.id;
    if isfield(cell_cfg, 'config_name')
        acceptance_name = cell_cfg.config_name;
    end
    acceptance_cfg = hil.config.acceptance(acceptance_name);
end
[acceptance_cfg, ~] = hil.validate_configuration(acceptance_cfg);
if isfield(acceptance_cfg, 'is_template') && acceptance_cfg.is_template
    error('hil:validation:AcceptanceTemplate', ...
        'Copy, rename, justify, and freeze candidate acceptance limits first.');
end
if ~strcmpi(acceptance_cfg.cell_configuration, cell_cfg.id)
    error('hil:validation:AcceptanceCellMismatch', ...
        'Acceptance configuration "%s" targets cell "%s", not "%s".', ...
        acceptance_cfg.id, acceptance_cfg.cell_configuration, cell_cfg.id);
end

dataset = hil.load_dataset(dataset_cfg, cell_cfg);
validation_tests = dataset.tests;
if isfield(dataset, 'validation_tests') && ...
        ~isempty(dataset.validation_tests)
    validation_tests = dataset.validation_tests;
end
validation_pack = pack_cfg;
validation_pack.imbalance_model.mode = 'uniform';
item_reports = cell(numel(validation_tests), 1);
accepted_count = 0;
voltage_errors = [];

for test_index = 1:numel(validation_tests)
    test = validation_tests(test_index);
    if numel(test.time_s) < 2
        continue;
    end

    if abs(double(test.time_s(1))) > 1e-9
        error('hil:validation:DatasetTimeOrigin', ...
            'Normalized record "%s" does not start at t=0.', test.record_id);
    end
    sample_time = median(diff(double(test.time_s)));
    if ~isfinite(sample_time) || sample_time <= 0
        continue;
    end
    time = double(test.time_s(:));
    uniform_time = (0:sample_time:time(end)).';
    cell_current = interp1(time, double(test.cell_current_A(:)), ...
        uniform_time, 'previous', 'extrap');
    ambient = interp1(time, double(test.ambient_temperature_C(:)), ...
        uniform_time, 'linear', NaN);
    if ~any(isfinite(ambient))
        ambient = repmat(double(test.test_temperature_C), size(uniform_time));
    else
        ambient(~isfinite(ambient)) = test.test_temperature_C;
    end

    simulation = hil.config.simulation('hppc_validation');
    simulation.id = sprintf('dataset_%04d', test_index);
    simulation.sample_time_s = sample_time;
    simulation.stop_time_s = uniform_time(end);
    simulation.ambient_temperature_C = ambient(1);
    [simulation.initial_soc, initial_soc_source] = ...
        initial_soc(test, params);
    simulation.engine = 'reference';
    profile = struct( ...
        'time_s', uniform_time, ...
        'pack_current_A', cell_current * double(pack_cfg.parallel_cells), ...
        'ambient_temperature_C', ambient, ...
        'name', simulation.id, ...
        'source', test.source_file, ...
        'sample_time_s', sample_time, ...
        'repeat_policy', 'one_shot', ...
        'scaling_mode', 'constant_cell_current');
    result = hil.run_reference( ...
        cell_cfg, validation_pack, simulation, params, profile);
    test_class = classify_test(test);
    if strcmp(test_class, 'hppc')
        item = hil.validate_hppc( ...
            test, result, validation_pack, acceptance_cfg);
    else
        item = hil.validate_dynamic_profile( ...
            test, result, validation_pack, acceptance_cfg);
    end
    item.initial_soc = simulation.initial_soc;
    item.initial_soc_source = initial_soc_source;
    accepted_count = accepted_count + 1;
    item_reports{accepted_count} = item;

    predicted = interp1(result.time_s, ...
        result.V_pack / validation_pack.series_groups, ...
        time, 'linear', NaN);
    error_values = predicted - double(test.cell_voltage_V(:));
    voltage_errors = [voltage_errors; error_values(isfinite(error_values))]; %#ok<AGROW>
end

if accepted_count == 0
    items = repmat(struct(), 0, 1);
else
    items = vertcat(item_reports{1:accepted_count});
end
report = struct();
report.passed = ~isempty(items) && all([items.passed]);
if report.passed
    report.status = 'PASS';
else
    report.status = 'FAIL';
end
report.test_count = numel(items);
report.aggregate_voltage_error = hil.metrics(voltage_errors);
report.items = items;
report.acceptance_configuration = acceptance_cfg;
report.dataset_report = dataset.report;
report.source_files = dataset.source_files;
if isfield(dataset, 'partition_provenance')
    report.partition_provenance = dataset.partition_provenance;
else
    report.partition_provenance = struct();
end
report.thermal_validation = aggregate_thermal_status(items);
end

function test_class = classify_test(test)
name = lower(char(test.test_type));
dynamic_tokens = {'udds', 'us06', 'la92', 'dynamic', 'drive', ...
    'wltp', 'ftp'};
hppc_tokens = {'hppc', 'hcgt', 'pulse', 'dst'};
if any(cellfun(@(token) contains(name, token), dynamic_tokens))
    test_class = 'dynamic';
    return;
end
if any(cellfun(@(token) contains(name, token), hppc_tokens))
    test_class = 'hppc';
    return;
end

current = double(test.cell_current_A(:));
threshold = max(0.20, 0.10 * max(abs(current)));
active = abs(current) > threshold;
transition_count = nnz(diff([false; active; false]) ~= 0);
if transition_count > 0 && transition_count <= 40
    test_class = 'hppc';
else
    test_class = 'dynamic';
end
end

function [value, source] = initial_soc(test, params)
if isfield(test, 'initial_soc') && isfinite(test.initial_soc)
    value = min(max(double(test.initial_soc), 0), 1);
    source = char_or_default(test, 'soc_source', 'explicit initial_soc');
    return;
end
if isfield(test, 'pulse_soc') && isfinite(test.pulse_soc)
    value = min(max(double(test.pulse_soc), 0), 1);
    source = char_or_default(test, 'soc_source', 'explicit pulse_soc');
    return;
end
finite_soc = test.soc(isfinite(test.soc));
if ~isempty(finite_soc)
    value = min(max(double(finite_soc(1)), 0), 1);
    source = 'normalized SoC trace';
    return;
end
capacity = test.capacity_Ah(isfinite(test.capacity_Ah));
if ~isempty(capacity) && max(capacity) > 0
    value = min(max(1.0 - double(capacity(1)) / max(double(capacity)), 0), 1);
    source = 'capacity trace';
    return;
end
if isfield(test, 'initial_ocv_V') && isfinite(test.initial_ocv_V)
    value = soc_from_ocv(double(test.initial_ocv_V), ...
        double(test.test_temperature_C), params);
    if isfinite(value)
        source = 'initial OCV inversion';
        return;
    end
end
value = 1.0;
source = 'fallback 100% SoC';
end

function value = soc_from_ocv(ocv, temperature_C, params)
value = NaN;
if ~all(isfield(params, {'OCV', 'soc_ocv_common', 'temp_bp_ocv'}))
    return;
end
temperature_axis = double(params.temp_bp_ocv(:));
ocv_table = double(params.OCV);
if size(ocv_table, 2) ~= numel(temperature_axis)
    return;
end
curve = interp1(temperature_axis, ocv_table.', temperature_C, ...
    'linear', 'extrap').';
soc_axis = double(params.soc_ocv_common(:));
[curve, unique_index] = unique(curve, 'stable');
soc_axis = soc_axis(unique_index);
if numel(curve) < 2
    return;
end
[curve, order] = sort(curve);
soc_axis = soc_axis(order);
value = interp1(curve, soc_axis, ocv, 'linear', NaN);
if isfinite(value)
    value = min(max(value, 0), 1);
end
end

function value = char_or_default(item, field_name, default)
if isfield(item, field_name) && ~isempty(item.(field_name))
    value = char(string(item.(field_name)));
else
    value = default;
end
end

function summary = aggregate_thermal_status(items)
summary = struct( ...
    'status', 'NOT_RUN', ...
    'evaluated_test_count', 0, ...
    'not_run_test_count', numel(items), ...
    'reason', 'No explicit measured surface-temperature channel was supplied.');
if isempty(items)
    return;
end
statuses = arrayfun(@(item) char(item.thermal_validation.status), ...
    items, 'UniformOutput', false);
summary.evaluated_test_count = nnz(~strcmp(statuses, 'NOT_RUN'));
summary.not_run_test_count = nnz(strcmp(statuses, 'NOT_RUN'));
if any(strcmp(statuses, 'FAIL'))
    summary.status = 'FAIL';
    summary.reason = 'At least one measured thermal holdout failed.';
elseif any(strcmp(statuses, 'PASS'))
    summary.status = 'PASS';
    summary.reason = 'All evaluated measured thermal holdouts passed.';
end
end
