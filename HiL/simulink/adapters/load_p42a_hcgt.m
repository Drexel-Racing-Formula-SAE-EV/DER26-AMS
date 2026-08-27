function dataset = load_p42a_hcgt(dataset_cfg, cell_cfg)
%LOAD_P42A_HCGT Adapt published P42A CDT/HCGT data with a cell holdout.
%
% HCGT records are split into individual 1C pulse windows. Their published
% OCV/SoC metadata is retained so validation never silently assumes 100 %
% SoC for a pulse that occurred later in the sequence. Chamber temperature
% is ambient evidence only; this adapter does not claim a measured cell
% surface-temperature channel.

root = resolve_root(dataset_cfg);
source_files = {};
raw_fit_items = {};
raw_holdout_items = {};
raw_ocv_items = {};
r0_rows = {};

[fit_cell_ids, holdout_cell_ids, temperatures, nominal_capacity_Ah, ...
    ocv_cell_id, r0_soc, fit_options] = adapter_contract( ...
    dataset_cfg, cell_cfg);
all_cell_ids = unique([fit_cell_ids(:); holdout_cell_ids(:)], 'stable');

if ~isempty(intersect(fit_cell_ids, holdout_cell_ids))
    error('hil:data:P42APartitionOverlap', ...
        'P42A fit and holdout cell IDs must be disjoint.');
end
if isempty(fit_cell_ids) || isempty(holdout_cell_ids)
    error('hil:data:P42APartitionEmpty', ...
        'P42A fitting and validation each require at least one cell ID.');
end

for temperature = temperatures
    hcgt_file = fullfile(root, 'phase_2', 'HCGT', ...
        sprintf('%ddegC', temperature), 'mat', ...
        sprintf('HCGT_%ddegC.mat', temperature));
    if ~isfile(hcgt_file)
        error('hil:data:P42AHCGTMissing', ...
            'Missing P42A HCGT file: %s', hcgt_file);
    end

    loaded = load(hcgt_file);
    if ~isfield(loaded, 'hcgt_data')
        error('hil:data:P42AHCGTContract', ...
            'File %s does not contain hcgt_data.', hcgt_file);
    end
    source_files{end + 1, 1} = hcgt_file; %#ok<AGROW>

    for cell_index = 1:numel(all_cell_ids)
        cell_id = all_cell_ids{cell_index};
        if ~isfield(loaded.hcgt_data, cell_id)
            continue;
        end
        record = loaded.hcgt_data.(cell_id);
        if ~all(isfield(record, {'time', 'current', 'voltage'}))
            continue;
        end

        windows = extract_hcgt_windows( ...
            record, cell_id, temperature, hcgt_file, ...
            nominal_capacity_Ah, fit_options);
        if any(strcmp(cell_id, fit_cell_ids))
            raw_fit_items = [raw_fit_items; windows(:)]; %#ok<AGROW>
        else
            raw_holdout_items = [raw_holdout_items; windows(:)]; %#ok<AGROW>
        end

        % Resistance records are fitting evidence and must never include
        % cells reserved for the independent holdout.
        if any(strcmp(cell_id, fit_cell_ids))
            if isfield(record, 'r0_1C')
                r0 = double(record.r0_1C(:));
            elseif isfield(record, 'r0_cby2')
                r0 = double(record.r0_cby2(:));
            else
                r0 = [];
            end
            count = min(numel(r0), numel(r0_soc));
            for row_index = 1:count
                if isfinite(r0(row_index)) && r0(row_index) > 0
                    r0_rows(end + 1, :) = { ... %#ok<AGROW>
                        cell_id, temperature, r0_soc(row_index), r0(row_index)};
                end
            end
        end
    end

    cdt_file = fullfile(root, 'phase_2', 'CDT', ...
        sprintf('%ddegC', temperature), 'mat', ...
        sprintf('%s_CDT_%ddegC.mat', ocv_cell_id, temperature));
    if ~isfile(cdt_file)
        error('hil:data:P42ACDTMissing', 'Missing P42A CDT file: %s', cdt_file);
    end
    cdt = load(cdt_file);
    if isfield(cdt, 'cby20')
        curve = cdt.cby20;
    elseif isfield(cdt, 'cby10')
        curve = cdt.cby10;
    else
        error('hil:data:P42ACDTContract', ...
            'No cby20 or cby10 discharge curve in %s.', cdt_file);
    end

    sample_count = numel(curve.voltage_dis);
    discharged_capacity = double(curve.capacity_dis(:));
    raw = struct();
    raw.time_s = (0:(sample_count - 1)).';
    raw.cell_current_A = zeros(sample_count, 1);
    raw.cell_voltage_V = curve.voltage_dis;
    raw.surface_temperature_C = nan(sample_count, 1);
    raw.surface_temperature_source = 'not_available';
    raw.ambient_temperature_C = temperature;
    raw.capacity_Ah = discharged_capacity;
    raw.soc = min(max( ...
        1.0 - discharged_capacity / nominal_capacity_Ah, 0.0), 1.0);
    raw.initial_soc = raw.soc(1);
    raw.initial_ocv_V = double(raw.cell_voltage_V(1));
    raw.pulse_soc = NaN;
    raw.soc_source = 'CDT discharged capacity / nominal capacity';
    raw.cell_id = ocv_cell_id;
    raw.test_temperature_C = temperature;
    raw.test_type = 'CDT_OCV';
    raw.pulse_identifier = zeros(sample_count, 1);
    raw.record_id = sprintf('%s_%dC_CDT_OCV', ocv_cell_id, temperature);
    raw.source_file = cdt_file;
    raw.nominal_capacity_Ah = nominal_capacity_Ah;
    raw.ocvsoc_metadata = [];
    raw.ocv_metadata = [];
    raw_ocv_items{end + 1, 1} = raw; %#ok<AGROW>
    source_files{end + 1, 1} = cdt_file; %#ok<AGROW>
end

if isempty(raw_fit_items)
    error('hil:data:P42ANoFitWindows', ...
        'No valid 1C HCGT pulse windows were found for the fit cells.');
end
if isempty(raw_holdout_items)
    error('hil:data:P42ANoHoldoutWindows', ...
        'No valid 1C HCGT pulse windows were found for the holdout cells.');
end

normalization_options = hil.dataset_normalization_options(dataset_cfg);
normalization_options.pulse_current_threshold_A = ...
    fit_options.rest_current_threshold_A;
[fit_tests, fit_report] = hil.normalize_dataset( ...
    vertcat(raw_fit_items{:}), normalization_options);
[validation_tests, holdout_report] = hil.normalize_dataset( ...
    vertcat(raw_holdout_items{:}), normalization_options);
require_partition_coverage( ...
    fit_tests, fit_cell_ids, temperatures, 'fit');
require_partition_coverage( ...
    validation_tests, holdout_cell_ids, temperatures, 'holdout');

ocv_options = hil.dataset_normalization_options( ...
    dataset_cfg, 'AllowMissingCurrent', true);
ocv_options.current_sign = 'positive_discharge';
[ocv_tests, ocv_report] = hil.normalize_dataset( ...
    vertcat(raw_ocv_items{:}), ocv_options);

if isempty(r0_rows)
    r0_records = table();
else
    r0_records = cell2table(r0_rows, 'VariableNames', ...
        {'cell_id', 'temperature_C', 'soc', 'r0_ohm'});
end

dataset = struct();
dataset.schema_version = 2;
dataset.id = dataset_cfg.id;
dataset.root = root;
dataset.tests = vertcat(fit_tests, validation_tests);
dataset.fit_tests = fit_tests;
dataset.validation_tests = validation_tests;
dataset.ocv_tests = ocv_tests;
dataset.r0_records = r0_records;
dataset.report = struct( ...
    'fit_hcgt', fit_report, ...
    'holdout_hcgt', holdout_report, ...
    'ocv', ocv_report);
dataset.partition_provenance = struct( ...
    'fit_cell_ids', {fit_cell_ids(:).'}, ...
    'holdout_cell_ids', {holdout_cell_ids(:).'}, ...
    'contract', 'disjoint cell-ID partition frozen before validation');
dataset.thermal_evidence = struct( ...
    'status', 'NOT_RUN', ...
    'reason', ['Published HCGT adapter provides chamber ambient only; ' ...
               'no measured cell-surface trace is claimed.']);
dataset.source_files = unique(source_files, 'stable');
end

function require_partition_coverage(tests, cell_ids, temperatures, label)
missing = {};
for cell_index = 1:numel(cell_ids)
    for temperature = temperatures
        found = arrayfun(@(test) ...
            strcmp(test.cell_id, cell_ids{cell_index}) && ...
            abs(double(test.test_temperature_C) - temperature) <= 0.5, ...
            tests);
        if ~any(found)
            missing{end + 1, 1} = sprintf( ... %#ok<AGROW>
                '%s@%gC', cell_ids{cell_index}, temperature);
        end
    end
end
if ~isempty(missing)
    error('hil:data:P42APartitionCoverage', ...
        'P42A %s partition lacks pulse windows for: %s', ...
        label, strjoin(missing, ', '));
end
end

function [fit_cell_ids, holdout_cell_ids, temperatures, ...
        nominal_capacity_Ah, ocv_cell_id, r0_soc, options] = ...
        adapter_contract(dataset_cfg, cell_cfg)
fit_cell_ids = dataset_cfg.fit_cell_ids;
holdout_cell_ids = dataset_cfg.holdout_cell_ids;
temperatures = dataset_cfg.temperatures_C(:).';
nominal_capacity_Ah = 4.2;
ocv_cell_id = 'A2';
options = struct( ...
    'hcgt_soc_regions', dataset_cfg.hcgt_soc_regions, ...
    'current_tolerance_fraction', dataset_cfg.current_tolerance_fraction, ...
    'rest_current_threshold_A', dataset_cfg.rest_current_threshold_A, ...
    'pre_pulse_window_s', dataset_cfg.pre_pulse_window_s, ...
    'post_pulse_window_s', dataset_cfg.post_pulse_window_s);
r0_soc = dataset_cfg.r0_soc_regions;

if ~isempty(cell_cfg) && ~isempty(fieldnames(cell_cfg))
    fit_cell_ids = cell_cfg.parameter_fit_options.cell_ids;
    if isfield(cell_cfg.parameter_fit_options, 'holdout_cell_ids')
        holdout_cell_ids = cell_cfg.parameter_fit_options.holdout_cell_ids;
    end
    temperatures = cell_cfg.temperature_breakpoints_C(:).';
    nominal_capacity_Ah = cell_cfg.nominal_capacity_Ah;
    ocv_cell_id = cell_cfg.parameter_fit_options.ocv_cell_id;
    r0_soc = cell_cfg.parameter_fit_options.r0_soc_regions;
    options.hcgt_soc_regions = ...
        cell_cfg.parameter_fit_options.hcgt_soc_regions;
    options.current_tolerance_fraction = ...
        cell_cfg.parameter_fit_options.current_tolerance_fraction;
    options.rest_current_threshold_A = ...
        cell_cfg.parameter_fit_options.rest_current_threshold_A;
end
end

function items = extract_hcgt_windows(record, cell_id, temperature, ...
        source_file, nominal_capacity_Ah, options)
time = double(record.time(:));
current = double(record.current(:));
voltage = double(record.voltage(:));
n = min([numel(time), numel(current), numel(voltage)]);
time = time(1:n);
current = current(1:n);
voltage = voltage(1:n);
finite = isfinite(time) & isfinite(current) & isfinite(voltage);
time = time(finite);
current = current(finite);
voltage = voltage(finite);

items = {};
if numel(time) < 3
    return;
end
[time, order] = sort(time);
current = current(order);
voltage = voltage(order);

target_current = nominal_capacity_Ah;
tolerance = options.current_tolerance_fraction * target_current;
positive_matches = abs(current - target_current) <= tolerance;
negative_matches = abs(current + target_current) <= tolerance;
if nnz(negative_matches) > nnz(positive_matches)
    discharge_current = -current;
else
    discharge_current = current;
end
pulse_mask = discharge_current > 0 & ...
    abs(discharge_current - target_current) <= tolerance;
[starts, ends] = local_segments(pulse_mask);

valid = false(size(starts));
for index = 1:numel(starts)
    duration = time(ends(index)) - time(starts(index));
    average_current = mean(discharge_current(starts(index):ends(index)), ...
        'omitnan');
    valid(index) = duration >= 1.0 && duration <= 180.0 && ...
        average_current >= 0.35 * target_current && ...
        average_current <= 1.90 * target_current;
end
starts = starts(valid);
ends = ends(valid);

ocvsoc = metadata_vector(record, 'ocvsoc');
ocv = metadata_vector(record, 'ocv');
for pulse_index = 1:numel(starts)
    start_index = starts(pulse_index);
    end_index = ends(pulse_index);
    pre_start = find(time >= ...
        time(start_index) - options.pre_pulse_window_s, 1, 'first');
    previous_active = find(abs(current(1:(start_index - 1))) > ...
        options.rest_current_threshold_A, 1, 'last');
    if ~isempty(previous_active)
        pre_start = max(pre_start, previous_active + 1);
    end

    post_end = find(time <= ...
        time(end_index) + options.post_pulse_window_s, 1, 'last');
    next_active_offset = find(abs(current((end_index + 1):end)) > ...
        options.rest_current_threshold_A, 1, 'first');
    if ~isempty(next_active_offset)
        next_active = end_index + next_active_offset;
        post_end = min(post_end, next_active - 1);
    end
    if isempty(pre_start)
        pre_start = start_index;
    end
    if isempty(post_end)
        post_end = end_index;
    end
    if post_end <= pre_start
        continue;
    end

    selected = pre_start:post_end;
    [pulse_soc, soc_source] = metadata_soc( ...
        ocvsoc, pulse_index, options.hcgt_soc_regions);
    initial_ocv = metadata_at(ocv, pulse_index);
    if ~isfinite(initial_ocv)
        rest_voltage = voltage(pre_start:max(pre_start, start_index - 1));
        initial_ocv = median(rest_voltage, 'omitnan');
    end

    raw = struct();
    raw.time_s = time(selected);
    raw.cell_current_A = current(selected);
    raw.cell_voltage_V = voltage(selected);
    raw.surface_temperature_C = nan(numel(selected), 1);
    raw.surface_temperature_source = 'not_available';
    raw.ambient_temperature_C = temperature;
    raw.capacity_Ah = nan(numel(selected), 1);
    raw.soc = nan(numel(selected), 1);
    raw.initial_soc = pulse_soc;
    raw.initial_ocv_V = initial_ocv;
    raw.pulse_soc = pulse_soc;
    raw.soc_source = soc_source;
    raw.cell_id = cell_id;
    raw.test_temperature_C = temperature;
    raw.test_type = 'HCGT_1C_PULSE_WINDOW';
    raw.pulse_identifier = repmat(pulse_index, numel(selected), 1);
    raw.record_id = sprintf( ...
        '%s_%dC_HCGT_pulse_%02d', cell_id, temperature, pulse_index);
    raw.source_file = source_file;
    raw.ocvsoc_metadata = ocvsoc;
    raw.ocv_metadata = ocv;
    items{end + 1, 1} = raw; %#ok<AGROW>
end
end

function [value, source] = metadata_soc(values, index, fallback)
value = metadata_at(values, index);
source = 'published HCGT ocvsoc metadata';
if ~isfinite(value) && index <= numel(fallback)
    value = double(fallback(index));
    source = 'configured HCGT pulse-region fallback';
end
if isfinite(value) && value > 1.0 && value <= 100.0
    value = value / 100.0;
end
if ~isfinite(value) || value < 0.0 || value > 1.0
    value = NaN;
    source = 'not_available';
end
end

function values = metadata_vector(record, field_name)
if isfield(record, field_name) && isnumeric(record.(field_name))
    raw_values = record.(field_name);
    values = double(raw_values(:));
else
    values = [];
end
end

function value = metadata_at(values, index)
if index <= numel(values)
    value = double(values(index));
else
    value = NaN;
end
end

function [starts, ends] = local_segments(mask)
edges = diff([false; mask(:) ~= 0; false]);
starts = find(edges == 1);
ends = find(edges == -1) - 1;
end

function root = resolve_root(dataset_cfg)
for index = 1:numel(dataset_cfg.root_candidates)
    candidate = fullfile(dataset_cfg.root, dataset_cfg.root_candidates{index});
    marker = fullfile(candidate, 'phase_2');
    if exist(marker, 'dir') == 7
        root = candidate;
        return;
    end
end
error('hil:data:P42ARootMissing', ...
    ['Could not locate Dataset_Molicell_P42A below HIL_DATA_ROOT="%s". ' ...
     'Set HIL_DATA_ROOT to the dataset or its parent.'], dataset_cfg.root);
end
