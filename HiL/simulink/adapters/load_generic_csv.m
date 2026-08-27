function dataset = load_generic_csv(dataset_cfg, ~)
%LOAD_GENERIC_CSV Load pulse/dynamic, OCV, and R0 CSV contracts.

if isfield(dataset_cfg, 'fit_files')
    fit_files = configured_files(dataset_cfg, 'fit_files', '', '');
else
    fit_files = configured_files(dataset_cfg, 'files', 'file', '*.csv');
end
holdout_files = configured_files( ...
    dataset_cfg, 'holdout_files', 'holdout_file', '');
ocv_files = configured_files(dataset_cfg, 'ocv_files', 'ocv_file', '');
if isempty(fit_files) && isempty(holdout_files) && isempty(ocv_files)
    error('hil:data:NoCSV', 'No CSV files were configured or found.');
end

[fit_tests, fit_report, fit_sources] = load_test_files( ...
    fit_files, dataset_cfg, false);
[validation_tests, holdout_report, holdout_sources] = load_test_files( ...
    holdout_files, dataset_cfg, false);
[tests, ~] = concatenate_tests(fit_tests, validation_tests);
[ocv_tests, ocv_report, ocv_sources] = load_test_files( ...
    ocv_files, dataset_cfg, true);

r0_records = table();
r0_sources = {};
if isfield(dataset_cfg, 'r0_file') && ~isempty(dataset_cfg.r0_file)
    r0_file = resolve_file(dataset_cfg.root, dataset_cfg.r0_file);
    r0_records = load_r0_table(r0_file);
    r0_sources = {r0_file};
end

dataset = struct( ...
    'schema_version', 1, ...
    'id', dataset_cfg.id, ...
    'tests', tests, ...
    'fit_tests', fit_tests, ...
    'validation_tests', validation_tests, ...
    'ocv_tests', ocv_tests, ...
    'r0_records', r0_records, ...
    'report', struct( ...
        'fit_tests', fit_report, ...
        'holdout_tests', holdout_report, ...
        'ocv', ocv_report, ...
        'r0_record_count', height(r0_records)), ...
    'partition_provenance', struct( ...
        'fit_sources', {fit_sources}, ...
        'holdout_sources', {holdout_sources}, ...
        'contract', 'separate configured CSV files'), ...
    'source_files', {unique([ ...
        fit_sources; holdout_sources; ocv_sources; r0_sources(:)], 'stable')});
end

function files = configured_files(cfg, plural_name, singular_name, wildcard)
if isfield(cfg, plural_name)
    files = cfg.(plural_name);
elseif ~isempty(singular_name) && isfield(cfg, singular_name)
    files = {cfg.(singular_name)};
elseif ~isempty(wildcard)
    listing = dir(fullfile(cfg.root, wildcard));
    files = fullfile({listing.folder}, {listing.name});
else
    files = {};
end
if ischar(files) || isstring(files)
    files = cellstr(files);
end
files = files(:);
end

function [tests, count] = concatenate_tests(varargin)
items = varargin(~cellfun(@isempty, varargin));
if isempty(items)
    tests = struct([]);
else
    tests = vertcat(items{:});
end
count = numel(tests);
end

function [tests, report, sources] = load_test_files(files, cfg, is_ocv)
test_cells = cell(numel(files), 1);
report_cells = cell(numel(files), 1);
sources = cell(numel(files), 1);
for index = 1:numel(files)
    file_path = resolve_file(cfg.root, files{index});
    table_data = readtable(file_path, 'VariableNamingRule', 'preserve');
    item = table2struct(table_data, 'ToScalar', true);
    item.source_file = file_path;
    if is_ocv
        if ~has_alias(item, {'time_s', 'time', 'timestamp_s', 't'})
            item.time_s = (0:(height(table_data) - 1)).';
        end
        item.test_type = 'OCV';
    elseif isfield(cfg, 'test_type')
        item.test_type = cfg.test_type;
    end
    options = hil.dataset_normalization_options( ...
        cfg, 'AllowMissingCurrent', is_ocv);
    [test_cells{index}, report_cells{index}] = ...
        hil.normalize_dataset(item, options);
    sources{index} = file_path;
end

nonempty = test_cells(~cellfun(@isempty, test_cells));
if isempty(nonempty)
    tests = struct([]);
else
    tests = vertcat(nonempty{:});
end
report = struct( ...
    'schema_version', 1, ...
    'input_file_count', numel(files), ...
    'accepted_record_count', numel(tests), ...
    'files', {report_cells});
end

function found = has_alias(value, aliases)
names = fieldnames(value);
found = false;
for index = 1:numel(aliases)
    if any(strcmpi(names, aliases{index}))
        found = true;
        return;
    end
end
end

function table_data = load_r0_table(file_path)
table_data = readtable(file_path, 'VariableNamingRule', 'preserve');
table_data = rename_alias(table_data, ...
    {'temperature_C', 'temperature', 'temp_C', 'test_temperature_C'}, ...
    'temperature_C');
table_data = rename_alias(table_data, ...
    {'soc', 'SoC', 'state_of_charge'}, 'soc');
table_data = rename_alias(table_data, ...
    {'r0_ohm', 'R0_ohm', 'R0', 'dc_resistance_ohm'}, 'r0_ohm');
if ~all(ismember({'temperature_C', 'soc', 'r0_ohm'}, ...
        table_data.Properties.VariableNames))
    error('hil:data:R0CSVContract', ...
        'R0 CSV must contain temperature_C, soc, and r0_ohm columns.');
end
valid = isfinite(table_data.temperature_C) & ...
    isfinite(table_data.soc) & isfinite(table_data.r0_ohm) & ...
    table_data.r0_ohm > 0;
table_data = table_data(valid, :);
end

function table_data = rename_alias(table_data, aliases, target)
names = table_data.Properties.VariableNames;
if ismember(target, names)
    return;
end
for index = 1:numel(aliases)
    match = find(strcmpi(names, aliases{index}), 1, 'first');
    if ~isempty(match)
        table_data.Properties.VariableNames{match} = target;
        return;
    end
end
end

function file_path = resolve_file(root, file_path)
file_path = char(file_path);
if ~isfile(file_path)
    file_path = fullfile(root, file_path);
end
if ~isfile(file_path)
    error('hil:data:CSVFileMissing', 'Missing configured CSV file: %s', file_path);
end
end
