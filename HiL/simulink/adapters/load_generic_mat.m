function dataset = load_generic_mat(dataset_cfg, ~)
%LOAD_GENERIC_MAT Load normalized or raw tests from a MAT file.

if isfield(dataset_cfg, 'file')
    file_path = dataset_cfg.file;
else
    error('hil:data:MATFileMissing', 'Dataset configuration requires field "file".');
end
if ~isfile(file_path)
    file_path = fullfile(dataset_cfg.root, file_path);
end

loaded = load(file_path);
if isfield(loaded, 'tests')
    raw_tests = loaded.tests;
elseif isfield(loaded, 'fit_tests')
    raw_tests = loaded.fit_tests;
else
    names = fieldnames(loaded);
    if numel(names) ~= 1
        error('hil:data:MATContract', ...
            ['MAT file must contain "tests", "fit_tests", or exactly one ' ...
            'raw dataset variable.']);
    end
    raw_tests = loaded.(names{1});
end

options = hil.dataset_normalization_options(dataset_cfg);
if isfield(loaded, 'fit_tests')
    [fit_tests, fit_report] = hil.normalize_dataset(loaded.fit_tests, options);
else
    [fit_tests, fit_report] = hil.normalize_dataset(raw_tests, options);
end
if isfield(loaded, 'validation_tests')
    [validation_tests, holdout_report] = hil.normalize_dataset( ...
        loaded.validation_tests, options);
else
    validation_tests = struct([]);
    holdout_report = struct('input_record_count', 0, 'accepted_record_count', 0);
end
test_items = {fit_tests, validation_tests};
test_items = test_items(~cellfun(@isempty, test_items));
if isempty(test_items)
    tests = struct([]);
else
    tests = vertcat(test_items{:});
end
if isfield(loaded, 'ocv_tests')
    [ocv_tests, ocv_report] = hil.normalize_dataset( ...
        loaded.ocv_tests, hil.dataset_normalization_options( ...
            dataset_cfg, 'AllowMissingCurrent', true));
else
    ocv_tests = struct([]);
    ocv_report = struct('input_record_count', 0, 'accepted_record_count', 0);
end
if isfield(loaded, 'r0_records')
    r0_records = loaded.r0_records;
    if isstruct(r0_records)
        r0_records = struct2table(r0_records);
    end
    if ~istable(r0_records)
        error('hil:data:MATR0Contract', ...
            'r0_records must be a table or struct array.');
    end
else
    r0_records = table();
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
        'ocv', ocv_report), ...
    'partition_provenance', struct( ...
        'fit_sources', {{sprintf('%s::fit_tests', file_path)}}, ...
        'holdout_sources', {{sprintf( ...
            '%s::validation_tests', file_path)}}, ...
        'contract', 'separate MAT variables'), ...
    'source_files', {{file_path}});
end
