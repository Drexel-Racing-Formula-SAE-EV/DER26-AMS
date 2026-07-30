function manifest = source_manifest(cell_cfg, dataset_cfg, dataset)
%SOURCE_MANIFEST Build auditable source metadata for a parameter artifact.

if nargin < 3
    dataset = struct();
end
if isfield(dataset, 'source_files')
    source_files = dataset.source_files;
else
    source_files = {};
end
if ischar(source_files) || isstring(source_files)
    source_files = cellstr(source_files);
end

files = repmat(struct('path', '', 'sha256', '', 'bytes', 0), 0, 1);
for index = 1:numel(source_files)
    file_path = char(source_files{index});
    if ~isfile(file_path)
        continue;
    end
    info = dir(file_path);
    files(end + 1, 1) = struct( ... %#ok<AGROW>
        'path', portable_path(file_path, dataset_cfg.root), ...
        'sha256', hil.file_sha256(file_path), ...
        'bytes', info.bytes);
end

manifest = struct();
manifest.schema_version = 1;
manifest.cell_manufacturer = cell_cfg.manufacturer;
manifest.cell_model = cell_cfg.model;
manifest.dataset_source = dataset_cfg.source;
manifest.dataset_version = dataset_cfg.version;
manifest.current_sign_convention = 'positive current = discharge';
manifest.cell_ids = cell_cfg.parameter_fit_options.cell_ids;
manifest.temperatures_C = cell_cfg.temperature_breakpoints_C(:).';
manifest.soc_coverage = [ ...
    min(cell_cfg.ocv_soc_breakpoints), max(cell_cfg.ocv_soc_breakpoints)];
manifest.test_types = test_types(dataset);
manifest.case_counts = struct( ...
    'fit', item_count(dataset, 'fit_tests'), ...
    'holdout', item_count(dataset, 'validation_tests'), ...
    'ocv', item_count(dataset, 'ocv_tests'), ...
    'r0', table_height(dataset, 'r0_records'));
if isfield(dataset, 'report')
    manifest.normalization_report = dataset.report;
else
    manifest.normalization_report = struct();
end
if isfield(dataset, 'partition_provenance')
    manifest.partition_provenance = dataset.partition_provenance;
end
manifest.source_files = files;
paths = hil.project_paths();
implementation_paths = { ...
    fullfile(paths.simulink_root, '+hil', 'normalize_dataset.m'), ...
    fullfile(paths.simulink_root, '+hil', 'build_ocv_lut.m'), ...
    fullfile(paths.simulink_root, '+hil', 'fit_r0.m'), ...
    fullfile(paths.simulink_root, '+hil', 'fit_r1c1.m'), ...
    fullfile(paths.simulink_root, '+hil', 'fit_r2c2.m'), ...
    fullfile(paths.simulink_root, '+hil', 'build_parameter_package.m')};
implementation = repmat(struct('path', '', 'sha256', ''), ...
    numel(implementation_paths), 1);
for index = 1:numel(implementation_paths)
    implementation(index).path = portable_path( ...
        implementation_paths{index}, paths.repo_root);
    implementation(index).sha256 = ...
        hil.file_sha256(implementation_paths{index});
end
manifest.fitting_implementation = implementation;
manifest.fitting_implementation_hash = ...
    hil.configuration_hash({implementation.sha256});
manifest.cell_configuration_hash = hil.configuration_hash(cell_cfg);
manifest.dataset_configuration_hash = hil.configuration_hash(dataset_cfg);
manifest.matlab_version = version;
manifest.simulink_version = toolbox_version('Simulink');
manifest.repository_commit = repository_commit(paths.repo_root);
manifest.known_limitations = cell_cfg.provenance.known_limitations;
end

function values = test_types(dataset)
if ~isfield(dataset, 'tests') || isempty(dataset.tests)
    values = {};
    return;
end
values = unique({dataset.tests.test_type}, 'stable');
values = values(~cellfun(@isempty, values));
end

function count = item_count(dataset, name)
if isfield(dataset, name)
    count = numel(dataset.(name));
else
    count = 0;
end
end

function count = table_height(dataset, name)
if isfield(dataset, name) && istable(dataset.(name))
    count = height(dataset.(name));
else
    count = 0;
end
end

function value = portable_path(file_path, root)
file_path = char(file_path);
root = char(root);
prefix = [root, filesep];
if startsWith(file_path, prefix)
    value = file_path((numel(prefix) + 1):end);
else
    [~, name, extension] = fileparts(file_path);
    value = [name, extension];
end
value = strrep(value, filesep, '/');
end

function value = toolbox_version(name)
items = ver(name);
if isempty(items)
    value = 'not installed';
else
    value = items(1).Version;
end
end

function commit = repository_commit(repo_root)
command = sprintf('git -C "%s" rev-parse HEAD', ...
    strrep(char(repo_root), '"', '\"'));
[status, output] = system(command);
if status == 0
    commit = strtrim(output);
else
    commit = 'unknown';
end
end
