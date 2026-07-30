function [params, build_report] = build_parameters(cell_cfg, varargin)
%BUILD_PARAMETERS Load the reviewed snapshot or refit from normalized data.

parser = inputParser();
parser.addParameter('DatasetConfiguration', struct(), @isstruct);
parser.addParameter('ForceRefit', false, @(x) islogical(x) && isscalar(x));
parser.addParameter('Save', true, @(x) islogical(x) && isscalar(x));
parser.addParameter('OutputDirectory', '', @(x) ischar(x) || isstring(x));
parser.parse(varargin{:});
options = parser.Results;

[cell_cfg, ~] = hil.validate_configuration(cell_cfg);
if isfield(cell_cfg, 'is_template') && cell_cfg.is_template
    error('hil:params:TemplateCell', ...
        'Cell configuration "%s" is a template and cannot be built.', cell_cfg.id);
end

snapshot_file = '';
if isfield(cell_cfg, 'parameter_source') && ...
        isfield(cell_cfg.parameter_source, 'legacy_snapshot_file')
    snapshot_file = cell_cfg.parameter_source.legacy_snapshot_file;
end

if ~options.ForceRefit && ~isempty(snapshot_file) && isfile(snapshot_file)
    loaded = load(snapshot_file);
    if isfield(loaded, 'params')
        params = loaded.params;
    else
        params = loaded;
    end
    validation = hil.validate_parameters(cell_cfg, params);
    mat_file = '';
    manifest_file = '';
    if options.Save
        output_directory = char(options.OutputDirectory);
        if isempty(output_directory)
            paths = hil.project_paths();
            output_directory = paths.parameters_generated;
        end
        [mat_file, manifest_file] = hil.save_parameter_package( ...
            params, output_directory);
    end
    build_report = struct( ...
        'mode', 'reviewed_legacy_snapshot', ...
        'snapshot_file', snapshot_file, ...
        'validation', validation, ...
        'parameter_file', mat_file, ...
        'manifest_file', manifest_file);
    return;
end

dataset_cfg = options.DatasetConfiguration;
if isempty(fieldnames(dataset_cfg))
    if isfield(cell_cfg, 'dataset_configuration')
        dataset_cfg = hil.config.dataset(cell_cfg.dataset_configuration);
    else
        error('hil:params:DatasetConfigRequired', ...
            'A dataset configuration is required to refit cell "%s".', cell_cfg.id);
    end
end

dataset = hil.load_dataset(dataset_cfg, cell_cfg);
initial = hil.build_initial_parameters(cell_cfg, dataset);
fit_tests = dataset.tests;
if isfield(dataset, 'fit_tests') && ~isempty(dataset.fit_tests)
    fit_tests = dataset.fit_tests;
end
fast = hil.fit_r1c1(fit_tests, cell_cfg);
slow = hil.fit_r2c2(fit_tests, fast, cell_cfg);

values = initial;
values.R1 = fast.R1;
values.C1 = fast.C1;
values.R2 = slow.R2;
values.C2 = slow.C2;
values.fit_quality.fast_branch = fast.report;
values.fit_quality.slow_branch = slow.report;

manifest = hil.source_manifest(cell_cfg, dataset_cfg, dataset);
params = hil.build_parameter_package(cell_cfg, values, manifest);
validation = hil.validate_parameters(cell_cfg, params);

mat_file = '';
manifest_file = '';
if options.Save
    output_directory = char(options.OutputDirectory);
    if isempty(output_directory)
        paths = hil.project_paths();
        output_directory = paths.parameters_generated;
    end
    [mat_file, manifest_file] = hil.save_parameter_package(params, output_directory);
end

build_report = struct( ...
    'mode', 'refit_from_normalized_dataset', ...
    'dataset_report', dataset.report, ...
    'validation', validation, ...
    'parameter_file', mat_file, ...
    'manifest_file', manifest_file);
end
