function artifact = build_candidate_cell( ...
    cell_configuration_name, dataset_configuration_name, ...
    pack_configuration_name, acceptance_configuration_name)
%BUILD_CANDIDATE_CELL Refit a completed candidate config and materialize a model.

arguments
    cell_configuration_name (1, :) char
    dataset_configuration_name (1, :) char
    pack_configuration_name (1, :) char = 'der26_75s6p'
    acceptance_configuration_name (1, :) char = ''
end

script_directory = fileparts(mfilename('fullpath'));
addpath(fileparts(script_directory), '-begin');
cell_cfg = hil.config.cell(cell_configuration_name);
if isfield(cell_cfg, 'is_template') && cell_cfg.is_template
    error('hil:candidate:Template', ...
        'Copy candidate_template.m and replace every placeholder first.');
end
dataset_cfg = hil.config.dataset(dataset_configuration_name);
pack_cfg = hil.config.pack(pack_configuration_name);
if isempty(acceptance_configuration_name)
    acceptance_configuration_name = cell_configuration_name;
end
acceptance_cfg = hil.config.acceptance(acceptance_configuration_name);
if isfield(acceptance_cfg, 'is_template') && acceptance_cfg.is_template
    error('hil:candidate:AcceptanceTemplate', ...
        'Copy, rename, justify, and freeze candidate acceptance limits first.');
end
sim_cfg = hil.config.simulation('hppc_validation');
dataset = hil.load_dataset(dataset_cfg, cell_cfg);
require_independent_holdout(dataset);
[params, build_report] = hil.build_parameters( ...
    cell_cfg, 'DatasetConfiguration', dataset_cfg, ...
    'ForceRefit', true, 'Save', true);
holdout_validation = hil.validate_dataset( ...
    cell_cfg, pack_cfg, dataset_cfg, params, acceptance_cfg);
if ~holdout_validation.passed
    error('hil:candidate:HoldoutAccuracy', ...
        'Independent candidate holdout failed one or more numerical gates.');
end
model = hil.configure_model(cell_cfg, pack_cfg, sim_cfg, params);
artifact = struct('parameters', params, ...
    'parameter_build_report', build_report, ...
    'holdout_validation', holdout_validation, ...
    'model', model);
end

function require_independent_holdout(dataset)
if ~isfield(dataset, 'validation_tests') || isempty(dataset.validation_tests)
    error('hil:candidate:HoldoutRequired', ...
        'Candidate fitting requires a nonempty independent holdout partition.');
end
if ~isfield(dataset, 'fit_tests') || isempty(dataset.fit_tests)
    error('hil:candidate:FitPartitionRequired', ...
        'Candidate fitting requires a nonempty fit partition.');
end
if ~isfield(dataset, 'partition_provenance')
    error('hil:candidate:PartitionProvenance', ...
        'The dataset adapter must describe fit/holdout partition provenance.');
end
fit_sources = dataset.partition_provenance.fit_sources;
holdout_sources = dataset.partition_provenance.holdout_sources;
overlap = intersect(fit_sources, holdout_sources);
if ~isempty(overlap)
    error('hil:candidate:HoldoutOverlap', ...
        'Fit and holdout partitions share the same source identity: %s', ...
        strjoin(overlap, ', '));
end
end
