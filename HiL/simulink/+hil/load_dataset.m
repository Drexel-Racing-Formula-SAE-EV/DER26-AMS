function dataset = load_dataset(dataset_cfg, cell_cfg)
%LOAD_DATASET Run a configured adapter and return normalized data.

if nargin < 2
    cell_cfg = struct();
end
[dataset_cfg, ~] = hil.validate_configuration(dataset_cfg);
if isfield(dataset_cfg, 'is_template') && dataset_cfg.is_template
    error('hil:data:TemplateDataset', ...
        'Dataset configuration "%s" is a non-buildable template.', ...
        dataset_cfg.id);
end

paths = hil.project_paths();
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
addpath(paths.adapters, '-begin');

adapter_name = char(dataset_cfg.adapter);
adapter_file = fullfile(paths.adapters, [adapter_name '.m']);
if exist(adapter_file, 'file') ~= 2
    error('hil:data:AdapterNotFound', ...
        'Dataset adapter "%s" was not found at %s.', adapter_name, adapter_file);
end

adapter = str2func(adapter_name);
dataset = adapter(dataset_cfg, cell_cfg);
if ~isstruct(dataset) || ~isfield(dataset, 'tests')
    error('hil:data:AdapterContract', ...
        'Adapter "%s" did not return a struct containing tests.', adapter_name);
end
end
