function [mat_file, manifest_file] = save_parameter_package(params, output_dir)
%SAVE_PARAMETER_PACKAGE Save flattened model variables plus the full package.

if nargin < 2 || isempty(output_dir)
    paths = hil.project_paths();
    output_dir = paths.parameters_generated;
end
hil.ensure_directory(output_dir);

short_hash = params.configuration_hash(1:min(12, numel(params.configuration_hash)));
base_name = sprintf('params_%s_%s', params.cell_id, short_hash);
mat_file = fullfile(output_dir, [base_name '.mat']);
manifest_file = fullfile(output_dir, [base_name '_manifest.json']);

save_data = params;
save_data.params = params;
save(mat_file, '-struct', 'save_data', '-v7');

manifest = struct( ...
    'schema_version', params.schema_version, ...
    'cell_id', params.cell_id, ...
    'configuration_hash', params.configuration_hash, ...
    'generation_timestamp_utc', params.generation_timestamp_utc, ...
    'validity_domain', params.validity_domain, ...
    'fit_quality', params.fit_quality, ...
    'source_manifest', params.source_manifest, ...
    'parameter_file', [base_name, '.mat']);
hil.write_json(manifest_file, manifest);
end
