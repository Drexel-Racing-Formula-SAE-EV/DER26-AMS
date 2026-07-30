function paths = project_paths()
%PROJECT_PATHS Resolve repository-relative HIL paths.
%
% HIL_DATA_ROOT may point at an external raw-data directory.
% HIL_OUTPUT_ROOT may redirect generated validation/build artifacts.

package_dir = fileparts(mfilename('fullpath'));
simulink_root = fileparts(package_dir);

paths = struct();
paths.simulink_root = simulink_root;
paths.hil_root = fileparts(simulink_root);
paths.repo_root = fileparts(paths.hil_root);
paths.models = fullfile(simulink_root, 'models');
paths.configs = fullfile(simulink_root, 'configs');
paths.adapters = fullfile(simulink_root, 'adapters');
paths.profiles = fullfile(simulink_root, 'profiles');
paths.scripts = fullfile(simulink_root, 'scripts');
paths.parameters = fullfile(simulink_root, 'parameters');
paths.parameters_source = fullfile(paths.parameters, 'source');
paths.parameters_generated = fullfile(paths.parameters, 'generated');
paths.parameter_manifests = fullfile(paths.parameters, 'manifests');
paths.validation = fullfile(simulink_root, 'validation');
paths.validation_baselines = fullfile(paths.validation, 'baselines');
paths.generated_code = fullfile(simulink_root, 'generated_code');
paths.esp32_plant = fullfile(paths.hil_root, 'esp32_plant');

data_root = getenv('HIL_DATA_ROOT');
if isempty(data_root)
    data_root = fullfile(simulink_root, 'data');
end
paths.data_root = char(data_root);

output_root = getenv('HIL_OUTPUT_ROOT');
if isempty(output_root)
    output_root = fullfile(paths.validation, 'generated');
end
paths.output_root = char(output_root);
paths.model_build_root = fullfile(paths.output_root, 'model_build');
paths.codegen_root = fullfile(paths.output_root, 'codegen');
end
