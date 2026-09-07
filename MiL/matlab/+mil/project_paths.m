function paths = project_paths()
%PROJECT_PATHS Resolve repository-relative DER26 MiL paths.
package_dir = fileparts(mfilename('fullpath'));
matlab_root = fileparts(package_dir);
paths = struct();
paths.matlab_root = matlab_root;
paths.mil_root = fileparts(matlab_root);
paths.repo_root = fileparts(paths.mil_root);
paths.scenarios = fullfile(matlab_root, 'configs', 'scenarios');
paths.scripts = fullfile(matlab_root, 'scripts');
paths.tests = fullfile(matlab_root, 'tests');
paths.docs = fullfile(paths.mil_root, 'docs');
paths.validation = fullfile(paths.mil_root, 'validation');
paths.output_root = fullfile(paths.validation, 'generated');
paths.hil_simulink = fullfile(paths.repo_root, 'HiL', 'simulink');
paths.hil_profiles = fullfile(paths.hil_simulink, 'profiles');
paths.hil_adapters = fullfile(paths.hil_simulink, 'adapters');
paths.ams = fullfile(paths.repo_root, 'AMS');
paths.fuse_replay = fullfile(paths.repo_root, 'Tools', 'fuse_replay');
end
