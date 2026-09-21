function paths = setup_mil()
%SETUP_MIL Add the DER26 MiL and reusable battery-plant packages to MATLAB.
script_dir = fileparts(mfilename('fullpath'));
matlab_root = fileparts(script_dir);
mil_root = fileparts(matlab_root);
repo_root = fileparts(mil_root);

addpath(matlab_root);
addpath(fullfile(matlab_root, 'configs', 'scenarios'));
addpath(fullfile(repo_root, 'HiL', 'simulink'));
addpath(fullfile(repo_root, 'HiL', 'simulink', 'profiles'));
addpath(fullfile(repo_root, 'HiL', 'simulink', 'adapters'));

paths = mil.project_paths();
fprintf('DER26 MiL ready: %s\n', paths.mil_root);
end
