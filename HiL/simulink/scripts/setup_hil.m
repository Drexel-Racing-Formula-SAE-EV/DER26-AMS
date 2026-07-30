% Add the reusable HIL package to the MATLAB path using this file's location.
hil_scripts_directory = fileparts(mfilename('fullpath'));
hil_simulink_directory = fileparts(hil_scripts_directory);
addpath(hil_simulink_directory, '-begin');
clear hil_scripts_directory hil_simulink_directory
