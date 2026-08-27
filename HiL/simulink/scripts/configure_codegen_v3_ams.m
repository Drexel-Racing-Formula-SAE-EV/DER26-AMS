% Compatibility entry point for the stable ESP32 code-generation workflow.
setup_hil
run(fullfile(fileparts(mfilename('fullpath')), 'generate_esp32_plant.m'))
