% Compatibility entry point for normalized P42A pulse validation.
setup_hil
run(fullfile(fileparts(mfilename('fullpath')), 'batch_validate_p42a_hcgt.m'))
