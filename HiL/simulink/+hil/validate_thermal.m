function report = validate_thermal(result, pack_cfg)
%VALIDATE_THERMAL Check numerical behavior and label screening limitations.

temperature_floor = min(result.T_ambient) - 1e-6;
temperature_ceiling = max(result.T_ambient) + 200;
checks = struct();
checks.core_finite = all(isfinite(result.T_core));
checks.surface_finite = all(isfinite(result.T_surf));
checks.sensor_finite = all(isfinite(result.T_sensor(:)));
checks.no_unforced_cooling_below_ambient = ...
    min(result.T_core) >= temperature_floor && ...
    min(result.T_surf) >= temperature_floor;
checks.screening_range = max(result.T_sensor(:)) <= temperature_ceiling;
checks.summary_consistent = ...
    max(abs(result.T_max - max(result.T_sensor, [], 2))) <= 2e-4 && ...
    max(abs(result.T_avg - mean(result.T_sensor, 2))) <= 2e-4;

names = fieldnames(checks);
failed = names(~cellfun(@(name) checks.(name), names));
report = struct();
report.passed = isempty(failed);
report.checks = checks;
report.failed_checks = failed;
report.initial_core_temperature_C = result.T_core(1);
report.final_core_temperature_C = result.T_core(end);
report.peak_core_temperature_C = max(result.T_core);
report.peak_surface_temperature_C = max(result.T_surf);
report.peak_sensor_temperature_C = max(result.T_sensor(:));
report.maximum_core_surface_delta_C = max(result.T_core - result.T_surf);
report.cooling_boundary = pack_cfg.cooling_boundary;
report.interpretation = ...
    ['Comparative screening only. Pack geometry, module conduction, busbar ' ...
     'heating, enclosure convection, and fan airflow are not calibrated.'];
if ~report.passed
    error('hil:validation:Thermal', ...
        'Thermal validation failed: %s', strjoin(failed, ', '));
end
end
