function files = export_report(result, output_directory)
%EXPORT_REPORT Export one simulation result as MAT, CSV, JSON, and plots.

if nargin < 2 || isempty(output_directory)
    if isfield(result, 'simulation_configuration') && ...
            isfield(result.simulation_configuration, 'output_directory')
        output_directory = result.simulation_configuration.output_directory;
    else
        paths = hil.project_paths();
        output_directory = fullfile(paths.output_root, 'simulation_report');
    end
end
hil.ensure_directory(output_directory);

profile_name = 'simulation';
if isfield(result, 'profile') && isfield(result.profile, 'name')
    profile_name = char(result.profile.name);
end
base_name = matlab.lang.makeValidName(profile_name);
mat_file = fullfile(output_directory, [base_name, '_result.mat']);
csv_file = fullfile(output_directory, [base_name, '_timeseries.csv']);
json_file = fullfile(output_directory, [base_name, '_summary.json']);
figure_file = fullfile(output_directory, [base_name, '_overview.png']);

save(mat_file, 'result', '-v7');
table_data = table( ...
    result.time_s(:), result.I_pack(:), result.V_pack(:), ...
    result.SoC_true(:), result.T_core(:), result.T_surf(:), ...
    result.V_min(:), result.V_max(:), result.T_max(:), result.T_avg(:), ...
    'VariableNames', { ...
        'time_s', 'I_pack_A', 'V_pack_V', 'SoC_true', ...
        'T_core_C', 'T_surf_C', 'V_min_V', 'V_max_V', ...
        'T_max_C', 'T_avg_C'});
writetable(table_data, csv_file);

summary = struct();
summary.schema_version = 1;
summary.engine = result.engine;
summary.profile = profile_name;
summary.configuration_hash = result.configuration_hash;
summary.parameter_hash = result.parameter_hash;
summary.sample_count = numel(result.time_s);
summary.start_time_s = result.time_s(1);
summary.end_time_s = result.time_s(end);
summary.final_soc = result.SoC_true(end);
summary.minimum_pack_voltage_V = min(result.V_pack);
summary.maximum_pack_voltage_V = max(result.V_pack);
summary.minimum_group_voltage_V = min(result.V_min);
summary.maximum_group_voltage_V = max(result.V_max);
summary.peak_core_temperature_C = max(result.T_core);
summary.peak_surface_temperature_C = max(result.T_surf);
summary.peak_sensor_temperature_C = max(result.T_max);
summary.files = struct('mat', mat_file, 'csv', csv_file);
hil.write_json(json_file, summary);

figure_written = false;
try
    figure_handle = figure('Visible', 'off', 'Color', 'white');
    cleanup = onCleanup(@() close(figure_handle)); %#ok<NASGU>
    layout = tiledlayout(figure_handle, 3, 1, ...
        'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile(layout);
    plot(result.time_s, result.I_pack, 'LineWidth', 1.0);
    ylabel('Current (A)');
    grid on;
    nexttile(layout);
    plot(result.time_s, result.V_pack, 'LineWidth', 1.0);
    hold on;
    plot(result.time_s, result.V_min * ...
        result.pack_configuration.series_groups, '--');
    ylabel('Voltage (V)');
    grid on;
    nexttile(layout);
    plot(result.time_s, result.T_core, 'LineWidth', 1.0);
    hold on;
    plot(result.time_s, result.T_surf, 'LineWidth', 1.0);
    plot(result.time_s, result.T_max, '--');
    ylabel('Temperature (C)');
    xlabel('Time (s)');
    legend('Core', 'Surface', 'Hottest sensor', 'Location', 'best');
    grid on;
    title(layout, sprintf('%s — %s', profile_name, result.engine), ...
        'Interpreter', 'none');
    exportgraphics(figure_handle, figure_file, 'Resolution', 150);
    figure_written = true;
catch exception
    warning('hil:report:FigureSkipped', ...
        'Result data was exported, but the overview figure was skipped: %s', ...
        exception.message);
end
if ~figure_written
    figure_file = '';
end

files = struct( ...
    'mat', mat_file, 'csv', csv_file, ...
    'json', json_file, 'figure', figure_file);
end
