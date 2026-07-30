function cfg = der26_75s6p()
%DER26_75S6P Current DER accumulator topology and HIL sensor image.

Ns = 75;
Np = 6;
segment_counts = [15, 15, 15, 15, 15];
Nsegments = numel(segment_counts);
Ntemp = 120;

group_to_segment = repelem(1:Nsegments, segment_counts);
temp_sensor_to_group = zeros(1, Ntemp);
for segment = 1:Nsegments
    sensor_indices = (segment - 1) * 24 + (1:24);
    local_group = floor((0:23) * segment_counts(segment) / 24) + 1;
    group_offset = sum(segment_counts(1:(segment - 1)));
    temp_sensor_to_group(sensor_indices) = group_offset + local_group;
end

location_type = repmat({'cell_surface'}, 1, Ntemp);
location_type(6:6:end) = {'interstitial'};
core_weight = zeros(1, Ntemp);
core_weight(6:6:end) = 0.65;

cfg = struct();
cfg.schema_version = 1;
cfg.kind = 'pack';
cfg.id = 'der26_75s6p';
cfg.is_template = false;
cfg.series_groups = Ns;
cfg.parallel_cells = Np;
cfg.segment_count = Nsegments;
cfg.groups_per_segment = segment_counts;
cfg.group_to_segment = group_to_segment;

cfg.temperature_sensor_count = Ntemp;
cfg.temp_sensor_to_group = temp_sensor_to_group;
cfg.temp_sensor_location_type = location_type;
cfg.temp_sensor_core_weight = core_weight;

cfg.initial_soc = 1.0;
cfg.initial_temperature_C = 25.0;
cfg.minimum_group_voltage_V = 2.5;
cfg.maximum_group_voltage_V = 4.2;

imbalance = struct();
imbalance.mode = 'deterministic_spread';
imbalance.seed = 26075;
imbalance.voltage_peak_to_peak_V = 0.008;
imbalance.soc_peak_to_peak = 0.010;
imbalance.temperature_peak_to_peak_C = 2.0;
imbalance.capacity_sigma_fraction = 0.015;
imbalance.capacity_bounds_fraction = [0.95, 1.05];
imbalance.r0_sigma_fraction = 0.035;
imbalance.r0_bounds_fraction = [0.90, 1.12];
imbalance.r1_sigma_fraction = 0.035;
imbalance.r1_bounds_fraction = [0.90, 1.12];
imbalance.c1_sigma_fraction = 0.020;
imbalance.c1_bounds_fraction = [0.94, 1.06];
imbalance.r2_sigma_fraction = 0.035;
imbalance.r2_bounds_fraction = [0.90, 1.12];
imbalance.c2_sigma_fraction = 0.020;
imbalance.c2_bounds_fraction = [0.94, 1.06];
imbalance.initial_soc_sigma = 0.0025;
imbalance.initial_soc_bounds = [-0.005, 0.005];
imbalance.thermal_resistance_sigma_fraction = 0.08;
imbalance.thermal_resistance_bounds_fraction = [0.80, 1.25];
cfg.imbalance_model = imbalance;

cfg.thermal_layout = struct( ...
    'model', 'explicit_sensor_to_group_mapping', ...
    'cell_to_cell_coupling_K_per_W', NaN, ...
    'module_conduction_K_per_W', NaN, ...
    'busbar_heat_model', 'not_calibrated', ...
    'geometry_status', 'screening assumptions only');

cfg.cooling_boundary = struct( ...
    'mode', 'natural_convection_nominal', ...
    'Rsa_multiplier', 1.0, ...
    'ambient_temperature_C', 25.0, ...
    'assumption', 'cell prior; enclosure and airflow not calibrated');
end
