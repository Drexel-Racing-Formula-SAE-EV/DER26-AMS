clear; clc; close all;

%% Model / params
mdl = 'drev_75s6p_p42a_accumulator_plant_v3_ams_outputs';

param_file = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'mat_params', ...
    'params_p42a_75s6p_final_batch_validated.mat');

load(param_file);

load_system(mdl);

%% Re-apply critical settings
set_param([mdl '/I_pack_to_cell'], 'Gain', '1/6');
set_param([mdl '/V_cell_to_pack'], 'Gain', '75');
set_param([mdl '/dSoC_per_step'], 'Gain', '-1/(3600*Q_nom)');
set_param([mdl '/SoC_Integrator'], 'SampleTime', '0.1');
set_param([mdl '/ T_clamp'], 'LowerLimit', '5', 'UpperLimit', '40');

set_param(mdl, 'SimulationCommand', 'update');

%% Simple test input
Ts = 0.1;
t = (0:Ts:120)';

I_pack = zeros(size(t), 'single');
I_pack(t >= 20 & t < 80) = single(100.0);   % 100 A discharge pulse

T_amb = single(25.0 * ones(size(t)));

ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries(I_pack, t), 'I_pack');
ds = ds.addElement(timeseries(T_amb, t), 'T_amb');

simIn = Simulink.SimulationInput(mdl);
simIn = simIn.setExternalInput(ds);
simIn = simIn.setVariable('SoC_init', single(1.0));
simIn = simIn.setVariable('T_init', single(25.0));

simIn = simIn.setModelParameter( ...
    'StopTime', num2str(t(end)), ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset');

out = sim(simIn);
Y = out.get('yout');

%% Extract outputs
% Expected order:
% 1 V_pack
% 2 T_core
% 3 T_surf
% 4 SoC_true
% 5 V_group
% 6 V_segment
% 7 T_sensor
% 8 SoC_group
% 9 V_min
% 10 V_max
% 11 T_max
% 12 T_avg

V_pack_ts   = Y.getElement(1).Values;
T_core_ts   = Y.getElement(2).Values;
T_surf_ts   = Y.getElement(3).Values;
SoC_ts      = Y.getElement(4).Values;
V_group_ts  = Y.getElement(5).Values;
V_seg_ts    = Y.getElement(6).Values;
T_sensor_ts = Y.getElement(7).Values;
SoC_grp_ts  = Y.getElement(8).Values;
V_min_ts    = Y.getElement(9).Values;
V_max_ts    = Y.getElement(10).Values;
T_max_ts    = Y.getElement(11).Values;
T_avg_ts    = Y.getElement(12).Values;

V_pack   = local_last_scalar(V_pack_ts);
T_core   = local_last_scalar(T_core_ts);
T_surf   = local_last_scalar(T_surf_ts);
SoC_true = local_last_scalar(SoC_ts);

V_group  = local_last_vector(V_group_ts);
V_segment = local_last_vector(V_seg_ts);
T_sensor = local_last_vector(T_sensor_ts);
SoC_group = local_last_vector(SoC_grp_ts);

V_min = local_last_scalar(V_min_ts);
V_max = local_last_scalar(V_max_ts);
T_max = local_last_scalar(T_max_ts);
T_avg = local_last_scalar(T_avg_ts);

%% Checks
sum_V_group = sum(V_group);
sum_V_seg   = sum(V_segment);

err_group_sum = sum_V_group - V_pack;
err_seg_sum   = sum_V_seg - V_pack;

fprintf('\n======= AMS output sanity test =======\n');

fprintf('V_pack final:             %.5f V\n', V_pack);
fprintf('sum(V_group):             %.5f V\n', sum_V_group);
fprintf('sum(V_segment):           %.5f V\n', sum_V_seg);
fprintf('sum(V_group)-V_pack:      %.8f V\n', err_group_sum);
fprintf('sum(V_segment)-V_pack:    %.8f V\n', err_seg_sum);

fprintf('\nVector lengths:\n');
fprintf('length(V_group):          %d\n', numel(V_group));
fprintf('length(V_segment):        %d\n', numel(V_segment));
fprintf('length(T_sensor):         %d\n', numel(T_sensor));
fprintf('length(SoC_group):        %d\n', numel(SoC_group));

fprintf('\nVoltage stats:\n');
fprintf('V_min output:             %.5f V\n', V_min);
fprintf('min(V_group):             %.5f V\n', min(V_group));
fprintf('V_max output:             %.5f V\n', V_max);
fprintf('max(V_group):             %.5f V\n', max(V_group));
fprintf('group spread:             %.3f mV\n', 1000*(max(V_group)-min(V_group)));

fprintf('\nTemperature stats:\n');
fprintf('T_core final:             %.3f C\n', T_core);
fprintf('T_surf final:             %.3f C\n', T_surf);
fprintf('T_max output:             %.3f C\n', T_max);
fprintf('max(T_sensor):            %.3f C\n', max(T_sensor));
fprintf('T_avg output:             %.3f C\n', T_avg);
fprintf('mean(T_sensor):           %.3f C\n', mean(T_sensor));

fprintf('\nSoC stats:\n');
fprintf('SoC_true final:           %.6f\n', SoC_true);
fprintf('min(SoC_group):           %.6f\n', min(SoC_group));
fprintf('max(SoC_group):           %.6f\n', max(SoC_group));
fprintf('SoC spread:               %.4f %%\n', 100*(max(SoC_group)-min(SoC_group)));

%% Pass/fail
pass = true;

if abs(err_group_sum) > 1e-3
    fprintf('\nFAIL: sum(V_group) does not match V_pack.\n');
    pass = false;
end

if abs(err_seg_sum) > 1e-3
    fprintf('\nFAIL: sum(V_segment) does not match V_pack.\n');
    pass = false;
end

if numel(V_group) ~= 75
    fprintf('\nFAIL: V_group length is not 75.\n');
    pass = false;
end

if numel(V_segment) ~= 5
    fprintf('\nFAIL: V_segment length is not 5.\n');
    pass = false;
end

if numel(T_sensor) ~= 120
    fprintf('\nFAIL: T_sensor length is not 120.\n');
    pass = false;
end

if abs(V_min - min(V_group)) > 1e-5 || abs(V_max - max(V_group)) > 1e-5
    fprintf('\nFAIL: V_min/V_max outputs do not match V_group.\n');
    pass = false;
end

if abs(T_max - max(T_sensor)) > 1e-5 || abs(T_avg - mean(T_sensor)) > 1e-5
    fprintf('\nFAIL: T_max/T_avg outputs do not match T_sensor.\n');
    pass = false;
end

if pass
    fprintf('\nPASS: AMS vector outputs are consistent.\n');
else
    fprintf('\nAMS output sanity test failed. Fix before codegen.\n');
end

%% Plots
figure;
plot(V_group, 'o-');
grid on;
xlabel('Group index');
ylabel('Voltage [V]');
title('Final V\_group[75]');

figure;
bar(V_segment);
grid on;
xlabel('Segment index');
ylabel('Voltage [V]');
title('Final V\_segment[5]');

figure;
plot(T_sensor, 'o-');
grid on;
xlabel('Sensor index');
ylabel('Temperature [C]');
title('Final T\_sensor[120]');

%% Helpers
function x = local_last_scalar(ts)
    D = ts.Data;
    x = double(squeeze(D(end)));
end

function v = local_last_vector(ts)
    D = ts.Data;
    ntime = numel(ts.Time);

    if isvector(D)
        v = squeeze(D);
    elseif size(D, 1) == ntime
        v = squeeze(D(end, :));
    elseif ndims(D) == 3 && size(D, 3) == ntime
        v = squeeze(D(:, :, end));
    elseif ndims(D) == 3 && size(D, 1) == ntime
        v = squeeze(D(end, :, :));
    else
        v = squeeze(D);
    end

    v = double(v(:));
end