clear; clc; close all;

%% Paths / model
root = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'extracted_minimal', 'Dataset_Molicell_P42A');

mdl = 'drev_75s6p_p42a_accumulator_plant_v1_r0_r1c1';

param_file = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'mat_params', ...
    'params_p42a_75s6p_r0_r1c1_r2c2_real_v2.mat');

load(param_file);

%% Load one real HCGT cell trace
T_test = 25;
cell_id = 'A2';

hfile = fullfile(root, 'phase_2', 'HCGT', sprintf('%ddegC', T_test), ...
    'mat', sprintf('HCGT_%ddegC.mat', T_test));

H = load(hfile);
A = H.hcgt_data.(cell_id);

t_all = double(A.time(:));
I_all = double(A.current(:));
V_all = double(A.voltage(:));

good = isfinite(t_all) & isfinite(I_all) & isfinite(V_all);
t_all = t_all(good);
I_all = I_all(good);
V_all = V_all(good);

[t_all, idx] = sort(t_all);
I_all = I_all(idx);
V_all = V_all(idx);

%% Detect 1C discharge pulse
Q_cell_Ah = 4.2;
I_1C = Q_cell_Ah;
I_tol = 0.65 * I_1C;

% Dataset likely uses negative current for discharge.
if sum(I_all < -1.0) >= sum(I_all > 1.0)
    I_cell_discharge_pos = -I_all;
else
    I_cell_discharge_pos = I_all;
end

pulse_mask = I_cell_discharge_pos > 0 & abs(I_cell_discharge_pos - I_1C) <= I_tol;

[starts, ends] = local_segments(pulse_mask);

% Keep real pulse segments.
keep = false(size(starts));
for k = 1:numel(starts)
    dur = t_all(ends(k)) - t_all(starts(k));
    Iavg = mean(I_cell_discharge_pos(starts(k):ends(k)), 'omitnan');

    keep(k) = dur >= 1.0 && dur <= 180.0 && ...
              Iavg >= 0.35 * I_1C && Iavg <= 1.90 * I_1C;
end

starts = starts(keep);
ends = ends(keep);

if isempty(starts)
    error('No usable 1C discharge pulse found.');
end

% Pick first 1C pulse.
pidx = 1;
s = starts(pidx);
e = ends(pidx);

% Include pre-rest and post-relaxation.
pre_s  = 60;
post_s = 240;

t0 = max(t_all(1), t_all(s) - pre_s);
t1 = min(t_all(end), t_all(e) + post_s);

win = t_all >= t0 & t_all <= t1;

t = t_all(win);
I_cell = I_cell_discharge_pos(win);
V_meas_cell = V_all(win);

% Reset time to zero.
t = t - t(1);

%% Set initial SoC close to corresponding HCGT OCV point
% A.ocvsoc starts near 95%, matching first pulse region.
if isfield(A, 'ocvsoc') && ~isempty(A.ocvsoc)
    SoC_init = single(A.ocvsoc(1));
else
    SoC_init = single(0.95);
end

T_init = single(T_test);

%% Convert single-cell test current to 75s6p pack input
% Plant internally does I_cell = I_pack / 6.
I_pack = single(6.0 * I_cell);
T_amb = single(T_test * ones(size(t)));

%% Load/update model
load_system(mdl);

set_param([mdl '/I_pack_to_cell'], 'Gain', '1/6');
set_param([mdl '/V_cell_to_pack'], 'Gain', '75');
set_param([mdl '/dSoC_per_step'], 'Gain', '-1/(3600*Q_nom)');
set_param([mdl '/SoC_Integrator'], 'SampleTime', '0.1');
set_param([mdl '/ T_clamp'], 'LowerLimit', '5', 'UpperLimit', '40');

set_param(mdl, 'SimulationCommand', 'update');

%% External input
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries(I_pack, t), 'I_pack');
ds = ds.addElement(timeseries(T_amb, t), 'T_amb');

simIn = Simulink.SimulationInput(mdl);
simIn = simIn.setExternalInput(ds);
simIn = simIn.setModelParameter( ...
    'StopTime', num2str(t(end)), ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset');

out = sim(simIn);

Y = out.get('yout');

V_pack = Y.getElement(1).Values;
SoC    = Y.getElement(4).Values;

V_model_cell = double(V_pack.Data(:)) ./ 75.0;
t_model = double(V_pack.Time(:));

V_meas_interp = interp1(t, V_meas_cell, t_model, 'linear', 'extrap');

err_mV = 1000.0 * (V_model_cell - V_meas_interp);

rmse_mV = sqrt(mean(err_mV.^2, 'omitnan'));
bias_mV = mean(err_mV, 'omitnan');
max_mV  = max(abs(err_mV));

fprintf('\n======= HCGT 1C pulse validation =======\n');
fprintf('Cell: %s, Temp: %d C\n', cell_id, T_test);
fprintf('Initial SoC used: %.4f\n', SoC_init);
fprintf('Window duration: %.1f s\n', t(end));
fprintf('RMSE: %.2f mV\n', rmse_mV);
fprintf('Bias: %.2f mV\n', bias_mV);
fprintf('Max abs error: %.2f mV\n', max_mV);
fprintf('Model SoC start/end: %.6f -> %.6f\n', SoC.Data(1), SoC.Data(end));

figure;
plot(t, V_meas_cell, 'LineWidth', 1.2); hold on;
plot(t_model, V_model_cell, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Cell voltage [V]');
legend('Measured P42A', 'Model', 'Location', 'best');
title('P42A HCGT 1C Pulse Validation');

figure;
plot(t_model, err_mV, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Model - measured [mV]');
title('P42A HCGT 1C Pulse Error');

figure;
plot(t, I_cell, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Cell current, discharge positive [A]');
title('Replay current');

%% Helper
function [starts, ends] = local_segments(mask)
    mask = mask(:) ~= 0;
    d = diff([false; mask; false]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
end