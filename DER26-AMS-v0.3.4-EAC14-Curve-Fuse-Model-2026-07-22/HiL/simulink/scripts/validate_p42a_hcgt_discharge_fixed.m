clear; clc; close all;

%% Paths / model
root = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'extracted_minimal', 'Dataset_Molicell_P42A');

mdl = 'drev_75s6p_p42a_accumulator_plant_v1_r0_r1c1';

param_file = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'mat_params', ...
    'params_p42a_75s6p_r0_r1c1_r2c2_real_v2.mat');

load(param_file);

%% Load HCGT trace
T_test = 25;
cell_id = 'A2';

hfile = fullfile(root, 'phase_2', 'HCGT', sprintf('%ddegC', T_test), ...
    'mat', sprintf('HCGT_%ddegC.mat', T_test));

H = load(hfile);
A = H.hcgt_data.(cell_id);

t_all = double(A.time(:));
I_raw = double(A.current(:));
V_all = double(A.voltage(:));

good = isfinite(t_all) & isfinite(I_raw) & isfinite(V_all);
t_all = t_all(good);
I_raw = I_raw(good);
V_all = V_all(good);

[t_all, idx] = sort(t_all);
I_raw = I_raw(idx);
V_all = V_all(idx);

%% Find true 1C pulse candidates
Q_cell_Ah = 4.2;
I_target = Q_cell_Ah;        % true 1C
I_tol = 0.25 * I_target;     % tight enough to reject C/2

pulse_mask = abs(abs(I_raw) - I_target) <= I_tol;

[starts, ends] = local_segments(pulse_mask);

fprintf('\n======= Candidate ~1C segments =======\n');

best_idx = NaN;
best_sign_factor = NaN;
pulse_count = 0;

for k = 1:numel(starts)
    s = starts(k);
    e = ends(k);

    dur = t_all(e) - t_all(s);
    Iavg_raw = mean(I_raw(s:e), 'omitnan');

    if dur < 1.0 || dur > 180.0 || abs(Iavg_raw) < 0.5
        continue;
    end

    pulse_count = pulse_count + 1;

    pre_idx = max(1, s-20):max(1, s-2);
    post_idx = min(numel(V_all), s+2):min(numel(V_all), s+20);

    Vpre = median(V_all(pre_idx), 'omitnan');
    Vpost = median(V_all(post_idx), 'omitnan');
    dV = Vpost - Vpre;

    fprintf('seg %2d / pulse %2d: t %.2f -> %.2f s, Iraw %.3f A, dV %.2f mV\n', ...
        k, pulse_count, t_all(s), t_all(e), Iavg_raw, dV*1000);

    % Select first true discharge pulse: voltage drops at pulse start.
    if dV < -3e-3
        best_idx = k;
        best_sign_factor = 1.0 / sign(Iavg_raw);
        break;
    end
end

if ~isfinite(best_idx)
    error('No true 1C discharge segment found. Check candidate printout.');
end

s = starts(best_idx);
e = ends(best_idx);

I_cell_discharge_pos_all = best_sign_factor .* I_raw;

fprintf('\nSelected 1C discharge pulse:\n');
fprintf('  segment index = %d\n', best_idx);
fprintf('  start = %.2f s\n', t_all(s));
fprintf('  end   = %.2f s\n', t_all(e));
fprintf('  raw Iavg = %.3f A\n', mean(I_raw(s:e), 'omitnan'));
fprintf('  corrected Iavg = %.3f A discharge-positive\n', ...
    mean(I_cell_discharge_pos_all(s:e), 'omitnan'));
fprintf('  sign_factor = %.0f\n', best_sign_factor);

%% Build validation window: discharge + relaxation only, stop before next pulse
pre_s = 5.0;

next_after = find(abs(I_raw(e+1:end)) > 0.5, 1, 'first');
if isempty(next_after)
    t1 = min(t_all(end), t_all(e) + 180.0);
else
    next_pulse_start_idx = e + next_after;
    t1 = min(t_all(next_pulse_start_idx) - 0.2, t_all(e) + 180.0);
end

t0 = max(t_all(1), t_all(s) - pre_s);

win = t_all >= t0 & t_all <= t1;

t = t_all(win);
I_cell = I_cell_discharge_pos_all(win);
V_meas_cell = V_all(win);

t = t - t(1);

%% Correct initial SoC: use HCGT OCV/SOC metadata, not inverse patched OCV
% For the first selected discharge pulse, use first HCGT OCV/SOC point.
% This avoids the artificial endpoint-patched OCV inverse-map error.
if isfield(A, 'ocvsoc') && numel(A.ocvsoc) >= 1
    SoC_init = single(A.ocvsoc(1));
else
    SoC_init = single(0.9485);
end

T_init = single(T_test);

fprintf('\nInitial condition:\n');
fprintf('  dataset SoC_init = %.6f\n', SoC_init);
if isfield(A, 'ocv') && numel(A.ocv) >= 1
    fprintf('  dataset OCV ref  = %.5f V\n', A.ocv(1));
end
fprintf('  measured V_start = %.5f V\n', V_meas_cell(1));

%% Convert single-cell current to 75s6p pack input
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

simIn = simIn.setVariable('SoC_init', SoC_init);
simIn = simIn.setVariable('T_init', T_init);

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

fprintf('\n======= HCGT true 1C discharge validation v4 =======\n');
fprintf('Cell: %s, Temp: %d C\n', cell_id, T_test);
fprintf('Window duration: %.1f s\n', t(end));
fprintf('RMSE: %.2f mV\n', rmse_mV);
fprintf('Bias: %.2f mV\n', bias_mV);
fprintf('Max abs error: %.2f mV\n', max_mV);
fprintf('Model SoC start/end: %.6f -> %.6f\n', SoC.Data(1), SoC.Data(end));

%% Plots
figure;
plot(t, V_meas_cell, 'LineWidth', 1.2); hold on;
plot(t_model, V_model_cell, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Cell voltage [V]');
legend('Measured P42A', 'Model', 'Location', 'best');
title('P42A HCGT true 1C discharge validation v4');

figure;
plot(t_model, err_mV, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Model - measured [mV]');
title('P42A HCGT true 1C discharge error v4');

figure;
plot(t, I_cell, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Cell current, discharge positive [A]');
title('Replay current - true 1C discharge only');

%% Helper
function [starts, ends] = local_segments(mask)
    mask = mask(:) ~= 0;
    d = diff([false; mask; false]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
end