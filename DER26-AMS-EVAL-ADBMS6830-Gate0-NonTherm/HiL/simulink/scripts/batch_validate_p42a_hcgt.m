clear; clc; close all;

%% ================= USER SETTINGS =================
mdl = 'drev_75s6p_p42a_accumulator_plant_v1_r0_r1c1';

root = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'extracted_minimal', 'Dataset_Molicell_P42A');

param_file = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'mat_params', ...
    'params_p42a_75s6p_r0_r1c1_r2c2_real_v2.mat');

out_dir = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'validation_results');

if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

% Start with this. Expand later if needed.
cells = {'A2','A4','A6','A8','A9','A11','A12','A13','A15','A19','A21','A37'};

% Batch set:
% 25C gets multiple C-rates.
% 5C/40C get 1C first.
jobs = [
    struct('T',25,'Crate',0.5)
    struct('T',25,'Crate',1.0)
    struct('T',25,'Crate',2.0)
    struct('T',25,'Crate',3.0)
    struct('T',5, 'Crate',1.0)
    struct('T',40,'Crate',1.0)
];

% Validate first discharge pulse per cell/job.
% Set to 8 later for all SOC levels.
max_pulses_per_case = 8;

pre_s = 5.0;
max_post_s = 180.0;

%% ================= LOAD PARAMS / MODEL =================
load(param_file);

load_system(mdl);

set_param([mdl '/I_pack_to_cell'], 'Gain', '1/6');
set_param([mdl '/V_cell_to_pack'], 'Gain', '75');
set_param([mdl '/dSoC_per_step'], 'Gain', '-1/(3600*Q_nom)');
set_param([mdl '/SoC_Integrator'], 'SampleTime', '0.1');
set_param([mdl '/ T_clamp'], 'LowerLimit', '5', 'UpperLimit', '40');

set_param(mdl, 'SimulationCommand', 'update');

%% ================= RESULT STORAGE =================
rows = {};

fprintf('\n======= P42A HCGT batch validation =======\n');

for j = 1:numel(jobs)
    T_test = jobs(j).T;
    Crate = jobs(j).Crate;
    I_target = 4.2 * Crate;

    fprintf('\n\n================ T=%dC, %.1fC =================\n', T_test, Crate);

    hfile = fullfile(root, 'phase_2', 'HCGT', sprintf('%ddegC', T_test), ...
        'mat', sprintf('HCGT_%ddegC.mat', T_test));

    H = load(hfile);

    for ic = 1:numel(cells)
        cell_id = cells{ic};

        if ~isfield(H.hcgt_data, cell_id)
            fprintf('%s missing, skipping\n', cell_id);
            continue;
        end

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

        %% Find target C-rate pulse candidates
        I_tol = max(0.20 * I_target, 0.40);  % reject neighboring C-rates

        pulse_mask = abs(abs(I_raw) - I_target) <= I_tol;
        [starts, ends] = local_segments(pulse_mask);

        discharge_segments = [];
        discharge_signs = [];

        pulse_counter = 0;

        for k = 1:numel(starts)
            s = starts(k);
            e = ends(k);

            dur = t_all(e) - t_all(s);
            Iavg_raw = mean(I_raw(s:e), 'omitnan');

            if dur < 1.0 || dur > 180.0 || abs(Iavg_raw) < 0.5
                continue;
            end

            pre_idx = max(1, s-20):max(1, s-2);
            post_idx = min(numel(V_all), s+2):min(numel(V_all), s+20);

            if numel(pre_idx) < 3 || numel(post_idx) < 3
                continue;
            end

            Vpre = median(V_all(pre_idx), 'omitnan');
            Vpost = median(V_all(post_idx), 'omitnan');
            dV = Vpost - Vpre;

            % Discharge pulse = voltage drops at start.
            if dV < -3e-3
                pulse_counter = pulse_counter + 1;

                sign_factor = 1.0 / sign(Iavg_raw);

                discharge_segments(end+1,:) = [s e pulse_counter]; %#ok<SAGROW>
                discharge_signs(end+1,1) = sign_factor; %#ok<SAGROW>
            end
        end

        if isempty(discharge_segments)
            fprintf('%s: no %.1fC discharge pulses found\n', cell_id, Crate);
            continue;
        end

        n_use = min(size(discharge_segments,1), max_pulses_per_case);

        for p = 1:n_use
            s = discharge_segments(p,1);
            e = discharge_segments(p,2);
            pulse_num = discharge_segments(p,3);
            sign_factor = discharge_signs(p);

            I_cell_discharge_pos_all = sign_factor .* I_raw;

            %% Validation window
            next_after = find(abs(I_raw(e+1:end)) > 0.5, 1, 'first');

            if isempty(next_after)
                t1 = min(t_all(end), t_all(e) + max_post_s);
            else
                next_start_idx = e + next_after;
                t1 = min(t_all(next_start_idx) - 0.2, t_all(e) + max_post_s);
            end

            t0 = max(t_all(1), t_all(s) - pre_s);
            win = t_all >= t0 & t_all <= t1;

            t = t_all(win);
            I_cell = I_cell_discharge_pos_all(win);
            V_meas_cell = V_all(win);

            t = t - t(1);

            %% Initial SoC from HCGT metadata
            if isfield(A, 'ocvsoc') && numel(A.ocvsoc) >= pulse_num
                SoC_init_case = single(A.ocvsoc(pulse_num));
            elseif isfield(A, 'ocvsoc') && ~isempty(A.ocvsoc)
                SoC_init_case = single(A.ocvsoc(1));
            else
                SoC_init_case = single(0.95);
            end

            T_init_case = single(T_test);

            %% Convert cell replay to pack replay
            I_pack = single(6.0 * I_cell);
            T_amb = single(T_test * ones(size(t)));

            ds = Simulink.SimulationData.Dataset;
            ds = ds.addElement(timeseries(I_pack, t), 'I_pack');
            ds = ds.addElement(timeseries(T_amb, t), 'T_amb');

            simIn = Simulink.SimulationInput(mdl);
            simIn = simIn.setExternalInput(ds);
            simIn = simIn.setVariable('SoC_init', SoC_init_case);
            simIn = simIn.setVariable('T_init', T_init_case);

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

            Iavg = mean(I_cell(I_cell > 0.5), 'omitnan');

            fprintf('%s T=%2dC %.1fC pulse%02d: RMSE=%6.2f mV, Bias=%7.2f mV, Max=%7.2f mV, SoC %.5f->%.5f\n', ...
                cell_id, T_test, Crate, pulse_num, rmse_mV, bias_mV, max_mV, ...
                SoC.Data(1), SoC.Data(end));

            rows(end+1,:) = { ...
                cell_id, T_test, Crate, pulse_num, Iavg, ...
                double(SoC_init_case), double(SoC.Data(1)), double(SoC.Data(end)), ...
                t(end), rmse_mV, bias_mV, max_mV}; %#ok<SAGROW>
        end
    end
end

%% ================= SAVE RESULTS =================
results = cell2table(rows, 'VariableNames', { ...
    'cell_id', 'temp_C', 'C_rate', 'pulse_num', 'Iavg_cell_A', ...
    'SoC_init_meta', 'SoC_model_start', 'SoC_model_end', ...
    'window_s', 'RMSE_mV', 'Bias_mV', 'MaxAbs_mV'});

csv_file = fullfile(out_dir, 'p42a_hcgt_batch_validation_summary.csv');
writetable(results, csv_file);

fprintf('\n======= Batch validation summary =======\n');
disp(results);

fprintf('\nSaved CSV:\n%s\n', csv_file);

%% Aggregate summary
if ~isempty(results)
    fprintf('\n======= Aggregate by temp/C-rate =======\n');

    [G, tempG, crateG] = findgroups(results.temp_C, results.C_rate);

    agg = table;
    agg.temp_C = tempG;
    agg.C_rate = crateG;
    agg.N = splitapply(@numel, results.RMSE_mV, G);
    agg.RMSE_mean_mV = splitapply(@mean, results.RMSE_mV, G);
    agg.RMSE_max_mV  = splitapply(@max,  results.RMSE_mV, G);
    agg.Bias_mean_mV = splitapply(@mean, results.Bias_mV, G);
    agg.MaxAbs_max_mV = splitapply(@max, results.MaxAbs_mV, G);

    disp(agg);

    agg_file = fullfile(out_dir, 'p42a_hcgt_batch_validation_aggregate.csv');
    writetable(agg, agg_file);

    fprintf('\nSaved aggregate CSV:\n%s\n', agg_file);
end

%% ================= LOCAL HELPER =================
function [starts, ends] = local_segments(mask)
    mask = mask(:) ~= 0;
    d = diff([false; mask; false]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
end