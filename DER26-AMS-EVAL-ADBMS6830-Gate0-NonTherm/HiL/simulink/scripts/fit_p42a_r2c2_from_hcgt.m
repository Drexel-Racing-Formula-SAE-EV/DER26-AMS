clear; clc; close all;

%% Paths
root = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'extracted_minimal', 'Dataset_Molicell_P42A');

in_param_file = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'mat_params', ...
    'params_p42a_75s6p_r0_r1c1_real_v1.mat');

out_param_file = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'mat_params', ...
    'params_p42a_75s6p_r0_r1c1_r2c2_real_v2.mat');

load(in_param_file);

cells = {'A2','A4','A6','A8','A9','A11','A12','A13','A15','A19','A21','A37'};
temps = [5 25 40];

soc_hcgt_1c = [95 85 75 65 50 35 25 15] ./ 100;

Q_cell_Ah = 4.2;
I_1C = Q_cell_Ah;
I_tol = 0.65 * I_1C;

R2_all = [];
C2_all = [];
tau2_all = [];

fprintf('\n======= Fitting scalar R2/C2 from HCGT 1C discharge relaxation residual =======\n');

for jt = 1:numel(temps)
    T = temps(jt);

    hfile = fullfile(root, 'phase_2', 'HCGT', sprintf('%ddegC', T), ...
        'mat', sprintf('HCGT_%ddegC.mat', T));

    H = load(hfile);

    fprintf('\n---------------- %d degC ----------------\n', T);

    for ic = 1:numel(cells)
        cname = cells{ic};

        if ~isfield(H.hcgt_data, cname)
            continue;
        end

        A = H.hcgt_data.(cname);

        t = double(A.time(:));
        I = double(A.current(:));
        V = double(A.voltage(:));

        good = isfinite(t) & isfinite(I) & isfinite(V);
        t = t(good);
        I = I(good);
        V = V(good);

        [t, idx] = sort(t);
        I = I(idx);
        V = V(idx);

        n_neg = sum(I < -1.0);
        n_pos = sum(I >  1.0);

        if n_neg >= n_pos
            pulse_mask = I < 0 & abs(abs(I) - I_1C) <= I_tol;
        else
            pulse_mask = I > 0 & abs(abs(I) - I_1C) <= I_tol;
        end

        [starts, ends] = local_segments(pulse_mask);

        keep = false(size(starts));

        for k = 1:numel(starts)
            dur = t(ends(k)) - t(starts(k));
            Iavg = mean(abs(I(starts(k):ends(k))), 'omitnan');

            keep(k) = dur >= 1.0 && dur <= 180.0 && ...
                      Iavg >= 0.35 * I_1C && Iavg <= 1.90 * I_1C;
        end

        starts = starts(keep);
        ends = ends(keep);

        nfit = min([numel(starts), numel(soc_hcgt_1c)]);

        if nfit < 5
            fprintf('%s: only %d usable pulse segments, skipping\n', cname, nfit);
            continue;
        end

        fprintf('%s: fitting slow residual on %d pulses\n', cname, nfit);

        for kp = 1:nfit
            s = starts(kp);
            e = ends(kp);

            soc_k = soc_hcgt_1c(kp);
            Iabs = mean(abs(I(s:e)), 'omitnan');
            tpulse = t(e) - t(s);

            r0 = e + 1;
            if r0 >= numel(t)
                continue;
            end

            rest_current_thresh = 0.20;
            next_pulse = find(abs(I(r0:end)) > rest_current_thresh, 1, 'first');

            if isempty(next_pulse)
                r1 = numel(t);
            else
                r1 = r0 + next_pulse - 3;
            end

            if r1 <= r0 + 20
                continue;
            end

            relax_idx = r0:r1;
            rel_t = t(relax_idx) - t(r0);
            rel_v = V(relax_idx);

            rel_keep = rel_t >= 0.5 & rel_t <= 240.0;
            rel_t = rel_t(rel_keep);
            rel_v = rel_v(rel_keep);

            if numel(rel_t) < 30
                continue;
            end

            % Estimate relaxation asymptote from final 15% of window.
            n = numel(rel_v);
            tail_n = max(5, round(0.15 * n));
            Vinf = median(rel_v(end-tail_n+1:end), 'omitnan');

            y_total = Vinf - rel_v;  % discharge relaxation deficit

            if max(y_total) < 0.003
                continue;
            end

            % Get fitted R1/C1 at this SOC/temp from existing tables.
            R1 = interp2(double(temp_bp(:).'), double(soc_common(:)), double(R1_2d), ...
                         double(T), double(soc_k), 'linear');

            invC1 = interp2(double(temp_bp(:).'), double(soc_common(:)), double(inv_C1_2d_fixed), ...
                            double(T), double(soc_k), 'linear');

            C1 = 1.0 / invC1;
            tau1 = R1 * C1;

            if ~isfinite(tau1) || tau1 < 0.5 || tau1 > 100
                continue;
            end

            A1_end = Iabs * R1 * (1.0 - exp(-tpulse / tau1));

            fast_term = A1_end .* exp(-rel_t ./ tau1);

            y_slow = y_total - fast_term;

            % Slow residual must be positive for log fit.
            ymax = max(y_slow);
            if ~isfinite(ymax) || ymax < 0.001
                continue;
            end

            fit_keep = y_slow > max(0.0005, 0.05 * ymax) & rel_t >= 5.0;

            t_fit = rel_t(fit_keep);
            y_fit = y_slow(fit_keep);

            if numel(t_fit) < 12
                continue;
            end

            t_fit = t_fit - t_fit(1);

            p = polyfit(t_fit, log(y_fit), 1);

            if p(1) >= 0
                continue;
            end

            tau2 = -1.0 / p(1);
            A2_end = exp(p(2));

            if tau2 < 20 || tau2 > 500
                continue;
            end

            denom = Iabs * (1.0 - exp(-tpulse / tau2));

            if denom <= 0
                continue;
            end

            R2_fit = A2_end / denom;
            C2_fit = tau2 / R2_fit;

            if R2_fit < 0.0002 || R2_fit > 0.030
                continue;
            end

            if C2_fit < 1000 || C2_fit > 1000000
                continue;
            end

            R2_all(end+1,1) = R2_fit; %#ok<SAGROW>
            C2_all(end+1,1) = C2_fit; %#ok<SAGROW>
            tau2_all(end+1,1) = tau2; %#ok<SAGROW>
        end
    end
end

fprintf('\nValid R2/C2 fits: %d\n', numel(R2_all));

if numel(R2_all) < 10
    warning('Too few valid R2/C2 fits. Keeping previous scalar R2/C2.');
else
    R2_med = median(R2_all, 'omitnan');
    tau2_med = median(tau2_all, 'omitnan');
    C2_med = tau2_med / R2_med;

    R2 = single(R2_med);
    C2 = single(C2_med);
end

fprintf('\n======= Final scalar slow branch =======\n');
fprintf('R2 = %.4f mOhm\n', double(R2) * 1000);
fprintf('C2 = %.1f F\n', double(C2));
fprintf('tau2 = %.2f s\n', double(R2) * double(C2));

save(out_param_file, ...
    'Q_nom', 'SoC_init', 'T_init', ...
    'temp_bp', 'temp_bp_ocv', ...
    'soc_ocv_common', 'soc_common', ...
    'OCV_2d', 'R0_2d_fix', ...
    'R1_2d', ...
    'inv_C1_2d_fixed', ...
    'neg_inv_R1C1_2d_fixed', ...
    'R2', 'C2', ...
    'Cc', 'Cs', 'Rcs', 'Rsa');

fprintf('\nSaved:\n%s\n', out_param_file);

if ~isempty(R2_all)
    figure;
    histogram(R2_all * 1000, 40);
    grid on;
    xlabel('R2 [mOhm]');
    ylabel('Count');
    title('P42A fitted R2 distribution');

    figure;
    histogram(tau2_all, 40);
    grid on;
    xlabel('\tau_2 [s]');
    ylabel('Count');
    title('P42A fitted tau2 distribution');

    figure;
    scatter(R2_all * 1000, tau2_all, 20, 'filled');
    grid on;
    xlabel('R2 [mOhm]');
    ylabel('\tau_2 [s]');
    title('P42A R2/tau2 accepted fits');
end

%% Local helper
function [starts, ends] = local_segments(mask)
    mask = mask(:) ~= 0;
    d = diff([false; mask; false]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
end