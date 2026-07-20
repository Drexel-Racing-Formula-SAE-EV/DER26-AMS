clear; clc; close all;

%% Paths
root = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'extracted_minimal', 'Dataset_Molicell_P42A');

in_param_file = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'mat_params', ...
    'params_p42a_75s6p_r0_real_v0_validated.mat');

if ~isfile(in_param_file)
    in_param_file = fullfile(getenv('USERPROFILE'), ...
        'Documents', 'BMS_P42A_Data', 'mat_params', ...
        'params_p42a_75s6p_r0_real_v0.mat');
end

out_param_file = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'mat_params', ...
    'params_p42a_75s6p_r0_r1c1_real_v1.mat');

load(in_param_file);

cells = {'A2','A4','A6','A8','A9','A11','A12','A13','A15','A19','A21','A37'};
temps = [5 25 40];

% 1C discharge pulses correspond to these HCGT SOC regions.
soc_hcgt_1c = [95 85 75 65 50 35 25 15] ./ 100;

Q_cell_Ah = 4.2;
I_1C = Q_cell_Ah;
I_tol = 0.65 * I_1C;

R1_raw  = nan(numel(soc_hcgt_1c), numel(temps), numel(cells));
C1_raw  = nan(numel(soc_hcgt_1c), numel(temps), numel(cells));
tau_raw = nan(numel(soc_hcgt_1c), numel(temps), numel(cells));

fprintf('\n======= Fitting R1/C1 from HCGT 1C discharge relaxations =======\n');

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

        % Dataset sign guard. Most battery datasets use negative discharge.
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

        if numel(starts) < 5
            fprintf('%s: only %d usable 1C pulse segments found, skipping\n', cname, numel(starts));
            continue;
        end

        nfit = min([numel(starts), numel(soc_hcgt_1c)]);

        fprintf('%s: fitting %d pulses\n', cname, nfit);

        for kp = 1:nfit
            s = starts(kp);
            e = ends(kp);

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

            if r1 <= r0 + 10
                continue;
            end

            relax_idx = r0:r1;
            rel_t = t(relax_idx) - t(r0);
            rel_v = V(relax_idx);

            rel_keep = rel_t >= 0.5 & rel_t <= 120.0;

            rel_t = rel_t(rel_keep);
            rel_v = rel_v(rel_keep);

            if numel(rel_t) < 20
                continue;
            end

            [tau, Aamp, ok] = local_fit_exp_recovery(rel_t, rel_v);

            if ~ok
                continue;
            end

            denom = Iabs * (1.0 - exp(-tpulse / tau));

            if denom <= 0
                continue;
            end

            R1 = Aamp / denom;
            C1 = tau / R1;

            if R1 < 0.0002 || R1 > 0.050
                continue;
            end

            if C1 < 100 || C1 > 200000
                continue;
            end

            R1_raw(kp,jt,ic)  = R1;
            C1_raw(kp,jt,ic)  = C1;
            tau_raw(kp,jt,ic) = tau;
        end
    end
end

%% Median across cells
R1_soc_temp  = nan(numel(soc_hcgt_1c), numel(temps));
C1_soc_temp  = nan(numel(soc_hcgt_1c), numel(temps));
tau_soc_temp = nan(numel(soc_hcgt_1c), numel(temps));

for jt = 1:numel(temps)
    for ks = 1:numel(soc_hcgt_1c)
        R1_soc_temp(ks,jt)  = median(squeeze(R1_raw(ks,jt,:)), 'omitnan');
        C1_soc_temp(ks,jt)  = median(squeeze(C1_raw(ks,jt,:)), 'omitnan');
        tau_soc_temp(ks,jt) = median(squeeze(tau_raw(ks,jt,:)), 'omitnan');
    end
end

%% Fill missing
for jt = 1:numel(temps)
    valid = isfinite(R1_soc_temp(:,jt)) & isfinite(C1_soc_temp(:,jt));

    if sum(valid) < 3
        warning('Too few valid R1/C1 points at %dC. Using fallback.', temps(jt));
        R1_soc_temp(:,jt)  = 0.0035;
        C1_soc_temp(:,jt)  = 3000.0;
        tau_soc_temp(:,jt) = R1_soc_temp(:,jt) .* C1_soc_temp(:,jt);
        continue;
    end

    R1_soc_temp(:,jt) = fillmissing(R1_soc_temp(:,jt), 'linear', 'EndValues', 'nearest');
    C1_soc_temp(:,jt) = fillmissing(C1_soc_temp(:,jt), 'linear', 'EndValues', 'nearest');
    tau_soc_temp(:,jt) = R1_soc_temp(:,jt) .* C1_soc_temp(:,jt);
end

%% Interpolate to model SOC grid
R1_2d_new = zeros(numel(soc_common), numel(temp_bp));
C1_2d_new = zeros(numel(soc_common), numel(temp_bp));

for jt = 1:numel(temp_bp)
    [soc_sort, idx] = sort(soc_hcgt_1c);

    R1_sort = R1_soc_temp(idx,jt);
    C1_sort = C1_soc_temp(idx,jt);

    R1_i = interp1(soc_sort, R1_sort, double(soc_common), 'linear', 'extrap');
    C1_i = interp1(soc_sort, C1_sort, double(soc_common), 'linear', 'extrap');

    R1_i = min(max(R1_i, 0.0003), 0.030);
    C1_i = min(max(C1_i, 300.0), 100000.0);

    R1_2d_new(:,jt) = R1_i;
    C1_2d_new(:,jt) = C1_i;
end

R1_2d = single(R1_2d_new);
C1_2d = single(C1_2d_new);

inv_C1_2d_fixed = single(1.0 ./ C1_2d);
neg_inv_R1C1_2d_fixed = single(-1.0 ./ (R1_2d .* C1_2d));

R2 = single(0.0040);
C2 = single(12000.0);

%% Save
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

fprintf('\n======= Saved updated R1/C1 params =======\n');
fprintf('%s\n', out_param_file);

fprintf('\nR1 ranges [mOhm]:\n');
fprintf('5C:  %.3f to %.3f\n', min(R1_2d(:,1))*1000, max(R1_2d(:,1))*1000);
fprintf('25C: %.3f to %.3f\n', min(R1_2d(:,2))*1000, max(R1_2d(:,2))*1000);
fprintf('40C: %.3f to %.3f\n', min(R1_2d(:,3))*1000, max(R1_2d(:,3))*1000);

C1_2d_print = 1.0 ./ double(inv_C1_2d_fixed);

fprintf('\nC1 ranges [F]:\n');
fprintf('5C:  %.1f to %.1f\n', min(C1_2d_print(:,1)), max(C1_2d_print(:,1)));
fprintf('25C: %.1f to %.1f\n', min(C1_2d_print(:,2)), max(C1_2d_print(:,2)));
fprintf('40C: %.1f to %.1f\n', min(C1_2d_print(:,3)), max(C1_2d_print(:,3)));

tau_2d = double(R1_2d) .* C1_2d_print;

fprintf('\nTau1 ranges [s]:\n');
fprintf('5C:  %.2f to %.2f\n', min(tau_2d(:,1)), max(tau_2d(:,1)));
fprintf('25C: %.2f to %.2f\n', min(tau_2d(:,2)), max(tau_2d(:,2)));
fprintf('40C: %.2f to %.2f\n', min(tau_2d(:,3)), max(tau_2d(:,3)));

figure;
plot(soc_common, R1_2d(:,1)*1000, '-o'); hold on;
plot(soc_common, R1_2d(:,2)*1000, '-o');
plot(soc_common, R1_2d(:,3)*1000, '-o');
grid on;
xlabel('SoC');
ylabel('R1 [mOhm]');
legend('5C','25C','40C','Location','best');
title('P42A fitted R1 table');

figure;
plot(soc_common, C1_2d_print(:,1), '-o'); hold on;
plot(soc_common, C1_2d_print(:,2), '-o');
plot(soc_common, C1_2d_print(:,3), '-o');
grid on;
xlabel('SoC');
ylabel('C1 [F]');
legend('5C','25C','40C','Location','best');
title('P42A fitted C1 table');

figure;
plot(soc_common, tau_2d(:,1), '-o'); hold on;
plot(soc_common, tau_2d(:,2), '-o');
plot(soc_common, tau_2d(:,3), '-o');
grid on;
xlabel('SoC');
ylabel('\tau_1 [s]');
legend('5C','25C','40C','Location','best');
title('P42A fitted tau1 table');

%% Local functions
function [starts, ends] = local_segments(mask)
    mask = mask(:) ~= 0;
    d = diff([false; mask; false]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
end

function [tau, Aamp, ok] = local_fit_exp_recovery(t, v)
    ok = false;
    tau = nan;
    Aamp = nan;

    t = double(t(:));
    v = double(v(:));

    good = isfinite(t) & isfinite(v);
    t = t(good);
    v = v(good);

    if numel(t) < 20
        return;
    end

    n = numel(v);
    tail_n = max(5, round(0.15 * n));
    Vinf = median(v(end-tail_n+1:end), 'omitnan');

    y = Vinf - v;
    ymax = max(y);

    if ~isfinite(ymax) || ymax < 0.002
        return;
    end

    keep = y > max(0.0005, 0.03 * ymax);

    t_fit = t(keep);
    y_fit = y(keep);

    if numel(t_fit) < 10
        return;
    end

    t_fit = t_fit - t_fit(1);
    logy = log(y_fit);

    p = polyfit(t_fit, logy, 1);

    if p(1) >= 0
        return;
    end

    tau_candidate = -1.0 / p(1);
    A_candidate = exp(p(2));

    if tau_candidate < 0.5 || tau_candidate > 300
        return;
    end

    if A_candidate < 0.001 || A_candidate > 1.0
        return;
    end

    tau = tau_candidate;
    Aamp = A_candidate;
    ok = true;
end