clear; clc;

root = fullfile(getenv('USERPROFILE'), ...
    'Documents', 'BMS_P42A_Data', 'extracted_minimal', 'Dataset_Molicell_P42A');

src_dir = fullfile(root, 'src');
addpath(genpath(src_dir));

%% Model breakpoints
Q_nom = single(4.2);       % physical P42A cell Ah, not 6p group Ah
SoC_init = single(1.0);
T_init   = single(25.0);

temp_bp     = single([5 25 40]);
temp_bp_ocv = single([5 25 40]);

soc_ocv_common = single((0:0.01:1).');          % 101x1
soc_common     = single(linspace(0.10,1.00,12).'); % 12x1

cell_id = 'A2';

%% Build OCV_2d from CDT C/20 discharge curves
OCV_2d = zeros(numel(soc_ocv_common), numel(temp_bp), 'single');

for jt = 1:numel(temp_bp)
    T = temp_bp(jt);

    cdt_file = fullfile(root, 'phase_2', 'CDT', sprintf('%ddegC',T), ...
        'mat', sprintf('%s_CDT_%ddegC.mat', cell_id, T));

    S = load(cdt_file);

    if isfield(S, 'cby20')
        D = S.cby20;
    elseif isfield(S, 'cby10')
        D = S.cby10;
    else
        error('No cby20 or cby10 found in %s', cdt_file);
    end

    V = double(D.voltage_dis(:));
    cap = double(D.capacity_dis(:));

    good = isfinite(V) & isfinite(cap);
    V = V(good);
    cap = cap(good);

    cap = cap - min(cap);
    cap_full = max(cap);

    if cap_full <= 0
        error('Bad capacity vector in %s', cdt_file);
    end

    soc = 1.0 - cap ./ cap_full;

    % Sort for interp1
    [soc_sort, idx] = sort(soc);
    V_sort = V(idx);

    % Remove duplicate SOC points
    [soc_u, ia] = unique(soc_sort, 'stable');
    V_u = V_sort(ia);

    ocv_interp = interp1(soc_u, V_u, double(soc_ocv_common), 'linear', 'extrap');

    % Conservative voltage clamp for P42A
    ocv_interp = min(max(ocv_interp, 2.50), 4.25);

    OCV_2d(:, jt) = single(ocv_interp);
end

%% Build R0_2d_fix from HCGT precomputed 1C R0, fallback to C/2
R0_2d_fix = zeros(numel(soc_common), numel(temp_bp), 'single');

hcgt_soc_default = [95 85 75 65 50 35 25 15 5] ./ 100;

for jt = 1:numel(temp_bp)
    T = temp_bp(jt);

    hfile = fullfile(root, 'phase_2', 'HCGT', sprintf('%ddegC',T), ...
        'mat', sprintf('HCGT_%ddegC.mat', T));

    H = load(hfile);
    A = H.hcgt_data.(cell_id);

    if isfield(A, 'r0_1C')
        r = double(A.r0_1C(:));
    elseif isfield(A, 'r0_cby2')
        r = double(A.r0_cby2(:));
    else
        error('No usable R0 field found for %s at %dC', cell_id, T);
    end

    r = r(isfinite(r) & r > 0 & r < 0.2);

    % Dataset R0 may contain extra rows/columns. Use first matching SOC count.
    n = min(numel(r), numel(hcgt_soc_default));
    r = r(1:n);
    soc_r = hcgt_soc_default(1:n);

    % Interpolate to model SOC breakpoints
    [soc_sort, idx] = sort(soc_r);
    r_sort = r(idx);

    R0_col = interp1(soc_sort, r_sort, double(soc_common), 'linear', 'extrap');

    % Sanity clamp: P42A DCIR should be around 10-25 mOhm cell-level
    R0_col = min(max(R0_col, 0.006), 0.050);

    R0_2d_fix(:, jt) = single(R0_col);
end

%% First-pass RC branch placeholders
% These are cell-level ECM parameters. Replace later with relaxation fits.
R1_2d = single(0.0035 * ones(numel(soc_common), numel(temp_bp)));

C1_2d = single(3000.0 * ones(numel(soc_common), numel(temp_bp)));

inv_C1_2d_fixed = single(1.0 ./ C1_2d);
neg_inv_R1C1_2d_fixed = single(-1.0 ./ (R1_2d .* C1_2d));

R2 = single(0.0040);
C2 = single(12000.0);

%% First-pass thermal parameters for one physical P42A cell
% Since model uses I_cell = I_pack/6 and Q_nom = 4.2Ah, keep this cell-level.
Cc  = single(55.0);
Cs  = single(15.0);
Rcs = single(1.5);
Rsa = single(8.0);

%% Save parameter file
out_dir = fullfile(getenv('USERPROFILE'), 'Documents', 'BMS_P42A_Data', 'mat_params');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

out_file = fullfile(out_dir, 'params_p42a_75s6p_initial.mat');

save(out_file, ...
    'Q_nom', 'SoC_init', 'T_init', ...
    'temp_bp', 'temp_bp_ocv', ...
    'soc_ocv_common', 'soc_common', ...
    'OCV_2d', ...
    'R0_2d_fix', ...
    'R1_2d', ...
    'inv_C1_2d_fixed', ...
    'neg_inv_R1C1_2d_fixed', ...
    'R2', 'C2', ...
    'Cc', 'Cs', 'Rcs', 'Rsa');

fprintf('\nSaved:\n%s\n', out_file);

%% Print sanity checks
fprintf('\nOCV full at 25C: %.4f V\n', OCV_2d(end,2));
fprintf('OCV empty at 25C: %.4f V\n', OCV_2d(1,2));
fprintf('R0 25C range: %.3f to %.3f mOhm\n', ...
    min(R0_2d_fix(:,2))*1000, max(R0_2d_fix(:,2))*1000);
fprintf('Pack full voltage estimate: %.2f V\n', 75 * OCV_2d(end,2));