%% generate_table1_ecm_parameters.m
% Table 1 — ECM Parameters
%
% Generates representative ECM parameter table at selected SoC.
% Default SoC = 0.80 to match HPPC validation figure.
%
% Parameters:
%   R0(SoC,T)
%   R1(SoC,T)
%   C1(SoC,T)
%   R2 fixed
%   C2 fixed
%   tau1 = R1*C1
%   tau2 = R2*C2

clear; clc;

%% ---------------- USER SETTINGS ----------------

param_file = "C:\Users\mahad\OneDrive\Documents\MATLAB\params_phaseC_final_validated_25C.mat";

TEMPS_C = [25 10 0];

SOC_REP = 0.80;

out_csv = "Table1_ECM_Parameters.csv";
out_tex = "Table1_ECM_Parameters.tex";

%% ---------------- LOAD PARAMETERS ----------------

if ~isfile(param_file)
    error("Parameter file not found:\n%s", param_file);
end

S = load(param_file);

required = [
    "temp_bp"
    "soc_common"
    "R0_2d_fix"
    "R1_2d"
    "inv_C1_2d_fixed"
    "R2"
    "C2"
];

for k = 1:numel(required)
    if ~isfield(S, required(k))
        error("Missing required variable: %s", required(k));
    end
end

temp_bp = double(S.temp_bp(:));
soc_bp  = double(S.soc_common(:));

R0_2d = double(S.R0_2d_fix);
R1_2d = double(S.R1_2d);
C1_2d = 1 ./ double(S.inv_C1_2d_fixed);

R2 = double(S.R2);
C2 = double(S.C2);

if numel(R2) > 1
    R2 = R2(1);
end

if numel(C2) > 1
    C2 = C2(1);
end

%% ---------------- ORIENTATION CHECK ----------------
% Expected table shape is either:
%   rows = temp, columns = SoC
% or
%   rows = SoC, columns = temp

[R0_2d, R1_2d, C1_2d] = orient_luts(R0_2d, R1_2d, C1_2d, temp_bp, soc_bp);

%% ---------------- EXTRACT REPRESENTATIVE VALUES ----------------

Temp_C = TEMPS_C(:);
SoC = SOC_REP * ones(numel(TEMPS_C), 1);

R0_ohm = nan(numel(TEMPS_C),1);
R1_ohm = nan(numel(TEMPS_C),1);
C1_F   = nan(numel(TEMPS_C),1);

for i = 1:numel(TEMPS_C)
    T = TEMPS_C(i);

    R0_ohm(i) = interp2(soc_bp, temp_bp, R0_2d, SOC_REP, T, "linear");
    R1_ohm(i) = interp2(soc_bp, temp_bp, R1_2d, SOC_REP, T, "linear");
    C1_F(i)   = interp2(soc_bp, temp_bp, C1_2d, SOC_REP, T, "linear");
end

R2_ohm = R2 * ones(numel(TEMPS_C),1);
C2_F   = C2 * ones(numel(TEMPS_C),1);

tau1_s = R1_ohm .* C1_F;
tau2_s = R2_ohm .* C2_F;

Table1 = table( ...
    Temp_C, ...
    SoC, ...
    R0_ohm, ...
    R1_ohm, ...
    C1_F, ...
    R2_ohm, ...
    C2_F, ...
    tau1_s, ...
    tau2_s, ...
    'VariableNames', { ...
        'Temp_C', ...
        'SoC', ...
        'R0_ohm', ...
        'R1_ohm', ...
        'C1_F', ...
        'R2_ohm', ...
        'C2_F', ...
        'tau1_s', ...
        'tau2_s' ...
    });

writetable(Table1, out_csv);

disp(Table1);

fprintf("\nSaved CSV:\n%s\n", out_csv);

%% ---------------- WRITE IEEE LATEX TABLE ----------------

fid = fopen(out_tex, "w");

if fid == -1
    error("Could not open LaTeX output file.");
end

fprintf(fid, "%% Table 1 — ECM Parameters\n");
fprintf(fid, "\\begin{table}[t]\n");
fprintf(fid, "\\caption{Representative ECM parameters at SoC $\\approx %.2f$. The fast-branch parameters are interpolated from the temperature- and SoC-dependent LUTs. The slow branch uses fixed $R_2$ and $C_2$ values.}\n", SOC_REP);
fprintf(fid, "\\label{tab:ecm_parameters}\n");
fprintf(fid, "\\centering\n");
fprintf(fid, "\\begin{tabular}{c c c c c c c c}\n");
fprintf(fid, "\\hline\n");
fprintf(fid, "$T$ & $R_0$ & $R_1$ & $C_1$ & $R_2$ & $C_2$ & $\\tau_1$ & $\\tau_2$ \\\\\n");
fprintf(fid, "($^\\circ$C) & ($\\Omega$) & ($\\Omega$) & (F) & ($\\Omega$) & (F) & (s) & (s) \\\\\n");
fprintf(fid, "\\hline\n");

for i = 1:height(Table1)
    fprintf(fid, "%d & %.5f & %.5f & %.1f & %.5f & %.1f & %.2f & %.2f \\\\\n", ...
        Table1.Temp_C(i), ...
        Table1.R0_ohm(i), ...
        Table1.R1_ohm(i), ...
        Table1.C1_F(i), ...
        Table1.R2_ohm(i), ...
        Table1.C2_F(i), ...
        Table1.tau1_s(i), ...
        Table1.tau2_s(i));
end

fprintf(fid, "\\hline\n");
fprintf(fid, "\\end{tabular}\n");
fprintf(fid, "\\end{table}\n");

fclose(fid);

fprintf("Saved LaTeX table:\n%s\n", out_tex);

%% ---------------- PRINT PAPER NOTE ----------------

fprintf("\nPaper note:\n");
fprintf("R0, R1, and C1 are interpolated from SoC-temperature LUTs at SoC = %.2f. ", SOC_REP);
fprintf("R2 and C2 are fixed slow-branch parameters. ");
fprintf("tau1 = R1*C1 and tau2 = R2*C2.\n");

%% ========================================================================
% Local helper
% ========================================================================

function [R0o, R1o, C1o] = orient_luts(R0, R1, C1, temp_bp, soc_bp)

    nt = numel(temp_bp);
    ns = numel(soc_bp);

    sz = size(R0);

    if isequal(sz, [nt ns])
        R0o = R0;
        R1o = R1;
        C1o = C1;
        return;
    end

    if isequal(sz, [ns nt])
        R0o = R0.';
        R1o = R1.';
        C1o = C1.';
        return;
    end

    error("Unexpected LUT shape. R0 size is [%d %d], expected [%d %d] or [%d %d].", ...
        sz(1), sz(2), nt, ns, ns, nt);
end