function initial = build_initial_parameters(cell_cfg, dataset)
%BUILD_INITIAL_PARAMETERS Build OCV/R0 plus conservative RC/thermal priors.

[ocv, ocv_report] = hil.build_ocv_lut(dataset.ocv_tests, cell_cfg);
[r0, r0_report] = hil.fit_r0(dataset.r0_records, cell_cfg);
fit_options = cell_cfg.parameter_fit_options;

shape = [numel(cell_cfg.soc_breakpoints), ...
    numel(cell_cfg.temperature_breakpoints_C)];
r1 = repmat(single(fit_options.fallback_R1_ohm), shape);
c1 = repmat(single(fit_options.fallback_C1_F), shape);

initial = struct();
initial.OCV = ocv;
initial.R0 = r0;
initial.R1 = r1;
initial.C1 = c1;
initial.R2 = single(fit_options.fallback_R2_ohm);
initial.C2 = single(fit_options.fallback_C2_F);
initial.Cc = single(cell_cfg.thermal_prior.Cc_J_per_K);
initial.Cs = single(cell_cfg.thermal_prior.Cs_J_per_K);
initial.Rcs = single(cell_cfg.thermal_prior.Rcs_K_per_W);
initial.Rsa = single(cell_cfg.thermal_prior.Rsa_K_per_W);
initial.fit_quality = struct('ocv', ocv_report, 'r0', r0_report);
end
