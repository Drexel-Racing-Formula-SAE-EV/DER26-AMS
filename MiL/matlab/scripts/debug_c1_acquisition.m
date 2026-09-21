%DEBUG_C1_ACQUISITION Re-run the C1 reference acquisition diagnostic.
%
% Intended after the 2026-08-28 rest-OCV acquisition correction. Production
% algorithms are disabled here so this isolates the independent reference.

clear functions;
rehash;
run(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), ...
    'matlab','scripts','setup_mil.m'));

cfg = mil.load_scenario('c1_hppc_bad_init');
cfg.production.estimator.enabled = false;
cfg.production.sop.enabled = false;
cfg.production.soh.enabled = false;
cfg.fuse.enabled = false;

basecfg = cfg;
basecfg.reference_ekf.initial_soc_offset = 0;
basecfg.reference_ekf.acquisition.enabled = false;
basecfg.acceptance.ekf.acquisition_mode = false;

base = mil.run_scenario(basecfg,'RunSoP',false);
bad = mil.run_scenario(cfg,'RunSoP',false);

vars = {'pass','soc_rmse','soc_p95_abs','soc_worst_abs', ...
    'convergence_time_s','post_convergence_soc_rmse', ...
    'post_convergence_soc_worst_abs','reject_fraction', ...
    'acquisition_completed','acquisition_time_s', ...
    'acquisition_reason','acquisition_anchor_soc','nis_mean','nees_mean'};

disp('Correct-initialization reference:');
base_table = struct2table(base.metrics.ekf.segment);
disp(base_table(:,vars));
disp('Directed +20 percentage-point acquisition reference:');
bad_table = struct2table(bad.metrics.ekf.segment);
disp(bad_table(:,vars));

r0_range = [min(base.reference.ekf.r0_ohm,[],1); ...
            max(base.reference.ekf.r0_ohm,[],1)];
disp('Correct-init LUT R0 range [min; max] ohm:');
disp(r0_range);
