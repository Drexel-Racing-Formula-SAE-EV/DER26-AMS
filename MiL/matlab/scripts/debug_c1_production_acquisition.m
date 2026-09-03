%DEBUG_C1_PRODUCTION_ACQUISITION Directed production-C startup checks.
%
% This script executes the checked-in AMS/Core/Src/estimator/ams_soc_ekf.c
% through MiL/host/production_estimator_runner. It is production-parity MiL/SIL
% evidence, not an independent oracle and not HIL.
%
% The production candidate uses:
%   - constrained SoC-only dynamic correction while acquisition is unresolved,
%   - retryable 20 s fixed-basis relaxation acquisition (10 s / 35 s basis),
%   - segment consensus before re-anchoring,
%   - conservative covariance reset and adaptive-R reset on anchor.

clear functions;
rehash;
script_path = mfilename('fullpath');
repo_root = fileparts(fileparts(fileparts(fileparts(script_path))));
run(fullfile(repo_root,'MiL','matlab','scripts','setup_mil.m'));

base = mil.load_scenario('c1_hppc_bad_init');
base.reference_ekf.enabled = false;
base.production.estimator.enabled = true;
base.production.sop.enabled = false;
base.production.soh.enabled = false;
base.fuse.enabled = false;
base.sop_oracle.enabled = false;
base.acceptance.ekf.acquisition_expected = true;

vars = {'pass','soc_rmse','soc_p95_abs','soc_worst_abs', ...
    'convergence_time_s','post_convergence_soc_rmse', ...
    'post_convergence_soc_worst_abs','reject_fraction', ...
    'soc_error_sigma_max','acquisition_completed','acquisition_time_s', ...
    'acquisition_reason','acquisition_anchor_soc','acquisition_vp1_V', ...
    'acquisition_vp2_V','acquisition_fit_rmse_mV_cell', ...
    'acquisition_fit_rcond','acquisition_consensus_soc', ...
    'acquisition_reject_count','acquisition_dynamic_update_fraction', ...
    'post_acquisition_soc_rmse','post_acquisition_soc_worst_abs', ...
    'post_acquisition_soc_error_sigma_max','production_nis_mean', ...
    'state_nees_mean','state_nees_p95','state_nees_max', ...
    'post_acquisition_nis_mean','post_acquisition_state_nees_mean', ...
    'post_acquisition_state_nees_p95','covariance_repair_count_max'};

relaxed = true_relaxed_case(base,25);

fprintf('\n===== PRODUCTION true relaxed +20 pp =====\n');
rp = mil.run_scenario(relaxed,'RunSoP',false,'Export',false);
show_segments(rp.metrics.production_ekf.segment,vars);

fprintf('\n===== PRODUCTION true relaxed -20 pp =====\n');
rm = relaxed;
rm.reference_ekf.initial_soc_offset = -0.20;
rmn = mil.run_scenario(rm,'RunSoP',false,'Export',false);
show_segments(rmn.metrics.production_ekf.segment,vars);

fprintf('\n===== PRODUCTION original HPPC +20 pp =====\n');
hp = base;
rhp = mil.run_scenario(hp,'RunSoP',false,'Export',false);
show_segments(rhp.metrics.production_ekf.segment,vars);

fprintf('\n===== PRODUCTION original HPPC -20 pp =====\n');
hm = base;
hm.reference_ekf.initial_soc_offset = -0.20;
rhm = mil.run_scenario(hm,'RunSoP',false,'Export',false);
show_segments(rhm.metrics.production_ekf.segment,vars);

fprintf('\n===== PRODUCTION boot under 10 A, then rest =====\n');
d = base;
d.profile = struct('kind','segments','duration_s',[20 100], ...
    'current_A',[10 0],'ambient_C',[25 25], ...
    'name','prod_c1d_nonzero_current_boot');
d.stop_time_s = 120;
rd = mil.run_scenario(d,'RunSoP',false,'Export',false);
show_segments(rd.metrics.production_ekf.segment,vars);

fprintf('\n===== PRODUCTION +1 A measured-current bias: NO false acquisition =====\n');
e = relaxed;
e.sensor.current.bias_50A_A = 1.0;
e.sensor.current.bias_800A_A = 1.0;
e.acceptance.ekf.acquisition_expected = false;
re = mil.run_scenario(e,'RunSoP',false,'Export',false);
show_segments(re.metrics.production_ekf.segment,vars);
if any([re.metrics.production_ekf.segment.acquisition_completed])
    error('mil:debug:ProductionCurrentBiasFalseAcquire', ...
        'Production +1 A current-bias case falsely completed fixed-basis acquisition.');
end
if any([re.metrics.production_ekf.segment.convergence_time_s] > ...
        e.acceptance.ekf.convergence_time_max_s)
    error('mil:debug:ProductionDynamicAcquireConvergence', ...
        'Production dynamic acquisition did not meet the unchanged convergence gate.');
end

fprintf('\n===== PRODUCTION segment 1 +20 mV/cell through first window =====\n');
f = true_relaxed_case(base,50);
faults = repmat(struct('type','cell_bias','start_s',0,'end_s',20.1, ...
    'target',1,'value',0.020),1,15);
for k = 1:15, faults(k).target = k; end
f.sensor.faults = faults;
rf = mil.run_scenario(f,'RunSoP',false,'Export',false);
show_segments(rf.metrics.production_ekf.segment,vars);
if rf.metrics.production_ekf.segment(1).acquisition_reject_count < 1
    error('mil:debug:ProductionConsensusNotExercised', ...
        'Biased segment did not exercise production consensus rejection.');
end
fault_recovery=mil.metrics.production_acquisition_fault_recovery( ...
    rf.production.ekf,rf.truth,rf.metrics.production_ekf,1,20.1,f.acceptance.ekf);
disp('Fault-specific acquisition recovery:');
disp(fault_recovery);
if ~fault_recovery.pass
    error('mil:debug:ProductionConsensusRecovery', ...
        ['Biased segment failed fault-specific recovery. Do not loosen the ', ...
         'clean-data convergence gate to hide this failure.']);
end

fprintf('\n===== PRODUCTION segment 3 PEC invalid first 5 s =====\n');
g = true_relaxed_case(base,30);
g.sensor.faults = struct('type','segment_pec_invalid','start_s',0, ...
    'end_s',5,'target',3,'value',0);
rg = mil.run_scenario(g,'RunSoP',false,'Export',false);
show_segments(rg.metrics.production_ekf.segment,vars);
if rg.metrics.production_ekf.segment(3).acquisition_time_s <= 20
    error('mil:debug:ProductionPECValidityGate', ...
        'PEC-invalid segment 3 did not delay acquisition as required.');
end

fprintf('\n===== PRODUCTION true relaxed 5 C =====\n');
hc = relaxed;
hc.initial_temperature_C = 5;
hc.ambient_temperature_C = 5;
hc.profile.ambient_C(:) = 5;
rhc = mil.run_scenario(hc,'RunSoP',false,'Export',false);
show_segments(rhc.metrics.production_ekf.segment,vars);

fprintf('\n===== PRODUCTION true relaxed 40 C =====\n');
hh = relaxed;
hh.initial_temperature_C = 40;
hh.ambient_temperature_C = 40;
hh.profile.ambient_C(:) = 40;
rhh = mil.run_scenario(hh,'RunSoP',false,'Export',false);
show_segments(rhh.metrics.production_ekf.segment,vars);

fprintf('\n===== PRODUCTION warm discharge +20/-20 pp =====\n');
[wmB,wtB] = warm_stream(base,+60,60,120);
run_warm_pair(wmB,wtB,base,vars);

fprintf('\n===== PRODUCTION warm charge +20/-20 pp =====\n');
[wmC,wtC] = warm_stream(base,-10,60,120);
run_warm_pair(wmC,wtC,base,vars);

fprintf('\nPRODUCTION C1 ACQUISITION DIRECTED MATRIX COMPLETE\n');

function cfg = true_relaxed_case(base,rest_s)
cfg = base;
cfg.id = 'prod_c1a_true_relaxed_fixed_basis';
cfg.description = sprintf(['Production C1A: true relaxed startup with %.1f s ', ...
    'acquisition opportunity then directed excitation'],rest_s);
cfg.profile = struct('kind','segments', ...
    'duration_s',[rest_s 10 10 10 10 55], ...
    'current_A',[0 60 0 -10 0 0], ...
    'ambient_C',[25 25 25 25 25 25], ...
    'name','prod_c1a_true_relaxed_then_excitation');
cfg.stop_time_s = sum(cfg.profile.duration_s);
end

function [wm,wt] = warm_stream(base,current_A,drive_s,rest_s)
pre = base;
pre.reference_ekf.enabled = false;
pre.production.estimator.enabled = false;
pre.production.sop.enabled = false;
pre.production.soh.enabled = false;
pre.profile = struct('kind','segments','duration_s',[drive_s rest_s], ...
    'current_A',[current_A 0],'ambient_C',[25 25], ...
    'name','prod_warm_restart_precondition');
pre.stop_time_s = 'profile';
F = mil.run_scenario(pre,'RunSoP',false,'Export',false);
k = find(F.truth.time_s >= drive_s,1,'first');

fields = {'time_s','timestamp_s','pack_current_A','current_valid', ...
    'current_calibrated','current_polarity_validated','measurement_valid', ...
    'segment_voltage_V','segment_surface_max_C'};
wm = struct();
for q = 1:numel(fields)
    name = fields{q};
    value = F.measurement.(name);
    wm.(name) = value(k:end,:);
end
wm.time_s = wm.time_s-wm.time_s(1);
wm.timestamp_s = wm.timestamp_s-wm.timestamp_s(1);
wm.sequence = uint32((0:numel(wm.time_s)-1).');

wt = struct();
for name = {'segment_soc','segment_vp1_V','segment_vp2_V','segment_r0_ohm'}
    wt.(name{1}) = F.truth.(name{1})(k:end,:);
end
end

function run_warm_pair(wm,wt,base,vars)
for offset = [0.20 -0.20]
    init = min(max(wt.segment_soc(1,:).' + offset,0.01),0.99);
    P = mil.production.run_estimator(wm,init);
    acfg = base.acceptance.ekf;
    acfg.acquisition_mode = true;
    acfg.acquisition_expected = true;
    M = mil.metrics.production_ekf(P,wt,acfg);
    fprintf('initial offset %+0.2f\n',offset);
    show_segments(M.segment,vars);
end
end

function show_segments(segment,vars)
T = struct2table(segment);
disp(T(:,vars));
end
