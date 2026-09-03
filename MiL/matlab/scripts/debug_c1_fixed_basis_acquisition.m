%DEBUG_C1_FIXED_BASIS_ACQUISITION Directed integrated reference checks.
%
% Runs only MATLAB/plant/reference paths. This remains the independent reference
% regression for the acquisition policy; the production-C candidate is exercised
% separately by debug_c1_production_acquisition.m. Acquisition is a SHADOW observer:
% normal EKF
% updates continue while the fixed-basis window is pending, and a successful
% fit may re-anchor the live state. Warm-restart cases keep physical plant state
% continuous and reboot only the estimator/measurement view.
%
% The original C1 HPPC profile does not contain a 20 s uninterrupted rest at
% startup, so it is NOT a valid positive-control stimulus for this acquisition
% policy. Positive controls use an explicit 25 s initial rest; fault cases that
% require a second clean acquisition window extend rest only as needed to prove
% retry. Boot-under-load and warm-restart cases test deferred acquisition separately.

clear functions;
rehash;
script_path = mfilename('fullpath');
repo_root = fileparts(fileparts(fileparts(fileparts(script_path))));
run(fullfile(repo_root,'MiL','matlab','scripts','setup_mil.m'));

base = mil.load_scenario('c1_hppc_bad_init');
base.production.estimator.enabled = false;
base.production.sop.enabled = false;
base.production.soh.enabled = false;
base.fuse.enabled = false;
base.sop_oracle.enabled = false;
base.reference_ekf.acquisition.hold_measurement_updates = false;

vars = {'pass','soc_rmse','soc_p95_abs','soc_worst_abs', ...
    'convergence_time_s','post_convergence_soc_rmse', ...
    'post_convergence_soc_worst_abs','reject_fraction', ...
    'acquisition_completed','acquisition_time_s', ...
    'acquisition_reason','acquisition_anchor_soc', ...
    'acquisition_vp1_V','acquisition_vp2_V', ...
    'acquisition_fit_rmse_mV_cell','acquisition_fit_rcond', ...
    'acquisition_consensus_soc','acquisition_reject_count', ...
    'acquisition_dynamic_update_fraction', ...
    'post_acquisition_soc_error_sigma_max', ...
    'post_acquisition_nis_mean','post_acquisition_nees_mean', ...
    'nis_mean','nees_mean'};

relaxed = true_relaxed_case(base,25);

fprintf('\n===== true relaxed +20 pp (25 s initial rest) =====\n');
rp = mil.run_scenario(relaxed,'RunSoP',false,'Export',false);
show_segments(rp.metrics.ekf.segment,vars);

fprintf('\n===== true relaxed -20 pp (25 s initial rest) =====\n');
rm = relaxed;
rm.reference_ekf.initial_soc_offset = -0.20;
rmn = mil.run_scenario(rm,'RunSoP',false,'Export',false);
show_segments(rmn.metrics.ekf.segment,vars);

fprintf('\n===== original HPPC +20 pp: dynamic acquisition + deferred fit =====\n');
hp = base;
rhp = mil.run_scenario(hp,'RunSoP',false,'Export',false);
show_segments(rhp.metrics.ekf.segment,vars);

fprintf('\n===== original HPPC -20 pp: dynamic acquisition + deferred fit =====\n');
hm = base;
hm.reference_ekf.initial_soc_offset = -0.20;
rhm = mil.run_scenario(hm,'RunSoP',false,'Export',false);
show_segments(rhm.metrics.ekf.segment,vars);

fprintf('\n===== boot under 10 A, then rest =====\n');
d = base;
d.profile = struct('kind','segments','duration_s',[20 100], ...
    'current_A',[10 0],'ambient_C',[25 25], ...
    'name','c1d_nonzero_current_boot');
d.stop_time_s = 120;
rd = mil.run_scenario(d,'RunSoP',false,'Export',false);
show_segments(rd.metrics.ekf.segment,vars);

fprintf('\n===== +1 A measured current bias: must not false-acquire =====\n');
e = relaxed;
e.sensor.current.bias_50A_A = 1.0;
e.sensor.current.bias_800A_A = 1.0;
re = mil.run_scenario(e,'RunSoP',false,'Export',false);
show_segments(re.metrics.ekf.segment,vars);
if any([re.metrics.ekf.segment.acquisition_completed])
    error('mil:debug:CurrentBiasFalseAcquire', ...
        '+1 A current-bias case falsely completed fixed-basis acquisition.');
end

fprintf('\n===== segment 1 +20 mV/cell through first window; retry at clean rest =====\n');
f = true_relaxed_case(base,45);
faults = repmat(struct('type','cell_bias','start_s',0,'end_s',20.1, ...
    'target',1,'value',0.020),1,15);
for k = 1:15, faults(k).target = k; end
f.sensor.faults = faults;
rf = mil.run_scenario(f,'RunSoP',false,'Export',false);
show_segments(rf.metrics.ekf.segment,vars);

fprintf('\n===== segment 3 PEC invalid first 5 s; delayed clean acquisition =====\n');
g = true_relaxed_case(base,30);
g.sensor.faults = struct('type','segment_pec_invalid','start_s',0, ...
    'end_s',5,'target',3,'value',0);
rg = mil.run_scenario(g,'RunSoP',false,'Export',false);
show_segments(rg.metrics.ekf.segment,vars);

fprintf('\n===== true relaxed 5 C =====\n');
hc = relaxed;
hc.initial_temperature_C = 5;
hc.ambient_temperature_C = 5;
hc.profile.ambient_C(:) = 5;
rhc = mil.run_scenario(hc,'RunSoP',false,'Export',false);
show_segments(rhc.metrics.ekf.segment,vars);

fprintf('\n===== true relaxed 40 C =====\n');
hh = relaxed;
hh.initial_temperature_C = 40;
hh.ambient_temperature_C = 40;
hh.profile.ambient_C(:) = 40;
rhh = mil.run_scenario(hh,'RunSoP',false,'Export',false);
show_segments(rhh.metrics.ekf.segment,vars);

fprintf('\n===== warm discharge +20/-20 pp =====\n');
[wmB,wtB,params,pack_cfg] = warm_stream(base,+60,60,120);
run_warm_pair(wmB,wtB,params,pack_cfg,base,vars);

fprintf('\n===== warm charge +20/-20 pp =====\n');
[wmC,wtC,params,pack_cfg] = warm_stream(base,-10,60,120);
run_warm_pair(wmC,wtC,params,pack_cfg,base,vars);

function cfg = true_relaxed_case(base,rest_s)
%TRUE_RELAXED_CASE Positive-control acquisition stimulus.
% The first 25 s are uninterrupted zero-current rest, long enough for the
% provisional 20 s window. Excitation starts only after acquisition should have
% completed, so failure to anchor near 20 s is an acquisition-policy defect
% rather than a profile/stimulus mismatch.
cfg = base;
cfg.id = 'c1a_true_relaxed_fixed_basis';
cfg.description = sprintf(['C1A: true relaxed startup with %.1f s acquisition ', ...
    'opportunity then directed discharge/charge excitation'],rest_s);
cfg.profile = struct('kind','segments', ...
    'duration_s',[rest_s 10 10 10 10 55], ...
    'current_A',[0 60 0 -10 0 0], ...
    'ambient_C',[25 25 25 25 25 25], ...
    'name','c1a_true_relaxed_then_excitation');
cfg.stop_time_s = sum(cfg.profile.duration_s);
end

function [wm,wt,params,pack_cfg] = warm_stream(base,current_A,drive_s,rest_s)
pre = base;
pre.reference_ekf.enabled = false;
pre.profile = struct('kind','segments','duration_s',[drive_s rest_s], ...
    'current_A',[current_A 0],'ambient_C',[25 25],'name','warm_restart_precondition');
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
cell_cfg = hil.load_configuration('cell',base.cell_id);
[params,~] = hil.build_parameters(cell_cfg,'Save',false);
pack_cfg = F.plant.pack_configuration;
end

function run_warm_pair(wm,wt,params,pack_cfg,base,vars)
for offset = [0.20 -0.20]
    rcfg = base.reference_ekf;
    rcfg.initial_soc = min(max(wt.segment_soc(1,:).' + offset,0.01),0.99);
    rcfg.initial_vp1_V = 0;
    rcfg.initial_vp2_V = 0;
    rcfg.acquisition.enabled = true;
    rcfg.acquisition.hold_measurement_updates = false;
    R = mil.reference.run_segment_ekf(wm,params,pack_cfg,rcfg);
    acfg = base.acceptance.ekf;
    acfg.acquisition_mode = true;
    M = mil.metrics.ekf(R,wt,acfg);
    fprintf('initial offset %+0.2f\n',offset);
    T = struct2table(M.segment);
    disp(T(:,vars));
end
end

function show_segments(segment,vars)
T = struct2table(segment);
disp(T(:,vars));
end
