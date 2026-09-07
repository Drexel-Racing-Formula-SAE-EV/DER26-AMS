function result = sop_snapshot(truth, index, params, pack_cfg, cfg)
%SOP_SNAPSHOT Distributed 75-group truth-based safe-current oracle.
%
% The oracle starts from hidden physical plant states and propagates every
% series group independently. It does not consume estimator states/covariance,
% does not linearize OCV, and does not add the production SoP uncertainty
% margins. Exact RC ZOH updates and a Tustin thermal step differ deliberately
% from both the reference plant Euler integrator and ams_sop.c.

if index < 1 || index > numel(truth.time_s)
    error('mil:soporacle:Index','Checkpoint index out of range.');
end
H = numel(cfg.horizons_s);
result = struct();
result.time_s = truth.time_s(index);
result.index = index;
result.horizons_s = double(cfg.horizons_s(:)).';
result.discharge_current_A = zeros(1,H);
result.charge_current_A = zeros(1,H);
result.discharge = repmat(empty_eval(),1,H);
result.charge = repmat(empty_eval(),1,H);

state0 = snapshot_state(truth,index);
for h = 1:H
    horizon = cfg.horizons_s(h);
    [result.discharge_current_A(h),result.discharge(h)] = solve_direction( ...
        state0,params,pack_cfg,cfg,horizon,cfg.discharge_current_caps_A(h),true);
    [mag,result.charge(h)] = solve_direction( ...
        state0,params,pack_cfg,cfg,horizon,cfg.charge_current_caps_A(h),false);
    result.charge_current_A(h) = -mag;
end

% Longer horizon capability should never exceed a shorter horizon solely due
% to the oracle. Current-path limits already enforce most of this, but retain
% the raw values for diagnostics rather than silently correcting them.
result.discharge_monotonic = all(diff(result.discharge_current_A) <= 1e-6);
result.charge_magnitude_monotonic = all(diff(abs(result.charge_current_A)) <= 1e-6);
end

function state = snapshot_state(truth,index)
state = struct();
state.soc = truth.group_soc(index,:).';
state.vp1 = truth.group_vp1_V(index,:).';
state.vp2 = truth.group_vp2_V(index,:).';
state.core = truth.group_core_C(index,:).';
state.surface = truth.group_surface_C(index,:).';
state.capacity_multiplier = truth.group_capacity_multiplier(:);
state.r0_multiplier = truth.group_r0_multiplier(:);
state.r1_multiplier = truth.group_r1_multiplier(:);
state.c1_multiplier = truth.group_c1_multiplier(:);
state.r2_multiplier = truth.group_r2_multiplier(:);
state.c2_multiplier = truth.group_c2_multiplier(:);
state.rsa_multiplier = truth.group_rsa_multiplier(:);
state.ambient_C = truth.ambient_C(index);
end

function [limit,limiting] = solve_direction(state0,params,pack_cfg,cfg,horizon,cap,discharge)
low = 0;
high = double(cap);
zero = evaluate_candidate(state0,params,pack_cfg,cfg,0,horizon,discharge);
if ~zero.feasible
    limit = 0;
    limiting = zero;
    return;
end
at_cap = evaluate_candidate(state0,params,pack_cfg,cfg, ...
    direction_sign(discharge)*high,horizon,discharge);
if at_cap.feasible
    limit = high;
    limiting = at_cap;
    limiting.binding = 'current_path';
    return;
end
limiting = at_cap;
for k = 1:double(cfg.bisection_iterations)
    mid = 0.5*(low+high);
    e = evaluate_candidate(state0,params,pack_cfg,cfg, ...
        direction_sign(discharge)*mid,horizon,discharge);
    if e.feasible
        low = mid;
    else
        high = mid;
        limiting = e;
    end
end
limit = low;
end

function s = direction_sign(discharge)
if discharge, s=1; else, s=-1; end
end

function eval = evaluate_candidate(state0,params,pack_cfg,cfg,pack_current_A,horizon,discharge)
state = state0;
Np = double(pack_cfg.parallel_cells);
Icell = pack_current_A/Np;
dt_nom = min(double(cfg.integration_step_s),max(horizon/5,1e-4));
eval = empty_eval();
eval.feasible = true;
eval.pack_current_A = pack_current_A;
eval.minimum_cell_voltage_V = Inf;
eval.maximum_cell_voltage_V = -Inf;
eval.minimum_soc = Inf;
eval.maximum_soc = -Inf;
eval.maximum_core_C = -Inf;
eval.maximum_surface_C = -Inf;
elapsed = 0;
while elapsed < horizon-1e-12
    [ok,eval] = check_state(state,params,cfg,Icell,discharge,eval);
    if ~ok, return; end
    dt = min(dt_nom,horizon-elapsed);
    soc_lut = min(max(state.soc,min(params.soc_common)),max(params.soc_common));
    temp_lut = min(max(state.core,min(params.temp_bp)),max(params.temp_bp));
    r0 = mil.util.r0(params,soc_lut,temp_lut).*state.r0_multiplier;
    r1 = mil.util.r1(params,soc_lut,temp_lut).*state.r1_multiplier;
    c1 = mil.util.c1(params,soc_lut,temp_lut).*state.c1_multiplier;
    r2 = double(params.R2).*state.r2_multiplier;
    c2 = double(params.C2).*state.c2_multiplier;
    a1 = exp(-dt./(r1.*c1));
    a2 = exp(-dt./(r2.*c2));
    state.vp1 = a1.*state.vp1 + r1.*(1-a1).*Icell;
    state.vp2 = a2.*state.vp2 + r2.*(1-a2).*Icell;
    state.soc = state.soc - Icell*dt ./ ...
        (3600*double(params.Q_nom).*state.capacity_multiplier);

    heat = Icell^2.*r0 + (state.vp1.^2)./r1 + (state.vp2.^2)./r2;
    Rsa = double(params.Rsa)*double(pack_cfg.cooling_boundary.Rsa_multiplier).* ...
        state.rsa_multiplier;
    [state.core,state.surface] = mil.util.thermal_step_tustin( ...
        state.core,state.surface,heat,state.ambient_C,dt, ...
        double(params.Cc),double(params.Cs),double(params.Rcs),Rsa);
    elapsed = elapsed+dt;
    eval.steps = eval.steps+1;
end
[~,eval] = check_state(state,params,cfg,Icell,discharge,eval);
end

function [ok,eval] = check_state(state,params,cfg,Icell,discharge,eval)
soc_lut = min(max(state.soc,min(params.soc_ocv_common)),max(params.soc_ocv_common));
temp_lut = min(max(state.core,min(params.temp_bp_ocv)),max(params.temp_bp_ocv));
r0 = mil.util.r0(params,state.soc,state.core).*state.r0_multiplier;
ocv = mil.util.ocv(params,soc_lut,temp_lut);
voltage = ocv-Icell.*r0-state.vp1-state.vp2;
eval.minimum_cell_voltage_V = min(eval.minimum_cell_voltage_V,min(voltage));
eval.maximum_cell_voltage_V = max(eval.maximum_cell_voltage_V,max(voltage));
eval.minimum_soc = min(eval.minimum_soc,min(state.soc));
eval.maximum_soc = max(eval.maximum_soc,max(state.soc));
eval.maximum_core_C = max(eval.maximum_core_C,max(state.core));
eval.maximum_surface_C = max(eval.maximum_surface_C,max(state.surface));
eval.minimum_surface_C = min(eval.minimum_surface_C,min(state.surface));
eval.pack_voltage_V = sum(voltage);

if any(~isfinite(voltage)) || any(~isfinite(state.soc)) || ...
        any(~isfinite(state.core)) || any(~isfinite(state.surface))
    [ok,eval] = fail(eval,'numeric',first_bad(~isfinite(voltage)|~isfinite(state.soc)));
    return;
end
if discharge
    idx = find(voltage < cfg.cell_uv_V,1);
    if ~isempty(idx), [ok,eval]=fail(eval,'cell_uv',idx); return; end
    idx = find(state.soc < cfg.soc_min,1);
    if ~isempty(idx), [ok,eval]=fail(eval,'soc_low',idx); return; end
    idx = find(state.core > cfg.discharge_temp_max_C | state.surface > cfg.discharge_temp_max_C,1);
    if ~isempty(idx), [ok,eval]=fail(eval,'discharge_temperature',idx); return; end
else
    idx = find(voltage > cfg.cell_ov_V,1);
    if ~isempty(idx), [ok,eval]=fail(eval,'cell_ov',idx); return; end
    idx = find(state.soc > cfg.soc_max,1);
    if ~isempty(idx), [ok,eval]=fail(eval,'soc_high',idx); return; end
    idx = find(state.core > cfg.charge_temp_max_C | state.surface > cfg.charge_temp_max_C,1);
    if ~isempty(idx), [ok,eval]=fail(eval,'charge_temperature_high',idx); return; end
    idx = find(state.surface < cfg.charge_temp_min_C,1);
    if ~isempty(idx), [ok,eval]=fail(eval,'charge_temperature_low',idx); return; end
end
ok = true;
end

function [ok,eval] = fail(eval,binding,index)
ok = false;
eval.feasible = false;
eval.binding = binding;
eval.limiting_group = index;
end

function idx = first_bad(mask)
idx=find(mask,1); if isempty(idx),idx=0;end
end

function e = empty_eval()
e = struct('feasible',false,'binding','none','limiting_group',0, ...
    'pack_current_A',0,'minimum_cell_voltage_V',Inf, ...
    'maximum_cell_voltage_V',-Inf,'minimum_soc',Inf,'maximum_soc',-Inf, ...
    'maximum_core_C',-Inf,'maximum_surface_C',-Inf,'minimum_surface_C',Inf, ...
    'pack_voltage_V',NaN,'steps',0);
end
