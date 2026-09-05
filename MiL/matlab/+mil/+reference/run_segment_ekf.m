function result = run_segment_ekf(meas, params, pack_cfg, cfg)
%RUN_SEGMENT_EKF Independent 3-state full-covariance EKF benchmark.
%
% This is deliberately NOT a MATLAB transcription of ams_soc_ekf.c. The
% production firmware uses a diagonal 3-state inner EKF plus scalar outer R0
% loop and adaptive R. This reference uses one full 3x3 covariance matrix,
% exact RC discretization, fixed measurement covariance, a Joseph update, and
% the reviewed R0 LUT evaluated from estimated SoC/temperature. R0 is not a
% fourth state: one segment-voltage scalar cannot independently observe SoC,
% two polarization states, and R0 at every sample. Production R0 adaptation is
% qualified separately through the direct production-C runner.
%
% Directed large-initial-error scenarios may enable a measurement-only fixed-
% basis relaxation acquisition stage. A continuous low-current voltage trace is
% fitted to
%
%   Vcell(t) = Vinf + A1*exp(-t/tau1) + A2*exp(-t/tau2)
%
% using fixed effective acquisition time constants. Vinf is inverted through
% the reviewed P42A OCV surface, while the fitted exponential amplitudes are
% propagated to the END of the acquisition window to initialize Vp1/Vp2. The
% algorithm never consumes hidden MiL scoring state. A robust cross-segment consensus
% prevents one coherent biased segment from independently declaring pack SoC.
%
% Acquisition runs as a SHADOW observer by default. While unresolved, the
% dynamic path propagates Vp1/Vp2 from current/model physics and uses a bounded
% SoC-only voltage correction, explicitly preventing a single scalar residual
% from inventing arbitrary polarization states. A successful fixed-basis fit may
% re-anchor all three states; rejected/interrupted fits remain retryable.
%
% The fixed basis is an acquisition model, not a claim that tau1/tau2 are
% physically identified cell parameters. The current provisional 10 s / 35 s
% basis and 20 s window came from directed MiL screening and require hardware
% correlation before qualification use.

N = numel(meas.time_s);
S = double(pack_cfg.segment_count);
series_per_segment = double(pack_cfg.groups_per_segment(:));
if any(series_per_segment ~= series_per_segment(1))
    error('mil:refekf:Topology','Reference currently expects equal segment sizes.');
end
Ns_seg = series_per_segment(1);
Np = double(pack_cfg.parallel_cells);
Qcell_Ah = double(params.Q_nom);
R2 = double(params.R2);
C2 = double(params.C2);

result = struct();
result.description = ['independent full-covariance 3-state EKF with LUT R0 ', ...
    'and optional fixed-basis relaxation acquisition'];
result.time_s = meas.time_s;
result.state = nan(N,S,3);
result.P = nan(N,S,3,3);
result.r0_ohm = nan(N,S);
result.innovation_V = nan(N,S);
result.innovation_variance_V2 = nan(N,S);
result.NIS = nan(N,S);
result.accepted = false(N,S);
result.valid = false(N,S);
result.core_temp_C = nan(N,S);
result.predicted_voltage_V = nan(N,S);
result.acquisition_hold = false(N,S);
result.acquisition_shadow_updates = false(N,S);
result.acquisition_dynamic_soc_update = false(N,S);
result.acquisition_anchor = false(N,S);
result.acquisition_soc = nan(N,S);
result.acquisition_candidate_soc = nan(N,S);
result.acquisition_ocv_cell_V = nan(N,S);
result.acquisition_vp1_V = nan(N,S);
result.acquisition_vp2_V = nan(N,S);
result.acquisition_fit_rmse_mV_cell = nan(N,S);
result.acquisition_fit_rcond = nan(N,S);
result.acquisition_consensus_soc = nan(N,S);
result.acquisition_window_elapsed_s = zeros(N,S);
result.acquisition_completed = false(1,S);
result.acquisition_time_s = inf(1,S);
result.acquisition_reason = strings(1,S);
result.acquisition_reject_count = zeros(1,S);

vp1_init = expand_per_segment(cfg.initial_vp1_V,S);
vp2_init = expand_per_segment(cfg.initial_vp2_V,S);
initial_soc = expand_per_segment(cfg.initial_soc,S);
qdiag = double(cfg.q_diag(:));
p0diag = double(cfg.p0_diag(:));
if numel(qdiag) ~= 3 || numel(p0diag) ~= 3
    error('mil:refekf:Covariance','q_diag and p0_diag must contain three elements.');
end

acq = default_acquisition();
if isfield(cfg,'acquisition') && isstruct(cfg.acquisition)
    acq = merge_acquisition(acq,cfg.acquisition);
end
validate_acquisition(acq,S);
acquisition_enabled = logical(acq.enabled);

if isfield(meas,'timestamp_s')
    estimator_time = double(meas.timestamp_s(:));
else
    estimator_time = double(meas.time_s(:));
end
start_time = estimator_time(1);

acq_state = init_acquisition_state(S,acquisition_enabled);
if acquisition_enabled
    result.acquisition_reason(:) = "waiting_for_low_current";
else
    result.acquisition_reason(:) = "disabled";
end

x = zeros(3,S);
P = zeros(3,3,S);
core = zeros(1,S);
for s = 1:S
    soc0 = min(max(initial_soc(s),0),1);
    t0 = meas.segment_surface_max_C(1,s);
    if ~isfinite(t0), t0 = 25.0; end
    x(:,s) = [soc0; vp1_init(s); vp2_init(s)];
    P(:,:,s) = diag(p0diag);
    if isfinite(meas.segment_surface_max_C(1,s))
        core(s) = meas.segment_surface_max_C(1,s);
    else
        core(s) = t0;
    end
end

for n = 1:N
    if n == 1
        dt = median(diff(estimator_time));
        if isempty(dt) || ~isfinite(dt) || dt <= 0
            dt = 0.1;
        end
    else
        dt = estimator_time(n) - estimator_time(n-1);
    end
    if ~isfinite(dt) || dt <= 0
        dt = 0.1;
    end
    dt = min(max(dt,1e-3),1.0);

    anchor = empty_anchor(S);
    if acquisition_enabled
        [acq_state,anchor,acq_diag] = acquisition_step( ...
            acq_state,meas,params,cfg,acq,estimator_time,n,Ns_seg,S);
        result.acquisition_candidate_soc(n,:) = acq_diag.candidate_soc;
        result.acquisition_ocv_cell_V(n,:) = acq_diag.ocv_cell_V;
        result.acquisition_vp1_V(n,:) = acq_diag.vp1_finish_V;
        result.acquisition_vp2_V(n,:) = acq_diag.vp2_finish_V;
        result.acquisition_fit_rmse_mV_cell(n,:) = acq_diag.fit_rmse_mV_cell;
        result.acquisition_fit_rcond(n,:) = acq_diag.fit_rcond;
        result.acquisition_consensus_soc(n,:) = acq_diag.consensus_soc;
        result.acquisition_window_elapsed_s(n,:) = acq_diag.window_elapsed_s;
    end

    for s = 1:S
        if ~isfinite(meas.pack_current_A(n))
            result.state(n,s,:) = x(:,s);
            result.P(n,s,:,:) = P(:,:,s);
            result.r0_ohm(n,s) = mil.util.r0(params,x(1,s),core(s));
            result.core_temp_C(n,s) = core(s);
            continue;
        end
        Icell = meas.pack_current_A(n) / Np;
        soc = min(max(x(1,s),0),1);
        t_lut = min(max(core(s),cfg.temperature_lut_bounds_C(1)), ...
            cfg.temperature_lut_bounds_C(2));
        r1 = mil.util.r1(params,soc,t_lut);
        c1 = mil.util.c1(params,soc,t_lut);
        if ~(isfinite(r1) && r1 > 0 && isfinite(c1) && c1 > 0)
            result.state(n,s,:) = x(:,s);
            result.P(n,s,:,:) = P(:,:,s);
            result.r0_ohm(n,s) = mil.util.r0(params,x(1,s),t_lut);
            result.core_temp_C(n,s) = core(s);
            continue;
        end

        a1 = exp(-dt/(r1*c1));
        a2 = exp(-dt/(R2*C2));
        xp = x(:,s);
        xp(1) = min(max(x(1,s) - Icell*dt/(3600*Qcell_Ah),0),1);
        xp(2) = a1*x(2,s) + r1*(1-a1)*Icell;
        xp(3) = a2*x(3,s) + R2*(1-a2)*Icell;
        F = diag([1,a1,a2]);
        Pp = F*P(:,:,s)*F.' + diag(qdiag) * max(dt/0.1,0.1);
        Pp = 0.5*(Pp+Pp.');

        y = meas.segment_voltage_V(n,s);
        current_ok = current_sample_valid(meas,n);
        measurement_available = current_ok && isfinite(y);
        acquisition_pending = acquisition_enabled && ~acq_state.done(s);
        acquisition_hold = acquisition_pending && logical(acq.hold_measurement_updates);
        result.acquisition_shadow_updates(n,s) = acquisition_pending && ~acquisition_hold;

        if anchor(s).apply
            correction = anchor(s).soc - xp(1);
            limit = double(acq.max_soc_correction_per_anchor);
            correction = min(max(correction,-limit),limit);
            xp(1) = min(max(xp(1)+correction,0),1);
            xp(2) = anchor(s).vp1_finish_V;
            xp(3) = anchor(s).vp2_finish_V;

            % Acquisition uncertainty is deliberately conservative and is NOT
            % inferred from the small model-fit residual. The latter was shown
            % by directed MiL to be a poor confidence indicator for short or
            % model-mismatched relaxation records.
            Pp = diag([double(acq.soc_sigma_init)^2, ...
                       double(acq.vp1_sigma_init_V)^2, ...
                       double(acq.vp2_sigma_init_V)^2]);

            result.acquisition_completed(s) = true;
            result.acquisition_time_s(s) = max(0,estimator_time(n)-start_time);
            result.acquisition_reason(s) = "fixed_basis_anchor";
            result.acquisition_anchor(n,s) = true;
            result.acquisition_soc(n,s) = xp(1);
            acquisition_hold = false;
        end

        d_ocv = ocv_slope(params,xp(1),t_lut);
        d_r0 = r0_slope(params,xp(1),t_lut);
        ocv = mil.util.ocv(params,xp(1),t_lut);
        r0 = mil.util.r0(params,xp(1),t_lut);
        yhat = Ns_seg * (ocv - Icell*r0 - xp(2) - xp(3));
        H = [Ns_seg*(d_ocv-Icell*d_r0), -Ns_seg, -Ns_seg];
        R = double(cfg.measurement_R_V2);

        % While acquisition is unresolved, do not let one terminal-voltage
        % scalar manufacture arbitrary Vp1/Vp2 corrections. Treat the two
        % polarization states as nuisance uncertainty, propagate them only
        % through the physical RC/current model, and allow a bounded SoC-only
        % voltage correction. This is an acquisition policy, not the normal
        % full-covariance EKF. A later fixed-basis relaxation fit can replace
        % all three states once a defensible low-current trajectory exists.
        dynamic_pending = acquisition_pending && ...
            logical(acq.dynamic_soc_only_updates) && ~acquisition_hold;
        if dynamic_pending
            soc_floor = double(acq.dynamic_soc_sigma_floor)^2;
            vp_floor = double(acq.dynamic_vp_sigma_floor_V)^2;
            Pp(1,1) = max(Pp(1,1),soc_floor);
            Pp(2,2) = max(Pp(2,2),vp_floor);
            Pp(3,3) = max(Pp(3,3),vp_floor);
            % The acquisition update deliberately does not claim SoC/Vp
            % correlation from a scalar observation it cannot identify.
            Pp(1,2:3) = 0;
            Pp(2:3,1) = 0;
            Rnuis = H(2:3)*Pp(2:3,2:3)*H(2:3).' + R;
            Syy = H(1)^2*Pp(1,1) + Rnuis;
        else
            Rnuis = R;
            Syy = H*Pp*H.' + R;
        end

        accept = false;
        innovation = NaN;
        nis = NaN;
        if measurement_available && isfinite(Syy) && Syy > 1e-12
            innovation = y-yhat;
            nis = innovation^2/Syy;
            accept = ~acquisition_hold && ...
                abs(innovation) <= cfg.innovation_gate_sigma*sqrt(Syy);
        end

        if accept && dynamic_pending
            Ksoc = Pp(1,1)*H(1)/Syy;
            raw_step = Ksoc*innovation;
            max_step = double(acq.dynamic_max_soc_step);
            soc_step = min(max(raw_step,-max_step),max_step);
            x(:,s) = xp;
            x(1,s) = min(max(xp(1)+soc_step,0),1);

            if abs(innovation) > 1e-12
                Keff = soc_step/innovation;
            else
                Keff = 0;
            end
            P(:,:,s) = Pp;
            a_soc = 1-Keff*H(1);
            P(1,1,s) = max(a_soc^2*Pp(1,1) + Keff^2*Rnuis,soc_floor);
            P(1,2:3,s) = 0;
            P(2:3,1,s) = 0;
            P(:,:,s) = 0.5*(P(:,:,s)+P(:,:,s).');
            result.acquisition_dynamic_soc_update(n,s) = true;
        elseif accept
            K = (Pp*H.')/Syy;
            x(:,s) = xp + K*innovation;
            x(1,s) = min(max(x(1,s),0),1);
            A = eye(3)-K*H;
            P(:,:,s) = A*Pp*A.' + K*R*K.';
            P(:,:,s) = 0.5*(P(:,:,s)+P(:,:,s).');
        else
            x(:,s) = xp;
            P(:,:,s) = Pp;
        end

        result.acquisition_hold(n,s) = acquisition_hold;

        surface = meas.segment_surface_max_C(n,s);
        if isfinite(surface)
            inv_r1 = 1/r1;
            r0_thermal = mil.util.r0(params,x(1,s),t_lut);
            qgen = Icell^2*max(r0_thermal,0) + x(2,s)^2*inv_r1 + x(3,s)^2/R2;
            qcs = (core(s)-surface)/double(params.Rcs);
            core(s) = core(s) + dt*(qgen-qcs)/double(params.Cc);
            core(s) = min(max(core(s),-10),60);
        end

        result.state(n,s,:) = x(:,s);
        result.P(n,s,:,:) = P(:,:,s);
        result.r0_ohm(n,s) = mil.util.r0(params,x(1,s),t_lut);
        result.innovation_V(n,s) = innovation;
        result.innovation_variance_V2(n,s) = Syy;
        result.NIS(n,s) = nis;
        result.accepted(n,s) = accept;
        result.valid(n,s) = all(isfinite(x(:,s))) && all(isfinite(P(:,:,s)),'all');
        result.core_temp_C(n,s) = core(s);
        result.predicted_voltage_V(n,s) = yhat;
    end
end

if acquisition_enabled
    result.acquisition_completed = acq_state.done;
    result.acquisition_reason = acq_state.reason;
    result.acquisition_reject_count = acq_state.reject_count;
    % Preserve the explicit anchor reason for completed segments.
    result.acquisition_reason(acq_state.done) = "fixed_basis_anchor";
end

result.soc = result.state(:,:,1);
result.vp1_V = result.state(:,:,2);
result.vp2_V = result.state(:,:,3);
result.acquisition_config = acq;
end

function [state,anchor,diagout] = acquisition_step( ...
    state,meas,params,cfg,acq,time,n,Ns_seg,S)
anchor = empty_anchor(S);
diagout = struct( ...
    'candidate_soc',nan(1,S), ...
    'ocv_cell_V',nan(1,S), ...
    'vp1_finish_V',nan(1,S), ...
    'vp2_finish_V',nan(1,S), ...
    'fit_rmse_mV_cell',nan(1,S), ...
    'fit_rcond',nan(1,S), ...
    'consensus_soc',nan(1,S), ...
    'window_elapsed_s',zeros(1,S));

candidate = repmat(empty_candidate(),1,S);
pack_current_A = double(meas.pack_current_A(n));
current_ok = current_sample_valid(meas,n);

for s = 1:S
    if state.done(s)
        state.reason(s) = "fixed_basis_anchor";
        continue;
    end

    segment_ok = current_ok && isfinite(pack_current_A) && ...
        isfinite(meas.segment_voltage_V(n,s)) && ...
        isfinite(meas.segment_surface_max_C(n,s));

    if ~state.active(s)
        if segment_ok && abs(pack_current_A) <= double(acq.current_enter_A)
            state.active(s) = true;
            state.start_index(s) = n;
            state.reason(s) = "collecting_relaxation";
        else
            state.reason(s) = "waiting_for_low_current";
        end
    elseif ~segment_ok || abs(pack_current_A) > double(acq.current_abort_A)
        state.active(s) = false;
        state.start_index(s) = 0;
        state.reason(s) = "relaxation_interrupted_retry";
        state.reject_count(s) = state.reject_count(s) + 1;
    end

    if ~state.active(s)
        continue;
    end

    i0 = state.start_index(s);
    elapsed = time(n)-time(i0);
    diagout.window_elapsed_s(s) = max(0,elapsed);
    if elapsed + 1e-12 < double(acq.window_s)
        continue;
    end

    [candidate(s),fit_reason] = fixed_basis_candidate( ...
        meas,params,cfg,acq,time,i0,n,s,Ns_seg);

    diagout.candidate_soc(s) = candidate(s).soc;
    diagout.ocv_cell_V(s) = candidate(s).ocv_cell_V;
    diagout.vp1_finish_V(s) = candidate(s).vp1_finish_V;
    diagout.vp2_finish_V(s) = candidate(s).vp2_finish_V;
    diagout.fit_rmse_mV_cell(s) = candidate(s).fit_rmse_mV_cell;
    diagout.fit_rcond(s) = candidate(s).fit_rcond;

    if ~candidate(s).valid
        state.active(s) = false;
        state.start_index(s) = 0;
        state.reason(s) = fit_reason;
        state.reject_count(s) = state.reject_count(s) + 1;
    end
end

ready = [candidate.valid];
accepted_pool = state.accepted_soc(state.done & isfinite(state.accepted_soc));
candidate_pool = [candidate(ready).soc];
pool = [accepted_pool(:); candidate_pool(:)];
if ~isempty(pool)
    consensus = median(pool,'omitnan');
else
    consensus = NaN;
end

if nnz(isfinite(pool)) >= double(acq.min_consensus_segments) && isfinite(consensus)
    for s = 1:S
        if ~ready(s)
            continue;
        end
        diagout.consensus_soc(s) = consensus;
        if abs(candidate(s).soc-consensus) <= double(acq.consensus_soc_max_deviation)
            anchor(s).apply = true;
            anchor(s).soc = candidate(s).soc;
            anchor(s).vp1_finish_V = candidate(s).vp1_finish_V;
            anchor(s).vp2_finish_V = candidate(s).vp2_finish_V;
            anchor(s).ocv_cell_V = candidate(s).ocv_cell_V;
            anchor(s).fit_rmse_mV_cell = candidate(s).fit_rmse_mV_cell;
            anchor(s).fit_rcond = candidate(s).fit_rcond;
            anchor(s).consensus_soc = consensus;
            state.done(s) = true;
            state.active(s) = false;
            state.start_index(s) = 0;
            state.accepted_soc(s) = candidate(s).soc;
            state.reason(s) = "fixed_basis_anchor";
        else
            % A coherent segment bias can produce a mathematically excellent
            % fit with the wrong OCV. Reject the outlier, clear its window, and
            % allow a later clean interval to retry instead of substituting the
            % pack median for the segment state.
            state.active(s) = false;
            state.start_index(s) = 0;
            state.reason(s) = "segment_consensus_reject_retry";
            state.reject_count(s) = state.reject_count(s) + 1;
        end
    end
elseif any(ready)
    for s = find(ready)
        state.active(s) = false;
        state.start_index(s) = 0;
        state.reason(s) = "insufficient_consensus_retry";
        state.reject_count(s) = state.reject_count(s) + 1;
    end
end
end

function [c,reason] = fixed_basis_candidate(meas,params,cfg,acq,time,i0,i1,s,Ns_seg)
c = empty_candidate();
reason = "fixed_basis_fit_reject_retry";
indices = decimated_window_indices(time,i0,i1,double(acq.fit_sample_period_s));
if numel(indices) < double(acq.min_fit_samples)
    reason = "insufficient_fit_samples_retry";
    return;
end

tr = time(indices)-time(indices(1));
y = double(meas.segment_voltage_V(indices,s))/Ns_seg;
temperature = double(meas.segment_surface_max_C(indices,s));
if any(~isfinite(tr)) || any(~isfinite(y)) || all(~isfinite(temperature))
    reason = "invalid_relaxation_data_retry";
    return;
end

tau1 = double(acq.tau1_s);
tau2 = double(acq.tau2_s);
X = [ones(size(tr)),exp(-tr/tau1),exp(-tr/tau2)];
normal_matrix = X.'*X;
rc = rcond(normal_matrix);
if ~isfinite(rc) || rc < double(acq.min_fit_rcond)
    reason = "fit_conditioning_reject_retry";
    c.fit_rcond = rc;
    return;
end

beta = X\y;
yhat = X*beta;
fit_rmse_V = sqrt(mean((y-yhat).^2));
window_elapsed = tr(end);
vp1_finish = -beta(2)*exp(-window_elapsed/tau1);
vp2_finish = -beta(3)*exp(-window_elapsed/tau2);

T = median(temperature(isfinite(temperature)),'omitnan');
T = min(max(T,cfg.temperature_lut_bounds_C(1)),cfg.temperature_lut_bounds_C(2));
[soc_anchor,in_range] = mil.util.soc_from_ocv(params,beta(1),T);

c.soc = soc_anchor;
c.ocv_cell_V = beta(1);
c.vp1_finish_V = vp1_finish;
c.vp2_finish_V = vp2_finish;
c.fit_rmse_mV_cell = 1000*fit_rmse_V;
c.fit_rcond = rc;
c.window_elapsed_s = window_elapsed;

if ~in_range || ~isfinite(soc_anchor)
    reason = "ocv_out_of_range_retry";
    return;
end
if c.fit_rmse_mV_cell > double(acq.max_fit_rmse_mV_cell)
    reason = "fit_residual_reject_retry";
    return;
end
if max(abs([vp1_finish,vp2_finish])) > double(acq.max_abs_vp_state_V) || ...
        abs(vp1_finish+vp2_finish) > double(acq.max_abs_total_polarization_V)
    reason = "polarization_plausibility_reject_retry";
    return;
end

c.valid = true;
reason = "candidate_ready";
end

function indices = decimated_window_indices(time,i0,i1,period_s)
if period_s <= 0
    indices = (i0:i1).';
    return;
end
indices = i0;
last_time = time(i0);
for k = i0+1:i1-1
    if time(k)-last_time >= period_s-1e-9
        indices(end+1,1) = k; %#ok<AGROW>
        last_time = time(k);
    end
end
if indices(end) ~= i1
    indices(end+1,1) = i1;
end
end

function tf = current_sample_valid(meas,n)
tf = isfinite(meas.pack_current_A(n)) && logical(meas.current_valid(n));
if isfield(meas,'current_calibrated')
    tf = tf && logical(meas.current_calibrated(n));
end
if isfield(meas,'current_polarity_validated')
    tf = tf && logical(meas.current_polarity_validated(n));
end
end

function state = init_acquisition_state(S,enabled)
state = struct();
state.active = false(1,S);
state.start_index = zeros(1,S);
state.done = repmat(~enabled,1,S);
state.accepted_soc = nan(1,S);
state.reject_count = zeros(1,S);
state.reason = strings(1,S);
if enabled
    state.reason(:) = "waiting_for_low_current";
else
    state.reason(:) = "disabled";
end
end

function a = empty_anchor(S)
t = struct('apply',false,'soc',NaN,'vp1_finish_V',NaN, ...
    'vp2_finish_V',NaN,'ocv_cell_V',NaN,'fit_rmse_mV_cell',NaN, ...
    'fit_rcond',NaN,'consensus_soc',NaN);
a = repmat(t,1,S);
end

function c = empty_candidate()
c = struct('valid',false,'soc',NaN,'ocv_cell_V',NaN, ...
    'vp1_finish_V',NaN,'vp2_finish_V',NaN,'fit_rmse_mV_cell',NaN, ...
    'fit_rcond',NaN,'window_elapsed_s',NaN);
end

function slope = ocv_slope(params,soc,temperature)
d = 1e-4;
hi = min(soc+d,1);
lo = max(soc-d,0);
slope = (mil.util.ocv(params,hi,temperature) - ...
    mil.util.ocv(params,lo,temperature)) / max(hi-lo,1e-9);
end

function slope = r0_slope(params,soc,temperature)
d = 1e-4;
hi = min(soc+d,1);
lo = max(soc-d,0);
slope = (mil.util.r0(params,hi,temperature) - ...
    mil.util.r0(params,lo,temperature)) / max(hi-lo,1e-9);
end

function acq = default_acquisition()
acq = struct( ...
    'enabled',false, ...
    'hold_measurement_updates',false, ...
    'dynamic_soc_only_updates',true, ...
    'dynamic_soc_sigma_floor',0.030, ...
    'dynamic_vp_sigma_floor_V',0.020, ...
    'dynamic_max_soc_step',0.030, ...
    'current_enter_A',0.50, ...
    'current_abort_A',1.00, ...
    'window_s',20.0, ...
    'fit_sample_period_s',1.0, ...
    'min_fit_samples',15, ...
    'tau1_s',10.0, ...
    'tau2_s',35.0, ...
    'min_fit_rcond',1.0e-5, ...
    'max_fit_rmse_mV_cell',5.0, ...
    'max_abs_vp_state_V',0.15, ...
    'max_abs_total_polarization_V',0.20, ...
    'consensus_soc_max_deviation',0.015, ...
    'min_consensus_segments',3, ...
    'max_soc_correction_per_anchor',0.30, ...
    'soc_sigma_init',0.030, ...
    'vp1_sigma_init_V',0.020, ...
    'vp2_sigma_init_V',0.020);
end

function out = merge_acquisition(base,override)
out = base;
names = fieldnames(override);
for k = 1:numel(names)
    out.(names{k}) = override.(names{k});
end
end

function validate_acquisition(acq,S)
if ~(isscalar(acq.enabled) && (islogical(acq.enabled) || isnumeric(acq.enabled)))
    error('mil:refekf:AcquisitionEnabled','acquisition.enabled must be scalar logical/numeric.');
end
if ~(isscalar(acq.hold_measurement_updates) && ...
        (islogical(acq.hold_measurement_updates) || isnumeric(acq.hold_measurement_updates)))
    error('mil:refekf:AcquisitionHold','acquisition.hold_measurement_updates must be scalar logical/numeric.');
end
if ~(isscalar(acq.dynamic_soc_only_updates) && ...
        (islogical(acq.dynamic_soc_only_updates) || isnumeric(acq.dynamic_soc_only_updates)))
    error('mil:refekf:AcquisitionDynamic','acquisition.dynamic_soc_only_updates must be scalar logical/numeric.');
end
numeric = [acq.dynamic_soc_sigma_floor,acq.dynamic_vp_sigma_floor_V, ...
    acq.dynamic_max_soc_step,acq.current_enter_A,acq.current_abort_A,acq.window_s, ...
    acq.fit_sample_period_s,acq.min_fit_samples,acq.tau1_s,acq.tau2_s, ...
    acq.min_fit_rcond,acq.max_fit_rmse_mV_cell,acq.max_abs_vp_state_V, ...
    acq.max_abs_total_polarization_V,acq.consensus_soc_max_deviation, ...
    acq.min_consensus_segments,acq.max_soc_correction_per_anchor, ...
    acq.soc_sigma_init,acq.vp1_sigma_init_V,acq.vp2_sigma_init_V];
if any(~isfinite(numeric)) || acq.dynamic_soc_sigma_floor <= 0 || ...
        acq.dynamic_vp_sigma_floor_V <= 0 || acq.dynamic_max_soc_step <= 0 || ...
        acq.dynamic_max_soc_step > 0.25 || acq.current_enter_A < 0 || ...
        acq.current_abort_A < acq.current_enter_A || acq.window_s <= 0 || ...
        acq.fit_sample_period_s <= 0 || acq.min_fit_samples < 3 || ...
        acq.tau1_s <= 0 || acq.tau2_s <= acq.tau1_s || ...
        acq.min_fit_rcond <= 0 || acq.max_fit_rmse_mV_cell <= 0 || ...
        acq.max_abs_vp_state_V <= 0 || acq.max_abs_total_polarization_V <= 0 || ...
        acq.consensus_soc_max_deviation <= 0 || acq.min_consensus_segments < 1 || ...
        acq.min_consensus_segments > S || acq.max_soc_correction_per_anchor <= 0 || ...
        acq.soc_sigma_init <= 0 || acq.vp1_sigma_init_V <= 0 || ...
        acq.vp2_sigma_init_V <= 0
    error('mil:refekf:AcquisitionConfig','Invalid fixed-basis acquisition configuration.');
end
end

function v = expand_per_segment(value,count)
v = double(value(:));
if isscalar(v)
    v = repmat(v,count,1);
elseif numel(v) ~= count
    error('mil:refekf:Config','Expected scalar or one value per segment.');
end
end
