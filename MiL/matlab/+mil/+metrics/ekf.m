function report = ekf(ref, truth, acceptance)
%EKF Score five-segment estimator accuracy and statistical consistency.
S = size(ref.soc,2);
report = struct();
report.NEES = nan(size(ref.soc));
segment_template = struct( ...
    'pass',false, ...
    'reason','', ...
    'samples',0, ...
    'soc_rmse',NaN, ...
    'soc_mae',NaN, ...
    'soc_p95_abs',NaN, ...
    'soc_worst_abs',NaN, ...
    'bias',NaN, ...
    'reject_fraction',1, ...
    'convergence_time_s',Inf, ...
    'convergence_required',true, ...
    'convergence_pass',false, ...
    'post_convergence_samples',0, ...
    'post_convergence_soc_rmse',NaN, ...
    'post_convergence_soc_p95_abs',NaN, ...
    'post_convergence_soc_worst_abs',NaN, ...
    'post_convergence_reject_fraction',1, ...
    'accuracy_window','full_run', ...
    'acquisition_completed',false, ...
    'acquisition_time_s',Inf, ...
    'acquisition_reason','', ...
    'acquisition_anchor_soc',NaN, ...
    'acquisition_ocv_cell_V',NaN, ...
    'acquisition_vp1_V',NaN, ...
    'acquisition_vp2_V',NaN, ...
    'acquisition_fit_rmse_mV_cell',NaN, ...
    'acquisition_fit_rcond',NaN, ...
    'acquisition_consensus_soc',NaN, ...
    'acquisition_reject_count',0, ...
    'acquisition_dynamic_update_fraction',0, ...
    'post_acquisition_soc_error_sigma_p95',NaN, ...
    'post_acquisition_soc_error_sigma_max',NaN, ...
    'post_acquisition_nis_mean',NaN, ...
    'post_acquisition_nees_mean',NaN, ...
    'nis_mean',NaN, ...
    'nis_mean_bounds',[NaN NaN], ...
    'nis_mean_pass',false, ...
    'nees_mean',NaN, ...
    'nees_mean_bounds',[NaN NaN], ...
    'nees_mean_pass',false, ...
    'consistency_diagnostic_pass',false);
report.segment = repmat(segment_template,1,S);
all_pass = true;
for s = 1:S
    valid = ref.valid(:,s) & isfinite(ref.soc(:,s));
    seg = segment_template;
    if ~any(valid)
        seg.reason = 'no valid samples';
        report.segment(s) = seg;
        all_pass = false;
        continue;
    end
    soc_error = ref.soc(valid,s)-truth.segment_soc(valid,s);
    abs_error = abs(soc_error);
    seg.samples = nnz(valid);
    seg.soc_rmse = sqrt(mean(soc_error.^2));
    seg.soc_mae = mean(abs_error);
    seg.soc_p95_abs = percentile(abs_error,95);
    seg.soc_worst_abs = max(abs_error);
    seg.bias = mean(soc_error);
    seg.reject_fraction = 1 - nnz(ref.accepted(valid,s))/nnz(valid);
    anchor_index = NaN;
    if isfield(ref,'acquisition_dynamic_soc_update')
        seg.acquisition_dynamic_update_fraction = ...
            nnz(ref.acquisition_dynamic_soc_update(valid,s))/nnz(valid);
    end
    if isfield(ref,'acquisition_completed')
        seg.acquisition_completed = logical(ref.acquisition_completed(s));
        seg.acquisition_time_s = double(ref.acquisition_time_s(s));
        seg.acquisition_reason = char(ref.acquisition_reason(s));
        if isfield(ref,'acquisition_anchor') && any(ref.acquisition_anchor(:,s))
            anchor_index = find(ref.acquisition_anchor(:,s),1,'first');
            seg.acquisition_anchor_soc = ref.acquisition_soc(anchor_index,s);
            if isfield(ref,'acquisition_ocv_cell_V')
                seg.acquisition_ocv_cell_V = ref.acquisition_ocv_cell_V(anchor_index,s);
            end
            if isfield(ref,'acquisition_vp1_V')
                seg.acquisition_vp1_V = ref.acquisition_vp1_V(anchor_index,s);
            end
            if isfield(ref,'acquisition_vp2_V')
                seg.acquisition_vp2_V = ref.acquisition_vp2_V(anchor_index,s);
            end
            if isfield(ref,'acquisition_fit_rmse_mV_cell')
                seg.acquisition_fit_rmse_mV_cell = ref.acquisition_fit_rmse_mV_cell(anchor_index,s);
            end
            if isfield(ref,'acquisition_fit_rcond')
                seg.acquisition_fit_rcond = ref.acquisition_fit_rcond(anchor_index,s);
            end
            if isfield(ref,'acquisition_consensus_soc')
                seg.acquisition_consensus_soc = ref.acquisition_consensus_soc(anchor_index,s);
            end
        end
        if isfield(ref,'acquisition_reject_count')
            seg.acquisition_reject_count = double(ref.acquisition_reject_count(s));
        end
    end
    [seg.convergence_time_s,settled_index] = settling_time(ref.time_s(valid),abs_error, ...
        acceptance.convergence_band,acceptance.convergence_hold_s);
    accepted_valid=ref.accepted(valid,s);
    if isfinite(seg.convergence_time_s)
        post_error=soc_error(settled_index:end);
        post_abs=abs_error(settled_index:end);
        seg.post_convergence_samples=numel(post_error);
        seg.post_convergence_soc_rmse=sqrt(mean(post_error.^2));
        seg.post_convergence_soc_p95_abs=percentile(post_abs,95);
        seg.post_convergence_soc_worst_abs=max(post_abs);
        seg.post_convergence_reject_fraction=1- ...
            nnz(accepted_valid(settled_index:end))/numel(post_error);
    end

    nis = ref.NIS(valid,s);
    nis = nis(isfinite(nis));
    if isempty(nis)
        seg.nis_mean = NaN;
        seg.nis_mean_bounds = [NaN NaN];
        seg.nis_mean_pass = false;
    else
        seg.nis_mean = mean(nis);
        alpha = acceptance.nis_mean_alpha;
        n = numel(nis);
        seg.nis_mean_bounds = [ ...
            mil.util.chi2inv_local(alpha/2,n)/n, ...
            mil.util.chi2inv_local(1-alpha/2,n)/n];
        seg.nis_mean_pass = seg.nis_mean >= seg.nis_mean_bounds(1) && ...
            seg.nis_mean <= seg.nis_mean_bounds(2);
    end

    indices=find(valid);
    nees_all=nan(numel(indices),1);
    for q=1:numel(indices)
        n=indices(q);
        P=squeeze(ref.P(n,s,:,:));
        xt=[truth.segment_soc(n,s);truth.segment_vp1_V(n,s); ...
            truth.segment_vp2_V(n,s)];
        xe=squeeze(ref.state(n,s,:));
        if all(isfinite(P),'all') && all(isfinite(xe)) && rcond(P)>1e-12
            e=xe-xt; nees_all(q)=e.'*(P\e);
        end
    end
    report.NEES(indices,s)=nees_all;
    nees = nees_all(isfinite(nees_all));
    if isempty(nees)
        seg.nees_mean = NaN;
        seg.nees_mean_bounds = [NaN NaN];
        seg.nees_mean_pass = false;
    else
        seg.nees_mean = mean(nees);
        alpha = acceptance.nees_mean_alpha;
        n = numel(nees);
        dof = 3*n;
        seg.nees_mean_bounds = [ ...
            mil.util.chi2inv_local(alpha/2,dof)/n, ...
            mil.util.chi2inv_local(1-alpha/2,dof)/n];
        seg.nees_mean_pass = seg.nees_mean >= seg.nees_mean_bounds(1) && ...
            seg.nees_mean <= seg.nees_mean_bounds(2);
    end

    % Acquisition scenarios can be intentionally very wrong before the anchor,
    % so full-run NIS/NEES can be dominated by the unacquired interval. Report
    % post-anchor consistency separately instead of hiding that behavior or
    % pretending the pre-anchor covariance was trustworthy.
    if isfinite(anchor_index)
        post_acq = false(size(valid));
        post_acq(anchor_index:end) = true;
        post_acq = post_acq & valid;

        nis_post = ref.NIS(post_acq,s);
        nis_post = nis_post(isfinite(nis_post));
        if ~isempty(nis_post)
            seg.post_acquisition_nis_mean = mean(nis_post);
        end

        nees_post = report.NEES(post_acq,s);
        nees_post = nees_post(isfinite(nees_post));
        if ~isempty(nees_post)
            seg.post_acquisition_nees_mean = mean(nees_post);
        end

        p_soc = squeeze(ref.P(:,s,1,1));
        all_soc_error = ref.soc(:,s)-truth.segment_soc(:,s);
        z_soc = abs(all_soc_error(post_acq)) ./ sqrt(p_soc(post_acq));
        z_soc = z_soc(isfinite(z_soc));
        if ~isempty(z_soc)
            seg.post_acquisition_soc_error_sigma_p95 = percentile(z_soc,95);
            seg.post_acquisition_soc_error_sigma_max = max(z_soc);
        end
    end

    acquisition_mode=isfield(acceptance,'acquisition_mode') && ...
        logical(acceptance.acquisition_mode);
    if acquisition_mode
        seg.accuracy_window='post_convergence';
        accuracy_pass = seg.post_convergence_soc_rmse <= acceptance.soc_rmse_max && ...
            seg.post_convergence_soc_p95_abs <= acceptance.soc_p95_abs_max && ...
            seg.post_convergence_soc_worst_abs <= acceptance.soc_worst_abs_max && ...
            seg.post_convergence_reject_fraction <= acceptance.max_reject_fraction;
    else
        accuracy_pass = seg.soc_rmse <= acceptance.soc_rmse_max && ...
            seg.soc_p95_abs <= acceptance.soc_p95_abs_max && ...
            seg.soc_worst_abs <= acceptance.soc_worst_abs_max && ...
            seg.reject_fraction <= acceptance.max_reject_fraction;
    end
    convergence_required = true;
    if isfield(acceptance,'convergence_required')
        convergence_required = logical(acceptance.convergence_required);
    end
    seg.convergence_required = convergence_required;
    seg.convergence_pass = (~convergence_required) || ...
        (isfinite(seg.convergence_time_s) && ...
         seg.convergence_time_s <= acceptance.convergence_time_max_s);
    convergence_pass = seg.convergence_pass;
    acquisition_required = acquisition_mode && isfield(ref,'acquisition_config') && ...
        logical(ref.acquisition_config.enabled);
    acquisition_pass = ~acquisition_required || seg.acquisition_completed;
    % NIS/NEES are reported as consistency diagnostics. They are not promoted
    % to hard release gates until measurement/process noise distributions are
    % hardware-characterized and frozen.
    seg.pass = accuracy_pass && convergence_pass && acquisition_pass;
    seg.consistency_diagnostic_pass = seg.nis_mean_pass && seg.nees_mean_pass;
    report.segment(s) = seg;
    all_pass = all_pass && seg.pass;
end
report.pass = all_pass;
report.note = ['NIS/NEES are diagnostic until sensor-noise/process-noise ', ...
    'assumptions are characterized; accuracy/convergence are current gates.'];
end

function value = percentile(x,p)
x = sort(x(:));
if isempty(x), value=NaN; return; end
q = 1 + (numel(x)-1)*p/100;
lo = floor(q); hi = ceil(q);
if lo == hi
    value = x(lo);
else
    value = x(lo)+(q-lo)*(x(hi)-x(lo));
end
end

function [tconv,index] = settling_time(t,error,band,hold_s)
tconv = Inf;
index = NaN;
if isempty(t), return; end
last_outside=find(error>band,1,'last');
if isempty(last_outside),candidate=1;else,candidate=last_outside+1;end
if candidate<=numel(t) && (t(end)-t(candidate))>=hold_s && ...
        all(error(candidate:end)<=band)
    index=candidate;
    tconv=t(candidate)-t(1);
end
end
