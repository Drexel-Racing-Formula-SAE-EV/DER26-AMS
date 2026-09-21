function report = production_ekf(prod,truth,acceptance)
%PRODUCTION_EKF Accuracy/settling score for the production-C EKF.
%
% convergence_time_s is retained for API compatibility. It is the timestamp
% immediately after the final excursion outside convergence_band, provided the
% remaining run stays in-band for convergence_hold_s. Directed startup
% convergence is therefore gated only where convergence_required is true.
%
% Production exposes the full symmetric covariance for [SoC,Vp1,Vp2].
% soc_error_sigma remains a useful component-wise diagnostic; state_nees_* is
% the actual three-state NEES, computed only where the production covariance
% is finite and positive definite. Production NIS uses the exact prior scalar
% innovation variance exported by the C estimator.
S=size(prod.soc,2);report=struct();
segment_template=struct( ...
    'pass',false, ...
    'reason','', ...
    'samples',0, ...
    'soc_rmse',NaN, ...
    'soc_mae',NaN, ...
    'soc_p95_abs',NaN, ...
    'soc_worst_abs',NaN, ...
    'bias',NaN, ...
    'measurements_used',0, ...
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
    'soc_error_sigma_max',NaN, ...
    'production_nis_samples',0, ...
    'production_nis_mean',NaN, ...
    'production_nis_p95',NaN, ...
    'production_nis_max',NaN, ...
    'state_nees_samples',0, ...
    'state_nees_invalid_count',0, ...
    'state_nees_mean',NaN, ...
    'state_nees_p95',NaN, ...
    'state_nees_max',NaN, ...
    'covariance_repair_count_max',0, ...
    'acquisition_completed',false, ...
    'acquisition_time_s',Inf, ...
    'acquisition_reason_code',uint8(0), ...
    'acquisition_reason','not_completed', ...
    'acquisition_anchor_soc',NaN, ...
    'acquisition_vp1_V',NaN, ...
    'acquisition_vp2_V',NaN, ...
    'acquisition_fit_rmse_mV_cell',NaN, ...
    'acquisition_fit_rcond',NaN, ...
    'acquisition_consensus_soc',NaN, ...
    'acquisition_reject_count',0, ...
    'acquisition_dynamic_update_fraction',NaN, ...
    'post_acquisition_samples',0, ...
    'post_acquisition_soc_rmse',NaN, ...
    'post_acquisition_soc_worst_abs',NaN, ...
    'post_acquisition_soc_error_sigma_max',NaN, ...
    'post_acquisition_nis_mean',NaN, ...
    'post_acquisition_nis_p95',NaN, ...
    'post_acquisition_state_nees_mean',NaN, ...
    'post_acquisition_state_nees_p95',NaN, ...
    'post_acquisition_state_nees_max',NaN, ...
    'r0_observation_count',0, ...
    'r0_observed',false, ...
    'r0_relative_error_p95',NaN, ...
    'r0_accuracy_required',false, ...
    'r0_accuracy_pass',false, ...
    'r0_accuracy_effective_pass',true, ...
    'r0_unobservable_step_drift_max_ohm',NaN, ...
    'r0_unobservable_drift_pass',false, ...
    'r0_unobservable_drift_required',false, ...
    'r0_unobservable_drift_effective_pass',true);
report.segment=repmat(segment_template,1,S);all_pass=true;

acquisition_mode=isfield(acceptance,'acquisition_mode') && ...
    logical(acceptance.acquisition_mode);
if isfield(acceptance,'acquisition_expected')
    acquisition_expected=logical(acceptance.acquisition_expected);
else
    acquisition_expected=acquisition_mode;
end

for s=1:S
    valid=prod.valid(:,s)&isfinite(prod.soc(:,s));
    seg=segment_template;
    if ~any(valid)
        seg.reason='no valid production samples';
        report.segment(s)=seg;
        all_pass=false;continue;
    end

    e_all=prod.soc(:,s)-truth.segment_soc(:,s);
    e=e_all(valid);ae=abs(e);
    seg.samples=nnz(valid);seg.soc_rmse=sqrt(mean(e.^2));
    seg.soc_mae=mean(ae);seg.soc_p95_abs=percentile(ae,95);seg.soc_worst_abs=max(ae);
    seg.bias=mean(e);

    p_soc=double(prod.P_diag(:,s,1));
    sigma=sqrt(max(p_soc,0));
    sigma_valid=valid & isfinite(sigma) & sigma>0;
    if any(sigma_valid)
        seg.soc_error_sigma_max=max(abs(e_all(sigma_valid))./sigma(sigma_valid));
    end

    if isfield(prod,'innovation_variance_V2')
        nis_mask=prod.measurement_used(:,s) & ...
            isfinite(prod.innovation_V(:,s)) & ...
            isfinite(prod.innovation_variance_V2(:,s)) & ...
            prod.innovation_variance_V2(:,s)>0;
        nis_values=(prod.innovation_V(nis_mask,s).^2)./...
            prod.innovation_variance_V2(nis_mask,s);
        if ~isempty(nis_values)
            seg.production_nis_samples=numel(nis_values);
            seg.production_nis_mean=mean(nis_values);
            seg.production_nis_p95=percentile(nis_values,95);
            seg.production_nis_max=max(nis_values);
        end
    else
        nis_mask=false(size(valid));
    end

    [nees_values,nees_valid_mask,nees_invalid_count]=...
        production_state_nees(prod,truth,s,valid);
    seg.state_nees_samples=numel(nees_values);
    seg.state_nees_invalid_count=nees_invalid_count;
    if ~isempty(nees_values)
        seg.state_nees_mean=mean(nees_values);
        seg.state_nees_p95=percentile(nees_values,95);
        seg.state_nees_max=max(nees_values);
    end
    if isfield(prod,'covariance_repair_count')
        seg.covariance_repair_count_max=double(max(prod.covariance_repair_count(:,s)));
    end

    used=prod.measurement_used(:,s);accepted=prod.accepted(:,s);
    seg.measurements_used=nnz(used);
    if seg.measurements_used>0
        seg.reject_fraction=1-nnz(accepted&used)/seg.measurements_used;
    else
        seg.reject_fraction=1;
    end

    [seg.convergence_time_s,settled_index]=settling_time( ...
        prod.time_s(valid),ae,acceptance.convergence_band,acceptance.convergence_hold_s);
    used_valid=used(valid);accepted_valid=accepted(valid);
    if isfinite(seg.convergence_time_s)
        post_e=e(settled_index:end);post_ae=ae(settled_index:end);
        seg.post_convergence_samples=numel(post_e);
        seg.post_convergence_soc_rmse=sqrt(mean(post_e.^2));
        seg.post_convergence_soc_p95_abs=percentile(post_ae,95);
        seg.post_convergence_soc_worst_abs=max(post_ae);
        post_used=used_valid(settled_index:end);
        if any(post_used)
            seg.post_convergence_reject_fraction=1- ...
                nnz(accepted_valid(settled_index:end)&post_used)/nnz(post_used);
        end
    end

    % Fixed-basis acquisition telemetry comes directly from production C.
    if isfield(prod,'acquisition_state')
        completed=(prod.acquisition_state(:,s)==uint8(2)) & ...
            (prod.acquisition_anchor_count(:,s)>0);
        ka=find(completed,1,'first');
        if ~isempty(ka)
            seg.acquisition_completed=true;
            seg.acquisition_time_s=prod.time_s(ka)-prod.time_s(1);
            seg.acquisition_reason_code=prod.acquisition_reason(ka,s);
            seg.acquisition_reason=acquisition_reason_string(seg.acquisition_reason_code);
            seg.acquisition_anchor_soc=prod.acquisition_candidate_soc(ka,s);
            seg.acquisition_vp1_V=prod.acquisition_vp1_finish_V(ka,s);
            seg.acquisition_vp2_V=prod.acquisition_vp2_finish_V(ka,s);
            seg.acquisition_fit_rmse_mV_cell=prod.acquisition_fit_rmse_mV_cell(ka,s);
            seg.acquisition_fit_rcond=prod.acquisition_fit_rcond(ka,s);
            seg.acquisition_consensus_soc=prod.acquisition_consensus_soc(ka,s);

            post_mask=false(size(valid));post_mask(ka:end)=true;
            post_mask=post_mask&valid;
            if any(post_mask)
                post_acq_e=e_all(post_mask);
                seg.post_acquisition_samples=numel(post_acq_e);
                seg.post_acquisition_soc_rmse=sqrt(mean(post_acq_e.^2));
                seg.post_acquisition_soc_worst_abs=max(abs(post_acq_e));
                sigma_post=post_mask & isfinite(sigma) & sigma>0;
                if any(sigma_post)
                    seg.post_acquisition_soc_error_sigma_max=max( ...
                        abs(e_all(sigma_post))./sigma(sigma_post));
                end
                if exist('nis_mask','var')
                    post_nis=nis_mask & post_mask;
                    if any(post_nis)
                        post_nis_values=(prod.innovation_V(post_nis,s).^2)./...
                            prod.innovation_variance_V2(post_nis,s);
                        seg.post_acquisition_nis_mean=mean(post_nis_values);
                        seg.post_acquisition_nis_p95=percentile(post_nis_values,95);
                    end
                end
                post_nees_mask=nees_valid_mask & post_mask;
                if any(post_nees_mask)
                    post_nees=production_state_nees_at_mask(prod,truth,s,post_nees_mask);
                    if ~isempty(post_nees)
                        seg.post_acquisition_state_nees_mean=mean(post_nees);
                        seg.post_acquisition_state_nees_p95=percentile(post_nees,95);
                        seg.post_acquisition_state_nees_max=max(post_nees);
                    end
                end
            end
        else
            last=find(isfinite(prod.time_s),1,'last');
            if ~isempty(last)
                seg.acquisition_reason_code=prod.acquisition_reason(last,s);
                seg.acquisition_reason=acquisition_reason_string(seg.acquisition_reason_code);
            end
        end
        seg.acquisition_reject_count=double(max(prod.acquisition_reject_count(:,s)));
        dynamic_steps=double(max(prod.acquisition_dynamic_step_count(:,s)));
        dynamic_updates=double(max(prod.acquisition_dynamic_update_count(:,s)));
        if dynamic_steps>0
            seg.acquisition_dynamic_update_fraction=dynamic_updates/dynamic_steps;
        end
    end

    % Raw EKF R0 evidence is the fresh LAST_OBSERVABLE event (bit 1 / 0x02),
    % not the mature resistance-SoH ADVISORY_VALID state (bit 3 / 0x08).
    % The latter requires many accepted observations and is scored by the SoH
    % metrics.  Keeping these layers separate lets short HPPC cases qualify the
    % raw estimator without pretending the slow resistance-SoH observer matured.
    last_observable=bitand(prod.resistance_status_flags(:,s),uint8(2))~=0;

    % Acquisition may intentionally re-anchor R0 to the LUT.  Exclude the
    % transition sample from both raw-R0 accuracy and unobservable-drift scoring
    % by requiring acquisition to have already been complete on the preceding
    % sample.  For runners without acquisition telemetry, retain legacy full-run
    % eligibility rather than inventing a completion state.
    post_acquisition=true(size(valid));
    if isfield(prod,'acquisition_state') && isfield(prod,'acquisition_anchor_count')
        acquisition_complete=prod.acquisition_state(:,s)==uint8(2) & ...
            prod.acquisition_anchor_count(:,s)>0;
        post_acquisition=[false;acquisition_complete(1:end-1)];
    end

    r0_valid=valid & post_acquisition & last_observable & ...
        isfinite(prod.r0_ohm(:,s)) & isfinite(truth.segment_r0_ohm(:,s)) & ...
        truth.segment_r0_ohm(:,s)>0;
    seg.r0_observation_count=nnz(r0_valid);
    r0_min_observations=1;
    if isfield(acceptance,'r0_min_observations')
        r0_min_observations=double(acceptance.r0_min_observations);
    end
    seg.r0_observed=seg.r0_observation_count>=r0_min_observations;
    if seg.r0_observation_count>0
        r0_relative=abs(prod.r0_ohm(r0_valid,s)-truth.segment_r0_ohm(r0_valid,s))./ ...
            truth.segment_r0_ohm(r0_valid,s);
        seg.r0_relative_error_p95=percentile(r0_relative,95);
        seg.r0_accuracy_pass=seg.r0_observed && ...
            seg.r0_relative_error_p95<=acceptance.r0_relative_error_p95_max;
    else
        seg.r0_relative_error_p95=NaN;seg.r0_accuracy_pass=false;
    end

    % Unobservable drift means R0 changes after acquisition without a fresh
    % accepted R0 observation.  Do not use the slow SoH accepted-count state:
    % that misclassified the intentional acquisition LUT reset as drift.
    r0_step=abs([0;diff(prod.r0_ohm(:,s))]);
    unobservable=valid & post_acquisition & ~last_observable;
    candidates=r0_step(unobservable&isfinite(r0_step));
    if isempty(candidates),seg.r0_unobservable_step_drift_max_ohm=0;
    else,seg.r0_unobservable_step_drift_max_ohm=max(candidates);end
    seg.r0_unobservable_drift_pass=seg.r0_unobservable_step_drift_max_ohm<= ...
        acceptance.r0_unobservable_step_drift_max_ohm;

    if acquisition_mode
        seg.accuracy_window='post_convergence';
        accuracy_pass=seg.post_convergence_soc_rmse<=acceptance.soc_rmse_max && ...
            seg.post_convergence_soc_p95_abs<=acceptance.soc_p95_abs_max && ...
            seg.post_convergence_soc_worst_abs<=acceptance.soc_worst_abs_max && ...
            seg.post_convergence_reject_fraction<=acceptance.max_reject_fraction;
    else
        accuracy_pass=seg.soc_rmse<=acceptance.soc_rmse_max && ...
            seg.soc_p95_abs<=acceptance.soc_p95_abs_max && ...
            seg.soc_worst_abs<=acceptance.soc_worst_abs_max && ...
            seg.reject_fraction<=acceptance.max_reject_fraction;
    end
    acquisition_pass=(~acquisition_expected)||seg.acquisition_completed;

    % R0 accuracy is an applicability-gated property.  The diagnostic is
    % computed whenever R0 becomes observable, but it only contributes to the
    % segment release verdict in scenarios that explicitly require EKF-R0.
    r0_accuracy_required=false;
    if isfield(acceptance,'r0_accuracy_required')
        r0_accuracy_required=logical(acceptance.r0_accuracy_required);
    end
    seg.r0_accuracy_required=r0_accuracy_required;
    seg.r0_accuracy_effective_pass=(~r0_accuracy_required) || ...
        (seg.r0_observed && seg.r0_accuracy_pass);
    r0_unobservable_drift_required=false;
    if isfield(acceptance,'r0_unobservable_drift_required')
        r0_unobservable_drift_required=logical(acceptance.r0_unobservable_drift_required);
    end
    seg.r0_unobservable_drift_required=r0_unobservable_drift_required;
    seg.r0_unobservable_drift_effective_pass=(~r0_unobservable_drift_required) || ...
        seg.r0_unobservable_drift_pass;
    convergence_required=true;
    if isfield(acceptance,'convergence_required')
        convergence_required=logical(acceptance.convergence_required);
    end
    seg.convergence_required=convergence_required;
    seg.convergence_pass=(~convergence_required) || ...
        (isfinite(seg.convergence_time_s) && ...
         seg.convergence_time_s<=acceptance.convergence_time_max_s);
    seg.pass=accuracy_pass && seg.convergence_pass && acquisition_pass && ...
        seg.r0_accuracy_effective_pass && seg.r0_unobservable_drift_effective_pass;
    if ~acquisition_pass
        seg.reason='required production acquisition did not complete';
    elseif ~seg.r0_accuracy_effective_pass
        if ~seg.r0_observed
            seg.reason='required production R0 observation did not occur';
        else
            seg.reason='required production R0 accuracy criterion failed';
        end
    elseif ~seg.r0_unobservable_drift_effective_pass
        seg.reason='required production R0 unobservable-drift criterion failed';
    elseif ~seg.pass && isempty(seg.reason)
        seg.reason='production EKF acceptance criterion failed';
    end
    report.segment(s)=seg;all_pass=all_pass&&seg.pass;
end
report.pass=all_pass;
report.note=['This is production-C accuracy/parity evidence. ', ...
    'soc_error_sigma is component-wise; state_nees is the actual ', ...
    '[SoC,Vp1,Vp2] NEES from production full covariance; production_nis ', ...
    'uses the C estimator prior innovation variance.'];
end

function [values,valid_mask,invalid_count]=production_state_nees(prod,truth,s,base_mask)
N=numel(prod.time_s);valid_mask=false(N,1);values=[];invalid_count=0;
required=isfield(prod,'P_full') && isfield(truth,'segment_vp1_V') && ...
    isfield(truth,'segment_vp2_V');
if ~required,return;end
for k=1:N
    if ~base_mask(k),continue;end
    e=[prod.soc(k,s)-truth.segment_soc(k,s); ...
       prod.vp1_V(k,s)-truth.segment_vp1_V(k,s); ...
       prod.vp2_V(k,s)-truth.segment_vp2_V(k,s)];
    P=squeeze(prod.P_full(k,s,:,:));
    if any(~isfinite(e)) || ~isequal(size(P),[3 3]) || any(~isfinite(P),'all')
        invalid_count=invalid_count+1;continue;
    end
    P=0.5*(P+P.');
    [R,flag]=chol(P);
    if flag~=0
        invalid_count=invalid_count+1;continue;
    end
    z=R.'\e;
    n=z.'*z;
    if ~isfinite(n) || n<0
        invalid_count=invalid_count+1;continue;
    end
    valid_mask(k)=true;
    values(end+1,1)=n; %#ok<AGROW>
end
end

function values=production_state_nees_at_mask(prod,truth,s,mask)
[values,~,~]=production_state_nees(prod,truth,s,mask);
end

function text=acquisition_reason_string(code)
switch double(code)
    case 0,text='waiting_for_low_current';
    case 1,text='collecting_relaxation';
    case 2,text='relaxation_interrupted_retry';
    case 3,text='insufficient_samples_retry';
    case 4,text='fit_conditioning_retry';
    case 5,text='fit_residual_retry';
    case 6,text='ocv_range_retry';
    case 7,text='polarization_retry';
    case 8,text='insufficient_consensus_retry';
    case 9,text='segment_consensus_retry';
    case 10,text='fixed_basis_anchor';
    otherwise,text=sprintf('unknown_%d',double(code));
end
end

function value=percentile(x,p)
x=sort(x(:));if isempty(x),value=NaN;return;end
q=1+(numel(x)-1)*p/100;lo=floor(q);hi=ceil(q);
if lo==hi,value=x(lo);else,value=x(lo)+(q-lo)*(x(hi)-x(lo));end
end
function [tconv,index]=settling_time(t,error,band,hold_s)
tconv=Inf;index=NaN;if isempty(t),return;end
last_outside=find(error>band,1,'last');
if isempty(last_outside),candidate=1;else,candidate=last_outside+1;end
if candidate<=numel(t)&&(t(end)-t(candidate))>=hold_s&&all(error(candidate:end)<=band)
    index=candidate;tconv=t(candidate)-t(1);
end
end
