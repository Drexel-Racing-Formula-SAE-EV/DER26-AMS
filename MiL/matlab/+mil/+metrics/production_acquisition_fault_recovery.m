function report = production_acquisition_fault_recovery(prod,truth,ekf_metrics, ...
    target_segment,fault_end_s,acceptance)
%PRODUCTION_ACQUISITION_FAULT_RECOVERY Score consensus-rejected segment recovery.
%
% This intentionally does not reuse the clean-start convergence gate. A
% coherent biased segment must first be rejected rather than falsely anchored;
% qualification then requires healthy peer isolation, eventual clean
% reacquisition inside the finite scenario, and bounded post-recovery error.
arguments
    prod struct
    truth struct
    ekf_metrics struct
    target_segment (1,1) double {mustBeInteger,mustBePositive}
    fault_end_s (1,1) double {mustBeFinite,mustBeNonnegative}
    acceptance struct
end

S=size(prod.soc,2);
if target_segment>S
    error('mil:acqFaultRecovery:Segment','target_segment exceeds estimator topology.');
end
seg=ekf_metrics.segment(target_segment);
healthy=setdiff(1:S,target_segment);

report=struct();
report.target_segment=target_segment;
report.fault_end_s=fault_end_s;
report.consensus_reject_count=double(seg.acquisition_reject_count);
report.consensus_rejection_observed=seg.acquisition_reject_count>=1;
report.acquisition_completed=logical(seg.acquisition_completed);
report.acquisition_time_s=double(seg.acquisition_time_s);
report.no_false_anchor_during_fault=~seg.acquisition_completed || ...
    seg.acquisition_time_s>fault_end_s;
report.reacquired_after_fault=seg.acquisition_completed && ...
    seg.acquisition_time_s>fault_end_s;
if report.reacquired_after_fault
    report.recovery_delay_s=seg.acquisition_time_s-fault_end_s;
else
    report.recovery_delay_s=Inf;
end

report.post_recovery_soc_worst_abs=double(seg.post_acquisition_soc_worst_abs);
limit=acceptance.soc_worst_abs_max;
report.post_recovery_accuracy_pass=isfinite(report.post_recovery_soc_worst_abs) && ...
    report.post_recovery_soc_worst_abs<=limit;

healthy_segments=ekf_metrics.segment(healthy);
report.healthy_segments_pass=all([healthy_segments.pass]);
report.healthy_segments_acquired=all([healthy_segments.acquisition_completed]);
if isempty(healthy_segments)
    report.healthy_max_acquisition_time_s=NaN;
else
    report.healthy_max_acquisition_time_s=max([healthy_segments.acquisition_time_s]);
end
% Healthy peers should not be held hostage by the bad segment. A peer anchor
% that occurs by the end of the corrupted window proves segment isolation;
% small timestamp quantization is allowed.
report.healthy_isolation_pass=report.healthy_segments_acquired && ...
    report.healthy_max_acquisition_time_s<=fault_end_s+0.25;

% Re-check post-anchor error directly from production/truth as a guard against
% accidental metric-window changes.
report.direct_post_recovery_soc_worst_abs=NaN;
if report.reacquired_after_fault
    k=find(prod.time_s>=prod.time_s(1)+seg.acquisition_time_s,1,'first');
    if ~isempty(k)
        e=prod.soc(k:end,target_segment)-truth.segment_soc(k:end,target_segment);
        e=e(isfinite(e));
        if ~isempty(e)
            report.direct_post_recovery_soc_worst_abs=max(abs(e));
        end
    end
end
report.direct_post_recovery_match=isfinite(report.direct_post_recovery_soc_worst_abs) && ...
    report.direct_post_recovery_soc_worst_abs<=limit;

report.pass=report.consensus_rejection_observed && ...
    report.no_false_anchor_during_fault && ...
    report.reacquired_after_fault && ...
    report.post_recovery_accuracy_pass && ...
    report.direct_post_recovery_match && ...
    report.healthy_segments_pass && report.healthy_isolation_pass;
report.note=['Fault-specific acceptance: reject a poisoned OCV candidate first, ', ...
    'preserve healthy-peer acquisition, then require eventual clean reacquisition ', ...
    'and bounded post-recovery SoC. The generic clean-data 60 s convergence gate ', ...
    'is intentionally not relaxed.'];
end
