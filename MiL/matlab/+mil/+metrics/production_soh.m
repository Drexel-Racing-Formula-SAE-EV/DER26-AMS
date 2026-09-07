function report = production_soh(prod,truth,acceptance)
%PRODUCTION_SOH Score observable production SoH against architecture-visible truth.
%
% Production capacity SoH consumes pack SoC formed from the five segment EKFs;
% with the distributed plant's capacity-weighted segment/pack SoC this identifies
% aggregate pack capacity, not the weakest individual series group's capacity.
% Production resistance SoH consumes one R0 estimate per 15-group segment, so it
% identifies the worst segment-equivalent resistance growth, not a single-group
% R0 multiplier. Weakest-cell/group safety remains qualified independently by
% the distributed voltage/SoP oracle.
capacity_target=double(truth.capacity_pack_mean);
resistance_target=max(double(truth.resistance_segment_mean));
report=struct();
report.capacity_target=capacity_target;
report.resistance_target=resistance_target;
idx=find(prod.capacity_valid,1,'last');
if isempty(idx)
    report.capacity_observed=false;report.capacity_pass=false;report.capacity_error=NaN;report.capacity_lower_overconfidence=NaN;
else
    report.capacity_observed=true;report.capacity_soh=double(prod.capacity_soh(idx));report.capacity_soh_lower=double(prod.capacity_soh_lower(idx));
    report.capacity_error=report.capacity_soh-capacity_target;
    report.capacity_lower_overconfidence=max(0,report.capacity_soh_lower-capacity_target);
    report.capacity_pass=abs(report.capacity_error)<=acceptance.capacity_abs_error_max && ...
        report.capacity_lower_overconfidence<=acceptance.capacity_lower_overconfidence_max;
end
idxr=find(prod.resistance_valid,1,'last');
if isempty(idxr)
    report.resistance_observed=false;report.resistance_pass=false;report.resistance_upper_underestimate=NaN;
else
    report.resistance_observed=true;report.resistance_growth=double(prod.resistance_growth(idxr));report.resistance_growth_upper=double(prod.resistance_growth_upper(idxr));
    report.resistance_error=report.resistance_growth-resistance_target;
    report.resistance_upper_underestimate=max(0,resistance_target-report.resistance_growth_upper);
    report.resistance_pass=report.resistance_upper_underestimate<=acceptance.resistance_upper_underestimate_max && ...
        abs(report.resistance_error)<=acceptance.resistance_abs_error_max;
end
if ~isfield(report,'resistance_error'),report.resistance_error=NaN;end
fresh_truth=resistance_target<=1.12+1e-9;
report.false_aging_applicable=fresh_truth;
report.false_aging_pass=~fresh_truth || ~report.resistance_observed || ...
    report.resistance_growth<=resistance_target+acceptance.false_aging_growth_max;
% Capacity and resistance observability are separate; run_scenario selects
% which evidence is a gate for each scenario family.
report.pass=report.capacity_pass && report.resistance_pass && report.false_aging_pass;
end
