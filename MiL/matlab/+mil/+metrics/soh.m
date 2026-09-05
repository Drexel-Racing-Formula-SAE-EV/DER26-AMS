function report = soh(estimate,truth_soh)
%SOH Score capacity/resistance estimates against plant-defined truth.
report=struct();
if isfield(estimate,'capacity_soh') && isfinite(estimate.capacity_soh)
    report.capacity_error_vs_weakest = estimate.capacity_soh - truth_soh.capacity_pack_weakest;
    report.capacity_error_vs_mean = estimate.capacity_soh - truth_soh.capacity_pack_mean;
else
    report.capacity_error_vs_weakest=NaN;
    report.capacity_error_vs_mean=NaN;
end
if isfield(estimate,'capacity_soh_lower') && isfinite(estimate.capacity_soh_lower)
    report.capacity_lower_is_conservative = ...
        estimate.capacity_soh_lower <= truth_soh.capacity_pack_weakest + 0.01;
else
    report.capacity_lower_is_conservative=false;
end
if isfield(estimate,'resistance_growth_upper') && isfinite(estimate.resistance_growth_upper)
    report.resistance_upper_error = estimate.resistance_growth_upper - truth_soh.resistance_pack_worst;
    report.resistance_upper_is_conservative = ...
        estimate.resistance_growth_upper >= truth_soh.resistance_pack_worst - 0.02;
else
    report.resistance_upper_error=NaN;
    report.resistance_upper_is_conservative=false;
end
end
