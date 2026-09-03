function summary = summarize_result(result)
%SUMMARIZE_RESULT Compact release/report-friendly scenario result.
summary=struct();
summary.schema_version=7;
summary.scenario_id=result.scenario_id;
summary.seed=result.seed;
summary.mil_configuration_hash=result.mil_configuration_hash;
summary.plant_configuration_hash=result.plant_configuration_hash;
summary.parameter_hash=result.parameter_hash;
summary.duration_s=result.truth.time_s(end)-result.truth.time_s(1);
summary.samples=numel(result.truth.time_s);
summary.minimum_cell_V=min(result.truth.minimum_cell_V);
summary.maximum_cell_V=max(result.truth.maximum_cell_V);
summary.maximum_core_C=max(result.truth.maximum_core_C);
summary.maximum_surface_C=max(result.truth.maximum_surface_C);
summary.invalid_current_samples=nnz(~result.measurement.current_valid);
summary.invalid_pack_voltage_samples=nnz(~result.measurement.pack_voltage_valid);
summary.invalid_measurement_samples=nnz(~result.measurement.measurement_valid);

if isfield(result.metrics,'ekf')
    summary.reference_ekf_pass=result.metrics.ekf.pass;
    summary.reference_ekf_max_soc_rmse=max([result.metrics.ekf.segment.soc_rmse]);
    summary.reference_ekf_max_soc_p95_abs=max([result.metrics.ekf.segment.soc_p95_abs]);
    summary.reference_ekf_max_soc_worst_abs=max([result.metrics.ekf.segment.soc_worst_abs]);
    summary.reference_ekf_max_convergence_time_s=max( ...
        [result.metrics.ekf.segment.convergence_time_s]);
    summary.reference_ekf_max_post_convergence_soc_rmse=max( ...
        [result.metrics.ekf.segment.post_convergence_soc_rmse]);
else
    summary.reference_ekf_pass=false;summary.reference_ekf_max_soc_rmse=NaN;
    summary.reference_ekf_max_soc_p95_abs=NaN;summary.reference_ekf_max_soc_worst_abs=NaN;
    summary.reference_ekf_max_convergence_time_s=NaN;
    summary.reference_ekf_max_post_convergence_soc_rmse=NaN;
end
if isfield(result.metrics,'production_ekf')
    summary.production_ekf_pass=result.metrics.production_ekf.pass;
    summary.production_ekf_max_soc_rmse=max([result.metrics.production_ekf.segment.soc_rmse]);
    summary.production_ekf_max_soc_p95_abs=max([result.metrics.production_ekf.segment.soc_p95_abs]);
    summary.production_ekf_max_soc_worst_abs=max([result.metrics.production_ekf.segment.soc_worst_abs]);
    summary.production_ekf_max_convergence_time_s=max( ...
        [result.metrics.production_ekf.segment.convergence_time_s]);
    summary.production_ekf_max_post_convergence_soc_rmse=max( ...
        [result.metrics.production_ekf.segment.post_convergence_soc_rmse]);
    completed=arrayfun(@(x)field_or(x,'acquisition_completed',false), ...
        result.metrics.production_ekf.segment);
    summary.production_ekf_acquisition_all_completed=all(completed);
    if any(completed)
        acq_times=arrayfun(@(x)field_or(x,'acquisition_time_s',NaN), ...
            result.metrics.production_ekf.segment(completed));
        summary.production_ekf_max_acquisition_time_s=max(acq_times);
        post_sigma=arrayfun(@(x)field_or(x,'post_acquisition_soc_error_sigma_max',NaN), ...
            result.metrics.production_ekf.segment(completed));
        summary.production_ekf_max_post_acquisition_soc_error_sigma=max_finite(post_sigma);
    else
        summary.production_ekf_max_acquisition_time_s=NaN;
        summary.production_ekf_max_post_acquisition_soc_error_sigma=NaN;
    end
    sigma_all=arrayfun(@(x)field_or(x,'soc_error_sigma_max',NaN), ...
        result.metrics.production_ekf.segment);
    summary.production_ekf_max_soc_error_sigma=max_finite(sigma_all);
    nis_mean=arrayfun(@(x)field_or(x,'production_nis_mean',NaN), ...
        result.metrics.production_ekf.segment);
    nis_p95=arrayfun(@(x)field_or(x,'production_nis_p95',NaN), ...
        result.metrics.production_ekf.segment);
    nees_mean=arrayfun(@(x)field_or(x,'state_nees_mean',NaN), ...
        result.metrics.production_ekf.segment);
    nees_p95=arrayfun(@(x)field_or(x,'state_nees_p95',NaN), ...
        result.metrics.production_ekf.segment);
    post_nis=arrayfun(@(x)field_or(x,'post_acquisition_nis_mean',NaN), ...
        result.metrics.production_ekf.segment);
    post_nees=arrayfun(@(x)field_or(x,'post_acquisition_state_nees_mean',NaN), ...
        result.metrics.production_ekf.segment);
    cov_repair=arrayfun(@(x)field_or(x,'covariance_repair_count_max',NaN), ...
        result.metrics.production_ekf.segment);
    nees_invalid=arrayfun(@(x)field_or(x,'state_nees_invalid_count',0), ...
        result.metrics.production_ekf.segment);
    summary.production_ekf_max_nis_mean=max_finite(nis_mean);
    summary.production_ekf_max_nis_p95=max_finite(nis_p95);
    summary.production_ekf_max_state_nees_mean=max_finite(nees_mean);
    summary.production_ekf_max_state_nees_p95=max_finite(nees_p95);
    summary.production_ekf_max_post_acquisition_nis_mean=max_finite(post_nis);
    summary.production_ekf_max_post_acquisition_state_nees_mean=max_finite(post_nees);
    summary.production_ekf_max_covariance_repair_count=max_finite(cov_repair);
    summary.production_ekf_state_nees_invalid_count=sum(nees_invalid);
    r0_counts=arrayfun(@(x)field_or(x,'r0_observation_count',0), ...
        result.metrics.production_ekf.segment);
    summary.production_ekf_min_r0_observation_count=min(r0_counts);
    observed=arrayfun(@(x)field_or(x,'r0_observed',false), ...
        result.metrics.production_ekf.segment);
    if any(observed)
        values=arrayfun(@(x)field_or(x,'r0_relative_error_p95',NaN), ...
            result.metrics.production_ekf.segment(observed));
        summary.production_ekf_max_r0_p95_relative=max(values);
    else
        summary.production_ekf_max_r0_p95_relative=NaN;
    end
    r0_required=arrayfun(@(x)field_or(x,'r0_accuracy_required',false), ...
        result.metrics.production_ekf.segment);
    r0_effective=arrayfun(@(x)field_or(x,'r0_accuracy_effective_pass',true), ...
        result.metrics.production_ekf.segment);
    summary.production_ekf_r0_accuracy_required=any(r0_required);
    summary.production_ekf_r0_accuracy_all_observed=all(observed);
    summary.production_ekf_r0_accuracy_pass=all(r0_effective);
    r0_drift_required=arrayfun(@(x)field_or(x,'r0_unobservable_drift_required',false), ...
        result.metrics.production_ekf.segment);
    r0_drift_effective=arrayfun(@(x)field_or(x,'r0_unobservable_drift_effective_pass',true), ...
        result.metrics.production_ekf.segment);
    summary.production_ekf_r0_unobservable_drift_required=any(r0_drift_required);
    summary.production_ekf_r0_unobservable_drift_pass=all(r0_drift_effective);
else
    summary.production_ekf_pass=false;summary.production_ekf_max_soc_rmse=NaN;
    summary.production_ekf_max_soc_p95_abs=NaN;summary.production_ekf_max_soc_worst_abs=NaN;
    summary.production_ekf_max_convergence_time_s=NaN;
    summary.production_ekf_max_post_convergence_soc_rmse=NaN;
    summary.production_ekf_acquisition_all_completed=false;
    summary.production_ekf_max_acquisition_time_s=NaN;
    summary.production_ekf_max_post_acquisition_soc_error_sigma=NaN;
    summary.production_ekf_max_soc_error_sigma=NaN;
    summary.production_ekf_max_nis_mean=NaN;
    summary.production_ekf_max_nis_p95=NaN;
    summary.production_ekf_max_state_nees_mean=NaN;
    summary.production_ekf_max_state_nees_p95=NaN;
    summary.production_ekf_max_post_acquisition_nis_mean=NaN;
    summary.production_ekf_max_post_acquisition_state_nees_mean=NaN;
    summary.production_ekf_max_covariance_repair_count=NaN;
    summary.production_ekf_state_nees_invalid_count=0;
    summary.production_ekf_max_r0_p95_relative=NaN;
    summary.production_ekf_min_r0_observation_count=0;
    summary.production_ekf_r0_accuracy_required=false;
    summary.production_ekf_r0_accuracy_all_observed=false;
    summary.production_ekf_r0_accuracy_pass=true;
    summary.production_ekf_r0_unobservable_drift_required=false;
    summary.production_ekf_r0_unobservable_drift_pass=true;
end
summary.production_ekf_preconditioned=false;
summary.production_ekf_precondition_rest_s=0;
if isfield(result,'production') && isfield(result.production,'ekf') && ...
        isfield(result.production.ekf,'precondition')
    summary.production_ekf_preconditioned=logical(result.production.ekf.precondition.applied);
    summary.production_ekf_precondition_rest_s=double(result.production.ekf.precondition.executed_rest_s);
end
if isfield(result.metrics,'sop_discharge')
    summary.sop_discharge_pass=result.metrics.sop_discharge.pass;
    summary.sop_discharge_max_unsafe_A=result.metrics.sop_discharge.max_unsafe_overprediction_A;
    summary.sop_discharge_mean_conservatism_A=result.metrics.sop_discharge.mean_conservatism_A;
else
    summary.sop_discharge_pass=true;summary.sop_discharge_max_unsafe_A=NaN;summary.sop_discharge_mean_conservatism_A=NaN;
end
if isfield(result.metrics,'sop_charge')
    summary.sop_charge_pass=result.metrics.sop_charge.pass;
    summary.sop_charge_max_unsafe_A=result.metrics.sop_charge.max_unsafe_overprediction_A;
    summary.sop_charge_mean_conservatism_A=result.metrics.sop_charge.mean_conservatism_A;
else
    summary.sop_charge_pass=true;summary.sop_charge_max_unsafe_A=NaN;summary.sop_charge_mean_conservatism_A=NaN;
end
if isfield(result.metrics,'production_soh')
    summary.production_soh_capacity_observed=result.metrics.production_soh.capacity_observed;
    summary.production_soh_capacity_pass=result.metrics.production_soh.capacity_pass;
    summary.production_soh_capacity_error=result.metrics.production_soh.capacity_error;
    summary.production_soh_capacity_target=result.metrics.production_soh.capacity_target;
    summary.production_soh_resistance_observed=result.metrics.production_soh.resistance_observed;
    summary.production_soh_resistance_pass=result.metrics.production_soh.resistance_pass;
    summary.production_soh_resistance_error=result.metrics.production_soh.resistance_error;
    summary.production_soh_resistance_target=result.metrics.production_soh.resistance_target;
    summary.production_soh_false_aging_pass=result.metrics.production_soh.false_aging_pass;
    summary.production_soh_capacity_accepted_windows=double(max(result.production.soh.accepted_windows));
    summary.production_soh_capacity_rejected_windows=double(max(result.production.soh.rejected_windows));
    summary.production_soh_max_rest_elapsed_s=max(double(result.production.soh.rest_elapsed_s));
    summary.production_soh_last_reason_flags=uint32(result.production.soh.reason_flags(end));
    summary.production_soh_final_resistance_confidence_pct=double(result.production.soh.resistance_confidence_pct(end));
else
    summary.production_soh_capacity_observed=false;summary.production_soh_capacity_pass=false;summary.production_soh_capacity_error=NaN;
    summary.production_soh_capacity_target=NaN;
    summary.production_soh_resistance_observed=false;summary.production_soh_resistance_pass=false;
    summary.production_soh_resistance_error=NaN;summary.production_soh_resistance_target=NaN;summary.production_soh_false_aging_pass=false;
    summary.production_soh_capacity_accepted_windows=0;
    summary.production_soh_capacity_rejected_windows=0;
    summary.production_soh_max_rest_elapsed_s=0;
    summary.production_soh_last_reason_flags=uint32(0);
    summary.production_soh_final_resistance_confidence_pct=0;
end
if isfield(result.metrics,'fuse')
    summary.fuse_model_consistency_pass=result.metrics.fuse.pass;
    summary.fuse_max_state_nonconservative=result.metrics.fuse.max_state_nonconservative;
    summary.fuse_max_cap_nonconservative_A=result.metrics.fuse.max_cap_nonconservative_A;
else
    summary.fuse_model_consistency_pass=true;
    summary.fuse_max_state_nonconservative=NaN;
    summary.fuse_max_cap_nonconservative_A=NaN;
end
summary.numeric_pass=result.metrics.numeric.pass;
summary.production_covariance_psd_checked=field_or( ...
    result.metrics.numeric,'production_covariance_psd_checked',0);
summary.production_covariance_repair_count_max=field_or( ...
    result.metrics.numeric,'production_covariance_repair_count_max',0);
summary.production_innovation_variance_checked=field_or( ...
    result.metrics.numeric,'production_innovation_variance_checked',0);
summary.fault_boundary_pass=result.metrics.faults.pass;
summary.acquisition_fault_recovery_required=false;
summary.acquisition_fault_recovery_pass=true;
summary.production_ekf_effective_pass=summary.production_ekf_pass;
if isfield(result.metrics,'acquisition_fault_recovery')
    afr=result.metrics.acquisition_fault_recovery;
    summary.acquisition_fault_recovery_required=true;
    summary.acquisition_fault_recovery_pass=logical(afr.pass);
    summary.acquisition_fault_recovery_delay_s=afr.recovery_delay_s;
    summary.acquisition_fault_recovery_reject_count=afr.consensus_reject_count;
    target=afr.target_segment;
    segments=result.metrics.production_ekf.segment;
    healthy=setdiff(1:numel(segments),target);
    summary.production_ekf_effective_pass=afr.pass && all([segments(healthy).pass]);
else
    summary.acquisition_fault_recovery_delay_s=NaN;
    summary.acquisition_fault_recovery_reject_count=0;
end
g=result.scenario.gates;
ekf_gate=(~g.ekf)||((summary.reference_ekf_pass||~result.scenario.reference_ekf.enabled) && ...
    (summary.production_ekf_effective_pass||~result.scenario.production.estimator.enabled));
sop_present=isfield(result.metrics,'sop_discharge')&&isfield(result.metrics,'sop_charge');
sop_gate=(~g.sop)||(~result.scenario.sop_oracle.enabled)||(~result.scenario.production.sop.enabled)|| ...
    (sop_present&&summary.sop_discharge_pass&&summary.sop_charge_pass);
capacity_required=field_or(g,'soh_capacity',true);
resistance_required=field_or(g,'soh_resistance',false);
false_aging_required=field_or(g,'soh_false_aging',false);
soh_gate=(~g.soh)||(~result.scenario.production.soh.enabled)|| ...
    ((~capacity_required||summary.production_soh_capacity_pass) && ...
     (~resistance_required||summary.production_soh_resistance_pass) && ...
     (~false_aging_required||summary.production_soh_false_aging_pass));
summary.pass=ekf_gate && soh_gate && sop_gate && (~g.numeric||summary.numeric_pass) && ...
    (~g.fault_boundary||summary.fault_boundary_pass) && ...
    (~result.scenario.fuse.enabled||summary.fuse_model_consistency_pass);
summary.note=['MiL pass combines independent reference checks with direct production-C execution. ', ...
    'Thermal/fuse physical qualification remains separate from model-consistency evidence.'];
end
function value=field_or(s,name,default)
if isfield(s,name),value=s.(name);else,value=default;end
end
function value=max_finite(values)
values=values(isfinite(values));
if isempty(values),value=NaN;else,value=max(values);end
end
