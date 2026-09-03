#!/usr/bin/env python3
"""Static integrity checks for the DER26 AMS MiL verification layer.

These checks intentionally do not pretend to execute MATLAB. They guard the
architecture that makes the MiL evidence meaningful: hidden truth separation,
canonical scenario inventory, direct production-C runners, and known safety
regressions fixed in the source tree.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MIL = ROOT / "MiL"
MAT = MIL / "matlab"
SCEN = MAT / "configs" / "scenarios"
errors: list[str] = []

def require(cond: bool, msg: str) -> None:
    if not cond:
        errors.append(msg)

def text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        errors.append(f"missing required file: {path.relative_to(ROOT)}")
        return ""

required = [
    MIL / "README.md",
    MIL / "VERIFICATION_SPEC.md",
    MAT / "+mil" / "run_scenario.m",
    MAT / "+mil" / "build_truth_bus.m",
    MAT / "+mil" / "build_measurement_bus.m",
    MAT / "+mil" / "current_window_model.m",
    MAT / "+mil" / "+reference" / "run_segment_ekf.m",
    MAT / "+mil" / "+util" / "soc_from_ocv.m",
    MAT / "+mil" / "+oracle" / "sop_snapshot.m",
    MAT / "+mil" / "+production" / "run_estimator.m",
    MAT / "+mil" / "+production" / "run_sop.m",
    MAT / "+mil" / "+production" / "run_soh.m",
    MAT / "+mil" / "+fuse" / "run_replay.m",
    MIL / "host" / "production_estimator_runner" / "main.c",
    MIL / "host" / "production_sop_runner" / "main.c",
    MIL / "host" / "production_soh_runner" / "main.c",
    MIL / "docs" / "REQUIREMENTS_TRACEABILITY.md",
]
for p in required:
    require(p.is_file(), f"missing required file: {p.relative_to(ROOT)}")

scenario_files = sorted(SCEN.glob("*.m"))
ids: list[str] = []
for p in scenario_files:
    s = text(p)
    require("mil.default_config()" in s, f"scenario does not inherit mil.default_config: {p.name}")
    m = re.search(r"cfg\.id\s*=\s*'([^']+)'", s)
    if not m:
        errors.append(f"scenario has no literal cfg.id: {p.name}")
    else:
        ids.append(m.group(1))
require(len(scenario_files) >= 39, f"expected at least 39 deterministic scenarios, found {len(scenario_files)}")
require(len(ids) == len(set(ids)), "duplicate scenario IDs detected")
for index in range(9):
    matches = [sid for sid in ids if sid.startswith(f"c{index}_")]
    require(len(matches) == 1, f"expected exactly one compact C{index} scenario, found {matches}")


for acquisition_case in ["c1_hppc_bad_init.m", "ekf_init_plus20.m", "ekf_init_minus20.m"]:
    acquisition_src = text(SCEN / acquisition_case)
    require("reference_ekf.acquisition.enabled=true" in acquisition_src,
            f"{acquisition_case} no longer enables explicit reference acquisition")

# Independent reference algorithms may not consume the hidden truth bus.
for p in sorted((MAT / "+mil" / "+reference").glob("*.m")):
    s = text(p).lower()
    require("truth." not in s and "(truth" not in s,
            f"hidden truth leaked into independent reference algorithm: {p.name}")

# No real-time HIL execution/code-generation orchestration is allowed here.
for p in MAT.rglob("*.m"):
    s = text(p).lower()
    forbidden = ["esp32", "generate_code", "arduino", "serialport(", "hil.run_hil"]
    for token in forbidden:
        require(token not in s, f"HIL/runtime token '{token}' found in MiL source: {p.relative_to(MIL)}")

run_ref = text(ROOT / "HiL" / "simulink" / "+hil" / "run_reference.m")
for token in ["store_group_truth", "Vp1_group", "Vp2_group", "T_core_group", "T_surf_group"]:
    require(token in run_ref, f"plant truth instrumentation missing token: {token}")

zero_src = text(ROOT / "AMS" / "Core" / "Src" / "ext_drivers" / "current_sensor.c")
require("(!dev->current_valid)" in zero_src or "!dev->current_valid" in zero_src,
        "current zero calibration no longer requires current_valid")
require("dev->reason != CURRENT_SENSOR_REASON_OK" in zero_src,
        "current zero calibration no longer requires CURRENT_SENSOR_REASON_OK")

measurement_src = text(MAT / "+mil" / "build_measurement_bus.m")
require("current_window_model" in measurement_src,
        "measurement bus no longer uses the 20 ms current-window model")
default_src = text(MAT / "+mil" / "default_config.m")
require("sample_time_s = 0.020" in default_src,
        "default Hall-current subclock is not 20 ms")
require("voltage_quantization_V = 0.001" in default_src,
        "cell snapshot quantization is not 1 mV")
require("temperature_quantization_C = 0.1" in default_src,
        "temperature snapshot quantization is not 0.1 C")
require("precondition_rest_s',30.0" in default_src,
        "ordinary MiL scenarios no longer default to a qualified 30 s production-estimator precondition")
fault_src = text(MAT / "+mil" / "apply_sensor_faults.m")
require("freeze_during_window" in fault_src,
        "stale/dropout faults no longer preserve the last plausible value")

# The independent fuse oracle must not call production observer APIs or stale I^2t fields.
fuse_oracle = text(ROOT / "Tools" / "fuse_replay" / "fuse_reference_oracle.c")
require("ams_fuse_" not in fuse_oracle, "independent fuse oracle calls production ams_fuse_* API")
for stale in ["typical_melting_i2t_a2s", "excess_i2t_a2s"]:
    require(stale not in fuse_oracle, f"stale I^2t field remains in fuse oracle: {stale}")

# Production runners must link source files, not reimplement the algorithms.
est_make = text(MIL / "host" / "production_estimator_runner" / "Makefile")
require("ams_soc_ekf.c" in est_make and "ams_estimator_lut.c" in est_make,
        "production estimator runner does not link checked-in estimator source")
sop_make = text(MIL / "host" / "production_sop_runner" / "Makefile")
require("ams_sop.c" in sop_make and "ams_estimator_lut.c" in sop_make,
        "production SoP runner does not link checked-in SoP source")
soh_make = text(MIL / "host" / "production_soh_runner" / "Makefile")
require("ams_soh.c" in soh_make,
        "production SoH runner does not link checked-in SoH source")

sop_campaign = text(MAT / "+mil" / "+oracle" / "sop_campaign.m")
require("combined_discharge_current_A" in sop_campaign and
        "fuse_discharge_current_cap_A" in sop_campaign,
        "SoP oracle no longer reports the independent fuse-combined envelope")

# Catch the old export field names that did not match the truth oracle schema.
export_src = text(MAT / "+mil" / "export_result.m")
require("discharge_limit_A" not in export_src and "charge_limit_A" not in export_src,
        "export_result uses stale SoP oracle field names")


# Guard against MATLAB's opaque "Subscripted assignment between dissimilar
# structures" failure class. A populated struct cannot be inserted by indexed
# assignment into a variable initialized as struct([]). This has already
# affected deterministic campaigns and the SoP oracle, so scan every MiL
# MATLAB source rather than maintaining a one-off file list.
unsafe_struct_seed = re.compile(
    r"(?m)^\s*(\w+)\s*=\s*struct\(\[\]\)\s*;"
)
for matlab_path in MAT.rglob("*.m"):
    src = text(matlab_path)
    for match in unsafe_struct_seed.finditer(src):
        name = match.group(1)
        tail = src[match.end():]
        indexed_write = re.search(
            rf"\b{re.escape(name)}\s*\([^\)]*\)\s*=", tail
        )
        require(indexed_write is None,
                f"unsafe struct([]) indexed aggregation in {matlab_path.relative_to(MIL)}: {name}")

# Campaign aggregation must seed the summary struct from the first completed
# scenario; indexed assignment into struct([]) fails in MATLAB before C0 can
# be appended. Keep an explicit schema guard for all later rows.
run_campaign_src = text(MAT / "+mil" / "run_campaign.m")
require("if k == 1" in run_campaign_src and "summaries=current_summary" in run_campaign_src,
        "run_campaign no longer seeds the campaign summary struct from the first scenario")
require("SummarySchemaMismatch" in run_campaign_src and "orderfields" in run_campaign_src,
        "run_campaign no longer guards/normalizes scenario summary schema")

sop_campaign_src = text(MAT / "+mil" / "+oracle" / "sop_campaign.m")
require("items = cell" in sop_campaign_src and "campaign=vertcat(items{:})" in sop_campaign_src,
        "SoP oracle checkpoint aggregation no longer stages structs safely in cells")
monte_carlo_src = text(MAT / "+mil" / "monte_carlo.m")
require("summary_cells=cell" in monte_carlo_src and "summaries=vertcat(summary_cells{:})" in monte_carlo_src,
        "Monte Carlo summary aggregation no longer stages structs safely in cells")

# Dynamic C4/C8 score the complete 600 s drive profile, but production SoP
# authority requires an acquired estimator. Keep startup qualification separate
# by preconditioning only those dynamic cases outside scored scenario time.
for dynamic_case in ["c4_hot_weak_group.m", "c8_dynamic_replay.m"]:
    src = text(SCEN / dynamic_case)
    require("precondition_rest_s=30.0" in src,
            f"{dynamic_case} no longer starts dynamic scoring from qualified production acquisition")
    require("cfg.stop_time_s=600" in src,
            f"{dynamic_case} no longer preserves the full 600 s scored dynamic profile")

# The separate nominal US06 smoke case has the same structural acquisition
# limitation as C8. Acquisition-specific +/-20 pp cases instead use the HPPC
# profile with an actual rest window and must not be preconditioned.
smoke_src = text(SCEN / "smoke_nominal_us06.m")
require("precondition_rest_s=30.0" in smoke_src,
        "nominal US06 smoke no longer starts from qualified production acquisition")
for acq_case in ["ekf_init_plus20.m", "ekf_init_minus20.m"]:
    src = text(SCEN / acq_case)
    require("simulation_id='hppc_validation'" in src and "cfg.stop_time_s=120" in src,
            f"{acq_case} no longer uses an acquisition-observable HPPC profile")
    require("precondition_rest_s=0.0" in src,
            f"{acq_case} must exercise real scored acquisition rather than preconditioning")
for acq_case in ["c0_bootstrap_current.m", "c1_hppc_bad_init.m", "ekf_acquisition_segment_bias_recovery.m"]:
    src = text(SCEN / acq_case)
    require("precondition_rest_s=0.0" in src,
            f"{acq_case} must not inherit the ordinary in-operation estimator precondition")

# Production SoH truth must match what the architecture can actually observe:
# aggregate capacity through pack SoC and segment-equivalent R0, while weak-cell
# safety remains a separate SoP/voltage concern.
prod_soh_metric = text(MAT / "+mil" / "+metrics" / "production_soh.m")
require("capacity_pack_mean" in prod_soh_metric and "resistance_segment_mean" in prod_soh_metric,
        "production SoH metric regressed to unobservable weakest-group truth")
soh_truth_src = text(MAT / "+mil" / "soh_truth.m")
require("capacity_pack_observable" in soh_truth_src and "resistance_pack_observable" in soh_truth_src,
        "SoH truth no longer exposes architecture-observable aggregate targets")

# Dedicated release SoH cases must inject aggregate (not one-cell) ageing and
# provide enough rest/step excitation to make their requested observer valid.
cap_only_src = text(SCEN / "soh_capacity_only.m")
require("repmat(struct" in cap_only_src and "capacity_multiplier',0.80" in cap_only_src and
        "'initial_soc_offset',0.0" in cap_only_src and
        "cfg.initial_soc=0.98" in cap_only_src and
        "'duration_s',[300 1600 300 1600 400 800 300]" in cap_only_src and
        "capacity_confidence_soc_windows" in cap_only_src,
        "aggregate capacity-only release scenario is no longer knee-to-knee observer-aligned")
res_only_src = text(SCEN / "soh_resistance_only.m")
require("r0_multiplier',1.40" in res_only_src and "repmat([40 60],1,120)" in res_only_src,
        "aggregate resistance-only release scenario is no longer observer-aligned")
cap_window_src = text(SCEN / "soh_capacity_window.m")
require("cfg.initial_soc=0.98" in cap_window_src and
        "'initial_soc_offset',0.0" in cap_window_src and
        "'capacity_multiplier',1.0" in cap_window_src and
        "'duration_s',[300 2000 300 2000 500 1000 300]" in cap_window_src and
        "capacity_confidence_soc_windows" in cap_window_src,
        "capacity-window release scenario lost knee-to-knee confidence anchors")

# Directed convergence is a default estimator gate.  Long observer-only
# scenarios may explicitly mark it not applicable; they must not alter the
# global 60 s threshold to manufacture a pass.
default_cfg_src = text(MAT / "+mil" / "default_config.m")
require("'convergence_required', true" in default_cfg_src and
        "'r0_accuracy_required', false" in default_cfg_src and
        "'r0_min_observations', 5" in default_cfg_src and
        "'convergence_time_max_s', 60.0" in default_cfg_src and
        "'r0_relative_error_p95_max', 0.15" in default_cfg_src,
        "default EKF applicability/accuracy contract changed")

# C5 must be structurally capable of satisfying the production resistance-SoH
# contract (>=50 qualified current steps) and two knee-to-knee capacity windows;
# do not weaken firmware thresholds merely to make the scenario pass.
c5_src = text(SCEN / "c5_soh_capacity_resistance.m")
for token in ["cfg.initial_soc=0.98", "repmat([40 60],1,40)",
              "r0_duration=ones(size(r0_pair))",
              "'duration_s',[300 780 r0_duration 740 300 charge_duration 300]",
              "charge_current=[-19 -23 -27 -31 -35 -31 -27 -23 -19 -15]",
              "'capacity_multiplier',1.0", "'initial_soc_offset',0.0",
              "capacity_confidence_soc_windows", "cfg.acceptance.ekf.convergence_required=false",
              "cfg.acceptance.ekf.r0_accuracy_required=true", "'EKF-R0'",
              "cfg.gates.soh_resistance=true"]:
    require(token in c5_src, f"C5 production-SoH observability profile missing token: {token}")
production_ekf_src = text(MAT / "+mil" / "+metrics" / "production_ekf.m")
for token in ["'r0_observation_count',0", "'r0_accuracy_required',false", "'r0_accuracy_effective_pass',true",
              "seg.r0_accuracy_effective_pass=(~r0_accuracy_required)",
              "seg.r0_observed && seg.r0_accuracy_pass",
              "last_observable=bitand(prod.resistance_status_flags(:,s),uint8(2))~=0",
              "post_acquisition=[false;acquisition_complete(1:end-1)]",
              "seg.pass=accuracy_pass && seg.convergence_pass && acquisition_pass &&"]:
    require(token in production_ekf_src,
            f"production EKF applicability-aware R0 gate missing token: {token}")

# Requirement-to-gate consistency: every scenario that claims EKF-R0 must make
# raw production R0 accuracy an effective release criterion.  Scenarios that do
# not own EKF-R0 retain the diagnostic without being failed for unobservability.
for scenario_file in SCEN.glob("*.m"):
    src = text(scenario_file)
    if "'EKF-R0'" in src:
        require("r0_accuracy_required=true" in src,
                f"{scenario_file.name} claims EKF-R0 but does not require R0 accuracy")
        require("r0_unobservable_drift_required=true" in src,
                f"{scenario_file.name} claims EKF-R0 but does not require unobservable R0 drift control")

soh_src = text(ROOT / "AMS" / "Core" / "Src" / "soh" / "ams_soh.c")
soh_hdr = text(ROOT / "AMS" / "Core" / "Inc" / "soh" / "ams_soh.h")
power_state_src = text(ROOT / "AMS" / "Core" / "Src" / "sop" / "ams_power_state.c")
run_soh_src = text(MAT / "+mil" / "+production" / "run_soh.m")
require("AMS_SOH_RESISTANCE_EPISODE_MIN_OBSERVATIONS 9u" in soh_hdr and
        "AMS_SOH_RESISTANCE_EPISODE_MAX_OBSERVATIONS 33u" in soh_hdr and
        "AMS_SOH_RESISTANCE_EPISODE_GAP_MS 2500u" in soh_hdr and
        "resistance_episode_median" in soh_src and
        "commit_resistance_episode" in soh_src,
        "resistance SoH lost episode-level median confirmation against correlated false ageing")
require("AMS_SOH_STATUS_LAST_OBSERVABLE" in power_state_src and
        "AMS_SOH_STATUS_ADVISORY_VALID" in power_state_src,
        "power-state SoH bridge no longer requires a fresh qualified R0 observation")
require("bitand(flags,uint8(8))~=0 & bitand(flags,uint8(2))~=0" in run_soh_src,
        "MATLAB production-SoH bridge no longer mirrors fresh R0 observation semantics")
preflight_src = text(MAT / "+mil" / "preflight_campaign.m")
require("EkfR0Unobservable" in preflight_src and "r0_steps<r0_min_observations" in preflight_src,
        "campaign preflight no longer rejects structurally unobservable raw EKF-R0 scenarios")
require("SohResistanceUnobservable" in preflight_src and "r0_steps<50" in preflight_src,
        "campaign preflight no longer rejects structurally unobservable resistance-SoH scenarios")
require("SohCapacityUnobservable" in preflight_src and "capacity_structural_excursion_count" in preflight_src and
        "delta_soc>=0.15" in preflight_src,
        "campaign preflight no longer rejects structurally unobservable capacity-SoH scenarios")
require("SohCapacityConfidenceUnobservable" in preflight_src and
        "capacity_confidence_anchor_count" in preflight_src and
        "capacity_confidence_excursion_count" in preflight_src and
        "effective_pack_capacity_ah" in preflight_src,
        "campaign preflight no longer checks empirically demonstrated capacity-anchor confidence")
require("EstimatorAcquisitionUnobservable" in preflight_src and "longest_acquisition_rest_s" in preflight_src,
        "campaign preflight no longer catches production-EKF scenarios with no acquisition opportunity")
run_tier_src = text(MAT / "+mil" / "run_tier.m")
require("preflight_campaign" in run_tier_src,
        "tiered deterministic campaigns no longer preflight all selected profiles")

# Production estimator preconditioning must be explicit, outside scored time,
# and trimmed before truth/metric alignment.
run_estimator_src = text(MAT / "+mil" / "+production" / "run_estimator.m")
for token in ["PreconditionRestS", "pre_count", "O=O(pre_count+1:end,:)", "EstimatorPreconditionInput"]:
    require(token in run_estimator_src,
            f"production estimator precondition contract missing token: {token}")
summary_src = text(MAT / "+mil" / "summarize_result.m")
require("summary.schema_version=7" in summary_src,
        "summary schema was not bumped for raw-R0 semantics fields")
require("production_soh_capacity_accepted_windows" in summary_src and
        "production_soh_last_reason_flags" in summary_src and
        "production_soh_capacity_target" in summary_src and
        "production_soh_resistance_target" in summary_src,
        "scenario summary no longer exposes production SoH observability/target diagnostics")
require("production_ekf_r0_accuracy_required" in summary_src and
        "production_ekf_r0_accuracy_all_observed" in summary_src and
        "production_ekf_r0_accuracy_pass" in summary_src and
        "production_ekf_min_r0_observation_count" in summary_src and
        "production_ekf_max_r0_p95_relative" in summary_src,
        "scenario summary no longer exposes applicability-aware production R0 accuracy")

# Local Windows production runners must be rebuilt when checked-in source
# changes, not merely when the executable is missing. This prevents an
# overlay-extracted firmware change from silently using a stale .exe with a
# still-compatible CSV schema.
runner_guard = text(MAT / "+mil" / "+production" / "ensure_windows_local_runner.m")
runner_sig = text(MAT / "+mil" / "+production" / "host_runner_signature.m")
require("host_runner_signature" in runner_guard and "built_signatures" in runner_guard,
        "Windows production runner freshness guard is missing source-signature tracking")
require("weighted_sum" in runner_sig and "fread" in runner_sig,
        "production runner source signature no longer depends on file contents")
for runner_path in ["estimator_runner_path.m", "sop_runner_path.m", "soh_runner_path.m"]:
    src = text(MAT / "+mil" / "+production" / runner_path)
    require("ensure_windows_local_runner" in src,
            f"{runner_path} no longer uses the Windows source-freshness rebuild guard")

# MATLAB structure arrays require every element to have an identical field
# schema. Keep invalid/no-sample segment records on the same fixed schema as
# valid metric records so mixed-validity campaigns cannot fail during scoring.
for metric_name in ["ekf.m", "production_ekf.m"]:
    metric_src = text(MAT / "+mil" / "+metrics" / metric_name)
    require("segment_template" in metric_src and "repmat(segment_template" in metric_src,
            f"{metric_name} no longer preallocates a fixed segment schema")

# MATLAB refuses assignment into a preallocated structure array when a later
# element gains a field that is absent from the template.  Catch that class of
# packaging-time defect statically by requiring every seg.<field>= assignment
# in production_ekf.m to already exist in segment_template.
import re
metric_src = text(MAT / "+mil" / "+metrics" / "production_ekf.m")
template_match = re.search(
    r"segment_template=struct\((.*?)\);\s*report\.segment",
    metric_src, re.S)
require(template_match is not None,
        "production_ekf.m segment_template could not be parsed")
template_fields = set(re.findall(r"'([A-Za-z0-9_]+)'\s*,", template_match.group(1)))
assigned_fields = set(re.findall(r"\bseg\.([A-Za-z0-9_]+)\s*=", metric_src))
missing_template_fields = sorted(assigned_fields - template_fields)
require(not missing_template_fields,
        "production_ekf.m assigns fields absent from segment_template: " +
        ", ".join(missing_template_fields))

reference_ekf = text(MAT / "+mil" / "+reference" / "run_segment_ekf.m")
require("nan(N,S,3)" in reference_ekf and "nan(N,S,3,3)" in reference_ekf,
        "independent reference EKF is no longer a three-state covariance model")
require("d_r0" in reference_ekf and "Icell*d_r0" in reference_ekf,
        "reference EKF LUT-R0 SoC Jacobian term is missing")
require("xp(4)" not in reference_ekf and "x(4,s)" not in reference_ekf,
        "unobservable R0 state was reintroduced into the reference EKF")
require("soc_from_ocv" in reference_ekf and "acquisition_hold" in reference_ekf,
        "reference EKF fixed-basis acquisition path is missing")
require("current_enter_A" in reference_ekf and "current_abort_A" in reference_ekf and
        "window_s" in reference_ekf,
        "reference EKF acquisition is no longer gated by retryable low-current window")
require("hold_measurement_updates" in reference_ekf and
        "acquisition_shadow_updates" in reference_ekf,
        "reference EKF acquisition no longer exposes the shadow-update policy")
require("dynamic_soc_only_updates" in reference_ekf and
        "acquisition_dynamic_soc_update" in reference_ekf and "Rnuis" in reference_ekf,
        "reference EKF constrained dynamic acquisition path is missing")
require("tau1_s" in reference_ekf and "tau2_s" in reference_ekf and
        "fixed_basis_candidate" in reference_ekf,
        "reference EKF fixed-basis relaxation fit is missing")
require("segment_consensus_reject_retry" in reference_ekf and
        "min_consensus_segments" in reference_ekf,
        "reference EKF cross-segment acquisition consensus is missing")
require("vp1_finish_V" in reference_ekf and "vp2_finish_V" in reference_ekf,
        "reference EKF acquisition does not initialize residual polarization at window end")
metric_ekf = text(MAT / "+mil" / "+metrics" / "ekf.m")
require("last_outside" in metric_ekf and "post_convergence_soc_rmse" in metric_ekf,
        "reference EKF scoring no longer uses settling/post-convergence metrics")
require("post_acquisition_soc_error_sigma_max" in metric_ekf and
        "post_acquisition_nees_mean" in metric_ekf,
        "reference EKF scoring no longer separates post-acquisition consistency")

if errors:
    print("DER26 MiL static checks: FAIL")
    for e in errors:
        print(f"  - {e}")
    sys.exit(1)
print(f"DER26 MiL static checks: PASS ({len(scenario_files)} scenarios, {len(required)} required artifacts)")
