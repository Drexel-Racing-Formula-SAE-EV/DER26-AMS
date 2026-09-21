#!/usr/bin/env python3
"""Static contract regression for the production startup acquisition candidate.

Numerical behavior is exercised by AMS host unit tests and MATLAB directed
cases. This check prevents the production policy wiring from silently drifting
away from the reviewed design.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
hdr = (ROOT / 'AMS/Core/Inc/estimator/ams_soc_ekf.h').read_text()
src = (ROOT / 'AMS/Core/Src/estimator/ams_soc_ekf.c').read_text()
task = (ROOT / 'AMS/Core/Src/tasks/estimator_task.c').read_text()
runner = (ROOT / 'MiL/host/production_estimator_runner/main.c').read_text()
wrapper = (ROOT / 'MiL/matlab/+mil/+production/run_estimator.m').read_text()
metric = (ROOT / 'MiL/matlab/+mil/+metrics/production_ekf.m').read_text()
debug = (ROOT / 'MiL/matlab/scripts/debug_c1_production_acquisition.m').read_text()
unit = (ROOT / 'AMS/host_tests/unit/ams_unit_test_runner.c').read_text()

checks = [
    ('AMS_EKF_ACQ_WINDOW_MS                    20000u' in hdr,
     'production acquisition window is no longer 20 s'),
    ('AMS_EKF_ACQ_TAU1_S                         10.0f' in hdr and
     'AMS_EKF_ACQ_TAU2_S                         35.0f' in hdr,
     'production fixed acquisition basis is no longer 10 s / 35 s'),
    ('AMS_EKF_ACQ_CURRENT_ENTER_A               0.50f' in hdr and
     'AMS_EKF_ACQ_CURRENT_ABORT_A               1.00f' in hdr,
     'production low-current hysteresis contract changed'),
    ('voltage_reference_cell_V' in hdr and 'const float yc = y - acq->voltage_reference_cell_V' in src,
     'float32 fit no longer centers voltage before sufficient-statistic regression'),
    ('ams_ekf_step_acquiring_gated' in src and 'acquisition_constrained' in src and
     'ekf->vp1_V = vp1_p;' in src and 'ekf->vp2_V = vp2_p;' in src,
     'constrained dynamic acquisition path is missing'),
    ('update_adaptive_r(ekf, innovation, series_count, uncertainty_temp_C);' in src and
     'Startup acquisition residuals' in src,
     'adaptive-R startup exclusion contract is missing'),
    ('AMS_SOH_REJECT_ACQUISITION' in src and
     'AMS_SOH_REJECT_ESTIMATOR |\n                                     AMS_SOH_REJECT_ACQUISITION' in src,
     'SoH advisory validity no longer rejects unresolved acquisition'),
    ('measurement.current.calibration_record_confident' in task and
     'measurement.current.uncertainty_mA' in task and
     'acquisition_current_trusted' in task,
     'target acquisition no longer uses runtime current calibration/uncertainty'),
    ('segment-local for voltage/temperature' in task and
     'estimator_selected_voltage' in task and 'collect_group_temp' in task,
     'healthy segment estimator continuity no longer uses segment-local validity'),
    ('tick_ms, current_calibrated, step_ok[s]' in runner,
     'host runner acquisition incorrectly requires polarity validation'),
    ('common_valid = logical(meas.current_valid(:))' in wrapper,
     'MATLAB production wrapper still globally suppresses all segments on one voltage fault'),
    ('required_schema' in wrapper and 's0_acq_state' in wrapper and 'EstimatorSchema' in wrapper,
     'MATLAB production wrapper no longer guards against stale host-runner CSV schema'),
    ('build_windows_msys2.cmd' in (ROOT / 'MiL/matlab/+mil/+production/estimator_runner_path.m').read_text() and
     (ROOT / 'MiL/host/production_estimator_runner/build_windows_msys2.cmd').is_file(),
     'Windows estimator runner is no longer force-rebuilt through the MSYS2 helper'),
    ('soc_error_sigma_max' in metric and 'state_nees_mean' in metric and
     'production_nis_mean' in metric and 'full covariance' in metric,
     'production full-covariance NIS/NEES diagnostics are missing'),
    ('p_soc_vp1' in hdr and 'p_soc_vp2' in hdr and 'p_vp1_vp2' in hdr and
     'covariance_joseph_update' in src and 'AMS_EKF_FAULT_COVARIANCE' in hdr,
     'production full 3x3 covariance retention/fail-closed path is missing'),
    ('innovation_variance_V2' in hdr and 'covariance_repair_count' in hdr and
     'pre_p_soc_vp1' in runner,
     'production covariance/pre-update telemetry contract is incomplete'),
    ('PRODUCTION +1 A measured-current bias: NO false acquisition' in debug and
     'PRODUCTION segment 3 PEC invalid first 5 s' in debug,
     'production directed acquisition matrix is incomplete'),
    ('test_fixed_basis_acquisition_and_consensus' in unit and
     'test_dynamic_acquisition_without_false_rest_anchor' in unit,
     'production acquisition C unit regressions are missing'),
]

failed = [msg for ok, msg in checks if not ok]
if failed:
    for msg in failed:
        print(f'FAIL: {msg}')
    raise SystemExit(1)
print('production acquisition policy regression: PASS')
