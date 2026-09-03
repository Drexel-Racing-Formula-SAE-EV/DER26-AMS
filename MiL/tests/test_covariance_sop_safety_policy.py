#!/usr/bin/env python3
"""Static contract guard for production covariance and unacquired SoP authority."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
hdr = (ROOT / 'AMS/Core/Inc/estimator/ams_soc_ekf.h').read_text()
ekf = (ROOT / 'AMS/Core/Src/estimator/ams_soc_ekf.c').read_text()
sop_h = (ROOT / 'AMS/Core/Inc/sop/ams_sop.h').read_text()
sop = (ROOT / 'AMS/Core/Src/sop/ams_sop.c').read_text()
power = (ROOT / 'AMS/Core/Src/sop/ams_power_state.c').read_text()
wrapper = (ROOT / 'MiL/matlab/+mil/+production/run_estimator.m').read_text()
sop_wrapper = (ROOT / 'MiL/matlab/+mil/+production/run_sop.m').read_text()
metric = (ROOT / 'MiL/matlab/+mil/+metrics/production_ekf.m').read_text()
numeric = (ROOT / 'MiL/matlab/+mil/+metrics/numeric_health.m').read_text()
unit = (ROOT / 'AMS/host_tests/unit/ams_unit_test_runner.c').read_text()
sop_test = (ROOT / 'AMS/host_tests/sop/sop_test.c').read_text()

checks = [
    ('float p_soc_vp1;' in hdr and 'float p_soc_vp2;' in hdr and
     'float p_vp1_vp2;' in hdr,
     'production estimator no longer stores all three covariance cross terms'),
    ('covariance_joseph_update' in ekf and 'covariance_positive_semidefinite' in ekf and
     'AMS_EKF_FAULT_COVARIANCE' in hdr,
     'production full-covariance update/fail-closed guard is missing'),
    ('const float uncertainty_temp_C = clampf_local(t_surf_C, 5.0f, 40.0f);' in ekf,
     'confidence floors no longer respond to measured surface temperature'),
    ('result.P_full=nan(N,S,3,3);' in wrapper and
     'innovation_variance_V2' in wrapper and 'pre_update' in wrapper,
     'MATLAB production wrapper no longer preserves full covariance/prior telemetry'),
    ('state_nees_mean' in metric and 'production_nis_mean' in metric and
     'covariance_repair_count_max' in metric,
     'production NIS/NEES/covariance repair diagnostics are incomplete'),
    ('production_covariance_psd_checked' in numeric and 'covariance_psd_3x3' in numeric,
     'numeric health no longer checks production full covariance'),
    ('AMS_SOP_REASON_ESTIMATOR_UNACQUIRED' in sop_h and
     'AMS_SOP_REASON_ESTIMATOR_UNACQUIRED' in sop,
     'unacquired-estimator SoP reason is missing'),
    ('AMS_SOP_REASON_ESTIMATOR_UNACQUIRED |' in sop and
     'fatal_input_reasons' in sop,
     'unacquired estimator is no longer a fatal SoP authority reason'),
    ('input->estimator_acquired = input->estimator_segment_topology;' in power and
     'input->estimator_acquired = 0u;' in power,
     'power-state path no longer aggregates segment acquisition status'),
    ('T.estimator_acquired=double(acquired);' in sop_wrapper,
     'production SoP host wrapper no longer mirrors acquisition authority state'),
    ('test_full_covariance_measurement_update' in unit and
     'test_covariance_temperature_floors_and_topology_r' in unit,
     'full-covariance C unit coverage is missing'),
    ('input.estimator_acquired = 0u;' in sop_test and
     'AMS_SOP_REASON_ESTIMATOR_UNACQUIRED' in sop_test,
     'SoP fail-closed acquisition unit coverage is missing'),
]

failed = [message for ok, message in checks if not ok]
if failed:
    for message in failed:
        print(f'FAIL: {message}')
    raise SystemExit(1)
print('production covariance + SoP acquisition safety policy: PASS')
