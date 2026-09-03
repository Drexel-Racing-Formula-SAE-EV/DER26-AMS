# Next MATLAB run — v2.6.14 C0-C8 core campaign

C1 raw-R0 diagnostics from the licensed v2.6.13 campaign proved the production estimator itself was healthy: five fresh post-acquisition R0 observations per segment, 0.35-0.92% p95 relative error, and zero unobservable drift. The prior failure came from using resistance-SoH `ADVISORY_VALID` as the raw EKF-R0 observability signal and from counting the intentional acquisition R0 LUT re-anchor as unobservable drift.

Run:

```matlab
clear functions;
rehash;
results = mil.run_core_campaign();
S = struct2table(results.summaries);
vars = {'scenario_id','production_ekf_pass', ...
    'production_ekf_min_r0_observation_count', ...
    'production_ekf_r0_accuracy_pass', ...
    'production_ekf_r0_unobservable_drift_pass', ...
    'production_soh_capacity_pass', ...
    'production_soh_resistance_pass','pass'};
disp(S(:,vars));
disp('FAILED:');
disp(S(~S.pass,vars));
```

Expected deterministic-core target: 9/9. Production AMS C is unchanged at v0.5.17, so current v0.5.17 host runners may be reused.
