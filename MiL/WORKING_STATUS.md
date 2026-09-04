# DER26 AMS MiL Working Status

Updated: 2026-09-03

This is an executable working MiL implementation, not final qualification evidence.
It reuses the distributed 75-group / 5-segment P42A 2RC + thermal MATLAB plant and
does not run or require the ESP32 HIL target.

## Implemented in this continuation

- exact C0-C8 canonical matrix plus extended scenario catalog (40 scenarios total);
- 20 ms Hall/ADC current subclock aggregated into 100 ms estimator windows;
- stale/dropout semantics that freeze the last numeric value and clear validity;
- 50 A/800 A stuck-low, stuck-high, disagreement and dropout cases;
- CSV drive-profile replay;
- capacity and resistance SoH scoring, observability, and false-aging checks;
- R0 adaptation accuracy/drift metrics;
- independent fuse replay and electrothermal/fuse/combined SoP truth envelopes;
- tiered PR/nightly/release deterministic and Monte Carlo runners;
- 62 requirement records with generated requirements-to-scenario traceability;
- generated release-input readiness audit;
- expanded finite/covariance/numerical-health checks;
- production SoH host self-test;
- AMS default/CubeMX/build manifest moved coherently to 1 Mbit/s CAN, with
  500/250-kbit/s fallback timing retained;
- runtime version banner corrected from v0.5.13 to v0.5.15;
- invalid `current zero` calibration rejection retained from the prior package.

## Verified in this packaging environment

- MiL static architecture checks;
- generated traceability freshness and coverage;
- production EKF host runner build/self-test;
- production SoP host runner build;
- production SoH host runner build/self-test;
- AMS host/unit regression suite;
- independent fuse-oracle regression, including the 50k randomized comparison;
- strict fuse replay cases;
- DER26-CAN-V4 ID/load/logger/IRQ contract gates; and
- fatal RTOS fail-low ordering gate.

The final package records the exact command results. MATLAB execution is not listed
as locally passed because MATLAB/Octave is not installed in this environment. See
`TEST_EVIDENCE_2026-08-29.md` for the production-acquisition candidate evidence and
the licensed-MATLAB reference results supplied from the workstation.


## C1 estimator status — production acquisition validated, covariance/authority integrated

Licensed-MATLAB production-C evidence has now validated the acquisition architecture
against the distributed plant. The directed matrix passed relaxed +/-20 pp, original
HPPC +/-20 pp, boot-under-load, denied-rest +1 A current bias, PEC-invalid delayed
acquisition, 5 C / 40 C, and warm discharge/charge restarts. The coherently biased
segment was rejected by consensus and later reacquired; that case is now scored by a
dedicated fault-recovery requirement instead of weakening the generic 60 s clean-data
convergence gate.

The current development tree additionally implements the previously tracked software
follow-ons:

- full production 3x3 `[SoC,Vp1,Vp2]` covariance with Joseph update and conservative
  PSD/fail-closed guards;
- exact production prior innovation variance plus pre-update/full-covariance host
  telemetry for real production NIS and three-state NEES;
- measured-surface-temperature confidence/R floors near the 5 C / 40 C LUT edges;
- numeric-health full-covariance PSD/innovation-variance checks;
- formal acquisition-fault recovery scenario/metric;
- production SoP `ESTIMATOR_UNACQUIRED` fatal authority reason, so model authority
  remains zero until all enabled segment estimators complete qualified acquisition;
- covariance/SoP static contract regressions and expanded C unit coverage;
- passive logger protocol v4 pages for all covariance cross terms, exact innovation
  sigma, covariance-repair count, and 1 Hz acquisition diagnostics;
- explicit CAN1 TX/SCE IRQ coverage for the enabled asynchronous-TX and bus-off/error
  notifications; and
- fatal RTOS fail-low ordering before diagnostic bookkeeping.

The synchronized production source marker remains `DER26-AMS-v0.5.17-20260903`; v2.6.12 changes only MiL scenario/scoring/summary semantics and does not modify production C.
These are development-candidate changes on that source line, not a vehicle-release claim.
See `docs/C1_ESTIMATOR_ACQUISITION_FINDINGS_2026-08-28.md`.

## Release-grade evidence still blocked on

1. Re-run the production C1 matrix with the full-covariance build and inspect production
   NIS/NEES/covariance-repair telemetry, especially at 5 C and 40 C.
2. Run v2.6.13 C5 first. It keeps the v2.6.10 convergence-applicability correction and short v2.6.8 R0 block, but now also makes raw R0 p95 accuracy an explicit C5 release gate. The same applicability-aware R0 gate is required in C1 and `weak_group_resistance`; unrelated scenarios keep R0 as diagnostics only. If C5 passes, rerun C0-C8 for 9/9, then execute cumulative PR/nightly/release tiers and Monte Carlo.
3. Freeze acceptance thresholds, parameter distributions, seeds, and holdout data.
4. Import the ECU's exact `CAN###.BIN` record source, then implement/verify byte-exact
   raw-log decoder vectors where that external source contract is required.
5. Replay real DER26 SD/CAN logs and correlate current, estimator, thermal, and fuse
   parameters.
6. Validate 1-Mbit/s CAN timing/utilization/error margin and all-node compatibility on
   vehicle hardware.
7. Correlate fuse/aging/thermal models with installed-component and aged-cell data.

The acquisition-current hysteresis and temperature covariance/R floors remain
provisional until those real-data correlations exist; do not invent qualification
constants from model-only results.

## Immediate next MATLAB work

Run the same production-directed C1 matrix against this full-covariance build first:

```matlab
repo = 'C:\DER_AMS\git\revert\DER26-AMS';
clear functions;
rehash;
run(fullfile(repo,'MiL','matlab','scripts','debug_c1_production_acquisition.m'));
```

Then run the formal coherent-bias recovery scenario and C0/C1/C7/core campaign:

```matlab
r_fault = mil.run_scenario('ekf_acquisition_segment_bias_recovery','RunSoP',false);
r_fault.summary

r_c1 = mil.run_scenario('c1_hppc_bad_init','RunSoP',false);
r_c1.summary

results = mil.run_core_campaign();
```

Do not freeze covariance/Q/R thresholds until the executed production NIS/NEES and
real-log correlation support them.

## 2026-09-03 v2.6.12 evidence

Licensed v2.6.11 C5 proves all production EKF segments and raw R0 accuracy pass,
but resistance SoH still false-aged because nine consecutive fresh R0 values were
correlated samples from the leading edge of one excitation episode. v2.6.12 waits
for the episode to close, then commits a bounded episode median. EKF-R0 now also
requires its unobservable-drift criterion wherever the requirement applies.

Covariance consistency is intentionally not recalibrated in this revision. C5
component-wise normalized errors are conservative while full-state NEES remains
high, pointing to covariance geometry/correlation rather than a scalar SoC-floor
or R-only problem.

## 2026-09-03 v2.6.13 MATLAB segment-schema hotfix

The first licensed v2.6.12 C5 attempt reached `mil.metrics.production_ekf` and failed before scoring with `Subscripted assignment between dissimilar structures`. The cause was a harness-only schema mismatch: `r0_unobservable_drift_required` and `r0_unobservable_drift_effective_pass` were assigned to `seg` after initialization but were absent from the preallocated `segment_template`. v2.6.13 adds both fields to the template and adds a static parser check requiring every `seg.<field>=` assignment in `production_ekf.m` to be represented in that fixed schema. Production AMS remains `DER26-AMS-v0.5.17-20260903`; no host-runner rebuild is required if v0.5.17 runners are already current.

## 2026-09-03 v2.6.14 raw-R0 metric semantics

Licensed C1 post-mortem proved the previous EKF-R0 verdict was a harness semantics error: each segment produced five fresh post-acquisition `LAST_OBSERVABLE` R0 updates with 0.35-0.92% p95 relative error and zero post-acquisition unobservable drift, while `ADVISORY_VALID` remained false because the slow resistance-SoH observer had only five accepted observations. v2.6.14 separates these layers. Raw EKF-R0 accuracy is now scored on `LAST_OBSERVABLE`, unobservable drift is scored only after acquisition and only when no fresh R0 update occurs, the intentional acquisition LUT re-anchor is excluded, and applicable raw-R0 tests require at least five observations with a campaign preflight necessary-condition check. No production C changed from AMS v0.5.17.

v2.6.15 advances production AMS to v0.5.18. The main-fuse observer now publishes
and applies symmetric charge/discharge current caps. Production startup/reset
uses a conservative maximum-state seed rather than a shadow-only zero-state
soak, and cold-soak completion no longer erases accumulated utilization. The
independent long-double oracle and strict replay validate both cap directions
and the new production reset policy. Fuse validation locks remain disabled by
default, so the current bench build remains shadow-only.

v2.6.16 advances the source marker to v0.5.19 and adds a five-SMB
`BENCH_VALIDATION` passive-ring observer. It permits advisory OCV SoC with an
explicit open-ring zero-current assumption and fixed 25 C fallback, while
leaving measurement validity, SoH, SoP, BMS_OK and balancing fail-closed. No
MiL model or host production estimator/SoH/SoP/fuse source changed.
