# MiL / Production-Acquisition Host and Static Evidence — 2026-08-29

Environment used for packaging: Linux host with C11 compiler and Python 3.
MATLAB/Octave is unavailable in this environment, so licensed-MATLAB numerical
results listed below are user-workstation evidence, not locally rerun evidence.

## Licensed MATLAB v2.4.2 reference evidence received

The directed constrained-reference matrix supplied from MATLAB R2025b showed:

- true relaxed +/-20 pp: pass, about 0.6 s dynamic convergence and 20 s re-anchor;
- original HPPC +/-20 pp: pass, about 31.1-44.1 s convergence before the deferred
  approximately 64.1 s fixed-basis re-anchor;
- boot under +10 A then rest: pass, about 0.6 s dynamic convergence and later anchor;
- +1 A measured-current bias: no false fixed-basis acquisition, while the constrained
  dynamic estimator converged in about 35.1 s;
- coherent segment-1 +20 mV/cell bias: first-window consensus rejection followed by
  later clean retry;
- segment-3 PEC invalidity: healthy segments acquired at the first clean opportunity
  and segment 3 acquired only after its own fresh valid window;
- 5 C / 40 C relaxed controls: pass; and
- warm discharge/charge restarts at +/-20 pp: pass.

These results are the design evidence used to authorize the production-C candidate
port. They are not hardware qualification.


## Licensed MATLAB v2.5.1 production-C acquisition evidence received

The user-workstation R2025b production-directed matrix executed the actual checked-in
production estimator through the host runner and showed:

- true relaxed +/-20 pp: pass, about 0.6 s convergence and 20 s acquisition;
- original HPPC +/-20 pp: pass, about 44.1 s convergence and deferred 64.1 s anchor;
- boot under +10 A then rest: pass, about 0.6 s convergence and 40.1 s anchor;
- +1 A measured-current bias: no false fixed-basis acquisition, while constrained
  dynamic estimation still passed at about 35.1 s convergence;
- coherent segment-1 +20 mV/cell bias: consensus rejection was exercised and the
  target later clean-reacquired; the one 60.1 s generic convergence miss is now
  represented by a dedicated fault-recovery requirement rather than a relaxed clean
  gate;
- segment-3 PEC invalidity: healthy peers acquired at 20 s and segment 3 at 25.1 s;
- 5 C / 40 C directed cases: accuracy passed, while covariance consistency remained
  visibly temperature dependent; and
- warm discharge/charge restarts at +/-20 pp: all directed cases passed.

This production-C evidence authorizes freezing the acquisition policy while evaluating
the new full-covariance uncertainty implementation. It remains model-based evidence,
not vehicle/hardware qualification.

## Production-C candidate checks passed in packaging environment

```text
make -C MiL check
make -C AMS/host_tests whole-source-analyze
```

`make -C MiL check` passed all included gates, including:

- 40-scenario MiL static architecture inventory;
- fixed-basis acquisition math regression;
- retryable constrained-shadow reference policy regression;
- production acquisition static policy regression;
- current generated traceability and frozen-input readiness audit;
- production estimator runner build/self-test;
- production SoP runner build;
- production SoH runner build/self-test;
- complete AMS unit suite, including fixed-basis acquisition/consensus, denied-rest
  dynamic acquisition, and SoH acquisition gating;
- thermistor model unit suite;
- independent fuse curve-state oracle, directed comparison, 50k randomized
  comparison, and fail-closed invalid-input test; and
- strict fuse replay invalid/reset/restore cases.

`make -C AMS/host_tests whole-source-analyze` passed GCC `-fanalyzer -fsyntax-only`
over the complete `AMS/Core/Src` tree in both bench and vehicle-profile compile
configurations.

## Numeric implementation issue found and fixed by C regression

A production fixed-basis C unit test exposed catastrophic float32 cancellation in the
raw sufficient-statistics residual expression. The candidate now centers each
segment's regression voltage around the first window sample before accumulating
`X' y` / `y' y` statistics, then restores the voltage reference in the fitted OCV
intercept. The exact synthetic fixed-basis acquisition / consensus C test passes with
this centered implementation.

## Evidence still pending

- licensed-MATLAB re-execution of the production C1 matrix with the new full
  covariance/NIS/NEES build;
- full C0-C8 production numerical campaign and Monte Carlo tiers;
- temperature Q/R/confidence-floor correlation using executed statistics and real logs;
- hardware-qualified current thresholds / uncertainty characterization;
- real SD/CAN log replay and thermal/fuse/model correlation; and
- target-hardware 1-Mbit/s CAN timing/error-margin evidence.

No missing item above is implied to have passed by the host/static evidence.


## v2.6.1 final integration additions

The completed development candidate closes the remaining software-only review items
that were actionable without vehicle measurements:

- production `[SoC,Vp1,Vp2]` covariance is full symmetric 3x3 with Joseph update,
  PSD/sanity repair, covariance faulting, and exact prior innovation variance;
- passive logger protocol is v4 and exports all three covariance cross terms, exact
  innovation sigma, covariance-repair count, and 1 Hz per-segment acquisition state;
- conservative 1-Mbit/s source budget with the completed tuning stream is 9.88%
  AMS-only, 19.33% with required CM200/ECU traffic, and 21.22% with the optional
  planning set;
- CAN1 TX and SCE vectors are now explicitly enabled and route through
  `HAL_CAN_IRQHandler(&hcan1)`, matching the enabled TX-mailbox and error/bus-off
  HAL notifications instead of leaving those vectors at the startup default handler;
- fatal RTOS stack-overflow, malloc-failure, and assert paths now execute the
  fail-low panic before best-effort diagnostic bookkeeping; and
- static regression gates preserve both the CAN interrupt contract and the RTOS
  fail-low ordering.

A final self-contained firmware-CI pass also exposed two stale host-test assumptions
from before the acquisition/segment-local validity redesign. The test harness was
corrected to (1) require qualified acquisition before exercising an accepted R0/SoH
advisory update, and (2) clear per-segment usable masks when intending to inject an
estimator-local voltage/temperature failure. The production behavior was not weakened
to satisfy those old assumptions. The corrected comprehensive host injection suite
passes.

The synchronized firmware source marker intentionally remains
`DER26-AMS-v0.5.15-20260826`; this package is a development/qualification candidate,
not a v0.5.16 release.

## v2.6.2 harness/telemetry corrections

The licensed-MATLAB C1 production matrix confirmed the v2.6.1 full-covariance host
schema was current, but the first C0 core-campaign scenario exposed a host parser
boundary: an intentionally invalid current path emitted `NaN` current with
`measurement_valid=0`, and the runner rejected the CSV row before exercising the
production fail-closed code. v2.6.2 permits that non-finite current only through the
normal parser path; `hardware_inputs_ready` remains false and production finite-input
checks retain authority. The host self-test now includes this case.

The same C1 run showed large `covariance_repair_count` values even when numeric PSD
checks passed. Review found the counter included routine covariance-floor enforcement
and deliberate acquisition cross-covariance clearing. v2.6.2 removes those policy
operations from the repair count so future nonzero counts identify actual covariance
correlation/PSD recovery events.

## v2.6.3 campaign/runner harness correction

The first licensed-MATLAB `mil.run_core_campaign()` attempt after the v2.6.2
invalid-current parser fix completed C0 execution but failed while appending the
first result summary with MATLAB's `Subscripted assignment between dissimilar
structures`. Review confirmed this was a harness construction bug: the campaign
initialized `summaries=struct([])` and then attempted indexed assignment of a
populated fixed-schema summary. v2.6.3 seeds the struct array from the first
scenario and schema-checks/order-normalizes all following summaries.

The same work strengthens Windows production-host parity. Local estimator, SoP,
and SoH executables are now rebuilt on first use and when a lightweight content
signature of their checked-in source dependencies changes, even if MATLAB
functions were not cleared and an old executable has a newer filesystem
modification time. This is important for telemetry-only source changes such as
covariance-repair semantics where CSV schema checking alone cannot detect a
stale executable.

Verification after the change:

- `make -C MiL check` — PASS.
- `make -C AMS/host_tests whole-source-analyze` — PASS for bench and vehicle
  profiles.
- production estimator runner long nominal synthetic sequence with current
  v2.6.2+ source produced zero covariance-repair events, confirming routine
  floor/decorrelation policy no longer increments the repair counter in the
  current executable.


## v2.6.4 SoP/Monte-Carlo aggregation hardening

Licensed MATLAB progressed through C0 and C1 and reached C2 before exposing a second
instance of MATLAB's structure-array construction trap in `mil.oracle.sop_campaign`: a
populated SoP checkpoint snapshot was indexed-assigned into `struct([])`. The same
latent pattern also existed in `mil.monte_carlo`. Both are now cell-staged and
concatenated only after real records exist.

A generic MiL static regression now scans every MATLAB source for the unsafe pattern
`name = struct([]); ... name(index) = ...`, rather than guarding only the two files that
have failed in licensed execution.

Verification after the correction:

- `make -C MiL check` — PASS.
- `make -C AMS/host_tests whole-source-analyze` — PASS for bench and vehicle profiles.
- repository-wide unsafe `struct([])` indexed-aggregation scan — PASS.

MATLAB is not installed in the packaging environment, so the next licensed execution
remains the full C0-C8 core campaign.

## v2.6.5 licensed MATLAB C0-C8 execution

The v2.6.5 profile/parser hardening package completed the full nine-case core campaign
on licensed MATLAB without a runtime/harness exception. Six scenarios passed their
final gates: C0, C1, C2, C3, C6 and C7. Three failed and were retained for diagnosis
rather than having their acceptance thresholds weakened:

- C4 hot/weak: reference EKF passed; production SoC RMSE/p95/worst were
  0.00704/0.01536/0.03847, covariance repairs were zero, numeric and charge/discharge
  SoP checks passed, but production acquisition never completed and reported a 579.1 s
  convergence time.
- C5 SoH: production EKF passed with 0.00244 SoC RMSE and completed acquisition in
  20 s, but production capacity and resistance SoH were both unobserved, so both SoH
  gates failed.
- C8 dynamic US06: reference EKF passed; production SoC RMSE/p95/worst were
  0.00219/0.00422/0.03056, covariance repairs were zero, numeric and SoP checks passed,
  but production acquisition never completed and reported a 301.1 s convergence time.

C7 intentionally disabled the clean-data reference-EKF gate for its measurement-fault
campaign; its overall fault-boundary result passed.

Profile analysis then established that the checked-in 600 s US06 current trace has no
20 s acquisition interval: its longest continuous |I|<=0.5 A interval is about 10 s.
That makes C4/C8 acquisition failure structurally inevitable from an unresolved boot,
while not indicating poor dynamic SoC accuracy.

## v2.6.6 observability correction verification available in packaging environment

The v2.6.6 candidate changes only MiL scenario/harness/oracle semantics around acquired
starts and SoH observability. Production estimator/SoP/SoH C thresholds are unchanged.
Available verification after the changes:

- `make -C MiL check` — PASS, including 40-scenario static inventory, profile assets,
  acquisition regressions, production covariance/SoP policy, traceability, release-input
  audit, production estimator/SoP/SoH host runners, AMS unit tests, thermistor tests,
  independent fuse oracle with 50k randomized comparisons, and strict fuse replays.
- `make -C AMS/host_tests whole-source-analyze` — PASS in bench and vehicle profiles.
- Tier preflight now checks acquisition, capacity-SoH, and resistance-SoH necessary
  observability conditions before deterministic campaign execution.

Superseded by the licensed v2.6.6 run documented below: C0-C4 and C6-C8 pass; C5 remains the only core failure pending v2.6.7.

## v2.6.7 capacity-observability evidence and packaging verification — 2026-08-31

Licensed v2.6.6 MATLAB evidence before this packaging revision:

- C0-C4 and C6-C8 pass; C5 is the only failed core case.
- C5 resistance SoH: observed, pass, final confidence 100%.
- C5 production SoC RMSE across five segments: ~0.09-0.22%; worst absolute SoC
  error ~0.24-0.44%; R0 observed on all segments with ~2.1-3.8% p95 relative error.
- Old C5 capacity observer: 0 accepted windows, 0 rejected windows, no anchor.
- Old C5 rest analysis: 4,199 potential rest samples; 0 passed the 1.5% SoC-sigma
  gate, while 3,579 passed polarization and all 4,199 passed innovation.
- Best old-C5 rest SoC sigma: 1.759%.
- Stationary production-C confidence sweep at 25 C: 5%, 10% and 98% SoC are below
  the 1.5% gate; tested 15-95% points are above it.

v2.6.7 therefore modifies only MiL qualification profiles/preflight semantics. Final
numerical C5/C0-C8 evidence still requires the licensed MATLAB workstation.

## v2.6.7 licensed C5 post-mortem / v2.6.8 corrective design — 2026-09-02

Licensed MATLAB executed the v2.6.7 C5 production-C scenario and the retained-result
post-mortem. The evidence separates the remaining failures cleanly:

- production EKF measurement rejection fell to ~0.149-0.151%, confirming the
  v2.6.7 sparse-edge change solved the prior ~6.06% rejection-rate issue;
- all five production EKFs nevertheless left the 3% convergence band for most of
  the deep cycle (final settling ~4546-4777 s), with several segments exceeding
  the 2% RMSE / 4% p95 acceptance limits;
- R0 was observed with 152-161 accepted updates per segment, but p95 relative R0
  error degraded to ~38.9-61.1%; the global conservative resistance-growth state
  first exceeded 1.10/1.20 near 1440.1 s and 1.30 near 1460.1 s, retaining a
  peak/final value 1.32384;
- the initial ~98% rest produced a qualified anchor at 60.0 s;
- the intended ~10% rest had 3001 samples, all passing SoC-sigma and temperature,
  2999 passing innovation, 2627 passing polarization, but **zero passing the
  unchanged 50 mV cell-spread gate**. Production rest elapsed therefore stayed
  0 s throughout that entire low-SOC rest;
- the final ~98% rest qualified at 5260.1 s. Because the low-SOC anchor never
  existed, production correctly compared final high-SOC against initial high-SOC
  and rejected that round-trip window for SOC_EXCURSION, THROUGHPUT and DIRECTION.

v2.6.8 is therefore a scenario-isolation correction, not a firmware-threshold
change. Positive capacity-observability cases balance capacity multipliers and
initial SoC offsets; C5 localizes 1 s R0 excitation to one mid-SOC block and uses
<5 A adjacent steps on the return charge so the R0 step gate is not repeatedly
retriggered.

Packaging-environment verification for v2.6.8:

```text
MiL static checks                                      PASS
profile assets / embedded C-array parser              PASS
capacity SoH profile arithmetic regression            PASS
fixed-basis acquisition regression                    PASS
acquisition shadow/production policy regressions      PASS
production covariance + SoP safety policy             PASS
traceability freshness                                 PASS
release-input audit                                    PASS
production estimator host runner                      PASS
production SoP host runner                            PASS
production SoH host runner                            PASS
AMS unit suite                                         PASS
fuse independent oracle + 50k randomized comparison   PASS
strict fuse replay cases                               PASS
whole-source GCC analyzer, bench profile              PASS
whole-source GCC analyzer, vehicle profile            PASS
```

MATLAB is not present in the packaging environment; v2.6.8 numerical C5 evidence is
pending the licensed workstation. The synchronized firmware source marker remains
`DER26-AMS-v0.5.15-20260826`.

## v2.6.8 licensed C5 / v2.6.9 production hardening — 2026-09-03

Licensed v2.6.8 C5:

- capacity: observed/pass, 0.9961 estimate, two accepted windows, zero rejected;
- R0 p95 relative error: 5.79-7.54%;
- reject fraction: 0.0059-0.0079%;
- four of five EKF segments pass; segment 1 fails only because the R0-block transient
  briefly reached 3.036% absolute SoC error, making strict settling time 1088.1 s;
- resistance SoH: observed but false-ageing failure, 1.161 retained growth vs 1.0096
  truth, upper 1.211.

Inspection showed the retained resistance algorithm accepted the maximum qualified
upper bound monotonically from repeated advisory-valid input. v2.6.9 now requires
fresh R0 observations and a median of nine before changing the slow retained ageing
state. The C5 R0 block is moved to the v2.6.6-proven ~80->62% SoC excitation region.

Packaging-environment evidence after the production change:

- `make -C MiL check`: PASS;
- production SoP/SoH host suite: PASS, including new transient-spike rejection;
- SoP/SoH ASan+UBSan suite: PASS;
- whole-source GCC analyzer bench/vehicle: PASS.

Licensed v2.6.9 C5 and full C0-C8 remain pending.


## Licensed v2.6.9 C5 + core result / v2.6.10 rationale — 2026-09-03

Licensed MATLAB v2.6.9 C5:

- capacity observed/pass: true / true; capacity SoH 0.9959, error -0.0041;
- capacity windows: 2 accepted, 0 rejected; max qualified rest 300 s;
- resistance observed/pass: true / true; growth 1.0879, upper 1.1379, truth 1.0096;
- false-ageing pass: true; resistance confidence 100%;
- production-EKF max full-run RMSE 0.017457, p95 absolute 0.028051, worst absolute
  0.033238, reject fraction below 8e-5;
- generic production-EKF pass false because the last-settling convergence metric
  reached as late as 4523.2 s.

Full C0-C8 rerun: C0-C4 and C6-C8 passed.  C5 alone failed, and its SoH/numeric
sub-gates passed.  Inspection confirmed the remaining failure is convergence-gate
applicability, not production SoH behavior.  v2.6.10 corrects that applicability
semantics and restores the shorter v2.6.8 R0 block.

## v2.6.11 static R0-gate hardening evidence — 2026-09-03

Before the next licensed MATLAB run, code review found that raw production R0 p95
accuracy was calculated but not an effective pass criterion.  v2.6.11 closes that
harness gap with an applicability-aware gate.  Scenarios claiming `EKF-R0` must now
set `r0_accuracy_required=true`; C5 was deliberately added to that requirement because
it contains an explicit R0-identification block.  The unchanged numerical threshold is
15% p95 relative error.

The revision also documents that `convergence_time_s` is a legacy final-settling
metric, not pure startup-convergence time; C5 continues to report it but does not gate
on it.  Production firmware remains `DER26-AMS-v0.5.16-20260903`.

Packaging-environment verification after the v2.6.11 harness change:

```text
MiL make check                                      PASS
40-scenario static inventory                        PASS
profile/parser regressions                          PASS
capacity profile arithmetic                         PASS
acquisition regressions                             PASS
traceability regeneration/check                     PASS
release-input audit                                 PASS
production EKF/SoP/SoH host runners                PASS
AMS unit suite                                      PASS
fuse oracle + 50k randomized comparison             PASS
strict fuse replay                                   PASS
whole-source GCC analyzer, bench profile            PASS
whole-source GCC analyzer, vehicle profile          PASS
```

Licensed numerical evidence for v2.6.11 is pending.


## v2.6.12 correlated-R0 episode evidence — 2026-09-03

Licensed v2.6.11 C5 retained-result diagnostics:

- all five production EKFs pass; raw R0 p95 = 5.79-7.54%; R0 unobservable drift passes;
- capacity SoH = 0.9961, two accepted windows, zero rejected;
- resistance SoH false-aged to 1.1393 against 1.0096 truth;
- each segment had 32 fresh R0 observations in one episode; first 9-sample median
  = 1.114-1.139, second = 1.084-1.108, third = 1.061-1.082;
- component normalized-error means are conservative (SoC ~0.19-0.28, Vp1 ~0.013-0.015,
  Vp2 ~0.006-0.009) while full-state NEES remains high (~12-14), so covariance
  calibration is not changed here pending a full correlation/eigenvalue audit.

Packaging verification after v0.5.17 episode-level hardening:

- `make -C MiL check`: PASS
- production SoH runner self-test: PASS
- SoP/SoH host core including correlated-episode regression: PASS
- SoP/SoH ASan+UBSan + topology/metamorphic sanitizer checks: PASS
- AMS unit suite: PASS
- fuse oracle + strict replay: PASS
- whole-source GCC analyzer bench + vehicle: PASS
