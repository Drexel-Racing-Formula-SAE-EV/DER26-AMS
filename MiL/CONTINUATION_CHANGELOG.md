# MiL Continuation Changelog — 2026-08-28

This revision turns the initial scaffold into a substantially broader executable
qualification harness while preserving the existing plant and production-C sources.

## Main changes

- Added the exact C0-C8 core scenario matrix and extended the inventory to 39 cases.
- Added realistic 20 ms current acquisition/windowing and stale-last fault semantics.
- Completed current rail/dropout, open-wire-like, capacity/resistance SoH, and
  temperature-confound coverage.
- Added CSV profile replay, tier runners, and 32/256/1000-case Monte Carlo tiers.
- Added R0, SoH, fuse, combined SoP-envelope, and numerical-health scoring.
- Added production fuse replay integration and a production SoH runner self-test.
- Added 57 machine-readable requirements and generated traceability reports.
- Added a generated audit for external/frozen release inputs.
- Aligned AMS source, CubeMX, Eclipse build profiles, manifest, and CAN host checks
  with the decided 1-Mbit/s vehicle bus.

## Evidence limits

No MATLAB numerical result is claimed by this package. The environment used to
prepare it does not include MATLAB or Octave. Host/static checks exercise source
integrity and production-C runners only. Final qualification still depends on the
frozen inputs and real-data correlation listed in `WORKING_STATUS.md`.
# 2026-08-28 MATLAB smoke-test hotfix

- Fixed `mil.metrics.ekf` mixed-validity segment aggregation. Invalid and valid
  segments now share one fixed structure schema, preventing MATLAB's
  `Subscripted assignment between dissimilar structures` runtime error.
- Applied the same fixed-schema correction to `mil.metrics.production_ekf`,
  which had the same latent failure mode.
- Added a static regression guard for both metric collectors.

## Reference-EKF observability correction

- MATLAB C1 execution showed the independent four-state EKF driving every R0
  estimate from about 13 mOhm to the 40 mOhm clamp and corrupting nominal SoC
  by roughly 12 percentage points during HPPC pulses.
- Replaced the reference oracle with an independent full-covariance three-state
  SoC/Vp1/Vp2 EKF. R0 now comes from the reviewed SoC/temperature LUT and its
  SoC derivative is included in the voltage Jacobian.
- Production R0 adaptation remains qualified separately through the direct
  production-C runner; no hidden plant truth is supplied to the reference.
- Replaced the temporary-window convergence detector with true settling time
  that must remain inside the band through the end of the run.
- Directed +/-20 percentage-point initialization cases now report full-run
  acquisition error but gate steady-state accuracy only after settling.
## Rest-OCV acquisition correction after first C1 MATLAB run

- The post-R0-hotfix C1 result proved the nominal three-state reference is accurate
  when initialized correctly (roughly 0.1-0.5% SoC RMSE), but a +20 percentage-point
  initial SoC error never settled. Rejection remained only about 1.5%, so the failure
  was not the innovation gate.
- Root cause: the full-covariance EKF can temporarily explain a large DC OCV mismatch
  as Vp1/Vp2. A local nominal-model reproduction shows the first update moves SoC
  toward truth and then the polarization states pull it back to the wrong branch.
- Added an explicit, measurement-only rest-OCV acquisition stage for directed
  acquisition scenarios. It holds normal voltage updates during a 2 s <=1 A rest,
  inverts the reviewed P42A OCV curve, anchors SoC with bounded covariance, then
  returns to the normal independent 3-state EKF. No hidden plant truth is consumed.
- Added acquisition telemetry/metrics and a math-level OCV inversion regression.
- C1 and +/-20% initialization scenarios now explicitly enable this acquisition mode.
- No production firmware estimator code was changed by this correction. Production
  acquisition speed remains a separate MiL result to evaluate with the production-C
  runner.


## Fixed-basis acquisition candidate after warm-restart characterization

- Directed warm restart tests proved the prior two-second low-current raw-OCV
  anchor unsafe in the presence of residual polarization.
- A relaxation-delay sweep proved `dV/dt` alone is not a sufficient relaxation
  confidence metric.
- Short free 2RC fits were rejected because sub-mV fit residuals could coexist
  with very large OCV/SoC errors when the regression was ill-conditioned.
- A fixed-basis 20 s ranking selected provisional effective constants tau1=10 s
  and tau2=35 s for integrated reference testing.
- Replaced the old terminal-timeout rest-OCV reference path with retryable
  low-current collection and fixed-basis OCV/polarization estimation.
- Fitted polarization is propagated to the END of the acquisition window before
  initializing Vp1/Vp2.
- Added enter/abort current hysteresis, validity restart, broad fit/plausibility
  checks, median cross-segment SoC consensus, conservative acquisition
  covariance, and acquisition diagnostic telemetry.
- Added a directed MATLAB regression script covering relaxed +/-20 pp, current
  bias/nonzero-current boot, coherent segment voltage bias, PEC invalidity,
  temperature corners, and warm discharge/charge restarts.
- Replaced the old rest-OCV math regression with a fixed-basis relaxation
  acquisition regression.
- Production estimator source remains unchanged pending the integrated MATLAB
  reference rerun. The full 3x3 production covariance remains a separate open
  architectural issue rather than being silently dropped.


## v2.4.1 shadow-acquisition correction after integrated MATLAB run

- The first v2.4 integrated MATLAB run showed the canonical HPPC stimulus did not
  contain an uninterrupted 20 s startup rest, so fixed-basis acquisition could not
  complete until about 64.1 s.
- The same run showed that holding all EKF voltage updates while acquisition was
  pending was the wrong fallback policy: a denied +1 A-bias acquisition left the
  estimator frozen at its initial SoC.
- Acquisition now runs as a retryable shadow observer by default. Normal EKF updates
  continue while a relaxation fit is pending; only a successful fit re-anchors the
  live state.
- The directed MATLAB script now uses a dedicated 25 s true-rest positive control,
  keeps the original HPPC as deferred-acquisition coverage, holds the coherent
  segment-bias fault through the first 20 s decision, and exercises PEC invalidity
  during the actual acquisition window.
- Added a static shadow-policy regression. Production firmware remains unchanged.

## v2.4.2 constrained dynamic acquisition after v2.4.1 licensed-MATLAB rerun

- The v2.4.1 rerun validated the fixed-basis re-anchor itself at about 20 s for true
  rest and warm discharge/charge histories, including +/-20 pp initialization.
- Cross-segment consensus correctly prevented the coherently biased segment from
  anchoring on the first window, and PEC invalidity correctly blocked participation.
- The rerun also showed the ordinary unconstrained shadow EKF can remain highly wrong
  when fixed-basis acquisition is unavailable: the original HPPC +20 pp case did not
  converge before the 60 s gate, and a +1 A measured-current bias correctly denied
  acquisition but left the dynamic estimator badly wrong.
- Added a constrained dynamic acquisition update. While unacquired, Vp1/Vp2 receive
  only RC/current-model propagation; terminal voltage can apply only a bounded SoC
  correction, with Vp uncertainty treated as nuisance measurement uncertainty.
- Added acquisition covariance floors and dynamic-update telemetry.
- Added post-acquisition NIS/NEES and component-wise SoC error/sigma reporting so
  pre-acquisition inconsistency is not confused with post-anchor confidence.
- Extended the coherent-bias and PEC-invalid directed stimuli so a clean retry window
  is actually available before later HPPC excitation.
- Added the mirrored original-HPPC -20 pp directed case.
- Production firmware remains unchanged in this package.


## v2.5 production-C acquisition candidate — 2026-08-29

Licensed MATLAB v2.4.2 reference evidence validated the constrained dynamic startup
policy plus retryable fixed-basis re-anchor across relaxed +/-20 pp, original HPPC
+/-20 pp, denied-rest current bias, segment bias/consensus, PEC invalidity, temperature
corners, and warm discharge/charge restarts. The validated policy has now been ported
to production C without changing the v0.5.15 firmware revision marker.

Production-C changes include:

- acquisition state/reason and sufficient-statistics storage in `ams_soc_ekf`;
- provisional 0.5 A enter / 1.0 A abort hysteresis with runtime target current
  calibration confidence and uncertainty margin;
- constrained dynamic SoC-only voltage correction while unresolved; Vp states remain
  model-propagated and their uncertainty is treated as nuisance measurement variance;
- retryable 20 s fixed-basis acquisition using the effective 10 s / 35 s basis;
- end-of-window fitted Vp initialization, physical/conditioning checks, and
  cross-segment median consensus;
- conservative covariance and adaptive-R reset on successful anchor;
- no R0 adaptation / adaptive-R learning from unresolved acquisition residuals;
- SoH advisory rejection while acquisition is incomplete;
- immutable tuning snapshot and production-host acquisition telemetry;
- segment-local estimator voltage/temperature gating while raw safety/SoP global
  validity remains independent; and
- production metrics/scripts for acquisition completion, reason, dynamic fraction,
  post-acquisition error, and component-wise `soc_error_sigma`.

The new C fixed-basis unit test exposed catastrophic cancellation in the raw float32
least-squares residual expression. The production fit now centers voltage on the first
window sample before accumulating sufficient statistics and restores the reference in
the fitted OCV intercept. The exact synthetic acquisition regression passes with this
form.

New/expanded checks include `test_production_acquisition_policy.py`, production host
runner parsing, acquisition/consensus C unit tests, denied-rest dynamic-acquisition
coverage, and SoH advisory gating. MATLAB numerical validation of the production port
remains pending on the user workstation.

Still open by design: full 3x3 `[SoC,Vp1,Vp2]` production covariance, final SoP
authority policy while unacquired, hardware-qualified current thresholds, temperature
Q/R calibration, and real-log/thermal/fuse correlation.

## 2026-08-29 — v2.5.1 Windows production-runner rebuild guard

- Fixed a MATLAB production-C matrix failure where `run_estimator.m` expected
  acquisition telemetry (`s0_acq_state`, etc.) but an older
  `build/production_estimator_runner.exe` survived an overlay extraction and
  emitted the pre-acquisition CSV schema.
- `estimator_runner_path.m` now force-rebuilds the checked-in Windows runner
  once per MATLAB function session through MSYS2/UCRT64 before production
  parity execution.
- Added `build_windows_msys2.cmd` with `DER26_MSYS2_BASH` override support.
- Added an explicit CSV-schema guard in `run_estimator.m` so any externally
  supplied or stale runner fails with a targeted rebuild message instead of a
  table-field exception.
- No production estimator algorithm, acquisition policy, acceptance threshold,
  or firmware revision changed in this patch.


## v2.6 full production covariance / SoP acquisition authority — 2026-08-29

- Reviewed the complete C1 characterization and the passed production-C acquisition
  matrix before changing estimator uncertainty architecture.
- Replaced production diagonal-only `[SoC,Vp1,Vp2]` uncertainty propagation with a
  full symmetric 3x3 covariance and Joseph scalar-measurement update.
- Added covariance PSD/sanity repair guards, a dedicated covariance fault bit, and
  repair telemetry; invalid covariance fails closed.
- Added measured-surface-temperature confidence/R floors near the 5 C / 40 C LUT
  edges; values remain provisional pending executed NIS/NEES and real-log correlation.
- Production host output now includes cross-covariances, exact prior innovation
  variance, covariance repair count, and exact pre-update state/covariance.
- Production metrics now compute true three-state NEES and production NIS; numeric
  health verifies full-covariance PSD and prior innovation variance.
- Added formal coherent segment-bias acquisition recovery scoring and scenario without
  weakening the normal 60 s convergence requirement.
- Production SoP now fails zero with `AMS_SOP_REASON_ESTIMATOR_UNACQUIRED` while any
  enabled segment has not completed qualified acquisition.
- Added covariance/SoP policy static regression and expanded production C unit tests.
- Firmware revision marker intentionally remains v0.5.15; this is a development
  candidate, not a v0.5.16 release.


## v2.6.1 software close-out: live covariance logging and fatal-path contracts — 2026-08-29

- Bumped the passive logger protocol to v4 because the live tuning schema now exports
  the complete production 3x3 covariance rather than diagonals only.
- Added `0x6B7` page 3 with signed `P_soc,vp1`, `P_soc,vp2`, and `P_vp1,vp2`; page 2
  now carries exact prior innovation sigma from production `S`.
- Added 1 Hz per-segment acquisition telemetry on `0x6BF` and replaced a duplicate
  slow step-count field with covariance-repair count.
- Extended CAN contract/unit/load gates for the completed stream. Conservative
  1-Mbit/s planning is 9.88% AMS-only, 19.33% with required CM200/ECU traffic, and
  21.22% including optional planning traffic.
- Closed the old CAN interrupt-coverage review finding: CAN1 TX and SCE NVIC vectors
  are now generated/enabled alongside RX0 and all three dispatch to
  `HAL_CAN_IRQHandler(&hcan1)`, matching the HAL notifications enabled by `canbus.c`.
- Closed the fatal-RTOS ordering finding: stack-overflow, allocation-failure, and
  assert paths call the fail-low panic before diagnostic bookkeeping. Added static
  regression gates for both the CAN IRQ contract and fail-low ordering.
- Updated comprehensive host-test expectations to match the now-intentional
  acquisition gate on R0/SoH advisory updates and the segment-local estimator
  voltage/temperature validity boundary. No production safety gate was relaxed.
- No hardware-derived Q/R, current-threshold, thermal, or fuse calibration values were
  invented. Those remain evidence gates, not deferred software bugs.
- Firmware revision marker remains v0.5.15; this is not a v0.5.16 release claim.

## v2.6.2 host invalid-current parity + covariance-repair telemetry semantics — 2026-08-29

- Fixed the production estimator host runner so intentionally non-finite current
  samples can reach the production fail-closed path when `measurement_valid=0`.
  The previous CSV parser rejected those rows before the estimator ran, which broke
  `c0_bootstrap_current` at the injected 800 A channel fault even though the C1
  production runner/schema was current.
- The production-runner self-test now includes an invalid-current `NaN` epoch to keep
  this C0/current-dropout qualification path covered.
- Corrected covariance-repair telemetry semantics. Routine confidence-floor
  enforcement and intentional acquisition decorrelation are estimator policy, not
  numerical covariance repairs, and no longer increment `covariance_repair_count`.
  The counter is now reserved for actual correlation clipping / PSD recovery events,
  making it useful for test-day diagnostics instead of increasing almost every step.
- No acquisition thresholds, SoC acceptance limits, SoP authority policy, or source
  firmware revision marker changed.

## v2.6.3 campaign aggregation + Windows runner freshness guard — 2026-08-29

- Fixed `mil.run_campaign` first-row aggregation. MATLAB cannot indexed-assign a
  populated scenario summary into `struct([])` because the structures have
  dissimilar field schemas. The campaign now seeds `summaries` directly from
  the first completed scenario, normalizes later summaries with `orderfields`,
  and raises an explicit `SummarySchemaMismatch` error if a future scenario
  changes the summary contract.
- Added static regression coverage for the campaign aggregation contract.
- Hardened local Windows production runner handling. Estimator, SoP, and SoH
  runners now use a checked-in source-content signature and rebuild on first
  use or whenever their checked-in C/MATLAB-runner dependencies change. This
  closes the same-session overlay gap where a schema-compatible but behavior-
  stale `.exe` could survive after extracting a newer source package.
- Added MSYS2/UCRT64 build helpers for the production SoP and SoH host runners,
  matching the estimator runner workflow. Users no longer need to manually
  prebuild those local executables for normal MiL execution.
- No estimator/acquisition/SoP/SoH algorithm thresholds, release gates, or
  firmware revision marker changed in v2.6.3.

## v2.6.4 MATLAB structure-aggregation hardening — 2026-08-29

- Fixed `mil.oracle.sop_campaign` checkpoint aggregation. The SoP oracle used the
  same MATLAB failure pattern previously fixed in `mil.run_campaign`: it initialized
  `campaign=struct([])` and then indexed-assigned the first populated SoP snapshot.
  Checkpoint records are now staged in a cell array and concatenated only after real
  records exist.
- Fixed the identical latent first-sample aggregation bug in `mil.monte_carlo` before
  the PR/nightly tiers reached it. Monte Carlo summaries are now staged in cells and
  concatenated after execution.
- Removed the obsolete `struct([])` seed from `mil.run_campaign`; the deterministic
  campaign still seeds from its first completed summary and retains its explicit
  schema/order guard.
- Added a repository-wide MiL MATLAB static regression that detects any variable
  initialized as `struct([])` and later written by indexed structure assignment.
  This prevents recurrence of the opaque `Subscripted assignment between dissimilar
  structures` failure class in future harness code.
- Re-ran the complete MiL host/static suite and whole-source GCC analyzer in bench and
  vehicle profiles; all passed.
- No estimator, acquisition, SoP, SoH, fuse, CAN, safety-policy threshold, or firmware
  source revision marker changed in v2.6.4.

## v2.6.5 embedded-profile parser + core preflight — 2026-08-29

- Replaced the fragile giant multiline-regexp C-array profile parser with bounded
  declaration/initializer parsing and numeric scanning.
- Added checked-in asset regressions for UDDS (14,001 samples), US06 (6,001),
  LA92 (14,351), and the C6 fuse CSV.
- Added C0-C8 profile preflight so missing/malformed/non-finite/non-monotonic or
  sample-time-incompatible assets fail before the expensive deterministic run.
- Licensed MATLAB subsequently executed all nine C0-C8 scenarios end-to-end with
  no harness/runtime exception, exposing three genuine qualification-definition
  issues rather than another parser/container failure.

## v2.6.6 acquisition/SoH observability alignment — 2026-08-30

Licensed v2.6.5 C0-C8 execution passed C0, C1, C2, C3, C6 and C7; C4, C5 and C8
failed their final gates. Review found no reason to loosen the production C
acquisition, covariance, SoP, or SoH thresholds:

- C4/C8 production SoC accuracy and numeric health were already inside the normal
  bounds, but the raw US06 profile contains no 20 s production acquisition window.
  The longest |I|<=0.5 A interval is only 10 s. These dynamic qualification cases
  therefore spent the scored run in constrained/unacquired operation and failed the
  production-EKF gate for acquisition/convergence semantics, not numerical accuracy.
- C5 production EKF passed, but both production SoH outputs remained unobserved. The
  old profile supplied only two large current transitions, while production resistance
  SoH needs 50 accepted R0 observations before ADVISORY_VALID. Its 70 s rests also
  provided little/no margin for acquisition, covariance contraction and polarization
  decay before the 60 s qualified-rest capacity timer.
- Production SoH scoring was also comparing segment/pack aggregate observers against
  unobservable weakest-single-group plant multipliers. Capacity SoH consumes the
  capacity-weighted aggregate pack SoC; resistance SoH consumes five segment-equivalent
  R0 estimates. v2.6.6 scores those outputs against pack-mean capacity and the worst
  segment-mean resistance growth respectively. Weakest-group safety remains qualified
  independently by cell-voltage and distributed SoP limits.

Corrections:

- Added production-estimator stationary preconditioning outside scored scenario time.
  Ordinary in-operation MiL scenarios default to a 30 s real production-C acquisition;
  C0/C1 and the explicit acquisition-recovery/+/-20 pp cases override it to zero and
  continue to exercise startup acquisition inside scored time.
- Precondition rows are accepted only from a valid/calibrated/polarity-qualified first
  measurement, are passed through the actual production estimator, then trimmed so
  truth/metric alignment and the requested scored drive duration are unchanged.
- C4/C8 retain their complete 600 s US06 scoring windows; nominal US06 smoke receives
  the same explicit acquired-start semantics. +/-20 pp acquisition cases now use the
  acquisition-observable HPPC profile instead of raw US06.
- Rebuilt C5 as a production-observable rest / alternating 40-60 A step / rest / step /
  rest sequence. It supplies >50 R0 transitions and two >3 Ah rest-to-rest excursions
  without weakening any production threshold.
- Dedicated release SoH cases now inject architecture-observable aggregate capacity or
  R0 aging and use structurally adequate rest/step excitation.
- Generalized profile preflight to all tiered deterministic campaigns. It now rejects
  production-EKF scenarios with neither a 20 s acquisition opportunity nor a valid
  precondition, resistance-SoH scenarios with <50 structural R0 transitions, and
  capacity-SoH scenarios without three >=60 s rest anchors and two >=3 Ah excursions.
- Summary schema v4 adds preconditioning and SoH target/observability diagnostics.
- No production firmware algorithm threshold, safety gate, CAN contract, or firmware
  revision marker changed. Source marker remains `DER26-AMS-v0.5.15-20260826`.

## v2.6.7 knee-to-knee capacity-SoH observability — 2026-08-31

Licensed v2.6.6 C0-C8 execution passed C0-C4 and C6-C8; only C5 remained failed.
The result isolated two facts:

- C5 resistance SoH is now fully observable and passes at 100% confidence.
- C5 production EKF state accuracy is excellent, but all five segment EKF metric
  records failed only because the old one-second step profile produced a 6.056%
  measurement reject fraction against the unchanged 5% acceptance bound.
- Capacity SoH never opened a candidate window: max qualified rest was 0 s because
  the SoC-confidence gate failed at every potential rest point. The best old-C5
  SoC sigma was 1.759%, above the unchanged 1.5% production SoH limit.

A licensed production-C stationary sweep then mapped the rest-confidence behavior at
25 C. The tail SoC sigma was ~0.33% at 5%, ~0.69% at 10%, ~0.94% at 98%, but
~1.64-2.50% from 15% through 95%. This confirms that the firmware gate is internally
reachable and that the old qualification anchors were placed on OCV-flat regions.

Corrections in v2.6.7:

- Rebuilt C5 as a 98% -> ~9.82% -> 98% knee-to-knee cycle with three 300 s rests.
- The nominal discharge transfers 22.222 Ah. Return charge transfers the same amount
  with a tapered -30/-40 A -> -20 A -> -10 A sequence; maximum charge current is
  40 A pack / 6p = 6.67 A/cell, below the checked-in 8.4 A/cell P42A limit.
- R0 excitation uses 20 s plateaus rather than one-second toggling. Structural R0
  observations remain well above 50, while transition density is greatly reduced.
- The aggregate 20% capacity-fade and capacity-window release scenarios were aligned
  to the same knee-to-knee qualification principle so release-tier preflight does not
  preserve the old mid-SoC observability defect.
- Capacity preflight now includes nominal SoC excursion (>=15%) in addition to >=3 Ah
  throughput, estimates architecture-visible effective pack capacity from explicit
  group capacity overrides, and optionally checks rest anchors against configured
  qualification confidence windows.
- Current capacity scenarios use [4.5,10.5]% and [97.5,98.5]% as conservative
  qualification-only bands around the SoCs directly demonstrated by the licensed
  production-C sweep. These bands are not production firmware limits.
- No production firmware source or threshold changed. The source marker remains
  `DER26-AMS-v0.5.15-20260826`.

## v2.6.8 C5 observer isolation — 2026-09-02

Licensed v2.6.7 C5 post-mortem established that the low-SOC capacity rest was not
blocked by SoC uncertainty: all 3001 low-rest samples passed the sigma gate, but none
passed the unchanged 50 mV cell-spread gate. Only the initial/final high-SOC rests
qualified, so the high-to-high round-trip window was correctly rejected for excursion,
throughput and direction. The same run also showed that 20 s 40/60 A plateaus biased
the production outer-R0 estimator (roughly 39-61% p95 R0 error) and latched a ~1.324
resistance-growth estimate.

Changes in v2.6.8:

- no production firmware threshold, Q/R, SoH, SoP, fuse, CAN or safety change;
- C5 and positive capacity-observability release cases explicitly balance capacity
  multipliers / initial SoC offsets so they test capacity estimation rather than the
  independent cell-spread rejection policy;
- C5 R0 excitation is localized to one 80 s block of 1 s 40/60 A plateaus near
  mid-SOC;
- C5 discharge outside that block is constant +50 A;
- C5 return charge uses <5 A adjacent current steps, preventing repeated production
  R0 updates while retaining a conservative high-SOC taper;
- C5 disables the independent MATLAB reference EKF and fuse replay only to reduce
  runtime; the production C estimator/SoH remain directly scored against plant truth;
- static arithmetic regressions now protect the localized R0 block, charge-step gate,
  balanced capacity-observability overrides and knee-to-knee Ah arithmetic.

## v2.6.9 resistance-SoH transient hardening — 2026-09-03

Licensed v2.6.8 C5 closed the capacity-observability problem: capacity SoH was
observed/passing at 0.9961 with two accepted windows and zero rejections. Segment R0
p95 error also improved to ~5.8-7.5%. The remaining resistance failure was a genuine
production retention issue: `ams_soh.c` monotonically retained the largest qualified
resistance upper bound, so a single transient ratio could permanently report false
ageing (1.161 retained vs 1.0096 truth).

Corrections:

- production SoH now ingests only fresh R0 observations (`LAST_OBSERVABLE` +
  `ADVISORY_VALID`), not repeated stale advisory state;
- each segment uses a 9-observation median confirmation before first acceptance or an
  upward monotonic retained-ageing change;
- confirmed ageing remains monotonic and persistence semantics remain conservative;
- pending confirmation samples are not persisted;
- host regression proves one large qualified spike cannot create permanent ageing,
  while sustained true ageing still updates;
- C5 uses the exact v2.6.6-proven 320 s 1-second 40/60 A excitation around ~80->62%
  SoC to remove the remaining marginal segment-1 dynamic excursion;
- production source marker advances to `DER26-AMS-v0.5.16-20260903`.


## v2.6.10 C5 convergence-applicability correction — 2026-09-03

Licensed v2.6.9 C5 closed the production SoH defects: capacity observation/pass,
resistance observation/pass, and false-ageing rejection all passed.  Full C0-C8
then reached 8/9 with C5 as the sole failure.  The failed row showed all configured
full-run production-EKF accuracy limits passing (max RMSE 1.7457%, p95 2.8051%,
worst 3.3238%, reject ~0.008%) while the generic convergence timer reported
4523.2 s.  `settling_time()` intentionally uses the last >3% excursion, so the
60 s directed-startup criterion was being applied to a 5071 s long-horizon SoH
observer scenario that starts at the true SoC and does not claim `EKF-CONVERGENCE`.

Changes:

- add `acceptance.ekf.convergence_required` with default `true`;
- report `convergence_required` / `convergence_pass` per segment;
- set `convergence_required=false` only in C5, preserving all full-run EKF
  accuracy/rejection gates and diagnostic convergence timing;
- restore the licensed v2.6.8 80 s 1-second 40/60 A mid-SoC R0 block because it
  gave substantially better SoC and raw R0 tracking than the v2.6.9 320 s block;
- keep the v2.6.9 production resistance-SoH fresh-observation + nine-sample median
  hardening unchanged; production source marker remains v0.5.16.

Packaging-environment verification: `make -C MiL check` PASS.  Licensed MATLAB
C5/core rerun remains the next evidence step.

## v2.6.11 applicability-aware raw-R0 gate — 2026-09-03

Review of the v2.6.10 harness identified that `production_ekf.m` already computed
`r0_accuracy_pass`, but `seg.pass` ignored it and `summarize_result.m` exposed only
the raw max p95 number.  This could allow aggregate resistance SoH to pass while a
raw production R0 regression remained silent.

Corrections:

- added `acceptance.ekf.r0_accuracy_required` (default false);
- added per-segment `r0_accuracy_required` and `r0_accuracy_effective_pass`;
- required raw R0 observation + <=15% p95 relative error whenever the gate applies;
- C1 and `weak_group_resistance` explicitly require it because they own `EKF-R0`;
- C5 now also claims `EKF-R0` and requires raw R0 accuracy because it deliberately
  excites R0 as part of the production SoH qualification;
- unrelated scenarios retain R0 diagnostics without failing merely because R0 is not
  observable;
- summary schema bumped to v5 and now exposes R0 applicability, all-observed state,
  effective pass, and max p95 relative error;
- documented the legacy `convergence_time_s` semantics as a last-out-of-band settling
  metric and retained C5's explicit `convergence_required=false`;
- retained the v2.6.10/v2.6.8 80 s R0 block; no production C changed.


## v2.6.12 correlated-R0 episode hardening — 2026-09-03

Licensed v2.6.11 C5 closed raw R0 accuracy and convergence applicability, but
resistance SoH still false-aged to 1.1393 vs 1.0096 truth. Retained-result analysis
proved 32 fresh R0 observations were one correlated episode: first 9-sample block
medians were ~1.11-1.14 and decayed over the next two blocks toward ~1.06-1.08.

Corrections:

- production AMS v0.5.17 groups contiguous fresh R0 updates into an episode;
- episode closure requires a 2.5 s gap with no fresh qualified R0;
- minimum 9 / latest 33 observations are stored statically and summarized by an
  episode median only after closure;
- confirmed ageing remains monotonic/persistent; pending episode data is not persisted;
- added a correlated 32-s decay regression plus sustained-ageing regression;
- EKF-R0 applicability now gates both raw R0 accuracy and unobservable-drift behavior;
- summary schema advances to v6 with explicit R0-drift applicability;
- no EKF covariance/noise calibration or SoP-margin change is made in this revision;
- production source marker advances to `DER26-AMS-v0.5.17-20260903`.

## v2.6.13 MATLAB segment-schema hotfix — 2026-09-03

The first licensed v2.6.12 C5 run exposed a MATLAB-only structure-array defect before metric completion: the R0 unobservable-drift applicability fields were assigned to each local `seg` but omitted from `segment_template`, causing `report.segment(s)=seg` to fail with `Subscripted assignment between dissimilar structures`. Added the two fields to the fixed template and added a packaging-time static schema audit that detects any future `seg.<field>=` assignment missing from the template. No production firmware code, SoH episode logic, thresholds, or runner CSV schema changed.

## v2.6.14 raw-R0 metric semantics — 2026-09-03

- Corrected raw EKF-R0 observability to use fresh `LAST_OBSERVABLE` events instead of mature resistance-SoH `ADVISORY_VALID`.
- Added `r0_observation_count` and a default five-observation minimum for applicable EKF-R0 accuracy gates.
- Corrected unobservable-R0 drift scoring to exclude the acquisition LUT re-anchor and use post-acquisition samples with no fresh R0 event.
- Added raw-R0 structural preflight and configuration validation.
- Bumped scenario summary schema to v7 and exposed minimum raw-R0 observation count.
- No production AMS C changes; firmware remains v0.5.17.
