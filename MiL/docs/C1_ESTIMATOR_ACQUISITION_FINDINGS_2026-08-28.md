# C1 estimator acquisition findings and redesign plan — 2026-08-28

Status: **design evidence / MiL screening, not hardware qualification**.

These findings came from licensed-MATLAB runs against the distributed 75-group
P42A plant plus direct execution of the checked-in production estimator host
runner. They are retained here so the acquisition fix does not hide or erase
architectural issues discovered during C1.

## Production findings that remain tracked

1. **Startup state ambiguity is real.** A large terminal-voltage innovation can
   be explained by SoC, Vp1 and Vp2 simultaneously. The first production update
   can move SoC close to truth while creating large artificial polarization.
2. **Startup behavior is sign/history dependent.** If the invented Vp sign happens
   to match real residual polarization, convergence can be very fast; if it is
   opposite, convergence can take roughly 90-100 s in the directed cases.
3. **The historical diagonal covariance became severely overconfident.** In the
   relaxed +20 pp production C1 run, component-wise `|SoC error|/sqrt(P_soc)`
   reached about 29.3-29.5 while actual SoC error remained about five percentage
   points. This finding drove the later production full-covariance change.
4. **Discarded SoC/Vp cross-covariances were architectural debt.** Production now
   retains the full 3x3 `[SoC,Vp1,Vp2]` covariance. This is still not, by itself,
   an acquisition solution: the independent full-covariance reference failed some
   unknown-history restarts when no explicit acquisition policy was used.
5. **Adaptive-R startup pollution is secondary but real.** Large startup residuals
   temporarily inflated production measurement covariance close to 1 V^2.
6. **R0 adaptation was not the C1 root cause.** Directed production runs converged
   toward the expected roughly 13 mOhm region.
7. **Innovation rejection was not the C1 root cause.** Rejection stayed low outside
   deliberate HPPC transition spikes.
8. **NIS/NEES remain diagnostics only.** The reference NIS changes strongly with
   temperature and current noise/process-noise assumptions are not yet hardware
   characterized/frozen.

## Acquisition-specific findings

- A raw two-second low-current OCV anchor is unsafe after recent current flow.
  Directed warm-discharge restart produced about -7.2 percentage points of SoC
  anchor error; warm-charge restart produced about +1.1 percentage points.
- Raw `dV/dt` alone cannot prove relaxation. Voltage slope can be small while
  residual polarization still causes material OCV bias.
- A coherent +20 mV/cell bias in one segment can produce a mathematically valid
  but wrong OCV anchor. Cross-segment consensus is required.
- PEC-invalid voltage must not participate. The prior reference correctly delayed
  an invalid segment until measurements recovered.
- A hard acquisition timeout must not permanently disable later acquisition.
- A knife-edge 1 A rest threshold is fragile under sensor bias/noise. Acquisition
  should use enter/abort hysteresis and prefer no anchor over a false anchor.
- Fit residual alone is not confidence. Short, ill-conditioned exponential fits
  produced sub-mV residuals with very large OCV/SoC errors.

## Selected provisional reference candidate

The directed fixed-basis ranking selected the following **effective acquisition
basis** for integrated MiL testing:

- low-current relaxation window: 20 s
- fit sample period: 1 s
- `tau1 = 10 s`
- `tau2 = 35 s`

The basis is intentionally described as an effective finite-window acquisition
model, not as physically identified P42A RC parameters.

The offline directed ranking for this basis reported approximately:

- mean absolute SoC anchor error: 0.189 percentage points
- worst absolute SoC anchor error: 0.413 percentage points
- mean absolute total-polarization error: 1.75 mV/cell
- worst absolute total-polarization error: 3.43 mV/cell

Those numbers are model-screening evidence only and must not become production
acceptance thresholds.

## Integrated reference policy in this revision

`mil.reference.run_segment_ekf` now supports a retryable fixed-basis acquisition:

1. enter collection only below `current_enter_A`;
2. abort/reset above `current_abort_A` or on invalid segment/current data;
3. collect a continuous 20 s low-current voltage trajectory;
4. fit `V = Vinf + A1 exp(-t/tau1) + A2 exp(-t/tau2)`;
5. invert `Vinf` through the reviewed OCV surface;
6. propagate the fitted polarization to the **end** of the 20 s window before
   initializing Vp1/Vp2;
7. require a minimum fit conditioning sanity check and broad physical bounds;
8. require median cross-segment SoC consensus; reject only the outlier segment
   and retry rather than replacing its state with the pack median;
9. initialize deliberately conservative acquisition covariance; and
10. run acquisition as a **shadow observer** by default: normal EKF updates continue
    while the fit window is pending, and only a successful fit re-anchors the live
    state.

There is no terminal acquisition timeout. A later valid low-current interval can
retry after boot under load, a fault, or a rejected fit. Shadow updates are required
so an unavailable acquisition opportunity does not freeze the estimator indefinitely.

## Integrated-v2.4 correction after first MATLAB rerun

The first integrated v2.4 MATLAB matrix exposed two harness/policy problems that
were not faults in the fixed-basis math itself:

- the canonical HPPC C1 profile did not provide a continuous 20 s low-current
  startup interval, so the first fit could not complete until about 64.1 s; and
- v2.4 held all normal EKF voltage updates while waiting for acquisition. This made
  full-run RMSE/rejection metrics look poor and left a +1 A current-bias case frozen
  at its initial SoC when acquisition was intentionally denied.

The corrected candidate therefore keeps acquisition retryable **and** runs it in
shadow mode. Directed positive-control cases now use an explicit 25 s zero-current
startup window before excitation. Negative fault tests are aligned with the actual
20 s decision window: coherent segment voltage bias persists through the first
fit, while PEC invalidity occupies the first 5 s so delayed reacquisition can be
observed. The original HPPC case is retained separately as a deferred-acquisition
/shadow-EKF behavior check rather than mislabeled as a true relaxed startup.

## v2.4.2 licensed-MATLAB reference validation

The constrained dynamic reference rerun closed the main acquisition-policy
characterization loop. The directed matrix showed:

- true relaxed +/-20 pp: about 0.6 s convergence, 20 s fixed-basis re-anchor;
- original HPPC +/-20 pp: about 31.1-44.1 s convergence, before the deferred
  approximately 64.1 s fixed-basis opportunity;
- boot under +10 A then rest: about 0.6 s convergence with later acquisition;
- +1 A measured-current bias: no fixed-basis acquisition, but constrained dynamic
  convergence in about 35.1 s;
- coherent +20 mV/cell segment bias: consensus rejection/retry exercised;
- PEC-invalid segment: acquisition delayed until a fresh valid window; and
- 5 C, 40 C, warm-discharge, and warm-charge +/-20 pp cases passed.

This result demonstrates why the production design needs **both** paths: a dynamic
fallback that does not manufacture polarization from one scalar residual, plus a
stronger low-current trajectory re-anchor when a valid opportunity appears.

## v2.5 production-C candidate implementation

The validated policy is now ported into the checked-in v0.5.15 production source
line. This is a development candidate, not a firmware release-version change.

The production candidate implements:

1. explicit retryable acquisition state/reason tracking per segment;
2. acquisition current enter/abort hysteresis (provisional 0.5/1.0 A) plus target
   runtime current calibration-record confidence and uncertainty margin;
3. constrained dynamic updates while unresolved: RC/current propagation for Vp1/Vp2,
   bounded SoC-only voltage correction, Vp nuisance uncertainty, covariance floors,
   and no R0/adaptive-R learning from unresolved startup residuals;
4. a fixed 20 s, 1 s-sampled effective basis (`tau1=10 s`, `tau2=35 s`) using
   sufficient statistics rather than storing the full voltage history;
5. end-of-window fitted Vp1/Vp2 initialization rather than stale beginning-of-window
   polarization;
6. physical/conditioning/residual checks and cross-segment median consensus;
7. conservative covariance initialization and adaptive-R history reset on a successful
   anchor;
8. SoH advisory rejection until acquisition is complete;
9. acquisition state, reason, fit, consensus, candidate-state, retry, and dynamic-path
   telemetry through the immutable tuning snapshot and production host runner; and
10. segment-local estimator voltage/temperature validity so one invalid segment does
    not suppress healthy advisory segment EKFs. Raw safety and SoP global validity
    remain independent and conservative.

### Float32 regression stability fix

The new production C acquisition unit test found a numeric failure that MATLAB's
double-precision prototype did not reveal. Computing the least-squares residual from
raw float32 sufficient statistics using a large-term subtraction equivalent to
`y^2 - beta'X'y` suffered catastrophic cancellation and could reject an exact
synthetic relaxation trajectory.

The production accumulator now centers the regression response on the first voltage
sample in the acquisition window before accumulating the sufficient statistics. The
fit then restores that reference into the fitted intercept/OCV. This keeps the same
linear model while greatly reducing cancellation. The exact synthetic fixed-basis C
regression now passes with plausible SoC, end-of-window polarization, conditioning,
and residual telemetry.

### Production evidence still required

MATLAB is not available in the packaging environment, so the distributed-plant
numerical validation of the production-C candidate remains a workstation step. Run
`MiL/matlab/scripts/debug_c1_production_acquisition.m` before treating the port as
accepted. Host/static evidence verifies buildability, policy wiring, and C-level
regressions only.

## v2.6 production covariance / authority completion

The production-directed v2.5.1 MATLAB matrix validated the acquisition port in the
actual production-C host path. Relaxed +/-20 pp, HPPC +/-20 pp, boot-under-load,
denied-rest current bias, PEC-invalid delayed acquisition, temperature corners, and
warm restart cases all met their directed behavior. The coherent segment-bias case
correctly rejected the poisoned candidate and recovered later; its one generic 60 s
clean-data convergence miss is now scored with a dedicated fault-recovery requirement
rather than by weakening the clean estimator gate.

The codebase now also implements the remaining estimator/safety architecture items
that were actionable without hardware correlation:

- production retains a full symmetric 3x3 covariance for `[SoC,Vp1,Vp2]`;
- the scalar voltage update uses the Joseph covariance form and preserves
  measurement-induced cross-covariances;
- covariance is sanity-checked/repair-counted and invalid covariance fails closed;
- production exports the exact prior innovation variance, full covariance, and
  pre-update state/covariance so NIS and true three-state NEES can be computed;
- numeric-health scoring checks production full-covariance PSD and innovation
  variance, rather than checking only diagonal variances;
- confidence/R floors increase toward the 5 C / 40 C electrical-LUT edges based on
  measured surface temperature (the core observer may lag after boot);
- production SoP has an explicit `ESTIMATOR_UNACQUIRED` fatal reason and returns no
  model authority while any enabled segment has not completed qualified acquisition;
- a formal `EKF-ACQ-FAULT-RECOVERY` scenario/metric scores consensus rejection,
  healthy-peer isolation, eventual clean reacquisition, and bounded post-recovery
  error without relaxing the generic 60 s convergence gate.

These changes remain on the synchronized v0.5.15 source line and are development
candidates, not a new release-number claim.

## Evidence still requiring real data / licensed MATLAB

The code changes above close the known software-architecture gaps, but the following
items cannot be truthfully frozen from static/host work alone:

- 0.5/1.0 A acquisition-current thresholds require target current-sensor noise, bias,
  calibration-confidence, and uncertainty correlation;
- temperature Q/R and covariance-floor magnitudes remain provisional until production
  NIS/NEES are rerun across the distributed MATLAB matrix and then correlated to logs;
- fuse/thermal physical correlation remains dependent on installed-component data; and
- release thresholds, Monte-Carlo distributions/seeds, holdout data, and vehicle CAN
  timing still require their planned executed evidence.

Do not convert these evidence dependencies into invented calibration constants merely
to make a software gate green.
