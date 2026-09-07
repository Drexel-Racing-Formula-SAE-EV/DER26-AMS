# DER26 AMS Model-in-the-Loop Verification

> **Current qualification harness (v2.6.16 five-SMB bench observer):** v2.6.15 fuse hardening remains intact. Production AMS v0.5.19 adds a strictly non-authoritative passive-ring SoC path for the five-SMB `BENCH_VALIDATION` image; no MiL estimator, SoH, SoP, or fuse model changed.

This directory is the working MiL verification layer for the DER26 accumulator-management firmware. It reuses the checked-in MATLAB battery plant under `HiL/simulink` strictly as a plant/parameter library; it does **not** run the real-time HIL target.

## Scope

The MiL layer currently covers:

- distributed 75-series-group / 5-segment P42A plant execution;
- strict hidden truth bus vs AMS-like measurement bus separation;
- dual-range Hall-current sensor measurement modelling and fault injection;
- independent MATLAB segment EKF reference implementation with retryable fixed-basis startup acquisition;
- direct host execution of the checked-in production EKF C implementation;
- independent truth-forward SoP oracle for 0.1 s / 1 s / 10 s / 30 s horizons;
- direct host execution of the checked-in production SoP C implementation;
- capacity/resistance SoH scoring, observability and temperature-confound cases;
- independent fuse replay plus electrothermal/fuse/combined SoP truth envelopes;
- deterministic C0-C8 core scenarios, extended fault cases, tiered Monte Carlo,
  result export and provenance;
- requirements traceability and frozen-input readiness audits;
- static checks that guard against truth leakage and circular production-model tests.

This is a **working development package**, not final release-qualification evidence yet. Licensed MATLAB has executed the v2.6.9 C5 and full C0-C8 campaign: C0-C4 and C6-C8 pass; C5 production capacity/resistance SoH and false-ageing pass. v2.6.11 corrects the remaining MiL verdict semantics before the next licensed run by making long-run convergence applicability explicit and by ensuring raw R0 accuracy can no longer regress silently in scenarios that claim `EKF-R0`. MATLAB is not installed in the packaging environment, so v2.6.11 numerical evidence remains pending on the licensed workstation.

## Architecture

```text
Existing distributed MATLAB battery plant
          |
          +--> hidden truth bus ---------------------> scoring/oracles only
          |
          `--> AMS measurement model
                 |  noise / bias / quantization
                 |  freshness / dropout / sensor faults
                 v
             measurement bus
                 |
        +--------+------------------+
        |                           |
 independent MATLAB reference    production-C host runners
 EKF / SoH                       EKF / SoH / SoP
        |                           |
        +------------+--------------+
                     v
               metrics / reports
                     ^
                     |
          independent truth SoP oracle
```

The reference estimator is intentionally implemented from the model equations rather than by translating production C line-for-line. Production parity is tested separately through host runners that directly link the checked-in firmware sources.

## Quick start

From MATLAB:

```matlab
cd MiL/matlab/scripts
setup_mil
r = mil.run_scenario('smoke_nominal_us06');
r.summary
```

Run a scenario without the expensive SoP oracle while developing:

```matlab
r = mil.run_scenario('ekf_init_plus20','RunSoP',false);
```

Run a campaign:

```matlab
results = mil.run_campaign({'smoke_nominal_us06','cold_5c','hot_40c'});
```

Run the canonical C0-C8 campaign or a cumulative development tier:

```matlab
results = mil.run_core_campaign();
results = mil.run_tier('pr');       % pr, nightly, or release
mc = mil.run_monte_carlo_tier('c8_dynamic_replay','pr');
```

Host-side integrity checks:

```bash
cd MiL
make static
make traceability
make host-runners
make check
```

## Canonical scenarios currently present

The exact core matrix is C0 bootstrap/current, C1 HPPC with bad initialization,
C2 low-SoC SoP, C3 high-SoC charge SoP, C4 hot weak group, C5 combined
capacity/resistance aging, C6 fuse transient, C7 measurement faults, and C8
dynamic replay. Extended cases cover cold/hot operation, weak groups, current
bias/gain/polarity/dual-range disagreement/ADC rails/dropout, voltage and
temperature bias, stale-last cell/temperature images, PEC-invalid/open-wire-like
images, timestamp jitter, qualifying bursts, endurance, and separate SoH
capacity/resistance/temperature-confound windows.

Current sensing is evaluated on its 20 ms Hall/ADC subclock and conservatively
aggregated into the 100 ms AMS estimator window. Invalid and stale channels retain
their last numeric value while validity/freshness is cleared, matching the
production-facing semantics rather than injecting NaNs into the estimator.


## C1 production estimator architecture

The production C path now carries the validated startup architecture: constrained
SoC-only dynamic correction while acquisition is unresolved, retryable 20 s
fixed-basis relaxation acquisition (`tau1=10 s`, `tau2=35 s` effective basis),
end-of-window residual-polarization initialization, current hysteresis/uncertainty
gating, and median cross-segment consensus. The implementation retains the full
3x3 `[SoC,Vp1,Vp2]` covariance, exports exact prior innovation variance and
pre-update covariance for production NIS/NEES, and fails closed on invalid
covariance. Production SoP model authority is also explicitly blocked while any
enabled segment remains unacquired.

A dedicated `ekf_acquisition_segment_bias_recovery` scenario scores the deliberate
coherent-bias case with fault-specific recovery requirements instead of weakening the
normal clean-data convergence gate. Passive logger protocol v4 now carries the full
production covariance, exact innovation sigma, repair count, and 1 Hz acquisition
diagnostics so those states survive into immutable raw CAN logs. The acquisition
basis/current thresholds and temperature confidence floors remain provisional until
hardware/log correlation. See `docs/C1_ESTIMATOR_ACQUISITION_FINDINGS_2026-08-28.md`.

## Qualification boundary and known blockers

`make check` proves host/static integrity; it does not execute MATLAB. Release
evidence still requires a licensed MATLAB run of the frozen C0-C8 and Monte Carlo
campaigns, frozen acceptance thresholds/distributions, and real DER26 log replay.
The exact ECU `CAN###.BIN` record definition is not present in this repository, so
byte-exact raw-log generation remains blocked until that source contract is
imported. See `docs/BASELINE_READINESS.md` and `WORKING_STATUS.md` for the current
gate state.

## Evidence boundary

The distributed 2RC plant is useful for algorithm verification but is not an independent electrochemical truth source. Thermal and fuse conclusions must not be promoted to physical qualification unless they are correlated against independent hardware data. Likewise, the SoP oracle deliberately differs numerically from `ams_sop.c`, but both remain ECM-based; eventual physical pulse/vehicle data remains required for final model validity.

See `VERIFICATION_SPEC.md` for the verification plan and acceptance philosophy.
