# MiL Host/Static Test Evidence — 2026-08-28

Environment: Linux host with C11 compiler and Python 3; MATLAB/Octave unavailable.

## Passed commands

```text
python3 -m py_compile MiL/tools/*.py MiL/tests/run_static_checks.py
make -C MiL check
make -C AMS/host_tests target-project-gate can-clock-contract-gate \
    can-load-budget can-v4-contract-gate
```

Observed pass evidence:

- 39 scenarios and 17 required MiL architecture artifacts;
- 57 requirements linked to scenarios and a current generated traceability report;
- current generated frozen-input readiness report;
- fixed-basis relaxation acquisition math regression;
- retryable shadow-acquisition policy regression;
- production estimator runner build/self-test;
- production SoP runner build;
- production SoH runner build/self-test;
- all AMS unit tests and thermistor-model unit tests;
- exact fuse curve-state tests, directed comparison, 50k randomized comparison,
  and invalid-input fail-closed test;
- strict invalid/reset/restore fuse replays;
- CubeIDE project identity/profile/1-Mbit/s build-definition gate;
- 54-MHz HSE-derived 1M/500k/250k CAN timing-source gate;
- 1-Mbit/s conservative CAN utilization planning gate;
- DER26-CAN-V4 ID/priority/scheduler publication contract gate.

## CAN planning result

At the declared 1-Mbit/s nominal bus and current planned publication rates:

- AMS traffic: approximately 9.88%;
- AMS + required CM200/ECU traffic: approximately 19.33%;
- with optional CM200 logging: approximately 21.22%.

These figures reflect the current passive logger protocol v4 budget, including the
full-covariance and 1 Hz acquisition diagnostic pages. They are source/planning
checks, not physical-bus timing evidence.

## Current acquisition-candidate boundary

The user ran the first integrated v2.4 fixed-basis matrix on licensed MATLAB. The
warm discharge/charge cases anchored successfully at about 20.1 s and were symmetric
for +/-20 pp initial error, but the run also exposed a policy/stimulus defect: the
canonical HPPC profile had no uninterrupted 20 s startup rest and v2.4 froze normal
EKF updates while acquisition was pending. This package corrects that reference
policy to retryable **shadow acquisition** and aligns the directed positive/fault
stimuli with the actual 20 s decision window.

The corrected v2.4.1 MATLAB matrix has not been executed in this packaging
environment because MATLAB is unavailable here. Production estimator acquisition
source remains unchanged pending that rerun.

## Not executed

- MATLAB C0-C8 numerical campaigns;
- MATLAB Monte Carlo tiers;
- real `CAN###.BIN`/SD replay;
- target-hardware CAN timing/error-margin measurement.

No claim in this package should reinterpret those missing runs as passed.

## Licensed MATLAB v2.4.1 evidence received 2026-08-29

Directed v2.4.1 results supplied from MATLAB R2025b showed:

- true relaxed +/-20 pp: fixed-basis acquisition completed at 20 s with small
  post-convergence errors;
- warm discharge/charge +/-20 pp: acquisition completed at ~20.1 s with good
  post-convergence behavior and correct residual-polarization sign handling;
- boot under 10 A then rest: no loaded acquisition; successful retry at ~40.1 s;
- +1 A measured-current bias: no false acquisition, but the unconstrained shadow
  estimator remained inaccurate;
- segment-1 +20 mV/cell coherent bias: healthy segments anchored at 20 s while the
  biased segment rejected/retried;
- segment-3 PEC invalidity: healthy segments anchored; the invalid segment did not
  participate until a later clean opportunity;
- 5 C and 40 C positive controls completed acquisition successfully; and
- original HPPC +20 pp remained outside the 60 s convergence target before its
  deferred ~64.1 s fixed-basis anchor.

These findings motivated the v2.4.2 constrained dynamic acquisition path. MATLAB
numerical execution of v2.4.2 remains pending in this packaging environment.
