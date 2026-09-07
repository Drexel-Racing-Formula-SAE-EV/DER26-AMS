# DER26 AMS v2.6.22 post-closeout regression fixes

**Date:** 2026-09-05  
**Firmware:** 0.5.25  
**Package:** v2.6.22  
**Parent:** v2.6.21 / firmware 0.5.24

This patch closes four defects found in the follow-up review of the v2.6.21
migration-readiness closeout. It does not begin the Zephyr migration and does not
change the approved SPI String-B chip-select mapping: firmware remains on PE4.

## 1. Qualified headless Release could silently build the wrong profile

### Defect

The headless build selected only Debug/Release optimization. It did not define
`AMS_BUILD_PROFILE`, so `ams_build_profile.h` fell back to BENCH. That made the
new headless Release path behaviorally different from the checked-in CubeIDE
configurations, which explicitly select profile 5. The script also used `-O2`
while CubeIDE Release uses size optimization, and its provenance did not record
the selected profile or an immutable hash of the authored source inputs.

### Fix

- `ci/stm32/build_ams_headless_gcc.sh` now requires a valid explicit profile
  selection and defaults to profile 5 (`bench_validation`) to match the checked-in
  CubeIDE configurations.
- `AMS_BENCH_VALIDATION_SINGLE_SMB` is validated and passed explicitly.
- Release uses `-Os -g0`, matching CubeIDE Release optimization.
- Provenance records the numeric/name profile, single-SMB selection, mode flags,
  warning policy, `.cproject` hash, and an immutable SHA-256 over the authored
  source/linker/CubeMX/build-configuration inputs.
- Invalid profile values are rejected before compiler/toolchain lookup.

This closes the source/tooling regression. Target artifact equivalence still
requires an approved ARM build and remains part of F-12 evidence.

## 2. AUX2 diagnostic scheduling replayed missed deadlines

### Defect

`adbms_aux2_next_due_tick` began at zero and each execution advanced the deadline
by one period. Startup delay or an ADBMS transport outage therefore accumulated
missed deadlines. After recovery the 10 Hz ADBMS task could execute AUX2 on many
consecutive cycles until the absolute schedule caught up.

### Fix

- Added an explicit AUX2 schedule-initialized state.
- A newly ready/recovered ring schedules the first AUX2 operation one period from
  the current epoch rather than replaying boot/outage history.
- After a due operation, `adbms_next_future_due()` advances to the first future
  absolute slot and skips any additionally missed slots.
- Loss of transport/topology readiness disarms the schedule so recovery starts a
  fresh cadence.

The existing thermistor/open-wire diagnostic policy is otherwise unchanged.

## 3. SoP/SoH freshness omitted constituent acquisition age

### Defect

SoP and SoH primarily checked the age of the enclosing published measurement. A
fresh publication could therefore contain older cell or temperature values and
still satisfy the solver freshness gate. This is reachable for temperature data
because the 24 thermistors are sampled three at a time over eight 10 Hz scan
positions; healthy oldest temperature data is naturally about 0.7-0.8 s old.
Cell age also needed to be composed with elapsed time from the voltage-complete
reference to solve time.

### Fix

- SoP segment input now carries effective maximum cell and temperature ages.
- SoH input now carries effective maximum cell and temperature ages.
- `ams_power_state.c` composes stored age with elapsed time to the current solve
  timestamp, including wrap-safe arithmetic, unknown-age propagation, and
  conservative rejection of implausible future/ancient references.
- Cell age is referenced to `voltage_complete_tick`; temperature age is referenced
  to the coherent publication tick used when its stored age was recorded.
- The existing 250 ms measurement/cell freshness bound is retained.
- Temperature freshness uses a separate 1000 ms bound so the normal ~800 ms full
  thermistor scan remains usable while genuinely stale thermal data fails closed.

This is a freshness/authority correction, not an estimator/model retune.

## 4. Three negative profile gates could pass for an unrelated compile error

### Defect

Three expected-failure compile tests omitted `AMS_EXPECTED_PROFILE`. The tests did
currently fail for the intended safety `#error`, but if that real policy guard were
removed they could still fail because the test harness itself was malformed,
creating a false PASS.

### Fix

The three tests now define `AMS_EXPECTED_PROFILE=1` and grep the compiler log for
the exact intended safety diagnostic after the expected failure. An unrelated
compiler error no longer satisfies the gate.

## Regression coverage

The following host checks passed after the fixes:

- `make measurement-integrity-test` — includes effective cell/temperature age
  composition, healthy thermistor cadence, stale thermal rejection, unknown age,
  and tick-wrap cases.
- `make profile-gates` — expected-failure authority gates fail for the exact
  intended policy diagnostics.
- `make review-fixes-test` — 15/15 PASS, including headless profile/provenance
  contract and invalid-profile rejection.
- `make adbms-v05-contract-gate` — PASS, including no-backlog AUX2 scheduling.
- `make whole-source-analyze` — PASS for bench and vehicle authored code.
- `make power-core` — PASS, including SoP/SoH production core tests and topology
  guard.
- `make unit` — PASS.

No full MiL campaign was rerun: the patch does not retune estimator/SoP/SoH model
parameters or numerical model equations. The SoP/SoH acceptance behavior for
stale constituent data is intentionally changed and is covered by focused host
regression.

No ARM target build, flash, or new hardware validation is claimed in this
environment. Target artifact, timing, stack, peripheral, CAN, and physical-current
evidence remain release gates.
