# SoP Metamorphic CI Update — 2026-07-22

## Changes

- Added `AMS/host_tests/sop/sop_metamorphic_oracle.c`.
- Added the CI-gating `make power-metamorphic` target.
- Added `power-metamorphic` to the top-level `make ci` dependency list.
- Added a reduced 500-state-per-direction metamorphic run to the SoP
  AddressSanitizer/UndefinedBehaviorSanitizer target.
- Added `AMS/host_tests/sop/sop_sensitivity_probe.c` and the non-gating
  `make power-sensitivity` target.
- Added documentation to the host-test README and test matrix.

## Default CI campaign

The gate runs:

- 20,000 deterministic drive states;
- 20,000 deterministic charge states;
- 40,000 total base states;
- multiple stricter-state perturbations for every base state.

It verifies:

- discharge and charge horizon nesting;
- DCL cannot increase when the weakest measured cell voltage is reduced;
- CCL magnitude cannot increase when the strongest measured cell voltage is
  raised;
- DCL and CCL magnitude cannot increase with higher R0 and R0 uncertainty;
- DCL and CCL magnitude cannot increase with higher pack-current uncertainty;
- DCL cannot increase with higher core/surface temperature;
- DCL cannot increase with lower SoC;
- CCL magnitude cannot increase with higher SoC;
- tighter static current ceilings cannot increase capability;
- charging remains blocked below the configured minimum charge temperature;
- NaN, infinity, and stale measurements fail to zero.

The test prints the first failing seed and accepts an optional state count:

```bash
make power-metamorphic SOP_METAMORPHIC_STATES=1000
make power-metamorphic SOP_METAMORPHIC_STATES=100000
```

## Validation performed

The default 40,000-state gate passed:

```text
SoP metamorphic CI: 20000 drive + 20000 charge seeded states
  valid base solves: drive=20000 charge=20000; nonzero: DCL=20000 CCL=15833
  violations: baseD=0 baseC=0 nest=0 lowerV=0 higherV=0 higherR=0
              higherUnc=0 higherTemp=0 lowerSoC=0 higherSoC=0
              tightD=0 tightC=0 coldCCL=0 invalid=0
RESULT: PASS (all metamorphic and fail-zero properties held)
```

The 500-drive + 500-charge sanitizer campaign also passed with AddressSanitizer
and UndefinedBehaviorSanitizer enabled.

All pre-existing `make ci` component targets were run and passed individually,
including unit tests, SoP production tests, Python differential oracles, ECU
consumer tests, dashboard decoder tests, thermistor validation, comprehensive
system SIL, APM SIL, profile/topology/production gates, static-allocation and
ownership gates, and GCC whole-source analysis.

## Interpretation limit

This is a black-box metamorphic test, not an independent electrothermal
trajectory implementation. It strongly checks public-API sign behavior,
monotonicity, nesting, and fail-zero semantics. It does not establish that the
current numeric model parameters or static current limits match the installed
accumulator, fuse, harness, cooling system, or vehicle duty cycle.
