# Safety Model

## Authority must be earned continuously

The software safety model is fail-low: missing, stale, incoherent, or unqualified information removes permission rather than creating authority.

The hardwired shutdown system remains an independent safety layer. AMS firmware adds measurement supervision and software permission; it does not replace the electrical shutdown chain.

## BMS_OK ownership

The safety/error supervisor is the sole normal software owner allowed to assert BMS_OK. Other fault or panic paths may force BMS_OK low but must not independently reassert it.

Representative permission inputs include:

- cell-voltage validity, freshness, and fault state;
- temperature validity, freshness, and fault state;
- current validity, calibration/direction state, and diagnostics;
- ADBMS communication and diagnostic health;
- charger/IMD/fuse/AIR-related state where enabled;
- RTOS health and task diagnostics;
- build-profile restrictions and validation gates.

Bench/HIL profiles intentionally inhibit authority where required.

## Build profiles are part of the safety case

Safety should not depend on a technician remembering which runtime command not to use. Compile-time build profiles restrict bench/HIL/service behavior and gate authority-producing configurations.

Validation macros are evidence gates, not evidence by themselves. Enabling a validation macro without completing the corresponding physical qualification is not a release process.

## Panic/fault handling

Panic and fatal fault paths are expected to prioritize the physical fail-low action and preserve diagnostic information only to the extent that the runtime state remains trustworthy.

## What host tests cannot prove

Host tests cannot establish:

- target ISR latency or WCET;
- Cortex-M7 task-stack margins;
- real CAN controller/error timing;
- ADBMS/isoSPI electrical integrity;
- current-sensor polarity/calibration;
- thermistor calibration and harness behavior;
- watchdog/power-cycle behavior;
- EMI/grounding robustness;
- contactor or mechanical response.

Those remain target, bench, HIL, and vehicle validation responsibilities.
