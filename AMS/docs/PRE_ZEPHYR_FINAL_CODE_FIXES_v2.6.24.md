# DER26 AMS v2.6.24 final pre-Zephyr code fixes

**Date:** 2026-09-05
**Firmware:** 0.5.27
**Package:** v2.6.24
**Baseline:** v2.6.23 / firmware 0.5.26

This closeout fixes the four defects found in the final clean pass before the Zephyr
migration. It does not start the Zephyr port, retune battery models, change estimator
equations, change CAN IDs/scheduler priority policy, or change the approved PE4
String-B SPI chip-select mapping.

## Fixed findings

1. **Physical CAN bus-off event timing/state classification**
   - `HAL_CAN_ErrorCallback()` records the first BOFF event tick and operating state.
   - DISCHARGE's 500 ms epoch begins at that physical interrupt, not the later 10 Hz
     task poll.
   - CHARGE, BALANCE, and CAN-backed HIL perform ISR-time physical fail-low.
   - Task-context polling performs counters, retained logging, charger flags, repeated
     bus-off accounting, and ABOM recovery using the ISR-captured event epoch.

2. **499/500/501 ms protected-generation completion race**
   - The CAN scheduler records `protected_required_last_complete_tick` together with the
     generation identity when all required protected frames actually complete on wire.
   - The safety supervisor checks actual completion event time at the 500 ms boundary.
   - The CAN task rejects a completion at or after 500 ms from closing a physical
     DISCHARGE recovery epoch even if it observes that completion before the supervisor.
   - Regression covers 499 ms accepted, 500 ms rejected, 501 ms rejected, and 32-bit
     tick wrap.

3. **Service `can recover` CHARGE/BALANCE re-latch**
   - Manual recovery no longer sets `can_busoff_recovery_active`.
   - A distinct `can_authority_refresh_pending` state keeps BMS authority low until a
     fresh required protected generation completes on wire without starting a new
     physical bus-off timer or causing CHARGE/BALANCE to re-latch immediately.

4. **Estimator tuning-CAN producer compiled out**
   - `AMS_ENABLE_TUNING_CAN` is now defined in common `app.h`, not only in the logger
     header.
   - Estimator producer and CAN consumer therefore compile from the same feature gate.
   - `whole-source-analyze` now uses `-Wundef -Werror=undef` in bench and vehicle
     profiles.
   - A host runtime gate verifies the estimator actually publishes a tuning snapshot at
     supported CAN bitrates.

## Verification

Passed in the supplied host environment:

- `make firmware-ci`
- comprehensive `make test` and IWDG-enabled safety coverage
- `make can-tx-scheduler-test`
- `make can-tx-regression-test`
- `make measurement-integrity-test`
- `make profile-gates`
- `make review-fixes-test` — 15/15
- `make can-v4-contract-gate`
- `make can-irq-contract-gate`
- `make adbms-v05-contract-gate`
- `make whole-source-analyze` with undefined-preprocessor identifiers promoted to errors
- focused Clang static analysis on `canbus.c`, `can_tx_scheduler.c`, `canbus_task.c`,
  `error_task.c`, and `estimator_task.c`

The broad `make clang-analyze` target exceeded the execution window; the focused
analysis of every production source file changed by this closeout completed cleanly.

## Not claimed

No ARM target build/flash, physical CAN bus-off injection, measured ISR latency, CAN
wire timing, watchdog timing, or other hardware qualification is claimed here. Those are
separate target-validation gates and are intentionally outside this code-only closeout.
