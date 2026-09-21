# DER26 AMS v2.6.25 pre-Zephyr freeze hardening

**Date:** 2026-09-06
**Firmware:** 0.5.28
**Package:** v2.6.25
**Parent baseline:** v2.6.24 / firmware 0.5.27

This closeout fixes the final four issues found by the clean pass after v2.6.24. It does not change battery-model parameters, estimator equations, CAN IDs/encoding, or the approved SPI String-B PE4 mapping.

## Fixes

1. **DISCHARGE bus-off recovery now requires transport settlement before completion evidence is eligible.** The 500 ms supervisor proof rejects protected-generation completion markers until the old controller epoch has been fully settled, the post-recovery authority baseline has been moved past all pre-bus-off completions, and CAN transport flags are healthy. A delayed completion callback from the pre-bus-off epoch therefore cannot satisfy the 500 ms recovery rule if the 10 Hz CAN task itself is delayed. If transport settlement has not completed by 500 ms, BMS_OK fails low.
2. **Bench `fault inject canbusoff` uses the same event path as physical BOFF.** Physical and injected events share tick/state capture, authority revocation, immediate CHARGE/BALANCE/HIL fail-low policy, recovery-epoch creation, TX suspension, and mailbox abort behavior. Task polling remains responsible for normal counters, logging, and recovery settlement.
3. **Repeated identical soft CAN errors refresh their age.** A new ISR-observed ACK/bit/error occurrence refreshes `can_last_error_tick` even when the HAL error bits are identical to the previous occurrence. The soft hold therefore expires from the most recent observed error, not the first code transition.
4. **Failed manual CAN recovery no longer partially clears the repeated-bus-off state.** `tx_latched_inhibit` and its 10 s bus-off window are cleared only after controller/mailbox settlement succeeds. A failed service recovery leaves the existing transport policy state intact.

## Focused regression coverage

The host suite now covers:

- delayed CAN-task settlement plus a late pre-bus-off completion at the 500 ms decision;
- valid post-settlement completion at 499 ms, exact 500 ms rejection, 501 ms rejection, and tick wrap;
- physical-equivalent CHARGE and DISCHARGE `fault inject canbusoff` behavior;
- repeated identical ACK errors refreshing the soft-fault hold timer;
- failed service recovery preserving the repeated-bus-off TX latch/window.

Validation completed in the review environment:

- comprehensive `make test` — PASS;
- `make safety-test` with fault injection enabled — PASS;
- CAN scheduler/transport focused regressions — PASS;
- measurement-integrity tests — PASS;
- profile/review/CAN-contract gates — PASS;
- whole-source GCC `-fanalyzer -Wundef -Werror=undef` for bench and vehicle profiles — PASS.

A full `firmware-ci` invocation progressed through the main unit/system/CAN/APM gates but exceeded the execution-time limit while building a later logger-contract target; no failure was observed before timeout. No ARM target build, flash, physical CAN bus-off injection, watchdog timing measurement, or new hardware validation is claimed.
