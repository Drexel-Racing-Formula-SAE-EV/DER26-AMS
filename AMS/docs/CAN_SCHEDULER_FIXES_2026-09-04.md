# CAN scheduler fixes — package v2.6.17

AMS source revision: `DER26-AMS-v0.5.20-20260904`.

Baseline: `DER26-AMS-MiL-v2.6.16-five-SMB-passive-ring-observer-2026-09-03(1).zip`,
SHA-256 `6baeb59d86d0d341d00a9ca6bb1087bfda8f50aeedb88cf680f40c540f257421`.

## Changes

| Reviewed defect | Correction |
|---|---|
| Full detail generation allocated on the CAN task stack | Publish directly into the serialized scheduler slot. No full-generation local or extra staging allocation. |
| Mailbox ownership overwritten before completion dispatch | Choose a fixed mailbox only when software ownership is FREE, hardware TME is set, and RQCP is clear. CAN1 TX/RX0/SCE dispatch through a wrapper that refills after HAL callbacks finish. |
| Hardware bus-off treated as recovered while HAL reports LISTENING | Check ESR.BOFF and settle hardware requests, completion flags, software ownership, and any active pump before advancing the epoch. Suspend and request aborts at the error ISR. |
| Shutdown compared against an unrelated generation counter | Every task-side critical publication uses the transport generation source. A separate request ID follows the frame into mailbox completion accounting. New pending commands win by publication order, and obsolete loaded commands receive abort requests. |
| Required completion checks array positions | Count and check PROTECTED_REQUIRED frames throughout the generation, including 0x687 and excluding advisory 0x68B. Reject mismatched required counts. |
| Fast tuning overwrites pending base telemetry | Mark tuning generations. Shed fast tuning when a base snapshot is pending; base telemetry can replace pending tuning or an older base snapshot. |

Recovery deliberately drops preserved pre-fault charger/protected payloads.
It keeps TX suspended until the CAN task has published fresh charger decisions
and a protected bundle. The charger cadence is forced due on that refresh
cycle. This prevents immediate retransmission of stale enable/authority data.

The same ownership path also retires terminal TX errors for which HAL issues
only an error callback. Failed loads of an obsolete reserved frame discard it
instead of making it eligible for retry. A latched repeated-bus-off condition
does not repeatedly increment the recovery counter.

The fixed-mailbox write uses the bundled STM32F7 HAL register encoding. HAL
continues to own setup, RX, notification dispatch, aborts and error handling.
Wire IDs, payload layouts, 1-Mbit/s timing and nominal publication rates are
unchanged. The extra shutdown request identity exists only in software.

`can diag` adds `CAN tuning shed`, `recovery_wait` and `refresh_wait` counters/
state. The logger contract now describes all phases being published together
every 500 ms, matching the implementation.

## Focused verification

`make -C AMS/host_tests can-tx-regression-test` passes ten new focused tests:

1. Interleaved required/advisory completion and required-count validation.
2. Critical publication order across reset, including failed obsolete reservations.
3. Base snapshot survival under tuning replacement and full 192-frame capacity.
4. Coalesced completions and task pumping before IRQ dispatch.
5. A completion arriving during task-side mailbox selection; register encoding.
6. Terminal transmit error retirement.
7. RX/SCE dispatch while a task has masked TME for a load/abort transaction.
8. Hardware BOFF/ownership settlement and fresh publication before restart.
9. Repeated-bus-off latch and one recovery count per event.
10. Shutdown identity, loaded-command supersession and completion winning an abort race.

The register-backed mock uses the production transport/scheduler and models
the bundled HAL's cached-TSR callback order. It does not execute a physical
bxCAN peripheral or prove interrupt latency.

The changed C translation units pass syntax checks using the five-SMB
`AMS_BUILD_PROFILE=5` configuration with `AMS_HOST_TEST=1` and with target code
paths enabled (`AMS_HOST_TEST=0`). The updated CAN IRQ routing check passes.
No existing MiL campaign, existing unit/stress suite or broad CI run was repeated.

Host compiler stack reports for detail publication:

| Optimization | Publication helper frame | Public wrapper frame |
|---|---:|---:|
| -O0 | 64 bytes | 48 bytes |
| -Os | 32 bytes | 8 bytes |
| -O2 | 48 bytes | 8 bytes |

These are individual host stack frames, excluding callees; they are not a
Cortex-M7 stack high-water measurement. The previous full-generation local
was already larger than the entire 6,144-byte CAN task stack.

## Package and target status

All v2.6.16 source files are retained, including CubeIDE project settings and
the five-SMB bench observer. MiL, EKF, SoH, SoP, fuse and sensor algorithms are
unchanged. The package contains source and focused tests, without new target
binaries or transient host build outputs.

An ARM cross-compiler was unavailable in this environment. A full STM32 build,
flash and physical CAN test were not performed. Rebuild the selected CubeIDE
configuration and confirm source revision `DER26-AMS-v0.5.20-20260904` after
flashing. A bench CAN run must still establish actual completion latency,
stack margin and recovery behavior on the connected bus.
