# Bruin Formula MK11 BMS Reference Review

Reviewed 2026-07-18 by Mahad Faisal with OpenAI Codex assistance.

## Reference boundary

Public reference: <https://github.com/bruinformula/mk11-bms-mcu>

Pinned revision: `037caccbf823c92f16ad86dcd192134adcc011f0`
(`ALL 4 DYNAMIC EVENTS PASSED`, 2026-06-20).

Passing all four competition dynamic events is useful vehicle-level evidence,
especially for operational sequencing. It is not by itself a software-safety
proof. The reviewed revision has no root license, no visible automated host
test/CI suite, commits generated Debug objects/maps/ELFs, uses dynamic RTOS
objects and infinite mutex waits, directly controls contactors in firmware,
and intentionally disables its overcurrent and IC-disconnect fault macros.
No Bruin source code or team-specific threshold was copied into DER26.

DER26 also has a different system boundary: present precharge and contactor
sequencing are hardware-owned. Bruin's firmware-controlled precharge and AIR
GPIO sequence therefore cannot be transplanted safely.

## Pattern comparison

| Bruin pattern | DER26 decision | Reason |
|---|---|---|
| Explicit mode entry/exit cleanup | Adopted as a centralized guarded state-transition boundary | Cleanup and audit behavior should not depend on which task changed the state. |
| Immediate charger-disable command on charge exit | Adopted and strengthened | DER26 previously zeroed only local targets after leaving charge and sent no disable frame. |
| One global byte fault register | Not copied | DER26's typed fault policies, latched reasons, freshness state and coherent supervisor evaluation carry more diagnostic and safety context. |
| Firmware precharge/contactors | Not applicable | Current DER26 hardware owns precharge and the shutdown loop; firmware has no validated AIR command/position authority. |
| Dynamic tasks/mutexes and infinite waits | Rejected | DER26 already uses static allocation, bounded ADBMS waits, task heartbeats and optional watchdog supervision. |
| Competition-tuned voltage/current thresholds and disabled faults | Rejected | Thresholds and enabled safety mechanisms must come from DER26 cell, sensor, rules and hardware validation. |
| Direct CAN HAL changes from mode functions | Rejected | DER26 keeps CAN start/filter/notification/recovery ownership inside the CAN service. |

## DER26 changes resulting from the comparison

### Guarded state transitions

- All runtime state writes now pass through `ams_state_transition_begin()`.
- The transition publishes previous state, current state, reason, tick and a
  saturating transition counter.
- Invalid requested states and corrupted current states are contained as
  `STATE_ERROR`.
- `state_transition_in_progress` blocks the supervisor while synchronous
  balance cleanup is running. If cleanup blocks or fails, BMS_OK cannot be
  reasserted by a higher-priority task.
- Each applied transition is written to the retained fault-event ring.
- `make state-ownership-gate` rejects direct state writers outside the
  initialization/transition boundary.

| Authority | Allowed transition |
|---|---|
| Startup supervisor | `START -> DISCHARGE` only, after every software safety input is ready |
| Service CLI build | `START/CHARGE/DISCHARGE/BALANCE -> CHARGE or DISCHARGE` |
| Corruption containment | Invalid current/target state -> `ERROR` |
| `NULL` or `ERROR` recovery | No runtime service transition; reset/controlled future recovery policy required |

### Charger exit behavior

- Leaving `STATE_CHARGE` schedules three zero-voltage, zero-current disable
  frames (`00 00 00 00 01`) through the CAN task.
- These frames are attempted before telemetry and before any possible charger
  enable command.
- A failed HAL queue operation consumes no retry, latches the charger fault,
  drops BMS_OK and is retried.
- A rapid re-entry to charge cannot place an enable frame behind a pending
  exit-disable frame in the same CAN task iteration.
- Counters and CLI status expose requests, successful frames, failures,
  remaining frames and last HAL status.
- CAN control HAL calls were removed from the state CLI; CAN transport control
  remains single-owner and is enforced by CI.

Three queued frames reduce exposure to one lost/arbitrated command, but they
are not a charger acknowledgement. Hardware testing must still verify the
charger's command watchdog, response behavior and actual output shutdown.

## Regression coverage

- Same-state cleanup with every ordinary readiness gate healthy cannot
  reassert BMS_OK until transition finish.
- Charge-to-drive transition invalidates BMS permission until current policy
  refresh and charger shutdown completion.
- Corrupt state values normalize to ERROR and request conservative charger
  shutdown.
- Charger exit emits exactly three prioritized zero-demand disable frames.
- Failed first transmission leaves all three frames pending and retains the
  charger/BMS fault; a later success resumes the burst.
- State transition events survive the retained-log CRC/commit validation.

## Hardware/target work still required

1. Cross-build and link with the exact STM32 ARM toolchain and inspect the map,
   stack high-water marks and task timing.
2. On an isolated LV CAN bench, sniff charge enable, state exit and all three
   disable frames. Inject mailbox saturation and bus-off during the exit.
3. Confirm the charger actually disables output on the first valid disable
   command and independently times out when commands stop.
4. Confirm drive entry policy with the real shutdown/precharge hardware;
   firmware still does not own or prove physical precharge completion.
5. Keep BMS_OK physically inhibited during first target validation.
