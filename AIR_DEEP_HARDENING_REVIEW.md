# AIR Contactor Monitor Deep Hardening Review

Date: 2026-07-17

Scope: AIR auxiliary-contact monitor, AIR task boundary, safety-supervisor
integration, host tests and documentation

Submitted package SHA-256:
`c5babdce1c32cd628847502c45ac783c29d1de968b06998bee9212374ff4daf6`

## Outcome

The AIR logic is materially stronger and all available host verification passes.
The feature remains intentionally disabled and fail-closed on the present board:
the current hardware does not provide validated `AIR_POS_AUX`, `AIR_NEG_AUX`,
optional `PRECHARGE_AUX`, or the reviewed board adapter required to enable it.

This review does **not** approve HV operation or establish that a physical
contactor will open. The hardwired shutdown circuit remains the primary safety
mechanism; firmware monitoring is diagnostic and an additional permission gate.

## Findings corrected

### 1. Shutdown could automatically re-arm

The submitted evaluator could reach a fully open steady `SHUTDOWN` state and
restore `permit`. That could allow the supervisor to reassert `BMS_OK` without
an explicit re-arm phase.

`SHUTDOWN` is now terminal and non-permitting. A clear in `SHUTDOWN` may erase
latched evidence, but the command owner must explicitly transition through
`OFF` and the monitor must re-prove the fresh open/bus-safe state before another
close sequence.

### 2. Closing transitions inherited incomplete authority

The old transition path derived prior permission from only `permit` and
`fault`. It did not require the complete coherent ready predicate. A malformed
or internally inconsistent snapshot could therefore carry permission through a
close request.

Closing authority now comes only from `ams_air_monitor_ready()`, including
valid configuration/command/feedback, boot-open proof, phase, transition state,
permit, fault booleans and both fault masks.

### 3. Precharge-to-Run could use stale proof

The `PRECHARGE -> RUN` decision trusted the prior cycle's
`precharge_complete`. If the load-side voltage collapsed in the current
transition snapshot, the transition could retain permission during the Run
settling window.

The transition now requires both the prior steady proof and fresh current
pack/load voltage proof, including the configured minimum completion and upper
plausibility ratios.

### 4. Energized supervision loss could recover automatically

Configuration, command, contact or voltage freshness loss in `PRECHARGE`/`RUN`
could drop permission and then automatically recover on the same phase. By that
time the hardwired circuit may already be opening.

After boot-open is established, a supervision loss in an energized phase now
latches. Recovered data cannot reassert permission until the system reaches a
verified open state and completes the controlled clear/re-arm sequence. A
transient loss while already in `OFF` remains recoverable.

### 5. Individually fresh samples could be mutually stale

Command, contact and voltage samples were aged independently. A mixture of old
and new—but individually in-timeout—samples could form one inconsistent safety
decision.

The configuration now includes `max_sample_skew_ms`. Required samples whose
timestamps exceed that bound raise `AMS_AIR_FAULT_SAMPLE_INCOHERENT`; the fault
latches if discovered in `PRECHARGE` or `RUN`.

### 6. Task schedule was not proven against physical deadlines

The original build checked only that publication timeout was not shorter than
the task period. It did not show that evaluation/publication could occur before
command, input, contact, precharge or bus deadlines.

`ams_air_monitor_schedule_valid()` now requires the evaluation period and
publication timeout to fit inside every applicable safety deadline, including
open-bus discharge. It also proves that periodic sampling can observe a raw
contact edge and complete the configured debounce before every make/release
deadline. Arithmetic uses a 64-bit intermediate.

### 7. Board configuration failure was permanent

The AIR task requested configuration once before its loop. A transient startup
ordering failure permanently left the task without configuration.

The task now retries only until the first valid, schedule-compatible
configuration is accepted, then holds it immutable. Runtime threshold changes
are not accepted silently.

### 8. Partial board reads could leave plausible fields

The return value of `ams_air_board_read_inputs()` was ignored. A future adapter
could partially populate an input object and report failure without the task
invalidating the object.

The hook is now explicitly all-or-nothing. A false result clears the entire
input object, and the task overwrites `now_tick` with scheduler time so the
adapter cannot control the evaluator's time base.

### 9. Ready and supervisor checks trusted duplicated booleans

The ready helper did not independently require boot-open, an operating phase,
authorized transition state, or zero masks. The supervisor hard-fault helper
also trusted the summarized fault booleans.

The ready predicate now checks the complete published state. The supervisor
also treats either nonzero active or latched mask as a hard fault, even if a
boolean is inconsistent.

### 10. AIR production code was not tested across a translation-unit boundary

The comprehensive harness directly included `air_monitor.c`, which could hide
linkage or object-boundary mistakes.

The AIR evaluator is now linked as a separate production translation unit in
normal, production-gate, APM, sanitizer and stress builds.

## Behavioral contract after hardening

| From | Allowed destinations |
| --- | --- |
| `OFF` | `OFF`, `PRECHARGE`, `SHUTDOWN` |
| `PRECHARGE` | `OFF`, `PRECHARGE`, `RUN`, `SHUTDOWN` |
| `RUN` | `OFF`, `RUN`, `SHUTDOWN` |
| `SHUTDOWN` | `OFF`, `SHUTDOWN` |

- Initial permission requires a fresh, debounced, all-open and—when enabled—
  discharged-bus proof.
- Direct `OFF -> RUN`, `RUN -> PRECHARGE`, and `SHUTDOWN -> PRECHARGE/RUN`
  requests latch a sequencing fault.
- An authorized close may retain existing shutdown-circuit permission only
  inside its reviewed transition deadline.
- An opening request never retains AIR-monitor permission.
- `SHUTDOWN` never permits; `OFF` is the explicit re-arm boundary.
- Fault clear requires fresh inputs, stable open feedback, a safe load-side bus
  when required, and `OFF` or `SHUTDOWN`.

## Verification completed

All of the following passed against the final source:

- complete `make ci` run: unit tests, comprehensive SIL/injection tests,
  ADBMS2950 unit/SIL, production feature gates, BMS_OK sole-owner test and GCC
  `-fanalyzer`;
- all 16 edges of the four-phase AIR transition graph;
- shutdown re-arm, current-snapshot precharge proof, stale/incoherent input,
  inconsistent snapshot/mask and exact schedule/deadline regressions;
- focused AIR enablement/fail-closed gate and feature-enabled `-Werror` syntax
  build of `app.c`, `air_task.c` and `air_monitor.c`;
- AddressSanitizer plus UndefinedBehaviorSanitizer;
- standalone UndefinedBehaviorSanitizer;
- 50,000-cycle deterministic fuzz/stress and 12,000-cycle concurrent stress;
- hardware-bring-up, CAN-fed ADBMS HIL, IMD-enabled and IWDG-enabled host
  profiles;
- standalone AIR evaluator compile with `-Wall -Wextra -Werror -Wconversion
  -Wsign-conversion -Wshadow -Wundef -Wstrict-prototypes
  -Wmissing-prototypes -pedantic`.

No test energized hardware or asserted that physical wiring exists.

## Required hardware and target exit gates

The following remain deliberately incomplete:

1. Add protected, defined-polarity `AIR_POS_AUX` and `AIR_NEG_AUX` inputs;
   preferably add `PRECHARGE_AUX` and supervised/open-wire detection.
2. Add or identify independent pack-side and load-side voltage measurements if
   voltage proof will be required. An auxiliary contact alone does not prove
   that the HV power contacts opened.
3. Select pull direction, line thresholds, RC time constants, ESD/transient
   protection, harness behavior, contact polarity and fail state from released
   schematics and component data.
4. Derive debounce, make/release, precharge, bus-discharge and ratio thresholds
   from actual contactors, precharge design and measured worst-case timing.
5. Implement reviewed strong `ams_air_board_get_config()` and
   `ams_air_board_read_inputs()` adapters. The command sample must describe the
   effective hardware phase after local shutdown permissions, not merely a
   remote requested state.
6. Set `AMS_AIR_AUX_BOARD_ADAPTER_READY=1`, a nonzero reviewed monitor period,
   and a publication timeout only after the schedule validator accepts the
   complete configuration.
7. Route clear requests through the AIR task/single writer. The pure clear API
   is intentionally not exposed as an arbitrary CLI write.
8. Build and link with the approved ARM toolchain; inspect warnings, map/size,
   stack margin, task jitter and ISR latency. ARM GCC, Clang, cppcheck, CBMC and
   Frama-C were unavailable in this workspace.
9. Perform current-limited LV contactor HIL: delayed/missing edges, bounce,
   welded AUX, broken/shorted harness, voltage disagreement, reset/brownout,
   task starvation and tick wrap. Verify physical BMS_OK behavior independently.
10. Retain the hardwired shutdown path as the primary response. Do not advance
    to HV testing solely because the host suite passes.
