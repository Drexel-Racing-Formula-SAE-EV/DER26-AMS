# AIR Contactor Monitoring Contract

## Current hardware and firmware status

- `AIR_CONTROL_MCU` is a divided sense of voltage on the common `AIR_CONTROL`
  net. It is **not** physical AIR+, AIR- or precharge contact feedback.
- With the feature disabled, the legacy `air_task` path only rereads that
  signal. It is deliberately not started because the supervisor already
  samples it. The feature-enabled path is a separate monitored producer.
- The current PCB has no `AIR_POS_AUX`, `AIR_NEG_AUX` or `PRECHARGE_AUX`
  inputs. Therefore `AMS_ENABLE_AIR_AUX_FEEDBACK` defaults to `0`.
- With the feature disabled, auxiliary feedback does not participate in the
  `BMS_OK` gate and is reported as unavailable, never as healthy.
- If the feature is enabled before a validated producer publishes feedback,
  the initialized `ams_air_monitor_t` is faulted and the supervisor holds
  `BMS_OK` low.
- `air_monitor.c` contains the hardware-independent evaluator. It accepts
  classified samples and an explicit configuration; it does not read GPIOs or
  embed unreviewed contactor timing constants.

The hardwired shutdown circuit remains the primary authority. Firmware
monitoring is diagnostic supervision and an additional permission gate; it
must not be the only mechanism that opens the contactors for a critical fault.

## Required future hardware contract

- Manufacturer-rated mirror/auxiliary contact for AIR+ and AIR-.
- Prefer a precharge-relay auxiliary contact where available.
- Protected `AIR_POS_AUX` and `AIR_NEG_AUX` inputs, plus optional
  `PRECHARGE_AUX`.
- Defined inactive levels, input series resistance, RC filtering, transient/ESD
  protection, and reviewed MCU voltage thresholds.
- Harness and connector contacts for each feedback channel.
- Prefer end-of-line supervision and ADC windows if open/short harness
  detection is required. A plain GPIO cannot distinguish an intentionally open
  contact from a broken conductor.
- Load-side/DC-link voltage sensing in addition to pack-side voltage. An
  auxiliary contact alone does not prove the main HV contacts conducted or
  opened.
- Test points or a safe low-voltage injection connector for each conditioned
  feedback channel.

The input polarity, mirror-contact truth table, line-supervision windows and
open/close timing limits must be taken from the selected contactor, driver and
harness design. Do not invent generic timing constants in firmware.

## Command-state authority

Feedback can only be judged against an authoritative commanded state. The
current single `AIR_CONTROL_MCU` signal is insufficient to reconstruct the
individual AIR+, AIR- and precharge commands.

Before enabling supervision, choose and document exactly one command owner:

1. If another controller owns sequencing, provide AMS with a fresh, CRC/
   counter-protected desired-state message or individual command-sense inputs.
2. If AMS owns sequencing in a future design, implement an explicit contactor
   state machine and protected coil-driver outputs.

Loss of command freshness must produce an unknown/shutdown expectation, never
reuse the last run command indefinitely.

The evaluator input is the **effective hardware phase**, after the local
shutdown permission is applied—not just an ECU's requested phase. If BMS_OK or
the hardwired shutdown circuit is forcing the contactors open, the adapter must
report Shutdown/Off even if a remote request still says Run. Keep the remote
request separately for diagnostics. This prevents expected contact opening
during a safety trip from being misclassified as a failed-close command.

## Expected steady-state table

| Phase | AIR- expected | AIR+ expected | Precharge expected |
|---|---|---|---|
| Off | Open | Open | Open |
| Precharge | Closed | Open | Closed; load-side bus rising |
| Run | Closed | Closed | Open |
| Shutdown | Open after qualified release time | Open after qualified release time | Open |

Transitions must have separate make and release deadlines based on the actual
parts. A transition may be pending during its allowed interval, but it is not a
validated steady state and must not silently satisfy a run-ready gate.

## Authoritative firmware evaluation

The future input producer may use GPIO polling, ADC line supervision and EXTI
edge timestamps, but it should publish one coherent snapshot. The safety
supervisor performs the authoritative evaluation.

At minimum, the snapshot must contain:

- feature/configuration validity;
- commanded phase and command-source freshness;
- debounced AIR+, AIR- and precharge contact states;
- raw or classified line-supervision state;
- last update and last edge timestamps;
- transition start time and active deadline;
- pack voltage and load-side voltage validity/value;
- current fault reason and latched fault reason.

EXTI is optional acceleration/diagnostics. An interrupt edge alone is not proof
that the contact stayed in the required state; task-level sampling must confirm
the steady state.

## Implemented evaluator interface

The pure evaluator is implemented by:

```c
bool ams_air_monitor_config_valid(const ams_air_monitor_config_t *config);

void ams_air_monitor_step(ams_air_monitor_t *monitor,
                          const ams_air_monitor_config_t *config,
                          const ams_air_monitor_inputs_t *inputs);

bool ams_air_monitor_request_clear(ams_air_monitor_t *monitor,
                                   const ams_air_monitor_config_t *config,
                                   const ams_air_monitor_inputs_t *inputs);
```

`ams_air_monitor_inputs_t` requires independently timestamped command, AIR+,
AIR-, optional precharge, pack-voltage and load-voltage samples. All age checks
use unsigned tick subtraction and are tested across the 32-bit tick rollover.

`ams_air_monitor_config_t` requires the board integration to provide:

- command, contact-sample and voltage-sample freshness limits;
- contact debounce time;
- separate AIR+/AIR-/precharge make and release limits;
- maximum permitted precharge duration;
- Run voltage-settle and open-bus discharge limits;
- minimum valid pack voltage and maximum open-state load voltage;
- precharge-complete and Run bus-ratio windows;
- whether precharge auxiliary and bus-voltage proof are required.
- a separately reviewed `AMS_AIR_MONITOR_PERIOD_MS` and
  `AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS` for task-liveness supervision.

There are deliberately no target defaults. A zeroed configuration is rejected,
and every configured interval is bounded to at most `INT32_MAX` milliseconds so
tick-wrap comparisons remain unambiguous.

The evaluator exposes separate meanings that must not be collapsed:

| Field | Meaning |
|---|---|
| `command_valid` | The desired phase is recognized and its source is fresh |
| `feedback_valid` | Required contact samples are fresh and debounced |
| `voltage_valid` | Required voltage samples are fresh and the pack input is plausible |
| `transition_pending` | A permitted state transition has not reached steady state |
| `steady_state_valid` | Contacts and required voltage proof match the present phase |
| `permit` | No active/latched fault prevents the commanded transition from continuing |
| `fault_latched` | A persistent fault requires an explicit controlled-clear request |

This separation avoids a control deadlock. During an authorized close
transition, a contact may legitimately be moving inside its make deadline.
That is `transition_pending`, not yet a failure. Dropping the shutdown-circuit
permission solely because the contact has not instantaneously closed could
remove coil power and prevent the transition from ever completing. Initial
boot is different: no close transition is authorized until an all-open state
has first been physically verified.

Opening is intentionally asymmetric. A transition to Off or Shutdown does not
inherit permission from Run; permission remains low until all required contacts
and the load-side bus have reached their verified open state. This prevents a
recovering transient fault from reasserting BMS_OK midway through opening.

The implemented transition policy is:

- initial boot must establish Off/Shutdown with all required contacts open;
- Off/Shutdown -> Precharge is allowed only after that boot-open proof;
- Precharge -> Run is allowed only after the Precharge contact state is steady
  and, when bus-voltage proof is configured, the completion ratio is met;
- a transition to Off or Shutdown is always allowed;
- direct Off -> Run and Run -> Precharge bypasses are rejected and latched.

Contact mismatches are evaluated against per-contact make/release deadlines.
Multiple simultaneous failures are preserved in `active_fault_mask` and
`latched_fault_mask`; `reason` is the primary human-readable reason and does
not discard the additional mask bits.

The feature-enabled task records `AIR_FAULT_LATCH` in the retained fault ring
only when new latch bits appear. The event stores the primary reason plus the
active and latched masks, avoiding one duplicate log entry per task cycle.

The safety supervisor independently checks the age of `last_update_tick`.
Consequently, a frozen AIR task cannot leave its last healthy snapshot valid
forever. This publication timeout is separate from the command, contact and
voltage sample timeouts evaluated inside the AIR task.

## Minimum fault policy

| Observation | Interpretation/action |
|---|---|
| Command open, AUX remains closed after release deadline | Possible welded/stuck contact; latch fault and keep `BMS_OK` low |
| Command closed, AUX remains open after make deadline | Coil, driver, harness or contactor failure; latch/inhibit |
| AIR+ and AIR- disagree outside an allowed transition | Contact mismatch; latch/inhibit |
| Precharge AUX closed in Run or Off | Precharge contact possible weld; latch/inhibit |
| Feedback missing, stale or line-faulted | Invalid; keep `BMS_OK` low when feature is enabled |
| AUX says open but load-side voltage remains high | Possible welded main contact or external backfeed; latch/inhibit |
| AUX says closed but expected bus response is absent | Main contact not conducting, voltage-sense fault or wiring problem; inhibit |
| Command source becomes stale | Enter shutdown/unknown expectation and inhibit |

Faults that imply welding or failed opening require a controlled reset and a
verified all-open state before clearing. A software reset by itself must not
clear evidence of a physically closed contact.

`ams_air_monitor_request_clear()` implements that restriction. It re-evaluates
the supplied samples and refuses to clear unless:

- command, feedback and required voltage samples are fresh;
- the commanded phase is Off or Shutdown;
- AIR+ and AIR- are stably open;
- precharge is stably open when its auxiliary input is required;
- no active fault remains;
- the load-side bus is below the reviewed open-state threshold when voltage
  proof is required.

## Precharge validation

Auxiliary contacts are not sufficient to validate precharge. The future state
machine should also check:

- valid pack-side and load-side/DC-link voltage measurements;
- plausible voltage rise and direction;
- completion threshold derived from the inverter/DC-link and precharge design;
- maximum precharge time derived from R/C tolerances;
- excessive current and unexpected bus voltage before precharge starts;
- AIR+ closure only after successful precharge;
- precharge relay opening after AIR+ is verified closed.

Exact ratios, current limits and timeouts remain design inputs and must not be
filled with generic placeholder values.

## Integration sequence

1. Review the new schematic/harness and selected contactor data sheets.
2. Assign pins and confirm polarity/line-supervision thresholds.
3. Implement a single board-specific raw-input producer. It must classify the
   protected inputs, timestamp every sample and publish the command source; it
   must not write `ams_air_monitor_t` fields individually.
4. Populate `ams_air_monitor_config_t` from reviewed component and system
   values, then call the existing pure evaluator from one task owner.
5. Add CLI and versioned CAN diagnostics without renaming command sense as
   physical state.
6. Run SIL fault injection for every state, edge, timeout, stale source and
   32-bit tick rollover.
7. Perform current-limited low-voltage HIL with contactor coils or simulators;
   keep HV and `BMS_OK` physically inhibited.
8. Validate pack/load voltage plausibility and real auxiliary timing.
9. Measure the AIR task stack high-water mark and worst-case scheduling jitter
   in the target build; confirm both fit the reviewed period/deadlines.
10. Only then set `AMS_ENABLE_AIR_AUX_FEEDBACK=1` in a vehicle target profile.

## Board-adapter stub boundary

The current-hardware legacy path in `air_task.c` remains deliberately unstarted
because it only samples `AIR_CONTROL_MCU`. The feature-enabled path is already
implemented around two fail-closed board hooks. When the hardware exists,
implement those hooks with this conceptual flow:

```c
read protected inputs / ADC windows
    -> classify OPEN, CLOSED, UNKNOWN or LINE_FAULT
read fresh individual AIR command phase
read pack and load-side voltage snapshots
    -> fill one local ams_air_monitor_inputs_t
copy the previous monitor state inside a short critical section
ams_air_monitor_step(&local_monitor, &reviewed_config, &inputs)
publish local_monitor inside a short critical section
```

The supplied feature-enabled task already follows that local compute / atomic
publish pattern through two weak board hooks. The hardware revision must supply
strong implementations of `ams_air_board_get_config()` and
`ams_air_board_read_inputs()`, set `AMS_AIR_AUX_BOARD_ADAPTER_READY=1`, and set
reviewed nonzero `AMS_AIR_MONITOR_PERIOD_MS` and
`AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS` values. A target build refuses to
enable the feature without those declarations.

The producer must build the input object locally and call the evaluator once.
The high-priority safety supervisor remains the sole normal owner of `BMS_OK`.
No ISR should directly assert `BMS_OK`; EXTI may only timestamp/wake and may
force an already-authorized output low through the existing panic-safe path.
