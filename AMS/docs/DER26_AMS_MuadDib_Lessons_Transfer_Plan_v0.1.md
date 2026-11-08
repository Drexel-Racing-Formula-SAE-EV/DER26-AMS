# DER26 AMS — Muad’Dib Lessons Transfer Plan

**Plan version:** 0.1
**Date:** 2026-09-16
**Current FreeRTOS reference package:** DER26-AMS-MiL v2.6.27
**Current AMS source:** v0.5.30 (`DER26-AMS-v0.5.30-20260906`)
**Target MCU:** STM32F767ZIT6
**Current RTOS:** FreeRTOS/CMSIS-RTOS2
**Planned RTOS:** Zephyr
**Primary objective:** transfer the most valuable real-hardware reliability lessons from the Rheem Muad’Dib inverter project into DER26 AMS without destabilizing the frozen FreeRTOS behavioral baseline or turning the Zephyr migration into a simultaneous architecture redesign.

---

## 1. Executive decision

The most useful Muad’Dib lessons for DER26 are **not the induction-motor FOC equations**. The transferable value is the engineering discipline around a safety-relevant embedded controller that has to survive noisy hardware, stale commands, stalled peripherals, timing faults, and failures that disappear when a debugger is attached.

The DER26 AMS already contains many of those ideas in mature form:

- checked ADC conversion with a finite timeout;
- immutable, timestamped measurement snapshots;
- per-reading freshness rather than only whole-packet freshness;
- explicit CAN authority and stale-data rules;
- state-dependent CAN bus-off policy;
- ISR-time fault timestamps for events where task-observation time is too late;
- a watchdog policy tied to software liveness/RTOS integrity rather than ordinary battery faults;
- retained panic and fault logs;
- calibration and build provenance;
- MiL/HiL/fault-injection infrastructure;
- fail-low BMS ownership and compile-time authority profiles.

The plan therefore has two goals:

1. **Formalize and complete the reliability patterns that are already present.**
2. **Add one major observability capability that the present AMS still lacks: a pre/post-fault flight recorder that turns track incidents into reproducible engineering evidence.**

The recommended implementation order is:

1. freeze and validate v2.6.27 as the comparison oracle;
2. add a portable flight-recorder core on a separate feature branch;
3. complete a bounded-hardware-transaction/deadline audit across all peripherals;
4. add an incident-to-regression replay pipeline;
5. unify runtime configuration/calibration provenance into a compact manifest;
6. preserve these mechanisms through Zephyr migration;
7. only after Zephyr parity, evaluate more invasive changes such as timer-triggered current acquisition/DMA and a hardware-assisted gross-overcurrent trip.

**Do not** add timer/DMA current acquisition or a new ADC analog-watchdog safety action immediately before the Zephyr port. Those are real design candidates, but they change timing, interrupt behavior, cache/DMA assumptions, and safety authority. They should be treated as separate post-parity engineering projects.

---

## 2. Source basis and current baseline

This plan is based on:

- the Muad’Dib project notes describing the RX26T induction-motor inverter, external PWM command handling, ADC/PWM synchronization, watchdog behavior, motor-parameter dependence, real-hardware faults, and the specific ADC-stall/EMI failure mode;
- the attached DER26 v2.6.27 / AMS v0.5.30 FreeRTOS repository;
- the existing DER26 Zephyr migration plan and its requirement to preserve behavior rather than redesign battery algorithms during the kernel port;
- the present AMS hardware architecture: STM32F767, dual-range DHAB current measurement, ADBMS6822 bridge, five ADBMS6830 SMBs, ADBMS2950/APM, CAN, IMD, AIR/precharge/shutdown interfaces, and six fan zones.

### 2.1 Important current v2.6.27 status

The attached v2.6.27 package explicitly identifies itself as a **software-freeze candidate**, not yet a certified frozen oracle. The final source edits were packaged without rerunning the complete v2.6.27 regression corpus.

Before any lessons-transfer work begins, the package should pass at minimum:

```bash
cd AMS/host_tests
make can-tx-regression-test
make can-tx-scheduler-test
make safety-test
make test
make measurement-integrity-test
make profile-gates
make review-fixes-test
make can-v4-contract-gate can-irq-contract-gate can-clock-contract-gate
make adbms-v05-contract-gate
make whole-source-analyze
```

Recommended confidence runs:

```bash
make firmware-ci
make firmware-asan
make ubsan
make stress
```

Only after those pass should `v2.6.27 / 0.5.30` be tagged and treated as the immutable FreeRTOS comparison baseline.

### 2.2 Baseline rule

Once frozen:

> **Do not keep improving the reference branch.**

All lessons-transfer work should occur on a new branch. The frozen FreeRTOS image remains the parity oracle for the Zephyr migration.

Suggested branching:

```text
freeze/der26-freertos-v2.6.27
        |
        +-- feature/ams-flight-recorder
        +-- feature/ams-io-contract-audit
        +-- zephyr/phase-1
```

If one of these branches uncovers a genuine pre-existing safety defect in the freeze candidate, fix it deliberately, bump the baseline, rerun the complete freeze gate, and freeze again. Do not casually fold feature work back into the oracle.

---

## 3. Muad’Dib lessons and DER26 mapping

| Muad’Dib lesson | What happened / why it mattered | DER26 equivalent | Current status | Planned action |
|---|---|---|---|---|
| Never wait forever on hardware | A peripheral status wait could hang under EMI; watchdog became the last line of recovery | ADC, SPI/isoSPI, CAN recovery, UART, capture peripherals | Partially transferred; ADC/SPI/UART already have finite waits | Formal deadline audit and fault-injection tests for every blocking hardware boundary |
| A timeout must fit the control deadline | “Finite” is not enough if a 500 ms wait blocks a 100 ms task | ADBMS SPI currently uses a broad `SPI_TIMEOUT` while acquisition is periodic | Needs review | Derive per-operation deadlines from bus timing and task budget; instrument worst-case time |
| Lost command must become a safe command | External PWM loss must not leave stale motor speed active | ECU commands, charger commands, CAN authority, stale sensor values | Strongly transferred | Preserve and make the contract explicit through Zephyr; no stale reuse as fresh authority |
| Event time matters more than observation time | PWM/ADC timing and actual hardware events matter | CAN bus-off, sample timestamps, current-window boundary | Strongly transferred | Preserve exact event-time semantics and record trigger timestamp in flight recorder |
| Sample timing must be deterministic when the physics requires it | Motor current sampling was synchronized to PWM because switching transients matter | Pack-current sampling affects coulomb count, fuse observer, SoP | Current implementation is task/poll based | Instrument jitter first; only prototype timer-triggered ADC later if evidence justifies it |
| Fast protection should not wait for a slow control loop | Motor power-stage protection is separate from high-level control | BMS_OK/shutdown path, severe current faults | Partly transferred through direct fail-low paths | Research ADC analog-watchdog gross-overcurrent trip after parity; do not replace software current protection |
| Parameters are part of the controller | Motor `Rr/Lr`, inductances, etc. directly changed control behavior | OCV/ECM/SoP/fuse/thermal/current calibration | Strong provenance work already exists | Add compact runtime configuration/calibration manifest and recorder linkage |
| Debugging needs telemetry that survives the fault | Real hardware faults can disappear with a debugger attached | Track shutdowns, CAN bus-off, ADBMS faults, current faults | Retained event log exists but lacks pre-fault trajectory | Add pre/post-fault flight recorder |
| First-cause evidence matters | Later failures can obscure the initiating event | Cascading BMS/CAN/ADBMS faults | Fault log preserves events, but context is sparse | First trigger freezes recorder; later faults append to event log without replacing root trigger |
| Recovery must be bounded and escalated | Retrying forever can be as bad as hanging forever | CAN, ADBMS, ADC, charger | CAN policy is mature; other paths are less uniform | Define per-subsystem recovery budgets, retries, and escalation rules |
| Bench incidents should become regression tests | A real hardware failure is most valuable if it never returns | MiL/HiL/host-test corpus | Infrastructure already strong | Add recorder decoder + incident-to-replay conversion workflow |

---

# Part I — work that should happen before/during the Zephyr migration

## 4. Workstream A — Freeze the current FreeRTOS oracle

### Objective

Make v2.6.27 / firmware 0.5.30 a trustworthy fixed point before adding new behavior.

### Tasks

1. Run the complete host regression list above.
2. Record:
   - repository/source tree hash;
   - build profile;
   - compiler/toolchain identity;
   - FreeRTOS middleware hash;
   - test result summary;
   - source version string;
   - generated binary/map hashes if an ARM target build is available.
3. Generate a freeze manifest:

```text
DER26_FREEZE_MANIFEST_v2.6.27.json
```

Recommended fields:

```json
{
  "package": "v2.6.27",
  "firmware": "0.5.30",
  "revision": "DER26-AMS-v0.5.30-20260906",
  "source_tree_sha256": "...",
  "toolchain": "...",
  "profile": "...",
  "freertos_hash": "...",
  "host_test_manifest_hash": "...",
  "build_epoch": "..."
}
```

4. Tag the source and make it read-only by process.
5. Generate a replay corpus from representative MiL scenarios to serve as a Zephyr parity oracle.

### Exit gate

- all required v2.6.27 host gates pass;
- baseline identity is independently reproducible;
- no feature work exists on the freeze branch;
- replay corpus is archived and immutable.

### Estimated effort

**0.5–1.5 days**, assuming the suite passes without finding a new defect.

---

## 5. Workstream B — Add an AMS pre/post-fault flight recorder

### Why this is the highest-value new transfer

The current retained log is good at answering:

> “What fault/event occurred?”

It is weaker at answering:

> “What was the electrical/software state for the previous several seconds, and what changed immediately before the fault?”

That is exactly the class of problem that made the Muad’Dib ADC/EMI failure difficult: a debugger can change the timing or make the problem disappear, while a watchdog reset destroys the live context unless the firmware captured it first.

The DER26 AMS should therefore have a compact **black-box recorder**.

### 5.1 Functional requirements

The recorder shall:

1. continuously record a compact system snapshot at a fixed rate;
2. keep a rolling pre-fault history;
3. accept a trigger from task or ISR context;
4. preserve the first trigger as root cause;
5. retain a small post-trigger window when the MCU remains alive;
6. survive software/watchdog reset if memory budget permits;
7. never allocate dynamically;
8. never block a safety task;
9. never perform flash writes in the real-time path;
10. expose data through service CLI and/or existing passive CAN logger paths;
11. use a stable, versioned, fixed-width schema independent of RTOS and compiler padding;
12. be replayable by host tooling.

### 5.2 Recommended sampling rate

Use the existing **20 Hz error/safety supervisor** as the single writer initially.

Reasons:

- it already runs frequently enough to capture the evolution of most battery/communication faults;
- it is the safety aggregation point;
- it avoids adding another RTOS task;
- it avoids multi-writer complexity;
- it can sample the latest immutable measurement snapshot;
- 20 Hz gives 50 ms resolution without meaningful CPU or memory pressure.

The recorder should store the exact ISR trigger timestamp independently, so a 50 ms sample interval does not reduce event-time accuracy.

### 5.3 Recommended history depth

Initial target:

```text
Pre-trigger history:  5.0 s
Post-trigger history: 0.5 s
Sample rate:          20 Hz
Total samples:        110
```

If one record is 64–80 bytes:

```text
110 × 64 B = 7.0 KiB
110 × 80 B = 8.6 KiB
```

That is small enough to be realistic on the STM32F767, but **the actual map file must approve the allocation** before enabling reset-retained storage.

### 5.4 Record format

Prefer fixed-point integers over floats in the retained schema.

Suggested v1 record:

```c
typedef struct
{
    uint32_t sequence;
    uint32_t tick_ms;
    uint32_t measurement_sequence;

    uint32_t pack_voltage_mV;
    int32_t  pack_current_mA;

    uint16_t min_cell_mV;
    uint16_t max_cell_mV;
    uint16_t min_cell_index;
    uint16_t max_cell_index;

    int16_t  max_temp_deci_C;
    uint16_t max_temp_index;

    uint16_t soc_permille;
    uint16_t current_uncertainty_mA;

    int32_t discharge_power_limit_W;
    int32_t charge_power_limit_W;

    uint32_t measurement_validity_flags;
    uint32_t safety_fault_mask;
    uint32_t adbms_fault_mask;
    uint32_t can_status_flags;
    uint32_t heartbeat_stale_mask;

    uint16_t current_range;
    uint8_t  ams_state;
    uint8_t  bms_ok;

    uint8_t  can_authority;
    uint8_t  can_busoff_active;
    uint8_t  balancing_active;
    uint8_t  estimator_valid;

    uint32_t crc32;
    uint32_t commit;
} ams_flight_record_v1_t;
```

Exact fields should be finalized after checking existing app/measurement structures. The important design rule is to log **engineering-state evidence**, not every raw variable.

### 5.5 Recorder header

Suggested header:

```c
typedef struct
{
    uint32_t magic;
    uint16_t schema_version;
    uint16_t record_size;

    uint32_t boot_sequence;
    uint32_t write_sequence;
    uint32_t write_index;
    uint32_t valid_count;

    uint32_t trigger_sequence;
    uint32_t trigger_tick_ms;
    uint16_t trigger_event;
    uint16_t trigger_reason;

    uint32_t post_trigger_remaining;
    uint32_t runtime_manifest_crc;

    uint32_t header_crc32;
    uint32_t commit;
} ams_flight_recorder_header_t;
```

### 5.6 Concurrency model

Keep the concurrency model intentionally simple:

```text
20 Hz error task = sole record writer
ISR/task faults = trigger-only writers
```

ISR trigger path must do only bounded primitive operations:

```c
ams_flight_recorder_trigger_isr(event, reason, tick);
```

It should:

- capture first trigger tick/reason;
- set a trigger sequence/flag;
- never copy the full app object;
- never take an RTOS mutex;
- never format strings;
- never transmit CAN/UART;
- never calculate large CRC blocks.

The 20 Hz writer sees the trigger and completes post-trigger capture.

### 5.7 First-cause policy

The recorder should be **first-trigger wins**.

Example:

```text
CAN bus-off at 14.200 s        -> trigger
BMS low at 14.201 s            -> ordinary retained event
ADBMS stale at 14.350 s        -> ordinary retained event
ECU stale at 14.600 s          -> ordinary retained event
```

The later cascade must not overwrite the original trigger.

Explicit service clear or cold boot can re-arm the recorder.

### 5.8 Reset-retention strategy

Stage the feature.

#### Stage A — ordinary SRAM ring

Implement the recorder and prove correctness without `.noinit` retention.

Benefits:

- lower risk;
- easy host testing;
- simple memory ownership;
- works for most BMS hard faults that do not intentionally reset the MCU.

#### Stage B — retained ring

After map/headroom review, place the header and ring in `.noinit`, using the same engineering principles already used by the retained AMS fault log:

- magic/version/schema check;
- fixed entry size;
- sequence number;
- CRC;
- commit word written last;
- boot-time recovery that tolerates one torn record;
- no trust in partially committed records.

Do not write the ring to internal flash on every sample.

### 5.9 Trigger sources

At minimum:

- BMS_OK hard drop;
- voltage latch;
- temperature latch;
- current latch;
- ADBMS diagnostic hard fault;
- repeated CAN bus-off hard latch;
- CHARGE/BALANCE immediate CAN bus-off;
- AIR/precharge hard fault;
- watchdog feed-stop decision;
- RTOS stack/assert/malloc panic;
- unexpected IRQ/default-handler panic;
- manual service trigger for validation.

### 5.10 CLI / service interface

Suggested service commands:

```text
recorder status
recorder dump
recorder dump 25
recorder header
recorder clear
recorder trigger-test
```

Vehicle profile can keep CLI compiled out. The recorder itself remains active.

### 5.11 Export format

Host tool:

```text
Tools/ams_flight_recorder_decode.py
```

Outputs:

```text
CSV
JSON
human-readable summary
MiL replay seed
```

CSV columns should use stable names and physical units.

### 5.12 Required tests

Pure host tests:

- wrap-around ring behavior;
- sequence wrap;
- first-trigger-wins;
- 5 s pre + 0.5 s post sizing;
- no post samples after freeze;
- manual clear/re-arm;
- CRC detection;
- torn header;
- torn record;
- duplicate sequence;
- reset between data write and commit write;
- tick wrap;
- invalid/no measurement snapshot;
- minimum/max cell index calculation;
- exact fixed-point saturation.

Integration tests:

- trigger from CAN SCE ISR path;
- trigger from ordinary error task;
- trigger during ADBMS SPI activity;
- trigger while CAN is saturated;
- trigger during CLI dump;
- watchdog reset preserves valid pre-fault records;
- recorder never changes BMS_OK timing;
- recorder disabled build yields identical safety outputs and CAN payloads.

### 5.13 Exit gate

- host corpus passes;
- no dynamic allocation;
- recorder writer WCET measured;
- added critical-section time measured and negligible;
- target map shows acceptable RAM headroom;
- BMS fail-low timing unchanged within measurement resolution;
- retained reset recovery is proven before `.noinit` mode becomes default;
- dump can be converted into a replay case.

### Estimated effort

**3–5 focused engineering days** for Stage A plus decoder/tests.
**1–3 additional days** for retained Stage B and target-reset testing.

---

## 6. Workstream C — Formal bounded hardware-transaction contract

### Why this matters

The Muad’Dib lesson is not simply “use a timeout.” It is:

> **Every hardware interaction has a deadline, a failure meaning, and a bounded recovery action. The timeout must fit inside the system deadline that owns it.**

DER26 already does much of this, but the policy is spread across individual drivers.

Examples in the current source include:

- ADC conversion: checked start/poll/stop with a 5 ms polling timeout;
- ADBMS SPI: HAL SPI calls use `SPI_TIMEOUT`;
- CLI UART TX: finite 20 ms timeout;
- CAN: callback-driven TX plus explicit bus-off/recovery state machines.

The main remaining job is to make these rules explicit and testable.

### 6.1 Create a transaction inventory

Add:

```text
AMS/docs/AMS_HARDWARE_TRANSACTION_CONTRACT.md
```

Required columns:

| Subsystem | Operation | Caller | Normal duration | Current timeout | Owner deadline | Retry count | Recovery action | Data-validity action | Hard-fault escalation |
|---|---|---|---:|---:|---:|---:|---|---|---|
| ADC current low range | convert | current task | measured | 5 ms | 20 ms task period | 0/1 | stop ADC | invalidate sample | current-sensor fault policy |
| ADC current high range | convert | current task | measured | 5 ms | 20 ms | 0/1 | stop ADC | invalidate sample | current-sensor fault policy |
| ADBMS SPI TX/RX | command/read | ADBMS task | measured | currently broad | 100 ms acquisition budget | bounded | end session/recover | invalidate epoch | ADBMS fail-low policy |
| CAN TX load | mailbox | CAN task/ISR | measured | nonblocking | frame deadline | scheduler policy | abort/recover | revoke authority as required | state-dependent bus-off policy |
| UART service TX | CLI | CLI task | measured | 20 ms | diagnostic only | none | drop/return error | none | never safety authority |

### 6.2 Audit rule

For each blocking HAL call, prove:

```text
T_timeout <= T_owner_deadline - T_required_cleanup_margin
```

A finite but overly large timeout is still a real-time design problem.

One specific audit target is the ADBMS shared SPI timeout. A value such as 500 ms is bounded, but it is much longer than a 100 ms-class acquisition cadence. Do **not** blindly change it. First measure actual frame/wake/session durations, then replace the broad constant with operation-specific budgets.

### 6.3 Per-operation timeouts

Prefer:

```text
ADBMS_SPI_COMMAND_TIMEOUT_MS
ADBMS_SPI_READ_TIMEOUT_MS
ADBMS_WAKE_SETTLE_US
ADBMS_SESSION_TIMEOUT_MS
ADC_CURRENT_TIMEOUT_MS
CLI_TX_TIMEOUT_MS
CAN_RECOVERY_SETTLE_TIMEOUT_MS
```

over one generic large timeout.

### 6.4 Recovery rules

Every transaction path should specify:

```text
1. what constitutes failure;
2. whether the current sample/epoch becomes invalid;
3. whether one local retry is permitted;
4. whether peripheral reinitialization is permitted;
5. how frequently recovery may repeat;
6. when recovery escalates to fail-low;
7. which diagnostic counter increments;
8. which exact timestamp is retained.
```

### 6.5 No stale-data substitution

Hard rule:

> A failed new measurement may leave an old value available for diagnostics, but it must not silently relabel that old value as a fresh successful measurement.

This is already consistent with the immutable measurement/freshness architecture and should be carried into every Zephyr adapter.

### 6.6 Static/code-review gates

Add CI checks for suspicious constructs in authored code:

```text
unbounded while() around peripheral state bits
HAL_MAX_DELAY in production authored code
osWaitForever around safety-critical driver ownership
retry loops without explicit attempt bound
hardware polling without elapsed-time comparison
```

This should be a targeted script, not a naive ban on all `while` loops.

Suggested tool:

```text
AMS/host_tests/tools/check_bounded_io.py
```

### 6.7 Fault-injection coverage

For every driver boundary, add injected:

- timeout;
- busy;
- generic HAL error;
- stale completion;
- partial transaction where relevant;
- repeated failure;
- successful recovery;
- recovery failure.

### Exit gate

- every blocking hardware operation exists in the contract table;
- every timeout fits its owner deadline;
- every failure has defined data-validity semantics;
- retry loops are bounded;
- host/static gates cover regressions;
- no new safety behavior is introduced merely by the audit.

### Estimated effort

**2–4 days** for inventory, instrumentation, tests, and obvious low-risk timeout cleanup.
Potential target-specific retuning should be scheduled separately after measurements.

---

## 7. Workstream D — Incident-to-regression pipeline

### Objective

Turn a real track/bench incident into a permanent regression case instead of a one-time debugging session.

### Flow

```text
vehicle event
   ↓
flight recorder + retained fault log + raw CAN log
   ↓
decoder
   ↓
normalized incident JSON
   ↓
MiL/host replay
   ↓
root-cause fix
   ↓
permanent regression test
```

### 7.1 Canonical incident file

Suggested schema:

```json
{
  "incident_id": "2026-xx-xx-track-session-03-shutdown-02",
  "firmware_revision": "DER26-AMS-v0.5.30-20260906",
  "build_profile": "VEHICLE",
  "runtime_manifest_crc": "0x...",
  "trigger": {
    "event": "CAN_REPEATED_BUSOFF",
    "tick_ms": 1234567,
    "reason": 4
  },
  "records": [ ... ],
  "retained_fault_log": [ ... ],
  "raw_can_artifact": "CAN123.BIN"
}
```

### 7.2 Replay levels

Support three replay modes:

#### Level 1 — pure core replay

Feed recorded measurements/timestamps into:

- fault logic;
- estimator;
- SoP;
- SoH;
- fuse model;
- CAN encoder.

Expected result should match the original run.

#### Level 2 — scheduler/policy replay

Replay:

- CAN errors;
- stale events;
- recovery timing;
- state transitions;
- command freshness.

#### Level 3 — HIL replay

Drive physical/simulated signals where possible.

### 7.3 Required tooling

```text
Tools/ams_flight_recorder_decode.py
Tools/ams_incident_pack.py
MiL/incidents/<incident-id>/incident.json
AMS/host_tests/incidents/<incident-id>_regression.c
```

### Exit gate

Take one synthetic incident and prove the complete flow from recorder dump to replay to regression.

### Estimated effort

**2–4 days**, much of which can overlap flight-recorder development.

---

## 8. Workstream E — Unified runtime configuration/calibration provenance

### Why this transfers from Muad’Dib

Muad’Dib demonstrated that physical-model parameters are not documentation; they are executable behavior. A large change in motor parameters changes slip estimation and control response.

The same is true for DER26 battery parameters:

- current-sensor offset/calibration;
- current uncertainty;
- OCV model;
- ECM parameters;
- EKF Q/R;
- SoP limits and horizons;
- fuse model;
- voltage/current/temperature thresholds;
- CAN contract revision;
- physical SMB count/topology.

DER26 already has substantial build/provenance controls. The next step is to create a **single compact runtime identity** that can be attached to every incident.

### 8.1 Runtime manifest

Suggested structure:

```c
typedef struct
{
    uint32_t schema_version;
    uint32_t firmware_version;
    uint32_t source_tree_id;
    uint32_t build_profile;

    uint32_t current_calibration_id;
    uint32_t current_calibration_crc;
    uint32_t estimator_model_crc;
    uint32_t sop_model_crc;
    uint32_t soh_model_crc;
    uint32_t fuse_model_crc;
    uint32_t thresholds_crc;
    uint32_t can_contract_crc;

    uint16_t nsmbs;
    uint16_t ncells;
    uint32_t manifest_crc;
} ams_runtime_manifest_t;
```

Do not duplicate long Git strings inside high-rate telemetry. Use compact stable IDs/CRCs and retain the full mapping in build artifacts.

### 8.2 Usage

Manifest appears in:

- boot diagnostic output;
- passive logger header/session-start record;
- flight recorder header;
- incident JSON;
- target release manifest.

### 8.3 Safety semantics

The manifest is diagnostic identity, not authority by itself.

If an active calibration record fails validation, the existing safety behavior still decides authority. A matching CRC does not turn an invalid calibration into a valid one.

### Exit gate

Any flight-recorder dump or raw logger session can be mapped unambiguously to:

- firmware source;
- build profile;
- calibration/model set;
- CAN contract.

### Estimated effort

**1–3 days** because much of the underlying provenance already exists.

---

## 9. Workstream F — Recovery budgets and escalation policy review

### Objective

Apply one consistent rule across subsystems:

> **Recover locally when safe, but never retry forever and never let recovery erase the initiating evidence.**

### 9.1 Preserve the mature CAN policy

The current CAN implementation already embodies this lesson strongly:

- physical-event identity at the ISR boundary;
- event sequence ownership;
- 3 events / 10 s repeated-bus-off escalation;
- state-dependent fail-low;
- bounded DISCHARGE grace;
- recovery settlement tied to a specific event sequence;
- service recovery that cannot erase a newer fault.

Do not generalize this into a giant framework before migration. Use it as the **design reference** for other drivers.

### 9.2 ADBMS recovery policy review

Document:

- local PEC/read retry count;
- wake/re-wake budget;
- session restart budget;
- device-reset budget;
- measurement epoch invalidation;
- balance urgent-mute ownership;
- repeated-recovery escalation;
- retained counters and first-failure tick.

The ADBMS6830 already has explicit diagnostics and bidirectional/communication mechanisms. The AMS should distinguish:

```text
transient transport failure
persistent transport failure
measurement invalidity
device diagnostic failure
physical chain/topology failure
```

rather than treating all as one generic SPI fault.

### 9.3 ADC/current recovery policy review

If one ADC conversion fails:

```text
invalidate this current sample
increment timeout/error diagnostic
stop/reset local ADC state if required
continue scheduler
```

If repeated failures exceed the approved safety window:

```text
current becomes unavailable
current fault policy takes authority
BMS fail-low as required
```

Do not use the watchdog as ordinary ADC recovery.

### 9.4 CLI/UART

CLI is diagnostic-only and compiled out of vehicle authority builds. Therefore:

- UART failure must never block safety work;
- line output may be dropped;
- no safety mutex should depend on UART completion;
- recorder dump can be slow because it is service-only.

### Exit gate

Each hardware subsystem has a short recovery-state table defining:

```text
failure → retry → recovery → escalation → evidence
```

### Estimated effort

**1–2 days** of design/documentation plus tests for missing cases.

---

# Part II — integration with the Zephyr migration

## 10. What should move unchanged into Zephyr

Treat these as behavioral contracts, not FreeRTOS implementation details:

1. **No-authority boot.**
2. **Sole normal owner of BMS_OK assertion.**
3. **Direct fail-low path independent of normal application scheduling.**
4. **Software-liveness watchdog semantics.**
5. **Immutable timestamped measurement publication.**
6. **Per-reading freshness and calibration provenance.**
7. **Current-window boundary semantics.**
8. **CAN on-wire authority semantics.**
9. **CAN event-time and repeated-fault sequencing.**
10. **State-dependent bus-off policy.**
11. **Retained fault log.**
12. **New flight-recorder schema and first-cause behavior.**
13. **Runtime manifest identity.**
14. **Bounded transaction/recovery contracts.**

## 11. Where the new work fits into the existing migration phases

### Before Zephyr Phase 1

Complete:

- baseline freeze;
- flight-recorder pure-C schema and host tests;
- hardware transaction inventory;
- runtime manifest schema.

Do not add timer/DMA acquisition.

### Zephyr Phase 1 — no-authority skeleton

Port only:

- runtime manifest boot output;
- retained fault log support;
- flight-recorder storage primitive;
- safety trigger API.

Verify BMS low before enabling full recorder collection.

### Zephyr Phase 2 — portable core

Flight recorder should compile in both systems from the same source.

Platform-specific code is limited to:

```text
current tick
retained-memory section declaration
optional interrupt-save primitive
CLI/backend export
```

### Zephyr Phase 3 — ADC/watchdog

Apply the hardware-transaction table directly to the new ADC/watchdog adapter.

Parity criteria include:

- same invalidation behavior on ADC timeout;
- same current fault semantics;
- same watchdog feed/stop policy;
- recorder evidence before/after injected failures.

### Zephyr Phase 4 — ADBMS

Use the bounded-operation deadlines established in Workstream C.

Do not blindly preserve a broad HAL timeout just because FreeRTOS used it. Preserve the **behavioral deadline/recovery contract**, then implement it with Zephyr APIs.

### Zephyr Phase 5 — supervisor/measurement

Make the 20 Hz safety supervisor the recorder writer again unless measured scheduling evidence shows a reason to change.

### Zephyr Phase 6 — CAN

Flight recorder trigger timing must use actual Zephyr CAN state/error callbacks, not delayed task observation.

### Zephyr parity exit

Replay the same incident corpus through both systems and compare:

- trigger event/reason;
- trigger timestamp relation;
- BMS safety output;
- fault masks;
- measurement validity;
- CAN payloads;
- recorder state trajectory.

---

# Part III — optional post-parity improvements

## 12. Workstream G — Measure current-acquisition timing before redesigning it

### Objective

Determine whether task-polled current acquisition is actually limiting estimator/fuse/SoP performance.

Do not assume motor-control sampling practices must be copied into a BMS.

### 12.1 Instrument the current implementation

Measure on target:

- scheduled release jitter of 50 Hz current task;
- actual ADC conversion start timestamp;
- conversion duration;
- interval between low- and high-range samples;
- worst-case jitter under CAN load;
- worst-case jitter during ADBMS activity;
- worst-case jitter with CLI enabled;
- longest integration gap;
- current-window boundary offset;
- impact on coulomb count using recorded current profiles.

Add temporary telemetry/histograms rather than a permanent high-rate debug stream.

### 12.2 Decision criterion

Only redesign acquisition if evidence shows one or more of:

- jitter contributes material coulomb-count error;
- 50 Hz misses current transients needed by the fuse observer;
- SoP current dynamics require greater bandwidth;
- dual-channel timing skew creates measurable range disagreement;
- target timing under Zephyr is worse than the approved bound.

If none are true, retain the simple implementation.

### Estimated effort

**1–2 days** for instrumentation and bench measurements.

---

## 13. Workstream H — Optional timer-triggered ADC acquisition prototype

### Status

**Post-Zephyr parity / experimental branch only.**

### Motivation

Muad’Dib synchronized motor-current measurement to hardware timing because its physics required deterministic sampling. DER26 could use a similar architecture if measurement evidence justifies it.

### Architecture option

```text
hardware timer
   ↓
ADC1 + ADC2 trigger
   ↓
DMA or ADC interrupt
   ↓
small fixed current-sample buffer
   ↓
current-processing task consumes accumulated samples
   ↓
20/50 Hz published current window remains same external contract
```

### Important constraint

The STM32F767 Cortex-M7 data cache makes DMA ownership a real design issue. Do not enable DMA casually.

The prototype must explicitly define:

- DMA-visible SRAM region;
- alignment;
- cache maintenance or non-cacheable placement;
- buffer ownership;
- overrun behavior;
- exact timestamp source;
- ADC pair synchronization.

### Suggested sample-rate process

Do not choose 1 kHz, 2 kHz, etc. because they are round numbers.

Choose rate from:

1. DHAB bandwidth/noise;
2. fuse-model transient requirement;
3. SoP model bandwidth;
4. CPU/DMA cost;
5. desired anti-alias filtering;
6. measured vehicle current spectrum.

### Compatibility requirement

The public `ams_current_window_t` behavior should remain stable unless a separate requirements review approves a change.

That allows a higher-rate acquisition backend without rewriting estimator/SoP/SoH consumers.

### Prototype tests

- exact sample interval histogram;
- dual-ADC skew;
- injected DMA overrun;
- cache coherency stress;
- saturation/rail behavior;
- current-range crossover;
- zero calibration;
- comparison against oscilloscope/current source;
- same current-window output for equivalent input waveform;
- CPU utilization and interrupt load.

### Exit decision

Adopt only if it provides a measured benefit large enough to justify the complexity.

### Estimated effort

**3–6 days prototype + hardware validation**, excluding any safety qualification.

---

## 14. Workstream I — Optional hardware-assisted gross-overcurrent trip

### Status

**Research item only until a formal threshold/validation review is complete.**

### Concept

Use STM32 ADC analog-watchdog capability or an equivalent fast hardware/peripheral path to detect an obviously dangerous current excursion independently of the 50 Hz current task.

This is analogous to the Muad’Dib principle that catastrophic power-stage faults should not depend only on the normal control loop.

### What this should be

A **secondary gross-fault backstop**, not the normal current-protection algorithm.

Normal current protection remains responsible for:

- calibrated engineering units;
- dual-range selection;
- uncertainty;
- duration filtering;
- operating-mode thresholds;
- fault logging and diagnosis.

The hardware-assisted path would only cover an extreme threshold that is safely distinguishable from normal operation and noise.

### Prerequisites

1. validated sensor polarity;
2. validated low/high-range transfer functions;
3. maximum expected normal current envelope;
4. sensor saturation behavior;
5. ADC count distribution/noise;
6. startup transient characterization;
7. hardware/fuse/contactor safety requirement;
8. exact action required by FSAE/system safety design;
9. false-trip analysis.

### Proposed action path

```text
ADC analog watchdog IRQ
    ↓
record exact event tick + raw ADC count
    ↓
ISR-safe BMS fail-low
    ↓
set current-hard-trip pending reason
    ↓
error task performs detailed diagnosis/logging
```

### Rules

- no floating-point work in ISR;
- no mutex in ISR;
- no CAN/UART in ISR;
- threshold registers initialized before vehicle authority;
- interrupt disabled until calibration/startup gates are satisfied;
- event captured in flight recorder;
- ordinary software current fault must still independently work.

### Validation

- calibrated current injection below/at/above threshold;
- noisy threshold dwell;
- power-up transients;
- range saturation;
- ADC rail/open sensor;
- ISR latency to BMS low;
- fault-log/flight-recorder evidence;
- no false trip under worst normal load.

### Estimated effort

**3–5 engineering days for a prototype**, plus however much physical qualification is required. Do not include this in the initial Zephyr migration critical path.

---

# Part IV — validation architecture

## 15. Test pyramid for the lessons-transfer work

### Layer 1 — pure host tests

Use for:

- recorder ring logic;
- CRC/commit recovery;
- runtime manifest;
- timeout/recovery state machines;
- fixed-point conversion;
- first-cause semantics;
- replay decoder.

### Layer 2 — existing SIL/MiL

Use for:

- realistic current/cell/temp trajectories;
- stale-data events;
- CAN recovery sequences;
- shutdown causality;
- incident replay.

### Layer 3 — sanitizer/stress

Run:

```bash
make firmware-asan
make ubsan
make stress
```

Specifically stress:

- recorder sequence wrap;
- concurrent trigger/sample;
- repeated reset recovery;
- maximal CAN/ADBMS diagnostic activity.

### Layer 4 — target low-voltage bench

Measure:

- recorder WCET;
- BMS low latency before/after feature;
- task jitter;
- ADC conversion time;
- SPI transaction time;
- watchdog reset retention;
- actual `.noinit` preservation;
- map/RAM headroom.

### Layer 5 — pack/HIL

Inject:

- cell OV/UV;
- temp fault;
- current fault;
- ADBMS comm failure;
- CAN bus-off;
- charger stale command;
- IMD state changes;
- repeated recovery failure.

Then prove each event leaves a useful recorder trace.

### Layer 6 — vehicle

For early sessions, define an incident collection checklist:

```text
1. Do not clear recorder immediately.
2. Save raw ECU CAN logger artifact.
3. Dump retained AMS fault log.
4. Dump flight recorder.
5. Record build/runtime manifest.
6. Record physical observation and driver report.
7. Pack into incident directory.
8. Replay before making code changes.
```

---

## 16. Flight-recorder acceptance matrix

| Case | Expected recorder behavior |
|---|---|
| Normal operation | Rolling overwrite only |
| Manual trigger | Freeze first trigger + post window |
| Current hard fault | Exact trigger reason; pre-fault current trajectory retained |
| Cell OV/UV | Min/max cell trajectory and measurement validity visible |
| Temp fault | Max-temp trajectory + stale/usable state visible |
| CAN immediate CHARGE bus-off | ISR trigger tick retained; BMS low remains immediate |
| DISCHARGE bus-off recovery succeeds | Trigger policy chosen explicitly: either record as non-freezing incident or freeze only if hard latch; do not accidentally freeze every recoverable transient unless desired |
| Repeated CAN bus-off | Third-event hard-latch trigger retained |
| ADBMS failure | Last valid measurement sequence/age visible |
| RTOS panic | Ring survives reset if retained mode enabled |
| Watchdog reset | Pre-reset history survives; reset cause links to same boot sequence |
| Power cycle | Previous recorder preserved only if product policy requires; otherwise explicit cold-boot clear |

### Trigger-policy recommendation

Do **not** freeze on every recoverable warning. That would make the recorder unavailable for the real root cause later in the same drive.

Use severity classes:

```text
INFO      -> event log only
WARNING   -> event log only
RECOVERABLE -> event log + optional marker in recorder stream
HARD FAULT -> freeze recorder
PANIC/RESET -> freeze immediately
```

---

## 17. Performance budgets

The feature should have explicit budgets.

Initial proposed engineering targets:

| Item | Target |
|---|---:|
| Recorder rate | 20 Hz |
| Normal recorder CPU time | < 100 µs per sample target; measure, do not assume |
| ISR trigger work | constant-time primitive writes only |
| Dynamic allocation | 0 bytes |
| Flash writes in sampling path | 0 |
| Ring RAM | target ≤ 12 KiB before map approval |
| Added interrupt-disabled time | minimal and measured |
| BMS fail-low latency regression | none beyond measurement resolution |
| CAN schedule effect | no missed protected deadlines |
| ADBMS schedule effect | no acquisition deadline regression |

These are engineering targets, not claims about current measured performance.

---

## 18. Files/modules to add or modify

### New portable modules

```text
AMS/Core/Inc/diagnostics/ams_flight_recorder.h
AMS/Core/Src/diagnostics/ams_flight_recorder.c
AMS/Core/Inc/diagnostics/ams_runtime_manifest.h
AMS/Core/Src/diagnostics/ams_runtime_manifest.c
```

### FreeRTOS integration points

Likely changes:

```text
AMS/Core/Src/tasks/error_task.c
    20 Hz recorder sample hook
    hard-fault trigger handoff

AMS/Core/Src/ext_drivers/ams_safety.c
    panic/reset trigger metadata
    retained-memory initialization/recovery

AMS/Core/Src/tasks/cli_task.c
    service dump/status/clear commands

AMS/Core/Inc/app.h
    recorder/manifest state only if root object ownership is appropriate

AMS/Core/Inc/ams_build_profile.h
    optional feature/profile gates
```

### Host/testing

```text
AMS/host_tests/unit/flight_recorder_test.c
AMS/host_tests/unit/runtime_manifest_test.c
AMS/host_tests/tools/check_bounded_io.py
Tools/ams_flight_recorder_decode.py
Tools/ams_incident_pack.py
```

### Documentation

```text
AMS/docs/AMS_FLIGHT_RECORDER_CONTRACT.md
AMS/docs/AMS_HARDWARE_TRANSACTION_CONTRACT.md
AMS/docs/AMS_INCIDENT_REPLAY_WORKFLOW.md
AMS/docs/AMS_RUNTIME_MANIFEST.md
```

### Zephyr integration later

Portable recorder source should remain the same. Replace only platform hooks for:

```text
time
retained section
interrupt-safe trigger primitive
service export backend
```

---

## 19. Build/profile policy

Recommended compile-time controls:

```c
AMS_ENABLE_FLIGHT_RECORDER
AMS_FLIGHT_RECORDER_RETAINED
AMS_ENABLE_SERVICE_RECORDER_DUMP
```

Suggested policy:

| Profile | Recorder | Retained | CLI dump |
|---|---|---|---|
| HOST/UNIT | yes | simulated | yes |
| BENCH | yes | yes after validation | yes |
| HIL | yes | yes | yes |
| VEHICLE | yes | yes after validation | no service CLI |
| NO-AUTHORITY ZEPHYR | yes | initially no, then yes | diagnostic backend only |

The recorder must never be a condition for BMS authority. If recorder initialization fails, record a diagnostic if possible, but do not create a new single point of failure in the safety path.

---

## 20. Detailed schedule

### Phase 0 — freeze oracle

**Duration:** 0.5–1.5 days

Deliverables:

- passing v2.6.27 host gates;
- freeze tag;
- manifest/hash bundle;
- archived replay corpus.

### Phase 1 — flight recorder core

**Duration:** 2–3 days

Deliverables:

- portable ring module;
- versioned schema;
- first-trigger logic;
- host unit tests;
- basic error-task sampling.

### Phase 2 — decoder and retained integration

**Duration:** 1–3 days

Deliverables:

- CLI dump;
- decoder;
- `.noinit` mode;
- torn-write recovery;
- reset tests;
- map/headroom evidence.

### Phase 3 — bounded I/O audit

**Duration:** 2–4 days

Deliverables:

- transaction contract;
- duration instrumentation;
- static bounded-I/O gate;
- targeted timeout/failure tests;
- timeout cleanup where evidence supports it.

### Phase 4 — incident replay pipeline

**Duration:** 2–4 days, overlaps Phase 2

Deliverables:

- normalized incident JSON;
- one end-to-end synthetic incident replay;
- one permanent regression generated from recorder data.

### Phase 5 — runtime manifest

**Duration:** 1–3 days

Deliverables:

- compact manifest;
- recorder linkage;
- logger/session exposure;
- host validation.

### Phase 6 — Zephyr carryover

Fold the finished pure-C modules into the existing migration phases. No separate rewrite.

### Phase 7 — post-parity acquisition study

**Duration:** 1–2 days measurement; 3–6 days prototype only if justified.

### Phase 8 — optional analog-watchdog backstop

**Duration:** 3–5 days prototype plus physical qualification.

### Overall additional work before/through early migration

A realistic focused scope is approximately **8–14 engineering days**, most of which creates reusable diagnostics/validation infrastructure rather than altering the battery-control algorithms.

The two optional post-parity hardware/timing projects are separate and should not delay the initial Zephyr migration.

---

## 21. Risk register

| Risk | Impact | Mitigation |
|---|---|---|
| Feature work moves the migration oracle | High | Freeze v2.6.27 first; feature branch only |
| Recorder adds too much RAM | Medium/high | Fixed-point schema, ≤12 KiB target, map gate |
| `.noinit` recorder recovery corrupts evidence | High | CRC + commit-last + boot recovery + torn-write tests |
| Recorder changes BMS fault latency | Critical | ISR trigger constant-time; target latency A/B measurement |
| Recorder causes priority inversion | High | single task writer; no mutex in ISR |
| Dump path blocks safety | High | service-only low-priority output; never called from ISR/error path |
| Timeout audit produces arbitrary smaller timeouts | High | measure bus timing; derive from owner deadline + margin |
| SPI timeout reduction creates false ADBMS failures | High | logic-analyzer/soak evidence before changing production value |
| Parameter manifest is mistaken for safety validation | Medium | manifest is identity only; existing calibration validity rules remain authoritative |
| Timer/DMA prototype creates cache bugs | High | defer until parity; explicit DMA/cache memory design |
| Analog watchdog false trips | Critical | experimental branch, calibrated threshold study, no vehicle authority until validated |
| Logging schema changes every release | Medium | versioned fixed-width schema; backward-compatible decoder |
| Too many recorder triggers make data unusable | Medium | first hard-fault trigger policy; warnings become markers only |
| Track data cannot reproduce incident | Medium | raw CAN + recorder + fault log + runtime manifest packaged together |

---

## 22. Definition of done for the lessons-transfer program

The program is complete when all of the following are true:

### Baseline

- v2.6.27/0.5.30 has a passing final host freeze gate or a deliberately superseded re-freeze;
- the comparison baseline is immutable and reproducible.

### Flight recorder

- 5 s pre-fault + 0.5 s post-fault history works;
- first hard fault owns the trigger;
- ISR trigger path is constant-time and nonblocking;
- no dynamic allocation;
- no flash write in real-time path;
- retained mode survives watchdog/software reset without trusting torn records;
- target RAM/WCET/headroom evidence exists;
- BMS fail-low timing is unchanged;
- CLI/tooling can export and decode data.

### Bounded I/O

- every blocking peripheral operation is documented;
- every timeout fits its owning deadline;
- every failure invalidates data correctly;
- retries are bounded;
- repeated failure escalation is documented;
- CI detects obvious reintroduction of unbounded hardware polling.

### Incident replay

- at least one recorder trace can become a host/MiL replay;
- a synthetic or real incident has been turned into a permanent regression test.

### Provenance

- every incident can be tied to the exact firmware/build/calibration/model set.

### Zephyr

- recorder, manifest, and transaction contracts are carried into Zephyr without algorithm drift;
- FreeRTOS and Zephyr produce equivalent safety decisions on the same incident corpus.

### Optional hardware improvements

- timer/DMA current acquisition is adopted only if measured benefit justifies it;
- hardware-assisted overcurrent trip is adopted only after a separate safety/threshold validation campaign.

---

## 23. Immediate implementation order

If starting this work now, use this exact order:

### Step 1

Rerun the final v2.6.27 host regression suite and freeze it.

### Step 2

Create only these first files:

```text
ams_flight_recorder.h
ams_flight_recorder.c
flight_recorder_test.c
AMS_FLIGHT_RECORDER_CONTRACT.md
```

Do not touch ADC timing, SPI timing, or BMS policy yet.

### Step 3

Integrate a **RAM-only** 20 Hz recorder writer into `error_task.c` and a manual test trigger.

Prove:

- wrap;
- freeze;
- first-trigger;
- post-trigger capture;
- zero safety-output change.

### Step 4

Add decoder + CLI dump.

### Step 5

Add reset-retained `.noinit` mode using CRC/commit mechanics modeled after the existing fault log.

### Step 6

Create the hardware transaction contract and measure actual ADC/SPI/UART timing. Do not reduce timeouts until evidence exists.

### Step 7

Add the runtime manifest and attach its CRC to the recorder.

### Step 8

Build the incident-to-regression toolchain.

### Step 9

Begin the Zephyr port with the FreeRTOS oracle still frozen.

### Step 10

After Zephyr bench parity, decide from measurements whether deterministic high-rate current acquisition or an ADC analog-watchdog trip is worth implementing.

---

## 24. Bottom line

The best transfer from Muad’Dib is a **system engineering pattern**:

```text
physical event
    ↓
deterministic/bounded acquisition
    ↓
explicit validity and freshness
    ↓
bounded control/recovery decision
    ↓
fast safe output when required
    ↓
preserve evidence
    ↓
replay the incident
    ↓
turn it into a regression test
```

DER26 already implements much of the middle of that chain. The major missing capability is the **preserve evidence → replay incident** portion.

Therefore the highest-return next feature is not another estimator algorithm or a more complicated RTOS abstraction. It is a small, robust, reset-tolerant flight recorder plus a formal bounded-I/O contract. Those two additions directly encode the most valuable lessons from the Muad’Dib project while making the upcoming Zephyr migration easier to validate rather than harder.
