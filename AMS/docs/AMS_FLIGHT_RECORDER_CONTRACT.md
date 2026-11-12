# DER26 AMS flight-recorder contract

**Schema:** v1
**Implementation status:** Stage A, ordinary SRAM
**Introduced from baseline:** v2.6.27 / firmware 0.5.30
**Feature branch firmware:** 0.5.31

## Purpose

The flight recorder captures the system trajectory around the first hard fault without becoming part of BMS authority. It is an observability feature: failure to initialize, record, or export data must never grant or remove BMS authority.

The initial implementation is intentionally conservative:

- the existing 20 Hz error/safety task is the sole full-record writer;
- the recorder uses fixed static storage and no dynamic allocation;
- Stage A lives in ordinary SRAM and does not claim reset retention;
- no flash writes occur in the real-time path;
- the recorder does not add a task or mutex;
- the vehicle CLI remains compiled out by the existing vehicle profile;
- retained `.noinit` support is deliberately deferred until target map/headroom and torn-write recovery are validated.

## Capture window

| Item | Value |
|---|---:|
| Sampling rate | 20 Hz |
| Nominal sample interval | 50 ms |
| Ring capacity | 110 records |
| Pre-trigger records retained after a full history exists | 100 |
| Post-trigger records | 10 |
| Record size | 80 bytes |
| Header size | 56 bytes |
| Recorder object size | 8,856 bytes |

The writer continuously overwrites the oldest sample before a trigger. The first trigger freezes trigger metadata. Ten subsequent samples are accepted, after which the ring becomes frozen. When the ring was already full, this leaves the newest 100 pre-trigger records and 10 post-trigger records.

## First-cause rule

The first trigger wins. Later faults may change ordinary recorded state and the existing retained fault log, but cannot replace:

- trigger sequence;
- trigger tick;
- trigger event;
- trigger reason.

`recorder clear` is the explicit service re-arm operation in service-capable builds.

## Record schema

`ams_flight_record_v1_t` is fixed at 80 bytes and protected by a compile-time size assertion. The current fields are:

- record sequence and 32-bit system tick;
- immutable measurement sequence;
- pack voltage and current;
- min/max usable cell voltage and flattened cell indices;
- maximum usable temperature and flattened sensor index;
- estimator SoC in permille;
- current uncertainty/range;
- 1 s discharge/charge power limits;
- measurement validity flags;
- compact safety-fault mask;
- ADBMS active/latched fault masks;
- CAN state flags;
- heartbeat stale mask;
- AMS state, BMS_OK, CAN authority, bus-off recovery, balancing and estimator validity;
- CRC32;
- commit word written last.

Cell and temperature summaries are generated only from the immutable measurement snapshot's usable masks. Missing measurements remain zero with their validity flags unset; old values are not relabeled as fresh.

## Header schema

The 56-byte header records:

- magic and schema version;
- record size;
- boot sequence placeholder;
- write sequence/index and valid count;
- trigger sequence/tick/event/reason;
- post-trigger remaining count;
- runtime-manifest CRC placeholder;
- triggered/frozen flags;
- header CRC and commit word.

Stage A sets runtime-manifest CRC to zero. Workstream E will connect the compact runtime manifest.

## Concurrency and timing

The production integration currently has one full writer: `error_task_update()` after the safety critical section has ended. The recorder therefore does not lengthen the BMS_OK decision critical section.

Service clear/manual trigger operations use a short RTOS critical section. Each CLI dump record is copied under a short critical section and formatted/transmitted after interrupts are re-enabled.

The generic trigger API performs only bounded scalar/header operations. Direct ISR triggers are not wired in Stage A; that is reserved for the next integration step after target latency measurement. CAN and panic events still appear through the 20 Hz safety writer and the existing retained fault log.

## Trigger policy in Stage A

Automatic:

- first aggregated `hard_fault` observed by the error supervisor.

Service validation:

- `recorder trigger-test`.

Future direct event-time triggers will include CAN hard bus-off, watchdog feed-stop and panic paths after the ISR path is target-measured.

## CLI

Service-capable builds expose:

```text
recorder status
recorder header
recorder dump [1..110]
recorder clear
recorder trigger-test
```

`recorder dump` emits paired `r1`/`r2` lines with a common sequence number so every line stays below the 128-byte CLI buffer ceiling. The host decoder joins those pairs.

Vehicle builds keep the service CLI compiled out. Recorder capture remains independent of CLI availability.

## Host decoder

```text
Tools/ams_flight_recorder_decode.py
```

Examples:

```bash
python3 Tools/ams_flight_recorder_decode.py capture.txt
python3 Tools/ams_flight_recorder_decode.py capture.txt --format csv -o incident.csv
python3 Tools/ams_flight_recorder_decode.py capture.txt --format json -o incident.json
```

## Stage-A acceptance

Completed in host tests:

- ring wrap and chronological export;
- first-trigger-wins;
- exact ten-record post-trigger freeze;
- clear/re-arm;
- record/header CRC rejection;
- 32-bit record-sequence wrap;
- error-task integration;
- no BMS-state mutation by recorder capture;
- whole-source bench and vehicle analysis.

Still required before Stage B retained mode:

- ARM target map/RAM headroom evidence;
- error-task recorder WCET measurement;
- target critical-section measurement for service operations;
- BMS fail-low A/B latency measurement;
- `.noinit` linker placement;
- reset/torn-write recovery tests;
- watchdog-reset survival test.

## Runtime identity linkage (0.5.31)

The Stage-A recorder header is now bound to the compact runtime manifest CRC
rather than the previous placeholder zero. The boot sequence comes from the
retained fault log established before `app_create()`. A service current-zero
identity change rebuilds the runtime manifest and starts a fresh recorder
evidence epoch so one dump cannot silently mix two calibration identities.

See `AMS_RUNTIME_MANIFEST_CONTRACT.md` and `AMS_INCIDENT_REPLAY_WORKFLOW.md`.
