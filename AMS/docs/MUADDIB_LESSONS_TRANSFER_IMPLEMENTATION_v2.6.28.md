# Muad'Dib lessons transfer — implementation closeout v2.6.28

**Date:** 2026-09-17
**Firmware:** 0.5.31
**Package target:** v2.6.28
**Parent baseline:** v2.6.27 / firmware 0.5.30

## Scope completed in this increment

This increment starts implementing the approved Muad'Dib-to-DER26 lessons
transfer plan without starting the Zephyr port or changing battery-control
algorithms.

### 1. Baseline freeze evidence

The untouched v2.6.27 source was rerun through the required code-only freeze
gates before feature work began. The generated external freeze manifest records:

- package/version identity;
- source-input-tree SHA-256;
- exact bundled FreeRTOS source-tree SHA-256;
- named host-gate status;
- explicit note that no ARM target build was run.

Artifact: `DER26_FREEZE_MANIFEST_v2.6.27.json` in the delivery bundle.

### 2. RAM pre/post-fault flight recorder

Added portable allocation-free recorder core:

- `Core/Inc/diagnostics/ams_flight_recorder.h`
- `Core/Src/diagnostics/ams_flight_recorder.c`

Stage-A characteristics:

- 20 Hz recording;
- 5.0 s pre-history (100 records);
- 0.5 s post-trigger window (10 records);
- fixed 80-byte record schema;
- 110-record capacity;
- 8,856-byte total recorder object;
- first-trigger-wins semantics;
- record CRC plus commit-last publication;
- header CRC plus commit;
- sequence/tick-wrap safe ring export;
- ordinary SRAM only; retained `.noinit` remains disabled pending target map and
  torn-write validation.

The 20 Hz safety supervisor is the sole normal recorder writer. Automatic Stage-A
triggering occurs on the first aggregated hard fault. Direct ISR triggering is
intentionally deferred until target interrupt-cost evidence exists; exact CAN fault
event timing remains preserved by the existing CAN/fault diagnostics.

### 3. Recorder service interface and decoder

Service-capable builds provide:

- `recorder status`
- `recorder header`
- `recorder dump [n]`
- `recorder clear`
- `recorder trigger-test`

Vehicle CLI remains compiled out. Dump lines are deliberately split into bounded
`r1`/`r2` records so the existing 128-byte CLI line buffer is never overrun.

`Tools/ams_flight_recorder_decode.py` converts a captured dump to summary, CSV, or
JSON with no third-party Python dependencies.

### 4. Bounded hardware-I/O contract

Added `AMS_HARDWARE_TRANSACTION_CONTRACT.md` plus static gate
`host_tests/tools/check_bounded_io.py`.

The contract captures the Muad'Dib lesson directly:

> no authored peripheral operation may rely on an unbounded hardware wait; a
> timeout must also fit the owning task's deadline/recovery budget.

The inventory covers current ADC reads, generic ADC access, ADBMS6830/2950 SPI,
microsecond timer waits, UART service output, CAN publication/recovery, watchdog
update, and safety-critical mutexes. Existing 500 ms ADBMS SPI timeouts are
recorded as measurement items rather than being arbitrarily retuned before target
evidence.

### 5. Compact runtime provenance manifest

Added:

- `Core/Inc/diagnostics/ams_runtime_manifest.h`
- `Core/Src/diagnostics/ams_runtime_manifest.c`
- `host_tests/unit/runtime_manifest_test.c`
- `AMS_RUNTIME_MANIFEST_CONTRACT.md`

The fixed 76-byte manifest fingerprints:

- firmware/source/build identity;
- build profile/config/features;
- applied current-sensor calibration identity;
- estimator/SoP/SoH/fuse model revisions;
- threshold and CAN contract revisions;
- SMB/cell/temp/thermistor topology.

It is identity only, never an authority input. The manifest CRC is stored in the
flight-recorder header. Service current-zero changes rebuild the manifest and begin
a new recorder evidence epoch so samples from two calibration identities cannot be
silently mixed.

The recorder boot sequence now comes from the already-retained fault log instead
of the Stage-A placeholder value.

### 6. Incident-to-regression evidence pipeline

Added:

- `Tools/ams_incident_pack.py`
- `AMS_INCIDENT_REPLAY_WORKFLOW.md`

The packer consumes a service capture containing version/recorder output and emits:

- `incident.json` — normalized evidence plus runtime identity and trigger metadata;
- `replay_seed.json` — deterministic time-series input for later host/MiL replay.

The tool deliberately does not auto-generate expected regression behavior from an
observed failure. An engineer must state the invariant before promoting the trace to
a permanent test, preventing a bug from becoming the new oracle by accident.

## Regression/validation added

New focused targets:

- `make flight-recorder-test`
- `make runtime-manifest-test`
- `make flight-recorder-decode-test`
- `make incident-pack-test`
- `make bounded-io-gate`

The comprehensive host suite also checks that the safety-task recorder is bound to a
valid nonzero runtime manifest and that recorder collection itself does not change
BMS authority.

After implementation, the following passed in the review environment:

- flight recorder core tests;
- runtime manifest tests;
- recorder decoder self-test;
- incident packer end-to-end self-test;
- bounded-I/O static gate;
- comprehensive host/injection suite;
- safety/watchdog suite;
- measurement-integrity suite;
- 19 CAN transport regressions;
- CAN scheduler tests;
- profile/review/CAN contract gates;
- static-allocation and state-ownership gates;
- whole-source GCC analysis for bench and vehicle profiles.

## Deliberately deferred

The following plan items remain deferred rather than being partially implemented:

1. retained `.noinit` flight-recorder placement;
2. direct ISR recorder trigger writes;
3. ADBMS SPI timeout retuning without measured target latency;
4. timer-triggered/DMA pack-current acquisition;
5. ADC analog-watchdog gross-overcurrent backstop;
6. Zephyr port itself.

Those require target memory/timing/hardware evidence or belong after Zephyr parity.

## Migration consequence

The new recorder, runtime-manifest and bounded-I/O behavior are now behavioral
contracts for the Zephyr port. They should move as common portable code; Zephyr
adapters should only replace platform time, retained-memory placement, low-level
peripheral operations and diagnostic export.
