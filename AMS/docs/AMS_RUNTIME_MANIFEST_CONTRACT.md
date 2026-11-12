# DER26 AMS Runtime Manifest Contract

**Introduced:** firmware 0.5.31 / package v2.6.28 development baseline
**Schema:** `AMS_RUNTIME_MANIFEST_SCHEMA = 1`

## Purpose

The runtime manifest is a compact identity record that lets a flight-recorder
capture, fault log, raw CAN capture, or incident bundle be tied to the exact
firmware/configuration/model context that produced it.

It is **diagnostic provenance only**. A matching CRC never grants BMS authority,
never makes a calibration valid, and never overrides existing measurement,
model-validation, profile, or safety gates.

## Identity fields

`ams_runtime_manifest_t` is a fixed-width 76-byte object with its own CRC. It
contains compact fingerprints for:

- firmware version;
- source/build identity (`AMS_SOURCE_REVISION`, build Git identity, build config
  fingerprint);
- build profile and feature flags;
- current-sensor calibration actually applied at runtime;
- estimator, SoP and SoH model revisions;
- EAC14-80 fuse-model revision identity;
- safety-threshold revision;
- CAN contract revision;
- physical topology (`NSMBS`, cells/segment, temperatures/segment, physical
  chain count, thermistor-model revision).

Long strings remain in build artifacts and normal version output; high-rate or
retained evidence carries only compact CRCs.

## Current calibration fingerprint

`current_calibration_crc` is calculated from the runtime current-sensor object,
including:

- formal calibration ID and capture metadata;
- formal calibration uncertainty;
- whether a record was actually loaded;
- service zero-calibration state/count;
- applied 50 A and 800 A zero offsets;
- ADC reference and sensor-supply values.

This makes a service-time zero adjustment visible in incident identity even
though a service zero capture is not, by itself, formal calibration evidence.

When `current zero` or `current zero clear` changes the applied identity, the
manifest is rebuilt and the RAM flight recorder is intentionally reinitialized
under the new manifest CRC. This avoids mixing pre-change and post-change
samples under one recorder identity.

## Flight-recorder linkage

At boot:

1. the retained fault log establishes the current boot sequence;
2. the RAM flight recorder is initialized with that boot sequence;
3. the runtime manifest is built from the initialized board/current-sensor
   state;
4. the recorder is rebound to `runtime_manifest.manifest_crc`.

`recorder header` therefore exposes the manifest CRC associated with every
record in that evidence epoch.

## CLI evidence

`version` prints compact runtime identity lines:

```text
runtime manifest crc:........ source:........ cal:........ valid:1
runtime models est:........ sop:........ soh:........ fuse:........
runtime contracts thresholds:........ can:........ topo:........ cal_id:...
```

The existing long-form build/model revision output remains available for human
mapping and release records.

## Integrity

The manifest CRC is field-by-field and endian-defined rather than a CRC over
compiler padding. String identities are individually CRC32 fingerprinted.
The runtime calibration fingerprint hashes explicit scalar fields and IEEE-754
32-bit float bit patterns used by the target/toolchain.

## Host acceptance

`make runtime-manifest-test` checks:

- schema/magic/size integrity;
- stable nonzero model/contract/topology identities;
- firmware-version packing;
- calibration fingerprint changes when applied calibration changes;
- manifest CRC changes with identity changes;
- corruption rejection.

The comprehensive host suite also checks that the recorder header is bound to a
valid nonzero runtime-manifest CRC.

## Zephyr parity requirement

The same source/schema should be reused in Zephyr. Platform migration may
change the full build system, but the runtime incident bundle must still map to
firmware source, build profile, calibration/model set, CAN contract and physical
topology without relying on a debugger or an informal note.
