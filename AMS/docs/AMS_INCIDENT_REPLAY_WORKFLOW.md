# DER26 AMS Incident-to-Regression Workflow

**Tool:** `Tools/ams_incident_pack.py`
**Incident schema:** 1

## Objective

A bench or track shutdown is not considered fully closed when its immediate
cause is understood. The useful end state is:

```text
physical incident
    -> preserved recorder/fault/CAN evidence
    -> normalized incident bundle
    -> replay seed
    -> host/MiL reproduction
    -> permanent regression
```

This workflow implements the evidence-preservation and normalization portion of
that chain without changing vehicle safety behavior.

## Capture checklist

For an incident, save one terminal capture containing, when possible:

```text
version
recorder status
recorder header
recorder dump
```

Also retain separately when available:

- fault-log dump;
- raw CAN logger/sniffer file;
- bench configuration and wiring notes;
- scope screenshots for hardware faults;
- test operator notes.

Do not clear the recorder or retained fault log until evidence has been copied.

## Packing an incident

Example:

```bash
python3 Tools/ams_incident_pack.py session03.txt \
  --incident-id 2026-09-17-bench-session03-shutdown02 \
  --output-dir MiL/incidents/2026-09-17-bench-session03-shutdown02 \
  --raw-can logs/session03.asc \
  --fault-log logs/session03_fault.txt \
  --notes "five-SMB ring, charger disconnected"
```

The tool requires complete `r1`/`r2` recorder pairs and produces:

### `incident.json`

Contains:

- schema and incident ID;
- packing timestamp;
- firmware/profile/features when present in CLI capture;
- recorder boot/schema/header CRC;
- runtime-manifest CRC;
- source and current-calibration fingerprints when present;
- trigger sequence/tick/event/reason;
- all normalized recorder samples;
- optional paths to raw CAN/fault-log evidence.

### `replay_seed.json`

Contains only the deterministic time-series fields needed to seed later host or
MiL replay adapters:

- tick/sequence;
- pack voltage/current;
- min/max cell;
- max temperature;
- SoC;
- measurement-validity flags;
- safety/ADBMS/CAN fault state;
- AMS state, BMS_OK and CAN authority.

The replay seed does **not** invent missing data. If a signal was not recorded,
it is not reconstructed from assumptions.

## Current scope

The current tool normalizes and packages evidence. It does not automatically
generate C tests because incident-specific expected behavior still requires an
engineer to define the invariant being locked down. This is deliberate: a tool
should not convert an observed bug into the new expected behavior merely
because it appeared in a trace.

## Promotion to a regression

For each accepted incident:

1. reproduce the behavior from `replay_seed.json` in host/MiL;
2. state the expected safety invariant explicitly;
3. fix production code if required;
4. add a named regression using the smallest deterministic trace that
   reproduces the issue;
5. retain `incident.json` as provenance for why the regression exists;
6. verify the same incident corpus during Zephyr parity work.

## Tool acceptance

`make incident-pack-test` runs an end-to-end synthetic capture through:

```text
CLI capture -> recorder decoder -> incident.json -> replay_seed.json
```

and validates normalized metadata and sample content.
