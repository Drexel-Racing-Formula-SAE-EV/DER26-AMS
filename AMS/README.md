# Current update: v0.5.57-r4 wake/session fix

See [WAKEFIX_R4_REVIEW.md](WAKEFIX_R4_REVIEW.md) for corrections, test evidence and remaining timing work. Full native vehicle-ci passes. Source only; no ARM or hardware qualification.

# DER26 AMS v0.5.57 — Vehicle Main Branch (1 Mbps)

**STM32CubeIDE source project** for the 1 Mbps AMS↔ECU vehicle bus. This is NOT
the separate 250 kbps charge-only image. Derived from the original
`v0.5.56-VEHICLE-1M-CLI4` variant with a targeted ADBMS2950B driver maintenance
import from the `APM-EVAL6822-DIAG5R3` branch.

## CubeIDE build configurations

- `VEHICLE COMMS 1M Debug/Release` (profile 4): five-SMB acquisition,
  ECU CAN telemetry, optional service CLI; BMS_OK and balancing permanently inhibited.
- `VEHICLE RELEASE GATED 1M Debug/Release` (profile 3): driving-profile
  source, rejects compilation until its existing target-validation evidence
  macros are supplied. The gate is **not** bypassed in this release.

CAN: **1 Mbit/s**; five SMBs; no installed ADBMS2950/APM; no fan task.
The charger-only 250 kbps CAN/project settings are not present.

## Consolidation / what's new

- Retains previously integrated asynchronous CLI, voltage/temperature
  acquisition timing, wake policy and CAN scheduling from v0.5.56 vehicle.
- Imports ADBMS2950B core driver updates from DIAG5R3 (calibration validity,
  single-shot and continuous conversion qualification, diagnostic register
  snapshot/fault observations, protocol and error-handling fixes). This driver
  is **not enabled in vehicle profiles 3/4**; no APM standalone wiring or
  evaluation-board configuration is imported.
- Adds host-only APM driver regression fixtures. Does not import standalone
  CLI commands, bench bridge readiness behavior, or charger CAN overrides.
- Sets release identity to v0.5.57; retains all installed vehicle power and
  authority gates, cell requirements, and CLI profile behavior.

## Usage

Import this directory via STM32CubeIDE > File > Import > Existing Projects
into Workspace. The default vehicle communications configuration is profile 4.
Choose the desired documented configuration explicitly, clean and build.

To reproduce supported host checks:

```sh
cd host_tests
make vehicle-variant-test cli-async-test can-tx-scheduler-test vehicle-project-check
make adbms-wake-policy-test measurement-integrity-test
make apm-hardening-test apm-driver-regression
```

## v0.5.57-r1 main-branch fixes

The initial SoC prior is **27% in both vehicle profiles**. Legacy project/clock
CI checks now validate the actual 1 Mbps vehicle `.ioc` and four CubeIDE
configs; all four build directories are unique. `make unit` now links with
host-only tick stubs, but its legacy assertions still fail; the supported
vehicle gate is `make -j2 vehicle-ci`. AIR auxiliary feedback remains OFF
because the hardware is not fitted. The dual-CAN charger gateway remains future
work and is not part of this package. See `VEHICLE_MAIN_FIXES_v0.5.57-r1.md`.

## Status

This release is *source only*. It is not an ARM-target-linked binary and
not a hardware-validated driving release. The configured vehicle profile 3
retains its evidence gates, including full cell observation requirements.
Charging is intentionally excluded from this vehicle project.
See `VEHICLE_VARIANT_README.md` for the original vehicle contract and bring-up.
Older charge-only documentation is retained under `docs/historical/` only.
