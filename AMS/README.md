# DER26 AMS Firmware

STM32F767 + FreeRTOS firmware for the DER26 Accumulator Management System.

The AMS acquires cell voltage/temperature and pack-current data, supervises battery/communication faults, runs battery-state estimation and power-limit logic, publishes accumulator authority over CAN, and owns the normal software path that may assert BMS_OK.

> This source tree contains bench/HIL/vehicle-oriented build profiles. A successful build or host test does not by itself authorize HV/vehicle operation.

Project documentation: [Documentation index](docs/README.md)

## Main application areas

```text
Core/Src/tasks/        RTOS tasks and scheduling ownership
Core/Src/estimator/    SoC/state estimator
Core/Src/measurement/  canonical measurement interface
Core/Src/sop/          State-of-Power, strategy, fuse observer
Core/Src/soh/          State-of-Health
Core/Src/ext_drivers/  ADBMS, CAN, safety, current, IMD, fan, charger, CLI
Core/Inc/...            matching interfaces/contracts
```

A maintainer-oriented module map is available in [`../docs/CODE_ORGANIZATION.md`](../docs/CODE_ORGANIZATION.md).

## Safety architecture

- The safety/error supervisor is the sole normal owner that may assert BMS_OK.
- Measurement freshness/validity and latched diagnostics participate in the permission gate.
- Bench/HIL/service functionality is compile-time constrained by build profile.
- Diagnostic/tuning CAN traffic is passive and lower priority than protected/status traffic.
- Hardware shutdown remains an independent safety layer; firmware does not replace it.

See:

- [`docs/CONCURRENCY_OWNERSHIP.md`](docs/CONCURRENCY_OWNERSHIP.md)
- [`docs/AIR_CONTACTOR_MONITORING.md`](docs/AIR_CONTACTOR_MONITORING.md)
- [`../docs/SAFETY_MODEL.md`](../docs/SAFETY_MODEL.md)

## Battery estimation and power authority

The current architecture includes segment/pack estimation, SoP/SoH logic, fuse observation, and CAN publication of battery power authority. Unvalidated current sign/calibration, stale measurement state, incomplete topology, or other required-invalid conditions are designed to fail closed rather than create battery authority.

Deep passive telemetry for estimator/SoP/fuse tuning is documented in [`docs/AMS_TUNING_CAN_SD_CONTRACT.md`](docs/AMS_TUNING_CAN_SD_CONTRACT.md). The live passive logger schema is protocol v4 and includes full estimator covariance plus acquisition diagnostics.

## CAN contracts

- [`docs/AMS_ECU_CAN_CONTRACT.md`](docs/AMS_ECU_CAN_CONTRACT.md)
- [`docs/AMS_LOGGER_CAN_CONTRACT_V1.md`](docs/AMS_LOGGER_CAN_CONTRACT_V1.md)
- [`docs/AMS_TUNING_CAN_SD_CONTRACT.md`](docs/AMS_TUNING_CAN_SD_CONTRACT.md)

## Host validation

```bash
cd host_tests
make test
```

Focused targets, sanitizer options, profile gates, and bring-up documentation are in `host_tests/README.md` and `host_tests/docs/`.

## STM32 target build

Import this directory directly into STM32CubeIDE. The checked-in `.ioc`, `.project`, `.cproject`, linker scripts, startup code, HAL, and FreeRTOS middleware are retained so the project remains reproducible from its native toolchain.

## Attribution

Original AMS design/code authorship and major 2026 update attribution are summarized in [`../docs/ATTRIBUTION.md`](../docs/ATTRIBUTION.md). Vendor code under `Drivers/` and `Middlewares/` retains its upstream notices.
