# Code Organization

This map focuses on project-authored firmware. Vendor HAL/CMSIS and FreeRTOS code remain under `Drivers/` and `Middlewares/`.

## RTOS tasks — `AMS/Core/Src/tasks/`

- `adbms_task.c` — accumulator acquisition and ADBMS scan ownership.
- `current_task.c` — current acquisition/processing.
- `estimator_task.c` — estimator and power-state cadence.
- `canbus_task.c` — CAN publication/service.
- `error_task.c` — fault aggregation and BMS_OK supervision.
- `cli_task.c` — service/bench diagnostics.
- `fan_task.c`, `imd_task.c`, `air_task.c` — supporting supervised functions.

## Estimation — `AMS/Core/Src/estimator/`

Battery state estimator and estimator parameter/look-up logic.

## Measurement — `AMS/Core/Src/measurement/`

Canonical measurement/state interface used to decouple acquisition details from estimator and safety consumers.

## Power/health — `AMS/Core/Src/sop/`, `AMS/Core/Src/soh/`

State-of-Power, power strategy, fuse observer, power-state publication, and State-of-Health logic.

## Board-facing services — `AMS/Core/Src/ext_drivers/`

The historical directory name is broader than the contents now imply. Major groups include:

- ADBMS/accumulator — `adbms6830.c`, `adbms2950.c`, `adbms_shared.c`, `accumulator.c`;
- communications — `canbus.c`, `can_tx_scheduler.c`, `charger.c`, `cli.c`;
- safety/diagnostics — `ams_safety.c`, `ams_rtos_diag.c`, voltage/current/temperature fault modules and observers;
- sensors/actuators — current sensor, fan, IMD, AIR monitor, thermistor model.

See [`../AMS/docs/CODE_MAP.md`](../AMS/docs/CODE_MAP.md) for the concise firmware-local map.
