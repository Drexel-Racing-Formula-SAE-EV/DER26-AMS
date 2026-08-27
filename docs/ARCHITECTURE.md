# AMS Firmware Architecture

## Responsibility boundary

The AMS owns accumulator-facing measurement, battery-state estimation, battery power authority, and the normal software path that may assert BMS_OK.

Major responsibilities include:

- ADBMS6830 cell-voltage and auxiliary-temperature acquisition;
- pack-current acquisition and current-path diagnostics;
- voltage, temperature, current, communication, fuse, charger, IMD, AIR, and RTOS fault state;
- segment/pack battery-state estimation;
- SoC, SoP, SoH, fuse observation, and power-state publication;
- CAN status, logger, and tuning telemetry;
- charger supervision;
- BMS_OK software permission logic.

The AMS does not replace the independent hardwired shutdown system and does not own low-level inverter/motor control.

## Runtime structure

The application is FreeRTOS-based. Major task ownership is separated rather than allowing unrelated tasks to mutate shared safety state arbitrarily.

```text
ADBMS / current / IMD / AIR / supporting acquisition
                       |
                       v
              canonical measurements
                       |
          +------------+------------+
          |                         |
          v                         v
     estimator                  fault logic
          |                         |
          v                         v
   SoP / SoH / fuse          safety supervisor
          |                         |
          +------------+------------+
                       |
                       v
                CAN publication
                       |
                  ECU / logger
```

Detailed ownership rules are documented in [`../AMS/docs/CONCURRENCY_OWNERSHIP.md`](../AMS/docs/CONCURRENCY_OWNERSHIP.md).

## Data quality model

Safety-relevant consumers are expected to use validity/freshness/diagnostic state rather than treating the presence of a numeric value as proof that the measurement is usable. Multi-frame and estimator data uses sequence/freshness metadata where required by the relevant contract.

## Observability

The firmware publishes compact vehicle-facing state plus passive logger/tuning telemetry. Detailed estimator/SoP/fuse observability is intentionally passive: diagnostic traffic must not become a source of battery or torque authority.
