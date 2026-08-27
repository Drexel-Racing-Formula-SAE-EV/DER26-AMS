# AMS Code Map

| Area | Primary location | Purpose |
|---|---|---|
| RTOS tasks | `Core/Src/tasks/` | acquisition, safety, estimator, CAN, CLI and supporting tasks |
| ADBMS/pack acquisition | `Core/Src/ext_drivers/adbms*.c`, `accumulator.c` | cell/AUX/APM transaction and topology handling |
| Current | `current_sensor.c`, `current_fault.c`, `current_task.c` | DHAB acquisition, filtering, validity and fault policy |
| Safety | `ams_safety.c`, `error_task.c`, fault modules | BMS_OK permission, fault aggregation and panic behavior |
| Estimator | `Core/Src/estimator/` | SoC/electrical state estimation |
| Power/health | `Core/Src/sop/`, `Core/Src/soh/` | SoP, SoH, strategy, fuse and power-state publication |
| CAN | `canbus.c`, `can_tx_scheduler.c`, `canbus_task.c` | protected and passive CAN publication |
| Diagnostics | `cli.c`, `cli_task.c`, `ams_rtos_diag.c` | service/bench observability |
| Tests | `host_tests/` | host SIL/unit/stress/profile/static-contract verification |
