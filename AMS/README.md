# DER 2026 AMS Firmware v0.1.0

Major firmware/test updates by Mahad Faisal, 2026.
Designed and written by Cole Bardin (cab572) and Brendan Hoag (beh73)
Updated: 6/23/2025

## Safety build profiles

The default build is the production-safe profile:

```c
AMS_ENABLE_HIL_CAN=0
AMS_ENABLE_SERVICE_CLI=0
AMS_HIL_REPLACE_ADBMS=0
```

In this profile, CAN HIL frames are recorded by the generic CAN diagnostics but
cannot modify estimator, cell, or temperature state. CLI commands that can
release BMS_OK/balancing, change AMS state, clear the fault log, enable runtime
watchdog controls, recover CAN manually, calibrate current zero, or inject
faults are refused.

For controlled low-voltage bench work, build with `AMS_HW_BRINGUP=1`. This also
enables the service CLI unless `AMS_ENABLE_SERVICE_CLI` is explicitly
overridden. For CAN-fed HIL, explicitly enable `AMS_ENABLE_HIL_CAN=1`.
`AMS_HIL_REPLACE_ADBMS=1` is rejected at compile time unless HIL CAN is enabled.
Never use either service or HIL settings in a vehicle release.

## BMS_OK and IMD fail-safe behavior

The high-priority error/safety task is the sole normal owner permitted to
assert BMS_OK. Other tasks and interrupt/fault paths may force BMS_OK low but
cannot reassert it. The safety supervisor evaluates voltage, temperature,
current, ADBMS diagnostics, task heartbeat, fuse, charger, IMD, RTOS, and
latched hard-fault gates before assertion.

IMD state initializes as `IMD_UNKNOWN`, invalid, and faulted. Since the current
board build still has IMD initialization/task startup disabled, BMS_OK will
remain low until the real IMD capture path is enabled and validated. Do not
replace this with a default-healthy value; use an explicit controlled bench
configuration if temporary bypass testing is required.

## AIR contactor supervision

`AIR_CONTROL_MCU` is only a sense of the existing common control-voltage net;
it is not AIR+, AIR- or precharge physical feedback. The legacy AIR task is not
started in the current hardware profile.

The hardware-independent monitor in `Core/Src/ext_drivers/air_monitor.c` is
fully implemented and SIL-tested, but `AMS_ENABLE_AIR_AUX_FEEDBACK` remains `0`
until protected auxiliary-contact inputs and load-side voltage proof exist. It
implements fresh command/input checks, debounce, boot-open proof, ordered
Off/Precharge/Run/Shutdown transitions, make/release deadlines, precharge and
bus-voltage plausibility, persistent fault masks, and verified-open clearing.
See `Core/docs/AIR_CONTACTOR_MONITORING.md` for the board-adapter contract.

A target build cannot enable the feature without explicitly declaring a
reviewed board adapter, monitor period, and supervisor publication timeout. If
the future task starts without valid configuration/samples—or stops publishing
afterward—the monitor remains fail-closed and the supervisor keeps BMS_OK low.

Host verification:

```sh
cd AMS/host_tests
make test              # full feature exercise profile
make production-gates  # default production gates remain closed
make air-feedback-stub-test # future AIR gate and task-adapter syntax
make unit
make asan
make analyze
```
