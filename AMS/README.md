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

Host verification:

```sh
cd AMS/host_tests
make test              # full feature exercise profile
make production-gates  # default production gates remain closed
make unit
make asan
make analyze
```
