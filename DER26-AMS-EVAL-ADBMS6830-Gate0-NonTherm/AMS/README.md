# DER 2026 AMS Firmware v0.1.0

Major firmware/test updates by Mahad Faisal, 2026.
Designed and written by Cole Bardin (cab572) and Brendan Hoag (beh73)
Updated: 6/23/2025

## Safety build profiles

This branch defaults to the dedicated physical `EVAL-ADBMS6830BMSW` bench
profile. It is a one-device, 16-cell, String-B monitor-only image with BMS_OK
compile-time locked low and every ADBMS actuator path disabled. See
[`../EVAL_ADBMS6830BMSW_BRINGUP.md`](../EVAL_ADBMS6830BMSW_BRINGUP.md) before
building or wiring it.

```c
AMS_EVAL_ADBMS6830_BMSW=1
AMS_BUILD_PROFILE=AMS_PROFILE_BENCH
AMS_ENABLE_HIL_CAN=0
AMS_ENABLE_SERVICE_CLI=1
AMS_HIL_REPLACE_ADBMS=0
```

The eval profile enables only the controlled service CLI needed for bench
diagnosis. It blocks BMS_OK release, balancing, state changes, DER thermistor
mux traffic, open-wire/self-test stimulation, APM2950 access, raw SPI patterns,
fan output and the estimator. It is incompatible with CAN-fed ADBMS HIL by
design: HIL replaces physical measurements and therefore cannot test SPI.

Set `AMS_EVAL_ADBMS6830_BMSW=0` before selecting any ordinary bench, CAN-HIL or
vehicle profile. The explicit vehicle profile remains impossible to compile
until all release-evidence gates and manifest revisions are provided. No profile
in this repository, including the eval profile, is itself authorization for HV
operation.

Normal firmware scans the ADBMS chain at 10 Hz. `AMS_HW_BRINGUP=1` deliberately
uses the slower 1 Hz bench profile while BMS_OK and balancing are inhibited.
Status, configuration, and full even/odd open-wire periods are specified in
milliseconds and converted to scan counts for the selected profile, avoiding
the old accidental multi-minute intervals. Actual cadence still depends on
measured target execution time and jitter.

The present vehicle hardware—not this firmware—owns contactor precharge
sequencing. `STATE_START` is therefore software initialization only. Once all
implemented software safety inputs are valid, the supervisor changes to the
normal discharge/drive current policy, keeps BMS_OK low for that iteration, and
waits for a fresh current result under the new policy before a permit is
possible. This does not prove precharge completion or physical AIR position.

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
make static-allocation-gate # application/kernel runtime objects use fixed storage
make air-feedback-stub-test # future AIR gate and task-adapter syntax
make unit
make asan
make analyze
```

All nine application tasks use fixed TCB/stack storage. The ADBMS recursive
mutex and FreeRTOS idle task, timer task, and timer queue are also statically
backed. The vendor CMSIS wrapper retains dynamic-allocation support at compile
time for compatibility, but the application startup path does not create its
tasks dynamically.

The bxCAN hardware filter admits only the charger extended data-frame ID in a
normal build. Explicit HIL builds add only the five approved standard HIL IDs;
the ISR/task allowlist and DLC checks remain a second validation layer.

The retained fault ring is schema-versioned and uses persistent boot/event
sequences, per-record CRC, and commit-last publication. Corrupt or interrupted
records are rejected and ring progress is reconstructed from valid records
after reset. Upgrading from the previous unversioned ring intentionally clears
old entries because their integrity cannot be established.
