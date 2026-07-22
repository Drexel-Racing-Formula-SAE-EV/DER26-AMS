# DER 2026 AMS Firmware v0.3.0

Major firmware/test updates by Mahad Faisal, 2026.
Designed and written by Cole Bardin (cab572) and Brendan Hoag (beh73)
Updated: 6/23/2025

## Safety build profiles

The default build is the production-restricted profile. It closes HIL/service
backdoors but is not a vehicle/HV release:

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
bench build. Explicit HIL builds add only the five approved standard HIL IDs;
the vehicle profile additionally admits the supervised `0x688` mission
request. The ISR/task allowlist and exact-DLC checks remain a second validation
layer.

The retained fault ring is schema-versioned and uses persistent boot/event
sequences, per-record CRC, and commit-last publication. Corrupt or interrupted
records are rejected and ring progress is reconstructed from valid records
after reset. Upgrading from the previous unversioned ring intentionally clears
old entries because their integrity cannot be established.

## Dynamic power and State of Health

The estimator path now contains a production-intent, fail-closed SoP/SoH
subsystem for the 75s6p P42A accumulator. It consumes five segment DADEKF
states, all 75 cell voltages, all 120 thermistors, calibrated current and
uncertainty, predicts electrothermal behavior at 0.1/1/10/30-second horizons,
and publishes power-protocol-v2 DCL/CCL/SoH frames at `0x684..0x687`.

The default source calibration uses actual DER26 system limits rather than cell
array capability: 118/80/70/70 A discharge and 11.5/10/10/10 A charge magnitude
at the four horizons. Unknown capacity or resistance health uses explicit 0.80
and 1.25 conservative priors. Invalid data, incomplete segment/sensor topology,
unvalidated current sign/calibration, or stale publication produces zero
limits.

v0.3.0 adds a strategy layer that cannot increase this physical envelope.
Endurance caps transients to the 30-second capability, Qualify uses the
recovered 1-second capability without removing any hard horizon, and Limp Home
latches from the weakest segment's conservative 30% SoC threshold. ECU mission
requests use protected CAN ID `0x688`; advisory strategy/fuse/readiness status
uses `0x689`. A calibration-gated EAC14-80 observer can only subtract from the
static current ceilings, and recovery rates are scheduled by voltage, thermal,
current-path/fuse, or SoC cause.

The implementation does not make a development image vehicle-ready. The
vehicle profile additionally requires SoP model, calibration, power/mission
CAN-contract, and fuse-model evidence macros plus immutable SoP/SoH revision
strings. Regen remains disabled until independently commissioned. Target
WCET/stack evidence, full-segment HIL, installed hardware characterization,
exact ECU integration, and SoH nonvolatile storage tests remain required.

Design and release procedures:

- `Docs/DYNAMIC_SOP_IMPLEMENTATION.md`
- `Docs/SOP_SOH_CALIBRATION_AND_LIMITS.md`
- `Docs/SOP_SOH_CAN_CONTRACT.md`
- `Docs/SOP_SOH_COMMISSIONING.md`
- `Docs/SOP_STRATEGY_FEATURE_REVIEW.md`
