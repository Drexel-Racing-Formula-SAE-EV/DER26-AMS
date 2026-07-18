# DER26 AMS Host Test Harness


Authored by Mahad Faisal, 2026.
Reusable host-side test harness for the DER26 AMS firmware.

This folder is meant to be copied into the AMS repository as:

```text
DER26-AMS-feature-canbus_charger/
  Core/
  Drivers/
  Middlewares/
  host_tests/        <-- this folder
```

Then run:

```bash
cd DER26-AMS-feature-canbus_charger/host_tests
make test
make system-sil
make apm-sil
make asan
make analyze
```

The harness compiles selected AMS production source files into a desktop executable and replaces board peripherals with fake host-side signals. It is useful before STM32CubeIDE/hardware testing because it catches logic regressions quickly.

## What this does

The harness exercises actual AMS logic from:

```text
Core/Src/ext_drivers/accumulator.c
Core/Src/ext_drivers/canbus.c
Core/Src/ext_drivers/charger.c
Core/Src/ext_drivers/current_sensor.c
Core/Src/ext_drivers/fans.c
Core/Src/ext_drivers/imd.c
Core/Src/tasks/canbus_task.c
Core/Src/tasks/adbms_task.c
Core/Src/tasks/error_task.c
Core/Src/tasks/fan_task.c
Core/Src/tasks/current_task.c
```

It provides fake implementations for the pieces that normally come from hardware or the RTOS:

```text
HAL CAN TX/RX
HAL GPIO read/write
HAL TIM PWM/input-capture
HAL UART
ADC read helper
FreeRTOS static-task create/delay/delete
CMSIS-RTOS tick functions
ADBMS6830 measurement helpers
ADBMS2950 command helpers
BMS_OK output latch
```

## What this does not do

This is not a replacement for hardware validation.

It does not prove:

```text
real ADBMS6830 SPI timing
real ADBMS PEC behavior
real cell voltage scaling against the board
real AUX/temp-channel mapping
thermistor calibration
ADBMS2950 current polarity/APM scaling or divider accuracy
charger byte polarity on the actual charger
IMD PWM behavior on the actual input capture pin
FreeRTOS scheduling on STM32F767
STM32CubeIDE link/startup behavior
pin mapping, connector wiring, or harness wiring
```

Correct interpretation:

> Passing this harness means the host-exercised firmware logic, fault gating, packet layout, bounds checks, and defensive behavior are consistent with the assumptions encoded in the tests.

It does **not** mean the car is safe to energize.

## Requirements

Linux, WSL, macOS, or a similar shell environment with:

```text
gcc
make
libc math library
optional: clang
```

On Windows, easiest path is WSL:

```bash
sudo apt update
sudo apt install build-essential clang
```

## Quick start

From the AMS repo root:

```bash
cp -r /path/to/DER26-AMS-host-test-harness host_tests
cd host_tests
make test
```

Expected output:

```text
PASS accumulator stats/balance
PASS voltage boundary/fuzz stats
PASS temp stats
PASS temp invalid/cold-valid fault behavior
PASS CAN telemetry packetization
PASS telemetry absent segments/invalid channels
PASS charger RX/TX parse
PASS CAN RX filter matrix
PASS charge-state disable matrix
PASS fan/current/null guards
PASS periods and driver edge cases
PASS one-iteration task injection tests
PASS fault matrix extra
ALL COMPREHENSIVE HOST INJECTION TESTS PASSED
```

Run sanitizer tests:

```bash
make asan
```

Run static analyzer:

```bash
make analyze
```

List test functions:

```bash
make list
```

Clean build artifacts:

```bash
make clean
```

## Running without copying into the repo

You can keep the harness somewhere else and point it to the AMS root:

```bash
cd DER26-AMS-host-test-harness
make AMS_ROOT=/absolute/path/to/DER26-AMS-feature-canbus_charger test
make AMS_ROOT=/absolute/path/to/DER26-AMS-feature-canbus_charger asan
```

## Test matrix

| Area | What is injected | What is checked |
|---|---|---|
| Cell voltage stats | Fake ADBMS cell codes | min/max/total voltage, invalid-cell skipping, valid count |
| Voltage fuzz | Deterministic mixed valid/invalid codes | bounded stats, no invalid values counted |
| Temperature stats | Fake NTC raw values | max/average temperature, valid count, invalid-temp skip |
| Invalid temps | all temp channels invalid | `temp_fault` asserts and `BMS_OK` drops |
| Cold valid temps | valid 0 C NTC values | no false temp fault |
| Balancing | pack with one high cell and one low cell | high cells get bounded ADBMS PWM duty, DCC stays off, low/invalid cells not balanced |
| Balancing on faults | invalid/no voltage data | balancing blocked and cleared |
| AMS ECU telemetry | fake state/current/IMD/voltage/temp/fan fields | 62 packets, headers `0..61`, CAN ID, word layout |
| AMS logger telemetry | fake pack/current/temp/fan/fault/ADBMS debug fields | dashboard IDs `0x690..0x6A5`, all cells, all temps, masks, invalid sentinels |
| Missing segments | only first 2 SMBs configured | absent segments zero-filled, no out-of-bounds access |
| Charger RX | fake extended CAN charger frame | voltage/current decode and fault bits |
| Charger RX filter | wrong FIFO status, ID type, ID, or DLC | ignored safely |
| bxCAN hardware filter | exact charger extended ID, explicit HIL standard IDs, configuration failure | correct register packing and fail-closed return |
| Charger TX | charge-state command path | target voltage/current and disable byte behavior |
| Charger timeout | stale `last_rx_tick` | communication fault and charger disable |
| CAN TX timeout | no free mailbox | HAL timeout returned, no infinite loop |
| Fan driver | valid, out-of-range, NaN, Inf | duty clamping and no unsafe cast behavior |
| System SIL boot/readiness | fake current ADC + fake ADBMS scan masks | BMS_OK requires current-valid and voltage-valid together, no false enable before startup scan |
| System SIL voltage degradation | single and repeated fake PEC/missed-cell scans | one missed/PEC-failed cell invalidates the scan and drops BMS_OK; a later full scan recovers |
| System SIL voltage thresholds | fake per-cell OV/UV values through ADBMS task | charge-stop vs hard OV, soft/hard/severe UV/OV, latched diagnostic reasons |
| System SIL current faults | fake DHAB current ADC sequences through current task | warning-only behavior, fast-trip debounce, latch persistence, stale ADC fail-safe |
| System SIL combined faults | current latch + voltage latch + reset calls | BMS_OK stays low until all relevant latches are cleared and data is healthy |
| System SIL ADBMS2950/APM | final-ring init, sample failures, identity/config and CLI actions | APM is observable, bounded and non-safety-gating until final-board validation |
| Full ADBMS6830 open wire | even/odd responses, bad PEC/counter, timeout, injected lead | complete result validation, exact cell mask, no stale overwrite, fail-closed diagnostic |
| Retained fault log | torn commit, CRC damage, metadata corruption, ring rollover | corrupt records rejected and chronology reconstructed |
| RTOS allocation gate | application task source and kernel/CMSIS storage path | fixed task/idle/timer/queue/mutex storage; no dynamic application task creation |
| Current sensor | fake timer capture | current conversion path and fault flag behavior |
| Task loops | one-iteration task runs via fake RTOS delay | state transitions and periodic behavior |
| Error task | hard/soft fault combinations | `BMS_OK` gating and aggregate fault flags |
| Null guards | null handles/devices/payloads | safe error returns, no crash |

## How the harness works

`src/ams_host_test_runner.c` is a single test translation unit. It includes the production AMS `.c` files directly:

```c
#include "Core/Src/ext_drivers/charger.c"
#include "Core/Src/ext_drivers/fans.c"
#include "Core/Src/ext_drivers/current_sensor.c"
#include "Core/Src/ext_drivers/imd.c"
#include "Core/Src/ext_drivers/accumulator.c"
#include "Core/Src/ext_drivers/canbus.c"
#include "Core/Src/tasks/canbus_task.c"
#include "Core/Src/tasks/adbms_task.c"
#include "Core/Src/tasks/error_task.c"
#include "Core/Src/tasks/fan_task.c"
#include "Core/Src/tasks/current_task.c"
```

This approach is deliberate:

1. It avoids changing the STM32CubeIDE project.
2. It lets the host harness provide fake HAL and RTOS symbols.
3. It makes static helper behavior testable in a controlled host process.
4. It keeps the harness as a removable plug-in folder.

The test runner defines a global `app_data_t app` just like the firmware expects. Tests fill that structure with fake pack, charger, CAN, current, and temperature data, then call the production functions or one loop iteration of a task.

One-iteration task testing is done with `setjmp/longjmp`. The fake `osDelayUntil()` advances the fake tick and exits the infinite task loop after one iteration. This allows task behavior to be tested without modifying the firmware task functions.

## Adding a new test

Add a function in `src/ams_host_test_runner.c`:

```c
static void test_my_new_case(void)
{
    init_fake_app();
    fill_nominal_pack(&app, 3.700f);

    /* inject condition */
    app.state = STATE_DISCARGE;
    app.some_fault = true;

    /* exercise code */
    run_one_error_task_iteration(&app);

    /* check behavior */
    CHECK(app.bms_state == false);
}
```

Then call it from `main()`:

```c
test_my_new_case(); puts("PASS my new case");
```

Rules for good tests:

```text
Use init_fake_app() at the start of every test.
Use fill_nominal_pack() when the test is not about invalid pack data.
Check both the primary output and the safety side effect.
For fault tests, check that BMS_OK drops when expected.
For non-fault tests, check that BMS_OK does not drop falsely.
Use exact values only for protocol fields; use ranges for float conversions.
Do not encode guessed hardware scaling as truth unless the schematic/datasheet confirms it.
```

## Debugging failures

A failed check prints:

```text
FAIL src/ams_host_test_runner.c:<line>: <condition>
```

Run under `gdb`:

```bash
make build
gdb build/ams_host_tests
run
bt
```

Run with sanitizers:

```bash
make asan
```

Common failure causes:

```text
AMS protocol header changed but tests still expect old packet numbers.
NSMBS, NCELLS, NTEMPS, or NFANS changed.
Charger CAN ID or byte layout changed.
Fault policy changed but the expected safety state was not updated.
Production source moved to a new path.
New STM32/HAL include dependency was added and needs a host stub.
```

## CI usage

A minimal CI step can run:

```bash
cd host_tests
make test
make asan
make analyze
```

Recommended policy:

```text
Run host tests before every AMS firmware PR.
Require make test to pass.
Require make asan to pass for changes touching safety logic, CAN, charger, or balancing.
Treat make analyze warnings as review blockers unless proven false-positive.
Keep hardware validation separate and documented.
```

## Updating the harness when firmware changes

When team firmware changes, update the harness in this order:

1. Build normally with `make test`.
2. Fix compile failures caused by renamed structs, paths, or functions.
3. Run `make list` and keep test names readable.
4. Update expected packet values only if the ECU/dashboard protocol intentionally changed.
5. Add a regression test for every bug found on hardware.
6. Run `make asan` and `make analyze` before handing code back.

## Current assumptions encoded in this harness

```text
AMS transmits ECU telemetry on standard CAN ID ECU_CANBUS_ID.
AMS telemetry uses 62 frames with packet headers 0..61.
AMS transmits dashboard/logger telemetry on standard CAN IDs 0x690..0x6A5.
The logger contract exports all 75 cell voltages and all 120 temperature sensors.
Cell voltage packets cover 5 SMB segments and 15 cells per segment.
Temperature packets cover 5 SMB segments and 17 temp channels per segment.
Fan packets cover six real fan duty values with zero padding.
Charge-state command uses extended CAN ID CCS_CANBUS_ID.
Charger RX uses extended CAN ID CHARGER_RX_ID.
Invalid/no valid cell voltage data is a voltage fault.
Invalid/no valid temperature data is a temperature fault.
Balancing must not run on invalid voltage data or while voltage/temp checks fail.
Hard safety faults must drop BMS_OK.
```

If any of those assumptions are wrong for the final car, change the production firmware and the harness together.
