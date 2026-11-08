# Charge-only consolidation — 2026-11-08

## Source baseline
- Charge-only v0.5.56 250k CubeIDE release (primary, preserved charge-only configs).
- Latest APM-EVAL6822-DIAG5R3 source (common firmware + diagnostic driver changes).
- Vehicle-1M-CLI4 source (included in APM-derived generic timing/CLI changes).

## Merged source files
- `host_tests/Makefile`
- `host_tests/production/cli_async_test.c`
- `host_tests/production/adbms_wake_policy_test.c`
- `host_tests/unit/ams_unit_test_runner.c`
- `host_tests/unit/apm_regression.c`
- `host_tests/unit/apm_diag5_test.c`
- `host_tests/unit/apm_link_clock.c`
- `host_tests/unit/measurement_integrity_test.c`
- `host_tests/src/ams_host_test_runner.c`
- `Core/Inc/app.h`
- `Core/Src/stm32f7xx_it.c`
- `Core/Src/tasks/adbms_task.c`
- `Core/Src/tasks/error_task.c`
- `Core/Src/tasks/cli_task.c`
- `Core/Src/tasks/canbus_task.c`
- `Core/Src/ext_drivers/adbms2950.c`
- `Core/Src/ext_drivers/accumulator.c`
- `Core/Src/ext_drivers/canbus.c`
- `Core/Src/ext_drivers/adbms6830.c`
- `Core/Src/ext_drivers/cli.c`
- `Core/Inc/ext_drivers/cli.h`
- `Core/Inc/ext_drivers/accumulator.h`
- `Core/Inc/ext_drivers/adbms2950.h`
- `Core/Inc/ext_drivers/adbms6830_data.h`
- `Core/Inc/ams_build_profile.h [profile-merge]`
- `Core/Inc/app.h [CAN-merge]`

## Intentional exclusions
- 1-Mbit/s CAN configuration; vehicle driving profile, vehicle .ioc, and mission-CAN mode.
- Standalone APM EVAL6822 project flags, SPI6 prescaler overrides and bench initialization/configuration.
- Eval-only docs, historic evidence and previous execution logs are not presented as new test results.
- Charger-only configuration remains the source of truth for cell observation masks and charging policy.

## Validation
Executed in this consolidation using the system-native host GCC (not ARM target GCC):

| Test target | Outcome |
| --- | --- |
| `make charge-only-ci` (charger-only, charge-review, CLI async, temperature integrity, ADBMS wake, task schedule, testday cell policy, APM regressions) | **PASS** |
| `make apm-diag5-check` including standalone host-only APM driver/CLI checks | **PASS** |
| `make unit` legacy comprehensive unit harness | **FAIL** (67 expectations; pre-existing specialized charge baseline fails 42 expectations). Needs separate test-harness reconciliation; not qualified. |
| `make test` generic profile-1/vehicle SIL harness | Not compatible with this deliberately profile-4/6-only charger project. |
| `arm-none-eabi-gcc` / STM32CubeIDE compile and MCU link | **NOT RUN** (ARM toolchain unavailable here) |
| Hardware/charger test | **NOT RUN** |

The standalone APM diagnostic headers can be selected **only for host tests** when `AMS_HOST_TEST` and `DER_APM_STANDALONE` are both present; the CubeIDE charge-only project cannot select that standalone mode.

Tests and captured output: `validation/consolidation/`. Neither the old archived logs nor the host-only APM regression establishes target hardware operation.

## Release configuration contract
- CubeIDE retains four configurations: `CHARGER LV 250k` Debug/Release (`AMS_BUILD_PROFILE=4`) and `75S 74OBS CHARGE-REVIEW 250k` Debug/Release (`AMS_BUILD_PROFILE=6`).
- CAN=250 kbps; no vehicle mission CAN or tuning CAN; target 312 V / 10 A; 75 physical groups with 74 observed inputs; initial SoC 27% in profile 6.
- Profile 6 retains the v0.5.56 charge-review charging behavior and balancing inhibition. The new timing/temperature checks may inhibit charger output when data are stale; they do not bypass existing interlocks.
- Debug and Release images were not generated. CubeIDE import and ARM link remain outstanding.

## Important source/hardware limit
S1C1 is still connected and unobserved in this source. The inherited inferred-C1 calculation
cannot replace direct cell monitoring, and no ARM compile or charging hardware test is claimed.
