# DER26 AMS C Firmware Hardening Review

Review date: 2026-07-14
Reviewed baseline: `main-mahad` at `40369dc`
Hardened branch: `c-hardening-review`
Final code commit reviewed: `4de44cf2b9589cdac4fa0013bffc92c6c35a2394`

## Executive result

The current C firmware has been hardened in four reviewable commits and passes
all host-side tests available in this workspace. The changes close concrete
concurrency, interrupt-context, ignored-output-write, numeric-conversion, input
validation, and peripheral-startup failure modes.

This is **not clearance to energize the accumulator, tractive system, or the
12 V BMS safety-output circuit**. No physical hardware was powered during this
review. The earlier Q2 MOSFET failure remains an electrical/root-cause issue
until the assembled PCB, device pinout/orientation, connected load, and gate/
source voltages are validated under a current-limited bench procedure.

The normal hardware build is deliberately fail-closed because the IMD task and
its timer/channel mapping are still disabled and unvalidated. Consequently,
the reviewed source should not assert BMS_OK in that configuration. This is a
release blocker, not a reason to bypass the IMD gate.

## Scope and method

The review covered the STM32F767 application, FreeRTOS task interactions,
ADBMS6830/ADBMS2950 access, accumulator measurement and balance control, CAN
receive/transmit handling, charger gating, current/temperature/voltage fault
paths, IMD and fan drivers, CLI service commands, watchdog/fault logging, and
the host test harness.

The uploaded/source baseline was preserved. Changes were made on a separate
branch in these commits:

1. `21cd803` — Harden ADBMS locking and shared safety state
2. `0821887` — Defer CAN RX processing out of the ISR
3. `23cd815` — Harden service inputs and balance fault handling
4. `4de44cf` — Expose peripheral startup failures

The cumulative change from `40369dc` is 25 files, 1,525 insertions, and 246
deletions. Most added lines are defensive code and fault-injection tests.

## Firmware ownership map

| Area | Normal owner | Safety rule after hardening |
| --- | --- | --- |
| BMS_OK high assertion | `error_task` supervisor | Only the supervisor may assert high after one atomic readiness decision; any safety path may force low. |
| ADBMS SPI6 transaction sequences | ADBMS task or serialized CLI action | A recursive, priority-inheriting mutex covers a whole logical operation; mutex failure enters the panic path. |
| Cell/temperature scan state | ADBMS task | Scan freshness, PEC, plausibility, diagnostics, and balance-write state all feed fail-closed gates. |
| CAN RX hardware callback | CAN ISR | ISR only copies a validated frame into an SPSC ring; parsing and application mutation occur in task context. |
| Charger command/gating | CAN task | Charge enable is recomputed from current safety gates and charger freshness; bus-off forces the charge path low. |
| Current fault state | Current task | Invalid/stale ADC pairs and confirmed current faults force BMS_OK low. |
| Fan command | Fan task | Failed PWM initialization or command is observable as `fan_fault`; current policy treats it as a soft fault. |
| Fault log and heartbeat state | Multiple tasks/ISRs | Updates and snapshots use short critical sections; the watchdog consumes a coherent safety view. |

## High-impact findings fixed

### 1. ADBMS operations could interleave

The original locking granularity did not protect an entire wake/convert/read/
diagnostic/balance sequence. CLI diagnostics could interleave with periodic
work while the drivers also used shared scratch buffers.

The hardened code uses a static recursive mutex with priority inheritance,
holds it around a complete periodic scan or service operation, and treats
mutex create/acquire/release failure as a safety panic. Driver scratch buffers
were made file-local. Tests cover nested locking and production ownership
rules.

### 2. CAN RX performed application work in interrupt context

The receive callback previously parsed charger/HIL traffic and mutated shared
application state. That increased ISR latency and created task/ISR ownership
races.

The callback now validates the CAN handle, copies one frame into a 96-entry
single-producer/single-consumer ring, and returns. Task context performs parsing
using the original receipt timestamp. The ring has 95 usable entries, which is
larger than the 65-frame full HIL accumulator image. Remote frames are not
parsed as data. Queue overflow and HAL receive failures are counted and become
an observable CAN soft fault.

Tests cover FIFO order, wraparound, exact capacity/overflow accounting, wrong
handles, remote frames, deferred HIL mutation, and HAL receive failure.

### 3. Balance/DCC write failures were incompletely handled

Several balance set/clear return values were ignored, including the end of the
periodic scan. A service state transition could also leave bleed outputs active
until the next scan.

All balance writes now pass through one result handler. Failure latches a
dedicated ADBMS balance-write diagnostic, saturates a failure counter, records
a fault event, and immediately forces BMS_OK low. A later confirmed successful
write clears only that balance-write fault, preserving unrelated ADBMS or CAN
diagnostics. State changes first force BMS_OK low and synchronously clear
balancing before publishing the new state.

### 4. Shared heartbeat/fault-log data could be torn

Heartbeat masks/timestamps and the retained fault-log ring are accessed by
multiple execution contexts. The hardened implementation uses short critical
sections for coherent updates and snapshot reads. The production ownership
test also enforces that readiness is evaluated and BMS_OK is updated inside the
same supervisor critical section.

### 5. CLI numeric input accepted malformed service commands

`atoi` treated malformed strings and overflow as zero, so input such as `abc`
could select channel zero. Scope repeat arguments also accepted partial input
and silently clamped large values.

Service integers now use `strtol` with range, overflow, and complete-string
validation. Tests cover malformed, trailing-junk, overflow, upper-bound, and
valid boundary cases.

### 6. Float-to-integer telemetry conversions could be undefined

NaN, infinity, or extreme finite values could reach direct casts used by CLI,
CAN, current fault logs, and fan PWM calculations. Those conversions are now
finite-checked and saturating. Full-scale fan PWM uses an exact integer path so
`UINT32_MAX` cannot round through an out-of-range float conversion.

### 7. IMD capture validation was optimistic

IMD timer-start return values were ignored, capture arithmetic could overflow,
and invalid capture relationships could publish a status. Initialization now
records each HAL startup result; reads require a started capture path and
nonzero clock, reject `high_count > total_count`, avoid integer overflow, and
clear all published values on failure.

This improves the driver but does not resolve the release blocker: `board.c`
still comments out `imd_init`, `app.c` does not start the IMD task, and the
source itself identifies the capture-channel mapping as unconfirmed.

### 8. Peripheral startup failures could masquerade as success

An ADBMS delay-timer start failure left a valid-looking handle that could enter
an unbounded counter wait. A fan PWM-start failure left enough fields populated
for later CCR writes to look successful. CAN start/notification results were
discarded.

The ADBMS driver now receives a delay timer only after `HAL_TIM_Base_Start`
succeeds; otherwise the timing diagnostic remains blocking and BMS_OK stays
low. Fan objects require a successful PWM start before accepting duty commands.
CAN startup/notification status is retained, shown by the CLI, and an initial
failure enters the existing timed recovery path.

## Verification matrix

All rows below passed at final code commit `4de44cf`.

| Profile | Result |
| --- | --- |
| `make ci BUILD_DIR=/tmp/der26-final-ci` | 30 unit tests, 80 comprehensive injection/SIL tests, production feature gates, BMS_OK ownership gate, and GCC static analyzer passed. |
| `make asan BUILD_DIR=/tmp/der26-final-asan` | Unit and comprehensive suites passed under AddressSanitizer plus UndefinedBehaviorSanitizer. |
| `make ubsan BUILD_DIR=/tmp/der26-final-ubsan` | Unit and comprehensive suites passed under standalone UBSan. |
| `make stress BUILD_DIR=/tmp/der26-final-stress` | Comprehensive suite passed with 50,000 long-fuzz cycles and 12,000 concurrent scheduler-abuse cycles. |
| `make hil-adbms-test` | Dedicated ADBMS-image replacement profile passed. |
| `make bringup-test` | Hardware-bring-up compile profile and comprehensive tests passed. |
| `make safety-test` | IWDG-enabled compile profile and comprehensive tests passed. |
| Per-file GCC syntax checks | `app.c`, `board.c`, accumulator/CAN/fan drivers, and ADBMS/CLI/fan tasks passed with the production include set. |
| `git diff --check` | No whitespace/error markers. |

The host suite exercises boundary thresholds, stale/invalid measurements,
PEC misses, task ordering, charge-disable precedence, balancing, CAN bus-off
and recovery, RX overflow, HIL replacement, watchdog gating, fault logging,
stack/heap diagnostics, startup garbage, fuzzed state, and concurrent task
interleavings.

## What was not verified

The following are outside the evidence produced by this review:

- No STM32 target compile/link was run because `arm-none-eabi-gcc` is not
  installed and this CubeIDE project has no checked-in generated target
  Makefile.
- Clang, clang-tidy, and cppcheck are also unavailable in this workspace.
- No firmware was flashed, no UART/ST-Link session was opened, and no board was
  powered.
- Host tests cannot validate PCB routing, footprint/pin mapping, isolation,
  MOSFET SOA, gate transients, current paths, SPI signal integrity, PEC behavior
  on a real daisy chain, interrupt timing, RTOS stack use on Cortex-M7, or the
  physical watchdog reset interval.
- The Tracealyzer project/archive was preserved but was not used as proof of
  target timing because there was no fresh target trace from this firmware.

## Release blockers and residual risks

These items should remain open before any vehicle/HV approval.

### Blockers

1. **Resolve the Q2 electrical failure before applying 12 V again.** Verify the
   exact fitted part, footprint/pinout/orientation, unpowered resistance and
   continuity, connected load, and gate/source/drain voltages with current
   limiting. Firmware review does not establish the root cause.
2. **Integrate and validate the IMD.** Confirm the exact timer instance,
   channels, pin, clock, PWM frequency/status mapping, stale timeout, and
   hardware fault cases. Then enable `imd_init` and the IMD task without
   weakening the supervisor gate.
3. **Run a real target build.** Use the approved STM32CubeIDE/ARM GCC version,
   inspect all warnings, link/map output, flash/RAM use, FPU ABI, and startup
   objects. Archive the exact ELF, map, compiler version, and build flags.
4. **Validate timing requirements.** `ADBMS_FREQ` is currently 1 Hz and is
   explicitly commented as a test setting. Confirm voltage/temperature fault
   response time, conversion delay, heartbeat limits, and rules compliance.
5. **Decide and validate production watchdog policy.** `AMS_ENABLE_IWDG`
   defaults to 0. The enabled profile is host-tested, but target reset timing,
   startup behavior, and service/recovery procedure require hardware testing.
6. **Approve safety thresholds and polarity/calibration.** Temperature limits
   are marked TODO in source. Current-sensor polarity/ranges, ADBMS conversion
   scaling, thermistor calibration, and pack topology must be checked against
   the released schematic/BOM, datasheets, and measured hardware.

### Policy decisions requiring owner sign-off

- A fan startup/command failure is observable but remains a soft fault and does
  not by itself block BMS_OK. Confirm that policy against thermal safety needs.
- A general CAN fault is soft outside the charge/HIL safety cases; charger
  failure in charge does block. Confirm which vehicle states require CAN for a
  safety function.
- The legacy ECU telemetry contract exports 17 temperatures per segment while
  the hardware model contains 24. Safety calculations use the full internal
  set, but dashboard/logger consumers need a documented contract.
- ADBMS2950/APM scaling and shunt polarity remain explicitly unproven; that
  path is disabled by default and must stay non-gating until validated.

### Lower-severity technical debt

- UART diagnostic counters are multi-context `volatile` counters and may lose
  diagnostic increments under simultaneous updates; they do not drive safety.
- A failed `HAL_ADC_Stop` is not propagated from the board ADC helper. The next
  failed start/read becomes invalid and is handled fail-closed, but explicit
  stop-status telemetry would improve diagnosis.
- CLI fixed-point display helpers cannot preserve a negative sign for values
  strictly between -1 and 0; control/fault calculations are unaffected.
- ADBMS driver scratch storage is now file-local and protected by the firmware
  mutex contract, but the low-level drivers are not independently reentrant.

## Required staged hardware validation

Do not begin with the full accumulator or tractive system. Use the team’s
approved electrical safety procedure and require an independent wiring review.

1. Perform unpowered schematic-to-PCB continuity, shorts, footprint, polarity,
   isolation, and connector pinout checks, including the Q2 circuit.
2. Build the exact normal and bring-up profiles with ARM GCC; retain ELF/map and
   verify that HIL and service actions are compiled out of the normal image.
3. Power only the low-voltage domain from a current-limited, fused source.
   Confirm rails, reset state, BMS_OK low, CS lines inactive, and current draw
   before connecting ADBMS/IMD/charger loads.
4. Validate UART diagnostics and CAN startup/recovery with outputs isolated.
5. Validate TIM1 timebase, ADBMS wake/PEC/config readback, and one known-safe
   monitor chain; inject open/stale/PEC/write failures and confirm BMS_OK stays
   low.
6. Validate current sensing at zero and known bidirectional currents, then
   temperature channels with known resistances/temperatures.
7. Integrate and fault-inject the IMD, then enable its task only after channel
   and mapping evidence is recorded.
8. Verify fan PWM electrically and decide whether fan failure must be a hard
   gate.
9. Run the target watchdog and starvation tests, capture a fresh RTOS trace,
   and measure stack high-water marks and worst-case task/ISR latency.
10. Only after the preceding gates pass should the project’s authorized HV
    procedure consider staged accumulator/tractive testing.

## Recommendation before C++ migration

Keep this C branch as the behavioral reference. Complete the target/hardware
blockers and preserve the passing host vectors first. A future C++ migration
should be incremental: compile selected `.c` modules as C-compatible units,
wrap HAL/RTOS boundaries without changing wire protocols or thresholds, and
require the same 30-unit/80-comprehensive, production-gate, sanitizer, stress,
and target-hardware evidence at every step.

## 2026-07-17 AIR deep-hardening addendum

The future AIR auxiliary-contact scaffold received a separate adversarial
review and hardening pass. It now enforces terminal/non-permitting Shutdown,
explicit Off re-arm, complete prior-snapshot authority for closing transitions,
current-snapshot precharge proof, energized-phase latching of supervision loss,
cross-sample timestamp coherence, schedule/debounce feasibility, immutable
accepted configuration, all-or-nothing board reads, stronger supervisor mask
checks, and separate-object host compilation.

See `AIR_DEEP_HARDENING_REVIEW.md` for the findings, final behavior, verification
matrix and required hardware/target exit gates. The feature remains disabled on
the current PCB and this addendum is not authorization for HV testing.
