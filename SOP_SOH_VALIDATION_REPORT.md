# DER26 AMS v0.3.0 SoP/SoH/strategy validation report

Date: 2026-07-22

## Result

The complete source-level and host-executable release suite passed. v0.3.0 is
suitable for an STM32F767 target build, full five-segment HIL, installed-fuse
characterization, ECU integration, and staged hardware commissioning. It is
not an installed-vehicle torque-authority release.

The requested ideas were not implemented indiscriminately. Mission profiles,
a subtractive fuse observer, cold-readiness/R0-progress diagnostics, and
cause-scheduled recovery were implemented. Autonomous stationary inverter
heating, safety-horizon removal, first-drive torque excitation, and purported
series-segment current shedding were rejected on physics, actuator-evidence,
or safety-boundary grounds. The detailed disposition is in
`Docs/SOP_STRATEGY_FEATURE_REVIEW.md`.

## Verified scope

| Area | Evidence | Result |
|---|---|---|
| Legacy isolated unit suite | 45 named tests | PASS |
| Comprehensive system SIL/injection suite | 104 named tests | PASS |
| ADBMS2950 variant | 45 unit + 104 system tests | PASS |
| Production SoP/SoH/strategy core | 18 named tests | PASS |
| Python differential checks | 8 methods: six solver/oracle methods plus two fuse-observer methods | PASS |
| Strategy/fuse randomized checks | 64 single-bit mission corruptions, 10,000 fuse cycles, 1,000 strategy cases, 300 solver cases | PASS |
| ECU power consumer | Protocol v2, atomic four-frame bundle, two-bundle recovery, CRC/counter/freshness, semantic rejection, and v1 mission encoder | PASS |
| ESP32 dashboard/logger | Required-bundle freshness plus independent `0x689` decoder, JSON rendering, and translation-unit syntax | PASS |
| Thermistor model | 10 unit tests plus all 281 manufacturer-table rows | PASS |
| Sanitizers | ASan + UBSan on unit, system, and strategy core; leak detection disabled for container compatibility | PASS |
| Stress | 50,000 seeded long-fuzz and 12,000 concurrent scheduler-abuse cycles | PASS |
| GCC analyzer | Harness/unit and all 46 `Core/Src` C files in bench and acknowledged vehicle profiles | PASS |
| Profile/build locks | Bench/HIL/vehicle, mission/fuse evidence gates, manifest schema 4, static allocation, state/CAN ownership | PASS |

`arm-none-eabi-gcc`, `idf.py`, and `clang` are not installed in this
environment. The checked-in ARM build script was invoked and stopped at the
missing ARM compiler before producing an object or binary. GCC `-fanalyzer` is
the recorded static-analysis result.

## Strategy and fuse checks

The C and differential suites verify:

- Endurance is the boot/stale-request default and applies the final recovered
  30-second cap to every shorter transmitted horizon;
- Qualify entry requires two sequential, CRC-valid, same-profile requests and
  a stationary-confirmed entry flag, while retaining every hard horizon;
- Limp Home latches at a 30% weakest-segment 3-sigma SoC lower bound and caps
  discharge to `min(30-second DCL, 35 A)`;
- no mission selection can increase any hard SoP result;
- wrong version, reserved bits, CRC, counter, DLC, age, or profile fails the
  mission request back to Endurance;
- the EAC14-80 observer cannot exceed static 118/80/70/70 A ceilings;
- unknown thermal initialization or an unvalidated model remains shadow-only;
- fuse budget accumulation, exponential cooldown, temperature derating,
  100% exhaustion latch, and 50% release hysteresis are finite and bounded;
- the independent Python implementation agrees with 500 seeded C-observer
  transitions and all four dynamic caps;
- invalid fuse state cannot create recovery headroom;
- all reductions are immediate; voltage, thermal, current-path/fuse, and SoC
  recoveries follow their separate rates and release conditions;
- advisory `0x689` loss cannot change the success/failure result of the four
  required power-frame transmissions.

## SoP/SoH regression checks

The v0.2.0 production solver and SoH guarantees remain covered:

- finite/configuration/input validation and deterministic zero fallback;
- calibrated current/polarity, age, complete five-segment topology, all 75
  series-group voltages, all 120 thermistors, estimator/model-domain state,
  balancing recovery, and direction authority;
- weakest-cell voltage, SOC, two-node core/surface temperature, current-path,
  covariance, model-error, capacity-SoH, and resistance-SoH constraints;
- four nested 0.1/1/10/30-second horizons and 16-iteration bisection against a
  brute-force boundary;
- measured-voltage anchoring and per-cell offset preservation;
- capacity rest-anchor observability and retained per-segment R0 upper bounds;
- schema-3 CRC32 persistence, semantic import, incomplete-validity retention,
  newest-slot selection, and conservative aged-state retention across reboot.

## CAN compatibility

Power frames `0x684..0x687` now use protocol version 2 because the binding range
and mission-applied active envelope changed. The required bundle is still
exactly those four frames. `0x689` is independently advisory. Mission request
`0x688` uses protocol version 1.

The paired portable consumer and passive dashboard in this source tree have
been updated. A protocol-v1 ECU fails safely to zero rather than accepting the
new bundle. The portable reference is conformance evidence only: it must be
ported into and HIL-tested with the exact vehicle ECU v2.4.1 transmit, torque,
power-conversion, and shutdown paths before vehicle authority is enabled.

## Host resource measurements

The O2 host benchmark ran 2,000 nominal solves:

| Metric | Measured |
|---|---:|
| Minimum solve | 113.868 us |
| Mean solve | 117.241 us |
| Maximum solve | 513.947 us |
| Feasibility evaluations | 24 |
| Prediction steps | 522 |
| `ams_sop_input_t` | 660 bytes |
| `ams_sop_config_t` | 212 bytes |
| `ams_sop_result_t` | 396 bytes |
| `ams_soh_estimator_t` | 152 bytes |

Host GCC O2 stack-frame report:

| Function | Frame |
|---|---:|
| `ams_sop_solve` | 992 bytes |
| `solve_direction` | 320 bytes |
| `check_current` | 800 bytes |
| `ams_fuse_observer_update` | 96 bytes |
| `ams_power_strategy_update` | 112 bytes |
| `ams_power_state_update` | 1488 bytes |
| `estimator_task_update` | 224 bytes |

The estimator task allocation is 1536 Cortex-M7 words (6144 bytes). Host frame
sizes do not include ARM ABI, complete call-chain, interrupt-stack, or RTOS
effects. Commissioning Gate 1 requires a target DWT maximum below 15 ms and at
least 25% live task-stack margin before authority.

## Required next evidence

This report does not claim:

- STM32F767 compilation/link/map, target flash/RAM, DWT WCET, or live stack
  high-water;
- ESP-IDF link or dashboard hardware operation;
- installed current sign/calibration/uncertainty;
- five-segment/75-cell/120-thermistor HIL;
- electrical or two-node thermal model validation on the installed pack;
- installed EAC14-80, holder, busbar, connector, cable, contactor, or repeated-
  pulse thermal characterization;
- exact ECU protocol-v2 consumer, mission source, torque/power conversion, or
  final-transmit-path integration;
- SoH nonvolatile page allocation and power-cut testing;
- dyno, low-energy vehicle, regen, or competition-rule acceptance.

These are explicit release gates. Keep the vehicle evidence macros zero, keep
regen disabled, and do not grant vehicle torque authority from v0.3.0 until the
applicable commissioning checklist is signed off.
