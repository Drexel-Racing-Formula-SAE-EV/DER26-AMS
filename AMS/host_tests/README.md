# AMS Host Validation Suite

Host-side verification for DER26 AMS production logic. The harness compiles selected firmware modules against fake HAL/RTOS/device adapters so safety and state-machine behavior can be exercised quickly without changing the STM32 project.

It complements target testing; it does **not** validate physical SPI/isoSPI timing, real sensor scaling, STM32 scheduling/WCET, EMI, wiring, or vehicle safety.

## Vehicle main-branch CI (v0.5.57-r1)

For the dedicated 1 Mbps vehicle firmware package, use:

```bash
make -j2 vehicle-ci
```

This target includes project identity, four CubeIDE configurations, CAN clock,
SoC initialization in both vehicle profiles, vehicle evidence/profile gates,
UART/CAN/ADBMS timing and measurement integrity, plus portable ADBMS2950B
driver regressions. Evidence macros are **synthetic and host-test only**.

The inherited `make unit` target now **links** after adding host-only tick
stubs; it still fails 68 assertions in legacy mixed-topology/bench fixtures.
The inherited `make test`/`make ci` suites also contain unsupported profile
and external-tools assumptions. They are **not** the vehicle release gate.
These failures are retained rather than disabled or silently marked as passed.

Other historical targets below are kept for cross-project test development.

Useful focused targets include:

```bash
make unit
make system-sil
make apm-sil
make fuse-oracle
make asan
make analyze
make production-gates
make static-allocation-gate
```

Use `make help` or inspect the Makefile for the full target list supported by this snapshot.

## What the harness exercises

The suite directly exercises production logic covering areas such as:

- accumulator/cell/temperature bookkeeping;
- voltage/current/temperature fault policy;
- BMS_OK fail-low conjunction behavior;
- ADBMS6830 and ADBMS2950 protocol/state handling;
- current sensing and calibration ownership;
- CAN parsing, filtering, scheduler behavior, and telemetry packetization;
- charger behavior;
- estimator/SoP/SoH/fuse logic;
- retained fault/panic infrastructure;
- RTOS/static-allocation and build-profile gates;
- deterministic state-machine and boundary/fuzz scenarios.

The detailed matrix is in [`docs/TEST_MATRIX.md`](docs/TEST_MATRIX.md).

## Host/target boundary

A PASS means the exercised C logic behaved consistently with the encoded contracts on the host. It does not prove:

- actual ADBMS electrical communication;
- actual cell/AUX mapping or thermistor calibration;
- DHAB/APM current polarity/scaling;
- real CAN transceiver/bus behavior;
- IMD timer capture behavior;
- Cortex-M7 task stack margin or WCET;
- GPIO safe-state behavior during real power/reset transients;
- harness, grounding, EMI, or connector integrity.

Keep those items in the bench/HIL/target validation plan.

## Harness structure

```text
host_tests/
├── src/          comprehensive production-logic runner
├── unit/         focused low-level tests
├── production/   build/profile/source contract checks
├── sop/          SoP/power-state validation and oracles
├── fuse/         fuse model/reference validation
├── tools/        static/protocol/load checks
├── include/      host fake interfaces
└── docs/         test matrix, bring-up references and limitations
```

The harness intentionally leaves the STM32CubeIDE target project untouched.

Focused current-window and CAN provenance regressions (no battery campaign):

```sh
make measurement-integrity-test
```

This target is included in `firmware-ci`. It covers ordering, carry-over,
range mixing, stale/gapped samples, tick wrap, and CAN quality/age/sequence.
