# Migration-readiness code closeout — v2.6.21 / AMS v0.5.24

Date: 2026-09-05

This closeout reviews the open findings in the Zephyr migration plan against the
supplied v2.6.20 source and fixes only items that have a behavior-preserving,
source-level disposition now. It does **not** change battery algorithms, current
conversion, CAN scheduling policy, watchdog policy, bus-off safety policy,
BMS_OK ownership, balancing authority, or hardware pin mapping.

The SPI String-B chip-select firmware mapping remains **PE4**. The earlier PF4
schematic annotation is treated as a schematic labeling error and no firmware
pin remap is made.

## Findings closed or corrected in source/tooling

### F-02 — CubeMX DER25 identity

Already closed in the supplied v2.6.20 source. `DER26-AMS.ioc` identifies both
the project name and project file as DER26-AMS, and the release-identity gate
checks those fields. The migration plan was stale on this point.

### F-03 — SPI CS_B PE4/PF4

Closed as documentation-only. PE4 is retained as the firmware/`.ioc` contract.
No source or pin-map change is made. The release-identity gate now rejects drift
from PE4/`CS_B` in both `main.h` and `DER26-AMS.ioc`.

### F-04 — CAN ISR concurrency documentation

Already closed in the supplied v2.6.20 source. The concurrency documentation
states that CAN ISR paths use FreeRTOS ISR APIs, and the CAN IRQ gate checks the
RX0/TX/SCE priorities against the configured FreeRTOS syscall ceiling and
CubeMX values.

### F-10 — vehicle CLI/UART surface

Added an independent `AMS_ENABLE_CLI` build-profile gate:

- BENCH, BENCH_VALIDATION, TESTDAY and HIL: `AMS_ENABLE_CLI=1`;
- VEHICLE: `AMS_ENABLE_CLI=0`;
- service CLI mutation cannot be enabled when the diagnostic CLI transport is
  disabled;
- VEHICLE cannot override the diagnostic CLI back on.

With CLI disabled, source startup no longer initializes USART3 for CLI, arms RX,
initializes the board CLI endpoint, creates the CLI task, or processes the UART
CLI callbacks. The build manifest schema is bumped to 6 and records the CLI
feature bit independently of service-mutation authority.

The final ARM link map must still be archived to prove dead CLI sections are not
present in the qualified vehicle binary; no ARM target compiler was available
in this review environment.

### F-12 — FreeRTOS source provenance portion

Added `host_tests/tools/check_freertos_provenance.py` and
`docs/FREERTOS_PROVENANCE.md`.

The bundled FreeRTOS source tree is pinned by an exact deterministic SHA-256:

`4393c390c3939c1ce11c713c9b862ec1b27cd9a3448e73a7a48c9c11dbdc3824`

The gate also records the existing metadata disagreement instead of falsifying
vendor history: `task.h` reports V10.2.0 while source banners contain V10.2.1.
This closes ambiguity about the exact checked-in bytes. F-12 remains partially
open until approved ARM Debug/Release artifacts and toolchain provenance are
produced and archived.

### F-13 — Release build time/provenance

The headless STM32 Release build now requires an explicit `SOURCE_DATE_EPOCH`.
It refuses an uncontrolled Release invocation rather than silently embedding
wall-clock time. The build also writes `BUILD_PROVENANCE.txt` and
`SHA256SUMS.txt`, including compiler identity, firmware revision, exact
FreeRTOS-tree provenance, linker-script hash and CubeMX `.ioc` hash.

Debug/IDE builds may still use wall-clock metadata and are not the qualified
reproducible Release path.

## Findings intentionally left open

These do not have a safe evidence-preserving source-only fix:

- **F-05 watchdog policy** — requires an approved requirement for external
  process faults versus software-liveness faults.
- **F-06 CAN bus-off discharge behavior** — requires a system-level stale-data
  policy including ECU behavior.
- **F-07 WCET/IRQ-off/jitter** — requires target measurements; source
  instrumentation alone would not close the finding.
- **F-08 current H path** — physical/electrical issue; no software calibration
  or pin swap is justified.
- **F-09 validation/evidence reconciliation** — requires linking accepted bench
  evidence to exact hardware/firmware/configuration, not repeating completed
  tests by default.
- **F-11 FreeRTOS heap/timer/newlib configuration** — requires allocation and
  libc/CMSIS audit before disabling dynamic facilities.
- **F-12 target release artifacts** — approved ARM compiler/toolchain is not
  present in this environment.
- **F-14 SRAM/linker-bank policy** — requires target MAP/cache/DMA/stack evidence
  before changing the memory layout.

## Verification performed

From `AMS/host_tests`:

```text
make profile-gates            PASS
make review-fixes-test        PASS
make whole-source-analyze     PASS (BENCH + VEHICLE source analysis)
```

The updated focused review suite contains 13 tests and verifies the new vehicle
CLI gating, invalid service-CLI-without-CLI configuration rejection, deterministic
Release-time requirement, canonical identity/IRQ contracts and exact FreeRTOS
source provenance.

No full MiL campaign was rerun because this patch does not touch the battery
algorithm or measurement paths covered by those campaigns. No ARM target build,
flash, new WCET measurement or hardware validation is claimed.
