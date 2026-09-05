# AMS review fixes — 2026-09-05

Package v2.6.18; firmware source `DER26-AMS-v0.5.21-20260905`.
Based on v2.6.17 / v0.5.20, retaining all six earlier CAN scheduler fixes.
No Zephyr code, algorithm tuning, hardware pin remapping, authority enabling,
or safety-policy changes are included.

## Fixed

- F-01: canonical `Core/Inc/ams_version.h` supplies numeric CLI version and
  derived source revision. `VER_*` aliases cannot silently drift from it.
- F-02: CubeMX project name/file now identify DER26. `target-project-gate`
  checks both exact `.ioc` fields and the existing CubeIDE metadata.
- F-04: concurrency documentation now correctly states that CAN ISR paths use
  FreeRTOS ISR APIs. The IRQ gate checks all RX0/TX/SCE HAL priorities against
  the FreeRTOS syscall limits, zero subpriority, and matching `.ioc` settings.
- F-12 (build tooling portion): headless script supports explicit Debug (`-O0`)
  and Release (`-O2`) modes, optional project warnings-as-errors, stack-usage
  files, source/debug path normalization, and tool preflight. It resolves the
  repository from its own location and creates a unique output directory,
  preserving previous build evidence instead of deleting `AMS/build`.
- F-13 (timestamp consistency/support): CLI reads the single `app.c` manifest
  timestamp, not its own compilation-unit timestamp. The shared header supports
  explicit `AMS_BUILD_DATE`/`AMS_BUILD_TIME`; the headless build accepts and
  validates `SOURCE_DATE_EPOCH`. Unconfigured IDE builds remain wall-clock dated.
- Corrected stale STM32F407 file-name comments in the F767 platform adapter.

## Targeted verification

Run from `AMS/host_tests`:

```sh
make review-fixes-test
```

The nine focused tests include negative fixtures for stale CLI version, stale
CubeMX name/file, per-file timestamps, illegal CAN IRQ priority, changed RTOS
syscall ceiling, and CubeMX/HAL IRQ drift. A compiled host probe verifies the
canonical source revision and a fixed-epoch timestamp. These are source/host
checks, not on-target timing or peripheral validation.

Results in this workspace: all nine tests PASS; CubeIDE/CubeMX identity and CAN
IRQ contract gates PASS; profile compile/rejection gates PASS; modified app/CLI
sources pass host `-fsyntax-only` with project warnings treated as errors in
BENCH_VALIDATION configuration. Shell syntax and patch whitespace checks PASS.
The Release build invocation stops cleanly at missing ARM compiler preflight,
before creating or deleting target build output. No MiL or full SIL campaign
was rerun.

## Headless target builds

With ARM GNU tools installed, run from any directory:

```sh
AMS_BUILD_TYPE=Debug bash ci/stm32/build_ams_headless_gcc.sh
SOURCE_DATE_EPOCH=946684800 AMS_BUILD_TYPE=Release AMS_WARNINGS_AS_ERRORS=1 \
  bash ci/stm32/build_ams_headless_gcc.sh
```

The epoch above is an example (2000-01-01 UTC), not this release's build date.
For release evidence use an approved source-commit timestamp and record it.
Artifacts appear in the printed `AMS/build/Debug.*` or `Release.*` directory,
including ELF, MAP, HEX, BIN, listing and compiler `.su` files. This changes the
old fixed `AMS/build/DER26-AMS.*` output path; external consumers must use the
printed directory. Build optimization does not select VEHICLE or override any
validation gate. The script retains the existing default build profile. Use
the existing CubeIDE BENCH_VALIDATION configurations for the five-SMB bench.

Same epoch alone does not prove reproducibility: compiler, libraries, flags,
source, paths, and configuration must also match. ARM target binaries were not
built here because `arm-none-eabi-gcc` is unavailable. Do not claim target or
byte-for-byte release-build validation from the host timestamp test.

## Review dispositions still open

| Finding | Disposition |
|---|---|
| F-03 CS_B PE4/PF4 | Keep validated firmware PE4. CLI already identifies the conflicting schematic note. No physical continuity evidence was produced here and uploaded schematics were not altered. |
| F-05 watchdog policy | Unchanged. Feed currently stops on specified external faults as well as internal failures. Changing reset-loop semantics requires an explicit approved policy. |
| F-06 discharge bus-off | Unchanged. Charge/HIL immediate fail-low and discharge soft-fault behavior require a system/ECU stale-data decision, not an inferred local rewrite. |
| F-07 WCET/IRQ-off/stack | Target measurements remain necessary; `.su` generation is tooling support, not measured runtime evidence. |
| F-08 current H path | Physical fault from the supplied ADC/TP4 measurements; no software calibration or pin swap applied. |
| F-09 validation/hardware | Keep reported five-SMB temperature validation accepted. Reconcile its evidence with source defaults; do not repeat it because of old comments. AIR auxiliary absence is a capability limitation. |
| F-10 vehicle CLI | Read-only CLI/task retained. Service mutation remains gated. Removing diagnostics is a product choice, not a proven correctness fix. |
| F-11 heap configuration | Retained: CMSIS/newlib/kernel dependencies need allocation audit before disabling heap/timers. Static app task/mutex creation is unchanged. |
| F-12 kernel provenance | Vendor `task.h` V10.2.0 versus V10.2.1 banner retained and documented; renaming vendor metadata would not establish actual provenance. Target release build remains unverified. |
| F-14 memory banks | Linker layout unchanged. No DMA/DTCM redesign without target map/cache/driver evidence. |

The broad shared application object, serialized driver buffers, critical-section
design and generated default-task scaffold are not rewritten in this patch.
They are architectural debt or measurement questions, not reproduced defects
with an established behavior-preserving fix. The alleged duplicate CAN
`tx_hal_load_error_count` initializer was rechecked: only one assignment exists
in this baseline, so there is nothing to remove.

Battery algorithms, MiL scenarios, current-sensor conversion, CAN scheduling,
watchdog feeding, BMS_OK ownership and all hardware validation gates are
unchanged. Rebuild the STM32 target to deploy the identity change; this package
does not flash hardware or establish vehicle readiness.
