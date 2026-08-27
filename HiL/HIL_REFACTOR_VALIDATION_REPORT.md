# HIL refactor validation report

Date: 2026-07-29

## Readiness statement

The reusable refactor and the new atomic CAN-image software are suitable for
merge. The HIL is not hardware-qualified until the MATLAB/Simulink Coder,
ESP-IDF, and bench gates below are executed.

| Qualification gate | Status in this package |
|---|---:|
| Static configuration and model-invariant source checks | PASS |
| Frozen 306-row generated-C host regression | PASS |
| P42A independent electrical holdout | NOT RUN — external dataset absent |
| P42A measured surface-temperature validation | NOT RUN — no measured channel |
| MATLAB/Simulink execution | NOT RUN — toolchain unavailable |
| Fresh generated-C/configured-Simulink parity | NOT RUN — toolchain unavailable |
| Staged ESP-IDF target build | NOT RUN — toolchain unavailable |
| ESP32/MCP2515 ↔ physical AMS bench | NOT RUN — hardware unavailable |

## Executed in this workspace

| Check | Result |
|---|---:|
| All ten repository static/host groups | PASS |
| Exact 306-row frozen generated-C baseline | PASS |
| Stable adapter, ESP32 application, and actual MCP2515 driver strict host compilation | PASS |
| MCP2515 success/retry/error/bus-off/timeout state-machine host test | PASS |
| Shared address/endian/CRC protocol vectors | PASS |
| Comprehensive AMS host injection suite | PASS |
| AMS ASan/UBSan suites (`detect_leaks=0`) | PASS |
| Focused atomic ADBMS replacement suite | PASS |
| Production safety and build-profile gates | PASS |
| Numerical acceptance configuration/self-test source checks | PASS |
| Staged generated-C qualification-tool smoke test | PASS |
| Qualified generated model/constants remained hash-frozen | PASS |

Repository check:

```bash
PYTHONDONTWRITEBYTECODE=1 \
python3 HiL/simulink/tests/run_static_checks.py
```

Result:

```text
PASS all static/host checks (10 groups)
```

AMS comprehensive check:

```bash
make -C AMS/host_tests clean test
```

Result:

```text
ALL COMPREHENSIVE HOST INJECTION TESTS PASSED
```

Focused atomic-image check:

```bash
make -C AMS/host_tests clean hil-adbms-test
```

Result:

```text
PASS HIL ADBMS image replacement
PASS HIL atomic image fault injection
PASS accumulator tick-wrap freshness
ALL HIL ADBMS REPLACEMENT TESTS PASSED
```

Production/profile gates:

```bash
make -C AMS/host_tests production-gates profile-gates
```

Result:

```text
ALL PRODUCTION SAFETY GATE TESTS PASSED
PASS explicit profiles and fail-closed vehicle evidence/manifest gates
```

The AMS source tree used by these commands is based on Git commit
`82c8607798b7d17726b52dcd96e6bdacd402559a` (`mahad-main`), plus the HIL
changes contained in this deliverable. `SOURCE_MANIFEST.json` records that
commit, the reviewed completion-package hash, important patched-file hashes,
and explicit tool statuses.

## Blocker closure

### Model artifact contract

`hil.configure_model` now returns both `parameter_hash` and
`simulation_configuration`. `hil.write_esp32_interface_files` can package a
fresh generated model without dereferencing a missing field.

### Numerical holdout acceptance

Candidate/P42A holdout PASS now requires frozen, temperature-aware limits for:

- comparable samples;
- voltage RMS, maximum, p95, mean bias, and endpoint error;
- endpoint SoC error when SoC evidence is present/required;
- surface-temperature RMS and maximum error;
- HPPC pulse count, loaded RMS/maximum error, and relaxation RMS error.

The P42A HCGT adapter now reserves A2/A4/A6/A8/A9/A11/A12/A13 for fitting
and A15/A19/A21/A37 for independent electrical holdout. It extracts
individual pulse windows, retains their published `ocvsoc`/OCV provenance,
and starts each normalized record at `t=0`.

At 25 °C, the P42A open-loop HIL screening gates include 12 mV HPPC RMS,
40 mV HPPC maximum, 25 mV HPPC p95, 15 mV loaded RMS, 12 mV relaxation RMS,
60 mV dynamic RMS, 150 mV dynamic maximum, 100 mV dynamic p95, 55 mV dynamic
absolute bias, 3% endpoint SoC error, and 4 °C surface-temperature RMS.

These are versioned engineering screening gates, not cell safety limits or
hardware-correlation evidence. Published HCGT chamber temperature is ambient,
not measured cell-surface temperature, so P42A thermal validation is reported
as `NOT_RUN`. Candidate limits must be justified and frozen before examining
the independent holdout. The candidate template likewise cannot report a
thermal `PASS` without an explicitly measured cell-surface-temperature trace.

### Fresh code generation and promotion

Direct generated-component installation is disabled. Code generation writes a
staged component and configured-Simulink parity case. The qualifier verifies
the source, model, parameter, configuration, pre-validation, component-file,
input, and reference hashes before compiling and comparing every output.
The pre-codegen evidence must contain a nonempty independent electrical
holdout `PASS`; missing external P42A data blocks fresh release generation.
The automation builds ESP-IDF against the staged component and only then may
atomically promote it. `--promote --skip-idf` is forbidden.

`parameter_distributed` generation is blocked because the embedded plant still
evolves one representative state.

### Atomic cell/temperature image

The current 75s6p publication is 67 atomic-image frames plus three
measurement/truth/summary frames per 100 ms:

- START with topology and frame counts;
- 25 cell and 40 temperature frames, each tagged with generation;
- per-channel receiver bitmaps;
- 250 ms assembly timeout;
- identical-duplicate acceptance and conflicting-duplicate rejection;
- canonical CRC32 COMMIT;
- fresh-image replay rejection and stale generation resynchronization;
- one-lock copy into the live ADBMS replacement image.

Dropped, partial, conflicting, corrupt, timed-out, or replayed generations do
not change the published image. The AMS CAN RX ring is 160 entries so two
complete 70-frame bursts fit with margin, subject to hardware high-water
measurement.

The ESP32 reports microsecond plant-step, image, and total-burst durations,
maximums, transmit failures, and deadline misses. A commanded model reset
performs an immediate zero-current step before publication, retains the
transport epoch, and cannot emit a valid all-zero accumulator image. The
MCP2515 path checks TXREQ, ABTF, MLOA, TXERR, bus-off, warning/passive, and
overflow state, clears only intended interrupt bits, and maintains separate
success/error/retry counters. It still uses TX buffer 0 and blocking polling;
hardware timing evidence remains mandatory.

## Frozen identities

Original frozen-oracle source archive SHA-256:
`8b610bf1c4f5ab6b556e59a1cb56682c83b853712d930ae30ad6796af0854d99`.

Reviewed completion-package input SHA-256:
`670167b4963d845f4dd812008fa9d0074a4d2d1943e45fa6c55e54fc54be883a`.

Generated model source SHA-256:
`48b3aed9305a19c3495cd09fbbff9089dcc0407dc4b395f49a3019276c4696c3`.

Constant source SHA-256:
`9801ebe2b699a173960888b97e585d91e8343518bf1711b950abe98ea39eb5d8`.

Parameter configuration hash:
`77b74591f934f0f3b5f895beff102845bb517ef31d78d137ad5b61cb713b4f75`.

Frozen plant/configuration identity:
`01fadc41270e1b1a7a3a9fe705c4161c9347d40edbf5f773b722cbd7657afefa`.

## Tool-dependent checks not executed here

MATLAB, Simulink, Simulink Coder, and ESP-IDF are not installed in this
workspace. They were not reported as executed. On a host with those tools:

```bash
python3 HiL/tools/run_toolchain_qualification.py --promote
```

That ordered command runs `run_all_tests`, staged code generation, fresh
generated-C/configured-Simulink parity, and an ESP-IDF build against the staged
component:

```bash
idf.py set-target esp32
idf.py build
```

Only after all those steps pass does it atomically promote the component.
Do not flash or replace the current frozen snapshot from an unqualified
staging directory.

## Hardware gates still open

Follow `HARDWARE_QUALIFICATION_PLAN.md`. Required evidence includes physical
scaling/endianness, complete-image atomicity, stale handling, deliberate
drop/duplicate/change/delay/reorder/replay/CRC faults, independent node reset
and power-cycle, bus-off recovery, long US06/endurance runs, task execution
time, 67-frame image time, 70-frame burst time, deadlines, stack high-water,
frame failures, queue high-water/drops, and AMS image age.

## Model-evidence boundary

- The generated plant evolves one representative 2RC electrical state and one
  core/surface thermal state, then expands sensor outputs.
- The MATLAB reference can independently evolve group states; the embedded
  generated plant cannot yet model real weak-group divergence or uneven
  cooling.
- The 12s2p case proves structural reuse, not physical accuracy.
- Thermal results remain comparative screening until pack installation and
  cooling parameters are calibrated.
- Candidate-cell retuning remains blocked on exact cell identity and
  independent electrical/thermal evidence.
- Closed-loop vehicle-control HIL remains a separate Phase 2.
