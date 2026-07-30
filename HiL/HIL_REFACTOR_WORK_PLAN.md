# HIL refactor work plan and status

## 1. Freeze the existing behavior — complete

- Preserved the validated model 1.67 generated-C source as the oracle.
- Reconstructed its exact OCV/R0/R1/C1 LUTs and scalar R2/C2/thermal values.
- Froze zero-current, 100 A pulse, synthetic HPPC, US06, hot-start, and
  cold-start host outputs with file/source hashes.
- Added a tolerance-based oracle comparator.

## 2. Remove machine and product hardcoding — complete

- Replaced local absolute paths with repository-relative resolution,
  `HIL_DATA_ROOT`, and `HIL_OUTPUT_ROOT`.
- Added independent cell, pack, simulation, and dataset configurations.
- Converted build, fitting, profile, expansion, validation, and run logic into
  reusable `+hil` functions.
- Kept old script names as thin compatibility entry points.

## 3. Make topology and outputs configurable — complete

- Parameterized `Ns`, `Np`, segment counts, explicit group-to-segment mapping,
  sensor count, sensor-to-group mapping, and core/surface weighting.
- Added uniform, deterministic regression, and parameter-distributed modes.
- Added a deliberately different 12s2p/three-segment/18-sensor structural case.
- Added conservation, dimension, min/max, temperature-summary, coulomb-count,
  current-scaling, and sample-time guards.

## 4. Stabilize the generated-code boundary — complete

- Added `plant_model_adapter.h/.c`; application code no longer names generated
  model functions, types, or state fields.
- Added a generated-name binding header and topology manifest.
- Removed 15-groups/24-sensors-per-segment indexing from the CAN image logic.
- Added host-C initialization, reset, state, finite-value, and topology tests.
- Added a clean ERT build/package function for MATLAB/Simulink Coder.

## 5. Validation, provenance, and reporting — software complete

- Added normalized data contracts and CSV/MAT/P42A adapters.
- Added explicit fit/holdout partitions; candidate builds reject absent or
  overlapping holdout provenance.
- Added parameter source hashes, configuration hashes, validity domains, and
  reviewable table export.
- Added pulse/dynamic error metrics: RMS, bias, bias-corrected RMS, maximum,
  p95, p99, pulse onset, loaded, relaxation, and endpoint errors.
- Added frozen, temperature-aware HPPC/dynamic acceptance limits. PASS now
  requires numerical voltage, SoC, and temperature accuracy, not just samples.
- Added thermal boundary sensitivities with explicit screening-only language.
- Added one-command MATLAB validation and repository static/host-C checks.
- Exposed UDDS, US06, LA92, synthetic HPPC, CSV/MAT, constant-current, and
  constant-power profiles behind explicit repeat and scaling policies.

## 6. Candidate cell retune — ready, awaiting inputs

The framework and templates are complete. A defensible retune cannot be
fabricated without:

- a selected cell manufacturer/model and operating limits;
- raw OCV/capacity curves at the configured temperatures;
- pulse or HPPC data for R0/R1/C1/R2/C2;
- dynamic validation data;
- measured thermal data or a documented provisional thermal prior.

Copy `simulink/configs/cells/candidate_template.m`, add a dataset configuration,
then call `scripts/build_candidate_cell.m`. The template is intentionally
non-buildable until every placeholder is replaced.

## 7. Atomic CAN image — software complete; target timing pending

- Added START and COMMIT control frames and a generation tag in every cell and
  temperature data frame.
- Added per-channel staging bitmaps, 250 ms assembly timeout, canonical CRC32,
  duplicate/conflict handling, fresh-image replay rejection, stale resync, and
  one-lock commit into the live ADBMS replacement image.
- Increased the AMS RX queue to hold two complete 70-frame plant bursts.
- Added host fault injection for drop, duplicate, change, reorder, delay,
  replay, CRC corruption, timeout, reset, stale age, and tick wrap.
- Added ESP32 microsecond plant/image/burst timing and deadline counters.

## 8. Hardware closure — external execution required

- Run MATLAB/Simulink code generation; direct snapshot installation is blocked
  until the fresh generated C passes the configured-Simulink parity gate.
- Build and flash with the project’s ESP-IDF toolchain.
- Verify CAN timing, scaling, freshness, reset behavior, and fault response on
  the AMS bench.
- Calibrate installation thermal parameters before any fanless conclusion.

Use `tools/run_toolchain_qualification.py --promote` when MATLAB, Simulink
Coder, and ESP-IDF are installed. The AMS host harness passes the focused
atomic-image and comprehensive injection suites. That is software-boundary
evidence, not a substitute for the bench checks above.
