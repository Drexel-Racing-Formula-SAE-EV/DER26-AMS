# DER26 AMS v0.3.4 — EAC14-80 curve-based fuse model

**Date:** 2026-07-22

## Added

- EAC14-80 current-versus-time curve model using rounded page-4 centerline
  points and the exact 800 A / 8020 A²s table anchor.
- Published temperature-derating curve support with no credited cold uplift.
- Explicit curve-extrapolation diagnostics.
- Dimensionless thermal/resource utilization state.
- Independent `long double` exact-state oracle.
- High-resolution numerical integration cross-check.
- 50,000-state randomized production/reference comparison.
- Expanded replay sweep and high-current curve-stress traces.
- Digitized source-data CSVs and manufacturer data request.

## Changed

- Removed the fixed `8020 A²s × usable_fraction` observer formulation.
- Replaced I²t-budget fields with normalized utilization and curve-time fields.
- Locked the model to the installed EAC14-80 rather than allowing unsafe
  rescaling to another fuse rating.
- Updated SoP tests, Python differential tests, replay output, and documentation.

## Validation

- Fuse curve-oracle tests: PASS.
- 50,000 randomized comparison: PASS.
- Production SoP/SoH core: PASS.
- Python/C fuse differential tests: PASS.
- 144-case strict replay sweep: PASS.
- No nonconservative state, cap, or latch result observed.

## Safety status

`AMS_FUSE_MODEL_VALIDATED` remains disabled by default. The data sheet curves
are typical centerlines, and low-current operation below the chart boundary is
extrapolated. This release is suitable for host characterization and future
calibration work, not vehicle fuse authority.
