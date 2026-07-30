# Cleanup Notes

Original upload: `migrateBMS (2).zip`

Original content had hundreds of generated/cache/report files, mostly from Simulink Coder HTML reports and `slprj/` output. This clean version keeps only source-level assets and minimal generated C needed for integration.

## Kept

- final validated Simulink model
- current v3 AMS-output Simulink model
- DER/P42A-related MATLAB scripts
- minimal generated C and shared utility source files
- small validation table and validation figure
- documentation and HIL-specific `.gitignore`

## Removed

- `.slxc` Simulink cache files
- `slprj/`
- generated HTML reports
- generated web assets, fonts, images, JavaScript, CSS
- old v0/v1/v2 model revisions and backup model
- duplicate generated-code zip
- generated `.mk`, `.rsp`, `.bat`, `.dmr`, and build metadata files
- `empty.csv`, which appears to be a large PuTTY UART log, not a source CSV
- old NN/Monte Carlo scripts that are not part of the DER non-NN HIL estimator path

## Reusable-framework follow-up completed

- `+hil/project_paths.m` resolves the repository and supports
  `HIL_DATA_ROOT`/`HIL_OUTPUT_ROOT`.
- The legacy scripts are thin wrappers around reusable functions.
- Cell, pack, simulation, and dataset assumptions are separate configurations.
- The exact generated constants and behavior have hashed source artifacts.
- Generic topology, validation/reporting, and a stable ESP32 adapter are in
  place.

See `HIL_REFACTOR_WORK_PLAN.md` and `HIL_REFACTOR_VALIDATION_REPORT.md` for the
current status and remaining external tool/hardware checks.
