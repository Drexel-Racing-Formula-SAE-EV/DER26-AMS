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

## Not fixed yet

The MATLAB scripts still need a path cleanup pass. Several scripts reference local absolute paths such as `Documents/BMS_P42A_Data/...`.

Recommended next cleanup PR after adding this folder:

- introduce `hil/simulink/scripts/hil_paths.m`
- replace hardcoded absolute paths with repo-relative paths or environment variables
- add a short regeneration guide once the raw P42A dataset location is finalized
