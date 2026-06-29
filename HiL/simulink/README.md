# Simulink Model Assets

## Models

Primary model:

```text
models/drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.slx
```

Also retained:

```text
models/drev_75s6p_p42a_accumulator_plant_v3_ams_outputs.slx
```

Older v0/v1/v2 models and backup models were removed from this clean folder to avoid stale model confusion. Recover them from the original `migrateBMS` archive if needed.

## Scripts

The scripts folder contains the MATLAB scripts used for:

- P42A parameter initialization
- R1/C1 fitting
- R2/C2 fitting
- batch HCGT validation
- AMS-output expansion
- Simulink Coder configuration
- sanity testing model outputs

Several scripts still contain local absolute paths. Before another developer regenerates the model, convert those paths into repo-relative or environment-variable based paths.

## Validation

The validation folder contains the small ECM parameter table and one validation figure. Large raw datasets and logs were intentionally not included.

## Removed from this clean version

Removed intentionally:

- `.slxc` cache files
- `slprj/`
- generated HTML reports
- generated web assets, fonts, images, and JavaScript bundles
- duplicate old Simulink model revisions
- PuTTY UART log named `empty.csv`
- zipped duplicate of generated code
- model advisor/cache metadata

Reason: these files create noisy PRs and are not needed for source review.
