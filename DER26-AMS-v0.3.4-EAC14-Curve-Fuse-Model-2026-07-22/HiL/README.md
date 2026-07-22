# DER AMS HIL Plant

This folder contains the cleaned Simulink plant assets for the DER 75s6p P42A accumulator HIL work.

Scope of this folder:

- Simulink source models for the 75s6p P42A accumulator plant.
- MATLAB scripts used to build, validate, and configure the plant model.
- Minimal generated C code from Simulink Coder for firmware/ESP32 integration.
- Small validation artifacts.

This folder intentionally does **not** include Simulink cache output, generated HTML reports, `slprj/`, `.slxc` files, duplicate backup models, or analyzer/build artifacts.

## Intended repo location

Place this at the AMS repo root as:

```text
hil/
```

Recommended branch:

```bash
git checkout -b feature/ams-hil-ekf-estimator
```

Do not mix this with the AMS cleanup/test-harness PR. Keep this as a separate feature branch.

## Directory layout

```text
hil/
  README.md
  .gitignore
  esp32_plant/
    README.md
  simulink/
    README.md
    models/
    scripts/
    generated_code/
      README.md
      model/
      sharedutils/
    validation/
```

## Current model target

Primary model:

```text
simulink/models/drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.slx
```

This model represents a 75s6p Molicel P42A accumulator plant with AMS-style outputs:

- `V_pack`
- `T_core`
- `T_surf`
- `SoC_true`
- `V_group[75]`
- `V_segment[5]`
- `T_sensor[120]`
- `SoC_group[75]`
- `V_min`
- `V_max`
- `T_max`
- `T_avg`

## Generated code status

The generated C code is included as a source snapshot under:

```text
simulink/generated_code/
```

This is meant to be integrated into the future ESP32 plant firmware. It is not yet wrapped as a complete ESP-IDF application in this cleanup package.

## Important limitation

Some MATLAB scripts reference local paths under the original developer machine, for example `Documents/BMS_P42A_Data/...`. Those scripts need path cleanup before another machine can regenerate the model from raw datasets.

The generated C snapshot embeds the LUT constants and is more portable than the MATLAB regeneration path.
