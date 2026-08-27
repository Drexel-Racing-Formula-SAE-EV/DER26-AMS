# Frozen generated-C reference

This directory retains the minimal Simulink model 1.67 source snapshot used to
reconstruct and audit the current P42A parameters. It is not the application
API.

The ESP32 application calls `plant_init`, `plant_reset`, `plant_step`,
`plant_get_outputs`, `plant_get_state`, and `plant_terminate` from
`esp32_plant/components/plant_model/plant_model_adapter.h`. Only
`plant_model_binding.h` knows generated identifiers.

For new code:

```matlab
codegen_artifact = hil.generate_code( ...
    cell_cfg, pack_cfg, sim_cfg, params, ...
    'InstallIntoEsp32', true);
```

The function creates a short hashed model name, clean ERT settings, generated
binding and topology manifest, and a self-contained ESP-IDF component.
`ert_main.c` is historical reference only and is never part of the ESP-IDF
component.

Each build also exports one full-rate input/reference CSV pair. Run the
component with `esp32_plant/tests/parity_runner.c`, then pass the reference and
candidate CSV paths to `hil.validate_codegen_parity`.

Generated code is a HIL source, not safety-authoritative firmware.
