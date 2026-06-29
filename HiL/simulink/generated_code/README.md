# Generated Simulink C Code

This folder contains the minimal generated C source snapshot required for integration work.

## Included

```text
model/
  drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.c
  drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.h
  drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_private.h
  drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_types.h
  ert_main.c

sharedutils/
  look2_iflf_binlc.c
  look2_iflf_binlc.h
  look2_iflf_pbinlc.c
  look2_iflf_pbinlc.h
  rt_defines.h
  rtwtypes.h
```

## Not included

The following were removed:

- generated HTML code reports
- `codedescriptor.dmr`
- buildInfo/codeInfo/compileInfo `.mat` metadata
- generated `.mk`, `.rsp`, `.bat` files
- Simulink cache internals

## Integration note

`ert_main.c` is retained only as a generated reference entry point. For ESP32 HIL integration, write a normal ESP-IDF `app_main()` wrapper and call the generated model initialize/step functions directly.

Expected call pattern:

```c
RT_MODEL_drev_75s6p_p42a_accu_T model;
DW_drev_75s6p_p42a_accumulato_T dwork;

model.dwork = &dwork;
drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_initialize(&model);

// every 100 ms:
drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_step(
    &model,
    I_pack_A,
    T_amb_C,
    &V_pack,
    &T_core,
    &T_surf,
    &SoC_true,
    V_group,
    V_segment,
    T_sensor,
    SoC_group,
    &V_min,
    &V_max,
    &T_max,
    &T_avg);
```

Do not make this generated plant safety-authoritative. It is a HIL truth/simulation source for estimator and AMS development.
