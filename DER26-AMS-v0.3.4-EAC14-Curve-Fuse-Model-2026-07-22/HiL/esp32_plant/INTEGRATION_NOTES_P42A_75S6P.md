# P42A 75s6p plant integration notes

This project keeps the existing ESP32 plant-node architecture and replaces only the plant source.

## Kept from the existing working system

- ESP-IDF project layout
- FreeRTOS task split:
  - `plant_task` at 100 ms
  - `can_tx_task` at 100 ms
- MCP2515 CAN driver component
- reset command on CAN ID `0x300` using bytes `A5 5A 52`
- shared plant state protected by `plant_mutex`
- current profile infrastructure from `drive_profiles.h`

The MCP2515 driver is configured for 250 kbit/s with an 8 MHz crystal, matching
the AMS STM32 CAN1 timing (`Prescaler=12`, `BS1=15TQ`, `BS2=2TQ`).

## Replaced/updated

- The old DFN replay `main.c` was replaced with a live Simulink plant-step loop.
- The old `single_cell_battery_phaseC_backup.*` generated model files were removed from the `plant_model` component.
- The new generated model is:
  - `drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.c/.h`
- `plant_shared.h` now includes AMS-style outputs:
  - `V_group[75]`
  - `V_segment[5]`
  - `T_sensor[120]`
  - `SoC_group[75]`
  - `V_min`, `V_max`, `T_max`, `T_avg`

## CAN scaling change

The old `0x200` pack-voltage field used `1 mV/bit`, which only supports 65.535 V and will saturate for a 75s pack.

For this 75s6p accumulator model, `0x200` now uses:

```text
V_pack raw uint16 = V_pack[V] * 100
V_pack[V] = raw * 0.01
```

So the voltage LSB is `10 mV/bit`, supporting up to 655.35 V.

Other base-frame fields remain:

```text
I_pack:   int16, 10 mA/bit, positive = discharge
T_surf:   int16, 0.01 C/bit
SoC_true: uint16, 0.01%/bit
T_core:   int16, 0.01 C/bit
```

## New CAN frame

A new optional AMS summary frame is sent at `0x202`:

```text
[0:1] V_min uint16, 1 mV/bit, min group voltage
[2:3] V_max uint16, 1 mV/bit, max group voltage
[4:5] T_max int16,  0.01 C/bit
[6:7] T_avg int16,  0.01 C/bit
```

## ADBMS replacement image frames

For bench firmware built with `AMS_HIL_REPLACE_ADBMS=1`, the plant also sends
a complete logical replacement for the ADBMS read layer:

```text
0x210 cell triplet:
[0]   segment index, 0..4
[1]   first cell index in this triplet
[2:3] cell[first + 0] uint16, 1 mV/bit
[4:5] cell[first + 1] uint16, 1 mV/bit
[6:7] cell[first + 2] uint16, 1 mV/bit

0x211 temperature triplet:
[0]   segment index, 0..4
[1]   first thermistor index in this triplet
[2:3] temp[first + 0] int16, 0.1 C/bit
[4:5] temp[first + 1] int16, 0.1 C/bit
[6:7] temp[first + 2] int16, 0.1 C/bit
```

The ESP32 sends 25 cell-image frames and 40 thermistor-image frames per plant
tick. The AMS parser converts these into the same accumulator raw structures
and update masks used by the ADBMS6830 path, then the existing voltage and
temperature fault logic runs unchanged. Freshness is still enforced; stale
image frames do not keep BMS_OK alive.

## Build note

Do not compile generated `ert_main.c`. This integration does not include it in the ESP-IDF component.

The plant model component compiles:

```text
drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.c
look2_iflf_binlc.c
look2_iflf_pbinlc.c
const_params.c
```

## Validation note

The generated plant was validated in Simulink before integration:

- AMS vector output sanity test passed.
- `sum(V_group)` and `sum(V_segment)` match `V_pack` to microvolt-level numerical error.
- Cell/group voltage spread is 8 mV.
- SoC group spread is 1%.
