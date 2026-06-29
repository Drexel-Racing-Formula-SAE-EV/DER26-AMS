# DER26 ESP32 P42A 75s6p HIL Plant

This folder contains the ESP32 plant node for the DER26 AMS HIL estimator branch.

The plant runs a generated Simulink electrothermal accumulator model for a 75s6p Molicel P42A pack and publishes CAN telemetry to the AMS estimator node through an MCP2515 Classic CAN controller.

## Layout

```text
hil/esp32_plant/
  CMakeLists.txt
  sdkconfig
  sdkconfig.ci
  main/
    main.c
    plant_shared.h
    drive_profiles.h
  components/
    CAN/
      mcp2515_driver.c
      mcp2515_driver.h
    plant_model/
      generated Simulink C model and lookup helpers
```

## CAN frames

Classic CAN 2.0, standard identifiers:

| ID | Frame | Layout |
|---:|---|---|
| `0x200` | Measurement | V_pack, I_pack, T_surf, counter |
| `0x201` | Truth/debug | SoC_true, T_core, counter, plant step |
| `0x202` | AMS summary | V_min, V_max, T_max, T_avg |
| `0x300` | Plant control RX | reset command |

Important scaling note: `0x200` pack voltage uses **10 mV/bit**, not 1 mV/bit. A 75s accumulator cannot fit in `uint16_t` at 1 mV/bit.

## Build

From this folder with ESP-IDF activated:

```bash
idf.py set-target esp32
idf.py build
```

Flash and monitor:

```bash
idf.py -p <PORT> flash monitor
```

## Reset command

Send standard CAN ID `0x300` with first three bytes:

```text
A5 5A 52
```

The plant task resets the generated model state, shared data, and plant step counter.

## Current profile selection

`main/main.c` selects a profile with `PLANT_PROFILE_MODE`:

```c
#define PROFILE_SYNTH_HPPC  0
#define PROFILE_UDDS_25C    1
#define PROFILE_US06_25C    2
#define PROFILE_LA92_25C    3
```

Default is US06 25C.

## Cleanup status

This version removes Eclipse workspace metadata, ESP-IDF build outputs, stale DFN replay files, and local generated binaries. It keeps only repo-relevant project source, generated plant model source, configuration, and documentation.

Code cleanup performed:

- fixed a duplicate `step24` declaration in `main.c`
- hardened MCP2515 SPI helpers with argument and state checks
- fixed MCP2515 TX SPI transaction length
- fixed MCP2515 RX SPI buffer sizing
- added finite-value guards before CAN integer scaling
- added null guards in CAN frame packers
- removed `ESP_ERROR_CHECK()` use from MCP2515 init so failures return errors instead of aborting

## Limitations

This is still not a hardware validation. Before relying on it for estimator results, verify:

- MCP2515 wiring and 8 MHz crystal
- actual CAN bus bitrate and termination
- pack-voltage scaling decode on the AMS side
- current polarity convention: positive current means discharge
- profile choice and ambient temperature configuration
- generated model consistency against Simulink outputs
