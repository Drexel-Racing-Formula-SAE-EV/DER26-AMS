# ESP32 plant cleanup notes

Source input contained an ESP-IDF project plus Eclipse workspace metadata and a full local build tree. This cleanup keeps only the source/configuration files needed for a repo branch.

## Removed

- Eclipse workspace `.metadata/`
- Eclipse project metadata `.project`, `.cproject`, `.settings/`
- ESP-IDF `build/` directory and generated binaries/maps
- `sdkconfig.old`
- old ESP-IDF hello-world README
- unused DFN aged replay headers not referenced by the plant runtime

## Kept

- root `CMakeLists.txt`
- `sdkconfig` and `sdkconfig.ci`
- `main/main.c`
- plant shared interface and drive profiles
- MCP2515 CAN component
- generated Simulink plant model source and lookup utilities
- build/integration documentation

## Code fixes

- fixed duplicate `uint32_t step24` declaration in the CAN TX task
- added finite-value guards before integer CAN scaling
- added null guards in CAN frame packing helpers
- fixed MCP2515 TX SPI transaction length
- fixed MCP2515 RX SPI buffer sizing
- added argument/state checks in the MCP2515 driver
- made MCP2515 init return errors instead of using `ESP_ERROR_CHECK()` abort paths
- aligned the MCP2515 timing with AMS CAN1 at 250 kbit/s for an 8 MHz crystal
- documented that 75s pack voltage uses 10 mV/count on CAN ID `0x200`
- added CAN `0x210`/`0x211` ADBMS replacement image frames for bench HIL firmware

## Validation performed in sandbox

ESP-IDF was not installed in the sandbox, so this was not a real `idf.py build`.

Performed checks:

- generated Simulink plant C compiled with host GCC
- `main/main.c` syntax checked with host stubs for ESP-IDF/FreeRTOS
- `components/CAN/mcp2515_driver.c` syntax checked with host stubs for ESP-IDF/FreeRTOS

Next validation:

```bash
cd hil/esp32_plant
idf.py set-target esp32
idf.py build
idf.py flash monitor
```
