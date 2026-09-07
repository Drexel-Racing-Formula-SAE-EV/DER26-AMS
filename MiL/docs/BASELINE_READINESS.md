# DER26 AMS MiL Baseline Readiness

This audit separates implemented MiL infrastructure from external/frozen-input blockers.
A `BLOCKED` item is not a failed host regression; it prevents release-grade qualification evidence.

| Item | Status | Evidence |
|---|---|---|
| Invalid current-zero rejection | **PASS** | `AMS/Core/Src/ext_drivers/current_sensor.c` |
| AMS source/CubeMX nominal CAN bitrate is 1 Mbit/s | **PASS** | `AMS/DER26-AMS.ioc` |
| CAN manifest declares 1M | **PASS** | `AMS/Core/Inc/ams_build_profile.h` |
| Tuning contract declares 1 Mbit/s image | **PASS** | `AMS/docs/AMS_TUNING_CAN_SD_CONTRACT.md` |
| Exact ECU CAN binary-record source imported | **BLOCKED** | `Required for byte-exact CAN###.BIN generation` |
| Qualification acceptance baseline frozen | **BLOCKED** | `MiL/baselines/acceptance/qualification_template.json` |

Implemented: 4/6 frozen-input gates.
