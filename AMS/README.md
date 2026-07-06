# DER 2026 AMS Firmware v0.1.0

Major firmware/test updates by Mahad Faisal, 2026.
Designed and written by Cole Bardin (cab572) and Brendan Hoag (beh73)
Updated: 6/23/2025

## Pre-Hardware Emulation

An experimental Renode boot/CLI smoke harness lives in `renode/`. It is meant
to catch firmware boot, FreeRTOS startup, UART CLI, and safe bring-up command
issues before AMS hardware is available.

See `renode/README.md` for setup and run commands.
