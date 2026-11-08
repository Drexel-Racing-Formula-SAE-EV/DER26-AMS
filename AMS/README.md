# DER26 AMS v0.5.57 — Consolidated charger-only source candidate

This is the 250-kbit/s charge-only STM32CubeIDE project, incorporating selected
common firmware improvements from the 1-Mbit/s vehicle CLI4 branch and the
ADBMS2950B / EVAL6822 DIAG5R3 branch.

## Build selections

- **Charger LV 250k Debug/Release**: `AMS_BUILD_PROFILE=4` (CAN diagnostic, BMS inhibited).
- **75S 74OBS CHARGE-REVIEW 250k Debug/Release**: `AMS_BUILD_PROFILE=6` (charge-review configuration).

The charge-only branch settings are preserved: 75 physical series cell groups,
74 observed inputs, 250 kbit/s CAN, no ECU mission CAN, 312.0 V / 10.0 A
charger request settings, and 27% initial SoC in profile 6. The existing
S1C1-unobserved source behavior is carried forward without relaxing an
additional protection gate. The missing C1 voltage still cannot be measured.

## Integration

- Temperature authority timestamps/freshness and scan cadence alignment.
- Asynchronous CLI TX/priority improvements and acquisition diagnostics.
- SPI / ADBMS improvements shared with vehicle and APM diagnostic branches.
- ADBMS2950B driver diagnostic and FLAG/current-sense changes included as source
  but NOT enabled in the charger-only profile or treated as validated on its hardware.
- Charger-only build configuration, state policy, and 250-kbit/s transport retained.

See `docs/CHARGE_ONLY_CONSOLIDATION_2026-11-08.md` for source provenance,
validation results and unresolved limitations. Source package only; no MCU ELF.
