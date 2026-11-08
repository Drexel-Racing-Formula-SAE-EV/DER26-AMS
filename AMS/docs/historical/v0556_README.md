# DER26 AMS v0.5.56 — 75s / 74 observed, diagnostics only

**NOT A CHARGING-ENABLED RELEASE.** This firmware intentionally cannot permit charging.

Hardware described: five 15s segments, 75 physical series positions, SMB1 C1 still electrically connected but its voltage cannot be read. The other 74 cell inputs are required/monitored. This does not establish a complete cell-overvoltage protection system.

## Build configurations
- `75S 74OBS DIAGNOSTIC 250k Debug/Release` (profile 6): diagnostic CAN/CLI and 75-series constants; BMS_OK permanently compile-inhibited, charge enable forced to disable at CAN load.
- `CHARGER LV 250k Debug/Release` (profile 4): inhibited CAN diagnostics.
- Default build is profile 4.

## Changed since v0.5.55
- Physical series count corrected to 75 while the measured channel requirement stays 74 (first SMB C1 skipped in input sampling only).
- Pack full target = 315 V at 4.20 V/cell; charger **configured** request target = 312.0 V (4.16 V/cell), current target = 10 A. **These are numeric settings only; the firmware sends DISABLE/zero demand**.
- Physical total is unavailable: reported pack voltage from 74 measured cells is 0, never mislabeled a 75-cell sum.
- Profile 6 has immutable `AMS_PROFILE_BMS_RUNTIME_AUTHORITY_ALLOWED=0` and default BMS inhibit=1, and charger disable reason includes voltage invalid regardless of 74/74 observed cell count.
- CLI bring-up shows 74 observed and 75 physical separately. Missing SMB1 C1 remains unprotected until external shutdown integration/validation or repair of voltage sense.

## Status
Host targeted tests only, where run. There is no ARM ELF, hardware validation, or proof of an independently acting overvoltage protector. Do not use this diagnostic project for physical charging. Charger and pack power are to remain physically disabled during diagnostics.
