# DER26 AMS v0.5.55 — physical 74s charging source

This is a dedicated 74-series accumulator configuration, NOT a 75-cell pack with one unmeasured cell. It assumes the first SMB C1 physical series position has been removed. The input wiring and unpopulated ADBMS channel treatment must match the actual 74s pack; firmware does not establish correct rewiring.

## Build
Import `AMS` as CubeIDE project `DER26-AMS-74S-CHARGER-250`.
Select **74S CHARGING 250k Debug** or **74S CHARGING 250k Release** (profile 6). These permit charging through normal safety supervision. The separate **CHARGER LV 250k Debug/Release** configurations (profile 4) remain inhibited for communications testing. Source default is inhibited profile 4.

No compiled ELF is included. An ARM target build/link and hardware validation remain outstanding.

## Configuration
- Five SMBs, ADBMS6822 String B / PE4; ADBMS2950 and fan disabled.
- Physical series count 74: first SMB C2-C15 (14 cells); SMBs 2-5 C1-C15 (60 cells). Every one of these 74 channels is required. Exactly SMB1 C1 is unpopulated.
- Complete measured pack sum is available only while all 74 readings are usable.
- Charger target **307.8 V / 10 A**: inherited 312/75 V per-cell target times 74, rounded down to 0.1 V. This changes the pack setting only; inherited per-cell, temperature and current protection thresholds are retained.
- CAN **250 kbit/s**; TX extended 0x1806E5F4; RX extended 0x18FF50E5.
- Passive balancing inhibited. Five segment estimators retain 14/15/15/15/15 observations. The inherited SoP engine remains advisory and rejects partial 15-channel segment input; it is not charge permission.
- No ECU mission dependency or propulsion mode.

## CLI and power requirement
USART3 PD8 TX / PD9 RX, 115200 baud, 8N1, no flow control, CR termination, 3.3 V UART.

`chargerpower on` is unavailable. BMS_OK is never forced to power the cart without normal measurement/IMD/fault supervision. A charger independently powered through the verified cart arrangement is required to obtain status before normal charge permission. If charger AC power depends exclusively on BMS_OK, bootstrap sequencing remains a hardware/integration blocker; this package does not bypass that interlock. The installed cart coil-voltage/interface discrepancy identified in the earlier review is unresolved.

Diagnostics: `ver`, `can`, `volt`, `temp`, `current`, `bringup ready`, `bmsok status`, `charger`.
`state charge` selects the charge policy; it does not force permission. `state stop` exits charging, lowers BMS_OK and requests three zero-demand disable frames. `bmsok release` clears only the mutable inhibit and cannot override faults.

## Fixes since v0.5.53
- Removed service-power supervisor bypass and its CAN producer.
- Any caller can immediately lower BMS_OK; low clears residual service state.
- Correct physical 74s count, required channel count, measured sum and pack voltage target.
- Charger-only recovery clears the inapplicable ECU refresh gate after transport settlement without claiming ECU authority. Charge bus-off hard-fault behavior remains in place.
- Final CAN load checks additionally reject stale/missing charger feedback, IMD failure, transition/inhibit, heartbeat, fuse and diagnostic faults.
- Charge-stop voltage inhibits normal BMS supervisor permission.

## Validation
Targeted host tests and native production-source syntax checks are recorded under `validation`. They do not establish target timing or hardware readiness. No ARM build/link, flashing, charging or electrical test was performed. The legacy full CI link issue was not resolved or qualified in this release.
