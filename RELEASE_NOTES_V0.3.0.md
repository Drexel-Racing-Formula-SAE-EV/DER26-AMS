# DER26 AMS v0.3.0 release notes

## Scope

v0.3.0 extends the v0.2.0 production-intent SoP/SoH system with a conservative
strategy layer. The underlying 75-cell/five-segment predictive solver remains
the hard authority and still solves 0.1, 1, 10, and 30-second horizons every
measurement cycle.

## Added

- Three mission profiles that can only derate the hard envelope:
  - Qualify uses the 1-second capability and requires stationary-confirmed
    entry.
  - Endurance suppresses transient peaks to the 30-second capability and is
    the boot/stale-request default.
  - Limp Home uses `min(30-second capability, 35 A)` and automatically latches
    at a 30% weakest-segment 3-sigma SoC lower bound.
- CRC/counter/version/freshness-protected ECU mission request on `0x688`, with
  two consecutive requests required.
- Advisory strategy status on `0x689` for mission, horizon, fuse utilization,
  thermal readiness, energy-to-25-degC, and R0 bootstrap progress.
- Subtractive EAC14-80 fuse I2t observer using the published 80 A component and
  8020 A2s typical value with a default 25% usable budget.
- Fuse temperature derating, hottest-surface-plus-15-degC proxy, cooldown
  memory, budget exhaustion, and calibration/initialization authority gates.
- Binding-specific recovery scheduling:
  - fast cell-voltage recovery;
  - slow thermal recovery;
  - fuse-headroom-scaled current-path recovery;
  - SoC recovery held until measured SoC and coulomb throughput restore it.
- Thermal energy/readiness diagnostics and natural-excitation R0 qualification
  progress.
- Dashboard, CLI, host tests, CAN reference consumer, build gates, and
  commissioning documentation for the new strategy layer.
- Power protocol version 2, including new binding values 13 (fuse thermal) and
  14 (mission profile).

## Deliberately not implemented as automatic vehicle controls

- Stationary inverter/motor AC-injection heating: no verified CM200 actuator
  contract, no thermal coupling from motor to pack, and nonzero movement/rule
  risk. Only diagnostics and a future restrained service procedure are
  supported.
- IMU/GPS/ESP32 removal of long safety horizons: all four horizons remain
  continuously active.
- Forced first-drive sinusoidal torque for R0 identification: DADEKF natural
  excitation and conservative priors remain authoritative.
- Segment-level current shedding or 10 ms torque micro-pulsing: all segments
  share one series current, so the method cannot redistribute energy and would
  increase RMS heating for a fixed average current.

## Compatibility

- `0x684`-`0x687` use power protocol version 2. The paired ECU consumer and
  dashboard decoder in this release must be updated together.
- `0x688` uses mission-request protocol version 1.
- `0x689` is advisory and is not a required fifth frame in the fail-zero power
  bundle.
- Existing protocol-version-1 ECU consumers fail safely to zero but will not
  accept v0.3.0 limits.

## New vehicle build gates

```text
AMS_MISSION_CAN_CONTRACT_VALIDATED=1
AMS_FUSE_MODEL_VALIDATED=1
```

These macros are evidence acknowledgements, not validation by themselves.
They must remain zero until the installed-vehicle procedures in
`Docs/SOP_SOH_COMMISSIONING.md` pass.

## Remaining release gates

- Clean STM32F767 ARM build, link map, DWT WCET, and live stack high-water.
- Installed current calibration and polarity evidence.
- Five-segment/75-cell/120-thermistor HIL.
- Electrical and two-node thermal characterization.
- Installed EAC14-80/holder/busbar temperature and repeated-pulse validation.
- ECU mission request, protocol-v2 bundle, torque conversion, and final
  transmit-path HIL.
- SoH NVM power-cut validation and staged dyno/vehicle tests.
