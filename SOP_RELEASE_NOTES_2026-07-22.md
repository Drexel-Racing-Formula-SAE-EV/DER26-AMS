# DER26 AMS v0.2.0 — robust SoP/SoH release notes

Date: 2026-07-22

## Outcome

This release replaces the isolated Dynamic SoP prototype with a production-
intent, fail-closed SoP/SoH subsystem integrated into the AMS estimator and CAN
paths. It is a source release for commissioning, not evidence that the installed
vehicle has passed its target, HIL, or hardware release gates.

## Major changes

- Four-horizon 0.1/1/10/30-second DCL and CCL for the real 75s6p P42A pack.
- Five-segment DADEKF integration using SOC, 2RC states, R0, temperatures,
  covariance, innovation, all 75 cell voltages, and all 120 thermistors.
- Measured-voltage anchoring and weak-cell offset preservation.
- Exact discrete RC transitions and stable two-node electrothermal prediction.
- Horizon-specific system current ceilings derived from the existing 80 A fuse,
  contactor, charger, and firmware overcurrent policy.
- Strict finite/range/age/calibration/polarity/topology/authorization checks.
- Conservative current, voltage, temperature, covariance, model, and SoH
  uncertainty bounds.
- Immediate reductions and invalidation; bounded DCL/CCL recovery.
- Observable rest-anchor capacity SoH and five-segment resistance SoH.
- Schema-3 CRC32 SoH persistence with per-segment resistance upper bounds and
  two-slot selection support.
- Versioned 0x684-0x687 CAN bundle with ID-bound CRC-8, common counter, age,
  direction, fallback, binding, confidence, and horizon data.
- Passive ESP32 dashboard/logger decoding and CSV/JSON exposure.
- Portable fail-zero ECU bundle consumer with two-good-bundle recovery.
- Corrected legacy temperature telemetry from 17/85 to 24/120 channels.
- Vehicle build interlocks for SoP model, calibration, CAN contract, and
  immutable model revisions.
- Estimator task stack increased to 1536 words after static-frame review.

## Verification added

- Target-C SoP/SoH, brute-force boundary, persistence, CRC, and integration
  tests.
- Independent Python oracle that parses separately generated HIL calibration
  tables and differentially tests C across deterministic and seeded cases.
- ESP32 dashboard decoder integrity/freshness tests.
- ECU transport consumer startup, CRC, counter, stale, malformed, direction,
  partial-bundle, and rollover tests.
- Host performance and GCC stack-frame reporting.
- Required CI integration for all of the above.

## Safety behavior

Any invalid, stale, nonfinite, uncalibrated, wrong-polarity, incomplete-
topology, incomplete-sensor, or model-domain input produces zero DCL and CCL.
Unknown SoH uses explicit conservative priors rather than blocking an otherwise
valid new pack. Drive, charger, and regen directions remain independently
authorized; regen is disabled by default pending vehicle validation.

The SoP subsystem does not directly control AIRs or BMS_OK. A vehicle build can
make estimator liveness mandatory, and the ECU consumer must independently
fail torque limits to zero.

## Remaining physical release gates

- Clean STM32F767 ARM build, map review, DWT WCET, and live stack high-water.
- Installed DHAB current polarity/calibration/uncertainty validation.
- P42A electrical and installed two-node thermal model validation.
- Exact fuse/contactor/busbar/cable/inverter/charger/regen coordination review.
- Full five-segment HIL and exact ECU integration/fault injection.
- Board-specific two-slot nonvolatile SoH storage adapter and power-cut testing.
- Staged dyno and low-energy vehicle validation.

See:

- `Docs/DYNAMIC_SOP_IMPLEMENTATION.md`
- `Docs/SOP_SOH_CALIBRATION_AND_LIMITS.md`
- `Docs/SOP_SOH_CAN_CONTRACT.md`
- `Docs/SOP_SOH_COMMISSIONING.md`
- `SOP_SOH_VALIDATION_REPORT.md`
