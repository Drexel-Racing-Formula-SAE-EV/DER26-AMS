# SoP/SoH calibration and system-limit register

## Rule for changing values

Every value in `ams_sop_default_config()` or `ams_soh_default_config()` is a
versioned safety calibration. A change requires a source, unit review, boundary
tests, Python/C differential tests, target timing measurement, and an update to
`AMS_THRESHOLD_REVISION`, `AMS_SOP_MODEL_REVISION`, or
`AMS_SOH_MODEL_REVISION` as applicable. Do not tune limits from a single track
log.

## Pack and model basis

| Item | Value used | Basis | Release action |
|---|---:|---|---|
| Topology | 75s6p | DER26 AMS/TS schematics | Verify physical segment harness and configured DADEKF mapping |
| Segments | 5 x 15 series groups | AMS schematic and firmware | All five estimators required for authority |
| Thermistors | 5 x 24 = 120 | SMB schematic and firmware | All usable for authority; validate installed sensor contact |
| Cell | Molicel INR-21700-P42A | Vehicle BOM | Lot traceability and incoming characterization |
| Cell nominal capacity | 4.2 Ah | P42A typical capacity | Use 4.0 Ah minimum in acceptance sensitivity study |
| Pack nominal capacity | 25.2 Ah | 4.2 Ah x 6p | Confirm against measured full-cycle capacity |
| Absolute cell voltage | 2.5-4.2 V | P42A specification | Not used as normal control threshold |
| ECM R2 | 4 mOhm/cell | Existing DER26 HIL calibration | Pulse-test identification required |
| ECM C2 | 12000 F/cell | Existing DER26 HIL calibration | Pulse-test identification required |
| Core heat capacity | 55 J/K | Existing DER26 HIL calibration | Thermal chamber/dyno validation required |
| Surface heat capacity | 15 J/K | Existing DER26 HIL calibration | Thermal chamber/dyno validation required |
| Core-surface resistance | 1.5 K/W | Existing DER26 HIL calibration | Thermal chamber/dyno validation required |
| Surface-ambient resistance | 8 K/W | Existing DER26 HIL calibration | Validate with installed cooling and airflow |

The target OCV, R0, C1, and tau1 tables are the production P42A estimator LUTs.
The independent Python oracle reads separately generated HIL tables to avoid
using the same implementation as its numerical truth source.

## Operational constraints

| Constraint | Default | Rationale |
|---|---:|---|
| Cell undervoltage | 2.80 V | 300 mV above the 2.5 V cell absolute minimum before additional uncertainty |
| Cell overvoltage | 4.15 V | 50 mV below the 4.2 V cell maximum before additional uncertainty |
| Minimum SOC | 0.05 | Avoid unobservable/depleted tail and preserve voltage margin |
| Maximum SOC | 0.98 | Preserve charge headroom |
| Discharge core/surface | 55 degC | Below P42A 60 degC discharge maximum before model/measurement margin |
| Charge core/surface | 42 degC | Below P42A 45 degC charge maximum before margin |
| Minimum charge surface | 3 degC | Above P42A 0 degC charge minimum before margin |

The predictor adds its uncertainty margins to these operational values. They
are not redundant copies of the hard AMS fault thresholds; the power envelope
must reduce torque before a hard protection trip is approached.

## Direction- and horizon-specific pack-current limits

| Horizon | Discharge | Charge/regen magnitude | Current hardware basis |
|---:|---:|---:|---|
| 0.1 s | 118 A | 11.5 A | Existing 120 A/100 ms discharge and 15 A/100 ms charge fast-trip policy, with margin |
| 1 s | 80 A | 10 A | 80 A main fuse, 100 A contactor, 85 A/500 ms discharge trip, 10 A charger |
| 10 s | 70 A | 10 A | Existing 70 A discharge warning and 10 A charge warning/charger policy |
| 30 s | 70 A | 10 A | Same sustained system policy; not the cell-array theoretical limit |

Installed protection references include an Eaton EAC14-80-SCT 80 A main fuse,
TE ECK100B 100 A contactors, 15 A charging-branch fuses, and a nominal 10 A
charger. The P42A 6p cell array can support much more current than these vehicle
components, so cell current rating must not be used as the pack limit.

The fuse data sheet's 8020 A2s value is a typical point measured at 800 A, not
a universal time-independent budget. v0.3.4 replaces the fixed-budget observer
with a subtractive model built from the digitized EAC14-80 current-versus-time
centerline, the exact 800 A table anchor, a 300 s thermal-memory constant, and
the published temperature derating curve. The hottest-surface-plus-15-degC
value remains a proxy because fuse temperature is not directly measured.

The default model allows 25% of the typical centerline time. This remains a
commissioning-only choice: the data sheet does not publish guaranteed-minimum
curve tolerances or repetitive-pulse aging. The model remains shadow-only until
`AMS_FUSE_MODEL_VALIDATED=1` and its initialization state is valid. Before
changing that calibration or relying on its derating:

1. Obtain guaranteed-minimum EAC14-80-SCT pre-arcing time-current/I2t data and
   curve tolerances from Eaton.
2. Include ambient/preheating, repeated-pulse thermal memory, holder/busbar
   heating, contactor temperature, cable ampacity, inverter limit, and rules.
3. Test the complete installed path with calibrated current and temperature
   instrumentation.
4. Replay measured acceleration, autocross, and endurance current traces.
5. Keep the independent AMS overcurrent trips below destructive component
   limits with documented coordination.

Regen remains zero in vehicle policy until `AMS_REGEN_TARGET_VALIDATED=1` and
the ECU/inverter, high-SOC, cold-cell, and shutdown behavior have been tested.

## Uncertainty defaults

| Term | Default | Application |
|---|---:|---|
| DADEKF confidence multiplier | 3 sigma | SOC, RC voltage, and R0 covariance |
| Cell-voltage measurement | 5 mV | Added to every predicted cell bound |
| Voltage model allowance | 20 mV | Added to measurement/covariance/innovation margin |
| Temperature measurement | 1.5 degC | Added to predicted core/surface temperature |
| Temperature model allowance | 1.5 degC | Added independently |
| Current uncertainty floor | 0.5 A pack | Applied in adverse current direction |
| Maximum estimator innovation | 100 mV/cell equivalent | Beyond this, SoP input is invalid |
| Maximum measurement age | 250 ms | Checked at solver and again at CAN publication |
| Unknown capacity lower bound | 0.80 | Used until capacity SoH is observable and valid |
| Unknown resistance upper bound | 1.25 | Used for any unqualified segment |

The 5 mV electronic value is not a complete installed-cell uncertainty budget.
Harness, connector, ADBMS gain/offset, reference drift, cell-tab gradients, and
model residuals must be measured. If the characterized bound is larger, the
configuration must increase; it must not be reduced to make more power.

## Prediction grid and publication policy

| Range | Step |
|---|---:|
| 0-1 s | 0.1 s |
| 1-10 s | 0.5 s |
| 10-30 s | 1.0 s |

Each current boundary uses 16 bisection iterations. Any decrease, invalidation,
or loss of authorization is immediate. Recovery defaults to 40 A/s DCL and
5 A/s CCL, with the following scheduled exceptions:

| Cause | DCL | CCL |
|---|---:|---:|
| Cell-voltage polarization | 800 A/s | 120 A/s |
| Thermal | 8 A/s | 2 A/s |
| Current path/fuse | up to 20 A/s | 4 A/s |
| SoC | 5 A/s only after 0.5% SoC and 453.6 As restoring throughput | same |

Mission strategy defaults are 30% weakest-segment SoC lower-bound trigger and
35 A Limp Home cap. The 35 A value is a conservative commissioning baseline,
not a measured energy-optimal current; vehicle road-load and efficiency data
are required before changing that claim or value.

## SoH observation defaults

| Requirement | Default |
|---|---:|
| Rest current including uncertainty | <= 0.5 A |
| Continuous rest duration | >= 60 s |
| Rest temperature | 10-40 degC |
| Cell spread | <= 50 mV |
| Maximum segment SOC sigma | <= 0.015 |
| Maximum innovation | <= 15 mV/cell |
| Maximum residual polarization | <= 20 mV |
| Window SOC excursion | >= 0.15 |
| Window charge throughput | >= 3 Ah |
| Capacity candidate domain | 50-110% nominal |
| Relative outlier threshold | 20% |
| Capacity observations before valid | 2 |
| Capacity sigma floor | 0.5 Ah |
| Resistance confidence per segment | >= 50% |
| Resistance uncertainty floor | 0.05 growth ratio |

Capacity SoH is intentionally slow and observable-only. A vehicle that never
reaches the stated rest/throughput conditions continues to use the conservative
prior; it does not manufacture an ageing estimate from elapsed time.

## Required characterization procedure

### Current path

1. Verify positive current means pack discharge using a bidirectional reference.
2. Calibrate all sensor ranges over charge and discharge, including zero drift,
   temperature, supply variation, saturation, and range transition.
3. Store a nonzero calibration ID and a conservative uncertainty bound.
4. Confirm disconnected, stuck, saturated, implausible dual-range, and stale
   inputs make DCL/CCL zero.

### Electrical model

1. Characterize representative new and aged P42A cells across the used SOC and
   temperature grid with HPPC-like pulses.
2. Fit OCV, R0, R1/C1, and R2/C2 with a held-out validation set.
3. Validate cell-voltage residual percentiles for 0.1, 1, 10, and 30 seconds.
4. Set the model margin above the worst accepted residual plus instrumentation
   uncertainty; demonstrate conservative weak-cell prediction.

### Thermal model

1. Instrument cell core proxies and the production thermistor locations in an
   installed segment.
2. Exercise current pulses, sustained loads, fans, hot soak, cold soak, and
   airflow loss.
3. Identify the two-node parameters and validate worst-case core/surface error
   over every horizon.
4. Replace the hottest-surface ambient proxy only if an independent, diagnosed
   ambient sensor is installed.

### SoH

1. Compare rest-anchor capacity against a calibrated full-cycle capacity test.
2. Validate current integration error over the longest expected observation
   window and across resets.
3. Compare R0 growth against controlled pulse tests for all five segments.
4. Power-cycle during both persistence-slot writes and verify newest-valid
   recovery without falsely marking incomplete observations valid.

## Build manifest gates

A vehicle profile must define and acknowledge all of:

```text
AMS_SOP_MODEL_VALIDATED=1
AMS_SOP_CALIBRATION_VALIDATED=1
AMS_SOP_CAN_CONTRACT_VALIDATED=1
AMS_MISSION_CAN_CONTRACT_VALIDATED=1
AMS_FUSE_MODEL_VALIDATED=1
AMS_SOP_MODEL_REVISION=<immutable revision>
AMS_SOH_MODEL_REVISION=<immutable revision>
AMS_THRESHOLD_REVISION=<immutable revision>
AMS_CURRENT_CALIBRATION_REVISION=<immutable revision>
```

The default values identify source configuration only. They are not evidence
that the installed vehicle completed these procedures.
