# DER26 AMS — Thermistor Mapping and Validation

> **Status:** Draft v0.3 — Vishay LUT and full extended Steinhart–Hart integration package generated  
> **Purpose:** Define the physical-to-logical temperature-sensor map, validate all 120 thermistor channels through the SMB mux network, verify conversion and fault classification, and establish freshness, thermal-policy, fan, balancing, CAN, and fail-closed behavior  
> **Audience:** Firmware, electrical, accumulator, thermal, systems, validation, and new team members  
> **Current firmware:** [DER26-AMS commit `32dffc7`](https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/commit/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9)  
> **Hardware:** Five DER SMB Rev 5 boards  
> **Sensors:** 24 thermistor channels per SMB, 120 total  
> **Mux architecture:** Three 8-channel ADG728-class mux groups per SMB  
> **Acquisition owner:** ADBMS task  
> **Fan consumer:** Fan task at 5 Hz  
> **Primary commands:** `temp`, `tempsns <ic> <sensor>`, `spi auxdiag`, `fault`  
> **Safety boundary:** BMS_OK and balancing remain inhibited until sensor identity, channel map, conversion, open/short behavior, freshness, and thermal policy are validated

> [!IMPORTANT]
> A plausible temperature is not enough. Every accepted value must be tied to the correct physical sensor, mux channel, SMB slot, current scan, valid PEC/counter state, and correct resistance-to-temperature model.

---

# 1. The complete temperature path

```text
Physical thermistor
      │
      ▼
SMB thermistor connector/harness
      │
      ▼
Bias/reference and divider
      │
      ▼
One of three 8-channel muxes
      │
      ▼
ADBMS6830 GPIO/AUX input
      │
      ▼
isoSPI chain
      │
      ▼
ADBMS task
      │
      ├── mux selection
      ├── settling delay
      ├── AUX conversion
      ├── transport/PEC/counter validation
      ├── raw-code to voltage
      ├── voltage/resistance to temperature
      ├── open/short/invalid checks
      ├── freshness and masks
      ├── jump/rate/overtemperature policy
      └── aggregate thermal publication
      │
      ▼
Shared temperature state
      │
      ├── Safety task / BMS_OK
      ├── Fan task
      ├── Estimator
      ├── CAN
      └── CLI
```

---

# 2. Current channel grouping

The current hardware uses three 8-channel mux groups per SMB.

```text
Mux group 0 → sensors 0–7
Mux group 1 → sensors 8–15
Mux group 2 → sensors 16–23
```

Working hardware notes associate the three groups with three ADBMS GPIO/AUX paths.

The exact schematic reference designators and GPIO assignments must be copied from the released SMB Rev 5 schematic into the master crosswalk.

## Logical indexing

```text
SMB / segment index: 0–4
Sensor index:        0–23
```

Human labels may be 1–24. Firmware indices are zero-based.

---

# 3. Current thermistor hardware contract

The SMB Rev 5 temperature-sense schematic explicitly specifies:

```text
NTC: NTCLE350E4103FHB0
Manufacturer: Vishay BCcomponents
R25: 10 kΩ
R25 tolerance: ±1%
B25/85: 3984 K
B25/85 tolerance: ±0.5%
```

The ordering code explains the tolerance and curve:

```text
NTCLE350E4103 F H B0
                 │ │
                 │ └── 3984 K curve family
                 └──── ±1% R25 option
```

The current board uses six `EXB-38V103JV` 10 kΩ ×4 resistor arrays as pull-down resistors. Each external thermistor is connected between `VREG` and its `Temp_n` node, while the corresponding 10 kΩ resistor connects the node to ground.

## Current divider

```text
VREG
  │
  └── NTCLE350E4103FHB0
          │
          ├── Temp_n → mux → ADBMS6830 AUX
          │
          └── 10 kΩ pull-down → GND
```

Therefore:

```text
Vtemp = VREG × Rpull / (RNTC + Rpull)

RNTC = Rpull × (VREG − Vtemp) / Vtemp
```

with the current nominal values:

```text
VREG  = 5.0 V nominal
Rpull = 10 000 Ω nominal
R25   = 10 000 Ω
```

## Current source record

| Field | Current value |
|---|---|
| Manufacturer | Vishay BCcomponents |
| Part number | `NTCLE350E4103FHB0` |
| Nominal resistance | 10 kΩ at 25 °C |
| R25 tolerance | ±1% |
| Datasheet B25/85 | 3984 K |
| B tolerance | ±0.5% |
| Operating range | −55 °C to +185 °C |
| Lead construction | PEEK-insulated Ag-plated NiFe leads |
| Pull resistor | Panasonic `EXB38V103JV`, 10 kΩ nominal, ±5%, ±200 ppm/K |
| Divider orientation | NTC to VREG, pull-down to GND |
| Schematic source | SMB Rev 5 `TEMP_SENSE.SchDoc` |
| Datasheet source | Vishay NTCLE350E4, document 29218 |

The thermistor identity and nominal manufacturer curve are **not unknown**. The generated integration package now reconciles the production and HIL conversion with this exact part. The remaining work is target-build integration and physical board-, harness-, mounting-, and thermal-system validation.

---

# 4. Vishay curve discovery and corrected model

The Vishay NTC R/T Calculator export for the exact ordering code
`NTCLE350E4103FHB0` contains the complete manufacturer curve from −20 °C to
120 °C at 0.5 °C spacing, plus both directions of Vishay's extended
Steinhart–Hart model.

## Important correction to the earlier diagnosis

The legacy production conversion used:

```text
L = ln(R / 10 000)

T = 1 / (A1 + B1 × L) − 273.15

A1 = 3.354016434680530×10⁻³
B1 = 2.565235508961260×10⁻⁴
```

Those are not arbitrary constants and should not be described simply as a
3898 K thermistor substituted for the specified 3984 K part. They are the
first two coefficients from Vishay's exact inverse extended Steinhart–Hart
curve. The actual problem is that the legacy implementation omitted the
manufacturer's `C1 × L²` and `D1 × L³` terms.

The complete inverse equation is:

```text
L = ln(R / R25)

T = 1 /
    (A1 + B1×L + C1×L² + D1×L³)
    − 273.15

A1 = 0.003354016434680530
B1 = 0.000256523550896126
C1 = 0.00000260597012072052
D1 = 0.000000063292612648746
```

The complete forward equation used for HIL is:

```text
Tk = T_C + 273.15

R = R25 × exp(A + B/Tk + C/Tk² + D/Tk³)

A = −14.65719769
B = 4798.842
C = −115334
D = −3730535
```

## Generated implementation package

The generated integration uses one versioned Vishay CSV as the source of truth:

```text
Vishay CSV
      │
      ├── generated public identity and coefficient header
      ├── generated 281-point nominal resistance LUT
      ├── production LUT R → T conversion
      ├── independent full extended-SH R → T reference
      ├── full Vishay T → R HIL inverse
      ├── comparison report
      └── compiled-C validation report
```

### Production path

```text
ADBMS raw code
      │
      ▼
V = (raw + 10 000) × 150 µV
      │
      ▼
RNTC = 10 kΩ × (VREG − V) / V
      │
      ▼
Binary search of descending Vishay R/T LUT
      │
      ▼
Linear interpolation between 0.5 °C points
```

### Independent reference path

The complete four-coefficient extended Steinhart–Hart equation remains callable
for parity testing and engineering analysis. Production safety conversion does
not depend on this second implementation.

### HIL inverse path

The former two-coefficient inverse in `accumulator.c` is replaced by the full
Vishay forward equation. Production and HIL therefore share one manufacturer
model rather than two manually duplicated approximations.

## Numerical results

| Check | Result |
|---|---:|
| Legacy truncated equation, maximum nominal-table error | 3.974671 °C at 120 °C |
| Legacy truncated equation, RMS nominal-table error | 1.491301 °C |
| Full Vishay inverse versus rounded CSV, maximum difference | 0.011263 °C |
| Production LUT versus full equation over dense range | 0.013266 °C maximum |
| 150 µV ADBMS raw-code round trip | 0.016038 °C maximum |
| Compiled LUT at all 281 CSV breakpoints | 0.000000 °C maximum |
| Maximum thermistor tolerance listed by Vishay in exported range | ±0.98 °C |

These are numerical model results. They do not include the full board-level and
installation uncertainty.

## Divider-component tolerance result

The populated pull-down is not a precision 1% resistor. The SMB BOM part
`EXB38V103JV` is nominally 10 kΩ with ±5% resistance tolerance and ±200 ppm/K
TCR. The runtime conversion intentionally uses the nominal 10 kΩ value, while
the generated tolerance artifact evaluates component corners separately.

Using the nominal divider equation to decode the physical corner cases:

| Corner model | Maximum absolute temperature error over −20 to 120 °C |
|---|---:|
| Pull-down ±5% alone | approximately 1.906 °C |
| Vishay thermistor Rmin/Rmax plus pull-down ±5% | approximately 2.862 °C |

The largest combined component corner occurs near the hot end of the exported
range. This is a deterministic worst-corner envelope, not a statistical RSS
uncertainty. It still excludes VREG error, pull-down TCR at actual PCB
temperature, ADBMS AUX total error, mux leakage/on-resistance, harness
resistance, sensor attachment, cell-to-sensor lag, and pack temperature
gradients.

This finding matters because the production LUT/extended-SH numerical error is
only hundredths of a degree. After the model correction, divider-component and
physical thermal uncertainties dominate the temperature path.

## Raw-code zero correction

ADBMS raw code `0` is physically valid:

```text
raw = 0
V = 1.500000 V
RNTC ≈ 23 333.33 Ω
production LUT ≈ 6.712 °C
```

It must not be rejected merely because its numeric value is zero. The shared
model rejects the explicit reset/clear sentinels:

```text
0xFFFF = −1
0x8000 = INT16_MIN
```

Existing tests and estimator/accumulator guards that treated raw zero as an
invalid thermistor code are corrected in the integration package.

## Electrical fault and model-boundary behavior

At nominal 5 V:

```text
V ≤ 0.100 V → open-circuit classification
V ≥ 4.900 V → short-circuit classification
```

Electrically valid resistance values beyond the exported −20 to 120 °C table
are handled conservatively:

```text
colder than LUT → report −20 °C, CLAMPED_COLD
hotter than LUT → report 120 °C, CLAMPED_HOT
```

The safety path can use the conservative endpoint. The estimator thermal
observer rejects model-clamped values because they are outside its exact
characterized sensor-conversion domain.

## Software validation completed for the integration package

- dedicated thermistor unit tests;
- every Vishay table row checked against compiled production C;
- dense 0.01 °C LUT/full-equation parity sweep;
- dense ADBMS raw-code round trip;
- exact open/short boundaries;
- 4.5–5.5 V reference-boundary behavior;
- raw-zero validity;
- reset/clear sentinels;
- monotonicity and endpoint tests;
- full host CI;
- GCC analyzer;
- AddressSanitizer;
- UndefinedBehaviorSanitizer;
- 50 000 seeded and 12 000 concurrent stress cycles;
- deterministic generator idempotency.

A full STM32 target link was not executed in the generation environment because
`arm-none-eabi-gcc` was unavailable. CubeIDE includes all files under `Core`, so
the new source is in the managed source tree, but the exact target Debug and
Release builds remain required before merge.

## Remaining physical validation

Before releasing thermal accuracy or using temperature-based SoP authority:

1. confirm installed thermistor purchasing/assembly records match
   `NTCLE350E4103FHB0`;
2. confirm the populated pull-down arrays are 10 kΩ;
3. run precision-resistance substitution through connector, mux, ADBMS and
   firmware;
4. measure VREG and ADBMS AUX accuracy;
5. validate actual thermistors against a calibrated temperature reference;
6. characterize attachment error and cell-to-sensor thermal lag;
7. verify all thermal warning, charge-stop, fan-max and hard-fault boundaries;
8. rerun thermal-observer and SoP sensitivity tests using measured uncertainty.

> [!IMPORTANT]
> The model approximation is now much smaller than the thermistor's own listed
tolerance. The dominant remaining uncertainty is expected to come from the
physical divider, measurement chain, mounting, thermal contact and pack
spatial gradients—not the LUT interpolation.

---

# 5. Physical sensor placement map

The system cannot validate thermal coverage without knowing what each sensor touches.

## Required location categories

Examples may include:

- cell-group surface;
- cell interconnect/busbar;
- segment interior;
- segment exterior;
- inlet/ambient;
- SMB PCB;
- balancing-resistor region;
- connector/harness region.

Do not invent a location from the sensor number.

## Master crosswalk

| Global sensor | Segment | Board ID | Physical position | Software slot | Firmware sensor index | Mux group/channel | ADBMS AUX path | Connector pin | Harness wire | Physical location | Attachment method |
|---:|---:|---|---|---:|---:|---|---|---|---|---|---|
| 1 | 0 | | P0 | | 0 | group 0 / ch 0 | | | | | |
| … | | | | | | | | | | | |
| 120 | 4 | | P4 | | 23 | group 2 / ch 7 | | | | | |

## Global numbering

Recommended:

```text
global_sensor = segment_index × 24 + local_sensor_index + 1
```

This convention must match CAN/logger/dashboard documentation.

---

# 6. Mux addressing model

## Visual model

```text
Requested sensor index
      │
      ▼
Determine mux group
      │
      ├── 0–7   → group 0
      ├── 8–15  → group 1
      └── 16–23 → group 2
      │
      ▼
Determine local mux channel 0–7
      │
      ▼
Write/select mux control
      │
      ▼
Wait settling interval
      │
      ▼
Start AUX conversion
      │
      ▼
Read selected ADBMS AUX path
```

## Required mapping formula

Conceptually:

```text
mux_group   = sensor_index / 8
mux_channel = sensor_index % 8
```

The actual firmware implementation and hardware address encoding must be checked against the current source and schematic.

## Required proof

For each group:

- channel 0 selects the correct physical sensor;
- channel 7 selects the correct physical sensor;
- switching does not alias another group;
- no off-by-one address;
- no previous channel remains selected;
- all unused mux paths are handled safely.

---

# 7. Settling-time validation

Mux switching, thermistor-divider impedance, filter capacitance, and ADBMS input behavior create settling delay.

## Visual model

```text
Select new mux channel
      │
      ▼
Old channel charge remains on node
      │
      ▼
RC settling
      │
      ▼
ADC conversion
```

If conversion starts too soon:

- the new reading is biased toward the previous sensor;
- alternating hot/cold channels can expose ghosting;
- the error may look plausible.

## Test

Use two adjacent mux channels with very different known resistances:

```text
cold-equivalent resistance
      ↔
hot-equivalent resistance
```

Alternate repeatedly and measure:

- raw AUX voltage versus time;
- error at current delay;
- minimum delay meeting approved accuracy;
- effect of scan order.

The released settling delay must have margin across component tolerance and temperature.

---

# 8. Direct sensor polling versus periodic publication

## Direct CLI diagnostic

```text
tempsns <ic> <sensor>
```

This performs a bounded direct transaction for one sensor.

Use it to isolate:

- mux address;
- one divider;
- one connector;
- one AUX path;
- conversion calculation.

## Periodic `temp`

The periodic publication includes:

- complete scan logic;
- per-sensor masks;
- freshness;
- aggregate min/max/average;
- jump/rate/open/short/fault policy.

## Visual model

```text
`tempsns`
  one direct diagnostic path

`temp`
  complete periodic system view
```

A direct correct result does not prove the periodic 120-channel scan is correct.

A periodic correct value does not replace the need to validate direct physical mapping.

---

# 9. Establish board and slot identity first

Use the completed [[Five-SMB Chain Bring-Up]] map.

```text
Physical P0–P4
      │
      ▼
Board IDs
      │
      ▼
Software slots I0–I4
```

Temperature mapping must use the same board-slot mapping as cell voltage.

Do not create a separate, inconsistent SMB order for thermistors.

---

# 10. Unique resistance fingerprints

## Board fingerprint

Assign one distinct known resistance to one sensor on each physical board.

Example:

```text
P0 sensor 0 → value A
P1 sensor 1 → value B
P2 sensor 2 → value C
P3 sensor 3 → value D
P4 sensor 4 → value E
```

## Channel fingerprint

Within one mux group, connect known resistances in a unique order.

The actual values must remain:

- within the valid conversion range;
- away from open/short thresholds;
- distinct beyond tolerance/noise;
- safe for the divider.

## Visual model

```text
Known resistance pattern
      │
      ▼
Measured AUX voltages
      │
      ▼
Converted temperatures
      │
      ▼
Identify board, mux group, and channel order
```

---

# 11. Resistance substitution fixture

Use a protected fixture that can select:

- nominal room-temperature resistance;
- colder-equivalent resistance;
- warmer-equivalent resistance;
- near-hot-limit resistance;
- open circuit;
- short or approved near-short.

## Required fixture record

- schematic;
- resistor values/tolerances;
- switch/contact resistance;
- connector pinout;
- maximum current;
- measurement uncertainty;
- isolation;
- labels.

Do not short a real thermistor harness directly without understanding the bias network.

---

# 12. Raw-code and voltage validation

Before validating degrees Celsius, validate the electrical reading.

## Flow

```text
Known resistor
      │
      ▼
Expected divider voltage
      │
      ▼
Measured SMB mux output
      │
      ▼
ADBMS raw AUX code
      │
      ▼
Firmware voltage
```

## Required record

| Board | Sensor | Known resistance | Expected voltage | DMM voltage | Raw code | Firmware voltage | Error |
|---|---:|---:|---:|---:|---:|---:|---:|
| | | | | | | | |

This separates:

- hardware divider/mux error;
- ADC error;
- resistance-to-temperature conversion error.

---

# 13. Resistance-to-temperature validation

## Required points

At minimum:

- cold point;
- room-temperature point;
- warm point;
- near warning threshold;
- near fault threshold.

Use:

- calibrated resistance substitutions;
- or a controlled thermal chamber/reference thermometer.

## Record

| Known resistance | Reference temperature | Firmware temperature | Error | Model/table version | Pass |
|---:|---:|---:|---:|---|---:|
| | | | | | |

## Conversion risks

- wrong nominal resistance;
- wrong Beta value;
- Celsius/Kelvin error;
- integer overflow;
- log-domain error;
- wrong divider equation;
- wrong pull-up/pull-down orientation;
- using voltage after clamp/saturation;
- table interpolation error;
- signedness/scale error.

---

# 14. Open and short classification

## Electrical model

```text
Thermistor open
      │
      ▼
Divider node approaches one rail

Thermistor short
      │
      ▼
Divider node approaches opposite rail
```

Which rail corresponds to open or short depends on the actual divider topology.

## Required proof

Do not label a high voltage “open” until the schematic and bench prove it.

## Test matrix

| Board | Sensor | Condition | Raw voltage | Firmware classification | Aggregate validity | Recovery |
|---|---:|---|---:|---|---|---|
| | | Open | | | | |
| | | Short | | | | |
| | | Nominal | | | | |

## Fail-closed requirement

```text
Open/short/invalid
      │
      ▼
sensor unusable
      │
      ▼
required-channel completeness affected
      │
      ▼
temperature validity/fault policy
      │
      ▼
BMS_OK low when required by current policy
```

The last numeric temperature must not remain marked valid.

---

# 15. Full 120-channel scan

For complete production-like thermal validity:

```text
5 SMBs × 24 sensors = 120
```

## Expected totals

```text
updated = 120
usable  = 120
stale   = 0
```

The current `temp` output should also expose or summarize:

- invalid count;
- open count;
- short count;
- jump count;
- excessive-rate count;
- maximum temperature;
- minimum temperature;
- average temperature;
- maximum rate;
- max/min location;
- per-SMB masks.

## Interpretation

| Condition | Meaning |
|---|---|
| `updated=120`, `usable<120` | Fresh data arrived but one or more channels were rejected |
| `updated<120` | Incomplete mux/AUX scan |
| `stale>0` | One or more sensors not refreshed |
| one entire group missing | Mux group/control/AUX path issue |
| every 8th channel wrong | Addressing or mux-boundary issue |
| values repeat in groups | stale selection or aliasing |

---

# 16. Complete mapping procedure

For each SMB:

1. verify representative sensor 0;
2. verify representative sensor 8;
3. verify representative sensor 16;
4. map sensors 0–7;
5. map sensors 8–15;
6. map sensors 16–23;
7. compare direct `tempsns` with periodic `temp`;
8. repeat for all five boards.

## Required matrix

| Physical board | Physical sensor | Software slot | Firmware sensor | Mux group | Mux channel | Known resistance | CLI result | Correct |
|---|---:|---:|---:|---:|---:|---:|---|---:|
| P0 | 1 | | 0 | 0 | 0 | | | |
| … | | | | | | | | |
| P4 | 24 | | 23 | 2 | 7 | | | |

No channel is complete until both electrical and physical-location mapping are recorded.

---

# 17. Freshness and stale behavior

## Visual model

```text
Last valid sensor scan
      │
      ▼
No new valid conversion
      │
      ▼
age increases
      │
      ▼
freshness timeout
      │
      ▼
sensor stale/unusable
      │
      ▼
aggregate temperature invalid/faulted
```

## Fault tests

- disconnect thermistor harness;
- disable one mux group in a test fixture;
- remove one SMB;
- break isoSPI chain;
- suppress AUX response;
- stall ADBMS task;
- restore.

Recovery requires a new valid scan with:

- correct mux selection;
- valid PEC/counter;
- current timestamp;
- valid conversion.

---

# 18. Jump and rate-of-rise validation

Thermal safety may include:

- absolute overtemperature;
- severe overtemperature;
- rapid rise;
- implausible jump;
- pending/debounce behavior.

## Jump test

Use a resistance switch to create a known instantaneous electrical step.

This proves software behavior, not real thermal dynamics.

## Rate test

Use a controlled resistance ramp or thermal chamber.

Record:

- sample period;
- filtered/unfiltered values;
- calculated rate;
- warning/fault threshold;
- persistence;
- recovery.

## Important distinction

```text
Electrical connection jump
      ≠
real physical temperature rise
```

Both need tests.

---

# 19. Thermal thresholds and charging behavior

The thermal policy can produce:

- warning;
- fan demand;
- maximum fan command;
- charge stop;
- overtemperature pending;
- overtemperature fault;
- severe fault.

## Visual model

```text
Temperature rises
      │
      ├── warning
      ├── fan demand
      ├── fan maximum
      ├── charge stop
      └── overtemperature fault
```

## Required tests

- threshold approach;
- threshold crossing;
- hysteresis;
- debounce/persistence;
- recovery;
- charge versus discharge state;
- one hot sensor;
- multiple hot sensors;
- one invalid sensor;
- hottest-sensor location.

The released limits must come from requirements and rule review, not this onboarding page.

---

# 20. Fan-task dependency

## Visual model

```text
ADBMS task publishes thermal summary
      │
      ▼
Fan task at 5 Hz
      │
      ├── read valid temperature state
      ├── determine demand/reason
      ├── command six PWM outputs
      └── publish fan status
```

## Important limitation

```text
PWM command
      ≠
fan rotation
      ≠
airflow
      ≠
cooling effectiveness
```

There is no documented tachometer or airflow feedback in the current system.

## Required cross-check

For temperature test points, verify:

- expected fan-demand percentage;
- expected reason;
- PWM at MCU pin;
- driver output;
- fan motion separately;
- fan heartbeat;
- fan fault behavior.

---

# 21. Balancing interaction

Passive balancing can heat:

- balance resistors;
- PCB area;
- adjacent cells;
- local thermistors.

During initial thermistor mapping:

```text
balance inhibit
```

must remain active.

## Later integrated test

```text
Known balancing command
      │
      ▼
Measure resistor/PCB/cell temperature
      │
      ▼
Verify intended nearby sensor responds
      │
      ▼
Verify thermal policy
```

## Current design-review topic

A thermistor near the balancing area has been discussed as a future hardware improvement.

Do not document it as present unless the current PCB assembly and schematic prove it.

---

# 22. Estimator relationship

The estimator uses temperature as an advisory model input.

## Visual model

```text
Fresh valid temperature
      │
      ▼
Select/update temperature-dependent model
      │
      ▼
Estimator correction and thermal observer

Invalid/stale temperature
      │
      ▼
Fallback/invalid estimator behavior
```

A thermistor mapping error can corrupt:

- model parameter selection;
- core-temperature estimate;
- SoC/R0 estimation;
- thermal telemetry.

Estimator output must remain advisory until the complete measurement/model chain is validated.

---

# 23. CAN relationship

Compact CAN thermal frame `0x682` exports:

- maximum temperature;
- minimum temperature;
- filtered average;
- maximum fan command;
- warning/fault flags.

Frame `0x683` exports:

- hottest segment;
- hottest sensor;
- usable temperature count.

## Required cross-check

```text
CLI `temp`
      │
      ▼
shared thermal state
      │
      ▼
0x682 / 0x683
      │
      ▼
ECU/logger decode
```

Verify:

- signed temperature scale;
- invalid sentinel handling;
- hottest location;
- usable count = 120;
- flags;
- no stale number exported as valid.

---

# 24. Sensor placement validation

Electrical mapping alone does not prove the sensor is attached to the intended thermal target.

## Required physical checks

- location photograph;
- sensor contact;
- adhesive/retention;
- electrical isolation;
- strain relief;
- harness routing;
- repeatability across segments;
- sensor not measuring free air unintentionally;
- sensor not detached after service.

## Thermal response test

Apply controlled heat locally to the intended physical location.

Verify:

- correct software sensor rises;
- adjacent sensors respond plausibly;
- no remote/incorrect sensor is identified;
- response time recorded;
- recovery recorded.

Do not use uncontrolled heating on real cells.

---

# 25. Accuracy and uncertainty

## Error sources

- thermistor tolerance;
- curve coefficient tolerance;
- pull resistor tolerance;
- reference-voltage error;
- ADBMS AUX ADC error;
- mux on-resistance/leakage;
- harness/contact resistance;
- self-heating;
- thermal contact error;
- sensor placement;
- conversion rounding;
- filtering;
- reference thermometer uncertainty.

## Required report

| Board | Sensor | Reference °C | AMS °C | Error °C | Method | Settling time | Pass |
|---|---:|---:|---:|---:|---|---:|---:|
| | | | | | | | |

The released acceptance limit must preserve adequate margin to thermal warning/fault thresholds.

---

# 26. Noise and EMI tests

Test under:

- quiet bench;
- CAN traffic;
- CLI traffic;
- fan PWM;
- charger LV activity;
- contactor-coil LV simulator;
- balancing later;
- inverter/EMI environment later.

Inspect:

- raw voltage variation;
- converted temperature variation;
- false open/short;
- false jump/rate;
- mux aliasing;
- stale/PEC faults;
- hottest-sensor instability.

Filtering can reduce noise but adds lag. Include filter delay in thermal fault timing.

---

# 27. Failure propagation

## Invalid sensor data

```text
Open/short/stale/invalid
      │
      ▼
Per-sensor unusable mask
      │
      ▼
Aggregate temp validity/fault policy
      │
      ▼
Safety task
      │
      ▼
BMS_OK low when current policy requires
```

## ADBMS task still runs

The temperature heartbeat may remain fresh while the data are invalid.

```text
Heartbeat fresh
      ≠
thermal measurement healthy
```

## ADBMS task stalls

```text
Temperature heartbeat stale
      │
      ▼
task-heartbeat fault
      │
      ▼
BMS_OK low
```

---

# 28. Symptom-first debugging

## One channel always reads open/short

Check:

- physical thermistor;
- connector;
- harness;
- divider;
- mux channel;
- AUX path;
- mapping;
- classification threshold.

## Eight-channel group missing

Check:

- mux power;
- address/control;
- group select;
- corresponding ADBMS GPIO/AUX path;
- settling;
- connector.

## Sensors repeat every eight channels

Check:

- mux-group selection;
- index division/modulo;
- previous mux still selected;
- wrong GPIO path.

## Direct `tempsns` works, periodic `temp` is wrong

Check:

- scan order;
- publication index;
- freshness masks;
- buffer overwrite;
- loop bounds;
- settling between channels.

## Periodic works, direct command fails

Check:

- ADBMS mutex ownership;
- CLI service path;
- command-counter resync;
- selected IC/slot;
- command usage.

## All temperatures plausible but validity false

Check:

- 120-channel completeness;
- stale sensors;
- one invalid/open/short;
- PEC/counter;
- diagnostic fault;
- threshold/pending/latch.

## Hottest location wrong

Check:

- physical map;
- slot map;
- zero/one-based index;
- equal-value tie behavior;
- filtered versus raw maximum.

---

# 29. Validation levels

## Level A — hardware identity

- thermistor part;
- divider;
- mux/GPIO map;
- connector/harness.

## Level B — electrical mapping

- all 120 channels;
- unique resistance pattern;
- raw voltage accuracy;
- settling.

## Level C — temperature conversion

- multiple temperatures;
- model/LUT;
- uncertainty;
- repeatability.

## Level D — fault policy

- open/short;
- stale;
- jump/rate;
- thresholds;
- latching/recovery.

## Level E — physical placement

- local heat response;
- attachment;
- segment consistency.

## Level F — integration

- fan demand;
- balancing heat;
- estimator;
- CAN;
- fault-to-BMS-low;
- EMI.

---

# 30. Pass criteria

## Mapping

- [ ] Five board slots frozen.
- [ ] All 120 sensors mapped.
- [ ] Three mux groups per board verified.
- [ ] No duplicated, shifted, or aliased channel.
- [ ] Physical locations documented.

## Electrical

- [ ] Thermistor part identified.
- [ ] Divider and reference measured.
- [ ] Raw voltage matches expected.
- [ ] Mux settling validated.
- [ ] Open/short polarity proven.

## Conversion

- [ ] Hardware part is confirmed as `NTCLE350E4103FHB0`.
- [ ] The firmware 3898.28 K coefficient versus datasheet 3984 K mismatch is resolved.
- [ ] Forward conversion and HIL inverse use one shared versioned thermistor definition.
- [ ] Approved LUT/equation version recorded.
- [ ] Multiple temperature points tested.
- [ ] Error within approved tolerance.
- [ ] Noise/drift recorded.
- [ ] Repeatability passed.

## Safety

- [ ] 120 updated/usable sensors for complete scan.
- [ ] Stale data become unusable.
- [ ] Open/short fail closed.
- [ ] Threshold and rate behavior tested.
- [ ] Temperature heartbeat timeout tested.
- [ ] BMS_OK-low timing measured.

## Interfaces

- [ ] Fan demand matches thermal policy.
- [ ] CAN thermal fields match CLI.
- [ ] Estimator ignores invalid/stale temperature.
- [ ] Balancing interaction separately scheduled.
- [ ] Evidence archived.

---

# 31. Evidence folder

```text
thermistor_validation_YYYY-MM-DD/
├── README.md
├── firmware_commit.txt
├── build_profile.txt
├── thermistor_identity/
├── source_documents.txt
├── physical_logical_map.csv
├── sensor_crosswalk_120.csv
├── mux_gpio_crosswalk.csv
├── resistance_fixture/
├── raw_voltage_validation.csv
├── settling_time/
├── conversion_accuracy/
├── open_short/
├── stale_recovery/
├── jump_rate/
├── threshold_tests/
├── physical_location_photos/
├── local_heat_response/
├── fan_crosscheck/
├── balancing_crosscheck/
├── can_crosscheck/
├── estimator_crosscheck/
├── cli_baseline.txt
└── result.md
```

## `sensor_crosswalk_120.csv`

```text
global_sensor,segment,board_id,physical_position,software_slot,firmware_sensor_index,mux_group,mux_channel,adbms_aux_path,connector_pin,harness_wire,physical_location,attachment,verified
```

## `result.md`

```text
Date:
Operators:
Firmware commit:
Build profile:
SMB IDs/revisions:
Thermistor part: NTCLE350E4103FHB0
R25 / tolerance: 10 kΩ / ±1%
Datasheet B25/85 / tolerance: 3984 K / ±0.5%
Legacy conversion before correction: Vishay A1/B1 terms only; C1/D1 omitted
Curve-resolution result: full manufacturer LUT plus full extended Steinhart–Hart
Conversion model/version: NTCLE350E4103FHB0 model revision 1
Mux/GPIO map:
All 120 mapped:
Settling delay:
Worst raw-voltage error:
Worst temperature error:
Open/short result:
Freshness result:
Threshold/rate result:
Physical placement result:
Fan result:
CAN result:
Estimator result:
Achieved validation level:
Open blockers:
```

---

# 32. Source anchors

- [Current commit `32dffc7`](https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/commit/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9)
- [`accumulator.h`](https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/blob/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9/AMS/Core/Inc/ext_drivers/accumulator.h)
- [`accumulator.c`](https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/blob/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9/AMS/Core/Src/ext_drivers/accumulator.c)
- [`adbms6830.c`](https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/blob/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9/AMS/Core/Src/ext_drivers/adbms6830.c)
- [`adbms_task.c`](https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/blob/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9/AMS/Core/Src/tasks/adbms_task.c)
- [`fan_task.c`](https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/blob/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9/AMS/Core/Src/tasks/fan_task.c)
- [`estimator_task.c`](https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/blob/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9/AMS/Core/Src/tasks/estimator_task.c)
- [`cli_task.c`](https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/blob/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9/AMS/Core/Src/tasks/cli_task.c)
- [`app.h`](https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/blob/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9/AMS/Core/Inc/app.h)

Generated integration package:

- `AMS/Core/Inc/ext_drivers/thermistor_model.h`;
- `AMS/Core/Inc/ext_drivers/thermistor_model_generated.h`;
- `AMS/Core/Src/ext_drivers/thermistor_model.c`;
- `AMS/Core/Src/ext_drivers/thermistor_lut_generated.h`;
- `Tools/thermistor_model/generate_thermistor_model.py`;
- `Tools/thermistor_model/validate_compiled_model.py`;
- `Tools/thermistor_model/generated/thermistor_comparison.csv`;
- `Tools/thermistor_model/generated/thermistor_board_tolerance.csv`;
- `Tools/thermistor_model/generated/compiled_validation.json`;
- `AMS/host_tests/unit/thermistor_model_unit_test.c`.

Hardware:

- SMB Rev 5 `TEMP_SENSE.SchDoc`, which specifies `NTCLE350E4103FHB0`;
- SMB BOM, including `EXB-38V103JV` 10 kΩ resistor arrays;
- Vishay NTCLE350E4 datasheet, document 29218;
- thermistor harness;
- accumulator mechanical/sensor placement drawings;
- exact thermistor datasheet;
- ADBMS6830B datasheet;
- five-SMB evidence.

---

# 33. Page-maintenance requirements

Update this page whenever any of these change:

- thermistor part;
- R25 and Beta/curve coefficients;
- firmware forward and HIL inverse conversion constants;
- divider/reference;
- mux part or mapping;
- ADBMS GPIO/AUX assignment;
- connector/harness;
- sensor placement;
- sensor count;
- LUT/equation;
- settling delay;
- thresholds/rate policy;
- freshness;
- fan policy;
- balancing layout;
- CAN format;
- estimator use;
- current firmware commit.

> [!IMPORTANT]
> Keep electrical channel mapping and physical sensor placement as separate proofs. A perfectly decoded sensor attached to the wrong location is still a system-level mapping failure.
