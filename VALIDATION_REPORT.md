# DER26 AMS Thermistor Model Integration — Validation Report v0.1

## 1. Scope

This package replaces the truncated two-coefficient thermistor conversion with a shared, generated model for the SMB thermistor specified by the DER26 hardware:

```text
Vishay NTCLE350E4103FHB0
R25 = 10 kΩ, ±1%
B25/85 = 3984 K, ±0.5%
```

The implementation provides both requested paths:

1. **Production conversion:** manufacturer resistance/temperature LUT, descending binary search, and linear interpolation.
2. **Independent mathematical reference:** full Vishay extended Steinhart–Hart inverse equation.

It also centralizes the full Vishay forward equation for HIL temperature-to-resistance/raw-code generation.

Current firmware baseline referenced by the package:

- Commit: `32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9`
- Commit link: <https://github.com/Drexel-Racing-Formula-SAE-EV/DER26-AMS/commit/32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9>

## 2. Important source correction

The previous firmware constants were not random and were not merely a different 3898 K Beta curve. They were Vishay's exact inverse coefficients `A1` and `B1`, with the higher-order `C1·L²` and `D1·L³` terms omitted.

Legacy form:

```text
L = ln(R/R25)
T = 1/(A1 + B1·L) - 273.15
```

Full Vishay inverse now retained:

```text
L = ln(R/R25)
T = 1/(A1 + B1·L + C1·L² + D1·L³) - 273.15

A1 = 0.003354016434680530
B1 = 0.000256523550896126
C1 = 0.00000260597012072052
D1 = 0.000000063292612648746
```

Full forward equation used for HIL:

```text
Tk = T°C + 273.15
R = R25 · exp(A + B/Tk + C/Tk² + D/Tk³)

A = -14.65719769
B = 4798.842
C = -115334
D = -3730535
```

## 3. Source identity and integrity

| Artifact | SHA-256 |
|---|---|
| Vishay CSV | `db3446078e15efcf3cc1d0647a4a580ba8a9a18b0edb9a4c60a819551fa7fa83` |
| Vishay datasheet | `50cdd86414a4e315b21cc3b834c21104e1c80292acfdf038f627a7618a9c9718` |

The generator rejects unexpected:

- ordering code;
- R25;
- B25/85;
- row count;
- temperature range;
- 0.5 °C grid;
- non-monotonic resistance data;
- invalid Rmin/Rnom/Rmax ordering.

The CSV contains 281 points from −20 °C through 120 °C at 0.5 °C spacing.

## 4. Runtime architecture

```text
ADBMS signed AUX code
      │
      ▼
V = (raw + 10000) × 150 µV
      │
      ▼
RNTC = Rpull × (VREG − V) / V
      │
      ▼
281-point Vishay nominal R/T LUT
      │
      ▼
descending binary search
      │
      ▼
linear interpolation
      │
      ▼
validated temperature + explicit status
```

The shared result reports:

- temperature;
- resistance;
- divider voltage;
- validity;
- electrical/model status;
- whether the model endpoint was clamped.

Statuses include:

- `OK`;
- `CLAMPED_COLD`;
- `CLAMPED_HOT`;
- `OPEN_CIRCUIT`;
- `SHORT_CIRCUIT`;
- `ADC_SENTINEL`;
- VREG/voltage/resistance/numeric faults.

## 5. Safety and estimator behavior

At nominal 5 V:

```text
V ≤ 0.100 V → open circuit
V ≥ 4.900 V → short circuit
```

Electrically valid values beyond the exported manufacturer table are handled as follows:

```text
colder than −20 °C → valid, reported −20 °C, CLAMPED_COLD
hotter than 120 °C → valid, reported 120 °C, CLAMPED_HOT
```

This preserves conservative safety behavior. The estimator explicitly rejects clamped temperatures because they are outside the exact sensor-model domain used for the thermal observer.

The accumulator now uses the shared thermistor status directly for open/short masks. The duplicated raw-voltage open/short classifier was removed so classification cannot drift away from the shared model.

## 6. Raw-code-zero defect corrected

ADBMS raw code `0` is a valid physical code:

```text
V = 1.500000 V
R ≈ 23,333.33 Ω
T ≈ 6.712385 °C using the production LUT
```

It must not be treated as “no reading.” The integration corrects host and estimator tests that previously used raw zero as an invalid thermistor value.

Only the explicit reset/clear sentinels are rejected:

```text
0xFFFF / -1
0x8000 / INT16_MIN
```

## 7. Numerical validation results

| Check | Measured result | Acceptance limit | Result |
|---|---:|---:|---|
| Compiled LUT at all 281 CSV points | 0.000000 °C max | 0.0006 °C | PASS |
| Full extended SH vs rounded CSV | 0.011261 °C max | 0.012 °C | PASS |
| Dense LUT vs full SH | 0.013266 °C max | 0.014 °C | PASS |
| 150 µV raw-code round trip | 0.016038 °C max | 0.020 °C | PASS |
| Forward equation relative resistance error | 1.291×10⁻⁵ max | 1.5×10⁻⁴ | PASS |
| Raw code zero accepted | Yes | Required | PASS |

Legacy truncated equation:

- maximum nominal-table error: **3.974671 °C at 120 °C**;
- RMS nominal-table error: **1.491301 °C**.

## 8. Component-corner analysis

The SMB BOM pull-down is `EXB38V103JV`, nominal 10 kΩ with ±5% resistance tolerance and ±200 ppm/K TCR. Runtime conversion uses the nominal resistance; a generated artifact separately analyzes deterministic component corners.

| Corner model | Maximum absolute decoded-temperature error over −20 to 120 °C |
|---|---:|
| Pull-down ±5% only | approximately 1.906 °C |
| Vishay Rmin/Rmax plus pull-down ±5% | approximately 2.862 °C |

The largest combined corner appears near 120 °C. This is not a full statistical uncertainty model. It excludes:

- VREG error;
- pull-down TCR at actual PCB temperature;
- ADBMS AUX total measurement error;
- mux leakage/on resistance;
- harness resistance;
- sensor mounting/contact error;
- cell-to-sensor thermal lag;
- pack spatial gradients.

The key engineering result is that the corrected model's numerical error is now much smaller than the physical divider and installation uncertainty.

## 9. Generated and added files

### Firmware model

- `AMS/Core/Inc/ext_drivers/thermistor_model.h`
- `AMS/Core/Inc/ext_drivers/thermistor_model_generated.h`
- `AMS/Core/Src/ext_drivers/thermistor_model.c`
- `AMS/Core/Src/ext_drivers/thermistor_lut_generated.h`

### Firmware integration

- `AMS/Core/Src/ext_drivers/accumulator.c`
- `AMS/Core/Inc/ext_drivers/accumulator.h`
- `AMS/Core/Src/ext_drivers/adbms6830.c`
- `AMS/Core/Inc/ext_drivers/adbms6830_functions.h`
- `AMS/Core/Src/tasks/estimator_task.c`
- `AMS/Core/Src/tasks/cli_task.c`

### Generation and evidence

- `Tools/thermistor_model/generate_thermistor_model.py`
- `Tools/thermistor_model/validate_compiled_model.py`
- `Tools/thermistor_model/README.md`
- exact source CSV and datasheet copies;
- generated comparison CSV;
- generated board-tolerance CSV;
- generated model manifest;
- independent compiled-C validation JSON.

### Host tests

- `AMS/host_tests/unit/thermistor_model_unit_test.c`
- `AMS/host_tests/Makefile`
- `AMS/host_tests/src/ams_host_test_runner.c`

The stale `NXFT15XV103FEAB050_convert()` helper was removed from the accumulator API and implementation.

## 10. Verification completed

The final modified tree passed:

- deterministic generator idempotency;
- Python syntax compilation;
- strict standalone C11 compile with `-Wall -Wextra -Werror -pedantic`;
- dedicated thermistor golden, dense, raw, fault, boundary, monotonicity and metadata tests;
- independent compiled-C validation against every Vishay CSV row;
- complete host `make ci`;
- production feature/ownership/static-allocation gates;
- GCC `-fanalyzer`;
- AddressSanitizer;
- UndefinedBehaviorSanitizer;
- 50,000 seeded long-fuzz cycles;
- 12,000 concurrent-stress cycles;
- clean binary-capable patch application to a fresh copy of the supplied baseline; the patched tree matched the modified source content and passed thermistor compiled validation plus unit tests.

## 11. Environment limitation

The execution environment did not contain `arm-none-eabi-gcc`, and direct network checkout of the exact Git commit was unavailable. Therefore:

- no final STM32F767 Debug/Release target link was performed here;
- the patch was generated and tested against the supplied hardened DER26 archive whose relevant file structure and logic match the current documented firmware baseline;
- the patch must still be applied and target-built from exact commit `32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9` before merge.

This limitation is explicit rather than hidden.

## 12. Required merge checks

1. Check out exact commit `32dffc7dd882ffe1c7c72d83f543b5cc2aa9abe9`.
2. Run `git apply --check DER26_AMS_Thermistor_Model_v0.1.patch`.
3. Apply the patch and inspect any context conflict.
4. Regenerate all thermistor artifacts and verify no diff.
5. Run host `make ci`, `make asan`, `make ubsan`, and `make stress`.
6. Refresh the CubeIDE project and clean-build Debug and Release.
7. Confirm `thermistor_model.c` is compiled and linked exactly once.
8. Inspect the map and section-size report.
9. Update the existing CI size policy if this legitimate LUT/code growth crosses its deliberately small budget.
10. Run low-voltage precision-resistance substitution through connector, mux, ADBMS, and firmware.
11. Validate actual installed thermistors against a calibrated temperature reference.
12. Re-run temperature thresholds, fan behavior, thermal observer, and SoP sensitivity using measured uncertainty.

## 13. Physical validation remains required

Software validation establishes mathematical and integration correctness. It does not prove:

- the exact external thermistor was purchased and installed;
- every pull-down element is within expected value;
- all 120 channels map to the correct physical location;
- mux settling and ADBMS AUX accuracy;
- sensor attachment quality;
- cell-to-sensor thermal response;
- complete thermal uncertainty at warning/fault limits.

Those tests remain the release gate for thermal accuracy and temperature-constrained SoP authority.
