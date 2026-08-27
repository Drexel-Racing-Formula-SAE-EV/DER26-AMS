# DER26 Vishay Thermistor Model Generator

This directory owns the shared conversion for the SMB thermistor specified by
`TEMP_SENSE.SchDoc`:

```text
Vishay NTCLE350E4103FHB0
R25 = 10 kOhm, +/-1%
B25/85 = 3984 K, +/-0.5%
```

## Source

`source/NTC_RT_Calculation_Vishay_NTCLE350E4103FHB0.csv` is the Vishay NTC R/T
Calculator export from -20 C through 120 C at 0.5 C intervals. The generation
script validates the exact ordering code, R25, B25/85, point count, temperature
grid, monotonicity, and tolerance-column ordering before producing firmware
artifacts.

## Regenerate

From the repository root:

```bash
python3 Tools/thermistor_model/generate_thermistor_model.py \
  --csv Tools/thermistor_model/source/NTC_RT_Calculation_Vishay_NTCLE350E4103FHB0.csv \
  --datasheet Tools/thermistor_model/source/ntcle350e4.pdf \
  --model-header AMS/Core/Inc/ext_drivers/thermistor_model_generated.h \
  --lut-header AMS/Core/Src/ext_drivers/thermistor_lut_generated.h \
  --comparison Tools/thermistor_model/generated/thermistor_comparison.csv \
  --tolerance-analysis Tools/thermistor_model/generated/thermistor_board_tolerance.csv \
  --manifest Tools/thermistor_model/generated/THERMISTOR_MODEL_MANIFEST.md
```

## Runtime architecture

- Production `R -> T`: manufacturer resistance LUT, binary search, linear interpolation.
- Independent reference `R -> T`: full Vishay extended Steinhart-Hart equation.
- HIL `T -> R`: full Vishay forward equation.
- Generated part identity, source hash, and full Vishay coefficients: `thermistor_model_generated.h`.
- Shared divider/raw-code conversion: `thermistor_model.c`.
- Board-component corner analysis: Vishay Rmin/Rmax plus the populated
  `EXB38V103JV` 10 kOhm, +/-5% pull-down tolerance.

The old two-coefficient conversion is not retained as a model. It is present
only in the generated comparison report to quantify the change.


## Independently validate the compiled C implementation

This second script does not import the generator. It reads the original Vishay
CSV, builds `thermistor_model.c` as an isolated shared library, calls the actual
C API through `ctypes`, checks every manufacturer table row, checks a 0.01 C
dense grid, and verifies the 150 uV ADBMS raw-code round trip:

```bash
python3 Tools/thermistor_model/validate_compiled_model.py \
  --repo . \
  --csv Tools/thermistor_model/source/NTC_RT_Calculation_Vishay_NTCLE350E4103FHB0.csv \
  --output Tools/thermistor_model/generated/compiled_validation.json
```
