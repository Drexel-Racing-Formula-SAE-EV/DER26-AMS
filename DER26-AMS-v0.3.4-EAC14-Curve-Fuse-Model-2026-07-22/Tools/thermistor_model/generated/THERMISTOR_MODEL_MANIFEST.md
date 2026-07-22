# DER26 Thermistor Model Manifest and Validation

## Source identity

- Part: `NTCLE350E4103FHB0`
- Vishay CSV: `NTC_RT_Calculation_Vishay_NTCLE350E4103FHB0.csv`
- CSV SHA-256: `db3446078e15efcf3cc1d0647a4a580ba8a9a18b0edb9a4c60a819551fa7fa83`
- Datasheet: `ntcle350e4.pdf`
- Datasheet SHA-256: `50cdd86414a4e315b21cc3b834c21104e1c80292acfdf038f627a7618a9c9718`
- R25: `10000 ohm`
- B25/85: `3984 K`
- Table: `281 points`, `-20.0 C` to `120.0 C`, `0.5 C` spacing

## Extended Steinhart-Hart coefficients

Forward `T -> R`:

```text
R = R25 * exp(A + B/Tk + C/Tk^2 + D/Tk^3)
A = -14.65719769000
B = 4798.84200000000
C = -115334.00000000000
D = -3730535.00000000000
```

Inverse `R -> T`:

```text
L = ln(R/R25)
T = 1/(A1 + B1*L + C1*L^2 + D1*L^3) - 273.15
A1 = 0.00335401643468052988
B1 = 0.000256523550896126009
C1 = 2.60597012072051983e-06
D1 = 6.32926126487459937e-08
```

## Numerical results

- Current truncated A1+B1 implementation maximum nominal table error: `3.974671 C` at `120.0 C`.
- Current truncated implementation RMS nominal table error: `1.491301 C`.
- Full Vishay inverse equation maximum difference from rounded CSV table: `0.011263 C` at `-18.0 C`.
- Full Vishay inverse equation RMS difference from rounded CSV table: `0.004821 C`.
- Dense LUT-versus-full-equation maximum difference: `0.013259 C`.
- Dense 150-uV ADC-code round-trip maximum difference: `0.016012 C`.
- Vishay CSV maximum listed thermistor tolerance contribution: `+/-0.98 C` within the exported range.

## Important raw-code finding

ADBMS raw code `0` maps to `1.500000 V`, `23333.333 ohm`, production-LUT temperature `6.712384 C`, and extended-equation temperature `6.705107 C`. It is therefore a valid physical code and must not be rejected merely because its numeric value is zero. Only the documented reset/clear sentinels `0xFFFF` and `0x8000` are rejected by the new shared model.

## Production selection

- Primary runtime conversion: manufacturer LUT with binary search and linear interpolation.
- Independent reference: full Vishay extended Steinhart-Hart inverse equation.
- HIL inverse: full Vishay forward equation.
- Electrically valid values beyond the exported LUT range are clamped to the nearest table endpoint and flagged `CLAMPED_COLD` or `CLAMPED_HOT`; the estimator rejects clamped values while the safety path receives a conservative extreme temperature.

## Board-component tolerance envelope

- Populated pull-down part: `Panasonic EXB38V103JV`, nominal `10 kohm`, resistance tolerance `+/-5.0%`.
- Pull-down tolerance alone produces up to approximately `1.906 C` nominal decode error in the exported range.
- Vishay CSV Rmin/Rmax combined with pull-down tolerance produces a worst-corner nominal decode envelope of approximately `+/-2.862 C`; the largest absolute corner occurs near `120.0 C`.
- This is a deterministic component-corner calculation, not a complete statistical uncertainty model.
- Official Panasonic part page: https://industrial.panasonic.com/ww/products/pt/resistor-network-array/models/EXB38V103JV

## Remaining physical validation

The runtime conversion uses nominal component values. The separate corner analysis includes the Vishay thermistor Rmin/Rmax and +/-5% pull-down resistance, but it still excludes VREG error, pull-down TCR over actual board temperature, ADBMS AUX total measurement error, mux leakage/on resistance, harness resistance, thermistor mounting/contact error, cell-to-sensor thermal lag, and pack gradients. Complete resistance-substitution and thermal-chamber validation remain required.
