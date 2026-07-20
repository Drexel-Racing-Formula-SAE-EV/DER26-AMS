#!/usr/bin/env python3
"""Generate the DER26 Vishay NTCLE350E4103FHB0 thermistor artifacts.

Inputs:
  - Vishay NTC R/T Calculator CSV export

Outputs:
  - firmware resistance LUT header
  - nominal comparison CSV
  - Markdown manifest/validation summary

The production firmware uses the manufacturer R/T table with piecewise-linear
interpolation.  The full Vishay extended Steinhart-Hart equation is retained as
an independent implementation and as the forward model used by HIL.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import re
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean

EXPECTED_PART = "NTCLE350E4103FHB0"
EXPECTED_R25_OHM = 10000.0
EXPECTED_B2585_K = 3984.0
EXPECTED_ROWS = 281
EXPECTED_TMIN_C = -20.0
EXPECTED_TMAX_C = 120.0
EXPECTED_STEP_C = 0.5
PULLDOWN_NOMINAL_OHM = 10000.0
PULLDOWN_TOLERANCE_PERCENT = 5.0


@dataclass(frozen=True)
class Coefficients:
    a: float
    b: float
    c: float
    d: float
    a1: float
    b1: float
    c1: float
    d1: float


@dataclass(frozen=True)
class Row:
    temperature_c: float
    rnom_ohm: float
    rmin_ohm: float
    rmax_ohm: float
    delta_r_percent: float
    delta_t_c: float
    alpha_percent_per_k: float


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()




def c_float_literal(value: float, significant_digits: int = 18) -> str:
    """Return a valid, deterministic C float literal.

    A bare value such as ``10000f`` is not valid ISO C, so integer-looking
    values receive an explicit decimal point.
    """
    text = format(value, f".{significant_digits}g")
    if ("." not in text) and ("e" not in text.lower()):
        text += ".0"
    return text + "f"

def _find_scalar(lines: list[str], label: str) -> str:
    for line in lines:
        fields = next(csv.reader([line]))
        if fields and fields[0].strip() == label:
            for value in fields[1:]:
                value = value.strip()
                if value:
                    return value
    raise ValueError(f"Missing scalar {label!r}")


def _extract_coefficient(lines: list[str], label: str, start: int) -> float:
    for line in lines[start:]:
        fields = next(csv.reader([line]))
        if fields and fields[0].strip() == label:
            values = [x.strip() for x in fields[1:] if x.strip()]
            if not values:
                break
            if len(values) == 2 and values[0] != values[1]:
                raise ValueError(f"Piecewise coefficient {label} differs: {values}")
            return float(values[0])
    raise ValueError(f"Missing coefficient {label!r}")


def parse_csv(path: Path) -> tuple[str, float, float, Coefficients, list[Row]]:
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    part = _find_scalar(lines, "Ordering Code")
    b2585 = float(_find_scalar(lines, "B(25/85) [K]"))
    r25 = float(_find_scalar(lines, "R at 25 deg C [ohm]"))

    forward_formula_idx = next(i for i, line in enumerate(lines) if line.startswith("RT=R25*EXP"))
    inverse_formula_idx = next(i for i, line in enumerate(lines) if line.startswith("T=1/(A1+"))
    table_idx = next(i for i, line in enumerate(lines) if line.startswith("T [celsius]"))

    coeffs = Coefficients(
        a=_extract_coefficient(lines, "A", forward_formula_idx + 1),
        b=_extract_coefficient(lines, "B", forward_formula_idx + 1),
        c=_extract_coefficient(lines, "C", forward_formula_idx + 1),
        d=_extract_coefficient(lines, "D", forward_formula_idx + 1),
        a1=_extract_coefficient(lines, "A1", inverse_formula_idx + 1),
        b1=_extract_coefficient(lines, "B1", inverse_formula_idx + 1),
        c1=_extract_coefficient(lines, "C1", inverse_formula_idx + 1),
        d1=_extract_coefficient(lines, "D1", inverse_formula_idx + 1),
    )

    reader = csv.DictReader(lines[table_idx:])
    rows: list[Row] = []
    for raw in reader:
        if not raw or raw.get("T [celsius]") in (None, ""):
            continue
        rows.append(
            Row(
                temperature_c=float(raw["T [celsius]"]),
                rnom_ohm=float(raw[" Rnom [ohms]"]),
                rmin_ohm=float(raw[" Rmin [ohms]"]),
                rmax_ohm=float(raw[" Rmax [ohms]"]),
                delta_r_percent=float(raw[" delta R/R [+-%]"]),
                delta_t_c=float(raw[" delta T [+- celsius]"]),
                alpha_percent_per_k=float(raw[" alpha [%/K]"]),
            )
        )

    validate(part, r25, b2585, rows)
    return part, r25, b2585, coeffs, rows


def validate(part: str, r25: float, b2585: float, rows: list[Row]) -> None:
    if part != EXPECTED_PART:
        raise ValueError(f"Expected part {EXPECTED_PART}, got {part}")
    if not math.isclose(r25, EXPECTED_R25_OHM, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(f"Expected R25={EXPECTED_R25_OHM}, got {r25}")
    if not math.isclose(b2585, EXPECTED_B2585_K, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(f"Expected B25/85={EXPECTED_B2585_K}, got {b2585}")
    if len(rows) != EXPECTED_ROWS:
        raise ValueError(f"Expected {EXPECTED_ROWS} table rows, got {len(rows)}")
    if not math.isclose(rows[0].temperature_c, EXPECTED_TMIN_C, abs_tol=1e-9):
        raise ValueError("Unexpected first temperature")
    if not math.isclose(rows[-1].temperature_c, EXPECTED_TMAX_C, abs_tol=1e-9):
        raise ValueError("Unexpected last temperature")

    for i, row in enumerate(rows):
        expected_t = EXPECTED_TMIN_C + EXPECTED_STEP_C * i
        if not math.isclose(row.temperature_c, expected_t, abs_tol=1e-9):
            raise ValueError(f"Temperature grid mismatch at row {i}: {row.temperature_c}")
        if not (row.rnom_ohm > 0.0 and row.rmin_ohm > 0.0 and row.rmax_ohm > 0.0):
            raise ValueError(f"Nonpositive resistance at row {i}")
        if not (row.rmin_ohm <= row.rnom_ohm <= row.rmax_ohm):
            raise ValueError(f"Tolerance ordering invalid at row {i}")
        if i and not (rows[i - 1].rnom_ohm > row.rnom_ohm):
            raise ValueError(f"Nominal resistance is not strictly descending at row {i}")


def sh_temperature_c(resistance_ohm: float, r25: float, c: Coefficients) -> float:
    x = math.log(resistance_ohm / r25)
    denominator = c.a1 + c.b1 * x + c.c1 * x * x + c.d1 * x * x * x
    return (1.0 / denominator) - 273.15


def truncated_temperature_c(resistance_ohm: float, r25: float, c: Coefficients) -> float:
    x = math.log(resistance_ohm / r25)
    return (1.0 / (c.a1 + c.b1 * x)) - 273.15


def resistance_from_temperature_c(temperature_c: float, r25: float, c: Coefficients) -> float:
    tk = temperature_c + 273.15
    exponent = c.a + c.b / tk + c.c / (tk * tk) + c.d / (tk * tk * tk)
    return r25 * math.exp(exponent)


def lut_temperature_c(resistance_ohm: float, rows: list[Row]) -> float:
    if resistance_ohm >= rows[0].rnom_ohm:
        return rows[0].temperature_c
    if resistance_ohm <= rows[-1].rnom_ohm:
        return rows[-1].temperature_c

    lo = 0
    hi = len(rows) - 1
    while (hi - lo) > 1:
        mid = lo + (hi - lo) // 2
        if resistance_ohm <= rows[mid].rnom_ohm:
            lo = mid
        else:
            hi = mid

    cold = rows[lo]
    hot = rows[lo + 1]
    fraction = (cold.rnom_ohm - resistance_ohm) / (cold.rnom_ohm - hot.rnom_ohm)
    return cold.temperature_c + fraction * (hot.temperature_c - cold.temperature_c)


def generate_model_header(
    path: Path, source_hash: str, part: str, r25: float, b2585: float, c: Coefficients
) -> None:
    content = f"""/* Auto-generated by Tools/thermistor_model/generate_thermistor_model.py.
 * Do not edit manually.
 *
 * Source: Vishay NTC R/T Calculator export for {part}
 * SHA-256: {source_hash}
 */
#ifndef INC_EXT_DRIVERS_THERMISTOR_MODEL_GENERATED_H_
#define INC_EXT_DRIVERS_THERMISTOR_MODEL_GENERATED_H_

#define THERMISTOR_MODEL_PART_NUMBER            \"{part}\"
#define THERMISTOR_MODEL_SOURCE_SHA256           \"{source_hash}\"
#define THERMISTOR_R25_OHM                      ({c_float_literal(r25)})
#define THERMISTOR_B2585_K                      ({c_float_literal(b2585)})
#define THERMISTOR_MODEL_MIN_TEMP_C             ({c_float_literal(EXPECTED_TMIN_C)})
#define THERMISTOR_MODEL_MAX_TEMP_C             ({c_float_literal(EXPECTED_TMAX_C)})
#define THERMISTOR_MODEL_TABLE_STEP_C            ({c_float_literal(EXPECTED_STEP_C)})

/* Full Vishay extended Steinhart-Hart inverse: R -> T. */
#define THERMISTOR_SH_A1                        ({c_float_literal(c.a1)})
#define THERMISTOR_SH_B1                        ({c_float_literal(c.b1)})
#define THERMISTOR_SH_C1                        ({c_float_literal(c.c1)})
#define THERMISTOR_SH_D1                        ({c_float_literal(c.d1)})

/* Full Vishay forward equation: T -> R. */
#define THERMISTOR_FWD_A                        ({c_float_literal(c.a, 15)})
#define THERMISTOR_FWD_B                        ({c_float_literal(c.b, 15)})
#define THERMISTOR_FWD_C                        ({c_float_literal(c.c, 15)})
#define THERMISTOR_FWD_D                        ({c_float_literal(c.d, 15)})

#endif /* INC_EXT_DRIVERS_THERMISTOR_MODEL_GENERATED_H_ */
"""
    path.write_text(content, encoding="utf-8")


def generate_lut_header(path: Path, source_hash: str, part: str, rows: list[Row]) -> None:
    values = []
    for i, row in enumerate(rows):
        suffix = "," if i < len(rows) - 1 else ""
        values.append(f"    {row.rnom_ohm:.2f}f{suffix} /* {row.temperature_c:6.1f} C */")

    content = f"""/* Auto-generated by Tools/thermistor_model/generate_thermistor_model.py.
 * Do not edit manually.
 *
 * Source: Vishay NTC R/T Calculator export for {part}
 * SHA-256: {source_hash}
 * Grid: {rows[0].temperature_c:.1f} C to {rows[-1].temperature_c:.1f} C in {EXPECTED_STEP_C:.1f} C steps
 */
#ifndef THERMISTOR_LUT_GENERATED_H
#define THERMISTOR_LUT_GENERATED_H

#include <stdint.h>

#define THERMISTOR_LUT_COUNT          {len(rows)}u
#define THERMISTOR_LUT_MIN_TEMP_C     ({rows[0].temperature_c:.1f}f)
#define THERMISTOR_LUT_MAX_TEMP_C     ({rows[-1].temperature_c:.1f}f)
#define THERMISTOR_LUT_STEP_C         ({EXPECTED_STEP_C:.1f}f)
#define THERMISTOR_LUT_COLD_R_OHM     ({rows[0].rnom_ohm:.2f}f)
#define THERMISTOR_LUT_HOT_R_OHM      ({rows[-1].rnom_ohm:.2f}f)

static const float thermistor_lut_resistance_ohm[THERMISTOR_LUT_COUNT] =
{{
{chr(10).join(values)}
}};

#endif /* THERMISTOR_LUT_GENERATED_H */
"""
    path.write_text(content, encoding="utf-8")


def write_comparison(path: Path, rows: list[Row], r25: float, c: Coefficients) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "temperature_c",
            "rnom_ohm",
            "rmin_ohm",
            "rmax_ohm",
            "vishay_delta_t_c",
            "current_truncated_c",
            "current_error_c",
            "extended_sh_c",
            "extended_sh_error_c",
            "lut_c",
            "lut_error_c",
        ])
        for row in rows:
            trunc = truncated_temperature_c(row.rnom_ohm, r25, c)
            sh = sh_temperature_c(row.rnom_ohm, r25, c)
            lut = lut_temperature_c(row.rnom_ohm, rows)
            writer.writerow([
                f"{row.temperature_c:.1f}",
                f"{row.rnom_ohm:.2f}",
                f"{row.rmin_ohm:.1f}",
                f"{row.rmax_ohm:.1f}",
                f"{row.delta_t_c:.2f}",
                f"{trunc:.9f}",
                f"{trunc - row.temperature_c:.9f}",
                f"{sh:.9f}",
                f"{sh - row.temperature_c:.9f}",
                f"{lut:.9f}",
                f"{lut - row.temperature_c:.9f}",
            ])


def write_board_tolerance_analysis(path: Path, rows: list[Row]) -> tuple[float, float, float]:
    """Write a deterministic component-tolerance envelope.

    This combines the Vishay CSV's Rmin/Rmax with the populated Panasonic
    EXB38V103JV pull-down's +/-5% resistance tolerance. It intentionally omits
    VREG, ADBMS, mux, harness, and mounting uncertainty.
    """
    pull_min = PULLDOWN_NOMINAL_OHM * (1.0 - PULLDOWN_TOLERANCE_PERCENT / 100.0)
    pull_max = PULLDOWN_NOMINAL_OHM * (1.0 + PULLDOWN_TOLERANCE_PERCENT / 100.0)
    max_combined_abs = 0.0
    max_pull_only_abs = 0.0
    max_combined_temperature = rows[0].temperature_c

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "true_temperature_c",
            "decoded_min_c",
            "decoded_max_c",
            "error_min_c",
            "error_max_c",
            "pull_only_error_min_c",
            "pull_only_error_max_c",
        ])

        for row in rows:
            combined = []
            pull_only = []
            for pull_actual in (pull_min, pull_max):
                for ntc_actual in (row.rmin_ohm, row.rmax_ohm):
                    decoded_resistance = (
                        PULLDOWN_NOMINAL_OHM * ntc_actual / pull_actual
                    )
                    combined.append(lut_temperature_c(decoded_resistance, rows))

                pull_only_resistance = (
                    PULLDOWN_NOMINAL_OHM * row.rnom_ohm / pull_actual
                )
                pull_only.append(lut_temperature_c(pull_only_resistance, rows))

            decoded_min = min(combined)
            decoded_max = max(combined)
            error_min = decoded_min - row.temperature_c
            error_max = decoded_max - row.temperature_c
            pull_error_min = min(pull_only) - row.temperature_c
            pull_error_max = max(pull_only) - row.temperature_c

            local_abs = max(abs(error_min), abs(error_max))
            if local_abs > max_combined_abs:
                max_combined_abs = local_abs
                max_combined_temperature = row.temperature_c
            max_pull_only_abs = max(
                max_pull_only_abs, abs(pull_error_min), abs(pull_error_max)
            )

            writer.writerow([
                f"{row.temperature_c:.1f}",
                f"{decoded_min:.9f}",
                f"{decoded_max:.9f}",
                f"{error_min:.9f}",
                f"{error_max:.9f}",
                f"{pull_error_min:.9f}",
                f"{pull_error_max:.9f}",
            ])

    return max_combined_abs, max_pull_only_abs, max_combined_temperature


def dense_validation(rows: list[Row], r25: float, c: Coefficients) -> tuple[float, float]:
    max_lut_sh = 0.0
    max_raw_round_trip = 0.0
    steps = 14000
    for i in range(steps + 1):
        t = EXPECTED_TMIN_C + (EXPECTED_TMAX_C - EXPECTED_TMIN_C) * i / steps
        r = resistance_from_temperature_c(t, r25, c)
        t_lut = lut_temperature_c(r, rows)
        t_sh = sh_temperature_c(r, r25, c)
        max_lut_sh = max(max_lut_sh, abs(t_lut - t_sh))

        v = 5.0 * 10000.0 / (r + 10000.0)
        raw = int(round(v / 0.000150 - 10000.0))
        v_q = (raw + 10000.0) * 0.000150
        r_q = 10000.0 * (5.0 - v_q) / v_q
        t_q = lut_temperature_c(r_q, rows)
        max_raw_round_trip = max(max_raw_round_trip, abs(t_q - t))

    return max_lut_sh, max_raw_round_trip


def generate_manifest(
    path: Path,
    csv_path: Path,
    csv_hash: str,
    datasheet_path: Path | None,
    datasheet_hash: str | None,
    part: str,
    r25: float,
    b2585: float,
    coeffs: Coefficients,
    rows: list[Row],
    max_combined_component_error_c: float,
    max_pull_only_error_c: float,
    max_combined_component_error_at_c: float,
) -> None:
    trunc_errors = [truncated_temperature_c(r.rnom_ohm, r25, coeffs) - r.temperature_c for r in rows]
    sh_errors = [sh_temperature_c(r.rnom_ohm, r25, coeffs) - r.temperature_c for r in rows]
    trunc_max_idx = max(range(len(rows)), key=lambda i: abs(trunc_errors[i]))
    sh_max_idx = max(range(len(rows)), key=lambda i: abs(sh_errors[i]))
    dense_lut_sh, dense_raw = dense_validation(rows, r25, coeffs)

    raw_zero_v = (0.0 + 10000.0) * 0.000150
    raw_zero_r = 10000.0 * (5.0 - raw_zero_v) / raw_zero_v
    raw_zero_t_lut = lut_temperature_c(raw_zero_r, rows)
    raw_zero_t_sh = sh_temperature_c(raw_zero_r, r25, coeffs)

    lines = [
        "# DER26 Thermistor Model Manifest and Validation",
        "",
        "## Source identity",
        "",
        f"- Part: `{part}`",
        f"- Vishay CSV: `{csv_path.name}`",
        f"- CSV SHA-256: `{csv_hash}`",
        f"- Datasheet: `{datasheet_path.name if datasheet_path else 'not supplied'}`",
        f"- Datasheet SHA-256: `{datasheet_hash or 'n/a'}`",
        f"- R25: `{r25:.0f} ohm`",
        f"- B25/85: `{b2585:.0f} K`",
        f"- Table: `{len(rows)} points`, `{rows[0].temperature_c:.1f} C` to `{rows[-1].temperature_c:.1f} C`, `{EXPECTED_STEP_C:.1f} C` spacing",
        "",
        "## Extended Steinhart-Hart coefficients",
        "",
        "Forward `T -> R`:",
        "",
        "```text",
        "R = R25 * exp(A + B/Tk + C/Tk^2 + D/Tk^3)",
        f"A = {coeffs.a:.11f}",
        f"B = {coeffs.b:.11f}",
        f"C = {coeffs.c:.11f}",
        f"D = {coeffs.d:.11f}",
        "```",
        "",
        "Inverse `R -> T`:",
        "",
        "```text",
        "L = ln(R/R25)",
        "T = 1/(A1 + B1*L + C1*L^2 + D1*L^3) - 273.15",
        f"A1 = {coeffs.a1:.18g}",
        f"B1 = {coeffs.b1:.18g}",
        f"C1 = {coeffs.c1:.18g}",
        f"D1 = {coeffs.d1:.18g}",
        "```",
        "",
        "## Numerical results",
        "",
        f"- Current truncated A1+B1 implementation maximum nominal table error: `{abs(trunc_errors[trunc_max_idx]):.6f} C` at `{rows[trunc_max_idx].temperature_c:.1f} C`.",
        f"- Current truncated implementation RMS nominal table error: `{math.sqrt(fmean([x*x for x in trunc_errors])):.6f} C`.",
        f"- Full Vishay inverse equation maximum difference from rounded CSV table: `{abs(sh_errors[sh_max_idx]):.6f} C` at `{rows[sh_max_idx].temperature_c:.1f} C`.",
        f"- Full Vishay inverse equation RMS difference from rounded CSV table: `{math.sqrt(fmean([x*x for x in sh_errors])):.6f} C`.",
        f"- Dense LUT-versus-full-equation maximum difference: `{dense_lut_sh:.6f} C`.",
        f"- Dense 150-uV ADC-code round-trip maximum difference: `{dense_raw:.6f} C`.",
        f"- Vishay CSV maximum listed thermistor tolerance contribution: `+/-{max(r.delta_t_c for r in rows):.2f} C` within the exported range.",
        "",
        "## Important raw-code finding",
        "",
        f"ADBMS raw code `0` maps to `{raw_zero_v:.6f} V`, `{raw_zero_r:.3f} ohm`, production-LUT temperature `{raw_zero_t_lut:.6f} C`, and extended-equation temperature `{raw_zero_t_sh:.6f} C`. It is therefore a valid physical code and must not be rejected merely because its numeric value is zero. Only the documented reset/clear sentinels `0xFFFF` and `0x8000` are rejected by the new shared model.",
        "",
        "## Production selection",
        "",
        "- Primary runtime conversion: manufacturer LUT with binary search and linear interpolation.",
        "- Independent reference: full Vishay extended Steinhart-Hart inverse equation.",
        "- HIL inverse: full Vishay forward equation.",
        "- Electrically valid values beyond the exported LUT range are clamped to the nearest table endpoint and flagged `CLAMPED_COLD` or `CLAMPED_HOT`; the estimator rejects clamped values while the safety path receives a conservative extreme temperature.",
        "",
        "## Board-component tolerance envelope",
        "",
        f"- Populated pull-down part: `Panasonic EXB38V103JV`, nominal `10 kohm`, resistance tolerance `+/-{PULLDOWN_TOLERANCE_PERCENT:.1f}%`.",
        f"- Pull-down tolerance alone produces up to approximately `{max_pull_only_error_c:.3f} C` nominal decode error in the exported range.",
        f"- Vishay CSV Rmin/Rmax combined with pull-down tolerance produces a worst-corner nominal decode envelope of approximately `+/-{max_combined_component_error_c:.3f} C`; the largest absolute corner occurs near `{max_combined_component_error_at_c:.1f} C`.",
        "- This is a deterministic component-corner calculation, not a complete statistical uncertainty model.",
        "- Official Panasonic part page: https://industrial.panasonic.com/ww/products/pt/resistor-network-array/models/EXB38V103JV",
        "",
        "## Remaining physical validation",
        "",
        "The runtime conversion uses nominal component values. The separate corner analysis includes the Vishay thermistor Rmin/Rmax and +/-5% pull-down resistance, but it still excludes VREG error, pull-down TCR over actual board temperature, ADBMS AUX total measurement error, mux leakage/on resistance, harness resistance, thermistor mounting/contact error, cell-to-sensor thermal lag, and pack gradients. Complete resistance-substitution and thermal-chamber validation remain required.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--datasheet", type=Path)
    parser.add_argument("--model-header", required=True, type=Path)
    parser.add_argument("--lut-header", required=True, type=Path)
    parser.add_argument("--comparison", required=True, type=Path)
    parser.add_argument("--tolerance-analysis", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    args = parser.parse_args()

    part, r25, b2585, coeffs, rows = parse_csv(args.csv)
    csv_hash = sha256(args.csv)
    datasheet_hash = sha256(args.datasheet) if args.datasheet and args.datasheet.exists() else None

    args.model_header.parent.mkdir(parents=True, exist_ok=True)
    args.lut_header.parent.mkdir(parents=True, exist_ok=True)
    args.comparison.parent.mkdir(parents=True, exist_ok=True)
    args.tolerance_analysis.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.parent.mkdir(parents=True, exist_ok=True)

    generate_model_header(args.model_header, csv_hash, part, r25, b2585, coeffs)
    generate_lut_header(args.lut_header, csv_hash, part, rows)
    write_comparison(args.comparison, rows, r25, coeffs)
    max_combined, max_pull_only, max_combined_at = write_board_tolerance_analysis(
        args.tolerance_analysis, rows
    )
    generate_manifest(
        args.manifest,
        args.csv,
        csv_hash,
        args.datasheet,
        datasheet_hash,
        part,
        r25,
        b2585,
        coeffs,
        rows,
        max_combined,
        max_pull_only,
        max_combined_at,
    )

    print(f"Generated {args.model_header}")
    print(f"Generated {args.lut_header}")
    print(f"Generated {args.comparison}")
    print(f"Generated {args.tolerance_analysis}")
    print(f"Generated {args.manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
