#!/usr/bin/env python3
"""Compare two host-runner CSV files against the frozen generated-C oracle."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

KEY_COLUMNS = ("scenario", "step")
NUMERIC_TOLERANCE = {
    "time_s": 1e-9,
    "I_pack_A": 1e-7,
    "V_pack_V": 5e-5,
    "SoC_true": 5e-7,
    "T_core_C": 5e-5,
    "T_surf_C": 5e-5,
    "V_min_V": 5e-6,
    "V_max_V": 5e-6,
    "T_max_C": 5e-5,
    "T_avg_C": 5e-5,
    "Vp1_V": 5e-7,
    "Vp2_V": 5e-7,
    "group_sum_error_V": 5e-5,
    "segment_sum_error_V": 1e-4,
    "T_ambient_C": 1e-7,
}


def load(path: Path) -> tuple[list[str], dict[tuple[str, int], dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header")
        rows: dict[tuple[str, int], dict[str, str]] = {}
        for row in reader:
            key = (row["scenario"], int(row["step"]))
            if key in rows:
                raise ValueError(f"{path} contains duplicate row {key}")
            rows[key] = row
        return reader.fieldnames, rows


def compare(baseline: Path, candidate: Path) -> None:
    baseline_fields, baseline_rows = load(baseline)
    candidate_fields, candidate_rows = load(candidate)
    if baseline_fields != candidate_fields:
        raise AssertionError(
            f"CSV columns changed:\nexpected={baseline_fields}\nactual={candidate_fields}"
        )
    if baseline_rows.keys() != candidate_rows.keys():
        missing = sorted(baseline_rows.keys() - candidate_rows.keys())
        extra = sorted(candidate_rows.keys() - baseline_rows.keys())
        raise AssertionError(f"row keys changed: missing={missing[:5]} extra={extra[:5]}")

    maxima = {name: 0.0 for name in NUMERIC_TOLERANCE}
    for key, expected in baseline_rows.items():
        actual = candidate_rows[key]
        for name, tolerance in NUMERIC_TOLERANCE.items():
            expected_value = float(expected[name])
            actual_value = float(actual[name])
            if not (math.isfinite(expected_value) and math.isfinite(actual_value)):
                raise AssertionError(f"non-finite {name} at {key}")
            error = abs(actual_value - expected_value)
            maxima[name] = max(maxima[name], error)
            if error > tolerance:
                raise AssertionError(
                    f"{name} changed at {key}: expected={expected_value:.9g} "
                    f"actual={actual_value:.9g} error={error:.3g} "
                    f"tolerance={tolerance:.3g}"
                )
    summary = ", ".join(f"{name}={value:.3g}" for name, value in maxima.items())
    print(f"PASS baseline parity ({len(baseline_rows)} rows): {summary}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    arguments = parser.parse_args()
    compare(arguments.baseline, arguments.candidate)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
