#!/usr/bin/env python3
"""Run focused initialization and calibration policy characterization."""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import tempfile
from pathlib import Path


def run_one(binary: Path, trace: Path, startup: str, reset: str,
            curve_fraction: float, tau: float) -> dict[str, str]:
    with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as tmp:
        summary_path = Path(tmp.name)
    try:
        command = [
            str(binary),
            "--trace", str(trace),
            "--output", os.devnull,
            "--summary", str(summary_path),
            "--startup", startup,
            "--reset", reset,
            "--curve-time-fraction", str(curve_fraction),
            "--cooling-tau-s", str(tau),
            "--strict",
        ]
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                f"Replay failed ({completed.returncode}): {' '.join(command)}\n"
                f"{completed.stderr}"
            )
        with summary_path.open(newline="") as handle:
            row = next(csv.DictReader(handle))
        return row
    finally:
        summary_path.unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--trace-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    cases: list[tuple[Path, str, str, float, float]] = []
    for pulse_name in (
        "single_100a_0p5s.csv",
        "single_100a_1p0s.csv",
        "single_100a_1p5s.csv",
        "single_100a_3p0s.csv",
        "single_160a_20s.csv",
        "single_200a_5s.csv",
    ):
        cases.append((args.trace_dir / pulse_name, "cold-soak", "unknown", 0.25, 300.0))

    traces = [
        args.trace_dir / "single_100a_1p5s.csv",
        args.trace_dir / "repeated_corner_exit.csv",
        args.trace_dir / "synthetic_endurance.csv",
    ]
    startups = ["cold-soak", "known-cold", "seeded:0.5", "seeded:0.8"]
    for trace in traces:
        for startup in startups:
            for curve_fraction in (0.25, 0.50, 0.75):
                for tau in (120.0, 300.0, 600.0):
                    cases.append((trace, startup, "unknown", curve_fraction, tau))

    for reset_name in ("reset_pre_exhaust.csv", "reset_midrun.csv"):
        reset_trace = args.trace_dir / reset_name
        for reset in ("unknown", "known-cold", "seeded:0.5", "seeded:0.8", "restore"):
            for curve_fraction in (0.25, 0.50, 0.75):
                cases.append((reset_trace, "cold-soak", reset, curve_fraction, 300.0))

    rows: list[dict[str, str]] = []
    for index, (trace, startup, reset, curve_fraction, tau) in enumerate(cases, 1):
        row = run_one(args.binary.resolve(), trace.resolve(), startup, reset,
                      curve_fraction, tau)
        # Keep the committed result portable; the C summary receives an absolute
        # path for reliable execution but the aggregate report only needs the
        # trace filename.
        row["trace"] = trace.name
        row["case_index"] = str(index)
        rows.append(row)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["case_index"] + [k for k in rows[0] if k != "case_index"]
    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} policy cases to {args.output}")


if __name__ == "__main__":
    main()
