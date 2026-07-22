#!/usr/bin/env python3
"""Generate deterministic fuse-observer characterization traces.

The current value on each row is treated as the measurement for the interval
since the previous timestamp, matching the production observer call pattern.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Callable, Iterable

DT_S = 0.1
IDLE_S = 305.0
HEADER = [
    "timestamp_ms",
    "current_a",
    "current_uncertainty_a",
    "temperature_proxy_c",
    "measurement_valid",
    "current_calibrated",
    "current_polarity_validated",
    "temperature_measured_at_fuse",
    "model_validated",
    "event",
]


def write_trace(
    path: Path,
    duration_s: float,
    current_fn: Callable[[float], float],
    temp_fn: Callable[[float], float] | None = None,
    event_fn: Callable[[float], str] | None = None,
    validity_fn: Callable[[float], tuple[int, int, int]] | None = None,
) -> None:
    temp_fn = temp_fn or (lambda _t: 30.0)
    event_fn = event_fn or (lambda _t: "")
    validity_fn = validity_fn or (lambda _t: (1, 1, 1))
    count = int(round(duration_s / DT_S))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(HEADER)
        for index in range(count + 1):
            t = index * DT_S
            measurement_valid, calibrated, polarity = validity_fn(t)
            writer.writerow(
                [
                    int(round(t * 1000.0)),
                    f"{current_fn(t):.6f}",
                    "0.500000",
                    f"{temp_fn(t):.6f}",
                    measurement_valid,
                    calibrated,
                    polarity,
                    0,
                    1,
                    event_fn(t),
                ]
            )


def pulse_trace_at(current_a: float, duration: float) -> Callable[[float], float]:
    def current(t: float) -> float:
        if t < IDLE_S:
            return 0.0
        if t < IDLE_S + duration:
            return current_a
        return 0.0

    return current


def repeated_corner_exit(t: float) -> float:
    if t < IDLE_S:
        return 0.0
    phase = (t - IDLE_S) % 5.0
    if phase < 1.5:
        return 100.0
    if phase < 2.0:
        return -25.0  # regenerative braking still heats the fuse
    return 20.0


def autocross_current(t: float) -> float:
    if t < IDLE_S:
        return 0.0
    x = t - IDLE_S
    phase = x % 12.0
    if phase < 1.2:
        return 110.0
    if phase < 2.2:
        return 85.0
    if phase < 3.2:
        return -35.0
    if phase < 5.0:
        return 30.0
    if phase < 6.0:
        return 100.0
    if phase < 7.0:
        return 70.0
    if phase < 8.0:
        return -20.0
    return 15.0 + 10.0 * math.sin(2.0 * math.pi * phase / 12.0)


def endurance_current(t: float) -> float:
    if t < IDLE_S:
        return 0.0
    x = t - IDLE_S
    phase = x % 20.0
    if phase < 2.0:
        return 95.0
    if phase < 5.0:
        return 68.0
    if phase < 6.0:
        return -25.0
    if phase < 12.0:
        return 55.0
    if phase < 14.0:
        return 82.0
    if phase < 15.0:
        return -15.0
    return 42.0


def endurance_temp(t: float) -> float:
    if t < IDLE_S:
        return 25.0
    return min(55.0, 25.0 + 0.018 * (t - IDLE_S))


def reset_current(t: float) -> float:
    if t < IDLE_S:
        return 0.0
    phase = (t - IDLE_S) % 8.0
    return 92.0 if phase < 1.0 else 45.0


def reset_event(t: float) -> str:
    return "reset" if abs(t - 335.0) < 0.01 else ""


def reset_pre_exhaust_event(t: float) -> str:
    return "reset" if abs(t - 305.2) < 0.01 else ""


def invalid_current(t: float) -> float:
    if t < IDLE_S:
        return 0.0
    return 75.0


def invalid_flags(t: float) -> tuple[int, int, int]:
    if 315.0 <= t < 315.2:
        return (0, 1, 1)
    if 325.0 <= t < 325.2:
        return (1, 0, 1)
    if 335.0 <= t < 335.2:
        return (1, 1, 0)
    return (1, 1, 1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=Path("traces"))
    args = parser.parse_args()
    out = args.output_dir

    for duration, name in [
        (0.5, "single_100a_0p5s.csv"),
        (1.0, "single_100a_1p0s.csv"),
        (1.5, "single_100a_1p5s.csv"),
        (3.0, "single_100a_3p0s.csv"),
    ]:
        write_trace(out / name, 1000.0, pulse_trace_at(100.0, duration))


    # Curve-domain stress cases.  These deliberately exceed the vehicle's
    # static current ceilings and exist only to exercise the preliminary fuse
    # curve model and oracle.
    write_trace(out / "single_160a_20s.csv", 700.0,
                pulse_trace_at(160.0, 20.0))
    write_trace(out / "single_200a_5s.csv", 700.0,
                pulse_trace_at(200.0, 5.0))

    write_trace(out / "repeated_corner_exit.csv", 600.0, repeated_corner_exit)
    write_trace(out / "synthetic_autocross.csv", 360.0, autocross_current)
    write_trace(
        out / "synthetic_endurance.csv",
        1500.0,
        endurance_current,
        endurance_temp,
    )
    write_trace(
        out / "reset_midrun.csv",
        480.0,
        reset_current,
        event_fn=reset_event,
    )
    write_trace(
        out / "reset_pre_exhaust.csv",
        480.0,
        reset_current,
        event_fn=reset_pre_exhaust_event,
    )
    write_trace(
        out / "invalid_samples.csv",
        360.0,
        invalid_current,
        validity_fn=invalid_flags,
    )
    print(f"Generated synthetic traces in {out}")


if __name__ == "__main__":
    main()
