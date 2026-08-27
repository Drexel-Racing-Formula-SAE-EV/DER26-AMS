#!/usr/bin/env python3
"""Compile and independently validate the DER26 thermistor C implementation.

This validator reads the original Vishay CSV, builds the production C module as
an isolated shared object, calls the actual exported functions through ctypes,
and emits a machine-readable report. It intentionally does not import the LUT
generator so that parser/generator mistakes are less likely to be mirrored.
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
import subprocess
import tempfile
from pathlib import Path

EXPECTED_PART = "NTCLE350E4103FHB0"
EXPECTED_ROWS = 281


class ThermistorResult(ctypes.Structure):
    _fields_ = [
        ("temperature_c", ctypes.c_float),
        ("resistance_ohm", ctypes.c_float),
        ("divider_voltage_v", ctypes.c_float),
        ("status", ctypes.c_int),
        ("valid", ctypes.c_bool),
        ("model_clamped", ctypes.c_bool),
    ]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_vishay_csv(path: Path) -> tuple[str, list[tuple[float, float]]]:
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    part = ""
    table_index = -1
    for index, line in enumerate(lines):
        fields = next(csv.reader([line]))
        if fields and fields[0].strip() == "Ordering Code":
            part = next(value.strip() for value in fields[1:] if value.strip())
        if fields and fields[0].strip() == "T [celsius]":
            table_index = index
            break

    if part != EXPECTED_PART:
        raise ValueError(f"Expected {EXPECTED_PART}, found {part!r}")
    if table_index < 0:
        raise ValueError("Vishay table header not found")

    rows: list[tuple[float, float]] = []
    reader = csv.DictReader(lines[table_index:])
    for raw in reader:
        if not raw or not raw.get("T [celsius]"):
            continue
        rows.append((float(raw["T [celsius]"]), float(raw[" Rnom [ohms]"])))

    if len(rows) != EXPECTED_ROWS:
        raise ValueError(f"Expected {EXPECTED_ROWS} rows, found {len(rows)}")
    return part, rows


def build_library(repo: Path, cc: str, output: Path) -> None:
    command = [
        cc,
        "-std=c11",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        "-fPIC",
        "-shared",
        f"-I{repo / 'AMS/Core/Inc'}",
        f"-I{repo / 'AMS/Core/Inc/ext_drivers'}",
        str(repo / "AMS/Core/Src/ext_drivers/thermistor_model.c"),
        "-lm",
        "-o",
        str(output),
    ]
    subprocess.run(command, check=True, text=True, capture_output=True)


def bind(lib_path: Path) -> ctypes.CDLL:
    lib = ctypes.CDLL(str(lib_path))
    lib.thermistor_temperature_lut_c.argtypes = [ctypes.c_float]
    lib.thermistor_temperature_lut_c.restype = ctypes.c_float
    lib.thermistor_temperature_steinhart_hart_c.argtypes = [ctypes.c_float]
    lib.thermistor_temperature_steinhart_hart_c.restype = ctypes.c_float
    lib.thermistor_resistance_from_temperature_c.argtypes = [ctypes.c_float]
    lib.thermistor_resistance_from_temperature_c.restype = ctypes.c_float
    lib.thermistor_from_adbms_raw.argtypes = [ctypes.c_int16, ctypes.c_float]
    lib.thermistor_from_adbms_raw.restype = ThermistorResult
    lib.thermistor_adbms_raw_from_temperature_c.argtypes = [
        ctypes.c_float,
        ctypes.c_float,
        ctypes.POINTER(ctypes.c_int16),
    ]
    lib.thermistor_adbms_raw_from_temperature_c.restype = ctypes.c_bool
    return lib


def validate(lib: ctypes.CDLL, rows: list[tuple[float, float]]) -> dict[str, object]:
    max_lut_table = (0.0, 0.0)
    max_sh_table = (0.0, 0.0)
    max_forward_abs = (0.0, 0.0)
    max_forward_rel = (0.0, 0.0)

    for temperature_c, resistance_ohm in rows:
        lut_c = float(lib.thermistor_temperature_lut_c(ctypes.c_float(resistance_ohm)))
        sh_c = float(lib.thermistor_temperature_steinhart_hart_c(ctypes.c_float(resistance_ohm)))
        forward_r = float(lib.thermistor_resistance_from_temperature_c(ctypes.c_float(temperature_c)))

        lut_error = abs(lut_c - temperature_c)
        sh_error = abs(sh_c - temperature_c)
        forward_abs = abs(forward_r - resistance_ohm)
        forward_rel = forward_abs / resistance_ohm

        if lut_error > max_lut_table[0]:
            max_lut_table = (lut_error, temperature_c)
        if sh_error > max_sh_table[0]:
            max_sh_table = (sh_error, temperature_c)
        if forward_abs > max_forward_abs[0]:
            max_forward_abs = (forward_abs, temperature_c)
        if forward_rel > max_forward_rel[0]:
            max_forward_rel = (forward_rel, temperature_c)

    max_dense_lut_sh = (0.0, 0.0)
    max_raw_roundtrip = (0.0, 0.0)
    for step in range(14001):
        temperature_c = -20.0 + step * 0.01
        resistance_ohm = float(
            lib.thermistor_resistance_from_temperature_c(ctypes.c_float(temperature_c))
        )
        lut_c = float(lib.thermistor_temperature_lut_c(ctypes.c_float(resistance_ohm)))
        sh_c = float(
            lib.thermistor_temperature_steinhart_hart_c(ctypes.c_float(resistance_ohm))
        )
        lut_sh_error = abs(lut_c - sh_c)
        if lut_sh_error > max_dense_lut_sh[0]:
            max_dense_lut_sh = (lut_sh_error, temperature_c)

        raw = ctypes.c_int16()
        ok = bool(
            lib.thermistor_adbms_raw_from_temperature_c(
                ctypes.c_float(temperature_c), ctypes.c_float(5.0), ctypes.byref(raw)
            )
        )
        if not ok:
            raise AssertionError(f"T->raw failed at {temperature_c:.2f} C")
        decoded = lib.thermistor_from_adbms_raw(raw, ctypes.c_float(5.0))
        if not decoded.valid:
            raise AssertionError(
                f"raw->T invalid at {temperature_c:.2f} C, raw={raw.value}, status={decoded.status}"
            )
        roundtrip_error = abs(float(decoded.temperature_c) - temperature_c)
        if roundtrip_error > max_raw_roundtrip[0]:
            max_raw_roundtrip = (roundtrip_error, temperature_c)

    raw_zero = lib.thermistor_from_adbms_raw(ctypes.c_int16(0), ctypes.c_float(5.0))
    if not raw_zero.valid or raw_zero.status != 0:
        raise AssertionError("ADBMS raw code zero was not accepted as a valid physical value")

    limits = {
        "lut_table_max_c": 0.0006,
        "extended_sh_table_max_c": 0.012,
        "dense_lut_sh_max_c": 0.014,
        "raw_roundtrip_max_c": 0.020,
        "forward_relative_max": 0.00015,
    }
    measured = {
        "lut_table_max_c": max_lut_table[0],
        "lut_table_max_at_c": max_lut_table[1],
        "extended_sh_table_max_c": max_sh_table[0],
        "extended_sh_table_max_at_c": max_sh_table[1],
        "forward_resistance_max_abs_ohm": max_forward_abs[0],
        "forward_resistance_max_abs_at_c": max_forward_abs[1],
        "forward_resistance_max_relative": max_forward_rel[0],
        "forward_resistance_max_relative_at_c": max_forward_rel[1],
        "dense_lut_sh_max_c": max_dense_lut_sh[0],
        "dense_lut_sh_max_at_c": max_dense_lut_sh[1],
        "raw_roundtrip_max_c": max_raw_roundtrip[0],
        "raw_roundtrip_max_at_c": max_raw_roundtrip[1],
        "raw_zero_temperature_c": float(raw_zero.temperature_c),
        "raw_zero_resistance_ohm": float(raw_zero.resistance_ohm),
        "raw_zero_voltage_v": float(raw_zero.divider_voltage_v),
    }

    pass_checks = {
        "lut_table": measured["lut_table_max_c"] <= limits["lut_table_max_c"],
        "extended_sh_table": measured["extended_sh_table_max_c"]
        <= limits["extended_sh_table_max_c"],
        "dense_lut_sh": measured["dense_lut_sh_max_c"] <= limits["dense_lut_sh_max_c"],
        "raw_roundtrip": measured["raw_roundtrip_max_c"]
        <= limits["raw_roundtrip_max_c"],
        "forward_relative": measured["forward_resistance_max_relative"]
        <= limits["forward_relative_max"],
        "raw_zero_valid": raw_zero.valid and raw_zero.status == 0,
    }

    return {
        "pass": all(pass_checks.values()),
        "checks": pass_checks,
        "limits": limits,
        "measured": measured,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cc", default="cc")
    args = parser.parse_args()

    repo = args.repo.resolve()
    csv_path = args.csv.resolve()
    part, rows = read_vishay_csv(csv_path)

    with tempfile.TemporaryDirectory(prefix="thermistor-c-validation-") as temp:
        library = Path(temp) / "libthermistor_model.so"
        build_library(repo, args.cc, library)
        report = validate(bind(library), rows)

    report.update(
        {
            "part": part,
            "csv": csv_path.name,
            "csv_sha256": sha256(csv_path),
            "row_count": len(rows),
            "compiler": args.cc,
        }
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
