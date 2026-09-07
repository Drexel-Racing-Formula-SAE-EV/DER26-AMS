#!/usr/bin/env python3
"""Regression checks for external drive-cycle/profile assets used by core MiL."""
from __future__ import annotations

import csv
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "HiL" / "esp32_plant" / "main" / "drive_profiles.h"
EXPECTED = {
    "udds25_i_10ma": (14001, 2, 109),
    "us06_25_i_10ma": (6001, 2, 15),
    "la92_25_i_10ma": (14351, 2, 14),
}


def parse_array(text: str, name: str) -> list[int]:
    decl = re.search(
        rf"static\s+const\s+int16_t\s+{re.escape(name)}\s*\[\s*\]\s*=\s*\{{",
        text,
    )
    assert decl, f"missing {name} declaration"
    close = text.find("};", decl.end())
    assert close >= 0, f"unterminated {name}"
    body = text[decl.end():close]
    body = re.sub(r"/\*[\s\S]*?\*/", " ", body)
    body = re.sub(r"//[^\r\n]*", " ", body)
    vals = [int(x) for x in re.findall(r"[-+]?\d+", body)]
    assert vals, f"empty {name}"
    assert all(-32768 <= x <= 32767 for x in vals), f"{name} outside int16 range"
    return vals


def check_drive_profiles() -> None:
    text = HEADER.read_text(encoding="utf-8")
    for name, (count, first, last) in EXPECTED.items():
        vals = parse_array(text, name)
        assert len(vals) == count, f"{name}: expected {count}, got {len(vals)}"
        assert vals[0] == first and vals[-1] == last, (
            f"{name}: endpoint mismatch {vals[0]}, {vals[-1]}"
        )


def check_fuse_csv() -> None:
    path = ROOT / "Tools" / "fuse_replay" / "traces" / "synthetic_autocross.csv"
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) >= 2, "synthetic_autocross.csv too short"
    required = {"timestamp_ms", "current_a", "temperature_proxy_c"}
    assert required <= set(rows[0]), "synthetic_autocross.csv missing core columns"
    t = [float(r["timestamp_ms"]) for r in rows]
    assert all(b > a for a, b in zip(t, t[1:])), "CSV timestamps not strictly increasing"
    for r in rows:
        float(r["current_a"])
        float(r["temperature_proxy_c"])


def check_matlab_parser_contract() -> None:
    loader = (ROOT / "HiL" / "simulink" / "profiles" / "load_current_profile.m").read_text()
    parser = (ROOT / "HiL" / "simulink" / "+hil" / "parse_int16_c_array.m").read_text()
    assert "hil.parse_int16_c_array" in loader
    assert "(.*?)" not in parser, "do not regress to a giant lazy multiline capture"
    assert "sscanf" in parser
    assert "CArrayParseMismatch" in parser


def check_core_preflight_contract() -> None:
    runner = (ROOT / "MiL" / "matlab" / "+mil" / "run_core_campaign.m").read_text()
    core_preflight = (ROOT / "MiL" / "matlab" / "+mil" / "preflight_core_campaign.m").read_text()
    preflight = (ROOT / "MiL" / "matlab" / "+mil" / "preflight_campaign.m").read_text()
    tier_runner = (ROOT / "MiL" / "matlab" / "+mil" / "run_tier.m").read_text()
    assert "mil.preflight_core_campaign" in runner
    assert "mil.core_campaign()" in core_preflight
    assert "mil.preflight_campaign" in core_preflight
    assert "mil.resolve_profile" in preflight
    assert "ProfileSampleTime" in preflight
    assert "EstimatorAcquisitionUnobservable" in preflight
    assert "SohCapacityUnobservable" in preflight
    assert "SohResistanceUnobservable" in preflight
    assert "mil.preflight_campaign" in tier_runner


if __name__ == "__main__":
    check_drive_profiles()
    check_fuse_csv()
    check_matlab_parser_contract()
    check_core_preflight_contract()
    print("PASS: core profile assets and embedded C-array parser contract")
