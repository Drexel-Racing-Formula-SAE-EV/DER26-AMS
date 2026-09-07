#!/usr/bin/env python3
"""Audit frozen-input readiness without treating known blockers as test failures."""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

MIL = Path(__file__).resolve().parents[1]
REPO = MIL.parent
OUT = MIL / "docs" / "BASELINE_READINESS.md"


def read(path: str) -> str:
    return (REPO / path).read_text(encoding="utf-8", errors="replace")


def audit() -> list[dict[str, str]]:
    current = read("AMS/Core/Src/ext_drivers/current_sensor.c")
    ioc = read("AMS/DER26-AMS.ioc")
    build = read("AMS/Core/Inc/ams_build_profile.h")
    contract = read("AMS/docs/AMS_TUNING_CAN_SD_CONTRACT.md")
    acceptance = json.loads((MIL / "baselines" / "acceptance" / "qualification_template.json").read_text())
    checks = [
        {"item": "Invalid current-zero rejection", "status": "PASS" if
         "dev->reason != CURRENT_SENSOR_REASON_OK" in current and "!dev->current_valid" in current else "BLOCKED",
         "evidence": "AMS/Core/Src/ext_drivers/current_sensor.c"},
        {"item": "AMS source/CubeMX nominal CAN bitrate is 1 Mbit/s", "status": "PASS" if
         re.search(r"CAN1\.CalculateBaudRate=1000000(?:\D|$)", ioc) else "BLOCKED",
         "evidence": "AMS/DER26-AMS.ioc"},
        {"item": "CAN manifest declares 1M", "status": "PASS" if
         re.search(r"DER26-CAN-[^\"\n]*1M", build) else "BLOCKED",
         "evidence": "AMS/Core/Inc/ams_build_profile.h"},
        {"item": "Tuning contract declares 1 Mbit/s image", "status": "PASS" if
         "1 Mbit/s image" in contract else "BLOCKED",
         "evidence": "AMS/docs/AMS_TUNING_CAN_SD_CONTRACT.md"},
        {"item": "Exact ECU CAN binary-record source imported", "status": "PASS" if
         any(REPO.glob("**/*can*log*record*.h")) else "BLOCKED",
         "evidence": "Required for byte-exact CAN###.BIN generation"},
        {"item": "Qualification acceptance baseline frozen", "status": "PASS" if
         acceptance.get("status") == "FROZEN" else "BLOCKED",
         "evidence": "MiL/baselines/acceptance/qualification_template.json"},
    ]
    return checks


def render() -> str:
    checks = audit()
    lines = [
        "# DER26 AMS MiL Baseline Readiness",
        "",
        "This audit separates implemented MiL infrastructure from external/frozen-input blockers.",
        "A `BLOCKED` item is not a failed host regression; it prevents release-grade qualification evidence.",
        "",
        "| Item | Status | Evidence |",
        "|---|---|---|",
    ]
    for check in checks:
        lines.append(f"| {check['item']} | **{check['status']}** | `{check['evidence']}` |")
    lines.extend(["", f"Implemented: {sum(c['status']=='PASS' for c in checks)}/{len(checks)} frozen-input gates.", ""])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    expected = render()
    if args.check:
        if not OUT.is_file() or OUT.read_text(encoding="utf-8") != expected:
            print("baseline readiness report is stale", file=sys.stderr)
            return 1
        print("DER26 MiL baseline readiness report: PASS (report current)")
        return 0
    OUT.write_text(expected, encoding="utf-8")
    print(f"wrote {OUT.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
