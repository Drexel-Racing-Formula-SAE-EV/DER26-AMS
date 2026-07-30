#!/usr/bin/env python3
"""Regenerate the checked-in HIL source/provenance manifest."""

from __future__ import annotations

import hashlib
import json
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


HIL_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = HIL_ROOT.parent
OUTPUT = HIL_ROOT / "SOURCE_MANIFEST.json"
BASE_COMMIT = "82c8607798b7d17726b52dcd96e6bdacd402559a"
INPUT_ARCHIVE_SHA256 = (
    "670167b4963d845f4dd812008fa9d0074a4d2d1943e45fa6c55e54fc54be883a"
)

IMPORTANT_FILES = [
    "AMS/Core/Inc/ext_drivers/accumulator.h",
    "AMS/Core/Src/ext_drivers/accumulator.c",
    "AMS/Core/Src/tasks/cli_task.c",
    "AMS/host_tests/src/ams_host_test_runner.c",
    "HiL/ATOMIC_CAN_IMAGE_PROTOCOL.md",
    "HiL/HARDWARE_QUALIFICATION_PLAN.md",
    "HiL/HIL_REFACTOR_VALIDATION_REPORT.md",
    "HiL/README.md",
    "HiL/esp32_plant/components/CAN/mcp2515_driver.c",
    "HiL/esp32_plant/components/CAN/mcp2515_driver.h",
    "HiL/esp32_plant/components/CAN/mcp2515_tx_status.h",
    "HiL/esp32_plant/main/main.c",
    "HiL/esp32_plant/main/plant_shared.h",
    "HiL/esp32_plant/README.md",
    "HiL/esp32_plant/tests/Makefile",
    "HiL/esp32_plant/tests/mcp2515_driver_host_test.c",
    "HiL/esp32_plant/tests/mcp2515_tx_status_test.c",
    "HiL/simulink/+hil/dataset_normalization_options.m",
    "HiL/simulink/+hil/configure_model.m",
    "HiL/simulink/+hil/codegen_chart_script.m",
    "HiL/simulink/+hil/export_codegen_parity_case.m",
    "HiL/simulink/+hil/export_validation_report.m",
    "HiL/simulink/+hil/fit_r1c1.m",
    "HiL/simulink/+hil/fit_r2c2.m",
    "HiL/simulink/+hil/generate_code.m",
    "HiL/simulink/+hil/normalize_dataset.m",
    "HiL/simulink/+hil/package_codegen_for_esp32.m",
    "HiL/simulink/+hil/validate_all.m",
    "HiL/simulink/+hil/validate_configuration.m",
    "HiL/simulink/+hil/validate_dataset.m",
    "HiL/simulink/+hil/validate_dynamic_profile.m",
    "HiL/simulink/+hil/validate_hppc.m",
    "HiL/simulink/+hil/write_esp32_interface_files.m",
    "HiL/simulink/adapters/load_p42a_hcgt.m",
    "HiL/simulink/configs/acceptances/p42a.m",
    "HiL/simulink/configs/cells/p42a.m",
    "HiL/simulink/configs/datasets/p42a_published_hcgt.m",
    "HiL/simulink/scripts/run_all_tests.m",
    "HiL/simulink/scripts/generate_esp32_plant.m",
    "HiL/simulink/scripts/validate_p42a_75s6p.m",
    "HiL/simulink/README.md",
    "HiL/simulink/tests/run_static_checks.py",
    "HiL/simulink/tools/qualify_generated_plant.py",
    "HiL/tools/run_toolchain_qualification.py",
    "HiL/tools/write_source_manifest.py",
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def command_version(command: list[str]) -> str:
    executable = shutil.which(command[0])
    if executable is None:
        return "unavailable"
    completed = subprocess.run(
        [executable, *command[1:]],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    lines = completed.stdout.strip().splitlines()
    return lines[0] if lines else "unknown"


def main() -> int:
    records: list[dict[str, str]] = []
    for name in IMPORTANT_FILES:
        path = REPO_ROOT / name
        if not path.is_file():
            raise RuntimeError(f"important source file is missing: {path}")
        records.append({"path": name, "sha256": sha256(path)})

    manifest = {
        "schema_version": 1,
        "manifest_scope": "HIL qualification-readiness source snapshot",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "base_repository_commit": BASE_COMMIT,
        "patch_review_revision": "hil-qualification-readiness-v3-2026-07-29",
        "reviewed_input_archive": "DER26-AMS-HIL-Completion-2026-07-29.zip",
        "reviewed_input_archive_sha256": INPUT_ARCHIVE_SHA256,
        "file_records": records,
        "toolchain_status": {
            "matlab": {
                "status": "NOT_RUN",
                "version": "unavailable in managed review environment",
            },
            "simulink": {
                "status": "NOT_RUN",
                "version": "unavailable in managed review environment",
            },
            "simulink_coder": {
                "status": "NOT_RUN",
                "version": "unavailable in managed review environment",
            },
            "esp_idf": {
                "status": "NOT_RUN",
                "version": command_version(["idf.py", "--version"]),
            },
            "hardware": {
                "status": "NOT_RUN",
                "version": "ESP32/MCP2515/AMS bench unavailable",
            },
            "host_python": {
                "status": "PASS",
                "version": sys.version.splitlines()[0],
            },
            "host_c_compiler": {
                "status": "PASS",
                "version": command_version(["cc", "--version"]),
            },
            "host_platform": {
                "status": "INFORMATIONAL",
                "version": platform.platform(),
            },
        },
    }
    OUTPUT.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote {OUTPUT} with {len(records)} source hashes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
