#!/usr/bin/env python3
"""Qualify staged generated C, build it in ESP-IDF, then optionally promote."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


HIL_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = HIL_ROOT.parent
SIMULINK_SCRIPTS = HIL_ROOT / "simulink" / "scripts"
QUALIFIER = HIL_ROOT / "simulink" / "tools" / "qualify_generated_plant.py"
ESP32_PROJECT = HIL_ROOT / "esp32_plant"
INSTALL_TARGET = ESP32_PROJECT / "components" / "plant_model"


def executable(name: str) -> str:
    resolved = shutil.which(name)
    if resolved is None:
        raise RuntimeError(f"required executable is not available: {name}")
    return resolved


def run(command: list[str], cwd: Path, environment: dict[str, str]) -> None:
    print("+", subprocess.list2cmdline(command), flush=True)
    subprocess.run(command, cwd=cwd, env=environment, check=True)


def command_version(command: list[str], cwd: Path) -> str:
    completed = subprocess.run(
        command,
        cwd=cwd,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    first_line = completed.stdout.strip().splitlines()
    return first_line[0] if first_line else "unknown"


def newest_artifact(output_root: Path) -> Path:
    candidates = list((output_root / "codegen").rglob("codegen_artifact.json"))
    if not candidates:
        raise RuntimeError(f"MATLAB produced no codegen artifact under {output_root}")
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise RuntimeError(f"{path} must contain one JSON object")
    return value


def tree_hash(root: Path) -> str:
    records: list[dict[str, str]] = []
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        records.append(
            {"path": path.relative_to(root).as_posix(), "sha256": digest}
        )
    if not records:
        raise RuntimeError(f"cannot hash empty directory: {root}")
    payload = json.dumps(records, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_commit() -> str:
    completed = subprocess.run(
        ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    return completed.stdout.strip() if completed.returncode == 0 else "unknown"


def copy_idf_project(destination: Path, component: Path) -> Path:
    def ignore(_directory: str, names: list[str]) -> set[str]:
        ignored = {
            name
            for name in names
            if name in {"build", "__pycache__", ".pytest_cache"}
            or name.endswith((".pyc", ".pyo"))
        }
        return ignored

    shutil.copytree(ESP32_PROJECT, destination, ignore=ignore)
    staged_target = destination / "components" / "plant_model"
    if staged_target.exists():
        shutil.rmtree(staged_target)
    shutil.copytree(component, staged_target)
    return staged_target


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-root",
        type=Path,
        default=HIL_ROOT / "qualification_output",
    )
    parser.add_argument(
        "--artifact",
        type=Path,
        help="reuse an existing codegen artifact and skip MATLAB",
    )
    parser.add_argument(
        "--promote",
        action="store_true",
        help=(
            "atomically replace the qualified snapshot only after staged "
            "generated-C parity and ESP-IDF build both pass"
        ),
    )
    parser.add_argument(
        "--skip-idf",
        action="store_true",
        help=(
            "stop after generated-C parity; promotion is forbidden because "
            "the staged ESP32 application was not compiled"
        ),
    )
    arguments = parser.parse_args()
    if arguments.promote and arguments.skip_idf:
        parser.error("--promote cannot be combined with --skip-idf")

    output_root = arguments.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["HIL_OUTPUT_ROOT"] = str(output_root)
    commands: list[dict[str, Any]] = []
    report: dict[str, Any] = {
        "schema_version": 2,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "passed": False,
        "repository_commit": git_commit(),
        "promoted": False,
        "idf_executed": False,
        "commands": commands,
    }
    report_file = output_root / "toolchain_qualification_report.json"

    try:
        if arguments.artifact is None:
            matlab = executable("matlab")
            matlab_command = [
                matlab,
                "-batch",
                (
                    f"cd('{str(SIMULINK_SCRIPTS).replace(chr(39), chr(39) * 2)}');"
                    "run_all_tests;validate_p42a_75s6p;generate_esp32_plant;"
                ),
            ]
            commands.append({"stage": "matlab", "command": matlab_command})
            run(matlab_command, REPO_ROOT, environment)
            artifact = newest_artifact(output_root)
        else:
            artifact = arguments.artifact.resolve()
            if not artifact.is_file():
                raise RuntimeError(f"codegen artifact does not exist: {artifact}")
        report["artifact"] = str(artifact)

        artifact_data = load_json(artifact)
        component = Path(
            str(artifact_data["esp32_component_directory"])
        ).resolve()
        if not component.is_dir():
            raise RuntimeError(f"staged component does not exist: {component}")
        component_before = tree_hash(component)

        parity_command = [sys.executable, str(QUALIFIER), str(artifact)]
        commands.append(
            {"stage": "generated_c_parity", "command": parity_command}
        )
        run(parity_command, REPO_ROOT, environment)

        if not arguments.skip_idf:
            idf = executable("idf.py")
            report["tool_versions"] = {
                "idf": command_version([idf, "--version"], REPO_ROOT),
                "python": sys.version.splitlines()[0],
            }
            with tempfile.TemporaryDirectory(
                prefix="hil-staged-idf-", dir=output_root
            ) as temporary:
                staged_project = Path(temporary) / "esp32_plant"
                staged_component = copy_idf_project(staged_project, component)
                if tree_hash(staged_component) != component_before:
                    raise RuntimeError(
                        "staged ESP-IDF component changed while being copied"
                    )
                set_target = [idf, "set-target", "esp32"]
                build = [idf, "build"]
                commands.extend(
                    [
                        {
                            "stage": "idf_set_target_staged",
                            "command": set_target,
                            "cwd": str(staged_project),
                        },
                        {
                            "stage": "idf_build_staged",
                            "command": build,
                            "cwd": str(staged_project),
                        },
                    ]
                )
                run(set_target, staged_project, environment)
                run(build, staged_project, environment)
                if tree_hash(staged_component) != component_before:
                    raise RuntimeError(
                        "ESP-IDF build modified the staged generated component"
                    )
            report["idf_executed"] = True
            report["staged_component_sha256"] = component_before
            idf_evidence_file = output_root / "staged_idf_evidence.json"
            idf_evidence = {
                "schema_version": 1,
                "generated_utc": datetime.now(timezone.utc).isoformat(),
                "passed": True,
                "artifact_file": str(artifact),
                "artifact_sha256": file_hash(artifact),
                "component_sha256": component_before,
                "idf_version": report["tool_versions"]["idf"],
                "build_order": [
                    "generated_c_parity",
                    "idf_set_target_staged",
                    "idf_build_staged",
                ],
            }
            idf_evidence_file.write_text(
                json.dumps(idf_evidence, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            report["idf_evidence_file"] = str(idf_evidence_file)
            report["idf_evidence_sha256"] = file_hash(idf_evidence_file)

        if arguments.promote:
            if not report["idf_executed"]:
                raise RuntimeError(
                    "promotion requires a successful staged ESP-IDF build"
                )
            promote_command = [
                sys.executable,
                str(QUALIFIER),
                str(artifact),
                "--install-target",
                str(INSTALL_TARGET),
                "--idf-evidence",
                str(idf_evidence_file),
            ]
            commands.append(
                {"stage": "atomic_promotion", "command": promote_command}
            )
            run(promote_command, REPO_ROOT, environment)
            report["promoted"] = True
            report["installed_component_sha256"] = tree_hash(INSTALL_TARGET)
            if report["installed_component_sha256"] != component_before:
                raise RuntimeError(
                    "promoted component hash differs from staged qualified component"
                )

        report["passed"] = True
        report_file.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"PASS toolchain qualification; report={report_file}")
        return 0
    except BaseException as error:
        report["error"] = f"{type(error).__name__}: {error}"
        report_file.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"FAIL toolchain qualification; report={report_file}", file=sys.stderr)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
