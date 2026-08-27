#!/usr/bin/env python3
"""Compile, parity-check, and optionally promote a staged generated plant."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import subprocess
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
SIMULINK_ROOT = SCRIPT_DIR.parent
HIL_ROOT = SIMULINK_ROOT.parent
REPO_ROOT = HIL_ROOT.parent
PARITY_RUNNER = HIL_ROOT / "esp32_plant" / "tests" / "parity_runner.c"

SCALAR_COLUMNS = {
    "V_pack",
    "T_core",
    "T_surf",
    "SoC_true",
    "V_min",
    "V_max",
    "T_max",
    "T_avg",
}
ARRAY_PREFIXES = ("V_group_", "V_segment_", "T_sensor_", "SoC_group_")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    require(isinstance(value, dict), f"{path} must contain one JSON object")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def component_hash(component: Path) -> str:
    records: list[dict[str, str]] = []
    for path in sorted(item for item in component.rglob("*") if item.is_file()):
        records.append(
            {
                "path": path.relative_to(component).as_posix(),
                "sha256": sha256(path),
            }
        )
    require(records, f"staged component is empty: {component}")
    payload = json.dumps(records, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def contained_path(raw: str, root: Path, description: str) -> Path:
    path = Path(raw)
    if not path.is_absolute():
        path = root / path
    path = path.resolve()
    require(
        path == root or root in path.parents,
        f"{description} must belong to artifact root {root}: {path}",
    )
    return path


def safe_relative_path(raw: str, description: str) -> Path:
    path = Path(raw)
    require(
        not path.is_absolute() and ".." not in path.parts,
        f"unsafe {description}: {raw}",
    )
    return path


def verify_identity(
    candidate: dict[str, Any],
    expected: dict[str, str],
    description: str,
) -> None:
    for field in (
        "model_name",
        "configuration_hash",
        "parameter_hash",
        "cell_configuration_hash",
        "pack_configuration_hash",
        "simulation_configuration_hash",
        "model_manifest_sha256",
    ):
        require(
            candidate.get(field) == expected[field],
            (
                f"{description} {field} mismatch: "
                f"{candidate.get(field)!r} != {expected[field]!r}"
            ),
        )


def header_macro(header: str, name: str) -> str:
    match = re.search(
        rf"#define\s+{re.escape(name)}\s+(?:\\\s*)?\"([^\"]+)\"",
        header,
        flags=re.MULTILINE,
    )
    require(match is not None, f"component header is missing {name}")
    return match.group(1)


def verify_component_manifest(
    component: Path,
    manifest_file: Path,
    expected_identity: dict[str, str],
) -> dict[str, Any]:
    require(
        manifest_file.parent == component,
        "component manifest must be inside the staged component",
    )
    manifest = load_json(manifest_file)
    require(manifest.get("schema_version") == 2, "component manifest schema must be 2")
    verify_identity(manifest, expected_identity, "component manifest")
    records = manifest.get("file_records")
    require(isinstance(records, list) and records, "component file_records are missing")
    recorded: set[str] = set()
    for record in records:
        require(isinstance(record, dict), "component file record must be an object")
        relative = safe_relative_path(
            str(record.get("path", "")), "component file record"
        )
        name = relative.as_posix()
        require(name not in recorded, f"duplicate component file record: {name}")
        path = component / relative
        require(path.is_file(), f"recorded component file is missing: {path}")
        require(
            sha256(path) == record.get("sha256"),
            f"component file hash mismatch: {name}",
        )
        recorded.add(name)
    actual = {
        path.relative_to(component).as_posix()
        for path in component.rglob("*")
        if path.is_file() and path.resolve() != manifest_file
    }
    require(
        actual == recorded,
        (
            "component file inventory mismatch: "
            f"unrecorded={sorted(actual - recorded)} "
            f"missing={sorted(recorded - actual)}"
        ),
    )

    header_file = component / "plant_model_manifest.h"
    require(header_file.is_file(), f"missing component identity header: {header_file}")
    header = header_file.read_text(encoding="utf-8")
    require(
        header_macro(header, "PLANT_MODEL_NAME") == expected_identity["model_name"],
        "component header model name does not match artifact",
    )
    require(
        header_macro(header, "PLANT_MODEL_PARAMETER_HASH")
        == expected_identity["parameter_hash"],
        "component header parameter hash does not match artifact",
    )
    require(
        header_macro(header, "PLANT_MODEL_CONFIGURATION_HASH")
        == expected_identity["configuration_hash"],
        "component header configuration hash does not match artifact",
    )
    model_source = component / f"{expected_identity['model_name']}.c"
    require(model_source.is_file(), f"generated model source is missing: {model_source}")
    require(
        header_macro(header, "PLANT_MODEL_SOURCE_SHA256") == sha256(model_source),
        "component header source hash does not match generated model source",
    )
    return {
        "manifest_file": str(manifest_file),
        "manifest_sha256": sha256(manifest_file),
        "file_count": len(recorded),
        "component_sha256": component_hash(component),
    }


def verify_source_manifest(
    source_manifest_file: Path,
    expected_sha256: str,
) -> dict[str, Any]:
    require(source_manifest_file.is_file(), "source manifest is missing")
    require(
        sha256(source_manifest_file) == expected_sha256,
        "source manifest hash does not match codegen artifact",
    )
    manifest = load_json(source_manifest_file)
    require(manifest.get("schema_version") == 1, "source manifest schema must be 1")
    for field in (
        "base_repository_commit",
        "patch_review_revision",
        "reviewed_input_archive_sha256",
        "toolchain_status",
        "file_records",
    ):
        require(field in manifest, f"source manifest is missing {field}")
    require(
        re.fullmatch(r"[0-9a-f]{40}", str(manifest["base_repository_commit"]))
        is not None,
        "source manifest base_repository_commit must be a full Git SHA",
    )
    require(
        manifest["reviewed_input_archive_sha256"]
        == "670167b4963d845f4dd812008fa9d0074a4d2d1943e45fa6c55e54fc54be883a",
        "source manifest does not identify the reviewed completion archive",
    )
    records = manifest["file_records"]
    require(isinstance(records, list) and records, "source manifest has no file records")
    seen: set[str] = set()
    for record in records:
        require(isinstance(record, dict), "source file record must be an object")
        relative = safe_relative_path(str(record.get("path", "")), "source file record")
        name = relative.as_posix()
        require(name not in seen, f"duplicate source file record: {name}")
        path = REPO_ROOT / relative
        require(path.is_file(), f"recorded source file is missing: {path}")
        require(
            sha256(path) == record.get("sha256"),
            f"source file hash mismatch; regenerate SOURCE_MANIFEST.json: {name}",
        )
        seen.add(name)
    return {
        "manifest_file": str(source_manifest_file),
        "manifest_sha256": expected_sha256,
        "base_repository_commit": manifest["base_repository_commit"],
        "patch_review_revision": manifest["patch_review_revision"],
        "reviewed_input_archive_sha256": manifest[
            "reviewed_input_archive_sha256"
        ],
        "file_count": len(seen),
        "toolchain_status": manifest["toolchain_status"],
    }


def verify_evidence_chain(
    artifact_file: Path,
    artifact: dict[str, Any],
) -> tuple[Path, Path, dict[str, Any], dict[str, Any]]:
    require(artifact.get("schema_version") == 2, "codegen artifact schema must be 2")
    artifact_root = contained_path(
        str(artifact.get("artifact_root", "")),
        artifact_file.parent.resolve(),
        "artifact_root",
    )
    require(
        artifact_root == artifact_file.parent.resolve(),
        "codegen_artifact.json must be stored directly in artifact_root",
    )
    require(
        Path(str(artifact.get("artifact_file", ""))).resolve() == artifact_file,
        "artifact_file does not identify the codegen artifact being qualified",
    )
    tool_versions = {}
    for field in ("matlab_version", "simulink_version", "simulink_coder_version"):
        value = str(artifact.get(field, "")).strip()
        require(
            value and value.lower() not in {"unknown", "not installed"},
            f"codegen artifact lacks a usable {field}",
        )
        tool_versions[field] = value
    codegen_log_file = contained_path(
        str(artifact.get("codegen_log_file", "")),
        artifact_root,
        "code-generation log",
    )
    require(codegen_log_file.is_file(), "code-generation log is missing")
    identity = artifact.get("identity")
    require(isinstance(identity, dict), "artifact identity is missing")
    expected_identity = {
        field: str(identity.get(field, ""))
        for field in (
            "model_name",
            "configuration_hash",
            "parameter_hash",
            "cell_configuration_hash",
            "pack_configuration_hash",
            "simulation_configuration_hash",
            "model_manifest_sha256",
        )
    }
    require(
        all(expected_identity.values()),
        "artifact identity contains empty required fields",
    )

    model_manifest_file = contained_path(
        str(artifact["model_manifest_file"]),
        artifact_root,
        "model manifest",
    )
    model_manifest = load_json(model_manifest_file)
    require(
        sha256(model_manifest_file) == expected_identity["model_manifest_sha256"],
        "model manifest hash does not match artifact identity",
    )
    require(
        model_manifest.get("model_name") == expected_identity["model_name"],
        "model manifest name mismatch",
    )
    require(
        model_manifest.get("configuration_hash")
        == expected_identity["configuration_hash"],
        "model manifest configuration hash mismatch",
    )
    require(
        model_manifest.get("parameter_hash") == expected_identity["parameter_hash"],
        "model manifest parameter hash mismatch",
    )
    for name in (
        "cell_configuration_hash",
        "pack_configuration_hash",
        "simulation_configuration_hash",
    ):
        require(
            model_manifest.get(name) == expected_identity[name],
            f"model manifest {name} mismatch",
        )
    artifact_model = artifact.get("model")
    require(isinstance(artifact_model, dict), "artifact model object is missing")
    require(
        artifact_model.get("model_name") == expected_identity["model_name"]
        and artifact_model.get("configuration_hash")
        == expected_identity["configuration_hash"]
        and artifact_model.get("parameter_hash")
        == expected_identity["parameter_hash"]
        and artifact_model.get("cell_configuration_hash")
        == expected_identity["cell_configuration_hash"]
        and artifact_model.get("pack_configuration_hash")
        == expected_identity["pack_configuration_hash"]
        and artifact_model.get("simulation_configuration_hash")
        == expected_identity["simulation_configuration_hash"],
        "artifact model identity does not agree with the top-level identity",
    )
    require(
        Path(str(artifact_model.get("manifest_file", ""))).resolve()
        == model_manifest_file,
        "artifact model manifest path mismatch",
    )

    prevalidation_file = contained_path(
        str(artifact["pre_codegen_validation_file"]),
        artifact_root,
        "pre-codegen validation",
    )
    prevalidation = load_json(prevalidation_file)
    require(
        prevalidation.get("passed") is True
        and prevalidation.get("status") == "PASS",
        "pre-codegen validation is missing or does not say PASS",
    )
    dataset_holdout = prevalidation.get("dataset_electrical_holdout")
    require(
        isinstance(dataset_holdout, dict)
        and dataset_holdout.get("passed") is True
        and dataset_holdout.get("status") == "PASS"
        and int(dataset_holdout.get("test_count", 0)) > 0,
        "pre-codegen independent electrical holdout is missing or not PASS",
    )
    require(
        prevalidation.get("configuration_hash")
        == expected_identity["configuration_hash"],
        "pre-codegen configuration hash mismatch",
    )
    require(
        prevalidation.get("parameter_hash") == expected_identity["parameter_hash"],
        "pre-codegen parameter hash mismatch",
    )
    prevalidation_identity = prevalidation.get("identity")
    require(
        isinstance(prevalidation_identity, dict),
        "pre-codegen validation identity is missing",
    )
    verify_identity(
        prevalidation_identity, expected_identity, "pre-codegen validation"
    )

    component = contained_path(
        str(artifact["esp32_component_directory"]),
        artifact_root,
        "staged component",
    )
    require(component.is_dir(), f"staged component not found: {component}")
    component_manifest_file = contained_path(
        str(artifact["component_manifest_file"]),
        artifact_root,
        "component manifest",
    )
    component_evidence = verify_component_manifest(
        component, component_manifest_file, expected_identity
    )
    component_manifest = load_json(component_manifest_file)
    component_model = component_manifest.get("model_artifact")
    require(isinstance(component_model, dict), "component model artifact is missing")
    require(
        component_model.get("model_name") == expected_identity["model_name"]
        and component_model.get("configuration_hash")
        == expected_identity["configuration_hash"]
        and component_model.get("parameter_hash")
        == expected_identity["parameter_hash"]
        and component_model.get("cell_configuration_hash")
        == expected_identity["cell_configuration_hash"]
        and component_model.get("pack_configuration_hash")
        == expected_identity["pack_configuration_hash"]
        and component_model.get("simulation_configuration_hash")
        == expected_identity["simulation_configuration_hash"],
        "component model artifact identity mismatch",
    )
    require(
        component_manifest.get("pack_configuration")
        == model_manifest.get("pack_configuration"),
        "component and model pack configurations differ",
    )

    parity_files = artifact.get("parity_files")
    require(isinstance(parity_files, dict), "artifact has no parity_files object")
    case_file = contained_path(
        str(parity_files["manifest"]), artifact_root, "parity manifest"
    )
    case = load_json(case_file)
    require(case.get("schema_version") == 2, "parity case schema must be 2")
    require(
        case.get("reference_origin") == "configured_simulink_output",
        "parity reference must come from configured Simulink output",
    )
    case_identity = case.get("identity")
    require(isinstance(case_identity, dict), "parity identity is missing")
    verify_identity(case_identity, expected_identity, "parity case")
    input_file = contained_path(
        str(case["input_file"]), artifact_root, "parity input"
    )
    reference_file = contained_path(
        str(case["reference_file"]), artifact_root, "parity reference"
    )
    require(input_file.is_file(), f"parity input is missing: {input_file}")
    require(reference_file.is_file(), f"parity reference is missing: {reference_file}")
    require(
        sha256(input_file) == case.get("input_sha256"),
        "parity input hash mismatch",
    )
    require(
        sha256(reference_file) == case.get("reference_sha256"),
        "parity reference hash mismatch",
    )

    source_manifest_file = contained_path(
        str(artifact["source_manifest_file"]),
        artifact_root,
        "source manifest",
    )
    source_evidence = verify_source_manifest(
        source_manifest_file, str(artifact["source_manifest_sha256"])
    )
    evidence = {
        "identity": expected_identity,
        "model_manifest_file": str(model_manifest_file),
        "model_manifest_sha256": sha256(model_manifest_file),
        "pre_codegen_validation_file": str(prevalidation_file),
        "pre_codegen_validation_sha256": sha256(prevalidation_file),
        "component": component_evidence,
        "parity_manifest_file": str(case_file),
        "parity_manifest_sha256": sha256(case_file),
        "parity_input_sha256": sha256(input_file),
        "parity_reference_sha256": sha256(reference_file),
        "source": source_evidence,
        "codegen_log_file": str(codegen_log_file),
        "codegen_log_sha256": sha256(codegen_log_file),
        "tool_versions": tool_versions,
    }
    return component, case_file, case, evidence


def source_files(component: Path) -> list[Path]:
    manifest = component / "plant_model_sources.mk"
    require(manifest.is_file(), f"missing source manifest: {manifest}")
    names = re.findall(
        r"(?<![A-Za-z0-9_./-])([A-Za-z0-9_./-]+\.c)(?![A-Za-z0-9_./-])",
        manifest.read_text(encoding="utf-8"),
    )
    require(names, f"no C sources listed in {manifest}")
    resolved: list[Path] = []
    for name in names:
        relative = Path(name)
        require(
            not relative.is_absolute() and ".." not in relative.parts,
            f"unsafe source path in {manifest}: {name}",
        )
        path = component / relative
        require(path.is_file(), f"listed generated source is missing: {path}")
        if path not in resolved:
            resolved.append(path)
    return resolved


def compiler_command() -> list[str]:
    configured = os.environ.get("CC", "").strip()
    if configured:
        command = shlex.split(configured)
        require(command, "CC did not contain an executable")
        require(shutil.which(command[0]) is not None, f"CC not found: {command[0]}")
        return command
    for candidate in ("cc", "gcc", "clang"):
        resolved = shutil.which(candidate)
        if resolved:
            return [resolved]
    raise RuntimeError("no host C compiler found (tried CC, cc, gcc, clang)")


def compile_runner(component: Path, executable: Path) -> list[str]:
    command = [
        *compiler_command(),
        "-std=c11",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{component}",
        str(PARITY_RUNNER),
        *map(str, source_files(component)),
        "-lm",
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True)
    return command


def run_candidate(
    executable: Path,
    case: dict[str, Any],
    output_file: Path,
) -> list[str]:
    input_file = Path(case["input_file"]).resolve()
    require(input_file.is_file(), f"parity input is missing: {input_file}")
    command = [
        str(executable),
        str(input_file),
        format(float(case["initial_soc"]), ".17g"),
        format(float(case["initial_temperature_C"]), ".17g"),
    ]
    with output_file.open("w", encoding="utf-8", newline="") as stream:
        subprocess.run(command, check=True, stdout=stream)
    return command


def load_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames is not None, f"{path} has no header")
        return list(reader.fieldnames), list(reader)


def compare(
    reference_file: Path,
    candidate_file: Path,
    scalar_tolerance: float,
    array_tolerance: float,
) -> dict[str, Any]:
    expected_columns, expected_rows = load_csv(reference_file)
    actual_columns, actual_rows = load_csv(candidate_file)
    require(actual_columns == expected_columns, "generated-C CSV columns changed")
    require(len(actual_rows) == len(expected_rows), "generated-C CSV row count changed")
    require(expected_rows, "parity reference has no rows")

    maxima = {name: 0.0 for name in expected_columns if name != "time_s"}
    for row_index, (expected, actual) in enumerate(
        zip(expected_rows, actual_rows, strict=True)
    ):
        for name in expected_columns:
            expected_value = float(expected[name])
            actual_value = float(actual[name])
            require(
                math.isfinite(expected_value) and math.isfinite(actual_value),
                f"non-finite {name} at parity row {row_index}",
            )
            error = abs(actual_value - expected_value)
            if name == "time_s":
                tolerance = 1e-9
            elif name in SCALAR_COLUMNS:
                tolerance = scalar_tolerance
            elif name.startswith(ARRAY_PREFIXES):
                tolerance = array_tolerance
            else:
                raise RuntimeError(f"unexpected parity output column: {name}")
            if name != "time_s":
                maxima[name] = max(maxima[name], error)
            require(
                error <= tolerance,
                (
                    f"{name} parity failed at row {row_index}: "
                    f"expected={expected_value:.9g} actual={actual_value:.9g} "
                    f"error={error:.3g} tolerance={tolerance:.3g}"
                ),
            )
    return {
        "passed": True,
        "row_count": len(expected_rows),
        "scalar_tolerance": scalar_tolerance,
        "array_tolerance": array_tolerance,
        "maximum_absolute_error": maxima,
    }


def promote(component: Path, target: Path, backup_root: Path) -> Path:
    require(target.name == "plant_model", "install target must end in plant_model")
    require(target.parent.name == "components", "install target must be a component")
    require(target.is_dir(), f"qualified snapshot target is missing: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = backup_root / f"qualified_snapshot_before_{stamp}"
    require(not backup.exists(), f"backup path already exists: {backup}")
    shutil.copytree(target, backup)

    staging = target.parent / f".{target.name}.promotion-{os.getpid()}"
    require(not staging.exists(), f"promotion staging path exists: {staging}")
    shutil.copytree(component, staging)
    displaced = target.parent / f".{target.name}.previous-{os.getpid()}"
    try:
        target.rename(displaced)
        staging.rename(target)
    except BaseException:
        if not target.exists() and displaced.exists():
            displaced.rename(target)
        raise
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    if displaced.exists():
        shutil.rmtree(displaced)
    return backup


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact", type=Path)
    parser.add_argument(
        "--install-target",
        type=Path,
        help=(
            "atomically promote after parity PASS and verified staged "
            "ESP-IDF evidence"
        ),
    )
    parser.add_argument(
        "--idf-evidence",
        type=Path,
        help="PASS record produced by run_toolchain_qualification.py",
    )
    arguments = parser.parse_args()
    if arguments.install_target is not None and arguments.idf_evidence is None:
        parser.error("--install-target requires --idf-evidence")
    if arguments.idf_evidence is not None and arguments.install_target is None:
        parser.error("--idf-evidence is only valid with --install-target")

    artifact_file = arguments.artifact.resolve()
    artifact = load_json(artifact_file)
    component, case_file, case, evidence = verify_evidence_chain(
        artifact_file, artifact
    )
    reference_file = Path(case["reference_file"]).resolve()
    output_directory = case_file.parent
    output_file = output_directory / "generated_c_output.csv"
    report_file = output_directory / "generated_c_parity_report.json"

    with tempfile.TemporaryDirectory(prefix="hil-generated-parity-") as temporary:
        executable = Path(temporary) / "parity_runner"
        compile_command = compile_runner(component, executable)
        run_command = run_candidate(executable, case, output_file)

    result = compare(
        reference_file,
        output_file,
        float(case["scalar_tolerance"]),
        float(case["array_tolerance"]),
    )
    report: dict[str, Any] = {
        "schema_version": 2,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "passed": True,
        "artifact_file": str(artifact_file),
        "component_directory": str(component),
        "component_sha256": component_hash(component),
        "reference_file": str(reference_file),
        "candidate_file": str(output_file),
        "compile_command": compile_command,
        "run_command": run_command,
        "comparison": result,
        "evidence_chain": evidence,
        "installed": False,
    }

    if arguments.install_target is not None:
        idf_evidence_file = arguments.idf_evidence.resolve()
        idf_evidence = load_json(idf_evidence_file)
        require(
            idf_evidence.get("passed") is True,
            "staged ESP-IDF evidence does not say PASS",
        )
        require(
            Path(str(idf_evidence.get("artifact_file", ""))).resolve()
            == artifact_file,
            "staged ESP-IDF evidence belongs to another codegen artifact",
        )
        require(
            idf_evidence.get("artifact_sha256") == sha256(artifact_file),
            "codegen artifact changed after the staged ESP-IDF build",
        )
        require(
            idf_evidence.get("component_sha256") == component_hash(component),
            "generated component changed after the staged ESP-IDF build",
        )
        backup = promote(
            component,
            arguments.install_target.resolve(),
            output_directory,
        )
        report["installed"] = True
        report["install_target"] = str(arguments.install_target.resolve())
        report["previous_snapshot_backup"] = str(backup)
        report["idf_evidence_file"] = str(idf_evidence_file)
        report["idf_evidence_sha256"] = sha256(idf_evidence_file)
    report_file.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"PASS generated-C/Simulink parity ({result['row_count']} rows); "
        f"report={report_file}"
    )
    if report["installed"]:
        print(f"PASS promoted staged component to {report['install_target']}")
    else:
        print("Qualified snapshot unchanged (no --install-target requested)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
