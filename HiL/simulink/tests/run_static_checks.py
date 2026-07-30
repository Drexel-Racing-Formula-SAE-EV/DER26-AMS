#!/usr/bin/env python3
"""Repository-level checks available without MATLAB or ESP-IDF."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

import numpy as np
from scipy.io import loadmat


SIMULINK_ROOT = Path(__file__).resolve().parents[1]
HIL_ROOT = SIMULINK_ROOT.parent
ESP32_ROOT = HIL_ROOT / "esp32_plant"
REPO_ROOT = HIL_ROOT.parent


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run(command: list[str], cwd: Path) -> str:
    completed = subprocess.run(
        command,
        cwd=cwd,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return completed.stdout


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def check_required_files() -> None:
    relative_files = [
        "+hil/configure_model.m",
        "+hil/dataset_normalization_options.m",
        "+hil/evaluate_validation_acceptance.m",
        "+hil/resolve_profile_inputs.m",
        "+hil/run_reference.m",
        "+hil/validate_acceptance_configuration.m",
        "+hil/validate_all.m",
        "+hil/validation_limits_at_temperature.m",
        "configs/cells/p42a.m",
        "configs/cells/candidate_template.m",
        "configs/acceptances/p42a.m",
        "configs/acceptances/candidate_template.m",
        "configs/packs/der26_75s6p.m",
        "configs/packs/structural_12s2p.m",
        "configs/simulations/udds_25c.m",
        "configs/simulations/la92_25c.m",
        "models/der_accumulator_2rc_thermal_plant.slx",
        "parameters/source/p42a_legacy_codegen_snapshot.mat",
        "scripts/run_all_tests.m",
        "scripts/generate_esp32_plant.m",
        "../tools/run_toolchain_qualification.py",
        "../ATOMIC_CAN_IMAGE_PROTOCOL.md",
        "../HARDWARE_QUALIFICATION_PLAN.md",
        "../SOURCE_MANIFEST.json",
        "../tools/write_source_manifest.py",
        "validation/baselines/p42a_75s6p_codegen_oracle.csv",
    ]
    missing = [
        str(path)
        for path in relative_files
        if not (SIMULINK_ROOT / path).is_file()
    ]
    require(not missing, f"missing required files: {missing}")


def check_no_machine_paths() -> None:
    forbidden = [
        re.compile(r"USERPROFILE", re.IGNORECASE),
        re.compile(r"[A-Za-z]:\\\\Users\\\\", re.IGNORECASE),
        re.compile(r"/Users/"),
        re.compile(r"Documents[/\\\\]BMS", re.IGNORECASE),
    ]
    failures: list[str] = []
    for path in SIMULINK_ROOT.rglob("*.m"):
        text = path.read_text(encoding="utf-8")
        for pattern in forbidden:
            if pattern.search(text):
                failures.append(f"{path.relative_to(SIMULINK_ROOT)}: {pattern.pattern}")
    require(not failures, f"machine-specific MATLAB paths remain: {failures}")


def check_model_archives() -> None:
    source = (
        SIMULINK_ROOT
        / "models"
        / "drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.slx"
    )
    generic = SIMULINK_ROOT / "models" / "der_accumulator_2rc_thermal_plant.slx"
    require(sha256(source) == sha256(generic), "generic template drifted from oracle")
    for path in (source, generic):
        with zipfile.ZipFile(path) as archive:
            bad = archive.testzip()
            require(bad is None, f"{path.name} has corrupt member {bad}")
            names = set(archive.namelist())
            require(
                "simulink/stateflow/chart_17.xml" in names,
                f"{path.name} lacks output-expansion chart",
            )
            root = archive.read("simulink/systems/system_root.xml").decode("utf-8")
            for block in (
                "I_pack_to_cell",
                "V_cell_to_pack",
                "SoC_Integrator",
                "Accumulator_Output_Expansion",
            ):
                require(block in root, f"{path.name} lacks block {block}")


def check_parameter_snapshot() -> None:
    manifest_path = (
        SIMULINK_ROOT
        / "parameters"
        / "manifests"
        / "p42a_legacy_codegen_snapshot.json"
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    require(manifest["dimensions"]["OCV"] == [101, 3], "OCV dimensions changed")
    for name in ("R0", "R1", "C1"):
        require(manifest["dimensions"][name] == [12, 3], f"{name} dimensions changed")
    require(manifest["scalars"]["R2_ohm"] == 0.004, "R2 changed")
    require(manifest["scalars"]["C2_F"] == 12000.0, "C2 changed")

    mat = loadmat(
        SIMULINK_ROOT
        / "parameters"
        / "source"
        / "p42a_legacy_codegen_snapshot.mat",
        squeeze_me=True,
        struct_as_record=False,
    )
    for name in ("OCV", "R0", "R1", "C1"):
        require(np.all(np.isfinite(mat[name])), f"{name} contains non-finite values")
        require(np.all(mat[name] > 0), f"{name} contains non-positive values")
    require(
        np.allclose(
            mat["neg_inv_R1C1_2d_fixed"],
            -1.0 / (mat["R1"] * mat["C1"]),
            rtol=3e-6,
            atol=1e-8,
        ),
        "R1/C1 inverse table is inconsistent",
    )


def check_frozen_hashes(source_archive: Path | None) -> None:
    baseline_manifest = json.loads(
        (
            SIMULINK_ROOT
            / "validation"
            / "baselines"
            / "p42a_75s6p_codegen_oracle_manifest.json"
        ).read_text(encoding="utf-8")
    )
    baseline = (
        SIMULINK_ROOT
        / "validation"
        / "baselines"
        / baseline_manifest["baseline_file"]
    )
    require(
        sha256(baseline) == baseline_manifest["baseline_sha256"],
        "frozen baseline hash does not match manifest",
    )
    model_source = (
        ESP32_ROOT
        / "components"
        / "plant_model"
        / "drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.c"
    )
    constants = ESP32_ROOT / "components" / "plant_model" / "const_params.c"
    require(
        sha256(model_source) == baseline_manifest["generated_model_source_sha256"],
        "generated model source changed without a new baseline",
    )
    require(
        sha256(constants) == baseline_manifest["constant_source_sha256"],
        "generated constants changed without a new baseline",
    )
    identity_header = (
        ESP32_ROOT / "components" / "plant_model" / "plant_model_manifest.h"
    ).read_text(encoding="utf-8")
    require(
        header_macro(identity_header, "PLANT_MODEL_CONFIGURATION_HASH")
        == baseline_manifest["plant_configuration_hash"],
        "frozen component configuration identity changed without promotion",
    )
    require(
        header_macro(identity_header, "PLANT_MODEL_PARAMETER_HASH")
        == baseline_manifest["parameter_configuration_hash"],
        "frozen component parameter identity changed without promotion",
    )
    if source_archive is not None:
        require(source_archive.is_file(), f"source archive not found: {source_archive}")
        require(
            sha256(source_archive) == baseline_manifest["source_archive_sha256"],
            "source archive hash does not match manifest",
        )


def check_source_manifest() -> None:
    manifest_file = HIL_ROOT / "SOURCE_MANIFEST.json"
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    require(manifest["schema_version"] == 1, "source manifest schema changed")
    require(
        manifest["base_repository_commit"]
        == "82c8607798b7d17726b52dcd96e6bdacd402559a",
        "source manifest base commit is not the reviewed repository commit",
    )
    require(
        manifest["reviewed_input_archive_sha256"]
        == "670167b4963d845f4dd812008fa9d0074a4d2d1943e45fa6c55e54fc54be883a",
        "source manifest does not identify the reviewed completion archive",
    )
    records = manifest.get("file_records")
    require(isinstance(records, list) and records, "source manifest has no hashes")
    seen: set[str] = set()
    for record in records:
        relative = Path(record["path"])
        require(
            not relative.is_absolute() and ".." not in relative.parts,
            f"unsafe source-manifest path: {relative}",
        )
        name = relative.as_posix()
        require(name not in seen, f"duplicate source-manifest record: {name}")
        path = REPO_ROOT / relative
        require(path.is_file(), f"source-manifest file is missing: {path}")
        require(
            sha256(path) == record["sha256"],
            f"source-manifest hash mismatch: {name}",
        )
        seen.add(name)
    for tool in ("matlab", "simulink", "simulink_coder", "esp_idf", "hardware"):
        require(
            tool in manifest["toolchain_status"],
            f"source manifest omits {tool} status",
        )


def check_source_contracts() -> None:
    scripts = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (SIMULINK_ROOT / "scripts").glob("*.m")
    )
    require(", .slbuild" not in scripts, "legacy malformed slbuild statement remains")
    main = (ESP32_ROOT / "main" / "main.c").read_text(encoding="utf-8")
    require(
        "drev_75s6p_p42a_accumulator" not in main,
        "application still depends on generated identifiers",
    )
    require("plant_model_adapter.h" in main, "application does not use stable adapter")
    for token in (
        "pack_image_start",
        "pack_image_commit",
        "image_crc32",
        "AMS_HIL_CAN_ID_IMAGE_CONTROL",
        "max_image_tx_us",
        "deadline_miss_count",
        "plant_shared.valid",
        "plant_step(0.0f, PROFILE_AMBIENT_C)",
    ):
        require(token in main, f"atomic ESP32 image contract lost {token}")
    require(
        "PLANT_SEGMENT_GROUP_COUNT" in main
        and "PLANT_SEGMENT_SENSOR_COUNT" in main
        and "PLANT_SEGMENT_GROUP_INDEX" in main
        and "PLANT_SEGMENT_SENSOR_INDEX" in main,
        "CAN expansion does not use the topology manifest",
    )
    cmake = (
        ESP32_ROOT / "components" / "plant_model" / "CMakeLists.txt"
    ).read_text(encoding="utf-8")
    require("plant_model_adapter.c" in cmake, "ESP-IDF component omits adapter")
    adapter_header = (
        ESP32_ROOT / "components" / "plant_model" / "plant_model_adapter.h"
    ).read_text(encoding="utf-8")
    for signature in (
        "bool plant_init(const plant_configuration_t *configuration);",
        "bool plant_reset(float initial_soc, float initial_temperature_C);",
        "bool plant_step(float pack_current_A, float ambient_temperature_C);",
        "bool plant_get_outputs(plant_output_t *output);",
    ):
        require(signature in adapter_header, f"stable adapter lost API: {signature}")
    source_list = (
        ESP32_ROOT / "components" / "plant_model" / "plant_model_sources.mk"
    ).read_text(encoding="utf-8")
    require(
        "plant_model_adapter.c" in source_list,
        "host source manifest omits stable adapter",
    )
    structural = (
        SIMULINK_ROOT / "configs" / "packs" / "structural_12s2p.m"
    ).read_text(encoding="utf-8")
    for token in ("Ns = 12", "Np = 2", "Ntemp = 18", "[4, 4, 4]"):
        require(token in structural, f"structural test lost token {token}")
    embedded_profiles = {
        "configs/simulations/us06_25c.m": "us06_25_i_10ma",
        "configs/simulations/udds_25c.m": "udds25_i_10ma",
        "configs/simulations/la92_25c.m": "la92_25_i_10ma",
    }
    for relative, array_name in embedded_profiles.items():
        profile_text = (SIMULINK_ROOT / relative).read_text(encoding="utf-8")
        require(
            array_name in profile_text,
            f"{relative} does not select embedded array {array_name}",
        )
    candidate_builder = (
        SIMULINK_ROOT / "scripts" / "build_candidate_cell.m"
    ).read_text(encoding="utf-8")
    for token in (
        "HoldoutRequired",
        "PartitionProvenance",
        "HoldoutOverlap",
        "HoldoutAccuracy",
    ):
        require(token in candidate_builder, f"candidate holdout gate lost {token}")
    generator = (
        SIMULINK_ROOT / "+hil" / "generate_code.m"
    ).read_text(encoding="utf-8")
    require(
        "UnsafeDirectInstall" in generator,
        "code generation no longer blocks unqualified snapshot replacement",
    )
    require(
        "DistributedPlantUnsupported" in generator,
        "representative-state code generation no longer blocks parameter_distributed",
    )
    require(
        "dataset_electrical_holdout" in generator
        and "DatasetValidationRequired" in generator,
        "release code generation no longer requires an electrical holdout PASS",
    )
    entrypoint = (
        SIMULINK_ROOT / "scripts" / "generate_esp32_plant.m"
    ).read_text(encoding="utf-8")
    require(
        "'InstallIntoEsp32', false" in entrypoint
        and "run_toolchain_qualification.py" in entrypoint
        and "--artifact" in entrypoint,
        "code-generation entry point bypasses staged ESP-IDF qualification",
    )
    configure_model = (
        SIMULINK_ROOT / "+hil" / "configure_model.m"
    ).read_text(encoding="utf-8")
    for token in ("'parameter_hash'", "'simulation_configuration'"):
        require(
            token in configure_model,
            f"returned model artifact lost field {token}",
        )
    validation = (
        SIMULINK_ROOT / "+hil" / "evaluate_validation_acceptance.m"
    ).read_text(encoding="utf-8")
    for token in (
        "rms_voltage_error_V",
        "maximum_voltage_error_V",
        "p95_voltage_error_V",
        "absolute_mean_bias_V",
        "endpoint_soc_error_fraction",
        "surface_temperature_rms_error_C",
        "loaded_rms_voltage_error_V",
        "relaxation_rms_voltage_error_V",
    ):
        require(token in validation, f"numerical validation gate lost {token}")
    dynamic_validation = (
        SIMULINK_ROOT / "+hil" / "validate_dynamic_profile.m"
    ).read_text(encoding="utf-8")
    require(
        "report.voltage_error.count >= 2" not in dynamic_validation,
        "dynamic validation regressed to sample-count-only PASS",
    )
    normalizer = (
        SIMULINK_ROOT / "+hil" / "normalize_dataset.m"
    ).read_text(encoding="utf-8")
    for token in (
        "time = time - time(1)",
        "'initial_soc'",
        "'initial_ocv_V'",
        "'pulse_soc'",
        "'soc_source'",
        "'surface_temperature_source'",
    ):
        require(token in normalizer, f"normalized dataset contract lost {token}")
    p42a_adapter = (
        SIMULINK_ROOT / "adapters" / "load_p42a_hcgt.m"
    ).read_text(encoding="utf-8")
    for token in (
        "raw.surface_temperature_C = nan",
        "raw.surface_temperature_source = 'not_available'",
        "extract_hcgt_windows",
        "fit_cell_ids",
        "holdout_cell_ids",
        "raw.initial_soc = pulse_soc",
    ):
        require(token in p42a_adapter, f"P42A evidence contract lost {token}")
    validate_all = (
        SIMULINK_ROOT / "+hil" / "validate_all.m"
    ).read_text(encoding="utf-8")
    for token in (
        "p42a_dataset_validation",
        "generated_c_parity",
        "esp_idf_build",
        "hardware_qualification",
        "'NOT_RUN'",
        "qualification_complete",
    ):
        require(token in validate_all, f"validation status matrix lost {token}")
    toolchain = (
        HIL_ROOT / "tools" / "run_toolchain_qualification.py"
    ).read_text(encoding="utf-8")
    require(
        "arguments.promote and arguments.skip_idf" in toolchain,
        "toolchain permits promotion while ESP-IDF is skipped",
    )
    require(
        toolchain.index("idf_build_staged") < toolchain.index("atomic_promotion"),
        "toolchain promotion is ordered before the staged ESP-IDF build",
    )
    qualifier = (
        SIMULINK_ROOT / "tools" / "qualify_generated_plant.py"
    ).read_text(encoding="utf-8")
    require(
        "--install-target requires --idf-evidence" in qualifier,
        "low-level qualifier can promote without staged ESP-IDF evidence",
    )
    mcp_driver = (
        ESP32_ROOT / "components" / "CAN" / "mcp2515_driver.c"
    ).read_text(encoding="utf-8")
    for token in (
        "MCP2515_TX_STATUS_CONTROLLER_ERROR",
        "MCP2515_TX_STATUS_BUS_OFF",
        "spi_bit_modify(REG_CANINTF",
        "successful_frames",
        "controller_retry_events",
    ):
        require(token in mcp_driver, f"MCP2515 completion contract lost {token}")
    receiver = (
        REPO_ROOT / "AMS" / "Core" / "Src" / "ext_drivers" / "accumulator.c"
    ).read_text(encoding="utf-8")
    for token in (
        "accumulator_hil_image_begin",
        "accumulator_hil_image_stage_cell_triplet",
        "accumulator_hil_image_stage_temp_triplet",
        "accumulator_hil_image_commit",
        "accumulator_hil_image_expire",
        "accumulator_hil_stage_crc",
        "hil_stage_cell_mask",
        "hil_stage_temp_mask",
        "hil_image_replay_reject_count",
        "hil_image_resync_count",
        "hil_image_duplicate_start_count",
    ):
        require(token in receiver, f"atomic AMS receiver lost {token}")


def check_matlab_block_structure() -> None:
    start = re.compile(
        r"^\s*(function|if|for|while|switch|try|arguments|parfor|spmd)\b"
    )
    finish = re.compile(r"^\s*end\s*(?:%.*)?$")
    failures: list[str] = []
    for path in SIMULINK_ROOT.rglob("*.m"):
        stack: list[tuple[str, int]] = []
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), 1
        ):
            if line.lstrip().startswith("%"):
                continue
            match = start.match(line)
            if match:
                kind = match.group(1)
                if kind == "function" and stack:
                    failures.append(
                        f"{path.relative_to(SIMULINK_ROOT)}:{line_number} "
                        f"nested inside {stack[-1]}"
                    )
                stack.append((kind, line_number))
            elif finish.match(line):
                if stack:
                    stack.pop()
                else:
                    failures.append(
                        f"{path.relative_to(SIMULINK_ROOT)}:{line_number} extra end"
                    )
        if stack:
            failures.append(
                f"{path.relative_to(SIMULINK_ROOT)} unclosed blocks {stack}"
            )
    require(not failures, f"MATLAB block structure failed: {failures}")


def check_python_tools() -> None:
    tools = [
        SIMULINK_ROOT / "tools" / "extract_legacy_params.py",
        SIMULINK_ROOT / "tools" / "compare_baseline.py",
        SIMULINK_ROOT / "tools" / "qualify_generated_plant.py",
        HIL_ROOT / "tools" / "run_toolchain_qualification.py",
        HIL_ROOT / "tools" / "write_source_manifest.py",
        Path(__file__),
    ]
    run([sys.executable, "-m", "py_compile", *map(str, tools)], SIMULINK_ROOT)


def header_macro(header: str, name: str) -> str:
    match = re.search(
        rf"#define\s+{re.escape(name)}\s+(?:\\\s*)?\"([^\"]+)\"",
        header,
        flags=re.MULTILINE,
    )
    require(match is not None, f"canonical component lacks {name}")
    return match.group(1)


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def smoke_codegen_artifact(root: Path, parity_output: str) -> Path:
    artifact_root = root / "artifact"
    component = artifact_root / "esp32_component"
    shutil.copytree(ESP32_ROOT / "components" / "plant_model", component)

    header = (component / "plant_model_manifest.h").read_text(encoding="utf-8")
    model_name = header_macro(header, "PLANT_MODEL_NAME")
    configuration_hash = header_macro(
        header, "PLANT_MODEL_CONFIGURATION_HASH"
    )
    parameter_hash = header_macro(header, "PLANT_MODEL_PARAMETER_HASH")
    pack_configuration = {
        "id": "der26_75s6p",
        "series_groups": 75,
        "parallel_cells": 6,
        "segment_count": 5,
        "temperature_sensor_count": 120,
    }

    model_manifest_file = (
        artifact_root / "configured_model" / "model_manifest.json"
    )
    model_manifest = {
        "schema_version": 1,
        "model_name": model_name,
        "configuration_hash": configuration_hash,
        "parameter_hash": parameter_hash,
        "cell_configuration_hash": "smoke-cell-hash",
        "pack_configuration_hash": "smoke-pack-hash",
        "simulation_configuration_hash": "smoke-simulation-hash",
        "cell_configuration": {"id": "p42a"},
        "pack_configuration": pack_configuration,
        "simulation_configuration": {"id": "hppc_validation"},
    }
    write_json(model_manifest_file, model_manifest)
    identity = {
        "model_name": model_name,
        "configuration_hash": configuration_hash,
        "parameter_hash": parameter_hash,
        "cell_configuration_hash": "smoke-cell-hash",
        "pack_configuration_hash": "smoke-pack-hash",
        "simulation_configuration_hash": "smoke-simulation-hash",
        "model_manifest_sha256": sha256(model_manifest_file),
    }
    model_artifact = {
        **identity,
        "manifest_file": str(model_manifest_file.resolve()),
    }

    component_records = [
        {
            "path": path.relative_to(component).as_posix(),
            "sha256": sha256(path),
        }
        for path in sorted(item for item in component.rglob("*") if item.is_file())
    ]
    component_manifest_file = component / "component_manifest.json"
    write_json(
        component_manifest_file,
        {
            "schema_version": 2,
            **identity,
            "model_artifact": model_artifact,
            "pack_configuration": pack_configuration,
            "files": [record["path"] for record in component_records],
            "file_records": component_records,
        },
    )

    prevalidation_file = artifact_root / "pre_codegen_validation.json"
    write_json(
        prevalidation_file,
        {
            "schema_version": 2,
            "status": "PASS",
            "passed": True,
            "configuration_hash": configuration_hash,
            "parameter_hash": parameter_hash,
            "identity": identity,
            "dataset_electrical_holdout": {
                "status": "PASS",
                "passed": True,
                "test_count": 1,
            },
        },
    )

    parity_directory = artifact_root / "parity"
    parity_directory.mkdir(parents=True)
    parity_input = parity_directory / "input_profile.csv"
    parity_input.write_text(
        "time_s,I_pack_A,T_ambient_C\n0,0,25\n0.01,100,25\n",
        encoding="utf-8",
    )
    reference_output = parity_directory / "reference_output.csv"
    reference_output.write_text(parity_output, encoding="utf-8")
    parity_case = parity_directory / "parity_case.json"
    write_json(
        parity_case,
        {
            "schema_version": 2,
            "reference_origin": "configured_simulink_output",
            "input_file": str(parity_input.resolve()),
            "input_sha256": sha256(parity_input),
            "reference_file": str(reference_output.resolve()),
            "reference_sha256": sha256(reference_output),
            "initial_soc": 1.0,
            "initial_temperature_C": 25.0,
            "scalar_tolerance": 2e-4,
            "array_tolerance": 5e-4,
            "identity": identity,
        },
    )

    source_manifest = artifact_root / "source_manifest.json"
    shutil.copy2(HIL_ROOT / "SOURCE_MANIFEST.json", source_manifest)
    codegen_log = artifact_root / "codegen.log"
    codegen_log.write_text("synthetic strict qualifier smoke fixture\n", encoding="utf-8")
    artifact = artifact_root / "codegen_artifact.json"
    write_json(
        artifact,
        {
            "schema_version": 2,
            "artifact_root": str(artifact_root.resolve()),
            "identity": identity,
            "model": model_artifact,
            "model_manifest_file": str(model_manifest_file.resolve()),
            "esp32_component_directory": str(component.resolve()),
            "component_manifest_file": str(component_manifest_file.resolve()),
            "pre_codegen_validation_file": str(prevalidation_file.resolve()),
            "parity_files": {"manifest": str(parity_case.resolve())},
            "source_manifest_file": str(source_manifest.resolve()),
            "source_manifest_sha256": sha256(source_manifest),
            "codegen_log_file": str(codegen_log.resolve()),
            "matlab_version": "smoke-fixture",
            "simulink_version": "smoke-fixture",
            "simulink_coder_version": "smoke-fixture",
            "artifact_file": str(artifact.resolve()),
        },
    )
    return artifact


def check_host_c() -> None:
    tests = ESP32_ROOT / "tests"
    run(["make", "clean"], tests)
    run(["make", "syntax"], tests)
    output = run(["make", "test"], tests)
    require("PASS plant_model_host_test" in output, "host adapter test did not pass")
    require(
        "PASS hil_image_protocol_test" in output,
        "host image protocol test did not pass",
    )
    require(
        "PASS mcp2515_tx_status_test" in output,
        "MCP2515 terminal-status test did not pass",
    )
    require(
        "PASS mcp2515_driver_host_test" in output,
        "actual MCP2515 driver host-state test did not pass",
    )
    run(["make", "all"], tests)
    with tempfile.TemporaryDirectory(prefix="hil-baseline-") as temporary:
        candidate = Path(temporary) / "candidate.csv"
        with candidate.open("w", encoding="utf-8") as stream:
            subprocess.run(
                [str(tests / "build" / "baseline_runner")],
                cwd=tests,
                check=True,
                text=True,
                stdout=stream,
            )
        baseline = (
            SIMULINK_ROOT
            / "validation"
            / "baselines"
            / "p42a_75s6p_codegen_oracle.csv"
        )
        output = run(
            [
                sys.executable,
                str(SIMULINK_ROOT / "tools" / "compare_baseline.py"),
                str(baseline),
                str(candidate),
            ],
            SIMULINK_ROOT,
        )
        require("PASS baseline parity" in output, "host baseline parity failed")
        parity_input = Path(temporary) / "parity_input.csv"
        parity_input.write_text(
            "time_s,I_pack_A,T_ambient_C\n"
            "0,0,25\n"
            "0.01,100,25\n",
            encoding="utf-8",
        )
        parity_output = run(
            [
                str(tests / "build" / "parity_runner"),
                str(parity_input),
                "1",
                "25",
            ],
            tests,
        )
        rows = parity_output.splitlines()
        require(len(rows) == 3, "parity runner did not emit two output rows")
        require(
            rows[0].startswith(
                "time_s,V_pack,T_core,T_surf,SoC_true,V_min,V_max,T_max,T_avg"
            ),
            "parity runner emitted an incompatible CSV header",
        )
        artifact = smoke_codegen_artifact(Path(temporary), parity_output)
        qualification = run(
            [
                sys.executable,
                str(SIMULINK_ROOT / "tools" / "qualify_generated_plant.py"),
                str(artifact),
            ],
            SIMULINK_ROOT,
        )
        require(
            "PASS generated-C/Simulink parity" in qualification,
            "staged generated-C qualification tool did not pass",
        )
        bypass = subprocess.run(
            [
                sys.executable,
                str(SIMULINK_ROOT / "tools" / "qualify_generated_plant.py"),
                str(artifact),
                "--install-target",
                str(ESP32_ROOT / "components" / "plant_model"),
            ],
            cwd=SIMULINK_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        require(
            bypass.returncode != 0
            and "--install-target requires --idf-evidence" in bypass.stdout,
            "low-level qualifier accepted promotion without ESP-IDF evidence",
        )
        tampered_input = artifact.parent / "parity" / "input_profile.csv"
        tampered_input.write_text(
            tampered_input.read_text(encoding="utf-8") + "0.02,0,25\n",
            encoding="utf-8",
        )
        tamper = subprocess.run(
            [
                sys.executable,
                str(SIMULINK_ROOT / "tools" / "qualify_generated_plant.py"),
                str(artifact),
            ],
            cwd=SIMULINK_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        require(
            tamper.returncode != 0 and "parity input hash mismatch" in tamper.stdout,
            "qualifier accepted a parity input after its manifest hash changed",
        )
        forbidden = subprocess.run(
            [
                sys.executable,
                str(HIL_ROOT / "tools" / "run_toolchain_qualification.py"),
                "--promote",
                "--skip-idf",
            ],
            cwd=HIL_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        require(
            forbidden.returncode != 0
            and "--promote cannot be combined with --skip-idf"
            in forbidden.stdout,
            "toolchain did not reject promotion without ESP-IDF",
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-archive", type=Path)
    arguments = parser.parse_args()
    checks = [
        ("required files", check_required_files),
        ("portable MATLAB paths", check_no_machine_paths),
        ("model archives", check_model_archives),
        ("parameter snapshot", check_parameter_snapshot),
        ("frozen hashes", lambda: check_frozen_hashes(arguments.source_archive)),
        ("source manifest", check_source_manifest),
        ("source contracts", check_source_contracts),
        ("MATLAB block structure", check_matlab_block_structure),
        ("Python tools", check_python_tools),
        ("host C and baseline", check_host_c),
    ]
    for name, check in checks:
        check()
        print(f"PASS {name}")
    print(f"PASS all static/host checks ({len(checks)} groups)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
