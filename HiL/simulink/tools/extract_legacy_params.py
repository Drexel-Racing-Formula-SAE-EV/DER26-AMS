#!/usr/bin/env python3
"""Reconstruct the reviewed P42A parameter source from generated-C constants.

This is a migration bridge, not a replacement for the external raw CDT/HCGT
dataset. It prevents the generated C snapshot from remaining the only
machine-readable source of the exact constants currently running on the ESP32.
"""

from __future__ import annotations

import csv
import hashlib
import json
import re
from pathlib import Path

import numpy as np
from scipy.io import savemat


ROOT = Path(__file__).resolve().parents[1]
CONST_PARAMS = (
    ROOT.parent
    / "esp32_plant"
    / "components"
    / "plant_model"
    / "const_params.c"
)
MODEL_SOURCE = (
    ROOT
    / "generated_code"
    / "model"
    / "drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.c"
)
OUTPUT_MAT = ROOT / "parameters" / "source" / "p42a_legacy_codegen_snapshot.mat"
OUTPUT_MANIFEST = (
    ROOT / "parameters" / "manifests" / "p42a_legacy_codegen_snapshot.json"
)
OUTPUT_SUMMARY = (
    ROOT / "validation" / "generated" / "p42a_legacy_parameter_summary.csv"
)


ARRAY_NAMES = {
    "ocv": "rtCP_pooled_2VS2GaLG7tXs",
    "soc_ocv": "rtCP_pooled_LAE26UlapTkv",
    "temperature": "rtCP_pooled_aZWafQUW4ySs",
    "r0": "rtCP_pooled_MJJiVyV6u4B7",
    "soc": "rtCP_pooled_oSJNQ9HBXQTR",
    "r1": "rtCP_pooled_3i7E1u0kL9f2",
    "neg_inv_r1c1": "rtCP_pooled_x01aWAzB59fl",
    "inv_c1": "rtCP_pooled_wGkYb7XWb2i2",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_array(text: str, name: str) -> np.ndarray:
    pattern = re.compile(
        rf"const\s+(?:real32_T|uint32_T)\s+{re.escape(name)}"
        rf"\[(\d+)\]\s*=\s*\{{(.*?)\}}\s*;",
        re.DOTALL,
    )
    match = pattern.search(text)
    if not match:
        raise RuntimeError(f"Could not find generated constant array {name}")

    expected_count = int(match.group(1))
    tokens = re.findall(
        r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[Ee][-+]?\d+)?", match.group(2)
    )
    values = np.asarray([float(token) for token in tokens], dtype=np.float32)
    if values.size != expected_count:
        raise RuntimeError(
            f"{name}: parsed {values.size} values, expected {expected_count}"
        )
    return values


def reshape_matlab(values: np.ndarray, rows: int, columns: int) -> np.ndarray:
    if values.size != rows * columns:
        raise RuntimeError(
            f"Cannot reshape {values.size} values to {rows}x{columns}"
        )
    return values.reshape((rows, columns), order="F")


def main() -> None:
    text = CONST_PARAMS.read_text(encoding="utf-8", errors="strict")
    arrays = {key: parse_array(text, name) for key, name in ARRAY_NAMES.items()}

    soc_ocv = arrays["soc_ocv"].reshape((-1, 1))
    soc = arrays["soc"].reshape((-1, 1))
    temperature = arrays["temperature"].reshape((1, -1))
    ocv = reshape_matlab(arrays["ocv"], soc_ocv.size, temperature.size)
    r0 = reshape_matlab(arrays["r0"], soc.size, temperature.size)
    r1 = reshape_matlab(arrays["r1"], soc.size, temperature.size)
    inv_c1 = reshape_matlab(arrays["inv_c1"], soc.size, temperature.size)
    neg_inv_r1c1 = reshape_matlab(
        arrays["neg_inv_r1c1"], soc.size, temperature.size
    )
    c1 = np.reciprocal(inv_c1, dtype=np.float32)

    if not np.allclose(
        neg_inv_r1c1,
        -1.0 / (r1 * c1),
        rtol=3e-6,
        atol=1e-8,
    ):
        raise RuntimeError("Generated R1/C1 inverse tables are internally inconsistent")

    source_files = [
        {
            "path": str(CONST_PARAMS.relative_to(ROOT.parent.parent)),
            "sha256": sha256_file(CONST_PARAMS),
            "bytes": CONST_PARAMS.stat().st_size,
        },
        {
            "path": str(MODEL_SOURCE.relative_to(ROOT.parent.parent)),
            "sha256": sha256_file(MODEL_SOURCE),
            "bytes": MODEL_SOURCE.stat().st_size,
        },
    ]
    configuration_payload = {
        "cell_id": "p42a",
        "capacity_Ah": 4.2,
        "soc": soc.flatten().tolist(),
        "soc_ocv": soc_ocv.flatten().tolist(),
        "temperature_C": temperature.flatten().tolist(),
        "source_sha256": [item["sha256"] for item in source_files],
    }
    configuration_hash = hashlib.sha256(
        json.dumps(configuration_payload, sort_keys=True).encode("utf-8")
    ).hexdigest()

    source_manifest = {
        "schema_version": 1,
        "cell_manufacturer": "Molicel",
        "cell_model": "INR-21700-P42A",
        "dataset_source": "Legacy generated-C migration snapshot",
        "dataset_version": "model-1.67-generated-2026-06-27",
        "current_sign_convention": "positive current = discharge",
        "cell_ids": ["A2", "A4", "A6", "A8", "A9", "A11", "A12", "A13", "A15", "A19", "A21", "A37"],
        "temperatures_C": temperature.flatten().tolist(),
        "soc_coverage": [float(soc_ocv.min()), float(soc_ocv.max())],
        "source_files": source_files,
        "known_limitations": [
            "This artifact preserves generated constants but is not the raw fitting evidence.",
            "External CDT/HCGT data is still required for a defensible refit.",
            "Thermal values are first-pass cell priors, not calibrated pack boundaries.",
        ],
    }

    params = {
        "schema_version": np.uint32(1),
        "cell_id": "p42a",
        "Q_nom": np.float32(4.2),
        "SoC_init": np.float32(1.0),
        "T_init": np.float32(25.0),
        "soc_common": soc.astype(np.float32),
        "soc_ocv_common": soc_ocv.astype(np.float32),
        "temp_bp": temperature.astype(np.float32),
        "temp_bp_ocv": temperature.astype(np.float32),
        "OCV": ocv.astype(np.float32),
        "R0": r0.astype(np.float32),
        "R1": r1.astype(np.float32),
        "C1": c1.astype(np.float32),
        "R2": np.float32(0.0040),
        "C2": np.float32(12000.0),
        "Cc": np.float32(55.0),
        "Cs": np.float32(15.0),
        "Rcs": np.float32(1.5),
        "Rsa": np.float32(8.0),
        "OCV_2d": ocv.astype(np.float32),
        "R0_2d_fix": r0.astype(np.float32),
        "R1_2d": r1.astype(np.float32),
        "inv_C1_2d_fixed": inv_c1.astype(np.float32),
        "neg_inv_R1C1_2d_fixed": neg_inv_r1c1.astype(np.float32),
        "fit_quality": {
            "migration_status": "exact generated-constant reconstruction",
            "electrical_source": "checked-in generated C",
            "raw_fit_metrics_available": np.uint8(0),
        },
        "validity_domain": {
            "soc": np.asarray([0.0, 1.0], dtype=np.float64),
            "temperature_C": np.asarray([5.0, 40.0], dtype=np.float64),
            "current_sign": "positive_discharge",
        },
        "source_manifest": source_manifest,
        "generation_timestamp_utc": "2026-06-27T23:54:03Z",
        "configuration_hash": configuration_hash,
    }

    OUTPUT_MAT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_SUMMARY.parent.mkdir(parents=True, exist_ok=True)

    flattened = dict(params)
    flattened["params"] = params
    savemat(OUTPUT_MAT, flattened, do_compression=True, long_field_names=True)

    manifest = {
        "schema_version": 1,
        "artifact": OUTPUT_MAT.name,
        "configuration_hash": configuration_hash,
        "source_manifest": source_manifest,
        "dimensions": {
            "OCV": list(ocv.shape),
            "R0": list(r0.shape),
            "R1": list(r1.shape),
            "C1": list(c1.shape),
        },
        "scalars": {
            "Q_nom_Ah": 4.2,
            "R2_ohm": 0.004,
            "C2_F": 12000.0,
            "Cc_J_per_K": 55.0,
            "Cs_J_per_K": 15.0,
            "Rcs_K_per_W": 1.5,
            "Rsa_K_per_W": 8.0,
        },
    }
    OUTPUT_MANIFEST.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    with OUTPUT_SUMMARY.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "Temp_C",
                "SoC",
                "R0_ohm",
                "R1_ohm",
                "C1_F",
                "R2_ohm",
                "C2_F",
                "tau1_s",
                "tau2_s",
                "source",
            ]
        )
        for column, temp in enumerate(temperature.flatten()):
            target_soc = 0.8
            r0_value = float(np.interp(target_soc, soc.flatten(), r0[:, column]))
            r1_value = float(np.interp(target_soc, soc.flatten(), r1[:, column]))
            c1_value = float(np.interp(target_soc, soc.flatten(), c1[:, column]))
            writer.writerow(
                [
                    f"{float(temp):.0f}",
                    f"{target_soc:.2f}",
                    f"{r0_value:.12g}",
                    f"{r1_value:.12g}",
                    f"{c1_value:.12g}",
                    "0.004",
                    "12000",
                    f"{r1_value * c1_value:.12g}",
                    "48",
                    "generated_c_model_1.67",
                ]
            )

    print(f"Wrote {OUTPUT_MAT}")
    print(f"Wrote {OUTPUT_MANIFEST}")
    print(f"Wrote {OUTPUT_SUMMARY}")


if __name__ == "__main__":
    main()
