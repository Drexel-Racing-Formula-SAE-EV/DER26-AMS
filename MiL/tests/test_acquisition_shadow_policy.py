#!/usr/bin/env python3
"""Static policy regression for retryable constrained shadow acquisition.

The numerical behavior is exercised in MATLAB. This guard prevents the reference
candidate from silently reverting to either frozen acquisition or unconstrained
full-state voltage updates while acquisition is pending.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
src = (ROOT / "MiL" / "matlab" / "+mil" / "+reference" / "run_segment_ekf.m").read_text()
cfg = (ROOT / "MiL" / "matlab" / "+mil" / "default_config.m").read_text()
dbg = (ROOT / "MiL" / "matlab" / "scripts" / "debug_c1_fixed_basis_acquisition.m").read_text()

required = [
    ("hold_measurement_updates',false" in src, "reference default does not use shadow acquisition"),
    ("acquisition_pending" in src and "acquisition_shadow_updates" in src,
     "reference acquisition shadow-update telemetry/policy is missing"),
    ("dynamic_soc_only_updates" in src and "acquisition_dynamic_soc_update" in src,
     "reference constrained dynamic SoC-only acquisition path is missing"),
    ("Rnuis" in src and "dynamic_max_soc_step" in src,
     "reference dynamic acquisition no longer treats Vp as nuisance uncertainty"),
    ("'hold_measurement_updates', false" in cfg,
     "global MiL default does not select shadow acquisition"),
    ("'dynamic_soc_only_updates', true" in cfg,
     "global MiL default does not select constrained dynamic acquisition"),
    ("true_relaxed_case" in dbg and "25 s initial rest" in dbg,
     "directed acquisition script does not use a true relaxed positive control"),
    ("original HPPC -20 pp" in dbg,
     "directed acquisition script no longer mirrors the no-rest HPPC initial error"),
    ("end_s',20.1" in dbg,
     "coherent voltage-bias regression no longer overlaps the first acquisition decision"),
]
for ok, message in required:
    if not ok:
        raise SystemExit(message)
print("retryable constrained shadow acquisition policy regression: PASS")
