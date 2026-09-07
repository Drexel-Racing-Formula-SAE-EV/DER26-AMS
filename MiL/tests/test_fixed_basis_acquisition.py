#!/usr/bin/env python3
"""Math-level regression for the fixed-basis relaxation acquisition.

This does not execute MATLAB and is not qualification evidence. It verifies the
checked-in P42A OCV surface is invertible, the selected 20 s / 10 s / 35 s
basis is numerically usable at 1 Hz, and a synthetic relaxation record can
recover asymptotic OCV plus residual polarization without hidden SoC input.
"""
from pathlib import Path
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[2]
MAT = ROOT / "HiL" / "simulink" / "parameters" / "source" / "p42a_legacy_codegen_snapshot.mat"
d = loadmat(MAT, squeeze_me=True)
soc_bp = np.asarray(d["soc_ocv_common"], dtype=float).reshape(-1)
temp_bp = np.asarray(d["temp_bp_ocv"], dtype=float).reshape(-1)
ocv = np.asarray(d["OCV"], dtype=float)

if ocv.shape != (soc_bp.size, temp_bp.size):
    raise SystemExit(f"unexpected OCV shape {ocv.shape}")
for j, temp in enumerate(temp_bp):
    if not np.all(np.diff(ocv[:, j]) > 0):
        raise SystemExit(f"OCV is not strictly monotonic at {temp:g} C")


def curve_at_temp(temp_c: float) -> np.ndarray:
    temp_c = float(np.clip(temp_c, temp_bp.min(), temp_bp.max()))
    return np.array([np.interp(temp_c, temp_bp, ocv[k, :]) for k in range(soc_bp.size)])


def inverse_ocv(v_cell: float, temp_c: float) -> float:
    curve = curve_at_temp(temp_c)
    if v_cell < curve[0] or v_cell > curve[-1]:
        return np.nan
    return float(np.interp(v_cell, curve, soc_bp))


for temp_c in (5.0, 25.0, 40.0):
    curve = curve_at_temp(temp_c)
    for true_soc in (0.15, 0.60, 0.95):
        v = float(np.interp(true_soc, soc_bp, curve))
        recovered = inverse_ocv(v, temp_c)
        if abs(recovered - true_soc) > 2e-6:
            raise SystemExit(
                f"inverse OCV mismatch T={temp_c} SoC={true_soc}: {recovered}"
            )

# Provisional embedded/reference acquisition basis selected by directed MiL.
TAU1_S = 10.0
TAU2_S = 35.0
WINDOW_S = 20.0
SAMPLE_S = 1.0

t = np.arange(0.0, WINDOW_S + 0.5 * SAMPLE_S, SAMPLE_S)
X = np.column_stack(
    [np.ones_like(t), np.exp(-t / TAU1_S), np.exp(-t / TAU2_S)]
)
rcond = 1.0 / np.linalg.cond(X.T @ X)
if not np.isfinite(rcond) or rcond < 1.0e-5:
    raise SystemExit(f"fixed acquisition basis is ill-conditioned: rcond={rcond:g}")

# Directed synthetic records exercise both residual-polarization signs. Hidden
# SoC is used only to generate/scoring the synthetic terminal voltage here.
rng = np.random.default_rng(260828)
for true_soc, vp1_0, vp2_0 in [
    (0.56, +0.046, +0.029),
    (0.607, -0.0085, -0.0048),
]:
    v_inf = float(np.interp(true_soc, soc_bp, curve_at_temp(25.0)))
    y = v_inf - vp1_0 * np.exp(-t / TAU1_S) - vp2_0 * np.exp(-t / TAU2_S)
    y = y + rng.normal(0.0, 0.0004, size=y.shape)
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    fitted_soc = inverse_ocv(float(beta[0]), 25.0)
    if not np.isfinite(fitted_soc) or abs(fitted_soc - true_soc) > 0.01:
        raise SystemExit(
            f"fixed-basis SoC recovery failed: true={true_soc}, fit={fitted_soc}"
        )
    vp1_finish = -float(beta[1]) * np.exp(-WINDOW_S / TAU1_S)
    vp2_finish = -float(beta[2]) * np.exp(-WINDOW_S / TAU2_S)
    true_finish = vp1_0 * np.exp(-WINDOW_S / TAU1_S) + vp2_0 * np.exp(-WINDOW_S / TAU2_S)
    if abs((vp1_finish + vp2_finish) - true_finish) > 0.010:
        raise SystemExit("fixed-basis residual-polarization recovery failed")

print("fixed-basis relaxation acquisition math regression: PASS")
