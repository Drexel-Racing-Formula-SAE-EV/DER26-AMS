"""Independent Python/C differential checks for the EAC14-80 curve observer."""

from __future__ import annotations

import ctypes
import math
import random
import subprocess
import tempfile
import unittest
from pathlib import Path

from test_reference import CConfig


ROOT = Path(__file__).resolve().parents[3]
AMS = ROOT / "AMS"
HORIZONS = (0.1, 1.0, 10.0, 30.0)
CURVE = (
    (154.0, 100.0), (168.0, 50.0), (180.9, 30.0),
    (192.8, 20.0), (219.2, 10.0), (249.2, 5.0),
    (274.9, 3.0), (300.8, 2.0), (350.9, 1.0),
    (411.1, 0.5), (458.9, 0.3), (500.2, 0.2),
    (576.6, 0.1), (652.8, 0.05), (705.8, 0.03),
    (752.5, 0.02), (800.0, 0.01253125), (850.0, 0.01078),
)


class FuseConfig(ctypes.Structure):
    _fields_ = [
        ("rated_current_a", ctypes.c_float),
        ("curve_time_fraction", ctypes.c_float),
        ("cooling_time_constant_s", ctypes.c_float),
        ("initialization_soak_s", ctypes.c_float),
        ("quiescent_current_a", ctypes.c_float),
        ("fuse_temperature_margin_c", ctypes.c_float),
        ("minimum_temperature_derating", ctypes.c_float),
        ("maximum_state_multiple", ctypes.c_float),
        ("low_current_fit_scale_s", ctypes.c_float),
        ("low_current_fit_exponent", ctypes.c_float),
        ("maximum_curve_time_s", ctypes.c_float),
        ("minimum_curve_time_s", ctypes.c_float),
    ]


class FuseState(ctypes.Structure):
    _fields_ = [
        ("thermal_utilization", ctypes.c_float),
        ("quiescent_time_s", ctypes.c_float),
        ("update_count", ctypes.c_uint32),
        ("invalid_count", ctypes.c_uint32),
        ("thermal_state_initialized", ctypes.c_uint8),
        ("budget_exhausted", ctypes.c_uint8),
    ]


class FuseInput(ctypes.Structure):
    _fields_ = [
        ("pack_current_a", ctypes.c_float),
        ("current_uncertainty_a", ctypes.c_float),
        ("temperature_proxy_c", ctypes.c_float),
        ("elapsed_s", ctypes.c_float),
        ("measurement_valid", ctypes.c_uint8),
        ("current_calibrated", ctypes.c_uint8),
        ("current_polarity_validated", ctypes.c_uint8),
        ("temperature_measured_at_fuse", ctypes.c_uint8),
        ("model_validated", ctypes.c_uint8),
    ]


class FuseResult(ctypes.Structure):
    _fields_ = [
        ("utilization", ctypes.c_float),
        ("remaining_utilization", ctypes.c_float),
        ("estimated_fuse_temperature_c", ctypes.c_float),
        ("temperature_derating", ctypes.c_float),
        ("continuous_current_a", ctypes.c_float),
        ("effective_current_a", ctypes.c_float),
        ("equivalent_25c_current_a", ctypes.c_float),
        ("typical_melt_time_s", ctypes.c_float),
        ("usable_melt_time_s", ctypes.c_float),
        ("discharge_current_cap_a", ctypes.c_float * 4),
        ("reason_flags", ctypes.c_uint16),
        ("valid", ctypes.c_uint8),
        ("authority_valid", ctypes.c_uint8),
        ("budget_exhausted", ctypes.c_uint8),
        ("curve_extrapolated", ctypes.c_uint8),
    ]


def interpolate(x: float, x0: float, y0: float,
                x1: float, y1: float) -> float:
    fraction = min(1.0, max(0.0, (x - x0) / (x1 - x0)))
    return y0 + fraction * (y1 - y0)


def temperature_derating(temp_c: float, minimum: float) -> float:
    points = (
        (-40.0, 1.15), (0.0, 1.06), (25.0, 1.0), (40.0, 0.97),
        (60.0, 0.93), (80.0, 0.89), (100.0, 0.85), (125.0, 0.80),
    )
    if temp_c <= points[0][0]:
        value = points[0][1]
    else:
        value = points[-1][1]
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            if temp_c <= x1:
                value = interpolate(temp_c, x0, y0, x1, y1)
                break
    return min(1.0, max(minimum, value))


def typical_melt_time(cfg: FuseConfig, current_a: float) -> float:
    if current_a <= cfg.rated_current_a:
        return math.inf
    if current_a < CURVE[0][0]:
        overcurrent = current_a / cfg.rated_current_a - 1.0
        time_s = cfg.low_current_fit_scale_s * (
            overcurrent ** (-cfg.low_current_fit_exponent))
        return min(cfg.maximum_curve_time_s,
                   max(cfg.minimum_curve_time_s, time_s))
    for (i0, t0), (i1, t1) in zip(CURVE, CURVE[1:]):
        if current_a <= i1:
            f = ((math.log(current_a) - math.log(i0)) /
                 (math.log(i1) - math.log(i0)))
            return math.exp(math.log(t0) + f * (math.log(t1) - math.log(t0)))
    (i0, t0), (i1, t1) = CURVE[-2], CURVE[-1]
    slope = (math.log(t1) - math.log(t0)) / (math.log(i1) - math.log(i0))
    time_s = math.exp(math.log(t1) + slope *
                      (math.log(current_a) - math.log(i1)))
    return min(cfg.maximum_curve_time_s,
               max(cfg.minimum_curve_time_s, time_s))


def source_rate(cfg: FuseConfig, equivalent_current_a: float) -> float:
    typical = typical_melt_time(cfg, equivalent_current_a)
    if not math.isfinite(typical):
        return 0.0
    usable = min(cfg.maximum_curve_time_s,
                 max(cfg.minimum_curve_time_s,
                     typical * cfg.curve_time_fraction))
    kernel = -cfg.cooling_time_constant_s * math.expm1(
        -usable / cfg.cooling_time_constant_s)
    return 1.0 / kernel


def predicted_production(state: float, cfg: FuseConfig,
                         candidate_a: float, uncertainty_a: float,
                         derating: float, horizon_s: float) -> float:
    effective = max(0.0, candidate_a) + uncertainty_a
    rate = source_rate(cfg, effective / derating)
    return (state * math.exp(-horizon_s / cfg.cooling_time_constant_s) +
            rate * horizon_s)


def cap_for_horizon(state: float, exhausted: bool, cfg: FuseConfig,
                    static_cap_a: float, uncertainty_a: float,
                    derating: float, horizon_s: float) -> float:
    if exhausted or static_cap_a <= 0.0:
        return 0.0
    if predicted_production(state, cfg, static_cap_a, uncertainty_a,
                            derating, horizon_s) <= 1.0:
        return static_cap_a
    low, high = 0.0, static_cap_a
    for _ in range(24):
        mid = 0.5 * (low + high)
        if predicted_production(state, cfg, mid, uncertainty_a,
                                derating, horizon_s) <= 1.0:
            low = mid
        else:
            high = mid
    return low


class FuseReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.tempdir = tempfile.TemporaryDirectory(prefix="der26_fuse_oracle_")
        library = Path(cls.tempdir.name) / "libams_fuse.so"
        command = [
            "gcc", "-std=c11", "-O2", "-fPIC", "-shared",
            "-Wall", "-Wextra", "-Werror",
            f"-I{AMS / 'Core/Inc'}",
            str(AMS / "Core/Src/sop/ams_fuse_observer.c"),
            str(AMS / "Core/Src/sop/ams_sop.c"),
            str(AMS / "Core/Src/estimator/ams_estimator_lut.c"),
            "-lm", "-o", str(library),
        ]
        subprocess.run(command, check=True, capture_output=True, text=True)
        cls.lib = ctypes.CDLL(str(library))
        cls.lib.ams_fuse_observer_default_config.argtypes = [
            ctypes.POINTER(FuseConfig)
        ]
        cls.lib.ams_sop_default_config.argtypes = [ctypes.POINTER(CConfig)]
        cls.lib.ams_fuse_observer_update.argtypes = [
            ctypes.POINTER(FuseState), ctypes.POINTER(FuseConfig),
            ctypes.POINTER(CConfig), ctypes.POINTER(FuseInput),
            ctypes.POINTER(FuseResult),
        ]
        cls.lib.ams_fuse_observer_update.restype = ctypes.c_bool

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tempdir.cleanup()

    def test_curve_anchor(self) -> None:
        cfg = FuseConfig()
        self.lib.ams_fuse_observer_default_config(ctypes.byref(cfg))
        self.assertAlmostEqual(typical_melt_time(cfg, 800.0),
                               0.01253125, delta=1e-10)
        self.assertTrue(math.isinf(typical_melt_time(cfg, 80.0)))
        self.assertGreater(typical_melt_time(cfg, 100.0), 1000.0)

    def test_seeded_state_and_cap_differential(self) -> None:
        fuse_cfg = FuseConfig()
        sop_cfg = CConfig()
        self.lib.ams_fuse_observer_default_config(ctypes.byref(fuse_cfg))
        self.lib.ams_sop_default_config(ctypes.byref(sop_cfg))
        state = FuseState()
        state.thermal_state_initialized = 1
        model_state = 0.0
        exhausted = False
        rng = random.Random(0xEAC1480)

        for _ in range(1000):
            source = FuseInput(
                pack_current_a=rng.uniform(-80.0, 220.0),
                current_uncertainty_a=rng.uniform(0.2, 2.0),
                temperature_proxy_c=rng.uniform(-20.0, 95.0),
                elapsed_s=rng.uniform(0.01, 0.2),
                measurement_valid=1,
                current_calibrated=1,
                current_polarity_validated=1,
                temperature_measured_at_fuse=0,
                model_validated=1,
            )
            result = FuseResult()
            self.assertTrue(self.lib.ams_fuse_observer_update(
                ctypes.byref(state), ctypes.byref(fuse_cfg),
                ctypes.byref(sop_cfg), ctypes.byref(source),
                ctypes.byref(result)))

            fuse_temp = (source.temperature_proxy_c +
                         fuse_cfg.fuse_temperature_margin_c)
            derating = temperature_derating(
                fuse_temp, fuse_cfg.minimum_temperature_derating)
            effective = abs(source.pack_current_a) + source.current_uncertainty_a
            rate = source_rate(fuse_cfg, effective / derating)
            model_state = (model_state * math.exp(
                -source.elapsed_s / fuse_cfg.cooling_time_constant_s) +
                rate * source.elapsed_s)
            model_state = min(fuse_cfg.maximum_state_multiple,
                              max(0.0, model_state))
            if model_state >= 1.0:
                exhausted = True
            elif model_state <= 0.5:
                exhausted = False

            self.assertAlmostEqual(result.estimated_fuse_temperature_c,
                                   fuse_temp, delta=2e-5)
            self.assertAlmostEqual(result.temperature_derating,
                                   derating, delta=2e-5)
            self.assertAlmostEqual(result.utilization,
                                   model_state, delta=5e-4)
            self.assertEqual(bool(result.budget_exhausted), exhausted)
            for index, horizon in enumerate(HORIZONS):
                expected = cap_for_horizon(
                    model_state, exhausted, fuse_cfg,
                    sop_cfg.discharge_current_max_a[index],
                    source.current_uncertainty_a, derating, horizon)
                self.assertAlmostEqual(result.discharge_current_cap_a[index],
                                       expected, delta=4e-3)
                self.assertLessEqual(result.discharge_current_cap_a[index],
                                     sop_cfg.discharge_current_max_a[index])

    def test_invalid_measurement_fails_closed(self) -> None:
        fuse_cfg = FuseConfig()
        sop_cfg = CConfig()
        self.lib.ams_fuse_observer_default_config(ctypes.byref(fuse_cfg))
        self.lib.ams_sop_default_config(ctypes.byref(sop_cfg))
        state = FuseState()
        source = FuseInput(pack_current_a=0.0,
                           current_uncertainty_a=0.5,
                           temperature_proxy_c=25.0,
                           elapsed_s=0.1,
                           measurement_valid=0,
                           current_calibrated=1,
                           current_polarity_validated=1,
                           temperature_measured_at_fuse=0,
                           model_validated=1)
        result = FuseResult()
        self.assertFalse(self.lib.ams_fuse_observer_update(
            ctypes.byref(state), ctypes.byref(fuse_cfg),
            ctypes.byref(sop_cfg), ctypes.byref(source),
            ctypes.byref(result)))
        self.assertEqual(result.valid, 0)
        self.assertEqual(list(result.discharge_current_cap_a),
                         [0.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
