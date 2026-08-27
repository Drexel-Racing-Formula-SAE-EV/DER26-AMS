"""Independent Python/C differential checks for the EAC14-80 observer."""

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


class FuseConfig(ctypes.Structure):
    _fields_ = [
        ("rated_current_a", ctypes.c_float),
        ("typical_melting_i2t_a2s", ctypes.c_float),
        ("usable_i2t_fraction", ctypes.c_float),
        ("cooling_time_constant_s", ctypes.c_float),
        ("initialization_soak_s", ctypes.c_float),
        ("quiescent_current_a", ctypes.c_float),
        ("fuse_temperature_margin_c", ctypes.c_float),
        ("minimum_temperature_derating", ctypes.c_float),
        ("maximum_state_multiple", ctypes.c_float),
    ]


class FuseState(ctypes.Structure):
    _fields_ = [
        ("excess_i2t_a2s", ctypes.c_float),
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
        ("usable_i2t_a2s", ctypes.c_float),
        ("remaining_i2t_a2s", ctypes.c_float),
        ("estimated_fuse_temperature_c", ctypes.c_float),
        ("temperature_derating", ctypes.c_float),
        ("continuous_current_a", ctypes.c_float),
        ("discharge_current_cap_a", ctypes.c_float * 4),
        ("reason_flags", ctypes.c_uint16),
        ("valid", ctypes.c_uint8),
        ("authority_valid", ctypes.c_uint8),
        ("budget_exhausted", ctypes.c_uint8),
    ]


def interpolate(x: float, x0: float, y0: float,
                x1: float, y1: float) -> float:
    fraction = min(1.0, max(0.0, (x - x0) / (x1 - x0)))
    return y0 + fraction * (y1 - y0)


def temperature_derating(temp_c: float, minimum: float) -> float:
    if temp_c <= 0.0:
        value = 1.03
    elif temp_c <= 25.0:
        value = interpolate(temp_c, 0.0, 1.03, 25.0, 1.0)
    elif temp_c <= 80.0:
        value = interpolate(temp_c, 25.0, 1.0, 80.0, 0.90)
    elif temp_c <= 125.0:
        value = interpolate(temp_c, 80.0, 0.90, 125.0, 0.80)
    else:
        value = 0.80
    return min(1.0, max(minimum, value))


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

    def test_seeded_state_and_cap_differential(self) -> None:
        fuse_cfg = FuseConfig()
        sop_cfg = CConfig()
        self.lib.ams_fuse_observer_default_config(ctypes.byref(fuse_cfg))
        self.lib.ams_sop_default_config(ctypes.byref(sop_cfg))
        state = FuseState()
        state.thermal_state_initialized = 1
        model_state = 0.0
        exhausted = False
        budget = (fuse_cfg.typical_melting_i2t_a2s *
                  fuse_cfg.usable_i2t_fraction)
        rng = random.Random(0xEAC1480)

        for _ in range(500):
            source = FuseInput(
                pack_current_a=rng.uniform(-15.0, 130.0),
                current_uncertainty_a=rng.uniform(0.2, 2.0),
                temperature_proxy_c=rng.uniform(-20.0, 90.0),
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
            continuous = fuse_cfg.rated_current_a * derating
            adverse = abs(source.pack_current_a) + source.current_uncertainty_a
            excess_rate = max(0.0, adverse * adverse -
                              continuous * continuous)
            model_state = (model_state * math.exp(
                -source.elapsed_s / fuse_cfg.cooling_time_constant_s) +
                excess_rate * source.elapsed_s)
            model_state = min(budget * fuse_cfg.maximum_state_multiple,
                              max(0.0, model_state))
            utilization = model_state / budget
            if utilization >= 1.0:
                exhausted = True
            elif utilization <= 0.5:
                exhausted = False
            remaining = max(0.0, budget - model_state)

            self.assertAlmostEqual(result.estimated_fuse_temperature_c,
                                   fuse_temp, delta=2e-5)
            self.assertAlmostEqual(result.temperature_derating,
                                   derating, delta=2e-5)
            self.assertAlmostEqual(result.utilization,
                                   utilization, delta=2e-4)
            self.assertEqual(bool(result.budget_exhausted), exhausted)
            for index, horizon in enumerate(HORIZONS):
                expected = 0.0 if exhausted else math.sqrt(
                    continuous * continuous + remaining / horizon)
                expected = min(sop_cfg.discharge_current_max_a[index],
                               expected)
                self.assertAlmostEqual(result.discharge_current_cap_a[index],
                                       expected, delta=2e-3)
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
