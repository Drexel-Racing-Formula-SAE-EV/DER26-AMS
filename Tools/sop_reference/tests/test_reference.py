"""Differential verification for the embedded SoP solver.

The Python side reads the generated HIL calibration.  The shared library side
compiles the target C LUT and solver.  Only their externally visible input and
result structures are shared.
"""

from __future__ import annotations

import ctypes
import random
import subprocess
import sys
import tempfile
import unittest
from copy import deepcopy
from math import fabs
from pathlib import Path

REFERENCE_DIR = Path(__file__).resolve().parents[1]
ROOT = REFERENCE_DIR.parents[1]
AMS = ROOT / "AMS"
sys.path.insert(0, str(REFERENCE_DIR))

from model import (  # noqa: E402
    BISECTION_ITERATIONS,
    CELLS_PER_SEGMENT,
    HORIZONS,
    SEGMENTS,
    Binding,
    Calibration,
    Config,
    Input,
    Segment,
    evaluate_current,
    monotonic,
    nominal_input,
    solve,
)


class CSegmentInput(ctypes.Structure):
    _fields_ = [
        ("soc", ctypes.c_float),
        ("vp1_v", ctypes.c_float),
        ("vp2_v", ctypes.c_float),
        ("r0_ohm", ctypes.c_float),
        ("core_temp_c", ctypes.c_float),
        ("surface_max_temp_c", ctypes.c_float),
        ("p_soc", ctypes.c_float),
        ("p_vp1", ctypes.c_float),
        ("p_vp2", ctypes.c_float),
        ("p_r0", ctypes.c_float),
        ("innovation_v", ctypes.c_float),
        ("capacity_soh_lower", ctypes.c_float),
        ("resistance_soh_upper", ctypes.c_float),
        ("cell_voltage_v", ctypes.c_float * CELLS_PER_SEGMENT),
        ("max_cell_age_ms", ctypes.c_uint32),
        ("cell_usable_mask", ctypes.c_uint16),
        ("estimator_valid", ctypes.c_uint8),
        ("model_domain_flags", ctypes.c_uint8),
        ("capacity_soh_valid", ctypes.c_uint8),
        ("resistance_soh_valid", ctypes.c_uint8),
    ]


class CInput(ctypes.Structure):
    _fields_ = [
        ("measurement_sequence", ctypes.c_uint32),
        ("measurement_timestamp_ms", ctypes.c_uint32),
        ("now_ms", ctypes.c_uint32),
        ("pack_current_a", ctypes.c_float),
        ("pack_current_uncertainty_a", ctypes.c_float),
        ("ambient_temp_c", ctypes.c_float),
        ("segment", CSegmentInput * SEGMENTS),
        ("operating_mode", ctypes.c_int),
        ("measurement_valid", ctypes.c_uint8),
        ("estimator_valid", ctypes.c_uint8),
        ("estimator_segment_topology", ctypes.c_uint8),
        ("current_calibrated", ctypes.c_uint8),
        ("current_polarity_validated", ctypes.c_uint8),
        ("ambient_measured", ctypes.c_uint8),
        ("balance_recovered", ctypes.c_uint8),
        ("discharge_authorized", ctypes.c_uint8),
        ("charger_authorized", ctypes.c_uint8),
        ("regen_authorized", ctypes.c_uint8),
    ]


class CConfig(ctypes.Structure):
    _fields_ = [
        ("cell_uv_operating_v", ctypes.c_float),
        ("cell_ov_operating_v", ctypes.c_float),
        ("soc_min", ctypes.c_float),
        ("soc_max", ctypes.c_float),
        ("discharge_core_temp_max_c", ctypes.c_float),
        ("discharge_surface_temp_max_c", ctypes.c_float),
        ("charge_core_temp_max_c", ctypes.c_float),
        ("charge_surface_temp_max_c", ctypes.c_float),
        ("charge_temp_min_c", ctypes.c_float),
        ("discharge_current_max_a", ctypes.c_float * len(HORIZONS)),
        ("charge_current_max_a", ctypes.c_float * len(HORIZONS)),
        ("horizons_s", ctypes.c_float * len(HORIZONS)),
        ("cell_capacity_ah", ctypes.c_float),
        ("parallel_cells", ctypes.c_float),
        ("r2_ohm", ctypes.c_float),
        ("c2_f", ctypes.c_float),
        ("core_thermal_capacity_j_per_k", ctypes.c_float),
        ("surface_thermal_capacity_j_per_k", ctypes.c_float),
        ("core_surface_resistance_k_per_w", ctypes.c_float),
        ("surface_ambient_resistance_k_per_w", ctypes.c_float),
        ("sigma_multiplier", ctypes.c_float),
        ("cell_voltage_measurement_uncertainty_v", ctypes.c_float),
        ("model_voltage_margin_v", ctypes.c_float),
        ("temperature_measurement_uncertainty_c", ctypes.c_float),
        ("model_temperature_margin_c", ctypes.c_float),
        ("current_uncertainty_floor_a", ctypes.c_float),
        ("max_innovation_per_cell_v", ctypes.c_float),
        ("max_measurement_age_ms", ctypes.c_float),
        ("default_capacity_soh_lower", ctypes.c_float),
        ("default_resistance_soh_upper", ctypes.c_float),
        ("fine_step_s", ctypes.c_float),
        ("medium_step_s", ctypes.c_float),
        ("coarse_step_s", ctypes.c_float),
        ("discharge_rise_rate_a_per_s", ctypes.c_float),
        ("charge_rise_rate_a_per_s", ctypes.c_float),
        ("discharge_voltage_recovery_a_per_s", ctypes.c_float),
        ("charge_voltage_recovery_a_per_s", ctypes.c_float),
        ("discharge_thermal_recovery_a_per_s", ctypes.c_float),
        ("charge_thermal_recovery_a_per_s", ctypes.c_float),
        ("discharge_current_path_recovery_a_per_s", ctypes.c_float),
        ("charge_current_path_recovery_a_per_s", ctypes.c_float),
        ("soc_recovery_rate_a_per_s", ctypes.c_float),
        ("soc_recovery_delta", ctypes.c_float),
        ("soc_recovery_charge_as", ctypes.c_float),
    ]


class CExtrema(ctypes.Structure):
    _fields_ = [
        ("minimum_cell_voltage_v", ctypes.c_float),
        ("maximum_cell_voltage_v", ctypes.c_float),
        ("minimum_soc", ctypes.c_float),
        ("maximum_soc", ctypes.c_float),
        ("maximum_core_temp_c", ctypes.c_float),
        ("maximum_surface_temp_c", ctypes.c_float),
        ("pack_voltage_v", ctypes.c_float),
    ]


class CResult(ctypes.Structure):
    _fields_ = [
        ("model_discharge_current_a", ctypes.c_float * len(HORIZONS)),
        ("model_charge_current_a", ctypes.c_float * len(HORIZONS)),
        ("discharge_current_a", ctypes.c_float * len(HORIZONS)),
        ("charge_current_a", ctypes.c_float * len(HORIZONS)),
        ("discharge_power_w", ctypes.c_float * len(HORIZONS)),
        ("charge_power_w", ctypes.c_float * len(HORIZONS)),
        ("discharge_binding", ctypes.c_int * len(HORIZONS)),
        ("charge_binding", ctypes.c_int * len(HORIZONS)),
        ("discharge_limiting_segment", ctypes.c_uint8 * len(HORIZONS)),
        ("discharge_limiting_cell", ctypes.c_uint8 * len(HORIZONS)),
        ("charge_limiting_segment", ctypes.c_uint8 * len(HORIZONS)),
        ("charge_limiting_cell", ctypes.c_uint8 * len(HORIZONS)),
        ("discharge_extrema", CExtrema * len(HORIZONS)),
        ("charge_extrema", CExtrema * len(HORIZONS)),
        ("measurement_sequence", ctypes.c_uint32),
        ("measurement_timestamp_ms", ctypes.c_uint32),
        ("solve_timestamp_ms", ctypes.c_uint32),
        ("reason_flags", ctypes.c_uint32),
        ("feasibility_evaluations", ctypes.c_uint32),
        ("prediction_steps", ctypes.c_uint32),
        ("valid", ctypes.c_uint8),
        ("authority_valid", ctypes.c_uint8),
        ("fallback_active", ctypes.c_uint8),
    ]


def to_c_input(source: Input) -> CInput:
    target = CInput()
    target.measurement_sequence = 42
    target.measurement_timestamp_ms = 950
    target.now_ms = 1000
    target.pack_current_a = source.pack_current_a
    target.pack_current_uncertainty_a = source.pack_current_uncertainty_a
    target.ambient_temp_c = source.ambient_temp_c
    target.operating_mode = 1  # AMS_SOP_MODE_DRIVE
    target.measurement_valid = 1
    target.estimator_valid = 1
    target.estimator_segment_topology = 1
    target.current_calibrated = 1
    target.current_polarity_validated = 1
    target.ambient_measured = 1
    target.balance_recovered = 1
    target.discharge_authorized = 1
    target.regen_authorized = 1
    for index, segment in enumerate(source.segments):
        out = target.segment[index]
        out.soc = segment.soc
        out.vp1_v = segment.vp1_v
        out.vp2_v = segment.vp2_v
        out.r0_ohm = segment.r0_ohm
        out.core_temp_c = segment.core_temp_c
        out.surface_max_temp_c = segment.surface_temp_c
        out.p_soc = segment.p_soc
        out.p_vp1 = segment.p_vp1
        out.p_vp2 = segment.p_vp2
        out.p_r0 = segment.p_r0
        out.innovation_v = segment.innovation_v
        out.capacity_soh_lower = segment.capacity_soh_lower
        out.resistance_soh_upper = segment.resistance_soh_upper
        out.max_cell_age_ms = 0
        out.cell_usable_mask = 0x7FFF
        out.estimator_valid = 1
        out.capacity_soh_valid = int(segment.capacity_soh_valid)
        out.resistance_soh_valid = int(segment.resistance_soh_valid)
        for cell, voltage in enumerate(segment.cells_v):
            out.cell_voltage_v[cell] = voltage
    return target


def random_input(rng: random.Random, calibration: Calibration) -> Input:
    pack_current = rng.uniform(-8.0, 55.0)
    uncertainty = rng.uniform(0.5, 2.0)
    ambient = rng.uniform(8.0, 36.0)
    segments: list[Segment] = []
    for _ in range(SEGMENTS):
        soc = rng.uniform(0.13, 0.93)
        core = rng.uniform(max(8.0, ambient - 2.0), min(41.0, ambient + 10.0))
        surface = rng.uniform(max(7.0, core - 4.0), min(40.0, core + 1.0))
        vp1 = rng.uniform(-0.025, 0.045)
        vp2 = rng.uniform(-0.010, 0.020)
        lut_r0 = calibration.r0_ohm(soc, core)
        growth = rng.uniform(1.0, 1.45)
        r0 = lut_r0 * rng.uniform(0.98, growth)
        cell_current = pack_current / 6.0
        measured_base = (calibration.ocv_v(soc, core) - vp1 - vp2
                         - r0 * cell_current)
        offsets = [rng.uniform(-0.025, 0.025) for _ in range(CELLS_PER_SEGMENT)]
        cells = [max(2.85, min(4.14, measured_base + offset)) for offset in offsets]
        segments.append(
            Segment(
                soc=soc,
                vp1_v=vp1,
                vp2_v=vp2,
                r0_ohm=r0,
                core_temp_c=core,
                surface_temp_c=surface,
                p_soc=rng.uniform(1.0e-7, 2.5e-5),
                p_vp1=rng.uniform(1.0e-8, 8.0e-6),
                p_vp2=rng.uniform(1.0e-8, 8.0e-6),
                p_r0=rng.uniform(1.0e-10, 2.0e-7),
                innovation_v=rng.uniform(-0.08, 0.08),
                capacity_soh_lower=rng.uniform(0.70, 1.0),
                resistance_soh_upper=growth,
                cells_v=cells,
            )
        )
    return Input(pack_current, uncertainty, ambient, segments)


class ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.calibration = Calibration()
        cls.tempdir = tempfile.TemporaryDirectory(prefix="der26_sop_oracle_")
        library = Path(cls.tempdir.name) / "libams_sop.so"
        command = [
            "gcc", "-std=c11", "-O2", "-fPIC", "-shared",
            "-Wall", "-Wextra", "-Werror",
            f"-I{AMS / 'Core/Inc'}",
            str(AMS / "Core/Src/sop/ams_sop.c"),
            str(AMS / "Core/Src/estimator/ams_estimator_lut.c"),
            "-lm", "-o", str(library),
        ]
        subprocess.run(command, check=True, capture_output=True, text=True)
        cls.lib = ctypes.CDLL(str(library))
        cls.lib.ams_sop_default_config.argtypes = [ctypes.POINTER(CConfig)]
        cls.lib.ams_sop_solve.argtypes = [
            ctypes.POINTER(CInput), ctypes.POINTER(CConfig),
            ctypes.POINTER(CResult),
        ]
        cls.lib.ams_sop_solve.restype = ctypes.c_int
        cls.c_config = CConfig()
        cls.lib.ams_sop_default_config(ctypes.byref(cls.c_config))

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tempdir.cleanup()

    def c_solve(self, source: Input) -> CResult:
        c_input = to_c_input(source)
        result = CResult()
        status = self.lib.ams_sop_solve(
            ctypes.byref(c_input), ctypes.byref(self.c_config),
            ctypes.byref(result)
        )
        self.assertEqual(status, 0)
        self.assertEqual(result.valid, 1)
        return result

    def compare_result(self, source: Input, current_tolerance: float = 0.035,
                       power_tolerance_w: float = 12.0) -> None:
        oracle = solve(source, Config(), self.calibration)
        target = self.c_solve(source)
        for horizon in range(len(HORIZONS)):
            self.assertAlmostEqual(
                target.model_discharge_current_a[horizon],
                oracle.model_discharge_current_a[horizon],
                delta=current_tolerance,
            )
            self.assertAlmostEqual(
                target.model_charge_current_a[horizon],
                oracle.model_charge_current_a[horizon],
                delta=current_tolerance,
            )
            self.assertAlmostEqual(
                target.discharge_power_w[horizon],
                oracle.discharge_power_w[horizon],
                delta=power_tolerance_w,
            )
            self.assertAlmostEqual(
                target.charge_power_w[horizon],
                oracle.charge_power_w[horizon],
                delta=power_tolerance_w,
            )
        self.assertTrue(monotonic(target.model_discharge_current_a, 0.01))
        self.assertTrue(monotonic(target.model_charge_current_a, 0.01))

    def test_hil_calibration_is_complete_and_self_consistent(self) -> None:
        self.assertIn("HiL/esp32_plant", str(self.calibration.source))
        self.assertEqual(len(self.calibration.ocv_table), 303)
        self.assertEqual(len(self.calibration.r0_table), 36)
        for soc in self.calibration.ecm_soc:
            for temperature in self.calibration.temperature:
                derived = (self.calibration.inv_c1(soc, temperature)
                           / -self.calibration.neg_inv_tau1(soc, temperature))
                direct = self.calibration.r1_direct_ohm(soc, temperature)
                self.assertAlmostEqual(derived, direct, delta=2.0e-8)

    def test_nominal_c_python_differential(self) -> None:
        self.compare_result(nominal_input(self.calibration))

    def test_boundary_scenarios_c_python_differential(self) -> None:
        cases: list[Input] = []
        weak = nominal_input(self.calibration)
        weak.segments[2].soc = 0.18
        weak.segments[2].cells_v[7] = 2.96
        cases.append(weak)
        hot = nominal_input(self.calibration)
        hot.ambient_temp_c = 43.0
        hot.segments[4].core_temp_c = 51.0
        hot.segments[4].surface_temp_c = 50.5
        cases.append(hot)
        high_soc = nominal_input(self.calibration)
        for segment in high_soc.segments:
            segment.soc = 0.965
            segment.cells_v = [4.105] * CELLS_PER_SEGMENT
        cases.append(high_soc)
        aged = nominal_input(self.calibration)
        for segment in aged.segments:
            segment.capacity_soh_lower = 0.68
            segment.resistance_soh_upper = 1.65
        cases.append(aged)
        for case in cases:
            with self.subTest(case=cases.index(case)):
                self.compare_result(case)

    def test_seeded_random_c_python_differential(self) -> None:
        rng = random.Random(20260722)
        for index in range(24):
            with self.subTest(index=index):
                self.compare_result(random_input(rng, self.calibration),
                                    current_tolerance=0.05,
                                    power_tolerance_w=18.0)

    def test_oracle_limit_is_feasible_and_next_bin_is_not(self) -> None:
        source = nominal_input(self.calibration)
        for segment in source.segments:
            segment.soc = 0.20
            segment.cells_v = [3.12] * CELLS_PER_SEGMENT
        result = solve(source, calibration=self.calibration)
        quantisation = (max(Config().discharge_current_max_a) /
                        (2 ** BISECTION_ITERATIONS))
        for index, horizon in enumerate(HORIZONS):
            at_limit = evaluate_current(
                source, Config(), self.calibration,
                result.model_discharge_current_a[index], True, horizon
            )
            self.assertTrue(at_limit.feasible)
            if result.discharge_binding[index] != Binding.CURRENT_PATH:
                above = evaluate_current(
                    source, Config(), self.calibration,
                    result.model_discharge_current_a[index]
                    + max(0.005, quantisation * 2.0),
                    True, horizon,
                )
                self.assertFalse(above.feasible)

    def test_covariance_and_soh_are_conservative(self) -> None:
        baseline = nominal_input(self.calibration)
        base = solve(baseline, calibration=self.calibration)
        stressed = deepcopy(baseline)
        for segment in stressed.segments:
            segment.p_soc *= 25.0
            segment.p_vp1 *= 25.0
            segment.p_vp2 *= 25.0
            segment.p_r0 *= 25.0
            segment.capacity_soh_lower = 0.70
            segment.resistance_soh_upper = 1.60
        lower = solve(stressed, calibration=self.calibration)
        for index in range(len(HORIZONS)):
            self.assertLessEqual(lower.model_discharge_current_a[index],
                                 base.model_discharge_current_a[index] + 1.0e-9)
            self.assertLessEqual(fabs(lower.model_charge_current_a[index]),
                                 fabs(base.model_charge_current_a[index]) + 1.0e-9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
