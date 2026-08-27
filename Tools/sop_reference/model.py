"""Independent desktop oracle for the DER26 robust finite-horizon SoP solver.

The embedded implementation reads its calibration through
``AMS/Core/Src/estimator/ams_estimator_lut.c``.  This oracle deliberately
parses the separately generated HIL plant constants instead.  The equations
are reimplemented in Python so differential tests can detect target-code,
table-layout, discretisation, and bisection errors.

Positive current is accumulator discharge; negative current is charge/regen.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass, field
from enum import IntEnum
from math import exp, fabs, inf, isfinite, sqrt
from pathlib import Path
import re
from typing import Iterable

SEGMENTS = 5
CELLS_PER_SEGMENT = 15
HORIZONS = (0.1, 1.0, 10.0, 30.0)
BISECTION_ITERATIONS = 16


class Binding(IntEnum):
    NONE = 0
    CELL_UV = 1
    CELL_OV = 2
    SOC_LOW = 3
    SOC_HIGH = 4
    CORE_TEMP = 5
    SURFACE_TEMP = 6
    CHARGE_TEMP_LOW = 7
    CURRENT_PATH = 8
    DIRECTION_INHIBIT = 9
    MODEL_DOMAIN = 10
    INVALID_INPUT = 11
    HORIZON_ENVELOPE = 12


@dataclass
class Segment:
    soc: float = 0.55
    vp1_v: float = 0.0
    vp2_v: float = 0.0
    r0_ohm: float = 0.014
    core_temp_c: float = 28.0
    surface_temp_c: float = 27.0
    p_soc: float = 1.0e-6
    p_vp1: float = 1.0e-7
    p_vp2: float = 1.0e-7
    p_r0: float = 1.0e-9
    innovation_v: float = 0.0
    capacity_soh_lower: float = 1.0
    resistance_soh_upper: float = 1.05
    capacity_soh_valid: bool = True
    resistance_soh_valid: bool = True
    cells_v: list[float] = field(
        default_factory=lambda: [3.75] * CELLS_PER_SEGMENT
    )


@dataclass
class Input:
    pack_current_a: float = 0.0
    pack_current_uncertainty_a: float = 0.5
    ambient_temp_c: float = 27.0
    segments: list[Segment] = field(
        default_factory=lambda: [Segment() for _ in range(SEGMENTS)]
    )


@dataclass
class Config:
    cell_uv_operating_v: float = 2.80
    cell_ov_operating_v: float = 4.15
    soc_min: float = 0.05
    soc_max: float = 0.98
    discharge_core_temp_max_c: float = 55.0
    discharge_surface_temp_max_c: float = 55.0
    charge_core_temp_max_c: float = 42.0
    charge_surface_temp_max_c: float = 42.0
    charge_temp_min_c: float = 3.0
    discharge_current_max_a: tuple[float, ...] = (118.0, 80.0, 70.0, 70.0)
    charge_current_max_a: tuple[float, ...] = (11.5, 10.0, 10.0, 10.0)
    horizons_s: tuple[float, ...] = HORIZONS
    cell_capacity_ah: float = 4.20
    parallel_cells: float = 6.0
    r2_ohm: float = 0.004
    c2_f: float = 12000.0
    core_thermal_capacity_j_per_k: float = 55.0
    surface_thermal_capacity_j_per_k: float = 15.0
    core_surface_resistance_k_per_w: float = 1.5
    surface_ambient_resistance_k_per_w: float = 8.0
    sigma_multiplier: float = 3.0
    cell_voltage_measurement_uncertainty_v: float = 0.005
    model_voltage_margin_v: float = 0.020
    temperature_measurement_uncertainty_c: float = 1.5
    model_temperature_margin_c: float = 1.5
    current_uncertainty_floor_a: float = 0.50
    default_capacity_soh_lower: float = 0.80
    default_resistance_soh_upper: float = 1.25
    fine_step_s: float = 0.10
    medium_step_s: float = 0.50
    coarse_step_s: float = 1.00


@dataclass
class Evaluation:
    feasible: bool
    binding: Binding
    limiting_segment: int = 0xFF
    limiting_cell: int = 0xFF
    minimum_cell_voltage_v: float = inf
    maximum_cell_voltage_v: float = -inf
    minimum_soc: float = inf
    maximum_soc: float = -inf
    maximum_core_temp_c: float = -inf
    maximum_surface_temp_c: float = -inf
    pack_voltage_v: float = 0.0
    steps: int = 0


@dataclass
class Result:
    model_discharge_current_a: list[float]
    model_charge_current_a: list[float]
    discharge_power_w: list[float]
    charge_power_w: list[float]
    discharge_binding: list[Binding]
    charge_binding: list[Binding]


@dataclass
class _ModelSegment:
    soc: float
    vp1_v: float
    vp2_v: float
    core_temp_c: float
    surface_temp_c: float
    soc0: float
    input_core_temp_c: float
    ocv0_v: float
    docv_dsoc_v: float
    docv_dtemp_v_per_c: float
    r0_upper_ohm: float
    r1_ohm: float
    tau1_s: float
    capacity_as: float
    voltage_margin_v: float
    cell_bias_v: list[float]
    p_soc: float


class Calibration:
    """P42A data loaded from the independent generated HIL plant."""

    _ARRAYS = {
        "ocv_table": "rtCP_pooled_2VS2GaLG7tXs",
        "ocv_soc": "rtCP_pooled_LAE26UlapTkv",
        "temperature": "rtCP_pooled_aZWafQUW4ySs",
        "r0_table": "rtCP_pooled_MJJiVyV6u4B7",
        "ecm_soc": "rtCP_pooled_oSJNQ9HBXQTR",
        "r1_direct_table": "rtCP_pooled_3i7E1u0kL9f2",
        "inv_c1_table": "rtCP_pooled_wGkYb7XWb2i2",
        "neg_inv_tau1_table": "rtCP_pooled_x01aWAzB59fl",
    }

    def __init__(self, source: Path | None = None):
        if source is None:
            root = Path(__file__).resolve().parents[2]
            source = root / "HiL/esp32_plant/components/plant_model/const_params.c"
        self.source = source
        text = source.read_text(encoding="utf-8")
        for attr, symbol in self._ARRAYS.items():
            setattr(self, attr, self._parse_array(text, symbol))
        expected = {
            "ocv_table": 303,
            "ocv_soc": 101,
            "temperature": 3,
            "r0_table": 36,
            "ecm_soc": 12,
            "r1_direct_table": 36,
            "inv_c1_table": 36,
            "neg_inv_tau1_table": 36,
        }
        for name, count in expected.items():
            values = getattr(self, name)
            if len(values) != count or not all(isfinite(v) for v in values):
                raise ValueError(f"invalid {name} calibration: {len(values)} values")

    @staticmethod
    def _parse_array(text: str, symbol: str) -> tuple[float, ...]:
        match = re.search(
            rf"const\s+real32_T\s+{re.escape(symbol)}\[\d+\]\s*=\s*"
            rf"\{{(.*?)\}}\s*;",
            text,
            flags=re.DOTALL,
        )
        if match is None:
            raise ValueError(f"missing HIL calibration symbol {symbol}")
        tokens = re.findall(
            r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?[Ff]?",
            match.group(1),
        )
        return tuple(float(token.rstrip("Ff")) for token in tokens)

    @staticmethod
    def _lut2d(
        x: float,
        y: float,
        x_breakpoints: tuple[float, ...],
        y_breakpoints: tuple[float, ...],
        table: tuple[float, ...],
    ) -> float:
        def bracket(value: float, points: tuple[float, ...]) -> tuple[int, float]:
            if value <= points[0]:
                return 0, 0.0
            if value >= points[-1]:
                return len(points) - 2, 1.0
            index = bisect_right(points, value) - 1
            fraction = (value - points[index]) / (points[index + 1] - points[index])
            return index, fraction

        ix, fx = bracket(x, x_breakpoints)
        iy, fy = bracket(y, y_breakpoints)
        nx = len(x_breakpoints)
        v00 = table[ix + iy * nx]
        v10 = table[ix + 1 + iy * nx]
        v01 = table[ix + (iy + 1) * nx]
        v11 = table[ix + 1 + (iy + 1) * nx]
        row0 = v00 + (v10 - v00) * fx
        row1 = v01 + (v11 - v01) * fx
        return row0 * (1.0 - fy) + row1 * fy

    def ocv_v(self, soc: float, temperature_c: float) -> float:
        return self._lut2d(soc, temperature_c, self.ocv_soc,
                           self.temperature, self.ocv_table)

    def r0_ohm(self, soc: float, temperature_c: float) -> float:
        return self._lut2d(soc, temperature_c, self.ecm_soc,
                           self.temperature, self.r0_table)

    def inv_c1(self, soc: float, temperature_c: float) -> float:
        return self._lut2d(soc, temperature_c, self.ecm_soc,
                           self.temperature, self.inv_c1_table)

    def neg_inv_tau1(self, soc: float, temperature_c: float) -> float:
        return self._lut2d(soc, temperature_c, self.ecm_soc,
                           self.temperature, self.neg_inv_tau1_table)

    def r1_direct_ohm(self, soc: float, temperature_c: float) -> float:
        return self._lut2d(soc, temperature_c, self.ecm_soc,
                           self.temperature, self.r1_direct_table)


def nominal_input(calibration: Calibration | None = None) -> Input:
    calibration = calibration or Calibration()
    value = calibration.r0_ohm(0.55, 28.0)
    return Input(segments=[Segment(r0_ohm=value) for _ in range(SEGMENTS)])


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _initialize_model(data: Input, cfg: Config, cal: Calibration) -> list[_ModelSegment]:
    present_cell_current = data.pack_current_a / cfg.parallel_cells
    model: list[_ModelSegment] = []
    for source in data.segments:
        lut_temp = _clamp(source.core_temp_c, 5.0, 40.0)
        lut_soc = _clamp(source.soc, 0.0, 1.0)
        inv_c1 = cal.inv_c1(lut_soc, lut_temp)
        neg_inv_tau = cal.neg_inv_tau1(lut_soc, lut_temp)
        inv_r1 = _clamp((-neg_inv_tau) / max(inv_c1, 1.0e-6), 0.0, 1000.0)
        r1 = 1.0 / inv_r1
        tau1 = -1.0 / neg_inv_tau
        lut_r0 = cal.r0_ohm(lut_soc, lut_temp)
        r0_sigma = cfg.sigma_multiplier * sqrt(source.p_r0)
        resistance_upper = (
            source.resistance_soh_upper
            if source.resistance_soh_valid
            else cfg.default_resistance_soh_upper
        )
        r0_state = max(source.r0_ohm, lut_r0)
        r0_upper = max(source.r0_ohm + r0_sigma, lut_r0 * resistance_upper)
        capacity_soh = (
            source.capacity_soh_lower
            if source.capacity_soh_valid
            else cfg.default_capacity_soh_lower
        )
        capacity_as = cfg.cell_capacity_ah * capacity_soh * 3600.0
        ocv0 = cal.ocv_v(lut_soc, lut_temp)
        soc_lo = _clamp(lut_soc - 0.005, 0.0, 1.0)
        soc_hi = _clamp(lut_soc + 0.005, 0.0, 1.0)
        docv_dsoc = (
            (cal.ocv_v(soc_hi, lut_temp) - cal.ocv_v(soc_lo, lut_temp))
            / (soc_hi - soc_lo)
            if soc_hi - soc_lo > 1.0e-6
            else 0.0
        )
        temp_lo = _clamp(lut_temp - 1.0, 5.0, 40.0)
        temp_hi = _clamp(lut_temp + 1.0, 5.0, 40.0)
        docv_dtemp = (
            (cal.ocv_v(lut_soc, temp_hi) - cal.ocv_v(lut_soc, temp_lo))
            / (temp_hi - temp_lo)
            if temp_hi - temp_lo > 1.0e-6
            else 0.0
        )
        sigma_voltage = cfg.sigma_multiplier * (
            fabs(docv_dsoc) * sqrt(source.p_soc)
            + sqrt(source.p_vp1)
            + sqrt(source.p_vp2)
            + fabs(present_cell_current) * sqrt(source.p_r0)
        )
        voltage_margin = (
            cfg.cell_voltage_measurement_uncertainty_v
            + cfg.model_voltage_margin_v
            + sigma_voltage
            + fabs(source.innovation_v) / CELLS_PER_SEGMENT
        )
        average_voltage = sum(source.cells_v) / CELLS_PER_SEGMENT
        model_voltage_now = (
            ocv0 - source.vp1_v - source.vp2_v - r0_state * present_cell_current
        )
        common_bias = average_voltage - model_voltage_now
        biases = [common_bias + value - average_voltage for value in source.cells_v]
        model.append(
            _ModelSegment(
                soc=source.soc,
                vp1_v=source.vp1_v,
                vp2_v=source.vp2_v,
                core_temp_c=max(source.core_temp_c, source.surface_temp_c),
                surface_temp_c=source.surface_temp_c,
                soc0=source.soc,
                input_core_temp_c=source.core_temp_c,
                ocv0_v=ocv0,
                docv_dsoc_v=docv_dsoc,
                docv_dtemp_v_per_c=docv_dtemp,
                r0_upper_ohm=r0_upper,
                r1_ohm=r1,
                tau1_s=tau1,
                capacity_as=capacity_as,
                voltage_margin_v=voltage_margin,
                cell_bias_v=biases,
                p_soc=source.p_soc,
            )
        )
    return model


def _thermal_step(core: float, surface: float, heat_w: float,
                  ambient: float, dt: float, cfg: Config) -> tuple[float, float]:
    a = 1.0 / (cfg.core_surface_resistance_k_per_w *
               cfg.core_thermal_capacity_j_per_k)
    b = 1.0 / (cfg.core_surface_resistance_k_per_w *
               cfg.surface_thermal_capacity_j_per_k)
    c = 1.0 / (cfg.surface_ambient_resistance_k_per_w *
               cfg.surface_thermal_capacity_j_per_k)
    half_dt = 0.5 * dt
    rhs_core = ((1.0 - half_dt * a) * core + half_dt * a * surface
                + dt * heat_w / cfg.core_thermal_capacity_j_per_k)
    rhs_surface = (half_dt * b * core + (1.0 - half_dt * (b + c)) * surface
                   + dt * c * ambient)
    m00 = 1.0 + half_dt * a
    m01 = -half_dt * a
    m10 = -half_dt * b
    m11 = 1.0 + half_dt * (b + c)
    determinant = m00 * m11 - m01 * m10
    return ((m11 * rhs_core - m01 * rhs_surface) / determinant,
            (-m10 * rhs_core + m00 * rhs_surface) / determinant)


def _step_size(elapsed: float, horizon: float, cfg: Config) -> float:
    if elapsed < 1.0:
        step, boundary = cfg.fine_step_s, 1.0
    elif elapsed < 10.0:
        step, boundary = cfg.medium_step_s, 10.0
    else:
        step, boundary = cfg.coarse_step_s, horizon
    step = min(step, horizon - elapsed)
    if boundary > elapsed:
        step = min(step, boundary - elapsed)
    return step


def evaluate_current(data: Input, cfg: Config, calibration: Calibration,
                     requested_pack_current_a: float, discharge: bool,
                     horizon_s: float) -> Evaluation:
    state = _initialize_model(data, cfg, calibration)
    result = Evaluation(feasible=True, binding=Binding.NONE)
    uncertainty = max(data.pack_current_uncertainty_a,
                      cfg.current_uncertainty_floor_a)
    conservative_pack_current = (
        requested_pack_current_a + uncertainty
        if discharge
        else requested_pack_current_a - uncertainty
    )
    cell_current = conservative_pack_current / cfg.parallel_cells
    elapsed = 0.0
    while elapsed < horizon_s - 1.0e-6:
        dt = _step_size(elapsed, horizon_s, cfg)
        pack_voltage = 0.0
        for segment_index, segment in enumerate(state):
            a1 = exp(-dt / segment.tau1_s)
            a2 = exp(-dt / (cfg.r2_ohm * cfg.c2_f))
            segment.vp1_v = (a1 * segment.vp1_v
                             + segment.r1_ohm * (1.0 - a1) * cell_current)
            segment.vp2_v = (a2 * segment.vp2_v
                             + cfg.r2_ohm * (1.0 - a2) * cell_current)
            segment.soc -= cell_current * dt / segment.capacity_as
            heat = cell_current * cell_current * (
                segment.r0_upper_ohm + segment.r1_ohm + cfg.r2_ohm
            )
            segment.core_temp_c, segment.surface_temp_c = _thermal_step(
                segment.core_temp_c, segment.surface_temp_c, heat,
                data.ambient_temp_c, dt, cfg
            )
            sigma_soc = cfg.sigma_multiplier * sqrt(segment.p_soc)
            conservative_soc = (segment.soc - sigma_soc if discharge
                                else segment.soc + sigma_soc)
            if discharge and conservative_soc < cfg.soc_min:
                result.feasible = False
                result.binding = Binding.SOC_LOW
                result.limiting_segment = segment_index
                return result
            if not discharge and conservative_soc > cfg.soc_max:
                result.feasible = False
                result.binding = Binding.SOC_HIGH
                result.limiting_segment = segment_index
                return result
            temperature_margin = (cfg.temperature_measurement_uncertainty_c
                                  + cfg.model_temperature_margin_c)
            core_upper = segment.core_temp_c + temperature_margin
            surface_upper = segment.surface_temp_c + temperature_margin
            if discharge:
                if core_upper > cfg.discharge_core_temp_max_c:
                    result.feasible = False
                    result.binding = Binding.CORE_TEMP
                    result.limiting_segment = segment_index
                    return result
                if surface_upper > cfg.discharge_surface_temp_max_c:
                    result.feasible = False
                    result.binding = Binding.SURFACE_TEMP
                    result.limiting_segment = segment_index
                    return result
            else:
                if segment.surface_temp_c - temperature_margin < cfg.charge_temp_min_c:
                    result.feasible = False
                    result.binding = Binding.CHARGE_TEMP_LOW
                    result.limiting_segment = segment_index
                    return result
                if core_upper > cfg.charge_core_temp_max_c:
                    result.feasible = False
                    result.binding = Binding.CORE_TEMP
                    result.limiting_segment = segment_index
                    return result
                if surface_upper > cfg.charge_surface_temp_max_c:
                    result.feasible = False
                    result.binding = Binding.SURFACE_TEMP
                    result.limiting_segment = segment_index
                    return result
            ocv = (segment.ocv0_v
                   + segment.docv_dsoc_v * (segment.soc - segment.soc0)
                   + segment.docv_dtemp_v_per_c
                   * (segment.core_temp_c - segment.input_core_temp_c))
            base_voltage = (ocv - segment.vp1_v - segment.vp2_v
                            - segment.r0_upper_ohm * cell_current)
            for cell_index, bias in enumerate(segment.cell_bias_v):
                voltage = base_voltage + bias
                result.minimum_cell_voltage_v = min(result.minimum_cell_voltage_v,
                                                    voltage)
                result.maximum_cell_voltage_v = max(result.maximum_cell_voltage_v,
                                                    voltage)
                pack_voltage += voltage
                if discharge and voltage - segment.voltage_margin_v < cfg.cell_uv_operating_v:
                    result.feasible = False
                    result.binding = Binding.CELL_UV
                    result.limiting_segment = segment_index
                    result.limiting_cell = cell_index
                    return result
                if not discharge and voltage + segment.voltage_margin_v > cfg.cell_ov_operating_v:
                    result.feasible = False
                    result.binding = Binding.CELL_OV
                    result.limiting_segment = segment_index
                    result.limiting_cell = cell_index
                    return result
            result.minimum_soc = min(result.minimum_soc, segment.soc)
            result.maximum_soc = max(result.maximum_soc, segment.soc)
            result.maximum_core_temp_c = max(result.maximum_core_temp_c,
                                             segment.core_temp_c)
            result.maximum_surface_temp_c = max(result.maximum_surface_temp_c,
                                                segment.surface_temp_c)
        result.pack_voltage_v = pack_voltage
        result.steps += 1
        elapsed += dt
    return result


def _solve_direction(data: Input, cfg: Config, calibration: Calibration,
                     horizon: float, current_cap: float,
                     discharge: bool) -> tuple[float, Evaluation]:
    zero = evaluate_current(data, cfg, calibration, 0.0, discharge, horizon)
    if not zero.feasible:
        return 0.0, zero
    signed_cap = current_cap if discharge else -current_cap
    at_cap = evaluate_current(data, cfg, calibration, signed_cap,
                              discharge, horizon)
    if at_cap.feasible:
        at_cap.binding = Binding.CURRENT_PATH
        at_cap.limiting_segment = 0xFF
        at_cap.limiting_cell = 0xFF
        return current_cap, at_cap
    lower, upper = 0.0, current_cap
    first_infeasible = at_cap
    for _ in range(BISECTION_ITERATIONS):
        candidate = 0.5 * (lower + upper)
        checked = evaluate_current(data, cfg, calibration,
                                   candidate if discharge else -candidate,
                                   discharge, horizon)
        if checked.feasible:
            lower = candidate
        else:
            upper = candidate
            first_infeasible = checked
    return lower, first_infeasible


def solve(data: Input, cfg: Config | None = None,
          calibration: Calibration | None = None) -> Result:
    cfg = cfg or Config()
    calibration = calibration or Calibration()
    discharge: list[float] = []
    charge: list[float] = []
    discharge_power: list[float] = []
    charge_power: list[float] = []
    discharge_binding: list[Binding] = []
    charge_binding: list[Binding] = []
    for index, horizon in enumerate(cfg.horizons_s):
        dcl, dlimit = _solve_direction(
            data, cfg, calibration, horizon,
            cfg.discharge_current_max_a[index], True
        )
        ccl_magnitude, climit = _solve_direction(
            data, cfg, calibration, horizon,
            cfg.charge_current_max_a[index], False
        )
        dfinal = evaluate_current(data, cfg, calibration, dcl, True, horizon)
        cfinal = evaluate_current(data, cfg, calibration, -ccl_magnitude,
                                  False, horizon)
        discharge.append(dcl)
        charge.append(-ccl_magnitude)
        discharge_power.append(max(0.0, dfinal.pack_voltage_v * dcl))
        charge_power.append(max(0.0, cfinal.pack_voltage_v * ccl_magnitude))
        discharge_binding.append(dlimit.binding)
        charge_binding.append(climit.binding)
    for index in range(1, len(cfg.horizons_s)):
        if discharge[index] > discharge[index - 1]:
            discharge[index] = discharge[index - 1]
            discharge_binding[index] = Binding.HORIZON_ENVELOPE
            checked = evaluate_current(data, cfg, calibration,
                                       discharge[index], True,
                                       cfg.horizons_s[index])
            discharge_power[index] = max(
                0.0, checked.pack_voltage_v * discharge[index]
            )
        if fabs(charge[index]) > fabs(charge[index - 1]):
            charge[index] = charge[index - 1]
            charge_binding[index] = Binding.HORIZON_ENVELOPE
            checked = evaluate_current(data, cfg, calibration,
                                       charge[index], False,
                                       cfg.horizons_s[index])
            charge_power[index] = max(
                0.0, checked.pack_voltage_v * fabs(charge[index])
            )
    return Result(discharge, charge, discharge_power, charge_power,
                  discharge_binding, charge_binding)


def monotonic(values: Iterable[float], tolerance: float = 1.0e-9) -> bool:
    magnitudes = [fabs(value) for value in values]
    return all(magnitudes[index] <= magnitudes[index - 1] + tolerance
               for index in range(1, len(magnitudes)))
