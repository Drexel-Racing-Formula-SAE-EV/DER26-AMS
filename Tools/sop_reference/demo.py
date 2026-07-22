"""Print the nominal DER26 predictive power envelope."""

from model import Calibration, nominal_input, solve


if __name__ == "__main__":
    calibration = Calibration()
    result = solve(nominal_input(calibration), calibration=calibration)
    for index, horizon in enumerate((0.1, 1.0, 10.0, 30.0)):
        print(
            f"{horizon:>4g} s  "
            f"DCL={result.model_discharge_current_a[index]:7.3f} A  "
            f"CCL={result.model_charge_current_a[index]:7.3f} A  "
            f"Pdis={result.discharge_power_w[index] / 1000.0:6.2f} kW  "
            f"Pchg={result.charge_power_w[index] / 1000.0:6.2f} kW  "
            f"bind={result.discharge_binding[index].name}/"
            f"{result.charge_binding[index].name}"
        )
