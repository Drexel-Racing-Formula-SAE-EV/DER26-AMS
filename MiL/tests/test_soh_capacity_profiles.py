#!/usr/bin/env python3
"""Static arithmetic regression for production-capacity SoH qualification profiles.

MATLAB remains authoritative for plant/estimator execution. This test protects the
simple current-time arithmetic that makes the three capacity-gated release profiles
structurally knee-to-knee observable and keeps C5 charge current inside the checked-in
P42A continuous charge rating.
"""
from pathlib import Path
import math
import re

ROOT = Path(__file__).resolve().parents[2]
SCEN = ROOT / "MiL" / "matlab" / "configs" / "scenarios"


def require(cond, msg):
    if not cond:
        raise SystemExit(f"FAIL: {msg}")


def text(name):
    return (SCEN / name).read_text()


def direct_segments(src):
    dm = re.search(r"'duration_s',\[([^\]]+)\]", src)
    im = re.search(r"'current_A',\[([^\]]+)\]", src)
    require(dm and im, "direct segment vectors missing")
    duration = [float(x) for x in dm.group(1).split()]
    current = [float(x) for x in im.group(1).split()]
    require(len(duration) == len(current), "duration/current vector length mismatch")
    return duration, current


def net_ah(duration, current):
    return sum(d * i for d, i in zip(duration, current)) / 3600.0


# C5 nominal 25.2 Ah pack: 98% -> 9.8166% -> ~98% structural cycle.
c5 = text("c5_soh_capacity_resistance.m")
require("repmat([40 60],1,40)" in c5 and "r0_duration=ones(size(r0_pair))" in c5,
        "C5 proven 1 s R0 block changed")
require("'duration_s',[300 780 r0_duration 740 300 charge_duration 300]" in c5,
        "C5 discharge/rest structure changed")
require("charge_current=[-19 -23 -27 -31 -35 -31 -27 -23 -19 -15]" in c5,
        "C5 bounded-step charge ramp changed")
require("charge_duration=[60 60 60 60 1531.4 300 200 100 100 100]" in c5,
        "C5 charge-ramp timing changed")
require("'capacity_multiplier',1.0" in c5 and "'initial_soc_offset',0.0" in c5,
        "C5 positive capacity-observability balancing override missing")
require("cfg.reference_ekf.enabled=false" in c5 and "cfg.fuse.enabled=false" in c5,
        "C5 production-only runtime reduction changed")
require("cfg.acceptance.ekf.convergence_required=false" in c5,
        "C5 long-horizon convergence applicability changed")
require("cfg.acceptance.ekf.r0_accuracy_required=true" in c5 and "'EKF-R0'" in c5,
        "C5 raw R0 accuracy is no longer an explicit release gate")
q_nom = 25.2
discharge_ah = 50.0 * (780.0 + 740.0) / 3600.0 + ((40.0 + 60.0) / 2.0) * 80.0 / 3600.0
charge_d = [60, 60, 60, 60, 1531.4, 300, 200, 100, 100, 100]
charge_i = [19, 23, 27, 31, 35, 31, 27, 23, 19, 15]
charge_ah = sum(d * i for d, i in zip(charge_d, charge_i)) / 3600.0
low_soc = 0.98 - discharge_ah / q_nom
final_soc = low_soc + charge_ah / q_nom
require(math.isclose(discharge_ah, 22.2222222222, abs_tol=1e-9), "C5 discharge throughput drifted")
require(abs(charge_ah - discharge_ah) <= 5e-4, "C5 round-trip throughput no longer balances")
require(0.045 <= low_soc <= 0.105, f"C5 low anchor {low_soc:.5f} left confidence window")
require(0.975 <= final_soc <= 0.985, f"C5 return anchor {final_soc:.5f} left confidence window")
require(max(charge_i) / 6.0 < 8.4, "C5 peak charge current exceeds P42A per-cell charge rating")
# The localized block retains more than the 50 structural step opportunities
# required for resistance confidence without the 320 s estimator perturbation.
require((2 * 40 - 1) >= 50, "C5 no longer has enough localized structural R0 transitions")
# Every charge-ramp step is <5 A; the production resistance gate therefore
# cannot be repeatedly retriggered during the return-to-high-SOC leg.
require(all(abs(charge_i[k] - charge_i[k - 1]) < 5 for k in range(1, len(charge_i))),
        "C5 charge ramp contains an unintended >=5 A R0-identification step")

# Nominal capacity-window profile.
cap_window = text("soh_capacity_window.m")
d, i = direct_segments(cap_window)
q = 25.2
# Three zero-current anchors split the two excursions.
z = [idx for idx, value in enumerate(i) if value == 0.0]
require(len(z) == 3, "capacity-window profile no longer has exactly three structural rest anchors")
first_ah = net_ah(d[z[0] + 1:z[1]], i[z[0] + 1:z[1]])
second_ah = net_ah(d[z[1] + 1:z[2]], i[z[1] + 1:z[2]])
low = 0.98 - first_ah / q
back = low - second_ah / q
require(0.045 <= low <= 0.105 and 0.975 <= back <= 0.985,
        "nominal capacity-window anchors left demonstrated confidence windows")

# Aggregate 20% fade profile uses 20.16 Ah effective capacity and must land at
# the same structural anchor SoCs.
cap_only = text("soh_capacity_only.m")
d, i = direct_segments(cap_only)
q = 25.2 * 0.80
z = [idx for idx, value in enumerate(i) if value == 0.0]
require(len(z) == 3, "capacity-only profile no longer has exactly three structural rest anchors")
first_ah = net_ah(d[z[0] + 1:z[1]], i[z[0] + 1:z[1]])
second_ah = net_ah(d[z[1] + 1:z[2]], i[z[1] + 1:z[2]])
low = 0.98 - first_ah / q
back = low - second_ah / q
require(0.045 <= low <= 0.105 and 0.975 <= back <= 0.985,
        "20% fade capacity-only anchors left demonstrated confidence windows")

print("capacity SoH profile arithmetic regression: PASS")
