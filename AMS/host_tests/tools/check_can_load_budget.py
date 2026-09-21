#!/usr/bin/env python3
"""DER26-CAN-V4 conservative utilization planning gate.

This is intentionally conservative and is not a substitute for CANalyzer or
classical-CAN response-time analysis on the complete vehicle. 135 bits is a
worst-case-ish planning occupancy for an 8-byte standard frame including SOF,
identifier/control/data/CRC/ACK/EOF, IFS, and maximum practical stuffing
allowance. Typical captures should therefore measure lower utilization.

The release invariant is stronger than the percentage gate: protected AMS
traffic must meet freshness/deadline requirements at 250 kbps even if detail
snapshot rate degrades. 1 Mbit/s is the intended DER26 vehicle configuration.
"""

BITS_PER_STD_8B_FRAME_CONSERVATIVE = 135
BITRATES = (1_000_000, 500_000, 250_000)

# AMS protected/status/power publication: 11 frames every 100 ms.
AMS_PROTECTED_FPS = 11 * 10

# Five-SMB final topology logger: host contract currently encodes
# 21 common frames + 20 per SMB + 3 APM frames = 124, plus estimator status
# and the RAW/AVG8/IIR comparison frame.
AMS_DETAIL_FRAMES_PER_SNAPSHOT = 126
AMS_DETAIL_SNAPSHOT_HZ = 2
AMS_DETAIL_FPS = AMS_DETAIL_FRAMES_PER_SNAPSHOT * AMS_DETAIL_SNAPSHOT_HZ

# Passive test-day tuning is compiled into 1-Mbit/s and 500-kbit/s images:
# 0x6B5/0x6B6 = 10 frames at 10 Hz (five segments),
# 0x6B7..0x6B9 = 50 frames at 2 Hz (full covariance page included),
# 0x6BA..0x6BE plus 0x6C0 = 27 frames at 5 Hz,
# 0x6BF = 2 global metadata frames at 10 Hz, plus
# five per-segment acquisition records at 1 Hz.
AMS_TUNING_FPS = (12 * 10) + (50 * 2) + (27 * 5) + (5 * 1)

# Conservative charging command allowance. Only relevant in charge state.
AMS_CRITICAL_FPS = 10
AMS_BASE_FPS = AMS_PROTECTED_FPS + AMS_DETAIL_FPS + AMS_CRITICAL_FPS

# Whole-vehicle planning load: six ECU-required CM200 broadcasts at 100 Hz,
# the ECU 100 Hz command, plus optional A6 at 100 Hz and A0-A2/AE at 10 Hz.
CM200_REQUIRED_FPS = 6 * 100
ECU_COMMAND_FPS = 100
CM200_OPTIONAL_LOGGING_FPS = 100 + (3 * 10) + 10

# 1-Mbit/s release planning limits. 500/250 kbps are reported for fallback
# correctness, not required to sustain nominal 2 Hz detail under full load.
AMS_1M_LOAD_LIMIT = 0.10
WHOLE_OPTIONAL_1M_LOAD_LIMIT = 0.25


def load(fps: int, bitrate: int) -> float:
    return fps * BITS_PER_STD_8B_FRAME_CONSERVATIVE / bitrate


print("DER26-CAN-V4 conservative planning model")
print(f"  protected: {AMS_PROTECTED_FPS} frame/s")
print(f"  detail:    {AMS_DETAIL_FPS} frame/s "
      f"({AMS_DETAIL_FRAMES_PER_SNAPSHOT} frames x {AMS_DETAIL_SNAPSHOT_HZ} Hz)")
print(f"  critical:  {AMS_CRITICAL_FPS} frame/s allowance")
print(f"  tuning:    {AMS_TUNING_FPS} frame/s (1M/500 kbps)")
print(f"  AMS base:  {AMS_BASE_FPS} frame/s")

for bitrate in BITRATES:
    tuning_fps = AMS_TUNING_FPS if bitrate >= 500_000 else 0
    ams_total_fps = AMS_BASE_FPS + tuning_fps
    whole_required_fps = ams_total_fps + CM200_REQUIRED_FPS + ECU_COMMAND_FPS
    whole_optional_fps = whole_required_fps + CM200_OPTIONAL_LOGGING_FPS
    suffix = f"{bitrate // 1000} kbps"
    ams = load(ams_total_fps, bitrate)
    whole_req = load(whole_required_fps, bitrate)
    whole_opt = load(whole_optional_fps, bitrate)
    print(f"  {suffix}: AMS ~= {ams*100:.2f}%")
    print(f"  {suffix}: + required CM200/ECU ~= {whole_req*100:.2f}%")
    print(f"  {suffix}: + optional CM200 logging ~= {whole_opt*100:.2f}%")

ams_1m_fps = AMS_BASE_FPS + AMS_TUNING_FPS
whole_opt_1m_fps = (ams_1m_fps + CM200_REQUIRED_FPS + ECU_COMMAND_FPS +
                     CM200_OPTIONAL_LOGGING_FPS)
ams_1m = load(ams_1m_fps, 1_000_000)
whole_opt_1m = load(whole_opt_1m_fps, 1_000_000)
if ams_1m > AMS_1M_LOAD_LIMIT:
    raise SystemExit(
        f"FAIL AMS 1-Mbit/s planning load {ams_1m:.3f} > {AMS_1M_LOAD_LIMIT:.3f}")
if whole_opt_1m > WHOLE_OPTIONAL_1M_LOAD_LIMIT:
    raise SystemExit(
        "FAIL whole-vehicle optional 1-Mbit/s planning load "
        f"{whole_opt_1m:.3f} > {WHOLE_OPTIONAL_1M_LOAD_LIMIT:.3f}")

print("NOTE 250 kbps: 0x6B5-0x6C0 tuning is absent; protected/safety deadlines")
print("     remain required and base detail may degrade under full-bus load.")
print("NOTE mailbox response time must be validated with higher-priority interference;")
print("     nominal three-frame serialization is not an accepted latency proof.")
print("PASS DER26-CAN-V4 1-Mbit/s conservative utilization planning gate")
