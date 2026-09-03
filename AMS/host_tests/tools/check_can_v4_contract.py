#!/usr/bin/env python3
"""Static DER26-CAN-V4 AMS contract gate."""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
app = (ROOT / "Core/Inc/app.h").read_text()
logger = (ROOT / "Core/Inc/ext_drivers/ams_can_logger.h").read_text()
profile = (ROOT / "Core/Inc/ams_build_profile.h").read_text()
canbus_h = (ROOT / "Core/Inc/ext_drivers/canbus.h").read_text()
canbus = (ROOT / "Core/Src/ext_drivers/canbus.c").read_text()
task = (ROOT / "Core/Src/tasks/canbus_task.c").read_text()
estimator = (ROOT / "Core/Inc/estimator/ams_soc_ekf.h").read_text()


def macro(text: str, name: str) -> int:
    m = re.search(rf"^#define\s+{re.escape(name)}\s+(0x[0-9A-Fa-f]+|\d+)u?\b", text, re.M)
    if not m:
        raise SystemExit(f"FAIL missing numeric macro {name}")
    return int(m.group(1), 0)

required_names = [
    "AMS_ECU_CAN_ID_STATUS", "AMS_ECU_CAN_ID_ELECTRICAL",
    "AMS_ECU_CAN_ID_THERMAL", "AMS_ECU_CAN_ID_HEALTH",
    "AMS_ECU_CAN_ID_SOP_DCL", "AMS_ECU_CAN_ID_SOP_CCL",
    "AMS_ECU_CAN_ID_SOH", "AMS_ECU_CAN_ID_SOP_ENVELOPE",
]
required = [macro(app, n) for n in required_names]
if required != list(range(0x680, 0x688)):
    raise SystemExit(f"FAIL protected required IDs are {required}, expected 0x680..0x687")

advisory_names = [
    "AMS_ECU_CAN_ID_STRATEGY_STATUS", "AMS_ECU_CAN_ID_SOP_BINDINGS",
    "AMS_ECU_CAN_ID_CURRENT_DIAG",
]
advisory = [macro(app, n) for n in advisory_names]
if advisory != [0x689, 0x68A, 0x68B]:
    raise SystemExit(f"FAIL protected advisory IDs {advisory}")

# 0x688 is intentionally the existing ECU->AMS mission request/control ID.
if macro(app, "AMS_ECU_CAN_ID_MISSION_REQUEST") != 0x688:
    raise SystemExit("FAIL mission request ID must remain 0x688")

logger_ids = []
for name, value in re.findall(r"^#define\s+(AMS_LOGGER_CAN_ID_[A-Z0-9_]+)\s+(0x[0-9A-Fa-f]+)u\b", logger, re.M):
    logger_ids.append((name, int(value, 0)))
if not logger_ids:
    raise SystemExit("FAIL no logger IDs found")
for name, value in logger_ids:
    if value < 0x690:
        raise SystemExit(f"FAIL detail ID {name}=0x{value:03X} outranks protected class")
if macro(logger, "AMS_LOGGER_CAN_ID_ESTIMATOR") != 0x6B2:
    raise SystemExit("FAIL estimator diagnostics must be 0x6B2")
if macro(logger, "AMS_LOGGER_CAN_ID_ESTIMATOR_VOLTAGE_COMPARE") != 0x6B4:
    raise SystemExit("FAIL estimator voltage comparison diagnostics must be 0x6B4")
tuning_names = [
    "AMS_LOGGER_CAN_ID_EKF_STATE", "AMS_LOGGER_CAN_ID_EKF_MODEL",
    "AMS_LOGGER_CAN_ID_EKF_COVARIANCE", "AMS_LOGGER_CAN_ID_EKF_SOH",
    "AMS_LOGGER_CAN_ID_EKF_CONTEXT", "AMS_LOGGER_CAN_ID_SOP_DISCHARGE",
    "AMS_LOGGER_CAN_ID_SOP_CHARGE", "AMS_LOGGER_CAN_ID_SOP_BINDING",
    "AMS_LOGGER_CAN_ID_FUSE_STATE", "AMS_LOGGER_CAN_ID_FUSE_LIMIT",
    "AMS_LOGGER_CAN_ID_TUNING_META", "AMS_LOGGER_CAN_ID_SOP_META",
]
if [macro(logger, name) for name in tuning_names] != list(range(0x6B5, 0x6C1)):
    raise SystemExit("FAIL passive tuning IDs must remain contiguous 0x6B5..0x6C0")
if macro(logger, "AMS_LOGGER_PROTOCOL_VERSION") != 4:
    raise SystemExit("FAIL logger protocol must be v4 for full-covariance/acquisition pages")
if macro(logger, "AMS_TUNING_CAN_ACQ_PERIOD_MS") != 1000:
    raise SystemExit("FAIL acquisition tuning telemetry must remain at 1 Hz")
if "250-kbit/s image" not in logger or "AMS tuning CAN is prohibited at 250" not in logger:
    raise SystemExit("FAIL 250-kbit/s tuning compile exclusion is missing")
if macro(app, "AMS_LEGACY_TELEM_CAN_ID") != 0x6B1:
    raise SystemExit("FAIL legacy compatibility ID must be relocated to 0x6B1")
if macro(canbus_h, "AMS_ECU_DIAG_FEEDBACK_CAN_ID") != 0x6F0:
    raise SystemExit("FAIL ECU->AMS diagnostic feedback must be 0x6F0")

if not re.search(r"#define\s+AMS_ENABLE_LEGACY_CAN_TELEMETRY\s+0\b", profile):
    raise SystemExit("FAIL legacy compatibility telemetry must default disabled")
if "vehicle build forbids legacy bulk telemetry" not in profile:
    raise SystemExit("FAIL vehicle build lacks legacy-telemetry compile gate")
if "hcan1.Init.TransmitFifoPriority = DISABLE" not in (ROOT / "Core/Src/main.c").read_text():
    raise SystemExit("FAIL TXFP must remain disabled so CAN-ID arbitration is authoritative")
if "CAN_TX_TIMEOUT_TICKS" in canbus:
    raise SystemExit("FAIL historical 1-tick mailbox timeout remains in production canbus.c")
if re.search(r"HAL_CAN_GetTxMailboxesFreeLevel[^\n]*\n(?:.|\n){0,300}?osDelay\(", canbus):
    raise SystemExit("FAIL production TX path still polls mailboxes with osDelay")
if "AMS_CAN_DETAIL_FULL_PERIOD_MS 500u" not in profile:
    raise SystemExit("FAIL detail snapshot period is not 500 ms / 2 Hz")
if "AMS_CAN_PROTECTED_PERIOD_MS 100u" not in profile:
    raise SystemExit("FAIL protected publication period is not 100 ms / 10 Hz")
if "canbus_publish_protected_generation" not in task or "canbus_publish_detail_snapshot" not in task:
    raise SystemExit("FAIL CAN task does not build V4 protected/detail publications")
if "canbus_publish_tuning_fast" not in task or "CANBUS_TX_BUILD_DETAIL" not in task:
    raise SystemExit("FAIL tuning telemetry is not explicitly built as DETAIL traffic")
if "p_soc_vp1" not in task or "p_soc_vp2" not in task or "p_vp1_vp2" not in task:
    raise SystemExit("FAIL tuning CAN does not export full EKF cross-covariance")
if "send_tuning_acquisition_meta" not in task or "acquisition_candidate_soc" not in task:
    raise SystemExit("FAIL tuning CAN lacks per-segment acquisition diagnostics")
if "covariance_repair_count" not in task or "innovation_variance_v2" not in task:
    raise SystemExit("FAIL tuning CAN lacks covariance-repair / exact innovation-variance diagnostics")
if "canbus_tuning_health_guard" not in task or "can_tuning_suppressed" not in task:
    raise SystemExit("FAIL tuning stream lacks one-way runtime suppression guard")
if "ams_can_tx_publish_protected" not in canbus or "ams_can_tx_publish_detail" not in canbus:
    raise SystemExit("FAIL CAN driver does not commit publications into V4 TX scheduler")
if "HAL_CAN_TxMailbox0CompleteCallback" not in canbus or "canbus_tx_kick_from_isr" not in canbus:
    raise SystemExit("FAIL interrupt-driven TX refill callbacks are missing")

# Prevent accidental resurrection of old active production IDs. Strings/docs are
# allowed; numeric production definitions are not.
for path in [ROOT / "Core/Inc/app.h", ROOT / "Core/Inc/ext_drivers/ams_can_logger.h"]:
    for line in path.read_text().splitlines():
        if re.search(r"^#define\s+\w+\s+0x0?69u?\b", line):
            raise SystemExit(f"FAIL historical high-priority detail ID remains: {path.name}: {line}")
if re.search(r"^#define\s+\w+\s+0x421u?\b", estimator, re.M):
    raise SystemExit("FAIL historical 0x421 estimator diagnostic ID remains")

print("PASS DER26-CAN-V4 AMS ID/priority/scheduler publication contract")
