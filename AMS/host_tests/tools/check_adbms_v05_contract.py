#!/usr/bin/env python3
"""Static contract gate for DER26 AMS v0.5 ADBMS6830 hardening.

This intentionally checks cross-translation-unit invariants that a local
_Static_assert cannot express without coupling the low-level ADBMS driver to
the application voltage-fault layer.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]

def read(rel):
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")

def macro(text, name, seen=None):
    if seen is None:
        seen = set()
    if name in seen:
        raise AssertionError(f"recursive macro {name}")
    seen.add(name)
    matches = list(re.finditer(rf"^\s*#define\s+{re.escape(name)}\s+([^\s/]+)", text, re.M))
    if not matches:
        raise AssertionError(f"missing macro {name}")
    # For a default #ifndef definition, the last textual define is the value
    # that matters to this source-contract gate.
    v = matches[-1].group(1).rstrip("uUlL")
    try:
        return int(v, 0)
    except ValueError:
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", v):
            return macro(text, v, seen)
        return v

build = read("Core/Inc/ams_build_profile.h")
voltage_h = read("Core/Inc/ext_drivers/voltage_fault.h")
adbms_c = read("Core/Src/ext_drivers/adbms6830.c")
acc_c = read("Core/Src/ext_drivers/accumulator.c")
main_c = read("Core/Src/main.c")
est_c = read("Core/Src/tasks/estimator_task.c")
adbms_task_c = read("Core/Src/tasks/adbms_task.c")
cli_c = read("Core/Src/tasks/cli_task.c")

session = macro(build, "AMS_ENABLE_ADBMS_AWAKE_SESSION")
guard = macro(build, "AMS_ADBMS_SESSION_GUARD_US")
read_confirm = macro(voltage_h, "VOLTAGE_READ_FAULT_CONFIRM_SCANS")
spi_div = macro(build, "AMS_ADBMS_SPI_PRESCALER_DIV")
fc = macro(build, "AMS_ADBMS_IIR_FC")
s_eco = macro(build, "AMS_S_PATH_ECO_VALIDATED")
s_diag = macro(build, "AMS_ENABLE_PERIODIC_S_DIAGNOSTIC")
est_src = macro(build, "AMS_ESTIMATOR_VOLTAGE_SOURCE")

if session:
    assert isinstance(read_confirm, int) and read_confirm >= 3, (
        "awake-session transport is inseparable from >=3-scan persistent read-fault qualification")
assert isinstance(guard, int) and 0 < guard < 4300, "session guard must be within 0..tIDLE_min"
assert spi_div in (64, 128, 256), "SPI experiment must remain /64,/128,/256"
assert isinstance(fc, int) and 0 <= fc <= 7, "FC must fit 3 bits"
assert not (s_diag and not s_eco), "periodic S diagnostic cannot precede S-path ECO validation"
assert est_src in (0, 1, 2), "estimator voltage source must be RAW/AVG8/IIR"

assert "rx_pec_error" not in adbms_c, "dead last-IC-wins rx_pec_error must stay deleted"
assert re.search(r"tx_cfga\.fc\s*=.*AMS_ADBMS_IIR_FC", adbms_c), "IIR FC must be explicitly configured"
assert "adbms6830_mute_checked" in acc_c and "accumulator_emergency_balance_inhibit" in acc_c, (
    "MUTE-first durable balance inhibit implementation missing")
assert "last_balance_durable_zero_verified" in acc_c, "durable-zero balance qualification missing"
assert "SPI_BAUDRATEPRESCALER_64" in main_c and "SPI_BAUDRATEPRESCALER_128" in main_c and "SPI_BAUDRATEPRESCALER_256" in main_c, (
    "SPI /64,/128,/256 compile-time experiment selector missing")
assert "AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && AMS_S_PATH_ECO_VALIDATED" in adbms_c, (
    "post-ECO S path must be doubly gated")
assert "measurement->cell_mv" in est_c and "AMS_ESTIMATOR_VOLTAGE_SOURCE" in est_c, (
    "estimator source selector/raw path missing")
assert "estimator_capture_voltage_products" in est_c and "voltage_compare_sequence" in est_c, (
    "simultaneous RAW/AVG8/IIR estimator comparison capture missing")
assert "estimator_selected_voltage" in est_c, (
    "live EKF input must select one captured product without changing raw safety")
# Raw safety normalization remains sourced from primary cell.c_codes. This is
# deliberately not allowed to switch to acell/fcell under a build option.
assert re.search(r"smb_ics\[[^]]+\]\.cell\.c_codes\[[^]]+\]", acc_c), (
    "raw C codes must remain present in accumulator safety normalization")

# Schedulability hardening: multi-ms conversion/settling waits must go through
# the cooperative wait abstraction. Direct literal busy waits above 1 ms are
# forbidden in the ADBMS driver; the 1 ms wake pulses remain protocol timing.
for m in re.finditer(r"adbms6830_us_delay\s*\([^,]+,\s*(\d+)u\s*\)", adbms_c):
    value = int(m.group(1))
    assert value <= 1000, f"direct {value} us ADBMS busy wait bypasses cooperative scheduler"
assert "adbms6830_wait_cooperative(dev," in adbms_c and "ADBMS6830_REDUNDANT_CONVERSION_WAIT_US" in adbms_c, "17 ms C/S conversion must yield"
assert "ADBMS6830_AUX_CONVERSION_WAIT_US" in adbms_c and "ADBMS6830_OPEN_WIRE_CONVERSION_WAIT_US" in adbms_c, "AUX/open-wire conversion waits must use cooperative scheduling"
assert "adbms_task_cooperative_wait" in adbms_task_c, "ADBMS task cooperative wait hook missing"
assert "DWT->CYCCNT" in adbms_task_c, "target DWT timing instrumentation missing"
assert "heartbeat.count[AMS_HEARTBEAT_CAN]" in cli_c and "heartbeat.count[AMS_HEARTBEAT_IMD]" in cli_c, (
    "task-starvation acceptance counters missing from CLI")
assert "guard is deliberately advisory" in adbms_c, "session guard preemption caveat must remain documented"
assert "AMS_HEARTBEAT_CURRENT" in cli_c and "AMS_HEARTBEAT_TEMP" in cli_c, (
    "higher-priority/current and temperature task cadence must be observable during starvation A/B")

# Coherent raw safety epoch: Status C freshness and Status D ASIC OV/UV must be
# frozen alongside CxV. Optional AVG8/IIR failures must be product-local.
assert "adbms6830_capture_coherent_cadc_counter" in adbms_c, "coherent CCTS freshness proof missing"
assert "coherent_statc_read_count" in adbms_c and "coherent_statd_read_count" in adbms_c, (
    "coherent Status C/D observability missing")
assert "avg8_read_fail_count" in adbms_c and "adbms6830_invalidate_avg8_product" in adbms_c, (
    "AVG8 failure must withdraw only the optional product")
assert "filtered_read_fail_count" in adbms_c and "adbms6830_invalidate_filtered_product" in adbms_c, (
    "filtered failure must withdraw only the optional product")
assert "product_status == HAL_BUSY" in adbms_c, (
    "only session-coherency expiry from optional products may escalate to raw-epoch restart")

# New diagnostic products and safety separation.
assert "adbms6830_run_startup_post" in adbms_c and "FLAG_D" in adbms_c, "startup FLAG_D POST missing"
assert "ADBMS6830_POST_SPIFLT" in adbms_c and "RDSTATCERR" in adbms_c, (
    "startup POST must exercise the RDSTATC ERR/SPIFLT reporting path")
assert "adbms6830_verify_mute_state" in adbms_c and "mute_st" in adbms_c, (
    "MUTE/UNMUTE must be verified through RDCFGA MUTE_ST")
assert "mute_verify_fail_count" in cli_c and "unmute_verify_fail_count" in cli_c, (
    "MUTE_ST verification failures must be visible")
assert "adbms6830_run_aux2_redundancy" in adbms_c, "AUX2 redundancy path missing"
assert "adbms6830_run_thermistor_open_wire" in adbms_c, "targeted thermistor open-wire path missing"
assert re.search(r"soakon\s*=\s*1u", adbms_c) and re.search(r"owrng\s*=\s*0u", adbms_c) and re.search(r"owa\s*=\s*7u", adbms_c), (
    "thermistor OW must use the ASIC soak-before-convert mechanism")
assert "AUX_PUP_PULL_UP" in adbms_c or "0x80u" in adbms_c, (
    "thermistor OW must exercise both pull-down and pull-up stimuli")
assert "config_restore_fail_count" in cli_c, "thermistor OW config restore failure must be visible"
assert "publish_sample" in adbms_c, "diagnostic AUX path must preserve normal temperature ownership"
assert "RDACA" in adbms_c and "RDFCA" in adbms_c, "AVG8/IIR group read products missing"
can_task_c = read("Core/Src/tasks/canbus_task.c")
logger_h = read("Core/Inc/ext_drivers/ams_can_logger.h")
assert "send_estimator_voltage_compare" in can_task_c, (
    "passive RAW/AVG8/IIR comparison telemetry missing")
assert "AMS_LOGGER_CAN_ID_ESTIMATOR_VOLTAGE_COMPARE" in logger_h and "0x6B4u" in logger_h, (
    "estimator comparison detail ID 0x6B4 missing")
assert "RDCVALL" not in adbms_c and "RDACALL" not in adbms_c and "RDFCALL" not in adbms_c, (
    "single-IC Read-All commands must not be used on the daisy-chain driver")
assert "accumulator_final_ring_topology_valid(acc)" in adbms_task_c, (
    "periodic ADBMS diagnostics must be gated by final-ring transport/topology readiness")
assert "adbms_aux2_next_due_tick" in adbms_task_c and "+= aux2_period_ms" in adbms_task_c, (
    "AUX2 cadence must use an absolute next-due schedule instead of last=now quantization")

# Durable zero, not transient MUTE alone, is the BMS_OK balance-safe proof.
error_c = read("Core/Src/tasks/error_task.c")
assert "adbms_balance_durable_zero_verified" in error_c, "BMS supervisor durable-zero gate missing"
assert "last_balance_durable_zero_verified" in acc_c, "accumulator durable-zero state missing"

print("PASS ADBMS v0.5 cross-module contract")
print(f"  session={session} guard={guard}us read_fault_confirm={read_confirm}")
print(f"  SPI=/{spi_div} FC={fc} estimator_source={est_src} S_ECO={s_eco} S_diag={s_diag}")
