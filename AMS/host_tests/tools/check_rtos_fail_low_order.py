#!/usr/bin/env python3
"""Verify fatal RTOS hooks fail BMS_OK low before diagnostic bookkeeping."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
text = (ROOT / "Core/Src/ext_drivers/ams_rtos_diag.c").read_text()
safety = (ROOT / "Core/Src/ext_drivers/ams_safety.c").read_text()

panic = re.search(r"void\s+ams_safety_panic\s*\([^)]*\)\s*\{(.*?)\n\}", safety, re.S)
if not panic:
    raise SystemExit("FAIL cannot locate ams_safety_panic")
body = panic.group(1)
force_idx = body.find("ams_safety_force_bms_low();")
log_idx = body.find("ams_fault_log_event_raw_tick")
if force_idx < 0 or log_idx < 0 or force_idx > log_idx:
    raise SystemExit("FAIL ams_safety_panic must force BMS_OK low before panic logging")

checks = [
    ("vApplicationStackOverflowHook", "AMS_PANIC_RTOS_STACK_OVERFLOW", "ams_rtos_set_fault"),
    ("vApplicationMallocFailedHook", "AMS_PANIC_RTOS_MALLOC_FAILED", "ams_rtos_set_fault"),
    ("ams_rtos_assert_failed", "AMS_PANIC_RTOS_ASSERT_FAILED", "ams_rtos_set_fault"),
]
for func, reason, diagnostic in checks:
    m = re.search(rf"void\s+{func}\s*\([^)]*\)\s*\{{(.*?)\n\}}", text, re.S)
    if not m:
        raise SystemExit(f"FAIL cannot locate {func}")
    b = m.group(1)
    p = b.find(f"ams_safety_panic({reason});")
    d = b.find(diagnostic)
    if p < 0 or d < 0 or p > d:
        raise SystemExit(f"FAIL {func} performs diagnostic work before fail-low panic")

print("PASS fatal RTOS hooks force fail-low panic before diagnostic bookkeeping")
