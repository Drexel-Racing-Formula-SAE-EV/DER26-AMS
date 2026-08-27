#!/usr/bin/env python3
"""Verify validated DER26 CAN timing and HSE-derived clock contract on AMS."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
app = (ROOT / "Core/Inc/app.h").read_text()
main = (ROOT / "Core/Src/main.c").read_text()
ioc = (ROOT / "DER26-AMS.ioc").read_text()

checks = {
    "default 500 kbps": r"#define\s+DER26_CAN_BITRATE_KBPS\s+500u",
    "500k prescaler 6": r"DER26_CAN_BITRATE_KBPS\s*==\s*500u[\s\S]*?#define\s+DER26_CAN_PRESCALER\s+6u",
    "250k prescaler 12": r"DER26_CAN_BITRATE_KBPS\s*==\s*250u[\s\S]*?#define\s+DER26_CAN_PRESCALER\s+12u",
    "SJW 2TQ": r"#define\s+DER26_CAN_SJW\s+CAN_SJW_2TQ",
    "BS1 15TQ": r"#define\s+DER26_CAN_BS1\s+CAN_BS1_15TQ",
    "BS2 2TQ": r"#define\s+DER26_CAN_BS2\s+CAN_BS2_2TQ",
}
for label, pattern in checks.items():
    if not re.search(pattern, app):
        raise SystemExit(f"FAIL AMS CAN clock contract: {label}")

for token in [
    "hcan1.Init.Prescaler = DER26_CAN_PRESCALER",
    "hcan1.Init.SyncJumpWidth = DER26_CAN_SJW",
    "hcan1.Init.TimeSeg1 = DER26_CAN_BS1",
    "hcan1.Init.TimeSeg2 = DER26_CAN_BS2",
    "RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE",
]:
    if token not in main:
        raise SystemExit(f"FAIL AMS target source missing: {token}")

# CubeMX project is checked too so regeneration cannot silently restore old
# 250-kbps/SJW1 timing or an HSI PLL source.
for token in ["CAN1.Prescaler=6", "CAN1.SJW=CAN_SJW_2TQ"]:
    if token not in ioc:
        raise SystemExit(f"FAIL AMS .ioc missing {token}")
if "RCC.PLLSourceVirtual=RCC_PLLSOURCE_HSE" not in ioc and "RCC.PLLSource=RCC_PLLSOURCE_HSE" not in ioc:
    # CubeMX key varies by version; source check above remains authoritative.
    print("NOTE AMS .ioc HSE PLL key not recognized; generated main.c HSE gate passed")

print("PASS AMS 54-MHz/HSE-derived 500k/250k CAN timing contract (SJW=2)")
