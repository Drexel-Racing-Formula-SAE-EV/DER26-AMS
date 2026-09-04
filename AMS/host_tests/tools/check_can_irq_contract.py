#!/usr/bin/env python3
"""Verify CAN1 NVIC/handler coverage matches the enabled HAL notifications."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
canbus = (ROOT / "Core/Src/ext_drivers/canbus.c").read_text()
msp = (ROOT / "Core/Src/stm32f7xx_hal_msp.c").read_text()
it_c = (ROOT / "Core/Src/stm32f7xx_it.c").read_text()
it_h = (ROOT / "Core/Inc/stm32f7xx_it.h").read_text()
ioc_files = list(ROOT.glob("*.ioc"))
if len(ioc_files) != 1:
    raise SystemExit(f"FAIL expected one CubeMX .ioc, found {len(ioc_files)}")
ioc = ioc_files[0].read_text()

required_notifications = [
    "CAN_IT_RX_FIFO0_MSG_PENDING",
    "CAN_IT_RX_FIFO0_OVERRUN",
    "CAN_IT_TX_MAILBOX_EMPTY",
    "CAN_IT_BUSOFF",
    "CAN_IT_ERROR",
]
for token in required_notifications:
    if token not in canbus:
        raise SystemExit(f"FAIL enabled CAN notification token missing: {token}")

# bxCAN routes the enabled classes above to RX0, TX, and SCE respectively.
required_vectors = ["CAN1_RX0", "CAN1_TX", "CAN1_SCE"]
for vector in required_vectors:
    irq = f"{vector}_IRQn"
    handler = f"{vector}_IRQHandler"
    if f"HAL_NVIC_EnableIRQ({irq});" not in msp:
        raise SystemExit(f"FAIL {irq} is not enabled in HAL_CAN_MspInit")
    if f"HAL_NVIC_DisableIRQ({irq});" not in msp:
        raise SystemExit(f"FAIL {irq} is not disabled in HAL_CAN_MspDeInit")
    if handler not in it_h:
        raise SystemExit(f"FAIL {handler} prototype missing")
    block = re.search(rf"void\s+{handler}\s*\(void\)\s*\{{(.*?)\n\}}", it_c, re.S)
    if not block or "canbus_irq_handler(&hcan1);" not in block.group(1):
        raise SystemExit(f"FAIL {handler} bypasses deferred CAN mailbox refill")
    if f"NVIC.{irq}=true" not in ioc:
        raise SystemExit(f"FAIL CubeMX .ioc does not persist enabled {irq}")

# RX1 is intentionally unused because no FIFO1 notification is activated.
if "HAL_CAN_IRQHandler(hcan);" not in canbus:
    raise SystemExit("FAIL CAN IRQ wrapper does not dispatch HAL callbacks")
if "CAN_IT_RX_FIFO1" in canbus:
    raise SystemExit("FAIL FIFO1 notification introduced without extending IRQ contract")

print("PASS CAN1 RX0/TX/SCE interrupt coverage matches enabled HAL notifications")
