#!/usr/bin/env python3
"""Targeted guard against reintroducing unbounded authored hardware waits."""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "Core" / "Src"

BANNED = (
    (re.compile(r"\bHAL_MAX_DELAY\b"), "HAL_MAX_DELAY"),
    (re.compile(r"\bosWaitForever\b"), "osWaitForever"),
    (re.compile(r"\bportMAX_DELAY\b"), "portMAX_DELAY"),
)

# Generated fatal handlers intentionally stop forever after fail-low/panic. They
# are not hardware transaction waits and are outside this gate's scope.
EXCLUDED_FILES = {
    "main.c",
    "stm32f7xx_it.c",
    "syscalls.c",
}


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//.*?$", "", text, flags=re.M)
    return text


def matching_brace(text: str, start: int) -> int:
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return i
    return -1


def peripheral_wait_findings(path: Path, text: str):
    findings = []
    # Restrict to obvious hardware-status waits. Timer waits are acceptable only
    # when the loop body contains a second software progress bound.
    for m in re.finditer(r"while\s*\((.*?)\)\s*\{", text, flags=re.S):
        cond = m.group(1)
        if not any(token in cond for token in
                   ("__HAL_", "IWDG->", "ADC->", "SPI->", "CAN->", "UART->", "TIM->")):
            continue
        brace = text.find("{", m.start())
        end = matching_brace(text, brace)
        if end < 0:
            findings.append((m.start(), "hardware while-loop with unparsable body"))
            continue
        body = text[brace + 1:end]
        bounded = any(token in cond + body for token in
                      ("timeout", "spin_budget", "max_spins", "spins", "HAL_TIMEOUT"))
        if not bounded:
            findings.append((m.start(), "peripheral-status while-loop lacks explicit software bound"))
    return findings


def line_number(text: str, pos: int) -> int:
    return text.count("\n", 0, pos) + 1


def main() -> int:
    failures = []
    for path in sorted(SRC.rglob("*.c")):
        if path.name in EXCLUDED_FILES:
            continue
        raw = path.read_text(encoding="utf-8", errors="replace")
        text = strip_comments(raw)
        rel = path.relative_to(ROOT)
        for regex, label in BANNED:
            for m in regex.finditer(text):
                failures.append((rel, line_number(text, m.start()),
                                 f"forbidden unbounded wait token {label}"))
        for pos, message in peripheral_wait_findings(path, text):
            failures.append((rel, line_number(text, pos), message))

    if failures:
        for rel, line, message in failures:
            print(f"FAIL {rel}:{line}: {message}")
        return 1
    print("PASS bounded-I/O static gate: no obvious unbounded authored hardware waits")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
