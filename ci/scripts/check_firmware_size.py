#!/usr/bin/env python3

import argparse
import subprocess
from pathlib import Path


def parse_size(elf: Path) -> dict[str, int]:
    result = subprocess.run(
        ["arm-none-eabi-size", str(elf)],
        check=True,
        text=True,
        capture_output=True,
    )

    lines = result.stdout.strip().splitlines()
    if len(lines) < 2:
        raise RuntimeError("Unexpected arm-none-eabi-size output")

    header = lines[0].split()
    values = lines[1].split()

    return {k: int(v, 16) if k == "hex" else int(v) for k, v in zip(header, values)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", required=True)
    parser.add_argument("--max-text", type=int, required=True)
    parser.add_argument("--max-data", type=int, required=True)
    parser.add_argument("--max-bss", type=int, required=True)
    args = parser.parse_args()

    size = parse_size(Path(args.elf))

    checks = [
        ("text", size["text"], args.max_text),
        ("data", size["data"], args.max_data),
        ("bss", size["bss"], args.max_bss),
    ]

    failed = False

    for name, actual, limit in checks:
        print(f"{name}: {actual} / {limit}")
        if actual > limit:
            print(f"ERROR: {name} exceeds limit")
            failed = True

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
