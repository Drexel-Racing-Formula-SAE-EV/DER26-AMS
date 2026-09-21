#!/usr/bin/env python3

import argparse
import subprocess
from pathlib import Path


def resolve_elf(requested: Path) -> Path:
    """Resolve the headless build's unique Debug.* / Release.* ELF path."""
    if requested.is_file():
        return requested

    # The headless builder intentionally preserves every build under a unique
    # AMS/build/<type>.<suffix>/ directory. CI historically passed the stable
    # AMS/build/DER26-AMS.elf path, so fall back to the newest matching artifact
    # when that compatibility link is absent (for example on older runners).
    parent = requested.parent
    candidates = [p for p in parent.glob(f"*/{requested.name}") if p.is_file()]
    if not candidates:
        raise FileNotFoundError(
            f"ELF not found at {requested} and no {parent}/*/{requested.name} exists"
        )

    resolved = max(candidates, key=lambda p: p.stat().st_mtime_ns)
    print(f"Resolved ELF: {requested} -> {resolved}")
    return resolved


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    elf = resolve_elf(Path(args.elf))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

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
    data = dict(zip(header, values))

    report_lines = [
        "# AMS Firmware Size Report",
        "",
        f"ELF: {elf}",
        "",
        "| Section | Bytes |",
        "|---|---:|",
        f"| text | {data.get('text', 'n/a')} |",
        f"| data | {data.get('data', 'n/a')} |",
        f"| bss | {data.get('bss', 'n/a')} |",
        f"| dec | {data.get('dec', 'n/a')} |",
        f"| hex | {data.get('hex', 'n/a')} |",
        "",
        "Raw arm-none-eabi-size output:",
        "",
        result.stdout.strip(),
        "",
    ]

    report = "\n".join(report_lines)
    out.write_text(report, encoding="utf-8")
    print(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
