#!/usr/bin/env python3
"""Generate deterministic DER26 MiL requirement-to-scenario traceability."""
from __future__ import annotations

import argparse
import csv
import io
import re
import sys
from pathlib import Path

MIL = Path(__file__).resolve().parents[1]
REPO = MIL.parent
CATALOG = MIL / "requirements" / "requirements.csv"
SCENARIOS = MIL / "matlab" / "configs" / "scenarios"
OUT_MD = MIL / "docs" / "REQUIREMENTS_TRACEABILITY.md"
OUT_CSV = MIL / "docs" / "requirements_traceability.csv"


def parse_scenarios() -> dict[str, dict[str, object]]:
    result: dict[str, dict[str, object]] = {}
    for path in sorted(SCENARIOS.glob("*.m")):
        source = path.read_text(encoding="utf-8")
        sid = re.search(r"cfg\.id\s*=\s*'([^']+)'", source)
        req = re.search(r"cfg\.requirements\s*=\s*\{([^}]*)\}", source)
        tier = re.search(r"cfg\.tier\s*=\s*'([^']+)'", source)
        if not sid or not req or not tier:
            raise ValueError(f"scenario metadata incomplete: {path.name}")
        result[sid.group(1)] = {
            "file": path.relative_to(REPO).as_posix(),
            "tier": tier.group(1),
            "requirements": re.findall(r"'([^']+)'", req.group(1)),
        }
    return result


def load_catalog() -> list[dict[str, str]]:
    with CATALOG.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    ids = [row["id"] for row in rows]
    if len(ids) != len(set(ids)):
        raise ValueError("duplicate requirement IDs in catalog")
    return rows


def render() -> tuple[str, str]:
    scenarios = parse_scenarios()
    catalog = load_catalog()
    known = {row["id"] for row in catalog}
    referenced = {rid for item in scenarios.values() for rid in item["requirements"]}
    unknown = sorted(referenced - known)
    if unknown:
        raise ValueError(f"scenario references unknown requirements: {unknown}")

    coverage: dict[str, list[str]] = {rid: [] for rid in known}
    tiers: dict[str, set[str]] = {rid: set() for rid in known}
    for sid, item in scenarios.items():
        for rid in item["requirements"]:
            coverage[rid].append(sid)
            tiers[rid].add(str(item["tier"]))

    output = io.StringIO()
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(["id", "statement", "verification", "scenario_coverage", "tiers", "fixed_evidence"])
    for row in catalog:
        rid = row["id"]
        writer.writerow([rid, row["statement"], row["verification"],
                         ";".join(coverage[rid]), ";".join(sorted(tiers[rid])),
                         row["fixed_evidence"]])
    csv_text = output.getvalue()

    lines = [
        "# DER26 AMS MiL Requirements Traceability",
        "",
        "Generated deterministically from `MiL/requirements/requirements.csv` and the checked-in scenario metadata.",
        "This is implementation traceability, not executed qualification evidence.",
        "",
        f"- Requirements: {len(catalog)}",
        f"- Scenarios: {len(scenarios)}",
        f"- Scenario-linked requirements: {sum(bool(v) for v in coverage.values())}",
        "",
        "| ID | Requirement | Scenario coverage | Tier | Fixed evidence |",
        "|---|---|---|---|---|",
    ]
    for row in catalog:
        rid = row["id"]
        cov = ", ".join(f"`{name}`" for name in coverage[rid]) or "host/static evidence"
        tier = ", ".join(sorted(tiers[rid])) or "host"
        lines.append(f"| `{rid}` | {row['statement']} | {cov} | {tier} | `{row['fixed_evidence']}` |")
    lines.extend(["", "## Compact C0-C8 campaign", ""])
    for idx in range(9):
        prefix = f"c{idx}_"
        names = [name for name in scenarios if name.startswith(prefix)]
        if len(names) != 1:
            raise ValueError(f"expected one compact C{idx} scenario, found {names}")
        item = scenarios[names[0]]
        lines.append(f"- C{idx}: `{names[0]}` - `{item['file']}`")
    return "\n".join(lines) + "\n", csv_text


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    md, csv_text = render()
    if args.check:
        stale = []
        for path, expected in ((OUT_MD, md), (OUT_CSV, csv_text)):
            if not path.is_file() or path.read_text(encoding="utf-8") != expected:
                stale.append(str(path.relative_to(REPO)))
        if stale:
            print("traceability outputs stale: " + ", ".join(stale), file=sys.stderr)
            return 1
        print("DER26 MiL traceability: PASS")
        return 0
    OUT_MD.parent.mkdir(parents=True, exist_ok=True)
    OUT_MD.write_text(md, encoding="utf-8")
    OUT_CSV.write_text(csv_text, encoding="utf-8")
    print(f"wrote {OUT_MD.relative_to(REPO)} and {OUT_CSV.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
