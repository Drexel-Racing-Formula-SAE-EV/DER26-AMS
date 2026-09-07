#!/usr/bin/env python3
"""Pin the exact bundled FreeRTOS source without guessing its marketing version."""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re
import sys

AMS = Path(__file__).resolve().parents[2]
SOURCE = AMS / "Middlewares/Third_Party/FreeRTOS/Source"
TASK_H = SOURCE / "include/task.h"

EXPECTED_TREE_SHA256 = "4393c390c3939c1ce11c713c9b862ec1b27cd9a3448e73a7a48c9c11dbdc3824"
EXPECTED_TASK_H_VERSION = "V10.2.0"
EXPECTED_BANNER_VERSION = "V10.2.1"


def tree_sha256() -> str:
    digest = hashlib.sha256()
    files = sorted((p for p in SOURCE.rglob("*") if p.is_file()),
                   key=lambda p: p.relative_to(SOURCE).as_posix())
    for path in files:
        relative = path.relative_to(SOURCE).as_posix().encode("utf-8")
        digest.update(relative)
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def task_version() -> str:
    match = re.search(r'^#define\s+tskKERNEL_VERSION_NUMBER\s+"([^"]+)"',
                      TASK_H.read_text(), re.M)
    if not match:
        raise SystemExit("FAIL missing tskKERNEL_VERSION_NUMBER in task.h")
    return match.group(1)


def banner_versions() -> set[str]:
    versions: set[str] = set()
    pattern = re.compile(r"FreeRTOS Kernel (V\d+\.\d+\.\d+)")
    for path in SOURCE.rglob("*"):
        if not path.is_file():
            continue
        text = path.read_text(errors="ignore")
        versions.update(pattern.findall(text[:600]))
    return versions


def check() -> tuple[str, str, set[str]]:
    digest = tree_sha256()
    api_version = task_version()
    banners = banner_versions()

    if digest != EXPECTED_TREE_SHA256:
        raise SystemExit(
            "FAIL bundled FreeRTOS source tree changed: "
            f"{digest} != {EXPECTED_TREE_SHA256}"
        )
    if api_version != EXPECTED_TASK_H_VERSION:
        raise SystemExit(
            "FAIL task.h kernel version changed: "
            f"{api_version} != {EXPECTED_TASK_H_VERSION}"
        )
    if EXPECTED_BANNER_VERSION not in banners:
        raise SystemExit(
            "FAIL expected FreeRTOS source banner version "
            f"{EXPECTED_BANNER_VERSION} not present"
        )
    return digest, api_version, banners


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", action="store_true",
                        help="print stable provenance fields for a target build manifest")
    args = parser.parse_args()

    digest, api_version, banners = check()
    banner_text = ",".join(sorted(banners))
    if args.manifest:
        print(f"freertos_tree_sha256={digest}")
        print(f"freertos_task_h_version={api_version}")
        print(f"freertos_banner_versions={banner_text}")
    else:
        print(
            "PASS bundled FreeRTOS provenance pinned: "
            f"tree={digest} task.h={api_version} banners={banner_text}"
        )
        print(
            "NOTE task.h and source banners disagree; the hash, not a guessed "
            "version label, is the authoritative bundled-source identity."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
