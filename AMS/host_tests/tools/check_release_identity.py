#!/usr/bin/env python3
"""Check release identity and CubeMX identity without building the target."""
from pathlib import Path
import re
import sys

ROOT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]

def require(condition, message):
    if not condition:
        raise SystemExit("FAIL " + message)

version = (ROOT / "Core/Inc/ams_version.h").read_text()
app = (ROOT / "Core/Inc/app.h").read_text()
profile = (ROOT / "Core/Inc/ams_build_profile.h").read_text()
ioc = (ROOT / "DER26-AMS.ioc").read_text()
for suffix in ("MAJOR", "MINOR", "PATCH"):
    require(re.search(rf"^#define AMS_VERSION_{suffix} [0-9]+$", version, re.M),
            f"missing canonical {suffix}")
for alias, suffix in (("MAJOR", "MAJOR"), ("MINOR", "MINOR"), ("BUG", "PATCH")):
    require(re.search(rf"^#define VER_{alias}\s+AMS_VERSION_{suffix}$", app, re.M),
            f"VER_{alias} must alias canonical version")
require('#include "ams_version.h"' in profile, "profile must include canonical identity")
require("#define AMS_SOURCE_REVISION" not in profile, "duplicate source revision in profile")
require('#define AMS_SOURCE_REVISION "DER26-AMS-v" AMS_VERSION_STRING "-" AMS_RELEASE_DATE' in version,
        "source revision must derive from canonical semantic version")
require(re.search(r'^ProjectManager.ProjectName=DER26-AMS$', ioc, re.M), "CubeMX project name")
require(re.search(r'^ProjectManager.ProjectFileName=DER26-AMS.ioc$', ioc, re.M), "CubeMX project file")
for path in (ROOT / "Core/Src").rglob("*.c"):
    require("__DATE__" not in path.read_text() and "__TIME__" not in path.read_text(),
            f"independent compilation timestamp in {path.name}")
manifest = (ROOT / "Core/Src/app.c").read_text()
cli = (ROOT / "Core/Src/tasks/cli_task.c").read_text()
require(".build_date = AMS_BUILD_DATE" in manifest and ".build_time = AMS_BUILD_TIME" in manifest,
        "manifest must own build timestamp")
require("ams_build_manifest.build_date, ams_build_manifest.build_time" in cli,
        "CLI must read manifest timestamp")
print("PASS canonical firmware/CLI identity, shared timestamp and CubeMX project identity")
