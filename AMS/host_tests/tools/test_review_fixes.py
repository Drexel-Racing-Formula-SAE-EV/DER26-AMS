#!/usr/bin/env python3
"""Focused regressions for release identity and CAN IRQ contract checks."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

AMS = Path(__file__).resolve().parents[2]
TOOLS = AMS / "host_tests/tools"

class ReviewFixTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "AMS"
        for relative in (
            "Core/Inc/app.h", "Core/Inc/ams_version.h", "Core/Inc/ams_build_profile.h",
            "Core/Inc/FreeRTOSConfig.h", "Core/Inc/stm32f7xx_it.h",
            "Core/Src/app.c", "Core/Src/tasks/cli_task.c",
            "Core/Src/ext_drivers/canbus.c", "Core/Src/stm32f7xx_hal_msp.c",
            "Core/Src/stm32f7xx_it.c", "DER26-AMS.ioc",
        ):
            destination = self.root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(AMS / relative, destination)

    def change(self, relative, before, after):
        path = self.root / relative
        content = path.read_text()
        self.assertIn(before, content)
        path.write_text(content.replace(before, after))

    def gate(self, script, should_pass):
        result = subprocess.run(["python3", str(TOOLS / script), str(self.root)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode == 0, should_pass, result.stdout + result.stderr)

    def test_current_contracts_pass(self):
        self.gate("check_release_identity.py", True)
        self.gate("check_can_irq_contract.py", True)

    def test_stale_cli_version_rejected(self):
        self.change("Core/Inc/app.h", "#define VER_BUG   AMS_VERSION_PATCH",
                    "#define VER_BUG   19")
        self.gate("check_release_identity.py", False)

    def test_stale_cube_name_rejected(self):
        self.change("DER26-AMS.ioc", "ProjectManager.ProjectName=DER26-AMS",
                    "ProjectManager.ProjectName=DER25-AMS")
        self.gate("check_release_identity.py", False)

    def test_stale_cube_filename_rejected(self):
        self.change("DER26-AMS.ioc", "ProjectManager.ProjectFileName=DER26-AMS.ioc",
                    "ProjectManager.ProjectFileName=DER25-AMS.ioc")
        self.gate("check_release_identity.py", False)

    def test_independent_timestamp_rejected(self):
        self.change("Core/Src/tasks/cli_task.c", "ams_build_manifest.build_date", "__DATE__")
        self.gate("check_release_identity.py", False)

    def test_irq_above_syscall_ceiling_rejected(self):
        self.change("Core/Src/stm32f7xx_hal_msp.c", "CAN1_RX0_IRQn, 5, 0", "CAN1_RX0_IRQn, 4, 0")
        self.gate("check_can_irq_contract.py", False)

    def test_changed_ceiling_rejected(self):
        self.change("Core/Inc/FreeRTOSConfig.h", "configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5",
                    "configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 6")
        self.gate("check_can_irq_contract.py", False)

    def test_cube_irq_drift_rejected(self):
        self.change("DER26-AMS.ioc", r"NVIC.CAN1_TX_IRQn=true\:5\:0", r"NVIC.CAN1_TX_IRQn=true\:6\:0")
        self.gate("check_can_irq_contract.py", False)

    def test_canonical_header_compiles_with_reproducible_timestamp(self):
        compiler = shutil.which("gcc")
        if compiler is None:
            self.skipTest("host GCC unavailable")
        program = '#include <stdio.h>\n#include "ams_version.h"\nint main(void) { puts(AMS_SOURCE_REVISION); puts(AMS_BUILD_DATE); puts(AMS_BUILD_TIME); return 0; }\n'
        binary = Path(self.temp.name) / "version_probe"
        env = dict(os.environ, SOURCE_DATE_EPOCH="946684800", LC_ALL="C")
        subprocess.run([compiler, "-Wall", "-Wextra", "-Werror", "-I", str(self.root / "Core/Inc"),
                        "-x", "c", "-", "-o", str(binary)], input=program, text=True, env=env, check=True)
        header = (self.root / "Core/Inc/ams_version.h").read_text()
        numbers = [re.search(rf"#define AMS_VERSION_{name} (\d+)", header).group(1)
                   for name in ("MAJOR", "MINOR", "PATCH")]
        date = re.search(r'#define AMS_RELEASE_DATE "(\d+)"', header).group(1)
        expected = "DER26-AMS-v" + ".".join(numbers) + "-" + date
        result = subprocess.check_output([str(binary)], text=True).splitlines()
        self.assertEqual(result, [expected, "Jan  1 2000", "00:00:00"])

if __name__ == "__main__":
    unittest.main(verbosity=2)
