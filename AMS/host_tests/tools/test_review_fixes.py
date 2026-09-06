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
            "Core/Inc/FreeRTOSConfig.h", "Core/Inc/stm32f7xx_it.h", "Core/Inc/main.h",
            "Core/Src/app.c", "Core/Src/main.c", "Core/Src/board.c",
            "Core/Src/tasks/cli_task.c",
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

    def test_cs_b_pin_drift_rejected(self):
        self.change("Core/Inc/main.h", "#define CS_B_GPIO_Port GPIOE",
                    "#define CS_B_GPIO_Port GPIOF")
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

    def test_vehicle_cli_path_is_compiled_out(self):
        profile = (self.root / "Core/Inc/ams_build_profile.h").read_text()
        vehicle = profile.split("#elif AMS_BUILD_PROFILE == AMS_PROFILE_VEHICLE", 1)[1]
        vehicle = vehicle.split("#else\n#error \"AMS_BUILD_PROFILE", 1)[0]
        self.assertIn("#define AMS_ENABLE_CLI 0", vehicle)

        app = (self.root / "Core/Src/app.c").read_text()
        self.assertRegex(
            app,
            re.compile(r"#if AMS_ENABLE_CLI\s+\(void\)cli_uart_start_rx.*?"
                       r"app\.cli_task = cli_task_start\(&app\);.*?"
                       r"#else\s+app\.cli_task = NULL;\s+#endif", re.S),
        )
        main = (self.root / "Core/Src/main.c").read_text()
        self.assertRegex(
            main,
            re.compile(r"#if AMS_ENABLE_CLI\s+MX_USART3_UART_Init\(\);\s+#endif", re.S),
        )
        board = (self.root / "Core/Src/board.c").read_text()
        self.assertRegex(
            board,
            re.compile(r"#if AMS_ENABLE_CLI\s+cli_device_init\(", re.S),
        )
        irq = (self.root / "Core/Src/stm32f7xx_it.c").read_text()
        self.assertIn("#if AMS_ENABLE_CLI\nvoid HAL_UART_RxCpltCallback", irq)
        self.assertIn("#endif /* AMS_ENABLE_CLI */", irq)

    def test_service_cli_cannot_exist_without_cli_transport(self):
        compiler = shutil.which("gcc")
        if compiler is None:
            self.skipTest("host GCC unavailable")
        program = '#include "ams_build_profile.h"\nint main(void){return 0;}\n'
        result = subprocess.run(
            [compiler, "-std=c11", "-I", str(self.root / "Core/Inc"),
             "-DAMS_BUILD_PROFILE=1", "-DAMS_ENABLE_CLI=0",
             "-x", "c", "-", "-fsyntax-only"],
            input=program, text=True, capture_output=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Service CLI mutation requires the diagnostic CLI task/UART",
                      result.stderr)

    def test_release_build_requires_source_date_epoch(self):
        script = AMS.parent / "ci/stm32/build_ams_headless_gcc.sh"
        env = dict(os.environ)
        env["AMS_BUILD_TYPE"] = "Release"
        env.pop("SOURCE_DATE_EPOCH", None)
        result = subprocess.run(["bash", str(script)], capture_output=True, text=True, env=env)
        self.assertEqual(result.returncode, 2)
        self.assertIn("Release builds require SOURCE_DATE_EPOCH", result.stderr)

    def test_headless_profile_release_flags_and_provenance_match_checked_in_release(self):
        script = (AMS.parent / "ci/stm32/build_ams_headless_gcc.sh").read_text()
        cproject = (AMS / ".cproject").read_text()
        self.assertIn('AMS_BUILD_PROFILE="${AMS_BUILD_PROFILE:-5}"', script)
        self.assertIn('Release) MODE_FLAGS=(-Os -g0)', script)
        self.assertIn('-DAMS_BUILD_PROFILE="$AMS_BUILD_PROFILE"', script)
        self.assertIn('build_profile=$AMS_BUILD_PROFILE', script)
        self.assertIn('build_profile_name=$AMS_BUILD_PROFILE_NAME', script)
        self.assertIn('source_input_tree_sha256=$SOURCE_INPUT_TREE_SHA256', script)
        self.assertNotIn('Release) MODE_FLAGS=(-O2', script)
        self.assertGreaterEqual(cproject.count('value="AMS_BUILD_PROFILE=5"'), 4)
        self.assertIn('optimization.level.value.os', cproject)

    def test_invalid_headless_profile_is_rejected_before_toolchain_lookup(self):
        script = AMS.parent / "ci/stm32/build_ams_headless_gcc.sh"
        env = dict(os.environ, AMS_BUILD_TYPE="Debug", AMS_BUILD_PROFILE="99")
        result = subprocess.run(["bash", str(script)], capture_output=True, text=True, env=env)
        self.assertEqual(result.returncode, 2)
        self.assertIn("AMS_BUILD_PROFILE must be one of 1,2,3,4,5", result.stderr)

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
