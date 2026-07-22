# DER26 AMS reusable host-side test harness
#
# Intended location:
#   <AMS repo root>/host_tests/Makefile
#
# Usage from inside host_tests:
#   make test
#   make asan
#   make analyze
#
# Usage from outside the repo:
#   make AMS_ROOT=/path/to/DER26-AMS-feature-canbus_charger test

AMS_ROOT ?= ..
REPO_ROOT ?= $(abspath $(AMS_ROOT)/..)
BUILD_DIR ?= build
CC ?= gcc
CLANG ?= clang
PYTHON ?= python3

SRC := src/ams_host_test_runner.c
BIN = $(BUILD_DIR)/ams_host_tests
UNIT_SRC := unit/ams_unit_test_runner.c
UNIT_BIN = $(BUILD_DIR)/ams_unit_tests
OWNER_SRC := production/bms_owner_test.c
OWNER_BIN = $(BUILD_DIR)/bms_owner_test
PROFILE_SRC := production/profile_compile_test.c
ESTIMATOR_TOPOLOGY_SRC := production/estimator_topology_test.c
ESTIMATOR_SRC := \
	$(AMS_ROOT)/Core/Src/estimator/ams_soc_ekf.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c

ADBMS2950_SRC := \
	$(AMS_ROOT)/Core/Src/ext_drivers/adbms2950.c

AIR_MONITOR_SRC := \
	$(AMS_ROOT)/Core/Src/ext_drivers/air_monitor.c

THERMISTOR_SRC := \
	$(AMS_ROOT)/Core/Src/ext_drivers/thermistor_model.c
MEASUREMENT_SRC := \
	$(AMS_ROOT)/Core/Src/measurement/ams_measurement.c
POWER_SRC := \
	$(AMS_ROOT)/Core/Src/soh/ams_soh.c \
	$(AMS_ROOT)/Core/Src/sop/ams_sop.c \
	$(AMS_ROOT)/Core/Src/sop/ams_fuse_observer.c \
	$(AMS_ROOT)/Core/Src/sop/ams_power_strategy.c \
	$(AMS_ROOT)/Core/Src/sop/ams_power_state.c \
	$(AMS_ROOT)/Core/Src/sop/ams_power_can.c
THERMISTOR_UNIT_SRC := unit/thermistor_model_unit_test.c
THERMISTOR_UNIT_BIN = $(BUILD_DIR)/thermistor_model_unit_tests
THERMISTOR_TOOL_DIR := $(REPO_ROOT)/Tools/thermistor_model
THERMISTOR_SOURCE_CSV := $(THERMISTOR_TOOL_DIR)/source/NTC_RT_Calculation_Vishay_NTCLE350E4103FHB0.csv
THERMISTOR_COMPILED_REPORT := $(BUILD_DIR)/thermistor_compiled_validation.json
SOP_REFERENCE_DIR := $(REPO_ROOT)/Tools/sop_reference
FUSE_REPLAY_DIR := $(REPO_ROOT)/Tools/fuse_replay
DASHBOARD_TEST_DIR := $(REPO_ROOT)/Tools/esp32_ams_dashboard_logger/tests
POWER_CONSUMER_SRC := \
	$(SOP_REFERENCE_DIR)/tests/ecu_power_consumer_test.c \
	$(SOP_REFERENCE_DIR)/ecu_power_consumer.c
POWER_CONSUMER_BIN := $(BUILD_DIR)/ecu_power_consumer_test

COMMON_DEFS := \
	-DSTM32F767xx \
	-DUSE_HAL_DRIVER \
	-DAMS_HOST_TEST=1

HOST_HW_BRINGUP ?= 0
HOST_BALANCE_INHIBIT_DEFAULT ?= 0

HOST_EXERCISE_DEFS = \
	-DAMS_BUILD_PROFILE=1 \
	-DAMS_HW_BRINGUP=$(HOST_HW_BRINGUP) \
	-DAMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT=$(HOST_BALANCE_INHIBIT_DEFAULT) \
	-DAMS_ENABLE_HIL_CAN=1 \
	-DAMS_ENABLE_MISSION_CAN=1 \
	-DAMS_ENABLE_SERVICE_CLI=1

VEHICLE_VALIDATION_DEFS := \
	-DAMS_BUILD_PROFILE=3 \
	-DAMS_VEHICLE_PROFILE_REVIEWED=1 \
	-DAMS_IMD_TARGET_VALIDATED=1 \
	-DAMS_IWDG_TARGET_VALIDATED=1 \
	-DAMS_CURRENT_POLARITY_VALIDATED=1 \
	-DAMS_CURRENT_CALIBRATION_VALIDATED=1 \
	-DAMS_BALANCE_TARGET_VALIDATED=1 \
	-DAMS_CAN_VEHICLE_CONTRACT_VALIDATED=1 \
	-DAMS_SOP_MODEL_VALIDATED=1 \
	-DAMS_SOP_CALIBRATION_VALIDATED=1 \
	-DAMS_SOP_CAN_CONTRACT_VALIDATED=1

VEHICLE_VALIDATION_DEFS += \
	-DAMS_MISSION_CAN_CONTRACT_VALIDATED=1 \
	-DAMS_FUSE_MODEL_VALIDATED=1

VEHICLE_MANIFEST_DEFS := \
	-DAMS_BUILD_GIT_COMMIT=\"host-profile-gate\" \
	-DAMS_CURRENT_CALIBRATION_REVISION=\"host-gate-not-release\" \
	-DAMS_CAN_CONTRACT_REVISION=\"host-gate-not-release\" \
	-DAMS_THRESHOLD_REVISION=\"host-gate-not-release\" \
	-DAMS_ESTIMATOR_MODEL_REVISION=\"host-gate-not-release\"

VEHICLE_MANIFEST_DEFS += \
	-DAMS_SOP_MODEL_REVISION=\"host-gate-not-release\" \
	-DAMS_SOH_MODEL_REVISION=\"host-gate-not-release\"

VEHICLE_PROFILE_DEFS := $(VEHICLE_VALIDATION_DEFS) $(VEHICLE_MANIFEST_DEFS)

INC := \
	-Iinclude \
	-I$(AMS_ROOT) \
	-I$(AMS_ROOT)/Core/Inc \
	-I$(AMS_ROOT)/Core/Inc/tasks \
	-I$(AMS_ROOT)/Core/Inc/ext_drivers \
	-I$(AMS_ROOT)/Drivers/CMSIS/Include \
	-I$(AMS_ROOT)/Drivers/CMSIS/Device/ST/STM32F7xx/Include \
	-I$(AMS_ROOT)/Drivers/STM32F7xx_HAL_Driver/Inc \
	-I$(AMS_ROOT)/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy \
	-I$(AMS_ROOT)/Middlewares/Third_Party/FreeRTOS/Source/include \
	-I$(AMS_ROOT)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
	-I$(AMS_ROOT)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1

WARN := \
	-Wall -Wextra \
	-Wno-unused-parameter \
	-Wno-unused-function \
	-Wno-missing-field-initializers \
	-Wno-pointer-to-int-cast \
	-Wno-int-to-pointer-cast

CFLAGS_COMMON := -std=c11 -g -O0 $(WARN) $(COMMON_DEFS) $(INC)
LDLIBS := -lm

.PHONY: help check-root compile unit power-core power-metamorphic power-sensitivity power-oracle power-consumer power-resources fuse-oracle fuse-replay dashboard-decoder thermistor-validate test profile-gates estimator-topology-gates production-gates static-allocation-gate state-ownership-gate target-project-gate whole-source-analyze system-sil apm-sil bringup-test hil-adbms-test imd-test air-feedback-stub-test safety-test stress ci asan ubsan analyze clang-analyze clean list make-build-dir

help:
	@echo "DER26 AMS host test harness"
	@echo ""
	@echo "Targets:"
	@echo "  make unit          Build and run isolated unit tests"
	@echo "  make power-core    Run production SoP/SoH/CRC/brute-force tests"
	@echo "  make power-metamorphic Run 40k seeded SoP monotonicity/fail-zero states"
	@echo "  make power-sensitivity Run non-gating 30 s DCL sensitivity characterization"
	@echo "  make power-oracle  Differential-test C SoP against independent HIL-table Python oracle"
	@echo "  make power-consumer Test the fail-zero ECU SoP/SoH bundle consumer"
	@echo "  make power-resources Report host runtime, static object sizes, and GCC stack frames"
	@echo "  make fuse-oracle   Compare curve observer to independent exact-state oracle"
	@echo "  make fuse-replay   Build replay tool and run synthetic policy characterization"
	@echo "  make dashboard-decoder Test logger SoP/SoH CRC, freshness and decode paths"
	@echo "  make test          Build and run comprehensive host tests"
	@echo "  make thermistor-validate Compile C model and compare against every Vishay CSV row"
	@echo "  make profile-gates Verify bench/HIL/vehicle defaults and vehicle evidence lock"
	@echo "  make production-gates Verify HIL/service CLI are disabled by default"
	@echo "  make static-allocation-gate Verify every application task uses fixed storage"
	@echo "  make state-ownership-gate Verify state and CAN-control ownership boundaries"
	@echo "  make target-project-gate Verify CubeIDE project identity, paths, warnings and linker scripts"
	@echo "  make system-sil    Alias for comprehensive system-level SIL/injection tests"
	@echo "  make apm-sil       Run actual-driver ADBMS2950 unit tests plus final-ring CLI/SIL tests"
	@echo "  make bringup-test  Run comprehensive tests with AMS_HW_BRINGUP=1"
	@echo "  make hil-adbms-test Run focused CAN-fed ADBMS replacement tests"
	@echo "  make imd-test       Run comprehensive tests with AMS_ENABLE_IMD=1"
	@echo "  make air-feedback-stub-test Verify future AUX enablement starts and faults fail-closed"
	@echo "  make safety-test    Run comprehensive tests with AMS_ENABLE_IWDG=1"
	@echo "  make stress        Run comprehensive SIL with extended seeded fuzz cycles"
	@echo "  make ci            Run unit, comprehensive, and analyzer targets"
	@echo "  make asan          Build and run with AddressSanitizer + UBSan"
	@echo "  make ubsan         Build and run with UBSan only"
	@echo "  make analyze       Run GCC static analyzer on the harness TU"
	@echo "  make whole-source-analyze Analyze every Core/Src C file in bench and vehicle profiles"
	@echo "  make clang-analyze Run Clang static analyzer if clang is installed"
	@echo "  make clean         Remove host-test build output"
	@echo ""
	@echo "Variables:"
	@echo "  AMS_ROOT=/path/to/DER26-AMS-feature-canbus_charger"
	@echo "  CC=gcc or clang"

check-root:
	@test -d "$(AMS_ROOT)/Core/Inc" || (echo "ERROR: AMS_ROOT does not point to AMS repo root: $(AMS_ROOT)"; exit 2)
	@test -f "$(AMS_ROOT)/Core/Src/tasks/canbus_task.c" || (echo "ERROR: missing Core/Src/tasks/canbus_task.c under AMS_ROOT"; exit 2)
	@test -f "$(AMS_ROOT)/Core/Src/ext_drivers/accumulator.c" || (echo "ERROR: missing Core/Src/ext_drivers/accumulator.c under AMS_ROOT"; exit 2)
	@test -f "$(AMS_ROOT)/Core/Src/estimator/ams_soc_ekf.c" || (echo "ERROR: missing Core/Src/estimator/ams_soc_ekf.c under AMS_ROOT"; exit 2)
	@test -f "$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c" || (echo "ERROR: missing Core/Src/estimator/ams_estimator_lut.c under AMS_ROOT"; exit 2)
	@test -f "$(AMS_ROOT)/Core/Src/ext_drivers/thermistor_model.c" || (echo "ERROR: missing thermistor_model.c under AMS_ROOT"; exit 2)
	@test -f "$(THERMISTOR_TOOL_DIR)/validate_compiled_model.py" || (echo "ERROR: missing thermistor compiled-model validator"; exit 2)
	@test -f "$(THERMISTOR_SOURCE_CSV)" || (echo "ERROR: missing Vishay thermistor source CSV"; exit 2)
	@test -f "$(SOP_REFERENCE_DIR)/model.py" || (echo "ERROR: missing independent SoP oracle"; exit 2)
	@test -f "$(FUSE_REPLAY_DIR)/fuse_reference_oracle.c" || (echo "ERROR: missing independent fuse oracle"; exit 2)
	@test -f "$(DASHBOARD_TEST_DIR)/decoder_smoke_test.c" || (echo "ERROR: missing dashboard decoder tests"; exit 2)


build:
	mkdir -p $(BUILD_DIR)
make-build-dir:
	@mkdir -p "$(BUILD_DIR)"

compile: check-root make-build-dir
	$(CC) $(CFLAGS_COMMON) $(SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC) $(POWER_SRC) -o $(BIN) $(LDLIBS)

unit: check-root make-build-dir
	$(CC) $(CFLAGS_COMMON) $(UNIT_SRC) $(ESTIMATOR_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC) -o $(UNIT_BIN) $(LDLIBS)
	$(UNIT_BIN)
	$(CC) $(CFLAGS_COMMON) $(THERMISTOR_UNIT_SRC) $(THERMISTOR_SRC) -o $(THERMISTOR_UNIT_BIN) $(LDLIBS)
	$(THERMISTOR_UNIT_BIN)

ci: unit power-core power-metamorphic fuse-oracle power-oracle power-consumer dashboard-decoder thermistor-validate test apm-sil profile-gates estimator-topology-gates production-gates static-allocation-gate state-ownership-gate target-project-gate analyze whole-source-analyze

power-core: check-root
	$(MAKE) -C sop test

power-metamorphic: check-root
	$(MAKE) -C sop metamorphic

# Useful when changing limits/model calibration; not a pass/fail CI target.
power-sensitivity: check-root
	$(MAKE) -C sop sensitivity

power-oracle: check-root
	PYTHONDONTWRITEBYTECODE=1 $(PYTHON) -m unittest discover -v \
		-s "$(SOP_REFERENCE_DIR)/tests"

power-consumer: check-root make-build-dir
	$(CC) -std=c11 -Wall -Wextra -Werror -pedantic -O2 \
		-I"$(SOP_REFERENCE_DIR)" $(POWER_CONSUMER_SRC) \
		-o "$(POWER_CONSUMER_BIN)"
	"$(POWER_CONSUMER_BIN)"


fuse-oracle: check-root
	$(MAKE) -C fuse clean test
	$(MAKE) -C "$(FUSE_REPLAY_DIR)" clean all smoke-ci

# Characterization only: builds the replay tool, regenerates deterministic
# traces, and writes the policy sweep.  It is intentionally not a CI gate.
fuse-replay: check-root
	$(MAKE) -C "$(FUSE_REPLAY_DIR)" clean all traces policy-sweep

power-resources: check-root
	$(MAKE) -C sop clean benchmark stack-report
	@mkdir -p "$(BUILD_DIR)/power-resources"
	$(CC) $(CFLAGS_COMMON) -O2 -fstack-usage $(VEHICLE_PROFILE_DEFS) -c \
		"$(AMS_ROOT)/Core/Src/tasks/estimator_task.c" \
		-o "$(BUILD_DIR)/power-resources/estimator_task.o"
	@cat "$(BUILD_DIR)/power-resources/estimator_task.su"

dashboard-decoder: check-root
	$(MAKE) -C "$(DASHBOARD_TEST_DIR)" clean test syntax

thermistor-validate: check-root make-build-dir
	$(PYTHON) "$(THERMISTOR_TOOL_DIR)/validate_compiled_model.py" \
		--repo "$(REPO_ROOT)" \
		--csv "$(THERMISTOR_SOURCE_CSV)" \
		--output "$(THERMISTOR_COMPILED_REPORT)" \
		--cc "$(CC)"

# Normal run: catches logic regressions and compile/link problems.
test: CFLAGS_COMMON += $(HOST_EXERCISE_DEFS)
test: compile
	$(BIN)

profile-gates: check-root make-build-dir
	$(CC) $(CFLAGS_COMMON) -Werror -DAMS_BUILD_PROFILE=1 -DAMS_EXPECTED_PROFILE=1 \
		$(PROFILE_SRC) -o "$(BUILD_DIR)/profile_bench" $(LDLIBS)
	"$(BUILD_DIR)/profile_bench"
	$(CC) $(CFLAGS_COMMON) -Werror -DAMS_BUILD_PROFILE=2 -DAMS_EXPECTED_PROFILE=2 \
		$(PROFILE_SRC) -o "$(BUILD_DIR)/profile_hil" $(LDLIBS)
	"$(BUILD_DIR)/profile_hil"
	$(CC) $(CFLAGS_COMMON) -Werror $(VEHICLE_PROFILE_DEFS) -DAMS_EXPECTED_PROFILE=3 \
		$(PROFILE_SRC) -o "$(BUILD_DIR)/profile_vehicle" $(LDLIBS)
	"$(BUILD_DIR)/profile_vehicle"
	@if $(CC) $(CFLAGS_COMMON) -Werror -DAMS_BUILD_PROFILE=3 -DAMS_EXPECTED_PROFILE=3 \
		$(PROFILE_SRC) -o "$(BUILD_DIR)/profile_vehicle_unsafe" $(LDLIBS) \
		>"$(BUILD_DIR)/profile_vehicle_unsafe.log" 2>&1; then \
		echo "ERROR: vehicle profile compiled without release-evidence acknowledgements"; \
		exit 1; \
	fi
	@if $(CC) $(CFLAGS_COMMON) -Werror $(VEHICLE_VALIDATION_DEFS) \
		-DAMS_EXPECTED_PROFILE=3 $(PROFILE_SRC) \
		-o "$(BUILD_DIR)/profile_vehicle_missing_manifest" $(LDLIBS) \
		>"$(BUILD_DIR)/profile_vehicle_missing_manifest.log" 2>&1; then \
		echo "ERROR: vehicle profile compiled without explicit manifest revisions"; \
		exit 1; \
	fi
	@echo "PASS explicit profiles and fail-closed vehicle evidence/manifest gates"

estimator-topology-gates: check-root make-build-dir
	$(CC) $(CFLAGS_COMMON) -Werror -DAMS_BUILD_PROFILE=1 \
		-DAMS_ESTIMATOR_DEFAULT_TOPOLOGY=1 \
		-DAMS_EXPECTED_ESTIMATOR_INSTANCES=1 \
		$(ESTIMATOR_TOPOLOGY_SRC) $(ESTIMATOR_SRC) \
		-o "$(BUILD_DIR)/estimator_topology_pack" $(LDLIBS)
	"$(BUILD_DIR)/estimator_topology_pack"
	$(CC) $(CFLAGS_COMMON) -Werror -DAMS_BUILD_PROFILE=1 \
		-DAMS_ESTIMATOR_DEFAULT_TOPOLOGY=2 \
		-DAMS_EXPECTED_ESTIMATOR_INSTANCES=5 \
		$(ESTIMATOR_TOPOLOGY_SRC) $(ESTIMATOR_SRC) \
		-o "$(BUILD_DIR)/estimator_topology_segments" $(LDLIBS)
	"$(BUILD_DIR)/estimator_topology_segments"
	$(CC) $(CFLAGS_COMMON) -Werror $(VEHICLE_PROFILE_DEFS) \
		-DAMS_EXPECTED_ESTIMATOR_INSTANCES=5 \
		$(ESTIMATOR_TOPOLOGY_SRC) $(ESTIMATOR_SRC) \
		-o "$(BUILD_DIR)/estimator_topology_vehicle_default" $(LDLIBS)
	"$(BUILD_DIR)/estimator_topology_vehicle_default"
	@if $(CC) $(CFLAGS_COMMON) -Werror -DAMS_BUILD_PROFILE=2 \
		-DAMS_ESTIMATOR_DEFAULT_TOPOLOGY=2 \
		-DAMS_EXPECTED_ESTIMATOR_INSTANCES=5 \
		$(ESTIMATOR_TOPOLOGY_SRC) $(ESTIMATOR_SRC) \
		-o "$(BUILD_DIR)/estimator_topology_hil_invalid" $(LDLIBS) \
		>"$(BUILD_DIR)/estimator_topology_hil_invalid.log" 2>&1; then \
		echo "ERROR: segment topology compiled against pack-only HIL contract"; \
		exit 1; \
	fi
	@echo "PASS pack/segment topology, vehicle five-segment default and pack-only HIL gate"

production-gates: BUILD_DIR = build-production
production-gates: CFLAGS_COMMON += $(VEHICLE_PROFILE_DEFS) -DAMS_HOST_PRODUCTION_GATE_TEST=1
production-gates: check-root
	@mkdir -p "$(BUILD_DIR)"
	$(CC) $(CFLAGS_COMMON) $(SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC) $(POWER_SRC) -o $(BIN) $(LDLIBS)
	$(BIN)
	$(CC) $(CFLAGS_COMMON) -ffunction-sections -fdata-sections \
		$(AMS_ROOT)/Core/Src/app.c $(MEASUREMENT_SRC) $(POWER_SRC) \
		$(ESTIMATOR_SRC) $(OWNER_SRC) \
		-Wl,--gc-sections -o $(OWNER_BIN) $(LDLIBS)
	$(OWNER_BIN)

static-allocation-gate: check-root
	@task_sources="$$(find "$(AMS_ROOT)/Core/Src/tasks" -maxdepth 1 -name '*_task.c' -type f)"; \
	if grep -nE '(^|[^A-Za-z0-9_])xTaskCreate[[:space:]]*\(' $$task_sources; then \
		echo "ERROR: dynamic application task creation is not permitted"; exit 1; \
	fi; \
	count="$$(grep -hE '(^|[^A-Za-z0-9_])xTaskCreateStatic[[:space:]]*\(' $$task_sources | wc -l)"; \
	test "$$count" -eq 9 || \
		(echo "ERROR: expected 9 static application tasks, found $$count"; exit 1)
	@if $(CC) $(CFLAGS_COMMON) -E "$(AMS_ROOT)/Core/Src/main.c" | \
		grep -qE 'defaultTaskHandle[[:space:]]*=[[:space:]]*osThreadNew'; then \
		echo "ERROR: Cube defaultTask is compiled and would allocate dynamically"; \
		exit 1; \
	fi
	@grep -q 'configSUPPORT_STATIC_ALLOCATION[[:space:]]*1' \
		"$(AMS_ROOT)/Core/Inc/FreeRTOSConfig.h"
	@grep -q 'vApplicationGetIdleTaskMemory' \
		"$(AMS_ROOT)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c"
	@grep -q 'vApplicationGetTimerTaskMemory' \
		"$(AMS_ROOT)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c"
	@grep -q 'xQueueCreateStatic' \
		"$(AMS_ROOT)/Middlewares/Third_Party/FreeRTOS/Source/timers.c"
	@grep -q 'cb_mem = &adbms_spi_mutex_cb' "$(AMS_ROOT)/Core/Src/app.c"
	@grep -q 'cb_mem = &current_window_mutex_cb' "$(AMS_ROOT)/Core/Src/app.c"
	@echo "PASS application, idle, timer-task, timer-queue, ADBMS and current-window static-allocation gate"

state-ownership-gate: check-root
	@if grep -R -nE '(data->state|app\.state)[[:space:]]*=[^=]' \
		"$(AMS_ROOT)/Core/Src" "$(AMS_ROOT)/Core/Inc" | \
		grep -v "$(AMS_ROOT)/Core/Src/app.c:" | \
		grep -v "$(AMS_ROOT)/Core/Inc/app.h:"; then \
		echo "ERROR: application state write exists outside app initialization/transition boundary"; \
		exit 1; \
	fi
	@if grep -R -nE 'HAL_CAN_(Start|Stop|ActivateNotification|DeactivateNotification|ConfigFilter)' \
		"$(AMS_ROOT)/Core/Src" | \
		grep -v "$(AMS_ROOT)/Core/Src/ext_drivers/canbus.c:"; then \
		echo "ERROR: CAN control HAL call exists outside CAN service"; \
		exit 1; \
	fi
	@grep -q 'ams_state_transition_begin' "$(AMS_ROOT)/Core/Src/tasks/error_task.c"
	@grep -q 'ams_state_transition_begin' "$(AMS_ROOT)/Core/Src/tasks/cli_task.c"
	@echo "PASS centralized state writers and CAN control ownership gate"

target-project-gate: check-root
	@$(PYTHON) -c 'import xml.etree.ElementTree as E; E.parse("$(AMS_ROOT)/.project"); E.parse("$(AMS_ROOT)/.cproject")'
	@grep -q '<name>DER26-AMS</name>' "$(AMS_ROOT)/.project"
	@grep -q 'project id="DER26-AMS' "$(AMS_ROOT)/.cproject"
	@if grep -q 'DER25-AMS' "$(AMS_ROOT)/.cproject" "$(AMS_ROOT)/.project"; then \
		echo "ERROR: stale DER25 CubeIDE project metadata remains"; exit 1; \
	fi
	@grep -Fq 'buildPath="$${workspace_loc:/$${ProjName}/Debug}"' "$(AMS_ROOT)/.cproject"
	@grep -Fq 'buildPath="$${workspace_loc:/$${ProjName}/Release}"' "$(AMS_ROOT)/.cproject"
	@test "$$(grep -c 'value="-Wall -Wextra"' "$(AMS_ROOT)/.cproject")" -eq 2
	@test "$$(grep -c 'STM32F767ZITX_FLASH.ld' "$(AMS_ROOT)/.cproject")" -eq 4
	@echo "PASS CubeIDE project identity, build paths, warning flags, XML and linker-script metadata"

system-sil: test

# Dedicated final-ring ADBMS2950 gate.  The unit binary links the real driver;
# the system binary exercises topology ownership, advisory fault behavior and
# the CLI through the deterministic SIL transport.
apm-sil: check-root
	@mkdir -p "$(BUILD_DIR)-apm"
	$(CC) $(CFLAGS_COMMON) -DAMS_ENABLE_APM_2950=1 -DAMS_APM_ENABLE_HV_DIVIDERS=0 \
		$(UNIT_SRC) $(ESTIMATOR_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC) \
		-o "$(BUILD_DIR)-apm/ams_apm_unit_tests" $(LDLIBS)
	"$(BUILD_DIR)-apm/ams_apm_unit_tests"
	$(CC) $(CFLAGS_COMMON) $(HOST_EXERCISE_DEFS) \
		-DAMS_ENABLE_APM_2950=1 -DAMS_APM_ENABLE_HV_DIVIDERS=0 \
		$(SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC) $(POWER_SRC) -o "$(BUILD_DIR)-apm/ams_apm_sil_tests" $(LDLIBS)
	"$(BUILD_DIR)-apm/ams_apm_sil_tests"

bringup-test: HOST_HW_BRINGUP = 1
bringup-test: HOST_BALANCE_INHIBIT_DEFAULT = 1
bringup-test: test

hil-adbms-test: CFLAGS_COMMON += -DAMS_HIL_REPLACE_ADBMS=1 -DAMS_HOST_ONLY_HIL_ADBMS_TEST=1
hil-adbms-test: test

imd-test: CFLAGS_COMMON += -DAMS_ENABLE_IMD=1
imd-test: test

air-feedback-stub-test: CFLAGS_COMMON += -DAMS_ENABLE_AIR_AUX_FEEDBACK=1 -DAMS_HOST_AIR_FEEDBACK_GATE_TEST=1 -DAMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS=50u
air-feedback-stub-test: compile
	$(BIN)
	$(CC) $(CFLAGS_COMMON) -Werror \
		-DAMS_AIR_AUX_BOARD_ADAPTER_READY=1 -DAMS_AIR_MONITOR_PERIOD_MS=10u \
		-fsyntax-only $(AMS_ROOT)/Core/Src/app.c \
		$(AMS_ROOT)/Core/Src/tasks/air_task.c $(AIR_MONITOR_SRC)

safety-test: CFLAGS_COMMON += -DAMS_ENABLE_IWDG=1
safety-test: test

stress: CFLAGS_COMMON += -DAMS_HOST_LONG_FUZZ_CYCLES=50000u -DAMS_HOST_CONCURRENT_FUZZ_CYCLES=12000u
stress: test

# Strongest local run: catches many memory and undefined-behavior bugs in host-exercised paths.
asan: CFLAGS_COMMON += $(HOST_EXERCISE_DEFS)
asan: check-root
	@mkdir -p "$(BUILD_DIR)"
	$(CC) $(CFLAGS_COMMON) -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer \
		$(UNIT_SRC) $(ESTIMATOR_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC) -o $(UNIT_BIN)-asan $(LDLIBS)
	ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 $(UNIT_BIN)-asan
	$(CC) $(CFLAGS_COMMON) -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer \
		$(SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC) $(POWER_SRC) -o $(BIN)-asan $(LDLIBS)
	ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 $(BIN)-asan
	$(MAKE) -C sop clean asan
	$(MAKE) -C fuse clean asan
	$(MAKE) -C "$(FUSE_REPLAY_DIR)" clean asan

ubsan: CFLAGS_COMMON += $(HOST_EXERCISE_DEFS)
ubsan: check-root
	@mkdir -p "$(BUILD_DIR)"
	$(CC) $(CFLAGS_COMMON) -fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer \
		$(UNIT_SRC) $(ESTIMATOR_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC) -o $(UNIT_BIN)-ubsan $(LDLIBS)
	UBSAN_OPTIONS=halt_on_error=1 $(UNIT_BIN)-ubsan
	$(CC) $(CFLAGS_COMMON) -fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer \
		$(SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC) $(POWER_SRC) -o $(BIN)-ubsan $(LDLIBS)
	UBSAN_OPTIONS=halt_on_error=1 $(BIN)-ubsan

# GCC static analyzer. Useful for null/bounds/uninitialized regressions.
analyze: CFLAGS_COMMON += $(HOST_EXERCISE_DEFS)
analyze: check-root
	@mkdir -p "$(BUILD_DIR)"
	$(CC) $(CFLAGS_COMMON) -fanalyzer -fsyntax-only $(SRC) $(THERMISTOR_SRC) $(POWER_SRC)
	$(CC) $(CFLAGS_COMMON) -fanalyzer -fsyntax-only $(UNIT_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC)

whole-source-analyze: check-root
	find "$(AMS_ROOT)/Core/Src" -type f -name '*.c' -print0 | \
		xargs -0 $(CC) $(CFLAGS_COMMON) -D_DEFAULT_SOURCE \
		$(HOST_EXERCISE_DEFS) -fanalyzer -fsyntax-only
	find "$(AMS_ROOT)/Core/Src" -type f -name '*.c' -print0 | \
		xargs -0 $(CC) $(CFLAGS_COMMON) -D_DEFAULT_SOURCE \
		$(VEHICLE_PROFILE_DEFS) -fanalyzer -fsyntax-only
	@echo "PASS whole-source GCC analysis in bench and vehicle profiles"

clang-analyze: CFLAGS_COMMON += $(HOST_EXERCISE_DEFS)
clang-analyze: check-root
	@mkdir -p "$(BUILD_DIR)"
	$(CLANG) $(CFLAGS_COMMON) --analyze $(SRC) $(THERMISTOR_SRC) $(POWER_SRC)
	$(CLANG) $(CFLAGS_COMMON) --analyze $(UNIT_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC) $(THERMISTOR_SRC)

list:
	@grep -E '^static void test_' $(SRC) | sed -E 's/static void (test_[^(]+).*/\1/'

clean:
	rm -rf $(BUILD_DIR) build-apm build-production "$(AMS_ROOT)/build"
	$(MAKE) -C sop clean
	$(MAKE) -C "$(DASHBOARD_TEST_DIR)" clean
