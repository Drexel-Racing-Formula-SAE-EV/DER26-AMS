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
BUILD_DIR ?= build
CC ?= gcc
CLANG ?= clang

SRC := src/ams_host_test_runner.c
BIN = $(BUILD_DIR)/ams_host_tests
UNIT_SRC := unit/ams_unit_test_runner.c
UNIT_BIN = $(BUILD_DIR)/ams_unit_tests
OWNER_SRC := production/bms_owner_test.c
OWNER_BIN = $(BUILD_DIR)/bms_owner_test
ESTIMATOR_SRC := \
	$(AMS_ROOT)/Core/Src/estimator/ams_soc_ekf.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c

ADBMS2950_SRC := \
	$(AMS_ROOT)/Core/Src/ext_drivers/adbms2950.c

AIR_MONITOR_SRC := \
	$(AMS_ROOT)/Core/Src/ext_drivers/air_monitor.c

COMMON_DEFS := \
	-DSTM32F767xx \
	-DUSE_HAL_DRIVER \
	-DAMS_HOST_TEST=1

HOST_EXERCISE_DEFS := \
	-DAMS_ENABLE_HIL_CAN=1 \
	-DAMS_ENABLE_SERVICE_CLI=1

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

.PHONY: help check-root compile unit test production-gates system-sil apm-sil bringup-test hil-adbms-test imd-test air-feedback-stub-test safety-test stress ci asan ubsan analyze clang-analyze clean list make-build-dir

help:
	@echo "DER26 AMS host test harness"
	@echo ""
	@echo "Targets:"
	@echo "  make unit          Build and run isolated unit tests"
	@echo "  make test          Build and run comprehensive host tests"
	@echo "  make production-gates Verify HIL/service CLI are disabled by default"
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


build:
	mkdir -p $(BUILD_DIR)
make-build-dir:
	@mkdir -p "$(BUILD_DIR)"

compile: check-root make-build-dir
	$(CC) $(CFLAGS_COMMON) $(SRC) -o $(BIN) $(LDLIBS)

unit: check-root make-build-dir
	$(CC) $(CFLAGS_COMMON) $(UNIT_SRC) $(ESTIMATOR_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC) -o $(UNIT_BIN) $(LDLIBS)
	$(UNIT_BIN)

ci: unit test apm-sil production-gates analyze

# Normal run: catches logic regressions and compile/link problems.
test: CFLAGS_COMMON += $(HOST_EXERCISE_DEFS)
test: compile
	$(BIN)

production-gates: BUILD_DIR = build-production
production-gates: CFLAGS_COMMON += -DAMS_HOST_PRODUCTION_GATE_TEST=1
production-gates: check-root
	@mkdir -p "$(BUILD_DIR)"
	$(CC) $(CFLAGS_COMMON) $(SRC) -o $(BIN) $(LDLIBS)
	$(BIN)
	$(CC) $(CFLAGS_COMMON) -ffunction-sections -fdata-sections \
		$(AMS_ROOT)/Core/Src/app.c $(OWNER_SRC) \
		-Wl,--gc-sections -o $(OWNER_BIN) $(LDLIBS)
	$(OWNER_BIN)

system-sil: test

# Dedicated final-ring ADBMS2950 gate.  The unit binary links the real driver;
# the system binary exercises topology ownership, advisory fault behavior and
# the CLI through the deterministic SIL transport.
apm-sil: check-root
	@mkdir -p "$(BUILD_DIR)-apm"
	$(CC) $(CFLAGS_COMMON) -DAMS_ENABLE_APM_2950=1 -DAMS_APM_ENABLE_HV_DIVIDERS=0 \
		$(UNIT_SRC) $(ESTIMATOR_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC) \
		-o "$(BUILD_DIR)-apm/ams_apm_unit_tests" $(LDLIBS)
	"$(BUILD_DIR)-apm/ams_apm_unit_tests"
	$(CC) $(CFLAGS_COMMON) $(HOST_EXERCISE_DEFS) \
		-DAMS_ENABLE_APM_2950=1 -DAMS_APM_ENABLE_HV_DIVIDERS=0 \
		$(SRC) -o "$(BUILD_DIR)-apm/ams_apm_sil_tests" $(LDLIBS)
	"$(BUILD_DIR)-apm/ams_apm_sil_tests"

bringup-test: CFLAGS_COMMON += -DAMS_HW_BRINGUP=1
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
	$(CC) $(CFLAGS_COMMON) -fsanitize=address,undefined -fno-omit-frame-pointer \
		$(UNIT_SRC) $(ESTIMATOR_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC) -o $(UNIT_BIN)-asan $(LDLIBS)
	ASAN_OPTIONS=detect_leaks=0 $(UNIT_BIN)-asan
	$(CC) $(CFLAGS_COMMON) -fsanitize=address,undefined -fno-omit-frame-pointer $(SRC) -o $(BIN)-asan $(LDLIBS)
	ASAN_OPTIONS=detect_leaks=0 $(BIN)-asan

ubsan: CFLAGS_COMMON += $(HOST_EXERCISE_DEFS)
ubsan: check-root
	@mkdir -p "$(BUILD_DIR)"
	$(CC) $(CFLAGS_COMMON) -fsanitize=undefined -fno-omit-frame-pointer \
		$(UNIT_SRC) $(ESTIMATOR_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC) -o $(UNIT_BIN)-ubsan $(LDLIBS)
	$(UNIT_BIN)-ubsan
	$(CC) $(CFLAGS_COMMON) -fsanitize=undefined -fno-omit-frame-pointer $(SRC) -o $(BIN)-ubsan $(LDLIBS)
	$(BIN)-ubsan

# GCC static analyzer. Useful for null/bounds/uninitialized regressions.
analyze: CFLAGS_COMMON += $(HOST_EXERCISE_DEFS)
analyze: check-root
	@mkdir -p "$(BUILD_DIR)"
	$(CC) $(CFLAGS_COMMON) -fanalyzer -fsyntax-only $(SRC)
	$(CC) $(CFLAGS_COMMON) -fanalyzer -fsyntax-only $(UNIT_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC)

clang-analyze: CFLAGS_COMMON += $(HOST_EXERCISE_DEFS)
clang-analyze: check-root
	@mkdir -p "$(BUILD_DIR)"
	$(CLANG) $(CFLAGS_COMMON) --analyze $(SRC)
	$(CLANG) $(CFLAGS_COMMON) --analyze $(UNIT_SRC) $(ADBMS2950_SRC) $(AIR_MONITOR_SRC)

list:
	@grep -E '^static void test_' $(SRC) | sed -E 's/static void (test_[^(]+).*/\1/'

clean:
	rm -rf $(BUILD_DIR)
