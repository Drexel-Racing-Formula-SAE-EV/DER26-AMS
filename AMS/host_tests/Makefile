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
BIN := $(BUILD_DIR)/ams_host_tests
UNIT_SRC := unit/ams_unit_test_runner.c
UNIT_BIN := $(BUILD_DIR)/ams_unit_tests
ESTIMATOR_SRC := \
	$(AMS_ROOT)/Core/Src/estimator/ams_soc_ekf.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c

ADBMS2950_SRC := \
	$(AMS_ROOT)/Core/Src/ext_drivers/adbms2950.c

COMMON_DEFS := \
	-DSTM32F767xx \
	-DUSE_HAL_DRIVER \
	-DAMS_HOST_TEST=1

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

.PHONY: help check-root compile unit test system-sil bringup-test ci asan ubsan analyze clang-analyze clean list make-build-dir

help:
	@echo "DER26 AMS host test harness"
	@echo ""
	@echo "Targets:"
	@echo "  make unit          Build and run isolated unit tests"
	@echo "  make test          Build and run comprehensive host tests"
	@echo "  make system-sil    Alias for comprehensive system-level SIL/injection tests"
	@echo "  make bringup-test  Run comprehensive tests with AMS_HW_BRINGUP=1"
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
	$(CC) $(CFLAGS_COMMON) $(UNIT_SRC) $(ESTIMATOR_SRC) $(ADBMS2950_SRC) -o $(UNIT_BIN) $(LDLIBS)
	$(UNIT_BIN)

ci: unit test analyze

# Normal run: catches logic regressions and compile/link problems.
test: compile
	$(BIN)

system-sil: test

bringup-test: CFLAGS_COMMON += -DAMS_HW_BRINGUP=1
bringup-test: test

# Strongest local run: catches many memory and undefined-behavior bugs in host-exercised paths.
asan: check-root $(BUILD_DIR)
	$(CC) $(CFLAGS_COMMON) -fsanitize=address,undefined -fno-omit-frame-pointer $(SRC) -o $(BIN)-asan $(LDLIBS)
	ASAN_OPTIONS=detect_leaks=0 $(BIN)-asan

ubsan: check-root $(BUILD_DIR)
	$(CC) $(CFLAGS_COMMON) -fsanitize=undefined -fno-omit-frame-pointer $(SRC) -o $(BIN)-ubsan $(LDLIBS)
	$(BIN)-ubsan

# GCC static analyzer. Useful for null/bounds/uninitialized regressions.
analyze: check-root $(BUILD_DIR)
	$(CC) $(CFLAGS_COMMON) -fanalyzer -fsyntax-only $(SRC)

clang-analyze: check-root $(BUILD_DIR)
	$(CLANG) $(CFLAGS_COMMON) --analyze $(SRC)

list:
	@grep -E '^static void test_' $(SRC) | sed -E 's/static void (test_[^(]+).*/\1/'

clean:
	rm -rf $(BUILD_DIR)
