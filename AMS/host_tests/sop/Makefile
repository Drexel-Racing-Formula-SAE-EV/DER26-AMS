AMS_ROOT ?= ../..
CC ?= gcc
CFLAGS := -std=c11 -O2 -Wall -Wextra -Werror \
	-Wno-int-to-pointer-cast -Wno-pointer-to-int-cast \
	-DSTM32F767xx -DUSE_HAL_DRIVER -DAMS_HOST_TEST=1 \
	-I$(AMS_ROOT)/host_tests/include \
	-I$(AMS_ROOT)/Core/Inc \
	-I$(AMS_ROOT)/Drivers/CMSIS/Include \
	-I$(AMS_ROOT)/Drivers/CMSIS/Device/ST/STM32F7xx/Include \
	-I$(AMS_ROOT)/Drivers/STM32F7xx_HAL_Driver/Inc \
	-I$(AMS_ROOT)/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
SRC := sop_test.c \
	$(AMS_ROOT)/Core/Src/sop/ams_sop.c \
	$(AMS_ROOT)/Core/Src/sop/ams_fuse_observer.c \
	$(AMS_ROOT)/Core/Src/sop/ams_power_strategy.c \
	$(AMS_ROOT)/Core/Src/sop/ams_power_state.c \
	$(AMS_ROOT)/Core/Src/sop/ams_power_can.c \
	$(AMS_ROOT)/Core/Src/soh/ams_soh.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c
BENCH_SRC := sop_benchmark.c \
	$(AMS_ROOT)/Core/Src/sop/ams_sop.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c
METAMORPHIC_SRC := sop_metamorphic_oracle.c \
	$(AMS_ROOT)/Core/Src/sop/ams_sop.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c
SENSITIVITY_SRC := sop_sensitivity_probe.c \
	$(AMS_ROOT)/Core/Src/sop/ams_sop.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c
METAMORPHIC_BIN := sop_metamorphic_oracle
SENSITIVITY_BIN := sop_sensitivity_probe
SOP_METAMORPHIC_STATES ?= 20000
SOP_METAMORPHIC_ASAN_STATES ?= 500

.PHONY: all test metamorphic sensitivity asan metamorphic-asan benchmark stack-report clean

all: test metamorphic
test:
	$(CC) $(CFLAGS) $(SRC) -lm -o sop_test
	./sop_test

metamorphic:
	$(CC) $(CFLAGS) $(METAMORPHIC_SRC) -lm -o $(METAMORPHIC_BIN)
	./$(METAMORPHIC_BIN) $(SOP_METAMORPHIC_STATES)

# Characterization only: intentionally not part of the pass/fail CI gate.
sensitivity:
	$(CC) $(CFLAGS) $(SENSITIVITY_SRC) -lm -o $(SENSITIVITY_BIN)
	./$(SENSITIVITY_BIN)

asan:
	$(CC) $(CFLAGS) -O1 -g -fsanitize=address,undefined $(SRC) -lm -o sop_test_asan
	ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 ./sop_test_asan
	$(MAKE) metamorphic-asan

metamorphic-asan:
	$(CC) $(CFLAGS) -O1 -g -fsanitize=address,undefined \
		$(METAMORPHIC_SRC) -lm -o $(METAMORPHIC_BIN)_asan
	ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
		./$(METAMORPHIC_BIN)_asan $(SOP_METAMORPHIC_ASAN_STATES)

benchmark:
	$(CC) $(CFLAGS) $(BENCH_SRC) -lm -o sop_benchmark
	./sop_benchmark
stack-report:
	mkdir -p build-resources
	$(CC) $(CFLAGS) -O2 -fstack-usage -c \
	$(AMS_ROOT)/Core/Src/sop/ams_sop.c -o build-resources/ams_sop.o
	$(CC) $(CFLAGS) -O2 -fstack-usage -c \
		$(AMS_ROOT)/Core/Src/sop/ams_fuse_observer.c \
		-o build-resources/ams_fuse_observer.o
	$(CC) $(CFLAGS) -O2 -fstack-usage -c \
		$(AMS_ROOT)/Core/Src/sop/ams_power_strategy.c \
		-o build-resources/ams_power_strategy.o
	$(CC) $(CFLAGS) -O2 -fstack-usage -c \
		$(AMS_ROOT)/Core/Src/soh/ams_soh.c -o build-resources/ams_soh.o
	$(CC) $(CFLAGS) -O2 -fstack-usage -c \
		$(AMS_ROOT)/Core/Src/sop/ams_power_state.c \
		-o build-resources/ams_power_state.o
	@cat build-resources/*.su
clean:
	rm -rf sop_test sop_test_asan sop_benchmark $(METAMORPHIC_BIN) $(METAMORPHIC_BIN)_asan $(SENSITIVITY_BIN) build-resources
