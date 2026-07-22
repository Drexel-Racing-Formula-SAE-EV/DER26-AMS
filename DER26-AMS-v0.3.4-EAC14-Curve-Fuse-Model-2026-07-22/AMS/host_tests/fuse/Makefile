AMS_ROOT ?= ../..
REPO_ROOT := $(abspath $(AMS_ROOT)/..)
TOOL_DIR := $(REPO_ROOT)/Tools/fuse_replay
CC ?= gcc
CFLAGS := -std=c11 -O2 -Wall -Wextra -Werror -pedantic \
	-Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion \
	-DSTM32F767xx -DUSE_HAL_DRIVER -DAMS_HOST_TEST=1 \
	-I$(AMS_ROOT)/host_tests/include \
	-I$(AMS_ROOT)/Core/Inc \
	-I$(TOOL_DIR) \
	-I$(AMS_ROOT)/Drivers/CMSIS/Include \
	-I$(AMS_ROOT)/Drivers/CMSIS/Device/ST/STM32F7xx/Include \
	-I$(AMS_ROOT)/Drivers/STM32F7xx_HAL_Driver/Inc \
	-I$(AMS_ROOT)/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
SRC := fuse_oracle_test.c \
	$(TOOL_DIR)/fuse_reference_oracle.c \
	$(AMS_ROOT)/Core/Src/sop/ams_fuse_observer.c \
	$(AMS_ROOT)/Core/Src/sop/ams_sop.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c
BIN := fuse_oracle_test

.PHONY: all test asan clean
all: test

test:
	$(CC) $(CFLAGS) $(SRC) -lm -o $(BIN)
	./$(BIN)

asan:
	$(CC) $(CFLAGS) -O1 -g -fsanitize=address,undefined \
		$(SRC) -lm -o $(BIN)_asan
	ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 ./$(BIN)_asan

clean:
	rm -f $(BIN) $(BIN)_asan
