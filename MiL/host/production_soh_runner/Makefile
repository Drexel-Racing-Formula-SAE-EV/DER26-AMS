REPO_ROOT := $(abspath ../../..)
AMS_ROOT := $(REPO_ROOT)/AMS
BUILD_DIR := build
BIN := $(BUILD_DIR)/production_soh_runner
CC ?= gcc
CFLAGS := -std=c11 -O2 -g -Wall -Wextra -Werror -pedantic -D_POSIX_C_SOURCE=200809L -I$(AMS_ROOT)/Core/Inc
SRC := main.c $(AMS_ROOT)/Core/Src/soh/ams_soh.c
LDLIBS := -lm
.PHONY: all clean test
all: $(BIN)
$(BIN): $(SRC)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $(SRC) -o $@ $(LDLIBS)
clean:
	rm -rf $(BUILD_DIR)
test: $(BIN)
	@mkdir -p $(BUILD_DIR)
	$(BIN) selftest_input.csv $(BUILD_DIR)/selftest_output.csv
	@test "$$(wc -l < $(BUILD_DIR)/selftest_output.csv)" -eq 3
	@echo "production SoH runner self-test PASS"
