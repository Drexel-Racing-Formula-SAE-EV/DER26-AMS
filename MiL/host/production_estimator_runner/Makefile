REPO_ROOT := $(abspath ../../..)
AMS_ROOT := $(REPO_ROOT)/AMS
BUILD_DIR := build
BIN := $(BUILD_DIR)/production_estimator_runner
CC ?= gcc
CFLAGS := -std=c11 -O2 -g -Wall -Wextra -Werror -pedantic \
	-D_POSIX_C_SOURCE=200809L -DAMS_BUILD_PROFILE=1 \
	-I$(AMS_ROOT)/Core/Inc
SRC := main.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_soc_ekf.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c
LDLIBS := -lm

.PHONY: all clean test
all: $(BIN)

$(BIN): $(SRC)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $(SRC) -o $@ $(LDLIBS)

# Self-test exercises a nominal epoch and an invalid epoch. It is deliberately
# tiny; MATLAB qualification drives the full scenario matrix.
test: $(BIN)
	@printf '%s\n' \
	'time_s,sequence,current_A,measurement_valid,current_calibrated,current_polarity_validated,balance_recovered,s0_v,s0_t,s0_valid,s1_v,s1_t,s1_valid,s2_v,s2_t,s2_valid,s3_v,s3_t,s3_valid,s4_v,s4_t,s4_valid' \
	'0.0,1,0,1,1,1,1,57.0,25,1,57.0,25,1,57.0,25,1,57.0,25,1,57.0,25,1' \
	'0.1,2,10,1,1,1,1,56.8,25,1,56.8,25,1,56.8,25,1,56.8,25,1,56.8,25,1' \
	'0.2,3,NaN,0,1,1,1,56.8,25,1,56.8,25,1,56.8,25,1,56.8,25,1,56.8,25,1' > $(BUILD_DIR)/selftest_input.csv
	$(BIN) $(BUILD_DIR)/selftest_input.csv $(BUILD_DIR)/selftest_output.csv 0.5 0.5 0.5 0.5 0.5
	@test "$$(wc -l < $(BUILD_DIR)/selftest_output.csv)" -eq 4
	@grep -q ',0,0,0,' $(BUILD_DIR)/selftest_output.csv
	@echo 'production estimator runner self-test PASS'

clean:
	rm -rf $(BUILD_DIR)
