REPO_ROOT := ../..
AMS_ROOT := $(REPO_ROOT)/AMS
CC ?= gcc
CFLAGS := -std=c11 -O2 -Wall -Wextra -Werror -pedantic \
	-Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion \
	-DSTM32F767xx -DUSE_HAL_DRIVER -DAMS_HOST_TEST=1 \
	-I. \
	-I$(AMS_ROOT)/host_tests/include \
	-I$(AMS_ROOT)/Core/Inc \
	-I$(AMS_ROOT)/Drivers/CMSIS/Include \
	-I$(AMS_ROOT)/Drivers/CMSIS/Device/ST/STM32F7xx/Include \
	-I$(AMS_ROOT)/Drivers/STM32F7xx_HAL_Driver/Inc \
	-I$(AMS_ROOT)/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
SRC := fuse_replay_main.c fuse_replay.c fuse_reference_oracle.c \
	$(AMS_ROOT)/Core/Src/sop/ams_fuse_observer.c \
	$(AMS_ROOT)/Core/Src/sop/ams_sop.c \
	$(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c
BIN := fuse_replay
ASAN_BIN := fuse_replay_asan

.PHONY: all clean traces policy-sweep sample smoke smoke-ci asan
all: $(BIN)

$(BIN): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -lm -o $(BIN)

traces:
	python3 generate_synthetic_traces.py --output-dir traces

sample: $(BIN) traces
	mkdir -p results
	./$(BIN) --trace traces/single_100a_1p5s.csv \
		--output results/single_100a_1p5s_replay.csv \
		--summary results/single_100a_1p5s_summary.csv --strict

smoke-ci: $(BIN)
	@test -f traces/invalid_samples.csv
	@test -f traces/reset_pre_exhaust.csv
	./$(BIN) --trace traces/invalid_samples.csv \
		--output /tmp/der26_fuse_invalid_replay.csv \
		--summary /tmp/der26_fuse_invalid_summary.csv --strict
	./$(BIN) --trace traces/reset_pre_exhaust.csv \
		--output /tmp/der26_fuse_reset_replay.csv \
		--summary /tmp/der26_fuse_reset_summary.csv \
		--reset seeded:0.8 --strict
	./$(BIN) --trace traces/reset_midrun.csv \
		--output /tmp/der26_fuse_restore_replay.csv \
		--summary /tmp/der26_fuse_restore_summary.csv \
		--reset restore --strict
	rm -f /tmp/der26_fuse_invalid_replay.csv /tmp/der26_fuse_invalid_summary.csv \
		/tmp/der26_fuse_reset_replay.csv /tmp/der26_fuse_reset_summary.csv \
		/tmp/der26_fuse_restore_replay.csv /tmp/der26_fuse_restore_summary.csv

asan: traces
	$(CC) $(CFLAGS) -O1 -g -fsanitize=address,undefined \
		$(SRC) -lm -o $(ASAN_BIN)
	ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 ./$(ASAN_BIN) \
		--trace traces/invalid_samples.csv --output /tmp/der26_fuse_asan.csv \
		--summary /tmp/der26_fuse_asan_summary.csv --strict
	rm -f /tmp/der26_fuse_asan.csv /tmp/der26_fuse_asan_summary.csv

smoke: $(BIN) traces
	mkdir -p results/smoke
	./$(BIN) --trace traces/invalid_samples.csv \
		--output results/smoke/invalid_samples_replay.csv \
		--summary results/smoke/invalid_samples_summary.csv --strict
	./$(BIN) --trace traces/reset_pre_exhaust.csv \
		--output results/smoke/reset_pre_exhaust_replay.csv \
		--summary results/smoke/reset_pre_exhaust_summary.csv \
		--reset seeded:0.8 --strict

policy-sweep: $(BIN) traces
	python3 run_policy_sweep.py --binary ./$(BIN) \
		--trace-dir traces --output results/policy_sweep.csv

clean:
	rm -f $(BIN) $(ASAN_BIN)
