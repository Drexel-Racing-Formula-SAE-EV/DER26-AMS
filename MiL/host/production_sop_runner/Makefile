REPO_ROOT := $(abspath ../../..)
AMS_ROOT := $(REPO_ROOT)/AMS
BUILD_DIR := build
BIN := $(BUILD_DIR)/production_sop_runner
CC ?= gcc
CFLAGS := -std=c11 -O2 -g -Wall -Wextra -Werror -pedantic \
	-D_POSIX_C_SOURCE=200809L -DAMS_BUILD_PROFILE=1 -I$(AMS_ROOT)/Core/Inc
SRC := main.c $(AMS_ROOT)/Core/Src/sop/ams_sop.c $(AMS_ROOT)/Core/Src/estimator/ams_estimator_lut.c
LDLIBS := -lm
.PHONY: all clean
all: $(BIN)
$(BIN): $(SRC)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $(SRC) -o $@ $(LDLIBS)
clean:
	rm -rf $(BUILD_DIR)
