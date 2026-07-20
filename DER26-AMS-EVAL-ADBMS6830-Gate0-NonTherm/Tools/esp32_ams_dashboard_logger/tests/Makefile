CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic -O2
BUILD_DIR := build
TARGET := $(BUILD_DIR)/decoder_smoke_test

.PHONY: all test clean

all: $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): decoder_smoke_test.c ../main/ams_can_decode.c ../main/ams_can_decode.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I../main decoder_smoke_test.c ../main/ams_can_decode.c -o $@

test: $(TARGET)
	./$(TARGET)

clean:
	rm -rf $(BUILD_DIR)
