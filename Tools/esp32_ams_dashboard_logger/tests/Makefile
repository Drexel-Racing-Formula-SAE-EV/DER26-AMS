CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic -O2
BUILD_DIR := build
TARGET := $(BUILD_DIR)/decoder_smoke_test
RENDER_TARGET := $(BUILD_DIR)/main_render_test

.PHONY: all test syntax clean

all: $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): decoder_smoke_test.c ../main/ams_can_decode.c ../main/ams_can_decode.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I../main decoder_smoke_test.c ../main/ams_can_decode.c -o $@

$(RENDER_TARGET): main_render_test.c ../main/main.c ../main/ams_can_decode.c ../main/ams_can_decode.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -Wformat=2 -Wno-overlength-strings \
		-Istubs -I../main main_render_test.c ../main/ams_can_decode.c -o $@

test: $(TARGET) $(RENDER_TARGET)
	./$(TARGET)
	./$(RENDER_TARGET) | python3 -c 'import json,sys; s=json.load(sys.stdin); p=s["power"]; assert "capacity_lower_pct" in p; assert p["binding_metadata_fresh"] is False; assert len(p["discharge_binding_by_horizon"]) == 3; assert len(p["charge_binding_by_horizon"]) == 3; assert len(p["discharge_segment_by_horizon"]) == 3; assert len(p["charge_segment_by_horizon"]) == 3; print("main_render_test PASS")'

syntax:
	$(CC) -std=c11 -Wall -Wextra -Werror -Wformat=2 -pedantic \
		-Wno-overlength-strings -fsyntax-only \
		-Istubs -I../main ../main/main.c

clean:
	rm -rf $(BUILD_DIR)
