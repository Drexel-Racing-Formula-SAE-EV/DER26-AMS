CC ?= cc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror
LDLIBS ?= -lm

MODEL_DIR := ../components/plant_model
MAIN_DIR := ../main
BUILD_DIR := build

include $(MODEL_DIR)/plant_model_sources.mk
MODEL_SOURCES := $(addprefix $(MODEL_DIR)/,$(PLANT_MODEL_SOURCE_FILES))

INCLUDES := -I$(MODEL_DIR) -I$(MAIN_DIR)

.PHONY: all test baseline syntax clean

all: $(BUILD_DIR)/plant_model_host_test $(BUILD_DIR)/hil_image_protocol_test $(BUILD_DIR)/mcp2515_tx_status_test $(BUILD_DIR)/mcp2515_driver_host_test $(BUILD_DIR)/baseline_runner $(BUILD_DIR)/parity_runner

$(BUILD_DIR):
	@mkdir -p $@

$(BUILD_DIR)/plant_model_host_test: plant_model_host_test.c $(MODEL_SOURCES) | $(BUILD_DIR)
	@$(CC) $(CFLAGS) $(INCLUDES) $^ $(LDLIBS) -o $@

$(BUILD_DIR)/hil_image_protocol_test: hil_image_protocol_test.c | $(BUILD_DIR)
	@$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@

$(BUILD_DIR)/mcp2515_tx_status_test: mcp2515_tx_status_test.c | $(BUILD_DIR)
	@$(CC) $(CFLAGS) -I../components/CAN $^ -o $@

$(BUILD_DIR)/mcp2515_driver_host_test: mcp2515_driver_host_test.c ../components/CAN/mcp2515_driver.c | $(BUILD_DIR)
	@$(CC) $(CFLAGS) -Ihost_stubs -I../components/CAN $^ -o $@

$(BUILD_DIR)/baseline_runner: baseline_runner.c $(MODEL_SOURCES) | $(BUILD_DIR)
	@$(CC) $(CFLAGS) $(INCLUDES) $^ $(LDLIBS) -o $@

$(BUILD_DIR)/parity_runner: parity_runner.c $(MODEL_SOURCES) | $(BUILD_DIR)
	@$(CC) $(CFLAGS) $(INCLUDES) $^ $(LDLIBS) -o $@

test: $(BUILD_DIR)/plant_model_host_test $(BUILD_DIR)/hil_image_protocol_test $(BUILD_DIR)/mcp2515_tx_status_test $(BUILD_DIR)/mcp2515_driver_host_test
	@./$(BUILD_DIR)/plant_model_host_test
	@./$(BUILD_DIR)/hil_image_protocol_test
	@./$(BUILD_DIR)/mcp2515_tx_status_test
	@./$(BUILD_DIR)/mcp2515_driver_host_test

baseline: $(BUILD_DIR)/baseline_runner
	@./$(BUILD_DIR)/baseline_runner

syntax:
	@$(CC) $(CFLAGS) -fsyntax-only \
		-Ihost_stubs -I../main -I$(MODEL_DIR) ../main/main.c
	@$(CC) $(CFLAGS) -fsyntax-only \
		-Ihost_stubs -I../components/CAN \
		../components/CAN/mcp2515_driver.c

clean:
	@rm -f $(BUILD_DIR)/plant_model_host_test \
		$(BUILD_DIR)/hil_image_protocol_test \
		$(BUILD_DIR)/mcp2515_tx_status_test \
		$(BUILD_DIR)/mcp2515_driver_host_test \
		$(BUILD_DIR)/baseline_runner $(BUILD_DIR)/parity_runner
