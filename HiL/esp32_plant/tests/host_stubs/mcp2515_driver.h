#ifndef HOST_STUB_MCP2515_DRIVER_H_
#define HOST_STUB_MCP2515_DRIVER_H_

#include <stdint.h>

#include "esp_log.h"

typedef struct {
    uint32_t successful_frames;
    uint32_t arbitration_lost_events;
    uint32_t transmit_error_events;
    uint32_t controller_retry_events;
    uint32_t aborted_frames;
    uint32_t bus_off_failures;
    uint32_t timeout_failures;
    uint32_t spi_failures;
    uint32_t transmit_warning_observations;
    uint32_t transmit_passive_observations;
    uint32_t receive_buffer_0_overflows;
    uint32_t receive_buffer_1_overflows;
} mcp2515_diagnostics_t;

esp_err_t mcp2515_init(void);
esp_err_t mcp2515_read_frame(uint16_t *id, uint8_t *data, uint8_t *length);
esp_err_t mcp2515_send_frame(uint16_t id, const uint8_t *data, uint8_t length);
void mcp2515_get_diagnostics(mcp2515_diagnostics_t *diagnostics);
void mcp2515_reset_diagnostics(void);

#endif /* HOST_STUB_MCP2515_DRIVER_H_ */
