/*
 * MCP2515 Classic CAN driver for the DER26 ESP32 AMS dashboard/logger.
 *
 * Wiring matches the existing ESP32 plant-node reference:
 *   ESP32 GPIO23 -> MCP2515 SI/MOSI
 *   ESP32 GPIO19 -> MCP2515 SO/MISO
 *   ESP32 GPIO18 -> MCP2515 SCK
 *   ESP32 GPIO5  -> MCP2515 CS
 *   ESP32 GPIO4  -> MCP2515 INT (optional; this driver polls)
 *
 * The AMS firmware currently configures CAN1 for 250 kbit/s, so this driver is
 * configured for 250 kbit/s with an 8 MHz MCP2515 crystal.
 */

#ifndef COMPONENTS_CAN_MCP2515_DRIVER_H_
#define COMPONENTS_CAN_MCP2515_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#define MCP_SPI_HOST    SPI2_HOST
#define MCP_PIN_MOSI    23
#define MCP_PIN_MISO    19
#define MCP_PIN_CLK     18
#define MCP_PIN_CS       5
#define MCP_PIN_INT      4
#define MCP_SPI_CLK_HZ  10000000

typedef struct
{
    uint32_t id;
    bool extended;
    uint8_t dlc;
    uint8_t data[8];
} mcp2515_frame_t;

esp_err_t mcp2515_init(void);
esp_err_t mcp2515_read_frame(mcp2515_frame_t *frame);

#endif /* COMPONENTS_CAN_MCP2515_DRIVER_H_ */
