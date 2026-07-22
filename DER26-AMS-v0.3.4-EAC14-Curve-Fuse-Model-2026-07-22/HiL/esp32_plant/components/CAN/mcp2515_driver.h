/*
 * mcp2515_driver.h
 *
 *  Created on: Apr 17, 2026
 *      Author: mahad
 */

#ifndef COMPONENTS_CAN_MCP2515_DRIVER_H_
#define COMPONENTS_CAN_MCP2515_DRIVER_H_

/*
 * mcp2515_driver.h
 *
 * Minimal MCP2515 TX/RX driver for ESP-IDF.
 * Configured for 250 kbit/s, 8 MHz crystal, matching AMS CAN1.
 * SPI2_HOST (HSPI): MOSI=23 MISO=19 CLK=18 CS=5 INT=4
 */

#include <stdint.h>
#include "esp_err.h"

/* Pin assignments — match your wiring */
#define MCP_SPI_HOST    SPI2_HOST
#define MCP_PIN_MOSI    23
#define MCP_PIN_MISO    19
#define MCP_PIN_CLK     18
#define MCP_PIN_CS       5
#define MCP_PIN_INT      4
#define MCP_SPI_CLK_HZ  10000000   /* 10 MHz — MCP2515 max 10 MHz */

/*
 * mcp2515_init()
 * Resets MCP2515, configures for 250 kbit/s @ 8 MHz, enters Normal mode.
 * Returns ESP_OK on success.
 */
esp_err_t mcp2515_init(void);

/*
 * mcp2515_send_frame()
 * Loads TXB0 and issues RTS. Blocks until TXB0 clears.
 * id  : 11-bit standard CAN ID
 * data: up to 8 bytes
 * len : DLC (0–8)
 * Returns ESP_OK, ESP_ERR_TIMEOUT, or ESP_FAIL on bus error.
 */
esp_err_t mcp2515_send_frame(uint16_t id, const uint8_t *data, uint8_t len);

esp_err_t mcp2515_read_frame(uint16_t *id, uint8_t *data, uint8_t *len);



#endif /* COMPONENTS_CAN_MCP2515_DRIVER_H_ */
