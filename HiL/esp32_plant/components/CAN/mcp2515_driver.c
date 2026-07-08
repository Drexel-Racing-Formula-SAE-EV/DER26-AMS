/*
 * mcp2515_driver.c
 *
 * Minimal MCP2515 Classic CAN driver for ESP-IDF.
 * Configured for 250 kbit/s with an 8 MHz MCP2515 crystal to match AMS CAN1.
 *
 * Uses standard 11-bit identifiers only. TX uses TXB0. RX polls RXB0/RXB1.
 */

#include "mcp2515_driver.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

static const char *TAG = "MCP2515";
static spi_device_handle_t s_spi = NULL;
/* MCP2515 instruction codes. */
#define MCP_RESET       0xC0U
#define MCP_READ        0x03U
#define MCP_WRITE       0x02U
#define MCP_BIT_MODIFY  0x05U
#define MCP_LOAD_TX0    0x40U
#define MCP_RTS_TX0     0x81U
#define MCP_READ_STATUS 0xA0U

/* MCP2515 register addresses. */
#define REG_RXM0SIDH  0x20U
#define REG_RXM0SIDL  0x21U
#define REG_RXM0EID8  0x22U
#define REG_RXM0EID0  0x23U
#define REG_RXM1SIDH  0x24U
#define REG_RXM1SIDL  0x25U
#define REG_RXM1EID8  0x26U
#define REG_RXM1EID0  0x27U
#define REG_CNF3      0x28U
#define REG_CNF2      0x29U
#define REG_CNF1      0x2AU
#define REG_CANINTE   0x2BU
#define REG_CANINTF   0x2CU
#define REG_EFLG      0x2DU
#define REG_TXB0CTRL  0x30U
#define REG_RXB0CTRL  0x60U
#define REG_RXB0SIDH  0x61U
#define REG_RXB1CTRL  0x70U
#define REG_RXB1SIDH  0x71U
#define REG_CANSTAT   0x0EU
#define REG_CANCTRL   0x0FU

/* CANCTRL mode bits. */
#define MODE_CONFIG  0x80U
#define MODE_NORMAL  0x00U
#define MODE_MASK    0xE0U

#define CANINTF_RX0IF 0x01U
#define CANINTF_RX1IF 0x02U
#define TXB0CTRL_TXREQ 0x08U
#define TXB0CTRL_TXERR 0x10U

#define MCP_RETURN_ON_ERROR(expr, tag, msg)      \
    do {                                          \
        esp_err_t _err = (expr);                  \
        if (_err != ESP_OK) {                     \
            ESP_LOGE(tag, "%s: %d", msg, _err);  \
            return _err;                          \
        }                                         \
    } while (0)

static esp_err_t spi_tx(const void *tx, size_t nbytes)
{
    if ((s_spi == NULL) || (tx == NULL) || (nbytes == 0U)) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t t = {
        .length = nbytes * 8U,
        .tx_buffer = tx,
    };

    return spi_device_transmit(s_spi, &t);
}

static esp_err_t spi_txrx(const void *tx, void *rx, size_t nbytes)
{
    if ((s_spi == NULL) || (tx == NULL) || (rx == NULL) || (nbytes == 0U)) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t t = {
        .length = nbytes * 8U,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    return spi_device_transmit(s_spi, &t);
}

static esp_err_t spi_write_byte(uint8_t reg, uint8_t val)
{
    const uint8_t tx[3] = { MCP_WRITE, reg, val };
    return spi_tx(tx, sizeof(tx));
}

static esp_err_t spi_read_byte(uint8_t reg, uint8_t *val)
{
    if (val == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t tx[3] = { MCP_READ, reg, 0x00U };
    uint8_t rx[3] = { 0U };
    esp_err_t err = spi_txrx(tx, rx, sizeof(tx));
    if (err == ESP_OK) {
        *val = rx[2];
    }
    return err;
}

static esp_err_t spi_bit_modify(uint8_t reg, uint8_t mask, uint8_t data)
{
    const uint8_t tx[4] = { MCP_BIT_MODIFY, reg, mask, data };
    return spi_tx(tx, sizeof(tx));
}

static esp_err_t spi_reset(void)
{
    const uint8_t tx = MCP_RESET;
    esp_err_t err = spi_tx(&tx, sizeof(tx));
    vTaskDelay(pdMS_TO_TICKS(10));
    return err;
}

static esp_err_t spi_read_buf(uint8_t addr, uint8_t *buf, size_t len)
{
    if ((buf == NULL) || (len == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (len > 13U) {
        len = 13U;
    }

    uint8_t tx[2U + 13U] = { 0U };
    uint8_t rx[2U + 13U] = { 0U };
    tx[0] = MCP_READ;
    tx[1] = addr;

    esp_err_t err = spi_txrx(tx, rx, 2U + len);
    if (err == ESP_OK) {
        memcpy(buf, &rx[2], len);
    }
    return err;
}

static esp_err_t wait_for_mode(uint8_t target_mode, uint32_t timeout_ms)
{
    for (uint32_t elapsed = 0U; elapsed < timeout_ms; elapsed++) {
        uint8_t stat = 0U;
        esp_err_t err = spi_read_byte(REG_CANSTAT, &stat);
        if (err != ESP_OK) {
            return err;
        }
        if ((stat & MODE_MASK) == target_mode) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t mcp2515_init(void)
{
    if (s_spi != NULL) {
        return ESP_OK;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = MCP_PIN_MOSI,
        .miso_io_num = MCP_PIN_MISO,
        .sclk_io_num = MCP_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    esp_err_t err = spi_bus_initialize(MCP_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %d", err);
        return err;
    }
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = MCP_SPI_CLK_HZ,
        .mode = 0,
        .spics_io_num = MCP_PIN_CS,
        .queue_size = 4,
    };

    err = spi_bus_add_device(MCP_SPI_HOST, &dev_cfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %d", err);
        s_spi = NULL;
        return err;
    }

    err = spi_reset();
    if (err != ESP_OK) {
        return err;
    }

    err = wait_for_mode(MODE_CONFIG, 100U);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Did not enter config mode after reset");
        return err;
    }

    /* 250 kbit/s with an 8 MHz crystal:
     * BRP=0 => TQ=250 ns, 16 TQ/bit.
     * Sync=1, PropSeg=5, PHSEG1=8, PHSEG2=2, sample point 87.5%.
     */
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_CNF1, 0x00U), TAG, "CNF1 write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_CNF2, 0xBCU), TAG, "CNF2 write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_CNF3, 0x01U), TAG, "CNF3 write failed");

    /* Polling driver: no MCP2515 interrupt pin required. */
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_CANINTE, 0x00U), TAG, "CANINTE write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_CANINTF, 0x00U), TAG, "CANINTF clear failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_EFLG, 0x00U), TAG, "EFLG clear failed");

    /* Accept all standard/extended frames into RXB0/RXB1. The application filters by ID. */
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_RXB0CTRL, 0x60U), TAG, "RXB0CTRL write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_RXB1CTRL, 0x60U), TAG, "RXB1CTRL write failed");

    MCP_RETURN_ON_ERROR(spi_write_byte(REG_RXM0SIDH, 0x00U), TAG, "RXM0SIDH write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_RXM0SIDL, 0x00U), TAG, "RXM0SIDL write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_RXM0EID8, 0x00U), TAG, "RXM0EID8 write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_RXM0EID0, 0x00U), TAG, "RXM0EID0 write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_RXM1SIDH, 0x00U), TAG, "RXM1SIDH write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_RXM1SIDL, 0x00U), TAG, "RXM1SIDL write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_RXM1EID8, 0x00U), TAG, "RXM1EID8 write failed");
    MCP_RETURN_ON_ERROR(spi_write_byte(REG_RXM1EID0, 0x00U), TAG, "RXM1EID0 write failed");

    MCP_RETURN_ON_ERROR(spi_bit_modify(REG_CANCTRL, MODE_MASK, MODE_NORMAL), TAG, "normal mode request failed");

    err = wait_for_mode(MODE_NORMAL, 100U);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Did not enter normal mode");
        return err;
    }

    ESP_LOGI(TAG, "MCP2515 ready: 250 kbit/s @ 8 MHz");
    return ESP_OK;
}

esp_err_t mcp2515_send_frame(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (s_spi == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((data == NULL) && (len > 0U)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (id > 0x7FFU) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > 8U) {
        len = 8U;
    }

    uint8_t tx_buf[14] = { 0U };
    tx_buf[0] = MCP_LOAD_TX0;
    tx_buf[1] = (uint8_t)(id >> 3);
    tx_buf[2] = (uint8_t)((id & 0x07U) << 5);
    tx_buf[5] = len & 0x0FU;
    if (len > 0U) {
        memcpy(&tx_buf[6], data, len);
    }

    esp_err_t err = spi_tx(tx_buf, 6U + len);
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t rts = MCP_RTS_TX0;
    err = spi_tx(&rts, sizeof(rts));
    if (err != ESP_OK) {
        return err;
    }

    for (uint32_t i = 0U; i < 20U; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        uint8_t ctrl = 0U;
        err = spi_read_byte(REG_TXB0CTRL, &ctrl);
        if (err != ESP_OK) {
            return err;
        }
        if ((ctrl & TXB0CTRL_TXREQ) == 0U) {
            return ESP_OK;
        }
        if ((ctrl & TXB0CTRL_TXERR) != 0U) {
            (void)spi_write_byte(REG_CANINTF, 0x00U);
            ESP_LOGW(TAG, "TX bus error, ctrl=0x%02X", ctrl);
            return ESP_FAIL;
        }
    }

    ESP_LOGW(TAG, "TX timeout");
    return ESP_ERR_TIMEOUT;
}

esp_err_t mcp2515_read_frame(uint16_t *id, uint8_t *data, uint8_t *len)
{
    if ((id == NULL) || (data == NULL) || (len == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_spi == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t canintf = 0U;
    esp_err_t err = spi_read_byte(REG_CANINTF, &canintf);
    if (err != ESP_OK) {
        return err;
    }

    const bool rx0 = (canintf & CANINTF_RX0IF) != 0U;
    const bool rx1 = (canintf & CANINTF_RX1IF) != 0U;
    if (!rx0 && !rx1) {
        return ESP_ERR_NOT_FOUND;
    }

    const uint8_t sid_addr = rx0 ? REG_RXB0SIDH : REG_RXB1SIDH;
    uint8_t buf[13] = { 0U };
    err = spi_read_buf(sid_addr, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }

    *id = (uint16_t)(((uint16_t)buf[0] << 3) | ((uint16_t)buf[1] >> 5));
    *len = buf[4] & 0x0FU;
    if (*len > 8U) {
        *len = 8U;
    }
    memcpy(data, &buf[5], *len);

    const uint8_t clear_mask = rx0 ? CANINTF_RX0IF : CANINTF_RX1IF;
    return spi_write_byte(REG_CANINTF, (uint8_t)(canintf & (uint8_t)~clear_mask));
}
