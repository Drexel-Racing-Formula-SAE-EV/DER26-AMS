#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/spi_master.h"
#include "freertos/task.h"
#include "mcp2515_driver.h"
#include "mcp2515_tx_status.h"

#define MCP_RESET 0xC0U
#define MCP_READ 0x03U
#define MCP_WRITE 0x02U
#define MCP_BIT_MODIFY 0x05U
#define MCP_LOAD_TX0 0x40U
#define MCP_RTS_TX0 0x81U

#define REG_CANSTAT 0x0EU
#define REG_CANCTRL 0x0FU
#define REG_EFLG 0x2DU
#define REG_TXB0CTRL 0x30U

#define MODE_CONFIG 0x80U
#define MODE_NORMAL 0x00U
#define MODE_MASK 0xE0U

#define MAX_POLLS 24U

#define CHECK(condition)                                                     \
    do                                                                       \
    {                                                                        \
        if (!(condition))                                                    \
        {                                                                    \
            fprintf(stderr, "CHECK failed at %s:%d: %s\n",                 \
                    __FILE__, __LINE__, #condition);                         \
            return EXIT_FAILURE;                                             \
        }                                                                    \
    } while (0)

static uint8_t s_registers[256];
static uint8_t s_ctrl_sequence[MAX_POLLS];
static uint8_t s_eflg_sequence[MAX_POLLS];
static size_t s_sequence_length;
static size_t s_poll_index;
static uint32_t s_txreq_clear_count;
static bool s_tx_requested;

static void configure_sequence(
    const uint8_t *ctrl,
    const uint8_t *eflg,
    size_t count)
{
    memset(s_ctrl_sequence, 0, sizeof(s_ctrl_sequence));
    memset(s_eflg_sequence, 0, sizeof(s_eflg_sequence));
    memcpy(s_ctrl_sequence, ctrl, count);
    memcpy(s_eflg_sequence, eflg, count);
    s_sequence_length = count;
    s_poll_index = 0U;
    s_txreq_clear_count = 0U;
    s_tx_requested = false;
    mcp2515_reset_diagnostics();
}

static size_t sequence_index(void)
{
    if (s_sequence_length == 0U)
    {
        return 0U;
    }
    if (s_poll_index >= s_sequence_length)
    {
        return s_sequence_length - 1U;
    }
    return s_poll_index;
}

esp_err_t spi_bus_initialize(
    int host,
    const spi_bus_config_t *configuration,
    int dma_channel)
{
    (void)host;
    (void)configuration;
    (void)dma_channel;
    return ESP_OK;
}

esp_err_t spi_bus_add_device(
    int host,
    const spi_device_interface_config_t *configuration,
    spi_device_handle_t *device)
{
    (void)host;
    (void)configuration;
    *device = (spi_device_handle_t)(uintptr_t)1U;
    return ESP_OK;
}

esp_err_t spi_device_transmit(
    spi_device_handle_t device,
    spi_transaction_t *transaction)
{
    (void)device;
    const uint8_t *tx = (const uint8_t *)transaction->tx_buffer;
    uint8_t *rx = (uint8_t *)transaction->rx_buffer;
    size_t bytes = transaction->length / 8U;
    if ((tx == NULL) || (bytes == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (tx[0] == MCP_RESET)
    {
        memset(s_registers, 0, sizeof(s_registers));
        s_registers[REG_CANSTAT] = MODE_CONFIG;
        return ESP_OK;
    }
    if ((tx[0] == MCP_WRITE) && (bytes >= 3U))
    {
        s_registers[tx[1]] = tx[2];
        return ESP_OK;
    }
    if ((tx[0] == MCP_BIT_MODIFY) && (bytes >= 4U))
    {
        uint8_t address = tx[1];
        s_registers[address] =
            (uint8_t)((s_registers[address] & (uint8_t)~tx[2]) |
                      (tx[3] & tx[2]));
        if ((address == REG_CANCTRL) && ((tx[2] & MODE_MASK) != 0U))
        {
            s_registers[REG_CANSTAT] =
                (uint8_t)((s_registers[REG_CANSTAT] & (uint8_t)~MODE_MASK) |
                          (tx[3] & MODE_MASK));
        }
        if ((address == REG_TXB0CTRL) &&
            ((tx[2] & MCP2515_TXBCTRL_TXREQ) != 0U) &&
            ((tx[3] & MCP2515_TXBCTRL_TXREQ) == 0U))
        {
            s_txreq_clear_count++;
        }
        return ESP_OK;
    }
    if ((tx[0] == MCP_READ) && (bytes >= 3U) && (rx != NULL))
    {
        memset(rx, 0, bytes);
        if (tx[1] == REG_TXB0CTRL)
        {
            rx[2] = s_ctrl_sequence[sequence_index()];
        }
        else if (tx[1] == REG_EFLG)
        {
            rx[2] = s_eflg_sequence[sequence_index()];
            s_poll_index++;
        }
        else
        {
            rx[2] = s_registers[tx[1]];
        }
        return ESP_OK;
    }
    if (tx[0] == MCP_LOAD_TX0)
    {
        return ESP_OK;
    }
    if (tx[0] == MCP_RTS_TX0)
    {
        s_tx_requested = true;
        return ESP_OK;
    }
    return ESP_FAIL;
}

void vTaskDelay(TickType_t delay)
{
    (void)delay;
}

static int expect_send(
    const uint8_t *ctrl,
    const uint8_t *eflg,
    size_t count,
    esp_err_t expected)
{
    static const uint8_t payload[2] = {0x12U, 0x34U};
    configure_sequence(ctrl, eflg, count);
    CHECK(mcp2515_send_frame(0x210U, payload, sizeof(payload)) == expected);
    CHECK(s_tx_requested);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(mcp2515_init() == ESP_OK);

    mcp2515_diagnostics_t diagnostics;
    const uint8_t clear[] = {0U};
    const uint8_t no_error[] = {0U};
    CHECK(expect_send(clear, no_error, 1U, ESP_OK) == EXIT_SUCCESS);
    mcp2515_get_diagnostics(&diagnostics);
    CHECK(diagnostics.successful_frames == 1U);
    CHECK(s_txreq_clear_count >= 1U);

    const uint8_t retry_ctrl[] = {
        MCP2515_TXBCTRL_TXREQ | MCP2515_TXBCTRL_MLOA,
        0U,
    };
    const uint8_t retry_eflg[] = {0U, 0U};
    CHECK(expect_send(retry_ctrl, retry_eflg, 2U, ESP_OK) == EXIT_SUCCESS);
    mcp2515_get_diagnostics(&diagnostics);
    CHECK(diagnostics.successful_frames == 1U);
    CHECK(diagnostics.arbitration_lost_events == 1U);
    CHECK(diagnostics.controller_retry_events == 1U);

    const uint8_t tx_error[] = {MCP2515_TXBCTRL_TXERR};
    CHECK(expect_send(tx_error, no_error, 1U, ESP_FAIL) == EXIT_SUCCESS);
    mcp2515_get_diagnostics(&diagnostics);
    CHECK(diagnostics.successful_frames == 0U);
    CHECK(diagnostics.transmit_error_events == 1U);

    const uint8_t aborted[] = {MCP2515_TXBCTRL_ABTF};
    CHECK(expect_send(aborted, no_error, 1U, ESP_FAIL) == EXIT_SUCCESS);
    mcp2515_get_diagnostics(&diagnostics);
    CHECK(diagnostics.aborted_frames == 1U);

    const uint8_t pending[] = {MCP2515_TXBCTRL_TXREQ};
    const uint8_t bus_off[] = {MCP2515_EFLG_TXBO};
    CHECK(expect_send(pending, bus_off, 1U, ESP_FAIL) == EXIT_SUCCESS);
    mcp2515_get_diagnostics(&diagnostics);
    CHECK(diagnostics.bus_off_failures == 1U);
    CHECK(s_txreq_clear_count >= 1U);

    CHECK(expect_send(pending, no_error, 1U, ESP_ERR_TIMEOUT) == EXIT_SUCCESS);
    mcp2515_get_diagnostics(&diagnostics);
    CHECK(diagnostics.timeout_failures == 1U);
    CHECK(s_txreq_clear_count >= 1U);

    puts("PASS mcp2515_driver_host_test");
    return EXIT_SUCCESS;
}
