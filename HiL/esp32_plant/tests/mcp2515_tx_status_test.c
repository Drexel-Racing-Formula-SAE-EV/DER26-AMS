#include <stdio.h>
#include <stdlib.h>

#include "mcp2515_tx_status.h"

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

int main(void)
{
    CHECK(mcp2515_classify_tx_status(
              MCP2515_TXBCTRL_TXREQ, 0U) ==
          MCP2515_TX_STATUS_PENDING);
    CHECK(mcp2515_classify_tx_status(0U, 0U) ==
          MCP2515_TX_STATUS_SUCCESS);
    CHECK(mcp2515_classify_tx_status(
              MCP2515_TXBCTRL_MLOA | MCP2515_TXBCTRL_TXERR, 0U) ==
          MCP2515_TX_STATUS_CONTROLLER_ERROR);
    CHECK(mcp2515_classify_tx_status(
              MCP2515_TXBCTRL_TXREQ |
                  MCP2515_TXBCTRL_MLOA |
                  MCP2515_TXBCTRL_TXERR,
              0U) ==
          MCP2515_TX_STATUS_PENDING);
    CHECK(mcp2515_classify_tx_status(
              MCP2515_TXBCTRL_ABTF, 0U) ==
          MCP2515_TX_STATUS_ABORTED);
    CHECK(mcp2515_classify_tx_status(
              MCP2515_TXBCTRL_TXREQ, MCP2515_EFLG_TXBO) ==
          MCP2515_TX_STATUS_BUS_OFF);

    puts("PASS mcp2515_tx_status_test");
    return EXIT_SUCCESS;
}
