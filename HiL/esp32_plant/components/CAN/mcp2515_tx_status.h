#ifndef COMPONENTS_CAN_MCP2515_TX_STATUS_H_
#define COMPONENTS_CAN_MCP2515_TX_STATUS_H_

#include <stdint.h>

#define MCP2515_TXBCTRL_ABTF  0x40U
#define MCP2515_TXBCTRL_MLOA  0x20U
#define MCP2515_TXBCTRL_TXERR 0x10U
#define MCP2515_TXBCTRL_TXREQ 0x08U

#define MCP2515_EFLG_RX1OVR 0x80U
#define MCP2515_EFLG_RX0OVR 0x40U
#define MCP2515_EFLG_TXBO   0x20U
#define MCP2515_EFLG_TXEP   0x10U
#define MCP2515_EFLG_TXWAR  0x04U
#define MCP2515_EFLG_EWARN  0x01U

typedef enum {
    MCP2515_TX_STATUS_PENDING = 0,
    MCP2515_TX_STATUS_SUCCESS,
    MCP2515_TX_STATUS_ABORTED,
    MCP2515_TX_STATUS_CONTROLLER_ERROR,
    MCP2515_TX_STATUS_BUS_OFF
} mcp2515_tx_status_t;

/*
 * In normal mode the controller retries arbitration loss and bus errors
 * while TXREQ remains set. A completed success therefore requires TXREQ,
 * ABTF, MLOA, and TXERR all clear, and the controller must not be bus-off.
 */
static inline mcp2515_tx_status_t mcp2515_classify_tx_status(
    uint8_t txbctrl,
    uint8_t eflg)
{
    if ((eflg & MCP2515_EFLG_TXBO) != 0U)
    {
        return MCP2515_TX_STATUS_BUS_OFF;
    }
    if ((txbctrl & MCP2515_TXBCTRL_TXREQ) != 0U)
    {
        return MCP2515_TX_STATUS_PENDING;
    }
    if ((txbctrl & MCP2515_TXBCTRL_ABTF) != 0U)
    {
        return MCP2515_TX_STATUS_ABORTED;
    }
    if ((txbctrl & (MCP2515_TXBCTRL_MLOA |
                    MCP2515_TXBCTRL_TXERR)) != 0U)
    {
        return MCP2515_TX_STATUS_CONTROLLER_ERROR;
    }
    return MCP2515_TX_STATUS_SUCCESS;
}

#endif /* COMPONENTS_CAN_MCP2515_TX_STATUS_H_ */
