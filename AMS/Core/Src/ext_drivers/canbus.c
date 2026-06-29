/**
 * @file canbus.c
 * @author Cole Bardin (cab572@drexel.edu)
 * @brief
 * @version 0.1
 * @date 2023-04-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <string.h>

#include "ext_drivers/canbus.h"
#include "ext_drivers/charger.h"
#include "app.h"

extern app_data_t app;

void canbus_device_init(canbus_device_t *dev, CAN_HandleTypeDef *hcan)
{
    if(dev == NULL)
    {
        return;
    }

    dev->hcan = hcan;
    dev->tx_mailbox = 0u;
    memset(&dev->rx_packet, 0, sizeof(dev->rx_packet));

    dev->tx_header.IDE = CAN_ID_STD;
    dev->tx_header.StdId = 0x00;
    dev->tx_header.ExtId = 0x00;
    dev->tx_header.RTR = CAN_RTR_DATA;
    dev->tx_header.DLC = DATALEN;
    dev->tx_header.TransmitGlobalTime = DISABLE;

    if(hcan != NULL)
    {
        (void)HAL_CAN_Start(hcan);
        (void)HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8] = {0};

    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) return;

    app.board.canbus.rx_packet.id = (rx_header.IDE == CAN_ID_EXT) ? rx_header.ExtId : rx_header.StdId;
    memcpy(app.board.canbus.rx_packet.data, rx_data, DATALEN);

    if(rx_header.IDE != CAN_ID_EXT) return;
    if(rx_header.ExtId != CHARGER_RX_ID) return;
    if(rx_header.DLC < 5u) return;

    charger_t *ccs = &app.board.charger;

    uint16_t v_raw = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    uint16_t i_raw = ((uint16_t)rx_data[2] << 8) | rx_data[3];
    uint8_t  flags = rx_data[4];

    ccs->read_voltage       = (float)v_raw * 0.1f;
    ccs->read_current       = (float)i_raw * 0.1f;
    ccs->flags              = flags;
    ccs->hardware_fail      = ((flags >> 0u) & 0x01u) != 0u;
    ccs->overtemp_fail      = ((flags >> 1u) & 0x01u) != 0u;
    ccs->input_volt_fail    = ((flags >> 2u) & 0x01u) != 0u;
    ccs->voltage_sense_fail = ((flags >> 3u) & 0x01u) != 0u;
    ccs->communication_fail = false;
    ccs->last_rx_tick       = osKernelGetTickCount();
    ccs->rx_count++;
}
