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


static uint16_t be_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t be_i16(const uint8_t *data)
{
    return (int16_t)be_u16(data);
}

static void canbus_parse_hil_frame(const CAN_RxHeaderTypeDef *rx_header, const uint8_t rx_data[8])
{
    if ((rx_header == NULL) || (rx_data == NULL) || (rx_header->IDE != CAN_ID_STD))
    {
        return;
    }

    uint32_t now = osKernelGetTickCount();

    switch (rx_header->StdId)
    {
        case AMS_HIL_CAN_ID_MEAS:
            if (rx_header->DLC >= 7U)
            {
                app.hil.meas.v_pack_V = (float)be_u16(&rx_data[0]) * 0.01f;
                app.hil.meas.i_pack_A = (float)be_i16(&rx_data[2]) * 0.01f;
                app.hil.meas.t_surf_C = (float)be_i16(&rx_data[4]) * 0.01f;
                app.hil.meas.counter = rx_data[6];
                app.hil.meas.last_rx_tick = now;
                app.hil.meas.fresh = 1U;
            }
            break;

        case AMS_HIL_CAN_ID_TRUTH:
            if (rx_header->DLC >= 8U)
            {
                app.hil.truth.soc_true = (float)be_u16(&rx_data[0]) * 0.0001f;
                app.hil.truth.t_core_C = (float)be_i16(&rx_data[2]) * 0.01f;
                app.hil.truth.counter = rx_data[4];
                app.hil.truth.plant_step = ((uint32_t)rx_data[5] << 16) |
                                           ((uint32_t)rx_data[6] << 8)  |
                                           ((uint32_t)rx_data[7]);
                app.hil.truth.last_rx_tick = now;
                app.hil.truth.fresh = 1U;
            }
            break;

        case AMS_HIL_CAN_ID_SUMMARY:
            if (rx_header->DLC >= 8U)
            {
                app.hil.summary.v_min_V = (float)be_u16(&rx_data[0]) * 0.001f;
                app.hil.summary.v_max_V = (float)be_u16(&rx_data[2]) * 0.001f;
                app.hil.summary.t_max_C = (float)be_i16(&rx_data[4]) * 0.01f;
                app.hil.summary.t_avg_C = (float)be_i16(&rx_data[6]) * 0.01f;
                app.hil.summary.last_rx_tick = now;
                app.hil.summary.fresh = 1U;
            }
            break;

        default:
            break;
    }
}

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

    canbus_parse_hil_frame(&rx_header, rx_data);

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
