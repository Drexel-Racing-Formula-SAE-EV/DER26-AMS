/**
 * @file canbus.c
 * @author Cole Bardin (cab572@drexel.edu)
 * @author Mahad Faisal (major firmware updates, 2026)
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

#define CANBUS_FILTER_BANK_CHARGER 0u
#define CANBUS_FILTER_BANK_HIL_0   1u
#define CANBUS_FILTER_BANK_HIL_1   2u
#define CANBUS_FILTER_BANK_MISSION 3u
#define CANBUS_SLAVE_FILTER_START 14u

static uint16_t canbus_filter_ext_high(uint32_t id)
{
    return (uint16_t)((id >> 13u) & 0xFFFFu);
}

static uint16_t canbus_filter_ext_low(uint32_t id)
{
    return (uint16_t)(((id << 3u) & 0xFFF8u) | CAN_ID_EXT | CAN_RTR_DATA);
}

static uint16_t canbus_filter_std16(uint32_t id)
{
    return (uint16_t)(((id & 0x7FFu) << 5u) | CAN_ID_STD | CAN_RTR_DATA);
}

HAL_StatusTypeDef canbus_configure_rx_filters(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter;

    if(hcan == NULL)
    {
        return HAL_ERROR;
    }

    memset(&filter, 0, sizeof(filter));
    filter.FilterBank = CANBUS_FILTER_BANK_CHARGER;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = canbus_filter_ext_high(CHARGER_RX_ID);
    filter.FilterIdLow = canbus_filter_ext_low(CHARGER_RX_ID);
    filter.FilterMaskIdHigh = 0xFFFFu;
    /* Match all remaining identifier bits plus IDE and RTR. Bit zero in the
     * bxCAN filter word is reserved and intentionally ignored. */
    filter.FilterMaskIdLow = 0xFFFEu;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = CANBUS_SLAVE_FILTER_START;
    if(HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

#if AMS_ENABLE_HIL_CAN
    /* HIL is an explicit non-production profile. Four standard identifiers fit
     * in the first 16-bit list bank; the fifth is repeated in the unused slots
     * of a second bank so no unrelated standard frames are admitted. */
    memset(&filter, 0, sizeof(filter));
    filter.FilterBank = CANBUS_FILTER_BANK_HIL_0;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = canbus_filter_std16(AMS_HIL_CAN_ID_MEAS);
    filter.FilterIdLow = canbus_filter_std16(AMS_HIL_CAN_ID_TRUTH);
    filter.FilterMaskIdHigh = canbus_filter_std16(AMS_HIL_CAN_ID_SUMMARY);
    filter.FilterMaskIdLow = canbus_filter_std16(AMS_HIL_CAN_ID_CELL_SAMPLE);
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = CANBUS_SLAVE_FILTER_START;
    if(HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    memset(&filter, 0, sizeof(filter));
    filter.FilterBank = CANBUS_FILTER_BANK_HIL_1;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = canbus_filter_std16(AMS_HIL_CAN_ID_TEMP_SAMPLE);
    filter.FilterIdLow = canbus_filter_std16(AMS_HIL_CAN_ID_TEMP_SAMPLE);
    filter.FilterMaskIdHigh = canbus_filter_std16(AMS_HIL_CAN_ID_TEMP_SAMPLE);
    filter.FilterMaskIdLow = canbus_filter_std16(AMS_HIL_CAN_ID_TEMP_SAMPLE);
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = CANBUS_SLAVE_FILTER_START;
    if(HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }
#endif

#if AMS_ENABLE_MISSION_CAN
    memset(&filter, 0, sizeof(filter));
    filter.FilterBank = CANBUS_FILTER_BANK_MISSION;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = canbus_filter_std16(AMS_MISSION_CAN_REQUEST_ID);
    filter.FilterIdLow = canbus_filter_std16(AMS_MISSION_CAN_REQUEST_ID);
    filter.FilterMaskIdHigh = canbus_filter_std16(AMS_MISSION_CAN_REQUEST_ID);
    filter.FilterMaskIdLow = canbus_filter_std16(AMS_MISSION_CAN_REQUEST_ID);
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = CANBUS_SLAVE_FILTER_START;
    if(HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }
#endif

    return HAL_OK;
}

static void canbus_memory_barrier(void)
{
#if AMS_HOST_TEST
    __asm__ volatile ("" ::: "memory");
#else
    __DMB();
#endif
}

static void canbus_increment_u32_sat(volatile uint32_t *value)
{
    if((value != NULL) && (*value != UINT32_MAX))
    {
        (*value)++;
    }
}

static void canbus_add_u32_sat(volatile uint32_t *value, uint32_t amount)
{
    if(value == NULL)
    {
        return;
    }
    if(amount > (UINT32_MAX - *value))
    {
        *value = UINT32_MAX;
    }
    else
    {
        *value += amount;
    }
}

static bool canbus_rx_header_allowed(const CAN_RxHeaderTypeDef *header)
{
    if((header == NULL) || (header->RTR != CAN_RTR_DATA))
    {
        return false;
    }

    if(header->IDE == CAN_ID_EXT)
    {
        return (header->ExtId == CHARGER_RX_ID) && (header->DLC >= 5u);
    }

#if AMS_ENABLE_HIL_CAN
    if(header->IDE == CAN_ID_STD)
    {
        switch(header->StdId)
        {
        case AMS_HIL_CAN_ID_MEAS:
            return header->DLC >= 7u;
        case AMS_HIL_CAN_ID_TRUTH:
        case AMS_HIL_CAN_ID_SUMMARY:
        case AMS_HIL_CAN_ID_CELL_SAMPLE:
        case AMS_HIL_CAN_ID_TEMP_SAMPLE:
            return header->DLC >= 8u;
        default:
            break;
        }
    }
#endif
#if AMS_ENABLE_MISSION_CAN
    if((header->IDE == CAN_ID_STD) &&
       (header->StdId == AMS_MISSION_CAN_REQUEST_ID))
    {
        return header->DLC == 8u;
    }
#endif
    return false;
}

static uint16_t canbus_queue_next(uint16_t index)
{
    index++;
    if(index >= (uint16_t)CANBUS_RX_QUEUE_DEPTH)
    {
        index = 0u;
    }
    return index;
}

static uint16_t canbus_queue_count_from_indices(uint16_t head, uint16_t tail)
{
    if(head >= tail)
    {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)((uint16_t)CANBUS_RX_QUEUE_DEPTH - tail + head);
}

uint16_t canbus_rx_queue_count(const canbus_device_t *dev)
{
    uint16_t head;
    uint16_t tail;

    if(dev == NULL)
    {
        return 0u;
    }

    head = dev->rx_queue_head;
    tail = dev->rx_queue_tail;
    return canbus_queue_count_from_indices(head, tail);
}

static bool canbus_enqueue_from_isr(canbus_device_t *dev,
                                    const CAN_RxHeaderTypeDef *rx_header,
                                    const uint8_t rx_data[DATALEN],
                                    uint32_t tick)
{
    uint16_t head;
    uint16_t next;
    uint16_t tail;
    uint16_t queued;
    canbus_rx_frame_t *frame;

    if((dev == NULL) || (rx_header == NULL) || (rx_data == NULL))
    {
        return false;
    }

    head = dev->rx_queue_head;
    next = canbus_queue_next(head);
    tail = dev->rx_queue_tail;
    if(next == tail)
    {
        canbus_increment_u32_sat(&dev->rx_queue_drop_count);
        return false;
    }

    frame = &dev->rx_queue[head];
    frame->id = (rx_header->IDE == CAN_ID_EXT) ? rx_header->ExtId : rx_header->StdId;
    frame->tick = tick;
    frame->ide = (uint8_t)rx_header->IDE;
    frame->rtr = (uint8_t)rx_header->RTR;
    frame->dlc = (uint8_t)((rx_header->DLC <= DATALEN) ? rx_header->DLC : DATALEN);
    memset(frame->data, 0, sizeof(frame->data));
    memcpy(frame->data, rx_data, frame->dlc);

    /* Publish the entry only after every byte of the frame is visible. */
    canbus_memory_barrier();
    dev->rx_queue_head = next;

    queued = canbus_queue_count_from_indices(next, tail);
    if(queued > dev->rx_queue_high_water)
    {
        dev->rx_queue_high_water = queued;
    }
    return true;
}

static bool canbus_dequeue(canbus_device_t *dev, canbus_rx_frame_t *out)
{
    uint16_t tail;
    uint16_t head;

    if((dev == NULL) || (out == NULL))
    {
        return false;
    }

    tail = dev->rx_queue_tail;
    head = dev->rx_queue_head;
    if(tail == head)
    {
        return false;
    }

    /* Acquire the producer's published entry before copying it. */
    canbus_memory_barrier();
    *out = dev->rx_queue[tail];
    canbus_memory_barrier();
    dev->rx_queue_tail = canbus_queue_next(tail);
    return true;
}

#if AMS_ENABLE_HIL_CAN
static uint16_t be_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t be_i16(const uint8_t *data)
{
    return (int16_t)be_u16(data);
}

static void canbus_parse_hil_frame(app_data_t *data, const canbus_rx_frame_t *frame)
{
    if((data == NULL) || (frame == NULL) ||
       (frame->ide != CAN_ID_STD) || (frame->rtr != CAN_RTR_DATA))
    {
        return;
    }

    const uint8_t *rx_data = frame->data;
    uint32_t now = frame->tick;

    switch(frame->id)
    {
        case AMS_HIL_CAN_ID_MEAS:
            if(frame->dlc >= 7U)
            {
                const ams_hil_meas_t next = {
                    .fresh = 1u,
                    .counter = rx_data[6],
                    .last_rx_tick = now,
                    .v_pack_V = (float)be_u16(&rx_data[0]) * 0.01f,
                    .i_pack_A = (float)be_i16(&rx_data[2]) * 0.01f,
                    .t_surf_C = (float)be_i16(&rx_data[4]) * 0.01f
                };
                taskENTER_CRITICAL();
                data->hil.meas = next;
                taskEXIT_CRITICAL();
            }
            break;

        case AMS_HIL_CAN_ID_TRUTH:
            if(frame->dlc >= 8U)
            {
                const ams_hil_truth_t next = {
                    .fresh = 1u,
                    .counter = rx_data[4],
                    .last_rx_tick = now,
                    .plant_step = ((uint32_t)rx_data[5] << 16) |
                                  ((uint32_t)rx_data[6] << 8) |
                                  (uint32_t)rx_data[7],
                    .soc_true = (float)be_u16(&rx_data[0]) * 0.0001f,
                    .t_core_C = (float)be_i16(&rx_data[2]) * 0.01f
                };
                taskENTER_CRITICAL();
                data->hil.truth = next;
                taskEXIT_CRITICAL();
            }
            break;

        case AMS_HIL_CAN_ID_SUMMARY:
            if(frame->dlc >= 8U)
            {
                const ams_hil_summary_t next = {
                    .fresh = 1u,
                    .last_rx_tick = now,
                    .v_min_V = (float)be_u16(&rx_data[0]) * 0.001f,
                    .v_max_V = (float)be_u16(&rx_data[2]) * 0.001f,
                    .t_max_C = (float)be_i16(&rx_data[4]) * 0.01f,
                    .t_avg_C = (float)be_i16(&rx_data[6]) * 0.01f
                };
                taskENTER_CRITICAL();
                data->hil.summary = next;
                taskEXIT_CRITICAL();
            }
            break;

        case AMS_HIL_CAN_ID_CELL_SAMPLE:
            if(frame->dlc >= 8U)
            {
                uint16_t cell_mv[3] = {
                    be_u16(&rx_data[2]),
                    be_u16(&rx_data[4]),
                    be_u16(&rx_data[6])
                };

                adbms_spi_lock();
                (void)accumulator_hil_ingest_cell_triplet(&data->acc,
                                                          rx_data[0],
                                                          rx_data[1],
                                                          cell_mv,
                                                          now);
                adbms_spi_unlock();
            }
            break;

        case AMS_HIL_CAN_ID_TEMP_SAMPLE:
            if(frame->dlc >= 8U)
            {
                int16_t temp_deci_c[3] = {
                    be_i16(&rx_data[2]),
                    be_i16(&rx_data[4]),
                    be_i16(&rx_data[6])
                };

                adbms_spi_lock();
                (void)accumulator_hil_ingest_temp_triplet(&data->acc,
                                                          rx_data[0],
                                                          rx_data[1],
                                                          temp_deci_c,
                                                          now);
                adbms_spi_unlock();
            }
            break;

        default:
            break;
    }
}
#endif /* AMS_ENABLE_HIL_CAN */

HAL_StatusTypeDef canbus_device_init(canbus_device_t *dev, CAN_HandleTypeDef *hcan)
{
    if(dev == NULL)
    {
        return HAL_ERROR;
    }

    dev->hcan = hcan;
    dev->tx_mailbox = 0u;
    memset(&dev->rx_packet, 0, sizeof(dev->rx_packet));
    memset(dev->rx_queue, 0, sizeof(dev->rx_queue));
    dev->rx_queue_head = 0u;
    dev->rx_queue_tail = 0u;
    dev->rx_queue_high_water = 0u;
    dev->rx_isr_count = 0u;
    dev->rx_processed_count = 0u;
    dev->rx_filtered_count = 0u;
    dev->rx_queue_drop_count = 0u;
    dev->rx_hal_error_count = 0u;
    dev->rx_queue_drop_reported = 0u;
    dev->rx_hal_error_reported = 0u;
	dev->init_status = HAL_ERROR;
	dev->start_status = HAL_ERROR;
	dev->notification_status = HAL_ERROR;
	dev->started = false;
	dev->notification_active = false;

    dev->tx_header.IDE = CAN_ID_STD;
    dev->tx_header.StdId = 0x00;
    dev->tx_header.ExtId = 0x00;
    dev->tx_header.RTR = CAN_RTR_DATA;
    dev->tx_header.DLC = DATALEN;
    dev->tx_header.TransmitGlobalTime = DISABLE;

    if(hcan == NULL)
    {
		return dev->init_status;
    }

	dev->start_status = HAL_CAN_Start(hcan);
	dev->started = (dev->start_status == HAL_OK);
	if(!dev->started)
	{
		dev->init_status = dev->start_status;
		return dev->init_status;
	}

	dev->notification_status = HAL_CAN_ActivateNotification(
		hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	dev->notification_active = (dev->notification_status == HAL_OK);
	dev->init_status = dev->notification_status;
	return dev->init_status;
}


const char *canbus_error_str(uint32_t err)
{
    if(err == HAL_CAN_ERROR_NONE) return "none";
    if((err & HAL_CAN_ERROR_BOF) != 0u) return "bus_off";
    if((err & HAL_CAN_ERROR_EPV) != 0u) return "error_passive";
    if((err & HAL_CAN_ERROR_EWG) != 0u) return "error_warning";
    if((err & HAL_CAN_ERROR_ACK) != 0u) return "ack";
    if((err & HAL_CAN_ERROR_CRC) != 0u) return "crc";
    if((err & HAL_CAN_ERROR_RX_FOV0) != 0u) return "rx_fifo0_overrun";
    if((err & HAL_CAN_ERROR_TIMEOUT) != 0u) return "timeout";
    if((err & HAL_CAN_ERROR_NOT_STARTED) != 0u) return "not_started";
    if((err & HAL_CAN_ERROR_NOT_READY) != 0u) return "not_ready";
    return "other";
}

static void canbus_record_rx_loss(canbus_device_t *dev, app_data_t *data)
{
    uint32_t drop_count;
    uint32_t hal_error_count;
    uint32_t loss_delta;

    drop_count = dev->rx_queue_drop_count;
    hal_error_count = dev->rx_hal_error_count;
    loss_delta = (drop_count - dev->rx_queue_drop_reported) +
                 (hal_error_count - dev->rx_hal_error_reported);

    dev->rx_queue_drop_reported = drop_count;
    dev->rx_hal_error_reported = hal_error_count;

    if(loss_delta == 0u)
    {
        return;
    }

    if((UINT32_MAX - data->can_error_count) < loss_delta)
    {
        data->can_error_count = UINT32_MAX;
    }
    else
    {
        data->can_error_count += loss_delta;
    }
    data->can_error_code |= HAL_CAN_ERROR_RX_FOV0;
    data->can_last_error_tick = osKernelGetTickCount();
    data->canbus_fault = true;
}

static void canbus_process_rx_frame(canbus_device_t *dev,
                                    app_data_t *data,
                                    const canbus_rx_frame_t *frame)
{
    charger_t *ccs;
    uint16_t v_raw;
    uint16_t i_raw;
    uint8_t flags;

    dev->rx_packet.id = frame->id;
    dev->rx_packet.ide = frame->ide;
    dev->rx_packet.rtr = frame->rtr;
    dev->rx_packet.dlc = frame->dlc;
    dev->rx_packet.tick = frame->tick;
    memset(dev->rx_packet.data, 0, sizeof(dev->rx_packet.data));
    memcpy(dev->rx_packet.data, frame->data, frame->dlc);

#if AMS_ENABLE_HIL_CAN
    canbus_parse_hil_frame(data, frame);
#endif

#if AMS_ENABLE_MISSION_CAN
    if((frame->rtr == CAN_RTR_DATA) && (frame->ide == CAN_ID_STD) &&
       (frame->id == AMS_MISSION_CAN_REQUEST_ID) && (frame->dlc == 8u))
    {
        taskENTER_CRITICAL();
        (void)ams_mission_request_ingest(&data->mission_request,
                                         frame->data,
                                         frame->tick);
        taskEXIT_CRITICAL();
        return;
    }
#endif

    if((frame->rtr != CAN_RTR_DATA) ||
       (frame->ide != CAN_ID_EXT) ||
       (frame->id != CHARGER_RX_ID) ||
       (frame->dlc < 5u))
    {
        return;
    }

    ccs = &data->board.charger;
    v_raw = ((uint16_t)frame->data[0] << 8) | frame->data[1];
    i_raw = ((uint16_t)frame->data[2] << 8) | frame->data[3];
    flags = frame->data[4];

    ccs->read_voltage = (float)v_raw * 0.1f;
    ccs->read_current = (float)i_raw * 0.1f;
    ccs->flags = flags;
    ccs->hardware_fail = ((flags >> 0u) & 0x01u) != 0u;
    ccs->overtemp_fail = ((flags >> 1u) & 0x01u) != 0u;
    ccs->input_volt_fail = ((flags >> 2u) & 0x01u) != 0u;
    ccs->voltage_sense_fail = ((flags >> 3u) & 0x01u) != 0u;
    ccs->communication_fail = false;
    ccs->last_rx_tick = frame->tick;
    ccs->rx_count++;
}

uint32_t canbus_process_rx_queue(canbus_device_t *dev, app_data_t *data, uint32_t max_frames)
{
    canbus_rx_frame_t frame;
    uint32_t processed = 0u;

    if((dev == NULL) || (data == NULL))
    {
        return 0u;
    }

    canbus_record_rx_loss(dev, data);

    while((processed < max_frames) && canbus_dequeue(dev, &frame))
    {
        canbus_process_rx_frame(dev, data, &frame);
        processed++;
    }

    canbus_add_u32_sat(&dev->rx_processed_count, processed);
    return processed;
}

HAL_StatusTypeDef canbus_recover(canbus_device_t *dev)
{
	HAL_StatusTypeDef reset_status;

    if((dev == NULL) || (dev->hcan == NULL))
    {
        return HAL_ERROR;
    }

	/* Stopping an already-stopped controller can itself return an error.  The
	 * recovery result is therefore based on reset, restart, and notification
	 * activation, while still making the best-effort stop first. */
	(void)HAL_CAN_Stop(dev->hcan);
	reset_status = HAL_CAN_ResetError(dev->hcan);
	dev->start_status = HAL_CAN_Start(dev->hcan);
	dev->started = (dev->start_status == HAL_OK);

	if(dev->started)
	{
		dev->notification_status = HAL_CAN_ActivateNotification(
			dev->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
	else
	{
		dev->notification_status = HAL_ERROR;
	}
	dev->notification_active = (dev->notification_status == HAL_OK);

	if(reset_status != HAL_OK)
	{
		dev->init_status = reset_status;
	}
	else if(dev->start_status != HAL_OK)
	{
		dev->init_status = dev->start_status;
	}
	else
	{
		dev->init_status = dev->notification_status;
	}

	return dev->init_status;
}

void canbus_poll_errors(canbus_device_t *dev, app_data_t *data)
{
    uint32_t err;
    uint32_t now;

    if((dev == NULL) || (dev->hcan == NULL) || (data == NULL))
    {
        return;
    }

    now = osKernelGetTickCount();
    err = HAL_CAN_GetError(dev->hcan);

    if(err != HAL_CAN_ERROR_NONE)
    {
        bool new_error = (err != data->can_error_code) ||
                         (((err & HAL_CAN_ERROR_BOF) != 0u) && !data->can_busoff_fault);

        if(new_error)
        {
            data->can_error_count++;
            data->can_last_error_tick = now;
        }
        data->can_error_code = err;
        data->canbus_fault = true;

        if((err & HAL_CAN_ERROR_BOF) != 0u)
        {
            if(!data->can_busoff_fault)
            {
                data->can_busoff_count++;
                ams_fault_log_event(AMS_FAULT_LOG_CAN_BUS_OFF, 0u, err, data->can_busoff_count);
            }

            data->can_busoff_fault = true;
            data->can_recover_pending = true;
            data->canbus_fault = true;

            if(data->state == STATE_CHARGE)
            {
                data->charger_fault = true;
                data->board.charger.communication_fail = true;
                set_bms(false);
            }

#if AMS_HIL_REPLACE_ADBMS
            data->adbms_diag_fault = true;
            set_bms(false);
#endif
        }
        else
        {
            (void)HAL_CAN_ResetError(dev->hcan);
        }
    }
    else if(!data->can_busoff_fault &&
            !data->can_recover_pending &&
            data->canbus_fault &&
            (data->can_error_code != HAL_CAN_ERROR_NONE) &&
            ((now - data->can_last_error_tick) > AMS_CAN_ERROR_SOFT_HOLD_MS))
    {
        data->canbus_fault = false;
    }

    if(data->can_recover_pending &&
       ((now - data->can_last_error_tick) >= AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS))
    {
        HAL_StatusTypeDef rec = canbus_recover(dev);
        if(rec == HAL_OK)
        {
            data->can_recover_count++;
            data->can_busoff_fault = false;
            data->can_recover_pending = false;
            data->can_error_code = HAL_CAN_ERROR_NONE;
            data->canbus_fault = false;
            ams_fault_log_event(AMS_FAULT_LOG_CAN_RECOVERED, 0u, data->can_recover_count, 0u);
        }
        else
        {
            data->can_error_count++;
            data->can_last_error_tick = now;
            data->canbus_fault = true;
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8] = {0};
    canbus_device_t *dev = &app.board.canbus;

    if((hcan == NULL) || (dev->hcan != hcan))
    {
        return;
    }

    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        canbus_increment_u32_sat(&dev->rx_hal_error_count);
        return;
    }

    canbus_increment_u32_sat(&dev->rx_isr_count);
    if(!canbus_rx_header_allowed(&rx_header))
    {
        canbus_increment_u32_sat(&dev->rx_filtered_count);
        return;
    }
    (void)canbus_enqueue_from_isr(dev, &rx_header, rx_data, osKernelGetTickCount());
}
