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

#if AMS_CAN_BUSOFF_REPEAT_LIMIT != 3u
#error "CAN bus-off recent-event storage assumes repeat limit 3"
#endif
#include "task.h"

#if AMS_HOST_TEST
#define CANBUS_ISR_ENTER_CRITICAL() ((UBaseType_t)0u)
#define CANBUS_ISR_EXIT_CRITICAL(mask) ((void)(mask))
#define CANBUS_ISR_TICK() osKernelGetTickCount()
#else
#define CANBUS_ISR_ENTER_CRITICAL() taskENTER_CRITICAL_FROM_ISR()
#define CANBUS_ISR_EXIT_CRITICAL(mask) taskEXIT_CRITICAL_FROM_ISR(mask)
#define CANBUS_ISR_TICK() ((uint32_t)xTaskGetTickCountFromISR())
#endif

#if CANBUS_BUSOFF_WINDOW_SLOTS != AMS_CAN_BUSOFF_REPEAT_LIMIT
#error "CAN bus-off sliding-window storage must match repeat policy limit"
#endif

extern app_data_t app;

#define CANBUS_FILTER_BANK_CHARGER 0u
#define CANBUS_FILTER_BANK_HIL_0   1u
#define CANBUS_FILTER_BANK_HIL_1   2u
#define CANBUS_FILTER_BANK_MISSION 3u
#define CANBUS_FILTER_BANK_ECU_DIAG 4u

_Static_assert(CANBUS_FILTER_BANK_ECU_DIAG < CANBUS_SLAVE_FILTER_START,
               "CAN1 filter ownership must stay below CAN2 split bank");

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

    /* DER26-CAN-V4 ECU->AMS application-level observability. List-mode
     * includes the RTR bit, so only a standard data frame at 0x6F0 reaches
     * FIFO0. Software still rejects RTR explicitly as defense in depth. */
    memset(&filter, 0, sizeof(filter));
    filter.FilterBank = CANBUS_FILTER_BANK_ECU_DIAG;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = canbus_filter_std16(AMS_ECU_DIAG_FEEDBACK_CAN_ID);
    filter.FilterIdLow = canbus_filter_std16(AMS_ECU_DIAG_FEEDBACK_CAN_ID);
    filter.FilterMaskIdHigh = canbus_filter_std16(AMS_ECU_DIAG_FEEDBACK_CAN_ID);
    filter.FilterMaskIdLow = canbus_filter_std16(AMS_ECU_DIAG_FEEDBACK_CAN_ID);
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = CANBUS_SLAVE_FILTER_START;
    if(HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

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

static void canbus_note_hal_load_error_locked(canbus_device_t *dev,
                                               const ams_can_tx_frame_t *frame)
{
    if((dev == NULL) || (frame == NULL))
    {
        return;
    }

    canbus_increment_u32_sat(&dev->tx_hal_load_error_count);
    switch(frame->tx_class)
    {
    case AMS_CAN_TX_CLASS_CRITICAL:
        canbus_increment_u32_sat(&dev->tx_hal_load_error_critical_count);
        break;
    case AMS_CAN_TX_CLASS_PROTECTED_REQUIRED:
    case AMS_CAN_TX_CLASS_PROTECTED_ADVISORY:
        canbus_increment_u32_sat(&dev->tx_hal_load_error_protected_count);
        break;
    case AMS_CAN_TX_CLASS_DETAIL:
    default:
        canbus_increment_u32_sat(&dev->tx_hal_load_error_detail_count);
        break;
    }

    if(frame->source_tag == CANBUS_TX_TAG_CHARGER_NORMAL)
    {
        canbus_increment_u32_sat(&dev->tx_charger_normal_load_error_count);
    }
    else if(frame->source_tag == CANBUS_TX_TAG_CHARGER_SHUTDOWN)
    {
        canbus_increment_u32_sat(&dev->tx_charger_shutdown_load_error_count);
    }

    dev->tx_last_hal_load_error_source_tag = frame->source_tag;
    dev->tx_last_hal_load_error_class = frame->tx_class;
    /* A HAL load failure is a real controller/HAL anomaly, not ordinary
     * mailbox occupancy. Stop retrying within this task period; task-context
     * error polling decides when the pump may try again. */
    dev->tx_suspended = true;
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

    if((header->IDE == CAN_ID_STD) &&
       (header->StdId == AMS_ECU_DIAG_FEEDBACK_CAN_ID))
    {
        return header->DLC == AMS_ECU_DIAG_FEEDBACK_DLC;
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


static void canbus_tx_note_charger_complete_from_isr(
    const canbus_tx_mailbox_meta_t *meta);
static void canbus_tx_abort_stale(canbus_device_t *dev);

static uint8_t canbus_mailbox_index_from_mask(uint32_t mailbox)
{
    if(mailbox == CAN_TX_MAILBOX0) return 0u;
    if(mailbox == CAN_TX_MAILBOX1) return 1u;
    if(mailbox == CAN_TX_MAILBOX2) return 2u;
    return 0xFFu;
}

static ams_can_tx_class_t canbus_classify_frame(uint32_t ide, uint32_t id)
{
    if(ide == CAN_ID_EXT)
    {
        return AMS_CAN_TX_CLASS_CRITICAL;
    }
    if((id >= AMS_ECU_CAN_ID_STATUS) &&
       (id <= AMS_ECU_CAN_ID_SOP_ENVELOPE))
    {
        return AMS_CAN_TX_CLASS_PROTECTED_REQUIRED;
    }
    if((id >= AMS_ECU_CAN_ID_STRATEGY_STATUS) &&
       (id <= AMS_ECU_CAN_ID_CURRENT_DIAG))
    {
        return AMS_CAN_TX_CLASS_PROTECTED_ADVISORY;
    }
    return AMS_CAN_TX_CLASS_DETAIL;
}

uint32_t canbus_tx_next_generation(canbus_device_t *dev)
{
    if(dev == NULL)
    {
        return 0u;
    }
    dev->tx_generation_counter++;
    if(dev->tx_generation_counter == 0u)
    {
        dev->tx_generation_counter = 1u;
    }
    return dev->tx_generation_counter;
}

HAL_StatusTypeDef canbus_tx_build_begin(canbus_device_t *dev,
                                        canbus_tx_build_kind_t kind,
                                        uint32_t generation,
                                        uint32_t publish_tick,
                                        uint16_t source_tag)
{
    if((dev == NULL) || (kind == CANBUS_TX_BUILD_NONE) ||
       dev->tx_builder.active)
    {
        return HAL_ERROR;
    }

    memset(&dev->tx_builder, 0, sizeof(dev->tx_builder));
    dev->tx_builder.active = true;
    dev->tx_builder.kind = kind;
    dev->tx_builder.generation = generation;
    dev->tx_builder.publish_tick = publish_tick;
    dev->tx_builder.source_tag = source_tag;
    return HAL_OK;
}

HAL_StatusTypeDef canbus_tx_build_append(canbus_device_t *dev,
                                         uint32_t ide,
                                         uint32_t id,
                                         const uint8_t payload[8])
{
    ams_can_tx_frame_t *frame;
    ams_can_tx_class_t tx_class;

    if((dev == NULL) || (payload == NULL) || !dev->tx_builder.active)
    {
        return HAL_ERROR;
    }

    uint16_t build_limit = CANBUS_TX_BUILD_MAX_FRAMES;
    if(dev->tx_builder.kind == CANBUS_TX_BUILD_CRITICAL)
    {
        build_limit = AMS_CAN_TX_CRITICAL_MAX_FRAMES;
    }
    else if(dev->tx_builder.kind == CANBUS_TX_BUILD_PROTECTED)
    {
        build_limit = AMS_CAN_TX_PROTECTED_MAX_FRAMES;
    }

    if(dev->tx_builder.frame_count >= build_limit)
    {
        canbus_increment_u32_sat(&dev->tx_build_overflow_count);
        if((dev->tx_builder.kind == CANBUS_TX_BUILD_DETAIL) ||
           (dev->tx_builder.kind == CANBUS_TX_BUILD_TUNING))
        {
            canbus_increment_u32_sat(&dev->tx_build_detail_overflow_count);
        }
        return HAL_ERROR;
    }

    tx_class = canbus_classify_frame(ide, id);
    if(((dev->tx_builder.kind == CANBUS_TX_BUILD_CRITICAL) &&
        (tx_class != AMS_CAN_TX_CLASS_CRITICAL)) ||
       ((dev->tx_builder.kind == CANBUS_TX_BUILD_PROTECTED) &&
        (tx_class != AMS_CAN_TX_CLASS_PROTECTED_REQUIRED) &&
        (tx_class != AMS_CAN_TX_CLASS_PROTECTED_ADVISORY)) ||
       (((dev->tx_builder.kind == CANBUS_TX_BUILD_DETAIL) ||
         (dev->tx_builder.kind == CANBUS_TX_BUILD_TUNING)) &&
        (tx_class != AMS_CAN_TX_CLASS_DETAIL)))
    {
        canbus_increment_u32_sat(&dev->tx_build_class_reject_count);
        return HAL_ERROR;
    }

    frame = &dev->tx_builder.frames[dev->tx_builder.frame_count++];
    memset(frame, 0, sizeof(*frame));
    frame->id = id;
    frame->ide = ide;
    frame->dlc = DATALEN;
    frame->tx_class = tx_class;
    frame->source_tag = dev->tx_builder.source_tag;
    frame->request_id = dev->tx_builder.request_id;
    memcpy(frame->data, payload, DATALEN);
    return HAL_OK;
}

void canbus_tx_build_cancel(canbus_device_t *dev)
{
    if(dev != NULL)
    {
        memset(&dev->tx_builder, 0, sizeof(dev->tx_builder));
    }
}

static bool canbus_tx_claim_task(canbus_device_t *dev)
{
    bool claimed = false;
    taskENTER_CRITICAL();
    canbus_increment_u32_sat(&dev->tx_pump_kick_count);
    if(dev->tx_pump_busy)
    {
        dev->tx_kick_pending = true;
        canbus_increment_u32_sat(&dev->tx_pump_deferred_kick_count);
    }
    else
    {
        dev->tx_pump_busy = true;
        claimed = true;
    }
    taskEXIT_CRITICAL();
    return claimed;
}

static bool canbus_tx_claim_isr(canbus_device_t *dev)
{
    bool claimed = false;
    UBaseType_t mask = CANBUS_ISR_ENTER_CRITICAL();
    canbus_increment_u32_sat(&dev->tx_pump_kick_count);
    if(dev->tx_pump_busy)
    {
        dev->tx_kick_pending = true;
        canbus_increment_u32_sat(&dev->tx_pump_deferred_kick_count);
    }
    else
    {
        dev->tx_pump_busy = true;
        claimed = true;
    }
    CANBUS_ISR_EXIT_CRITICAL(mask);
    return claimed;
}

static bool canbus_tx_release_or_reloop_task(canbus_device_t *dev)
{
    bool reloop;
    taskENTER_CRITICAL();
    reloop = dev->tx_kick_pending;
    if(reloop)
    {
        dev->tx_kick_pending = false;
    }
    else
    {
        dev->tx_pump_busy = false;
    }
    taskEXIT_CRITICAL();
    return reloop;
}

static bool canbus_tx_release_or_reloop_isr(canbus_device_t *dev)
{
    bool reloop;
    UBaseType_t mask = CANBUS_ISR_ENTER_CRITICAL();
    reloop = dev->tx_kick_pending;
    if(reloop)
    {
        dev->tx_kick_pending = false;
    }
    else
    {
        dev->tx_pump_busy = false;
    }
    CANBUS_ISR_EXIT_CRITICAL(mask);
    return reloop;
}

static void canbus_tx_fill_header(CAN_TxHeaderTypeDef *header,
                                  const ams_can_tx_frame_t *frame)
{
    memset(header, 0, sizeof(*header));
    header->IDE = frame->ide;
    header->RTR = CAN_RTR_DATA;
    header->DLC = frame->dlc;
    header->TransmitGlobalTime = DISABLE;
    if(frame->ide == CAN_ID_EXT)
    {
        header->ExtId = frame->id;
    }
    else
    {
        header->StdId = frame->id;
    }
}

static bool canbus_tx_tme_irq_disable(canbus_device_t *dev)
{
    if((dev == NULL) || (dev->hcan == NULL))
    {
        return false;
    }
    if(HAL_CAN_DeactivateNotification(dev->hcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        canbus_increment_u32_sat(&dev->tx_irq_mask_error_count);
        dev->tx_suspended = true;
        return false;
    }
    return true;
}

static bool canbus_tx_tme_irq_restore(canbus_device_t *dev)
{
    if((dev == NULL) || (dev->hcan == NULL))
    {
        return false;
    }
    if(HAL_CAN_ActivateNotification(dev->hcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        canbus_increment_u32_sat(&dev->tx_irq_mask_error_count);
        /* Losing mailbox-complete notification would strand scheduler state.
         * Stop application TX; task-context recovery/manual service can safely
         * re-arm the controller instead of continuing half-observed. */
        dev->tx_suspended = true;
        return false;
    }
    return true;
}

static void canbus_tx_load_failed_locked(canbus_device_t *dev,
                                         const ams_can_tx_token_t *token,
                                         const ams_can_tx_frame_t *frame)
{
    ams_can_tx_load_failed(&dev->tx_scheduler, token);
    canbus_note_hal_load_error_locked(dev, frame);
}

static bool canbus_hardware_busoff(const canbus_device_t *dev)
{
#if AMS_HOST_TEST
    /* The existing synchronous host harness has no register instance. */
    if(dev->hcan->Instance == NULL)
    {
        return false; /* Synchronous legacy harness has no hardware BOFF. */
    }
#endif
    return (dev->hcan->Instance->ESR & CAN_ESR_BOFF) != 0u;
}

/* HAL_CAN_AddTxMessage chooses from hardware-empty mailboxes alone. A mailbox
 * can become empty before its completion callback retires the old token, even
 * between a task-side check and HAL's TSR read. Select one fixed mailbox using
 * BOTH hardware flags and software ownership, then write that exact mailbox.
 * The register encoding is the same as the bundled STM32F7 HAL. */
static HAL_StatusTypeDef canbus_tx_add_owned(canbus_device_t *dev,
                                            const CAN_TxHeaderTypeDef *header,
                                            const uint8_t data[8],
                                            uint32_t *mailbox)
{
#if AMS_HOST_TEST
    if(dev->hcan->Instance == NULL)
    {
        return HAL_CAN_AddTxMessage(dev->hcan, header, data, mailbox);
    }
#endif
    if(HAL_CAN_GetState(dev->hcan) != HAL_CAN_STATE_LISTENING)
    {
        return HAL_ERROR;
    }
    if(dev->tx_suspended || canbus_hardware_busoff(dev))
    {
        return HAL_BUSY;
    }
    uint32_t tsr = dev->hcan->Instance->TSR;
    for(uint8_t i = 0u; i < 3u; i++)
    {
        if((dev->tx_mailbox_meta[i].state != CANBUS_TX_MB_FREE) ||
           ((tsr & (CAN_TSR_TME0 << i)) == 0u) ||
           ((tsr & (CAN_TSR_RQCP0 << (8u * i))) != 0u))
        {
            continue;
        }
        CAN_TxMailBox_TypeDef *mb = &dev->hcan->Instance->sTxMailBox[i];
        mb->TIR = (header->IDE == CAN_ID_STD) ?
            (header->StdId << CAN_TI0R_STID_Pos) :
            ((header->ExtId << CAN_TI0R_EXID_Pos) | CAN_ID_EXT);
        mb->TDTR = header->DLC;
        mb->TDLR = (uint32_t)data[0] | ((uint32_t)data[1] << 8u) |
                   ((uint32_t)data[2] << 16u) | ((uint32_t)data[3] << 24u);
        mb->TDHR = (uint32_t)data[4] | ((uint32_t)data[5] << 8u) |
                   ((uint32_t)data[6] << 16u) | ((uint32_t)data[7] << 24u);
        *mailbox = CAN_TX_MAILBOX0 << i;
        mb->TIR |= CAN_TI0R_TXRQ;
        return HAL_OK;
    }
    return HAL_BUSY; /* Pending completions are congestion, not a load fault. */
}

/* Load one already-reserved scheduler frame. Ordinary traffic masks only the
 * bxCAN TX-mailbox-empty notification across the fixed-mailbox write and
 * metadata commit, closing the "TX completes before mailbox metadata exists"
 * race while leaving unrelated interrupts serviceable. A task-context charger
 * frame adds one short FreeRTOS critical section around its final fail-safe
 * revalidation and TXRQ write so a higher-priority state transition cannot
 * make a just-built ENABLE command stale at the hardware-load boundary. */
static void canbus_charger_fail_safe_at_load(ams_can_tx_frame_t *frame)
{
    if((frame == NULL) ||
       (frame->source_tag != CANBUS_TX_TAG_CHARGER_NORMAL) ||
       (frame->data[4] != CHARGER_CMD_ENABLE))
    {
        return;
    }

    /* A charger enable decision is built in task context, but the higher-
     * priority safety supervisor can change state after the scheduler commit
     * and before the frame reaches bxCAN. Revalidate at the actual hardware
     * load boundary. State transitions and BMS fail-low updates are serialized
     * with this check by the task critical section below; ISR pump execution
     * cannot be preempted by a task.
     *
     * An already-loaded frame can still win the unavoidable hardware abort/
     * transmit race, which is why state exit also sends the repeated zero-
     * demand shutdown burst. This guard prevents software from loading a new
     * stale enable after the transition has become visible. */
    if((app.state != STATE_CHARGE) || !app.bms_state ||
       app.board.charger.shutdown_pending || app.board.charger.tx_fail)
    {
        frame->data[0] = 0u;
        frame->data[1] = 0u;
        frame->data[2] = 0u;
        frame->data[3] = 0u;
        frame->data[4] = CHARGER_CMD_DISABLE;
    }
}

static bool canbus_tx_load_reserved(canbus_device_t *dev,
                                    bool from_isr,
                                    const ams_can_tx_token_t *token,
                                    const ams_can_tx_frame_t *frame)
{
    CAN_TxHeaderTypeDef header;
    ams_can_tx_frame_t load_frame;
    uint32_t mailbox = 0u;
    HAL_StatusTypeDef status;

    if((dev == NULL) || (token == NULL) || (frame == NULL))
    {
        return false;
    }

    if(!canbus_tx_tme_irq_disable(dev))
    {
        if(from_isr)
        {
            UBaseType_t mask = CANBUS_ISR_ENTER_CRITICAL();
            canbus_tx_load_failed_locked(dev, token, frame);
            CANBUS_ISR_EXIT_CRITICAL(mask);
        }
        else
        {
            taskENTER_CRITICAL();
            canbus_tx_load_failed_locked(dev, token, frame);
            taskEXIT_CRITICAL();
        }
        return false;
    }

    load_frame = *frame;
    if(from_isr)
    {
        /* Task-owned state is stable for the duration of this ISR. */
        canbus_charger_fail_safe_at_load(&load_frame);
        canbus_tx_fill_header(&header, &load_frame);
        status = canbus_tx_add_owned(dev, &header, load_frame.data, &mailbox);
    }
    else
    {
        /* Keep the final charger-authority check and bxCAN mailbox load in one
         * task critical section so the higher-priority safety supervisor cannot
         * transition out of CHARGE between those two operations. HAL CAN load
         * is bounded register work and TME callbacks are already masked here. */
        taskENTER_CRITICAL();
        canbus_charger_fail_safe_at_load(&load_frame);
        canbus_tx_fill_header(&header, &load_frame);
        status = canbus_tx_add_owned(dev, &header, load_frame.data, &mailbox);
        taskEXIT_CRITICAL();
    }
    if(status != HAL_OK)
    {
        (void)canbus_tx_tme_irq_restore(dev);
        if(from_isr)
        {
            UBaseType_t mask = CANBUS_ISR_ENTER_CRITICAL();
            ams_can_tx_load_failed(&dev->tx_scheduler, token);
            if(status != HAL_BUSY) canbus_note_hal_load_error_locked(dev, frame);
            CANBUS_ISR_EXIT_CRITICAL(mask);
        }
        else
        {
            taskENTER_CRITICAL();
            ams_can_tx_load_failed(&dev->tx_scheduler, token);
            if(status != HAL_BUSY) canbus_note_hal_load_error_locked(dev, frame);
            taskEXIT_CRITICAL();
        }
        return false;
    }

#if AMS_HOST_TEST
    if(dev->hcan->Instance == NULL)
    {
        /* Legacy synchronous host mode. Register-backed tests use the target
         * ownership and callback path below. */
        taskENTER_CRITICAL();
        ams_can_tx_mark_loaded(&dev->tx_scheduler, token);
        ams_can_tx_mark_complete(&dev->tx_scheduler, token, true, osKernelGetTickCount());
        taskEXIT_CRITICAL();
        canbus_tx_mailbox_meta_t host_meta = {0};
        host_meta.source_tag = frame->source_tag;
        host_meta.request_id = frame->request_id;
        host_meta.token = *token;
        canbus_tx_note_charger_complete_from_isr(&host_meta);
        canbus_increment_u32_sat(&dev->tx_complete_count);
        (void)canbus_tx_tme_irq_restore(dev);
        return true;
    }
#endif
    bool abort_after_load = false;
    bool epoch_valid = false;
    uint8_t index = canbus_mailbox_index_from_mask(mailbox);
    if(index >= 3u)
    {
        /* HAL accepted a frame but returned an impossible mailbox selector.
         * Do not retransmit the source frame blindly because it may already be
         * on the wire. Suspend application TX and let task-context recovery
         * reset the controller epoch. */
        canbus_increment_u32_sat(&dev->tx_hal_load_error_count);
        dev->tx_suspended = true;
        (void)canbus_tx_tme_irq_restore(dev);
        return false;
    }

    uint32_t loaded_tick = from_isr ? CANBUS_ISR_TICK() : osKernelGetTickCount();
    if(from_isr)
    {
        UBaseType_t mask = CANBUS_ISR_ENTER_CRITICAL();
        epoch_valid = (token->controller_epoch == dev->tx_scheduler.controller_epoch);
        if(epoch_valid)
        {
            /* RESERVED is a brief metadata transition while TME callbacks are
             * masked. No callback can observe partially-written ownership. */
            dev->tx_mailbox_meta[index].state = CANBUS_TX_MB_RESERVED;
            dev->tx_mailbox_meta[index].token = *token;
            dev->tx_mailbox_meta[index].can_id = frame->id;
            dev->tx_mailbox_meta[index].loaded_tick = loaded_tick;
            dev->tx_mailbox_meta[index].tx_class = frame->tx_class;
            dev->tx_mailbox_meta[index].source_tag = frame->source_tag;
            dev->tx_mailbox_meta[index].request_id = frame->request_id;
            dev->tx_mailbox_meta[index].controller_epoch =
                dev->tx_scheduler.controller_epoch;
            ams_can_tx_mark_loaded(&dev->tx_scheduler, token);
            dev->tx_mailbox_meta[index].state = CANBUS_TX_MB_LOADED;
            abort_after_load = dev->tx_suspended || ams_can_tx_token_requires_abort(
                &dev->tx_scheduler, token);
            if(abort_after_load)
            {
                dev->tx_mailbox_meta[index].state = CANBUS_TX_MB_ABORT_REQUESTED;
                ams_can_tx_mark_abort_requested(&dev->tx_scheduler, token);
                canbus_increment_u32_sat(&dev->tx_abort_request_count);
            }
        }
        CANBUS_ISR_EXIT_CRITICAL(mask);
    }
    else
    {
        taskENTER_CRITICAL();
        epoch_valid = (token->controller_epoch == dev->tx_scheduler.controller_epoch);
        if(epoch_valid)
        {
            dev->tx_mailbox_meta[index].state = CANBUS_TX_MB_RESERVED;
            dev->tx_mailbox_meta[index].token = *token;
            dev->tx_mailbox_meta[index].can_id = frame->id;
            dev->tx_mailbox_meta[index].loaded_tick = loaded_tick;
            dev->tx_mailbox_meta[index].tx_class = frame->tx_class;
            dev->tx_mailbox_meta[index].source_tag = frame->source_tag;
            dev->tx_mailbox_meta[index].request_id = frame->request_id;
            dev->tx_mailbox_meta[index].controller_epoch =
                dev->tx_scheduler.controller_epoch;
            ams_can_tx_mark_loaded(&dev->tx_scheduler, token);
            dev->tx_mailbox_meta[index].state = CANBUS_TX_MB_LOADED;
            abort_after_load = dev->tx_suspended || ams_can_tx_token_requires_abort(
                &dev->tx_scheduler, token);
            if(abort_after_load)
            {
                dev->tx_mailbox_meta[index].state = CANBUS_TX_MB_ABORT_REQUESTED;
                ams_can_tx_mark_abort_requested(&dev->tx_scheduler, token);
                canbus_increment_u32_sat(&dev->tx_abort_request_count);
            }
        }
        taskEXIT_CRITICAL();
    }

    if(!epoch_valid)
    {
        /* A changed epoch must not leave an unowned hardware request alive. */
        (void)HAL_CAN_AbortTxRequest(dev->hcan, mailbox);
        dev->tx_recovery_pending = true;
        (void)canbus_tx_tme_irq_restore(dev);
        return false;
    }

    /* If this generation became stale or TX was suspended while the frame was
     * RESERVED, request abort before re-enabling TME
     * callbacks. This prevents a completion callback from freeing/reusing the
     * mailbox between our stale check and the abort request. */
    if(abort_after_load)
    {
        HAL_StatusTypeDef abort_status = HAL_CAN_AbortTxRequest(dev->hcan, mailbox);
        if(abort_status != HAL_OK)
        {
            if(from_isr)
            {
                UBaseType_t mask = CANBUS_ISR_ENTER_CRITICAL();
                if(dev->tx_mailbox_meta[index].state == CANBUS_TX_MB_ABORT_REQUESTED)
                {
                    dev->tx_mailbox_meta[index].state = CANBUS_TX_MB_LOADED;
                    ams_can_tx_mark_abort_failed(&dev->tx_scheduler, token);
                    canbus_increment_u32_sat(&dev->tx_abort_request_fail_count);
                }
                CANBUS_ISR_EXIT_CRITICAL(mask);
            }
            else
            {
                taskENTER_CRITICAL();
                if(dev->tx_mailbox_meta[index].state == CANBUS_TX_MB_ABORT_REQUESTED)
                {
                    dev->tx_mailbox_meta[index].state = CANBUS_TX_MB_LOADED;
                    ams_can_tx_mark_abort_failed(&dev->tx_scheduler, token);
                    canbus_increment_u32_sat(&dev->tx_abort_request_fail_count);
                }
                taskEXIT_CRITICAL();
            }
        }
    }

    (void)canbus_tx_tme_irq_restore(dev);
    return true;
}

static void canbus_tx_pump_owned(canbus_device_t *dev, bool from_isr)
{
    for(;;)
    {
        while(!dev->tx_suspended && !dev->tx_latched_inhibit &&
              !canbus_hardware_busoff(dev) &&
              (HAL_CAN_GetTxMailboxesFreeLevel(dev->hcan) > 0u))
        {
            ams_can_tx_token_t token;
            ams_can_tx_frame_t frame;
            bool have_frame;

            if(from_isr)
            {
                UBaseType_t mask = CANBUS_ISR_ENTER_CRITICAL();
                have_frame = ams_can_tx_reserve_next(&dev->tx_scheduler,
                                                     &token, &frame);
                CANBUS_ISR_EXIT_CRITICAL(mask);
            }
            else
            {
                taskENTER_CRITICAL();
                have_frame = ams_can_tx_reserve_next(&dev->tx_scheduler,
                                                     &token, &frame);
                taskEXIT_CRITICAL();
            }
            if(!have_frame)
            {
                break;
            }

            if(!canbus_tx_load_reserved(dev, from_isr, &token, &frame))
            {
                /* Source remains PENDING on ordinary HAL load failure. Avoid
                 * spinning inside one task/ISR invocation; a later kick or
                 * controller recovery will retry the newest state. */
                break;
            }
        }

        if(from_isr)
        {
            if(!canbus_tx_release_or_reloop_isr(dev))
            {
                break;
            }
        }
        else if(!canbus_tx_release_or_reloop_task(dev))
        {
            break;
        }
    }
}

void canbus_tx_kick(canbus_device_t *dev)
{
    if((dev == NULL) || (dev->hcan == NULL))
    {
        return;
    }
    if(canbus_tx_claim_task(dev))
    {
        canbus_tx_pump_owned(dev, false);
    }
}

static void canbus_tx_kick_from_isr(canbus_device_t *dev)
{
    if((dev == NULL) || (dev->hcan == NULL))
    {
        return;
    }
    if(canbus_tx_claim_isr(dev))
    {
        canbus_tx_pump_owned(dev, true);
    }
}

HAL_StatusTypeDef canbus_tx_build_commit(canbus_device_t *dev,
                                         uint16_t required_count)
{
    bool ok = false;
    uint32_t stale_generation = 0u;

    if((dev == NULL) || !dev->tx_builder.active ||
       (dev->tx_builder.frame_count == 0u))
    {
        return HAL_ERROR;
    }

    taskENTER_CRITICAL();
    switch(dev->tx_builder.kind)
    {
    case CANBUS_TX_BUILD_CRITICAL:
        ok = ams_can_tx_publish_critical(&dev->tx_scheduler,
                                         dev->tx_builder.generation,
                                         dev->tx_builder.publish_tick,
                                         dev->tx_builder.frames,
                                         dev->tx_builder.frame_count);
        break;
    case CANBUS_TX_BUILD_PROTECTED:
        ok = ams_can_tx_publish_protected(&dev->tx_scheduler,
                                          dev->tx_builder.generation,
                                          dev->tx_builder.publish_tick,
                                          dev->tx_builder.frames,
                                          dev->tx_builder.frame_count,
                                          required_count,
                                          &stale_generation);
        break;
    case CANBUS_TX_BUILD_DETAIL:
        ok = ams_can_tx_publish_detail(&dev->tx_scheduler,
                                       dev->tx_builder.generation,
                                       dev->tx_builder.publish_tick,
                                       dev->tx_builder.frames,
                                       dev->tx_builder.frame_count);
        break;
    case CANBUS_TX_BUILD_TUNING:
        ok = ams_can_tx_publish_tuning(&dev->tx_scheduler,
                                       dev->tx_builder.generation,
                                       dev->tx_builder.publish_tick,
                                       dev->tx_builder.frames,
                                       dev->tx_builder.frame_count);
        break;
    default:
        break;
    }
    taskEXIT_CRITICAL();

    memset(&dev->tx_builder, 0, sizeof(dev->tx_builder));
    if(!ok)
    {
        canbus_increment_u32_sat(&dev->tx_build_commit_reject_count);
        return HAL_ERROR;
    }

    canbus_tx_abort_stale(dev);
    canbus_tx_kick(dev);
    return HAL_OK;
}

static void canbus_tx_abort_stale(canbus_device_t *dev)
{
    uint32_t abort_mask = 0u;

    if((dev == NULL) || (dev->hcan == NULL))
    {
        return;
    }

    /* Keep TX-complete/abort callbacks from freeing and reusing a mailbox
     * between the metadata transition and HAL_CAN_AbortTxRequest(). RX/error
     * interrupts remain enabled; this is not a global critical section. */
    if(!canbus_tx_tme_irq_disable(dev))
    {
        return;
    }

    taskENTER_CRITICAL();
    for(uint8_t i = 0u; i < 3u; i++)
    {
        canbus_tx_mailbox_meta_t *meta = &dev->tx_mailbox_meta[i];
        if((meta->state == CANBUS_TX_MB_LOADED) &&
           ams_can_tx_token_requires_abort(&dev->tx_scheduler, &meta->token) &&
           (meta->controller_epoch == dev->tx_scheduler.controller_epoch))
        {
            meta->state = CANBUS_TX_MB_ABORT_REQUESTED;
            ams_can_tx_mark_abort_requested(&dev->tx_scheduler, &meta->token);
            abort_mask |= (CAN_TX_MAILBOX0 << i);
            canbus_increment_u32_sat(&dev->tx_abort_request_count);
        }
    }
    taskEXIT_CRITICAL();

    if(abort_mask != 0u)
    {
        HAL_StatusTypeDef status = HAL_CAN_AbortTxRequest(dev->hcan, abort_mask);
        if(status != HAL_OK)
        {
            taskENTER_CRITICAL();
            for(uint8_t i = 0u; i < 3u; i++)
            {
                canbus_tx_mailbox_meta_t *meta = &dev->tx_mailbox_meta[i];
                if((meta->state == CANBUS_TX_MB_ABORT_REQUESTED) &&
                   ((abort_mask & (CAN_TX_MAILBOX0 << i)) != 0u) &&
                   (meta->controller_epoch == dev->tx_scheduler.controller_epoch))
                {
                    meta->state = CANBUS_TX_MB_LOADED;
                    ams_can_tx_mark_abort_failed(&dev->tx_scheduler,
                                                 &meta->token);
                    canbus_increment_u32_sat(&dev->tx_abort_request_fail_count);
                }
            }
            taskEXIT_CRITICAL();
        }
    }

    (void)canbus_tx_tme_irq_restore(dev);
}

static void canbus_tx_reset_mailbox_metadata(canbus_device_t *dev)
{
    for(uint8_t i = 0u; i < 3u; i++)
    {
        memset(&dev->tx_mailbox_meta[i], 0, sizeof(dev->tx_mailbox_meta[i]));
        dev->tx_mailbox_meta[i].state = CANBUS_TX_MB_FREE;
    }
}

void canbus_tx_note_busoff(canbus_device_t *dev)
{
    if((dev == NULL) || (dev->hcan == NULL))
    {
        return;
    }
    taskENTER_CRITICAL();
    dev->tx_suspended = true;
    dev->tx_recovery_pending = true;
    taskEXIT_CRITICAL();
    /* Keep tokens until hardware completes/aborts. ABOM must never retry an
     * old enable command after software has forgotten its mailbox owner. */
    (void)HAL_CAN_AbortTxRequest(dev->hcan,
        CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
}

static bool canbus_tx_mailboxes_settled(const canbus_device_t *dev)
{
    if(dev->tx_pump_busy)
    {
        return false;
    }
#if AMS_HOST_TEST
    if(dev->hcan->Instance == NULL) return true;
#endif
    uint32_t tsr = dev->hcan->Instance->TSR;
    const uint32_t empty = CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2;
    if(((tsr & empty) != empty) ||
       ((tsr & (CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2)) != 0u))
    {
        return false;
    }
    for(uint8_t i = 0u; i < 3u; i++)
    {
        if((dev->tx_mailbox_meta[i].state != CANBUS_TX_MB_FREE) ||
           ((dev->hcan->Instance->sTxMailBox[i].TIR & CAN_TI0R_TXRQ) != 0u))
        {
            return false;
        }
    }
    return true;
}

static bool canbus_tx_note_recovered_for_sequence(canbus_device_t *dev,
                                                     uint32_t expected_sequence)
{
    bool mailboxes_settled;
    bool transport_ready;

    if((dev == NULL) || (dev->hcan == NULL))
    {
        return false;
    }

    /* Recovery settlement and controller-epoch reset are one transaction with
     * respect to physical BOFF identity. The sequence is checked after the HAL
     * state read as well as the hardware BOFF bit so a new SCE event cannot be
     * erased even if ABOM has already cleared the electrical BOFF condition. */
    taskENTER_CRITICAL();
    transport_ready =
        (HAL_CAN_GetState(dev->hcan) == HAL_CAN_STATE_LISTENING) &&
        !canbus_hardware_busoff(dev) &&
        (dev->busoff_event_sequence == expected_sequence);
    mailboxes_settled = transport_ready && canbus_tx_mailboxes_settled(dev);
    if(!mailboxes_settled ||
       (dev->busoff_event_sequence != expected_sequence) ||
       canbus_hardware_busoff(dev))
    {
        taskEXIT_CRITICAL();
        if(!mailboxes_settled)
        {
            (void)HAL_CAN_AbortTxRequest(dev->hcan,
                CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        }
        return false;
    }

    ams_can_tx_controller_epoch_reset(&dev->tx_scheduler);
    canbus_tx_reset_mailbox_metadata(dev);
    /* Recovery is a freshness boundary. The task republishes current charger
     * decisions and a complete protected bundle before TX resumes. */
    memset(&dev->tx_scheduler.critical_active, 0,
           sizeof(dev->tx_scheduler.critical_active));
    memset(&dev->tx_scheduler.protected_active, 0,
           sizeof(dev->tx_scheduler.protected_active));
    dev->tx_recovery_pending = false;
    dev->tx_refresh_pending = true;
    dev->tx_suspended = true;
    canbus_increment_u32_sat(&dev->tx_recovery_epoch_count);
    taskEXIT_CRITICAL();
    return true;
}

bool canbus_tx_note_recovered(canbus_device_t *dev)
{
    uint32_t sequence;

    if((dev == NULL) || (dev->hcan == NULL))
    {
        return false;
    }

    taskENTER_CRITICAL();
    sequence = dev->busoff_event_sequence;
    taskEXIT_CRITICAL();
    return canbus_tx_note_recovered_for_sequence(dev, sequence);
}

void canbus_tx_resume_after_refresh(canbus_device_t *dev)
{
    if((dev == NULL) || (dev->hcan == NULL)) return;
    taskENTER_CRITICAL();
    if(dev->tx_refresh_pending && !dev->tx_recovery_pending &&
       !dev->tx_latched_inhibit && !canbus_hardware_busoff(dev))
    {
        dev->tx_refresh_pending = false;
        dev->tx_suspended = false;
    }
    taskEXIT_CRITICAL();
    canbus_tx_kick(dev);
}

static void canbus_tx_note_charger_complete_from_isr(const canbus_tx_mailbox_meta_t *meta)
{
    charger_t *ccs = &app.board.charger;
    if(meta->source_tag == CANBUS_TX_TAG_CHARGER_NORMAL)
    {
        ccs->tx_fail = false;
        ccs->last_tx_status = HAL_OK;
        canbus_increment_u32_sat(&ccs->tx_count);
    }
    else if(meta->source_tag == CANBUS_TX_TAG_CHARGER_SHUTDOWN)
    {
        ccs->tx_fail = false;
        ccs->last_tx_status = HAL_OK;
        ccs->last_shutdown_status = HAL_OK;
        canbus_increment_u32_sat(&ccs->tx_count);
        canbus_increment_u32_sat(&ccs->shutdown_tx_count);
        if(ccs->shutdown_pending &&
           (ccs->shutdown_request_count == meta->request_id))
        {
            if(ccs->shutdown_frames_remaining > 0u)
            {
                ccs->shutdown_frames_remaining--;
            }
            if(ccs->shutdown_frames_remaining == 0u)
            {
                ccs->shutdown_pending = false;
            }
        }
    }
}

static void canbus_tx_complete_callback(canbus_device_t *dev,
                                        uint8_t mailbox_index,
                                        bool aborted)
{
    canbus_tx_mailbox_meta_t meta;
    UBaseType_t mask;

    if((dev == NULL) || (mailbox_index >= 3u))
    {
        return;
    }

    mask = CANBUS_ISR_ENTER_CRITICAL();
    meta = dev->tx_mailbox_meta[mailbox_index];
    if((meta.state != CANBUS_TX_MB_LOADED) &&
       (meta.state != CANBUS_TX_MB_ABORT_REQUESTED))
    {
        canbus_increment_u32_sat(&dev->tx_unexpected_callback_count);
        CANBUS_ISR_EXIT_CRITICAL(mask);
        return;
    }

    if(meta.controller_epoch != dev->tx_scheduler.controller_epoch)
    {
        canbus_increment_u32_sat(&dev->tx_unexpected_callback_count);
        memset(&dev->tx_mailbox_meta[mailbox_index], 0,
               sizeof(dev->tx_mailbox_meta[mailbox_index]));
        dev->tx_mailbox_meta[mailbox_index].state = CANBUS_TX_MB_FREE;
        CANBUS_ISR_EXIT_CRITICAL(mask);
        return;
    }

    if(aborted)
    {
        canbus_increment_u32_sat(&dev->tx_abort_complete_count);
    }
    else
    {
        canbus_increment_u32_sat(&dev->tx_complete_count);
        if(meta.state == CANBUS_TX_MB_ABORT_REQUESTED)
        {
            canbus_increment_u32_sat(&dev->tx_abort_race_complete_count);
        }
        canbus_tx_note_charger_complete_from_isr(&meta);
    }
    ams_can_tx_mark_complete(&dev->tx_scheduler, &meta.token, !aborted, CANBUS_ISR_TICK());
    memset(&dev->tx_mailbox_meta[mailbox_index], 0,
           sizeof(dev->tx_mailbox_meta[mailbox_index]));
    dev->tx_mailbox_meta[mailbox_index].state = CANBUS_TX_MB_FREE;
    CANBUS_ISR_EXIT_CRITICAL(mask);

    /* Refill only after the HAL has dispatched its entire TSR snapshot. */
}

void canbus_irq_handler(CAN_HandleTypeDef *hcan)
{
    canbus_device_t *dev = &app.board.canbus;
    bool tx_enabled = (hcan->Instance->IER & CAN_IT_TX_MAILBOX_EMPTY) != 0u;
    HAL_CAN_IRQHandler(hcan);
    if(dev->hcan != hcan) return;

    /* HAL clears TERR/ALST without issuing a mailbox callback. With refill
     * deferred, an owned mailbox that is empty with RQCP already cleared is
     * exactly such a terminal failure. Newly arrived completions still have
     * RQCP set and belong to the next HAL dispatch. */
    if(tx_enabled)
    {
        uint32_t tsr = hcan->Instance->TSR;
        for(uint8_t i = 0u; i < 3u; i++)
        {
            if(((tsr & (CAN_TSR_TME0 << i)) != 0u) &&
               ((tsr & (CAN_TSR_RQCP0 << (8u * i))) == 0u) &&
               ((dev->tx_mailbox_meta[i].state == CANBUS_TX_MB_LOADED) ||
                (dev->tx_mailbox_meta[i].state == CANBUS_TX_MB_ABORT_REQUESTED)))
            {
                canbus_tx_complete_callback(dev, i, true);
            }
        }
    }
    /* RX/SCE may interrupt a task while it masks TME around a load or abort.
     * Do not start a new pump or re-enable TME inside that transaction. */
    if(tx_enabled) canbus_tx_kick_from_isr(dev);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if(app.board.canbus.hcan == hcan)
    {
        canbus_tx_complete_callback(&app.board.canbus, 0u, false);
    }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if(app.board.canbus.hcan == hcan)
    {
        canbus_tx_complete_callback(&app.board.canbus, 1u, false);
    }
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if(app.board.canbus.hcan == hcan)
    {
        canbus_tx_complete_callback(&app.board.canbus, 2u, false);
    }
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
    if(app.board.canbus.hcan == hcan)
    {
        canbus_tx_complete_callback(&app.board.canbus, 0u, true);
    }
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
    if(app.board.canbus.hcan == hcan)
    {
        canbus_tx_complete_callback(&app.board.canbus, 1u, true);
    }
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
    if(app.board.canbus.hcan == hcan)
    {
        canbus_tx_complete_callback(&app.board.canbus, 2u, true);
    }
}

static void canbus_force_bms_low_from_isr(app_data_t *data)
{
    if(data == NULL)
    {
        return;
    }

    /* ISR-safe fail-low path: only primitive state/flag writes plus one GPIO
     * reset. Retained logging and normal balance-mute accounting remain task
     * context work. This same primitive is safe from controlled bench fault
     * injection running in task context. */
    data->bms_state = false;
    data->adbms_urgent_mute_requested = true;
    HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, GPIO_PIN_RESET);
}

void canbus_record_busoff_event(canbus_device_t *dev, app_data_t *data,
                                uint32_t event_tick, uint8_t event_state)
{
    uint8_t count;

    if((dev == NULL) || (data == NULL))
    {
        return;
    }

    /* Every genuine physical-equivalent BOFF transition gets a sequence
     * number immediately. The 10 Hz CAN task later consumes the sequence
     * delta; it is not allowed to collapse clustered events. */
    dev->busoff_event_tick = event_tick;
    dev->busoff_event_state = event_state;
    dev->busoff_event_sequence++;
    dev->busoff_event_pending = true;

    /* Maintain the exact sliding window in ISR-owned state. Only the most
     * recent AMS_CAN_BUSOFF_REPEAT_LIMIT events are needed to decide whether
     * the repeat threshold has been reached. Unsigned subtraction preserves
     * the 32-bit tick-wrap contract. */
    count = dev->busoff_window_count;
    while((count > 0u) &&
          ((uint32_t)(event_tick - dev->busoff_recent_ticks[0]) >
           AMS_CAN_BUSOFF_REPEAT_WINDOW_MS))
    {
        for(uint8_t i = 1u; i < count; i++)
        {
            dev->busoff_recent_ticks[i - 1u] = dev->busoff_recent_ticks[i];
        }
        count--;
    }
    if(count < AMS_CAN_BUSOFF_REPEAT_LIMIT)
    {
        dev->busoff_recent_ticks[count++] = event_tick;
    }
    else
    {
        for(uint8_t i = 1u; i < AMS_CAN_BUSOFF_REPEAT_LIMIT; i++)
        {
            dev->busoff_recent_ticks[i - 1u] = dev->busoff_recent_ticks[i];
        }
        dev->busoff_recent_ticks[AMS_CAN_BUSOFF_REPEAT_LIMIT - 1u] = event_tick;
    }
    dev->busoff_window_count = count;
    dev->busoff_window_start_tick =
        (count > 0u) ? dev->busoff_recent_ticks[0] : event_tick;

    /* Revoke authority immediately and expose the physical-equivalent fault
     * to the higher-priority supervisor before task-context accounting runs.
     * The first event owns the 500 ms discharge epoch; later BOFFs do not
     * move that safety deadline. */
    data->can_authority_ready = false;
    data->can_busoff_fault = true;
    data->canbus_fault = true;
    if(!data->can_busoff_recovery_active)
    {
        data->can_busoff_recovery_start_tick = event_tick;
        data->can_busoff_recovery_state = event_state;
        data->can_busoff_recovery_active = true;
    }

    /* CHARGE/BALANCE and CAN-backed HIL have no continuity grace. */
    if(!data->can_busoff_hard_fault_latched)
    {
        ams_can_policy_latch_reason_t reason = AMS_CAN_POLICY_LATCH_NONE;
        if(event_state == (uint8_t)STATE_CHARGE)
        {
            reason = AMS_CAN_POLICY_LATCH_CHARGE_BUSOFF;
        }
        else if(event_state == (uint8_t)STATE_BALANCE)
        {
            reason = AMS_CAN_POLICY_LATCH_BALANCE_BUSOFF;
        }
#if AMS_HIL_REPLACE_ADBMS
        else
        {
            reason = AMS_CAN_POLICY_LATCH_HIL_MEASUREMENT_LOSS;
        }
#endif
        if(reason != AMS_CAN_POLICY_LATCH_NONE)
        {
            data->can_busoff_hard_fault_latched = true;
            data->can_busoff_policy_latch_reason = reason;
            canbus_force_bms_low_from_isr(data);
        }
    }

    /* The third genuine event in the sliding 10-second window is itself the
     * safety boundary. Latch in ISR context rather than waiting for the CAN
     * task, otherwise three clustered BOFFs could be hidden by task latency. */
    if(count >= AMS_CAN_BUSOFF_REPEAT_LIMIT)
    {
        dev->tx_latched_inhibit = true;
        data->can_authority_ready = false;
        if(!data->can_busoff_hard_fault_latched)
        {
            data->can_busoff_hard_fault_latched = true;
            data->can_busoff_policy_latch_reason =
                AMS_CAN_POLICY_LATCH_REPEATED_BUSOFF;
        }
        canbus_force_bms_low_from_isr(data);
    }

    /* Suspend/abort at each event boundary. A later event during pending
     * recovery is preserved by the sequence even though recovery state is
     * already active. */
    dev->tx_suspended = true;
    if(dev->hcan != NULL)
    {
        (void)HAL_CAN_AbortTxRequest(dev->hcan,
            CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
}

void canbus_sce_irq_note(CAN_HandleTypeDef *hcan)
{
    canbus_device_t *dev = &app.board.canbus;

    if((hcan == NULL) || (dev->hcan != hcan) || (hcan->Instance == NULL))
    {
        return;
    }

    /* SCE/ERRI is the event source. HAL_CAN_GetError() is intentionally not
     * used here because HAL ErrorCode accumulates BOF until reset and could
     * make a later unrelated error callback look like another bus-off. */
    if(((hcan->Instance->MSR & CAN_MSR_ERRI) != 0u) &&
       ((hcan->Instance->ESR & CAN_ESR_BOFF) != 0u))
    {
        canbus_record_busoff_event(dev, &app, CANBUS_ISR_TICK(),
                                   (uint8_t)app.state);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    canbus_device_t *dev = &app.board.canbus;
    uint32_t isr_error;

    if((hcan == NULL) || (dev->hcan != hcan))
    {
        return;
    }

    isr_error = HAL_CAN_GetError(hcan);
    dev->error_isr_code |= isr_error;
#if AMS_HOST_TEST
    if(((isr_error & HAL_CAN_ERROR_BOF) != 0u) &&
       (dev->hcan->Instance == NULL))
    {
        /* Legacy host harness has no SCE register instance; treat the direct
         * callback as one physical-equivalent event for regression parity. */
        canbus_record_busoff_event(dev, &app, CANBUS_ISR_TICK(),
                                   (uint8_t)app.state);
    }
#endif
    if((isr_error & HAL_CAN_ERROR_BOF) != 0u)
    {
        /* Sticky HAL BOF is not physical event identity, but it remains safe
         * to keep TX suspended while task/SCE bookkeeping settles. */
        dev->tx_suspended = true;
        (void)HAL_CAN_AbortTxRequest(dev->hcan,
            CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
    if((isr_error & HAL_CAN_ERROR_RX_FOV0) != 0u)
    {
        canbus_increment_u32_sat(&dev->rx_fifo_overrun_count);
    }
    dev->error_isr_pending = true;
}

HAL_StatusTypeDef canbus_device_init(canbus_device_t *dev, CAN_HandleTypeDef *hcan)
{
    if(dev == NULL)
    {
        return HAL_ERROR;
    }

    dev->hcan = hcan;
    dev->tx_mailbox = 0u;
    memset(&dev->rx_packet, 0, sizeof(dev->rx_packet));
    ams_can_tx_scheduler_init(&dev->tx_scheduler);
    memset(&dev->tx_builder, 0, sizeof(dev->tx_builder));
    canbus_tx_reset_mailbox_metadata(dev);
    dev->tx_pump_busy = false;
    dev->tx_kick_pending = false;
    dev->tx_suspended = false;
    dev->tx_recovery_pending = false;
    dev->tx_refresh_pending = false;
    dev->tx_latched_inhibit = false;
    dev->error_isr_pending = false;
    dev->error_isr_code = HAL_CAN_ERROR_NONE;
    dev->busoff_event_sequence = 0u;
    dev->busoff_event_consumed_sequence = 0u;
    dev->busoff_event_pending = false;
    dev->busoff_event_tick = 0u;
    dev->busoff_event_state = (uint8_t)STATE_NULL;
    for(uint8_t i = 0u; i < CANBUS_BUSOFF_WINDOW_SLOTS; i++)
    {
        dev->busoff_recent_ticks[i] = 0u;
    }
    dev->tx_generation_counter = 0u;
    dev->tx_hal_load_error_count = 0u;
    dev->tx_hal_load_error_reported = 0u;
    dev->tx_hal_load_error_critical_count = 0u;
    dev->tx_hal_load_error_critical_reported = 0u;
    dev->tx_hal_load_error_protected_count = 0u;
    dev->tx_hal_load_error_protected_reported = 0u;
    dev->tx_hal_load_error_detail_count = 0u;
    dev->tx_hal_load_error_detail_reported = 0u;
    dev->tx_charger_normal_load_error_count = 0u;
    dev->tx_charger_normal_load_error_reported = 0u;
    dev->tx_charger_shutdown_load_error_count = 0u;
    dev->tx_charger_shutdown_load_error_reported = 0u;
    dev->tx_last_hal_load_error_source_tag = CANBUS_TX_TAG_NONE;
    dev->tx_last_hal_load_error_class = AMS_CAN_TX_CLASS_DETAIL;
    dev->tx_complete_count = 0u;
    dev->tx_abort_request_count = 0u;
    dev->tx_abort_complete_count = 0u;
    dev->tx_abort_race_complete_count = 0u;
    dev->tx_abort_request_fail_count = 0u;
    dev->tx_unexpected_callback_count = 0u;
    dev->tx_pump_kick_count = 0u;
    dev->tx_pump_deferred_kick_count = 0u;
    dev->tx_irq_mask_error_count = 0u;
    dev->tx_irq_mask_error_reported = 0u;
    dev->tx_build_overflow_count = 0u;
    dev->tx_build_detail_overflow_count = 0u;
    dev->tx_build_class_reject_count = 0u;
    dev->tx_build_commit_reject_count = 0u;
    dev->tx_recovery_epoch_count = 0u;
    dev->busoff_window_start_tick = 0u;
    dev->busoff_window_count = 0u;
    memset(dev->rx_queue, 0, sizeof(dev->rx_queue));
    dev->rx_queue_head = 0u;
    dev->rx_queue_tail = 0u;
    dev->rx_queue_high_water = 0u;
    dev->rx_isr_count = 0u;
    dev->rx_processed_count = 0u;
    dev->rx_filtered_count = 0u;
    dev->rx_remote_reject_count = 0u;
    dev->rx_malformed_count = 0u;
    dev->rx_fifo_overrun_count = 0u;
    dev->rx_queue_drop_count = 0u;
    dev->rx_hal_error_count = 0u;
    dev->rx_feedback_count = 0u;
    dev->rx_queue_drop_reported = 0u;
    dev->rx_hal_error_reported = 0u;
    dev->rx_fifo_overrun_reported = 0u;
    dev->ecu_feedback_version = 0u;
    dev->ecu_feedback_protected_sequence = 0u;
    dev->ecu_feedback_snapshot_sequence = 0u;
    dev->ecu_feedback_flags = 0u;
    dev->ecu_feedback_rx_diag = 0u;
    dev->ecu_feedback_uptime_counter = 0u;
    dev->ecu_feedback_last_tick = 0u;
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
		hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
             CAN_IT_RX_FIFO0_OVERRUN |
             CAN_IT_TX_MAILBOX_EMPTY |
             CAN_IT_BUSOFF | CAN_IT_ERROR);
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
    uint32_t fifo_overrun_count;
    uint32_t loss_delta;

    drop_count = dev->rx_queue_drop_count;
    hal_error_count = dev->rx_hal_error_count;
    fifo_overrun_count = dev->rx_fifo_overrun_count;
    loss_delta = (drop_count - dev->rx_queue_drop_reported) +
                 (hal_error_count - dev->rx_hal_error_reported) +
                 (fifo_overrun_count - dev->rx_fifo_overrun_reported);

    dev->rx_queue_drop_reported = drop_count;
    dev->rx_hal_error_reported = hal_error_count;
    dev->rx_fifo_overrun_reported = fifo_overrun_count;

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

    if((frame->rtr == CAN_RTR_DATA) &&
       (frame->ide == CAN_ID_STD) &&
       (frame->id == AMS_ECU_DIAG_FEEDBACK_CAN_ID) &&
       (frame->dlc == AMS_ECU_DIAG_FEEDBACK_DLC))
    {
        dev->ecu_feedback_version = frame->data[0];
        dev->ecu_feedback_protected_sequence = frame->data[1];
        dev->ecu_feedback_snapshot_sequence = frame->data[2];
        dev->ecu_feedback_flags = frame->data[3];
        dev->ecu_feedback_rx_diag =
            (uint16_t)(((uint16_t)frame->data[4] << 8u) | frame->data[5]);
        dev->ecu_feedback_uptime_counter =
            (uint16_t)(((uint16_t)frame->data[6] << 8u) | frame->data[7]);
        dev->ecu_feedback_last_tick = frame->tick;
        canbus_increment_u32_sat(&dev->rx_feedback_count);
        return;
    }

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

HAL_StatusTypeDef canbus_recover(canbus_device_t *dev, app_data_t *data)
{
    HAL_StatusTypeDef reset_status;
    uint32_t service_sequence;
    bool recovery_settled = false;
    bool service_commit = false;

    if((dev == NULL) || (dev->hcan == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    taskENTER_CRITICAL();
    if(dev->tx_pump_busy)
    {
        taskEXIT_CRITICAL();
        return HAL_BUSY;
    }
    /* Explicit service recovery may clear events that existed when service was
     * requested, but a BOFF arriving after this snapshot belongs to a new fault
     * epoch and must survive the recovery attempt. */
    service_sequence = dev->busoff_event_sequence;
    dev->tx_suspended = true;
    taskEXIT_CRITICAL();

    /* Manual/service recovery is the only path that clears the repeated
     * bus-off application-TX latch. Settle hardware ownership and require
     * fresh task publications before resuming transmission. */
    canbus_tx_note_busoff(dev);
    /* Stopping an already-stopped controller can itself return an error. The
     * recovery result is therefore based on reset, restart, and notification
     * activation, while still making the best-effort stop first. */
    (void)HAL_CAN_Stop(dev->hcan);
    reset_status = HAL_CAN_ResetError(dev->hcan);
    dev->start_status = HAL_CAN_Start(dev->hcan);
    dev->started = (dev->start_status == HAL_OK);

    if(dev->started)
    {
        dev->notification_status = HAL_CAN_ActivateNotification(
            dev->hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_RX_FIFO0_OVERRUN |
            CAN_IT_TX_MAILBOX_EMPTY |
            CAN_IT_BUSOFF | CAN_IT_ERROR);
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

    if(dev->init_status == HAL_OK)
    {
        recovery_settled =
            canbus_tx_note_recovered_for_sequence(dev, service_sequence);
        if(!recovery_settled)
        {
            dev->init_status = HAL_BUSY; /* Task polling finishes settlement. */
        }
        else
        {
            /* Do not split the new-event check from either transport OR
             * application safety-state clearing. A BOFF after controller
             * settlement must win over the administrative recovery command;
             * otherwise a CLI-side post-return cleanup could erase that event
             * before the 10 Hz CAN task observes it. */
            taskENTER_CRITICAL();
            if((dev->busoff_event_sequence == service_sequence) &&
               !canbus_hardware_busoff(dev))
            {
                dev->tx_latched_inhibit = false;
                dev->busoff_window_start_tick = 0u;
                dev->busoff_window_count = 0u;
                for(uint8_t i = 0u; i < CANBUS_BUSOFF_WINDOW_SLOTS; i++)
                {
                    dev->busoff_recent_ticks[i] = 0u;
                }

                data->can_busoff_fault = false;
                data->can_recover_pending = false;
                data->can_error_code = HAL_CAN_ERROR_NONE;
                data->canbus_fault = false;
                data->can_authority_ready = false;
                data->can_authority_complete_generation_baseline =
                    dev->tx_scheduler.
                        protected_required_last_complete_generation;
                data->can_busoff_recovery_active = false;
                data->can_busoff_recovery_start_tick = 0u;
                data->can_busoff_recovery_state = (uint8_t)STATE_NULL;
                data->can_authority_refresh_pending = true;
                data->can_busoff_hard_fault_latched = false;
                data->can_busoff_policy_latch_reason =
                    AMS_CAN_POLICY_LATCH_NONE;
                canbus_increment_u32_sat(&data->can_recover_count);
                service_commit = true;
            }
            taskEXIT_CRITICAL();

            if(!service_commit)
            {
                dev->init_status = HAL_BUSY;
                /* canbus_tx_note_recovered_for_sequence() already created a
                 * fresh controller epoch. Restore pending-recovery state for
                 * the newer event rather than leaving transport refreshable. */
                canbus_tx_note_busoff(dev);
            }
            else
            {
                ams_fault_log_event(AMS_FAULT_LOG_CAN_RECOVERED, 0u,
                                    data->can_recover_count, 0u);
            }
        }
    }

    return dev->init_status;
}

static void canbus_latch_policy_fault(app_data_t *data,
                                      ams_can_policy_latch_reason_t reason)
{
    if(data == NULL)
    {
        return;
    }

    data->can_authority_ready = false;
    if(!data->can_busoff_hard_fault_latched)
    {
        data->can_busoff_hard_fault_latched = true;
        data->can_busoff_policy_latch_reason = reason;
    }

    /* Immediate physical fail-low. The high-priority supervisor subsequently
     * incorporates the sticky CAN policy latch into hard_fault. */
    set_bms(false);
}

void canbus_poll_errors(canbus_device_t *dev, app_data_t *data)
{
    uint32_t err;
    uint32_t now;
    uint32_t busoff_event_tick;
    uint8_t busoff_event_state;
    uint32_t busoff_event_sequence;
    uint32_t new_busoff_events = 0u;
    uint32_t tx_irq_mask_new = 0u;
    uint32_t tx_hal_load_new = 0u;
    uint32_t charger_normal_load_new = 0u;
    uint32_t charger_shutdown_load_new = 0u;
    bool have_busoff_event = false;
    bool had_new_hal_load_error = false;
    bool had_new_critical_load_error = false;
    bool had_new_detail_load_error = false;
    bool had_new_isr_error = false;

    if((dev == NULL) || (dev->hcan == NULL) || (data == NULL))
    {
        return;
    }

    now = osKernelGetTickCount();
    taskENTER_CRITICAL();
    err = HAL_CAN_GetError(dev->hcan);
    if(dev->error_isr_pending)
    {
        err |= dev->error_isr_code;
        dev->error_isr_pending = false;
        dev->error_isr_code = HAL_CAN_ERROR_NONE;
        had_new_isr_error = true;
    }
    busoff_event_tick = now;
    busoff_event_state = (uint8_t)data->state;
    busoff_event_sequence = dev->busoff_event_sequence;
    new_busoff_events =
        busoff_event_sequence - dev->busoff_event_consumed_sequence;
    if(new_busoff_events != 0u)
    {
        busoff_event_tick = dev->busoff_event_tick;
        busoff_event_state = dev->busoff_event_state;
        dev->busoff_event_consumed_sequence = busoff_event_sequence;
        dev->busoff_event_pending = false;
        have_busoff_event = true;
    }

    /* ISR-written TX diagnostics use snapshot-and-ack semantics. Update each
     * reported watermark to exactly the value used for this poll while the CAN
     * ISR is masked; increments after this snapshot remain visible next poll
     * instead of being accidentally acknowledged without processing. */
    tx_irq_mask_new =
        dev->tx_irq_mask_error_count - dev->tx_irq_mask_error_reported;
    dev->tx_irq_mask_error_reported = dev->tx_irq_mask_error_count;

    tx_hal_load_new =
        dev->tx_hal_load_error_count - dev->tx_hal_load_error_reported;
    dev->tx_hal_load_error_reported = dev->tx_hal_load_error_count;

    had_new_critical_load_error =
        (dev->tx_hal_load_error_critical_count !=
         dev->tx_hal_load_error_critical_reported) ||
        (dev->tx_hal_load_error_protected_count !=
         dev->tx_hal_load_error_protected_reported);
    dev->tx_hal_load_error_critical_reported =
        dev->tx_hal_load_error_critical_count;
    dev->tx_hal_load_error_protected_reported =
        dev->tx_hal_load_error_protected_count;

    had_new_detail_load_error =
        dev->tx_hal_load_error_detail_count !=
        dev->tx_hal_load_error_detail_reported;
    dev->tx_hal_load_error_detail_reported =
        dev->tx_hal_load_error_detail_count;

    charger_normal_load_new =
        dev->tx_charger_normal_load_error_count -
        dev->tx_charger_normal_load_error_reported;
    dev->tx_charger_normal_load_error_reported =
        dev->tx_charger_normal_load_error_count;
    charger_shutdown_load_new =
        dev->tx_charger_shutdown_load_error_count -
        dev->tx_charger_shutdown_load_error_reported;
    dev->tx_charger_shutdown_load_error_reported =
        dev->tx_charger_shutdown_load_error_count;
    taskEXIT_CRITICAL();
    if(canbus_hardware_busoff(dev)) err |= HAL_CAN_ERROR_BOF;

#if AMS_HOST_TEST
    /* Compatibility only for legacy host harnesses with no register-backed
     * CAN instance. On target, SCE/ERRI is the sole physical BOFF identity.
     * Never synthesize from a hardware/sticky BOF observation here: a real SCE
     * can arrive just after the task snapshot and would otherwise be counted a
     * second time by this fallback. */
    if((dev->hcan->Instance == NULL) && !have_busoff_event &&
       ((err & HAL_CAN_ERROR_BOF) != 0u) &&
       !data->can_recover_pending && !dev->tx_recovery_pending)
    {
        canbus_record_busoff_event(dev, data, now, (uint8_t)data->state);
        taskENTER_CRITICAL();
        busoff_event_sequence = dev->busoff_event_sequence;
        new_busoff_events =
            busoff_event_sequence - dev->busoff_event_consumed_sequence;
        busoff_event_tick = dev->busoff_event_tick;
        busoff_event_state = dev->busoff_event_state;
        dev->busoff_event_consumed_sequence = busoff_event_sequence;
        dev->busoff_event_pending = false;
        taskEXIT_CRITICAL();
        have_busoff_event = (new_busoff_events != 0u);
    }
#endif

    if(tx_irq_mask_new != 0u)
    {
        canbus_add_u32_sat(&data->can_error_count, tx_irq_mask_new);
        data->can_last_error_tick = now;
        data->canbus_fault = true;
    }

    if(tx_hal_load_new != 0u)
    {
        had_new_hal_load_error = true;
        canbus_add_u32_sat(&data->can_error_count, tx_hal_load_new);
        data->can_last_error_tick = now;
    }

    if(had_new_critical_load_error)
    {
        /* Critical/protected HAL-load failures are transport-health relevant.
         * The source frame remains pending; a full mailbox alone never reaches
         * this path because the pump calls HAL only with a reported free slot. */
        data->canbus_fault = true;
    }

    if(had_new_detail_load_error)
    {
        /* Detail loss is observable but never grants/revokes authority by
         * itself. A genuine controller error is handled separately below. */
    }

    if((charger_normal_load_new != 0u) ||
       (charger_shutdown_load_new != 0u))
    {
        charger_t *ccs = &data->board.charger;

        ccs->tx_fail = true;
        ccs->last_tx_status = HAL_ERROR;
        ccs->disable_reason_mask |= CHARGER_DISABLE_REASON_TX_FAIL;
        canbus_add_u32_sat(&ccs->tx_fail_count,
                           charger_normal_load_new +
                           charger_shutdown_load_new);
        canbus_add_u32_sat(&ccs->shutdown_tx_fail_count,
                           charger_shutdown_load_new);
        if(charger_shutdown_load_new != 0u)
        {
            ccs->last_shutdown_status = HAL_ERROR;
        }
        data->charger_fault = true;
        data->canbus_fault = true;
        set_bms(false);
    }

    if(have_busoff_event)
    {
        /* Event multiplicity comes from the ISR sequence delta. This is the
         * critical distinction from the old single pending boolean: three BOFF
         * transitions before one 10 Hz poll are accounted as three events. */
        canbus_add_u32_sat(&data->can_busoff_count, new_busoff_events);
        canbus_add_u32_sat(&data->can_error_count, new_busoff_events);
        data->can_last_error_tick = busoff_event_tick;
        data->can_error_code = err | HAL_CAN_ERROR_BOF;
        data->can_busoff_fault = true;
        data->can_recover_pending = true;
        data->canbus_fault = true;
        data->can_authority_ready = false;
        data->can_authority_refresh_pending = false;
        taskENTER_CRITICAL();
        data->can_authority_complete_generation_baseline =
            dev->tx_scheduler.protected_required_last_complete_generation;
        taskEXIT_CRITICAL();
        canbus_tx_note_busoff(dev);
        ams_fault_log_event(AMS_FAULT_LOG_CAN_BUS_OFF,
                            (new_busoff_events > UINT16_MAX) ? UINT16_MAX :
                                (uint16_t)new_busoff_events,
                            data->can_error_code, data->can_busoff_count);

        /* The ISR already owns immediate policy latching. Repeat these task
         * state side-effects only where they carry non-GPIO diagnostic state. */
        if(busoff_event_state == (uint8_t)STATE_CHARGE)
        {
            data->charger_fault = true;
            data->board.charger.communication_fail = true;
        }
#if AMS_HIL_REPLACE_ADBMS
        data->adbms_diag_fault = true;
#endif
        /* A newly observed BOFF event and recovery settlement are distinct
         * task iterations. This preserves the event boundary and prevents a
         * host/fast-recovery path from clearing the fault in the same poll. */
        return;
    }

    /* HAL LISTENING is only software state. Hardware BOFF, outstanding TXRQ,
     * completion flags and software ownership must all settle before reset. */
    if(data->can_recover_pending ||
       (dev->tx_recovery_pending && ((err & HAL_CAN_ERROR_BOF) == 0u)))
    {
        uint32_t recovery_sequence;

        data->can_recover_pending = true;
        /* Snapshot physical event identity before any potentially long HAL or
         * mailbox-settlement observations. A bus-off arriving during those
         * checks belongs to a newer recovery transaction. */
        taskENTER_CRITICAL();
        recovery_sequence = dev->busoff_event_sequence;
        taskEXIT_CRITICAL();
        if(!canbus_hardware_busoff(dev) &&
           canbus_tx_mailboxes_settled(dev) &&
           (HAL_CAN_GetState(dev->hcan) == HAL_CAN_STATE_LISTENING))
        {
            HAL_StatusTypeDef reset_status;
            bool recovery_commit = false;

            reset_status = HAL_CAN_ResetError(dev->hcan);
            if((reset_status == HAL_OK) &&
               canbus_tx_note_recovered_for_sequence(dev, recovery_sequence))
            {
                /* Mailbox/controller settlement and the task-visible recovery
                 * commit are separate only for the HAL operation itself. The
                 * final sequence/hardware check and all fault clearing occur
                 * atomically so a newly arrived SCE event cannot be erased. */
                taskENTER_CRITICAL();
                if((dev->busoff_event_sequence == recovery_sequence) &&
                   !canbus_hardware_busoff(dev))
                {
                    data->can_authority_complete_generation_baseline =
                        dev->tx_scheduler.
                            protected_required_last_complete_generation;
                    data->can_error_code = HAL_CAN_ERROR_NONE;
                    data->can_recover_pending = false;
                    canbus_increment_u32_sat(&data->can_recover_count);
                    if(dev->tx_latched_inhibit)
                    {
                        data->can_busoff_fault = true;
                        data->canbus_fault = true;
                    }
                    else
                    {
                        data->can_busoff_fault = false;
                        data->canbus_fault = false;
                    }
                    recovery_commit = true;
                }
                taskEXIT_CRITICAL();

                if(recovery_commit)
                {
                    ams_fault_log_event(AMS_FAULT_LOG_CAN_RECOVERED, 0u,
                                        data->can_recover_count,
                                        dev->tx_latched_inhibit ? 1u : 0u);
                }
                else
                {
                    /* A newer event won the race after controller settlement.
                     * Preserve it as pending recovery and never clear its fault. */
                    data->can_last_error_tick = now;
                    data->can_busoff_fault = true;
                    data->can_recover_pending = true;
                    data->canbus_fault = true;
                    data->can_authority_ready = false;
                    canbus_tx_note_busoff(dev);
                }
            }
            else
            {
                data->can_last_error_tick = now;
                data->canbus_fault = true;
            }
        }
        else
        {
            data->canbus_fault = true;
            (void)HAL_CAN_AbortTxRequest(dev->hcan,
                CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        }
        return;
    }


    if(dev->tx_latched_inhibit)
    {
        data->can_busoff_fault = true;
        data->canbus_fault = true;
        canbus_latch_policy_fault(data, AMS_CAN_POLICY_LATCH_TX_INHIBIT);
        return;
    }

    if(had_new_hal_load_error && (err == HAL_CAN_ERROR_NONE) &&
       !dev->tx_refresh_pending && !dev->tx_recovery_pending)
    {
        /* Resume eligibility for the next publication/kick. Do not kick here:
         * limiting recovery to the next CAN-task publication prevents a
         * persistent HAL failure from spinning through repeated retries in one
         * 100 ms cycle. */
        dev->tx_suspended = false;
    }

    if(err != HAL_CAN_ERROR_NONE)
    {
        bool new_error_code = (err != data->can_error_code);
        if(new_error_code || had_new_isr_error)
        {
            /* One ISR observation may coalesce multiple controller events, but
             * it is still a new occurrence for diagnostics even when the HAL
             * error bits match the previous occurrence. */
            canbus_increment_u32_sat(&data->can_error_count);
        }
        /* Refresh the hold timer whenever the controller still presents an
         * error. Repeated identical ACK/bit errors must age from the most
         * recent observation, not the first code transition. */
        data->can_last_error_tick = now;
        data->can_error_code = err;
        data->canbus_fault = true;
        (void)HAL_CAN_ResetError(dev->hcan);
    }
    else if(data->canbus_fault &&
            ((uint32_t)(now - data->can_last_error_tick) >
             AMS_CAN_ERROR_SOFT_HOLD_MS))
    {
        data->can_error_code = HAL_CAN_ERROR_NONE;
        data->canbus_fault = false;
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
    if(rx_header.RTR != CAN_RTR_DATA)
    {
        canbus_increment_u32_sat(&dev->rx_remote_reject_count);
        canbus_increment_u32_sat(&dev->rx_filtered_count);
        return;
    }
    if(!canbus_rx_header_allowed(&rx_header))
    {
        /* Hardware filters should already reject almost all unrelated IDs. A
         * frame reaching this point with a known ID but wrong DLC is malformed;
         * all other unexpected data frames are simply filtered. */
        if(((rx_header.IDE == CAN_ID_STD) &&
            (rx_header.StdId == AMS_ECU_DIAG_FEEDBACK_CAN_ID)) ||
           ((rx_header.IDE == CAN_ID_EXT) &&
            (rx_header.ExtId == CHARGER_RX_ID)))
        {
            canbus_increment_u32_sat(&dev->rx_malformed_count);
        }
        canbus_increment_u32_sat(&dev->rx_filtered_count);
        return;
    }
    (void)canbus_enqueue_from_isr(dev, &rx_header, rx_data,
                                  CANBUS_ISR_TICK());
}
