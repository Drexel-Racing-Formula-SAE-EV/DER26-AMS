/**
 * @file canbus_task.c
 * @author Cole Bardin (cab572@drexel.edu)
 * @author Mahad Faisal (major firmware updates, 2026)
 * @brief
 * @version 0.1
 * @date 2024-03-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "tasks/canbus_task.h"
#include "ext_drivers/ams_can_logger.h"
#include "ext_drivers/canbus.h"
#include "ext_drivers/charger.h"
#include "app.h"
#include "estimator/ams_soc_ekf.h"
#include "sop/ams_power_can.h"

#include <math.h>
#include <string.h>

void canbus_task_fn(void *arg);

static StaticTask_t canbus_task_tcb;
static StackType_t canbus_task_stack[AMS_STACK_CAN_WORDS];
static TaskHandle_t canbus_task_handle = NULL;
static ams_measurement_snapshot_t canbus_measurement_cache;
static const ams_measurement_snapshot_t canbus_invalid_measurement = {0};

#define ECU_SEG_CELLS 15u
/* Hardware and the production contract both expose all 24 thermistors/SMB. */
#define ECU_SEG_TEMPS NTEMPS
#define ECU_FANS      10u
#define ECU_TEMP_INVALID_DECI_C ((uint16_t)0x8000u)
#define CAN_TX_TIMEOUT_TICKS 1u
#define CAN_MEASUREMENT_TIMEOUT_MS 500u
#define CAN_ECU_COMPACT_PROTOCOL_VERSION 1u
#define CAN_ECU_FAST_FREQ AMS_CAN_ECU_FAST_FREQ_HZ
#define CAN_ECU_FAST_PERIOD_MS AMS_CAN_ECU_FAST_PERIOD_MS
#define CAN_ECU_SLOW_DIV ((CAN_ECU_FAST_FREQ + CAN_FREQ - 1u) / CAN_FREQ)
#define CAN_CHARGER_DIV ((CHARGER_COMMAND_PERIOD_MS + CAN_ECU_FAST_PERIOD_MS - 1u) / CAN_ECU_FAST_PERIOD_MS)

#if (CAN_ECU_FAST_FREQ % CAN_FREQ) != 0
#error "CAN fast frequency must be an integer multiple of the full telemetry sweep frequency"
#endif

#define CAN_TELEMETRY_PHASE_COUNT (CAN_ECU_FAST_FREQ / CAN_FREQ)

#if CAN_TELEMETRY_PHASE_COUNT != NSMBS
#error "Current phased CAN contract requires one telemetry phase per SMB"
#endif

typedef struct
{
    const ams_measurement_snapshot_t *snapshot;
    uint32_t measurement_sequence;
    bool voltage_valid;
    bool temperature_valid;
    bool current_valid;
    float pack_voltage_V;
    float current_A;
    uint16_t min_cell_mv;
    uint16_t max_cell_mv;
    int16_t min_temp_deci_c;
    int16_t max_temp_deci_c;
    int16_t avg_temp_deci_c;
    uint16_t usable_cell_count;
    uint16_t usable_temp_count;
    uint8_t min_cell_seg;
    uint8_t min_cell_index;
    uint8_t max_cell_seg;
    uint8_t max_cell_index;
    uint8_t min_temp_seg;
    uint8_t min_temp_index;
    uint8_t max_temp_seg;
    uint8_t max_temp_index;
} can_measurement_view_t;

typedef struct
{
    state_t state;
    bool bms_state;
    bool bms_output_inhibit;
    bool hard_fault;
    bool soft_fault;
    bool canbus_fault;
    bool voltage_valid;
    bool current_valid;
    bool temperature_valid;
    bool voltage_fault;
    bool temp_fault;
    bool current_fault;
    bool charger_fault;
    bool adbms_diag_fault;
    bool task_heartbeat_fault;
    bool logger_heartbeat_fault;
    bool temp_warning;
    bool temp_fan_max;
    bool temp_charge_stop;
    bool temp_overtemp_pending;
    bool overtemp_fault;
    bool severe_overtemp_fault;
    bool fan_fault;
    bool temp_read_fault;
    uint8_t voltage_fault_reason;
    uint8_t temp_fault_reason;
    uint8_t current_fault_reason;
    int16_t max_temp_deci_c;
    int16_t min_temp_deci_c;
    int16_t avg_temp_deci_c;
    uint8_t max_fan_percent;
} can_compact_state_snapshot_t;

static uint16_t sat_u16_scaled(float x, float scale);
static int16_t sat_i16_scaled(float x, float scale);
static uint8_t sat_u8_u32(uint32_t x);

static void can_measurement_view_build(const app_data_t *data,
                                       const ams_measurement_snapshot_t *snapshot,
                                       can_measurement_view_t *view)
{
    if((data == NULL) || (view == NULL))
    {
        return;
    }

    memset(view, 0, sizeof(*view));
    view->snapshot = snapshot;
    if(snapshot == NULL)
    {
        view->voltage_valid = data->voltage_valid && !data->voltage_read_fault;
        view->temperature_valid = data->temp_valid && !data->temp_read_fault;
        view->current_valid = data->current_valid && !data->current_sensor_fault;
        view->pack_voltage_V = (data->total_voltage > 0.0f) ?
                               data->total_voltage : data->acc.total_volt;
        view->current_A = data->current;
        view->min_cell_mv = data->acc.min_voltage_mv;
        view->max_cell_mv = data->acc.max_voltage_mv;
        view->min_temp_deci_c = data->acc.min_temp_deci_c;
        view->max_temp_deci_c = data->acc.max_temp_deci_c;
        view->avg_temp_deci_c = data->acc.filtered_avg_temp_deci_c;
        view->usable_cell_count = data->voltage_usable_cell_count;
        view->usable_temp_count = data->temp_usable_sensor_count;
        view->min_cell_seg = data->min_voltage_seg;
        view->min_cell_index = data->min_voltage_cell;
        view->max_cell_seg = data->max_voltage_seg;
        view->max_cell_index = data->max_voltage_cell;
        view->min_temp_seg = data->min_temp_seg;
        view->min_temp_index = data->min_temp_sensor;
        view->max_temp_seg = data->max_temp_seg;
        view->max_temp_index = data->max_temp_sensor;
        return;
    }

    view->measurement_sequence = snapshot->sequence;
    view->voltage_valid =
        ((snapshot->validity_flags & AMS_MEAS_VALID_VOLTAGE) != 0u);
    view->temperature_valid =
        ((snapshot->validity_flags & AMS_MEAS_VALID_TEMPERATURE) != 0u);
    view->current_valid =
        ((snapshot->validity_flags & AMS_MEAS_VALID_CURRENT) != 0u) &&
        snapshot->current.valid;
    view->current_A = snapshot->current.average_A;

    uint32_t pack_mv = 0u;
    int64_t temp_sum = 0;
    bool have_cell = false;
    bool have_temp = false;
    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        for(uint8_t cell = 0u; cell < NCELLS; cell++)
        {
            if((snapshot->cell_usable_mask[seg] & (uint16_t)(1u << cell)) == 0u)
            {
                continue;
            }
            uint16_t mv = snapshot->cell_mv[seg][cell];
            pack_mv += mv;
            if(!have_cell || (mv < view->min_cell_mv))
            {
                view->min_cell_mv = mv;
                view->min_cell_seg = seg;
                view->min_cell_index = cell;
            }
            if(!have_cell || (mv > view->max_cell_mv))
            {
                view->max_cell_mv = mv;
                view->max_cell_seg = seg;
                view->max_cell_index = cell;
            }
            have_cell = true;
            if(view->usable_cell_count != UINT16_MAX)
            {
                view->usable_cell_count++;
            }
        }

        for(uint8_t sensor = 0u; sensor < NTEMPS; sensor++)
        {
            if((snapshot->temp_usable_mask[seg] & (1UL << sensor)) == 0u)
            {
                continue;
            }
            int16_t temperature = snapshot->temp_deci_c[seg][sensor];
            if(!have_temp || (temperature < view->min_temp_deci_c))
            {
                view->min_temp_deci_c = temperature;
                view->min_temp_seg = seg;
                view->min_temp_index = sensor;
            }
            if(!have_temp || (temperature > view->max_temp_deci_c))
            {
                view->max_temp_deci_c = temperature;
                view->max_temp_seg = seg;
                view->max_temp_index = sensor;
            }
            temp_sum += (int64_t)temperature;
            have_temp = true;
            if(view->usable_temp_count != UINT16_MAX)
            {
                view->usable_temp_count++;
            }
        }
    }

    view->pack_voltage_V = (float)pack_mv / 1000.0f;
    if(view->usable_temp_count > 0u)
    {
        view->avg_temp_deci_c =
            (int16_t)(temp_sum / (int64_t)view->usable_temp_count);
    }
    view->voltage_valid = view->voltage_valid && have_cell;
    view->temperature_valid = view->temperature_valid && have_temp;
}

static HAL_StatusTypeDef canbus_wait_tx_mailbox(canbus_device_t *canbus)
{
    if((canbus == NULL) || (canbus->hcan == NULL))
    {
        return HAL_ERROR;
    }

    uint32_t start = osKernelGetTickCount();

    while(HAL_CAN_GetTxMailboxesFreeLevel(canbus->hcan) == 0u)
    {
        if((osKernelGetTickCount() - start) >= CAN_TX_TIMEOUT_TICKS)
        {
            return HAL_TIMEOUT;
        }
        osDelay(1u);
    }

    return HAL_OK;
}

static HAL_StatusTypeDef canbus_send(canbus_device_t *canbus,
                                     uint32_t ide,
                                     uint32_t id,
                                     const uint8_t payload[8])
{
    if((canbus == NULL) || (canbus->hcan == NULL) || (payload == NULL))
    {
        return HAL_ERROR;
    }

    CAN_TxHeaderTypeDef *tx_header = &canbus->tx_header;

    tx_header->IDE = ide;
    tx_header->RTR = CAN_RTR_DATA;
    tx_header->DLC = DATALEN;

    if(ide == CAN_ID_EXT)
    {
        tx_header->ExtId = id;
    }
    else
    {
        tx_header->StdId = id;
    }

    if(canbus_wait_tx_mailbox(canbus) != HAL_OK)
    {
        return HAL_TIMEOUT;
    }

    return HAL_CAN_AddTxMessage(canbus->hcan, tx_header, (uint8_t *)payload, &canbus->tx_mailbox);
}

static HAL_StatusTypeDef send_ams_packet(canbus_device_t *canbus,
                                         uint16_t packet,
                                         uint16_t word0,
                                         uint16_t word1,
                                         uint16_t word2)
{
    uint8_t can_data[8] = {0};

    can_data[0] = TO_MSB16(packet);
    can_data[1] = TO_LSB16(packet);
    can_data[2] = TO_MSB16(word0);
    can_data[3] = TO_LSB16(word0);
    can_data[4] = TO_MSB16(word1);
    can_data[5] = TO_LSB16(word1);
    can_data[6] = TO_MSB16(word2);
    can_data[7] = TO_LSB16(word2);

    return canbus_send(canbus, CAN_ID_STD, ECU_CANBUS_ID, can_data);
}

static uint16_t cell_mv_for_ecu(const app_data_t *data, uint8_t seg, uint8_t cell)
{
    if((data == NULL) ||
       (seg >= accumulator_configured_smb_count(&data->acc)) ||
       (cell >= NCELLS))
    {
        return 0u;
    }

    return accumulator_cell_voltage_mv(&data->acc, seg, cell);
}

static uint16_t cell_mv_for_view(const app_data_t *data,
                                 const can_measurement_view_t *view,
                                 uint8_t seg,
                                 uint8_t cell)
{
    if((view != NULL) && (view->snapshot != NULL))
    {
        if((seg >= NSMBS) || (cell >= NCELLS) ||
           ((view->snapshot->cell_usable_mask[seg] &
             (uint16_t)(1u << cell)) == 0u))
        {
            return 0u;
        }
        return view->snapshot->cell_mv[seg][cell];
    }
    return cell_mv_for_ecu(data, seg, cell);
}

static uint16_t temp_deci_c_for_ecu(const app_data_t *data, uint8_t seg, uint8_t sensor)
{
    if((data == NULL) ||
       (seg >= accumulator_configured_smb_count(&data->acc)) ||
       (sensor >= NTEMPS))
    {
        return ECU_TEMP_INVALID_DECI_C;
    }

    if(!accumulator_temp_sensor_usable(&data->acc, seg, sensor))
    {
        return ECU_TEMP_INVALID_DECI_C;
    }

    return (uint16_t)accumulator_temp_deci_c(&data->acc, seg, sensor);
}

static uint16_t temp_deci_c_for_view(const app_data_t *data,
                                     const can_measurement_view_t *view,
                                     uint8_t seg,
                                     uint8_t sensor)
{
    if((view != NULL) && (view->snapshot != NULL))
    {
        if((seg >= NSMBS) || (sensor >= NTEMPS) ||
           ((view->snapshot->temp_usable_mask[seg] &
             (1UL << sensor)) == 0u))
        {
            return ECU_TEMP_INVALID_DECI_C;
        }
        return (uint16_t)view->snapshot->temp_deci_c[seg][sensor];
    }
    return temp_deci_c_for_ecu(data, seg, sensor);
}

static uint16_t fan_percent_for_ecu(const app_data_t *data, uint8_t fan)
{
    if((data == NULL) || (fan >= NFANS))
    {
        return 0u;
    }

    return sat_u16_scaled(data->board.fans[fan].duty_cycle, 10.0f);
}

static void canbus_saturating_increment(uint32_t *value)
{
    if((value != NULL) && (*value != UINT32_MAX))
    {
        (*value)++;
    }
}

static void canbus_record_task_tx_status(app_data_t *data, HAL_StatusTypeDef ret)
{
    if(data == NULL)
    {
        return;
    }

    uint32_t now = osKernelGetTickCount();

    if(ret != HAL_OK)
    {
        data->canbus_fault = true;
        canbus_saturating_increment(&data->can_error_count);
        data->can_last_error_tick = now;
        if(data->can_error_code == HAL_CAN_ERROR_NONE)
        {
            data->can_error_code = HAL_CAN_ERROR_TIMEOUT;
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
}

static uint8_t ecu_bool_bit(bool value, uint8_t bit)
{
    return value ? (uint8_t)(1u << bit) : 0u;
}

static void ecu_put_u16(uint8_t payload[8], uint8_t offset, uint16_t value)
{
    payload[offset] = TO_MSB16(value);
    payload[(uint8_t)(offset + 1u)] = TO_LSB16(value);
}

static void ecu_put_i16(uint8_t payload[8], uint8_t offset, int16_t value)
{
    ecu_put_u16(payload, offset, (uint16_t)value);
}

static uint16_t ecu_pack_voltage_deci_v(const app_data_t *data)
{
    float pack_v = 0.0f;

    if(data == NULL)
    {
        return 0u;
    }

    pack_v = (data->total_voltage > 0.0f) ? data->total_voltage : data->acc.total_volt;
    return sat_u16_scaled(pack_v, 10.0f);
}

static uint8_t ecu_max_fan_percent(const app_data_t *data)
{
    float max_duty = 0.0f;

    if(data == NULL)
    {
        return 0u;
    }

    for(uint8_t fan = 0u; fan < NFANS; fan++)
    {
        if(isfinite(data->board.fans[fan].duty_cycle) &&
           (data->board.fans[fan].duty_cycle > max_duty))
        {
            max_duty = data->board.fans[fan].duty_cycle;
        }
    }

    if(max_duty >= 100.0f)
    {
        return 100u;
    }

    return (uint8_t)(max_duty + 0.5f);
}

/* Freeze every non-measurement field used by the compact ECU bundle.  Cell,
 * temperature, and current values already come from the immutable measurement
 * store; this short snapshot prevents supervisor or fan updates between the
 * four CAN frames from producing a contradictory bundle. */
static bool can_compact_state_snapshot_capture(
    const app_data_t *data,
    can_compact_state_snapshot_t *snapshot)
{
    if((data == NULL) || (snapshot == NULL))
    {
        return false;
    }

    taskENTER_CRITICAL();
    snapshot->state = data->state;
    snapshot->bms_state = data->bms_state;
    snapshot->bms_output_inhibit = data->bms_output_inhibit;
    snapshot->hard_fault = data->hard_fault;
    snapshot->soft_fault = data->soft_fault;
    snapshot->canbus_fault = data->canbus_fault;
    snapshot->voltage_valid = data->voltage_valid;
    snapshot->current_valid = data->current_valid;
    snapshot->temperature_valid = data->temp_valid;
    snapshot->voltage_fault = data->voltage_fault;
    snapshot->temp_fault = data->temp_fault;
    snapshot->current_fault = data->current_fault;
    snapshot->charger_fault = data->charger_fault;
    snapshot->adbms_diag_fault = data->adbms_diag_fault;
    snapshot->task_heartbeat_fault = data->task_heartbeat_fault;
    snapshot->logger_heartbeat_fault = data->logger_heartbeat_fault;
    snapshot->temp_warning = data->temp_warning;
    snapshot->temp_fan_max = data->temp_fan_max;
    snapshot->temp_charge_stop = data->temp_charge_stop;
    snapshot->temp_overtemp_pending = data->temp_overtemp_pending;
    snapshot->overtemp_fault = data->overtemp_fault;
    snapshot->severe_overtemp_fault = data->severe_overtemp_fault;
    snapshot->fan_fault = data->fan_fault;
    snapshot->temp_read_fault = data->temp_read_fault;
    snapshot->voltage_fault_reason = (uint8_t)data->voltage_fault_reason;
    snapshot->temp_fault_reason = (uint8_t)data->temp_fault_reason;
    snapshot->current_fault_reason = (uint8_t)data->current_fault_reason;
    snapshot->max_temp_deci_c = data->acc.max_temp_deci_c;
    snapshot->min_temp_deci_c = data->acc.min_temp_deci_c;
    snapshot->avg_temp_deci_c = data->acc.filtered_avg_temp_deci_c;
    snapshot->max_fan_percent = ecu_max_fan_percent(data);
    taskEXIT_CRITICAL();

    return true;
}

static HAL_StatusTypeDef send_ecu_compact_status(canbus_device_t *canbus,
                                                  const can_compact_state_snapshot_t *state,
                                                  const can_measurement_view_t *view,
                                                  uint8_t sequence)
{
    if((canbus == NULL) || (state == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t payload[8] = {0};
    payload[0] = CAN_ECU_COMPACT_PROTOCOL_VERSION;
    payload[1] = sequence;
    payload[2] = (uint8_t)state->state;
    payload[3] = ecu_bool_bit(state->bms_state, 0u) |
                 ecu_bool_bit(state->bms_output_inhibit, 1u) |
                 ecu_bool_bit(state->hard_fault, 2u) |
                 ecu_bool_bit(state->soft_fault, 3u) |
                 ecu_bool_bit((view != NULL) ? view->voltage_valid : state->voltage_valid, 4u) |
                 ecu_bool_bit((view != NULL) ? view->current_valid : state->current_valid, 5u) |
                 ecu_bool_bit((view != NULL) ? view->temperature_valid : state->temperature_valid, 6u) |
                 ecu_bool_bit(state->canbus_fault, 7u);
    payload[4] = ecu_bool_bit(state->voltage_fault, 0u) |
                 ecu_bool_bit(state->temp_fault, 1u) |
                 ecu_bool_bit(state->current_fault, 2u) |
                 /*
                  * Bit 3 is reserved until AMS firmware actually decodes the
                  * IMD input. During staged bench testing, IMD supervision is
                  * offloaded to the hardwired shutdown path, so do not report
                  * a firmware-validated IMD_OK bit to the ECU.
                  */
                 0u |
                 ecu_bool_bit(state->charger_fault, 4u) |
                 ecu_bool_bit(state->adbms_diag_fault, 5u) |
                 ecu_bool_bit(state->task_heartbeat_fault, 6u) |
                 ecu_bool_bit(state->logger_heartbeat_fault, 7u);
    payload[5] = state->voltage_fault_reason;
    payload[6] = state->temp_fault_reason;
    payload[7] = state->current_fault_reason;

    return canbus_send(canbus, CAN_ID_STD, AMS_ECU_CAN_ID_STATUS, payload);
}

static HAL_StatusTypeDef send_ecu_compact_electrical(canbus_device_t *canbus,
                                                      const app_data_t *data,
                                                      const can_measurement_view_t *view)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t payload[8] = {0};
    ecu_put_u16(payload, 0u,
                (view != NULL) ? sat_u16_scaled(view->pack_voltage_V, 10.0f) :
                                 ecu_pack_voltage_deci_v(data));
    ecu_put_i16(payload, 2u,
                sat_i16_scaled((view != NULL) ? view->current_A : data->current,
                               10.0f));
    ecu_put_u16(payload, 4u,
                (view != NULL) ? view->min_cell_mv : data->acc.min_voltage_mv);
    ecu_put_u16(payload, 6u,
                (view != NULL) ? view->max_cell_mv : data->acc.max_voltage_mv);

    return canbus_send(canbus, CAN_ID_STD, AMS_ECU_CAN_ID_ELECTRICAL, payload);
}

static HAL_StatusTypeDef send_ecu_compact_thermal(canbus_device_t *canbus,
                                                   const can_compact_state_snapshot_t *state,
                                                   const can_measurement_view_t *view)
{
    if((canbus == NULL) || (state == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t payload[8] = {0};
    bool temperature_valid = (view != NULL) ? view->temperature_valid : state->temperature_valid;
    ecu_put_i16(payload, 0u, temperature_valid ?
                ((view != NULL) ? view->max_temp_deci_c : state->max_temp_deci_c) :
                (int16_t)ECU_TEMP_INVALID_DECI_C);
    ecu_put_i16(payload, 2u, temperature_valid ?
                ((view != NULL) ? view->min_temp_deci_c : state->min_temp_deci_c) :
                (int16_t)ECU_TEMP_INVALID_DECI_C);
    ecu_put_i16(payload, 4u, temperature_valid ?
                ((view != NULL) ? view->avg_temp_deci_c : state->avg_temp_deci_c) :
                (int16_t)ECU_TEMP_INVALID_DECI_C);
    payload[6] = state->max_fan_percent;
    payload[7] = ecu_bool_bit(state->temp_warning, 0u) |
                 ecu_bool_bit(state->temp_fan_max, 1u) |
                 ecu_bool_bit(state->temp_charge_stop, 2u) |
                 ecu_bool_bit(state->temp_overtemp_pending, 3u) |
                 ecu_bool_bit(state->overtemp_fault, 4u) |
                 ecu_bool_bit(state->severe_overtemp_fault, 5u) |
                 ecu_bool_bit(state->fan_fault, 6u) |
                 ecu_bool_bit(!temperature_valid || state->temp_read_fault, 7u);

    return canbus_send(canbus, CAN_ID_STD, AMS_ECU_CAN_ID_THERMAL, payload);
}

static HAL_StatusTypeDef send_ecu_compact_health(canbus_device_t *canbus,
                                                  const app_data_t *data,
                                                  const can_measurement_view_t *view)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t payload[8] = {0};
    payload[0] = (view != NULL) ? view->max_cell_seg : data->max_voltage_seg;
    payload[1] = (view != NULL) ? view->max_cell_index : data->max_voltage_cell;
    payload[2] = (view != NULL) ? view->min_cell_seg : data->min_voltage_seg;
    payload[3] = (view != NULL) ? view->min_cell_index : data->min_voltage_cell;
    payload[4] = (view != NULL) ? view->max_temp_seg : data->max_temp_seg;
    payload[5] = (view != NULL) ? view->max_temp_index : data->max_temp_sensor;
    payload[6] = sat_u8_u32((view != NULL) ? view->usable_cell_count :
                                             data->voltage_usable_cell_count);
    payload[7] = sat_u8_u32((view != NULL) ? view->usable_temp_count :
                                             data->temp_usable_sensor_count);

    return canbus_send(canbus, CAN_ID_STD, AMS_ECU_CAN_ID_HEALTH, payload);
}

static HAL_StatusTypeDef send_ecu_compact_telemetry(canbus_device_t *canbus,
                                                     const app_data_t *data,
                                                     const can_measurement_view_t *view,
                                                     uint8_t sequence)
{
    HAL_StatusTypeDef ret = HAL_OK;
    can_compact_state_snapshot_t state;

    if(!can_compact_state_snapshot_capture(data, &state))
    {
        return HAL_ERROR;
    }

    ret |= send_ecu_compact_status(canbus, &state, view, sequence);
    ret |= send_ecu_compact_electrical(canbus, data, view);
    ret |= send_ecu_compact_thermal(canbus, &state, view);
    ret |= send_ecu_compact_health(canbus, data, view);

    return ret;
}

static HAL_StatusTypeDef send_ecu_power_bundle(canbus_device_t *canbus,
                                               const app_data_t *data,
                                               uint8_t sequence,
                                               uint32_t now_ms)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    ams_power_can_snapshot_t snapshot;
    taskENTER_CRITICAL();
    snapshot = data->power_can_snapshot;
    taskEXIT_CRITICAL();

    uint8_t payload[8];
    HAL_StatusTypeDef status = HAL_OK;
    ams_power_can_encode_dcl(&snapshot, sequence, now_ms, payload);
    status |= canbus_send(canbus, CAN_ID_STD, AMS_POWER_CAN_DCL_ID, payload);
    ams_power_can_encode_ccl(&snapshot, sequence, now_ms, payload);
    status |= canbus_send(canbus, CAN_ID_STD, AMS_POWER_CAN_CCL_ID, payload);
    ams_power_can_encode_soh(&snapshot, sequence, now_ms, payload);
    status |= canbus_send(canbus, CAN_ID_STD, AMS_POWER_CAN_SOH_ID, payload);
    ams_power_can_encode_envelope(&snapshot, sequence, now_ms, payload);
    status |= canbus_send(canbus, CAN_ID_STD, AMS_POWER_CAN_ENVELOPE_ID,
                          payload);
    ams_power_can_encode_strategy(&snapshot, sequence, now_ms, payload);
    /* Strategy status is advisory. Its loss must not invalidate or reclassify
     * the atomic four-frame fail-zero power bundle. */
    (void)canbus_send(canbus, CAN_ID_STD, AMS_POWER_CAN_STRATEGY_ID, payload);
    ams_power_can_encode_bindings(&snapshot, sequence, now_ms, payload);
    /* Per-horizon binding metadata is also advisory.  The scalar DCL/CCL
     * frames remain the only torque-authority inputs. */
    (void)canbus_send(canbus, CAN_ID_STD, AMS_POWER_CAN_BINDINGS_ID, payload);
    return status;
}


static HAL_StatusTypeDef send_ecu_ams_status_view(
    canbus_device_t *canbus,
    const app_data_t *data,
    const can_measurement_view_t *view)
{
    HAL_StatusTypeDef ret = HAL_OK;

    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    ret |= send_ams_packet(canbus,
                           0u,
                           (uint16_t)data->state,
                           /* Legacy field: AIR_CONTROL_MCU control-net sense,
                            * not physical contactor auxiliary feedback. */
                           (uint16_t)data->air_state,
                           (uint16_t)sat_i16_scaled(
                               (view != NULL) ? view->current_A : data->current,
                               10.0f));

    ret |= send_ams_packet(canbus,
                           1u,
                           (uint16_t)data->imd_ok,
                           (uint16_t)data->imd_status,
                           (uint16_t)sat_i16_scaled(data->board.imd.duty, 10.0f));

    ret |= send_ams_packet(canbus,
                           2u,
                           ((view != NULL) ? view->temperature_valid :
                              (data->temp_valid && isfinite(data->max_temp))) ?
                               (uint16_t)((view != NULL) ?
                                   view->max_temp_deci_c :
                                   sat_i16_scaled(data->max_temp, 10.0f)) :
                               ECU_TEMP_INVALID_DECI_C,
                           (view != NULL) ? view->min_cell_mv :
                               sat_u16_scaled(data->min_voltage, 1000.0f),
                           (view != NULL) ? view->max_cell_mv :
                               sat_u16_scaled(data->max_voltage, 1000.0f));

    return ret;
}

/* Retain the legacy test/helper entry point. The production phased task calls
 * the view-aware function above with its immutable measurement epoch. */
static HAL_StatusTypeDef send_ecu_ams_status(canbus_device_t *canbus,
                                              const app_data_t *data)
{
    return send_ecu_ams_status_view(canbus, data, NULL);
}

static HAL_StatusTypeDef send_ecu_ams_voltages(canbus_device_t *canbus, const app_data_t *data)
{
    HAL_StatusTypeDef ret = HAL_OK;

    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        for(uint8_t packet = 0u; packet < 5u; packet++)
        {
            uint8_t cell = (uint8_t)(packet * 3u);
            ret |= send_ams_packet(canbus,
                                   (uint16_t)(3u + (seg * 5u) + packet),
                                   cell_mv_for_ecu(data, seg, cell),
                                   cell_mv_for_ecu(data, seg, (uint8_t)(cell + 1u)),
                                   (cell + 2u < ECU_SEG_CELLS) ? cell_mv_for_ecu(data, seg, (uint8_t)(cell + 2u)) : 0u);
        }
    }

    return ret;
}

static HAL_StatusTypeDef send_ecu_ams_temps(canbus_device_t *canbus, const app_data_t *data)
{
    HAL_StatusTypeDef ret = HAL_OK;

    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        for(uint8_t packet = 0u; packet < 8u; packet++)
        {
            uint8_t sensor = (uint8_t)(packet * 3u);
            ret |= send_ams_packet(canbus,
                                   (uint16_t)(28u + (seg * 8u) + packet),
                                   temp_deci_c_for_ecu(data, seg, sensor),
                                   temp_deci_c_for_ecu(data, seg, (uint8_t)(sensor + 1u)),
                                   (sensor + 2u < ECU_SEG_TEMPS) ? temp_deci_c_for_ecu(data, seg, (uint8_t)(sensor + 2u)) : 0u);
        }
    }

    return ret;
}

static HAL_StatusTypeDef send_ecu_ams_fans(canbus_device_t *canbus, const app_data_t *data)
{
    HAL_StatusTypeDef ret = HAL_OK;

    for(uint8_t packet = 0u; packet < 4u; packet++)
    {
        uint8_t fan = (uint8_t)(packet * 3u);
        ret |= send_ams_packet(canbus,
                               (uint16_t)(68u + packet),
                               fan_percent_for_ecu(data, fan),
                               fan_percent_for_ecu(data, (uint8_t)(fan + 1u)),
                               (fan + 2u < ECU_FANS) ? fan_percent_for_ecu(data, (uint8_t)(fan + 2u)) : 0u);
    }

    return ret;
}

static HAL_StatusTypeDef send_ecu_ams_phase(canbus_device_t *canbus,
                                             const app_data_t *data,
                                             const can_measurement_view_t *view,
                                             uint8_t phase)
{
    if((canbus == NULL) || (data == NULL) || (phase >= NSMBS))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret = HAL_OK;
    if(phase == 0u)
    {
        ret |= send_ecu_ams_status_view(canbus, data, view);
        ret |= send_ecu_ams_fans(canbus, data);
    }

    for(uint8_t packet = 0u; packet < 5u; packet++)
    {
        uint8_t cell = (uint8_t)(packet * 3u);
        ret |= send_ams_packet(
            canbus,
            (uint16_t)(3u + (phase * 5u) + packet),
            cell_mv_for_view(data, view, phase, cell),
            cell_mv_for_view(data, view, phase, (uint8_t)(cell + 1u)),
            cell_mv_for_view(data, view, phase, (uint8_t)(cell + 2u)));
    }

    for(uint8_t packet = 0u; packet < 8u; packet++)
    {
        uint8_t sensor = (uint8_t)(packet * 3u);
        ret |= send_ams_packet(
            canbus,
            (uint16_t)(28u + (phase * 8u) + packet),
            temp_deci_c_for_view(data, view, phase, sensor),
            temp_deci_c_for_view(data, view, phase, (uint8_t)(sensor + 1u)),
            (sensor + 2u < ECU_SEG_TEMPS) ?
                temp_deci_c_for_view(data, view, phase, (uint8_t)(sensor + 2u)) :
                0u);
    }

    return ret;
}

static uint16_t sat_u16_scaled(float x, float scale)
{
    if(!isfinite(x) || x <= 0.0f)
    {
        return 0u;
    }

    x *= scale;
    if(x >= 65535.0f)
    {
        return 65535u;
    }

    return (uint16_t)(x + 0.5f);
}

static int16_t sat_i16_scaled(float x, float scale)
{
    if(!isfinite(x))
    {
        return 0;
    }

    x *= scale;
    if(x >= 32767.0f)
    {
        return INT16_MAX;
    }
    if(x <= -32768.0f)
    {
        return INT16_MIN;
    }

    return (int16_t)((x >= 0.0f) ? (x + 0.5f) : (x - 0.5f));
}

static uint8_t sat_u8_u32(uint32_t x)
{
    return (x > 255u) ? 255u : (uint8_t)x;
}

static uint16_t sat_u16_u32(uint32_t x)
{
    return (x > 65535u) ? 65535u : (uint16_t)x;
}

static uint16_t logger_elapsed_deciseconds(uint32_t now, uint32_t then)
{
    if(then == 0u)
    {
        return 0xFFFFu;
    }

    return sat_u16_u32((now - then) / 100u);
}

static uint8_t logger_count_bits16(uint16_t x)
{
    uint8_t count = 0u;

    while(x != 0u)
    {
        count = (uint8_t)(count + (uint8_t)(x & 1u));
        x >>= 1u;
    }

    return count;
}

static uint8_t logger_count_bits32(uint32_t x)
{
    uint8_t count = 0u;

    while(x != 0u)
    {
        count = (uint8_t)(count + (uint8_t)(x & 1u));
        x >>= 1u;
    }

    return count;
}

static uint8_t logger_bool_bit(bool value, uint8_t bit)
{
    return value ? (uint8_t)(1u << bit) : 0u;
}

static void logger_put_u16(uint8_t payload[8], uint8_t offset, uint16_t value)
{
    payload[offset] = TO_MSB16(value);
    payload[(uint8_t)(offset + 1u)] = TO_LSB16(value);
}

static void logger_put_i16(uint8_t payload[8], uint8_t offset, int16_t value)
{
    logger_put_u16(payload, offset, (uint16_t)value);
}

static void logger_put_u24(uint8_t payload[8], uint8_t offset, uint32_t value)
{
    value &= 0x00FFFFFFu;
    payload[offset] = (uint8_t)((value >> 16u) & 0xFFu);
    payload[(uint8_t)(offset + 1u)] = (uint8_t)((value >> 8u) & 0xFFu);
    payload[(uint8_t)(offset + 2u)] = (uint8_t)(value & 0xFFu);
}

static void logger_put_u32(uint8_t payload[8], uint8_t offset, uint32_t value)
{
    payload[offset] = (uint8_t)((value >> 24u) & 0xFFu);
    payload[(uint8_t)(offset + 1u)] = (uint8_t)((value >> 16u) & 0xFFu);
    payload[(uint8_t)(offset + 2u)] = (uint8_t)((value >> 8u) & 0xFFu);
    payload[(uint8_t)(offset + 3u)] = (uint8_t)(value & 0xFFu);
}

static HAL_StatusTypeDef send_logger_frame(canbus_device_t *canbus, uint32_t id, const uint8_t payload[8])
{
    return canbus_send(canbus, CAN_ID_STD, id, payload);
}

static uint16_t temp_deci_c_for_logger(const app_data_t *data, uint8_t seg, uint8_t sensor)
{
    if((data == NULL) ||
       (seg >= accumulator_configured_smb_count(&data->acc)) ||
       (sensor >= NTEMPS) ||
       !accumulator_temp_sensor_usable(&data->acc, seg, sensor))
    {
        return AMS_LOGGER_TEMP_INVALID_DECI_C;
    }

    return (uint16_t)accumulator_temp_deci_c(&data->acc, seg, sensor);
}

static uint16_t temp_deci_c_for_logger_view(const app_data_t *data,
                                            const can_measurement_view_t *view,
                                            uint8_t seg,
                                            uint8_t sensor)
{
    if((view != NULL) && (view->snapshot != NULL))
    {
        if((seg >= NSMBS) || (sensor >= NTEMPS) ||
           ((view->snapshot->temp_usable_mask[seg] &
             (1UL << sensor)) == 0u))
        {
            return AMS_LOGGER_TEMP_INVALID_DECI_C;
        }
        return (uint16_t)view->snapshot->temp_deci_c[seg][sensor];
    }
    return temp_deci_c_for_logger(data, seg, sensor);
}

static uint16_t cell_mv_for_logger(const app_data_t *data, uint8_t seg, uint8_t cell)
{
    if((data == NULL) ||
       (seg >= accumulator_configured_smb_count(&data->acc)) ||
       (cell >= NCELLS))
    {
        return 0u;
    }

    return accumulator_cell_voltage_mv(&data->acc, seg, cell);
}

static uint16_t cell_mv_for_logger_view(const app_data_t *data,
                                        const can_measurement_view_t *view,
                                        uint8_t seg,
                                        uint8_t cell)
{
    if((view != NULL) && (view->snapshot != NULL))
    {
        if((seg >= NSMBS) || (cell >= NCELLS) ||
           ((view->snapshot->cell_usable_mask[seg] &
             (uint16_t)(1u << cell)) == 0u))
        {
            return 0u;
        }
        return view->snapshot->cell_mv[seg][cell];
    }
    return cell_mv_for_logger(data, seg, cell);
}

static uint8_t logger_max_fan_decipct(const app_data_t *data)
{
    float max_duty = 0.0f;

    if(data == NULL)
    {
        return 0u;
    }

    for(uint8_t fan = 0u; fan < NFANS; fan++)
    {
        if(isfinite(data->board.fans[fan].duty_cycle) &&
           (data->board.fans[fan].duty_cycle > max_duty))
        {
            max_duty = data->board.fans[fan].duty_cycle;
        }
    }

    if(max_duty >= 100.0f)
    {
        return 100u;
    }

    return (uint8_t)(max_duty + 0.5f);
}

static bool logger_charger_hw_fault(const charger_t *ccs)
{
    if(ccs == NULL)
    {
        return true;
    }

    return (ccs->hardware_fail      ||
            ccs->overtemp_fail      ||
            ccs->input_volt_fail    ||
            ccs->voltage_sense_fail ||
            ccs->communication_fail ||
            ccs->tx_fail);
}

static uint16_t charger_disable_reasons(const app_data_t *data, bool charger_hw_fault)
{
    if(data == NULL)
    {
        return CHARGER_DISABLE_REASON_HARD_FAULT;
    }

    uint16_t reasons = CHARGER_DISABLE_REASON_NONE;

    if(charger_hw_fault)          reasons |= CHARGER_DISABLE_REASON_HW_FAULT;
    if(data->hard_fault)          reasons |= CHARGER_DISABLE_REASON_HARD_FAULT;
    if(data->voltage_fault)       reasons |= CHARGER_DISABLE_REASON_VOLTAGE_FAULT;
    if(data->charge_voltage_stop) reasons |= CHARGER_DISABLE_REASON_VOLTAGE_CHARGE_STOP;
    if(!data->voltage_valid)      reasons |= CHARGER_DISABLE_REASON_VOLTAGE_INVALID;
    if(data->temp_charge_stop)    reasons |= CHARGER_DISABLE_REASON_TEMP_CHARGE_STOP;
    if(data->temp_fault)          reasons |= CHARGER_DISABLE_REASON_TEMP_FAULT;
    if(data->current_fault)       reasons |= CHARGER_DISABLE_REASON_CURRENT_FAULT;
    if(!data->current_valid)      reasons |= CHARGER_DISABLE_REASON_CURRENT_INVALID;
    if(!data->bms_state)          reasons |= CHARGER_DISABLE_REASON_BMS_NOT_OK;
    if(data->board.charger.shutdown_pending)
                                  reasons |= CHARGER_DISABLE_REASON_STATE_EXIT;

    return reasons;
}

static bool logger_disable_charge(const app_data_t *data, bool charger_hw_fault)
{
    return charger_disable_reasons(data, charger_hw_fault) != CHARGER_DISABLE_REASON_NONE;
}

static bool charger_disable_reasons_force_bms_low(uint16_t reasons)
{
    const uint16_t safety_mask = CHARGER_DISABLE_REASON_HW_FAULT |
                                 CHARGER_DISABLE_REASON_HARD_FAULT |
                                 CHARGER_DISABLE_REASON_VOLTAGE_FAULT |
                                 CHARGER_DISABLE_REASON_VOLTAGE_INVALID |
                                 CHARGER_DISABLE_REASON_TEMP_CHARGE_STOP |
                                 CHARGER_DISABLE_REASON_TEMP_FAULT |
                                 CHARGER_DISABLE_REASON_CURRENT_FAULT |
                                 CHARGER_DISABLE_REASON_CURRENT_INVALID |
                                 CHARGER_DISABLE_REASON_TX_FAIL |
                                 CHARGER_DISABLE_REASON_STATE_EXIT;

    return (reasons & safety_mask) != 0u;
}

static HAL_StatusTypeDef canbus_send_charger_command(canbus_device_t *canbus,
                                                     uint16_t voltage_deci_v,
                                                     uint16_t current_deci_a,
                                                     uint8_t command)
{
    uint8_t can_data[8] = {0};

    if(canbus == NULL)
    {
        return HAL_ERROR;
    }

    can_data[0] = TO_MSB16(voltage_deci_v);
    can_data[1] = TO_LSB16(voltage_deci_v);
    can_data[2] = TO_MSB16(current_deci_a);
    can_data[3] = TO_LSB16(current_deci_a);
    can_data[4] = command;

    return canbus_send(canbus, CAN_ID_EXT, CCS_CANBUS_ID, can_data);
}

static HAL_StatusTypeDef send_logger_summaries(canbus_device_t *canbus,
                                               const app_data_t *data,
                                               const can_measurement_view_t *view,
                                               uint8_t sequence)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret = HAL_OK;
    const accumulator_t *acc = &data->acc;
    const charger_t *ccs = &data->board.charger;
    const bool charger_hw_fault = logger_charger_hw_fault(ccs);
    const bool disable_charge = logger_disable_charge(data, charger_hw_fault);
    const adbms6830_spi_debug_t *smb_dbg = adbms6830_spi_debug_get(&acc->smb);
    const adbms2950_spi_debug_t *apm_dbg = adbms2950_spi_debug_get(&acc->apm);

    uint8_t payload[8] = {0};
    payload[0] = AMS_LOGGER_PROTOCOL_VERSION;
    payload[1] = sequence;
    payload[2] = (uint8_t)data->state;
    payload[3] = logger_bool_bit(data->bms_state, AMS_LOGGER_HEARTBEAT_FLAG_BMS_OK) |
                 /* Legacy AIR_STATE means AIR_CONTROL_MCU control-net sense. */
                 logger_bool_bit(data->air_state, AMS_LOGGER_HEARTBEAT_FLAG_AIR_STATE) |
                 logger_bool_bit(data->imd_ok, AMS_LOGGER_HEARTBEAT_FLAG_IMD_OK) |
                 logger_bool_bit(data->hard_fault, AMS_LOGGER_HEARTBEAT_FLAG_HARD_FAULT) |
                 logger_bool_bit(data->soft_fault, AMS_LOGGER_HEARTBEAT_FLAG_SOFT_FAULT) |
                 logger_bool_bit(data->charger_fault, AMS_LOGGER_HEARTBEAT_FLAG_CHARGER_FAULT) |
                 logger_bool_bit(data->canbus_fault, AMS_LOGGER_HEARTBEAT_FLAG_CANBUS_FAULT) |
                 logger_bool_bit(data->bms_output_inhibit, AMS_LOGGER_HEARTBEAT_FLAG_BMS_OUTPUT_INHIBIT);
    payload[4] = logger_bool_bit((view != NULL) ? view->voltage_valid : data->voltage_valid, AMS_LOGGER_VALID_FLAG_VOLTAGE_VALID) |
                 logger_bool_bit((view != NULL) ? view->current_valid : data->current_valid, AMS_LOGGER_VALID_FLAG_CURRENT_VALID) |
                 logger_bool_bit((view != NULL) ? view->temperature_valid : data->temp_valid, AMS_LOGGER_VALID_FLAG_TEMP_VALID) |
                 logger_bool_bit(data->voltage_read_fault, AMS_LOGGER_VALID_FLAG_VOLTAGE_READ_FAULT) |
                 logger_bool_bit(data->temp_read_fault, AMS_LOGGER_VALID_FLAG_TEMP_READ_FAULT) |
                 logger_bool_bit(data->current_sensor_fault, AMS_LOGGER_VALID_FLAG_CURRENT_SENSOR_FAULT) |
                 logger_bool_bit(data->voltage_fault_latched, AMS_LOGGER_VALID_FLAG_VOLTAGE_LATCHED) |
                 logger_bool_bit(data->temp_fault_latched, AMS_LOGGER_VALID_FLAG_TEMP_LATCHED);
    payload[5] = logger_bool_bit(data->current_fault, AMS_LOGGER_CURRENT_FLAG_FAULT) |
                 logger_bool_bit(data->current_sensor_fault, AMS_LOGGER_CURRENT_FLAG_SENSOR_FAULT) |
                 logger_bool_bit(data->current_overcurrent_warning, AMS_LOGGER_CURRENT_FLAG_WARNING) |
                 logger_bool_bit(data->current_overcurrent_pending, AMS_LOGGER_CURRENT_FLAG_PENDING) |
                 logger_bool_bit(data->current_overcurrent_fault, AMS_LOGGER_CURRENT_FLAG_CONFIRMED) |
                 logger_bool_bit(data->current_fault_latched, AMS_LOGGER_CURRENT_FLAG_LATCHED) |
                 logger_bool_bit(data->fuse_fault, AMS_LOGGER_CURRENT_FLAG_FUSE_FAULT) |
                 logger_bool_bit(data->estimator_fault, AMS_LOGGER_CURRENT_FLAG_ESTIMATOR_FAULT);
    logger_put_u16(payload, 6u, sat_u16_u32(osKernelGetTickCount() / 1000u));
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_HEARTBEAT, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = (uint8_t)data->voltage_fault_reason;
    payload[1] = (uint8_t)data->voltage_fault_latched_reason;
    payload[2] = (uint8_t)data->temp_fault_reason;
    payload[3] = (uint8_t)data->temp_fault_pending_reason;
    payload[4] = (uint8_t)data->temp_fault_latched_reason;
    payload[5] = (uint8_t)data->current_fault_reason;
    payload[6] = (uint8_t)data->current_fault_latched_reason;
    payload[7] = (uint8_t)data->current_fault_mode;
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_FAULT_REASONS, payload);

    memset(payload, 0, sizeof(payload));
    logger_put_u16(payload, 0u,
                   sat_u16_scaled((view != NULL) ? view->pack_voltage_V :
                                  ((data->total_voltage > 0.0f) ?
                                   data->total_voltage : acc->total_volt),
                                  10.0f));
    logger_put_i16(payload, 2u,
                   sat_i16_scaled((view != NULL) ? view->current_A : data->current,
                                  10.0f));
    logger_put_u16(payload, 4u,
                   (view != NULL) ? view->min_cell_mv : acc->min_voltage_mv);
    logger_put_u16(payload, 6u,
                   (view != NULL) ? view->max_cell_mv : acc->max_voltage_mv);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_PACK_ELECTRICAL, payload);

    memset(payload, 0, sizeof(payload));
    logger_put_i16(payload, 0u,
                   (view != NULL) ? view->max_temp_deci_c : acc->max_temp_deci_c);
    logger_put_i16(payload, 2u,
                   (view != NULL) ? view->min_temp_deci_c : acc->min_temp_deci_c);
    logger_put_i16(payload, 4u,
                   (view != NULL) ? view->avg_temp_deci_c :
                   sat_i16_scaled((data->avg_temp != 0.0f) ?
                                  data->avg_temp : acc->avg_temp,
                                  10.0f));
    payload[6] = logger_max_fan_decipct(data);
    payload[7] = logger_bool_bit(data->temp_warning, 0u) |
                 logger_bool_bit(data->temp_fan_max, 1u) |
                 logger_bool_bit(data->temp_charge_stop, 2u) |
                 logger_bool_bit(data->temp_overtemp_pending, 3u) |
                 logger_bool_bit(data->overtemp_fault, 4u) |
                 logger_bool_bit(data->severe_overtemp_fault, 5u);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_FAN, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = (view != NULL) ? view->max_cell_seg : data->max_voltage_seg;
    payload[1] = (view != NULL) ? view->max_cell_index : data->max_voltage_cell;
    payload[2] = (view != NULL) ? view->min_cell_seg : data->min_voltage_seg;
    payload[3] = (view != NULL) ? view->min_cell_index : data->min_voltage_cell;
    payload[4] = sat_u8_u32((view != NULL) ? view->usable_cell_count :
                                             data->voltage_usable_cell_count);
    payload[5] = sat_u8_u32(data->voltage_updated_cell_count);
    payload[6] = sat_u8_u32(data->voltage_stale_cell_count);
    payload[7] = sat_u8_u32(data->voltage_pec_fail_cell_count);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_VOLTAGE_HEALTH, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = (view != NULL) ? view->max_temp_seg : data->max_temp_seg;
    payload[1] = (view != NULL) ? view->max_temp_index : data->max_temp_sensor;
    payload[2] = (view != NULL) ? view->min_temp_seg : data->min_temp_seg;
    payload[3] = (view != NULL) ? view->min_temp_index : data->min_temp_sensor;
    payload[4] = sat_u8_u32((view != NULL) ? view->usable_temp_count :
                                             data->temp_usable_sensor_count);
    payload[5] = sat_u8_u32(data->temp_updated_sensor_count);
    payload[6] = sat_u8_u32(data->temp_stale_sensor_count);
    payload[7] = sat_u8_u32(data->temp_invalid_sensor_count);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_HEALTH, payload);

    memset(payload, 0, sizeof(payload));
    logger_put_u16(payload, 0u, sat_u16_scaled(ccs->target_voltage, 10.0f));
    logger_put_u16(payload, 2u, sat_u16_scaled(ccs->target_current, 10.0f));
    logger_put_u16(payload, 4u, sat_u16_scaled(ccs->read_voltage, 10.0f));
    payload[6] = logger_bool_bit(ccs->hardware_fail, AMS_LOGGER_CHARGER_FLAG_HW_FAIL) |
                 logger_bool_bit(ccs->overtemp_fail, AMS_LOGGER_CHARGER_FLAG_OVERTEMP_FAIL) |
                 logger_bool_bit(ccs->input_volt_fail, AMS_LOGGER_CHARGER_FLAG_INPUT_VOLT_FAIL) |
                 logger_bool_bit(ccs->voltage_sense_fail, AMS_LOGGER_CHARGER_FLAG_VOLTAGE_SENSE_FAIL) |
                 logger_bool_bit(ccs->communication_fail || ccs->tx_fail, AMS_LOGGER_CHARGER_FLAG_COMM_FAIL) |
                 logger_bool_bit(data->temp_charge_stop, AMS_LOGGER_CHARGER_FLAG_TEMP_CHARGE_STOP) |
                 logger_bool_bit(data->charge_voltage_stop, AMS_LOGGER_CHARGER_FLAG_VOLTAGE_CHARGE_STOP) |
                 logger_bool_bit(disable_charge, AMS_LOGGER_CHARGER_FLAG_DISABLE_CHARGE);
    payload[7] = ccs->flags;
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_CHARGER, payload);

    memset(payload, 0, sizeof(payload));
    logger_put_i16(payload, 0u,
                   sat_i16_scaled((view != NULL) ? view->current_A : data->current,
                                  10.0f));
    payload[2] = (uint8_t)data->current_selected_range;
    payload[3] = (uint8_t)data->current_meas_reason;
    payload[4] = (uint8_t)data->current_fault_reason;
    payload[5] = (uint8_t)data->current_fault_latched_reason;
    logger_put_u16(payload, 6u, sat_u16_u32(data->current_fault_state.pending_ms));
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_CURRENT_DETAIL, payload);

    memset(payload, 0, sizeof(payload));
    if(smb_dbg != NULL)
    {
        payload[0] = (uint8_t)smb_dbg->last_status;
        payload[1] = (uint8_t)smb_dbg->last_xfer_status;
        payload[2] = (uint8_t)smb_dbg->last_op;
        payload[3] = sat_u8_u32(smb_dbg->error_count);
        logger_put_u16(payload, 4u, smb_dbg->last_read_pec_fail_mask);
        logger_put_u16(payload, 6u, smb_dbg->cmd_counter_mismatch_mask);
    }
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_6830_LINK, payload);

    memset(payload, 0, sizeof(payload));
    if(smb_dbg != NULL)
    {
        logger_put_u16(payload, 0u, sat_u16_u32(smb_dbg->error_count));
        logger_put_u16(payload, 2u, sat_u16_u32(smb_dbg->cmd_counter_error_count));
        logger_put_u16(payload, 4u, smb_dbg->last_read_pec_pass_mask);
        payload[6] = smb_dbg->last_cmd[0];
        payload[7] = smb_dbg->last_cmd[1];
    }
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_6830_COUNTERS, payload);

    memset(payload, 0, sizeof(payload));
    if(apm_dbg != NULL)
    {
        payload[0] = (uint8_t)apm_dbg->last_status;
        payload[1] = (uint8_t)apm_dbg->last_xfer_status;
        payload[2] = (uint8_t)apm_dbg->last_op;
        payload[3] = sat_u8_u32(apm_dbg->error_count);
        logger_put_u16(payload, 4u, apm_dbg->last_read_pec_fail_mask);
        payload[6] = apm_dbg->enabled ? 1u : 0u;
        payload[7] = (uint8_t)acc->apm.num_ics;
    }
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_2950_LINK, payload);

    memset(payload, 0, sizeof(payload));
    logger_put_u16(payload, 0u, data->heartbeat.stale_mask);
    logger_put_u16(payload, 2u, data->heartbeat.seen_mask);
    logger_put_u16(payload, 4u, data->heartbeat.safety_stale_mask);
    payload[6] = (uint8_t)(logger_bool_bit(data->task_heartbeat_fault, 0u) |
                           logger_bool_bit(data->logger_heartbeat_fault, 1u));
    payload[7] = sat_u8_u32(data->heartbeat.count[AMS_HEARTBEAT_LOGGER]);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TASK_HEALTH, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = (uint8_t)((data->can_error_code >> 24u) & 0xFFu);
    payload[1] = (uint8_t)((data->can_error_code >> 16u) & 0xFFu);
    payload[2] = (uint8_t)((data->can_error_code >> 8u) & 0xFFu);
    payload[3] = (uint8_t)(data->can_error_code & 0xFFu);
    payload[4] = sat_u8_u32(data->can_busoff_count);
    payload[5] = sat_u8_u32(data->can_error_count);
    payload[6] = sat_u8_u32(data->can_recover_count);
    payload[7] = (uint8_t)(logger_bool_bit(data->can_busoff_fault, 0u) |
                           logger_bool_bit(data->can_recover_pending, 1u));
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_CAN_DIAG, payload);

    memset(payload, 0, sizeof(payload));
    logger_put_u32(payload, 0u, data->reset_flags);
    payload[4] = sat_u8_u32(data->last_panic_reason);
    payload[5] = sat_u8_u32(data->safety_panic_count);
    payload[6] = sat_u8_u32(data->bms_output_block_count);
    payload[7] = logger_bool_bit(ams_safety_panic_active(), AMS_LOGGER_SAFETY_FLAG_PANIC_ACTIVE) |
                 logger_bool_bit(data->bms_output_inhibit, AMS_LOGGER_SAFETY_FLAG_BMS_OUTPUT_INHIBIT) |
                 logger_bool_bit(data->balance_inhibit, AMS_LOGGER_SAFETY_FLAG_BALANCE_INHIBIT) |
                 logger_bool_bit(data->bms_state, AMS_LOGGER_SAFETY_FLAG_BMS_STATE) |
                 logger_bool_bit(data->hard_fault, AMS_LOGGER_SAFETY_FLAG_HARD_FAULT) |
                 logger_bool_bit(data->task_heartbeat_fault, AMS_LOGGER_SAFETY_FLAG_TASK_HEARTBEAT_FAULT) |
                 logger_bool_bit(data->logger_heartbeat_fault, AMS_LOGGER_SAFETY_FLAG_LOGGER_HEARTBEAT_FAULT);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_SAFETY_DIAG, payload);

    memset(payload, 0, sizeof(payload));
    uint32_t now = osKernelGetTickCount();
    payload[0] = logger_bool_bit(data->watchdog_runtime_enabled, AMS_LOGGER_WATCHDOG_FLAG_RUNTIME_ENABLED) |
                 logger_bool_bit(data->watchdog_hw_started, AMS_LOGGER_WATCHDOG_FLAG_HW_STARTED) |
                 logger_bool_bit(ams_safety_watchdog_ok(data), AMS_LOGGER_WATCHDOG_FLAG_FEED_GATE_OK);
    payload[1] = sat_u8_u32(data->watchdog_last_block_reason);
    logger_put_u16(payload, 2u, sat_u16_u32(data->watchdog_feed_count));
    logger_put_u16(payload, 4u, sat_u16_u32(data->watchdog_block_count));
    logger_put_u16(payload, 6u, logger_elapsed_deciseconds(now, data->watchdog_last_feed_tick));
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_WATCHDOG_DIAG, payload);

    memset(payload, 0, sizeof(payload));
    logger_put_u16(payload, 0u, sat_u16_u32(data->adbms_scan_count));
    payload[2] = sat_u8_u32(data->adbms_status_diag_count);
    payload[3] = sat_u8_u32(data->adbms_config_diag_count);
    payload[4] = sat_u8_u32(data->adbms_open_wire_diag_count);
    payload[5] = (uint8_t)data->adbms_last_diag_status;
    payload[6] = logger_bool_bit(data->adbms_diag_fault, AMS_LOGGER_ADBMS_DIAG_FLAG_DIAG_FAULT) |
                 logger_bool_bit(data->adbms_config_fault, AMS_LOGGER_ADBMS_DIAG_FLAG_CONFIG_FAULT) |
                 logger_bool_bit(data->adbms_status_fault, AMS_LOGGER_ADBMS_DIAG_FLAG_STATUS_FAULT) |
                 logger_bool_bit(data->adbms_open_wire_fault, AMS_LOGGER_ADBMS_DIAG_FLAG_OPEN_WIRE_FAULT) |
                 logger_bool_bit(data->adbms_scan_active, AMS_LOGGER_ADBMS_DIAG_FLAG_SCAN_ACTIVE) |
                 logger_bool_bit((AMS_HIL_REPLACE_ADBMS != 0), AMS_LOGGER_ADBMS_DIAG_FLAG_HIL_REPLACE) |
                 logger_bool_bit(data->adbms_balance_write_fault, AMS_LOGGER_ADBMS_DIAG_FLAG_BALANCE_WRITE_FAULT);
    payload[7] = logger_bool_bit(data->hil.meas.fresh != 0u, AMS_LOGGER_HIL_FLAG_MEAS_FRESH) |
                 logger_bool_bit(data->hil.truth.fresh != 0u, AMS_LOGGER_HIL_FLAG_TRUTH_FRESH) |
                 logger_bool_bit(data->hil.summary.fresh != 0u, AMS_LOGGER_HIL_FLAG_SUMMARY_FRESH);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_ADBMS_DIAG, payload);

    current_sensor_t current_snapshot;
    ams_current_window_lock();
    current_snapshot = data->board.current_sensor;
    ams_current_window_unlock();
    const current_sensor_t *cur = &current_snapshot;
    memset(payload, 0, sizeof(payload));
    logger_put_u16(payload, 0u, cur->count_high);
    logger_put_u16(payload, 2u, cur->count_low);
    payload[4] = (uint8_t)data->current_selected_range;
    payload[5] = (uint8_t)data->current_meas_reason;
    payload[6] = logger_bool_bit(cur->count_high_fresh, AMS_LOGGER_CURRENT_ADC_FLAG_HIGH_FRESH) |
                 logger_bool_bit(cur->count_low_fresh, AMS_LOGGER_CURRENT_ADC_FLAG_LOW_FRESH) |
                 logger_bool_bit(cur->last_read_ok, AMS_LOGGER_CURRENT_ADC_FLAG_LAST_READ_OK) |
                 logger_bool_bit(cur->current_valid, AMS_LOGGER_CURRENT_ADC_FLAG_CURRENT_VALID) |
                 logger_bool_bit(data->current_sensor_fault, AMS_LOGGER_CURRENT_ADC_FLAG_SENSOR_FAULT) |
                 logger_bool_bit(cur->zero_calibrated, AMS_LOGGER_CURRENT_ADC_FLAG_ZERO_CALIBRATED);
    payload[7] = sat_u8_u32(cur->zero_cal_count);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_CURRENT_ADC, payload);

    memset(payload, 0, sizeof(payload));
    logger_put_i16(payload, 0u, sat_i16_scaled(ccs->read_current, 10.0f));
    logger_put_u16(payload, 2u, ccs->disable_reason_mask);
    payload[4] = (uint8_t)ccs->last_tx_status;
    payload[5] = sat_u8_u32(ccs->tx_count);
    payload[6] = sat_u8_u32(ccs->rx_count);
    payload[7] = sat_u8_u32(ccs->tx_fail_count);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_CHARGER_DETAIL, payload);

    memset(payload, 0, sizeof(payload));
    logger_put_u16(payload, 0u, sat_u16_u32(data->rtos_heap_free_bytes / 16u));
    logger_put_u16(payload, 2u, sat_u16_u32(data->rtos_heap_min_ever_free_bytes / 16u));
    logger_put_u16(payload, 4u, data->rtos_stack_warn_mask);
    payload[6] = sat_u8_u32(data->rtos_min_stack_high_water_words);
    payload[7] = logger_bool_bit(data->rtos_fault, AMS_LOGGER_RTOS_FLAG_FAULT) |
                 logger_bool_bit(data->rtos_stack_warning, AMS_LOGGER_RTOS_FLAG_STACK_WARN) |
                 logger_bool_bit(data->rtos_heap_warning, AMS_LOGGER_RTOS_FLAG_HEAP_WARN) |
                 logger_bool_bit((data->rtos_fault_flags & AMS_RTOS_FAULT_FLAG_STACK_OVERFLOW) != 0u, AMS_LOGGER_RTOS_FLAG_STACK_OVERFLOW) |
                 logger_bool_bit((data->rtos_fault_flags & AMS_RTOS_FAULT_FLAG_MALLOC_FAILED) != 0u, AMS_LOGGER_RTOS_FLAG_MALLOC_FAILED) |
                 logger_bool_bit((data->rtos_fault_flags & AMS_RTOS_FAULT_FLAG_ASSERT_FAILED) != 0u, AMS_LOGGER_RTOS_FLAG_ASSERT_FAILED);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_RTOS_DIAG, payload);

    return ret;
}

static HAL_StatusTypeDef send_logger_details(canbus_device_t *canbus, const app_data_t *data)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret = HAL_OK;
    const accumulator_t *acc = &data->acc;
    uint8_t payload[8] = {0};

    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        for(uint8_t cell = 0u; cell < NCELLS; cell = (uint8_t)(cell + 3u))
        {
            payload[0] = seg;
            payload[1] = cell;
            logger_put_u16(payload, 2u, cell_mv_for_logger(data, seg, cell));
            logger_put_u16(payload, 4u, cell_mv_for_logger(data, seg, (uint8_t)(cell + 1u)));
            logger_put_u16(payload, 6u, cell_mv_for_logger(data, seg, (uint8_t)(cell + 2u)));
            ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_CELL_DETAIL, payload);
        }
    }

    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        for(uint8_t sensor = 0u; sensor < NTEMPS; sensor = (uint8_t)(sensor + 3u))
        {
            payload[0] = seg;
            payload[1] = sensor;
            logger_put_u16(payload, 2u, temp_deci_c_for_logger(data, seg, sensor));
            logger_put_u16(payload, 4u, temp_deci_c_for_logger(data, seg, (uint8_t)(sensor + 1u)));
            logger_put_u16(payload, 6u, temp_deci_c_for_logger(data, seg, (uint8_t)(sensor + 2u)));
            ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_DETAIL, payload);
        }
    }

    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        uint16_t updated = (seg < accumulator_configured_smb_count(acc)) ? acc->updated_voltage_mask[seg] : 0u;
        uint16_t usable = (seg < accumulator_configured_smb_count(acc)) ? acc->usable_voltage_mask[seg] : 0u;
        uint16_t stale = (seg < accumulator_configured_smb_count(acc)) ? acc->stale_voltage_mask[seg] : 0u;
        uint16_t pec = (seg < accumulator_configured_smb_count(acc)) ? acc->pec_fail_voltage_mask[seg] : 0u;

        payload[0] = seg;
        logger_put_u16(payload, 1u, updated);
        logger_put_u16(payload, 3u, usable);
        logger_put_u16(payload, 5u, stale);
        payload[7] = 0u;
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_VOLTAGE_MASKS, payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = seg;
        logger_put_u16(payload, 1u, pec);
        payload[3] = logger_count_bits16(pec);
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_VOLTAGE_PEC, payload);

        uint32_t temp_updated = (seg < accumulator_configured_smb_count(acc)) ? acc->updated_temp_mask[seg] : 0u;
        uint32_t temp_usable = (seg < accumulator_configured_smb_count(acc)) ? acc->usable_temp_mask[seg] : 0u;
        uint32_t temp_stale = (seg < accumulator_configured_smb_count(acc)) ? acc->stale_temp_mask[seg] : 0u;
        uint32_t temp_invalid = (seg < accumulator_configured_smb_count(acc)) ? acc->invalid_temp_mask[seg] : 0u;

        payload[0] = seg;
        logger_put_u24(payload, 1u, temp_updated);
        logger_put_u24(payload, 4u, temp_usable);
        payload[7] = 0u;
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_MASKS_A, payload);

        payload[0] = seg;
        logger_put_u24(payload, 1u, temp_stale);
        logger_put_u24(payload, 4u, temp_invalid);
        payload[7] = 0u;
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_MASKS_B, payload);
    }

    memset(payload, 0, sizeof(payload));
    logger_put_i16(payload, 0u, sat_i16_scaled(data->temp_filtered_max, 10.0f));
    logger_put_i16(payload, 2u, sat_i16_scaled(data->temp_max_rate_c_per_s, 10.0f));
    payload[4] = logger_bool_bit(data->temp_open_sensor_count > 0u, AMS_LOGGER_TEMP_DIAG_FLAG_OPEN) |
                 logger_bool_bit(data->temp_short_sensor_count > 0u, AMS_LOGGER_TEMP_DIAG_FLAG_SHORT) |
                 logger_bool_bit(data->temp_jump_sensor_count > 0u, AMS_LOGGER_TEMP_DIAG_FLAG_JUMP) |
                 logger_bool_bit(data->temp_rate_rise_sensor_count > 0u, AMS_LOGGER_TEMP_DIAG_FLAG_RATE_RISE) |
                 logger_bool_bit(data->temp_usable_sensor_count > 0u, AMS_LOGGER_TEMP_DIAG_FLAG_FILTER_VALID);
    payload[5] = data->fan_control_reason;
    payload[6] = sat_u8_u32((uint32_t)((isfinite(data->fan_command_percent) && (data->fan_command_percent > 0.0f)) ?
                                      (data->fan_command_percent + 0.5f) : 0.0f));
    payload[7] = logger_bool_bit(data->fan_state, AMS_LOGGER_FAN_DIAG_FLAG_FAN_ON) |
                 logger_bool_bit(data->fan_fault, AMS_LOGGER_FAN_DIAG_FLAG_DRIVER_FAULT) |
                 logger_bool_bit(!data->temp_valid || data->temp_read_fault, AMS_LOGGER_FAN_DIAG_FLAG_TEMP_INVALID) |
                 logger_bool_bit(data->temp_fault, AMS_LOGGER_FAN_DIAG_FLAG_TEMP_FAULT) |
                 logger_bool_bit(data->temp_fan_max, AMS_LOGGER_FAN_DIAG_FLAG_TEMP_FAN_MAX);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_DIAG, payload);

    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        uint32_t temp_open = (seg < accumulator_configured_smb_count(acc)) ? acc->temp_open_mask[seg] : 0u;
        uint32_t temp_short = (seg < accumulator_configured_smb_count(acc)) ? acc->temp_short_mask[seg] : 0u;
        uint32_t temp_jump = (seg < accumulator_configured_smb_count(acc)) ? acc->temp_jump_mask[seg] : 0u;
        uint32_t temp_rate = (seg < accumulator_configured_smb_count(acc)) ? acc->temp_rate_rise_mask[seg] : 0u;
        uint16_t voltage_jump = (seg < accumulator_configured_smb_count(acc)) ? acc->voltage_jump_mask[seg] : 0u;
        uint16_t voltage_stuck = (seg < accumulator_configured_smb_count(acc)) ? acc->voltage_stuck_mask[seg] : 0u;

        memset(payload, 0, sizeof(payload));
        payload[0] = seg;
        logger_put_u24(payload, 1u, temp_open);
        logger_put_u24(payload, 4u, temp_short);
        payload[7] = (uint8_t)((logger_count_bits32(temp_open) & 0x0Fu) |
                               ((logger_count_bits32(temp_short) & 0x0Fu) << 4u));
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_DIAG_A, payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = seg;
        logger_put_u24(payload, 1u, temp_jump);
        logger_put_u24(payload, 4u, temp_rate);
        payload[7] = (uint8_t)((logger_count_bits32(temp_jump) & 0x0Fu) |
                               ((logger_count_bits32(temp_rate) & 0x0Fu) << 4u));
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_DIAG_B, payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = seg;
        logger_put_u16(payload, 1u, voltage_jump);
        logger_put_u16(payload, 3u, voltage_stuck);
        payload[5] = logger_count_bits16(voltage_jump);
        payload[6] = logger_count_bits16(voltage_stuck);
        payload[7] = logger_bool_bit(voltage_jump != 0u, AMS_LOGGER_VOLTAGE_DIAG_FLAG_JUMP) |
                     logger_bool_bit(voltage_stuck != 0u, AMS_LOGGER_VOLTAGE_DIAG_FLAG_STUCK);
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_VOLTAGE_DIAG, payload);
    }

    return ret;
}

static HAL_StatusTypeDef send_logger_detail_phase(
    canbus_device_t *canbus,
    const app_data_t *data,
    const can_measurement_view_t *view,
    uint8_t phase)
{
    if((canbus == NULL) || (data == NULL) || (phase >= NSMBS))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret = HAL_OK;
    const accumulator_t *acc = &data->acc;
    uint8_t payload[8] = {0};

    for(uint8_t cell = 0u; cell < NCELLS; cell = (uint8_t)(cell + 3u))
    {
        memset(payload, 0, sizeof(payload));
        payload[0] = phase;
        payload[1] = cell;
        logger_put_u16(payload, 2u,
                       cell_mv_for_logger_view(data, view, phase, cell));
        logger_put_u16(payload, 4u,
                       cell_mv_for_logger_view(data,
                                               view,
                                               phase,
                                               (uint8_t)(cell + 1u)));
        logger_put_u16(payload, 6u,
                       cell_mv_for_logger_view(data,
                                               view,
                                               phase,
                                               (uint8_t)(cell + 2u)));
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_CELL_DETAIL, payload);
    }

    for(uint8_t sensor = 0u; sensor < NTEMPS; sensor = (uint8_t)(sensor + 3u))
    {
        memset(payload, 0, sizeof(payload));
        payload[0] = phase;
        payload[1] = sensor;
        logger_put_u16(payload, 2u,
                       temp_deci_c_for_logger_view(data, view, phase, sensor));
        logger_put_u16(payload, 4u,
                       temp_deci_c_for_logger_view(data,
                                                  view,
                                                  phase,
                                                  (uint8_t)(sensor + 1u)));
        logger_put_u16(payload, 6u,
                       temp_deci_c_for_logger_view(data,
                                                  view,
                                                  phase,
                                                  (uint8_t)(sensor + 2u)));
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_DETAIL, payload);
    }

    uint16_t updated =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->updated_voltage_mask[phase] : 0u;
    uint16_t usable = (view != NULL) && (view->snapshot != NULL) ?
        view->snapshot->cell_usable_mask[phase] :
        ((phase < accumulator_configured_smb_count(acc)) ?
         acc->usable_voltage_mask[phase] : 0u);
    uint16_t stale =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->stale_voltage_mask[phase] : 0u;
    uint16_t pec =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->pec_fail_voltage_mask[phase] : 0u;

    memset(payload, 0, sizeof(payload));
    payload[0] = phase;
    logger_put_u16(payload, 1u, updated);
    logger_put_u16(payload, 3u, usable);
    logger_put_u16(payload, 5u, stale);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_VOLTAGE_MASKS, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = phase;
    logger_put_u16(payload, 1u, pec);
    payload[3] = logger_count_bits16(pec);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_VOLTAGE_PEC, payload);

    uint32_t temp_updated =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->updated_temp_mask[phase] : 0u;
    uint32_t temp_usable = (view != NULL) && (view->snapshot != NULL) ?
        view->snapshot->temp_usable_mask[phase] :
        ((phase < accumulator_configured_smb_count(acc)) ?
         acc->usable_temp_mask[phase] : 0u);
    uint32_t temp_stale =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->stale_temp_mask[phase] : 0u;
    uint32_t temp_invalid =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->invalid_temp_mask[phase] : 0u;

    memset(payload, 0, sizeof(payload));
    payload[0] = phase;
    logger_put_u24(payload, 1u, temp_updated);
    logger_put_u24(payload, 4u, temp_usable);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_MASKS_A, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = phase;
    logger_put_u24(payload, 1u, temp_stale);
    logger_put_u24(payload, 4u, temp_invalid);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_MASKS_B, payload);

    if(phase == 0u)
    {
        memset(payload, 0, sizeof(payload));
        logger_put_i16(payload, 0u,
                       sat_i16_scaled(data->temp_filtered_max, 10.0f));
        logger_put_i16(payload, 2u,
                       sat_i16_scaled(data->temp_max_rate_c_per_s, 10.0f));
        payload[4] =
            logger_bool_bit(data->temp_open_sensor_count > 0u,
                            AMS_LOGGER_TEMP_DIAG_FLAG_OPEN) |
            logger_bool_bit(data->temp_short_sensor_count > 0u,
                            AMS_LOGGER_TEMP_DIAG_FLAG_SHORT) |
            logger_bool_bit(data->temp_jump_sensor_count > 0u,
                            AMS_LOGGER_TEMP_DIAG_FLAG_JUMP) |
            logger_bool_bit(data->temp_rate_rise_sensor_count > 0u,
                            AMS_LOGGER_TEMP_DIAG_FLAG_RATE_RISE) |
            logger_bool_bit(data->temp_usable_sensor_count > 0u,
                            AMS_LOGGER_TEMP_DIAG_FLAG_FILTER_VALID);
        payload[5] = data->fan_control_reason;
        payload[6] = sat_u8_u32(
            (uint32_t)((isfinite(data->fan_command_percent) &&
                        (data->fan_command_percent > 0.0f)) ?
                       (data->fan_command_percent + 0.5f) : 0.0f));
        payload[7] =
            logger_bool_bit(data->fan_state,
                            AMS_LOGGER_FAN_DIAG_FLAG_FAN_ON) |
            logger_bool_bit(data->fan_fault,
                            AMS_LOGGER_FAN_DIAG_FLAG_DRIVER_FAULT) |
            logger_bool_bit(!data->temp_valid || data->temp_read_fault,
                            AMS_LOGGER_FAN_DIAG_FLAG_TEMP_INVALID) |
            logger_bool_bit(data->temp_fault,
                            AMS_LOGGER_FAN_DIAG_FLAG_TEMP_FAULT) |
            logger_bool_bit(data->temp_fan_max,
                            AMS_LOGGER_FAN_DIAG_FLAG_TEMP_FAN_MAX);
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_DIAG, payload);
    }

    uint32_t temp_open =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->temp_open_mask[phase] : 0u;
    uint32_t temp_short =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->temp_short_mask[phase] : 0u;
    uint32_t temp_jump =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->temp_jump_mask[phase] : 0u;
    uint32_t temp_rate =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->temp_rate_rise_mask[phase] : 0u;
    uint16_t voltage_jump =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->voltage_jump_mask[phase] : 0u;
    uint16_t voltage_stuck =
        (phase < accumulator_configured_smb_count(acc)) ?
        acc->voltage_stuck_mask[phase] : 0u;

    memset(payload, 0, sizeof(payload));
    payload[0] = phase;
    logger_put_u24(payload, 1u, temp_open);
    logger_put_u24(payload, 4u, temp_short);
    payload[7] = (uint8_t)((logger_count_bits32(temp_open) & 0x0Fu) |
                           ((logger_count_bits32(temp_short) & 0x0Fu) << 4u));
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_DIAG_A, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = phase;
    logger_put_u24(payload, 1u, temp_jump);
    logger_put_u24(payload, 4u, temp_rate);
    payload[7] = (uint8_t)((logger_count_bits32(temp_jump) & 0x0Fu) |
                           ((logger_count_bits32(temp_rate) & 0x0Fu) << 4u));
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_DIAG_B, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = phase;
    logger_put_u16(payload, 1u, voltage_jump);
    logger_put_u16(payload, 3u, voltage_stuck);
    payload[5] = logger_count_bits16(voltage_jump);
    payload[6] = logger_count_bits16(voltage_stuck);
    payload[7] =
        logger_bool_bit(voltage_jump != 0u,
                        AMS_LOGGER_VOLTAGE_DIAG_FLAG_JUMP) |
        logger_bool_bit(voltage_stuck != 0u,
                        AMS_LOGGER_VOLTAGE_DIAG_FLAG_STUCK);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_VOLTAGE_DIAG, payload);

    return ret;
}

static HAL_StatusTypeDef send_logger_phase(canbus_device_t *canbus,
                                            const app_data_t *data,
                                            const can_measurement_view_t *view,
                                            uint8_t phase,
                                            uint8_t sequence)
{
    if((canbus == NULL) || (data == NULL) ||
       (phase >= CAN_TELEMETRY_PHASE_COUNT))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t payload[8] = {0};
    payload[0] = AMS_LOGGER_SNAPSHOT_VERSION;
    payload[1] = sequence;
    payload[2] = phase;
    payload[3] = CAN_TELEMETRY_PHASE_COUNT;
    logger_put_u32(payload,
                   4u,
                   (view != NULL) ? view->measurement_sequence : 0u);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_SNAPSHOT_META, payload);

    if(phase == 0u)
    {
        ret |= send_logger_summaries(canbus, data, view, sequence);
    }
    ret |= send_logger_detail_phase(canbus, data, view, phase);
    return ret;
}

static HAL_StatusTypeDef send_logger_telemetry(canbus_device_t *canbus, const app_data_t *data)
{
    static uint8_t sequence = 0u;
    HAL_StatusTypeDef ret;

    can_measurement_view_t view;
    can_measurement_view_build(data, NULL, &view);

    ret = send_logger_summaries(canbus, data, &view, sequence);
    ret |= send_logger_details(canbus, data);
    sequence++;

    return ret;
}



static uint16_t sat_u16_from_float(float x)
{
    if(!isfinite(x) || x <= 0.0f)
    {
        return 0u;
    }
    if(x >= 65535.0f)
    {
        return 65535u;
    }
    return (uint16_t)(x + 0.5f);
}

static int16_t sat_i16_from_float(float x)
{
    if(!isfinite(x))
    {
        return 0;
    }
    if(x >= 32767.0f)
    {
        return INT16_MAX;
    }
    if(x <= -32768.0f)
    {
        return INT16_MIN;
    }
    return (int16_t)((x >= 0.0f) ? (x + 0.5f) : (x - 0.5f));
}

static HAL_StatusTypeDef send_estimator_status(canbus_device_t *canbus, const app_data_t *data)
{
    if((data == NULL) ||
       (data->estimator.enabled == 0u) ||
       (data->estimator.instance_count == 0u) ||
       (data->estimator.active_index >= data->estimator.instance_count) ||
       (data->estimator.active_index >= AMS_EKF_MAX_INSTANCES))
    {
        return HAL_OK;
    }

    const ams_ekf_instance_t *inst = &data->estimator.inst[data->estimator.active_index];
    if((inst->valid == 0u) && (data->estimator.cc_valid == 0u))
    {
        return HAL_OK;
    }

    uint8_t payload[8] = {0};
    float soc = (inst->valid != 0u) ? inst->soc : data->estimator.cc_soc;
    uint16_t soc_centi_pct = sat_u16_from_float(soc * 10000.0f);
    int16_t innov_mV = sat_i16_from_float(inst->innovation_V * 1000.0f);
    uint16_t r0_0p01_mohm = sat_u16_from_float(inst->r0_ohm * 100000.0f);

    payload[0] = data->estimator.active_index;
    payload[1] = ams_estimator_status_flags(&data->estimator);
    payload[2] = TO_MSB16(soc_centi_pct);
    payload[3] = TO_LSB16(soc_centi_pct);
    payload[4] = TO_MSB16((uint16_t)innov_mV);
    payload[5] = TO_LSB16((uint16_t)innov_mV);
    payload[6] = TO_MSB16(r0_0p01_mohm);
    payload[7] = TO_LSB16(r0_0p01_mohm);

    return canbus_send(canbus, CAN_ID_STD, AMS_ESTIMATOR_STATUS_CAN_ID, payload);
}

static void canbus_record_tx_class(uint32_t *attempt_count,
                                   uint32_t *failure_count,
                                   HAL_StatusTypeDef status)
{
    canbus_saturating_increment(attempt_count);
    if(status != HAL_OK)
    {
        canbus_saturating_increment(failure_count);
    }
}

static HAL_StatusTypeDef canbus_run_periodic_charger_command(
    app_data_t *data,
    canbus_device_t *canbus,
    charger_t *ccs,
    uint16_t *charger_div,
    bool charger_shutdown_attempted,
    uint16_t voltage10x,
    uint16_t current10x)
{
    if((data == NULL) || (canbus == NULL) || (ccs == NULL) ||
       (charger_div == NULL))
    {
        return HAL_ERROR;
    }

    if(charger_shutdown_attempted)
    {
        /* Never queue enable behind an exit-disable frame in the same task
         * iteration. A rapid service re-entry first completes the shutdown
         * burst and then waits at least one CAN cycle. */
        ccs->target_voltage = 0.0f;
        ccs->target_current = 0.0f;
        *charger_div = 0u;
        return HAL_OK;
    }

    ccs->target_voltage = CHARGE_MAX_VOLTAGE;
    ccs->target_current = CHARGE_MAX_CURRENT;
    (*charger_div)++;
    if(*charger_div < CAN_CHARGER_DIV)
    {
        return HAL_OK;
    }
    *charger_div = 0u;

    if((osKernelGetTickCount() - ccs->last_rx_tick) > CHARGER_RX_TIMEOUT_MS)
    {
        ccs->communication_fail = true;
    }

    bool charger_hw_fault = ccs->hardware_fail ||
                            ccs->overtemp_fail ||
                            ccs->input_volt_fail ||
                            ccs->voltage_sense_fail ||
                            ccs->communication_fail ||
                            ccs->tx_fail;
    uint16_t disable_reasons = charger_disable_reasons(data,
                                                       charger_hw_fault);
    bool disable_charge =
        (disable_reasons != CHARGER_DISABLE_REASON_NONE);

    ccs->disable_reason_mask = disable_reasons;
    data->charger_fault = charger_hw_fault;
    if(charger_disable_reasons_force_bms_low(disable_reasons))
    {
        set_bms(false);
    }

    HAL_StatusTypeDef status = canbus_send_charger_command(
        canbus,
        voltage10x,
        current10x,
        disable_charge ? CHARGER_CMD_DISABLE : CHARGER_CMD_ENABLE);
    canbus_record_tx_class(&data->can_tx_critical_attempt_count,
                           &data->can_tx_critical_fail_count,
                           status);
    if(status == HAL_OK)
    {
        ccs->tx_fail = false;
        ccs->last_tx_status = HAL_OK;
        canbus_saturating_increment(&ccs->tx_count);
    }
    else
    {
        ccs->tx_fail = true;
        ccs->last_tx_status = status;
        ccs->disable_reason_mask |= CHARGER_DISABLE_REASON_TX_FAIL;
        canbus_saturating_increment(&ccs->tx_fail_count);
        data->charger_fault = true;
        set_bms(false);
    }
    return status;
}

static void canbus_wait_next_period(app_data_t *data, uint32_t entry_tick)
{
    if(data != NULL)
    {
        uint32_t duration_ms = osKernelGetTickCount() - entry_tick;
        data->can_task_last_duration_ms = duration_ms;
        if(duration_ms > data->can_task_max_duration_ms)
        {
            data->can_task_max_duration_ms = duration_ms;
        }
        canbus_saturating_increment(&data->can_task_cycle_count);
        if(duration_ms > CAN_ECU_FAST_PERIOD_MS)
        {
            canbus_saturating_increment(&data->can_task_deadline_miss_count);
        }
    }

    (void)osDelayUntil(entry_tick + CAN_ECU_FAST_PERIOD_MS);
}

TaskHandle_t canbus_task_start(app_data_t *data)
{
    if(data == NULL)
    {
        return NULL;
    }

    if(canbus_task_handle == NULL)
    {
        canbus_task_handle = xTaskCreateStatic(canbus_task_fn,
                                               "CANBus Task",
                                               AMS_STACK_CAN_WORDS,
                                               (void *)data,
                                               CAN_PRIO,
                                               canbus_task_stack,
                                               &canbus_task_tcb);
    }

    return canbus_task_handle;
}

void canbus_task_fn(void *arg)
{
    app_data_t *data = (app_data_t *)arg;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }


    canbus_device_t *canbus = &data->board.canbus;
    charger_t *ccs = &data->board.charger;
    HAL_StatusTypeDef ret;
    uint32_t entry;
    uint8_t ecu_sequence = 0u;
    uint8_t telemetry_phase = 0u;
    uint8_t logger_sequence = 0u;
    uint16_t charger_div = CAN_CHARGER_DIV;

    const uint16_t voltage10x = (uint16_t)(CHARGE_MAX_VOLTAGE * 10.0f);
    const uint16_t current10x = (uint16_t)(CHARGE_MAX_CURRENT * 10.0f);

    for(;;)
    {
        entry = osKernelGetTickCount();
        ret = HAL_OK;
        bool have_measurement = ams_measurement_store_copy_latest(
            &data->measurement_store,
            &canbus_measurement_cache);
        have_measurement = have_measurement &&
            ((uint32_t)(entry - canbus_measurement_cache.publication_tick) <=
             CAN_MEASUREMENT_TIMEOUT_MS);
        can_measurement_view_t measurement_view;
        can_measurement_view_build(
            data,
            have_measurement ? &canbus_measurement_cache :
                               &canbus_invalid_measurement,
            &measurement_view);
        (void)canbus_process_rx_queue(canbus, data, CANBUS_RX_QUEUE_DEPTH);
        canbus_poll_errors(canbus, data);

        if(data->can_busoff_fault || data->can_recover_pending)
        {
            data->canbus_fault = true;
            ams_heartbeat_kick(data, AMS_HEARTBEAT_CAN, osKernelGetTickCount());
            canbus_wait_next_period(data, entry);
            continue;
        }

        /* State-exit shutdown commands take priority over telemetry and over
         * any later charger-enable command.  A transition records a request
         * generation under the RTOS critical section; only consume one frame
         * from that same generation so a concurrent newer transition cannot
         * be accidentally acknowledged by an older send completion. */
        bool charger_shutdown_attempted = false;
        if(ccs->shutdown_pending)
        {
            uint32_t shutdown_generation;
            HAL_StatusTypeDef shutdown_status;

            taskENTER_CRITICAL();
            shutdown_generation = ccs->shutdown_request_count;
            taskEXIT_CRITICAL();

            charger_shutdown_attempted = true;
            ccs->target_voltage = 0.0f;
            ccs->target_current = 0.0f;
            shutdown_status = canbus_send_charger_command(canbus,
                                                          0u,
                                                          0u,
                                                          CHARGER_CMD_DISABLE);
            canbus_record_tx_class(&data->can_tx_critical_attempt_count,
                                   &data->can_tx_critical_fail_count,
                                   shutdown_status);
            ret |= shutdown_status;

            taskENTER_CRITICAL();
            ccs->last_shutdown_status = shutdown_status;
            ccs->last_tx_status = shutdown_status;
            if(shutdown_status == HAL_OK)
            {
                ccs->tx_fail = false;
                ccs->disable_reason_mask &=
                    (uint16_t)~CHARGER_DISABLE_REASON_TX_FAIL;
                ccs->disable_reason_mask |= CHARGER_DISABLE_REASON_STATE_EXIT;
                canbus_saturating_increment(&ccs->tx_count);
                canbus_saturating_increment(&ccs->shutdown_tx_count);

                if((ccs->shutdown_request_count == shutdown_generation) &&
                   ccs->shutdown_pending)
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
            else
            {
                ccs->tx_fail = true;
                ccs->disable_reason_mask |=
                    CHARGER_DISABLE_REASON_STATE_EXIT |
                    CHARGER_DISABLE_REASON_TX_FAIL;
                canbus_saturating_increment(&ccs->tx_fail_count);
                canbus_saturating_increment(&ccs->shutdown_tx_fail_count);
                data->charger_fault = true;
            }
            taskEXIT_CRITICAL();

            if(shutdown_status != HAL_OK)
            {
                set_bms(false);
            }
        }

        /* A normal charge command is safety-relevant traffic and therefore
         * runs before compact status/detail telemetry. State-exit shutdown
         * traffic above retains the absolute first slot. */
        if(data->state == STATE_CHARGE)
        {
            ret |= canbus_run_periodic_charger_command(
                data,
                canbus,
                ccs,
                &charger_div,
                charger_shutdown_attempted,
                voltage10x,
                current10x);
        }

        const uint8_t bundle_sequence = ecu_sequence++;
        HAL_StatusTypeDef compact_status = send_ecu_compact_telemetry(
            canbus,
            data,
            &measurement_view,
            bundle_sequence);
        compact_status |= send_ecu_power_bundle(canbus,
                                                data,
                                                bundle_sequence,
                                                entry);
        canbus_record_tx_class(&data->can_tx_compact_bundle_count,
                               &data->can_tx_compact_bundle_fail_count,
                               compact_status);
        ret |= compact_status;
        bool compact_tx_ok = (compact_status == HAL_OK);
        if(!compact_tx_ok && (data->state != STATE_CHARGE))
        {
            /*
             * The compact ECU frames are the high-priority AMS heartbeat. If
             * they cannot be queued, do not spend this cycle trying to dump
             * slower full-cell/logger telemetry onto a congested or broken bus.
             *
             * Charge mode is the exception: still attempt the charger command
             * path so a TX failure can be latched as a charger fault and force
             * BMS low.
             */
            canbus_poll_errors(canbus, data);
            canbus_record_task_tx_status(data, ret);
            canbus_saturating_increment(&data->can_tx_detail_suppressed_count);
            data->canbus_fault = data->canbus_fault || data->can_busoff_fault;
            ams_heartbeat_kick(data, AMS_HEARTBEAT_CAN, osKernelGetTickCount());
            canbus_wait_next_period(data, entry);
            continue;
        }

        if(data->state != STATE_CHARGE)
        {
            HAL_StatusTypeDef detail_status = HAL_OK;

            ccs->target_voltage = 0.0f;
            ccs->target_current = 0.0f;
            ccs->read_voltage   = 0.0f;
            ccs->read_current   = 0.0f;
            ccs->communication_fail = false;
            /* A failed or incomplete state-exit shutdown burst is a blocking
             * charger fault.  Do not erase it merely because the application
             * has already changed out of charge mode. */
            data->charger_fault = ccs->shutdown_pending || ccs->tx_fail;

            detail_status |= send_ecu_ams_phase(canbus,
                                                data,
                                                &measurement_view,
                                                telemetry_phase);
            HAL_StatusTypeDef logger_status =
                send_logger_phase(canbus,
                                  data,
                                  &measurement_view,
                                  telemetry_phase,
                                  logger_sequence);
            detail_status |= logger_status;
            if(logger_status == HAL_OK)
            {
                ams_heartbeat_kick(data,
                                  AMS_HEARTBEAT_LOGGER,
                                  osKernelGetTickCount());
            }
            if(telemetry_phase == 0u)
            {
                detail_status |= send_estimator_status(canbus, data);
            }
            canbus_record_tx_class(&data->can_tx_detail_phase_count,
                                   &data->can_tx_detail_phase_fail_count,
                                   detail_status);
            ret |= detail_status;
            telemetry_phase++;
            if(telemetry_phase >= CAN_TELEMETRY_PHASE_COUNT)
            {
                telemetry_phase = 0u;
                logger_sequence++;
            }
            canbus_poll_errors(canbus, data);

            canbus_record_task_tx_status(data, ret);
            data->canbus_fault = data->canbus_fault || data->can_busoff_fault;
            ams_heartbeat_kick(data, AMS_HEARTBEAT_CAN, osKernelGetTickCount());
            canbus_wait_next_period(data, entry);
        }
        else
        {
            if(compact_tx_ok)
            {
                HAL_StatusTypeDef detail_status = HAL_OK;
                HAL_StatusTypeDef logger_status =
                    send_logger_phase(canbus,
                                      data,
                                      &measurement_view,
                                      telemetry_phase,
                                      logger_sequence);
                detail_status |= logger_status;
                if(logger_status == HAL_OK)
                {
                    ams_heartbeat_kick(data,
                                      AMS_HEARTBEAT_LOGGER,
                                      osKernelGetTickCount());
                }
                if(telemetry_phase == 0u)
                {
                    detail_status |= send_estimator_status(canbus, data);
                }
                canbus_record_tx_class(&data->can_tx_detail_phase_count,
                                       &data->can_tx_detail_phase_fail_count,
                                       detail_status);
                ret |= detail_status;

                telemetry_phase++;
                if(telemetry_phase >= CAN_TELEMETRY_PHASE_COUNT)
                {
                    telemetry_phase = 0u;
                    logger_sequence++;
                }
            }
            else
            {
                canbus_saturating_increment(&data->can_tx_detail_suppressed_count);
            }

            canbus_poll_errors(canbus, data);
            canbus_record_task_tx_status(data, ret);
            data->canbus_fault = data->canbus_fault || data->can_busoff_fault;

            ams_heartbeat_kick(data, AMS_HEARTBEAT_CAN, osKernelGetTickCount());
            canbus_wait_next_period(data, entry);
        }
    }
}
