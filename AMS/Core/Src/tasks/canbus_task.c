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
#if AMS_ENABLE_TUNING_CAN
static ams_tuning_snapshot_t canbus_tuning_cache;
#endif

#define ECU_SEG_CELLS 15u
/* Hardware and the production contract both expose all 24 thermistors/SMB. */
#define ECU_SEG_TEMPS NTEMPS
#define ECU_FANS      10u
#define ECU_TEMP_INVALID_DECI_C ((uint16_t)0x8000u)
#define CAN_MEASUREMENT_TIMEOUT_MS 500u
#define CAN_ECU_COMPACT_PROTOCOL_VERSION 1u
#define CAN_ECU_FAST_FREQ AMS_CAN_ECU_FAST_FREQ_HZ
#define CAN_ECU_FAST_PERIOD_MS AMS_CAN_ECU_FAST_PERIOD_MS
#define CAN_ECU_SLOW_DIV ((CAN_ECU_FAST_FREQ + CAN_FREQ - 1u) / CAN_FREQ)
#define CAN_CHARGER_DIV ((CHARGER_COMMAND_PERIOD_MS + CAN_ECU_FAST_PERIOD_MS - 1u) / CAN_ECU_FAST_PERIOD_MS)

/* Bench topology sends one detail phase per configured SMB.  The normal
 * vehicle image uses five phases; this temporary one-SMB image uses one. */
#define CAN_TELEMETRY_PHASE_COUNT NSMBS

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
#if AMS_APM_STANDALONE_EVAL_BENCH && AMS_ENABLE_APM_2950
        /* Standalone ADBMS2950B evaluation image: expose the physically
         * measured 50 uOhm shunt current on the bench CAN contract. This is
         * diagnostic-only because the build remains output-inhibited. */
        if(data->acc.apm.health.sample_valid &&
           data->acc.apm.health.current_valid &&
           ((uint32_t)(osKernelGetTickCount() -
                       data->acc.apm.health.last_update_ms) <=
            ACCUMULATOR_APM_SAMPLE_STALE_TIMEOUT_MS))
        {
            view->current_valid = true;
            view->current_A = data->acc.apm.health.current_a;
            if(data->acc.apm.health.pack_voltage_valid)
            {
                view->pack_voltage_V = data->acc.apm.health.pack_voltage_v;
            }
        }
#endif
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

static HAL_StatusTypeDef canbus_send(canbus_device_t *canbus,
                                     uint32_t ide,
                                     uint32_t id,
                                     const uint8_t payload[8])
{
    if((canbus == NULL) || (payload == NULL))
    {
        return HAL_ERROR;
    }
    /* DER26-CAN-V4: encoding is task-side only. Physical bxCAN serialization
     * is owned by the asynchronous TX pump and mailbox-complete callbacks. */
#if AMS_HOST_TEST
    /* Legacy host tests exercise individual static packet encoders without a
     * scheduler build context. Keep that test-only escape hatch out of target
     * firmware so production code can never bypass the asynchronous service. */
    if(!canbus->tx_builder.active)
    {
        if(HAL_CAN_GetTxMailboxesFreeLevel(canbus->hcan) == 0u)
        {
            return HAL_TIMEOUT;
        }
        CAN_TxHeaderTypeDef header = {0};
        uint32_t mailbox = 0u;
        header.IDE = ide;
        header.RTR = CAN_RTR_DATA;
        header.DLC = DATALEN;
        header.TransmitGlobalTime = DISABLE;
        if(ide == CAN_ID_EXT) header.ExtId = id; else header.StdId = id;
        return HAL_CAN_AddTxMessage(canbus->hcan, &header, (uint8_t *)payload,
                                    &mailbox);
    }
#endif
    return canbus_tx_build_append(canbus, ide, id, payload);
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

    return canbus_send(canbus, CAN_ID_STD, AMS_LEGACY_TELEM_CAN_ID, can_data);
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
        /* A V4 publication failure is a software/scheduler health event, not
         * a bxCAN mailbox timeout. Keep the HAL error code reserved for actual
         * controller/HAL errors; class-specific publish/load counters expose
         * the application-side failure without recreating the old misleading
         * "CAN timeout" diagnostic. */
        data->canbus_fault = true;
        canbus_saturating_increment(&data->can_error_count);
        data->can_last_error_tick = now;
    }
    else if(!data->can_busoff_fault &&
            !data->can_recover_pending &&
            data->canbus_fault &&
            (data->can_error_code == HAL_CAN_ERROR_NONE) &&
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

static HAL_StatusTypeDef send_ecu_current_diag(canbus_device_t *canbus,
                                                   const app_data_t *data,
                                                   const can_measurement_view_t *view)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t payload[8] = {0};
    uint32_t sample_tick = data->current_sample_tick;
    uint32_t sample_sequence = data->current_sample_sequence;
    bool valid = (view != NULL) ? view->current_valid : data->current_valid;
    uint8_t source = valid ? 1u : 0u;   /* 1 = DHAB, 2 = ADBMS2950 */
    uint8_t quality = valid ? 2u : 0u;  /* calibrated primary */

#if AMS_APM_STANDALONE_EVAL_BENCH && AMS_ENABLE_APM_2950
    if(data->acc.apm.health.sample_valid &&
       data->acc.apm.health.current_valid)
    {
        source = 2u;
        sample_tick = data->acc.apm.health.last_update_ms;
        sample_sequence = data->acc.apm.health.sample_count;
        quality = data->acc.apm.redundant_sample.valid ? 3u : 2u;
        valid = ((uint32_t)(osKernelGetTickCount() - sample_tick) <=
                 ACCUMULATOR_APM_SAMPLE_STALE_TIMEOUT_MS);
    }
#endif

    if(!valid)
    {
        source = 0u;
        quality = 0u;
        sample_tick = osKernelGetTickCount();
        sample_sequence = 0u;
    }

    payload[0] = source;
    payload[1] = quality;
    payload[2] = valid ? 1u : 0u; /* physical/canonical boundary */
    payload[3] = 1u;              /* boot source epoch */
    ecu_put_u16(payload, 4u, (uint16_t)sample_sequence);
    ecu_put_u16(payload, 6u,
                sat_u16_scaled((float)(osKernelGetTickCount() - sample_tick),
                               1.0f));
    return canbus_send(canbus, CAN_ID_STD, AMS_ECU_CAN_ID_CURRENT_DIAG, payload);
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
    ret |= send_ecu_current_diag(canbus, data, view);

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

    /* V4 transport diagnostics let the ECU/post-run logger distinguish
     * intentional source supersession from physical-bus/ECU-side loss.
     * This is observational detail only and never participates in authority. */
    memset(payload, 0, sizeof(payload));
    logger_put_u16(payload, 0u, sat_u16_u32(canbus->tx_scheduler.protected_deadline_miss));
    logger_put_u16(payload, 2u, sat_u16_u32(canbus->tx_scheduler.detail_superseded));
    logger_put_u16(payload, 4u, sat_u16_u32(canbus->tx_scheduler.detail_discarded_on_recovery));
    payload[6] = sat_u8_u32(canbus->tx_scheduler.protected_superseded);
    payload[7] = logger_bool_bit(canbus->tx_suspended, 0u) |
                 logger_bool_bit(canbus->tx_latched_inhibit, 1u) |
                 logger_bool_bit(data->can_busoff_fault, 2u) |
                 logger_bool_bit(data->can_recover_pending, 3u) |
                 logger_bool_bit(canbus->tx_build_detail_overflow_count != 0u, 4u) |
                 logger_bool_bit(canbus->tx_build_commit_reject_count != 0u, 5u);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TX_SCHED_DIAG, payload);

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
                 logger_bool_bit(data->adbms_balance_write_fault, AMS_LOGGER_ADBMS_DIAG_FLAG_BALANCE_WRITE_FAULT) |
                 logger_bool_bit(data->adbms_voltage_redundancy_degraded, AMS_LOGGER_ADBMS_DIAG_FLAG_VOLTAGE_DEGRADED);
    payload[7] = logger_bool_bit(data->hil.meas.fresh != 0u, AMS_LOGGER_HIL_FLAG_MEAS_FRESH) |
                 logger_bool_bit(data->hil.truth.fresh != 0u, AMS_LOGGER_HIL_FLAG_TRUTH_FRESH) |
                 logger_bool_bit(data->hil.summary.fresh != 0u, AMS_LOGGER_HIL_FLAG_SUMMARY_FRESH) |
                 logger_bool_bit(!data->temp_valid, AMS_LOGGER_ADBMS_STATE_FLAG_TEMP_UNAVAILABLE) |
                 logger_bool_bit(data->acc.smb_ready, AMS_LOGGER_ADBMS_STATE_FLAG_SMB_READY) |
                 logger_bool_bit(data->adbms_last_voltage_scan_ok, AMS_LOGGER_ADBMS_STATE_FLAG_LAST_VOLTAGE_SCAN_OK) |
                 logger_bool_bit(data->adbms_voltage_redundancy_degraded, AMS_LOGGER_ADBMS_STATE_FLAG_C_ONLY_MODE) |
                 logger_bool_bit((data->adbms_fault_active_mask & AMS_ADBMS_FAULT_S_REDUNDANCY) != 0u,
                                 AMS_LOGGER_ADBMS_STATE_FLAG_S_REDUNDANCY_FAULT);
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
                 logger_bool_bit((data->rtos_fault_flags & AMS_RTOS_FAULT_FLAG_ASSERT_FAILED) != 0u, AMS_LOGGER_RTOS_FLAG_ASSERT_FAILED) |
                 logger_bool_bit(data->rtos_stack_critical, AMS_LOGGER_RTOS_FLAG_STACK_CRITICAL);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_RTOS_DIAG, payload);


#if AMS_ENABLE_APM_2950
    {
        const adbms2950_health_t *apm = &acc->apm.health;
        const adbms2950_redundant_sample_t *red = &acc->apm.redundant_sample;
        uint32_t age_ms = (apm->last_update_ms != 0u) ?
            (uint32_t)(osKernelGetTickCount() - apm->last_update_ms) : UINT32_MAX;

        memset(payload, 0, sizeof(payload));
        logger_put_i16(payload, 0u,
                       apm->current_valid ? sat_i16_scaled(apm->current_a, 100.0f) : INT16_MIN);
        logger_put_i16(payload, 2u,
                       red->valid ? sat_i16_scaled(red->current2_a, 100.0f) : INT16_MIN);
        logger_put_u16(payload, 4u,
                       apm->pack_voltage_valid ? sat_u16_scaled(apm->pack_voltage_v, 10.0f) : UINT16_MAX);
        logger_put_u16(payload, 6u,
                       red->valid ? sat_u16_scaled(red->pack_voltage2_v, 10.0f) : UINT16_MAX);
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_APM_SAMPLE, payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = logger_bool_bit(apm->initialized, 0u) |
                     logger_bool_bit(apm->sid_valid, 1u) |
                     logger_bool_bit(apm->config_valid, 2u) |
                     logger_bool_bit(apm->sample_valid, 3u) |
                     logger_bool_bit(apm->current_valid, 4u) |
                     logger_bool_bit(apm->pack_voltage_valid, 5u) |
                     logger_bool_bit(apm->refup, 6u) |
                     logger_bool_bit(apm->snapshot_active, 7u);
        payload[1] = (uint8_t)apm->last_stage;
        payload[2] = (uint8_t)apm->last_reason;
        payload[3] = apm->device_id;
        logger_put_u16(payload, 4u, apm->i1_conversion_count);
        logger_put_u16(payload, 6u, sat_u16_scaled((float)age_ms, 1.0f));
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_APM_HEALTH, payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = (uint8_t)((uint32_t)apm->i1_raw >> 24u);
        payload[1] = (uint8_t)((uint32_t)apm->i1_raw >> 16u);
        payload[2] = (uint8_t)((uint32_t)apm->i1_raw >> 8u);
        payload[3] = (uint8_t)((uint32_t)apm->i1_raw);
        logger_put_i16(payload, 4u, apm->vb1_raw);
        payload[6] = apm->i1_conversion_phase;
        payload[7] = (uint8_t)acc->apm.calibration.profile;
        ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_APM_RAW, payload);
    }
#endif

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
    uint8_t phase,
    uint8_t sequence)
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
        payload[0] = ams_logger_phase_tag(sequence, phase);
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
        payload[0] = ams_logger_phase_tag(sequence, phase);
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
    payload[0] = ams_logger_phase_tag(sequence, phase);
    logger_put_u16(payload, 1u, updated);
    logger_put_u16(payload, 3u, usable);
    logger_put_u16(payload, 5u, stale);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_VOLTAGE_MASKS, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = ams_logger_phase_tag(sequence, phase);
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
    payload[0] = ams_logger_phase_tag(sequence, phase);
    logger_put_u24(payload, 1u, temp_updated);
    logger_put_u24(payload, 4u, temp_usable);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_MASKS_A, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = ams_logger_phase_tag(sequence, phase);
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
    payload[0] = ams_logger_phase_tag(sequence, phase);
    logger_put_u24(payload, 1u, temp_open);
    logger_put_u24(payload, 4u, temp_short);
    payload[7] = (uint8_t)((logger_count_bits32(temp_open) & 0x0Fu) |
                           ((logger_count_bits32(temp_short) & 0x0Fu) << 4u));
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_DIAG_A, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = ams_logger_phase_tag(sequence, phase);
    logger_put_u24(payload, 1u, temp_jump);
    logger_put_u24(payload, 4u, temp_rate);
    payload[7] = (uint8_t)((logger_count_bits32(temp_jump) & 0x0Fu) |
                           ((logger_count_bits32(temp_rate) & 0x0Fu) << 4u));
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_DIAG_B, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = ams_logger_phase_tag(sequence, phase);
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
    ret |= send_logger_detail_phase(canbus, data, view, phase, sequence);
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

static HAL_StatusTypeDef send_estimator_voltage_compare(
    canbus_device_t *canbus, const app_data_t *data)
{
    if((canbus == NULL) || (data == NULL) ||
       (data->estimator.enabled == 0u) ||
       (data->estimator.instance_count == 0u) ||
       (data->estimator.active_index >= data->estimator.instance_count) ||
       (data->estimator.active_index >= AMS_EKF_MAX_INSTANCES))
    {
        return HAL_OK;
    }

    const uint8_t active = data->estimator.active_index;
    const uint16_t bit = (uint16_t)(1u << active);
    const bool raw_valid = (data->estimator.voltage_raw_valid_mask & bit) != 0u;
    const bool avg_valid = (data->estimator.voltage_avg8_valid_mask & bit) != 0u;
    const bool iir_valid = (data->estimator.voltage_iir_valid_mask & bit) != 0u;
    const int16_t avg_minus_raw_mV =
        (raw_valid && avg_valid) ?
        sat_i16_from_float((data->estimator.voltage_avg8_V[active] -
                            data->estimator.voltage_raw_V[active]) * 1000.0f) : 0;
    const int16_t iir_minus_raw_mV =
        (raw_valid && iir_valid) ?
        sat_i16_from_float((data->estimator.voltage_iir_V[active] -
                            data->estimator.voltage_raw_V[active]) * 1000.0f) : 0;

    uint8_t payload[8] = {0};
    payload[0] = active;
    payload[1] = (uint8_t)((raw_valid ? 0x01u : 0u) |
                           (avg_valid ? 0x02u : 0u) |
                           (iir_valid ? 0x04u : 0u) |
                           ((AMS_ESTIMATOR_VOLTAGE_SOURCE & 0x03u) << 4u));
    payload[2] = TO_MSB16((uint16_t)avg_minus_raw_mV);
    payload[3] = TO_LSB16((uint16_t)avg_minus_raw_mV);
    payload[4] = TO_MSB16((uint16_t)iir_minus_raw_mV);
    payload[5] = TO_LSB16((uint16_t)iir_minus_raw_mV);
    payload[6] = TO_MSB16((uint16_t)data->estimator.voltage_compare_sequence);
    payload[7] = TO_LSB16((uint16_t)data->estimator.voltage_compare_sequence);

    return canbus_send(canbus, CAN_ID_STD,
                       AMS_LOGGER_CAN_ID_ESTIMATOR_VOLTAGE_COMPARE, payload);
}

#if AMS_ENABLE_TUNING_CAN
static bool canbus_copy_tuning_snapshot(app_data_t *data,
                                        ams_tuning_snapshot_t *out)
{
    if((data == NULL) || (out == NULL))
    {
        return false;
    }

    ams_tuning_store_t *store = &data->tuning_store;
    uint8_t index;
    taskENTER_CRITICAL();
    if(!store->published)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    index = store->published_index;
    store->reader_count[index]++;
    taskEXIT_CRITICAL();

    memcpy(out, &store->buffer[index], sizeof(*out));

    taskENTER_CRITICAL();
    if(store->reader_count[index] > 0u)
    {
        store->reader_count[index]--;
    }
    taskEXIT_CRITICAL();
    return true;
}

static uint16_t tuning_u16_scaled(float value, float scale)
{
    if(!isfinite(value))
    {
        return UINT16_MAX;
    }
    return sat_u16_from_float(value * scale);
}

/* Signed tuning quantities reserve INT16_MIN as an invalid sentinel so a
 * non-finite covariance cannot silently decode as a physically meaningful
 * zero correlation. */
static int16_t tuning_i16_scaled(float value, float scale)
{
    if(!isfinite(value))
    {
        return INT16_MIN;
    }

    const float scaled = value * scale;
    if(scaled >= 32767.0f)
    {
        return INT16_MAX;
    }
    if(scaled <= -32767.0f)
    {
        return -32767;
    }
    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) :
                                             (scaled - 0.5f));
}

static HAL_StatusTypeDef send_tuning_ekf_fast(
    canbus_device_t *canbus,
    const ams_tuning_snapshot_t *snapshot)
{
    HAL_StatusTypeDef status = HAL_OK;
    const uint8_t sequence = (uint8_t)snapshot->snapshot_sequence;
    for(uint8_t i = 0u; i < snapshot->instance_count; i++)
    {
        const ams_tuning_segment_t *seg = &snapshot->segment[i];
        uint8_t payload[8] = {0};
        payload[0] = i;
        payload[1] = sequence;
        logger_put_u16(payload, 2u, tuning_u16_scaled(seg->soc, 10000.0f));
        logger_put_i16(payload, 4u, sat_i16_from_float(seg->vp1_v * 1000.0f));
        logger_put_i16(payload, 6u, sat_i16_from_float(seg->vp2_v * 1000.0f));
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_EKF_STATE,
                                    payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = i;
        payload[1] = sequence;
        logger_put_u16(payload, 2u,
                       tuning_u16_scaled(seg->measured_v, 1000.0f));
        logger_put_u16(payload, 4u,
                       tuning_u16_scaled(seg->v_pred_v, 1000.0f));
        logger_put_i16(payload, 6u,
                       sat_i16_from_float(seg->innovation_v * 1000.0f));
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_EKF_MODEL,
                                    payload);
    }
    return status;
}

static HAL_StatusTypeDef send_tuning_ekf_slow(
    canbus_device_t *canbus,
    const ams_tuning_snapshot_t *snapshot)
{
    HAL_StatusTypeDef status = HAL_OK;
    const uint8_t sequence = (uint8_t)(snapshot->snapshot_sequence & 0x3Fu);
    for(uint8_t i = 0u; i < snapshot->instance_count; i++)
    {
        const ams_tuning_segment_t *seg = &snapshot->segment[i];
        uint8_t payload[8] = {0};
        payload[0] = i;
        payload[1] = sequence;
        logger_put_u16(payload, 2u,
                       tuning_u16_scaled(seg->p_soc, 1000000000.0f));
        logger_put_u16(payload, 4u,
                       tuning_u16_scaled(seg->p_vp1, 1000000000.0f));
        logger_put_u16(payload, 6u,
                       tuning_u16_scaled(seg->p_vp2, 1000000000.0f));
        status |= send_logger_frame(canbus,
                                    AMS_LOGGER_CAN_ID_EKF_COVARIANCE,
                                    payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = i;
        payload[1] = (uint8_t)(0x40u | sequence);
        logger_put_u16(payload, 2u,
                       tuning_u16_scaled(seg->p_r0, 1000000000000.0f));
        logger_put_u16(payload, 4u,
                       tuning_u16_scaled(seg->r_meas_v2, 1000000000.0f));
        logger_put_u16(payload, 6u, (uint16_t)seg->dt_clamp_count);
        status |= send_logger_frame(canbus,
                                    AMS_LOGGER_CAN_ID_EKF_COVARIANCE,
                                    payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = i;
        payload[1] = (uint8_t)(0x80u | sequence);
        logger_put_u16(payload, 2u,
                       (uint16_t)seg->innovation_reject_count);
        logger_put_u16(payload, 4u, (uint16_t)seg->fault_flags);
        logger_put_u16(payload, 6u,
                       tuning_u16_scaled(
                           sqrtf(fmaxf(0.0f, seg->innovation_variance_v2)),
                           1000.0f));
        status |= send_logger_frame(canbus,
                                    AMS_LOGGER_CAN_ID_EKF_COVARIANCE,
                                    payload);

        /* Covariance page 3: signed full-covariance cross terms. Scale 1e7
         * gives 1e-7 native-unit resolution and +/-3.2767e-3 range, which
         * comfortably covers the qualified startup and normal covariance
         * envelopes while preserving useful offline NEES precision. */
        memset(payload, 0, sizeof(payload));
        payload[0] = i;
        payload[1] = (uint8_t)(0xC0u | sequence);
        logger_put_i16(payload, 2u,
                       tuning_i16_scaled(seg->p_soc_vp1, 10000000.0f));
        logger_put_i16(payload, 4u,
                       tuning_i16_scaled(seg->p_soc_vp2, 10000000.0f));
        logger_put_i16(payload, 6u,
                       tuning_i16_scaled(seg->p_vp1_vp2, 10000000.0f));
        status |= send_logger_frame(canbus,
                                    AMS_LOGGER_CAN_ID_EKF_COVARIANCE,
                                    payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = i;
        payload[1] = sequence;
        logger_put_u16(payload, 2u,
                       tuning_u16_scaled(seg->r0_ohm, 1000000.0f));
        logger_put_u16(payload, 4u,
                       tuning_u16_scaled(seg->resistance_growth_ratio,
                                         10000.0f));
        payload[6] = seg->soh_confidence_pct;
        payload[7] = (uint8_t)((seg->valid ? 0x01u : 0u) |
                               ((seg->soh_status_flags & 0x0Fu) << 1u) |
                               ((seg->soh_reject_flags != 0u) ? 0x20u : 0u));
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_EKF_SOH,
                                    payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = i;
        payload[1] = (uint8_t)(0x80u | sequence);
        logger_put_u16(payload, 2u,
                       tuning_u16_scaled(seg->reference_r0_ohm, 1000000.0f));
        logger_put_u16(payload, 4u,
                       tuning_u16_scaled(seg->r0_variance_ohm2,
                                         1000000000000.0f));
        logger_put_u16(payload, 6u, (uint16_t)seg->soh_reject_flags);
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_EKF_SOH,
                                    payload);

        /* Context page 0: synchronized RAW/AVG8/IIR segment-voltage products. */
        memset(payload, 0, sizeof(payload));
        payload[0] = i;
        payload[1] = sequence;
        logger_put_u16(payload, 2u,
            (seg->voltage_valid_flags & 0x01u) ?
            tuning_u16_scaled(seg->voltage_raw_v, 1000.0f) : UINT16_MAX);
        logger_put_u16(payload, 4u,
            (seg->voltage_valid_flags & 0x02u) ?
            tuning_u16_scaled(seg->voltage_avg8_v, 1000.0f) : UINT16_MAX);
        logger_put_u16(payload, 6u,
            (seg->voltage_valid_flags & 0x04u) ?
            tuning_u16_scaled(seg->voltage_iir_v, 1000.0f) : UINT16_MAX);
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_EKF_CONTEXT,
                                    payload);

        /* Context page 1: current and feed-forward surface/core temperature. */
        memset(payload, 0, sizeof(payload));
        payload[0] = i;
        payload[1] = (uint8_t)(0x40u | sequence);
        logger_put_i16(payload, 2u,
                       sat_i16_from_float(seg->current_a * 10.0f));
        logger_put_i16(payload, 4u,
                       sat_i16_from_float(seg->surface_temp_c * 10.0f));
        logger_put_i16(payload, 6u,
                       sat_i16_from_float(seg->t_core_c * 10.0f));
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_EKF_CONTEXT,
                                    payload);

        /* Context page 2: acquisition keys and age (10-ms units). */
        memset(payload, 0, sizeof(payload));
        payload[0] = i;
        payload[1] = (uint8_t)(0x80u | sequence);
        logger_put_u16(payload, 2u, (uint16_t)seg->measurement_sequence);
        logger_put_u16(payload, 4u, (uint16_t)seg->current_sequence);
        payload[6] = sat_u8_u32(seg->measurement_age_ms / 10u);
        payload[7] = sat_u8_u32(seg->current_age_ms / 10u);
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_EKF_CONTEXT,
                                    payload);

        /* Context page 3: health/counter detail. The global full estimator
         * step is already carried by 0x6BF, so the previous duplicate low16
         * segment step is replaced with the covariance-repair counter. */
        memset(payload, 0, sizeof(payload));
        payload[0] = i;
        payload[1] = (uint8_t)(0xC0u | sequence);
        payload[2] = (uint8_t)((seg->fresh_temp_count > 255u) ?
                               255u : seg->fresh_temp_count);
        payload[3] = seg->model_domain_flags;
        logger_put_u16(payload, 4u,
                       (uint16_t)seg->covariance_repair_count);
        payload[6] = seg->soh_accepted_count;
        payload[7] = seg->soh_rejected_count;
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_EKF_CONTEXT,
                                    payload);
    }
    return status;
}

static HAL_StatusTypeDef send_tuning_meta(
    canbus_device_t *canbus,
    const ams_tuning_snapshot_t *snapshot)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t payload[8] = {0};
    const uint8_t sequence = (uint8_t)snapshot->snapshot_sequence;

    payload[0] = 0u;
    payload[1] = sequence;
    payload[2] = snapshot->instance_count;
    payload[3] = (uint8_t)((snapshot->power_valid ? 0x01u : 0u) |
                           (snapshot->power_authority_valid ? 0x02u : 0u));
    logger_put_u32(payload, 4u, snapshot->estimator_step);
    status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TUNING_META, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = 1u;
    payload[1] = sequence;
    logger_put_u32(payload, 2u, snapshot->source_tick_ms);
    logger_put_u16(payload, 6u, (uint16_t)snapshot->measurement_sequence);
    status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TUNING_META, payload);
    return status;
}

static HAL_StatusTypeDef send_tuning_acquisition_meta(
    canbus_device_t *canbus,
    const ams_tuning_snapshot_t *snapshot)
{
    if((canbus == NULL) || (snapshot == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_OK;
    const uint8_t sequence = (uint8_t)snapshot->snapshot_sequence;
    for(uint8_t i = 0u; i < snapshot->instance_count; i++)
    {
        const ams_tuning_segment_t *seg = &snapshot->segment[i];
        uint8_t payload[8] = {0};
        /* 0x6BF acquisition page: byte0 bit7 distinguishes per-segment
         * acquisition telemetry from global metadata pages 0/1. */
        payload[0] = (uint8_t)(0x80u | (i & 0x0Fu));
        payload[1] = sequence;
        payload[2] = seg->acquisition_state;
        payload[3] = seg->acquisition_reason;
        payload[4] = seg->acquisition_sample_count;
        payload[5] = seg->acquisition_reject_count;
        logger_put_u16(payload, 6u,
                       tuning_u16_scaled(seg->acquisition_candidate_soc,
                                         10000.0f));
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TUNING_META,
                                    payload);
    }
    return status;
}

static HAL_StatusTypeDef send_tuning_sop_fuse(
    canbus_device_t *canbus,
    const ams_tuning_snapshot_t *snapshot)
{
    HAL_StatusTypeDef status = HAL_OK;
    const uint8_t sequence = (uint8_t)snapshot->snapshot_sequence;
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        const ams_tuning_sop_horizon_t *sop = &snapshot->horizon[h];
        uint8_t payload[8] = {h, sequence, 0u, 0u, 0u, 0u, 0u, 0u};
        logger_put_i16(payload, 2u,
                       sat_i16_from_float(sop->raw_model_discharge_a * 10.0f));
        logger_put_i16(payload, 4u,
                       sat_i16_from_float(sop->strategy_discharge_a * 10.0f));
        logger_put_i16(payload, 6u,
                       sat_i16_from_float(sop->final_discharge_a * 10.0f));
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_SOP_DISCHARGE,
                                    payload);

        payload[0] = h;
        payload[1] = sequence;
        logger_put_i16(payload, 2u,
                       sat_i16_from_float(sop->raw_model_charge_a * 10.0f));
        logger_put_i16(payload, 4u,
                       sat_i16_from_float(sop->strategy_charge_a * 10.0f));
        logger_put_i16(payload, 6u,
                       sat_i16_from_float(sop->final_charge_a * 10.0f));
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_SOP_CHARGE,
                                    payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = h;
        payload[1] = sequence;
        payload[2] = sop->discharge_binding;
        payload[3] = sop->discharge_segment;
        payload[4] = sop->discharge_cell;
        payload[5] = sop->charge_binding;
        payload[6] = sop->charge_segment;
        payload[7] = sop->charge_cell;
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_SOP_BINDING,
                                    payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = (uint8_t)(0x10u | h);
        payload[1] = sequence;
        logger_put_u16(payload, 2u,
                       tuning_u16_scaled(sop->discharge_min_cell_v, 1000.0f));
        logger_put_u16(payload, 4u,
                       tuning_u16_scaled(sop->charge_max_cell_v, 1000.0f));
        logger_put_i16(payload, 6u,
                       sat_i16_from_float(sop->discharge_power_w / 100.0f));
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_SOP_BINDING,
                                    payload);

        /* v3 closes the charge-power/full-reason reconstruction gap. */
        memset(payload, 0, sizeof(payload));
        payload[0] = h;
        payload[1] = sequence;
        logger_put_i16(payload, 2u,
                       sat_i16_from_float(sop->charge_power_w / 100.0f));
        logger_put_u32(payload, 4u, snapshot->reason_flags);
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_SOP_META, payload);

        memset(payload, 0, sizeof(payload));
        payload[0] = h;
        payload[1] = (uint8_t)((snapshot->fuse_valid ? 0x01u : 0u) |
                               (snapshot->fuse_authority_valid ? 0x02u : 0u) |
                               (snapshot->fuse_budget_exhausted ? 0x04u : 0u));
        logger_put_u16(payload, 2u,
                       tuning_u16_scaled(snapshot->fuse_cap_a[h], 10.0f));
        logger_put_u16(payload, 4u,
                       tuning_u16_scaled(snapshot->hardware_discharge_cap_a[h],
                                         10.0f));
        logger_put_u16(payload, 6u, (uint16_t)snapshot->reason_flags);
        status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_FUSE_LIMIT,
                                    payload);
    }

    uint8_t payload[8] = {0};
    payload[0] = 0u;
    payload[1] = (uint8_t)((snapshot->fuse_valid ? 0x01u : 0u) |
                           (snapshot->fuse_authority_valid ? 0x02u : 0u) |
                           (snapshot->fuse_budget_exhausted ? 0x04u : 0u));
    logger_put_u16(payload, 2u,
                   tuning_u16_scaled(snapshot->fuse_utilization, 10000.0f));
    logger_put_i16(payload, 4u,
                   sat_i16_from_float(snapshot->fuse_temperature_c * 10.0f));
    logger_put_u16(payload, 6u,
                   tuning_u16_scaled(snapshot->fuse_derating, 10000.0f));
    status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_FUSE_STATE, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = 1u;
    payload[1] = (uint8_t)snapshot->fuse_reason_flags;
    logger_put_i16(payload, 2u,
                   sat_i16_from_float(snapshot->fuse_effective_current_a * 10.0f));
    logger_put_i16(payload, 4u,
                   sat_i16_from_float(snapshot->fuse_equivalent_current_a * 10.0f));
    logger_put_u16(payload, 6u,
                   tuning_u16_scaled(snapshot->fuse_usable_melt_time_s, 10.0f));
    status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_FUSE_STATE, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = 2u;
    payload[1] = sequence;
    logger_put_u16(payload, 2u, snapshot->fuse_reason_flags);
    logger_put_u16(payload, 4u,
                   tuning_u16_scaled(snapshot->fuse_typical_melt_time_s, 10.0f));
    logger_put_u16(payload, 6u,
                   tuning_u16_scaled(snapshot->fuse_usable_melt_time_s, 10.0f));
    status |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_FUSE_STATE, payload);
    return status;
}
#endif

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

static HAL_StatusTypeDef canbus_publish_charger_command(
    canbus_device_t *canbus,
    uint32_t request_id,
    uint16_t source_tag,
    uint16_t voltage10x,
    uint16_t current10x,
    bool disable)
{
    if(canbus == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = canbus_tx_build_begin(
        canbus, CANBUS_TX_BUILD_CRITICAL, canbus_tx_next_generation(canbus),
        osKernelGetTickCount(), source_tag);
    if(status != HAL_OK)
    {
        return status;
    }

    canbus->tx_builder.request_id = request_id;
    status = canbus_send_charger_command(canbus, voltage10x, current10x,
                                         disable ? CHARGER_CMD_DISABLE :
                                                   CHARGER_CMD_ENABLE);
    if(status == HAL_OK)
    {
        status = canbus_tx_build_commit(canbus, 0u);
    }
    else
    {
        canbus_tx_build_cancel(canbus);
    }
    return status;
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

    bool charger_hw_fault = ccs->hardware_fail || ccs->overtemp_fail ||
                            ccs->input_volt_fail || ccs->voltage_sense_fail ||
                            ccs->communication_fail || ccs->tx_fail;
    uint16_t disable_reasons = charger_disable_reasons(data, charger_hw_fault);
    bool disable_charge = (disable_reasons != CHARGER_DISABLE_REASON_NONE);
    ccs->disable_reason_mask = disable_reasons;
    data->charger_fault = charger_hw_fault;
    if(charger_disable_reasons_force_bms_low(disable_reasons))
    {
        set_bms(false);
    }

    HAL_StatusTypeDef status = canbus_publish_charger_command(
        canbus, 0u,
        CANBUS_TX_TAG_CHARGER_NORMAL, voltage10x, current10x, disable_charge);
    canbus_record_tx_class(&data->can_tx_critical_attempt_count,
                           &data->can_tx_critical_fail_count, status);
    if(status != HAL_OK)
    {
        ccs->tx_fail = true;
        ccs->last_tx_status = status;
        ccs->disable_reason_mask |= CHARGER_DISABLE_REASON_TX_FAIL;
        canbus_saturating_increment(&ccs->tx_fail_count);
        data->charger_fault = true;
        set_bms(false);
    }
    /* Successful publication is not a wire ACK/completion. The mailbox
     * completion callback owns tx_count and clears tx_fail on actual success. */
    return status;
}

static HAL_StatusTypeDef canbus_publish_protected_generation(
    canbus_device_t *canbus,
    app_data_t *data,
    const can_measurement_view_t *view,
    uint8_t wire_sequence,
    uint32_t publish_tick)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = canbus_tx_build_begin(
        canbus, CANBUS_TX_BUILD_PROTECTED,
        canbus_tx_next_generation(canbus), publish_tick, CANBUS_TX_TAG_NONE);
    if(status != HAL_OK)
    {
        return status;
    }

    status = send_ecu_compact_telemetry(canbus, data, view, wire_sequence);
    status |= send_ecu_power_bundle(canbus, data, wire_sequence, publish_tick);
    if(status == HAL_OK)
    {
        /* Required V4 set is 0x680-0x687; 0x689/0x68A/0x68B are advisory and
         * are selected only after all required frames by the scheduler. */
        status = canbus_tx_build_commit(canbus, 8u);
    }
    else
    {
        canbus_tx_build_cancel(canbus);
    }
    return status;
}

static HAL_StatusTypeDef canbus_publish_detail_snapshot(
    canbus_device_t *canbus,
    const app_data_t *data,
    const can_measurement_view_t *view,
    uint8_t snapshot_sequence,
    uint32_t publish_tick
#if AMS_ENABLE_TUNING_CAN
    , const ams_tuning_snapshot_t *tuning
#endif
    )
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = canbus_tx_build_begin(
        canbus, CANBUS_TX_BUILD_DETAIL,
        canbus_tx_next_generation(canbus), publish_tick, CANBUS_TX_TAG_NONE);
    if(status != HAL_OK)
    {
        return status;
    }

    for(uint8_t phase = 0u; phase < CAN_TELEMETRY_PHASE_COUNT; phase++)
    {
        status |= send_logger_phase(canbus, data, view, phase,
                                    snapshot_sequence);
    }
    status |= send_estimator_status(canbus, data);
    status |= send_estimator_voltage_compare(canbus, data);
#if AMS_ENABLE_TUNING_CAN
    if(tuning != NULL)
    {
        status |= send_tuning_ekf_slow(canbus, tuning);
    }
#endif
#if AMS_ENABLE_LEGACY_CAN_TELEMETRY
    status |= send_ecu_ams_phase(canbus, data, view, 0u);
#endif

    if(status == HAL_OK)
    {
        status = canbus_tx_build_commit(canbus, 0u);
    }
    else
    {
        canbus_tx_build_cancel(canbus);
    }
    return status;
}

#if AMS_ENABLE_TUNING_CAN
static HAL_StatusTypeDef canbus_publish_tuning_fast(
    canbus_device_t *canbus,
    const ams_tuning_snapshot_t *tuning,
    bool include_sop,
    bool include_acquisition,
    uint32_t publish_tick)
{
    if((canbus == NULL) || (tuning == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = canbus_tx_build_begin(
        canbus, CANBUS_TX_BUILD_TUNING,
        canbus_tx_next_generation(canbus), publish_tick, CANBUS_TX_TAG_NONE);
    if(status != HAL_OK)
    {
        return status;
    }
    status = send_tuning_ekf_fast(canbus, tuning);
    status |= send_tuning_meta(canbus, tuning);
    if(include_acquisition)
    {
        status |= send_tuning_acquisition_meta(canbus, tuning);
    }
    if(include_sop)
    {
        status |= send_tuning_sop_fuse(canbus, tuning);
    }
    if(status == HAL_OK)
    {
        status = canbus_tx_build_commit(canbus, 0u);
    }
    else
    {
        canbus_tx_build_cancel(canbus);
    }
    return status;
}

static void canbus_tuning_health_guard(app_data_t *data,
                                       const canbus_device_t *canbus)
{
    if((data == NULL) || (canbus == NULL) || data->can_tuning_suppressed)
    {
        return;
    }

    uint32_t reasons = 0u;
    if(canbus->tx_scheduler.protected_deadline_miss != 0u)
    {
        reasons |= AMS_TUNING_SUPPRESS_PROTECTED_DEADLINE;
    }
    if(canbus->tx_scheduler.protected_required_latency_over_50ms != 0u)
    {
        reasons |= AMS_TUNING_SUPPRESS_PROTECTED_LATENCY;
    }
    if(canbus->tx_hal_load_error_protected_count != 0u)
    {
        reasons |= AMS_TUNING_SUPPRESS_PROTECTED_HAL;
    }
    if(data->can_task_deadline_miss_count != 0u)
    {
        reasons |= AMS_TUNING_SUPPRESS_CAN_TASK_DEADLINE;
    }
    if(data->can_busoff_fault || data->can_recover_pending ||
       canbus->tx_latched_inhibit)
    {
        reasons |= AMS_TUNING_SUPPRESS_BUS_HEALTH;
    }
    if((canbus->tx_build_overflow_count != 0u) ||
       (canbus->tx_build_class_reject_count != 0u) ||
       (canbus->tx_build_commit_reject_count != 0u))
    {
        reasons |= AMS_TUNING_SUPPRESS_BUILD_INTEGRITY;
    }
    if(reasons != 0u)
    {
        data->can_tuning_suppressed = true;
        data->can_tuning_suppression_reason_mask |= reasons;
        canbus_saturating_increment(&data->can_tuning_suppression_count);
    }
}
#endif

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
    uint8_t ecu_sequence = 0u;
    uint8_t logger_sequence = 0u;
    uint16_t charger_div = CAN_CHARGER_DIV;
    uint32_t last_detail_publish_tick = osKernelGetTickCount();
#if AMS_ENABLE_TUNING_CAN
    uint32_t last_tuning_publish_tick = osKernelGetTickCount();
    uint32_t last_tuning_sop_publish_tick = osKernelGetTickCount();
    uint32_t last_tuning_acq_publish_tick = osKernelGetTickCount();
#endif

    const uint16_t voltage10x = (uint16_t)(CHARGE_MAX_VOLTAGE * 10.0f);
    const uint16_t current10x = (uint16_t)(CHARGE_MAX_CURRENT * 10.0f);

    for(;;)
    {
        uint32_t entry = osKernelGetTickCount();
        HAL_StatusTypeDef task_status = HAL_OK;
        bool have_measurement = ams_measurement_store_copy_latest(
            &data->measurement_store, &canbus_measurement_cache);
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
#if AMS_ENABLE_TUNING_CAN
        canbus_tuning_health_guard(data, canbus);
#endif

        if(data->can_busoff_fault || data->can_recover_pending ||
           canbus->tx_latched_inhibit)
        {
            data->canbus_fault = true;
            ams_heartbeat_kick(data, AMS_HEARTBEAT_CAN, entry);
            canbus_wait_next_period(data, entry);
            continue;
        }

        /* State-exit charger disable retains absolute software priority. */
        if(canbus->tx_refresh_pending)
        {
            /* Force a fresh charger decision this cycle before resumed TX. */
            charger_div = CAN_CHARGER_DIV;
        }
        bool charger_shutdown_attempted = false;
        if(ccs->shutdown_pending)
        {
            uint32_t shutdown_request_id;
            taskENTER_CRITICAL();
            shutdown_request_id = ccs->shutdown_request_count;
            taskEXIT_CRITICAL();

            charger_shutdown_attempted = true;
            ccs->target_voltage = 0.0f;
            ccs->target_current = 0.0f;
            HAL_StatusTypeDef shutdown_status = canbus_publish_charger_command(
                canbus, shutdown_request_id, CANBUS_TX_TAG_CHARGER_SHUTDOWN,
                0u, 0u, true);
            canbus_record_tx_class(&data->can_tx_critical_attempt_count,
                                   &data->can_tx_critical_fail_count,
                                   shutdown_status);
            task_status |= shutdown_status;

            taskENTER_CRITICAL();
            ccs->last_shutdown_status = shutdown_status;
            ccs->last_tx_status = shutdown_status;
            if(shutdown_status != HAL_OK)
            {
                ccs->tx_fail = true;
                ccs->disable_reason_mask |=
                    CHARGER_DISABLE_REASON_STATE_EXIT |
                    CHARGER_DISABLE_REASON_TX_FAIL;
                canbus_saturating_increment(&ccs->tx_fail_count);
                canbus_saturating_increment(&ccs->shutdown_tx_fail_count);
                data->charger_fault = true;
            }
            else
            {
                ccs->disable_reason_mask |= CHARGER_DISABLE_REASON_STATE_EXIT;
            }
            taskEXIT_CRITICAL();
            if(shutdown_status != HAL_OK)
            {
                set_bms(false);
            }
        }

        if(data->state == STATE_CHARGE)
        {
            task_status |= canbus_run_periodic_charger_command(
                data, canbus, ccs, &charger_div, charger_shutdown_attempted,
                voltage10x, current10x);
        }
        else
        {
            ccs->target_voltage = 0.0f;
            ccs->target_current = 0.0f;
            ccs->read_voltage = 0.0f;
            ccs->read_current = 0.0f;
            ccs->communication_fail = false;
            data->charger_fault = ccs->shutdown_pending || ccs->tx_fail;
        }

        /* 10 Hz latest-generation protected authority/status publication. */
        HAL_StatusTypeDef protected_status = canbus_publish_protected_generation(
            canbus, data, &measurement_view, ecu_sequence++, entry);
        canbus_record_tx_class(&data->can_tx_compact_bundle_count,
                               &data->can_tx_compact_bundle_fail_count,
                               protected_status);
        task_status |= protected_status;

        if(canbus->tx_refresh_pending && (task_status == HAL_OK))
        {
            canbus_tx_resume_after_refresh(canbus);
        }

        /* 2 Hz coherent detail snapshot. ACTIVE runs to completion; only the
         * latest PENDING snapshot may supersede an older PENDING snapshot. */
        if((uint32_t)(entry - last_detail_publish_tick) >=
           AMS_CAN_DETAIL_FULL_PERIOD_MS)
        {
            last_detail_publish_tick = entry;
#if AMS_ENABLE_TUNING_CAN
            const ams_tuning_snapshot_t *slow_tuning = NULL;
            if(!data->can_tuning_suppressed &&
               canbus_copy_tuning_snapshot(data, &canbus_tuning_cache))
            {
                slow_tuning = &canbus_tuning_cache;
            }
#endif
            HAL_StatusTypeDef detail_status = canbus_publish_detail_snapshot(
                canbus, data, &measurement_view, logger_sequence++, entry
#if AMS_ENABLE_TUNING_CAN
                , slow_tuning
#endif
                );
            canbus_record_tx_class(&data->can_tx_detail_phase_count,
                                   &data->can_tx_detail_phase_fail_count,
                                   detail_status);
            if(detail_status == HAL_OK)
            {
                ams_heartbeat_kick(data, AMS_HEARTBEAT_LOGGER, entry);
            }
        }

#if AMS_ENABLE_TUNING_CAN
        if(!data->can_tuning_suppressed &&
           ((uint32_t)(entry - last_tuning_publish_tick) >=
            AMS_TUNING_CAN_FAST_PERIOD_MS) &&
           canbus_copy_tuning_snapshot(data, &canbus_tuning_cache))
        {
            last_tuning_publish_tick = entry;
            bool include_sop =
                ((uint32_t)(entry - last_tuning_sop_publish_tick) >=
                 AMS_TUNING_CAN_SOP_PERIOD_MS);
            bool include_acquisition =
                ((uint32_t)(entry - last_tuning_acq_publish_tick) >=
                 AMS_TUNING_CAN_ACQ_PERIOD_MS);
            if(include_sop)
            {
                last_tuning_sop_publish_tick = entry;
            }
            if(include_acquisition)
            {
                last_tuning_acq_publish_tick = entry;
            }
            HAL_StatusTypeDef tuning_status = canbus_publish_tuning_fast(
                canbus, &canbus_tuning_cache, include_sop,
                include_acquisition, entry);
            canbus_record_tx_class(&data->can_tx_detail_phase_count,
                                   &data->can_tx_detail_phase_fail_count,
                                   tuning_status);
        }
#endif

        canbus_poll_errors(canbus, data);
        canbus_record_task_tx_status(data, task_status);
        data->canbus_fault = data->canbus_fault || data->can_busoff_fault ||
                             canbus->tx_latched_inhibit;
        ams_heartbeat_kick(data, AMS_HEARTBEAT_CAN, entry);
        canbus_wait_next_period(data, entry);
    }
}
