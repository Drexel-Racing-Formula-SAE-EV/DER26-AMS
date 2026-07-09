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

#include <math.h>
#include <string.h>

void canbus_task_fn(void *arg);

#define ECU_SEG_CELLS 15u
/*
 * Hardware has 24 thermistors per SMB (3x ADG728 muxes into GPIO1/2/3).
 * The ECU telemetry contract currently exports 17 temps per segment in six
 * packets. Keep this as an interface choice unless the ECU/dashboard packet
 * contract is changed with the rest of the vehicle.
 */
#define ECU_SEG_TEMPS 17u
#define ECU_FANS      10u
#define ECU_TEMP_INVALID_DECI_C ((uint16_t)0x8000u)
#define CAN_TX_TIMEOUT_TICKS 10u
#define CAN_ECU_COMPACT_PROTOCOL_VERSION 1u
#define CAN_ECU_FAST_FREQ 10u
#define CAN_ECU_FAST_PERIOD_MS (1000u / CAN_ECU_FAST_FREQ)
#define CAN_ECU_SLOW_DIV ((CAN_ECU_FAST_FREQ + CAN_FREQ - 1u) / CAN_FREQ)
#define CAN_CHARGER_DIV ((CHARGER_COMMAND_PERIOD_MS + CAN_ECU_FAST_PERIOD_MS - 1u) / CAN_ECU_FAST_PERIOD_MS)

static uint16_t sat_u16_scaled(float x, float scale);
static int16_t sat_i16_scaled(float x, float scale);
static uint8_t sat_u8_u32(uint32_t x);

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

static uint16_t fan_percent_for_ecu(const app_data_t *data, uint8_t fan)
{
    if((data == NULL) || (fan >= NFANS))
    {
        return 0u;
    }

    return (uint16_t)(data->board.fans[fan].duty_cycle * 10.0f);
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
        data->can_error_count++;
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

static HAL_StatusTypeDef send_ecu_compact_status(canbus_device_t *canbus,
                                                  const app_data_t *data,
                                                  uint8_t sequence)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t payload[8] = {0};
    payload[0] = CAN_ECU_COMPACT_PROTOCOL_VERSION;
    payload[1] = sequence;
    payload[2] = (uint8_t)data->state;
    payload[3] = ecu_bool_bit(data->bms_state, 0u) |
                 ecu_bool_bit(data->bms_output_inhibit, 1u) |
                 ecu_bool_bit(data->hard_fault, 2u) |
                 ecu_bool_bit(data->soft_fault, 3u) |
                 ecu_bool_bit(data->voltage_valid, 4u) |
                 ecu_bool_bit(data->current_valid, 5u) |
                 ecu_bool_bit(data->temp_valid, 6u) |
                 ecu_bool_bit(data->canbus_fault, 7u);
    payload[4] = ecu_bool_bit(data->voltage_fault, 0u) |
                 ecu_bool_bit(data->temp_fault, 1u) |
                 ecu_bool_bit(data->current_fault, 2u) |
                 ecu_bool_bit(data->imd_ok, 3u) |
                 ecu_bool_bit(data->charger_fault, 4u) |
                 ecu_bool_bit(data->adbms_diag_fault, 5u) |
                 ecu_bool_bit(data->task_heartbeat_fault, 6u) |
                 ecu_bool_bit(data->logger_heartbeat_fault, 7u);
    payload[5] = (uint8_t)data->voltage_fault_reason;
    payload[6] = (uint8_t)data->temp_fault_reason;
    payload[7] = (uint8_t)data->current_fault_reason;

    return canbus_send(canbus, CAN_ID_STD, AMS_ECU_CAN_ID_STATUS, payload);
}

static HAL_StatusTypeDef send_ecu_compact_electrical(canbus_device_t *canbus,
                                                      const app_data_t *data)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t payload[8] = {0};
    ecu_put_u16(payload, 0u, ecu_pack_voltage_deci_v(data));
    ecu_put_i16(payload, 2u, sat_i16_scaled(data->current, 10.0f));
    ecu_put_u16(payload, 4u, data->acc.min_voltage_mv);
    ecu_put_u16(payload, 6u, data->acc.max_voltage_mv);

    return canbus_send(canbus, CAN_ID_STD, AMS_ECU_CAN_ID_ELECTRICAL, payload);
}

static HAL_StatusTypeDef send_ecu_compact_thermal(canbus_device_t *canbus,
                                                   const app_data_t *data)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t payload[8] = {0};
    ecu_put_i16(payload, 0u, data->temp_valid ? data->acc.max_temp_deci_c : (int16_t)ECU_TEMP_INVALID_DECI_C);
    ecu_put_i16(payload, 2u, data->temp_valid ? data->acc.min_temp_deci_c : (int16_t)ECU_TEMP_INVALID_DECI_C);
    ecu_put_i16(payload, 4u, data->temp_valid ? data->acc.filtered_avg_temp_deci_c : (int16_t)ECU_TEMP_INVALID_DECI_C);
    payload[6] = ecu_max_fan_percent(data);
    payload[7] = ecu_bool_bit(data->temp_warning, 0u) |
                 ecu_bool_bit(data->temp_fan_max, 1u) |
                 ecu_bool_bit(data->temp_charge_stop, 2u) |
                 ecu_bool_bit(data->temp_overtemp_pending, 3u) |
                 ecu_bool_bit(data->overtemp_fault, 4u) |
                 ecu_bool_bit(data->severe_overtemp_fault, 5u) |
                 ecu_bool_bit(data->fan_fault, 6u) |
                 ecu_bool_bit(!data->temp_valid || data->temp_read_fault, 7u);

    return canbus_send(canbus, CAN_ID_STD, AMS_ECU_CAN_ID_THERMAL, payload);
}

static HAL_StatusTypeDef send_ecu_compact_health(canbus_device_t *canbus,
                                                  const app_data_t *data)
{
    if((canbus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t payload[8] = {0};
    payload[0] = data->max_voltage_seg;
    payload[1] = data->max_voltage_cell;
    payload[2] = data->min_voltage_seg;
    payload[3] = data->min_voltage_cell;
    payload[4] = data->max_temp_seg;
    payload[5] = data->max_temp_sensor;
    payload[6] = sat_u8_u32(data->voltage_usable_cell_count);
    payload[7] = sat_u8_u32(data->temp_usable_sensor_count);

    return canbus_send(canbus, CAN_ID_STD, AMS_ECU_CAN_ID_HEALTH, payload);
}

static HAL_StatusTypeDef send_ecu_compact_telemetry(canbus_device_t *canbus,
                                                     const app_data_t *data,
                                                     uint8_t sequence)
{
    HAL_StatusTypeDef ret = HAL_OK;

    ret |= send_ecu_compact_status(canbus, data, sequence);
    ret |= send_ecu_compact_electrical(canbus, data);
    ret |= send_ecu_compact_thermal(canbus, data);
    ret |= send_ecu_compact_health(canbus, data);

    return ret;
}


static HAL_StatusTypeDef send_ecu_ams_status(canbus_device_t *canbus, const app_data_t *data)
{
    HAL_StatusTypeDef ret = HAL_OK;

    ret |= send_ams_packet(canbus,
                           0u,
                           (uint16_t)data->state,
                           (uint16_t)data->air_state,
                           (uint16_t)((int16_t)(data->current * 10.0f)));

    ret |= send_ams_packet(canbus,
                           1u,
                           (uint16_t)data->imd_ok,
                           (uint16_t)data->imd_status,
                           (uint16_t)((int16_t)(data->board.imd.duty * 10.0f)));

    ret |= send_ams_packet(canbus,
                           2u,
                           data->temp_valid ? (uint16_t)((int16_t)(data->max_temp * 10.0f)) :
                                              ECU_TEMP_INVALID_DECI_C,
                           (uint16_t)(data->min_voltage * 1000.0f),
                           (uint16_t)(data->max_voltage * 1000.0f));

    return ret;
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
        for(uint8_t packet = 0u; packet < 6u; packet++)
        {
            uint8_t sensor = (uint8_t)(packet * 3u);
            ret |= send_ams_packet(canbus,
                                   (uint16_t)(28u + (seg * 6u) + packet),
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
                               (uint16_t)(58u + packet),
                               fan_percent_for_ecu(data, fan),
                               fan_percent_for_ecu(data, (uint8_t)(fan + 1u)),
                               (fan + 2u < ECU_FANS) ? fan_percent_for_ecu(data, (uint8_t)(fan + 2u)) : 0u);
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
                                 CHARGER_DISABLE_REASON_TX_FAIL;

    return (reasons & safety_mask) != 0u;
}

static HAL_StatusTypeDef send_logger_summaries(canbus_device_t *canbus,
                                               const app_data_t *data,
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
                 logger_bool_bit(data->air_state, AMS_LOGGER_HEARTBEAT_FLAG_AIR_STATE) |
                 logger_bool_bit(data->imd_ok, AMS_LOGGER_HEARTBEAT_FLAG_IMD_OK) |
                 logger_bool_bit(data->hard_fault, AMS_LOGGER_HEARTBEAT_FLAG_HARD_FAULT) |
                 logger_bool_bit(data->soft_fault, AMS_LOGGER_HEARTBEAT_FLAG_SOFT_FAULT) |
                 logger_bool_bit(data->charger_fault, AMS_LOGGER_HEARTBEAT_FLAG_CHARGER_FAULT) |
                 logger_bool_bit(data->canbus_fault, AMS_LOGGER_HEARTBEAT_FLAG_CANBUS_FAULT) |
                 logger_bool_bit(data->bms_output_inhibit, AMS_LOGGER_HEARTBEAT_FLAG_BMS_OUTPUT_INHIBIT);
    payload[4] = logger_bool_bit(data->voltage_valid, AMS_LOGGER_VALID_FLAG_VOLTAGE_VALID) |
                 logger_bool_bit(data->current_valid, AMS_LOGGER_VALID_FLAG_CURRENT_VALID) |
                 logger_bool_bit(data->temp_valid, AMS_LOGGER_VALID_FLAG_TEMP_VALID) |
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
    logger_put_u16(payload, 0u, sat_u16_scaled((data->total_voltage > 0.0f) ? data->total_voltage : acc->total_volt, 10.0f));
    logger_put_i16(payload, 2u, sat_i16_scaled(data->current, 10.0f));
    logger_put_u16(payload, 4u, acc->min_voltage_mv);
    logger_put_u16(payload, 6u, acc->max_voltage_mv);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_PACK_ELECTRICAL, payload);

    memset(payload, 0, sizeof(payload));
    logger_put_i16(payload, 0u, acc->max_temp_deci_c);
    logger_put_i16(payload, 2u, acc->min_temp_deci_c);
    logger_put_i16(payload, 4u, sat_i16_scaled((data->avg_temp != 0.0f) ? data->avg_temp : acc->avg_temp, 10.0f));
    payload[6] = logger_max_fan_decipct(data);
    payload[7] = logger_bool_bit(data->temp_warning, 0u) |
                 logger_bool_bit(data->temp_fan_max, 1u) |
                 logger_bool_bit(data->temp_charge_stop, 2u) |
                 logger_bool_bit(data->temp_overtemp_pending, 3u) |
                 logger_bool_bit(data->overtemp_fault, 4u) |
                 logger_bool_bit(data->severe_overtemp_fault, 5u);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_TEMP_FAN, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = data->max_voltage_seg;
    payload[1] = data->max_voltage_cell;
    payload[2] = data->min_voltage_seg;
    payload[3] = data->min_voltage_cell;
    payload[4] = sat_u8_u32(data->voltage_usable_cell_count);
    payload[5] = sat_u8_u32(data->voltage_updated_cell_count);
    payload[6] = sat_u8_u32(data->voltage_stale_cell_count);
    payload[7] = sat_u8_u32(data->voltage_pec_fail_cell_count);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_VOLTAGE_HEALTH, payload);

    memset(payload, 0, sizeof(payload));
    payload[0] = data->max_temp_seg;
    payload[1] = data->max_temp_sensor;
    payload[2] = data->min_temp_seg;
    payload[3] = data->min_temp_sensor;
    payload[4] = sat_u8_u32(data->temp_usable_sensor_count);
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
    logger_put_i16(payload, 0u, sat_i16_scaled(data->current, 10.0f));
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
                 logger_bool_bit((AMS_HIL_REPLACE_ADBMS != 0), AMS_LOGGER_ADBMS_DIAG_FLAG_HIL_REPLACE);
    payload[7] = logger_bool_bit(data->hil.meas.fresh != 0u, AMS_LOGGER_HIL_FLAG_MEAS_FRESH) |
                 logger_bool_bit(data->hil.truth.fresh != 0u, AMS_LOGGER_HIL_FLAG_TRUTH_FRESH) |
                 logger_bool_bit(data->hil.summary.fresh != 0u, AMS_LOGGER_HIL_FLAG_SUMMARY_FRESH);
    ret |= send_logger_frame(canbus, AMS_LOGGER_CAN_ID_ADBMS_DIAG, payload);

    const current_sensor_t *cur = &data->board.current_sensor;
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

static HAL_StatusTypeDef send_logger_telemetry(canbus_device_t *canbus, const app_data_t *data)
{
    static uint8_t sequence = 0u;
    HAL_StatusTypeDef ret;

    ret = send_logger_summaries(canbus, data, sequence);
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

TaskHandle_t canbus_task_start(app_data_t *data)
{
    TaskHandle_t handle = NULL;

    if(data == NULL)
    {
        return NULL;
    }

    xTaskCreate(canbus_task_fn, "CANBus Task", AMS_STACK_CAN_WORDS, (void *)data, CAN_PRIO, &handle);
    return handle;
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
    uint8_t slow_div = CAN_ECU_SLOW_DIV;
    uint16_t charger_div = CAN_CHARGER_DIV;

    const uint16_t voltage10x = (uint16_t)(CHARGE_MAX_VOLTAGE * 10.0f);
    const uint16_t current10x = (uint16_t)(CHARGE_MAX_CURRENT * 10.0f);

    for(;;)
    {
        entry = osKernelGetTickCount();
        ret = HAL_OK;
        canbus_poll_errors(canbus, data);

        if(data->can_busoff_fault || data->can_recover_pending)
        {
            data->canbus_fault = true;
            ams_heartbeat_kick(data, AMS_HEARTBEAT_CAN, osKernelGetTickCount());
            osDelayUntil(entry + CAN_ECU_FAST_PERIOD_MS);
            continue;
        }

        ret |= send_ecu_compact_telemetry(canbus, data, ecu_sequence++);
        bool slow_due = false;
        slow_div++;
        if(slow_div >= CAN_ECU_SLOW_DIV)
        {
            slow_div = 0u;
            slow_due = true;
        }

        if(data->state != STATE_CHARGE)
        {
            ccs->target_voltage = 0.0f;
            ccs->target_current = 0.0f;
            ccs->read_voltage   = 0.0f;
            ccs->read_current   = 0.0f;
            ccs->communication_fail = false;
            data->charger_fault = false;

            if(slow_due)
            {
                ret |= send_ecu_ams_status(canbus, data);
                ret |= send_ecu_ams_voltages(canbus, data);
                ret |= send_ecu_ams_temps(canbus, data);
                ret |= send_ecu_ams_fans(canbus, data);
                ret |= send_logger_telemetry(canbus, data);
                ams_heartbeat_kick(data, AMS_HEARTBEAT_LOGGER, osKernelGetTickCount());
                ret |= send_estimator_status(canbus, data);
            }
            canbus_poll_errors(canbus, data);

            canbus_record_task_tx_status(data, ret);
            data->canbus_fault = data->canbus_fault || data->can_busoff_fault;
            ams_heartbeat_kick(data, AMS_HEARTBEAT_CAN, osKernelGetTickCount());
            osDelayUntil(entry + CAN_ECU_FAST_PERIOD_MS);
        }
        else if(data->state == STATE_CHARGE)
        {
            ccs->target_voltage = CHARGE_MAX_VOLTAGE;
            ccs->target_current = CHARGE_MAX_CURRENT;

            charger_div++;
            if(charger_div >= CAN_CHARGER_DIV)
            {
                charger_div = 0u;

                if((osKernelGetTickCount() - ccs->last_rx_tick) > CHARGER_RX_TIMEOUT_MS)
                {
                    ccs->communication_fail = true;
                }

                bool charger_hw_fault = (ccs->hardware_fail      ||
                                         ccs->overtemp_fail      ||
                                         ccs->input_volt_fail    ||
                                         ccs->voltage_sense_fail ||
                                         ccs->communication_fail ||
                                         ccs->tx_fail);

                uint16_t disable_reasons = charger_disable_reasons(data, charger_hw_fault);
                bool disable_charge = (disable_reasons != CHARGER_DISABLE_REASON_NONE);

                ccs->disable_reason_mask = disable_reasons;
                data->charger_fault = charger_hw_fault;
                if(charger_disable_reasons_force_bms_low(disable_reasons))
                {
                    set_bms(0);
                }

                uint8_t can_data[8] = {0};
                can_data[0] = TO_MSB16(voltage10x);
                can_data[1] = TO_LSB16(voltage10x);
                can_data[2] = TO_MSB16(current10x);
                can_data[3] = TO_LSB16(current10x);
                can_data[4] = disable_charge ? CHARGER_CMD_DISABLE : CHARGER_CMD_ENABLE;
                can_data[5] = 0u;
                can_data[6] = 0u;
                can_data[7] = 0u;

                HAL_StatusTypeDef charger_tx_status = canbus_send(canbus, CAN_ID_EXT, CCS_CANBUS_ID, can_data);
                ret |= charger_tx_status;
                if(charger_tx_status == HAL_OK)
                {
                    ccs->tx_fail = false;
                    ccs->last_tx_status = HAL_OK;
                    ccs->tx_count++;
                }
                else
                {
                    ccs->tx_fail = true;
                    ccs->last_tx_status = charger_tx_status;
                    ccs->disable_reason_mask |= CHARGER_DISABLE_REASON_TX_FAIL;
                    ccs->tx_fail_count++;
                    data->charger_fault = true;
                    set_bms(0);
                }

                ret |= send_logger_telemetry(canbus, data);
                ams_heartbeat_kick(data, AMS_HEARTBEAT_LOGGER, osKernelGetTickCount());
            }

            canbus_poll_errors(canbus, data);
            canbus_record_task_tx_status(data, ret);
            data->canbus_fault = data->canbus_fault || data->can_busoff_fault;

            ams_heartbeat_kick(data, AMS_HEARTBEAT_CAN, osKernelGetTickCount());
            osDelayUntil(entry + CAN_ECU_FAST_PERIOD_MS);
        }
        else
        {
            ams_heartbeat_kick(data, AMS_HEARTBEAT_CAN, osKernelGetTickCount());
            osDelayUntil(entry + CAN_ECU_FAST_PERIOD_MS);
        }
    }
}
