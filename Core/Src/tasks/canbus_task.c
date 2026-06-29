/**
 * @file canbus_task.c
 * @author Cole Bardin (cab572@drexel.edu)
 * @brief
 * @version 0.1
 * @date 2024-03-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "tasks/canbus_task.h"
#include "ext_drivers/canbus.h"
#include "ext_drivers/charger.h"
#include "app.h"

#include <math.h>

void canbus_task_fn(void *arg);

#define ECU_SEG_CELLS 15u
#define ECU_SEG_TEMPS 17u
#define ECU_FANS      10u
#define CAN_TX_TIMEOUT_TICKS 10u

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
       (seg >= (uint8_t)data->acc.smb.num_ics) ||
       (cell >= NCELLS))
    {
        return 0u;
    }

    int16_t code = data->acc.smb.ics[seg].cell.c_codes[cell];
    if((code == 0) || (code == INT16_MIN))
    {
        return 0u;
    }

    float volts = ((float)code + 10000.0f) * 0.000150f;
    if((volts < 0.5f) || (volts > 5.0f))
    {
        return 0u;
    }

    return (uint16_t)(volts * 1000.0f);
}

static uint16_t temp_deci_c_for_ecu(const app_data_t *data, uint8_t seg, uint8_t sensor)
{
    if((data == NULL) ||
       (seg >= (uint8_t)data->acc.smb.num_ics) ||
       (sensor >= NTEMPS))
    {
        return 0u;
    }

    int16_t raw = data->acc.smb.ics[seg].temp.raw[sensor];
    if((raw == 0) || (raw == -1) || (raw == INT16_MIN))
    {
        return 0u;
    }

    float voltage = ((float)raw + 10000.0f) * 0.000150f;
    if((voltage <= 0.0f) || (voltage >= 5.0f))
    {
        return 0u;
    }

    float resistance = 10000.0f * (5.0f - voltage) / voltage;
    if(resistance <= 0.0f)
    {
        return 0u;
    }

    float x = logf(resistance / 10000.0f);
    float temp_c = (1.0f / (3.354016435e-3f + 2.565235509e-4f * x)) - 273.15f;
    if((temp_c < -40.0f) || (temp_c > 150.0f))
    {
        return 0u;
    }

    return (uint16_t)((int16_t)(temp_c * 10.0f));
}

static uint16_t fan_percent_for_ecu(const app_data_t *data, uint8_t fan)
{
    if((data == NULL) || (fan >= NFANS))
    {
        return 0u;
    }

    return (uint16_t)(data->board.fans[fan].duty_cycle * 10.0f);
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
                           (uint16_t)((int16_t)(data->max_temp * 10.0f)),
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

TaskHandle_t canbus_task_start(app_data_t *data)
{
    TaskHandle_t handle = NULL;

    if(data == NULL)
    {
        return NULL;
    }

    xTaskCreate(canbus_task_fn, "CANBus Task", 256, (void *)data, CAN_PRIO, &handle);
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

    const uint16_t voltage10x = (uint16_t)(CHARGE_MAX_VOLTAGE * 10.0f);
    const uint16_t current10x = (uint16_t)(CHARGE_MAX_CURRENT * 10.0f);

    for(;;)
    {
        entry = osKernelGetTickCount();
        ret = HAL_OK;

        if(data->state != STATE_CHARGE)
        {
            ccs->target_voltage = 0.0f;
            ccs->target_current = 0.0f;
            ccs->read_voltage   = 0.0f;
            ccs->read_current   = 0.0f;
            ccs->communication_fail = false;
            data->charger_fault = false;

            ret |= send_ecu_ams_status(canbus, data);
            ret |= send_ecu_ams_voltages(canbus, data);
            ret |= send_ecu_ams_temps(canbus, data);
            ret |= send_ecu_ams_fans(canbus, data);

            data->canbus_fault = (ret != HAL_OK);
            osDelayUntil(entry + (1000 / CAN_FREQ));
        }
        else if(data->state == STATE_CHARGE)
        {
            ccs->target_voltage = CHARGE_MAX_VOLTAGE;
            ccs->target_current = CHARGE_MAX_CURRENT;

            if((osKernelGetTickCount() - ccs->last_rx_tick) > 5000u)
            {
                ccs->communication_fail = true;
            }

            bool charger_hw_fault = (ccs->hardware_fail      ||
                                     ccs->overtemp_fail      ||
                                     ccs->input_volt_fail    ||
                                     ccs->voltage_sense_fail ||
                                     ccs->communication_fail);

            bool disable_charge = (charger_hw_fault ||
                                   data->hard_fault ||
                                   data->voltage_fault ||
                                   data->temp_fault ||
                                   !data->bms_state);

            data->charger_fault = charger_hw_fault;
            if(disable_charge)
            {
                set_bms(0);
            }

            uint8_t can_data[8] = {0};
            can_data[0] = TO_MSB16(voltage10x);
            can_data[1] = TO_LSB16(voltage10x);
            can_data[2] = TO_MSB16(current10x);
            can_data[3] = TO_LSB16(current10x);
            can_data[4] = disable_charge ? 1u : 0u;
            can_data[5] = 0u;
            can_data[6] = 0u;
            can_data[7] = 0u;

            ret = canbus_send(canbus, CAN_ID_EXT, CCS_CANBUS_ID, can_data);
            if(ret == HAL_OK)
            {
                ccs->tx_count++;
            }
            data->canbus_fault = (ret != HAL_OK);

            osDelayUntil(entry + 1000u);
        }
        else
        {
            osDelayUntil(entry + (1000 / CAN_FREQ));
        }
    }
}
