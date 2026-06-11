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

void canbus_task_fn(void *arg);

TaskHandle_t canbus_task_start(app_data_t *data)
{
    TaskHandle_t handle;
    xTaskCreate(canbus_task_fn, "CANBus Task", 256, (void *)data, CAN_PRIO, &handle);
    return handle;
}

void canbus_task_fn(void *arg)
{
    app_data_t *data = (app_data_t *)arg;

    canbus_device_t *canbus = &data->board.canbus;
    charger_t *ccs = &data->board.charger;
    CAN_TxHeaderTypeDef *tx_header = &canbus->tx_header;
    HAL_StatusTypeDef ret;
    uint32_t entry;
    uint16_t packet;
    uint8_t can_data[8] = {0};

    uint16_t voltage10x = (uint16_t)(CHARGE_MAX_VOLTAGE * 10.0f);
    uint16_t current10x = (uint16_t)(CHARGE_MAX_CURRENT * 10.0f);
    bool disable_charge = false;

    tx_header->StdId = ECU_CANBUS_ID;

    for(;;)
    {
        entry = osKernelGetTickCount();

        if(data->state == STATE_DISCARGE)
        {
            tx_header->IDE   = CAN_ID_STD;
            tx_header->StdId = ECU_CANBUS_ID;
            ccs->target_voltage = 0.0f;
            ccs->target_current = 0.0f;
            ccs->read_voltage   = 0.0f;
            ccs->read_current   = 0.0f;

            // TODO: turn into huge packet index like ECU
            ret = 0;
            packet = 0;
            can_data[0] = TO_MSB16(packet);
            can_data[1] = TO_LSB16(packet);
            can_data[2] = TO_MSB16(data->state);
            can_data[3] = TO_LSB16(data->state);
            can_data[4] = TO_MSB16(data->air_state);
            can_data[5] = TO_LSB16(data->air_state);
            can_data[6] = TO_MSB16((int16_t)(data->current * 10.0));
            can_data[7] = TO_LSB16((int16_t)(data->current * 10.0));
            ret = HAL_CAN_AddTxMessage(canbus->hcan, tx_header, can_data, &canbus->tx_mailbox);
            data->canbus_fault = ret;

            packet = 1;
            can_data[0] = TO_MSB16(packet);
            can_data[1] = TO_LSB16(packet);
            can_data[2] = TO_MSB16(data->imd_ok);
            can_data[3] = TO_LSB16(data->imd_ok);
            can_data[4] = TO_MSB16(data->imd_status);
            can_data[5] = TO_LSB16(data->imd_status);
            can_data[6] = TO_MSB16((int16_t)(data->board.imd.duty * 10.0));
            can_data[7] = TO_LSB16((int16_t)(data->board.imd.duty * 10.0));
            ret = HAL_CAN_AddTxMessage(canbus->hcan, tx_header, can_data, &canbus->tx_mailbox);
            data->canbus_fault = ret;

            packet = 2;
            can_data[0] = TO_MSB16(packet);
            can_data[1] = TO_LSB16(packet);
            can_data[2] = TO_MSB16((int16_t)(data->max_temp * 10.0));
            can_data[3] = TO_LSB16((int16_t)(data->max_temp * 10.0));
            can_data[4] = TO_MSB16((int16_t)(data->min_voltage * 10.0));
            can_data[5] = TO_LSB16((int16_t)(data->min_voltage * 10.0));
            can_data[6] = TO_MSB16((int16_t)(data->max_voltage * 10.0));
            can_data[7] = TO_LSB16((int16_t)(data->max_voltage * 10.0));
            ret = HAL_CAN_AddTxMessage(canbus->hcan, tx_header, can_data, &canbus->tx_mailbox);
            data->canbus_fault = ret;

            // TODO: write out all the other packets!
            osDelayUntil(entry + (1000 / CAN_FREQ));
        }
        else if(data->state == STATE_CHARGE)
        {
            tx_header->IDE   = CAN_ID_EXT;
            tx_header->ExtId = CCS_CANBUS_ID;
            ccs->target_voltage = CHARGE_MAX_VOLTAGE;
            ccs->target_current = CHARGE_MAX_CURRENT;

            // Watchdog: if no message received in 5 seconds, flag comms failure
            if((osKernelGetTickCount() - ccs->last_rx_tick) > 5000)
            {
                ccs->communication_fail = true;
            }

            if(ccs->hardware_fail   ||
               ccs->overtemp_fail   ||
               ccs->input_volt_fail ||
               ccs->communication_fail
			   )
            {
                disable_charge = true;
                data->charger_fault = true;

                set_bms(0);
            }

            can_data[0] = TO_MSB16(voltage10x);
            can_data[1] = TO_LSB16(voltage10x);
            can_data[2] = TO_MSB16(current10x);
            can_data[3] = TO_LSB16(current10x);
            can_data[4] = disable_charge;
            can_data[5] = 0;
            can_data[6] = 0;
            can_data[7] = 0;
            ret = HAL_CAN_AddTxMessage(canbus->hcan, tx_header, can_data, &canbus->tx_mailbox);
            ccs->tx_count++;
            data->canbus_fault = ret;

            osDelayUntil(entry + 1000);
        }
        else
        {
            osDelayUntil(entry + (1000 / CAN_FREQ));
        }
    }
}

