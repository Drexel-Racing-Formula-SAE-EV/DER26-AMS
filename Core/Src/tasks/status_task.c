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

/**
 * @brief CANBus task function
 *
 * @param arg App_data struct pointer converted to void pointer
 */
void canbus_task_fn(void *arg);

TaskHandle_t canbus_task_start(app_data_t *data)
{
    TaskHandle_t handle;
    xTaskCreate(canbus_task_fn, "CANBus Task", 128, (void *)data, CAN_PRIO, &handle);
    return handle;
}

void canbus_task_fn(void *arg)
{
    app_data_t *data = (app_data_t *)arg;

    canbus_device_t *canbus = &data->board.canbus;

    CAN_TxHeaderTypeDef *tx_header = &canbus->tx_header;
    HAL_StatusTypeDef ret;
    canbus_packet_t can_packet;
    uint32_t entry;
    uint16_t packet;
    uint8_t can_data[8] = {0};

    tx_header->StdId = ECU_CANBUS_ID;



    for(;;)
    {
    	entry = osKernelGetTickCount();
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
    	xTaskNotify(data->canbus_task, CANBUS_STATUS, eSetBits);

    	packet = 1;
    	can_data[0] = TO_MSB16(packet);
    	can_data[1] = TO_LSB16(packet);
    	can_data[2] = TO_MSB16(data->imd_ok);
    	can_data[3] = TO_LSB16(data->imd_ok);
    	can_data[4] = TO_MSB16(data->imd_status);
    	can_data[5] = TO_LSB16(data->imd_status);
    	can_data[6] = TO_MSB16((int16_t)(data->board.imd.duty * 10.0));
    	can_data[7] = TO_LSB16((int16_t)(data->board.imd.duty * 10.0));
    	xTaskNotify(data->canbus_task, CANBUS_STATUS, eSetBits);

    	packet = 2;
		can_data[0] = TO_MSB16(packet);
		can_data[1] = TO_LSB16(packet);
		can_data[2] = TO_MSB16((int16_t)(data->max_temp * 10.0));
		can_data[3] = TO_LSB16((int16_t)(data->max_temp * 10.0));
		can_data[4] = TO_MSB16((int16_t)(data->min_voltage * 10.0));
		can_data[5] = TO_LSB16((int16_t)(data->min_voltage * 10.0));
		can_data[6] = TO_MSB16((int16_t)(data->max_voltage * 10.0));
		can_data[7] = TO_LSB16((int16_t)(data->max_voltage * 10.0));
		xTaskNotify(data->canbus_task, CANBUS_STATUS, eSetBits);

    	packet = 3;
    	can_data[0] = TO_MSB16(packet);
    	can_data[1] = TO_LSB16(packet);
    	can_data[2] = TO_MSB16((int16_t)(data->total_voltage * 10.0));
    	can_data[3] = TO_LSB16((int16_t)(data->total_voltage * 10.0));
    	can_data[4] = TO_MSB16((int16_t)(data->avg_temp * 10.0));
    	can_data[5] = TO_LSB16((int16_t)(data->avg_temp * 10.0));
    	can_data[6] = TO_MSB16(data->fan_state);
    	can_data[7] = TO_LSB16(data->fan_state);
    	xTaskNotify(data->canbus_task, CANBUS_STATUS, eSetBits);

    	packet = 4;
    	can_data[0] = TO_MSB16(packet);
    	can_data[1] = TO_LSB16(packet);
    	can_data[2] = TO_MSB16(data->hard_fault);
    	can_data[3] = TO_LSB16(data->hard_fault);
    	can_data[4] = TO_MSB16(data->soft_fault);
    	can_data[5] = TO_LSB16(data->soft_fault);
    	can_data[6] = TO_MSB16(data->fan_fault);
    	can_data[7] = TO_LSB16(data->fan_fault);
    	xTaskNotify(data->canbus_task, CANBUS_STATUS, eSetBits);

    	packet = 5;
    	can_data[0] = TO_MSB16(packet);
    	can_data[1] = TO_LSB16(packet);
    	can_data[2] = TO_MSB16(data->cli_fault);
    	can_data[3] = TO_LSB16(data->cli_fault);
    	can_data[4] = TO_MSB16(data->canbus_fault);
    	can_data[5] = TO_LSB16(data->canbus_fault);
    	can_data[6] = TO_MSB16(data->current_fault);
    	can_data[7] = TO_LSB16(data->current_fault);
    	xTaskNotify(data->canbus_task, CANBUS_STATUS, eSetBits);

        osDelayUntil(entry + (1000 / CAN_FREQ));

    }
}

