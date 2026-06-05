/*
 * adbms_task.h
 *
 *  Created on: June 3, 2025
 *      Author: Cole Bardin
 */

#include <tasks/adbms_task.h>

void adbms_task_fn(void *argument);

TaskHandle_t adbms_task_start(app_data_t *data)
{
	TaskHandle_t handle;
	xTaskCreate(adbms_task_fn, "adbms task", 1024, (void *)data, ADBMS_PRIO, &handle);
	return handle;
}

void adbms_task_fn(void *argument)
{
	app_data_t *data = (app_data_t *) argument;
	accumulator_t *acc = &data->acc;
	uint32_t entry;

	for(;;)
	{
        entry = osKernelGetTickCount();

//
        taskENTER_CRITICAL();
        accumulator_read_volt(acc);
        taskEXIT_CRITICAL();

        data->total_voltage = (acc->apm.vbat[0] + acc->apm.vbat[1]) / 2.0f;

//        taskENTER_CRITICAL();
//        accumulator_read_temp(acc);
//        taskEXIT_CRITICAL();

        osDelayUntil(entry + (1000 / ADBMS_FREQ));
	}
}



