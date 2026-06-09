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

        accumulator_read_volt(acc);


        adbms6830_update_cell_voltage_limits(acc);
        data->max_voltage = (acc->max_volt);
        data->min_voltage = (acc->min_volt);

        data->total_voltage = (acc->apm.vbat[0] + acc->apm.vbat[1]) / 2.0f;

        if(data->min_voltage < UNDERVOLT && data->min_voltage != 0){
			set_bms(0);
		}

//        taskENTER_CRITICAL();
//        accumulator_read_temp(acc);
//        taskEXIT_CRITICAL();
////
//        accumulator_update_temp_stats(acc);
//        data->max_temp = (acc->max_temp);
//        data->avg_temp = (acc->avg_temp);
//
//        if(data->max_temp > TEMP_THRESH_H){
//        	set_bms(0);
//        }

        osDelayUntil(entry + (1000 / ADBMS_FREQ));
	}
}



