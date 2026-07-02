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
    TaskHandle_t handle = NULL;

    if(data == NULL)
    {
        return NULL;
    }

    xTaskCreate(adbms_task_fn, "adbms task", 1024, (void *)data, ADBMS_PRIO, &handle);
    return handle;
}

void adbms_task_fn(void *argument)
{
	app_data_t *data = (app_data_t *) argument;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

	accumulator_t *acc = &data->acc;
	uint32_t entry;

	for(;;)
	{
        entry = osKernelGetTickCount();

//
        // Turn off balancing before reading
        accumulator_clear_balance(acc);

        // Wait for cell voltages to recover from discharge load
        osDelay(100);

        // Now read voltages with no load on cells
        accumulator_read_volt(acc);
        accumulator_update_voltage_stats(acc);
        data->max_voltage = acc->max_volt;
        data->min_voltage = acc->min_volt;
        data->total_voltage = acc->total_volt;

        bool voltage_read_invalid = ((acc->valid_voltage_count == 0u) ||
                                     (data->min_voltage <= 0.0f) ||
                                     (data->max_voltage <= 0.0f) ||
                                     (data->total_voltage <= 0.0f));

        data->voltage_fault = (voltage_read_invalid ||
                               (data->min_voltage < UNDERVOLT) ||
                               (data->max_voltage > OVERVOLT));
        if(data->voltage_fault)
        {
            set_bms(0);
        }

        accumulator_read_temp(acc);

        accumulator_update_temp_stats(acc);
        data->max_temp = acc->max_temp;
        data->avg_temp = acc->avg_temp;

        bool temp_read_invalid = (acc->valid_temp_count == 0u);
        data->temp_fault = (temp_read_invalid || (data->max_temp > TEMP_THRESH_H));

        if(data->temp_fault)
        {
            set_bms(0);
        }

        bool bms_ok_ready = (!data->voltage_fault &&
                             !data->temp_fault &&
                             !data->fuse_fault &&
                             !data->charger_fault &&
                             data->current_valid &&
                             !data->current_fault);
        set_bms(bms_ok_ready);

        // Resume balancing only after both voltage and temperature checks are clean.
        if((data->state == STATE_CHARGE) &&
           !data->voltage_fault &&
           !data->temp_fault &&
           data->bms_state)
        {
            accumulator_set_balance(acc);
        }
        else
        {
            accumulator_clear_balance(acc);
        }

        osDelayUntil(entry + (1000 / ADBMS_FREQ));
	}
}

