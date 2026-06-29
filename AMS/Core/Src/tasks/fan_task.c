/*
 * fan_task.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Cassius Garcia
 */
#include "tasks/fan_task.h"

void fan_task_fn(void *argument);

TaskHandle_t fan_task_start(app_data_t *data)
{
    TaskHandle_t handle = NULL;

    if(data == NULL)
    {
        return NULL;
    }

    xTaskCreate(fan_task_fn, "fan task", 128, (void *)data, FAN_PRIO, &handle);
    return handle;
}

void fan_task_fn(void *argument)
{
    app_data_t *data = (app_data_t *) argument;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    uint32_t entry;
    float percent = 0.0f;

    for(;;)
    {
        entry = osKernelGetTickCount();

        if(data->max_temp >= TEMP_THRESH_H)
        {
            percent = 100.0f;
            data->fan_state = true;
        }
        else if(data->max_temp <= TEMP_THRESH_L)
        {
            percent = 0.0f;
            data->fan_state = false;
        }
        /* Between thresholds, keep the previous fan state for hysteresis. */

        data->fan_fault = false;
        for(int i = 0; i < NFANS; i++)
        {
            if(set_fan_percent(&data->board.fans[i], percent) != 0)
            {
                data->fan_fault = true;
            }
        }

        osDelayUntil(entry + (1000 / FAN_FREQ));
    }
}
