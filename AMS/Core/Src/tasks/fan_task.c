/*
 * fan_task.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Cassius Garcia
 */
#include "tasks/fan_task.h"

#include <math.h>

void fan_task_fn(void *argument);

static float fan_percent_from_temp(const app_data_t *data)
{
    float span;

    if((data == NULL) ||
       data->temp_fault ||
       (data->acc.valid_temp_count == 0u) ||
       !isfinite(data->max_temp))
    {
        return 100.0f;
    }

    if(data->max_temp <= TEMP_THRESH_L)
    {
        return 0.0f;
    }

    if(data->max_temp >= TEMP_THRESH_H)
    {
        return 100.0f;
    }

    span = (float)(TEMP_THRESH_H - TEMP_THRESH_L);
    if(span <= 0.0f)
    {
        return 100.0f;
    }

    return ((data->max_temp - (float)TEMP_THRESH_L) * 100.0f) / span;
}

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

        percent = fan_percent_from_temp(data);
        data->fan_state = (percent > 0.0f);

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
