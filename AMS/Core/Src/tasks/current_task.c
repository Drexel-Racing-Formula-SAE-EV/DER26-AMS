/*
 * currentt_task.c
 *
 *  Created on: Apr 15, 2024
 *      Author: Justin Nguyen
 */

#include "tasks/current_task.h"

void current_task_fn(void *argument);

TaskHandle_t current_task_start(app_data_t *data)
{
    TaskHandle_t handle = NULL;

    if(data == NULL)
    {
        return NULL;
    }

    xTaskCreate(current_task_fn, "current task", 128, (void *)data, CUR_PRIO, &handle);
    return handle;
}

void current_task_fn(void *argument)
{
    app_data_t *app_data = (app_data_t *) argument;
    if(app_data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    current_sensor_t *current_sensor = &app_data->board.current_sensor;
    uint32_t entry;

    for(;;)
    {
        entry = osKernelGetTickCount();

        bool current_ok = current_sensor_read_adc(current_sensor);
        app_data->current_fault = !current_ok;

        if(current_ok)
        {
            (void)current_sensor_convert(current_sensor);
            app_data->current = current_sensor->current;
        }

        osDelayUntil(entry + (1000 / CURRENT_FREQ));
    }
}
