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
    TaskHandle_t handle;
    xTaskCreate(current_task_fn, "current task", 128, (void *)data, CUR_PRIO, &handle);
    return handle;
}

void current_task_fn(void *argument)
{
    app_data_t *app_data = (app_data_t *) argument;
    current_sensor_t *current_sensor = &app_data->board.current_sensor;
    uint32_t entry ;

    for(;;)
    {
        entry = osKernelGetTickCount();

        stm32f767z_adc_switch_channel(current_sensor->hadc_high, current_sensor->channel_high);
        current_sensor->count_high = stm32f767z_adc_read(current_sensor->hadc_high);
        stm32f767z_adc_switch_channel(current_sensor->hadc_low, current_sensor->channel_low);
        current_sensor->count_low  = stm32f767z_adc_read(current_sensor->hadc_low);

        current_sensor_convert(current_sensor);

        app_data->current = current_sensor->current;

        osDelayUntil(entry + (1000 / CURRENT_FREQ));
    }
}
