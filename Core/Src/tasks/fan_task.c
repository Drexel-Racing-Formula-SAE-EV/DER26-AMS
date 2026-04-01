/*
 * fan_task.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Cassius Garcia
 */
#include "tasks/fan_task.h"
//#include "main.h"

void fan_task_fn(void *argument);

TaskHandle_t fan_task_start(app_data_t *data)
{
    TaskHandle_t handle;
    xTaskCreate(fan_task_fn, "fan task", 128, (void *)data, FAN_PRIO, &handle);
    return handle;
}

void fan_task_fn(void *argument)
{
    app_data_t *data = (app_data_t *) argument;
    uint32_t entry;
    int i;

    /*
    for(i = 0; i < NFANS; i++) set_fan_percent(&data->board.fans[i], 100.0);
    data->fan_state = true;
    osDelay(2000);
    for(i = 0; i < NFANS; i++) set_fan_percent(&data->board.fans[i], 0.0);
    data->fan_state = false;
    */
    float percent = 0.0;
    for(;;)
    {
        entry = osKernelGetTickCount();

        for(i = 0; i < NFANS; i++) set_fan_percent(&data->board.fans[i], percent);
        percent += 10.0;
        if(percent > 100.0) percent = 0.0;

        osDelay(1000);
        /*
        if(data->max_temp > TEMP_THRESH_H)
        {
            for(int i = 0; i < NFANS; i++) set_fan_percent(&data->board.fans[i], 100.0);
            data->fan_state = true;
        }
        else if(data->max_temp < TEMP_THRESH_L)
        {
            for(int i = 0; i < NFANS; i++) set_fan_percent(&data->board.fans[i], 0.0);
            data->fan_state = false;
        }
        */
        //osDelayUntil(entry + (1000 / FAN_FREQ));
    }
}
