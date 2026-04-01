/*
*   imd_task.h
*   Created: 3/25/2024
*   Author: Brendan Hoag
*   Purpose: Device driver task for IR151-3204 ground fault monitoring system
*   datasheet: https://www.benderinc.com/products/ground-fault-monitoring-ungrounded/isometer-ir155-03-04-series/
*/

#include "tasks/imd_task.h"
/*
* function: imd_task_fn
* ---------------------
* The actual function the task will run
* - Reads from data pins
* - Update status, frequency, duty cycle
* - Call shutdown on error -> might be job of super manager task rather than imd
*
* argument - a void pointer to be cast back to an app_data_t to get appropriate data from
*/
void imd_task_fn(void *argument);


TaskHandle_t imd_task_start(app_data_t *data)
{
    TaskHandle_t handle;
    // todo: appropriate task priority
    xTaskCreate(imd_task_fn, "imd task", 128, (void *) data, IMD_PRIO, &handle);
    return handle;
}

// todo: fix channel #, not sure which channel this should exist on atm
void imd_task_fn(void *argument){
    app_data_t *data;
    imd_t *imd;

    data = (app_data_t *) argument;
    imd = &data->board.imd;
    uint32_t entry;

    for(;;)
    {
        entry = osKernelGetTickCount();
        data->imd_status = imd->status;
        data->imd_ok = imd->OK_HS;
        osDelayUntil(entry + (1000 / IMD_FREQ));
    }

}
