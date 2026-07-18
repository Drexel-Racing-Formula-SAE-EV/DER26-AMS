/*
*   imd_task.h
*   Created: 3/25/2024
*   Author: Brendan Hoag
*   Modified by: Mahad Faisal (major firmware updates, 2026)
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

static StaticTask_t imd_task_tcb;
static StackType_t imd_task_stack[AMS_STACK_IMD_WORDS];
static TaskHandle_t imd_task_handle = NULL;


TaskHandle_t imd_task_start(app_data_t *data)
{
    if(data == NULL)
    {
        return NULL;
    }

    if(imd_task_handle == NULL)
    {
        imd_task_handle = xTaskCreateStatic(imd_task_fn,
                                            "imd task",
                                            AMS_STACK_IMD_WORDS,
                                            (void *)data,
                                            IMD_PRIO,
                                            imd_task_stack,
                                            &imd_task_tcb);
    }

    return imd_task_handle;
}

bool imd_task_update(app_data_t *data, uint32_t now)
{
    bool valid;
    bool ok;
    imd_status_t status;

    if(data == NULL)
    {
        return false;
    }

    imd_t *imd = &data->board.imd;
    valid = (imd_read_at(imd, now) == 0);
    status = valid ? imd->status : IMD_UNKNOWN;
    ok = valid && imd->OK_HS && (status == IMD_NORMAL);

    taskENTER_CRITICAL();
    data->imd_valid = valid;
    data->imd_status = status;
    data->imd_ok = ok;
    data->imd_fault = !ok;
    if(valid)
    {
        data->imd_last_valid_tick = now;
    }
    taskEXIT_CRITICAL();

    if(!ok)
    {
        set_bms(false);
    }

    ams_heartbeat_kick(data, AMS_HEARTBEAT_IMD, now);
    return ok;
}

void imd_task_fn(void *argument)
{
    app_data_t *data;

    data = (app_data_t *) argument;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    uint32_t entry;

    for(;;)
    {
    	entry = osKernelGetTickCount();
		(void)imd_task_update(data, entry);
    	osDelayUntil(entry + (1000 / IMD_FREQ));
    }

}
