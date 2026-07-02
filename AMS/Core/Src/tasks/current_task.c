/*
 * currentt_task.c
 *
 *  Created on: Apr 15, 2024
 *      Author: Justin Nguyen
 */

#include "tasks/current_task.h"

#define CURRENT_TASK_PERIOD_MS (1000u / CURRENT_FREQ)

void current_task_fn(void *argument);

static current_fault_mode_t current_task_fault_mode_from_state(state_t state)
{
    switch(state)
    {
        case STATE_START:
            return CURRENT_FAULT_MODE_PRECHARGE;
        case STATE_CHARGE:
            return CURRENT_FAULT_MODE_CHARGE;
        case STATE_DISCARGE:
            return CURRENT_FAULT_MODE_DRIVE;
        case STATE_BALANCE:
        case STATE_NULL:
        case STATE_ERROR:
        default:
            return CURRENT_FAULT_MODE_IDLE;
    }
}

static void current_task_publish_fault_state(app_data_t *app_data)
{
    current_fault_state_t *fault = &app_data->current_fault_state;

    app_data->current_sensor_fault = fault->sensor_fault;
    app_data->current_overcurrent_warning = fault->warning;
    app_data->current_overcurrent_pending = fault->pending;
    app_data->current_overcurrent_fault = fault->confirmed;
    app_data->current_fault_latched = fault->latched;
    app_data->current_fault_reason = fault->reason;
    app_data->current_fault_latched_reason = fault->latched_reason;
    app_data->current_fault_mode = fault->mode;

    app_data->current_fault = (app_data->current_sensor_fault ||
                               app_data->current_overcurrent_fault ||
                               app_data->current_fault_latched);
}

TaskHandle_t current_task_start(app_data_t *data)
{
    TaskHandle_t handle = NULL;

    if(data == NULL)
    {
        return NULL;
    }

    xTaskCreate(current_task_fn, "current task", 160, (void *)data, CUR_PRIO, &handle);
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

        if(current_sensor_read_adc(current_sensor))
        {
            (void)current_sensor_convert(current_sensor);
        }

        app_data->current_valid = current_sensor->current_valid;
        app_data->current_selected_range = current_sensor->selected_range;
        app_data->current_meas_reason = current_sensor->reason;

        if(current_sensor->current_valid)
        {
            app_data->current = current_sensor->current;
        }

        current_fault_update(&app_data->current_fault_state,
                             current_task_fault_mode_from_state(app_data->state),
                             app_data->current,
                             current_sensor->current_valid,
                             current_sensor->reason,
                             CURRENT_TASK_PERIOD_MS);
        current_task_publish_fault_state(app_data);

        if(app_data->current_fault)
        {
            set_bms(0);
        }

        osDelayUntil(entry + CURRENT_TASK_PERIOD_MS);
    }
}
