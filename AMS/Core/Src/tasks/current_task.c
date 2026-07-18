/*
 * currentt_task.c
 *
 *  Created on: Apr 15, 2024
 *      Author: Justin Nguyen
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#include "tasks/current_task.h"

#include <math.h>

#define CURRENT_TASK_PERIOD_MS (1000u / CURRENT_FREQ)

void current_task_fn(void *argument);

static StaticTask_t current_task_tcb;
static StackType_t current_task_stack[AMS_STACK_CURRENT_WORDS];
static TaskHandle_t current_task_handle = NULL;

static uint32_t current_task_abs_deciamps(float current_a)
{
    double scaled;

    if(!isfinite(current_a) || (current_a <= 0.0f))
    {
        return 0u;
    }

    scaled = (double)current_a * 10.0;
    if(scaled >= (double)UINT32_MAX)
    {
        return UINT32_MAX;
    }

    return (uint32_t)(scaled + 0.5);
}

static current_fault_mode_t current_task_fault_mode_from_state(state_t state)
{
    switch(state)
    {
        case STATE_START:
            /* STATE_START is software initialization, not control of the
             * hardware precharge circuit.  The conservative precharge limits
             * are retained here only to detect unexpected pack current while
             * BMS_OK and the shutdown loop should still be open. */
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

static void current_task_publish_fault_state(app_data_t *app_data,
                                             const current_fault_state_t *fault,
                                             const current_sensor_t *sensor,
                                             float published_current)
{
    if((app_data == NULL) || (fault == NULL) || (sensor == NULL))
    {
        return;
    }

    /* The supervisor runs at a higher priority.  Publish the measurement,
     * state-machine internals, and flattened gates as one snapshot so it can
     * never combine fields from two different current samples. */
    taskENTER_CRITICAL();
    app_data->current_valid = sensor->current_valid;
    app_data->current_selected_range = sensor->selected_range;
    app_data->current_meas_reason = sensor->reason;
    app_data->current = published_current;
    app_data->current_fault_state = *fault;
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
    taskEXIT_CRITICAL();
}

TaskHandle_t current_task_start(app_data_t *data)
{
    if(data == NULL)
    {
        return NULL;
    }

    if(current_task_handle == NULL)
    {
        current_task_handle = xTaskCreateStatic(current_task_fn,
                                                "current task",
                                                AMS_STACK_CURRENT_WORDS,
                                                (void *)data,
                                                CUR_PRIO,
                                                current_task_stack,
                                                &current_task_tcb);
    }

    return current_task_handle;
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

        bool current_was_latched = app_data->current_fault_latched;
        current_fault_reason_t current_prev_latched_reason = app_data->current_fault_latched_reason;
        float published_current = current_sensor->current_valid ?
                                  current_sensor->current : app_data->current;
        current_fault_state_t next_fault = app_data->current_fault_state;

        current_fault_update(&next_fault,
                             current_task_fault_mode_from_state(app_data->state),
                             published_current,
                             current_sensor->current_valid,
                             current_sensor->reason,
                             CURRENT_TASK_PERIOD_MS);
        current_task_publish_fault_state(app_data,
                                         &next_fault,
                                         current_sensor,
                                         published_current);

        if(app_data->current_fault_latched &&
           (!current_was_latched ||
            (current_prev_latched_reason != app_data->current_fault_latched_reason)))
        {
            ams_fault_log_event(AMS_FAULT_LOG_CURRENT_LATCH,
                                (uint16_t)app_data->current_fault_latched_reason,
                                current_task_abs_deciamps(app_data->current_fault_state.abs_current_a),
                                (uint32_t)app_data->current_fault_mode);
        }

        if((!app_data->current_valid) || app_data->current_fault)
        {
            set_bms(0);
        }

        ams_heartbeat_kick(app_data, AMS_HEARTBEAT_CURRENT, osKernelGetTickCount());
        osDelayUntil(entry + CURRENT_TASK_PERIOD_MS);
    }
}
