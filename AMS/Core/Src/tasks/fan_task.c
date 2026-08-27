/*
 * fan_task.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Cassius Garcia
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */
#include "tasks/fan_task.h"

#include "ext_drivers/fans.h"

#include <math.h>

void fan_task_fn(void *argument);

static StaticTask_t fan_task_tcb;
static StackType_t fan_task_stack[AMS_STACK_FAN_WORDS];
static TaskHandle_t fan_task_handle = NULL;

#define FAN_MIN_COMMAND_PERCENT        25.0f
#define FAN_CHARGE_WARM_PERCENT       35.0f
#define FAN_OFF_HYSTERESIS_C           3.0f

static float fan_temp_for_control(const app_data_t *data)
{
    if(data == NULL)
    {
        return NAN;
    }

    /* Use the live validated max for fan actuation. Filtered temperature is
     * exported for display/logging, not for delaying cooling response. */
    return data->max_temp;
}

static float fan_percent_from_temp(const app_data_t *data, uint8_t *reason_out)
{
    float span;
    float max_temp;
    float percent;
    bool was_on;

    if(reason_out != NULL)
    {
        *reason_out = FAN_CONTROL_REASON_OFF_COOL;
    }

    if(data == NULL)
    {
        if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_TEMP_INVALID;
        return 100.0f;
    }

    if(!data->temp_valid ||
       data->temp_read_fault ||
       (data->temp_usable_sensor_count == 0u) ||
       !isfinite(data->max_temp))
    {
        if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_TEMP_INVALID;
        return 100.0f;
    }

    if(data->temp_fault)
    {
        if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_TEMP_FAULT;
        return 100.0f;
    }

    max_temp = fan_temp_for_control(data);
    if(!isfinite(max_temp))
    {
        if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_TEMP_INVALID;
        return 100.0f;
    }

    if(data->temp_fan_max || (max_temp >= TEMP_FAN_MAX_C))
    {
        if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_MAX_TEMP;
        return 100.0f;
    }

    span = (float)(TEMP_FAN_MAX_C - TEMP_FAN_RAMP_START_C);
    if(span <= 0.0f)
    {
        if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_MAX_TEMP;
        return 100.0f;
    }

    was_on = isfinite(data->fan_command_percent) &&
             (data->fan_command_percent > 0.5f) &&
             (data->fan_control_reason != FAN_CONTROL_REASON_MAX_TEMP) &&
             (data->fan_control_reason != FAN_CONTROL_REASON_TEMP_INVALID) &&
             (data->fan_control_reason != FAN_CONTROL_REASON_TEMP_FAULT) &&
             (data->fan_control_reason != FAN_CONTROL_REASON_DRIVER_FAULT);

    if(max_temp <= (TEMP_FAN_RAMP_START_C - FAN_OFF_HYSTERESIS_C))
    {
        if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_OFF_COOL;
        return 0.0f;
    }

    if(max_temp <= TEMP_FAN_RAMP_START_C)
    {
        if(was_on)
        {
            if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_MIN_HYSTERESIS;
            return FAN_MIN_COMMAND_PERCENT;
        }

        if((data->state == STATE_CHARGE) && (max_temp >= (TEMP_FAN_RAMP_START_C - FAN_OFF_HYSTERESIS_C)))
        {
            if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_CHARGE_WARM;
            return FAN_CHARGE_WARM_PERCENT;
        }

        if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_OFF_COOL;
        return 0.0f;
    }

    percent = ((max_temp - (float)TEMP_FAN_RAMP_START_C) * 100.0f) / span;
    if(percent < FAN_MIN_COMMAND_PERCENT)
    {
        percent = FAN_MIN_COMMAND_PERCENT;
    }

    if((data->state == STATE_CHARGE) && (percent < FAN_CHARGE_WARM_PERCENT))
    {
        percent = FAN_CHARGE_WARM_PERCENT;
        if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_CHARGE_WARM;
        return percent;
    }

    if(reason_out != NULL) *reason_out = FAN_CONTROL_REASON_RAMP;
    return percent;
}

TaskHandle_t fan_task_start(app_data_t *data)
{
    if(data == NULL)
    {
        return NULL;
    }

    if(fan_task_handle == NULL)
    {
        fan_task_handle = xTaskCreateStatic(fan_task_fn,
                                            "fan task",
                                            AMS_STACK_FAN_WORDS,
                                            (void *)data,
                                            FAN_PRIO,
                                            fan_task_stack,
                                            &fan_task_tcb);
    }

    return fan_task_handle;
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
    uint8_t reason = FAN_CONTROL_REASON_OFF_COOL;

    for(;;)
    {
        entry = osKernelGetTickCount();

        percent = fan_percent_from_temp(data, &reason);
        data->fan_state = (percent > 0.5f);
        data->fan_command_percent = percent;
        data->fan_control_reason = reason;
        data->fan_last_update_tick = entry;

        data->fan_fault = false;
        for(int i = 0; i < NFANS; i++)
        {
            if(set_fan_percent(&data->board.fans[i], percent) != 0)
            {
                data->fan_fault = true;
				if(data->fan_set_fail_count != UINT32_MAX)
				{
					data->fan_set_fail_count++;
				}
            }
        }

        if(data->fan_fault)
        {
            data->fan_control_reason = FAN_CONTROL_REASON_DRIVER_FAULT;
        }

        /* Fan actuation is part of the thermal safety chain. A live
         * temperature task cannot compensate for a fan task that stopped
         * executing, so publish an independent supervisor heartbeat. */
        ams_heartbeat_kick(data, AMS_HEARTBEAT_FAN, osKernelGetTickCount());

        osDelayUntil(entry + (1000 / FAN_FREQ));
    }
}
