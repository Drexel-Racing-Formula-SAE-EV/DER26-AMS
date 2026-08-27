/*
 * air_task.c
 *
 *  Created on: Apr 3, 2024
 *      Author: Cole Bardin
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */
#include "tasks/air_task.h"

void air_task_fn(void *argument);

static StaticTask_t air_task_tcb;
static StackType_t air_task_stack[AMS_STACK_AIR_WORDS];
static TaskHandle_t air_task_handle = NULL;

__weak bool ams_air_board_get_config(ams_air_monitor_config_t *config)
{
    (void)config;
    return false;
}

__weak bool ams_air_board_read_inputs(ams_air_monitor_inputs_t *inputs,
                                      uint32_t now)
{
    (void)inputs;
    (void)now;
    return false;
}

/*
 * CURRENT-HARDWARE LEGACY PATH / NOT STARTED
 * ------------------------------------------
 * With AMS_ENABLE_AIR_AUX_FEEDBACK=0, this task only samples AIR_CONTROL_MCU.
 * That net indicates voltage on the common AIR control line; it is not AIR+,
 * AIR- or precharge physical-state feedback. The high-priority supervisor
 * already samples the same net, so app.c does not start that legacy path.
 *
 * The future hardware revision needs protected AIR_POS_AUX and AIR_NEG_AUX
 * inputs (and preferably
 * PRECHARGE_AUX plus load-side HV sensing).  A single producer should debounce
 * those inputs, perform line supervision where available, construct one local
 * ams_air_monitor_inputs_t, and call ams_air_monitor_step() with reviewed
 * ams_air_monitor_config_t values. The pure evaluator already implements
 * freshness, debounce, transition deadlines, voltage plausibility, latching
 * and verified-open clearing. The feature-enabled path below is the single
 * producer and publishes one coherent snapshot; the weak board hooks keep it
 * fail-closed until the reviewed hardware adapter replaces them.
 * The safety supervisor remains the authoritative evaluator and sole normal
 * owner of BMS_OK assertion.  See docs/AIR_CONTACTOR_MONITORING.md.
 */

TaskHandle_t air_task_start(app_data_t *data)
{
    if(data == NULL)
    {
        return NULL;
    }

    if(air_task_handle == NULL)
    {
        air_task_handle = xTaskCreateStatic(air_task_fn,
                                            "air task",
                                            AMS_STACK_AIR_WORDS,
                                            (void *)data,
                                            AIR_PRIO,
                                            air_task_stack,
                                            &air_task_tcb);
    }

    return air_task_handle;
}

void air_task_fn(void *argument)
{
	app_data_t *data = (app_data_t *) argument;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

	uint32_t entry;

#if AMS_ENABLE_AIR_AUX_FEEDBACK
    ams_air_monitor_config_t config = {0};
    bool config_available = false;
#endif

	for(;;)
	{
		entry = osKernelGetTickCount();

#if AMS_ENABLE_AIR_AUX_FEEDBACK
        ams_air_monitor_inputs_t inputs = {0};
        ams_air_monitor_t next;
        uint32_t previous_latched_mask;
        bool inputs_available;

        /* Accept exactly one reviewed valid configuration. Retrying only until
         * the first valid result supports deterministic board-startup ordering;
         * runtime configuration changes are intentionally ignored. */
        if(!config_available)
        {
            ams_air_monitor_config_t candidate = {0};
            if(ams_air_board_get_config(&candidate) &&
               ams_air_monitor_schedule_valid(
                   &candidate,
                   AMS_AIR_MONITOR_PERIOD_MS,
                   AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS))
            {
                config = candidate;
                config_available = true;
            }
        }

        inputs.now_tick = entry;
        inputs_available = ams_air_board_read_inputs(&inputs, entry);
        if(!inputs_available)
        {
            /* A failed/partial board acquisition must not leave valid bits or
             * fresh timestamps from a partially populated/stale object. */
            inputs = (ams_air_monitor_inputs_t){0};
        }
        /* The scheduler owns evaluator time. A board adapter may timestamp the
         * individual samples, but it may not override the evaluation clock. */
        inputs.now_tick = entry;

        /* Preserve the evaluator's debounce/transition history, update a local
         * copy, then publish it atomically for the high-priority supervisor. */
        taskENTER_CRITICAL();
        next = data->air_monitor;
        taskEXIT_CRITICAL();
        previous_latched_mask = next.latched_fault_mask;

        ams_air_monitor_step(&next,
                             config_available ? &config : NULL,
                             &inputs);

        taskENTER_CRITICAL();
        data->air_monitor = next;
        data->air_monitor_inputs = inputs;
        taskEXIT_CRITICAL();

        if((next.latched_fault_mask & ~previous_latched_mask) != 0u)
        {
            ams_fault_log_event(AMS_FAULT_LOG_AIR_FAULT_LATCH,
                                (uint32_t)next.reason,
                                next.active_fault_mask,
                                next.latched_fault_mask);
        }

        osDelayUntil(entry + AMS_AIR_MONITOR_PERIOD_MS);
#else
		/* Legacy command/control sense only. This path is not started by app.c. */
		data->air_state = (HAL_GPIO_ReadPin(AIR_CTRL_GPIO_Port, AIR_CTRL_Pin) ==
		                   GPIO_PIN_SET);

		osDelayUntil(entry + (1000 / AIR_FREQ));
#endif
	}
}
