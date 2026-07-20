/**
 * @file error_task.c
 * @author Ian Kennedy (ibk24@drexel.edu)
 * @author Mahad Faisal (major firmware updates, 2026)
 * @brief
 * @version 0.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "main.h"
#include "tasks/error_task.h"
#include "ext_drivers/ams_safety.h"
#include "ext_drivers/ams_rtos_diag.h"

/**
 * @brief Actual ERROR task function
 *
 * @param arg App_data struct pointer converted to void pointer
 */
void error_task_fn(void *arg);

static StaticTask_t error_task_tcb;
static StackType_t error_task_stack[AMS_STACK_ERROR_WORDS];
static TaskHandle_t error_task_handle = NULL;

static bool error_task_air_publication_fresh(const app_data_t *data,
                                             uint32_t now)
{
#if AMS_ENABLE_AIR_AUX_FEEDBACK
    return (data != NULL) &&
           (AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS > 0u) &&
           ((uint32_t)(now - data->air_monitor.last_update_tick) <=
            AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS);
#else
    (void)data;
    (void)now;
    return true;
#endif
}

static bool error_task_air_feedback_ready(const app_data_t *data,
                                          uint32_t now)
{
#if AMS_ENABLE_AIR_AUX_FEEDBACK
    return error_task_air_publication_fresh(data, now) &&
           ams_air_monitor_ready(&data->air_monitor);
#else
    (void)data;
    (void)now;
    return true;
#endif
}

static bool error_task_air_feedback_fault(const app_data_t *data,
                                          uint32_t now)
{
#if AMS_ENABLE_AIR_AUX_FEEDBACK
    return !error_task_air_publication_fresh(data, now) ||
           data->air_monitor.fault ||
           data->air_monitor.fault_latched ||
           (data->air_monitor.active_fault_mask != 0u) ||
           (data->air_monitor.latched_fault_mask != 0u);
#else
    (void)data;
    (void)now;
    return false;
#endif
}

static bool error_task_operating_inputs_ready(const app_data_t *data,
                                              uint32_t now)
{
    if(data == NULL)
    {
        return false;
    }

    return data->voltage_valid &&
           !data->voltage_fault &&
           data->temp_valid &&
           !data->temp_fault &&
           data->current_valid &&
           !data->current_fault &&
           !data->adbms_diag_fault &&
           !data->task_heartbeat_fault &&
           !data->fuse_fault &&
           !data->charger_fault &&
           error_task_air_feedback_ready(data, now) &&
           data->imd_valid &&
           data->imd_ok &&
           !data->imd_fault &&
           !data->hard_fault;
}

static bool error_task_current_policy_matches_state(const app_data_t *data)
{
    if(data == NULL)
    {
        return false;
    }

    switch(data->state)
    {
    case STATE_CHARGE:
        return data->current_fault_mode == CURRENT_FAULT_MODE_CHARGE;
    case STATE_DISCARGE:
        return data->current_fault_mode == CURRENT_FAULT_MODE_DRIVE;
    case STATE_BALANCE:
        return data->current_fault_mode == CURRENT_FAULT_MODE_IDLE;
    case STATE_NULL:
    case STATE_START:
    case STATE_ERROR:
    default:
        return false;
    }
}

static bool error_task_bms_ready(const app_data_t *data, uint32_t now)
{
    if(data == NULL)
    {
        return false;
    }

#if AMS_ENABLE_IWDG
    /* A compile-enabled hardware watchdog is part of the safety architecture,
     * not an optional diagnostic.  If its irreversible start handshake did
     * not complete, keep BMS_OK low. */
    if(!ams_safety_watchdog_hw_started())
    {
        return false;
    }
#endif

    /* Caller holds the short safety critical section. */
    return ams_state_allows_bms_ok(data->state) &&
           !data->state_transition_in_progress &&
           error_task_current_policy_matches_state(data) &&
           error_task_operating_inputs_ready(data, now) &&
           ((data->state != STATE_CHARGE) || !data->temp_charge_stop) &&
           !data->hard_fault;
}

TaskHandle_t error_task_start(app_data_t *data)
{
    if(data == NULL)
    {
        return NULL;
    }

    if(error_task_handle == NULL)
    {
        error_task_handle = xTaskCreateStatic(error_task_fn,
                                              "ERROR task",
                                              AMS_STACK_ERROR_WORDS,
                                              (void *)data,
                                              ERR_PRIO,
                                              error_task_stack,
                                              &error_task_tcb);
    }

    return error_task_handle;
}

void error_task_update(app_data_t *data, uint32_t now)
{
    bool startup_transitioned = false;
    bool log_state_transition = false;
    state_t transition_from = STATE_NULL;
    state_t transition_to = STATE_NULL;
    ams_state_transition_reason_t transition_reason = AMS_STATE_TRANSITION_BOOT;
    uint32_t transition_count = 0u;

    if(data == NULL)
    {
        return;
    }

	/* AIR_CONTROL_MCU reports voltage on the existing common contactor-control
	 * net.  It is retained for telemetry only and must not be interpreted as an
	 * auxiliary/mirror-contact result. The future board adapter must construct
	 * one local ams_air_monitor_inputs_t and call ams_air_monitor_step(); it must
	 * not publish individual AIR monitor fields piecemeal. */
	data->air_state = (HAL_GPIO_ReadPin(AIR_CTRL_GPIO_Port, AIR_CTRL_Pin) ==
	                   GPIO_PIN_SET);
    (void)ams_heartbeat_update(data, now);
    ams_rtos_diag_update(data);

    /* Aggregate faults, evaluate readiness, and update BMS_OK as one atomic
     * supervisor decision.  Keeping interrupts masked through set_bms() closes
     * the race where an ISR could force BMS_OK low after the snapshot and the
     * supervisor could then reassert it from stale readiness. */
    taskENTER_CRITICAL();

    /* An out-of-range state is memory corruption, not an operating mode.
     * Normalize it to the explicit fail-safe state before evaluating the
     * output gate.  STATE_NULL and STATE_ERROR remain intact but can never
     * authorize BMS_OK. */
    if(!ams_state_is_valid(data->state))
    {
        set_bms(false);
        if(ams_state_transition_begin(data,
                                      STATE_ERROR,
                                      AMS_STATE_TRANSITION_CORRUPT_CURRENT_STATE,
                                      now) == AMS_STATE_TRANSITION_APPLIED)
        {
            transition_from = data->state_previous;
            transition_to = data->state;
            transition_reason = data->state_transition_reason;
            transition_count = data->state_transition_count;
            log_state_transition = true;
        }
        ams_state_transition_finish(data);
    }

    data->hard_fault = (data->fuse_fault ||
                        error_task_air_feedback_fault(data, now) ||
                        data->temp_fault ||
                        data->voltage_fault ||
                        data->imd_fault ||
                        data->charger_fault ||
                        data->adbms_diag_fault ||
                        data->task_heartbeat_fault ||
                        data->current_overcurrent_fault ||
                        data->current_fault_latched ||
                        data->rtos_fault);

    data->soft_fault = (data->cli_fault ||
                        data->canbus_fault ||
                        data->logger_heartbeat_fault ||
                        data->current_sensor_fault ||
                        data->current_overcurrent_warning ||
                        data->current_overcurrent_pending ||
                        data->temp_warning ||
                        data->temp_overtemp_pending ||
                        data->fan_fault ||
                        data->rtos_stack_warning ||
                        data->rtos_heap_warning);

    /* The present vehicle hardware owns precharge sequencing.  Firmware only
     * supplies the BMS_OK/shutdown-loop permit and has no AIR command or
     * precharge-complete input.  STATE_START therefore means software
     * initialization only.  Once every software safety input is valid, move
     * to the normal drive policy; BMS_OK remains low until the current task has
     * published a fresh DRIVE-policy result on a later iteration.  This avoids
     * both the old permanent 1.2 A startup limit and a mode-transition window
     * evaluated with stale current thresholds. */
    if((data->state == STATE_START) &&
       error_task_operating_inputs_ready(data, now))
    {
        set_bms(false);
        if(ams_state_transition_begin(data,
                                      STATE_DISCARGE,
                                      AMS_STATE_TRANSITION_STARTUP_READY,
                                      now) == AMS_STATE_TRANSITION_APPLIED)
        {
            startup_transitioned = true;
            transition_from = data->state_previous;
            transition_to = data->state;
            transition_reason = data->state_transition_reason;
            transition_count = data->state_transition_count;
            log_state_transition = true;
        }
        ams_state_transition_finish(data);
    }

    /* This task is the sole normal owner allowed to assert BMS_OK.
     * Measurement/communication tasks may still force the output low for
     * immediate response, but they cannot reassert it. */
    data->bms_supervisor_ready = !startup_transitioned &&
                                 error_task_bms_ready(data, now);
    set_bms(data->bms_supervisor_ready);

    taskEXIT_CRITICAL();

    if(log_state_transition)
    {
        ams_fault_log_event(AMS_FAULT_LOG_STATE_TRANSITION,
                            (uint16_t)transition_reason,
                            (((uint32_t)(uint16_t)transition_from) << 16u) |
                                (uint32_t)(uint16_t)transition_to,
                            transition_count);
    }

    ams_safety_watchdog_task_update(data);
}

void error_task_fn(void *arg)
{
	app_data_t *data = (app_data_t *)arg;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }


    uint32_t entry;

    for(;;)
    {
        entry = osKernelGetTickCount();
        error_task_update(data, entry);
//        data->cascadia_error = HAL_GPIO_ReadPin(MTR_Fault_GPIO_Port, MTR_Fault_Pin);
//		data->imd_fail = HAL_GPIO_ReadPin(IMD_Fail_GPIO_Port, IMD_Fail_Pin);
//		data->bms_fail = HAL_GPIO_ReadPin(BMS_Fail_GPIO_Port, BMS_Fail_Pin);
//		data->bspd_fail = HAL_GPIO_ReadPin(BSPD_Fail_GPIO_Port, BSPD_Fail_Pin);
//
//		if(!data->board.ams.air_state && !data->imd_fail && !data->bms_fail && !data->bspd_fail) set_ssa(100);
//		else set_ssa(0);
//
////		data->hard_fault = (data->apps_fault ||
////				            data->bse_fault ||
////							data->coolant_fault ||
////							data->cascadia_error
////						    );
////		data->hard_fault = (data->coolant_fault ||
////							data->cascadia_error
////						    );
//
//        data->soft_fault =(data->bppc_fault ||
//        				   data->cli_fault ||
//						   data->acc_fault ||
//						   data->canbus_fault ||
//						   data->dashboard_fault
//						   );
//
//        if(data->fw_override) set_ecu_ok(data->fw_override_state);
//        else set_ecu_ok(!data->coolant_fault);
//        // I believe this needs to be set low on an APPS/BSE fault (rules say disable inverter but no need to disable tractive system)
//        set_cascadia_enable(!data->hard_fault);
//
//        if(data->hard_fault){
//        	set_ecu_ok(0);
//        }
        osDelayUntil(entry + (1000 / ERR_FREQ));
    }
}
