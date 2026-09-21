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

static void error_task_latch_can_policy(app_data_t *data,
                                        ams_can_policy_latch_reason_t reason)
{
    if(data == NULL)
    {
        return;
    }

    data->can_authority_ready = false;
    if(!data->can_busoff_hard_fault_latched)
    {
        data->can_busoff_hard_fault_latched = true;
        data->can_busoff_policy_latch_reason = reason;
    }
    set_bms(false);
}


static bool error_task_can_recovered_before_discharge_deadline(
    const app_data_t *data)
{
    const canbus_device_t *canbus;
    const ams_can_tx_scheduler_t *sched;
    uint32_t generation;
    uint32_t completion_tick;
    uint32_t baseline;
    bool transport_settled;

    if(data == NULL)
    {
        return false;
    }

    canbus = &data->board.canbus;
    sched = &canbus->tx_scheduler;

    /* A mailbox completion is eligible only after task-context recovery has
     * fully settled the old controller epoch and moved the authority baseline
     * past every pre-bus-off completion. Without this gate, a delayed callback
     * from the old epoch could be mistaken for a 499 ms post-recovery delivery
     * when the lower-priority CAN task itself is delayed beyond 500 ms.
     *
     * Snapshot flags and scheduler markers under one interrupt mask so a new
     * CAN error callback cannot race this safety decision. The lower-priority
     * CAN task cannot preempt the error supervisor. */
    taskENTER_CRITICAL();
    transport_settled = canbus->started && canbus->notification_active &&
                        !data->canbus_fault &&
                        !data->can_busoff_fault &&
                        !data->can_recover_pending &&
                        (canbus->busoff_event_sequence ==
                         canbus->busoff_event_consumed_sequence) &&
                        !canbus->tx_recovery_pending &&
                        !canbus->tx_refresh_pending &&
                        !canbus->tx_suspended &&
                        !canbus->tx_latched_inhibit &&
                        !data->can_busoff_hard_fault_latched;
    baseline = data->can_authority_complete_generation_baseline;
    generation = sched->protected_required_last_complete_generation;
    completion_tick = sched->protected_required_last_complete_tick;
    taskEXIT_CRITICAL();

    if(!transport_settled)
    {
        return false;
    }

    return (generation != 0u) &&
           (generation != baseline) &&
           ((uint32_t)(completion_tick -
                       data->can_busoff_recovery_start_tick) <
            AMS_CAN_DISCHARGE_BUSOFF_HARD_FAULT_MS);
}

static void error_task_update_can_policy(app_data_t *data, uint32_t now)
{
    if(data == NULL)
    {
        return;
    }

    /* A repeated-bus-off TX inhibit is always a hard, service-resettable
     * communication fault regardless of the current operating state. */
    if(data->board.canbus.tx_latched_inhibit)
    {
        error_task_latch_can_policy(data, AMS_CAN_POLICY_LATCH_TX_INHIBIT);
    }

    if(!data->can_busoff_recovery_active)
    {
        return;
    }

    switch((state_t)data->can_busoff_recovery_state)
    {
    case STATE_CHARGE:
        error_task_latch_can_policy(data, AMS_CAN_POLICY_LATCH_CHARGE_BUSOFF);
        break;
    case STATE_BALANCE:
        error_task_latch_can_policy(data, AMS_CAN_POLICY_LATCH_BALANCE_BUSOFF);
        break;
    case STATE_DISCARGE:
        /* Drive-state continuity gets one bounded recovery interval. The ECU
         * contract expires torque authority at 300 ms; AMS fail-low follows
         * at 500 ms if a fresh required protected generation has not completed
         * on the wire. Unsigned subtraction preserves tick-wrap behavior. */
        if((uint32_t)(now - data->can_busoff_recovery_start_tick) >=
           AMS_CAN_DISCHARGE_BUSOFF_HARD_FAULT_MS)
        {
            /* Decide from actual wire-completion time, not whether the lower
             * priority CAN task has already observed that completion. */
            if(!error_task_can_recovered_before_discharge_deadline(data))
            {
                error_task_latch_can_policy(
                    data, AMS_CAN_POLICY_LATCH_DISCHARGE_TIMEOUT);
            }
        }
        break;
    case STATE_START:
    case STATE_NULL:
    case STATE_ERROR:
    default:
        break;
    }
}

static bool error_task_can_startup_ready(const app_data_t *data)
{
    if(data == NULL)
    {
        return false;
    }

    return data->can_authority_ready &&
           !data->can_busoff_recovery_active &&
           !data->can_authority_refresh_pending &&
           !data->can_busoff_fault &&
           !data->can_recover_pending &&
           !data->board.canbus.tx_latched_inhibit &&
           !data->can_busoff_hard_fault_latched;
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
           /* MUTE is the fast kill, but watchdog behavior makes it transient.
            * Whenever balancing is not actively and successfully applied,
            * BMS_OK requires verified persistent DCC/PWM zero state. */
           (data->adbms_balance_active ||
            data->adbms_balance_durable_zero_verified) &&
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
           !data->can_authority_refresh_pending &&
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

    error_task_update_can_policy(data, now);

    data->hard_fault = (data->fuse_fault ||
                        data->can_busoff_hard_fault_latched ||
                        error_task_air_feedback_fault(data, now) ||
                        data->temp_fault ||
                        data->voltage_fault ||
                        data->imd_fault ||
                        data->charger_fault ||
                        data->adbms_diag_fault ||
                        data->task_heartbeat_fault ||
                        data->current_overcurrent_fault ||
                        data->current_fault_latched ||
                        data->rtos_fault ||
                        data->rtos_stack_critical);

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
       error_task_can_startup_ready(data) &&
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
        osDelayUntil(entry + (1000 / ERR_FREQ));
    }
}
