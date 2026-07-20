/*
 * adbms_task.h
 *
 *  Created on: June 3, 2025
 *      Author: Cole Bardin
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#include <tasks/adbms_task.h>

#define ADBMS_STATUS_DIAG_PERIOD_MS            1000u
#define ADBMS_CONFIG_DIAG_PERIOD_MS            6000u
#define ADBMS_OPEN_WIRE_DIAG_PERIOD_MS        30000u
#define ADBMS_TEMP_POLICY_MAX_ELAPSED_MS       1000u

void adbms_task_fn(void *argument);

static StaticTask_t adbms_task_tcb;
static StackType_t adbms_task_stack[AMS_STACK_ADBMS_WORDS];
static TaskHandle_t adbms_task_handle = NULL;

static bool adbms_status_diag_has_safety_fault(const adbms6830_driver_t *smb)
{
    return !adbms6830_safety_diagnostics_ok(smb);
}

#if AMS_ADBMS_OPEN_WIRE_ENABLED
static bool adbms_open_wire_auto_allowed(const app_data_t *data)
{
    return (data != NULL) &&
           ((data->state == STATE_CHARGE) ||
            (data->state == STATE_DISCARGE) ||
            (data->state == STATE_BALANCE)) &&
           data->voltage_valid &&
           data->temp_valid &&
           data->current_valid &&
           !data->hard_fault &&
           !data->voltage_fault &&
           !data->temp_fault &&
           !data->current_fault;
}
#endif

static bool adbms_diag_deadline_due(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0);
}

static uint32_t adbms_diag_next_deadline(uint32_t now,
                                         uint32_t deadline,
                                         uint32_t period_ms)
{
    uint32_t next = deadline + period_ms;
    return adbms_diag_deadline_due(now, next) ? (now + period_ms) : next;
}

static void adbms_task_run_periodic_diagnostics(app_data_t *data,
                                                uint32_t now)
{
    accumulator_t *acc;
    adbms6830_driver_t *smb;
    HAL_StatusTypeDef status;
    bool ran_diagnostic = false;
    uint32_t diagnostic_start = now;
    uint32_t lateness_ms = 0u;

    if(data == NULL)
    {
        return;
    }

    acc = &data->acc;
    smb = &acc->smb;

    if(!data->adbms_diag_schedule_initialized)
    {
        data->adbms_status_diag_next_tick = now + ADBMS_STATUS_DIAG_PERIOD_MS;
        data->adbms_config_diag_next_tick = now + ADBMS_CONFIG_DIAG_PERIOD_MS;
        data->adbms_open_wire_diag_next_tick = now + ADBMS_OPEN_WIRE_DIAG_PERIOD_MS;
        data->adbms_diag_schedule_initialized = true;
    }

    if(adbms_diag_deadline_due(now, data->adbms_status_diag_next_tick))
    {
        bool status_fault;
        lateness_ms = now - data->adbms_status_diag_next_tick;
        data->adbms_status_diag_next_tick =
            adbms_diag_next_deadline(now,
                                     data->adbms_status_diag_next_tick,
                                     ADBMS_STATUS_DIAG_PERIOD_MS);
        ran_diagnostic = true;

        status = adbms6830_refresh_diagnostics(smb);
        status_fault = (!acc->delay_timer_ready ||
                        !acc->smb_ready ||
                        (status != HAL_OK) ||
                        adbms_status_diag_has_safety_fault(smb));
        taskENTER_CRITICAL();
        data->adbms_last_diag_status = status;
		if(data->adbms_status_diag_count != UINT32_MAX)
		{
			data->adbms_status_diag_count++;
		}
		data->adbms_status_fault = status_fault;
        taskEXIT_CRITICAL();
    }

    if(adbms_diag_deadline_due(now, data->adbms_config_diag_next_tick))
    {
#if AMS_ADBMS_FULL_CONFIG_ON_INIT
        const adbms6830_diag_health_t *health;
#endif
        uint32_t config_lateness = now - data->adbms_config_diag_next_tick;
        if(config_lateness > lateness_ms)
        {
            lateness_ms = config_lateness;
        }
        data->adbms_config_diag_next_tick =
            adbms_diag_next_deadline(now,
                                     data->adbms_config_diag_next_tick,
                                     ADBMS_CONFIG_DIAG_PERIOD_MS);
        ran_diagnostic = true;

#if AMS_ADBMS_FULL_CONFIG_ON_INIT
        status = adbms6830_verify_config_readback(smb);
        health = adbms6830_diag_health_get(smb);
        taskENTER_CRITICAL();
        data->adbms_last_diag_status = status;
        if(data->adbms_config_diag_count != UINT32_MAX)
        {
            data->adbms_config_diag_count++;
        }
        data->adbms_config_fault = ((status != HAL_OK) ||
                                    ((health != NULL) &&
                                     (health->config_mismatch_mask != 0u)));
        taskEXIT_CRITICAL();
#else
        /* Monitor-only mode intentionally leaves reset-default CFGA/B in
         * place. Probe a PEC/counter-checked read without comparing it to an
         * unwritten DER configuration image. */
        status = adbms6830_spi_probe_rdcfga(smb);
        taskENTER_CRITICAL();
        data->adbms_last_diag_status = status;
        if(data->adbms_config_diag_count != UINT32_MAX)
        {
            data->adbms_config_diag_count++;
        }
        data->adbms_config_fault = (status != HAL_OK);
        taskEXIT_CRITICAL();
#endif
    }

#if AMS_ADBMS_OPEN_WIRE_ENABLED
    if(adbms_diag_deadline_due(now, data->adbms_open_wire_diag_next_tick))
    {
        uint32_t open_wire_lateness = now - data->adbms_open_wire_diag_next_tick;
        if(open_wire_lateness > lateness_ms)
        {
            lateness_ms = open_wire_lateness;
        }
        data->adbms_open_wire_diag_next_tick =
            adbms_diag_next_deadline(now,
                                     data->adbms_open_wire_diag_next_tick,
                                     ADBMS_OPEN_WIRE_DIAG_PERIOD_MS);
        if(adbms_open_wire_auto_allowed(data))
        {
            /* One diagnostic is an inseparable even+odd S-ADC pair. */
            ran_diagnostic = true;
            status = adbms6830_run_open_wire_diagnostic(smb);
            taskENTER_CRITICAL();
            data->adbms_last_diag_status = status;
            if(data->adbms_open_wire_diag_count != UINT32_MAX)
            {
                data->adbms_open_wire_diag_count++;
            }
            if(status != HAL_OK)
            {
                data->adbms_open_wire_fault = true;
            }
            taskEXIT_CRITICAL();
        }
    }
#endif

    if(ran_diagnostic)
    {
        uint32_t duration_ms = osKernelGetTickCount() - diagnostic_start;
        data->adbms_diag_last_duration_ms = duration_ms;
        if(duration_ms > data->adbms_diag_max_duration_ms)
        {
            data->adbms_diag_max_duration_ms = duration_ms;
        }
        data->adbms_diag_last_lateness_ms = lateness_ms;
        if(lateness_ms > data->adbms_diag_max_lateness_ms)
        {
            data->adbms_diag_max_lateness_ms = lateness_ms;
        }
    }

    taskENTER_CRITICAL();
    data->adbms_diag_fault = (data->adbms_config_fault ||
                              data->adbms_status_fault ||
                              data->adbms_open_wire_fault ||
                              data->adbms_balance_write_fault);
    taskEXIT_CRITICAL();
}

static uint32_t adbms_task_pack_cell_extremes(const app_data_t *data)
{
    if(data == NULL)
    {
        return 0u;
    }

    return (((uint32_t)data->max_voltage_seg & 0xFFu) << 24) |
           (((uint32_t)data->max_voltage_cell & 0xFFu) << 16) |
           (((uint32_t)data->min_voltage_seg & 0xFFu) << 8) |
           ((uint32_t)data->min_voltage_cell & 0xFFu);
}

static uint32_t adbms_task_pack_temp_extremes(const app_data_t *data)
{
    if(data == NULL)
    {
        return 0u;
    }

    return (((uint32_t)data->max_temp_seg & 0xFFu) << 24) |
           (((uint32_t)data->max_temp_sensor & 0xFFu) << 16) |
           (((uint32_t)data->min_temp_seg & 0xFFu) << 8) |
           ((uint32_t)data->min_temp_sensor & 0xFFu);
}

static uint16_t adbms_task_diag_reason_bits(const app_data_t *data)
{
    uint16_t reason = 0u;

    if(data == NULL)
    {
        return reason;
    }

    if(data->adbms_config_fault)
    {
        reason |= 0x0001u;
    }
    if(data->adbms_status_fault)
    {
        reason |= 0x0002u;
    }
    if(data->adbms_open_wire_fault)
    {
        reason |= 0x0004u;
    }
    if(data->adbms_balance_write_fault)
    {
        reason |= 0x0008u;
    }

    return reason;
}

bool adbms_record_balance_write_result(app_data_t *data, int result)
{
    bool was_faulted;
    uint16_t reason_bits = 0u;
    uint32_t fail_count = 0u;

    if(data == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    was_faulted = data->adbms_balance_write_fault;
    if(result == 0)
    {
        data->adbms_balance_write_fault = false;
#if AMS_HIL_REPLACE_ADBMS
        data->adbms_diag_fault = (data->can_busoff_fault || data->can_recover_pending);
#else
        data->adbms_diag_fault = (data->adbms_config_fault ||
                                  data->adbms_status_fault ||
                                  data->adbms_open_wire_fault);
#endif
        taskEXIT_CRITICAL();
        return true;
    }

    data->adbms_balance_write_fault = true;
    if(data->adbms_balance_write_fail_count != UINT32_MAX)
    {
        data->adbms_balance_write_fail_count++;
    }
    data->adbms_last_diag_status = HAL_ERROR;
    data->adbms_diag_fault = true;
    reason_bits = adbms_task_diag_reason_bits(data);
    fail_count = data->adbms_balance_write_fail_count;
    taskEXIT_CRITICAL();
    if(!was_faulted)
    {
        ams_fault_log_event(AMS_FAULT_LOG_ADBMS_DIAG_FAIL,
                            reason_bits,
                            (uint32_t)HAL_ERROR,
                            fail_count);
    }
    set_bms(false);
    return false;
}

static void adbms_task_publish_voltage_state(app_data_t *data,
                                             const voltage_fault_state_t *fault,
                                             const accumulator_t *acc)
{
    if((data == NULL) || (fault == NULL) || (acc == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    data->voltage_fault_state = *fault;
    data->max_voltage = acc->max_volt;
    data->min_voltage = acc->min_volt;
    data->total_voltage = acc->total_volt;
    data->voltage_valid = fault->voltage_valid;
    data->voltage_read_fault = fault->read_fault;
    data->voltage_warning = fault->warning;
    data->charge_voltage_stop = fault->charge_stop;
    data->overvoltage_fault = fault->overvoltage_fault;
    data->undervoltage_fault = fault->undervoltage_fault;
    data->voltage_fault_latched = fault->latched;
    data->voltage_fault_reason = fault->reason;
    data->voltage_fault_latched_reason = fault->latched_reason;
    data->voltage_usable_cell_count = fault->usable_cell_count;
    data->voltage_updated_cell_count = fault->updated_cell_count;
    data->voltage_stale_cell_count = fault->stale_cell_count;
    data->voltage_pec_fail_cell_count = fault->pec_fail_cell_count;
    data->voltage_jump_cell_count = acc->voltage_jump_cell_count;
    data->voltage_stuck_cell_count = acc->voltage_stuck_cell_count;
    data->voltage_max_delta_mv = acc->voltage_max_delta_mv;
    data->voltage_max_delta_seg = acc->voltage_max_delta_seg;
    data->voltage_max_delta_cell = acc->voltage_max_delta_cell;
    data->max_voltage_seg = fault->max_cell_segment;
    data->max_voltage_cell = fault->max_cell_index;
    data->min_voltage_seg = fault->min_cell_segment;
    data->min_voltage_cell = fault->min_cell_index;

    data->voltage_fault = (fault->read_fault ||
                           fault->overvoltage_fault ||
                           fault->undervoltage_fault ||
                           fault->latched);
    taskEXIT_CRITICAL();
}

static void adbms_task_publish_temperature_state(app_data_t *data,
                                                 const temperature_fault_state_t *fault,
                                                 const accumulator_t *acc)
{
    if((data == NULL) || (fault == NULL) || (acc == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    data->temp_fault_state = *fault;
    data->max_temp = acc->max_temp;
    data->avg_temp = acc->avg_temp;
    data->temp_valid = fault->temp_valid;
    data->temp_read_fault = fault->read_fault;
    data->temp_warning = fault->warning;
    data->temp_fan_max = fault->fan_max;
    data->temp_charge_stop = fault->charge_stop;
    data->temp_overtemp_pending = fault->pending;
    data->overtemp_fault = fault->overtemp_fault;
    data->severe_overtemp_fault = fault->severe_overtemp_fault;
    data->temp_fault_latched = fault->latched;
    data->temp_fault_reason = fault->reason;
    data->temp_fault_pending_reason = fault->pending_reason;
    data->temp_fault_latched_reason = fault->latched_reason;
    data->temp_fault_pending_ms = fault->pending_ms;
    data->temp_usable_sensor_count = fault->usable_sensor_count;
    data->temp_updated_sensor_count = fault->updated_sensor_count;
    data->temp_stale_sensor_count = fault->stale_sensor_count;
    data->temp_invalid_sensor_count = fault->invalid_sensor_count;
    data->temp_open_sensor_count = fault->open_sensor_count;
    data->temp_short_sensor_count = fault->short_sensor_count;
    data->temp_jump_sensor_count = fault->jump_sensor_count;
    data->temp_rate_rise_sensor_count = fault->rate_rise_sensor_count;
    data->temp_filtered_max = (float)fault->filtered_max_temp_deci_c / 10.0f;
    data->temp_filtered_avg = (float)fault->filtered_avg_temp_deci_c / 10.0f;
    data->temp_max_rate_c_per_s = (float)fault->max_rate_deci_c_per_s / 10.0f;
    data->temp_max_rate_seg = fault->max_rate_segment;
    data->temp_max_rate_sensor = fault->max_rate_sensor;
    data->max_temp_seg = fault->max_temp_segment;
    data->max_temp_sensor = fault->max_temp_sensor;
    data->min_temp_seg = fault->min_temp_segment;
    data->min_temp_sensor = fault->min_temp_sensor;

    data->temp_fault = (fault->read_fault ||
                        fault->overtemp_fault ||
                        fault->latched);
    taskEXIT_CRITICAL();
}

TaskHandle_t adbms_task_start(app_data_t *data)
{
    if(data == NULL)
    {
        return NULL;
    }

    if(adbms_task_handle == NULL)
    {
        adbms_task_handle = xTaskCreateStatic(adbms_task_fn,
                                              "adbms task",
                                              AMS_STACK_ADBMS_WORDS,
                                              (void *)data,
                                              ADBMS_PRIO,
                                              adbms_task_stack,
                                              &adbms_task_tcb);
    }

    return adbms_task_handle;
}

void adbms_task_fn(void *argument)
{
	app_data_t *data = (app_data_t *) argument;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

	accumulator_t *acc = &data->acc;
	uint32_t entry;

	for(;;)
	{
	    entry = osKernelGetTickCount();
	    uint16_t balance_mask_at_acquisition[NSMBS] = {0u};
		    bool balance_was_active = false;
		    bool balance_recovery_verified =
		        (AMS_ADBMS_BALANCE_WRITES_ENABLED != 0);
	    uint32_t balance_off_ms = 0u;
	    uint32_t balance_clear_tick = 0u;
	    bool balance_clear_timed = false;
	    ams_current_window_t current_window = {0};
	    bool current_window_timing_valid = false;
	    uint32_t voltage_complete_tick;
	    uint32_t balance_apply_tick = 0u;
	    /* Own the complete accumulator-image epoch, including the initial
	     * balance-shadow capture. Individual physical accumulator/driver
	     * helpers lock recursively. In CAN-fed HIL builds, the same lock
	     * excludes injected cell/temperature writes until the immutable
	     * snapshot has been published. */
	    adbms_spi_lock();
	    for(uint8_t seg = 0u; seg < NSMBS; seg++)
	    {
	        balance_mask_at_acquisition[seg] =
	            accumulator_balance_shadow_mask(acc, seg);
	        balance_was_active = balance_was_active ||
	                             (balance_mask_at_acquisition[seg] != 0u);
	    }
	#if !AMS_HIL_REPLACE_ADBMS && AMS_ADBMS_BALANCE_WRITES_ENABLED
		    bool balance_recovery_required =
		        balance_was_active || data->adbms_balance_write_fault;
#endif
	    data->adbms_scan_active = true;
	    /* Retained as a saturating-free lifetime scan counter. Diagnostic
	     * scheduling uses independent wrap-safe wall-clock deadlines. */
	    data->adbms_scan_count++;

        bool voltage_was_latched = data->voltage_fault_latched;
        voltage_fault_reason_t voltage_prev_latched_reason = data->voltage_fault_latched_reason;
        bool temp_was_latched = data->temp_fault_latched;
        temperature_fault_reason_t temp_prev_latched_reason = data->temp_fault_latched_reason;
	#if !AMS_HIL_REPLACE_ADBMS
	        bool adbms_diag_was_faulted = data->adbms_diag_fault;
#endif

	    /* Turn off balancing before reading so cell voltages recover from load.
	     * Do not impose this delay on every non-balancing scan: at 10 Hz the
	     * old unconditional 100 ms wait consumed the complete task period before
	     * conversion, temperature, diagnostics, and SPI work even started. */
	#if !AMS_HIL_REPLACE_ADBMS && AMS_ADBMS_BALANCE_WRITES_ENABLED
	    if(balance_recovery_required)
	    {
	        balance_recovery_verified = adbms_record_balance_write_result(
	            data, accumulator_clear_balance(acc));
	        if(!balance_recovery_verified)
	        {
            adbms_diag_was_faulted = true;
	        }
	        else
	        {
	            balance_clear_tick = osKernelGetTickCount();
	            balance_clear_timed = true;
	            if(data->adbms_balance_active)
	            {
	                data->adbms_last_balance_on_ms =
	                    (uint32_t)(balance_clear_tick -
	                               data->adbms_balance_apply_tick);
	            }
	        }
	        if(data->adbms_balance_recovery_count != UINT32_MAX)
	        {
	            data->adbms_balance_recovery_count++;
	        }
	        osDelay(AMS_ADBMS_BALANCE_RECOVERY_MS);
	    }
#endif

#if AMS_HIL_REPLACE_ADBMS
        accumulator_hil_refresh_update_masks(acc,
                                             osKernelGetTickCount(),
                                             AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS);
#else
		if(accumulator_read_volt(acc) == 0)
		{
			/* The compatible ADCV command also starts the ADBMS2950 conversion.
			 * Read the one-device String-B APM subset only after the successful
			 * SMB transaction has proven the complete ring was awake.  This makes
			 * the three shared SNAP/UNSNAP counter increments deterministic.
			 * Results remain diagnostic only and never enter BMS_OK. */
			(void)accumulator_read_apm(acc, osKernelGetTickCount());
		}
#endif
        accumulator_update_voltage_stats_at(acc, osKernelGetTickCount());
	    voltage_complete_tick = osKernelGetTickCount();
	    if(balance_clear_timed)
	    {
	        balance_off_ms =
	            (uint32_t)(voltage_complete_tick - balance_clear_tick);
	    }
	    data->adbms_last_balance_off_ms = balance_off_ms;
	    ams_current_window_lock();
	    (void)ams_current_window_rotate(&data->current_window,
	                                    voltage_complete_tick,
	                                    &current_window);
	    current_window_timing_valid =
	        (data->current_timing_fault_count ==
	         data->current_timing_fault_count_at_last_epoch);
	    data->current_timing_fault_count_at_last_epoch =
	        data->current_timing_fault_count;
	    ams_current_window_unlock();

        voltage_fault_state_t next_voltage_fault = data->voltage_fault_state;
        voltage_fault_update(&next_voltage_fault, acc);
        adbms_task_publish_voltage_state(data, &next_voltage_fault, acc);

        if(data->voltage_fault_latched &&
           (!voltage_was_latched ||
            (voltage_prev_latched_reason != data->voltage_fault_latched_reason)))
        {
            ams_fault_log_event(AMS_FAULT_LOG_VOLTAGE_LATCH,
                                (uint16_t)data->voltage_fault_latched_reason,
                                (((uint32_t)data->acc.max_voltage_mv) << 16) |
                                    (uint32_t)data->acc.min_voltage_mv,
                                adbms_task_pack_cell_extremes(data));
        }

        if(data->voltage_fault)
        {
            set_bms(0);
        }
        ams_heartbeat_kick(data, AMS_HEARTBEAT_ADBMS, osKernelGetTickCount());

#if AMS_HIL_REPLACE_ADBMS
        accumulator_hil_refresh_update_masks(acc,
                                             osKernelGetTickCount(),
                                             AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS);
#else
        (void)accumulator_read_temp(acc);
#endif
        accumulator_update_temp_stats_at(acc, osKernelGetTickCount());
        uint32_t temp_policy_tick = osKernelGetTickCount();
        uint32_t temp_policy_elapsed_ms = AMS_ADBMS_TASK_PERIOD_MS;
        if(data->temp_policy_tick_valid)
        {
            temp_policy_elapsed_ms = temp_policy_tick - data->temp_policy_last_tick;
            if(temp_policy_elapsed_ms > ADBMS_TEMP_POLICY_MAX_ELAPSED_MS)
            {
                temp_policy_elapsed_ms = ADBMS_TEMP_POLICY_MAX_ELAPSED_MS;
            }
        }
        data->temp_policy_last_tick = temp_policy_tick;
        data->temp_policy_last_elapsed_ms = temp_policy_elapsed_ms;
        data->temp_policy_tick_valid = true;
        temperature_fault_state_t next_temp_fault = data->temp_fault_state;
        temperature_fault_update_with_period(&next_temp_fault,
                                             acc,
                                             temp_policy_elapsed_ms);
        adbms_task_publish_temperature_state(data, &next_temp_fault, acc);

        if(data->temp_fault_latched &&
           (!temp_was_latched ||
            (temp_prev_latched_reason != data->temp_fault_latched_reason)))
        {
            ams_fault_log_event(AMS_FAULT_LOG_TEMP_LATCH,
                                (uint16_t)data->temp_fault_latched_reason,
                                (((uint32_t)(uint16_t)data->acc.max_temp_deci_c) << 16) |
                                    (uint32_t)(uint16_t)data->acc.min_temp_deci_c,
                                adbms_task_pack_temp_extremes(data));
        }

	    if(data->temp_fault)
	    {
	        set_bms(0);
	    }
	    ams_heartbeat_kick(data, AMS_HEARTBEAT_TEMP, osKernelGetTickCount());

#if AMS_HIL_REPLACE_ADBMS
	    taskENTER_CRITICAL();
	    data->adbms_config_fault = false;
	    data->adbms_status_fault = false;
	    data->adbms_open_wire_fault = false;
	    data->adbms_balance_write_fault = false;
        /* In ADBMS-image HIL mode, CAN is the measurement transport. Do not
         * clear a CAN bus-off/recovery-pending condition inside the ADBMS task
         * just because the last injected image has not aged out yet. */
	    data->adbms_diag_fault = (data->can_busoff_fault || data->can_recover_pending);
	    data->adbms_last_diag_status = data->adbms_diag_fault ? HAL_ERROR : HAL_OK;
	    taskEXIT_CRITICAL();
#else
	    adbms_task_run_periodic_diagnostics(data, osKernelGetTickCount());
        if(data->adbms_diag_fault && !adbms_diag_was_faulted)
        {
            ams_fault_log_event(AMS_FAULT_LOG_ADBMS_DIAG_FAIL,
                                adbms_task_diag_reason_bits(data),
                                (uint32_t)data->adbms_last_diag_status,
                                data->adbms_scan_count);
        }
#endif

	    /* The active measurement transport does not change the fail-closed
	     * action.  In physical-SPI builds this covers ADBMS diagnostics; in
	     * CAN-fed HIL builds it covers a bus-off/recovery-pending condition.
	     * Keep this outside the compile-time split so neither profile can retain
	     * a previously asserted BMS_OK while its transport is unhealthy. */
	    if(data->adbms_diag_fault)
	    {
	        set_bms(0);
	    }

	    /* Do not assert BMS_OK from a measurement task.  The error/safety
	     * supervisor owns the complete readiness decision and is the only task
	     * permitted to drive the output high. */

        /* Voltage charge-stop can still balance; hard faults/temp stop cannot. */
#if !AMS_HIL_REPLACE_ADBMS && AMS_ADBMS_BALANCE_WRITES_ENABLED
	        bool balance_applied_active = false;
	        if((data->state == STATE_CHARGE) &&
	           !data->balance_inhibit &&
	           data->voltage_valid &&
	           !data->voltage_fault &&
           data->temp_valid &&
           !data->temp_charge_stop &&
           !data->temp_fault &&
           !data->hard_fault &&
           !data->current_fault &&
           data->current_valid &&
           data->bms_state)
        {
            bool balance_write_ok = adbms_record_balance_write_result(
                data, accumulator_set_balance(acc));
            balance_applied_active = balance_write_ok &&
                                     accumulator_balance_shadow_active(acc);
        }
        else
        {
	        (void)adbms_record_balance_write_result(data, accumulator_clear_balance(acc));
	        }
	        data->adbms_balance_active = balance_applied_active;
	        if(balance_applied_active)
	        {
	            balance_apply_tick = osKernelGetTickCount();
	            data->adbms_balance_apply_tick = balance_apply_tick;
	        }
#else
	        const bool balance_applied_active = false;
	        data->adbms_balance_active = false;
#endif

	        uint32_t measurement_flags = 0u;
	        if(data->voltage_valid && !data->voltage_read_fault)
	        {
	            measurement_flags |= AMS_MEAS_VALID_VOLTAGE;
	        }
	        if(data->temp_valid && !data->temp_read_fault)
	        {
	            measurement_flags |= AMS_MEAS_VALID_TEMPERATURE;
	        }
        if(current_window.valid)
        {
            measurement_flags |= AMS_MEAS_VALID_CURRENT;
        }
        if((data->current_sample_sequence != 0u) &&
           current_window_timing_valid)
        {
            measurement_flags |= AMS_MEAS_CURRENT_TIMING_VALID;
        }
	        if(balance_recovery_verified)
	        {
	            measurement_flags |= AMS_MEAS_BALANCE_RECOVERED;
	        }
	        if(balance_was_active)
	        {
	            measurement_flags |= AMS_MEAS_BALANCE_WAS_ACTIVE;
	        }

	        uint32_t publication_tick = osKernelGetTickCount();
	        ams_measurement_snapshot_t *measurement =
	            ams_measurement_store_begin_write(&data->measurement_store);
	        ams_measurement_snapshot_prepare(measurement,
	                                         acc,
	                                         &current_window,
	                                         entry,
	                                         voltage_complete_tick,
	                                         publication_tick,
	                                         balance_mask_at_acquisition,
	                                         balance_off_ms,
	                                         measurement_flags);
	        (void)ams_measurement_store_publish(&data->measurement_store,
	                                            measurement);

	        data->adbms_scan_active = false;
	        adbms_spi_unlock();
	        uint32_t scan_end = osKernelGetTickCount();

	        uint32_t deadline = entry + AMS_ADBMS_TASK_PERIOD_MS;
	        if(balance_applied_active)
	        {
	            /* Base the minimum on-time on the verified write/readback time,
	             * not the scan entry. This prevents an overrun from setting PWM
	             * and immediately clearing it in the next iteration. */
	            uint32_t balance_deadline =
	                balance_apply_tick + AMS_ADBMS_BALANCE_MIN_ON_MS;
	            if((int32_t)(balance_deadline - deadline) > 0)
	            {
	                deadline = balance_deadline;
	            }
	        }

	        uint32_t duration_ms = (uint32_t)(scan_end - entry);
	        data->adbms_last_scan_duration_ms = duration_ms;
	        if(duration_ms > data->adbms_max_scan_duration_ms)
	        {
	            data->adbms_max_scan_duration_ms = duration_ms;
	        }
	        data->adbms_last_schedule_interval_ms = (uint32_t)(deadline - entry);
	        if((int32_t)(scan_end - deadline) > 0)
	        {
	            if(data->adbms_scan_deadline_miss_count != UINT32_MAX)
	            {
	                data->adbms_scan_deadline_miss_count++;
	            }
	        }
	        osDelayUntil(deadline);
	}
}
