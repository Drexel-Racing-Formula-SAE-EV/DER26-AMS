/*
 * app.c
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#include "app.h"

#include "cmsis_os.h"
#include "tasks/fan_task.h"
#include "tasks/cli_task.h"
#include "tasks/error_task.h"
#include "tasks/canbus_task.h"
#include "tasks/air_task.h"
#include "tasks/imd_task.h"
#include "tasks/current_task.h"
#include <tasks/adbms_task.h>
#include "tasks/estimator_task.h"
#include "ext_drivers/ams_rtos_diag.h"
#include "task.h"
#include <string.h>

app_data_t app = {0};

const ams_build_manifest_t ams_build_manifest = {
    .magic = AMS_BUILD_MANIFEST_MAGIC,
    .schema = AMS_BUILD_MANIFEST_SCHEMA,
    .profile = AMS_BUILD_PROFILE,
    .feature_flags = AMS_BUILD_FEATURE_FLAGS_VALUE,
    .estimator_topology = AMS_ESTIMATOR_DEFAULT_TOPOLOGY,
    .config_fingerprint = AMS_BUILD_CONFIG_FINGERPRINT,
    .profile_name = AMS_BUILD_PROFILE_NAME,
    .git_commit = AMS_BUILD_GIT_COMMIT,
    .build_date = __DATE__,
    .build_time = __TIME__,
	.estimator_model_revision = AMS_ESTIMATOR_MODEL_REVISION,
	.sop_model_revision = AMS_SOP_MODEL_REVISION,
	.soh_model_revision = AMS_SOH_MODEL_REVISION,
    .current_calibration_revision = AMS_CURRENT_CALIBRATION_REVISION,
    .can_contract_revision = AMS_CAN_CONTRACT_REVISION,
    .threshold_revision = AMS_THRESHOLD_REVISION
};
static osMutexId_t adbms_spi_mutex;
static StaticSemaphore_t adbms_spi_mutex_cb;
static TaskHandle_t adbms_spi_owner;
static uint16_t adbms_spi_lock_depth;
static uint32_t adbms_spi_lock_acquired_tick;
static osMutexId_t current_window_mutex;
static StaticSemaphore_t current_window_mutex_cb;

/*
 * ADBMS operations contain nested helper calls and prepare shared driver
 * buffers before the final HAL SPI transfer.  The mutex therefore has to be
 * recursive and must be held for the complete logical operation, not only for
 * the final transfer.  Static allocation also keeps this safety-critical lock
 * independent of the FreeRTOS heap.
 */
static const osMutexAttr_t adbms_spi_mutex_attr =
{
	.name = "ADBMS SPI",
	.attr_bits = osMutexRecursive | osMutexPrioInherit,
	.cb_mem = &adbms_spi_mutex_cb,
	.cb_size = sizeof(adbms_spi_mutex_cb),
};

static const osMutexAttr_t current_window_mutex_attr =
{
	.name = "current window",
	.attr_bits = osMutexPrioInherit,
	.cb_mem = &current_window_mutex_cb,
	.cb_size = sizeof(current_window_mutex_cb),
};

static void adbms_spi_lock_panic(ams_panic_reason_t reason)
{
	ams_safety_panic(reason);
	for(;;)
	{
	}
}

uint32_t ams_heartbeat_timeout_ms(ams_heartbeat_id_t id)
{
	switch(id)
	{
	case AMS_HEARTBEAT_ADBMS:   return AMS_HEARTBEAT_ADBMS_TIMEOUT_MS;
	case AMS_HEARTBEAT_CURRENT: return AMS_HEARTBEAT_CURRENT_TIMEOUT_MS;
	case AMS_HEARTBEAT_TEMP:    return AMS_HEARTBEAT_TEMP_TIMEOUT_MS;
	case AMS_HEARTBEAT_CAN:     return AMS_HEARTBEAT_CAN_TIMEOUT_MS;
	case AMS_HEARTBEAT_LOGGER:  return AMS_HEARTBEAT_LOGGER_TIMEOUT_MS;
	case AMS_HEARTBEAT_IMD:     return AMS_HEARTBEAT_IMD_TIMEOUT_MS;
	case AMS_HEARTBEAT_FAN:     return AMS_HEARTBEAT_FAN_TIMEOUT_MS;
	case AMS_HEARTBEAT_ESTIMATOR: return AMS_HEARTBEAT_ESTIMATOR_TIMEOUT_MS;
	default:                    return 0u;
	}
}

const char *ams_heartbeat_name(ams_heartbeat_id_t id)
{
	switch(id)
	{
	case AMS_HEARTBEAT_ADBMS:   return "adbms";
	case AMS_HEARTBEAT_CURRENT: return "current";
	case AMS_HEARTBEAT_TEMP:    return "temp";
	case AMS_HEARTBEAT_CAN:     return "can";
	case AMS_HEARTBEAT_LOGGER:  return "logger";
	case AMS_HEARTBEAT_IMD:     return "imd";
	case AMS_HEARTBEAT_FAN:     return "fan";
	case AMS_HEARTBEAT_ESTIMATOR: return "estimator";
	default:                    return "unknown";
	}
}

const char *ams_adbms_state_str(ams_adbms_state_t state)
{
    switch(state)
    {
    case AMS_ADBMS_STATE_OFFLINE: return "offline";
    case AMS_ADBMS_STATE_WAKING: return "waking";
    case AMS_ADBMS_STATE_IDENTIFIED: return "identified";
    case AMS_ADBMS_STATE_CONFIGURING: return "configuring";
    case AMS_ADBMS_STATE_MEASURING: return "measuring";
    case AMS_ADBMS_STATE_READY_REDUNDANT: return "ready_redundant";
    case AMS_ADBMS_STATE_READY_C_ONLY_DEGRADED: return "ready_c_only";
    case AMS_ADBMS_STATE_FAULTED: return "faulted";
    case AMS_ADBMS_STATE_RECOVERING: return "recovering";
    default: return "unknown";
    }
}

const char *ams_adbms_state_reason_str(ams_adbms_state_reason_t reason)
{
    switch(reason)
    {
    case AMS_ADBMS_STATE_REASON_BOOT: return "boot";
    case AMS_ADBMS_STATE_REASON_SCAN_BEGIN: return "scan_begin";
    case AMS_ADBMS_STATE_REASON_IDENTITY_OK: return "identity_ok";
    case AMS_ADBMS_STATE_REASON_CONFIG_OK: return "config_ok";
    case AMS_ADBMS_STATE_REASON_MEASUREMENT_OK: return "measurement_ok";
    case AMS_ADBMS_STATE_REASON_FAULT_ACTIVE: return "fault_active";
    case AMS_ADBMS_STATE_REASON_RECOVERY_REQUEST: return "recovery_request";
    case AMS_ADBMS_STATE_REASON_RECOVERY_RESULT: return "recovery_result";
    default: return "unknown";
    }
}


void ams_adbms_transition_state(app_data_t *data,
                                ams_adbms_state_t next,
                                ams_adbms_state_reason_t reason,
                                uint32_t now)
{
    ams_adbms_state_t previous;
    uint16_t active_mask;

    if(data == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    previous = data->adbms_lifecycle_state;
    if(previous == next)
    {
        data->adbms_lifecycle_reason = reason;
        taskEXIT_CRITICAL();
        return;
    }

    data->adbms_lifecycle_previous_state = previous;
    data->adbms_lifecycle_state = next;
    data->adbms_lifecycle_reason = reason;
    data->adbms_lifecycle_last_transition_tick = now;
    if(data->adbms_lifecycle_transition_count != UINT32_MAX)
    {
        data->adbms_lifecycle_transition_count++;
    }
    active_mask = data->adbms_fault_active_mask;
    taskEXIT_CRITICAL();

    ams_fault_log_event(AMS_FAULT_LOG_ADBMS_STATE_TRANSITION,
                        (uint16_t)reason,
                        (((uint32_t)previous & 0xFFu) << 8u) |
                            ((uint32_t)next & 0xFFu),
                        active_mask);
}

void ams_heartbeat_init(app_data_t *data, uint32_t now)
{
	if(data == NULL)
	{
		return;
	}

	data->heartbeat.boot_tick = now;
	data->heartbeat.seen_mask = 0u;
	data->heartbeat.stale_mask = 0u;
	data->heartbeat.safety_stale_mask = 0u;
	data->heartbeat.logger_stale_mask = 0u;
	for(uint8_t i = 0u; i < (uint8_t)AMS_HEARTBEAT_COUNT; i++)
	{
		data->heartbeat.last_tick[i] = now;
		data->heartbeat.last_gap_ms[i] = 0u;
		data->heartbeat.max_gap_ms[i] = 0u;
		data->heartbeat.count[i] = 0u;
	}

	data->task_heartbeat_fault = false;
	data->logger_heartbeat_fault = false;
	data->heartbeat_stale_mask = 0u;
	data->heartbeat_seen_mask = 0u;
}

void ams_heartbeat_kick(app_data_t *data, ams_heartbeat_id_t id, uint32_t now)
{
	if((data == NULL) || ((int)id < 0) || (id >= AMS_HEARTBEAT_COUNT))
	{
		return;
	}

	/* Multiple tasks kick different bits in the same mask.  Protect the whole
	 * read-modify-write so one preemption cannot erase another task's kick. */
	taskENTER_CRITICAL();
    if(data->heartbeat.count[id] != 0u)
    {
        uint32_t gap = (uint32_t)(now - data->heartbeat.last_tick[id]);
        data->heartbeat.last_gap_ms[id] = gap;
        if(gap > data->heartbeat.max_gap_ms[id])
        {
            data->heartbeat.max_gap_ms[id] = gap;
        }
    }
	data->heartbeat.last_tick[id] = now;
	if(data->heartbeat.count[id] != UINT32_MAX)
	{
		data->heartbeat.count[id]++;
	}
	data->heartbeat.seen_mask |= AMS_HEARTBEAT_BIT(id);
	data->heartbeat.seen_mask &= (uint16_t)((1u << (uint16_t)AMS_HEARTBEAT_COUNT) - 1u);
	data->heartbeat_seen_mask = data->heartbeat.seen_mask;
	taskEXIT_CRITICAL();
}

uint16_t ams_heartbeat_update(app_data_t *data, uint32_t now)
{
	uint16_t stale = 0u;
	uint16_t all_mask = (uint16_t)((1u << (uint16_t)AMS_HEARTBEAT_COUNT) - 1u);
	bool startup_grace;

	if(data == NULL)
	{
		return 0u;
	}

	/* Keep last_tick[] and seen_mask coherent with concurrent task kicks.  The
	 * monitor has only a handful of entries, so this critical section is deliberately
	 * short and bounded. */
	taskENTER_CRITICAL();
	startup_grace = (now - data->heartbeat.boot_tick) < AMS_HEARTBEAT_STARTUP_GRACE_MS;

	for(uint8_t i = 0u; i < (uint8_t)AMS_HEARTBEAT_COUNT; i++)
	{
		ams_heartbeat_id_t id = (ams_heartbeat_id_t)i;
		uint16_t bit = AMS_HEARTBEAT_BIT(id);
		uint32_t timeout_ms = ams_heartbeat_timeout_ms(id);

		if(timeout_ms == 0u)
		{
			continue;
		}

		if((data->heartbeat.seen_mask & bit) == 0u)
		{
			if(!startup_grace)
			{
				stale |= bit;
			}
		}
		else if((now - data->heartbeat.last_tick[i]) > timeout_ms)
		{
			stale |= bit;
		}
	}

	data->heartbeat.stale_mask = (uint16_t)(stale & all_mask);
	data->heartbeat.safety_stale_mask = (uint16_t)(stale & AMS_HEARTBEAT_SAFETY_MASK);
	data->heartbeat.logger_stale_mask = (uint16_t)(stale & AMS_HEARTBEAT_LOGGER_MASK);
	data->heartbeat_stale_mask = data->heartbeat.stale_mask;
	data->heartbeat_seen_mask = data->heartbeat.seen_mask;
	data->task_heartbeat_fault = (data->heartbeat.safety_stale_mask != 0u);
	data->logger_heartbeat_fault = (data->heartbeat.logger_stale_mask != 0u);

	stale = data->heartbeat.stale_mask;
	taskEXIT_CRITICAL();
	return stale;
}

void adbms_spi_lock(void)
{
	uint32_t wait_start;
	uint32_t acquired_tick;
	TaskHandle_t current_owner;

	if(adbms_spi_mutex == NULL)
	{
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_ACQUIRE_FAILED);
	}

	wait_start = osKernelGetTickCount();
	if(osMutexAcquire(adbms_spi_mutex, AMS_ADBMS_MUTEX_TIMEOUT_TICKS) != osOK)
	{
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_ACQUIRE_FAILED);
	}

	acquired_tick = osKernelGetTickCount();
	current_owner = xTaskGetCurrentTaskHandle();
	taskENTER_CRITICAL();
	if(app.adbms_spi_lock_acquire_count != UINT32_MAX)
	{
		app.adbms_spi_lock_acquire_count++;
	}
	uint32_t wait_ticks = (uint32_t)(acquired_tick - wait_start);
	if(wait_ticks != 0u)
	{
		if(app.adbms_spi_lock_contention_count != UINT32_MAX)
		{
			app.adbms_spi_lock_contention_count++;
		}
		if(wait_ticks > app.adbms_spi_lock_max_wait_ticks)
		{
			app.adbms_spi_lock_max_wait_ticks = wait_ticks;
		}
	}

	if(adbms_spi_lock_depth == 0u)
	{
		adbms_spi_owner = current_owner;
		adbms_spi_lock_depth = 1u;
		adbms_spi_lock_acquired_tick = acquired_tick;
	}
	else if(adbms_spi_owner == current_owner)
	{
		if(adbms_spi_lock_depth != UINT16_MAX)
		{
			adbms_spi_lock_depth++;
		}
	}
	else
	{
		if(app.adbms_spi_lock_violation_count != UINT32_MAX)
		{
			app.adbms_spi_lock_violation_count++;
		}
		taskEXIT_CRITICAL();
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_ACQUIRE_FAILED);
	}
	taskEXIT_CRITICAL();
}

void adbms_spi_unlock(void)
{
	TaskHandle_t current_owner = xTaskGetCurrentTaskHandle();
	uint32_t now = osKernelGetTickCount();

	if(adbms_spi_mutex == NULL)
	{
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_RELEASE_FAILED);
	}

	taskENTER_CRITICAL();
	if((adbms_spi_lock_depth == 0u) || (adbms_spi_owner != current_owner))
	{
		if(app.adbms_spi_lock_violation_count != UINT32_MAX)
		{
			app.adbms_spi_lock_violation_count++;
		}
		taskEXIT_CRITICAL();
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_RELEASE_FAILED);
	}

	adbms_spi_lock_depth--;
	if(adbms_spi_lock_depth == 0u)
	{
		uint32_t hold_ticks = (uint32_t)(now - adbms_spi_lock_acquired_tick);
		if(hold_ticks > app.adbms_spi_lock_max_hold_ticks)
		{
			app.adbms_spi_lock_max_hold_ticks = hold_ticks;
		}
		adbms_spi_owner = NULL;
		adbms_spi_lock_acquired_tick = 0u;
	}
	taskEXIT_CRITICAL();

	if(osMutexRelease(adbms_spi_mutex) != osOK)
	{
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_RELEASE_FAILED);
	}
}

void ams_current_window_lock(void)
{
	if(current_window_mutex == NULL)
	{
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_ACQUIRE_FAILED);
	}

	if(osMutexAcquire(current_window_mutex,
	                  AMS_CURRENT_WINDOW_MUTEX_TIMEOUT_TICKS) != osOK)
	{
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_ACQUIRE_FAILED);
	}
}

void ams_current_window_unlock(void)
{
	if((current_window_mutex == NULL) ||
	   (osMutexRelease(current_window_mutex) != osOK))
	{
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_RELEASE_FAILED);
	}
}

void app_create(void)
{
	app.hard_fault = false;
	app.soft_fault = false;
	ams_safety_sync_app(&app);
	app.watchdog_feed_count = 0u;
	app.watchdog_block_count = 0u;
	app.watchdog_last_feed_tick = 0u;
	app.watchdog_last_block_reason = AMS_WATCHDOG_BLOCK_NONE;
	app.watchdog_last_logged_block_reason = AMS_WATCHDOG_BLOCK_NONE;
	app.can_error_code = HAL_CAN_ERROR_NONE;
	app.can_busoff_count = 0u;
	app.can_error_count = 0u;
	app.can_recover_count = 0u;
	app.can_last_error_tick = 0u;
	app.can_busoff_fault = false;
	app.can_recover_pending = false;
    ams_rtos_diag_init(&app);
	app.fan_fault = false;
	app.cli_fault = false;
	app.canbus_fault = false;
	app.current_fault = true;
	app.current_sensor_fault = true;
	app.current_overcurrent_warning = false;
	app.current_overcurrent_pending = false;
	app.current_overcurrent_fault = false;
	app.current_fault_latched = false;
	app.current_fault_reason = CURRENT_FAULT_REASON_SENSOR_NOT_READY;
	app.current_fault_latched_reason = CURRENT_FAULT_REASON_NONE;
	app.current_fault_mode = CURRENT_FAULT_MODE_IDLE;
	current_fault_init(&app.current_fault_state);
	app.fuse_fault = false;
	ams_main_fuse_monitor_init(&app.main_fuse_monitor);
	ams_parallel_connection_observer_init(&app.parallel_connection_observer);
	app.temp_fault = true;
	app.temp_valid = false;
	app.temp_read_fault = true;
	app.temp_warning = false;
	app.temp_fan_max = true;
	app.temp_charge_stop = false;
	app.temp_overtemp_pending = false;
	app.overtemp_fault = false;
	app.severe_overtemp_fault = false;
	app.temp_fault_latched = false;
	app.temp_fault_reason = TEMPERATURE_FAULT_REASON_NOT_READY;
	app.temp_fault_pending_reason = TEMPERATURE_FAULT_REASON_NONE;
	app.temp_fault_latched_reason = TEMPERATURE_FAULT_REASON_NONE;
	app.temp_fault_pending_ms = 0u;
	app.temp_usable_sensor_count = 0u;
	app.temp_updated_sensor_count = 0u;
	app.temp_stale_sensor_count = 0u;
	app.temp_invalid_sensor_count = 0u;
	app.temp_open_sensor_count = 0u;
	app.temp_short_sensor_count = 0u;
	app.temp_jump_sensor_count = 0u;
	app.temp_rate_rise_sensor_count = 0u;
	app.temp_filtered_max = 0.0f;
	app.temp_filtered_avg = 0.0f;
	app.temp_max_rate_c_per_s = 0.0f;
	app.temp_max_rate_seg = 0u;
	app.temp_max_rate_sensor = 0u;
	app.max_temp_seg = 0u;
	app.max_temp_sensor = 0u;
	app.min_temp_seg = 0u;
	app.min_temp_sensor = 0u;
	temperature_fault_init(&app.temp_fault_state);
	app.voltage_fault = true;
	app.voltage_valid = false;
	app.voltage_read_fault = true;
	app.voltage_warning = false;
	app.charge_voltage_stop = false;
	app.overvoltage_fault = false;
	app.undervoltage_fault = false;
	app.voltage_fault_latched = false;
	app.voltage_fault_reason = VOLTAGE_FAULT_REASON_NOT_READY;
	app.voltage_fault_latched_reason = VOLTAGE_FAULT_REASON_NONE;
	app.voltage_usable_cell_count = 0u;
	app.voltage_updated_cell_count = 0u;
	app.voltage_stale_cell_count = 0u;
	app.voltage_pec_fail_cell_count = 0u;
	app.voltage_jump_cell_count = 0u;
	app.voltage_stuck_cell_count = 0u;
	app.voltage_max_delta_mv = 0u;
	app.voltage_max_delta_seg = 0u;
	app.voltage_max_delta_cell = 0u;
	app.max_voltage_seg = 0u;
	app.max_voltage_cell = 0u;
	app.min_voltage_seg = 0u;
	app.min_voltage_cell = 0u;
	voltage_fault_init(&app.voltage_fault_state);
	app.estimator_fault = false;
	ams_power_state_init(&app.power_state);
	app.power_can_snapshot = app.power_state.can_snapshot;
	ams_mission_request_init(&app.mission_request);
	app.power_limit_fault = true;
	ams_current_window_init(&app.current_window, osKernelGetTickCount());
	ams_measurement_store_init(&app.measurement_store);

	app.charger_fault = false;
	app.adbms_diag_fault = false;
	app.adbms_config_fault = false;
	app.adbms_status_fault = false;
	app.adbms_open_wire_fault = false;
	app.adbms_open_wire_restore_fault = false;
	app.adbms_open_wire_last_path = ADBMS6830_OPEN_WIRE_PATH_C;
	memset(app.adbms_sense_path_open_mask, 0,
	       sizeof(app.adbms_sense_path_open_mask));
	memset(app.adbms_sense_path_open_sticky_mask, 0,
	       sizeof(app.adbms_sense_path_open_sticky_mask));
	app.adbms_balance_write_fault = false;
    app.adbms_urgent_mute_requested = false;
    app.adbms_mute_asserted = false;
    app.adbms_balance_durable_zero_verified = false;
    app.adbms_balance_inhibit_reason = ACCUMULATOR_BALANCE_INHIBIT_NONE;
    app.adbms_urgent_mute_request_count = 0u;
    app.adbms_urgent_mute_service_count = 0u;
    app.adbms_urgent_mute_fail_count = 0u;
    app.adbms_aux2_diag_count = 0u;
    app.adbms_aux2_diag_fail_count = 0u;
    app.adbms_aux2_next_due_tick = 0u;
    app.adbms_aux2_next_sensor = 0u;
    app.adbms_therm_ow_diag_count = 0u;
    app.adbms_therm_ow_diag_fail_count = 0u;
    app.adbms_therm_ow_last_tick = 0u;
    app.adbms_therm_ow_next_sensor = 0u;
    app.adbms_s_diag_last_tick = 0u;
    app.adbms_post_fail_count = 0u;
	app.adbms_fault_active_mask = 0u;
	app.adbms_fault_latched_mask = 0u;
	app.adbms_first_fault_mask = 0u;
	app.adbms_fault_injection_mask = 0u;
	app.adbms_fault_injection_cell = 0u;
	app.adbms_c_authority_mask = 0u;
	app.adbms_c_authority_valid = false;
	app.adbms_voltage_scan_attempted = false;
	app.adbms_last_voltage_scan_ok = false;
	app.adbms_voltage_redundancy_degraded = false;
	app.adbms_lifecycle_state = AMS_ADBMS_STATE_OFFLINE;
	app.adbms_lifecycle_previous_state = AMS_ADBMS_STATE_OFFLINE;
	app.adbms_lifecycle_reason = AMS_ADBMS_STATE_REASON_BOOT;
	app.adbms_lifecycle_transition_count = 0u;
	app.adbms_lifecycle_last_transition_tick = 0u;
	app.adbms_first_fault_tick = 0u;
	app.adbms_last_fault_tick = 0u;
	app.adbms_last_recovery_tick = 0u;
	app.adbms_fault_transition_count = 0u;
	app.adbms_device_reset_count = 0u;
	app.adbms_device_reset_driver_count_seen = 0u;
	app.adbms_last_device_reset_mask = 0u;
	app.adbms_c_last_valid_tick = 0u;
	app.adbms_s_last_valid_tick = 0u;
	app.adbms_temp_last_valid_tick = 0u;
	app.adbms_status_last_valid_tick = 0u;
	app.adbms_config_last_valid_tick = 0u;
	app.adbms_identity_last_valid_tick = 0u;
	app.adbms_config_expected_fingerprint = 0u;
	app.adbms_config_readback_fingerprint = 0u;
	memset(app.adbms_balance_shadow_plan,
	       0,
	       sizeof(app.adbms_balance_shadow_plan));
	app.adbms_balance_shadow_plan_tick = 0u;
	app.adbms_scan_active = false;
	app.adbms_balance_active = false;
	app.adbms_scan_count = 0u;
	app.adbms_status_diag_count = 0u;
	app.adbms_config_diag_count = 0u;
	app.adbms_open_wire_diag_count = 0u;
	app.adbms_balance_write_fail_count = 0u;
	app.adbms_balance_recovery_count = 0u;
	app.adbms_scan_deadline_miss_count = 0u;
	app.adbms_last_scan_duration_ms = 0u;
	app.adbms_max_scan_duration_ms = 0u;
    app.adbms_last_scan_cpu_us = 0u;
    app.adbms_max_scan_cpu_us = 0u;
    app.adbms_last_scan_yield_us = 0u;
    app.adbms_max_scan_yield_us = 0u;
	app.adbms_last_schedule_interval_ms = AMS_ADBMS_TASK_PERIOD_MS;
	app.adbms_last_balance_on_ms = 0u;
	app.adbms_last_balance_off_ms = 0u;
	app.adbms_balance_apply_tick = 0u;
	app.adbms_spi_lock_acquire_count = 0u;
	app.adbms_spi_lock_contention_count = 0u;
	app.adbms_spi_lock_max_wait_ticks = 0u;
	app.adbms_spi_lock_max_hold_ticks = 0u;
	app.adbms_spi_lock_violation_count = 0u;
	app.adbms_last_diag_status = HAL_OK;
	app.task_heartbeat_fault = false;
	app.logger_heartbeat_fault = false;
	app.heartbeat_stale_mask = 0u;
	app.heartbeat_seen_mask = 0u;
    app.bms_state     = false;
#if AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT || \
    (AMS_HW_BRINGUP && !AMS_HW_BRINGUP_BMS_OK_RELEASED_DEFAULT)
	app.bms_output_inhibit = true;
#else
	app.bms_output_inhibit = false;
#endif
	app.bms_output_block_count = 0u;
	app.bms_supervisor_ready = false;
#if AMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT
	app.balance_inhibit = true;
#else
	app.balance_inhibit = false;
#endif

	/* AIR_CONTROL_MCU is only a command/control-voltage sense.  It must never be
	 * presented as physical contactor feedback.  The auxiliary monitor remains
	 * disabled in this hardware revision; if a build enables its safety gate
	 * before a validated producer exists, initialization holds it fail-closed. */
	app.air_state = false;
	ams_air_monitor_init(&app.air_monitor,
	                     (AMS_ENABLE_AIR_AUX_FEEDBACK != 0));
	app.air_monitor_inputs = (ams_air_monitor_inputs_t){0};
	/* Fail closed until the IMD driver and capture path have produced a
	 * validated result.  The IMD task is currently disabled on this board, so
	 * treating the value as healthy here would be unsafe. */
	app.imd_ok = false;
	app.imd_valid = false;
	app.imd_fault = true;
	app.imd_status = IMD_UNKNOWN;
	app.imd_last_valid_tick = 0u;

	app.fan_state = false;
	app.fan_command_percent = 0.0f;
	app.fan_control_reason = 0u;
	app.fan_set_fail_count = 0u;
	app.fan_last_update_tick = 0u;

	app.state = STATE_START;
	app.state_previous = STATE_NULL;
	app.state_transition_reason = AMS_STATE_TRANSITION_BOOT;
	app.state_transition_count = 0u;
	app.state_transition_last_tick = osKernelGetTickCount();
	app.state_transition_in_progress = false;

	app.max_temp = 0.0;
	app.avg_temp = 0.0;
	app.max_voltage = 0.0;
	app.min_voltage = 0.0;
	app.current = 0.0;
	app.current_valid = false;
	app.current_sample_tick = 0u;
	app.current_sample_sequence = 0u;
	app.current_selected_range = CURRENT_SENSOR_RANGE_UNKNOWN;
	app.current_meas_reason = CURRENT_SENSOR_REASON_ADC_READ;

	board_init(&app.board);
	if(app.board.canbus.init_status != HAL_OK)
	{
		app.canbus_fault = true;
		app.can_recover_pending = true;
		app.can_error_code = app.board.canbus.started ?
			HAL_CAN_ERROR_NOT_READY : HAL_CAN_ERROR_NOT_STARTED;
		app.can_error_count = 1u;
		app.can_last_error_tick = osKernelGetTickCount();
	}
	for(uint8_t fan_index = 0u; fan_index < NFANS; fan_index++)
	{
		if(!app.board.fans[fan_index].initialized)
		{
			app.fan_fault = true;
		}
	}
	ams_heartbeat_init(&app, osKernelGetTickCount());
	/* A compile-enabled IWDG must protect startup as well as steady state.  It
	 * is fed during the bounded heartbeat grace period, then only while the
	 * safety supervisor's complete health gate remains satisfied. */
	ams_safety_watchdog_boot_arm(&app);
	ams_safety_sync_app(&app);
	ams_fault_log_event(AMS_FAULT_LOG_BOOT, 0u, app.reset_flags, app.last_panic_reason);
	set_bms(0);
	adbms_spi_mutex = osMutexNew(&adbms_spi_mutex_attr);
	if(adbms_spi_mutex == NULL)
	{
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_CREATE_FAILED);
	}
	current_window_mutex = osMutexNew(&current_window_mutex_attr);
	if(current_window_mutex == NULL)
	{
		adbms_spi_lock_panic(AMS_PANIC_MUTEX_CREATE_FAILED);
	}

    ams_adbms_transition_state(&app,
                               AMS_ADBMS_STATE_WAKING,
                               AMS_ADBMS_STATE_REASON_SCAN_BEGIN,
                               osKernelGetTickCount());
	accumulator_init(&app.acc,
					 app.board.stm32f767z.hspi6,
					 CS_A_GPIO_Port,
					 CS_B_GPIO_Port,
					 CS_A_Pin,
					 CS_B_Pin,
					 app.board.stm32f767z.htim1);
    app.adbms_mute_asserted = app.acc.last_balance_mute_ok;
    app.adbms_balance_durable_zero_verified =
        app.acc.last_balance_durable_zero_verified;
    app.adbms_balance_inhibit_reason =
        (accumulator_balance_inhibit_reason_t)app.acc.last_balance_inhibit_reason;
    app.adbms_post_fail_count = app.acc.smb.post.fail_count;
    if(app.acc.smb_ready)
    {
        ams_adbms_transition_state(&app,
                                   AMS_ADBMS_STATE_IDENTIFIED,
                                   AMS_ADBMS_STATE_REASON_IDENTITY_OK,
                                   osKernelGetTickCount());
        ams_adbms_transition_state(&app,
                                   AMS_ADBMS_STATE_CONFIGURING,
                                   AMS_ADBMS_STATE_REASON_CONFIG_OK,
                                   osKernelGetTickCount());
        ams_adbms_transition_state(&app,
                                   AMS_ADBMS_STATE_MEASURING,
                                   AMS_ADBMS_STATE_REASON_SCAN_BEGIN,
                                   osKernelGetTickCount());
    }
    else
    {
        ams_adbms_transition_state(&app,
                                   AMS_ADBMS_STATE_FAULTED,
                                   AMS_ADBMS_STATE_REASON_FAULT_ACTIVE,
                                   osKernelGetTickCount());
    }
#if AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP
	/* Compile-time only degraded authority.  There is intentionally no runtime
	 * fallback into this state after an S-channel fault. */
	app.adbms_voltage_redundancy_degraded = true;
	app.adbms_fault_active_mask |= AMS_ADBMS_FAULT_VOLTAGE_DEGRADED;
	app.adbms_fault_latched_mask |= AMS_ADBMS_FAULT_VOLTAGE_DEGRADED;
#endif
#if !AMS_HIL_REPLACE_ADBMS
	if(!app.acc.delay_timer_ready || !app.acc.smb_transport_ready)
	{
		/* ADBMS startup is not trustworthy without the microsecond timer and a
		 * transport-clean identity/configuration/Status image. */
		app.adbms_status_fault = true;
		app.adbms_diag_fault = true;
		app.adbms_last_diag_status = !app.acc.smb_transport_ready ?
		                             app.acc.smb_init_status :
		                             app.acc.delay_timer_status;
	}
	else if(!app.acc.smb_ready)
	{
		/* The bus and primary measurement path are usable, but a safety-policy
		 * class such as CSxFLT still blocks the normal redundant build.  Preserve
		 * the inhibit without mislabeling the Status transport as failed. */
		app.adbms_status_fault = false;
		app.adbms_diag_fault = true;
		app.adbms_last_diag_status = HAL_OK;
	}
#endif
	(void)cli_uart_start_rx(&app.board.cli);

	app.cli_task = cli_task_start(&app);
	app.fan_task = fan_task_start(&app);
	app.error_task = error_task_start(&app);
	app.canbus_task = canbus_task_start(&app);
#if AMS_ENABLE_AIR_AUX_FEEDBACK
	/* The target build guards above require a reviewed board adapter and period.
	 * The task computes locally and atomically publishes the monitor snapshot. */
	app.air_task = air_task_start(&app);
#else
	/* Do not start the legacy task: AIR_CONTROL_MCU is already sampled by the
	 * supervisor and cannot determine physical contactor position. */
	app.air_task = NULL;
#endif
#if AMS_ENABLE_IMD
	app.imd_task = imd_task_start(&app);
#endif
	app.current_task = current_task_start(&app);
	app.adbms_task = adbms_task_start(&app);
	app.estimator_task = estimator_task_start(&app);

	if((app.cli_task == NULL) ||
	   (app.fan_task == NULL) ||
	   (app.error_task == NULL) ||
	   (app.canbus_task == NULL) ||
#if AMS_ENABLE_AIR_AUX_FEEDBACK
	   (app.air_task == NULL) ||
#endif
#if AMS_ENABLE_IMD
	   (app.imd_task == NULL) ||
#endif
	   (app.current_task == NULL) ||
	   (app.adbms_task == NULL) ||
	   (app.estimator_task == NULL))
	{
		ams_safety_panic(AMS_PANIC_TASK_CREATE_FAILED);
		for(;;)
		{
		}
	}

	/* BMS_OK assertion is owned exclusively by the high-priority error/safety
	 * supervisor task.  Other contexts may only force the output low. */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
#if AMS_ENABLE_IMD
	if((htim != NULL) && (htim == app.board.imd.htim))
	{
		imd_capture_event(&app.board.imd, osKernelGetTickCount());
	}
#else
	(void)htim;
#endif
}

void set_bms(bool state)
{
	bool previous = app.bms_state;
	bool assertion_owner = false;

#if !AMS_PROFILE_BMS_RUNTIME_AUTHORITY_ALLOWED
    /* Observation profiles are immutable no-authority images. This final hardware writer
     * does not trust the mutable runtime inhibit as the only barrier.  Convert
     * every attempted assertion into an ordinary fail-low call so the urgent
     * ASIC balance-mute path is requested as well. */
    if(state)
    {
        if(app.bms_output_block_count != UINT32_MAX)
        {
            app.bms_output_block_count++;
        }
        app.bms_output_inhibit = true;
        state = false;
    }
#endif

    /* Any fail-low BMS_OK decision also requests the ASIC-native fast balance
     * kill.  The ADBMS task services this flag at interruptible wait boundaries
     * while it owns the recursive transport mutex; high-priority safety tasks
     * never block waiting for SPI ownership. */
    if(!state && !app.adbms_urgent_mute_requested)
    {
        app.adbms_urgent_mute_requested = true;
        if(app.adbms_urgent_mute_request_count != UINT32_MAX)
        {
            app.adbms_urgent_mute_request_count++;
        }
    }

	if(state && (app.error_task != NULL))
	{
		assertion_owner = (xTaskGetCurrentTaskHandle() == app.error_task);
	}

	if(state && (!assertion_owner ||
	             app.bms_output_inhibit ||
	             ams_safety_panic_active()))
	{
		app.bms_output_block_count++;
		app.bms_state = false;
		HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, GPIO_PIN_RESET);
		if(previous)
		{
			ams_fault_log_event(AMS_FAULT_LOG_BMS_OK_DROPPED, 0u, app.hard_fault, app.soft_fault);
		}
		return;
	}

	app.bms_state = state;
	HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);

	if(state && !previous)
	{
		ams_fault_log_event(AMS_FAULT_LOG_BMS_OK_ASSERTED, 0u, app.hard_fault, app.soft_fault);
	}
	else if(!state && previous)
	{
		ams_fault_log_event(AMS_FAULT_LOG_BMS_OK_DROPPED, 0u, app.hard_fault, app.soft_fault);
	}
}
