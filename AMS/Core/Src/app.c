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

app_data_t app = {0};
static osMutexId_t adbms_spi_mutex;

uint32_t ams_heartbeat_timeout_ms(ams_heartbeat_id_t id)
{
	switch(id)
	{
	case AMS_HEARTBEAT_ADBMS:   return AMS_HEARTBEAT_ADBMS_TIMEOUT_MS;
	case AMS_HEARTBEAT_CURRENT: return AMS_HEARTBEAT_CURRENT_TIMEOUT_MS;
	case AMS_HEARTBEAT_TEMP:    return AMS_HEARTBEAT_TEMP_TIMEOUT_MS;
	case AMS_HEARTBEAT_CAN:     return AMS_HEARTBEAT_CAN_TIMEOUT_MS;
	case AMS_HEARTBEAT_LOGGER:  return AMS_HEARTBEAT_LOGGER_TIMEOUT_MS;
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
	default:                    return "unknown";
	}
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
		data->heartbeat.count[i] = 0u;
	}

	data->task_heartbeat_fault = false;
	data->logger_heartbeat_fault = false;
	data->heartbeat_stale_mask = 0u;
	data->heartbeat_seen_mask = 0u;
}

void ams_heartbeat_kick(app_data_t *data, ams_heartbeat_id_t id, uint32_t now)
{
	if((data == NULL) || (id >= AMS_HEARTBEAT_COUNT))
	{
		return;
	}

	data->heartbeat.last_tick[id] = now;
	data->heartbeat.count[id]++;
	data->heartbeat.seen_mask |= AMS_HEARTBEAT_BIT(id);
	data->heartbeat.seen_mask &= (uint16_t)((1u << (uint16_t)AMS_HEARTBEAT_COUNT) - 1u);
	data->heartbeat_seen_mask = data->heartbeat.seen_mask;
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

	return data->heartbeat.stale_mask;
}

void adbms_spi_lock(void)
{
	if(adbms_spi_mutex != NULL)
	{
		(void)osMutexAcquire(adbms_spi_mutex, osWaitForever);
	}
}

void adbms_spi_unlock(void)
{
	if(adbms_spi_mutex != NULL)
	{
		(void)osMutexRelease(adbms_spi_mutex);
	}
}

void app_create()
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

	app.charger_fault = false;
	app.adbms_diag_fault = false;
	app.adbms_config_fault = false;
	app.adbms_status_fault = false;
	app.adbms_open_wire_fault = false;
	app.adbms_scan_active = false;
	app.adbms_scan_count = 0u;
	app.adbms_status_diag_count = 0u;
	app.adbms_config_diag_count = 0u;
	app.adbms_open_wire_diag_count = 0u;
	app.adbms_last_diag_status = HAL_OK;
	app.task_heartbeat_fault = false;
	app.logger_heartbeat_fault = false;
	app.heartbeat_stale_mask = 0u;
	app.heartbeat_seen_mask = 0u;
    app.bms_state     = false;
#if AMS_HW_BRINGUP && !AMS_HW_BRINGUP_BMS_OK_RELEASED_DEFAULT
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

	app.air_state = false;
	/* Fail closed until the IMD driver and capture path have produced a
	 * validated result.  The IMD task is currently disabled on this board, so
	 * treating the value as healthy here would be unsafe. */
	app.imd_ok = false;
	app.imd_valid = false;
	app.imd_fault = true;
	app.imd_status = IMD_UNKNOWN;

	app.fan_state = false;
	app.fan_command_percent = 0.0f;
	app.fan_control_reason = 0u;
	app.fan_set_fail_count = 0u;
	app.fan_last_update_tick = 0u;

	app.state = STATE_START;

	app.max_temp = 0.0;
	app.avg_temp = 0.0;
	app.max_voltage = 0.0;
	app.min_voltage = 0.0;
	app.current = 0.0;
	app.current_valid = false;
	app.current_selected_range = CURRENT_SENSOR_RANGE_UNKNOWN;
	app.current_meas_reason = CURRENT_SENSOR_REASON_ADC_READ;

	board_init(&app.board);
	ams_heartbeat_init(&app, osKernelGetTickCount());
	ams_safety_sync_app(&app);
	ams_fault_log_event(AMS_FAULT_LOG_BOOT, 0u, app.reset_flags, app.last_panic_reason);
	set_bms(0);
	adbms_spi_mutex = osMutexNew(NULL);

	accumulator_init(&app.acc,
					 app.board.stm32f767z.hspi6,
					 CS_A_GPIO_Port,
					 CS_B_GPIO_Port,
					 CS_A_Pin,
					 CS_B_Pin,
					 app.board.stm32f767z.htim1);
	(void)cli_uart_start_rx(&app.board.cli);

	app.cli_task = cli_task_start(&app);
	app.fan_task = fan_task_start(&app);
	app.error_task = error_task_start(&app);
	app.canbus_task = canbus_task_start(&app);
//	app.air_task = air_task_start(&app);
//	app.imd_task = imd_task_start(&app);
	app.current_task = current_task_start(&app);
	app.adbms_task = adbms_task_start(&app);
	app.estimator_task = estimator_task_start(&app);

	if((app.cli_task == NULL) ||
	   (app.fan_task == NULL) ||
	   (app.error_task == NULL) ||
	   (app.canbus_task == NULL) ||
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

void set_bms(bool state)
{
	bool previous = app.bms_state;
	bool assertion_owner = false;

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
