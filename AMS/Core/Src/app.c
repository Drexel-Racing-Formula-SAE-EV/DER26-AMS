/*
 * app.c
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 */

#include "app.h"

#include <assert.h>

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

app_data_t app = {0};
static osMutexId_t adbms_spi_mutex;

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
	app.max_voltage_seg = 0u;
	app.max_voltage_cell = 0u;
	app.min_voltage_seg = 0u;
	app.min_voltage_cell = 0u;
	voltage_fault_init(&app.voltage_fault_state);
	app.estimator_fault = false;

    app.charger_fault = false;
    app.bms_state     = false;
#if AMS_HW_BRINGUP && !AMS_HW_BRINGUP_BMS_OK_RELEASED_DEFAULT
	app.bms_output_inhibit = true;
#else
	app.bms_output_inhibit = false;
#endif
	app.bms_output_block_count = 0u;

	app.air_state = false;
	app.imd_ok = true;
	app.imd_status = IMD_NORMAL;

	app.fan_state = false;

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
	set_bms(0);
	adbms_spi_mutex = osMutexNew(NULL);

	accumulator_init(&app.acc,
					 app.board.stm32f767z.hspi6,
					 CS_A_GPIO_Port,
					 CS_B_GPIO_Port,
					 CS_A_Pin,
					 CS_B_Pin,
					 app.board.stm32f767z.htim1);
	HAL_UART_Receive_IT(app.board.stm32f767z.huart3, &app.board.cli.c, 1);

	app.cli_task = cli_task_start(&app);
	app.fan_task = fan_task_start(&app);
	app.error_task = error_task_start(&app);
	app.canbus_task = canbus_task_start(&app);
//	app.air_task = air_task_start(&app);
//	app.imd_task = imd_task_start(&app);
	app.current_task = current_task_start(&app);
	app.adbms_task = adbms_task_start(&app);
	app.estimator_task = estimator_task_start(&app);

	assert(app.cli_task != NULL);
	assert(app.fan_task != NULL);
	assert(app.error_task != NULL);
	assert(app.canbus_task != NULL);
	assert(app.current_task != NULL);
	assert(app.adbms_task != NULL);
	assert(app.estimator_task != NULL);

	if((app.cli_task == NULL) ||
	   (app.fan_task == NULL) ||
	   (app.error_task == NULL) ||
	   (app.canbus_task == NULL) ||
	   (app.current_task == NULL) ||
	   (app.adbms_task == NULL) ||
	   (app.estimator_task == NULL))
	{
		set_bms(0);
		return;
	}

	/* BMS_OK is asserted by adbms_task after the first clean measurement pass. */
}

void set_bms(bool state)
{
	if(state && app.bms_output_inhibit)
	{
		app.bms_output_block_count++;
		app.bms_state = false;
		HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, GPIO_PIN_RESET);
		return;
	}

	app.bms_state = state;
	HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
