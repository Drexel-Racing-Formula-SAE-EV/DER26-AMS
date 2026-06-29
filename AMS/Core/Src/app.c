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

app_data_t app = {0};

void app_create()
{
	app.hard_fault = false;
	app.soft_fault = false;
	app.fan_fault = false;
	app.cli_fault = false;
	app.canbus_fault = false;
	app.current_fault = false;
	app.fuse_fault = false;
	app.temp_fault = false;
	app.voltage_fault = false;

    app.charger_fault = false;
    app.bms_state     = false;

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

	board_init(&app.board);
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

	assert(app.cli_task != NULL);
	assert(app.fan_task != NULL);
	assert(app.error_task != NULL);
	assert(app.canbus_task != NULL);
	assert(app.current_task != NULL);
	assert(app.adbms_task != NULL);

	if((app.cli_task == NULL) ||
	   (app.fan_task == NULL) ||
	   (app.error_task == NULL) ||
	   (app.canbus_task == NULL) ||
	   (app.current_task == NULL) ||
	   (app.adbms_task == NULL))
	{
		set_bms(0);
		return;
	}

	set_bms(1);
}

void set_bms(bool state)
{
	app.bms_state = state;
	HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, state);
}
