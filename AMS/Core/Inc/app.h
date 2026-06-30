/*
 * app.h
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f7xx_hal.h"
#include "main.h"
#include "board.h"
#include "ext_drivers/accumulator.h"
#include "estimator/ams_soc_ekf.h"

#define VER_MAJOR 0
#define VER_MINOR 1
#define VER_BUG   0

#define ERR_FREQ 20
#define CLI_FREQ 20
#define AIR_FREQ 2 // used to be 10
#define CURRENT_FREQ 10
#define ADBMS_FREQ 1 // was 10, testing with 1
#define IMD_FREQ 10
#define FAN_FREQ 5
#define CAN_FREQ 2
#define ESTIMATOR_FREQ 10

// prios taken from DER24 defaults
#define ERR_PRIO 17
#define AIR_PRIO  7
#define CAN_PRIO  8
#define CLI_PRIO 14
#define CUR_PRIO  7
#define FAN_PRIO  7
#define IMD_PRIO  6
#define ADBMS_PRIO  9
#define EST_PRIO 8

#define ECU_CANBUS_ID 0x69u

#define TO_LSB16(x) ((uint16_t)x & 0xff)
#define TO_MSB16(x) ((((uint16_t)x & 0xff00) >> 8) & 0xff)

// TODO: check temp thresholds
#define TEMP_THRESH_H 60.0
#define TEMP_THRESH_L 40.0
#define OVERVOLT 4.2
#define UNDERVOLT 2.5

typedef enum
{
	STATE_NULL,
	STATE_START,
	STATE_CHARGE,
	STATE_DISCARGE,
	STATE_BALANCE,
	STATE_ERROR
} state_t;

typedef struct
{
	float total_voltage;
	float max_voltage;
	float min_voltage;
	float max_temp;
	float avg_temp;
	float current;

	bool hard_fault;
	bool soft_fault;

	bool fan_fault;
	bool cli_fault;
	bool canbus_fault;
	bool current_fault;
	bool fuse_fault;
	bool temp_fault;
	bool voltage_fault;
	bool estimator_fault;

	bool air_state;
	bool imd_ok;
	imd_status_t imd_status;
    bool fan_state;

    bool charger_fault;
    bool bms_state;

	state_t state;

	board_t board;
	accumulator_t acc;
	ams_estimator_t estimator;
	ams_hil_input_t hil;

	TaskHandle_t fan_task;
	TaskHandle_t cli_task;
	TaskHandle_t error_task;
	TaskHandle_t canbus_task;
	TaskHandle_t air_task;
	TaskHandle_t imd_task;
	TaskHandle_t current_task;
	TaskHandle_t adbms_task;
	TaskHandle_t estimator_task;
} app_data_t;

void app_create();
void set_bms(bool state);

#endif /* INC_APP_H_ */
