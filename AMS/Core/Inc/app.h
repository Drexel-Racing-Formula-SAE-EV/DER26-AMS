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
#include "ext_drivers/current_fault.h"
#include "ext_drivers/voltage_fault.h"

#define VER_MAJOR 0
#define VER_MINOR 1
#define VER_BUG   0

#ifndef AMS_HW_BRINGUP
#define AMS_HW_BRINGUP 0
#endif

#ifndef AMS_HW_BRINGUP_BMS_OK_RELEASED_DEFAULT
#define AMS_HW_BRINGUP_BMS_OK_RELEASED_DEFAULT 0
#endif

#define ERR_FREQ 20
#define CLI_FREQ 20
#define AIR_FREQ 2 // used to be 10
#define CURRENT_FREQ 50
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
/* Voltage thresholds are defined in ext_drivers/voltage_fault.h in mV. */

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
	bool current_valid;
	current_sensor_range_t current_selected_range;
	current_sensor_reason_t current_meas_reason;

	bool hard_fault;
	bool soft_fault;

	bool fan_fault;
	bool cli_fault;
	bool canbus_fault;
	bool current_fault;
	bool current_sensor_fault;
	bool current_overcurrent_warning;
	bool current_overcurrent_pending;
	bool current_overcurrent_fault;
	bool current_fault_latched;
	current_fault_reason_t current_fault_reason;
	current_fault_reason_t current_fault_latched_reason;
	current_fault_mode_t current_fault_mode;
	current_fault_state_t current_fault_state;
	bool fuse_fault;
	bool temp_fault;
	bool voltage_fault;
	bool voltage_valid;
	bool voltage_read_fault;
	bool voltage_warning;
	bool charge_voltage_stop;
	bool overvoltage_fault;
	bool undervoltage_fault;
	bool voltage_fault_latched;
	voltage_fault_reason_t voltage_fault_reason;
	voltage_fault_reason_t voltage_fault_latched_reason;
	voltage_fault_state_t voltage_fault_state;
	uint16_t voltage_usable_cell_count;
	uint16_t voltage_updated_cell_count;
	uint16_t voltage_stale_cell_count;
	uint8_t max_voltage_seg;
	uint8_t max_voltage_cell;
	uint8_t min_voltage_seg;
	uint8_t min_voltage_cell;
	bool estimator_fault;

	bool air_state;
	bool imd_ok;
	imd_status_t imd_status;
    bool fan_state;

    bool charger_fault;
    bool bms_state;
    bool bms_output_inhibit;
    uint32_t bms_output_block_count;

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
void adbms_spi_lock(void);
void adbms_spi_unlock(void);

#endif /* INC_APP_H_ */
