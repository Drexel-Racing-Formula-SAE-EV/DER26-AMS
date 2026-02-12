/*
 * board.h
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 */

#ifndef INC_BOARD_H_
#define INC_BOARD_H_

#include "ext_drivers/fans.h"
#include "ext_drivers/stm32f767z.h"
#include "ext_drivers/imd.h"
#include "ext_drivers/cli.h"
#include "ext_drivers/canbus.h"
#include "ext_drivers/current_sensor.h"
#include "ext_drivers/accumulator.h"

#define NFANS 6
#define CANBUS_STATUS	0x1 // Notification bit value for Status messages

typedef struct
{
	stm32f767z_t stm32f767z;
	// todo: change actual fan struct org. this is a demo
	fan_t fans[NFANS];
	imd_t imd;
	cli_device_t cli;
	canbus_device_t canbus;
	current_sensor_t current_sensor;
} board_t;

void board_init(board_t *board);

#endif /* INC_BOARD_H_ */
