/*
*   imd.h
*   Created: 2/5/2024
*   Author: Brendan Hoag
*   Purpose: Device driver for IR151-3204 ground fault monitoring system
*   datasheet: https://www.benderinc.com/products/ground-fault-monitoring-ungrounded/isometer-ir155-03-04-series/
*/
#ifndef INC_IMD_H
#define INC_IMD_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"

/*
* enum: imd_status_t
* ------------------
* Represents a status message read from the IMD
* Each status comes in as a multiple of 10Hz
* To find current status, divide reading from PWM by 10
*/
typedef enum
{
    IMD_SHORT_TO_CHASSIS_GROUND,
    IMD_NORMAL,
    IMD_UNDERVOLT,
    IMD_SPEED_START,
    IMD_DEVICE_ERROR,
    IMD_GROUND_FAULT
} imd_status_t;

/*
* struct: imd_t
* -------------
* Represents an IR151-3204 ground fault monitoring system
*
* OK_HS: a boolean representing the high-level status of the system
*   - Corresponds to pin 8 OK_HS
*   - A high bit is good, a low bit is some error state
*
* status: an imd_status_t representing the status of the IMD
*   - Should be NORMAL_CONDITION (1) if OK_HS is 1
*   - Will be another status if OK_HS is 0
*   - Update based on frequency
*
* timer: a TIM_TypeDef pointer representing the associated timer of the IMD
*
* frequency: a float representing the data out PWM signal on the high side
*   - Corresponds to pin 5 M_HS on IR151-3204, pin 6 on IR151-3203
*   - Tells us what state the IMD is in
*
* duty_cycle: a float representing the duty cycle of the IMD (expressed as a percent)
*/
typedef struct
{
    bool OK_HS;
    imd_status_t status;
	uint32_t clock_freq;
	TIM_HandleTypeDef *htim;
	TIM_TypeDef *tim;
	HAL_TIM_ActiveChannel high_channel;
	HAL_TIM_ActiveChannel total_channel;
	GPIO_TypeDef *status_port;
	uint16_t status_pin;
	uint32_t high_count;
	uint32_t total_count;
	float duty;
	float freq;
	int ret;
} imd_t;

/*
* function: imd_init
* ------------------
*
* imd: a pointer to and imd_t we want to initialize
*/
void imd_init(imd_t *dev, uint32_t clock_freq, TIM_HandleTypeDef *htim, TIM_TypeDef *tim, HAL_TIM_ActiveChannel high_channel, HAL_TIM_ActiveChannel total_channel, GPIO_TypeDef *status_port, uint16_t status_pin);

int imd_read(imd_t *dev);

#endif
