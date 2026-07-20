/*
*   imd.h
*   Created: 2/5/2024
*   Author: Brendan Hoag
*   Modified by: Mahad Faisal (major firmware updates, 2026)
*   Purpose: Device driver for IR151-3204 ground fault monitoring system
*   datasheet: https://www.benderinc.com/products/ground-fault-monitoring-ungrounded/isometer-ir155-03-04-series/
*/
#ifndef INC_IMD_H
#define INC_IMD_H

#include <stdint.h>
#include <stdbool.h>
#include "ams_build_profile.h"
#include "stm32f7xx_hal.h"

#ifndef AMS_ENABLE_IMD
#define AMS_ENABLE_IMD 0
#endif

#ifndef AMS_IMD_CAPTURE_TIMEOUT_MS
#define AMS_IMD_CAPTURE_TIMEOUT_MS 250u
#endif

/*
* enum: imd_status_t
* ------------------
* Represents a status message read from the IMD
* Each status comes in as a multiple of 10Hz
* To find current status, divide reading from PWM by 10
*/
typedef enum
{
    IMD_SHORT_TO_CHASSIS_GROUND = 0,
    IMD_NORMAL                  = 1,
    IMD_UNDERVOLT               = 2,
    IMD_SPEED_START             = 3,
    IMD_DEVICE_ERROR            = 4,
    IMD_GROUND_FAULT            = 5,
    IMD_UNKNOWN                 = 0xFF
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
	uint32_t high_channel;
	uint32_t total_channel;
	GPIO_TypeDef *status_port;
	uint16_t status_pin;
	uint32_t high_count;
	uint32_t total_count;
	float duty;
	float freq;
	bool capture_started;
	volatile bool capture_seen;
	/* The ISR publishes one period/high/timestamp tuple under this sequence
	 * counter. Even values are stable; odd values mean a write is in progress. */
	volatile uint32_t capture_sequence;
	volatile uint32_t captured_high_count;
	volatile uint32_t captured_total_count;
	volatile uint32_t capture_count;
	volatile uint32_t last_capture_tick;
	HAL_StatusTypeDef init_status;
	int ret;
} imd_t;

/*
* function: imd_init
* ------------------
*
* imd: a pointer to and imd_t we want to initialize
*/
void imd_init(imd_t *dev, uint32_t clock_freq, TIM_HandleTypeDef *htim, TIM_TypeDef *tim, uint32_t high_channel, uint32_t total_channel, GPIO_TypeDef *status_port, uint16_t status_pin);

int imd_read(imd_t *dev);
int imd_read_at(imd_t *dev, uint32_t now);
void imd_capture_event(imd_t *dev, uint32_t now);

#endif
