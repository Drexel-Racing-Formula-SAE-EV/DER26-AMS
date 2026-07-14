/*
 * fans.h
 *
 *  Created on: Jan 22, 2024
 *      Author: Cassius Garcia
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef INC_FANS_H_
#define INC_FANS_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f7xx_hal.h"

typedef enum
{
    FAN_CONTROL_REASON_OFF_COOL = 0,
    FAN_CONTROL_REASON_RAMP,
    FAN_CONTROL_REASON_MIN_HYSTERESIS,
    FAN_CONTROL_REASON_CHARGE_WARM,
    FAN_CONTROL_REASON_MAX_TEMP,
    FAN_CONTROL_REASON_TEMP_INVALID,
    FAN_CONTROL_REASON_TEMP_FAULT,
    FAN_CONTROL_REASON_DRIVER_FAULT
} fan_control_reason_t;

typedef struct
{
	TIM_TypeDef *timer;
	TIM_HandleTypeDef *htim;
	int channel;
	uint64_t max_timer_val;
	volatile uint32_t *CCR;
	float duty_cycle;
	bool initialized;
	HAL_StatusTypeDef init_status;
} fan_t;

int set_fan_percent(fan_t *fan, float percent);
const char *fan_control_reason_str(uint8_t reason);

int fan_init(fan_t *fan, TIM_TypeDef *timer, TIM_HandleTypeDef *htim, uint64_t max_timer_val, volatile uint32_t *CCR, int channel);

#endif /* INC_FANS_H_ */
