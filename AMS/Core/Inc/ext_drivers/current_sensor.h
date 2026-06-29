/*
 * current_sensor.h
 *
 *  Created on: Mar 3th, 2024
 *      Author: Justin Nguyen
 */

#ifndef INC_CURRENT_SENSOR_H_
#define INC_CURRENT_SENSOR_H_

#include <stdint.h>
#include <stm32f7xx_hal.h>

typedef struct
{
	float current;
	float voltage_high;
    float voltage_low;
	float current_high;
	float current_low;
	uint16_t count_high;
    uint16_t count_low;
    ADC_HandleTypeDef *hadc_high;
    ADC_HandleTypeDef *hadc_low;
    uint32_t channel_high;
    uint32_t channel_low;
} current_sensor_t;

void current_sensor_init(current_sensor_t *dev,
						 ADC_HandleTypeDef *hadc_low,
						 ADC_HandleTypeDef *hadc_high,
						 uint32_t channel_low,
						 uint32_t channel_high
						 );
float current_sensor_current_read(current_sensor_t *dev);
float current_sensor_convert(current_sensor_t *dev);

#endif /* INC_CURRENT_SENSOR_H_ */
