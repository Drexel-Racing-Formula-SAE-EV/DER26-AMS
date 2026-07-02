/*
 * stm32f407g.h
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 */

#ifndef INC_EXT_DRIVERS_STM32F407G_H_
#define INC_EXT_DRIVERS_STM32F407G_H_

#include <stm32f7xx_hal.h>
#include "cmsis_os.h"

typedef struct
{
	ADC_HandleTypeDef *hadc1;
	ADC_HandleTypeDef *hadc2;

	CAN_HandleTypeDef *hcan1;

	SPI_HandleTypeDef *hspi6;

	TIM_HandleTypeDef *htim1;
	TIM_HandleTypeDef *htim2;
	TIM_HandleTypeDef *htim3;
	TIM_HandleTypeDef *htim4;
	TIM_HandleTypeDef *htim5;

	UART_HandleTypeDef *huart3;
} stm32f767z_t;

void stm32f767z_init(stm32f767z_t * dev);
typedef struct
{
    HAL_StatusTypeDef status;
    uint16_t count;
} stm32f767z_adc_read_result_t;

stm32f767z_adc_read_result_t stm32f767z_adc_read_checked(ADC_HandleTypeDef *hadc, uint32_t timeout_ms);
uint16_t stm32f767z_adc_read(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef stm32f767z_adc_switch_channel(ADC_HandleTypeDef *hadc, uint32_t channel);

#endif /* INC_EXT_DRIVERS_STM32F407G_H_ */
