/*
 * stm32f407g.h
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef INC_EXT_DRIVERS_STM32F407G_H_
#define INC_EXT_DRIVERS_STM32F407G_H_

#include <stm32f7xx_hal.h>
#include "cmsis_os.h"

/* C_SENSE_H_MCU and C_SENSE_L_MCU each see the 100 kOhm / 150 kOhm
 * divider's approximately 60 kOhm Thevenin resistance.  Three ADC clock
 * cycles cannot settle the STM32F767 sample-and-hold capacitor through that
 * source.  Use the longest supported acquisition window; at the configured
 * 18 MHz ADC clock it is about 26.7 us, negligible beside the 20 ms current
 * task period.  Physical accuracy, sign and gain still require bench
 * validation. */
#ifndef AMS_CURRENT_ADC_SAMPLING_TIME
#define AMS_CURRENT_ADC_SAMPLING_TIME ADC_SAMPLETIME_480CYCLES
#endif

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
