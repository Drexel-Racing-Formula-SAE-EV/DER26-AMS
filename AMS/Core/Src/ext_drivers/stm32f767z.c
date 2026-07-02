/*
 * stm32f407g.c
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 */

#include "ext_drivers/stm32f767z.h"

#define ADC_POLL_TIMEOUT_MS 5u

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern CAN_HandleTypeDef hcan1;

extern SPI_HandleTypeDef hspi6;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

extern UART_HandleTypeDef huart3;

void stm32f767z_init(stm32f767z_t * dev)
{
    if(dev == NULL)
    {
        return;
    }

	dev->hadc1 = &hadc1;
	dev->hadc2 = &hadc2;

	dev->hcan1 = &hcan1;

	dev->hspi6 = &hspi6;

	dev->htim1 = &htim1;
	dev->htim2 = &htim2;
	dev->htim3 = &htim3;
	dev->htim4 = &htim4;
	dev->htim5 = &htim5;

	dev->huart3 = &huart3;
}

uint16_t stm32f767z_adc_read(ADC_HandleTypeDef *hadc)
{
	uint16_t count = 0u;

    if(hadc == NULL)
    {
        return 0u;
    }

	if(HAL_ADC_Start(hadc) != HAL_OK)
    {
        return 0u;
    }

	if(HAL_ADC_PollForConversion(hadc, ADC_POLL_TIMEOUT_MS) == HAL_OK)
    {
        count = (uint16_t)HAL_ADC_GetValue(hadc);
    }

	(void)HAL_ADC_Stop(hadc);
	return count;
}

HAL_StatusTypeDef stm32f767z_adc_switch_channel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    if(hadc == NULL)
    {
        return HAL_ERROR;
    }

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	return HAL_ADC_ConfigChannel(hadc, &sConfig);
}
