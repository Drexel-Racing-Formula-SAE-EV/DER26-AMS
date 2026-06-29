/*
*   imd.c
*   Created: 2/5/2024
*   Author: Brendan Hoag
*   Purpose: Device driver for IR151-3204 ground fault monitoring system
*   datasheet: https://www.benderinc.com/products/ground-fault-monitoring-ungrounded/isometer-ir155-03-04-series/
*/
#include "ext_drivers/imd.h"

// todo: better dummy init values
void imd_init(imd_t *dev, uint32_t clock_freq, TIM_HandleTypeDef *htim, TIM_TypeDef *tim, HAL_TIM_ActiveChannel high_channel, HAL_TIM_ActiveChannel total_channel, GPIO_TypeDef *status_port, uint16_t status_pin)
{
    if(dev == NULL)
    {
        return;
    }

	dev->clock_freq = clock_freq;
	dev->htim = htim;
	dev->tim = tim;
	dev->high_channel = high_channel;
	dev->total_channel = total_channel;
	dev->status_port = status_port;
	dev->status_pin = status_pin;
	dev->duty = 0;
	dev->freq = 0;
	dev->high_count = 0;
	dev->total_count = 0;
	dev->ret = 0;
	dev->duty = 0.5;
	dev->freq = 10.0;
	dev->OK_HS = true;
	dev->status = IMD_NORMAL;
    if(htim != NULL)
    {
	    (void)HAL_TIM_Base_Start(htim);
	    (void)HAL_TIM_IC_Start_IT(htim, total_channel);
	    (void)HAL_TIM_IC_Start(htim, high_channel);
    }
}

int imd_read(imd_t *dev)
{
    if((dev == NULL) || (dev->htim == NULL) || (dev->status_port == NULL))
    {
        return 1;
    }

	dev->OK_HS = HAL_GPIO_ReadPin(dev->status_port, dev->status_pin);
	dev->total_count = HAL_TIM_ReadCapturedValue(dev->htim, dev->total_channel);
	if (dev->total_count != 0)
	{
		dev->high_count = HAL_TIM_ReadCapturedValue(dev->htim, dev->high_channel);
		dev->duty = (float)(dev->high_count * 100) / (float)dev->total_count;
		dev->freq = (float)dev->clock_freq / (float)dev->total_count;
		dev->status = 0.5 + dev->freq / 10.0; // TODO: probably need to round to nearest 10
		return 0;
	}
	return 1;
}
