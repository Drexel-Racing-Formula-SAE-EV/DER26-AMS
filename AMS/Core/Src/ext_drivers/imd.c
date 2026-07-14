/*
*   imd.c
*   Created: 2/5/2024
*   Author: Brendan Hoag
*   Modified by: Mahad Faisal (major firmware updates, 2026)
*   Purpose: Device driver for IR151-3204 ground fault monitoring system
*   datasheet: https://www.benderinc.com/products/ground-fault-monitoring-ungrounded/isometer-ir155-03-04-series/
*/
#include "ext_drivers/imd.h"

#include <math.h>

static int imd_fail_closed(imd_t *dev)
{
    if(dev != NULL)
    {
        dev->ret = 1;
        dev->OK_HS = false;
        dev->status = IMD_UNKNOWN;
        dev->duty = 0.0f;
        dev->freq = 0.0f;
    }
    return 1;
}

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
	/* Fail closed until both the digital status and PWM/status channel have
	 * been read successfully.  Dummy "healthy" values can make an unconnected
	 * or uninitialized IMD look valid to the safety supervisor. */
	dev->ret = 1;
	dev->duty = 0.0f;
	dev->freq = 0.0f;
	dev->OK_HS = false;
	dev->status = IMD_UNKNOWN;
    dev->capture_started = false;
    dev->init_status = HAL_ERROR;
    if(htim != NULL)
    {
	    dev->init_status = HAL_TIM_Base_Start(htim);
        if(dev->init_status == HAL_OK)
        {
	        dev->init_status = HAL_TIM_IC_Start_IT(htim, total_channel);
        }
        if(dev->init_status == HAL_OK)
        {
	        dev->init_status = HAL_TIM_IC_Start(htim, high_channel);
        }
        dev->capture_started = (dev->init_status == HAL_OK);
    }
}

int imd_read(imd_t *dev)
{
    if((dev == NULL) || !dev->capture_started || (dev->clock_freq == 0u) ||
       (dev->htim == NULL) || (dev->status_port == NULL))
    {
        return imd_fail_closed(dev);
    }

	dev->OK_HS = HAL_GPIO_ReadPin(dev->status_port, dev->status_pin);
	dev->total_count = HAL_TIM_ReadCapturedValue(dev->htim, dev->total_channel);
	if (dev->total_count != 0)
	{
		dev->high_count = HAL_TIM_ReadCapturedValue(dev->htim, dev->high_channel);
		if(dev->high_count > dev->total_count)
        {
            return imd_fail_closed(dev);
        }

		/* Convert before multiplying so a valid 32-bit capture cannot overflow
		 * in integer arithmetic. */
		dev->duty = ((float)dev->high_count * 100.0f) / (float)dev->total_count;
		dev->freq = (float)dev->clock_freq / (float)dev->total_count;
		if(!isfinite(dev->duty) || !isfinite(dev->freq) ||
           (dev->duty < 0.0f) || (dev->duty > 100.0f))
        {
            return imd_fail_closed(dev);
        }
		/* Preserve the existing frequency-to-status interpretation, but reject
		 * values outside the defined status range instead of publishing an
		 * arbitrary enum value.  The mapping still needs hardware validation. */
		int status_code = (int)(0.5f + (dev->freq / 10.0f));
		if((status_code >= (int)IMD_SHORT_TO_CHASSIS_GROUND) &&
		   (status_code <= (int)IMD_GROUND_FAULT))
		{
			dev->status = (imd_status_t)status_code;
			dev->ret = 0;
			return 0;
		}

		dev->status = IMD_UNKNOWN;
		return imd_fail_closed(dev);
	}
	return imd_fail_closed(dev);
}
