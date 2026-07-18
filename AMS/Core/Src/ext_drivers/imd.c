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

static void imd_memory_barrier(void)
{
#if AMS_HOST_TEST
    __asm__ volatile ("" ::: "memory");
#else
    __DMB();
#endif
}

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

void imd_init(imd_t *dev, uint32_t clock_freq, TIM_HandleTypeDef *htim, TIM_TypeDef *tim, uint32_t high_channel, uint32_t total_channel, GPIO_TypeDef *status_port, uint16_t status_pin)
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
    dev->capture_seen = false;
    dev->capture_sequence = 0u;
    dev->captured_high_count = 0u;
    dev->captured_total_count = 0u;
    dev->capture_count = 0u;
    dev->last_capture_tick = 0u;
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

void imd_capture_event(imd_t *dev, uint32_t now)
{
    uint32_t sequence;

    if((dev == NULL) || !dev->capture_started)
    {
        return;
    }

    /* TIM2 PWM-input/reset mode updates the period and high-time CCRs at the
     * same input edge. Copy both in the capture ISR and publish them as one
     * tuple; reading the live CCRs later can otherwise combine two cycles. */
    sequence = dev->capture_sequence;
    if((sequence & 1u) != 0u)
    {
        sequence++;
    }
    dev->capture_sequence = sequence + 1u;
    imd_memory_barrier();
    dev->captured_total_count =
        HAL_TIM_ReadCapturedValue(dev->htim, dev->total_channel);
    dev->captured_high_count =
        HAL_TIM_ReadCapturedValue(dev->htim, dev->high_channel);
    dev->last_capture_tick = now;
    if(dev->capture_count != UINT32_MAX)
    {
        dev->capture_count++;
    }
    dev->capture_seen = true;
    imd_memory_barrier();
    dev->capture_sequence = sequence + 2u;
}

static bool imd_capture_snapshot(const imd_t *dev,
                                 uint32_t *high_count,
                                 uint32_t *total_count,
                                 uint32_t *capture_tick)
{
    if((dev == NULL) || (high_count == NULL) ||
       (total_count == NULL) || (capture_tick == NULL))
    {
        return false;
    }

    for(uint8_t attempt = 0u; attempt < 3u; attempt++)
    {
        uint32_t before = dev->capture_sequence;
        bool seen;

        if((before & 1u) != 0u)
        {
            continue;
        }
        imd_memory_barrier();
        *total_count = dev->captured_total_count;
        *high_count = dev->captured_high_count;
        *capture_tick = dev->last_capture_tick;
        seen = dev->capture_seen;
        imd_memory_barrier();
        if(seen && (before == dev->capture_sequence) &&
           ((dev->capture_sequence & 1u) == 0u))
        {
            return true;
        }
    }
    return false;
}

int imd_read_at(imd_t *dev, uint32_t now)
{
    uint32_t captured_high;
    uint32_t captured_total;
    uint32_t captured_tick;

    if((dev == NULL) || !dev->capture_started || (dev->clock_freq == 0u) ||
       (dev->htim == NULL) || (dev->status_port == NULL))
    {
        return imd_fail_closed(dev);
    }

    /* Capture registers retain their old value indefinitely.  Require a real
     * capture interrupt and reject it once its age exceeds two slow (10 Hz)
     * PWM periods.  Unsigned subtraction remains correct across tick wrap. */
    if(!imd_capture_snapshot(dev, &captured_high, &captured_total, &captured_tick) ||
       ((uint32_t)(now - captured_tick) > AMS_IMD_CAPTURE_TIMEOUT_MS))
    {
        return imd_fail_closed(dev);
    }

	dev->OK_HS = HAL_GPIO_ReadPin(dev->status_port, dev->status_pin);
	dev->total_count = captured_total;
	if (dev->total_count != 0)
	{
		dev->high_count = captured_high;
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
		float rounded_status = 0.5f + (dev->freq / 10.0f);
		if(!isfinite(rounded_status) ||
		   (rounded_status < 0.0f) ||
		   (rounded_status >= ((float)IMD_GROUND_FAULT + 1.0f)))
		{
			return imd_fail_closed(dev);
		}

		/* The explicit range check above makes this float-to-int conversion
		 * defined even if the timer clock/capture registers are corrupted. */
		int status_code = (int)rounded_status;
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

int imd_read(imd_t *dev)
{
    return imd_read_at(dev, HAL_GetTick());
}
