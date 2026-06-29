/*
 * current_sensor.c
 *
 *  Created on: Mar 3th, 2024
 *      Author: Justin Nguyen
 */

#include <ext_drivers/current_sensor.h>
#include <ext_drivers/stm32f767z.h>
#include <math.h>

#define VREF 3.3f
#define UC   5.0f
#define U0   2.5f
#define SL   2.5f
#define SH   40.0f
#define LOW_RANGE_LIMIT_A 50.0f

void current_sensor_init(current_sensor_t *dev,
                         ADC_HandleTypeDef *hadc_low,
                         ADC_HandleTypeDef *hadc_high,
                         uint32_t channel_low,
                         uint32_t channel_high)
{
    if(dev == NULL)
    {
        return;
    }

    dev->hadc_low = hadc_low;
    dev->hadc_high = hadc_high;
    dev->channel_low = channel_low;
    dev->channel_high = channel_high;
    dev->voltage_high = 0.0f;
    dev->voltage_low = 0.0f;
    dev->current_low = 0.0f;
    dev->current_high = 0.0f;
    dev->current = 0.0f;
    dev->count_high = 0u;
    dev->count_low = 0u;
}

float current_sensor_convert(current_sensor_t *dev)
{
    if(dev == NULL)
    {
        return 0.0f;
    }

    dev->voltage_low  = (float)dev->count_low  * VREF / 4095.0f;
    dev->voltage_high = (float)dev->count_high * VREF / 4095.0f;

    dev->current_low  = (((5.0f / UC) * dev->voltage_low)  - U0) * 1000.0f / SL;
    dev->current_high = (((5.0f / UC) * dev->voltage_high) - U0) * 1000.0f / SH;

    if(fabsf(dev->current_low) > LOW_RANGE_LIMIT_A)
    {
        dev->current = dev->current_high;
    }
    else
    {
        dev->current = dev->current_low;
    }

    return dev->current;
}

float current_sensor_current_read(current_sensor_t *dev)
{
    if(dev == NULL)
    {
        return 0.0f;
    }

    if((dev->hadc_high != NULL) &&
       (stm32f767z_adc_switch_channel(dev->hadc_high, dev->channel_high) == HAL_OK))
    {
        dev->count_high = stm32f767z_adc_read(dev->hadc_high);
    }

    if((dev->hadc_low != NULL) &&
       (stm32f767z_adc_switch_channel(dev->hadc_low, dev->channel_low) == HAL_OK))
    {
        dev->count_low = stm32f767z_adc_read(dev->hadc_low);
    }

    return current_sensor_convert(dev);
}
