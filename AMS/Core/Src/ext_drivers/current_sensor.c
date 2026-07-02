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
#define SENSOR_DIVIDER_TOP_OHM    100000.0f
#define SENSOR_DIVIDER_BOTTOM_OHM 150000.0f
#define SENSOR_DIVIDER_GAIN       (SENSOR_DIVIDER_BOTTOM_OHM / (SENSOR_DIVIDER_TOP_OHM + SENSOR_DIVIDER_BOTTOM_OHM))

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
    dev->sensor_voltage_high = 0.0f;
    dev->sensor_voltage_low = 0.0f;
    dev->current_low = 0.0f;
    dev->current_high = 0.0f;
    dev->current = 0.0f;
    dev->count_high = 0u;
    dev->count_low = 0u;
    dev->last_read_ok = false;
}

float current_sensor_convert(current_sensor_t *dev)
{
    if(dev == NULL)
    {
        return 0.0f;
    }

    dev->voltage_low  = (float)dev->count_low  * VREF / 4095.0f;
    dev->voltage_high = (float)dev->count_high * VREF / 4095.0f;

    /*
     * AMS Rev3.1 conditions both DHAB current sensor outputs with a
     * 100k/150k divider before the STM32 ADC, scaling the sensor's 0..5 V
     * output range to roughly 0..3 V at the MCU. Undo that divider before
     * applying the sensor offset/sensitivity math.
     */
    dev->sensor_voltage_low = dev->voltage_low / SENSOR_DIVIDER_GAIN;
    dev->sensor_voltage_high = dev->voltage_high / SENSOR_DIVIDER_GAIN;

    dev->current_low  = (((5.0f / UC) * dev->sensor_voltage_low)  - U0) * 1000.0f / SL;
    dev->current_high = (((5.0f / UC) * dev->sensor_voltage_high) - U0) * 1000.0f / SH;

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

bool current_sensor_read_adc(current_sensor_t *dev)
{
    if(dev == NULL)
    {
        return false;
    }

    bool high_ok = false;
    bool low_ok = false;
    uint16_t count_high = dev->count_high;
    uint16_t count_low = dev->count_low;

    if((dev->hadc_high != NULL) &&
       (stm32f767z_adc_switch_channel(dev->hadc_high, dev->channel_high) == HAL_OK))
    {
        count_high = stm32f767z_adc_read(dev->hadc_high);
        high_ok = true;
    }

    if((dev->hadc_low != NULL) &&
       (stm32f767z_adc_switch_channel(dev->hadc_low, dev->channel_low) == HAL_OK))
    {
        count_low = stm32f767z_adc_read(dev->hadc_low);
        low_ok = true;
    }

    dev->last_read_ok = high_ok && low_ok;
    if(dev->last_read_ok)
    {
        dev->count_high = count_high;
        dev->count_low = count_low;
    }

    return dev->last_read_ok;
}

float current_sensor_current_read(current_sensor_t *dev)
{
    if(dev == NULL)
    {
        return 0.0f;
    }

    if(current_sensor_read_adc(dev))
    {
        return current_sensor_convert(dev);
    }

    return dev->current;
}
