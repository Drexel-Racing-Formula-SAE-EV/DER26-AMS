/*
 * current_sensor.c
 *
 *  Created on: Mar 3th, 2024
 *      Author: Justin Nguyen
 */

#include <ext_drivers/current_sensor.h>
#include <math.h>

#define VREF 3.3
#define UC      5.0
#define U0      2.5
#define SL   2.5
#define SH   40.0

void current_sensor_init(current_sensor_t *dev, ADC_HandleTypeDef *hadc_low,ADC_HandleTypeDef *hadc_high, uint32_t channel_low,uint32_t channel_high )
{
    dev->hadc_low = hadc_low;
    dev->hadc_high = hadc_high;
    dev->channel_low= channel_low;
    dev->channel_high= channel_high;
    dev->voltage_high = 0;
    dev->voltage_low = 0;
    dev->current_low = 0;
    dev->current_high = 0;
    dev->current = 0;
    dev->count_high =0;
    dev->count_low = 0;
}

float current_sensor_convert(current_sensor_t *dev)
{
    dev->voltage_low  = (float)dev->count_low * VREF / 4095.0;
    dev->voltage_high = (float)dev->count_high * VREF / 4095.0;

    dev->current_low  = ((5 / UC) * dev->voltage_low - U0) * 1000.0 / SL;
    dev->current_high = ((5 / UC) * dev->voltage_high - U0) * 1000.0 / SH;

    if(dev->current_low > 50.0){
        dev->current = dev->current_high;
        return dev->current_high;
    }
    else{
        dev->current = dev->current_low;
        return dev->current_low;
    }
}
