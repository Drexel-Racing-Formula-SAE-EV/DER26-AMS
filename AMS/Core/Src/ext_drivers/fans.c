/*
 * fans.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Cassius Garcia
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */
#include "ext_drivers/fans.h"
#include <math.h>


const char *fan_control_reason_str(uint8_t reason)
{
    switch((fan_control_reason_t)reason)
    {
    case FAN_CONTROL_REASON_OFF_COOL:       return "off_cool";
    case FAN_CONTROL_REASON_RAMP:           return "ramp";
    case FAN_CONTROL_REASON_MIN_HYSTERESIS: return "min_hysteresis";
    case FAN_CONTROL_REASON_CHARGE_WARM:    return "charge_warm";
    case FAN_CONTROL_REASON_MAX_TEMP:       return "max_temp";
    case FAN_CONTROL_REASON_TEMP_INVALID:   return "temp_invalid";
    case FAN_CONTROL_REASON_TEMP_FAULT:     return "temp_fault";
    case FAN_CONTROL_REASON_DRIVER_FAULT:   return "driver_fault";
    default:                                return "unknown";
    }
}

static uint32_t fan_channel_to_hal(int channel)
{
    switch(channel)
    {
    case 1: return TIM_CHANNEL_1;
    case 2: return TIM_CHANNEL_2;
    case 3: return TIM_CHANNEL_3;
    case 4: return TIM_CHANNEL_4;
    default: return 0xFFFFFFFFu;
    }
}

int fan_init(fan_t *fan, TIM_TypeDef *timer, TIM_HandleTypeDef *htim, uint64_t max_timer_val, volatile uint32_t *CCR, int channel)
{
    if((fan == NULL) || (htim == NULL) || (CCR == NULL))
    {
        return 1;
    }

    uint32_t hal_channel = fan_channel_to_hal(channel);
    if(hal_channel == 0xFFFFFFFFu)
    {
        return 1;
    }

    fan->timer = timer;
    fan->htim = htim;
    fan->channel = channel;
    fan->max_timer_val = max_timer_val;
    fan->CCR = CCR;
    fan->duty_cycle = 0.0f;

    if(HAL_TIM_PWM_Start(htim, hal_channel) != HAL_OK)
    {
        return 1;
    }

    return set_fan_percent(fan, 0.0f);
}

int set_fan_percent(fan_t *fan, float percent)
{
    if((fan == NULL) || (fan->CCR == NULL))
    {
        return 1;
    }

    if(!isfinite(percent))
    {
        percent = 0.0f;
    }
    else if(percent > 100.0f)
    {
        percent = 100.0f;
    }
    else if(percent < 0.0f)
    {
        percent = 0.0f;
    }

    fan->duty_cycle = percent;
    *(fan->CCR) = (uint32_t)(((float)fan->max_timer_val * percent) / 100.0f);
    return 0;
}
