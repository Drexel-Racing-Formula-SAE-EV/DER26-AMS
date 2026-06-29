/*
 * fans.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Cassius Garcia
 */
#include "ext_drivers/fans.h"
#include <math.h>

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
