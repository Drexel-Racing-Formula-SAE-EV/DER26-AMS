/*
 * fans.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Cassius Garcia
 */
#include "ext_drivers/fans.h"

int fan_init(fan_t *fan, TIM_TypeDef *timer, TIM_HandleTypeDef *htim, uint64_t max_timer_val, volatile uint32_t *CCR, int channel){
    fan->timer = timer;
    fan->htim = htim;
    fan->channel = channel;
    fan->max_timer_val = max_timer_val;
    fan->CCR = CCR;

    uint32_t ch = TIM_CHANNEL_1;

    HAL_TIM_PWM_Start(htim, (channel - 1) * 4);
    set_fan_percent(fan, 0.0);

    return 0;
}

int set_fan_percent(fan_t *fan, float percent){
    if(percent > 100.0 || percent < 0.0) return 1;
    fan->duty_cycle = percent;
    *(fan->CCR) = (volatile uint32_t)fan->max_timer_val * (volatile uint32_t)percent / 100;
    return 0;
}
