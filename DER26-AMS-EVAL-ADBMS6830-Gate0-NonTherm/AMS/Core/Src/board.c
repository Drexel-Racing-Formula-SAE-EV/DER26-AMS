/*
 * board.c
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#include "board.h"

#include "main.h"
#include "ext_drivers/charger.h"

#define FAN_MAX 3360
/* Rechecked net path: C_SNS_L=PC0/ADC2_IN10/50A, C_SNS_H=PA3/ADC1_IN3/800A. */
#define CUR_SEN_CH_L ADC_CHANNEL_10
#define CUR_SEN_CH_H ADC_CHANNEL_3

#if AMS_ENABLE_IMD
static uint32_t board_apb1_timer_clock_hz(void)
{
	RCC_ClkInitTypeDef clocks = {0};
	uint32_t flash_latency = 0u;
	uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();

	HAL_RCC_GetClockConfig(&clocks, &flash_latency);
	if(clocks.APB1CLKDivider != RCC_HCLK_DIV1)
	{
		timer_clock *= 2u;
	}
	return timer_clock;
}
#endif

void board_init(board_t *board)
{
    if(board == NULL)
    {
        return;
    }

	stm32f767z_init(&board->stm32f767z);

	fan_init(&board->fans[0], TIM3, board->stm32f767z.htim3, FAN_MAX, &TIM3->CCR2, 2);
	fan_init(&board->fans[1], TIM3, board->stm32f767z.htim3, FAN_MAX, &TIM3->CCR4, 4);
	fan_init(&board->fans[2], TIM4, board->stm32f767z.htim4, FAN_MAX, &TIM4->CCR3, 3);
	fan_init(&board->fans[3], TIM4, board->stm32f767z.htim4, FAN_MAX, &TIM4->CCR4, 4);
	fan_init(&board->fans[4], TIM5, board->stm32f767z.htim5, FAN_MAX, &TIM5->CCR1, 1);
	fan_init(&board->fans[5], TIM5, board->stm32f767z.htim5, FAN_MAX, &TIM5->CCR2, 2);


	canbus_device_init(&board->canbus, board->stm32f767z.hcan1);
	charger_init(&board->charger, &board->canbus);
	cli_device_init(&board->cli, board->stm32f767z.huart3);
	current_sensor_init(&board->current_sensor,
						board->stm32f767z.hadc2,
						board->stm32f767z.hadc1,
						CUR_SEN_CH_L,
						CUR_SEN_CH_H
					   );

#if AMS_ENABLE_IMD
	/* TIM2 is configured in PWM-input/reset mode: CH1 captures the period and
	 * CH2 captures high time.  TIM5 belongs to fan outputs and must not be used
	 * for IMD capture. */
	imd_init(&board->imd,
	         board_apb1_timer_clock_hz(),
	         board->stm32f767z.htim2,
	         TIM2,
	         TIM_CHANNEL_2,
	         TIM_CHANNEL_1,
	         IMD_STAT_GPIO_Port,
	         IMD_STAT_Pin);
#endif

	return;
}
