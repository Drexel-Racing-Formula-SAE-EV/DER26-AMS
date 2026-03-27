/*
 * board.c
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 */

#include "board.h"

#include "main.h"

#define FAN_MAX 3360
#define CUR_SEN_CH_L 14
#define CUR_SEN_CH_H 1

void board_init(board_t *board)
{
	stm32f767z_init(&board->stm32f767z);

	fan_init(&board->fans[0], TIM3, board->stm32f767z.htim3, FAN_MAX, &TIM3->CCR2, 2);
	fan_init(&board->fans[1], TIM3, board->stm32f767z.htim3, FAN_MAX, &TIM3->CCR4, 4);
	fan_init(&board->fans[2], TIM4, board->stm32f767z.htim4, FAN_MAX, &TIM4->CCR3, 3);
	fan_init(&board->fans[3], TIM4, board->stm32f767z.htim4, FAN_MAX, &TIM4->CCR4, 4);
	fan_init(&board->fans[4], TIM5, board->stm32f767z.htim5, FAN_MAX, &TIM5->CCR1, 1);
	fan_init(&board->fans[5], TIM5, board->stm32f767z.htim5, FAN_MAX, &TIM5->CCR2, 2);


	canbus_device_init(&board->canbus, board->stm32f767z.hcan1);

	cli_device_init(&board->cli, board->stm32f767z.huart3);
	/*current_sensor_init(&board->current_sensor,
						&board->stm32f767z.hadc2,
						&board->stm32f767z.hadc1,
						CUR_SEN_CH_L,
						CUR_SEN_CH_H
					   );*/

	//imd_init(&board->imd, 84000000, &board->stm32f767z.htim5, TIM5, TIM_CHANNEL_2, TIM_CHANNEL_1, IMD_STATUS_MCU_GPIO_Port, IMD_STATUS_MCU_Pin);

	return;
}
