/*
 * accumulator.h
 *
 *  Created on: Feb 1, 2024
 *      Author: cole
 */

#ifndef INC_EXT_DRIVERS_ACCUMULATOR_H_
#define INC_EXT_DRIVERS_ACCUMULATOR_H_

#include <stdbool.h>
#include "stm32f7xx_hal.h"
#include "ext_drivers/adbms2950.h"
#include "ext_drivers/adbms6830_functions.h"

/* SMB Macros */
#define NSEGS 1
#define NCELLS 15
#define NTEMPS 24
#define MUX_ADDR7_00 0x4C
#define MUX_ADDR7_01 0x4D
#define MUX_ADDR7_02 0x4E
#define VNTC 5.0
#define BALANCE_THRESH  0.010f   // 10mV


/* APM Macros */
#define NAPMS 1
#define HVEN1 GPO1
#define HVEN2 GPO2

#define NSMBS 5

typedef struct
{
	float total_volt;
	float max_temp;
	float avg_temp;
	float max_volt;
	float min_volt;
	uint16_t valid_voltage_count;
	uint16_t valid_temp_count;

	adbms2950_asic apm_ics[NAPMS];
	adbms2950_driver_t apm;

	adbms6830_asic smb_ics[NSMBS];
	adbms6830_driver_t smb;
} accumulator_t;

void accumulator_init(accumulator_t *dev,
				      SPI_HandleTypeDef *hspi,
					  GPIO_TypeDef *cs_port_a,
					  GPIO_TypeDef *cs_port_b,
					  uint16_t cs_pin_a,
					  uint16_t cs_pin_b,
					  TIM_HandleTypeDef* htim
					  );
int accumulator_read_volt(accumulator_t *dev);
int accumulator_read_temp(accumulator_t *dev);
int accumulator_set_temp_ch(accumulator_t *dev, uint8_t channel);
int accumulator_stat_temp(accumulator_t *dev);
int accumulator_set_mux_ch(accumulator_t *dev, uint8_t channel, uint8_t addr7);
float NXFT15XV103FEAB050_convert(float ratio);
void accumulator_update_voltage_stats(accumulator_t *dev);
void accumulator_update_temp_stats(accumulator_t *dev);
int accumulator_set_balance(accumulator_t *dev);
int accumulator_clear_balance(accumulator_t *dev);

#endif /* INC_EXT_DRIVERS_ACCUMULATOR_H_ */
