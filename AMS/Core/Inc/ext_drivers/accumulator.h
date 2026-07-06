/*
 * accumulator.h
 *
 *  Created on: Feb 1, 2024
 *      Author: cole
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
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
#define BALANCE_START_MV             4100u
#define BALANCE_ON_DELTA_MV            20u
#define BALANCE_MAX_CELLS_PER_SEG       4u
/* Treat SMB balance resistors as 20 ohm / 0.75 W until assembly confirms otherwise. */
#define BALANCE_PWM_DUTY       PWM_33_0_PCT
#define ACCUMULATOR_CELL_STALE_TIMEOUT_MS      2500u
#define ACCUMULATOR_CELL_MAX_CONSEC_MISSES     2u
#define ACCUMULATOR_CELL_VALID_MIN_MV          500u
#define ACCUMULATOR_CELL_VALID_MAX_MV          5000u
#define ACCUMULATOR_TEMP_STALE_TIMEOUT_MS      12000u
#define ACCUMULATOR_TEMP_MAX_CONSEC_MISSES     10u
#define ACCUMULATOR_TEMP_VALID_MIN_DECI_C      (-400)
#define ACCUMULATOR_TEMP_VALID_MAX_DECI_C      1500


/* APM Macros */
/* Keep the ADBMS2950/APM off the normal AMS loop until board bring-up is
 * complete. Set to 1 locally to initialize the APM path for CLI-only probing.
 */
#ifndef AMS_ENABLE_APM_2950_DEBUG
#define AMS_ENABLE_APM_2950_DEBUG 0
#endif

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

	int16_t temp_deci_c[NSMBS][NTEMPS];
	bool temp_sensor_valid[NSMBS][NTEMPS];
	uint32_t temp_last_update_ms[NSMBS][NTEMPS];
	uint8_t temp_consecutive_misses[NSMBS][NTEMPS];

	uint32_t updated_temp_mask[NSMBS];
	uint32_t usable_temp_mask[NSMBS];
	uint32_t stale_temp_mask[NSMBS];
	uint32_t invalid_temp_mask[NSMBS];

	uint16_t updated_temp_count;
	uint16_t usable_temp_count;
	uint16_t stale_temp_count;
	uint16_t invalid_temp_count;
	int16_t max_temp_deci_c;
	int16_t min_temp_deci_c;
	uint8_t max_temp_seg;
	uint8_t max_temp_sensor;
	uint8_t min_temp_seg;
	uint8_t min_temp_sensor;
	bool temp_full_updated;
	bool temp_full_usable;
	bool temp_startup_scan_complete;

	uint16_t cell_voltage_mv[NSMBS][NCELLS];
	bool cell_voltage_valid[NSMBS][NCELLS];
	uint32_t cell_voltage_last_update_ms[NSMBS][NCELLS];
	uint8_t cell_voltage_consecutive_misses[NSMBS][NCELLS];

	uint16_t updated_voltage_mask[NSMBS];
	uint16_t usable_voltage_mask[NSMBS];
	uint16_t pec_fail_voltage_mask[NSMBS];
	uint16_t stale_voltage_mask[NSMBS];

	uint16_t updated_voltage_count;
	uint16_t usable_voltage_count;
	uint16_t stale_voltage_count;
	uint16_t pec_fail_cell_count;
	uint16_t max_voltage_mv;
	uint16_t min_voltage_mv;
	uint8_t max_voltage_seg;
	uint8_t max_voltage_cell;
	uint8_t min_voltage_seg;
	uint8_t min_voltage_cell;
	bool voltage_full_updated;
	bool voltage_full_usable;
	bool voltage_startup_scan_complete;

	adbms2950_asic apm_ics[NAPMS];
	adbms2950_driver_t apm;

	adbms6830_asic smb_ics[NSMBS];
	adbms6830_driver_t smb;
} accumulator_t;

typedef struct
{
	bool enabled;
	bool initialized;
	bool hold_missing;
	uint32_t missing_mask[NSMBS];
	uint32_t invalid_mask[NSMBS];
	float min_temp_c;
	float max_temp_c;
	uint8_t min_seg;
	uint8_t min_sensor;
	uint8_t max_seg;
	uint8_t max_sensor;
	uint32_t apply_count;
} accumulator_temp_fake_status_t;

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
void accumulator_update_voltage_stats_at(accumulator_t *dev, uint32_t now_ms);
bool accumulator_cell_voltage_usable(const accumulator_t *dev, uint8_t seg, uint8_t cell);
uint16_t accumulator_cell_voltage_mv(const accumulator_t *dev, uint8_t seg, uint8_t cell);
void accumulator_update_temp_stats(accumulator_t *dev);
void accumulator_update_temp_stats_at(accumulator_t *dev, uint32_t now_ms);
bool accumulator_temp_sensor_usable(const accumulator_t *dev, uint8_t seg, uint8_t sensor);
int16_t accumulator_temp_deci_c(const accumulator_t *dev, uint8_t seg, uint8_t sensor);
uint8_t accumulator_configured_smb_count(const accumulator_t *dev);
int accumulator_set_balance(accumulator_t *dev);
int accumulator_clear_balance(accumulator_t *dev);

bool accumulator_temp_fake_enabled(void);
void accumulator_temp_fake_reset(accumulator_t *dev);
int accumulator_temp_fake_set_all(accumulator_t *dev, float temp_c);
int accumulator_temp_fake_set_sensor(accumulator_t *dev, uint8_t seg, uint8_t sensor, float temp_c);
void accumulator_temp_fake_set_missing_mask(accumulator_t *dev, uint8_t seg, uint32_t mask);
void accumulator_temp_fake_set_invalid_mask(accumulator_t *dev, uint8_t seg, uint32_t mask);
void accumulator_temp_fake_set_hold_missing(accumulator_t *dev, bool hold_missing);
accumulator_temp_fake_status_t accumulator_temp_fake_get_status(const accumulator_t *dev);

#endif /* INC_EXT_DRIVERS_ACCUMULATOR_H_ */
