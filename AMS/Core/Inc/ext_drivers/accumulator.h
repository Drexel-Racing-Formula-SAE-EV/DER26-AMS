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
#include "ams_build_profile.h"
#include "stm32f7xx_hal.h"
#include "ext_drivers/adbms2950.h"
#include "ext_drivers/adbms6830_functions.h"

/* SMB Macros */
#define NSEGS 1
#define NCELLS AMS_ADBMS_CELL_COUNT
#define NTEMPS 24
#define ACCUMULATOR_CELL_MASK ((uint16_t)((NCELLS == 16u) ? 0xFFFFu : ((1u << NCELLS) - 1u)))

#define ACCUMULATOR_STATUS_OK           0
#define ACCUMULATOR_STATUS_ERROR       -1
#define ACCUMULATOR_STATUS_UNAVAILABLE -2
#define ACCUMULATOR_STATUS_DISABLED    -3
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
#define ACCUMULATOR_TEMP_OPEN_LOW_MV          100u
#define ACCUMULATOR_TEMP_SHORT_HIGH_MV        4900u
#define ACCUMULATOR_TEMP_IMPLAUSIBLE_JUMP_DECI_C 250
#define ACCUMULATOR_TEMP_RATE_WARN_DECI_C_PER_S 50
#define ACCUMULATOR_TEMP_FILTER_ALPHA_NUM     1
#define ACCUMULATOR_TEMP_FILTER_ALPHA_DEN     8

#define ACCUMULATOR_CELL_IMPLAUSIBLE_JUMP_MV  250u
#define ACCUMULATOR_CELL_STUCK_SAME_COUNT     120u


/* APM Macros */
/* Keep the ADBMS2950/APM off the normal AMS loop until board bring-up is
 * complete. Set to 1 locally to initialize the APM path for CLI-only probing.
 */
#ifndef AMS_ENABLE_APM_2950_DEBUG
#define AMS_ENABLE_APM_2950_DEBUG 0
#endif

#if AMS_EVAL_ADBMS6830_BMSW && AMS_ENABLE_APM_2950_DEBUG
#error "ADBMS2950/APM debug must remain disabled in the 6830 eval-board profile"
#endif

#define NAPMS 1
#define HVEN1 GPO1
#define HVEN2 GPO2

#define NSMBS AMS_ADBMS_MONITOR_COUNT

#if (NSMBS < 1u) || (NSMBS > ADBMS6830_MAX_TRACKED_ICS)
#error "NSMBS must fit the ADBMS6830 tracked-IC capacity"
#endif

#if (NCELLS < 1u) || (NCELLS > CELL)
#error "NCELLS must fit the ADBMS6830 cell-channel capacity"
#endif

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
	int16_t temp_raw_code[NSMBS][NTEMPS];
	int16_t temp_filtered_deci_c[NSMBS][NTEMPS];
	bool temp_sensor_valid[NSMBS][NTEMPS];
	uint32_t temp_filter_valid_mask[NSMBS];
	uint32_t temp_last_update_ms[NSMBS][NTEMPS];
	uint8_t temp_consecutive_misses[NSMBS][NTEMPS];

	uint32_t updated_temp_mask[NSMBS];
	uint32_t usable_temp_mask[NSMBS];
	uint32_t stale_temp_mask[NSMBS];
	uint32_t invalid_temp_mask[NSMBS];
	uint32_t temp_open_mask[NSMBS];
	uint32_t temp_short_mask[NSMBS];
	uint32_t temp_jump_mask[NSMBS];
	uint32_t temp_rate_rise_mask[NSMBS];

	uint16_t updated_temp_count;
	uint16_t usable_temp_count;
	uint16_t stale_temp_count;
	uint16_t invalid_temp_count;
	uint16_t temp_open_count;
	uint16_t temp_short_count;
	uint16_t temp_jump_count;
	uint16_t temp_rate_rise_count;
	int16_t max_temp_deci_c;
	int16_t min_temp_deci_c;
	int16_t filtered_max_temp_deci_c;
	int16_t filtered_min_temp_deci_c;
	int16_t filtered_avg_temp_deci_c;
	int16_t temp_max_rate_deci_c_per_s;
	uint8_t max_temp_seg;
	uint8_t max_temp_sensor;
	uint8_t min_temp_seg;
	uint8_t min_temp_sensor;
	uint8_t temp_max_rate_seg;
	uint8_t temp_max_rate_sensor;
	bool temp_full_updated;
	bool temp_full_usable;
	bool temp_startup_scan_complete;

	uint16_t cell_voltage_mv[NSMBS][NCELLS];
	bool cell_voltage_valid[NSMBS][NCELLS];
	uint32_t cell_voltage_last_update_ms[NSMBS][NCELLS];
	uint8_t cell_voltage_consecutive_misses[NSMBS][NCELLS];
	uint8_t cell_voltage_same_count[NSMBS][NCELLS];
	uint32_t hil_cell_last_update_ms[NSMBS][NCELLS];
	uint32_t hil_temp_last_update_ms[NSMBS][NTEMPS];
	uint16_t hil_cell_seen_mask[NSMBS];
	uint32_t hil_temp_seen_mask[NSMBS];

	uint16_t updated_voltage_mask[NSMBS];
	uint16_t usable_voltage_mask[NSMBS];
	uint16_t pec_fail_voltage_mask[NSMBS];
	uint16_t stale_voltage_mask[NSMBS];
	uint16_t voltage_jump_mask[NSMBS];
	uint16_t voltage_stuck_mask[NSMBS];

	uint16_t updated_voltage_count;
	uint16_t usable_voltage_count;
	uint16_t stale_voltage_count;
	uint16_t pec_fail_cell_count;
	uint16_t voltage_jump_cell_count;
	uint16_t voltage_stuck_cell_count;
	uint16_t voltage_max_delta_mv;
	uint16_t max_voltage_mv;
	uint16_t min_voltage_mv;
	uint8_t max_voltage_seg;
	uint8_t max_voltage_cell;
	uint8_t min_voltage_seg;
	uint8_t min_voltage_cell;
	uint8_t voltage_max_delta_seg;
	uint8_t voltage_max_delta_cell;
	bool voltage_full_updated;
	bool voltage_full_usable;
	bool voltage_startup_scan_complete;

	/* The ADBMS wake/conversion delays use this timer as their microsecond
	 * timebase.  Keep the startup result explicit: a populated timer handle is
	 * not proof that HAL_TIM_Base_Start() succeeded. */
	bool delay_timer_ready;
	HAL_StatusTypeDef delay_timer_status;
	bool smb_ready;
	HAL_StatusTypeDef smb_init_status;

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
int accumulator_hil_ingest_cell_triplet(accumulator_t *dev,
                                        uint8_t seg,
                                        uint8_t first_cell,
                                        const uint16_t cell_mv[3],
                                        uint32_t now_ms);
int accumulator_hil_ingest_temp_triplet(accumulator_t *dev,
                                        uint8_t seg,
                                        uint8_t first_sensor,
                                        const int16_t temp_deci_c[3],
                                        uint32_t now_ms);
void accumulator_hil_refresh_update_masks(accumulator_t *dev,
                                          uint32_t now_ms,
                                          uint32_t timeout_ms);

#endif /* INC_EXT_DRIVERS_ACCUMULATOR_H_ */
