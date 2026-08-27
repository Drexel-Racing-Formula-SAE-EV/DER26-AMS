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
#define ACCUMULATOR_TEMP_IMPLAUSIBLE_JUMP_DECI_C 250
#define ACCUMULATOR_TEMP_RATE_WARN_DECI_C_PER_S 50
#define ACCUMULATOR_TEMP_FILTER_ALPHA_NUM     1
#define ACCUMULATOR_TEMP_FILTER_ALPHA_DEN     8

#define ACCUMULATOR_CELL_IMPLAUSIBLE_JUMP_MV  250u
#define ACCUMULATOR_CELL_STUCK_SAME_COUNT     120u


/* Single-SMB CAN/logging bench topology:
 *   MCU SPI6 + CS_B/PE4 -> ADBMS6822 -> one ADBMS6830B SMB.
 * The standalone ADBMS2950 evaluation path is disabled in this release so a
 * missing APM cannot disturb the validated one-SMB transport test.  The shared
 * source still supports the APM/final-ring profile through explicit overrides. */
#ifndef AMS_APM_STANDALONE_EVAL_BENCH
#define AMS_APM_STANDALONE_EVAL_BENCH 0
#endif

#ifndef AMS_ENABLE_APM_2950
#define AMS_ENABLE_APM_2950 0
#endif

#ifndef AMS_HIL_REPLACE_ADBMS
#define AMS_HIL_REPLACE_ADBMS 0
#endif

/* GPO1/GPO2 drive the APM high-voltage divider enables.  Leave them forced
 * low for first hardware tests; current measurement does not require them. */
#ifndef AMS_APM_ENABLE_HV_DIVIDERS
#define AMS_APM_ENABLE_HV_DIVIDERS 0
#endif

#define ACCUMULATOR_APM_SAMPLE_STALE_TIMEOUT_MS 2500u

/* Keep one storage slot so the disabled APM object remains standard C. */
#define NAPMS 1
#define HVEN1 GPO1
#define HVEN2 GPO2

/* The single-SMB bench image remains the default. Host/SIL builds may
 * override NSMBS explicitly to exercise the production five-segment topology. */
#ifndef NSMBS
#if (AMS_BUILD_PROFILE == AMS_PROFILE_TESTDAY) || \
    ((AMS_BUILD_PROFILE == AMS_PROFILE_BENCH_VALIDATION) && !AMS_BENCH_VALIDATION_SINGLE_SMB)
#define NSMBS 5
#else
#define NSMBS 1
#endif
#endif
#ifndef ACCUMULATOR_PHYSICAL_CHAIN_COUNT
#define ACCUMULATOR_PHYSICAL_CHAIN_COUNT NSMBS
#endif
#ifndef ACCUMULATOR_SMB_STRING
#if (AMS_ENABLE_APM_2950 && !AMS_APM_STANDALONE_EVAL_BENCH) || \
    (AMS_BUILD_PROFILE == AMS_PROFILE_TESTDAY) || \
    ((AMS_BUILD_PROFILE == AMS_PROFILE_BENCH_VALIDATION) && !AMS_BENCH_VALIDATION_SINGLE_SMB)
/* Final ring: five SMBs are the leading String-A subset; the ADBMS2950 is
 * addressed independently from String B. */
#define ACCUMULATOR_SMB_STRING STRING_A
#else
/* Single-SMB EVAL jumper reaches the monitor through CS_B / String B. */
#define ACCUMULATOR_SMB_STRING STRING_B
#endif
#endif

/* The physical DER26 accumulator is five 15s segments.  The single-SMB
 * release is intentionally a bench image, but a vehicle build must never be
 * able to inherit that one-device storage topology by accident. */
#if (AMS_BUILD_PROFILE == AMS_PROFILE_VEHICLE) && (NSMBS != 5)
#error "Vehicle profile requires NSMBS=5 for the five physical accumulator segments"
#endif
#if (AMS_BUILD_PROFILE == AMS_PROFILE_VEHICLE) &&     (ACCUMULATOR_PHYSICAL_CHAIN_COUNT != 6)
#error "Vehicle profile requires ACCUMULATOR_PHYSICAL_CHAIN_COUNT=6 (five SMBs plus one ADBMS2950)"
#endif
#if (AMS_BUILD_PROFILE == AMS_PROFILE_VEHICLE) && !AMS_ENABLE_APM_2950
#error "Vehicle profile requires AMS_ENABLE_APM_2950=1 for the installed final ring"
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
	uint16_t cell_voltage_avg8_mv[NSMBS][NCELLS];
	uint16_t cell_voltage_iir_mv[NSMBS][NCELLS];
	uint16_t avg8_usable_voltage_mask[NSMBS];
	uint16_t iir_usable_voltage_mask[NSMBS];
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
	/* Read-only measurement/diagnostic traffic may remain usable even when the
	 * full safety-ready decision is blocked by a reported CSxFLT.  This flag is
	 * established only after identity, configuration and non-CS diagnostics are
	 * transport-clean.  It never authorizes balancing or BMS_OK. */
	bool smb_transport_ready;
	bool smb_ready;
	HAL_StatusTypeDef smb_init_status;
	bool apm_ready;
	HAL_StatusTypeDef apm_init_status;
	/* One-shot proof that the immediately preceding SMB scan completed while
	 * the complete mixed ring was awake.  accumulator_read_apm() consumes this
	 * token before broadcasting SNAP/UNSNAP commands. */
	bool apm_full_ring_awake_token;

    /* Last hardware balancing transition result.  MUTE is the fast inhibit;
     * durable_zero_verified is the persistent safe state after DCC/PWM zero
     * has been written and read back.  Keep these separate because the
     * ADBMS watchdog can clear MUTE while zeroed PWM/DCC remains safe. */
    bool last_balance_mute_ok;
    bool last_balance_durable_zero_verified;
    bool last_balance_unmute_ok;
    uint8_t last_balance_inhibit_reason;

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
/* Run a complete C-path baseline/even/odd sense-path diagnostic, then always
 * restore and validate a normal redundant C/S conversion before returning.
 * The result identifies an electrical sense-path open; it cannot distinguish
 * a blown 1 A fuse from a wire, connector, tab, trace or resistor open. */
int accumulator_run_c_open_wire_diagnostic(
    accumulator_t *dev,
    adbms6830_open_wire_result_t *result);
/* Read the String-B APM after a successful accumulator_read_volt() has woken
 * the complete mixed ring.  The one-shot readiness token is consumed even if
 * the APM transaction fails.  Standalone service reads must use the driver
 * API and resynchronize SMB counter tracking after their transaction. */
int accumulator_read_apm(accumulator_t *dev, uint32_t now_ms);
int accumulator_read_temp(accumulator_t *dev);
int accumulator_set_temp_ch(accumulator_t *dev, uint8_t channel);
int accumulator_stat_temp(accumulator_t *dev);
int accumulator_set_mux_ch(accumulator_t *dev, uint8_t channel, uint8_t addr7);
void accumulator_update_voltage_stats(accumulator_t *dev);
void accumulator_update_voltage_stats_at(accumulator_t *dev, uint32_t now_ms);
bool accumulator_cell_voltage_usable(const accumulator_t *dev, uint8_t seg, uint8_t cell);
uint16_t accumulator_cell_voltage_mv(const accumulator_t *dev, uint8_t seg, uint8_t cell);
void accumulator_update_temp_stats(accumulator_t *dev);
void accumulator_update_temp_stats_at(accumulator_t *dev, uint32_t now_ms);
bool accumulator_temp_sensor_usable(const accumulator_t *dev, uint8_t seg, uint8_t sensor);
int16_t accumulator_temp_deci_c(const accumulator_t *dev, uint8_t seg, uint8_t sensor);
uint8_t accumulator_configured_smb_count(const accumulator_t *dev);
/* Validate the immutable assumptions that make leading-subset transactions
 * safe in the final mixed ring.  This is deliberately independent of APM
 * sample health: the APM remains advisory, but a corrupted device order,
 * pointer, shared-bus binding or write-end assignment must stop all writes. */
bool accumulator_final_ring_topology_valid(const accumulator_t *dev);
/* True when the verified outbound CFGB/PWM shadow requests at least one
 * discharge path.  It does not independently read hardware; set/clear APIs
 * already perform the authoritative readback before returning success. */
bool accumulator_balance_shadow_active(const accumulator_t *dev);
uint16_t accumulator_balance_shadow_mask(const accumulator_t *dev, uint8_t seg);
/* Compute the exact balancing selection without changing the ADBMS register
 * shadow or touching the bus.  This supports safe policy validation while
 * physical balancing is inhibited. */
bool accumulator_plan_balance(const accumulator_t *dev,
                              uint16_t planned_mask[NSMBS]);
typedef enum
{
    ACCUMULATOR_BALANCE_INHIBIT_NONE = 0,
    ACCUMULATOR_BALANCE_INHIBIT_USER,
    ACCUMULATOR_BALANCE_INHIBIT_VOLTAGE,
    ACCUMULATOR_BALANCE_INHIBIT_TEMP,
    ACCUMULATOR_BALANCE_INHIBIT_COMM,
    ACCUMULATOR_BALANCE_INHIBIT_ADBMS_HEALTH,
    ACCUMULATOR_BALANCE_INHIBIT_SHUTDOWN,
    ACCUMULATOR_BALANCE_INHIBIT_CONFIG,
    ACCUMULATOR_BALANCE_INHIBIT_WRITE_FAILURE
} accumulator_balance_inhibit_reason_t;

int accumulator_set_balance(accumulator_t *dev);
int accumulator_clear_balance(accumulator_t *dev);
int accumulator_emergency_balance_inhibit(accumulator_t *dev,
                                          accumulator_balance_inhibit_reason_t reason);
int accumulator_balance_requalify_and_unmute(accumulator_t *dev);
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
