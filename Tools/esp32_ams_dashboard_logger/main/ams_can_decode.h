/*
 * ams_can_decode.h
 *
 * Pure decoder for the DER26 AMS dashboard/logger CAN contract. This file has
 * no ESP-IDF dependency so the decoder can be host-tested with GCC.
 */

#ifndef AMS_CAN_DECODE_H_
#define AMS_CAN_DECODE_H_

#include <stdbool.h>
#include <stdint.h>

#define AMS_DASH_SEGMENTS       5u
#define AMS_DASH_CELLS_PER_SEG  15u
#define AMS_DASH_TEMPS_PER_SEG  24u
#define AMS_DASH_TEMP_INVALID   ((int16_t)0x8000)

#define AMS_LOGGER_PROTOCOL_VERSION       1u

#define AMS_LOGGER_CAN_ID_HEARTBEAT       0x690u
#define AMS_LOGGER_CAN_ID_FAULT_REASONS   0x691u
#define AMS_LOGGER_CAN_ID_PACK_ELECTRICAL 0x692u
#define AMS_LOGGER_CAN_ID_TEMP_FAN        0x693u
#define AMS_LOGGER_CAN_ID_VOLTAGE_HEALTH  0x694u
#define AMS_LOGGER_CAN_ID_TEMP_HEALTH     0x695u
#define AMS_LOGGER_CAN_ID_CHARGER         0x696u
#define AMS_LOGGER_CAN_ID_CURRENT_DETAIL  0x697u
#define AMS_LOGGER_CAN_ID_6830_LINK       0x698u
#define AMS_LOGGER_CAN_ID_6830_COUNTERS   0x699u
#define AMS_LOGGER_CAN_ID_2950_LINK       0x69Au
#define AMS_LOGGER_CAN_ID_TASK_HEALTH     0x69Bu
#define AMS_LOGGER_CAN_ID_CAN_DIAG        0x69Cu
#define AMS_LOGGER_CAN_ID_SAFETY_DIAG     0x69Du
#define AMS_LOGGER_CAN_ID_WATCHDOG_DIAG   0x69Eu
#define AMS_LOGGER_CAN_ID_ADBMS_DIAG      0x69Fu

#define AMS_LOGGER_CAN_ID_CELL_DETAIL     0x6A0u
#define AMS_LOGGER_CAN_ID_TEMP_DETAIL     0x6A1u
#define AMS_LOGGER_CAN_ID_VOLTAGE_MASKS   0x6A2u
#define AMS_LOGGER_CAN_ID_TEMP_MASKS_A    0x6A3u
#define AMS_LOGGER_CAN_ID_TEMP_MASKS_B    0x6A4u
#define AMS_LOGGER_CAN_ID_VOLTAGE_PEC     0x6A5u
#define AMS_LOGGER_CAN_ID_CURRENT_ADC     0x6A6u
#define AMS_LOGGER_CAN_ID_CHARGER_DETAIL  0x6A7u
#define AMS_LOGGER_CAN_ID_TEMP_DIAG       0x6A8u
#define AMS_LOGGER_CAN_ID_TEMP_DIAG_A     0x6A9u
#define AMS_LOGGER_CAN_ID_TEMP_DIAG_B     0x6AAu
#define AMS_LOGGER_CAN_ID_VOLTAGE_DIAG    0x6ABu
#define AMS_LOGGER_CAN_ID_RTOS_DIAG       0x6ACu

#define AMS_POWER_CAN_PROTOCOL_VERSION    2u
#define AMS_POWER_CAN_ID_DCL               0x684u
#define AMS_POWER_CAN_ID_CCL               0x685u
#define AMS_POWER_CAN_ID_SOH               0x686u
#define AMS_POWER_CAN_ID_ENVELOPE          0x687u
#define AMS_POWER_CAN_ID_STRATEGY          0x689u
#define AMS_POWER_CAN_ID_BINDINGS          0x68Au
#define AMS_POWER_CAN_FRAME_COUNT          6u
#define AMS_POWER_CAN_REQUIRED_FRAME_COUNT 4u

#define AMS_POWER_FLAG_VALID              (1u << 0u)
#define AMS_POWER_FLAG_AUTHORITY_VALID    (1u << 1u)
#define AMS_POWER_FLAG_CAPACITY_PRIOR     (1u << 2u)
#define AMS_POWER_FLAG_RESISTANCE_PRIOR   (1u << 3u)
#define AMS_POWER_FLAG_AMBIENT_PROXY      (1u << 4u)
#define AMS_POWER_FLAG_SLEWED             (1u << 5u)
#define AMS_POWER_FLAG_DIRECTION_INHIBIT  (1u << 6u)
#define AMS_POWER_FLAG_FALLBACK           (1u << 7u)

#define AMS_DASH_CAN_ID_ECU_AMS           0x069u
#define AMS_DASH_CAN_ID_HIL_MEAS          0x200u
#define AMS_DASH_CAN_ID_HIL_TRUTH         0x201u
#define AMS_DASH_CAN_ID_HIL_SUMMARY       0x202u
#define AMS_DASH_CAN_ID_HIL_CELL_SAMPLE   0x210u
#define AMS_DASH_CAN_ID_HIL_TEMP_SAMPLE   0x211u
#define AMS_DASH_CAN_ID_HIL_CTRL          0x300u
#define AMS_DASH_CAN_ID_ESTIMATOR_STATUS  0x421u

typedef struct
{
    uint32_t id;
    bool extended;
    uint8_t dlc;
    uint8_t data[8];
} ams_can_frame_t;

typedef struct
{
    uint32_t rx_frames;
    uint32_t logger_frames;
    uint32_t unknown_frames;
    uint32_t ignored_frames;
    uint32_t malformed_frames;
    uint32_t last_rx_ms;
    uint32_t last_heartbeat_ms;

    uint8_t protocol_version;
    uint8_t sequence;
    uint8_t state;
    uint8_t status_flags;
    uint8_t validity_flags;
    uint8_t current_flags;
    uint16_t ams_uptime_s;

    uint8_t voltage_reason;
    uint8_t voltage_latched_reason;
    uint8_t temp_reason;
    uint8_t temp_pending_reason;
    uint8_t temp_latched_reason;
    uint8_t current_reason;
    uint8_t current_latched_reason;
    uint8_t current_mode;

    uint16_t pack_voltage_dV;
    int16_t current_dA;
    uint16_t min_cell_mv;
    uint16_t max_cell_mv;

    int16_t max_temp_dC;
    int16_t min_temp_dC;
    int16_t avg_temp_dC;
    uint8_t max_fan_percent;
    uint8_t temp_flags;

    uint8_t max_voltage_seg;
    uint8_t max_voltage_cell;
    uint8_t min_voltage_seg;
    uint8_t min_voltage_cell;
    uint8_t voltage_usable_count;
    uint8_t voltage_updated_count;
    uint8_t voltage_stale_count;
    uint8_t voltage_pec_fail_count;

    uint8_t max_temp_seg;
    uint8_t max_temp_sensor;
    uint8_t min_temp_seg;
    uint8_t min_temp_sensor;
    uint8_t temp_usable_count;
    uint8_t temp_updated_count;
    uint8_t temp_stale_count;
    uint8_t temp_invalid_count;
    int16_t temp_filtered_max_dC;
    int16_t temp_max_rate_dC_per_s;
    uint8_t temp_diag_flags;
    uint8_t fan_control_reason;
    uint8_t fan_command_percent;
    uint8_t fan_diag_flags;

    uint16_t charger_target_voltage_dV;
    uint16_t charger_target_current_dA;
    uint16_t charger_read_voltage_dV;
    uint8_t charger_flags;
    uint8_t charger_raw_flags;
    int16_t charger_read_current_dA;
    uint16_t charger_disable_reason_mask;
    uint8_t charger_last_tx_status;
    uint8_t charger_tx_count;
    uint8_t charger_rx_count;
    uint8_t charger_tx_fail_count;

    uint8_t current_selected_range;
    uint8_t current_meas_reason;
    uint16_t current_pending_ms;
    uint16_t current_adc_high_count;
    uint16_t current_adc_low_count;
    uint8_t current_adc_flags;
    uint8_t current_zero_cal_count;

    uint8_t adbms6830_last_status;
    uint8_t adbms6830_last_xfer_status;
    uint8_t adbms6830_last_op;
    uint8_t adbms6830_error_count_u8;
    uint16_t adbms6830_pec_fail_mask;
    uint16_t adbms6830_counter_mismatch_mask;
    uint16_t adbms6830_error_count;
    uint16_t adbms6830_counter_error_count;
    uint16_t adbms6830_pec_pass_mask;
    uint8_t adbms6830_last_cmd0;
    uint8_t adbms6830_last_cmd1;

    uint8_t adbms2950_last_status;
    uint8_t adbms2950_last_xfer_status;
    uint8_t adbms2950_last_op;
    uint8_t adbms2950_error_count_u8;
    uint16_t adbms2950_pec_fail_mask;
    uint8_t adbms2950_debug_enabled;
    uint8_t adbms2950_ic_count;

    uint16_t heartbeat_stale_mask;
    uint16_t heartbeat_seen_mask;
    uint16_t heartbeat_safety_stale_mask;
    uint8_t task_health_flags;
    uint8_t logger_heartbeat_count;

    uint32_t can_error_code;
    uint8_t can_busoff_count;
    uint8_t can_error_count;
    uint8_t can_recover_count;
    uint8_t can_diag_flags;

    uint32_t safety_reset_flags;
    uint8_t safety_last_panic_reason;
    uint8_t safety_panic_count;
    uint8_t safety_bms_block_count;
    uint8_t safety_flags;

    uint8_t watchdog_flags;
    uint8_t watchdog_last_block_reason;
    uint16_t watchdog_feed_count;
    uint16_t watchdog_block_count;
    uint16_t watchdog_last_feed_age_ds;

    uint16_t rtos_heap_free_div16;
    uint16_t rtos_heap_min_div16;
    uint16_t rtos_stack_warn_mask;
    uint8_t rtos_min_stack_high_water_words;
    uint8_t rtos_flags;

    uint16_t adbms_scan_count;
    uint8_t adbms_status_diag_count;
    uint8_t adbms_config_diag_count;
    uint8_t adbms_open_wire_diag_count;
    uint8_t adbms_last_diag_status;
    uint8_t adbms_diag_flags;
    uint8_t adbms_hil_flags;

    uint32_t estimator_last_rx_ms;
    uint8_t estimator_active_index;
    uint8_t estimator_flags;
    uint16_t estimator_soc_centi_pct;
    int16_t estimator_innovation_mV;
    uint16_t estimator_r0_0p01_mohm;

    uint32_t power_last_rx_ms[AMS_POWER_CAN_FRAME_COUNT];
    uint32_t power_crc_error_count;
    uint32_t power_version_error_count;
    uint32_t power_counter_error_count;
    uint8_t power_counter_seen_mask;
    uint8_t power_counter[AMS_POWER_CAN_FRAME_COUNT];
    uint8_t dcl_flags;
    uint8_t ccl_flags;
    uint16_t dcl_current_dA;
    uint16_t ccl_current_dA;
    uint16_t dcl_power_10W;
    uint16_t ccl_power_10W;
    uint8_t dcl_binding;
    uint8_t ccl_binding;
    uint8_t dcl_limiting_segment;
    uint8_t ccl_limiting_segment;
    uint8_t capacity_soh_pct;
    uint8_t capacity_soh_lower_pct;
    uint8_t resistance_growth_upper_pct;
    uint8_t combined_soh_pct;
    uint8_t capacity_confidence_pct;
    uint8_t resistance_confidence_pct;
    uint8_t capacity_soh_valid;
    uint8_t resistance_soh_valid;
    uint8_t envelope_discharge_a[3];
    uint8_t envelope_charge_a[3];
    uint8_t envelope_discharge_binding[3];
    uint8_t envelope_charge_binding[3];
    uint8_t envelope_discharge_segment[3];
    uint8_t envelope_charge_segment[3];
    uint8_t mission_profile;
    uint8_t mission_horizon_index;
    uint8_t thermal_ready;
    uint8_t fuse_authority_valid;
    uint8_t limp_latched;
    uint8_t mission_fallback;
    uint8_t fuse_utilization_pct;
    int8_t minimum_core_temp_c;
    uint16_t thermal_energy_to_target_dWh;
    uint8_t r0_bootstrap_progress_pct;

    uint32_t hil_last_rx_ms;
    uint16_t hil_pack_voltage_cV;
    int16_t hil_current_cA;
    int16_t hil_surface_temp_cC;
    uint16_t hil_soc_centi_pct;
    int16_t hil_core_temp_cC;
    uint16_t hil_min_cell_mv;
    uint16_t hil_max_cell_mv;
    int16_t hil_max_temp_cC;
    int16_t hil_avg_temp_cC;
    uint8_t hil_meas_counter;
    uint8_t hil_truth_counter;
    uint32_t hil_plant_step;
    uint8_t hil_flags;

    uint16_t cell_mv[AMS_DASH_SEGMENTS][AMS_DASH_CELLS_PER_SEG];
    int16_t temp_dC[AMS_DASH_SEGMENTS][AMS_DASH_TEMPS_PER_SEG];

    uint16_t voltage_updated_mask[AMS_DASH_SEGMENTS];
    uint16_t voltage_usable_mask[AMS_DASH_SEGMENTS];
    uint16_t voltage_stale_mask[AMS_DASH_SEGMENTS];
    uint16_t voltage_pec_mask[AMS_DASH_SEGMENTS];
    uint16_t voltage_jump_mask[AMS_DASH_SEGMENTS];
    uint16_t voltage_stuck_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_updated_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_usable_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_stale_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_invalid_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_open_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_short_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_jump_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_rate_rise_mask[AMS_DASH_SEGMENTS];
} ams_dash_state_t;

void ams_dash_state_init(ams_dash_state_t *state);
bool ams_dash_decode_frame(ams_dash_state_t *state,
                           const ams_can_frame_t *frame,
                           uint32_t now_ms);
bool ams_dash_data_stale(const ams_dash_state_t *state,
                         uint32_t now_ms,
                         uint32_t timeout_ms);
bool ams_dash_power_data_stale(const ams_dash_state_t *state,
                               uint32_t now_ms,
                               uint32_t timeout_ms);
uint8_t ams_dash_power_crc8(uint16_t can_id, const uint8_t payload[7]);

#endif /* AMS_CAN_DECODE_H_ */
