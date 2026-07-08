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

#define AMS_LOGGER_CAN_ID_CELL_DETAIL     0x6A0u
#define AMS_LOGGER_CAN_ID_TEMP_DETAIL     0x6A1u
#define AMS_LOGGER_CAN_ID_VOLTAGE_MASKS   0x6A2u
#define AMS_LOGGER_CAN_ID_TEMP_MASKS_A    0x6A3u
#define AMS_LOGGER_CAN_ID_TEMP_MASKS_B    0x6A4u
#define AMS_LOGGER_CAN_ID_VOLTAGE_PEC     0x6A5u

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

    uint16_t charger_target_voltage_dV;
    uint16_t charger_target_current_dA;
    uint16_t charger_read_voltage_dV;
    uint8_t charger_flags;
    uint8_t charger_raw_flags;

    uint8_t current_selected_range;
    uint8_t current_meas_reason;
    uint16_t current_pending_ms;

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

    uint16_t cell_mv[AMS_DASH_SEGMENTS][AMS_DASH_CELLS_PER_SEG];
    int16_t temp_dC[AMS_DASH_SEGMENTS][AMS_DASH_TEMPS_PER_SEG];

    uint16_t voltage_updated_mask[AMS_DASH_SEGMENTS];
    uint16_t voltage_usable_mask[AMS_DASH_SEGMENTS];
    uint16_t voltage_stale_mask[AMS_DASH_SEGMENTS];
    uint16_t voltage_pec_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_updated_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_usable_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_stale_mask[AMS_DASH_SEGMENTS];
    uint32_t temp_invalid_mask[AMS_DASH_SEGMENTS];
} ams_dash_state_t;

void ams_dash_state_init(ams_dash_state_t *state);
bool ams_dash_decode_frame(ams_dash_state_t *state,
                           const ams_can_frame_t *frame,
                           uint32_t now_ms);
bool ams_dash_data_stale(const ams_dash_state_t *state,
                         uint32_t now_ms,
                         uint32_t timeout_ms);

#endif /* AMS_CAN_DECODE_H_ */
