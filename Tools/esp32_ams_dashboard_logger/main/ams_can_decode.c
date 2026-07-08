#include "ams_can_decode.h"

#include <string.h>

static uint16_t be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8u) | p[1]);
}

static int16_t be_i16(const uint8_t *p)
{
    return (int16_t)be_u16(p);
}

static uint32_t be_u24(const uint8_t *p)
{
    return (((uint32_t)p[0] << 16u) |
            ((uint32_t)p[1] << 8u) |
            (uint32_t)p[2]);
}

static void init_temp_array(ams_dash_state_t *state)
{
    for(uint8_t seg = 0u; seg < AMS_DASH_SEGMENTS; seg++)
    {
        for(uint8_t sensor = 0u; sensor < AMS_DASH_TEMPS_PER_SEG; sensor++)
        {
            state->temp_dC[seg][sensor] = AMS_DASH_TEMP_INVALID;
        }
    }
}

void ams_dash_state_init(ams_dash_state_t *state)
{
    if(state == NULL)
    {
        return;
    }

    memset(state, 0, sizeof(*state));
    init_temp_array(state);
}

bool ams_dash_data_stale(const ams_dash_state_t *state,
                         uint32_t now_ms,
                         uint32_t timeout_ms)
{
    if(state == NULL)
    {
        return true;
    }

    if(state->last_heartbeat_ms == 0u)
    {
        return true;
    }

    return (now_ms - state->last_heartbeat_ms) > timeout_ms;
}

static void decode_cell_detail(ams_dash_state_t *state, const uint8_t data[8])
{
    uint8_t seg = data[0];
    uint8_t start = data[1];

    if((seg >= AMS_DASH_SEGMENTS) || (start >= AMS_DASH_CELLS_PER_SEG))
    {
        return;
    }

    for(uint8_t i = 0u; i < 3u; i++)
    {
        uint8_t cell = (uint8_t)(start + i);
        if(cell < AMS_DASH_CELLS_PER_SEG)
        {
            state->cell_mv[seg][cell] = be_u16(&data[(uint8_t)(2u + (i * 2u))]);
        }
    }
}

static void decode_temp_detail(ams_dash_state_t *state, const uint8_t data[8])
{
    uint8_t seg = data[0];
    uint8_t start = data[1];

    if((seg >= AMS_DASH_SEGMENTS) || (start >= AMS_DASH_TEMPS_PER_SEG))
    {
        return;
    }

    for(uint8_t i = 0u; i < 3u; i++)
    {
        uint8_t sensor = (uint8_t)(start + i);
        if(sensor < AMS_DASH_TEMPS_PER_SEG)
        {
            state->temp_dC[seg][sensor] = be_i16(&data[(uint8_t)(2u + (i * 2u))]);
        }
    }
}

bool ams_dash_decode_frame(ams_dash_state_t *state,
                           const ams_can_frame_t *frame,
                           uint32_t now_ms)
{
    if((state == NULL) || (frame == NULL) || (frame->dlc < 8u) || frame->extended)
    {
        if(state != NULL)
        {
            state->unknown_frames++;
        }
        return false;
    }

    const uint8_t *d = frame->data;
    bool decoded = true;

    state->rx_frames++;
    state->last_rx_ms = now_ms;

    switch(frame->id)
    {
        case AMS_LOGGER_CAN_ID_HEARTBEAT:
            state->protocol_version = d[0];
            state->sequence = d[1];
            state->state = d[2];
            state->status_flags = d[3];
            state->validity_flags = d[4];
            state->current_flags = d[5];
            state->ams_uptime_s = be_u16(&d[6]);
            state->last_heartbeat_ms = now_ms;
            break;

        case AMS_LOGGER_CAN_ID_FAULT_REASONS:
            state->voltage_reason = d[0];
            state->voltage_latched_reason = d[1];
            state->temp_reason = d[2];
            state->temp_pending_reason = d[3];
            state->temp_latched_reason = d[4];
            state->current_reason = d[5];
            state->current_latched_reason = d[6];
            state->current_mode = d[7];
            break;

        case AMS_LOGGER_CAN_ID_PACK_ELECTRICAL:
            state->pack_voltage_dV = be_u16(&d[0]);
            state->current_dA = be_i16(&d[2]);
            state->min_cell_mv = be_u16(&d[4]);
            state->max_cell_mv = be_u16(&d[6]);
            break;

        case AMS_LOGGER_CAN_ID_TEMP_FAN:
            state->max_temp_dC = be_i16(&d[0]);
            state->min_temp_dC = be_i16(&d[2]);
            state->avg_temp_dC = be_i16(&d[4]);
            state->max_fan_percent = d[6];
            state->temp_flags = d[7];
            break;

        case AMS_LOGGER_CAN_ID_VOLTAGE_HEALTH:
            state->max_voltage_seg = d[0];
            state->max_voltage_cell = d[1];
            state->min_voltage_seg = d[2];
            state->min_voltage_cell = d[3];
            state->voltage_usable_count = d[4];
            state->voltage_updated_count = d[5];
            state->voltage_stale_count = d[6];
            state->voltage_pec_fail_count = d[7];
            break;

        case AMS_LOGGER_CAN_ID_TEMP_HEALTH:
            state->max_temp_seg = d[0];
            state->max_temp_sensor = d[1];
            state->min_temp_seg = d[2];
            state->min_temp_sensor = d[3];
            state->temp_usable_count = d[4];
            state->temp_updated_count = d[5];
            state->temp_stale_count = d[6];
            state->temp_invalid_count = d[7];
            break;

        case AMS_LOGGER_CAN_ID_CHARGER:
            state->charger_target_voltage_dV = be_u16(&d[0]);
            state->charger_target_current_dA = be_u16(&d[2]);
            state->charger_read_voltage_dV = be_u16(&d[4]);
            state->charger_flags = d[6];
            state->charger_raw_flags = d[7];
            break;

        case AMS_LOGGER_CAN_ID_CURRENT_DETAIL:
            state->current_dA = be_i16(&d[0]);
            state->current_selected_range = d[2];
            state->current_meas_reason = d[3];
            state->current_reason = d[4];
            state->current_latched_reason = d[5];
            state->current_pending_ms = be_u16(&d[6]);
            break;

        case AMS_LOGGER_CAN_ID_6830_LINK:
            state->adbms6830_last_status = d[0];
            state->adbms6830_last_xfer_status = d[1];
            state->adbms6830_last_op = d[2];
            state->adbms6830_error_count_u8 = d[3];
            state->adbms6830_pec_fail_mask = be_u16(&d[4]);
            state->adbms6830_counter_mismatch_mask = be_u16(&d[6]);
            break;

        case AMS_LOGGER_CAN_ID_6830_COUNTERS:
            state->adbms6830_error_count = be_u16(&d[0]);
            state->adbms6830_counter_error_count = be_u16(&d[2]);
            state->adbms6830_pec_pass_mask = be_u16(&d[4]);
            state->adbms6830_last_cmd0 = d[6];
            state->adbms6830_last_cmd1 = d[7];
            break;

        case AMS_LOGGER_CAN_ID_2950_LINK:
            state->adbms2950_last_status = d[0];
            state->adbms2950_last_xfer_status = d[1];
            state->adbms2950_last_op = d[2];
            state->adbms2950_error_count_u8 = d[3];
            state->adbms2950_pec_fail_mask = be_u16(&d[4]);
            state->adbms2950_debug_enabled = d[6];
            state->adbms2950_ic_count = d[7];
            break;

        case AMS_LOGGER_CAN_ID_TASK_HEALTH:
            state->heartbeat_stale_mask = be_u16(&d[0]);
            state->heartbeat_seen_mask = be_u16(&d[2]);
            state->heartbeat_safety_stale_mask = be_u16(&d[4]);
            state->task_health_flags = d[6];
            state->logger_heartbeat_count = d[7];
            break;

        case AMS_LOGGER_CAN_ID_CAN_DIAG:
            state->can_error_code = ((uint32_t)d[0] << 24) |
                                    ((uint32_t)d[1] << 16) |
                                    ((uint32_t)d[2] << 8)  |
                                    ((uint32_t)d[3]);
            state->can_busoff_count = d[4];
            state->can_error_count = d[5];
            state->can_recover_count = d[6];
            state->can_diag_flags = d[7];
            break;

        case AMS_LOGGER_CAN_ID_CELL_DETAIL:
            decode_cell_detail(state, d);
            break;

        case AMS_LOGGER_CAN_ID_TEMP_DETAIL:
            decode_temp_detail(state, d);
            break;

        case AMS_LOGGER_CAN_ID_VOLTAGE_MASKS:
            if(d[0] < AMS_DASH_SEGMENTS)
            {
                state->voltage_updated_mask[d[0]] = be_u16(&d[1]);
                state->voltage_usable_mask[d[0]] = be_u16(&d[3]);
                state->voltage_stale_mask[d[0]] = be_u16(&d[5]);
            }
            break;

        case AMS_LOGGER_CAN_ID_TEMP_MASKS_A:
            if(d[0] < AMS_DASH_SEGMENTS)
            {
                state->temp_updated_mask[d[0]] = be_u24(&d[1]);
                state->temp_usable_mask[d[0]] = be_u24(&d[4]);
            }
            break;

        case AMS_LOGGER_CAN_ID_TEMP_MASKS_B:
            if(d[0] < AMS_DASH_SEGMENTS)
            {
                state->temp_stale_mask[d[0]] = be_u24(&d[1]);
                state->temp_invalid_mask[d[0]] = be_u24(&d[4]);
            }
            break;

        case AMS_LOGGER_CAN_ID_VOLTAGE_PEC:
            if(d[0] < AMS_DASH_SEGMENTS)
            {
                state->voltage_pec_mask[d[0]] = be_u16(&d[1]);
            }
            break;

        default:
            decoded = false;
            state->unknown_frames++;
            break;
    }

    if(decoded)
    {
        state->logger_frames++;
    }

    return decoded;
}
