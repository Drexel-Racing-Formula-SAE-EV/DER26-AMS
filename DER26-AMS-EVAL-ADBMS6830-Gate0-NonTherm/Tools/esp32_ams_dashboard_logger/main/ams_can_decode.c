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

static bool is_known_non_dashboard_std_id(uint32_t id)
{
    switch(id)
    {
        case AMS_DASH_CAN_ID_ECU_AMS:
        case AMS_DASH_CAN_ID_HIL_CELL_SAMPLE:
        case AMS_DASH_CAN_ID_HIL_TEMP_SAMPLE:
        case AMS_DASH_CAN_ID_HIL_CTRL:
            return true;
        default:
            return false;
    }
}

static void decode_hil_meas(ams_dash_state_t *state, const uint8_t data[8], uint32_t now_ms)
{
    state->hil_pack_voltage_cV = be_u16(&data[0]);
    state->hil_current_cA = be_i16(&data[2]);
    state->hil_surface_temp_cC = be_i16(&data[4]);
    state->hil_meas_counter = data[6];
    state->hil_flags |= 0x01u;
    state->hil_last_rx_ms = now_ms;
}

static void decode_hil_truth(ams_dash_state_t *state, const uint8_t data[8], uint32_t now_ms)
{
    state->hil_soc_centi_pct = be_u16(&data[0]);
    state->hil_core_temp_cC = be_i16(&data[2]);
    state->hil_truth_counter = data[4];
    state->hil_plant_step = (((uint32_t)data[5] << 16u) |
                             ((uint32_t)data[6] << 8u) |
                             (uint32_t)data[7]);
    state->hil_flags |= 0x02u;
    state->hil_last_rx_ms = now_ms;
}

static void decode_hil_summary(ams_dash_state_t *state, const uint8_t data[8], uint32_t now_ms)
{
    state->hil_min_cell_mv = be_u16(&data[0]);
    state->hil_max_cell_mv = be_u16(&data[2]);
    state->hil_max_temp_cC = be_i16(&data[4]);
    state->hil_avg_temp_cC = be_i16(&data[6]);
    state->hil_flags |= 0x04u;
    state->hil_last_rx_ms = now_ms;
}

bool ams_dash_decode_frame(ams_dash_state_t *state,
                           const ams_can_frame_t *frame,
                           uint32_t now_ms)
{
    if((state == NULL) || (frame == NULL))
    {
        return false;
    }

    state->rx_frames++;
    state->last_rx_ms = now_ms;

    if(frame->extended)
    {
        state->ignored_frames++;
        return false;
    }

    if(frame->dlc < 8u)
    {
        state->malformed_frames++;
        return false;
    }

    const uint8_t *d = frame->data;
    bool decoded = true;

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

        case AMS_LOGGER_CAN_ID_TEMP_DIAG:
            state->temp_filtered_max_dC = be_i16(&d[0]);
            state->temp_max_rate_dC_per_s = be_i16(&d[2]);
            state->temp_diag_flags = d[4];
            state->fan_control_reason = d[5];
            state->fan_command_percent = d[6];
            state->fan_diag_flags = d[7];
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

        case AMS_LOGGER_CAN_ID_SAFETY_DIAG:
            state->safety_reset_flags = ((uint32_t)d[0] << 24) |
                                        ((uint32_t)d[1] << 16) |
                                        ((uint32_t)d[2] << 8)  |
                                        ((uint32_t)d[3]);
            state->safety_last_panic_reason = d[4];
            state->safety_panic_count = d[5];
            state->safety_bms_block_count = d[6];
            state->safety_flags = d[7];
            break;

        case AMS_LOGGER_CAN_ID_WATCHDOG_DIAG:
            state->watchdog_flags = d[0];
            state->watchdog_last_block_reason = d[1];
            state->watchdog_feed_count = be_u16(&d[2]);
            state->watchdog_block_count = be_u16(&d[4]);
            state->watchdog_last_feed_age_ds = be_u16(&d[6]);
            break;

        case AMS_LOGGER_CAN_ID_RTOS_DIAG:
            state->rtos_heap_free_div16 = be_u16(&d[0]);
            state->rtos_heap_min_div16 = be_u16(&d[2]);
            state->rtos_stack_warn_mask = be_u16(&d[4]);
            state->rtos_min_stack_high_water_words = d[6];
            state->rtos_flags = d[7];
            break;

        case AMS_LOGGER_CAN_ID_ADBMS_DIAG:
            state->adbms_scan_count = be_u16(&d[0]);
            state->adbms_status_diag_count = d[2];
            state->adbms_config_diag_count = d[3];
            state->adbms_open_wire_diag_count = d[4];
            state->adbms_last_diag_status = d[5];
            state->adbms_diag_flags = d[6];
            state->adbms_hil_flags = d[7];
            break;

        case AMS_LOGGER_CAN_ID_CURRENT_ADC:
            state->current_adc_high_count = be_u16(&d[0]);
            state->current_adc_low_count = be_u16(&d[2]);
            state->current_selected_range = d[4];
            state->current_meas_reason = d[5];
            state->current_adc_flags = d[6];
            state->current_zero_cal_count = d[7];
            break;

        case AMS_LOGGER_CAN_ID_CHARGER_DETAIL:
            state->charger_read_current_dA = be_i16(&d[0]);
            state->charger_disable_reason_mask = be_u16(&d[2]);
            state->charger_last_tx_status = d[4];
            state->charger_tx_count = d[5];
            state->charger_rx_count = d[6];
            state->charger_tx_fail_count = d[7];
            break;

        case AMS_DASH_CAN_ID_ESTIMATOR_STATUS:
            state->estimator_active_index = d[0];
            state->estimator_flags = d[1];
            state->estimator_soc_centi_pct = be_u16(&d[2]);
            state->estimator_innovation_mV = be_i16(&d[4]);
            state->estimator_r0_0p01_mohm = be_u16(&d[6]);
            state->estimator_last_rx_ms = now_ms;
            break;

        case AMS_DASH_CAN_ID_HIL_MEAS:
            decode_hil_meas(state, d, now_ms);
            break;

        case AMS_DASH_CAN_ID_HIL_TRUTH:
            decode_hil_truth(state, d, now_ms);
            break;

        case AMS_DASH_CAN_ID_HIL_SUMMARY:
            decode_hil_summary(state, d, now_ms);
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

        case AMS_LOGGER_CAN_ID_TEMP_DIAG_A:
            if(d[0] < AMS_DASH_SEGMENTS)
            {
                state->temp_open_mask[d[0]] = be_u24(&d[1]);
                state->temp_short_mask[d[0]] = be_u24(&d[4]);
            }
            break;

        case AMS_LOGGER_CAN_ID_TEMP_DIAG_B:
            if(d[0] < AMS_DASH_SEGMENTS)
            {
                state->temp_jump_mask[d[0]] = be_u24(&d[1]);
                state->temp_rate_rise_mask[d[0]] = be_u24(&d[4]);
            }
            break;

        case AMS_LOGGER_CAN_ID_VOLTAGE_DIAG:
            if(d[0] < AMS_DASH_SEGMENTS)
            {
                state->voltage_jump_mask[d[0]] = be_u16(&d[1]);
                state->voltage_stuck_mask[d[0]] = be_u16(&d[3]);
            }
            break;

        default:
            decoded = false;
            if(is_known_non_dashboard_std_id(frame->id))
            {
                state->ignored_frames++;
            }
            else
            {
                state->unknown_frames++;
            }
            break;
    }

    if(decoded)
    {
        state->logger_frames++;
    }

    return decoded;
}
