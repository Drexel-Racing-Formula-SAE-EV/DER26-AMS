/*
 * temperature_fault.c
 * Author: Mahad Faisal (2026)
 *
 * Temperature policy for DER26 AMS.
 *
 * Temperature sensing is slower than cell voltage because the SMB mux scan
 * reads three of twenty-four sensors per ADBMS task cycle. This policy treats
 * full scan freshness as a safety input, while warning-only hot/cold states
 * remain recoverable telemetry/fan-control states.
 *
 * Threshold basis:
 * - Molicel P42A datasheet: charge temperature 0 C to 45 C.
 * - Molicel P42A datasheet: discharge temperature -40 C to 60 C.
 * - 65 C is a severe diagnostic tier pending team signoff on whether it should
 *   drive a distinct hardware action beyond the 60 C BMS_OK drop/latch.
 */

#include "ext_drivers/temperature_fault.h"

#include <stddef.h>

static void temperature_clear_non_latched(temperature_fault_state_t *state)
{
    state->temp_valid = false;
    state->read_fault = false;
    state->warning = false;
    state->fan_max = false;
    state->charge_stop = false;
    state->overtemp_fault = false;
    state->severe_overtemp_fault = false;
    state->confirmed = false;
    state->reason = TEMPERATURE_FAULT_REASON_NONE;
}

static void temperature_set_latched(temperature_fault_state_t *state, temperature_fault_reason_t reason)
{
    state->confirmed = true;
    state->latched = true;
    state->latched_reason = reason;
    state->reason = reason;
}

void temperature_fault_init(temperature_fault_state_t *state)
{
    if(state == NULL)
    {
        return;
    }

    *state = (temperature_fault_state_t){0};
    state->reason = TEMPERATURE_FAULT_REASON_NOT_READY;
    state->pending_reason = TEMPERATURE_FAULT_REASON_NONE;
    state->latched_reason = TEMPERATURE_FAULT_REASON_NONE;
}

void temperature_fault_reset_latch(temperature_fault_state_t *state)
{
    if(state == NULL)
    {
        return;
    }

    state->latched = false;
    state->latched_reason = TEMPERATURE_FAULT_REASON_NONE;
    if(state->confirmed)
    {
        state->confirmed = false;
        state->reason = TEMPERATURE_FAULT_REASON_NONE;
    }
    state->pending = false;
    state->pending_reason = TEMPERATURE_FAULT_REASON_NONE;
    state->pending_ms = 0u;
    state->threshold_deci_c = 0;
}

void temperature_fault_update(temperature_fault_state_t *state, const accumulator_t *acc)
{
    temperature_fault_update_with_period(state, acc, TEMP_FAULT_DEFAULT_SAMPLE_MS);
}

static void clear_pending(temperature_fault_state_t *state)
{
    state->pending = false;
    state->pending_reason = TEMPERATURE_FAULT_REASON_NONE;
    state->pending_ms = 0u;
    state->threshold_deci_c = 0;
}

static void update_hot_pending(temperature_fault_state_t *state,
                               temperature_fault_reason_t reason,
                               int16_t threshold_deci_c,
                               uint32_t confirm_ms,
                               uint32_t sample_period_ms)
{
    state->warning = true;
    state->fan_max = true;
    state->pending = true;
    state->reason = reason;
    state->threshold_deci_c = threshold_deci_c;

    if(state->pending_reason != reason)
    {
        state->pending_reason = reason;
        state->pending_ms = 0u;
    }

    if(UINT32_MAX - state->pending_ms < sample_period_ms)
    {
        state->pending_ms = UINT32_MAX;
    }
    else
    {
        state->pending_ms += sample_period_ms;
    }

    if(state->pending_ms >= confirm_ms)
    {
        state->overtemp_fault = true;
        if(reason == TEMPERATURE_FAULT_REASON_HOT_SEVERE)
        {
            state->severe_overtemp_fault = true;
        }
        temperature_set_latched(state, reason);
    }
}

void temperature_fault_update_with_period(temperature_fault_state_t *state,
                                          const accumulator_t *acc,
                                          uint32_t sample_period_ms)
{
    if(state == NULL)
    {
        return;
    }

    temperature_clear_non_latched(state);

    if(acc == NULL)
    {
        state->read_fault = true;
        state->confirmed = true;
        state->fan_max = true;
        state->reason = TEMPERATURE_FAULT_REASON_NOT_READY;
        clear_pending(state);
        return;
    }

    state->usable_sensor_count = acc->usable_temp_count;
    state->updated_sensor_count = acc->updated_temp_count;
    state->stale_sensor_count = acc->stale_temp_count;
    state->invalid_sensor_count = acc->invalid_temp_count;
    state->open_sensor_count = acc->temp_open_count;
    state->short_sensor_count = acc->temp_short_count;
    state->jump_sensor_count = acc->temp_jump_count;
    state->rate_rise_sensor_count = acc->temp_rate_rise_count;
    state->max_temp_deci_c = acc->max_temp_deci_c;
    state->min_temp_deci_c = acc->min_temp_deci_c;
    state->filtered_max_temp_deci_c = acc->filtered_max_temp_deci_c;
    state->filtered_avg_temp_deci_c = acc->filtered_avg_temp_deci_c;
    state->max_rate_deci_c_per_s = acc->temp_max_rate_deci_c_per_s;
    state->max_temp_segment = acc->max_temp_seg;
    state->max_temp_sensor = acc->max_temp_sensor;
    state->min_temp_segment = acc->min_temp_seg;
    state->min_temp_sensor = acc->min_temp_sensor;
    state->max_rate_segment = acc->temp_max_rate_seg;
    state->max_rate_sensor = acc->temp_max_rate_sensor;

    if(!acc->temp_startup_scan_complete)
    {
        state->read_fault = true;
        state->confirmed = true;
        state->fan_max = true;
        clear_pending(state);
        state->reason = TEMPERATURE_FAULT_REASON_NOT_READY;
        return;
    }

    if(!acc->temp_full_usable)
    {
        state->read_fault = true;
        state->confirmed = true;
        state->fan_max = true;
        clear_pending(state);
        if(acc->temp_open_count > 0u)
        {
            state->reason = TEMPERATURE_FAULT_REASON_OPEN_SENSOR;
        }
        else if(acc->temp_short_count > 0u)
        {
            state->reason = TEMPERATURE_FAULT_REASON_SHORT_SENSOR;
        }
        else if(acc->invalid_temp_count > 0u)
        {
            state->reason = TEMPERATURE_FAULT_REASON_INVALID_SENSOR;
        }
        else if(acc->stale_temp_count > 0u)
        {
            state->reason = TEMPERATURE_FAULT_REASON_STALE_SCAN;
        }
        else
        {
            state->reason = TEMPERATURE_FAULT_REASON_PARTIAL_SCAN;
        }
        return;
    }

    state->temp_valid = true;

    if(acc->max_temp_deci_c >= (int16_t)(TEMP_HOT_SEVERE_C * 10.0f))
    {
        update_hot_pending(state,
                           TEMPERATURE_FAULT_REASON_HOT_SEVERE,
                           (int16_t)(TEMP_HOT_SEVERE_C * 10.0f),
                           TEMP_HOT_SEVERE_CONFIRM_MS,
                           sample_period_ms);
        return;
    }

    if(acc->max_temp_deci_c >= (int16_t)(TEMP_HOT_HARD_C * 10.0f))
    {
        update_hot_pending(state,
                           TEMPERATURE_FAULT_REASON_HOT_HARD,
                           (int16_t)(TEMP_HOT_HARD_C * 10.0f),
                           TEMP_HOT_HARD_CONFIRM_MS,
                           sample_period_ms);
        return;
    }

    clear_pending(state);

    if(acc->max_temp_deci_c >= (int16_t)(TEMP_FAN_MAX_C * 10.0f))
    {
        state->fan_max = true;
        state->charge_stop = true;
        state->warning = true;
        state->reason = TEMPERATURE_FAULT_REASON_HOT_FAN_MAX;
        return;
    }

    if(acc->max_temp_deci_c >= (int16_t)(TEMP_CHARGE_MAX_C * 10.0f))
    {
        state->charge_stop = true;
        state->warning = true;
        state->reason = TEMPERATURE_FAULT_REASON_HOT_CHARGE_STOP;
        return;
    }

    if(acc->min_temp_deci_c <= (int16_t)(TEMP_CHARGE_MIN_C * 10.0f))
    {
        state->charge_stop = true;
        state->warning = true;
        state->reason = TEMPERATURE_FAULT_REASON_COLD_CHARGE_STOP;
        return;
    }

    /* Jump/rate masks are diagnostic telemetry only. Do not let display/filter
     * diagnostics change the established temp warning/fault policy. */
    /* The SMB mux intentionally updates three of twenty-four temperature
     * sensors per ADBMS cycle. Once the full sensor set is fresh and usable,
     * a partial update in this cycle is normal cadence, not a warning.
     */
}

const char *temperature_fault_reason_str(temperature_fault_reason_t reason)
{
    switch(reason)
    {
        case TEMPERATURE_FAULT_REASON_NONE:           return "none";
        case TEMPERATURE_FAULT_REASON_NOT_READY:      return "not_ready";
        case TEMPERATURE_FAULT_REASON_PARTIAL_SCAN:   return "partial_scan";
        case TEMPERATURE_FAULT_REASON_STALE_SCAN:     return "stale_scan";
        case TEMPERATURE_FAULT_REASON_INVALID_SENSOR: return "invalid_sensor";
        case TEMPERATURE_FAULT_REASON_OPEN_SENSOR:    return "open_sensor";
        case TEMPERATURE_FAULT_REASON_SHORT_SENSOR:   return "short_sensor";
        case TEMPERATURE_FAULT_REASON_IMPLAUSIBLE_JUMP: return "implausible_jump";
        case TEMPERATURE_FAULT_REASON_RATE_RISE_WARNING: return "rate_rise_warning";
        case TEMPERATURE_FAULT_REASON_COLD_CHARGE_STOP: return "cold_charge_stop";
        case TEMPERATURE_FAULT_REASON_HOT_CHARGE_STOP:  return "hot_charge_stop";
        case TEMPERATURE_FAULT_REASON_HOT_WARNING:    return "hot_warning";
        case TEMPERATURE_FAULT_REASON_HOT_FAN_MAX:    return "hot_fan_max";
        case TEMPERATURE_FAULT_REASON_HOT_HARD:       return "hot_hard";
        case TEMPERATURE_FAULT_REASON_HOT_SEVERE:     return "hot_severe";
        case TEMPERATURE_FAULT_REASON_COLD_WARNING:   return "cold_warning";
        default:                                      return "unknown";
    }
}
