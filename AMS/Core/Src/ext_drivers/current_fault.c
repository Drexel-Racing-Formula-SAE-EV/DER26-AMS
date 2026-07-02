/*
 * current_fault.c
 *
 * DER26 AMS current fault policy.
 *
 * Known hardware assumptions captured here:
 * - Main tractive-system fuse: 80 A.
 * - Plug-in charger: 10 A nominal, protected by 15 A charge fuses.
 * - Precharge resistor: 560 ohm, so 315 V / 560 ohm ~= 0.56 A initial.
 * - ECU-side regen command path is currently not implemented. CM200 EEPROM
 *   settings are still pending, so regen limits below are conservative
 *   placeholders until the RMS GUI EEPROM export is reviewed.
 * - Current sign is still bench-confirmed later. For now, positive current is
 *   treated as discharge because that is the conventional AMS pack-current
 *   contract used by this first-pass logic.
 */

#include "ext_drivers/current_fault.h"

#include <math.h>
#include <stddef.h>

#define CURRENT_DIRECTION_DISCHARGE_POSITIVE 1u

#define CURRENT_SENSOR_STARTUP_IGNORE_MS     250u
#define CURRENT_SENSOR_FAULT_CONFIRM_MS      250u

#define CURRENT_DISCHARGE_WARN_A             70.0f
#define CURRENT_DISCHARGE_TRIP_A             85.0f
#define CURRENT_DISCHARGE_FAST_TRIP_A        120.0f
#define CURRENT_DISCHARGE_EXTREME_A          240.0f
#define CURRENT_DISCHARGE_TRIP_MS            500u
#define CURRENT_DISCHARGE_FAST_TRIP_MS       100u

#define CURRENT_CHARGE_WARN_A                10.5f
#define CURRENT_CHARGE_TRIP_A                12.0f
#define CURRENT_CHARGE_FAST_TRIP_A           15.0f
#define CURRENT_CHARGE_EXTREME_A             30.0f
#define CURRENT_CHARGE_TRIP_MS               500u
#define CURRENT_CHARGE_FAST_TRIP_MS          100u

#define CURRENT_REGEN_ENABLED_PLACEHOLDER     0u
#define CURRENT_REGEN_UNEXPECTED_A           5.0f
#define CURRENT_REGEN_WARN_A                 20.0f
#define CURRENT_REGEN_TRIP_A                 25.0f
#define CURRENT_REGEN_FAST_TRIP_A            40.0f
#define CURRENT_REGEN_EXTREME_A              80.0f
#define CURRENT_REGEN_TRIP_MS                500u
#define CURRENT_REGEN_FAST_TRIP_MS           100u

#define CURRENT_PRECHARGE_WARN_A             0.8f
#define CURRENT_PRECHARGE_TRIP_A             1.2f
#define CURRENT_PRECHARGE_FAST_TRIP_A        2.0f
#define CURRENT_PRECHARGE_TRIP_MS            200u
#define CURRENT_PRECHARGE_FAST_TRIP_MS       40u

typedef struct
{
    bool active;
    bool warning_only;
    current_fault_reason_t warning_reason;
    current_fault_reason_t pending_reason;
    current_fault_reason_t fast_reason;
    current_fault_reason_t extreme_reason;
    float threshold_a;
    uint32_t confirm_ms;
} current_fault_eval_t;

static bool current_is_discharge(float current_a)
{
#if CURRENT_DIRECTION_DISCHARGE_POSITIVE
    return current_a > 0.0f;
#else
    return current_a < 0.0f;
#endif
}

static current_fault_reason_t sensor_reason_to_fault_reason(current_sensor_reason_t reason)
{
    switch(reason)
    {
        case CURRENT_SENSOR_REASON_ADC_READ:
            return CURRENT_FAULT_REASON_SENSOR_ADC_READ;
        case CURRENT_SENSOR_REASON_ADC_IMPLAUSIBLE:
            return CURRENT_FAULT_REASON_SENSOR_IMPLAUSIBLE;
        case CURRENT_SENSOR_REASON_SENSOR_SATURATION:
            return CURRENT_FAULT_REASON_SENSOR_SATURATION;
        case CURRENT_SENSOR_REASON_CHANNEL_MISMATCH:
            return CURRENT_FAULT_REASON_SENSOR_CHANNEL_MISMATCH;
        case CURRENT_SENSOR_REASON_NULL:
        case CURRENT_SENSOR_REASON_NOT_MAPPED:
        case CURRENT_SENSOR_REASON_OK:
        default:
            return CURRENT_FAULT_REASON_SENSOR_NOT_READY;
    }
}

static current_fault_eval_t eval_discharge(float abs_current_a)
{
    current_fault_eval_t eval = {0};

    if(abs_current_a >= CURRENT_DISCHARGE_EXTREME_A)
    {
        eval.active = true;
        eval.extreme_reason = CURRENT_FAULT_REASON_DISCHARGE_EXTREME;
        eval.pending_reason = CURRENT_FAULT_REASON_DISCHARGE_EXTREME;
        eval.threshold_a = CURRENT_DISCHARGE_EXTREME_A;
        eval.confirm_ms = 0u;
    }
    else if(abs_current_a >= CURRENT_DISCHARGE_FAST_TRIP_A)
    {
        eval.active = true;
        eval.fast_reason = CURRENT_FAULT_REASON_DISCHARGE_FAST_OVERCURRENT;
        eval.pending_reason = CURRENT_FAULT_REASON_DISCHARGE_FAST_OVERCURRENT;
        eval.threshold_a = CURRENT_DISCHARGE_FAST_TRIP_A;
        eval.confirm_ms = CURRENT_DISCHARGE_FAST_TRIP_MS;
    }
    else if(abs_current_a >= CURRENT_DISCHARGE_TRIP_A)
    {
        eval.active = true;
        eval.pending_reason = CURRENT_FAULT_REASON_DISCHARGE_OVERCURRENT;
        eval.threshold_a = CURRENT_DISCHARGE_TRIP_A;
        eval.confirm_ms = CURRENT_DISCHARGE_TRIP_MS;
    }
    else if(abs_current_a >= CURRENT_DISCHARGE_WARN_A)
    {
        eval.active = true;
        eval.warning_only = true;
        eval.warning_reason = CURRENT_FAULT_REASON_DISCHARGE_WARNING;
        eval.threshold_a = CURRENT_DISCHARGE_WARN_A;
    }

    return eval;
}

static current_fault_eval_t eval_charge(float abs_current_a)
{
    current_fault_eval_t eval = {0};

    if(abs_current_a >= CURRENT_CHARGE_EXTREME_A)
    {
        eval.active = true;
        eval.extreme_reason = CURRENT_FAULT_REASON_CHARGE_EXTREME;
        eval.pending_reason = CURRENT_FAULT_REASON_CHARGE_EXTREME;
        eval.threshold_a = CURRENT_CHARGE_EXTREME_A;
        eval.confirm_ms = 0u;
    }
    else if(abs_current_a >= CURRENT_CHARGE_FAST_TRIP_A)
    {
        eval.active = true;
        eval.fast_reason = CURRENT_FAULT_REASON_CHARGE_FAST_OVERCURRENT;
        eval.pending_reason = CURRENT_FAULT_REASON_CHARGE_FAST_OVERCURRENT;
        eval.threshold_a = CURRENT_CHARGE_FAST_TRIP_A;
        eval.confirm_ms = CURRENT_CHARGE_FAST_TRIP_MS;
    }
    else if(abs_current_a >= CURRENT_CHARGE_TRIP_A)
    {
        eval.active = true;
        eval.pending_reason = CURRENT_FAULT_REASON_CHARGE_OVERCURRENT;
        eval.threshold_a = CURRENT_CHARGE_TRIP_A;
        eval.confirm_ms = CURRENT_CHARGE_TRIP_MS;
    }
    else if(abs_current_a >= CURRENT_CHARGE_WARN_A)
    {
        eval.active = true;
        eval.warning_only = true;
        eval.warning_reason = CURRENT_FAULT_REASON_CHARGE_WARNING;
        eval.threshold_a = CURRENT_CHARGE_WARN_A;
    }

    return eval;
}

static current_fault_eval_t eval_regen(float abs_current_a)
{
    current_fault_eval_t eval = {0};

#if CURRENT_REGEN_ENABLED_PLACEHOLDER == 0u
    if(abs_current_a >= CURRENT_REGEN_UNEXPECTED_A)
    {
        eval.active = true;
        eval.warning_only = true;
        eval.warning_reason = CURRENT_FAULT_REASON_REGEN_UNEXPECTED;
        eval.threshold_a = CURRENT_REGEN_UNEXPECTED_A;
    }
#endif

    if(abs_current_a >= CURRENT_REGEN_EXTREME_A)
    {
        eval.active = true;
        eval.extreme_reason = CURRENT_FAULT_REASON_REGEN_EXTREME;
        eval.pending_reason = CURRENT_FAULT_REASON_REGEN_EXTREME;
        eval.threshold_a = CURRENT_REGEN_EXTREME_A;
        eval.confirm_ms = 0u;
    }
    else if(abs_current_a >= CURRENT_REGEN_FAST_TRIP_A)
    {
        eval.active = true;
        eval.fast_reason = CURRENT_FAULT_REASON_REGEN_FAST_OVERCURRENT;
        eval.pending_reason = CURRENT_FAULT_REASON_REGEN_FAST_OVERCURRENT;
        eval.threshold_a = CURRENT_REGEN_FAST_TRIP_A;
        eval.confirm_ms = CURRENT_REGEN_FAST_TRIP_MS;
    }
    else if(abs_current_a >= CURRENT_REGEN_TRIP_A)
    {
        eval.active = true;
        eval.pending_reason = CURRENT_FAULT_REASON_REGEN_OVERCURRENT;
        eval.threshold_a = CURRENT_REGEN_TRIP_A;
        eval.confirm_ms = CURRENT_REGEN_TRIP_MS;
    }
    else if(abs_current_a >= CURRENT_REGEN_WARN_A)
    {
        eval.active = true;
        eval.warning_only = true;
        eval.warning_reason = CURRENT_FAULT_REASON_REGEN_WARNING;
        eval.threshold_a = CURRENT_REGEN_WARN_A;
    }

    return eval;
}

static current_fault_eval_t eval_precharge(float abs_current_a)
{
    current_fault_eval_t eval = {0};

    if(abs_current_a >= CURRENT_PRECHARGE_FAST_TRIP_A)
    {
        eval.active = true;
        eval.fast_reason = CURRENT_FAULT_REASON_PRECHARGE_FAST_OVERCURRENT;
        eval.pending_reason = CURRENT_FAULT_REASON_PRECHARGE_FAST_OVERCURRENT;
        eval.threshold_a = CURRENT_PRECHARGE_FAST_TRIP_A;
        eval.confirm_ms = CURRENT_PRECHARGE_FAST_TRIP_MS;
    }
    else if(abs_current_a >= CURRENT_PRECHARGE_TRIP_A)
    {
        eval.active = true;
        eval.pending_reason = CURRENT_FAULT_REASON_PRECHARGE_OVERCURRENT;
        eval.threshold_a = CURRENT_PRECHARGE_TRIP_A;
        eval.confirm_ms = CURRENT_PRECHARGE_TRIP_MS;
    }
    else if(abs_current_a >= CURRENT_PRECHARGE_WARN_A)
    {
        eval.active = true;
        eval.warning_only = true;
        eval.warning_reason = CURRENT_FAULT_REASON_PRECHARGE_WARNING;
        eval.threshold_a = CURRENT_PRECHARGE_WARN_A;
    }

    return eval;
}

static void clear_non_latched_faults(current_fault_state_t *state)
{
    state->sensor_fault = false;
    state->warning = false;
    state->pending = false;
    state->confirmed = false;
    state->reason = CURRENT_FAULT_REASON_NONE;
    state->pending_reason = CURRENT_FAULT_REASON_NONE;
    state->pending_ms = 0u;
    state->threshold_a = 0.0f;
}

void current_fault_init(current_fault_state_t *state)
{
    if(state == NULL)
    {
        return;
    }

    *state = (current_fault_state_t){0};
    state->reason = CURRENT_FAULT_REASON_SENSOR_NOT_READY;
    state->pending_reason = CURRENT_FAULT_REASON_NONE;
    state->latched_reason = CURRENT_FAULT_REASON_NONE;
    state->mode = CURRENT_FAULT_MODE_IDLE;
}

void current_fault_reset_latch(current_fault_state_t *state)
{
    if(state == NULL)
    {
        return;
    }

    state->latched = false;
    state->latched_reason = CURRENT_FAULT_REASON_NONE;
    if(state->confirmed)
    {
        state->confirmed = false;
        state->reason = CURRENT_FAULT_REASON_NONE;
    }
}

void current_fault_update(current_fault_state_t *state,
                          current_fault_mode_t mode,
                          float current_a,
                          bool current_valid,
                          current_sensor_reason_t measurement_reason,
                          uint32_t sample_period_ms)
{
    current_fault_eval_t eval = {0};
    bool discharge_current;

    if(state == NULL)
    {
        return;
    }

    if(sample_period_ms == 0u)
    {
        sample_period_ms = 1u;
    }

    state->mode = mode;
    state->abs_current_a = isfinite(current_a) ? fabsf(current_a) : 0.0f;

    if(!current_valid || !isfinite(current_a))
    {
        state->sensor_invalid_ms += sample_period_ms;
        state->warning = false;
        state->pending = false;
        state->confirmed = false;
        state->pending_reason = CURRENT_FAULT_REASON_NONE;
        state->pending_ms = 0u;
        state->threshold_a = 0.0f;
        state->reason = sensor_reason_to_fault_reason(measurement_reason);

        if(state->sensor_invalid_ms >= (CURRENT_SENSOR_STARTUP_IGNORE_MS + CURRENT_SENSOR_FAULT_CONFIRM_MS))
        {
            state->sensor_fault = true;
        }
        return;
    }

    state->sensor_invalid_ms = 0u;
    state->sensor_fault = false;

    discharge_current = current_is_discharge(current_a);

    switch(mode)
    {
        case CURRENT_FAULT_MODE_PRECHARGE:
            eval = eval_precharge(state->abs_current_a);
            break;

        case CURRENT_FAULT_MODE_CHARGE:
            eval = discharge_current ? eval_discharge(state->abs_current_a) : eval_charge(state->abs_current_a);
            break;

        case CURRENT_FAULT_MODE_DRIVE:
            eval = discharge_current ? eval_discharge(state->abs_current_a) : eval_regen(state->abs_current_a);
            break;

        case CURRENT_FAULT_MODE_IDLE:
        default:
            eval = discharge_current ? eval_discharge(state->abs_current_a) : eval_charge(state->abs_current_a);
            break;
    }

    if(!eval.active)
    {
        clear_non_latched_faults(state);
        return;
    }

    state->threshold_a = eval.threshold_a;

    if(eval.warning_only)
    {
        state->warning = true;
        state->pending = false;
        state->confirmed = false;
        state->pending_ms = 0u;
        state->pending_reason = CURRENT_FAULT_REASON_NONE;
        state->reason = eval.warning_reason;
        return;
    }

    state->warning = false;
    state->pending = true;

    if(state->pending_reason != eval.pending_reason)
    {
        state->pending_reason = eval.pending_reason;
        state->pending_ms = 0u;
    }

    state->pending_ms += sample_period_ms;
    state->reason = eval.pending_reason;

    if((eval.confirm_ms == 0u) || (state->pending_ms >= eval.confirm_ms))
    {
        state->pending = false;
        state->confirmed = true;
        state->latched = true;
        state->latched_reason = eval.pending_reason;
    }
}

const char *current_fault_reason_str(current_fault_reason_t reason)
{
    switch(reason)
    {
        case CURRENT_FAULT_REASON_NONE:                         return "none";
        case CURRENT_FAULT_REASON_SENSOR_NOT_READY:             return "sensor_not_ready";
        case CURRENT_FAULT_REASON_SENSOR_ADC_READ:              return "sensor_adc_read";
        case CURRENT_FAULT_REASON_SENSOR_IMPLAUSIBLE:           return "sensor_implausible";
        case CURRENT_FAULT_REASON_SENSOR_SATURATION:            return "sensor_saturation";
        case CURRENT_FAULT_REASON_SENSOR_CHANNEL_MISMATCH:      return "sensor_channel_mismatch";
        case CURRENT_FAULT_REASON_DISCHARGE_WARNING:            return "discharge_warning";
        case CURRENT_FAULT_REASON_DISCHARGE_OVERCURRENT:        return "discharge_overcurrent";
        case CURRENT_FAULT_REASON_DISCHARGE_FAST_OVERCURRENT:   return "discharge_fast_overcurrent";
        case CURRENT_FAULT_REASON_DISCHARGE_EXTREME:            return "discharge_extreme";
        case CURRENT_FAULT_REASON_CHARGE_WARNING:               return "charge_warning";
        case CURRENT_FAULT_REASON_CHARGE_OVERCURRENT:           return "charge_overcurrent";
        case CURRENT_FAULT_REASON_CHARGE_FAST_OVERCURRENT:      return "charge_fast_overcurrent";
        case CURRENT_FAULT_REASON_CHARGE_EXTREME:               return "charge_extreme";
        case CURRENT_FAULT_REASON_REGEN_WARNING:                return "regen_warning";
        case CURRENT_FAULT_REASON_REGEN_OVERCURRENT:            return "regen_overcurrent";
        case CURRENT_FAULT_REASON_REGEN_FAST_OVERCURRENT:       return "regen_fast_overcurrent";
        case CURRENT_FAULT_REASON_REGEN_EXTREME:                return "regen_extreme";
        case CURRENT_FAULT_REASON_REGEN_UNEXPECTED:             return "regen_unexpected";
        case CURRENT_FAULT_REASON_PRECHARGE_WARNING:            return "precharge_warning";
        case CURRENT_FAULT_REASON_PRECHARGE_OVERCURRENT:        return "precharge_overcurrent";
        case CURRENT_FAULT_REASON_PRECHARGE_FAST_OVERCURRENT:   return "precharge_fast_overcurrent";
        default:                                                return "unknown";
    }
}

const char *current_fault_mode_str(current_fault_mode_t mode)
{
    switch(mode)
    {
        case CURRENT_FAULT_MODE_IDLE:       return "idle";
        case CURRENT_FAULT_MODE_DRIVE:      return "drive";
        case CURRENT_FAULT_MODE_CHARGE:     return "charge";
        case CURRENT_FAULT_MODE_PRECHARGE:  return "precharge";
        default:                            return "unknown";
    }
}
