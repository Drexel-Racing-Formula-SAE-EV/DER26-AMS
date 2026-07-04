/*
 * voltage_fault.c
 * Author: Mahad Faisal (2026)
 *
 * Staged cell-voltage policy for DER26 AMS.
 *
 * Threshold basis:
 * - Molicel P42A charge voltage limit: 4.20 V/cell.
 * - Pack topology: 75s total, 15s per segment.
 * - Charger target is expected around 312 V, or 4.16 V/cell.
 *
 * HARD thresholds drop BMS_OK and latch. WARN/CHARGE_STOP are observable
 * states but do not pretend to be a second shutdown path.
 */

#include "ext_drivers/voltage_fault.h"

#include <stddef.h>

static void clear_non_latched(voltage_fault_state_t *state)
{
    state->voltage_valid = false;
    state->read_fault = false;
    state->warning = false;
    state->charge_stop = false;
    state->overvoltage_fault = false;
    state->undervoltage_fault = false;
    state->confirmed = false;
    state->reason = VOLTAGE_FAULT_REASON_NONE;
}

static void set_latched(voltage_fault_state_t *state, voltage_fault_reason_t reason)
{
    state->confirmed = true;
    state->latched = true;
    state->latched_reason = reason;
    state->reason = reason;
}

void voltage_fault_init(voltage_fault_state_t *state)
{
    if(state == NULL)
    {
        return;
    }

    *state = (voltage_fault_state_t){0};
    state->reason = VOLTAGE_FAULT_REASON_NOT_READY;
    state->latched_reason = VOLTAGE_FAULT_REASON_NONE;
}

void voltage_fault_reset_latch(voltage_fault_state_t *state)
{
    if(state == NULL)
    {
        return;
    }

    state->latched = false;
    state->latched_reason = VOLTAGE_FAULT_REASON_NONE;
    if(state->confirmed)
    {
        state->confirmed = false;
        state->reason = VOLTAGE_FAULT_REASON_NONE;
    }
}

void voltage_fault_update(voltage_fault_state_t *state, const accumulator_t *acc)
{
    if(state == NULL)
    {
        return;
    }

    clear_non_latched(state);

    if(acc == NULL)
    {
        state->read_fault = true;
        state->confirmed = true;
        state->reason = VOLTAGE_FAULT_REASON_NOT_READY;
        return;
    }

    state->usable_cell_count = acc->usable_voltage_count;
    state->updated_cell_count = acc->updated_voltage_count;
    state->stale_cell_count = acc->stale_voltage_count;
    state->pec_fail_cell_count = acc->pec_fail_cell_count;
    state->max_cell_mv = acc->max_voltage_mv;
    state->min_cell_mv = acc->min_voltage_mv;
    state->max_cell_segment = acc->max_voltage_seg;
    state->max_cell_index = acc->max_voltage_cell;
    state->min_cell_segment = acc->min_voltage_seg;
    state->min_cell_index = acc->min_voltage_cell;

    if(!acc->voltage_startup_scan_complete)
    {
        state->read_fault = true;
        state->confirmed = true;
        state->reason = VOLTAGE_FAULT_REASON_NOT_READY;
        return;
    }

    if(!acc->voltage_full_usable)
    {
        state->read_fault = true;
        state->confirmed = true;
        if(acc->stale_voltage_count > 0u)
        {
            state->reason = VOLTAGE_FAULT_REASON_STALE_SCAN;
        }
        else if(acc->pec_fail_cell_count > 0u)
        {
            state->reason = VOLTAGE_FAULT_REASON_PEC_FAILURE;
        }
        else
        {
            state->reason = VOLTAGE_FAULT_REASON_PARTIAL_SCAN;
        }
        return;
    }

    state->voltage_valid = true;

    if(acc->max_voltage_mv >= CELL_OV_SEVERE_MV)
    {
        state->overvoltage_fault = true;
        set_latched(state, VOLTAGE_FAULT_REASON_OV_SEVERE);
        return;
    }

    if(acc->min_voltage_mv <= CELL_UV_SEVERE_MV)
    {
        state->undervoltage_fault = true;
        set_latched(state, VOLTAGE_FAULT_REASON_UV_SEVERE);
        return;
    }

    if(acc->max_voltage_mv >= CELL_OV_HARD_MV)
    {
        state->overvoltage_fault = true;
        set_latched(state, VOLTAGE_FAULT_REASON_OV_HARD);
        return;
    }

    if(acc->min_voltage_mv <= CELL_UV_HARD_MV)
    {
        state->undervoltage_fault = true;
        set_latched(state, VOLTAGE_FAULT_REASON_UV_HARD);
        return;
    }

    if(acc->max_voltage_mv >= CELL_CHARGE_STOP_MV)
    {
        state->charge_stop = true;
        state->warning = true;
        state->reason = VOLTAGE_FAULT_REASON_CHARGE_STOP;
        return;
    }

    if(acc->min_voltage_mv <= CELL_UV_SOFT_MV)
    {
        state->warning = true;
        state->reason = VOLTAGE_FAULT_REASON_UV_SOFT;
        return;
    }

    if(acc->max_voltage_mv >= CELL_OV_WARN_MV)
    {
        state->warning = true;
        state->reason = VOLTAGE_FAULT_REASON_OV_WARNING;
        return;
    }

    if(acc->min_voltage_mv <= CELL_UV_WARN_MV)
    {
        state->warning = true;
        state->reason = VOLTAGE_FAULT_REASON_UV_WARNING;
        return;
    }

    if(!acc->voltage_full_updated)
    {
        state->warning = true;
        state->reason = (acc->pec_fail_cell_count > 0u) ?
                        VOLTAGE_FAULT_REASON_PEC_FAILURE :
                        VOLTAGE_FAULT_REASON_PARTIAL_SCAN;
    }
}

const char *voltage_fault_reason_str(voltage_fault_reason_t reason)
{
    switch(reason)
    {
        case VOLTAGE_FAULT_REASON_NONE:                return "none";
        case VOLTAGE_FAULT_REASON_NOT_READY:           return "not_ready";
        case VOLTAGE_FAULT_REASON_PARTIAL_SCAN:        return "partial_scan";
        case VOLTAGE_FAULT_REASON_STALE_SCAN:          return "stale_scan";
        case VOLTAGE_FAULT_REASON_PEC_FAILURE:         return "pec_failure";
        case VOLTAGE_FAULT_REASON_OPEN_WIRE_RESERVED:  return "open_wire_reserved";
        case VOLTAGE_FAULT_REASON_IMPLAUSIBLE_CELL:    return "implausible_cell";
        case VOLTAGE_FAULT_REASON_OV_WARNING:          return "ov_warning";
        case VOLTAGE_FAULT_REASON_CHARGE_STOP:         return "charge_stop";
        case VOLTAGE_FAULT_REASON_OV_HARD:             return "ov_hard";
        case VOLTAGE_FAULT_REASON_OV_SEVERE:           return "ov_severe";
        case VOLTAGE_FAULT_REASON_UV_WARNING:          return "uv_warning";
        case VOLTAGE_FAULT_REASON_UV_SOFT:             return "uv_soft";
        case VOLTAGE_FAULT_REASON_UV_HARD:             return "uv_hard";
        case VOLTAGE_FAULT_REASON_UV_SEVERE:           return "uv_severe";
        default:                                       return "unknown";
    }
}
