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
#include <string.h>

static void clear_non_latched(voltage_fault_state_t *state)
{
    state->voltage_valid = false;
    state->read_fault = false;
    state->read_fault_pending = false;
    state->warning = false;
    state->charge_stop = false;
    state->overvoltage_fault = false;
    state->undervoltage_fault = false;
    state->confirmed = false;
    state->hardware_warning = false;
    state->hardware_disagreement = false;
    state->hardware_status_valid_ic_mask = 0u;
    memset(state->software_ov_mask, 0, sizeof(state->software_ov_mask));
    memset(state->software_uv_mask, 0, sizeof(state->software_uv_mask));
    memset(state->hardware_ov_mask, 0, sizeof(state->hardware_ov_mask));
    memset(state->hardware_uv_mask, 0, sizeof(state->hardware_uv_mask));
    memset(state->hardware_disagreement_mask, 0,
           sizeof(state->hardware_disagreement_mask));
    state->reason = VOLTAGE_FAULT_REASON_NONE;
}

static void set_latched(voltage_fault_state_t *state, voltage_fault_reason_t reason)
{
    state->confirmed = true;
    state->latched = true;
    state->latched_reason = reason;
    state->reason = reason;
}

static void set_read_scan_fault(voltage_fault_state_t *state,
                                voltage_fault_reason_t reason,
                                bool had_previous_valid_scan)
{
    if(state->read_fault_streak != UINT8_MAX)
    {
        state->read_fault_streak++;
    }

    state->read_fault = true;
    state->reason = reason;

    if(had_previous_valid_scan &&
       (state->read_fault_streak < VOLTAGE_READ_FAULT_CONFIRM_SCANS))
    {
        /* Grace applies only to transport/read availability. The current
         * measurement remains invalid (read_fault=true), so estimator/SoP/CAN
         * torque authority fail zero immediately while contactors are spared
         * from opening on one isolated bad frame. */
        state->voltage_valid = true;
        state->read_fault_pending = true;
        state->warning = true;
        state->charge_stop = true;
        state->confirmed = false;
        return;
    }

    state->voltage_valid = false;
    state->read_fault_pending = false;
    state->confirmed = true;
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

    bool had_previous_valid_scan = state->voltage_valid;
    clear_non_latched(state);

    if(acc == NULL)
    {
        state->read_fault_streak = 0u;
        state->read_fault = true;
        state->confirmed = true;
        state->reason = VOLTAGE_FAULT_REASON_NOT_READY;
        return;
    }

    const bool read_grace_eligible = had_previous_valid_scan &&
        (acc->smb.ics != NULL) &&
        (acc->smb.num_ics > 0) &&
        (acc->smb.num_ics <= (int)NSMBS) &&
        (acc->smb.num_ics <= (int)ADBMS6830_MAX_TRACKED_ICS) &&
        (acc->smb.monitored_cell_count > 0u) &&
        (acc->smb.monitored_cell_count <= NCELLS) &&
        (((uint16_t)acc->smb.num_ics *
          (uint16_t)acc->smb.monitored_cell_count) ==
         (uint16_t)AMS_EXPECTED_CELL_COUNT);

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
        state->read_fault_streak = 0u;
        state->read_fault = true;
        state->confirmed = true;
        state->reason = VOLTAGE_FAULT_REASON_NOT_READY;
        return;
    }

    if(!acc->voltage_full_updated)
    {
        voltage_fault_reason_t reason = VOLTAGE_FAULT_REASON_PARTIAL_SCAN;
        if(acc->pec_fail_cell_count > 0u)
        {
            reason = VOLTAGE_FAULT_REASON_PEC_FAILURE;
        }
        else if(acc->stale_voltage_count > 0u)
        {
            reason = VOLTAGE_FAULT_REASON_STALE_SCAN;
        }
        set_read_scan_fault(state, reason, read_grace_eligible);
        return;
    }

    if(!acc->voltage_full_usable)
    {
        set_read_scan_fault(state,
                            VOLTAGE_FAULT_REASON_STALE_SCAN,
                            read_grace_eligible);
        return;
    }

    /* Never walk cell/status arrays using a corrupted topology field merely
     * because a stale aggregate full_usable bit remained set. */
    if((acc->smb.num_ics <= 0) ||
       (acc->smb.num_ics > (int)NSMBS) ||
       (acc->smb.num_ics > (int)ADBMS6830_MAX_TRACKED_ICS) ||
       (acc->smb.monitored_cell_count == 0u) ||
       (acc->smb.monitored_cell_count > NCELLS) ||
       (acc->usable_voltage_count !=
        ((uint16_t)acc->smb.num_ics *
         (uint16_t)acc->smb.monitored_cell_count)) ||
       (acc->updated_voltage_count !=
        ((uint16_t)acc->smb.num_ics *
         (uint16_t)acc->smb.monitored_cell_count)))
    {
        state->read_fault_streak = 0u;
        state->read_fault = true;
        state->confirmed = true;
        state->reason = VOLTAGE_FAULT_REASON_NOT_READY;
        return;
    }

    state->read_fault_streak = 0u;
    state->voltage_valid = true;

    /* Keep the ADBMS Status-D comparator image separate from the generic
     * silicon-diagnostic fault class. OV/UV is an expected operating
     * condition owned by this voltage policy, not evidence of a bad SPI or
     * monitor. C16 and every other unpopulated channel are masked. */
    {
        const adbms6830_diag_health_t *health =
            adbms6830_diag_health_get(&acc->smb);
        uint8_t ic_count = 0u;
        uint16_t cell_mask = (acc->smb.monitored_cell_count >= 16u) ?
                             UINT16_MAX :
                             (uint16_t)((1u << acc->smb.monitored_cell_count) - 1u);

        if((acc->smb.num_ics > 0) &&
           (acc->smb.num_ics <= (int)NSMBS) &&
           (acc->smb.num_ics <= (int)ADBMS6830_MAX_TRACKED_ICS))
        {
            ic_count = (uint8_t)acc->smb.num_ics;
        }

        for(uint8_t seg = 0u; seg < ic_count; seg++)
        {
            uint16_t definite_ov = 0u;
            uint16_t definite_not_ov = 0u;
            uint16_t definite_uv = 0u;
            uint16_t definite_not_uv = 0u;
            uint16_t bit_ic = (uint16_t)(1u << seg);

            for(uint8_t cell = 0u; cell < acc->smb.monitored_cell_count; cell++)
            {
                uint16_t bit = (uint16_t)(1u << cell);
                if((acc->usable_voltage_mask[seg] & bit) == 0u)
                {
                    continue;
                }
                if(acc->cell_voltage_mv[seg][cell] >= CELL_OV_HARD_MV)
                {
                    state->software_ov_mask[seg] |= bit;
                }
                if(acc->cell_voltage_mv[seg][cell] <= ADBMS_UV_WARN_MV)
                {
                    state->software_uv_mask[seg] |= bit;
                }

                if(acc->cell_voltage_mv[seg][cell] >=
                   (CELL_OV_HARD_MV + ADBMS_OVUV_COMPARE_MARGIN_MV))
                {
                    definite_ov |= bit;
                }
                else if(acc->cell_voltage_mv[seg][cell] <=
                        (CELL_OV_HARD_MV - ADBMS_OVUV_COMPARE_MARGIN_MV))
                {
                    definite_not_ov |= bit;
                }

                if(acc->cell_voltage_mv[seg][cell] <=
                   (ADBMS_UV_WARN_MV - ADBMS_OVUV_COMPARE_MARGIN_MV))
                {
                    definite_uv |= bit;
                }
                else if(acc->cell_voltage_mv[seg][cell] >=
                        (ADBMS_UV_WARN_MV + ADBMS_OVUV_COMPARE_MARGIN_MV))
                {
                    definite_not_uv |= bit;
                }
            }

            if((health != NULL) &&
               ((health->status_invalid_ic_mask & bit_ic) == 0u) &&
               acc->smb.diag[seg].statd_valid)
            {
                state->hardware_status_valid_ic_mask |= bit_ic;
                state->hardware_ov_mask[seg] =
                    (uint16_t)(acc->smb.diag[seg].cell_ov_mask & cell_mask);
                state->hardware_uv_mask[seg] =
                    (uint16_t)(acc->smb.diag[seg].cell_uv_mask & cell_mask);
                state->hardware_disagreement_mask[seg] =
                    (uint16_t)((definite_ov &
                                (uint16_t)~state->hardware_ov_mask[seg]) |
                               (definite_not_ov &
                                state->hardware_ov_mask[seg]) |
                               (definite_uv &
                                (uint16_t)~state->hardware_uv_mask[seg]) |
                               (definite_not_uv &
                                state->hardware_uv_mask[seg]));
                if(state->hardware_disagreement_mask[seg] != 0u)
                {
                    state->hardware_disagreement = true;
                }
                if((state->hardware_ov_mask[seg] |
                    state->hardware_uv_mask[seg]) != 0u)
                {
                    state->hardware_warning = true;
                }
            }
        }
    }

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

    if(state->hardware_disagreement)
    {
        state->warning = true;
        state->reason = VOLTAGE_FAULT_REASON_HW_STATUS_DISAGREEMENT;
        return;
    }

    if(state->hardware_warning)
    {
        state->warning = true;
        state->reason = VOLTAGE_FAULT_REASON_HW_STATUS_WARNING;
        return;
    }
    state->reason = VOLTAGE_FAULT_REASON_NONE;
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
        case VOLTAGE_FAULT_REASON_HW_STATUS_DISAGREEMENT: return "hw_status_disagreement";
        case VOLTAGE_FAULT_REASON_HW_STATUS_WARNING:   return "hw_status_warning";
        default:                                       return "unknown";
    }
}
