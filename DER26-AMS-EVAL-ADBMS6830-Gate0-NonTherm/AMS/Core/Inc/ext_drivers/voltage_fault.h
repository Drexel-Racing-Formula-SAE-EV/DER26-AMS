/*
 * voltage_fault.h
 * Author: Mahad Faisal (2026)
 *
 * DER26 AMS cell-voltage fault policy.
 *
 * Cell-level checks are the safety source of truth. Pack/segment totals are
 * telemetry only: a single overvoltage cell can be hidden by normal total pack
 * voltage, so do not use pack-total thresholds as backup OV/UV protection.
 */

#ifndef INC_EXT_DRIVERS_VOLTAGE_FAULT_H_
#define INC_EXT_DRIVERS_VOLTAGE_FAULT_H_

#include <stdbool.h>
#include <stdint.h>
#include "ext_drivers/accumulator.h"

#define AMS_EXPECTED_CELL_COUNT          (NSMBS * NCELLS)
#define AMS_SERIES_CELLS                 AMS_EXPECTED_CELL_COUNT
#define AMS_SEGMENT_SERIES_CELLS         NCELLS

#define CELL_OV_WARN_MV                  4150u
#define CELL_CHARGE_STOP_MV              4180u
#define CELL_OV_HARD_MV                  4200u
#define CELL_OV_SEVERE_MV                4250u

#define CELL_UV_WARN_MV                  3000u
#define CELL_UV_SOFT_MV                  2800u
#define CELL_UV_HARD_MV                  2500u
#define CELL_UV_SEVERE_MV                2300u

#define PACK_FULL_TARGET_MV              (CELL_OV_HARD_MV * AMS_SERIES_CELLS)
#define SEGMENT_FULL_TARGET_MV           (CELL_OV_HARD_MV * AMS_SEGMENT_SERIES_CELLS)

/* ADBMS UV at 3.0 V is intentionally treated as early warning/telemetry. */
#define ADBMS_UV_WARN_MV                 CELL_UV_WARN_MV
#define FW_UV_HARD_MV                    CELL_UV_HARD_MV

typedef enum
{
    VOLTAGE_FAULT_REASON_NONE = 0,
    VOLTAGE_FAULT_REASON_NOT_READY,
    VOLTAGE_FAULT_REASON_PARTIAL_SCAN,
    VOLTAGE_FAULT_REASON_STALE_SCAN,
    VOLTAGE_FAULT_REASON_PEC_FAILURE,
    VOLTAGE_FAULT_REASON_OPEN_WIRE_RESERVED,
    VOLTAGE_FAULT_REASON_IMPLAUSIBLE_CELL,

    VOLTAGE_FAULT_REASON_OV_WARNING,
    VOLTAGE_FAULT_REASON_CHARGE_STOP,
    VOLTAGE_FAULT_REASON_OV_HARD,
    VOLTAGE_FAULT_REASON_OV_SEVERE,

    VOLTAGE_FAULT_REASON_UV_WARNING,
    VOLTAGE_FAULT_REASON_UV_SOFT,
    VOLTAGE_FAULT_REASON_UV_HARD,
    VOLTAGE_FAULT_REASON_UV_SEVERE
} voltage_fault_reason_t;

typedef struct
{
    bool voltage_valid;
    bool read_fault;
    bool warning;
    bool charge_stop;
    bool overvoltage_fault;
    bool undervoltage_fault;
    bool confirmed;
    bool latched;

    voltage_fault_reason_t reason;
    voltage_fault_reason_t latched_reason;

    uint16_t usable_cell_count;
    uint16_t updated_cell_count;
    uint16_t stale_cell_count;
    uint16_t pec_fail_cell_count;

    uint16_t max_cell_mv;
    uint16_t min_cell_mv;
    uint8_t max_cell_segment;
    uint8_t max_cell_index;
    uint8_t min_cell_segment;
    uint8_t min_cell_index;
} voltage_fault_state_t;

void voltage_fault_init(voltage_fault_state_t *state);
void voltage_fault_reset_latch(voltage_fault_state_t *state);
void voltage_fault_update(voltage_fault_state_t *state, const accumulator_t *acc);
const char *voltage_fault_reason_str(voltage_fault_reason_t reason);

#endif /* INC_EXT_DRIVERS_VOLTAGE_FAULT_H_ */
