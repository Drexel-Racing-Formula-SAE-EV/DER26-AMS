/*
 * current_fault.h
 *
 * Current measurement/fault policy for DER26 AMS.
 *
 * The measurement layer lives in current_sensor.c. This file only classifies a
 * validated pack current into warning/pending/confirmed/latched fault states.
 */

#ifndef INC_CURRENT_FAULT_H_
#define INC_CURRENT_FAULT_H_

#include <stdbool.h>
#include <stdint.h>
#include "ext_drivers/current_sensor.h"

typedef enum
{
    CURRENT_FAULT_MODE_IDLE = 0,
    CURRENT_FAULT_MODE_DRIVE,
    CURRENT_FAULT_MODE_CHARGE,
    CURRENT_FAULT_MODE_PRECHARGE
} current_fault_mode_t;

typedef enum
{
    CURRENT_FAULT_REASON_NONE = 0,

    CURRENT_FAULT_REASON_SENSOR_NOT_READY,
    CURRENT_FAULT_REASON_SENSOR_ADC_READ,
    CURRENT_FAULT_REASON_SENSOR_IMPLAUSIBLE,
    CURRENT_FAULT_REASON_SENSOR_SATURATION,
    CURRENT_FAULT_REASON_SENSOR_CHANNEL_MISMATCH,

    CURRENT_FAULT_REASON_DISCHARGE_WARNING,
    CURRENT_FAULT_REASON_DISCHARGE_OVERCURRENT,
    CURRENT_FAULT_REASON_DISCHARGE_FAST_OVERCURRENT,
    CURRENT_FAULT_REASON_DISCHARGE_EXTREME,

    CURRENT_FAULT_REASON_CHARGE_WARNING,
    CURRENT_FAULT_REASON_CHARGE_OVERCURRENT,
    CURRENT_FAULT_REASON_CHARGE_FAST_OVERCURRENT,
    CURRENT_FAULT_REASON_CHARGE_EXTREME,

    CURRENT_FAULT_REASON_REGEN_WARNING,
    CURRENT_FAULT_REASON_REGEN_OVERCURRENT,
    CURRENT_FAULT_REASON_REGEN_FAST_OVERCURRENT,
    CURRENT_FAULT_REASON_REGEN_EXTREME,
    CURRENT_FAULT_REASON_REGEN_UNEXPECTED,

    CURRENT_FAULT_REASON_PRECHARGE_WARNING,
    CURRENT_FAULT_REASON_PRECHARGE_OVERCURRENT,
    CURRENT_FAULT_REASON_PRECHARGE_FAST_OVERCURRENT
} current_fault_reason_t;

typedef struct
{
    bool sensor_fault;
    bool warning;
    bool pending;
    bool confirmed;
    bool latched;

    current_fault_reason_t reason;
    current_fault_reason_t pending_reason;
    current_fault_reason_t latched_reason;
    current_fault_mode_t mode;

    uint32_t sensor_invalid_ms;
    uint32_t pending_ms;

    float abs_current_a;
    float threshold_a;
} current_fault_state_t;

void current_fault_init(current_fault_state_t *state);
void current_fault_reset_latch(current_fault_state_t *state);
void current_fault_update(current_fault_state_t *state,
                          current_fault_mode_t mode,
                          float current_a,
                          bool current_valid,
                          current_sensor_reason_t measurement_reason,
                          uint32_t sample_period_ms);
const char *current_fault_reason_str(current_fault_reason_t reason);
const char *current_fault_mode_str(current_fault_mode_t mode);

#endif /* INC_CURRENT_FAULT_H_ */
