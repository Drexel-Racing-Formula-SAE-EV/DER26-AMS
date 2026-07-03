/*
 * temperature_fault.h
 *
 * DER26 AMS temperature validity, fault, and fan policy.
 */

#ifndef INC_EXT_DRIVERS_TEMPERATURE_FAULT_H_
#define INC_EXT_DRIVERS_TEMPERATURE_FAULT_H_

#include <stdbool.h>
#include <stdint.h>
#include "ext_drivers/accumulator.h"

#define AMS_EXPECTED_TEMP_SENSOR_COUNT     (NSMBS * NTEMPS)

#define TEMP_FAN_RAMP_START_C              35.0f
#define TEMP_CHARGE_MIN_C                  0.0f
#define TEMP_CHARGE_MAX_C                  45.0f
#define TEMP_FAN_MAX_C                     50.0f
#define TEMP_HOT_WARNING_C                 50.0f
#define TEMP_HOT_HARD_C                    60.0f
#define TEMP_HOT_SEVERE_C                  65.0f
#define TEMP_HOT_HARD_CONFIRM_MS           2000u
#define TEMP_HOT_SEVERE_CONFIRM_MS         2000u
#define TEMP_FAULT_DEFAULT_SAMPLE_MS       1000u

typedef enum
{
    TEMPERATURE_FAULT_REASON_NONE = 0,
    TEMPERATURE_FAULT_REASON_NOT_READY,
    TEMPERATURE_FAULT_REASON_PARTIAL_SCAN,
    TEMPERATURE_FAULT_REASON_STALE_SCAN,
    TEMPERATURE_FAULT_REASON_INVALID_SENSOR,
    TEMPERATURE_FAULT_REASON_COLD_CHARGE_STOP,
    TEMPERATURE_FAULT_REASON_HOT_CHARGE_STOP,
    TEMPERATURE_FAULT_REASON_HOT_WARNING,
    TEMPERATURE_FAULT_REASON_HOT_FAN_MAX,
    TEMPERATURE_FAULT_REASON_HOT_HARD,
    TEMPERATURE_FAULT_REASON_HOT_SEVERE,
    TEMPERATURE_FAULT_REASON_COLD_WARNING
} temperature_fault_reason_t;

typedef struct
{
    bool temp_valid;
    bool read_fault;
    bool warning;
    bool fan_max;
    bool charge_stop;
    bool pending;
    bool overtemp_fault;
    bool severe_overtemp_fault;
    bool confirmed;
    bool latched;

    temperature_fault_reason_t reason;
    temperature_fault_reason_t pending_reason;
    temperature_fault_reason_t latched_reason;
    uint32_t pending_ms;
    int16_t threshold_deci_c;

    uint16_t usable_sensor_count;
    uint16_t updated_sensor_count;
    uint16_t stale_sensor_count;
    uint16_t invalid_sensor_count;

    int16_t max_temp_deci_c;
    int16_t min_temp_deci_c;
    uint8_t max_temp_segment;
    uint8_t max_temp_sensor;
    uint8_t min_temp_segment;
    uint8_t min_temp_sensor;
} temperature_fault_state_t;

void temperature_fault_init(temperature_fault_state_t *state);
void temperature_fault_reset_latch(temperature_fault_state_t *state);
void temperature_fault_update(temperature_fault_state_t *state, const accumulator_t *acc);
void temperature_fault_update_with_period(temperature_fault_state_t *state,
                                          const accumulator_t *acc,
                                          uint32_t sample_period_ms);
const char *temperature_fault_reason_str(temperature_fault_reason_t reason);

#endif /* INC_EXT_DRIVERS_TEMPERATURE_FAULT_H_ */
