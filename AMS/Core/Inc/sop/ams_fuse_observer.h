/*
 * Conservative main-fuse thermal observer for the DER26 accumulator.
 *
 * This observer is deliberately subtractive: it may reduce the static SoP
 * current envelope, but it can never raise a configured current limit.  The
 * Eaton EAC14-80 published I2t value is a typical manufacturing value, not a
 * guaranteed clearing boundary, so authority remains gated by vehicle
 * calibration and a known thermal initialization state.
 */

#ifndef INC_SOP_AMS_FUSE_OBSERVER_H_
#define INC_SOP_AMS_FUSE_OBSERVER_H_

#include <stdbool.h>
#include <stdint.h>

#include "sop/ams_sop.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_FUSE_REASON_NONE                 0x0000u
#define AMS_FUSE_REASON_INPUT_INVALID        0x0001u
#define AMS_FUSE_REASON_TEMPERATURE_PROXY    0x0002u
#define AMS_FUSE_REASON_MODEL_UNVALIDATED    0x0004u
#define AMS_FUSE_REASON_INITIAL_STATE_UNKNOWN 0x0008u
#define AMS_FUSE_REASON_BUDGET_DERATED       0x0010u
#define AMS_FUSE_REASON_BUDGET_EXHAUSTED     0x0020u

typedef struct
{
    float rated_current_a;
    float typical_melting_i2t_a2s;
    float usable_i2t_fraction;
    float cooling_time_constant_s;
    float initialization_soak_s;
    float quiescent_current_a;
    float fuse_temperature_margin_c;
    float minimum_temperature_derating;
    float maximum_state_multiple;
} ams_fuse_observer_config_t;

typedef struct
{
    float excess_i2t_a2s;
    float quiescent_time_s;
    uint32_t update_count;
    uint32_t invalid_count;
    uint8_t thermal_state_initialized;
    uint8_t budget_exhausted;
} ams_fuse_observer_t;

typedef struct
{
    float pack_current_a;
    float current_uncertainty_a;
    float temperature_proxy_c;
    float elapsed_s;
    uint8_t measurement_valid;
    uint8_t current_calibrated;
    uint8_t current_polarity_validated;
    uint8_t temperature_measured_at_fuse;
    uint8_t model_validated;
} ams_fuse_observer_input_t;

typedef struct
{
    float utilization;
    float usable_i2t_a2s;
    float remaining_i2t_a2s;
    float estimated_fuse_temperature_c;
    float temperature_derating;
    float continuous_current_a;
    float discharge_current_cap_a[AMS_SOP_HORIZONS];
    uint16_t reason_flags;
    uint8_t valid;
    uint8_t authority_valid;
    uint8_t budget_exhausted;
} ams_fuse_observer_result_t;

void ams_fuse_observer_default_config(ams_fuse_observer_config_t *cfg);
bool ams_fuse_observer_config_valid(const ams_fuse_observer_config_t *cfg);
void ams_fuse_observer_init(ams_fuse_observer_t *observer);
float ams_fuse_temperature_derating(float fuse_temperature_c,
                                     float minimum_derating);
bool ams_fuse_observer_update(ams_fuse_observer_t *observer,
                              const ams_fuse_observer_config_t *cfg,
                              const ams_sop_config_t *sop_cfg,
                              const ams_fuse_observer_input_t *input,
                              ams_fuse_observer_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* INC_SOP_AMS_FUSE_OBSERVER_H_ */
