/*
 * Preliminary main-fuse thermal observer for the DER26 accumulator.
 *
 * The model is based on a digitized EAC14-80 typical time-current/I2t curve,
 * not on the single 8020 A^2s value.  The published curve is typical rather
 * than guaranteed-minimum data, so this observer remains subtractive and its
 * authority must stay evidence-gated until Eaton/vehicle characterization is
 * available.
 */

#ifndef INC_SOP_AMS_FUSE_OBSERVER_H_
#define INC_SOP_AMS_FUSE_OBSERVER_H_

#include <stdbool.h>
#include <stdint.h>

#include "sop/ams_sop.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_FUSE_EAC14_80_RATED_CURRENT_A       80.0f

#define AMS_FUSE_REASON_NONE                   0x0000u
#define AMS_FUSE_REASON_INPUT_INVALID          0x0001u
#define AMS_FUSE_REASON_TEMPERATURE_PROXY      0x0002u
#define AMS_FUSE_REASON_MODEL_UNVALIDATED      0x0004u
#define AMS_FUSE_REASON_INITIAL_STATE_UNKNOWN  0x0008u
#define AMS_FUSE_REASON_CURVE_DERATED          0x0010u
#define AMS_FUSE_REASON_BUDGET_EXHAUSTED       0x0020u
#define AMS_FUSE_REASON_CURVE_EXTRAPOLATED     0x0040u

/* Backward-compatible name used by older diagnostics. */
#define AMS_FUSE_REASON_BUDGET_DERATED AMS_FUSE_REASON_CURVE_DERATED

typedef struct
{
    float rated_current_a;

    /* Fraction of the digitized typical melt time made available to the
     * preliminary software model.  This is a commissioning margin, not a
     * substitute for guaranteed-minimum manufacturer data. */
    float curve_time_fraction;

    float cooling_time_constant_s;
    float initialization_soak_s;
    float quiescent_current_a;
    float fuse_temperature_margin_c;
    float minimum_temperature_derating;
    /* Overload-memory headroom above the 1.0 exhaustion threshold.  Keeping
     * severe overload debt (instead of saturating at 1.0) prevents an
     * unrealistically short recovery after a large pulse. */
    float maximum_state_multiple;

    /* Low-current asymptote used below the lowest digitized point:
     * t = scale * (I/In - 1)^(-exponent). */
    float low_current_fit_scale_s;
    float low_current_fit_exponent;
    float maximum_curve_time_s;
    float minimum_curve_time_s;
} ams_fuse_observer_config_t;

typedef struct
{
    /* Dimensionless normalized thermal/damage state.  1.0 is the preliminary
     * curve-derived exhaustion boundary. */
    float thermal_utilization;
    float quiescent_time_s;
    uint32_t update_count;
    uint32_t invalid_count;
    uint8_t thermal_state_initialized;
    uint8_t budget_exhausted;
    /* Set when startup/reset used the authoritative worst-case bound rather
     * than externally established cold state.  Clears only after the bound
     * cools through the 0.50 release threshold. */
    uint8_t initial_state_conservative;
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
    float remaining_utilization;
    float estimated_fuse_temperature_c;
    float temperature_derating;
    float continuous_current_a;
    float effective_current_a;
    float equivalent_25c_current_a;
    float typical_melt_time_s;
    float usable_melt_time_s;
    float discharge_current_cap_a[AMS_SOP_HORIZONS];
    /* Positive magnitudes.  ams_sop_result_t publishes charge current with a
     * negative sign, so the strategy layer applies -charge_current_cap_a. */
    float charge_current_cap_a[AMS_SOP_HORIZONS];
    uint16_t reason_flags;
    uint8_t valid;
    uint8_t authority_valid;
    uint8_t budget_exhausted;
    uint8_t curve_extrapolated;
} ams_fuse_observer_result_t;

void ams_fuse_observer_default_config(ams_fuse_observer_config_t *cfg);
bool ams_fuse_observer_config_valid(const ams_fuse_observer_config_t *cfg);
/* Zero-state initialization is only suitable when a cold fuse is established
 * by external evidence.  Until the configured soak completes, authority is
 * withheld and accumulated utilization is retained. */
void ams_fuse_observer_init(ams_fuse_observer_t *observer);
/* Production/reset initialization for an unknown prior fuse state.  Seeds the
 * configured upper bound, latches exhaustion, and makes the bound immediately
 * usable once the separately gated model is validated. */
bool ams_fuse_observer_init_conservative(
    ams_fuse_observer_t *observer,
    const ams_fuse_observer_config_t *cfg);
float ams_fuse_temperature_derating(float fuse_temperature_c,
                                     float minimum_derating);

/* Typical EAC14-80 melting time at 25 C from the digitized January-2026
 * datasheet curve.  Returns INFINITY at/below rated current.  `extrapolated`
 * is set when the result comes from the low/high-current extension rather than
 * a digitized interval. */
float ams_fuse_typical_melt_time_s(const ams_fuse_observer_config_t *cfg,
                                   float equivalent_25c_current_a,
                                   uint8_t *extrapolated);

bool ams_fuse_observer_update(ams_fuse_observer_t *observer,
                              const ams_fuse_observer_config_t *cfg,
                              const ams_sop_config_t *sop_cfg,
                              const ams_fuse_observer_input_t *input,
                              ams_fuse_observer_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* INC_SOP_AMS_FUSE_OBSERVER_H_ */
