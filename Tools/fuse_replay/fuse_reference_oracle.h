/*
 * Independent long-double reference for the DER26 EAC14-80 curve observer.
 *
 * This module intentionally includes no production AMS headers and calls no
 * production functions.  Curve points are duplicated from the reviewed source
 * evidence so numerical agreement can be checked without circular execution.
 */
#ifndef DER26_FUSE_REFERENCE_ORACLE_H
#define DER26_FUSE_REFERENCE_ORACLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FUSE_REF_HORIZON_COUNT 4u
#define FUSE_REF_REASON_NONE                   0x0000u
#define FUSE_REF_REASON_INPUT_INVALID          0x0001u
#define FUSE_REF_REASON_TEMPERATURE_PROXY      0x0002u
#define FUSE_REF_REASON_MODEL_UNVALIDATED      0x0004u
#define FUSE_REF_REASON_INITIAL_STATE_UNKNOWN  0x0008u
#define FUSE_REF_REASON_CURVE_DERATED          0x0010u
#define FUSE_REF_REASON_BUDGET_EXHAUSTED       0x0020u
#define FUSE_REF_REASON_CURVE_EXTRAPOLATED     0x0040u

/* Backward name retained for old replay consumers. */
#define FUSE_REF_REASON_BUDGET_DERATED FUSE_REF_REASON_CURVE_DERATED

typedef struct
{
    long double rated_current_a;
    long double curve_time_fraction;
    long double cooling_time_constant_s;
    long double initialization_soak_s;
    long double quiescent_current_a;
    long double fuse_temperature_margin_c;
    long double minimum_temperature_derating;
    long double maximum_state_multiple;
    long double low_current_fit_scale_s;
    long double low_current_fit_exponent;
    long double maximum_curve_time_s;
    long double minimum_curve_time_s;
    long double horizons_s[FUSE_REF_HORIZON_COUNT];
    long double discharge_static_cap_a[FUSE_REF_HORIZON_COUNT];
    long double charge_static_cap_a[FUSE_REF_HORIZON_COUNT];
} fuse_ref_config_t;

typedef struct
{
    long double thermal_utilization;
    long double quiescent_time_s;
    uint64_t update_count;
    uint64_t invalid_count;
    uint8_t thermal_state_initialized;
    uint8_t budget_exhausted;
    uint8_t initial_state_conservative;
} fuse_ref_state_t;

typedef struct
{
    long double pack_current_a;
    long double current_uncertainty_a;
    long double temperature_proxy_c;
    long double elapsed_s;
    uint8_t measurement_valid;
    uint8_t current_calibrated;
    uint8_t current_polarity_validated;
    uint8_t temperature_measured_at_fuse;
    uint8_t model_validated;
} fuse_ref_input_t;

typedef struct
{
    long double utilization;
    long double remaining_utilization;
    long double estimated_fuse_temperature_c;
    long double temperature_derating;
    long double continuous_current_a;
    long double effective_current_a;
    long double equivalent_25c_current_a;
    long double typical_melt_time_s;
    long double usable_melt_time_s;
    long double discharge_current_cap_a[FUSE_REF_HORIZON_COUNT];
    long double charge_current_cap_a[FUSE_REF_HORIZON_COUNT];
    uint16_t reason_flags;
    uint8_t valid;
    uint8_t authority_valid;
    uint8_t budget_exhausted;
    uint8_t curve_extrapolated;
} fuse_ref_result_t;

bool fuse_ref_config_valid(const fuse_ref_config_t *cfg);
void fuse_ref_state_init(fuse_ref_state_t *state);
bool fuse_ref_state_seed_utilization(fuse_ref_state_t *state,
                                     const fuse_ref_config_t *cfg,
                                     long double utilization);
long double fuse_ref_temperature_derating(long double fuse_temperature_c,
                                          long double minimum_derating);
long double fuse_ref_typical_melt_time_s(const fuse_ref_config_t *cfg,
                                         long double equivalent_25c_current_a,
                                         uint8_t *extrapolated);
bool fuse_ref_step_exact_zoh(fuse_ref_state_t *state,
                             const fuse_ref_config_t *cfg,
                             const fuse_ref_input_t *input,
                             fuse_ref_result_t *result);
long double fuse_ref_integrate_trapezoidal(long double initial_utilization,
                                           long double source_rate_per_s,
                                           long double elapsed_s,
                                           long double cooling_tau_s,
                                           uint32_t subdivisions);

#ifdef __cplusplus
}
#endif
#endif
