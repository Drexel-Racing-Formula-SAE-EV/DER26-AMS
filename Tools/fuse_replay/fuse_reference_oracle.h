/*
 * Independent high-precision reference for the DER26 main-fuse observer.
 *
 * This module deliberately does not include or call production AMS headers or
 * functions.  It exists to keep the replay/calibration path independent from
 * ams_fuse_observer.c.  The replay adapter copies public configuration values
 * into these standalone types.
 */

#ifndef DER26_FUSE_REFERENCE_ORACLE_H
#define DER26_FUSE_REFERENCE_ORACLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FUSE_REF_HORIZON_COUNT 4u

#define FUSE_REF_REASON_NONE                  0x0000u
#define FUSE_REF_REASON_INPUT_INVALID         0x0001u
#define FUSE_REF_REASON_TEMPERATURE_PROXY     0x0002u
#define FUSE_REF_REASON_MODEL_UNVALIDATED     0x0004u
#define FUSE_REF_REASON_INITIAL_STATE_UNKNOWN 0x0008u
#define FUSE_REF_REASON_BUDGET_DERATED        0x0010u
#define FUSE_REF_REASON_BUDGET_EXHAUSTED      0x0020u

typedef struct
{
    long double rated_current_a;
    long double typical_melting_i2t_a2s;
    long double usable_i2t_fraction;
    long double cooling_time_constant_s;
    long double initialization_soak_s;
    long double quiescent_current_a;
    long double fuse_temperature_margin_c;
    long double minimum_temperature_derating;
    long double maximum_state_multiple;
    long double horizons_s[FUSE_REF_HORIZON_COUNT];
    long double discharge_static_cap_a[FUSE_REF_HORIZON_COUNT];
} fuse_ref_config_t;

typedef struct
{
    long double excess_i2t_a2s;
    long double quiescent_time_s;
    uint64_t update_count;
    uint64_t invalid_count;
    uint8_t thermal_state_initialized;
    uint8_t budget_exhausted;
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
    long double usable_i2t_a2s;
    long double remaining_i2t_a2s;
    long double estimated_fuse_temperature_c;
    long double temperature_derating;
    long double continuous_current_a;
    long double discharge_current_cap_a[FUSE_REF_HORIZON_COUNT];
    uint16_t reason_flags;
    uint8_t valid;
    uint8_t authority_valid;
    uint8_t budget_exhausted;
} fuse_ref_result_t;

bool fuse_ref_config_valid(const fuse_ref_config_t *cfg);
void fuse_ref_state_init(fuse_ref_state_t *state);

/* Seed an already-initialized reference state from a utilization fraction.
 * This is for policy characterization only; it does not change production
 * startup behavior. */
bool fuse_ref_state_seed_utilization(fuse_ref_state_t *state,
                                     const fuse_ref_config_t *cfg,
                                     long double utilization);

long double fuse_ref_temperature_derating(long double fuse_temperature_c,
                                          long double minimum_derating);

/* Exact zero-order-hold solution of
 *   dx/dt = -x/tau + max(0, I_eff^2 - I_cont^2)
 * for one sample interval.  This is intentionally more accurate than the
 * production observer's conservative q*dt injection approximation. */
bool fuse_ref_step_exact_zoh(fuse_ref_state_t *state,
                             const fuse_ref_config_t *cfg,
                             const fuse_ref_input_t *input,
                             fuse_ref_result_t *result);

/* Independent numerical integration helper used to verify the exact oracle
 * itself.  Heun/trapezoidal integration is applied over equal substeps. */
long double fuse_ref_integrate_trapezoidal(long double initial_i2t_a2s,
                                           long double excess_rate_a2,
                                           long double elapsed_s,
                                           long double cooling_tau_s,
                                           uint32_t subdivisions);

#ifdef __cplusplus
}
#endif

#endif /* DER26_FUSE_REFERENCE_ORACLE_H */
