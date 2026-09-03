#ifndef DER26_FUSE_REPLAY_H
#define DER26_FUSE_REPLAY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    FUSE_REPLAY_INIT_COLD_SOAK = 0,
    FUSE_REPLAY_INIT_KNOWN_COLD,
    FUSE_REPLAY_INIT_SEEDED_UTILIZATION
} fuse_replay_init_kind_t;

typedef struct
{
    fuse_replay_init_kind_t kind;
    double utilization;
} fuse_replay_init_policy_t;

typedef enum
{
    FUSE_REPLAY_RESET_UNKNOWN = 0,
    FUSE_REPLAY_RESET_KNOWN_COLD,
    FUSE_REPLAY_RESET_SEEDED_UTILIZATION,
    FUSE_REPLAY_RESET_RESTORE_PRE_RESET
} fuse_replay_reset_kind_t;

typedef struct
{
    const char *trace_path;
    const char *output_path;
    const char *summary_path;
    fuse_replay_init_policy_t startup_policy;
    fuse_replay_reset_kind_t reset_kind;
    double reset_seed_utilization;
    double curve_time_fraction;
    double cooling_time_constant_s;
    double rated_current_a;
    double initialization_soak_s;
    double quiescent_current_a;
    double current_uncertainty_default_a;
    double temperature_default_c;
    double state_abs_tolerance;
    double state_rel_tolerance;
    double cap_tolerance_a;
    bool strict;
} fuse_replay_options_t;

typedef struct
{
    uint64_t samples;
    uint64_t valid_updates;
    uint64_t invalid_updates;
    uint64_t reset_events;
    uint64_t authority_loss_events;
    uint64_t production_underestimate_violations;
    uint64_t cap_nonconservative_violations;
    uint64_t cap_tolerance_violations;
    uint64_t state_tolerance_violations;
    uint64_t result_mismatch_violations;
    uint64_t latch_mismatch_samples;
    uint64_t latch_nonconservative_violations;
    int64_t first_authority_ms;
    int64_t first_reset_ms;
    int64_t first_post_reset_authority_ms;
    int64_t first_post_reset_exhaust_ms;
    int64_t first_production_exhaust_ms;
    int64_t first_reference_exhaust_ms;
    int64_t first_production_recovery_ms;
    int64_t first_reference_recovery_ms;
    double production_utilization_before_reset;
    double production_utilization_after_reset_seed;
    double max_production_utilization;
    double max_reference_utilization;
    double final_production_utilization;
    double final_reference_utilization;
    double max_state_abs_error;
    double max_state_rel_error;
    double max_cap_abs_error_a;
    bool strict_pass;
} fuse_replay_summary_t;

void fuse_replay_default_options(fuse_replay_options_t *options);
const char *fuse_replay_init_policy_name(const fuse_replay_init_policy_t *policy);
const char *fuse_replay_reset_policy_name(fuse_replay_reset_kind_t kind);
bool fuse_replay_run(const fuse_replay_options_t *options,
                     fuse_replay_summary_t *summary);

#ifdef __cplusplus
}
#endif

#endif /* DER26_FUSE_REPLAY_H */
