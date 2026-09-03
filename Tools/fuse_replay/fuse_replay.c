#define _POSIX_C_SOURCE 200809L

#include "fuse_replay.h"
#include "fuse_reference_oracle.h"

#include "sop/ams_fuse_observer.h"
#include "sop/ams_sop.h"

#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 2048
#define MAX_COLUMNS 24
#define NO_TIME INT64_C(-1)

typedef struct
{
    int timestamp_ms;
    int time_s;
    int current_a;
    int uncertainty_a;
    int temperature_c;
    int measurement_valid;
    int current_calibrated;
    int polarity_validated;
    int temperature_measured;
    int model_validated;
    int event;
    int count;
} csv_columns_t;

typedef struct
{
    uint64_t timestamp_ms;
    double current_a;
    double uncertainty_a;
    double temperature_c;
    uint8_t measurement_valid;
    uint8_t current_calibrated;
    uint8_t polarity_validated;
    uint8_t temperature_measured;
    uint8_t model_validated;
    bool reset_event;
} trace_row_t;

static char *trim(char *text)
{
    while((*text != '\0') && isspace((unsigned char)*text))
    {
        ++text;
    }
    char *end = text + strlen(text);
    while((end > text) && isspace((unsigned char)end[-1]))
    {
        --end;
    }
    *end = '\0';
    return text;
}

static void normalize_name(char *text)
{
    for(char *p = text; *p != '\0'; ++p)
    {
        if((*p == ' ') || (*p == '-') || (*p == '/'))
        {
            *p = '_';
        }
        else
        {
            *p = (char)tolower((unsigned char)*p);
        }
    }
}

static int split_csv(char *line, char **fields, int max_fields)
{
    int count = 0;
    char *cursor = line;
    while((count < max_fields) && (cursor != NULL))
    {
        fields[count++] = trim(cursor);
        char *comma = strchr(cursor, ',');
        if(comma == NULL)
        {
            break;
        }
        *comma = '\0';
        cursor = comma + 1;
    }
    return count;
}

static bool parse_double_str(const char *text, double *value)
{
    if((text == NULL) || (value == NULL) || (*text == '\0'))
    {
        return false;
    }
    errno = 0;
    char *end = NULL;
    const double parsed = strtod(text, &end);
    if((errno != 0) || (end == text) || (*trim(end) != '\0'))
    {
        return false;
    }
    *value = parsed;
    return true;
}

static bool parse_u8_str(const char *text, uint8_t *value)
{
    double parsed;
    if(!parse_double_str(text, &parsed) || !isfinite(parsed) ||
       (parsed < 0.0) || (parsed > 1.0))
    {
        return false;
    }
    *value = (parsed >= 0.5) ? 1u : 0u;
    return true;
}

static void columns_init(csv_columns_t *columns)
{
    memset(columns, -1, sizeof(*columns));
    columns->count = 0;
}

static bool parse_header(char *line, csv_columns_t *columns)
{
    char *fields[MAX_COLUMNS];
    const int count = split_csv(line, fields, MAX_COLUMNS);
    if(count <= 0)
    {
        return false;
    }
    columns_init(columns);
    columns->count = count;

    for(int i = 0; i < count; ++i)
    {
        normalize_name(fields[i]);
        const char *name = fields[i];
        if((strcmp(name, "timestamp_ms") == 0) ||
           (strcmp(name, "time_ms") == 0))
        {
            columns->timestamp_ms = i;
        }
        else if((strcmp(name, "time_s") == 0) ||
                (strcmp(name, "timestamp_s") == 0))
        {
            columns->time_s = i;
        }
        else if((strcmp(name, "current_a") == 0) ||
                (strcmp(name, "pack_current_a") == 0))
        {
            columns->current_a = i;
        }
        else if((strcmp(name, "current_uncertainty_a") == 0) ||
                (strcmp(name, "uncertainty_a") == 0))
        {
            columns->uncertainty_a = i;
        }
        else if((strcmp(name, "temperature_proxy_c") == 0) ||
                (strcmp(name, "ambient_temp_c") == 0) ||
                (strcmp(name, "temperature_c") == 0) ||
                (strcmp(name, "temp_c") == 0))
        {
            columns->temperature_c = i;
        }
        else if(strcmp(name, "measurement_valid") == 0)
        {
            columns->measurement_valid = i;
        }
        else if(strcmp(name, "current_calibrated") == 0)
        {
            columns->current_calibrated = i;
        }
        else if((strcmp(name, "current_polarity_validated") == 0) ||
                (strcmp(name, "polarity_validated") == 0))
        {
            columns->polarity_validated = i;
        }
        else if((strcmp(name, "temperature_measured_at_fuse") == 0) ||
                (strcmp(name, "temperature_measured") == 0))
        {
            columns->temperature_measured = i;
        }
        else if(strcmp(name, "model_validated") == 0)
        {
            columns->model_validated = i;
        }
        else if(strcmp(name, "event") == 0)
        {
            columns->event = i;
        }
    }

    return ((columns->timestamp_ms >= 0) || (columns->time_s >= 0)) &&
           (columns->current_a >= 0);
}

static bool field_present(int index, int count)
{
    return (index >= 0) && (index < count);
}

static bool parse_row(char *line,
                      const csv_columns_t *columns,
                      const fuse_replay_options_t *options,
                      trace_row_t *row)
{
    char *fields[MAX_COLUMNS];
    const int count = split_csv(line, fields, MAX_COLUMNS);
    if((count <= 0) || (columns == NULL) || (row == NULL))
    {
        return false;
    }

    memset(row, 0, sizeof(*row));
    row->uncertainty_a = options->current_uncertainty_default_a;
    row->temperature_c = options->temperature_default_c;
    row->measurement_valid = 1u;
    row->current_calibrated = 1u;
    row->polarity_validated = 1u;
    row->model_validated = 1u;

    double timestamp;
    if(field_present(columns->timestamp_ms, count))
    {
        if(!parse_double_str(fields[columns->timestamp_ms], &timestamp) ||
           !isfinite(timestamp) || (timestamp < 0.0) ||
           (timestamp > (double)UINT64_MAX))
        {
            return false;
        }
        row->timestamp_ms = (uint64_t)llround(timestamp);
    }
    else if(field_present(columns->time_s, count))
    {
        if(!parse_double_str(fields[columns->time_s], &timestamp) ||
           !isfinite(timestamp) || (timestamp < 0.0) ||
           (timestamp > ((double)UINT64_MAX / 1000.0)))
        {
            return false;
        }
        row->timestamp_ms = (uint64_t)llround(timestamp * 1000.0);
    }
    else
    {
        return false;
    }

    if(!field_present(columns->current_a, count) ||
       !parse_double_str(fields[columns->current_a], &row->current_a))
    {
        return false;
    }

    if(field_present(columns->uncertainty_a, count) &&
       (*fields[columns->uncertainty_a] != '\0') &&
       !parse_double_str(fields[columns->uncertainty_a],
                         &row->uncertainty_a))
    {
        return false;
    }
    if(field_present(columns->temperature_c, count) &&
       (*fields[columns->temperature_c] != '\0') &&
       !parse_double_str(fields[columns->temperature_c],
                         &row->temperature_c))
    {
        return false;
    }

#define PARSE_FLAG(member, column) \
    do { \
        if(field_present((column), count) && (*fields[(column)] != '\0') && \
           !parse_u8_str(fields[(column)], &row->member)) \
        { \
            return false; \
        } \
    } while(0)

    PARSE_FLAG(measurement_valid, columns->measurement_valid);
    PARSE_FLAG(current_calibrated, columns->current_calibrated);
    PARSE_FLAG(polarity_validated, columns->polarity_validated);
    PARSE_FLAG(temperature_measured, columns->temperature_measured);
    PARSE_FLAG(model_validated, columns->model_validated);
#undef PARSE_FLAG

    if(field_present(columns->event, count))
    {
        char *event = trim(fields[columns->event]);
        normalize_name(event);
        row->reset_event = (strcmp(event, "reset") == 0) ||
                           (strcmp(event, "mcu_reset") == 0);
    }
    return true;
}

static void map_ref_config(const ams_fuse_observer_config_t *prod,
                           const ams_sop_config_t *sop,
                           fuse_ref_config_t *ref)
{
    memset(ref, 0, sizeof(*ref));
    ref->rated_current_a = prod->rated_current_a;
    ref->curve_time_fraction = prod->curve_time_fraction;
    ref->cooling_time_constant_s = prod->cooling_time_constant_s;
    ref->initialization_soak_s = prod->initialization_soak_s;
    ref->quiescent_current_a = prod->quiescent_current_a;
    ref->fuse_temperature_margin_c = prod->fuse_temperature_margin_c;
    ref->minimum_temperature_derating = prod->minimum_temperature_derating;
    ref->maximum_state_multiple = prod->maximum_state_multiple;
    ref->low_current_fit_scale_s = prod->low_current_fit_scale_s;
    ref->low_current_fit_exponent = prod->low_current_fit_exponent;
    ref->maximum_curve_time_s = prod->maximum_curve_time_s;
    ref->minimum_curve_time_s = prod->minimum_curve_time_s;
    for(uint32_t h = 0u; h < FUSE_REF_HORIZON_COUNT; ++h)
    {
        ref->horizons_s[h] = sop->horizons_s[h];
        ref->discharge_static_cap_a[h] = sop->discharge_current_max_a[h];
    }
}

static void map_ref_input(const ams_fuse_observer_input_t *prod,
                          fuse_ref_input_t *ref)
{
    memset(ref, 0, sizeof(*ref));
    ref->pack_current_a = prod->pack_current_a;
    ref->current_uncertainty_a = prod->current_uncertainty_a;
    ref->temperature_proxy_c = prod->temperature_proxy_c;
    ref->elapsed_s = prod->elapsed_s;
    ref->measurement_valid = prod->measurement_valid;
    ref->current_calibrated = prod->current_calibrated;
    ref->current_polarity_validated = prod->current_polarity_validated;
    ref->temperature_measured_at_fuse = prod->temperature_measured_at_fuse;
    ref->model_validated = prod->model_validated;
}

static bool seed_states(ams_fuse_observer_t *prod,
                        fuse_ref_state_t *ref,
                        const ams_fuse_observer_config_t *pcfg,
                        const fuse_ref_config_t *rcfg,
                        const fuse_replay_init_policy_t *policy)
{
    ams_fuse_observer_init(prod);
    fuse_ref_state_init(ref);
    if(policy->kind == FUSE_REPLAY_INIT_COLD_SOAK)
    {
        return true;
    }

    const double util = (policy->kind == FUSE_REPLAY_INIT_KNOWN_COLD) ?
                        0.0 : policy->utilization;
    if(!isfinite(util) || (util < 0.0) ||
       (util > (double)pcfg->maximum_state_multiple))
    {
        return false;
    }
    prod->thermal_utilization = (float)util;
    prod->quiescent_time_s = pcfg->initialization_soak_s;
    prod->thermal_state_initialized = 1u;
    prod->budget_exhausted = (util >= 1.0) ? 1u : 0u;
    return fuse_ref_state_seed_utilization(ref, rcfg, (long double)util);
}

static bool apply_reset(ams_fuse_observer_t *prod,
                        fuse_ref_state_t *ref,
                        const ams_fuse_observer_config_t *pcfg,
                        const fuse_ref_config_t *rcfg,
                        const fuse_replay_options_t *options)
{
    if(options->reset_kind == FUSE_REPLAY_RESET_RESTORE_PRE_RESET)
    {
        const double prod_util = (double)prod->thermal_utilization;
        const long double ref_util = ref->thermal_utilization;
        const uint8_t prod_exhausted = prod->budget_exhausted;
        const uint8_t ref_exhausted = ref->budget_exhausted;

        ams_fuse_observer_init(prod);
        fuse_ref_state_init(ref);
        prod->thermal_utilization = (float)prod_util;
        prod->quiescent_time_s = pcfg->initialization_soak_s;
        prod->thermal_state_initialized = 1u;
        prod->budget_exhausted = prod_exhausted;
        if(!fuse_ref_state_seed_utilization(ref, rcfg, ref_util))
        {
            return false;
        }
        ref->budget_exhausted = ref_exhausted;
        return true;
    }

    fuse_replay_init_policy_t policy;
    if(options->reset_kind == FUSE_REPLAY_RESET_UNKNOWN)
    {
        policy.kind = FUSE_REPLAY_INIT_COLD_SOAK;
        policy.utilization = 0.0;
    }
    else if(options->reset_kind == FUSE_REPLAY_RESET_KNOWN_COLD)
    {
        policy.kind = FUSE_REPLAY_INIT_KNOWN_COLD;
        policy.utilization = 0.0;
    }
    else
    {
        policy.kind = FUSE_REPLAY_INIT_SEEDED_UTILIZATION;
        policy.utilization = options->reset_seed_utilization;
    }
    return seed_states(prod, ref, pcfg, rcfg, &policy);
}

void fuse_replay_default_options(fuse_replay_options_t *options)
{
    if(options == NULL)
    {
        return;
    }
    memset(options, 0, sizeof(*options));
    options->startup_policy.kind = FUSE_REPLAY_INIT_COLD_SOAK;
    options->reset_kind = FUSE_REPLAY_RESET_UNKNOWN;
    options->reset_seed_utilization = 0.80;
    options->curve_time_fraction = 0.25;
    options->cooling_time_constant_s = 300.0;
    options->rated_current_a = 80.0;
    options->initialization_soak_s = 300.0;
    options->quiescent_current_a = 5.0;
    options->current_uncertainty_default_a = 0.5;
    options->temperature_default_c = 30.0;
    options->state_abs_tolerance = 2.0e-5;
    options->state_rel_tolerance = 0.002;
    options->cap_tolerance_a = 0.20;
}

const char *fuse_replay_init_policy_name(const fuse_replay_init_policy_t *policy)
{
    if(policy == NULL)
    {
        return "invalid";
    }
    switch(policy->kind)
    {
        case FUSE_REPLAY_INIT_COLD_SOAK: return "cold-soak";
        case FUSE_REPLAY_INIT_KNOWN_COLD: return "known-cold";
        case FUSE_REPLAY_INIT_SEEDED_UTILIZATION: return "seeded";
        default: return "invalid";
    }
}

const char *fuse_replay_reset_policy_name(fuse_replay_reset_kind_t kind)
{
    switch(kind)
    {
        case FUSE_REPLAY_RESET_UNKNOWN: return "unknown";
        case FUSE_REPLAY_RESET_KNOWN_COLD: return "known-cold";
        case FUSE_REPLAY_RESET_SEEDED_UTILIZATION: return "seeded";
        case FUSE_REPLAY_RESET_RESTORE_PRE_RESET: return "restore";
        default: return "invalid";
    }
}

static void write_output_header(FILE *output)
{
    fprintf(output,
        "timestamp_ms,current_a,elapsed_s,event,"
        "prod_initialized,ref_initialized,prod_authority,ref_authority,"
        "prod_state_utilization,ref_state_utilization,state_abs_error,"
        "prod_utilization,ref_utilization,prod_remaining_utilization,"
        "ref_remaining_utilization,prod_exhausted,ref_exhausted,"
        "prod_cap_0p1s_a,ref_cap_0p1s_a,prod_cap_1s_a,ref_cap_1s_a,"
        "prod_cap_10s_a,ref_cap_10s_a,prod_cap_30s_a,ref_cap_30s_a,"
        "prod_reason_flags,ref_reason_flags\n");
}

static void write_output_row(FILE *output,
                             const trace_row_t *row,
                             double elapsed_s,
                             const ams_fuse_observer_t *pstate,
                             const fuse_ref_state_t *rstate,
                             const ams_fuse_observer_result_t *pout,
                             const fuse_ref_result_t *rout)
{
    const long double state_error = fabsl(
        (long double)pstate->thermal_utilization - rstate->thermal_utilization);
    fprintf(output,
        "%" PRIu64 ",%.9g,%.9g,%s,%u,%u,%u,%u,"
        "%.9g,%.12Lg,%.12Lg,%.9g,%.12Lg,%.9g,%.12Lg,%u,%u,"
        "%.9g,%.12Lg,%.9g,%.12Lg,%.9g,%.12Lg,%.9g,%.12Lg,0x%04x,0x%04x\n",
        row->timestamp_ms,
        row->current_a,
        elapsed_s,
        row->reset_event ? "reset" : "",
        (unsigned)pstate->thermal_state_initialized,
        (unsigned)rstate->thermal_state_initialized,
        (unsigned)pout->authority_valid,
        (unsigned)rout->authority_valid,
        (double)pstate->thermal_utilization,
        rstate->thermal_utilization,
        state_error,
        (double)pout->utilization,
        rout->utilization,
        (double)pout->remaining_utilization,
        rout->remaining_utilization,
        (unsigned)pout->budget_exhausted,
        (unsigned)rout->budget_exhausted,
        (double)pout->discharge_current_cap_a[0],
        rout->discharge_current_cap_a[0],
        (double)pout->discharge_current_cap_a[1],
        rout->discharge_current_cap_a[1],
        (double)pout->discharge_current_cap_a[2],
        rout->discharge_current_cap_a[2],
        (double)pout->discharge_current_cap_a[3],
        rout->discharge_current_cap_a[3],
        (unsigned)pout->reason_flags,
        (unsigned)rout->reason_flags);
}

static void update_time_metric(uint64_t timestamp_ms,
                               bool condition,
                               int64_t *metric)
{
    if(condition && (*metric == NO_TIME))
    {
        if(timestamp_ms <= (uint64_t)INT64_MAX)
        {
            *metric = (int64_t)timestamp_ms;
        }
    }
}

static bool write_summary_csv(const fuse_replay_options_t *options,
                              const fuse_replay_summary_t *summary)
{
    if(options->summary_path == NULL)
    {
        return true;
    }
    FILE *file = fopen(options->summary_path, "w");
    if(file == NULL)
    {
        fprintf(stderr, "ERROR: cannot open summary %s: %s\n",
                options->summary_path, strerror(errno));
        return false;
    }

    fprintf(file,
        "trace,startup_policy,startup_utilization,reset_policy,reset_utilization,"
        "curve_time_fraction,cooling_tau_s,rated_current_a,"
        "samples,valid_updates,invalid_updates,reset_events,authority_loss_events,"
        "first_authority_ms,first_reset_ms,first_post_reset_authority_ms,first_post_reset_exhaust_ms,first_prod_exhaust_ms,first_ref_exhaust_ms,"
        "first_prod_recovery_ms,first_ref_recovery_ms,"
        "prod_utilization_before_reset,prod_utilization_after_reset_seed,max_prod_utilization,max_ref_utilization,final_prod_utilization,"
        "final_ref_utilization,max_state_abs_error,max_state_rel_error,"
        "max_cap_abs_error_a,production_underestimate_violations,"
        "cap_nonconservative_violations,cap_tolerance_violations,state_tolerance_violations,"
        "result_mismatch_violations,latch_mismatch_samples,latch_nonconservative_violations,strict_pass\n");
    fprintf(file,
        "%s,%s,%.9g,%s,%.9g,%.9g,%.9g,%.9g,"
        "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
        "%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ","
        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
        "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%u\n",
        options->trace_path,
        fuse_replay_init_policy_name(&options->startup_policy),
        options->startup_policy.utilization,
        fuse_replay_reset_policy_name(options->reset_kind),
        options->reset_seed_utilization,
        options->curve_time_fraction,
        options->cooling_time_constant_s,
        options->rated_current_a,
        summary->samples,
        summary->valid_updates,
        summary->invalid_updates,
        summary->reset_events,
        summary->authority_loss_events,
        summary->first_authority_ms,
        summary->first_reset_ms,
        summary->first_post_reset_authority_ms,
        summary->first_post_reset_exhaust_ms,
        summary->first_production_exhaust_ms,
        summary->first_reference_exhaust_ms,
        summary->first_production_recovery_ms,
        summary->first_reference_recovery_ms,
        summary->production_utilization_before_reset,
        summary->production_utilization_after_reset_seed,
        summary->max_production_utilization,
        summary->max_reference_utilization,
        summary->final_production_utilization,
        summary->final_reference_utilization,
        summary->max_state_abs_error,
        summary->max_state_rel_error,
        summary->max_cap_abs_error_a,
        summary->production_underestimate_violations,
        summary->cap_nonconservative_violations,
        summary->cap_tolerance_violations,
        summary->state_tolerance_violations,
        summary->result_mismatch_violations,
        summary->latch_mismatch_samples,
        summary->latch_nonconservative_violations,
        summary->strict_pass ? 1u : 0u);

    return fclose(file) == 0;
}

static void print_summary(const fuse_replay_options_t *options,
                          const fuse_replay_summary_t *summary)
{
    printf("Fuse replay: %s\n", options->trace_path);
    printf("  startup=%s", fuse_replay_init_policy_name(
        &options->startup_policy));
    if(options->startup_policy.kind == FUSE_REPLAY_INIT_SEEDED_UTILIZATION)
    {
        printf("(%.1f%%)", options->startup_policy.utilization * 100.0);
    }
    printf(" reset=%s", fuse_replay_reset_policy_name(options->reset_kind));
    if(options->reset_kind == FUSE_REPLAY_RESET_SEEDED_UTILIZATION)
    {
        printf("(%.1f%%)", options->reset_seed_utilization * 100.0);
    }
    printf("\n");
    printf("  config: usable=%.1f%% tau=%.1fs rated=%.1fA soak=%.1fs\n",
           options->curve_time_fraction * 100.0,
           options->cooling_time_constant_s,
           options->rated_current_a,
           options->initialization_soak_s);
    printf("  samples=%" PRIu64 " valid=%" PRIu64 " invalid=%" PRIu64
           " resets=%" PRIu64 "\n",
           summary->samples, summary->valid_updates,
           summary->invalid_updates, summary->reset_events);
    printf("  reset utilization before/seed=%.6f/%.6f\n",
           summary->production_utilization_before_reset,
           summary->production_utilization_after_reset_seed);
    printf("  max utilization: production=%.6f reference=%.6f\n",
           summary->max_production_utilization,
           summary->max_reference_utilization);
    printf("  final utilization: production=%.6f reference=%.6f\n",
           summary->final_production_utilization,
           summary->final_reference_utilization);
    printf("  first authority=%" PRId64 " ms; reset=%" PRId64 " ms; post-reset authority=%" PRId64 " ms\n",
           summary->first_authority_ms, summary->first_reset_ms,
           summary->first_post_reset_authority_ms);
    printf("  post-reset exhaust=%" PRId64 " ms\n",
           summary->first_post_reset_exhaust_ms);
    printf("  first exhaust prod/ref=%" PRId64
           "/%" PRId64 " ms\n",
           summary->first_production_exhaust_ms,
           summary->first_reference_exhaust_ms);
    printf("  recovery prod/ref=%" PRId64 "/%" PRId64 " ms\n",
           summary->first_production_recovery_ms,
           summary->first_reference_recovery_ms);
    printf("  max oracle delta: utilization=%.6g (rel %.6g), cap=%.6g A\n",
           summary->max_state_abs_error,
           summary->max_state_rel_error,
           summary->max_cap_abs_error_a);
    printf("  violations: understate=%" PRIu64 " cap=%" PRIu64
           " cap-tolerance=%" PRIu64 " state-tolerance=%" PRIu64 " result=%" PRIu64
           " latch-mismatch=%" PRIu64 " latch-nonconservative=%" PRIu64 "\n",
           summary->production_underestimate_violations,
           summary->cap_nonconservative_violations,
           summary->cap_tolerance_violations,
           summary->state_tolerance_violations,
           summary->result_mismatch_violations,
           summary->latch_mismatch_samples,
           summary->latch_nonconservative_violations);
    printf("RESULT: %s%s\n",
           summary->strict_pass ? "PASS" : "FAIL",
           options->strict ? " (strict)" : " (reported; --strict not requested)");
}

bool fuse_replay_run(const fuse_replay_options_t *options,
                     fuse_replay_summary_t *summary)
{
    if((options == NULL) || (summary == NULL) ||
       (options->trace_path == NULL) ||
       !isfinite(options->curve_time_fraction) ||
       (options->curve_time_fraction <= 0.0) ||
       (options->curve_time_fraction > 1.0) ||
       !isfinite(options->cooling_time_constant_s) ||
       (options->cooling_time_constant_s <= 0.0) ||
       !isfinite(options->rated_current_a) ||
       (options->rated_current_a <= 0.0))
    {
        return false;
    }

    memset(summary, 0, sizeof(*summary));
    summary->first_authority_ms = NO_TIME;
    summary->first_reset_ms = NO_TIME;
    summary->first_post_reset_authority_ms = NO_TIME;
    summary->first_post_reset_exhaust_ms = NO_TIME;
    summary->first_production_exhaust_ms = NO_TIME;
    summary->first_reference_exhaust_ms = NO_TIME;
    summary->first_production_recovery_ms = NO_TIME;
    summary->first_reference_recovery_ms = NO_TIME;

    FILE *input = fopen(options->trace_path, "r");
    if(input == NULL)
    {
        fprintf(stderr, "ERROR: cannot open trace %s: %s\n",
                options->trace_path, strerror(errno));
        return false;
    }
    FILE *output = stdout;
    if(options->output_path != NULL)
    {
        output = fopen(options->output_path, "w");
        if(output == NULL)
        {
            fprintf(stderr, "ERROR: cannot open output %s: %s\n",
                    options->output_path, strerror(errno));
            fclose(input);
            return false;
        }
    }

    ams_fuse_observer_config_t pcfg;
    ams_sop_config_t scfg;
    fuse_ref_config_t rcfg;
    ams_fuse_observer_default_config(&pcfg);
    ams_sop_default_config(&scfg);
    pcfg.curve_time_fraction = (float)options->curve_time_fraction;
    pcfg.cooling_time_constant_s = (float)options->cooling_time_constant_s;
    pcfg.rated_current_a = (float)options->rated_current_a;
    pcfg.initialization_soak_s = (float)options->initialization_soak_s;
    pcfg.quiescent_current_a = (float)options->quiescent_current_a;
    map_ref_config(&pcfg, &scfg, &rcfg);
    if(!ams_fuse_observer_config_valid(&pcfg) ||
       !fuse_ref_config_valid(&rcfg))
    {
        fprintf(stderr, "ERROR: replay configuration is invalid\n");
        fclose(input);
        if(output != stdout) fclose(output);
        return false;
    }

    ams_fuse_observer_t pstate;
    fuse_ref_state_t rstate;
    if(!seed_states(&pstate, &rstate, &pcfg, &rcfg,
                    &options->startup_policy))
    {
        fprintf(stderr, "ERROR: invalid startup policy\n");
        fclose(input);
        if(output != stdout) fclose(output);
        return false;
    }

    char line[MAX_LINE];
    csv_columns_t columns;
    bool header_found = false;
    while(fgets(line, sizeof(line), input) != NULL)
    {
        char *candidate = trim(line);
        if((*candidate == '\0') || (*candidate == '#'))
        {
            continue;
        }
        if(!parse_header(candidate, &columns))
        {
            fprintf(stderr, "ERROR: invalid CSV header\n");
            fclose(input);
            if(output != stdout) fclose(output);
            return false;
        }
        header_found = true;
        break;
    }
    if(!header_found)
    {
        fprintf(stderr, "ERROR: trace has no header\n");
        fclose(input);
        if(output != stdout) fclose(output);
        return false;
    }

    write_output_header(output);
    uint64_t previous_ms = 0u;
    bool have_previous = false;
    bool prior_prod_authority = false;
    bool awaiting_post_reset_authority = false;
    bool awaiting_post_reset_exhaust = false;
    bool prod_has_exhausted = false;
    bool ref_has_exhausted = false;
    ams_fuse_observer_result_t pout;
    fuse_ref_result_t rout;
    memset(&pout, 0, sizeof(pout));
    memset(&rout, 0, sizeof(rout));

    uint64_t source_line = 1u;
    while(fgets(line, sizeof(line), input) != NULL)
    {
        ++source_line;
        char *candidate = trim(line);
        if((*candidate == '\0') || (*candidate == '#'))
        {
            continue;
        }

        trace_row_t row;
        if(!parse_row(candidate, &columns, options, &row))
        {
            fprintf(stderr, "ERROR: invalid trace row at line %" PRIu64 "\n",
                    source_line);
            fclose(input);
            if(output != stdout) fclose(output);
            return false;
        }
        ++summary->samples;

        if(row.reset_event)
        {
            ++summary->reset_events;
            update_time_metric(row.timestamp_ms, true, &summary->first_reset_ms);
            summary->production_utilization_before_reset =
                (double)pstate.thermal_utilization;
            awaiting_post_reset_authority = true;
            awaiting_post_reset_exhaust = true;
            prod_has_exhausted = false;
            ref_has_exhausted = false;
            if(!apply_reset(&pstate, &rstate, &pcfg, &rcfg, options))
            {
                fprintf(stderr, "ERROR: reset policy failed at line %" PRIu64
                        "\n", source_line);
                fclose(input);
                if(output != stdout) fclose(output);
                return false;
            }
            summary->production_utilization_after_reset_seed =
                (double)pstate.thermal_utilization;
            memset(&pout, 0, sizeof(pout));
            memset(&rout, 0, sizeof(rout));
        }

        if(!have_previous)
        {
            previous_ms = row.timestamp_ms;
            have_previous = true;
            write_output_row(output, &row, 0.0, &pstate, &rstate,
                             &pout, &rout);
            continue;
        }
        if(row.timestamp_ms <= previous_ms)
        {
            fprintf(stderr, "ERROR: timestamps must increase at line %" PRIu64
                    "\n", source_line);
            fclose(input);
            if(output != stdout) fclose(output);
            return false;
        }
        const double elapsed_s =
            (double)(row.timestamp_ms - previous_ms) / 1000.0;
        previous_ms = row.timestamp_ms;

        ams_fuse_observer_input_t pin;
        memset(&pin, 0, sizeof(pin));
        pin.pack_current_a = (float)row.current_a;
        pin.current_uncertainty_a = (float)row.uncertainty_a;
        pin.temperature_proxy_c = (float)row.temperature_c;
        pin.elapsed_s = (float)elapsed_s;
        pin.measurement_valid = row.measurement_valid;
        pin.current_calibrated = row.current_calibrated;
        pin.current_polarity_validated = row.polarity_validated;
        pin.temperature_measured_at_fuse = row.temperature_measured;
        pin.model_validated = row.model_validated;
        fuse_ref_input_t rin;
        map_ref_input(&pin, &rin);

        const bool pok = ams_fuse_observer_update(&pstate, &pcfg, &scfg,
                                                  &pin, &pout);
        const bool rok = fuse_ref_step_exact_zoh(&rstate, &rcfg, &rin, &rout);
        if(pok && rok)
        {
            ++summary->valid_updates;
        }
        else
        {
            ++summary->invalid_updates;
        }
        if(pok != rok)
        {
            ++summary->result_mismatch_violations;
        }
        if((pout.valid != rout.valid) ||
           (pout.authority_valid != rout.authority_valid))
        {
            ++summary->result_mismatch_violations;
        }
        if(pout.budget_exhausted != rout.budget_exhausted)
        {
            ++summary->latch_mismatch_samples;
            if((pout.budget_exhausted == 0u) &&
               (rout.budget_exhausted != 0u))
            {
                ++summary->latch_nonconservative_violations;
            }
        }

        const double pstate_value = (double)pstate.thermal_utilization;
        const double rstate_value = (double)rstate.thermal_utilization;
        const double state_abs = fabs(pstate_value - rstate_value);
        const double state_rel = state_abs / fmax(1.0, fabs(rstate_value));
        if(state_abs > summary->max_state_abs_error)
        {
            summary->max_state_abs_error = state_abs;
        }
        if(state_rel > summary->max_state_rel_error)
        {
            summary->max_state_rel_error = state_rel;
        }
        const double state_tolerance = options->state_abs_tolerance +
            options->state_rel_tolerance * fmax(1.0, fabs(rstate_value));
        if(state_abs > state_tolerance)
        {
            ++summary->state_tolerance_violations;
        }
        if(pstate_value + options->state_abs_tolerance < rstate_value)
        {
            ++summary->production_underestimate_violations;
        }

        for(uint32_t h = 0u; h < FUSE_REF_HORIZON_COUNT; ++h)
        {
            const double cap_abs = fabs(
                (double)pout.discharge_current_cap_a[h] -
                (double)rout.discharge_current_cap_a[h]);
            if((pout.budget_exhausted == rout.budget_exhausted) &&
               (cap_abs > summary->max_cap_abs_error_a))
            {
                summary->max_cap_abs_error_a = cap_abs;
            }
            if((pout.budget_exhausted == rout.budget_exhausted) &&
               (cap_abs > options->cap_tolerance_a))
            {
                ++summary->cap_tolerance_violations;
            }
            if((double)pout.discharge_current_cap_a[h] >
               (double)rout.discharge_current_cap_a[h] +
               options->cap_tolerance_a)
            {
                ++summary->cap_nonconservative_violations;
            }
        }

        summary->max_production_utilization = fmax(
            summary->max_production_utilization, (double)pout.utilization);
        summary->max_reference_utilization = fmax(
            summary->max_reference_utilization, (double)rout.utilization);
        summary->final_production_utilization = (double)pout.utilization;
        summary->final_reference_utilization = (double)rout.utilization;

        update_time_metric(row.timestamp_ms,
                           pout.authority_valid != 0u,
                           &summary->first_authority_ms);
        if(awaiting_post_reset_authority && (pout.authority_valid != 0u))
        {
            update_time_metric(row.timestamp_ms, true,
                               &summary->first_post_reset_authority_ms);
            awaiting_post_reset_authority = false;
        }
        if(awaiting_post_reset_exhaust &&
           (pout.budget_exhausted != 0u))
        {
            update_time_metric(row.timestamp_ms, true,
                               &summary->first_post_reset_exhaust_ms);
            awaiting_post_reset_exhaust = false;
        }
        update_time_metric(row.timestamp_ms,
                           pout.budget_exhausted != 0u,
                           &summary->first_production_exhaust_ms);
        update_time_metric(row.timestamp_ms,
                           rout.budget_exhausted != 0u,
                           &summary->first_reference_exhaust_ms);
        if(pout.budget_exhausted != 0u) prod_has_exhausted = true;
        if(rout.budget_exhausted != 0u) ref_has_exhausted = true;
        if(!row.reset_event)
        {
            update_time_metric(row.timestamp_ms,
                               prod_has_exhausted &&
                               (pout.budget_exhausted == 0u),
                               &summary->first_production_recovery_ms);
            update_time_metric(row.timestamp_ms,
                               ref_has_exhausted &&
                               (rout.budget_exhausted == 0u),
                               &summary->first_reference_recovery_ms);
        }

        const bool current_authority = pout.authority_valid != 0u;
        if(prior_prod_authority && !current_authority)
        {
            ++summary->authority_loss_events;
        }
        prior_prod_authority = current_authority;

        write_output_row(output, &row, elapsed_s, &pstate, &rstate,
                         &pout, &rout);
    }

    fclose(input);
    if((output != stdout) && (fclose(output) != 0))
    {
        return false;
    }

    summary->strict_pass =
        (summary->production_underestimate_violations == 0u) &&
        (summary->cap_nonconservative_violations == 0u) &&
        (summary->cap_tolerance_violations == 0u) &&
        (summary->state_tolerance_violations == 0u) &&
        (summary->result_mismatch_violations == 0u) &&
        (summary->latch_nonconservative_violations == 0u);

    print_summary(options, summary);
    if(!write_summary_csv(options, summary))
    {
        return false;
    }
    return !options->strict || summary->strict_pass;
}
