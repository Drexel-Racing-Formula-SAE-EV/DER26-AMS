/*
 * DER26 AMS MiL production estimator runner.
 *
 * Links the checked-in production estimator/LUT sources directly on the host.
 * The CSV adapter intentionally mirrors the production estimator-task boundary:
 * invalid or incoherent AMS measurement epochs invalidate the live estimator
 * state rather than feeding fabricated measurements to the EKF.
 */

#include "estimator/ams_soc_ekf.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SEGMENTS 5u
#define LINE_MAX_LEN 4096u
#define INPUT_FIELDS (7u + (3u * SEGMENTS))

static bool parse_double(const char *text, double *value)
{
    if ((text == NULL) || (value == NULL))
    {
        return false;
    }
    errno = 0;
    char *end = NULL;
    double v = strtod(text, &end);
    if ((end == text) || (errno == ERANGE))
    {
        return false;
    }
    while ((*end == ' ') || (*end == '\t') || (*end == '\r') || (*end == '\n'))
    {
        end++;
    }
    if (*end != '\0')
    {
        return false;
    }
    *value = v;
    return true;
}

static bool parse_u32(const char *text, uint32_t *value)
{
    double v = 0.0;
    if (!parse_double(text, &v) || !isfinite(v) || (v < 0.0) || (v > 4294967295.0))
    {
        return false;
    }
    *value = (uint32_t)llround(v);
    return true;
}

static size_t split_csv(char *line, char **fields, size_t capacity)
{
    size_t count = 0u;
    char *save = NULL;
    for (char *tok = strtok_r(line, ",", &save);
         (tok != NULL) && (count < capacity);
         tok = strtok_r(NULL, ",", &save))
    {
        fields[count++] = tok;
    }
    return count;
}

static void write_header(FILE *out)
{
    (void)fprintf(out,
        "time_s,sequence,current_A,measurement_valid,dt_s,epoch_coherent");
    for (unsigned s = 0u; s < SEGMENTS; s++)
    {
        (void)fprintf(out,
            ",s%u_step_ok,s%u_measurement_used,s%u_measurement_accepted"
            ",s%u_soc,s%u_vp1_V,s%u_vp2_V,s%u_r0_ohm,s%u_tcore_C"
            ",s%u_p_soc,s%u_p_soc_vp1,s%u_p_soc_vp2"
            ",s%u_p_vp1,s%u_p_vp1_vp2,s%u_p_vp2,s%u_p_r0,s%u_R_V2"
            ",s%u_vpred_V,s%u_innovation_V,s%u_innovation_variance_V2,s%u_valid,s%u_fault_flags"
            ",s%u_model_domain_flags,s%u_covariance_repair_count"
            ",s%u_r0_update_result,s%u_soh_reject_flags"
            ",s%u_soh_accepted_count,s%u_soh_rejected_count,s%u_resistance_growth_ratio"
            ",s%u_resistance_confidence_pct,s%u_resistance_status_flags",
            s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s);
        (void)fprintf(out,
            ",s%u_pre_soc,s%u_pre_vp1_V,s%u_pre_vp2_V,s%u_pre_r0_ohm"
            ",s%u_pre_p_soc,s%u_pre_p_soc_vp1,s%u_pre_p_soc_vp2"
            ",s%u_pre_p_vp1,s%u_pre_p_vp1_vp2,s%u_pre_p_vp2",
            s,s,s,s,s,s,s,s,s,s);
        (void)fprintf(out,
            ",s%u_acq_state,s%u_acq_reason,s%u_acq_sample_count"
            ",s%u_acq_reject_count,s%u_acq_candidate_ready,s%u_acq_anchor_count"
            ",s%u_acq_candidate_soc,s%u_acq_ocv_cell_V"
            ",s%u_acq_vp1_finish_V,s%u_acq_vp2_finish_V"
            ",s%u_acq_fit_rmse_mV_cell,s%u_acq_fit_rcond"
            ",s%u_acq_consensus_soc,s%u_acq_dynamic_step_count"
            ",s%u_acq_dynamic_update_count",
            s,s,s,s,s,s,s,s,s,s,s,s,s,s,s);
    }
    (void)fputc('\n', out);
}

static int usage(const char *argv0)
{
    (void)fprintf(stderr,
        "usage: %s INPUT.csv OUTPUT.csv [soc0 soc1 soc2 soc3 soc4]\n"
        "input columns: time_s,sequence,current_A,measurement_valid,"
        "current_calibrated,current_polarity_validated,balance_recovered,"
        "then for each segment: voltage_V,temp_C,valid\n",
        argv0);
    return 2;
}

int main(int argc, char **argv)
{
    if ((argc != 3) && (argc != 8))
    {
        return usage(argv[0]);
    }

    float soc_init[SEGMENTS];
    for (unsigned s = 0u; s < SEGMENTS; s++)
    {
        soc_init[s] = AMS_EKF_DEFAULT_SOC_INIT;
        if (argc == 8)
        {
            double v = 0.0;
            if (!parse_double(argv[3 + (int)s], &v) || !isfinite(v) || (v < 0.0) || (v > 1.0))
            {
                (void)fprintf(stderr, "invalid initial SoC for segment %u\n", s);
                return 2;
            }
            soc_init[s] = (float)v;
        }
    }

    FILE *in = fopen(argv[1], "r");
    if (in == NULL)
    {
        perror("open input");
        return 2;
    }
    FILE *out = fopen(argv[2], "w");
    if (out == NULL)
    {
        perror("open output");
        (void)fclose(in);
        return 2;
    }

    ams_estimator_t est;
    memset(&est, 0, sizeof(est));
    est.enabled = 1u;
    est.instance_count = SEGMENTS;
    est.active_index = 0u;
    for (unsigned s = 0u; s < SEGMENTS; s++)
    {
        ams_ekf_config_t cfg;
        ams_ekf_make_segment_config(&cfg, (uint8_t)s);
        cfg.soc_init = soc_init[s];
        ams_ekf_init(&est.inst[s], &cfg);
    }

    char line[LINE_MAX_LEN];
    if (fgets(line, sizeof(line), in) == NULL)
    {
        (void)fprintf(stderr, "input CSV is empty\n");
        (void)fclose(in);
        (void)fclose(out);
        return 2;
    }
    write_header(out);

    bool have_previous_time = false;
    double previous_time_s = 0.0;
    unsigned long row = 1ul;

    while (fgets(line, sizeof(line), in) != NULL)
    {
        row++;
        if ((line[0] == '\0') || (line[0] == '\n') || (line[0] == '\r'))
        {
            continue;
        }

        char *fields[INPUT_FIELDS + 4u];
        const size_t count = split_csv(line, fields, INPUT_FIELDS + 4u);
        if (count != INPUT_FIELDS)
        {
            (void)fprintf(stderr, "row %lu: expected %u fields, got %zu\n",
                          row, (unsigned)INPUT_FIELDS, count);
            (void)fclose(in);
            (void)fclose(out);
            return 3;
        }

        double time_s_d = 0.0;
        double current_d = 0.0;
        uint32_t sequence = 0u;
        double measurement_valid_d = 0.0;
        double current_calibrated_d = 0.0;
        double polarity_validated_d = 0.0;
        double balance_recovered_d = 0.0;
        /* current_A may intentionally be NaN when the modeled current path is
         * invalid.  Keep parsing that row and let measurement_valid plus the
         * production finite-input checks fail it closed.  Rejecting the CSV
         * here would prevent C0/current-dropout qualification from reaching
         * the production estimator at all.  time_s must remain finite because
         * it defines host epoch timing. */
        if (!parse_double(fields[0], &time_s_d) || !isfinite(time_s_d) ||
            !parse_u32(fields[1], &sequence) ||
            !parse_double(fields[2], &current_d) ||
            !parse_double(fields[3], &measurement_valid_d) ||
            !parse_double(fields[4], &current_calibrated_d) ||
            !parse_double(fields[5], &polarity_validated_d) ||
            !parse_double(fields[6], &balance_recovered_d))
        {
            (void)fprintf(stderr, "row %lu: invalid common field\n", row);
            (void)fclose(in);
            (void)fclose(out);
            return 3;
        }

        float segment_v[SEGMENTS];
        float segment_t[SEGMENTS];
        bool segment_valid[SEGMENTS];
        size_t idx = 7u;
        for (unsigned s = 0u; s < SEGMENTS; s++)
        {
            double v = 0.0, t = 0.0, valid = 0.0;
            if (!parse_double(fields[idx++], &v) ||
                !parse_double(fields[idx++], &t) ||
                !parse_double(fields[idx++], &valid))
            {
                (void)fprintf(stderr, "row %lu: invalid segment %u field\n", row, s);
                (void)fclose(in);
                (void)fclose(out);
                return 3;
            }
            segment_v[s] = (float)v;
            segment_t[s] = (float)t;
            segment_valid[s] = (valid != 0.0) && isfinite(v) && isfinite(t);
        }

        float dt_s = AMS_EKF_DEFAULT_DT_S;
        bool epoch_coherent = true;
        if (have_previous_time)
        {
            const double dt = time_s_d - previous_time_s;
            epoch_coherent = isfinite(dt) && (dt >= 0.001) && (dt <= 1.0);
            if (epoch_coherent)
            {
                dt_s = (float)dt;
            }
        }
        previous_time_s = time_s_d;
        have_previous_time = true;

        const bool measurement_valid = measurement_valid_d != 0.0;
        const bool current_calibrated = current_calibrated_d != 0.0;
        const bool polarity_validated = polarity_validated_d != 0.0;
        const bool balance_recovered = balance_recovered_d != 0.0;
        const bool hardware_inputs_ready = measurement_valid && epoch_coherent &&
                                           isfinite(current_d);
        const uint32_t tick_ms = (time_s_d <= 0.0) ? 0u :
            ((time_s_d >= 4294967.0) ? UINT32_MAX : (uint32_t)llround(time_s_d * 1000.0));

        (void)fprintf(out, "%.9g,%u,%.9g,%u,%.9g,%u",
                      time_s_d, sequence, current_d, measurement_valid ? 1u : 0u,
                      dt_s, epoch_coherent ? 1u : 0u);

        bool measurement_used[SEGMENTS] = {false};
        bool step_ok[SEGMENTS] = {false};
        bool accepted[SEGMENTS] = {false};
        /* Capture the exact production state immediately before each host
         * update. This closes the old t=0 ambiguity without inserting an
         * extra CSV row that would break time alignment with the plant. */
        float pre_soc[SEGMENTS], pre_vp1[SEGMENTS], pre_vp2[SEGMENTS];
        float pre_r0[SEGMENTS], pre_p_soc[SEGMENTS];
        float pre_p_soc_vp1[SEGMENTS], pre_p_soc_vp2[SEGMENTS];
        float pre_p_vp1[SEGMENTS], pre_p_vp1_vp2[SEGMENTS], pre_p_vp2[SEGMENTS];
        for(unsigned s = 0u; s < SEGMENTS; s++)
        {
            const ams_ekf_instance_t *ekf = &est.inst[s];
            pre_soc[s] = ekf->soc;
            pre_vp1[s] = ekf->vp1_V;
            pre_vp2[s] = ekf->vp2_V;
            pre_r0[s] = ekf->r0_ohm;
            pre_p_soc[s] = ekf->p_soc;
            pre_p_soc_vp1[s] = ekf->p_soc_vp1;
            pre_p_soc_vp2[s] = ekf->p_soc_vp2;
            pre_p_vp1[s] = ekf->p_vp1;
            pre_p_vp1_vp2[s] = ekf->p_vp1_vp2;
            pre_p_vp2[s] = ekf->p_vp2;
        }
        ams_ekf_r0_update_result_t r0_result[SEGMENTS];
        uint32_t soh_reject[SEGMENTS] = {0u};
        for (unsigned s = 0u; s < SEGMENTS; s++)
        {
            r0_result[s] = AMS_EKF_R0_UPDATE_NOT_REQUESTED;
            /* Mirror estimator_task.c: SoC estimation may continue with a
             * valid current sample even when R0/SoH calibration authority is
             * not yet established. Calibration/polarity confidence gates R0
             * adaptation; it does not gate the inner SoC estimator. */
            measurement_used[s] = hardware_inputs_ready && segment_valid[s] &&
                                  balance_recovered;

            ams_ekf_instance_t *ekf = &est.inst[s];
            ams_resistance_soh_t *soh = &est.resistance_soh[s];
            const bool acquisition_pending = !ams_ekf_acquisition_complete(ekf);

            if (measurement_used[s])
            {
                soh_reject[s] = ams_resistance_soh_gate(ekf, (float)current_d,
                                                       true, balance_recovered,
                                                       current_calibrated && polarity_validated);
                if(acquisition_pending)
                {
                    soh_reject[s] |= AMS_SOH_REJECT_ACQUISITION;
                }
                const uint32_t rejects_before = ekf->innovation_reject_count;
                if(acquisition_pending)
                {
                    step_ok[s] = ams_ekf_step_acquiring_gated(
                        ekf, (float)current_d, segment_v[s], segment_t[s], dt_s,
                        &r0_result[s]);
                }
                else
                {
                    step_ok[s] = ams_ekf_step_gated(
                        ekf, (float)current_d, segment_v[s], segment_t[s], dt_s,
                        soh_reject[s] == AMS_SOH_REJECT_NONE, &r0_result[s]);
                }
                accepted[s] = step_ok[s] &&
                    (ekf->innovation_reject_count == rejects_before);

                /* Near-zero acquisition needs a trusted zero/current scale,
                 * not current-direction validation. The host CSV has no
                 * uncertainty field, so MiL parity uses zero additional
                 * uncertainty; target firmware uses the runtime calibrated
                 * uncertainty from the current snapshot. */
                ams_ekf_acquisition_observe(
                    ekf, (float)current_d, 0.0f, segment_v[s], segment_t[s],
                    tick_ms, current_calibrated, step_ok[s]);
                ams_resistance_soh_record(soh, ekf, sequence, tick_ms,
                                          current_calibrated && polarity_validated,
                                          soh_reject[s], r0_result[s], step_ok[s]);
            }
            else
            {
                ekf->valid = 0u;
                ekf->fault_flags = epoch_coherent ? AMS_EKF_FAULT_BAD_INPUT :
                                                    AMS_EKF_FAULT_EPOCH_TIMING;
                soh_reject[s] = ams_resistance_soh_gate(
                    ekf, (float)current_d, false, balance_recovered,
                    current_calibrated && polarity_validated);
                if(acquisition_pending)
                {
                    soh_reject[s] |= AMS_SOH_REJECT_ACQUISITION;
                }
                ams_ekf_acquisition_observe(
                    ekf, (float)current_d, 0.0f, NAN, NAN, tick_ms,
                    current_calibrated, false);
                ams_resistance_soh_record(soh, ekf, sequence, tick_ms,
                                          current_calibrated && polarity_validated,
                                          soh_reject[s],
                                          AMS_EKF_R0_UPDATE_NOT_REQUESTED,
                                          false);
            }
        }

        /* Resolve all segment candidates together so one coherent biased
         * segment cannot independently redefine pack SoC. */
        ams_estimator_acquisition_resolve(&est);

        for (unsigned s = 0u; s < SEGMENTS; s++)
        {
            const ams_ekf_instance_t *ekf = &est.inst[s];
            const ams_resistance_soh_t *soh = &est.resistance_soh[s];
            const ams_ekf_acquisition_t *acq = &ekf->acquisition;
            (void)fprintf(out,
                ",%u,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g"
                ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g"
                ",%.9g,%.9g,%.9g,%u,%u,%u,%u,%u,%u,%u,%u,%.9g,%u,%u",
                step_ok[s] ? 1u : 0u,
                measurement_used[s] ? 1u : 0u,
                accepted[s] ? 1u : 0u,
                ekf->soc, ekf->vp1_V, ekf->vp2_V, ekf->r0_ohm,
                ekf->t_core_C, ekf->p_soc, ekf->p_soc_vp1, ekf->p_soc_vp2,
                ekf->p_vp1, ekf->p_vp1_vp2, ekf->p_vp2, ekf->p_r0,
                ekf->r_meas_V2, ekf->v_pred_V, ekf->innovation_V,
                ekf->innovation_variance_V2,
                (unsigned)ekf->valid, (unsigned)ekf->fault_flags,
                (unsigned)ekf->model_domain_flags,
                (unsigned)ekf->covariance_repair_count,
                (unsigned)r0_result[s], (unsigned)soh_reject[s],
                (unsigned)soh->accepted_count, (unsigned)soh->rejected_count,
                soh->resistance_growth_ratio,
                (unsigned)soh->observation_confidence_pct,
                (unsigned)soh->status_flags);
            (void)fprintf(out,
                ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                pre_soc[s], pre_vp1[s], pre_vp2[s], pre_r0[s],
                pre_p_soc[s], pre_p_soc_vp1[s], pre_p_soc_vp2[s],
                pre_p_vp1[s], pre_p_vp1_vp2[s], pre_p_vp2[s]);
            (void)fprintf(out,
                ",%u,%u,%u,%u,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u",
                (unsigned)acq->state,
                (unsigned)acq->reason,
                (unsigned)acq->sample_count,
                (unsigned)acq->reject_count,
                (unsigned)acq->candidate_ready,
                (unsigned)acq->anchor_count,
                acq->candidate_soc,
                acq->candidate_ocv_cell_V,
                acq->candidate_vp1_finish_V,
                acq->candidate_vp2_finish_V,
                acq->fit_rmse_mV_cell,
                acq->fit_rcond,
                acq->consensus_soc,
                (unsigned)acq->dynamic_step_count,
                (unsigned)acq->dynamic_update_count);
        }
        (void)fputc('\n', out);
    }

    if (ferror(in) || ferror(out))
    {
        (void)fprintf(stderr, "I/O error while processing estimator CSV\n");
        (void)fclose(in);
        (void)fclose(out);
        return 4;
    }

    (void)fclose(in);
    if (fclose(out) != 0)
    {
        perror("close output");
        return 4;
    }
    return 0;
}
