/*
 * ams_soc_ekf.c
 * Author: Mahad Faisal (2026)
 *
 * No-NN adaptive dual EKF ported from the RA8M1 BMS estimator into a small,
 * instance-based STM32 AMS module.
 */

#include "estimator/ams_soc_ekf.h"
#include "estimator/ams_estimator_lut.h"

#include <math.h>
#include <string.h>

#define AMS_EKF_Q_SOC       5.0e-7f
#define AMS_EKF_Q_VP1       1.0e-5f
#define AMS_EKF_Q_VP2       1.0e-6f
#define AMS_EKF_Q_R0        1.0e-8f
#define AMS_EKF_P0_SOC      1.0e-2f
#define AMS_EKF_P0_VP1      1.0e-3f
#define AMS_EKF_P0_VP2      1.0e-3f
#define AMS_EKF_P0_R0       1.0e-4f
#define AMS_EKF_P_R0_MAX    1.0e-2f
#define AMS_EKF_DSOC        1.0e-4f

/*
 * RA8M1 parity note:
 * The working RA8M1 estimator uses a 3-state inner EKF [SoC,Vp1,Vp2],
 * a scalar outer R0 loop, adaptive measurement covariance R, and a
 * feed-forward T_core observer.  R0, OCV, inv_C1, and negative inv_tau1 are
 * LUT driven.  The slow Vp2 branch is intentionally fixed in the RA model:
 * R2=0.004 ohm, C2=12000 F, tau2=48 s.  There is no R2/C2 LUT in the
 * validated RA8M1 source because that branch was not reliably identifiable
 * from the HPPC fit used for this bring-up.
 */
#define AMS_EKF_INV_C2      8.3333333e-5f   /* 1 / 12000 F */
#define AMS_EKF_INV_TAU2    2.08333333e-2f  /* 1 / 48 s */
#define AMS_EKF_INV_R2      250.0f          /* 1 / 0.004 ohm */
#define AMS_EKF_INV_CC      1.81818176e-2f  /* 1 / 55 J/K */
#define AMS_EKF_INV_RCS     6.66666687e-1f  /* 1 / 1.5 K/W */

#define AMS_EKF_R0_MIN_OHM  0.005f
#define AMS_EKF_R0_MAX_OHM  0.040f          /* Match RA8M1 DAEKF clamp */

static void saturating_increment(uint32_t *value);

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

static float sqrf_local(float x)
{
    return x * x;
}

static float temperature_edge_fraction(float temp_c)
{
    const float t = clampf_local(temp_c, 5.0f, 40.0f);
    if(t <= 25.0f)
    {
        return clampf_local((25.0f - t) / 20.0f, 0.0f, 1.0f);
    }
    return clampf_local((t - 25.0f) / 15.0f, 0.0f, 1.0f);
}

static float interpolate_edge_value(float temp_c,
                                    float nominal,
                                    float edge)
{
    const float w = temperature_edge_fraction(temp_c);
    return nominal + (w * (edge - nominal));
}

static float measurement_r_floor_v2(float series_count, float temp_c)
{
    const float sigma_cell = interpolate_edge_value(
        temp_c,
        AMS_EKF_R_SIGMA_FLOOR_PER_CELL_V,
        AMS_EKF_R_SIGMA_FLOOR_TEMP_EDGE_PER_CELL_V);
    return sqrf_local(series_count * sigma_cell);
}

static float measurement_r_init_v2(float series_count)
{
    return sqrf_local(series_count * AMS_EKF_R_INIT_SIGMA_PER_CELL_V);
}

static void covariance_load(const ams_ekf_instance_t *ekf, float p[3][3])
{
    p[0][0] = ekf->p_soc;
    p[0][1] = ekf->p_soc_vp1;
    p[0][2] = ekf->p_soc_vp2;
    p[1][0] = ekf->p_soc_vp1;
    p[1][1] = ekf->p_vp1;
    p[1][2] = ekf->p_vp1_vp2;
    p[2][0] = ekf->p_soc_vp2;
    p[2][1] = ekf->p_vp1_vp2;
    p[2][2] = ekf->p_vp2;
}

static void covariance_store(ams_ekf_instance_t *ekf, float p[3][3])
{
    ekf->p_soc = p[0][0];
    ekf->p_soc_vp1 = 0.5f * (p[0][1] + p[1][0]);
    ekf->p_soc_vp2 = 0.5f * (p[0][2] + p[2][0]);
    ekf->p_vp1 = p[1][1];
    ekf->p_vp1_vp2 = 0.5f * (p[1][2] + p[2][1]);
    ekf->p_vp2 = p[2][2];
}

static bool covariance_all_finite(float p[3][3])
{
    for(uint8_t row = 0u; row < 3u; row++)
    {
        for(uint8_t col = 0u; col < 3u; col++)
        {
            if(!isfinite(p[row][col]))
            {
                return false;
            }
        }
    }
    return true;
}

static void covariance_limit_pair(float *cov, float var_a, float var_b)
{
    const float bound = sqrtf(fmaxf(0.0f, var_a * var_b));
    if(!isfinite(*cov) || !isfinite(bound))
    {
        *cov = 0.0f;
        return;
    }
    *cov = clampf_local(*cov, -bound, bound);
}

static bool covariance_positive_semidefinite(float p[3][3])
{
    if(!covariance_all_finite(p) ||
       (p[0][0] < 0.0f) || (p[1][1] < 0.0f) || (p[2][2] < 0.0f))
    {
        return false;
    }

    const float d01 = (p[0][0] * p[1][1]) - (p[0][1] * p[0][1]);
    const float d02 = (p[0][0] * p[2][2]) - (p[0][2] * p[0][2]);
    const float d12 = (p[1][1] * p[2][2]) - (p[1][2] * p[1][2]);
    const float det =
        (p[0][0] * ((p[1][1] * p[2][2]) - (p[1][2] * p[2][1]))) -
        (p[0][1] * ((p[1][0] * p[2][2]) - (p[1][2] * p[2][0]))) +
        (p[0][2] * ((p[1][0] * p[2][1]) - (p[1][1] * p[2][0])));

    const float scale = fmaxf(1.0e-20f,
        p[0][0] * p[1][1] * p[2][2]);
    const float tol2 = 1.0e-5f * fmaxf(1.0e-20f,
        fmaxf(p[0][0] * p[1][1],
              fmaxf(p[0][0] * p[2][2], p[1][1] * p[2][2])));
    const float tol3 = 1.0e-4f * scale;
    return (d01 >= -tol2) && (d02 >= -tol2) && (d12 >= -tol2) &&
           (det >= -tol3);
}

static bool covariance_sanitize(ams_ekf_instance_t *ekf,
                                float p[3][3],
                                float temp_c,
                                bool acquisition_constrained)
{
    if(!covariance_all_finite(p))
    {
        return false;
    }

    float soc_sigma_floor;
    float vp_sigma_floor;
    if(acquisition_constrained)
    {
        soc_sigma_floor = AMS_EKF_ACQ_DYNAMIC_SOC_SIGMA_FLOOR;
        vp_sigma_floor = AMS_EKF_ACQ_DYNAMIC_VP_SIGMA_FLOOR_V;
    }
    else
    {
        soc_sigma_floor = interpolate_edge_value(
            temp_c,
            AMS_EKF_SOC_SIGMA_FLOOR_NOMINAL,
            AMS_EKF_SOC_SIGMA_FLOOR_TEMP_EDGE);
        vp_sigma_floor = interpolate_edge_value(
            temp_c,
            AMS_EKF_VP_SIGMA_FLOOR_NOMINAL_V,
            AMS_EKF_VP_SIGMA_FLOOR_TEMP_EDGE_V);
    }

    const float soc_floor = sqrf_local(soc_sigma_floor);
    const float vp_floor = sqrf_local(vp_sigma_floor);
    /* Confidence floors and acquisition decorrelation are intentional
     * estimator policy, not evidence of a numerically damaged covariance.
     * Do not count routine floor enforcement as a covariance repair; otherwise
     * a healthy estimator increments the repair counter almost every sample
     * and hides genuine PSD/correlation recovery events. */
    bool repaired = false;
    if(p[0][0] < soc_floor) { p[0][0] = soc_floor; }
    if(p[1][1] < vp_floor) { p[1][1] = vp_floor; }
    if(p[2][2] < vp_floor) { p[2][2] = vp_floor; }

    if(acquisition_constrained)
    {
        p[0][1] = p[1][0] = 0.0f;
        p[0][2] = p[2][0] = 0.0f;
        p[1][2] = p[2][1] = 0.0f;
    }
    else
    {
        const float old01 = p[0][1];
        const float old02 = p[0][2];
        const float old12 = p[1][2];
        covariance_limit_pair(&p[0][1], p[0][0], p[1][1]);
        covariance_limit_pair(&p[0][2], p[0][0], p[2][2]);
        covariance_limit_pair(&p[1][2], p[1][1], p[2][2]);
        p[1][0] = p[0][1];
        p[2][0] = p[0][2];
        p[2][1] = p[1][2];
        repaired = repaired || (old01 != p[0][1]) ||
                   (old02 != p[0][2]) || (old12 != p[1][2]);

        /* A rare float-roundoff loss of positive-semidefiniteness is repaired
         * conservatively by dropping correlations, not by shrinking diagonal
         * uncertainty.  If the diagonal matrix is still invalid, fail closed. */
        if(!covariance_positive_semidefinite(p))
        {
            p[0][1] = p[1][0] = 0.0f;
            p[0][2] = p[2][0] = 0.0f;
            p[1][2] = p[2][1] = 0.0f;
            repaired = true;
            if(!covariance_positive_semidefinite(p))
            {
                return false;
            }
        }
    }

    if(repaired)
    {
        saturating_increment(&ekf->covariance_repair_count);
    }
    return true;
}

static float covariance_innovation_variance(float p[3][3],
                                            const float h[3],
                                            float r_meas)
{
    float ph[3] = {0.0f, 0.0f, 0.0f};
    for(uint8_t row = 0u; row < 3u; row++)
    {
        for(uint8_t col = 0u; col < 3u; col++)
        {
            ph[row] += p[row][col] * h[col];
        }
    }
    return (h[0] * ph[0]) + (h[1] * ph[1]) + (h[2] * ph[2]) + r_meas;
}

static bool covariance_joseph_update(float prior[3][3],
                                     const float gain[3],
                                     const float h[3],
                                     float r_meas,
                                     float posterior[3][3])
{
    float a[3][3];
    float ap[3][3] = {{0.0f}};
    for(uint8_t row = 0u; row < 3u; row++)
    {
        for(uint8_t col = 0u; col < 3u; col++)
        {
            a[row][col] = ((row == col) ? 1.0f : 0.0f) -
                          (gain[row] * h[col]);
        }
    }

    for(uint8_t row = 0u; row < 3u; row++)
    {
        for(uint8_t col = 0u; col < 3u; col++)
        {
            for(uint8_t k = 0u; k < 3u; k++)
            {
                ap[row][col] += a[row][k] * prior[k][col];
            }
        }
    }

    for(uint8_t row = 0u; row < 3u; row++)
    {
        for(uint8_t col = 0u; col < 3u; col++)
        {
            float value = gain[row] * r_meas * gain[col];
            for(uint8_t k = 0u; k < 3u; k++)
            {
                value += ap[row][k] * a[col][k];
            }
            posterior[row][col] = value;
        }
    }

    /* Keep exact symmetry in the stored representation. */
    for(uint8_t row = 0u; row < 3u; row++)
    {
        for(uint8_t col = (uint8_t)(row + 1u); col < 3u; col++)
        {
            const float symmetric = 0.5f *
                (posterior[row][col] + posterior[col][row]);
            posterior[row][col] = symmetric;
            posterior[col][row] = symmetric;
        }
    }
    return covariance_all_finite(posterior);
}

static bool finite3(float a, float b, float c)
{
    return (isfinite(a) && isfinite(b) && isfinite(c));
}

static void saturating_increment(uint32_t *value)
{
    if((value != NULL) && (*value != UINT32_MAX))
    {
        (*value)++;
    }
}

static void saturating_increment_u8(uint8_t *value)
{
    if((value != NULL) && (*value != UINT8_MAX))
    {
        (*value)++;
    }
}

static void acquisition_window_clear(ams_ekf_acquisition_t *acq,
                                     ams_ekf_acquisition_reason_t reason)
{
    if(acq == NULL)
    {
        return;
    }

    acq->state = AMS_EKF_ACQ_WAITING;
    acq->reason = (uint8_t)reason;
    acq->sample_count = 0u;
    acq->candidate_ready = 0u;
    acq->start_tick = 0u;
    acq->last_sample_tick = 0u;
    acq->xtx00 = 0.0f;
    acq->xtx01 = 0.0f;
    acq->xtx02 = 0.0f;
    acq->xtx11 = 0.0f;
    acq->xtx12 = 0.0f;
    acq->xtx22 = 0.0f;
    acq->xty0 = 0.0f;
    acq->xty1 = 0.0f;
    acq->xty2 = 0.0f;
    acq->y2_sum = 0.0f;
    acq->temperature_sum_C = 0.0f;
    acq->voltage_reference_cell_V = NAN;
    acq->candidate_soc = NAN;
    acq->candidate_ocv_cell_V = NAN;
    acq->candidate_vp1_finish_V = NAN;
    acq->candidate_vp2_finish_V = NAN;
    acq->fit_rmse_mV_cell = NAN;
    acq->fit_rcond = NAN;
    acq->consensus_soc = NAN;
}

static void acquisition_init(ams_ekf_acquisition_t *acq)
{
    if(acq == NULL)
    {
        return;
    }

    memset(acq, 0, sizeof(*acq));
    acquisition_window_clear(acq,
                             AMS_EKF_ACQ_REASON_WAITING_FOR_LOW_CURRENT);
}

static float matrix3_norm1_flat(const float *a)
{
    if(a == NULL)
    {
        return 0.0f;
    }
    float max_sum = 0.0f;
    for(uint8_t col = 0u; col < 3u; col++)
    {
        float sum = 0.0f;
        for(uint8_t row = 0u; row < 3u; row++)
        {
            sum += fabsf(a[(3u * row) + col]);
        }
        if(sum > max_sum)
        {
            max_sum = sum;
        }
    }
    return max_sum;
}

static bool matrix3_inverse(const float a[3][3],
                            float inv[3][3],
                            float *rcond)
{
    if((a == NULL) || (inv == NULL))
    {
        return false;
    }

    const float c00 = (a[1][1] * a[2][2]) - (a[1][2] * a[2][1]);
    const float c01 = -((a[1][0] * a[2][2]) - (a[1][2] * a[2][0]));
    const float c02 = (a[1][0] * a[2][1]) - (a[1][1] * a[2][0]);
    const float c10 = -((a[0][1] * a[2][2]) - (a[0][2] * a[2][1]));
    const float c11 = (a[0][0] * a[2][2]) - (a[0][2] * a[2][0]);
    const float c12 = -((a[0][0] * a[2][1]) - (a[0][1] * a[2][0]));
    const float c20 = (a[0][1] * a[1][2]) - (a[0][2] * a[1][1]);
    const float c21 = -((a[0][0] * a[1][2]) - (a[0][2] * a[1][0]));
    const float c22 = (a[0][0] * a[1][1]) - (a[0][1] * a[1][0]);

    const float det = (a[0][0] * c00) + (a[0][1] * c01) +
                      (a[0][2] * c02);
    if(!isfinite(det) || (fabsf(det) < 1.0e-12f))
    {
        if(rcond != NULL)
        {
            *rcond = 0.0f;
        }
        return false;
    }

    const float inv_det = 1.0f / det;
    /* inverse = transpose(cofactor)/det */
    inv[0][0] = c00 * inv_det;
    inv[0][1] = c10 * inv_det;
    inv[0][2] = c20 * inv_det;
    inv[1][0] = c01 * inv_det;
    inv[1][1] = c11 * inv_det;
    inv[1][2] = c21 * inv_det;
    inv[2][0] = c02 * inv_det;
    inv[2][1] = c12 * inv_det;
    inv[2][2] = c22 * inv_det;

    const float norm_a = matrix3_norm1_flat(&a[0][0]);
    const float norm_inv = matrix3_norm1_flat(&inv[0][0]);
    float rc = 0.0f;
    if(isfinite(norm_a) && isfinite(norm_inv) &&
       (norm_a > 0.0f) && (norm_inv > 0.0f))
    {
        rc = 1.0f / (norm_a * norm_inv);
    }
    if(rcond != NULL)
    {
        *rcond = rc;
    }
    return isfinite(rc) && (rc > 0.0f);
}

static bool soc_from_ocv(float ocv_cell_V, float temp_c, float *soc)
{
    if((soc == NULL) || !isfinite(ocv_cell_V) || !isfinite(temp_c))
    {
        return false;
    }

    const float t = clampf_local(temp_c, 5.0f, 40.0f);
    const float v0 = ams_p42a_ocv_v(0.0f, t);
    const float v1 = ams_p42a_ocv_v(1.0f, t);
    if(!isfinite(v0) || !isfinite(v1))
    {
        return false;
    }

    const float vlo = fminf(v0, v1);
    const float vhi = fmaxf(v0, v1);
    if((ocv_cell_V < vlo) || (ocv_cell_V > vhi))
    {
        return false;
    }

    float lo = 0.0f;
    float hi = 1.0f;
    const bool increasing = (v1 >= v0);
    for(uint8_t k = 0u; k < 24u; k++)
    {
        const float mid = 0.5f * (lo + hi);
        const float vmid = ams_p42a_ocv_v(mid, t);
        if(!isfinite(vmid))
        {
            return false;
        }
        if((vmid < ocv_cell_V) == increasing)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }

    *soc = clampf_local(0.5f * (lo + hi), 0.0f, 1.0f);
    return true;
}

static void acquisition_add_sample(ams_ekf_instance_t *ekf,
                                   float v_meas_V,
                                   float t_surf_C,
                                   uint32_t tick)
{
    ams_ekf_acquisition_t *acq = &ekf->acquisition;
    const float series_count = (float)ekf->cfg.series_group_count;
    const float elapsed_s = (float)((uint32_t)(tick - acq->start_tick)) /
                            1000.0f;
    const float e1 = expf(-elapsed_s / AMS_EKF_ACQ_TAU1_S);
    const float e2 = expf(-elapsed_s / AMS_EKF_ACQ_TAU2_S);
    const float y = v_meas_V / series_count;
    if((acq->sample_count == 0u) ||
       !isfinite(acq->voltage_reference_cell_V))
    {
        acq->voltage_reference_cell_V = y;
    }
    const float yc = y - acq->voltage_reference_cell_V;

    acq->xtx00 += 1.0f;
    acq->xtx01 += e1;
    acq->xtx02 += e2;
    acq->xtx11 += e1 * e1;
    acq->xtx12 += e1 * e2;
    acq->xtx22 += e2 * e2;
    acq->xty0 += yc;
    acq->xty1 += e1 * yc;
    acq->xty2 += e2 * yc;
    acq->y2_sum += yc * yc;
    acq->temperature_sum_C += t_surf_C;
    if(acq->sample_count != UINT8_MAX)
    {
        acq->sample_count++;
    }
    acq->last_sample_tick = tick;
}

static ams_ekf_acquisition_reason_t acquisition_build_candidate(
    ams_ekf_instance_t *ekf)
{
    ams_ekf_acquisition_t *acq = &ekf->acquisition;
    if(acq->sample_count < AMS_EKF_ACQ_MIN_FIT_SAMPLES)
    {
        return AMS_EKF_ACQ_REASON_INSUFFICIENT_SAMPLES_RETRY;
    }

    const float normal[3][3] = {
        {acq->xtx00, acq->xtx01, acq->xtx02},
        {acq->xtx01, acq->xtx11, acq->xtx12},
        {acq->xtx02, acq->xtx12, acq->xtx22}
    };
    float inv[3][3];
    float rcond = 0.0f;
    if(!matrix3_inverse(normal, inv, &rcond) ||
       !isfinite(rcond) || (rcond < AMS_EKF_ACQ_MIN_FIT_RCOND))
    {
        acq->fit_rcond = rcond;
        return AMS_EKF_ACQ_REASON_FIT_CONDITIONING_RETRY;
    }

    const float rhs[3] = {acq->xty0, acq->xty1, acq->xty2};
    float beta[3] = {0.0f, 0.0f, 0.0f};
    for(uint8_t row = 0u; row < 3u; row++)
    {
        beta[row] = (inv[row][0] * rhs[0]) +
                    (inv[row][1] * rhs[1]) +
                    (inv[row][2] * rhs[2]);
    }
    if(!finite3(beta[0], beta[1], beta[2]))
    {
        return AMS_EKF_ACQ_REASON_FIT_CONDITIONING_RETRY;
    }

    float sse = acq->y2_sum - ((beta[0] * rhs[0]) +
                               (beta[1] * rhs[1]) +
                               (beta[2] * rhs[2]));
    if(!isfinite(sse))
    {
        return AMS_EKF_ACQ_REASON_FIT_CONDITIONING_RETRY;
    }
    if(sse < 0.0f)
    {
        /* Roundoff can make the normal-equation residual slightly negative. */
        sse = 0.0f;
    }
    const float fit_rmse_mV_cell =
        1000.0f * sqrtf(sse / (float)acq->sample_count);
    acq->fit_rcond = rcond;
    acq->fit_rmse_mV_cell = fit_rmse_mV_cell;
    if(!isfinite(fit_rmse_mV_cell) ||
       (fit_rmse_mV_cell > AMS_EKF_ACQ_MAX_FIT_RMSE_MV_CELL))
    {
        return AMS_EKF_ACQ_REASON_FIT_RESIDUAL_RETRY;
    }

    const float ocv_fit = beta[0] + acq->voltage_reference_cell_V;
    const float elapsed_s =
        (float)((uint32_t)(acq->last_sample_tick - acq->start_tick)) /
        1000.0f;
    const float vp1_finish = -beta[1] *
        expf(-elapsed_s / AMS_EKF_ACQ_TAU1_S);
    const float vp2_finish = -beta[2] *
        expf(-elapsed_s / AMS_EKF_ACQ_TAU2_S);
    if(!finite3(ocv_fit, vp1_finish, vp2_finish) ||
       (fabsf(vp1_finish) > AMS_EKF_ACQ_MAX_ABS_VP_STATE_V) ||
       (fabsf(vp2_finish) > AMS_EKF_ACQ_MAX_ABS_VP_STATE_V) ||
       (fabsf(vp1_finish + vp2_finish) >
        AMS_EKF_ACQ_MAX_ABS_TOTAL_POLARIZATION_V))
    {
        return AMS_EKF_ACQ_REASON_POLARIZATION_RETRY;
    }

    const float t_avg = acq->temperature_sum_C /
                        (float)acq->sample_count;
    float soc_anchor = NAN;
    if(!soc_from_ocv(ocv_fit, t_avg, &soc_anchor))
    {
        return AMS_EKF_ACQ_REASON_OCV_RANGE_RETRY;
    }

    acq->candidate_soc = soc_anchor;
    acq->candidate_ocv_cell_V = ocv_fit;
    acq->candidate_vp1_finish_V = vp1_finish;
    acq->candidate_vp2_finish_V = vp2_finish;
    acq->candidate_ready = 1u;
    return AMS_EKF_ACQ_REASON_COLLECTING_RELAXATION;
}

void ams_ekf_make_pack_config(ams_ekf_config_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->enabled = 1U;
    cfg->first_series_group = 0U;
    cfg->series_group_count = AMS_EKF_PACK_SERIES_GROUPS;
    cfg->parallel_cell_count = AMS_EKF_PACK_PARALLEL_CELLS;
    cfg->cell_capacity_Ah = AMS_EKF_CELL_CAPACITY_AH;
    cfg->sample_time_s = AMS_EKF_DEFAULT_DT_S;
    cfg->soc_init = AMS_EKF_DEFAULT_SOC_INIT;
    cfg->r0_init_ohm = AMS_EKF_DEFAULT_R0_INIT_OHM;
}

void ams_ekf_make_segment_config(ams_ekf_config_t *cfg, uint8_t segment_index)
{
    ams_ekf_make_group_range_config(cfg, (uint16_t)segment_index * 15U, 15U);
}

void ams_ekf_make_group_range_config(ams_ekf_config_t *cfg,
                                     uint16_t first_series_group,
                                     uint16_t series_group_count)
{
    if (cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->enabled = 1U;
    cfg->first_series_group = first_series_group;
    cfg->series_group_count = series_group_count;
    cfg->parallel_cell_count = AMS_EKF_PACK_PARALLEL_CELLS;
    cfg->cell_capacity_Ah = AMS_EKF_CELL_CAPACITY_AH;
    cfg->sample_time_s = AMS_EKF_DEFAULT_DT_S;
    cfg->soc_init = AMS_EKF_DEFAULT_SOC_INIT;
    cfg->r0_init_ohm = AMS_EKF_DEFAULT_R0_INIT_OHM;
}

static bool config_valid(const ams_ekf_config_t *cfg)
{
    if ((cfg == NULL) || (cfg->enabled == 0U))
    {
        return false;
    }

    if ((cfg->series_group_count == 0U) ||
        ((uint32_t)cfg->first_series_group + (uint32_t)cfg->series_group_count > AMS_EKF_PACK_SERIES_GROUPS))
    {
        return false;
    }

    if ((!isfinite(cfg->parallel_cell_count)) || (cfg->parallel_cell_count < 1.0f))
    {
        return false;
    }

    if ((!isfinite(cfg->cell_capacity_Ah)) || (cfg->cell_capacity_Ah <= 0.0f))
    {
        return false;
    }

    return true;
}

void ams_ekf_init(ams_ekf_instance_t *ekf, const ams_ekf_config_t *cfg)
{
    if (ekf == NULL)
    {
        return;
    }

    memset(ekf, 0, sizeof(*ekf));

    if (!config_valid(cfg))
    {
        ekf->fault_flags = AMS_EKF_FAULT_DISABLED | AMS_EKF_FAULT_BAD_CONFIG;
        return;
    }

    ekf->cfg = *cfg;
    ekf->soc = clampf_local(cfg->soc_init, 0.0f, 1.0f);
    ekf->vp1_V = 0.0f;
    ekf->vp2_V = 0.0f;
    ekf->r0_ohm = cfg->r0_init_ohm;
    if ((!isfinite(ekf->r0_ohm)) || (ekf->r0_ohm <= 0.0f))
    {
        ekf->r0_ohm = ams_p42a_r0_ohm(ekf->soc, 25.0f);
    }
    ekf->r0_ohm = clampf_local(ekf->r0_ohm, AMS_EKF_R0_MIN_OHM, AMS_EKF_R0_MAX_OHM);

    ekf->t_core_C = 25.0f;
    ekf->p_soc = AMS_EKF_P0_SOC;
    ekf->p_soc_vp1 = 0.0f;
    ekf->p_soc_vp2 = 0.0f;
    ekf->p_vp1 = AMS_EKF_P0_VP1;
    ekf->p_vp1_vp2 = 0.0f;
    ekf->p_vp2 = AMS_EKF_P0_VP2;
    ekf->p_r0 = AMS_EKF_P0_R0;
    ekf->r_meas_V2 = measurement_r_init_v2((float)cfg->series_group_count);

    for (uint8_t i = 0U; i < AMS_EKF_ADAPT_WIN; i++)
    {
        ekf->innov_hist[i] = ekf->r_meas_V2;
    }

    ekf->innovation_variance_V2 = NAN;
    ekf->valid = 0U;
    ekf->fault_flags = AMS_EKF_FAULT_NONE;
    ekf->model_domain_flags = AMS_EKF_MODEL_DOMAIN_NONE;
    acquisition_init(&ekf->acquisition);
}

static void update_adaptive_r(ams_ekf_instance_t *ekf,
                              float innovation_V,
                              float series_count,
                              float temp_c)
{
    ekf->innov_hist[ekf->innov_idx] = innovation_V * innovation_V;
    ekf->innov_idx = (uint8_t)((ekf->innov_idx + 1U) % AMS_EKF_ADAPT_WIN);

    float sum = 0.0f;
    for (uint8_t i = 0U; i < AMS_EKF_ADAPT_WIN; i++)
    {
        sum += ekf->innov_hist[i];
    }

    ekf->r_meas_V2 = sum / (float)AMS_EKF_ADAPT_WIN;
    const float r_floor = measurement_r_floor_v2(series_count, temp_c);
    if (ekf->r_meas_V2 < r_floor)
    {
        ekf->r_meas_V2 = r_floor;
    }
    else if (ekf->r_meas_V2 > 25.0f)
    {
        ekf->r_meas_V2 = 25.0f;
    }
}

static bool ams_ekf_step_impl(ams_ekf_instance_t *ekf,
                              float i_pack_A,
                              float v_meas_V,
                              float t_surf_C,
                              float dt_s,
                              bool allow_r0_update,
                              bool acquisition_constrained,
                              ams_ekf_r0_update_result_t *r0_result)
{
    if(r0_result != NULL)
    {
        *r0_result = AMS_EKF_R0_UPDATE_NOT_REQUESTED;
    }
    if ((ekf == NULL) || (ekf->cfg.enabled == 0U))
    {
        return false;
    }

    ekf->fault_flags = AMS_EKF_FAULT_NONE;
    ekf->model_domain_flags = AMS_EKF_MODEL_DOMAIN_NONE;

    if (!config_valid(&ekf->cfg))
    {
        ekf->fault_flags |= AMS_EKF_FAULT_BAD_CONFIG;
        ekf->valid = 0U;
        return false;
    }

    if (!finite3(i_pack_A, v_meas_V, t_surf_C))
    {
        ekf->fault_flags |= AMS_EKF_FAULT_BAD_INPUT;
        ekf->valid = 0U;
        return false;
    }

    float series_count = (float)ekf->cfg.series_group_count;
    float v_min = series_count * 2.0f;
    float v_max = series_count * 4.5f;
    if ((v_meas_V < v_min) || (v_meas_V > v_max))
    {
        ekf->fault_flags |= AMS_EKF_FAULT_BAD_VOLTAGE;
        ekf->valid = 0U;
        return false;
    }

    if (fabsf(i_pack_A) > 1500.0f)
    {
        ekf->fault_flags |= AMS_EKF_FAULT_BAD_CURRENT;
        ekf->valid = 0U;
        return false;
    }

    if ((t_surf_C < -40.0f) || (t_surf_C > 120.0f))
    {
        ekf->fault_flags |= AMS_EKF_FAULT_BAD_TEMP;
        ekf->valid = 0U;
        return false;
    }

    float dt = dt_s;
    bool dt_clamped = false;
    if ((!isfinite(dt)) || (dt <= 0.0f))
    {
        dt = ekf->cfg.sample_time_s;
        dt_clamped = true;
    }
    if(dt < 0.001f)
    {
        dt = 0.001f;
        dt_clamped = true;
    }
    else if(dt > 1.0f)
    {
        dt = 1.0f;
        dt_clamped = true;
    }
    if(dt_clamped)
    {
        ekf->fault_flags |= AMS_EKF_FAULT_DT_CLAMPED;
        saturating_increment(&ekf->dt_clamp_count);
    }

    if(ekf->t_core_C < 5.0f)
    {
        ekf->model_domain_flags |= AMS_EKF_MODEL_DOMAIN_TEMP_LOW;
    }
    else if(ekf->t_core_C > 40.0f)
    {
        ekf->model_domain_flags |= AMS_EKF_MODEL_DOMAIN_TEMP_HIGH;
    }
    float t_lut_C = clampf_local(ekf->t_core_C, 5.0f, 40.0f);
    /* Confidence floors must react to the temperature we actually measure
     * at this epoch. The core observer intentionally starts at 25 C and can
     * lag a cold/hot pack, so using t_core here would suppress the very edge
     * uncertainty inflation intended to cover that model mismatch. */
    const float uncertainty_temp_C = clampf_local(t_surf_C, 5.0f, 40.0f);
    float soc0 = clampf_local(ekf->soc, 0.0f, 1.0f);
    float i_cell_A = i_pack_A / ekf->cfg.parallel_cell_count;

    float inv_c1 = ams_p42a_inv_c1(soc0, t_lut_C);
    float neg_inv_tau1 = ams_p42a_neg_inv_tau1(soc0, t_lut_C);

    float q_nom_inv = 1.0f / (3600.0f * ekf->cfg.cell_capacity_Ah);
    float soc_p = soc0 - (q_nom_inv * i_cell_A * dt);
    soc_p = clampf_local(soc_p, 0.0f, 1.0f);

    float vp1_p = ekf->vp1_V + ((inv_c1 * i_cell_A) + (neg_inv_tau1 * ekf->vp1_V)) * dt;
    float vp2_p = ekf->vp2_V + ((AMS_EKF_INV_C2 * i_cell_A) - (AMS_EKF_INV_TAU2 * ekf->vp2_V)) * dt;

    float f_vp1 = 1.0f + (neg_inv_tau1 * dt);
    float f_vp2 = 1.0f - (AMS_EKF_INV_TAU2 * dt);

    float p_prior[3][3];
    float p_pred[3][3] = {{0.0f}};
    covariance_load(ekf, p_prior);
    p_pred[0][0] = p_prior[0][0] + AMS_EKF_Q_SOC;
    p_pred[0][1] = p_pred[1][0] = f_vp1 * p_prior[0][1];
    p_pred[0][2] = p_pred[2][0] = f_vp2 * p_prior[0][2];
    p_pred[1][1] = (f_vp1 * f_vp1 * p_prior[1][1]) + AMS_EKF_Q_VP1;
    p_pred[1][2] = p_pred[2][1] =
        f_vp1 * f_vp2 * p_prior[1][2];
    p_pred[2][2] = (f_vp2 * f_vp2 * p_prior[2][2]) + AMS_EKF_Q_VP2;
    ekf->p_r0 += AMS_EKF_Q_R0;

    if(acquisition_constrained)
    {
        allow_r0_update = false;
        saturating_increment(&ekf->acquisition.dynamic_step_count);
    }
    if(!covariance_sanitize(ekf, p_pred, uncertainty_temp_C,
                            acquisition_constrained))
    {
        ekf->fault_flags |= AMS_EKF_FAULT_COVARIANCE;
        ekf->valid = 0u;
        return false;
    }

    float v_est = series_count * (ams_p42a_ocv_v(soc_p, t_lut_C) -
                                  (ekf->r0_ohm * i_cell_A) -
                                  vp1_p - vp2_p);
    float innovation = v_meas_V - v_est;

    /* Gate the measurement update against the PRIOR measurement covariance.
     * A bad-but-in-range sample must not first inflate adaptive R and only
     * protect the following sample.  Rejected samples are predict-only and
     * do not enter the adaptive-R history. */
    float r_meas = fmaxf(ekf->r_meas_V2,
                         measurement_r_floor_v2(series_count, uncertainty_temp_C));
    ekf->r_meas_V2 = r_meas;
    float s_hi = clampf_local(soc_p + AMS_EKF_DSOC, 0.0f, 1.0f);
    float s_lo = clampf_local(soc_p - AMS_EKF_DSOC, 0.0f, 1.0f);
    float d_ocv = (ams_p42a_ocv_v(s_hi, t_lut_C) - ams_p42a_ocv_v(s_lo, t_lut_C)) /
                  ((s_hi - s_lo) + 1.0e-10f);

    float h_soc = series_count * d_ocv;
    float h_vp1 = -series_count;
    float h_vp2 = -series_count;

    const float h[3] = {h_soc, h_vp1, h_vp2};
    const float vp_nuisance_V2 =
        (h_vp1 * h_vp1 * p_pred[1][1]) +
        (h_vp2 * h_vp2 * p_pred[2][2]) +
        (2.0f * h_vp1 * h_vp2 * p_pred[1][2]);
    float s_x;
    if(acquisition_constrained)
    {
        s_x = (h_soc * h_soc * p_pred[0][0]) +
              vp_nuisance_V2 + r_meas;
    }
    else
    {
        s_x = covariance_innovation_variance(p_pred, h, r_meas);
    }
    ekf->innovation_variance_V2 = s_x;
    float innovation_per_cell = fabsf(innovation) / series_count;
    const float innovation_gate_sigma = acquisition_constrained ?
        AMS_EKF_ACQ_DYNAMIC_GATE_SIGMA : 6.0f;
    const bool bootstrap_step = ekf->step_count < AMS_EKF_ACQUISITION_STEPS;
    bool innovation_rejected = (!isfinite(s_x)) || (s_x <= 1.0e-10f) ||
        (!isfinite(innovation_per_cell));

    if(!innovation_rejected)
    {
        if(acquisition_constrained)
        {
            innovation_rejected =
                (innovation_per_cell > AMS_EKF_BOOTSTRAP_MAX_INNOVATION_PER_CELL_V) ||
                ((innovation * innovation) >
                 ((innovation_gate_sigma * innovation_gate_sigma) * s_x));
        }
        else if(bootstrap_step)
        {
            /* The configured initial SoC is deliberately conservative and
             * can be several hundred mV/cell away from a healthy partially
             * charged pack.  Permit a short bounded acquisition window, but do
             * not let an absurd yet input-window-valid sample (for example a
             * contactor-bounce collapse near 2 V/cell) seed the estimator. */
            innovation_rejected =
                innovation_per_cell >
                AMS_EKF_BOOTSTRAP_MAX_INNOVATION_PER_CELL_V;
        }
        else
        {
            innovation_rejected =
                (innovation_per_cell > AMS_SOH_MAX_INNOVATION_PER_CELL_V) ||
                ((innovation * innovation) >
                 ((innovation_gate_sigma * innovation_gate_sigma) * s_x));
        }
    }

    if(innovation_rejected)
    {
        ekf->fault_flags |= AMS_EKF_FAULT_INNOVATION_REJECT;
        saturating_increment(&ekf->innovation_reject_count);
        if(allow_r0_update && (r0_result != NULL))
        {
            *r0_result = AMS_EKF_R0_UPDATE_REJECT_INNOVATION;
        }

        /* Predict-only: preserve physical evolution without allowing the
         * rejected measurement to move SoC/polarization/covariance. */
        ekf->soc = soc_p;
        ekf->vp1_V = vp1_p;
        ekf->vp2_V = vp2_p;
        covariance_store(ekf, p_pred);
    }
    else
    {
        if(allow_r0_update)
        {
            float h_r0 = -series_count * i_cell_A;
            float s_r0 = (h_r0 * h_r0 * ekf->p_r0) + r_meas;
            float k_r0 = (s_r0 > 1.0e-10f) ?
                         (ekf->p_r0 * h_r0 / s_r0) : NAN;
            float r0_candidate = ekf->r0_ohm + (k_r0 * innovation);

            if(!isfinite(k_r0) || !isfinite(r0_candidate))
            {
                if(r0_result != NULL)
                {
                    *r0_result = AMS_EKF_R0_UPDATE_REJECT_NUMERIC;
                }
            }
            else
            {
                float a_r0 = 1.0f - (k_r0 * h_r0);
                ekf->p_r0 = (a_r0 * a_r0 * ekf->p_r0) +
                            (k_r0 * k_r0 * r_meas);
                ekf->r0_ohm = clampf_local(r0_candidate,
                                           AMS_EKF_R0_MIN_OHM,
                                           AMS_EKF_R0_MAX_OHM);
                if(ekf->r0_ohm != r0_candidate)
                {
                    ekf->fault_flags |= AMS_EKF_FAULT_CLAMPED;
                    if(r0_result != NULL)
                    {
                        *r0_result = AMS_EKF_R0_UPDATE_CLAMPED;
                    }
                }
                else if(r0_result != NULL)
                {
                    *r0_result = AMS_EKF_R0_UPDATE_APPLIED;
                }
            }
        }

        float inv_s = 1.0f / s_x;

        if(acquisition_constrained)
        {
            const float k_soc = p_pred[0][0] * h_soc * inv_s;
            float raw_soc_step = k_soc * innovation;
            float soc_step = clampf_local(raw_soc_step,
                                          -AMS_EKF_ACQ_DYNAMIC_MAX_SOC_STEP,
                                           AMS_EKF_ACQ_DYNAMIC_MAX_SOC_STEP);
            ekf->soc = clampf_local(soc_p + soc_step, 0.0f, 1.0f);
            ekf->vp1_V = vp1_p;
            ekf->vp2_V = vp2_p;

            float k_effective = 0.0f;
            if(fabsf(innovation) > 1.0e-12f)
            {
                k_effective = soc_step / innovation;
            }
            const float a_soc = 1.0f - (k_effective * h_soc);
            const float r_nuisance = vp_nuisance_V2 + r_meas;
            float p_post[3][3];
            memcpy(p_post, p_pred, sizeof(p_post));
            p_post[0][0] = (a_soc * a_soc * p_pred[0][0]) +
                           (k_effective * k_effective * r_nuisance);
            p_post[0][1] = p_post[1][0] = 0.0f;
            p_post[0][2] = p_post[2][0] = 0.0f;
            p_post[1][2] = p_post[2][1] = 0.0f;
            if(!covariance_sanitize(ekf, p_post, uncertainty_temp_C, true))
            {
                ekf->fault_flags |= AMS_EKF_FAULT_COVARIANCE;
                ekf->valid = 0u;
                return false;
            }
            covariance_store(ekf, p_post);
            saturating_increment(&ekf->acquisition.dynamic_update_count);
        }
        else
        {
            float gain[3] = {0.0f, 0.0f, 0.0f};
            for(uint8_t row = 0u; row < 3u; row++)
            {
                gain[row] = ((p_pred[row][0] * h[0]) +
                             (p_pred[row][1] * h[1]) +
                             (p_pred[row][2] * h[2])) * inv_s;
            }

            ekf->soc = clampf_local(soc_p + (gain[0] * innovation),
                                     0.0f, 1.0f);
            ekf->vp1_V = vp1_p + (gain[1] * innovation);
            ekf->vp2_V = vp2_p + (gain[2] * innovation);

            float p_post[3][3] = {{0.0f}};
            if(!covariance_joseph_update(p_pred, gain, h, r_meas, p_post) ||
               !covariance_sanitize(ekf, p_post, uncertainty_temp_C, false))
            {
                ekf->fault_flags |= AMS_EKF_FAULT_COVARIANCE;
                ekf->valid = 0u;
                return false;
            }
            covariance_store(ekf, p_post);

            /* Adapt for the NEXT sample only after this innovation passed the
             * current sample's authority gate. Startup acquisition residuals
             * are deliberately excluded from this history. */
            update_adaptive_r(ekf, innovation, series_count, uncertainty_temp_C);
        }
    }

    float p_final[3][3];
    covariance_load(ekf, p_final);
    if(!covariance_sanitize(ekf, p_final, uncertainty_temp_C,
                            acquisition_constrained))
    {
        ekf->fault_flags |= AMS_EKF_FAULT_COVARIANCE;
        ekf->valid = 0u;
        return false;
    }
    covariance_store(ekf, p_final);
    if (ekf->p_r0  < 1.0e-14f) { ekf->p_r0  = 1.0e-14f; }
    if (ekf->p_r0  > AMS_EKF_P_R0_MAX) { ekf->p_r0 = AMS_EKF_P_R0_MAX; }

    float inv_r1 = ams_p42a_inv_r1_from_luts(inv_c1, neg_inv_tau1);
    float q_gen = (i_cell_A * i_cell_A * ekf->r0_ohm) +
                  (vp1_p * vp1_p * inv_r1) +
                  (vp2_p * vp2_p * AMS_EKF_INV_R2);
    float q_cs = (ekf->t_core_C - t_surf_C) * AMS_EKF_INV_RCS;
    ekf->t_core_C += (q_gen - q_cs) * AMS_EKF_INV_CC * dt;
    float core_before_clamp = ekf->t_core_C;
    ekf->t_core_C = clampf_local(ekf->t_core_C, -10.0f, 60.0f);
    if(ekf->t_core_C != core_before_clamp)
    {
        ekf->model_domain_flags |= AMS_EKF_MODEL_DOMAIN_CORE_CLAMP;
    }

    ekf->v_pred_V = v_est;
    ekf->innovation_V = innovation;
    ekf->last_i_pack_A = i_pack_A;
    ekf->last_v_meas_V = v_meas_V;
    ekf->last_t_surf_C = t_surf_C;
    saturating_increment(&ekf->step_count);
    ekf->valid = 1U;

    return true;
}

bool ams_ekf_step_gated(ams_ekf_instance_t *ekf,
                        float i_pack_A,
                        float v_meas_V,
                        float t_surf_C,
                        float dt_s,
                        bool allow_r0_update,
                        ams_ekf_r0_update_result_t *r0_result)
{
    return ams_ekf_step_impl(ekf,
                             i_pack_A,
                             v_meas_V,
                             t_surf_C,
                             dt_s,
                             allow_r0_update,
                             false,
                             r0_result);
}

bool ams_ekf_step_acquiring_gated(ams_ekf_instance_t *ekf,
                                  float i_pack_A,
                                  float v_meas_V,
                                  float t_surf_C,
                                  float dt_s,
                                  ams_ekf_r0_update_result_t *r0_result)
{
    return ams_ekf_step_impl(ekf,
                             i_pack_A,
                             v_meas_V,
                             t_surf_C,
                             dt_s,
                             false,
                             true,
                             r0_result);
}

bool ams_ekf_step(ams_ekf_instance_t *ekf,
                  float i_pack_A,
                  float v_meas_V,
                  float t_surf_C,
                  float dt_s)
{
    return ams_ekf_step_gated(ekf,
                              i_pack_A,
                              v_meas_V,
                              t_surf_C,
                              dt_s,
                              true,
                              NULL);
}

bool ams_ekf_acquisition_complete(const ams_ekf_instance_t *ekf)
{
    return (ekf != NULL) &&
           (ekf->acquisition.state == AMS_EKF_ACQ_COMPLETE);
}

void ams_ekf_acquisition_observe(ams_ekf_instance_t *ekf,
                                 float i_pack_A,
                                 float current_uncertainty_A,
                                 float v_meas_V,
                                 float t_surf_C,
                                 uint32_t measurement_tick,
                                 bool current_trusted,
                                 bool measurement_valid)
{
    if((ekf == NULL) || (ekf->cfg.enabled == 0u) ||
       (ekf->acquisition.state == AMS_EKF_ACQ_COMPLETE))
    {
        return;
    }

    ams_ekf_acquisition_t *acq = &ekf->acquisition;
    const bool values_finite = finite3(i_pack_A, v_meas_V, t_surf_C) &&
                               isfinite(current_uncertainty_A) &&
                               (current_uncertainty_A >= 0.0f);
    const float current_abs_A = values_finite ?
        (fabsf(i_pack_A) + current_uncertainty_A) : INFINITY;

    /* A candidate that is waiting for pack consensus remains valid only while
     * the same low-current/valid epoch continues. If the operating condition
     * changes before consensus is possible, discard it rather than applying a
     * stale relaxation estimate later. */
    if(acq->candidate_ready != 0u)
    {
        if(!measurement_valid || !current_trusted || !values_finite ||
           (current_abs_A > AMS_EKF_ACQ_CURRENT_ABORT_A))
        {
            saturating_increment_u8(&acq->reject_count);
            acquisition_window_clear(
                acq, AMS_EKF_ACQ_REASON_RELAXATION_INTERRUPTED_RETRY);
        }
        return;
    }

    if(!measurement_valid || !current_trusted || !values_finite)
    {
        if(acq->state == AMS_EKF_ACQ_COLLECTING)
        {
            saturating_increment_u8(&acq->reject_count);
        }
        acquisition_window_clear(
            acq, AMS_EKF_ACQ_REASON_WAITING_FOR_LOW_CURRENT);
        return;
    }

    if(acq->state == AMS_EKF_ACQ_WAITING)
    {
        if(current_abs_A > AMS_EKF_ACQ_CURRENT_ENTER_A)
        {
            acq->reason = AMS_EKF_ACQ_REASON_WAITING_FOR_LOW_CURRENT;
            return;
        }

        acquisition_window_clear(acq,
                                 AMS_EKF_ACQ_REASON_COLLECTING_RELAXATION);
        acq->state = AMS_EKF_ACQ_COLLECTING;
        acq->reason = AMS_EKF_ACQ_REASON_COLLECTING_RELAXATION;
        acq->start_tick = measurement_tick;
        acq->last_sample_tick = measurement_tick;
        acquisition_add_sample(ekf, v_meas_V, t_surf_C, measurement_tick);
    }
    else if(current_abs_A > AMS_EKF_ACQ_CURRENT_ABORT_A)
    {
        saturating_increment_u8(&acq->reject_count);
        acquisition_window_clear(
            acq, AMS_EKF_ACQ_REASON_RELAXATION_INTERRUPTED_RETRY);
        return;
    }
    else if((uint32_t)(measurement_tick - acq->last_sample_tick) >=
            AMS_EKF_ACQ_SAMPLE_PERIOD_MS)
    {
        acquisition_add_sample(ekf, v_meas_V, t_surf_C, measurement_tick);
    }

    if(acq->state != AMS_EKF_ACQ_COLLECTING)
    {
        return;
    }

    if((uint32_t)(measurement_tick - acq->start_tick) < AMS_EKF_ACQ_WINDOW_MS)
    {
        return;
    }

    const ams_ekf_acquisition_reason_t result =
        acquisition_build_candidate(ekf);
    if(acq->candidate_ready == 0u)
    {
        saturating_increment_u8(&acq->reject_count);
        acquisition_window_clear(acq, result);
    }
}

static void sort_small_float(float *values, uint8_t count)
{
    if(values == NULL)
    {
        return;
    }
    for(uint8_t i = 1u; i < count; i++)
    {
        const float key = values[i];
        uint8_t j = i;
        while((j > 0u) && (values[j - 1u] > key))
        {
            values[j] = values[j - 1u];
            j--;
        }
        values[j] = key;
    }
}

static float acquisition_consensus_median(const ams_estimator_t *est,
                                          uint8_t *pool_count)
{
    float pool[AMS_EKF_MAX_INSTANCES];
    uint8_t count = 0u;
    if(est == NULL)
    {
        if(pool_count != NULL)
        {
            *pool_count = 0u;
        }
        return NAN;
    }

    for(uint8_t i = 0u;
        (i < est->instance_count) && (i < AMS_EKF_MAX_INSTANCES);
        i++)
    {
        const ams_ekf_instance_t *inst = &est->inst[i];
        if(inst->cfg.enabled == 0u)
        {
            continue;
        }

        float candidate = NAN;
        if(inst->acquisition.candidate_ready != 0u)
        {
            candidate = inst->acquisition.candidate_soc;
        }
        else if((inst->acquisition.state == AMS_EKF_ACQ_COMPLETE) &&
                (inst->valid != 0u))
        {
            /* Use the live propagated SoC of already acquired peers.  Reusing
             * the old anchor value after current has flowed would make a late
             * retry compare against stale pack state. */
            candidate = inst->soc;
        }

        if(isfinite(candidate) && (count < AMS_EKF_MAX_INSTANCES))
        {
            pool[count++] = candidate;
        }
    }

    if(pool_count != NULL)
    {
        *pool_count = count;
    }
    if(count == 0u)
    {
        return NAN;
    }

    sort_small_float(pool, count);
    if((count & 1u) != 0u)
    {
        return pool[count / 2u];
    }
    return 0.5f * (pool[(count / 2u) - 1u] + pool[count / 2u]);
}

static void acquisition_apply_anchor(ams_ekf_instance_t *ekf,
                                     float consensus_soc)
{
    if((ekf == NULL) || (ekf->acquisition.candidate_ready == 0u))
    {
        return;
    }

    ams_ekf_acquisition_t *acq = &ekf->acquisition;
    float correction = acq->candidate_soc - ekf->soc;
    correction = clampf_local(correction,
                              -AMS_EKF_ACQ_MAX_SOC_CORRECTION,
                               AMS_EKF_ACQ_MAX_SOC_CORRECTION);
    ekf->soc = clampf_local(ekf->soc + correction, 0.0f, 1.0f);
    ekf->vp1_V = acq->candidate_vp1_finish_V;
    ekf->vp2_V = acq->candidate_vp2_finish_V;

    ekf->p_soc = AMS_EKF_ACQ_SOC_SIGMA_INIT * AMS_EKF_ACQ_SOC_SIGMA_INIT;
    ekf->p_soc_vp1 = 0.0f;
    ekf->p_soc_vp2 = 0.0f;
    ekf->p_vp1 = AMS_EKF_ACQ_VP1_SIGMA_INIT_V *
                 AMS_EKF_ACQ_VP1_SIGMA_INIT_V;
    ekf->p_vp1_vp2 = 0.0f;
    ekf->p_vp2 = AMS_EKF_ACQ_VP2_SIGMA_INIT_V *
                 AMS_EKF_ACQ_VP2_SIGMA_INIT_V;
    ekf->p_r0 = AMS_EKF_P0_R0;

    const float t_lut = clampf_local(ekf->t_core_C, 5.0f, 40.0f);
    ekf->r0_ohm = clampf_local(ams_p42a_r0_ohm(ekf->soc, t_lut),
                               AMS_EKF_R0_MIN_OHM,
                               AMS_EKF_R0_MAX_OHM);
    ekf->r_meas_V2 = measurement_r_init_v2(
        (float)ekf->cfg.series_group_count);
    ekf->innov_idx = 0u;
    for(uint8_t i = 0u; i < AMS_EKF_ADAPT_WIN; i++)
    {
        ekf->innov_hist[i] = ekf->r_meas_V2;
    }

    acq->state = AMS_EKF_ACQ_COMPLETE;
    acq->reason = AMS_EKF_ACQ_REASON_FIXED_BASIS_ANCHOR;
    acq->candidate_ready = 0u;
    acq->consensus_soc = consensus_soc;
    saturating_increment(&acq->anchor_count);
}

void ams_estimator_acquisition_resolve(ams_estimator_t *est)
{
    if((est == NULL) || (est->instance_count == 0u))
    {
        return;
    }

    uint8_t enabled_count = 0u;
    uint8_t ready_count = 0u;
    for(uint8_t i = 0u;
        (i < est->instance_count) && (i < AMS_EKF_MAX_INSTANCES);
        i++)
    {
        if(est->inst[i].cfg.enabled != 0u)
        {
            enabled_count++;
            if(est->inst[i].acquisition.candidate_ready != 0u)
            {
                ready_count++;
            }
        }
    }
    if(ready_count == 0u)
    {
        return;
    }

    uint8_t pool_count = 0u;
    const float consensus = acquisition_consensus_median(est, &pool_count);
    uint8_t required = AMS_EKF_ACQ_MIN_CONSENSUS_SEGMENTS;
    if(enabled_count < required)
    {
        required = enabled_count;
    }
    if(required == 0u)
    {
        required = 1u;
    }

    if(!isfinite(consensus) || (pool_count < required))
    {
        /* Keep ready candidates pending rather than throwing away a good fit
         * merely because another valid segment finishes a few epochs later.
         * Observe() will invalidate them if low-current/validity conditions
         * cease before consensus becomes available. */
        for(uint8_t i = 0u;
            (i < est->instance_count) && (i < AMS_EKF_MAX_INSTANCES);
            i++)
        {
            if(est->inst[i].acquisition.candidate_ready != 0u)
            {
                est->inst[i].acquisition.reason =
                    AMS_EKF_ACQ_REASON_INSUFFICIENT_CONSENSUS_RETRY;
            }
        }
        return;
    }

    for(uint8_t i = 0u;
        (i < est->instance_count) && (i < AMS_EKF_MAX_INSTANCES);
        i++)
    {
        ams_ekf_instance_t *inst = &est->inst[i];
        ams_ekf_acquisition_t *acq = &inst->acquisition;
        if(acq->candidate_ready == 0u)
        {
            continue;
        }

        acq->consensus_soc = consensus;
        if(fabsf(acq->candidate_soc - consensus) <=
           AMS_EKF_ACQ_CONSENSUS_SOC_DEVIATION)
        {
            acquisition_apply_anchor(inst, consensus);
        }
        else
        {
            saturating_increment_u8(&acq->reject_count);
            acquisition_window_clear(
                acq, AMS_EKF_ACQ_REASON_SEGMENT_CONSENSUS_RETRY);
            acq->consensus_soc = consensus;
        }
    }
}

uint32_t ams_resistance_soh_gate(const ams_ekf_instance_t *ekf,
                                 float i_pack_A,
                                 bool epoch_coherent,
                                 bool balance_recovered,
                                 bool current_calibration_confident)
{
    uint32_t reject = AMS_SOH_REJECT_NONE;

    if(!epoch_coherent)
    {
        reject |= AMS_SOH_REJECT_EPOCH;
    }
    if(!balance_recovered)
    {
        reject |= AMS_SOH_REJECT_BALANCE_RECOVERY;
    }
    if(!current_calibration_confident)
    {
        reject |= AMS_SOH_REJECT_CURRENT_CALIBRATION;
    }
    if((ekf == NULL) || (ekf->cfg.enabled == 0u) ||
       !config_valid((ekf != NULL) ? &ekf->cfg : NULL) ||
       !isfinite(i_pack_A))
    {
        return reject | AMS_SOH_REJECT_ESTIMATOR;
    }
    if(fabsf(i_pack_A) < AMS_SOH_MIN_PACK_CURRENT_A)
    {
        reject |= AMS_SOH_REJECT_LOW_CURRENT;
    }
    if((ekf->step_count == 0u) || !isfinite(ekf->last_i_pack_A) ||
       (fabsf(i_pack_A - ekf->last_i_pack_A) <
        AMS_SOH_MIN_PACK_CURRENT_STEP_A))
    {
        reject |= AMS_SOH_REJECT_LOW_CURRENT_STEP;
    }
    if(!isfinite(ekf->soc) || !isfinite(ekf->t_core_C) ||
       (ekf->soc < AMS_SOH_MIN_SOC) || (ekf->soc > AMS_SOH_MAX_SOC) ||
       (ekf->t_core_C < AMS_SOH_MIN_MODEL_TEMP_C) ||
       (ekf->t_core_C > AMS_SOH_MAX_MODEL_TEMP_C) ||
       (ekf->model_domain_flags != AMS_EKF_MODEL_DOMAIN_NONE))
    {
        reject |= AMS_SOH_REJECT_MODEL_DOMAIN;
    }

    return reject;
}

static void resistance_soh_count_reasons(ams_resistance_soh_t *soh,
                                         uint32_t flags)
{
    if((flags & AMS_SOH_REJECT_EPOCH) != 0u)
    {
        saturating_increment(&soh->reject_epoch_count);
    }
    if((flags & AMS_SOH_REJECT_CURRENT_CALIBRATION) != 0u)
    {
        saturating_increment(&soh->reject_current_calibration_count);
    }
    if((flags & AMS_SOH_REJECT_LOW_CURRENT) != 0u)
    {
        saturating_increment(&soh->reject_low_current_count);
    }
    if((flags & AMS_SOH_REJECT_LOW_CURRENT_STEP) != 0u)
    {
        saturating_increment(&soh->reject_low_current_step_count);
    }
    if((flags & AMS_SOH_REJECT_BALANCE_RECOVERY) != 0u)
    {
        saturating_increment(&soh->reject_balance_recovery_count);
    }
    if((flags & AMS_SOH_REJECT_MODEL_DOMAIN) != 0u)
    {
        saturating_increment(&soh->reject_model_domain_count);
    }
    if((flags & AMS_SOH_REJECT_INNOVATION) != 0u)
    {
        saturating_increment(&soh->reject_innovation_count);
    }
    if((flags & AMS_SOH_REJECT_R0_CLAMP) != 0u)
    {
        saturating_increment(&soh->reject_r0_clamp_count);
    }
    if((flags & AMS_SOH_REJECT_NUMERIC) != 0u)
    {
        saturating_increment(&soh->reject_numeric_count);
    }
    if((flags & AMS_SOH_REJECT_ESTIMATOR) != 0u)
    {
        saturating_increment(&soh->reject_estimator_count);
    }
    if((flags & AMS_SOH_REJECT_ACQUISITION) != 0u)
    {
        saturating_increment(&soh->reject_acquisition_count);
    }
}

void ams_resistance_soh_record(ams_resistance_soh_t *soh,
                               const ams_ekf_instance_t *ekf,
                               uint32_t measurement_sequence,
                               uint32_t tick,
                               bool current_calibration_confident,
                               uint32_t precheck_reject_flags,
                               ams_ekf_r0_update_result_t r0_result,
                               bool estimator_step_ok)
{
    if(soh == NULL)
    {
        return;
    }

    uint32_t reject = precheck_reject_flags;
    if(!estimator_step_ok || (ekf == NULL) || (ekf->valid == 0u))
    {
        reject |= AMS_SOH_REJECT_ESTIMATOR;
    }
    switch(r0_result)
    {
    case AMS_EKF_R0_UPDATE_REJECT_INNOVATION:
        reject |= AMS_SOH_REJECT_INNOVATION;
        break;
    case AMS_EKF_R0_UPDATE_REJECT_NUMERIC:
        reject |= AMS_SOH_REJECT_NUMERIC;
        break;
    case AMS_EKF_R0_UPDATE_CLAMPED:
        reject |= AMS_SOH_REJECT_R0_CLAMP;
        break;
    case AMS_EKF_R0_UPDATE_NOT_REQUESTED:
        if(reject == AMS_SOH_REJECT_NONE)
        {
            reject |= AMS_SOH_REJECT_NUMERIC;
        }
        break;
    case AMS_EKF_R0_UPDATE_APPLIED:
    default:
        break;
    }

    soh->last_measurement_sequence = measurement_sequence;
    soh->last_observation_tick = tick;
    soh->last_reject_flags = reject;
    soh->status_flags = 0u;
    if(current_calibration_confident)
    {
        soh->status_flags |= AMS_SOH_STATUS_CALIBRATION_CONFIDENT;
    }

    /* Never retain a plausible value from an older record when the current
     * estimator object cannot produce a finite reference. */
    soh->estimated_cell_r0_ohm = NAN;
    soh->reference_cell_r0_ohm = NAN;
    soh->resistance_growth_ratio = NAN;
    soh->r0_variance_ohm2 = NAN;

    if(ekf != NULL)
    {
        soh->estimated_cell_r0_ohm = ekf->r0_ohm;
        soh->r0_variance_ohm2 = ekf->p_r0;
        if(isfinite(ekf->soc) && isfinite(ekf->t_core_C))
        {
            float t_ref_C = clampf_local(ekf->t_core_C,
                                         AMS_SOH_MIN_MODEL_TEMP_C,
                                         AMS_SOH_MAX_MODEL_TEMP_C);
            soh->reference_cell_r0_ohm =
                ams_p42a_r0_ohm(clampf_local(ekf->soc, 0.0f, 1.0f),
                                 t_ref_C);
            if(isfinite(soh->reference_cell_r0_ohm) &&
               (soh->reference_cell_r0_ohm > 0.0f))
            {
                soh->resistance_growth_ratio =
                    soh->estimated_cell_r0_ohm /
                    soh->reference_cell_r0_ohm;
            }
        }
    }

    if((reject == AMS_SOH_REJECT_NONE) &&
       (r0_result == AMS_EKF_R0_UPDATE_APPLIED))
    {
        saturating_increment(&soh->accepted_count);
        soh->last_accept_tick = tick;
        soh->status_flags |= AMS_SOH_STATUS_LAST_OBSERVABLE;
    }
    else
    {
        saturating_increment(&soh->rejected_count);
        resistance_soh_count_reasons(soh, reject);
    }

    uint32_t confidence = (soh->accepted_count >=
                           AMS_SOH_MIN_ACCEPTED_OBSERVATIONS) ? 100u :
        ((soh->accepted_count * 100u) /
         AMS_SOH_MIN_ACCEPTED_OBSERVATIONS);
    if(!current_calibration_confident)
    {
        confidence = 0u;
    }
    soh->observation_confidence_pct = (uint8_t)confidence;

    bool converged = (soh->accepted_count >=
                      AMS_SOH_MIN_ACCEPTED_OBSERVATIONS) &&
                     isfinite(soh->resistance_growth_ratio) &&
                     (soh->resistance_growth_ratio > 0.0f) &&
                     isfinite(soh->r0_variance_ohm2) &&
                     (soh->r0_variance_ohm2 <=
                      AMS_SOH_MAX_R0_VARIANCE_OHM2);
    if(converged)
    {
        soh->status_flags |= AMS_SOH_STATUS_CONVERGED;
    }
    bool observation_fresh = (soh->accepted_count != 0u) &&
        ((uint32_t)(tick - soh->last_accept_tick) <=
         AMS_SOH_MAX_ACCEPT_AGE_MS);
    uint32_t current_invalid_flags = AMS_SOH_REJECT_CURRENT_CALIBRATION |
                                     AMS_SOH_REJECT_MODEL_DOMAIN |
                                     AMS_SOH_REJECT_NUMERIC |
                                     AMS_SOH_REJECT_ESTIMATOR |
                                     AMS_SOH_REJECT_ACQUISITION;
    if(converged && current_calibration_confident && observation_fresh &&
       ((reject & current_invalid_flags) == 0u))
    {
        soh->status_flags |= AMS_SOH_STATUS_ADVISORY_VALID;
    }
    if(soh->persistence_valid != 0u)
    {
        soh->status_flags |= AMS_SOH_STATUS_PERSISTED;
    }
}

bool ams_estimator_configure_pack(ams_estimator_t *est)
{
    if (est == NULL)
    {
        return false;
    }

    memset(est->inst, 0, sizeof(est->inst));
    memset(est->resistance_soh, 0, sizeof(est->resistance_soh));
    est->enabled = 1U;
    est->instance_count = 1U;
    est->active_index = 0U;
    est->fault_flags = AMS_EKF_FAULT_NONE;

    ams_ekf_config_t cfg;
    ams_ekf_make_pack_config(&cfg);
    ams_ekf_init(&est->inst[0], &cfg);
    return (est->inst[0].fault_flags == AMS_EKF_FAULT_NONE);
}

bool ams_estimator_configure_segments(ams_estimator_t *est)
{
    if (est == NULL)
    {
        return false;
    }

    memset(est->inst, 0, sizeof(est->inst));
    memset(est->resistance_soh, 0, sizeof(est->resistance_soh));
    est->enabled = 1U;
    est->instance_count = 5U;
    est->active_index = 0U;
    est->fault_flags = AMS_EKF_FAULT_NONE;

    for (uint8_t i = 0U; i < est->instance_count; i++)
    {
        ams_ekf_config_t cfg;
        ams_ekf_make_segment_config(&cfg, i);
        ams_ekf_init(&est->inst[i], &cfg);
        if (est->inst[i].fault_flags != AMS_EKF_FAULT_NONE)
        {
            est->fault_flags |= est->inst[i].fault_flags;
            return false;
        }
    }

    return true;
}

bool ams_estimator_configure_even_split(ams_estimator_t *est, uint8_t instance_count)
{
    if ((est == NULL) || (instance_count == 0U) || (instance_count > AMS_EKF_MAX_INSTANCES) ||
        (instance_count > AMS_EKF_PACK_SERIES_GROUPS))
    {
        return false;
    }

    memset(est->inst, 0, sizeof(est->inst));
    memset(est->resistance_soh, 0, sizeof(est->resistance_soh));
    est->enabled = 1U;
    est->instance_count = instance_count;
    est->active_index = 0U;
    est->fault_flags = AMS_EKF_FAULT_NONE;

    uint16_t first = 0U;
    uint16_t base = (uint16_t)(AMS_EKF_PACK_SERIES_GROUPS / instance_count);
    uint16_t rem = (uint16_t)(AMS_EKF_PACK_SERIES_GROUPS % instance_count);

    for (uint8_t i = 0U; i < instance_count; i++)
    {
        uint16_t count = (uint16_t)(base + ((i < rem) ? 1U : 0U));
        ams_ekf_config_t cfg;
        ams_ekf_make_group_range_config(&cfg, first, count);
        ams_ekf_init(&est->inst[i], &cfg);
        if (est->inst[i].fault_flags != AMS_EKF_FAULT_NONE)
        {
            est->fault_flags |= est->inst[i].fault_flags;
            return false;
        }
        first = (uint16_t)(first + count);
    }

    return (first == AMS_EKF_PACK_SERIES_GROUPS);
}

void ams_estimator_cc_reset(ams_estimator_t *est, float soc_init)
{
    if (est == NULL)
    {
        return;
    }

    est->cc_soc = clampf_local(soc_init, 0.0f, 1.0f);
    est->cc_valid = 1U;
    est->cc_last_dt_clamped = 0U;
    est->cc_step_count = 0U;
    est->cc_dt_clamp_count = 0U;
}

bool ams_estimator_cc_step(ams_estimator_t *est, float i_pack_A, float dt_s)
{
    if (est == NULL)
    {
        return false;
    }

    if (!isfinite(i_pack_A) || (fabsf(i_pack_A) > 1500.0f))
    {
        return false;
    }

    float dt = dt_s;
    bool dt_clamped = false;
    if ((!isfinite(dt)) || (dt <= 0.0f))
    {
        dt = AMS_EKF_DEFAULT_DT_S;
        dt_clamped = true;
    }
    if(dt < 0.001f)
    {
        dt = 0.001f;
        dt_clamped = true;
    }
    else if(dt > 1.0f)
    {
        dt = 1.0f;
        dt_clamped = true;
    }
    est->cc_last_dt_clamped = dt_clamped ? 1U : 0U;
    if(dt_clamped)
    {
        saturating_increment(&est->cc_dt_clamp_count);
    }

    if (est->cc_valid == 0U)
    {
        return false;
    }

    return ams_estimator_cc_apply_charge(est, (double)i_pack_A * (double)dt);
}

bool ams_estimator_cc_apply_charge(ams_estimator_t *est, double charge_As)
{
    if((est == NULL) || (est->cc_valid == 0U) || !isfinite(charge_As))
    {
        return false;
    }

    double pack_capacity_As = (double)AMS_EKF_PACK_PARALLEL_CELLS *
                              3600.0 *
                              (double)AMS_EKF_CELL_CAPACITY_AH;
    if(pack_capacity_As <= 0.0)
    {
        return false;
    }

    /* Positive current/charge is discharge in the existing estimator
     * convention. The physical sign remains a target-validation gate. */
    est->cc_soc -= (float)(charge_As / pack_capacity_As);
    est->cc_soc = clampf_local(est->cc_soc, 0.0f, 1.0f);
    saturating_increment(&est->cc_step_count);
    return true;
}

void ams_estimator_init_default(ams_estimator_t *est)
{
    if (est == NULL)
    {
        return;
    }

    memset(est, 0, sizeof(*est));
    est->input_source = AMS_ESTIMATOR_INPUT_NONE;
    ams_estimator_cc_reset(est, AMS_EKF_DEFAULT_SOC_INIT);
#if AMS_ESTIMATOR_DEFAULT_TOPOLOGY == AMS_ESTIMATOR_TOPOLOGY_SEGMENTS
    if(!ams_estimator_configure_segments(est))
#else
    if(!ams_estimator_configure_pack(est))
#endif
    {
        est->enabled = 0u;
        est->fault_flags |= AMS_EKF_FAULT_BAD_CONFIG;
    }
}

void ams_estimator_refresh_summary(ams_estimator_t *est,
                                   ams_estimator_input_source_t source,
                                   uint32_t tick)
{
    if (est == NULL)
    {
        return;
    }

    est->input_source = source;
    est->last_update_tick = tick;
    saturating_increment(&est->step_count);
    est->fault_flags = AMS_EKF_FAULT_NONE;
    est->model_domain_flags = AMS_EKF_MODEL_DOMAIN_NONE;

    if ((est->active_index >= est->instance_count) ||
        (est->active_index >= AMS_EKF_MAX_INSTANCES))
    {
        est->fault_flags = AMS_EKF_FAULT_BAD_CONFIG;
        return;
    }

    uint32_t aggregate_faults = AMS_EKF_FAULT_NONE;
    for (uint8_t i = 0U; i < est->instance_count; i++)
    {
        if (est->inst[i].cfg.enabled != 0U)
        {
            aggregate_faults |= est->inst[i].fault_flags;

            /* Convergence is historical evidence, but advisory validity is
             * a live contract. If acquisition stops or this estimator input
             * becomes unusable, do not retain a plausible current-valid bit
             * merely because no new SoH observation arrived to clear it. */
            ams_resistance_soh_t *soh = &est->resistance_soh[i];
            bool observation_fresh = (soh->accepted_count != 0u) &&
                ((uint32_t)(tick - soh->last_accept_tick) <=
                 AMS_SOH_MAX_ACCEPT_AGE_MS);
            if((est->inst[i].valid == 0u) || !observation_fresh)
            {
                soh->status_flags &=
                    (uint8_t)~(AMS_SOH_STATUS_ADVISORY_VALID |
                               AMS_SOH_STATUS_LAST_OBSERVABLE);
            }
        }
    }

    const ams_ekf_instance_t *active = &est->inst[est->active_index];

    float soc_weighted = 0.0f;
    float r0_weighted = 0.0f;
    float t_core_weighted = 0.0f;
    float v_pred_sum = 0.0f;
    float innov_sum = 0.0f;
    float weight_sum = 0.0f;

    for (uint8_t i = 0U; i < est->instance_count; i++)
    {
        const ams_ekf_instance_t *inst = &est->inst[i];
        if ((inst->cfg.enabled != 0U) && (inst->valid != 0U))
        {
            float weight = (float)inst->cfg.series_group_count;
            soc_weighted += inst->soc * weight;
            r0_weighted += inst->r0_ohm * weight;
            t_core_weighted += inst->t_core_C * weight;
            v_pred_sum += inst->v_pred_V;
            innov_sum += inst->innovation_V;
            weight_sum += weight;
            est->model_domain_flags |= inst->model_domain_flags;
        }
    }

    if (weight_sum > 0.0f)
    {
        est->pack_soc = soc_weighted / weight_sum;
        est->representative_cell_r0_ohm = r0_weighted / weight_sum;
        est->pack_t_core_C = t_core_weighted / weight_sum;
        est->pack_v_pred_V = v_pred_sum;
        est->pack_innovation_V = innov_sum;
    }
    else if (est->cc_valid != 0U)
    {
        est->pack_soc = est->cc_soc;
        est->representative_cell_r0_ohm = active->r0_ohm;
        est->pack_v_pred_V = active->v_pred_V;
        est->pack_innovation_V = active->innovation_V;
        est->pack_t_core_C = active->t_core_C;
    }
    else
    {
        est->pack_soc = active->soc;
        est->representative_cell_r0_ohm = active->r0_ohm;
        est->pack_v_pred_V = active->v_pred_V;
        est->pack_innovation_V = active->innovation_V;
        est->pack_t_core_C = active->t_core_C;
    }

    est->pack_r0_ohm = est->representative_cell_r0_ohm;
    est->estimated_pack_r0_ohm =
        est->representative_cell_r0_ohm *
        ((float)AMS_EKF_PACK_SERIES_GROUPS / AMS_EKF_PACK_PARALLEL_CELLS);
    est->fault_flags = aggregate_faults;
}

uint8_t ams_estimator_status_flags(const ams_estimator_t *est)
{
    if ((est == NULL) || (est->enabled == 0U) ||
        (est->active_index >= est->instance_count) ||
        (est->active_index >= AMS_EKF_MAX_INSTANCES))
    {
        return AMS_EKF_FLAG_FAULTED;
    }

    const ams_ekf_instance_t *active = &est->inst[est->active_index];
    uint8_t flags = 0U;

    if (active->valid != 0U)
    {
        flags |= AMS_EKF_FLAG_VALID;
    }
    if (est->input_source == AMS_ESTIMATOR_INPUT_HIL_CAN)
    {
        flags |= AMS_EKF_FLAG_HIL_SOURCE;
    }
    if ((active->fault_flags | est->fault_flags) != AMS_EKF_FAULT_NONE)
    {
        flags |= AMS_EKF_FLAG_FAULTED;
    }
    if (((active->fault_flags | est->fault_flags) & AMS_EKF_FAULT_STALE_INPUT) != 0UL)
    {
        flags |= AMS_EKF_FLAG_STALE;
    }
    if (((active->fault_flags | est->fault_flags) & AMS_EKF_FAULT_CLAMPED) != 0UL)
    {
        flags |= AMS_EKF_FLAG_CLAMPED;
    }
    if ((active->valid == 0U) && (est->cc_valid != 0U))
    {
        flags |= AMS_EKF_FLAG_CC_FALLBACK;
    }
    if(est->model_domain_flags != AMS_EKF_MODEL_DOMAIN_NONE)
    {
        flags |= AMS_EKF_FLAG_MODEL_CLAMPED;
    }
    if((est->resistance_soh[est->active_index].status_flags &
        AMS_SOH_STATUS_ADVISORY_VALID) != 0u)
    {
        flags |= AMS_EKF_FLAG_SOH_ADVISORY;
    }

    return flags;
}
