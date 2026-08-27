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
#define AMS_EKF_R_INIT      1.0e-2f
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
    ekf->p_vp1 = AMS_EKF_P0_VP1;
    ekf->p_vp2 = AMS_EKF_P0_VP2;
    ekf->p_r0 = AMS_EKF_P0_R0;
    ekf->r_meas_V2 = AMS_EKF_R_INIT;

    for (uint8_t i = 0U; i < AMS_EKF_ADAPT_WIN; i++)
    {
        ekf->innov_hist[i] = AMS_EKF_R_INIT;
    }

    ekf->valid = 0U;
    ekf->fault_flags = AMS_EKF_FAULT_NONE;
    ekf->model_domain_flags = AMS_EKF_MODEL_DOMAIN_NONE;
}

static void update_adaptive_r(ams_ekf_instance_t *ekf, float innovation_V)
{
    ekf->innov_hist[ekf->innov_idx] = innovation_V * innovation_V;
    ekf->innov_idx = (uint8_t)((ekf->innov_idx + 1U) % AMS_EKF_ADAPT_WIN);

    float sum = 0.0f;
    for (uint8_t i = 0U; i < AMS_EKF_ADAPT_WIN; i++)
    {
        sum += ekf->innov_hist[i];
    }

    ekf->r_meas_V2 = sum / (float)AMS_EKF_ADAPT_WIN;
    if (ekf->r_meas_V2 < 1.0e-6f)
    {
        ekf->r_meas_V2 = 1.0e-6f;
    }
    else if (ekf->r_meas_V2 > 25.0f)
    {
        ekf->r_meas_V2 = 25.0f;
    }
}

bool ams_ekf_step_gated(ams_ekf_instance_t *ekf,
                        float i_pack_A,
                        float v_meas_V,
                        float t_surf_C,
                        float dt_s,
                        bool allow_r0_update,
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

    float psoc_p = ekf->p_soc + AMS_EKF_Q_SOC;
    float pvp1_p = (f_vp1 * f_vp1 * ekf->p_vp1) + AMS_EKF_Q_VP1;
    float pvp2_p = (f_vp2 * f_vp2 * ekf->p_vp2) + AMS_EKF_Q_VP2;
    ekf->p_r0 += AMS_EKF_Q_R0;

    float v_est = series_count * (ams_p42a_ocv_v(soc_p, t_lut_C) -
                                  (ekf->r0_ohm * i_cell_A) -
                                  vp1_p - vp2_p);
    float innovation = v_meas_V - v_est;

    /* Gate the measurement update against the PRIOR measurement covariance.
     * A bad-but-in-range sample must not first inflate adaptive R and only
     * protect the following sample.  Rejected samples are predict-only and
     * do not enter the adaptive-R history. */
    float r_meas = ekf->r_meas_V2;
    float s_hi = clampf_local(soc_p + AMS_EKF_DSOC, 0.0f, 1.0f);
    float s_lo = clampf_local(soc_p - AMS_EKF_DSOC, 0.0f, 1.0f);
    float d_ocv = (ams_p42a_ocv_v(s_hi, t_lut_C) - ams_p42a_ocv_v(s_lo, t_lut_C)) /
                  ((s_hi - s_lo) + 1.0e-10f);

    float h_soc = series_count * d_ocv;
    float h_vp1 = -series_count;
    float h_vp2 = -series_count;

    float s_x = (h_soc * h_soc * psoc_p) +
                (h_vp1 * h_vp1 * pvp1_p) +
                (h_vp2 * h_vp2 * pvp2_p) + r_meas;
    float innovation_per_cell = fabsf(innovation) / series_count;
    const float innovation_gate_sigma = 6.0f;
    const bool bootstrap_step = ekf->step_count < AMS_EKF_ACQUISITION_STEPS;
    bool innovation_rejected = (!isfinite(s_x)) || (s_x <= 1.0e-10f) ||
        (!isfinite(innovation_per_cell));

    if(!innovation_rejected)
    {
        if(bootstrap_step)
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
        ekf->p_soc = psoc_p;
        ekf->p_vp1 = pvp1_p;
        ekf->p_vp2 = pvp2_p;
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
        float k_soc = psoc_p * h_soc * inv_s;
        float k_vp1 = pvp1_p * h_vp1 * inv_s;
        float k_vp2 = pvp2_p * h_vp2 * inv_s;

        ekf->soc = clampf_local(soc_p + (k_soc * innovation), 0.0f, 1.0f);
        ekf->vp1_V = vp1_p + (k_vp1 * innovation);
        ekf->vp2_V = vp2_p + (k_vp2 * innovation);

        float a00 = 1.0f - (k_soc * h_soc);
        float a01 =      - (k_soc * h_vp1);
        float a02 =      - (k_soc * h_vp2);

        float a10 =      - (k_vp1 * h_soc);
        float a11 = 1.0f - (k_vp1 * h_vp1);
        float a12 =      - (k_vp1 * h_vp2);

        float a20 =      - (k_vp2 * h_soc);
        float a21 =      - (k_vp2 * h_vp1);
        float a22 = 1.0f - (k_vp2 * h_vp2);

        ekf->p_soc = (a00 * a00 * psoc_p) + (a01 * a01 * pvp1_p) + (a02 * a02 * pvp2_p) +
                     (k_soc * k_soc * r_meas);
        ekf->p_vp1 = (a10 * a10 * psoc_p) + (a11 * a11 * pvp1_p) + (a12 * a12 * pvp2_p) +
                     (k_vp1 * k_vp1 * r_meas);
        ekf->p_vp2 = (a20 * a20 * psoc_p) + (a21 * a21 * pvp1_p) + (a22 * a22 * pvp2_p) +
                     (k_vp2 * k_vp2 * r_meas);

        /* Adapt for the NEXT sample only after this innovation passed the
         * current sample's authority gate. */
        update_adaptive_r(ekf, innovation);
    }

    if (ekf->p_soc < 1.0e-12f) { ekf->p_soc = 1.0e-12f; }
    if (ekf->p_vp1 < 1.0e-12f) { ekf->p_vp1 = 1.0e-12f; }
    if (ekf->p_vp2 < 1.0e-12f) { ekf->p_vp2 = 1.0e-12f; }
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
                                     AMS_SOH_REJECT_ESTIMATOR;
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
