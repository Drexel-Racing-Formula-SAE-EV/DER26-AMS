/*
 * ams_soc_ekf.c
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

bool ams_ekf_step(ams_ekf_instance_t *ekf,
                  float i_pack_A,
                  float v_meas_V,
                  float t_surf_C,
                  float dt_s)
{
    if ((ekf == NULL) || (ekf->cfg.enabled == 0U))
    {
        return false;
    }

    ekf->fault_flags = AMS_EKF_FAULT_NONE;

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
    if ((!isfinite(dt)) || (dt <= 0.0f))
    {
        dt = ekf->cfg.sample_time_s;
    }
    dt = clampf_local(dt, 0.001f, 1.0f);

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

    update_adaptive_r(ekf, innovation);
    float r_meas = ekf->r_meas_V2;

    float h_r0 = -series_count * i_cell_A;
    float s_r0 = (h_r0 * h_r0 * ekf->p_r0) + r_meas;
    float k_r0 = (s_r0 > 1.0e-10f) ? (ekf->p_r0 * h_r0 / s_r0) : 0.0f;

    ekf->r0_ohm += k_r0 * innovation;

    float a_r0 = 1.0f - (k_r0 * h_r0);
    ekf->p_r0 = (a_r0 * a_r0 * ekf->p_r0) + (k_r0 * k_r0 * r_meas);

    float r0_before_clamp = ekf->r0_ohm;
    ekf->r0_ohm = clampf_local(ekf->r0_ohm, AMS_EKF_R0_MIN_OHM, AMS_EKF_R0_MAX_OHM);
    if (ekf->r0_ohm != r0_before_clamp)
    {
        ekf->fault_flags |= AMS_EKF_FAULT_CLAMPED;
    }

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
    float inv_s = (s_x > 1.0e-10f) ? (1.0f / s_x) : 0.0f;

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

    if (ekf->p_soc < 1.0e-12f) { ekf->p_soc = 1.0e-12f; }
    if (ekf->p_vp1 < 1.0e-12f) { ekf->p_vp1 = 1.0e-12f; }
    if (ekf->p_vp2 < 1.0e-12f) { ekf->p_vp2 = 1.0e-12f; }
    if (ekf->p_r0  < 1.0e-14f) { ekf->p_r0  = 1.0e-14f; }

    float inv_r1 = ams_p42a_inv_r1_from_luts(inv_c1, neg_inv_tau1);
    float q_gen = (i_cell_A * i_cell_A * ekf->r0_ohm) +
                  (vp1_p * vp1_p * inv_r1) +
                  (vp2_p * vp2_p * AMS_EKF_INV_R2);
    float q_cs = (ekf->t_core_C - t_surf_C) * AMS_EKF_INV_RCS;
    ekf->t_core_C += (q_gen - q_cs) * AMS_EKF_INV_CC * dt;
    ekf->t_core_C = clampf_local(ekf->t_core_C, -10.0f, 60.0f);

    ekf->v_pred_V = v_est;
    ekf->innovation_V = innovation;
    ekf->last_i_pack_A = i_pack_A;
    ekf->last_v_meas_V = v_meas_V;
    ekf->last_t_surf_C = t_surf_C;
    ekf->step_count++;
    ekf->valid = 1U;

    return true;
}

bool ams_estimator_configure_pack(ams_estimator_t *est)
{
    if (est == NULL)
    {
        return false;
    }

    memset(est->inst, 0, sizeof(est->inst));
    est->enabled = 1U;
    est->instance_count = 1U;
    est->active_index = 0U;

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
    est->enabled = 1U;
    est->instance_count = 5U;
    est->active_index = 0U;

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
    est->enabled = 1U;
    est->instance_count = instance_count;
    est->active_index = 0U;

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

void ams_estimator_init_default(ams_estimator_t *est)
{
    if (est == NULL)
    {
        return;
    }

    memset(est, 0, sizeof(*est));
    est->input_source = AMS_ESTIMATOR_INPUT_NONE;
    (void)ams_estimator_configure_pack(est);
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
    est->step_count++;
    est->fault_flags = AMS_EKF_FAULT_NONE;

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
        }
    }

    if (weight_sum > 0.0f)
    {
        est->pack_soc = soc_weighted / weight_sum;
        est->pack_r0_ohm = r0_weighted / weight_sum;
        est->pack_t_core_C = t_core_weighted / weight_sum;
        est->pack_v_pred_V = v_pred_sum;
        est->pack_innovation_V = innov_sum;
    }
    else
    {
        est->pack_soc = active->soc;
        est->pack_r0_ohm = active->r0_ohm;
        est->pack_v_pred_V = active->v_pred_V;
        est->pack_innovation_V = active->innovation_V;
        est->pack_t_core_C = active->t_core_C;
    }

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

    return flags;
}
