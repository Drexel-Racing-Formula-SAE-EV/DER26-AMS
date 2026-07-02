
/*
 * ams_unit_test_runner.c
 *
 * Dedicated host-side unit tests for the AMS physics-only DAEKF estimator.
 *
 * These are intentionally smaller than the comprehensive host injection
 * harness. They test estimator/LUT/config/status behavior in isolation so CI
 * failures are easier to localize.
 */

#include "estimator/ams_soc_ekf.h"
#include "estimator/ams_estimator_lut.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_EPS_SMALL 1.0e-5f
#define TEST_EPS_MED   1.0e-3f

static int g_failures = 0;

#define TEST_FAIL(msg)                                                       \
    do                                                                       \
    {                                                                        \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, (msg));               \
        g_failures++;                                                        \
    } while (0)

#define EXPECT_TRUE(expr)                                                    \
    do                                                                       \
    {                                                                        \
        if (!(expr))                                                         \
        {                                                                    \
            TEST_FAIL("expected true: " #expr);                              \
        }                                                                    \
    } while (0)

#define EXPECT_FALSE(expr)                                                   \
    do                                                                       \
    {                                                                        \
        if ((expr))                                                          \
        {                                                                    \
            TEST_FAIL("expected false: " #expr);                             \
        }                                                                    \
    } while (0)

#define EXPECT_FINITE(x)                                                     \
    do                                                                       \
    {                                                                        \
        if (!isfinite((float)(x)))                                            \
        {                                                                    \
            TEST_FAIL("expected finite: " #x);                               \
        }                                                                    \
    } while (0)

#define EXPECT_RANGE(x, lo, hi)                                               \
    do                                                                       \
    {                                                                        \
        float _v = (float)(x);                                                \
        if ((!isfinite(_v)) || (_v < (float)(lo)) || (_v > (float)(hi)))      \
        {                                                                    \
            printf("FAIL %s:%d: %s=%f not in [%f,%f]\n",                     \
                   __FILE__, __LINE__, #x, _v, (float)(lo), (float)(hi));     \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

#define EXPECT_NEAR(x, y, eps)                                                \
    do                                                                       \
    {                                                                        \
        float _x = (float)(x);                                                \
        float _y = (float)(y);                                                \
        float _e = (float)(eps);                                              \
        if ((!isfinite(_x)) || (!isfinite(_y)) || (fabsf(_x - _y) > _e))      \
        {                                                                    \
            printf("FAIL %s:%d: %s=%f not near %s=%f eps=%f\n",              \
                   __FILE__, __LINE__, #x, _x, #y, _y, _e);                  \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

static void test_lut_basic_ranges(void)
{
    float ocv_0 = ams_p42a_ocv_v(0.0f, 25.0f);
    float ocv_50 = ams_p42a_ocv_v(0.5f, 25.0f);
    float ocv_100 = ams_p42a_ocv_v(1.0f, 25.0f);

    EXPECT_FINITE(ocv_0);
    EXPECT_FINITE(ocv_50);
    EXPECT_FINITE(ocv_100);

    EXPECT_RANGE(ocv_0, 2.5f, 3.8f);
    EXPECT_RANGE(ocv_50, 3.2f, 4.1f);
    EXPECT_RANGE(ocv_100, 4.0f, 4.3f);

    EXPECT_TRUE(ocv_0 < ocv_50);
    EXPECT_TRUE(ocv_50 < ocv_100);

    float r0 = ams_p42a_r0_ohm(0.5f, 25.0f);
    float inv_c1 = ams_p42a_inv_c1(0.5f, 25.0f);
    float neg_inv_tau1 = ams_p42a_neg_inv_tau1(0.5f, 25.0f);
    float inv_r1 = ams_p42a_inv_r1_from_luts(inv_c1, neg_inv_tau1);

    EXPECT_RANGE(r0, 0.005f, 0.040f);
    EXPECT_RANGE(inv_c1, 1.0e-5f, 2.0e-3f);
    EXPECT_RANGE(neg_inv_tau1, -1.0f, -1.0e-4f);
    EXPECT_RANGE(inv_r1, 1.0f, 1000.0f);
}

static void test_lut_clamps_out_of_range_queries(void)
{
    float ocv_low = ams_p42a_ocv_v(-1.0f, -100.0f);
    float ocv_min = ams_p42a_ocv_v(0.0f, 5.0f);
    float ocv_high = ams_p42a_ocv_v(2.0f, 200.0f);
    float ocv_max = ams_p42a_ocv_v(1.0f, 40.0f);

    EXPECT_NEAR(ocv_low, ocv_min, TEST_EPS_SMALL);
    EXPECT_NEAR(ocv_high, ocv_max, TEST_EPS_SMALL);
}

static void test_pack_config_and_init(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_pack_config(&cfg);

    EXPECT_TRUE(cfg.enabled == 1U);
    EXPECT_TRUE(cfg.first_series_group == 0U);
    EXPECT_TRUE(cfg.series_group_count == 75U);
    EXPECT_NEAR(cfg.parallel_cell_count, 6.0f, TEST_EPS_SMALL);
    EXPECT_NEAR(cfg.cell_capacity_Ah, 4.2f, TEST_EPS_SMALL);
    EXPECT_NEAR(cfg.sample_time_s, 0.1f, TEST_EPS_SMALL);

    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);

    EXPECT_TRUE(ekf.cfg.enabled == 1U);
    EXPECT_TRUE(ekf.fault_flags == AMS_EKF_FAULT_NONE);
    EXPECT_RANGE(ekf.soc, 0.0f, 1.0f);
    EXPECT_RANGE(ekf.r0_ohm, 0.005f, 0.040f);
    EXPECT_TRUE(ekf.valid == 0U);
}

static void test_segment_config_layout(void)
{
    for (uint8_t seg = 0U; seg < 5U; seg++)
    {
        ams_ekf_config_t cfg;
        ams_ekf_make_segment_config(&cfg, seg);

        EXPECT_TRUE(cfg.enabled == 1U);
        EXPECT_TRUE(cfg.first_series_group == (uint16_t)(seg * 15U));
        EXPECT_TRUE(cfg.series_group_count == 15U);

        ams_ekf_instance_t ekf;
        ams_ekf_init(&ekf, &cfg);
        EXPECT_TRUE(ekf.fault_flags == AMS_EKF_FAULT_NONE);
    }
}

static void test_even_split_configuration(void)
{
    ams_estimator_t est;
    ams_estimator_init_default(&est);

    EXPECT_TRUE(ams_estimator_configure_even_split(&est, 10U));
    EXPECT_TRUE(est.enabled == 1U);
    EXPECT_TRUE(est.instance_count == 10U);

    uint16_t expected_first[10] = {0U, 8U, 16U, 24U, 32U, 40U, 47U, 54U, 61U, 68U};
    uint16_t expected_count[10] = {8U, 8U, 8U, 8U, 8U, 7U, 7U, 7U, 7U, 7U};

    for (uint8_t i = 0U; i < 10U; i++)
    {
        EXPECT_TRUE(est.inst[i].cfg.enabled == 1U);
        EXPECT_TRUE(est.inst[i].cfg.first_series_group == expected_first[i]);
        EXPECT_TRUE(est.inst[i].cfg.series_group_count == expected_count[i]);
        EXPECT_TRUE(est.inst[i].fault_flags == AMS_EKF_FAULT_NONE);
    }

    EXPECT_FALSE(ams_estimator_configure_even_split(&est, 0U));
    EXPECT_FALSE(ams_estimator_configure_even_split(&est, 11U));
}

static void test_bad_config_rejected(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_group_range_config(&cfg, 74U, 2U);

    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);

    EXPECT_TRUE((ekf.fault_flags & AMS_EKF_FAULT_BAD_CONFIG) != 0UL);
    EXPECT_FALSE(ams_ekf_step(&ekf, 0.0f, 300.0f, 25.0f, 0.1f));
}

static void test_r0_initialization_clamp(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_pack_config(&cfg);

    cfg.r0_init_ohm = 0.100f;
    ams_ekf_instance_t high;
    ams_ekf_init(&high, &cfg);
    EXPECT_NEAR(high.r0_ohm, 0.040f, TEST_EPS_SMALL);

    cfg.r0_init_ohm = 0.001f;
    ams_ekf_instance_t low;
    ams_ekf_init(&low, &cfg);
    EXPECT_NEAR(low.r0_ohm, 0.005f, TEST_EPS_SMALL);
}

static void test_single_step_nominal_pack(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_pack_config(&cfg);

    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);

    float i_pack_A = 10.0f;
    float i_cell_A = i_pack_A / 6.0f;
    float v_meas_V = 75.0f * (ams_p42a_ocv_v(1.0f, 25.0f) - (0.0147f * i_cell_A));

    bool ok = ams_ekf_step(&ekf, i_pack_A, v_meas_V, 25.0f, 0.1f);

    EXPECT_TRUE(ok);
    EXPECT_TRUE(ekf.valid == 1U);
    EXPECT_RANGE(ekf.soc, 0.0f, 1.0f);
    EXPECT_RANGE(ekf.r0_ohm, 0.005f, 0.040f);
    EXPECT_RANGE(ekf.t_core_C, -10.0f, 60.0f);
    EXPECT_FINITE(ekf.v_pred_V);
    EXPECT_FINITE(ekf.innovation_V);
    EXPECT_TRUE(ekf.step_count == 1U);
}

static void test_invalid_step_inputs(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_pack_config(&cfg);

    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);

    EXPECT_FALSE(ams_ekf_step(&ekf, NAN, 300.0f, 25.0f, 0.1f));
    EXPECT_TRUE((ekf.fault_flags & AMS_EKF_FAULT_BAD_INPUT) != 0UL);

    EXPECT_FALSE(ams_ekf_step(&ekf, 0.0f, 100.0f, 25.0f, 0.1f));
    EXPECT_TRUE((ekf.fault_flags & AMS_EKF_FAULT_BAD_VOLTAGE) != 0UL);

    EXPECT_FALSE(ams_ekf_step(&ekf, 2000.0f, 300.0f, 25.0f, 0.1f));
    EXPECT_TRUE((ekf.fault_flags & AMS_EKF_FAULT_BAD_CURRENT) != 0UL);

    EXPECT_FALSE(ams_ekf_step(&ekf, 0.0f, 300.0f, 130.0f, 0.1f));
    EXPECT_TRUE((ekf.fault_flags & AMS_EKF_FAULT_BAD_TEMP) != 0UL);
}

static void test_200_step_numerical_stability(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_pack_config(&cfg);

    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);

    for (uint16_t k = 0U; k < 200U; k++)
    {
        float current_A = (k < 100U) ? 12.0f : -4.0f;
        float i_cell_A = current_A / 6.0f;
        float v_meas_V = 75.0f * (ams_p42a_ocv_v(ekf.soc, 25.0f) - (ekf.r0_ohm * i_cell_A));

        bool ok = ams_ekf_step(&ekf, current_A, v_meas_V, 25.0f, 0.1f);
        EXPECT_TRUE(ok);
        EXPECT_TRUE(ekf.valid == 1U);
        EXPECT_RANGE(ekf.soc, 0.0f, 1.0f);
        EXPECT_RANGE(ekf.r0_ohm, 0.005f, 0.040f);
        EXPECT_RANGE(ekf.t_core_C, -10.0f, 60.0f);
        EXPECT_FINITE(ekf.vp1_V);
        EXPECT_FINITE(ekf.vp2_V);
        EXPECT_FINITE(ekf.p_soc);
        EXPECT_FINITE(ekf.p_vp1);
        EXPECT_FINITE(ekf.p_vp2);
        EXPECT_FINITE(ekf.p_r0);
    }

    EXPECT_TRUE(ekf.step_count == 200U);
}

static void test_estimator_summary_aggregation(void)
{
    ams_estimator_t est;
    ams_estimator_init_default(&est);

    EXPECT_TRUE(ams_estimator_configure_even_split(&est, 10U));

    float weight_sum = 0.0f;
    float soc_sum = 0.0f;
    float r0_sum = 0.0f;
    float temp_sum = 0.0f;
    float v_pred_sum = 0.0f;
    float innovation_sum = 0.0f;

    for (uint8_t i = 0U; i < est.instance_count; i++)
    {
        ams_ekf_instance_t *inst = &est.inst[i];
        float w = (float)inst->cfg.series_group_count;

        inst->valid = 1U;
        inst->soc = 0.50f + (0.01f * (float)i);
        inst->r0_ohm = 0.010f + (0.001f * (float)i);
        inst->t_core_C = 25.0f + (float)i;
        inst->v_pred_V = 30.0f + (float)i;
        inst->innovation_V = 0.10f * (float)i;
        inst->fault_flags = (i == 7U) ? AMS_EKF_FAULT_CLAMPED : AMS_EKF_FAULT_NONE;

        weight_sum += w;
        soc_sum += inst->soc * w;
        r0_sum += inst->r0_ohm * w;
        temp_sum += inst->t_core_C * w;
        v_pred_sum += inst->v_pred_V;
        innovation_sum += inst->innovation_V;
    }

    ams_estimator_refresh_summary(&est, AMS_ESTIMATOR_INPUT_HIL_CAN, 1234U);

    EXPECT_NEAR(est.pack_soc, soc_sum / weight_sum, TEST_EPS_MED);
    EXPECT_NEAR(est.pack_r0_ohm, r0_sum / weight_sum, TEST_EPS_MED);
    EXPECT_NEAR(est.pack_t_core_C, temp_sum / weight_sum, TEST_EPS_MED);
    EXPECT_NEAR(est.pack_v_pred_V, v_pred_sum, TEST_EPS_MED);
    EXPECT_NEAR(est.pack_innovation_V, innovation_sum, TEST_EPS_MED);
    EXPECT_TRUE((est.fault_flags & AMS_EKF_FAULT_CLAMPED) != 0UL);
}

static void test_estimator_status_flags(void)
{
    ams_estimator_t est;
    ams_estimator_init_default(&est);

    uint8_t flags = ams_estimator_status_flags(&est);
    EXPECT_TRUE((flags & AMS_EKF_FLAG_VALID) == 0U);

    ams_ekf_instance_t *active = &est.inst[0];
    active->valid = 1U;
    active->fault_flags = AMS_EKF_FAULT_NONE;

    ams_estimator_refresh_summary(&est, AMS_ESTIMATOR_INPUT_HIL_CAN, 55U);
    flags = ams_estimator_status_flags(&est);

    EXPECT_TRUE((flags & AMS_EKF_FLAG_VALID) != 0U);
    EXPECT_TRUE((flags & AMS_EKF_FLAG_HIL_SOURCE) != 0U);
    EXPECT_TRUE((flags & AMS_EKF_FLAG_FAULTED) == 0U);

    active->fault_flags = AMS_EKF_FAULT_STALE_INPUT;
    ams_estimator_refresh_summary(&est, AMS_ESTIMATOR_INPUT_HIL_CAN, 56U);
    flags = ams_estimator_status_flags(&est);

    EXPECT_TRUE((flags & AMS_EKF_FLAG_FAULTED) != 0U);
    EXPECT_TRUE((flags & AMS_EKF_FLAG_STALE) != 0U);
}

static void test_coulomb_count_baseline(void)
{
    ams_estimator_t est;
    ams_estimator_init_default(&est);

    EXPECT_TRUE(est.cc_valid == 1U);
    EXPECT_NEAR(est.cc_soc, 1.0f, TEST_EPS_SMALL);

    EXPECT_TRUE(ams_estimator_cc_step(&est, 60.0f, 60.0f));
    EXPECT_TRUE(est.cc_soc < 1.0f);
    float after_discharge = est.cc_soc;

    EXPECT_TRUE(ams_estimator_cc_step(&est, -30.0f, 60.0f));
    EXPECT_TRUE(est.cc_soc > after_discharge);
    EXPECT_TRUE(est.cc_soc <= 1.0f);

    float before_bad_sample = est.cc_soc;
    EXPECT_FALSE(ams_estimator_cc_step(&est, NAN, 0.1f));
    EXPECT_TRUE(est.cc_valid == 1U);
    EXPECT_NEAR(est.cc_soc, before_bad_sample, TEST_EPS_SMALL);

    EXPECT_FALSE(ams_estimator_cc_step(&est, 1500.1f, 0.1f));
    EXPECT_TRUE(est.cc_valid == 1U);
    EXPECT_NEAR(est.cc_soc, before_bad_sample, TEST_EPS_SMALL);

    EXPECT_TRUE(ams_estimator_cc_step(&est, 20.0f, 0.1f));
    EXPECT_TRUE(est.cc_soc < before_bad_sample);

    ams_estimator_cc_reset(&est, 0.50f);
    EXPECT_TRUE(est.cc_valid == 1U);
    EXPECT_NEAR(est.cc_soc, 0.50f, TEST_EPS_SMALL);

    est.inst[0].valid = 0U;
    ams_estimator_refresh_summary(&est, AMS_ESTIMATOR_INPUT_HARDWARE, 77U);
    EXPECT_NEAR(est.pack_soc, est.cc_soc, TEST_EPS_SMALL);
    EXPECT_TRUE((ams_estimator_status_flags(&est) & AMS_EKF_FLAG_CC_FALLBACK) != 0U);
}

static void run_test(const char *name, void (*fn)(void))
{
    int before = g_failures;
    fn();

    if (g_failures == before)
    {
        printf("PASS %s\n", name);
    }
}

int main(void)
{
    run_test("lut basic ranges", test_lut_basic_ranges);
    run_test("lut clamp behavior", test_lut_clamps_out_of_range_queries);
    run_test("pack config and init", test_pack_config_and_init);
    run_test("segment config layout", test_segment_config_layout);
    run_test("even split configuration", test_even_split_configuration);
    run_test("bad config rejection", test_bad_config_rejected);
    run_test("R0 initialization clamp", test_r0_initialization_clamp);
    run_test("single-step nominal pack", test_single_step_nominal_pack);
    run_test("invalid step inputs", test_invalid_step_inputs);
    run_test("200-step numerical stability", test_200_step_numerical_stability);
    run_test("estimator summary aggregation", test_estimator_summary_aggregation);
    run_test("estimator status flags", test_estimator_status_flags);
    run_test("coulomb count baseline", test_coulomb_count_baseline);

    if (g_failures != 0)
    {
        printf("AMS UNIT TESTS FAILED: %d failure(s)\n", g_failures);
        return 1;
    }

    printf("ALL AMS UNIT TESTS PASSED\n");
    return 0;
}
