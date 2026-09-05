
/*
 * ams_unit_test_runner.c
 * Author: Mahad Faisal (2026)
 *
 * Dedicated host-side unit tests for the AMS physics-only DAEKF estimator.
 *
 * These are intentionally smaller than the comprehensive host injection
 * harness. They test estimator/LUT/config/status behavior in isolation so CI
 * failures are easier to localize.
 */

#include "estimator/ams_soc_ekf.h"
#include "estimator/ams_estimator_lut.h"
#include "ext_drivers/current_sensor.h"
#include "ext_drivers/stm32f767z.h"
#include "ext_drivers/adbms6830_functions.h"
#include "ext_drivers/adbms2950.h"
#include "ext_drivers/voltage_fault.h"
#include "ext_drivers/main_fuse_monitor.h"
#include "ext_drivers/parallel_connection_observer.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint16_t unit_adc_read_counts[2] = {0u, 0u};
static HAL_StatusTypeDef unit_adc_read_statuses[2] = {HAL_OK, HAL_OK};
static uint32_t unit_adc_read_index = 0u;

HAL_StatusTypeDef stm32f767z_adc_switch_channel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    (void)channel;
    return (hadc != NULL) ? HAL_OK : HAL_ERROR;
}

stm32f767z_adc_read_result_t stm32f767z_adc_read_checked(ADC_HandleTypeDef *hadc, uint32_t timeout_ms)
{
    (void)timeout_ms;
    stm32f767z_adc_read_result_t result = { HAL_ERROR, 0u };
    uint32_t idx;

    if(hadc == NULL)
    {
        return result;
    }

    idx = unit_adc_read_index;
    if(idx > 1u)
    {
        idx = 1u;
    }

    result.status = unit_adc_read_statuses[idx];
    result.count = (result.status == HAL_OK) ? unit_adc_read_counts[idx] : 0u;
    unit_adc_read_index++;
    return result;
}

uint16_t stm32f767z_adc_read(ADC_HandleTypeDef *hadc)
{
    return stm32f767z_adc_read_checked(hadc, 5u).count;
}


static uint8_t unit_spi_last_tx[BUFSZ];
static uint8_t unit_spi_last_txrx_tx[BUFSZ];
static uint8_t unit_spi_txrx_response[BUFSZ];
static uint8_t unit_spi_txrx_sequence[32][BUFSZ];
static uint16_t unit_spi_last_tx_len = 0u;
static uint16_t unit_spi_last_txrx_len = 0u;
static uint32_t unit_spi_tx_calls = 0u;
static uint32_t unit_spi_txrx_calls = 0u;
static uint32_t unit_spi_txrx_sequence_count = 0u;
static HAL_StatusTypeDef unit_spi_tx_status_sequence[32];
static uint32_t unit_spi_tx_status_sequence_count = 0u;
static uint32_t unit_gpio_write_calls = 0u;
static GPIO_PinState unit_gpio_states[64];
static HAL_StatusTypeDef unit_spi_tx_status = HAL_OK;
static HAL_StatusTypeDef unit_spi_txrx_status = HAL_OK;

static void unit_spi_reset(void)
{
    memset(unit_spi_last_tx, 0, sizeof(unit_spi_last_tx));
    memset(unit_spi_last_txrx_tx, 0, sizeof(unit_spi_last_txrx_tx));
    memset(unit_spi_txrx_response, 0, sizeof(unit_spi_txrx_response));
    memset(unit_spi_txrx_sequence, 0, sizeof(unit_spi_txrx_sequence));
    memset(unit_gpio_states, 0, sizeof(unit_gpio_states));
    unit_spi_last_tx_len = 0u;
    unit_spi_last_txrx_len = 0u;
    unit_spi_tx_calls = 0u;
    unit_spi_txrx_calls = 0u;
    unit_spi_txrx_sequence_count = 0u;
    memset(unit_spi_tx_status_sequence, 0, sizeof(unit_spi_tx_status_sequence));
    unit_spi_tx_status_sequence_count = 0u;
    unit_gpio_write_calls = 0u;
    unit_spi_tx_status = HAL_OK;
    unit_spi_txrx_status = HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_StatusTypeDef status;

    (void)Timeout;
    if((hspi == NULL) || (pData == NULL) || (Size > BUFSZ))
    {
        return HAL_ERROR;
    }

    memcpy(unit_spi_last_tx, pData, Size);
    unit_spi_last_tx_len = Size;
    status = ((unit_spi_tx_status_sequence_count > 0u) &&
              (unit_spi_tx_calls < unit_spi_tx_status_sequence_count))
                 ? unit_spi_tx_status_sequence[unit_spi_tx_calls]
                 : unit_spi_tx_status;
    unit_spi_tx_calls++;
    return status;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                           uint8_t *pTxData,
                                           uint8_t *pRxData,
                                           uint16_t Size,
                                           uint32_t Timeout)
{
    (void)Timeout;
    if((hspi == NULL) || (pTxData == NULL) || (pRxData == NULL) || (Size > BUFSZ))
    {
        return HAL_ERROR;
    }

    memcpy(unit_spi_last_txrx_tx, pTxData, Size);
    unit_spi_last_txrx_len = Size;
    if((unit_spi_txrx_sequence_count > 0u) &&
       (unit_spi_txrx_calls < unit_spi_txrx_sequence_count))
    {
        memcpy(pRxData, unit_spi_txrx_sequence[unit_spi_txrx_calls], Size);
    }
    else
    {
        memcpy(pRxData, unit_spi_txrx_response, Size);
    }
    unit_spi_txrx_calls++;
    return unit_spi_txrx_status;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    (void)GPIOx;
    if(GPIO_Pin < (uint16_t)(sizeof(unit_gpio_states) / sizeof(unit_gpio_states[0])))
    {
        unit_gpio_states[GPIO_Pin] = PinState;
    }
    unit_gpio_write_calls++;
}

static TIM_TypeDef unit_delay_timer_instance;
static TIM_HandleTypeDef unit_delay_timer;
static uint32_t unit_delay_counter;
static bool unit_delay_timer_advances = true;

static void unit_delay_counter_set(uint32_t value)
{
    unit_delay_counter = value;
}

static uint32_t unit_delay_counter_get(void)
{
    if(unit_delay_timer_advances && (unit_delay_counter != UINT32_MAX))
    {
        unit_delay_counter++;
    }
    return unit_delay_counter;
}

/* Replace only the timer counter access used by the directly-included ADBMS
 * driver.  This gives unit tests a deterministic advancing or frozen 1 MHz
 * timer without a background host thread. */
#undef __HAL_TIM_SET_COUNTER
#undef __HAL_TIM_GET_COUNTER
#define __HAL_TIM_SET_COUNTER(handle, value) unit_delay_counter_set((uint32_t)(value))
#define __HAL_TIM_GET_COUNTER(handle) unit_delay_counter_get()

#include "Core/Src/ext_drivers/current_sensor.c"
#include "Core/Src/ext_drivers/current_fault.c"
#include "Core/Src/ext_drivers/voltage_fault.c"
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#endif
#include "Core/Src/ext_drivers/adbms_shared.c"
#include "Core/Src/ext_drivers/adbms6830.c"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#define TEST_EPS_SMALL 1.0e-5f
#define TEST_EPS_MED   1.0e-3f

static uint32_t unit_adbms_fake_time_us = 0u;
static uint32_t unit_adbms_time_us(void *ctx)
{
    (void)ctx;
    return unit_adbms_fake_time_us;
}

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

static bool unit_covariance_psd(const ams_ekf_instance_t *ekf)
{
    if(ekf == NULL)
    {
        return false;
    }
    const float p00 = ekf->p_soc;
    const float p01 = ekf->p_soc_vp1;
    const float p02 = ekf->p_soc_vp2;
    const float p11 = ekf->p_vp1;
    const float p12 = ekf->p_vp1_vp2;
    const float p22 = ekf->p_vp2;
    if(!isfinite(p00) || !isfinite(p01) || !isfinite(p02) ||
       !isfinite(p11) || !isfinite(p12) || !isfinite(p22) ||
       (p00 < 0.0f) || (p11 < 0.0f) || (p22 < 0.0f))
    {
        return false;
    }
    const float d01 = p00 * p11 - p01 * p01;
    const float d02 = p00 * p22 - p02 * p02;
    const float d12 = p11 * p22 - p12 * p12;
    const float det = p00 * (p11 * p22 - p12 * p12) -
                      p01 * (p01 * p22 - p12 * p02) +
                      p02 * (p01 * p12 - p11 * p02);
    const float tol2 = 1.0e-7f * fmaxf(1.0e-20f,
        fmaxf(p00 * p11, fmaxf(p00 * p22, p11 * p22)));
    const float tol3 = 1.0e-6f * fmaxf(1.0e-20f, p00 * p11 * p22);
    return (d01 >= -tol2) && (d02 >= -tol2) && (d12 >= -tol2) &&
           (det >= -tol3);
}

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

static void test_r0_observability_and_accounting(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_pack_config(&cfg);
    cfg.soc_init = 0.50f;

    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);

    uint32_t reject = ams_resistance_soh_gate(&ekf,
                                              30.0f,
                                              true,
                                              true,
                                              true);
    EXPECT_TRUE((reject & AMS_SOH_REJECT_LOW_CURRENT_STEP) != 0u);

    ekf.step_count = AMS_EKF_ACQUISITION_STEPS;
    ekf.last_i_pack_A = 0.0f;
    reject = ams_resistance_soh_gate(&ekf,
                                     30.0f,
                                     true,
                                     true,
                                     true);
    EXPECT_TRUE(reject == AMS_SOH_REJECT_NONE);

    float i_cell_A = 30.0f / cfg.parallel_cell_count;
    float v_meas_V = (float)cfg.series_group_count *
        (ams_p42a_ocv_v(ekf.soc, 25.0f) - (ekf.r0_ohm * i_cell_A));
    float r0_before = ekf.r0_ohm;
    ams_ekf_r0_update_result_t result = AMS_EKF_R0_UPDATE_APPLIED;
    EXPECT_TRUE(ams_ekf_step_gated(&ekf,
                                   30.0f,
                                   v_meas_V,
                                   25.0f,
                                   0.1f,
                                   false,
                                   &result));
    EXPECT_TRUE(result == AMS_EKF_R0_UPDATE_NOT_REQUESTED);
    EXPECT_NEAR(ekf.r0_ohm, r0_before, TEST_EPS_SMALL);

    ams_ekf_init(&ekf, &cfg);
    ekf.step_count = AMS_EKF_ACQUISITION_STEPS;
    ekf.last_i_pack_A = 0.0f;
    i_cell_A = 30.0f / cfg.parallel_cell_count;
    v_meas_V = (float)cfg.series_group_count *
        (ams_p42a_ocv_v(ekf.soc, 25.0f) - (ekf.r0_ohm * i_cell_A));
    EXPECT_TRUE(ams_ekf_step_gated(&ekf,
                                   30.0f,
                                   v_meas_V,
                                   25.0f,
                                   0.1f,
                                   true,
                                   &result));
    EXPECT_TRUE(result == AMS_EKF_R0_UPDATE_APPLIED);

    ams_resistance_soh_t soh = {0};
    ams_resistance_soh_record(&soh,
                              &ekf,
                              7u,
                              1000u,
                              true,
                              AMS_SOH_REJECT_NONE,
                              result,
                              true);
    EXPECT_TRUE(soh.accepted_count == 1u);
    EXPECT_TRUE(soh.rejected_count == 0u);
    EXPECT_TRUE((soh.status_flags & AMS_SOH_STATUS_LAST_OBSERVABLE) != 0u);
    EXPECT_FINITE(soh.reference_cell_r0_ohm);
    EXPECT_FINITE(soh.resistance_growth_ratio);

    ams_resistance_soh_record(&soh,
                              &ekf,
                              8u,
                              1100u,
                              false,
                              AMS_SOH_REJECT_CURRENT_CALIBRATION |
                                  AMS_SOH_REJECT_LOW_CURRENT,
                              AMS_EKF_R0_UPDATE_NOT_REQUESTED,
                              true);
    EXPECT_TRUE(soh.rejected_count == 1u);
    EXPECT_TRUE(soh.reject_current_calibration_count == 1u);
    EXPECT_TRUE(soh.reject_low_current_count == 1u);
    EXPECT_TRUE(soh.observation_confidence_pct == 0u);

    ams_ekf_init(&ekf, &cfg);
    ekf.step_count = AMS_EKF_ACQUISITION_STEPS;
    ekf.last_i_pack_A = 0.0f;
    i_cell_A = 30.0f / cfg.parallel_cell_count;
    v_meas_V = (float)cfg.series_group_count *
        (ams_p42a_ocv_v(ekf.soc, 25.0f) - (ekf.r0_ohm * i_cell_A)) +
        ((float)cfg.series_group_count *
         (AMS_SOH_MAX_INNOVATION_PER_CELL_V + 0.01f));
    EXPECT_TRUE(ams_ekf_step_gated(&ekf,
                                   30.0f,
                                   v_meas_V,
                                   25.0f,
                                   0.1f,
                                   true,
                                   &result));
    EXPECT_TRUE(result == AMS_EKF_R0_UPDATE_REJECT_INNOVATION);

    memset(&soh, 0, sizeof(soh));
    soh.accepted_count = AMS_SOH_MIN_ACCEPTED_OBSERVATIONS - 1u;
    ekf.valid = 1u;
    ekf.p_r0 = AMS_SOH_MAX_R0_VARIANCE_OHM2 * 0.5f;
    ams_resistance_soh_record(&soh,
                              &ekf,
                              99u,
                              2000u,
                              true,
                              AMS_SOH_REJECT_NONE,
                              AMS_EKF_R0_UPDATE_APPLIED,
                              true);
    EXPECT_TRUE(soh.accepted_count == AMS_SOH_MIN_ACCEPTED_OBSERVATIONS);
    EXPECT_TRUE(soh.observation_confidence_pct == 100u);
    EXPECT_TRUE((soh.status_flags & AMS_SOH_STATUS_ADVISORY_VALID) != 0u);
    EXPECT_TRUE((soh.status_flags & AMS_SOH_STATUS_PERSISTED) == 0u);
    EXPECT_TRUE(soh.persistence_valid == 0u);

    ams_estimator_t summary_estimator;
    ams_estimator_init_default(&summary_estimator);
    summary_estimator.inst[0].valid = 1u;
    summary_estimator.resistance_soh[0] = soh;
    ams_estimator_refresh_summary(&summary_estimator,
                                  AMS_ESTIMATOR_INPUT_HARDWARE,
                                  2000u + AMS_SOH_MAX_ACCEPT_AGE_MS);
    EXPECT_TRUE((summary_estimator.resistance_soh[0].status_flags &
                 AMS_SOH_STATUS_ADVISORY_VALID) != 0u);
    ams_estimator_refresh_summary(&summary_estimator,
                                  AMS_ESTIMATOR_INPUT_HARDWARE,
                                  2000u + AMS_SOH_MAX_ACCEPT_AGE_MS + 1u);
    EXPECT_TRUE((summary_estimator.resistance_soh[0].status_flags &
                 AMS_SOH_STATUS_ADVISORY_VALID) == 0u);
    EXPECT_TRUE((summary_estimator.resistance_soh[0].status_flags &
                 AMS_SOH_STATUS_CONVERGED) != 0u);

    ams_resistance_soh_record(&soh,
                              &ekf,
                              100u,
                              2000u + AMS_SOH_MAX_ACCEPT_AGE_MS + 1u,
                              true,
                              AMS_SOH_REJECT_LOW_CURRENT,
                              AMS_EKF_R0_UPDATE_NOT_REQUESTED,
                              true);
    EXPECT_TRUE((soh.status_flags & AMS_SOH_STATUS_CONVERGED) != 0u);
    EXPECT_TRUE((soh.status_flags & AMS_SOH_STATUS_ADVISORY_VALID) == 0u);
}


static float unit_acquisition_segment_voltage(float soc,
                                              float temperature_C,
                                              float elapsed_s,
                                              float vp1_start_V,
                                              float vp2_start_V,
                                              float cell_bias_V)
{
    const float cell_v = ams_p42a_ocv_v(soc, temperature_C) -
        (vp1_start_V * expf(-elapsed_s / AMS_EKF_ACQ_TAU1_S)) -
        (vp2_start_V * expf(-elapsed_s / AMS_EKF_ACQ_TAU2_S)) +
        cell_bias_V;
    return 15.0f * cell_v;
}

static void test_fixed_basis_acquisition_and_consensus(void)
{
    const float truth_soc = 0.60f;
    const float temperature_C = 25.0f;
    const float vp1_start_V = 0.040f;
    const float vp2_start_V = 0.030f;
    ams_estimator_t est;
    memset(&est, 0, sizeof(est));
    est.enabled = 1u;
    est.instance_count = 5u;

    for(uint8_t s = 0u; s < est.instance_count; s++)
    {
        ams_ekf_config_t cfg;
        ams_ekf_make_segment_config(&cfg, s);
        cfg.soc_init = 0.80f;
        ams_ekf_init(&est.inst[s], &cfg);
    }

    /* Segment 0 has a coherent +20 mV/cell bias for the entire first
     * acquisition window. The other four segments should anchor together,
     * while segment 0 must be rejected by cross-segment consensus. */
    for(uint16_t k = 0u; k <= 200u; k++)
    {
        const float elapsed_s = 0.1f * (float)k;
        const uint32_t tick = (uint32_t)k * 100u;
        for(uint8_t s = 0u; s < est.instance_count; s++)
        {
            const float bias_V = (s == 0u) ? 0.020f : 0.0f;
            const float v_meas_V = unit_acquisition_segment_voltage(
                truth_soc,
                temperature_C,
                elapsed_s,
                vp1_start_V,
                vp2_start_V,
                bias_V);
            ams_ekf_r0_update_result_t r0_result =
                AMS_EKF_R0_UPDATE_APPLIED;
            EXPECT_TRUE(ams_ekf_step_acquiring_gated(&est.inst[s],
                                                      0.0f,
                                                      v_meas_V,
                                                      temperature_C,
                                                      0.1f,
                                                      &r0_result));
            EXPECT_TRUE(r0_result == AMS_EKF_R0_UPDATE_NOT_REQUESTED);
            ams_ekf_acquisition_observe(&est.inst[s],
                                        0.0f,
                                        0.0f,
                                        v_meas_V,
                                        temperature_C,
                                        tick,
                                        true,
                                        true);
        }
        ams_estimator_acquisition_resolve(&est);
    }

    const float expected_vp1_finish = vp1_start_V *
        expf(-20.0f / AMS_EKF_ACQ_TAU1_S);
    const float expected_vp2_finish = vp2_start_V *
        expf(-20.0f / AMS_EKF_ACQ_TAU2_S);

    EXPECT_FALSE(ams_ekf_acquisition_complete(&est.inst[0]));
    EXPECT_TRUE(est.inst[0].acquisition.reject_count >= 1u);
    EXPECT_TRUE(est.inst[0].acquisition.reason ==
                AMS_EKF_ACQ_REASON_SEGMENT_CONSENSUS_RETRY);

    for(uint8_t s = 1u; s < est.instance_count; s++)
    {
        const ams_ekf_instance_t *ekf = &est.inst[s];
        EXPECT_TRUE(ams_ekf_acquisition_complete(ekf));
        EXPECT_TRUE(ekf->acquisition.anchor_count == 1u);
        EXPECT_NEAR(ekf->soc, truth_soc, 0.0025f);
        EXPECT_NEAR(ekf->vp1_V, expected_vp1_finish, 0.0015f);
        EXPECT_NEAR(ekf->vp2_V, expected_vp2_finish, 0.0015f);
        EXPECT_TRUE(ekf->acquisition.fit_rcond >=
                    AMS_EKF_ACQ_MIN_FIT_RCOND);
        EXPECT_TRUE(ekf->acquisition.fit_rmse_mV_cell <= 1.0f);
        EXPECT_NEAR(ekf->r_meas_V2, 1.0e-2f, TEST_EPS_SMALL);
        EXPECT_NEAR(ekf->p_soc,
                    AMS_EKF_ACQ_SOC_SIGMA_INIT * AMS_EKF_ACQ_SOC_SIGMA_INIT,
                    TEST_EPS_SMALL);
        EXPECT_NEAR(ekf->p_soc_vp1, 0.0f, TEST_EPS_SMALL);
        EXPECT_NEAR(ekf->p_soc_vp2, 0.0f, TEST_EPS_SMALL);
        EXPECT_NEAR(ekf->p_vp1_vp2, 0.0f, TEST_EPS_SMALL);
        EXPECT_TRUE(unit_covariance_psd(ekf));
    }
}

static void test_dynamic_acquisition_without_false_rest_anchor(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_segment_config(&cfg, 0u);
    cfg.soc_init = 0.80f;

    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);
    const float initial_error = fabsf(ekf.soc - 0.60f);

    /* 1 A is above the provisional acquisition-enter threshold. The
     * constrained dynamic estimator must remain alive and improve SoC, but
     * the relaxation observer must never claim a fixed-basis anchor. */
    for(uint16_t k = 0u; k <= 400u; k++)
    {
        const float current_A = 1.0f;
        const float i_cell_A = current_A / cfg.parallel_cell_count;
        const float v_meas_V = 15.0f *
            (ams_p42a_ocv_v(0.60f, 25.0f) - (ekf.r0_ohm * i_cell_A));
        ams_ekf_r0_update_result_t r0_result = AMS_EKF_R0_UPDATE_APPLIED;
        const bool ok = ams_ekf_step_acquiring_gated(&ekf,
                                                      current_A,
                                                      v_meas_V,
                                                      25.0f,
                                                      0.1f,
                                                      &r0_result);
        EXPECT_TRUE(ok);
        EXPECT_TRUE(r0_result == AMS_EKF_R0_UPDATE_NOT_REQUESTED);
        ams_ekf_acquisition_observe(&ekf,
                                    current_A,
                                    0.0f,
                                    v_meas_V,
                                    25.0f,
                                    (uint32_t)k * 100u,
                                    true,
                                    ok);
    }

    EXPECT_FALSE(ams_ekf_acquisition_complete(&ekf));
    EXPECT_TRUE(ekf.acquisition.state == AMS_EKF_ACQ_WAITING);
    EXPECT_TRUE(ekf.acquisition.reason ==
                AMS_EKF_ACQ_REASON_WAITING_FOR_LOW_CURRENT);
    EXPECT_TRUE(ekf.acquisition.anchor_count == 0u);
    EXPECT_TRUE(ekf.acquisition.dynamic_step_count > 0u);
    EXPECT_TRUE(ekf.acquisition.dynamic_update_count > 0u);
    EXPECT_TRUE(fabsf(ekf.soc - 0.60f) < initial_error);
    EXPECT_TRUE(ekf.p_soc >=
        (AMS_EKF_ACQ_DYNAMIC_SOC_SIGMA_FLOOR *
         AMS_EKF_ACQ_DYNAMIC_SOC_SIGMA_FLOOR));
    EXPECT_NEAR(ekf.p_soc_vp1, 0.0f, TEST_EPS_SMALL);
    EXPECT_NEAR(ekf.p_soc_vp2, 0.0f, TEST_EPS_SMALL);
    EXPECT_NEAR(ekf.p_vp1_vp2, 0.0f, TEST_EPS_SMALL);
    EXPECT_TRUE(unit_covariance_psd(&ekf));
}

static void test_full_covariance_measurement_update(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_segment_config(&cfg, 0u);
    cfg.soc_init = 0.60f;
    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);

    const float voltage_V = 15.0f * ams_p42a_ocv_v(0.60f, 25.0f);
    EXPECT_TRUE(ams_ekf_step(&ekf, 0.0f, voltage_V, 25.0f, 0.1f));
    EXPECT_TRUE(unit_covariance_psd(&ekf));
    EXPECT_FINITE(ekf.p_soc_vp1);
    EXPECT_FINITE(ekf.p_soc_vp2);
    EXPECT_FINITE(ekf.p_vp1_vp2);
    EXPECT_TRUE((fabsf(ekf.p_soc_vp1) + fabsf(ekf.p_soc_vp2) +
                 fabsf(ekf.p_vp1_vp2)) > 1.0e-10f);
    EXPECT_TRUE(isfinite(ekf.innovation_variance_V2) &&
                ekf.innovation_variance_V2 > 0.0f);
}

static void test_covariance_temperature_floors_and_topology_r(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_segment_config(&cfg, 0u);
    cfg.soc_init = 0.60f;
    const float voltage_25 = 15.0f * ams_p42a_ocv_v(0.60f, 25.0f);
    const float voltage_5 = 15.0f * ams_p42a_ocv_v(0.60f, 5.0f);

    ams_ekf_instance_t nominal;
    ams_ekf_init(&nominal, &cfg);
    nominal.p_soc = nominal.p_vp1 = nominal.p_vp2 = 1.0e-12f;
    nominal.p_soc_vp1 = nominal.p_soc_vp2 = nominal.p_vp1_vp2 = 0.0f;
    nominal.r_meas_V2 = 0.0f;
    nominal.step_count = AMS_EKF_ACQUISITION_STEPS;
    EXPECT_TRUE(ams_ekf_step(&nominal,0.0f,voltage_25,25.0f,0.1f));
    EXPECT_TRUE(nominal.p_soc >= AMS_EKF_SOC_SIGMA_FLOOR_NOMINAL *
                                  AMS_EKF_SOC_SIGMA_FLOOR_NOMINAL);
    EXPECT_TRUE(nominal.p_vp1 >= AMS_EKF_VP_SIGMA_FLOOR_NOMINAL_V *
                                  AMS_EKF_VP_SIGMA_FLOOR_NOMINAL_V);
    EXPECT_TRUE(nominal.r_meas_V2 >=
        (15.0f * AMS_EKF_R_SIGMA_FLOOR_PER_CELL_V) *
        (15.0f * AMS_EKF_R_SIGMA_FLOOR_PER_CELL_V));

    ams_ekf_instance_t cold;
    ams_ekf_init(&cold, &cfg);
    cold.p_soc = cold.p_vp1 = cold.p_vp2 = 1.0e-12f;
    cold.p_soc_vp1 = cold.p_soc_vp2 = cold.p_vp1_vp2 = 0.0f;
    cold.r_meas_V2 = 0.0f;
    cold.step_count = AMS_EKF_ACQUISITION_STEPS;
    EXPECT_TRUE(ams_ekf_step(&cold,0.0f,voltage_5,5.0f,0.1f));
    EXPECT_TRUE(cold.p_soc >= AMS_EKF_SOC_SIGMA_FLOOR_TEMP_EDGE *
                               AMS_EKF_SOC_SIGMA_FLOOR_TEMP_EDGE);
    EXPECT_TRUE(cold.p_vp1 >= AMS_EKF_VP_SIGMA_FLOOR_TEMP_EDGE_V *
                               AMS_EKF_VP_SIGMA_FLOOR_TEMP_EDGE_V);
    EXPECT_TRUE(cold.r_meas_V2 >=
        (15.0f * AMS_EKF_R_SIGMA_FLOOR_TEMP_EDGE_PER_CELL_V) *
        (15.0f * AMS_EKF_R_SIGMA_FLOOR_TEMP_EDGE_PER_CELL_V));
    EXPECT_TRUE(cold.p_soc > nominal.p_soc);
    EXPECT_TRUE(cold.r_meas_V2 > nominal.r_meas_V2);
    EXPECT_TRUE(unit_covariance_psd(&nominal));
    EXPECT_TRUE(unit_covariance_psd(&cold));
}

static void test_soh_advisory_invalid_until_acquisition_complete(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_segment_config(&cfg, 0u);
    cfg.soc_init = 0.50f;
    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);
    ekf.valid = 1u;
    ekf.p_r0 = AMS_SOH_MAX_R0_VARIANCE_OHM2 * 0.5f;

    ams_resistance_soh_t soh;
    memset(&soh, 0, sizeof(soh));
    soh.accepted_count = AMS_SOH_MIN_ACCEPTED_OBSERVATIONS;
    soh.last_accept_tick = 1000u;

    ams_resistance_soh_record(&soh,
                              &ekf,
                              1u,
                              1100u,
                              true,
                              AMS_SOH_REJECT_ACQUISITION,
                              AMS_EKF_R0_UPDATE_NOT_REQUESTED,
                              true);
    EXPECT_TRUE(soh.reject_acquisition_count == 1u);
    EXPECT_TRUE((soh.status_flags & AMS_SOH_STATUS_CONVERGED) != 0u);
    EXPECT_TRUE((soh.status_flags & AMS_SOH_STATUS_ADVISORY_VALID) == 0u);
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

static void test_ekf_innovation_gate_and_dt_observability(void)
{
    ams_ekf_config_t cfg;
    ams_ekf_make_pack_config(&cfg);
    cfg.soc_init = 0.50f;

    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);
    const float nominal_v = (float)cfg.series_group_count *
        ams_p42a_ocv_v(cfg.soc_init, 25.0f);

    /* Bootstrap is intentionally looser than the steady-state statistical
     * gate so a reasonable configured-SoC mismatch can acquire. It must still
     * reject a gross bad-but-range-valid first sample. */
    {
        ams_ekf_instance_t fresh;
        ams_ekf_init(&fresh, &cfg);
        const float collapsed_v =
            (float)cfg.series_group_count * 2.10f;
        EXPECT_TRUE(ams_ekf_step(&fresh, 0.0f, collapsed_v, 25.0f, 0.1f));
        EXPECT_TRUE((fresh.fault_flags &
                     AMS_EKF_FAULT_INNOVATION_REJECT) != 0u);
        EXPECT_TRUE(fresh.innovation_reject_count == 1u);
        EXPECT_TRUE(fresh.step_count == 1u);
    }

    for(uint8_t i = 0u; i < 20u; i++)
    {
        EXPECT_TRUE(ams_ekf_step(&ekf, 0.0f, nominal_v, 25.0f, 0.1f));
        EXPECT_TRUE((ekf.fault_flags & AMS_EKF_FAULT_INNOVATION_REJECT) == 0u);
    }

    const float soc_before = ekf.soc;
    const float r_before = ekf.r_meas_V2;
    ams_ekf_r0_update_result_t r0_result = AMS_EKF_R0_UPDATE_APPLIED;
    /* 90 mV/cell is inside the broad 2.0-4.5 V input sanity window and below
     * the 100 mV/cell absolute SoH gate, but it is a gross statistical
     * innovation after the estimator has a valid prior. */
    const float bad_v = nominal_v -
        ((float)cfg.series_group_count * 0.090f);
    EXPECT_TRUE(ams_ekf_step_gated(&ekf,
                                   0.0f,
                                   bad_v,
                                   25.0f,
                                   0.1f,
                                   true,
                                   &r0_result));
    EXPECT_TRUE((ekf.fault_flags & AMS_EKF_FAULT_INNOVATION_REJECT) != 0u);
    EXPECT_TRUE(ekf.innovation_reject_count == 1u);
    EXPECT_TRUE(r0_result == AMS_EKF_R0_UPDATE_REJECT_INNOVATION);
    EXPECT_NEAR(ekf.r_meas_V2, r_before, TEST_EPS_SMALL);
    EXPECT_NEAR(ekf.soc, soc_before, 1.0e-4f);

    /* A valid sample clears the one-step rejection flag. */
    EXPECT_TRUE(ams_ekf_step(&ekf, 0.0f, ekf.v_pred_V, 25.0f, 0.1f));
    EXPECT_TRUE((ekf.fault_flags & AMS_EKF_FAULT_INNOVATION_REJECT) == 0u);
    EXPECT_TRUE(ekf.innovation_reject_count == 1u);

    ams_ekf_init(&ekf, &cfg);
    EXPECT_TRUE(ams_ekf_step(&ekf, 0.0f, nominal_v, 25.0f, 5.0f));
    EXPECT_TRUE((ekf.fault_flags & AMS_EKF_FAULT_DT_CLAMPED) != 0u);
    EXPECT_TRUE(ekf.dt_clamp_count == 1u);
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
        EXPECT_FINITE(ekf.p_soc_vp1);
        EXPECT_FINITE(ekf.p_soc_vp2);
        EXPECT_FINITE(ekf.p_vp1);
        EXPECT_FINITE(ekf.p_vp1_vp2);
        EXPECT_FINITE(ekf.p_vp2);
        EXPECT_FINITE(ekf.p_r0);
        EXPECT_TRUE(unit_covariance_psd(&ekf));
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
    EXPECT_TRUE(est.cc_last_dt_clamped == 1U);
    EXPECT_TRUE(est.cc_dt_clamp_count == 1u);
    EXPECT_TRUE(est.cc_soc < 1.0f);
    float after_discharge = est.cc_soc;

    EXPECT_TRUE(ams_estimator_cc_step(&est, -30.0f, 60.0f));
    EXPECT_TRUE(est.cc_last_dt_clamped == 1U);
    EXPECT_TRUE(est.cc_dt_clamp_count == 2u);
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
    EXPECT_TRUE(est.cc_last_dt_clamped == 0U);
    EXPECT_TRUE(est.cc_soc < before_bad_sample);

    ams_estimator_cc_reset(&est, 0.50f);
    EXPECT_TRUE(est.cc_valid == 1U);
    EXPECT_TRUE(est.cc_dt_clamp_count == 0u);
    EXPECT_NEAR(est.cc_soc, 0.50f, TEST_EPS_SMALL);

    est.inst[0].valid = 0U;
    ams_estimator_refresh_summary(&est, AMS_ESTIMATOR_INPUT_HARDWARE, 77U);
    EXPECT_NEAR(est.pack_soc, est.cc_soc, TEST_EPS_SMALL);
    EXPECT_TRUE((ams_estimator_status_flags(&est) & AMS_EKF_FLAG_CC_FALLBACK) != 0U);
}

static uint16_t unit_adc_count_for_mcu_voltage(float voltage_v)
{
    return (uint16_t)((voltage_v * 4095.0f / 3.3f) + 0.5f);
}

static uint16_t unit_adc_count_for_sensor_voltage(float sensor_voltage_v)
{
    return unit_adc_count_for_mcu_voltage(sensor_voltage_v * 0.6f);
}

static uint16_t unit_adc_count_for_sensor_voltage_vref(float sensor_voltage_v,
                                                       float adc_vref_v)
{
    float count = (sensor_voltage_v * 0.6f * 4095.0f) / adc_vref_v;
    if(count < 0.0f)
    {
        count = 0.0f;
    }
    if(count > 4095.0f)
    {
        count = 4095.0f;
    }
    return (uint16_t)lroundf(count);
}

static void unit_adc_set_sequence(uint16_t high_count, uint16_t low_count)
{
    unit_adc_read_counts[0] = high_count;
    unit_adc_read_counts[1] = low_count;
    unit_adc_read_statuses[0] = HAL_OK;
    unit_adc_read_statuses[1] = HAL_OK;
    unit_adc_read_index = 0u;
}

static void unit_adc_set_status_sequence(HAL_StatusTypeDef high_status, HAL_StatusTypeDef low_status)
{
    unit_adc_read_counts[0] = 0u;
    unit_adc_read_counts[1] = 0u;
    unit_adc_read_statuses[0] = high_status;
    unit_adc_read_statuses[1] = low_status;
    unit_adc_read_index = 0u;
}


static void unit_current_sensor_mark_fresh(current_sensor_t *sensor)
{
    sensor->last_read_ok = true;
    sensor->count_high_fresh = true;
    sensor->count_low_fresh = true;
}

static void test_current_sensor_conversion_zero_and_range_selection(void)
{
    current_sensor_t sensor = {0};
    float current;

    sensor.count_high = unit_adc_count_for_sensor_voltage(2.5f);
    sensor.count_low = unit_adc_count_for_sensor_voltage(2.5f);
    unit_current_sensor_mark_fresh(&sensor);
    current = current_sensor_convert(&sensor);

    EXPECT_TRUE(sensor.current_valid);
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_OK);
    EXPECT_TRUE(sensor.selected_range == CURRENT_SENSOR_RANGE_50A);
    EXPECT_NEAR(current, 0.0f, 0.05f);
    EXPECT_NEAR(sensor.sensor_voltage_high, 2.5f, 0.01f);
    EXPECT_NEAR(sensor.sensor_voltage_low, 2.5f, 0.01f);

    sensor = (current_sensor_t){0};
    sensor.count_high = unit_adc_count_for_sensor_voltage(2.55f);
    sensor.count_low = unit_adc_count_for_sensor_voltage(3.3f);
    unit_current_sensor_mark_fresh(&sensor);
    current = current_sensor_convert(&sensor);

    EXPECT_TRUE(sensor.current_valid);
    EXPECT_TRUE(sensor.selected_range == CURRENT_SENSOR_RANGE_50A);
    EXPECT_NEAR(current, 20.0f, 0.25f);
    EXPECT_NEAR(sensor.current_50a, 20.0f, 0.25f);
    EXPECT_NEAR(sensor.current_800a, 20.0f, 1.0f);

    sensor = (current_sensor_t){0};
    sensor.count_high = unit_adc_count_for_sensor_voltage(2.65f);
    sensor.count_low = unit_adc_count_for_sensor_voltage(4.75f);
    unit_current_sensor_mark_fresh(&sensor);
    current = current_sensor_convert(&sensor);

    EXPECT_TRUE(sensor.current_valid);
    EXPECT_TRUE(sensor.selected_range == CURRENT_SENSOR_RANGE_800A);
    EXPECT_NEAR(current, 60.0f, 1.0f);
}

static void test_current_sensor_invalid_conditions(void)
{
    current_sensor_t sensor = {0};

    sensor.count_high = unit_adc_count_for_sensor_voltage(4.75f);
    sensor.count_low = unit_adc_count_for_sensor_voltage(4.75f);
    unit_current_sensor_mark_fresh(&sensor);
    (void)current_sensor_convert(&sensor);
    EXPECT_FALSE(sensor.current_valid);
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_SENSOR_SATURATION);

    sensor = (current_sensor_t){0};
    sensor.count_high = unit_adc_count_for_sensor_voltage(2.5f);
    sensor.count_low = unit_adc_count_for_sensor_voltage(3.3f);
    unit_current_sensor_mark_fresh(&sensor);
    (void)current_sensor_convert(&sensor);
    EXPECT_FALSE(sensor.current_valid);
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_CHANNEL_MISMATCH);

    sensor = (current_sensor_t){0};
    sensor.count_high = 3900u;
    sensor.count_low = unit_adc_count_for_sensor_voltage(2.5f);
    unit_current_sensor_mark_fresh(&sensor);
    (void)current_sensor_convert(&sensor);
    EXPECT_FALSE(sensor.current_valid);
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_ADC_IMPLAUSIBLE);

    sensor = (current_sensor_t){0};
    sensor.count_high = unit_adc_count_for_sensor_voltage(2.5f);
    sensor.count_low = unit_adc_count_for_sensor_voltage(2.5f);
    sensor.last_read_ok = false;
    sensor.count_high_fresh = true;
    sensor.count_low_fresh = false;
    (void)current_sensor_convert(&sensor);
    EXPECT_FALSE(sensor.current_valid);
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_ADC_READ);

    EXPECT_TRUE(strcmp(current_sensor_reason_str(CURRENT_SENSOR_REASON_ADC_READ), "adc_read") == 0);
    EXPECT_TRUE(strcmp(current_sensor_range_str(CURRENT_SENSOR_RANGE_50A), "50A") == 0);
}

static void test_current_sensor_read_adc_status_path(void)
{
    static ADC_HandleTypeDef adc_high;
    static ADC_HandleTypeDef adc_low;
    current_sensor_t sensor;

    current_sensor_init(&sensor, &adc_low, &adc_high, 14u, 1u);
    unit_adc_set_sequence(unit_adc_count_for_sensor_voltage(2.5f),
                          unit_adc_count_for_sensor_voltage(2.5f));
    EXPECT_TRUE(current_sensor_read_adc(&sensor));
    EXPECT_TRUE(sensor.last_read_ok);
    EXPECT_TRUE(sensor.count_high == unit_adc_count_for_sensor_voltage(2.5f));
    EXPECT_TRUE(sensor.count_low == unit_adc_count_for_sensor_voltage(2.5f));

    current_sensor_init(&sensor, &adc_low, &adc_high, 14u, 1u);
    unit_adc_set_status_sequence(HAL_TIMEOUT, HAL_OK);
    EXPECT_FALSE(current_sensor_read_adc(&sensor));
    EXPECT_FALSE(sensor.last_read_ok);
    EXPECT_FALSE(sensor.current_valid);
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_ADC_READ);

    current_sensor_init(&sensor, NULL, &adc_high, 14u, 1u);
    EXPECT_FALSE(current_sensor_read_adc(&sensor));
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_ADC_READ);
}


static void test_current_fault_policy(void)
{
    current_fault_state_t fault;

    current_fault_init(&fault);
    for(int i = 0; i < 24; i++)
    {
        current_fault_update(&fault,
                             CURRENT_FAULT_MODE_DRIVE,
                             90.0f,
                             true,
                             CURRENT_SENSOR_REASON_OK,
                             20u);
    }
    EXPECT_FALSE(fault.confirmed);
    EXPECT_TRUE(fault.pending);
    EXPECT_TRUE(fault.pending_reason == CURRENT_FAULT_REASON_DISCHARGE_OVERCURRENT);

    current_fault_update(&fault,
                         CURRENT_FAULT_MODE_DRIVE,
                         90.0f,
                         true,
                         CURRENT_SENSOR_REASON_OK,
                         20u);
    EXPECT_TRUE(fault.confirmed);
    EXPECT_TRUE(fault.latched);
    EXPECT_TRUE(fault.latched_reason == CURRENT_FAULT_REASON_DISCHARGE_OVERCURRENT);

    current_fault_init(&fault);
    current_fault_update(&fault,
                         CURRENT_FAULT_MODE_PRECHARGE,
                         2.5f,
                         true,
                         CURRENT_SENSOR_REASON_OK,
                         20u);
    EXPECT_TRUE(fault.pending);
    EXPECT_FALSE(fault.confirmed);
    current_fault_update(&fault,
                         CURRENT_FAULT_MODE_PRECHARGE,
                         2.5f,
                         true,
                         CURRENT_SENSOR_REASON_OK,
                         20u);
    EXPECT_TRUE(fault.confirmed);
    EXPECT_TRUE(fault.latched_reason == CURRENT_FAULT_REASON_PRECHARGE_FAST_OVERCURRENT);

    current_fault_init(&fault);
    for(int i = 0; i < 25; i++)
    {
        current_fault_update(&fault,
                             CURRENT_FAULT_MODE_DRIVE,
                             0.0f,
                             false,
                             CURRENT_SENSOR_REASON_ADC_READ,
                             20u);
    }
    EXPECT_TRUE(fault.sensor_fault);
    EXPECT_TRUE(fault.reason == CURRENT_FAULT_REASON_SENSOR_ADC_READ);

    current_fault_init(&fault);
    current_fault_update(&fault,
                         CURRENT_FAULT_MODE_DRIVE,
                         -6.0f,
                         true,
                         CURRENT_SENSOR_REASON_OK,
                         20u);
    EXPECT_TRUE(fault.warning);
    EXPECT_TRUE(fault.reason == CURRENT_FAULT_REASON_REGEN_UNEXPECTED);
    EXPECT_TRUE(strcmp(current_fault_reason_str(CURRENT_FAULT_REASON_PRECHARGE_OVERCURRENT), "precharge_overcurrent") == 0);
    EXPECT_TRUE(strcmp(current_fault_mode_str(CURRENT_FAULT_MODE_DRIVE), "drive") == 0);
}


static void test_current_sensor_requires_fresh_pair_and_channel_mapping(void)
{
    current_sensor_t sensor = {0};

    sensor.count_high = unit_adc_count_for_sensor_voltage(2.5625f); /* 25A on 800A channel */
    sensor.count_low = unit_adc_count_for_sensor_voltage(3.5000f);  /* 25A on 50A channel */
    sensor.last_read_ok = true;
    sensor.count_high_fresh = true;
    sensor.count_low_fresh = false;
    (void)current_sensor_convert(&sensor);
    EXPECT_FALSE(sensor.current_valid);
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_ADC_READ);

    sensor = (current_sensor_t){0};
    sensor.count_high = unit_adc_count_for_sensor_voltage(2.5625f);
    sensor.count_low = unit_adc_count_for_sensor_voltage(3.5000f);
    unit_current_sensor_mark_fresh(&sensor);
    (void)current_sensor_convert(&sensor);
    EXPECT_TRUE(sensor.current_valid);
    EXPECT_TRUE(sensor.selected_range == CURRENT_SENSOR_RANGE_50A);
    EXPECT_NEAR(sensor.current_50a, 25.0f, 0.30f);
    EXPECT_NEAR(sensor.current_800a, 25.0f, 1.00f);
    EXPECT_NEAR(sensor.current, 25.0f, 0.30f);

    sensor = (current_sensor_t){0};
    sensor.count_high = unit_adc_count_for_sensor_voltage(2.7000f); /* 80A on 800A channel */
    sensor.count_low = unit_adc_count_for_sensor_voltage(4.7500f);  /* 50A channel saturated */
    unit_current_sensor_mark_fresh(&sensor);
    (void)current_sensor_convert(&sensor);
    EXPECT_TRUE(sensor.current_valid);
    EXPECT_TRUE(sensor.selected_range == CURRENT_SENSOR_RANGE_800A);
    EXPECT_NEAR(sensor.current, 80.0f, 1.50f);
}


static void test_current_sensor_zero_cal_and_hysteresis(void)
{
    current_sensor_t sensor = {0};

    /* A failed conversion must never be accepted as a zero capture. This
     * covers the bench failure where the 800 A ADC channel was near ground
     * while stale raw-current members still contained finite zeroes. */
    sensor.count_high = 1u;
    sensor.count_low = unit_adc_count_for_sensor_voltage(2.500f);
    unit_current_sensor_mark_fresh(&sensor);
    (void)current_sensor_convert(&sensor);
    EXPECT_FALSE(sensor.current_valid);
    EXPECT_FALSE(current_sensor_zero_calibrate(&sensor));
    EXPECT_FALSE(sensor.zero_calibrated);
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_ZERO_CAL_REJECTED);

    sensor = (current_sensor_t){0};

    sensor.count_high = unit_adc_count_for_sensor_voltage(2.505f);  /* +2 A on 800 A channel */
    sensor.count_low = unit_adc_count_for_sensor_voltage(2.540f);   /* +1 A on 50 A channel */
    unit_current_sensor_mark_fresh(&sensor);
    (void)current_sensor_convert(&sensor);
    EXPECT_TRUE(sensor.current_valid);
    EXPECT_TRUE(current_sensor_zero_calibrate(&sensor));
    EXPECT_TRUE(sensor.zero_calibrated);
    EXPECT_TRUE(sensor.zero_cal_count == 1u);
    EXPECT_FALSE(sensor.current_valid);
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_CALIBRATION_CHANGED);

    (void)current_sensor_convert(&sensor);
    EXPECT_TRUE(sensor.current_valid);
    EXPECT_NEAR(sensor.current_50a, 0.0f, 0.05f);
    EXPECT_NEAR(sensor.current_800a, 0.0f, 0.20f);
    EXPECT_NEAR(sensor.current, 0.0f, 0.05f);

    current_sensor_zero_clear(&sensor);
    EXPECT_FALSE(sensor.zero_calibrated);
    EXPECT_FALSE(sensor.current_valid);
    EXPECT_TRUE(sensor.reason == CURRENT_SENSOR_REASON_CALIBRATION_CHANGED);

    sensor = (current_sensor_t){0};
    sensor.count_high = unit_adc_count_for_sensor_voltage(2.650f);  /* +60 A on 800 A channel */
    sensor.count_low = unit_adc_count_for_sensor_voltage(4.750f);   /* 50 A channel at clamp */
    unit_current_sensor_mark_fresh(&sensor);
    (void)current_sensor_convert(&sensor);
    EXPECT_TRUE(sensor.current_valid);
    EXPECT_TRUE(sensor.selected_range == CURRENT_SENSOR_RANGE_800A);

    sensor.count_high = unit_adc_count_for_sensor_voltage(2.600f);  /* +40 A on 800 A channel */
    sensor.count_low = unit_adc_count_for_sensor_voltage(4.100f);   /* +40 A on 50 A channel */
    unit_current_sensor_mark_fresh(&sensor);
    (void)current_sensor_convert(&sensor);
    EXPECT_TRUE(sensor.current_valid);
    EXPECT_TRUE(sensor.selected_range == CURRENT_SENSOR_RANGE_800A);

    sensor.count_high = unit_adc_count_for_sensor_voltage(2.590f);  /* +36 A on 800 A channel */
    sensor.count_low = unit_adc_count_for_sensor_voltage(3.940f);   /* +36 A on 50 A channel */
    unit_current_sensor_mark_fresh(&sensor);
    (void)current_sensor_convert(&sensor);
    EXPECT_TRUE(sensor.current_valid);
    EXPECT_TRUE(sensor.selected_range == CURRENT_SENSOR_RANGE_50A);
}

static void test_current_sensor_calibration_record_integrity(void)
{
    current_sensor_t source;
    current_sensor_t restored;
    current_sensor_calibration_record_t record;
    current_sensor_calibration_record_t corrupted;
    current_sensor_calibration_metadata_t metadata = {
        .calibration_id = 42u,
        .capture_time_s = 1784563200u,
        .calibration_temp_deci_c = 235,
        .uncertainty_50a_mA = 200u,
        .uncertainty_800a_mA = 2000u
    };

    current_sensor_init(&source, NULL, NULL, 0u, 0u);
    source.count_high = unit_adc_count_for_sensor_voltage(2.505f);
    source.count_low = unit_adc_count_for_sensor_voltage(2.540f);
    unit_current_sensor_mark_fresh(&source);
    (void)current_sensor_convert(&source);
    EXPECT_TRUE(current_sensor_zero_calibrate(&source));
    EXPECT_FALSE(current_sensor_calibration_confident(&source));
    EXPECT_TRUE(current_sensor_calibration_record_create(&source,
                                                          &metadata,
                                                          &record));
    EXPECT_TRUE(current_sensor_calibration_record_valid(&record));
    EXPECT_TRUE(record.magic == CURRENT_SENSOR_CALIBRATION_MAGIC);
    EXPECT_TRUE(record.schema == CURRENT_SENSOR_CALIBRATION_SCHEMA);
    EXPECT_TRUE(record.size == CURRENT_SENSOR_CALIBRATION_RECORD_SIZE);
    EXPECT_TRUE(record.calibration_id == metadata.calibration_id);

    corrupted = record;
    corrupted.zero_offset_50a_mA++;
    EXPECT_FALSE(current_sensor_calibration_record_valid(&corrupted));
    corrupted = record;
    corrupted.magic ^= 1u;
    EXPECT_FALSE(current_sensor_calibration_record_valid(&corrupted));

    current_sensor_init(&restored, NULL, NULL, 0u, 0u);
    /* Restoring a record requires a newly sampled, physically proven zero
     * whose live offsets agree with the stored board calibration. */
    restored.count_high = source.count_high;
    restored.count_low = source.count_low;
    unit_current_sensor_mark_fresh(&restored);
    (void)current_sensor_convert(&restored);
    EXPECT_FALSE(current_sensor_calibration_apply(&restored,
                                                   &record,
                                                   false));
    EXPECT_FALSE(restored.zero_calibrated);

    current_sensor_t wrong_board;
    current_sensor_init(&wrong_board, NULL, NULL, 0u, 0u);
    wrong_board.count_high = unit_adc_count_for_sensor_voltage(2.5f);
    wrong_board.count_low = unit_adc_count_for_sensor_voltage(2.5f);
    unit_current_sensor_mark_fresh(&wrong_board);
    (void)current_sensor_convert(&wrong_board);
    EXPECT_FALSE(current_sensor_calibration_apply(&wrong_board,
                                                   &record,
                                                   true));
    EXPECT_FALSE(wrong_board.zero_calibrated);

    EXPECT_TRUE(current_sensor_calibration_apply(&restored,
                                                  &record,
                                                  true));
    EXPECT_TRUE(restored.zero_calibrated);
    EXPECT_TRUE(restored.calibration_loaded_from_record);
    EXPECT_TRUE(restored.calibration_restore_count == 1u);
    EXPECT_TRUE(restored.calibration_id == metadata.calibration_id);
    EXPECT_TRUE(restored.calibration_capture_time_s ==
                metadata.capture_time_s);
    EXPECT_TRUE(restored.calibration_temp_deci_c ==
                metadata.calibration_temp_deci_c);
    EXPECT_NEAR(restored.zero_offset_50a,
                source.zero_offset_50a,
                0.0011f);
    EXPECT_NEAR(restored.zero_offset_800a,
                source.zero_offset_800a,
                0.0011f);
    EXPECT_TRUE(current_sensor_calibration_confident(&restored));
    EXPECT_FALSE(restored.current_valid);
    EXPECT_TRUE(restored.reason == CURRENT_SENSOR_REASON_CALIBRATION_CHANGED);

    restored.calibration_capture_time_s =
        CURRENT_SENSOR_CALIBRATION_TIME_UNKNOWN;
    EXPECT_FALSE(current_sensor_calibration_confident(&restored));
    restored.calibration_capture_time_s = metadata.capture_time_s;
    restored.calibration_temp_deci_c = 1201;
    EXPECT_FALSE(current_sensor_calibration_confident(&restored));
    restored.calibration_temp_deci_c = metadata.calibration_temp_deci_c;
    restored.calibration_uncertainty_50a_mA = 0u;
    EXPECT_FALSE(current_sensor_calibration_confident(&restored));
    restored.calibration_uncertainty_50a_mA = metadata.uncertainty_50a_mA;
    EXPECT_TRUE(current_sensor_calibration_confident(&restored));

    current_sensor_set_reference_voltages(&restored, 3.2f, 4.9f);
    EXPECT_FALSE(restored.zero_calibrated);
    EXPECT_FALSE(restored.calibration_loaded_from_record);
    EXPECT_FALSE(current_sensor_calibration_confident(&restored));

    metadata.uncertainty_50a_mA = 501u;
    metadata.uncertainty_800a_mA = 5001u;
    EXPECT_TRUE(current_sensor_calibration_record_create(&source,
                                                          &metadata,
                                                          &record));
    current_sensor_init(&restored, NULL, NULL, 0u, 0u);
    restored.count_high = source.count_high;
    restored.count_low = source.count_low;
    unit_current_sensor_mark_fresh(&restored);
    EXPECT_TRUE(current_sensor_calibration_apply(&restored,
                                                  &record,
                                                  true));
    EXPECT_FALSE(current_sensor_calibration_confident(&restored));

    current_sensor_zero_clear(&restored);
    restored.count_high = unit_adc_count_for_sensor_voltage(2.5f);
    restored.count_low = unit_adc_count_for_sensor_voltage(2.74f);
    unit_current_sensor_mark_fresh(&restored);
    EXPECT_FALSE(current_sensor_calibration_apply(&restored,
                                                   &record,
                                                   true));
    EXPECT_FALSE(restored.zero_calibrated);

    metadata.calibration_temp_deci_c = 1201;
    EXPECT_FALSE(current_sensor_calibration_record_create(&source,
                                                           &metadata,
                                                           &record));

    /* Restore must evaluate the live ADC pair using the reference voltages
     * captured in the record, not whatever defaults happen to be loaded in
     * the destination object before restore. */
    current_sensor_t referenced_source;
    current_sensor_t referenced_restore;
    current_sensor_calibration_record_t referenced_record;
    metadata.calibration_temp_deci_c = 250;
    metadata.uncertainty_50a_mA = 300u;
    metadata.uncertainty_800a_mA = 3000u;
    metadata.calibration_id = 43u;
    current_sensor_init(&referenced_source, NULL, NULL, 0u, 0u);
    current_sensor_set_reference_voltages(&referenced_source, 3.2f, 4.9f);
    referenced_source.count_low =
        unit_adc_count_for_sensor_voltage_vref(2.4892f, 3.2f);
    referenced_source.count_high =
        unit_adc_count_for_sensor_voltage_vref(2.4549f, 3.2f);
    unit_current_sensor_mark_fresh(&referenced_source);
    (void)current_sensor_convert(&referenced_source);
    EXPECT_TRUE(current_sensor_zero_calibrate(&referenced_source));
    EXPECT_TRUE(current_sensor_calibration_record_create(&referenced_source,
                                                          &metadata,
                                                          &referenced_record));

    current_sensor_init(&referenced_restore, NULL, NULL, 0u, 0u);
    referenced_restore.count_low = referenced_source.count_low;
    referenced_restore.count_high = referenced_source.count_high;
    unit_current_sensor_mark_fresh(&referenced_restore);
    EXPECT_TRUE(current_sensor_calibration_apply(&referenced_restore,
                                                  &referenced_record,
                                                  true));
    EXPECT_NEAR(referenced_restore.adc_vref_v, 3.2f, 0.00001f);
    EXPECT_NEAR(referenced_restore.sensor_supply_v, 4.9f, 0.00001f);
    EXPECT_FALSE(referenced_restore.current_valid);
    (void)current_sensor_convert(&referenced_restore);
    EXPECT_TRUE(referenced_restore.current_valid);
    EXPECT_NEAR(referenced_restore.current, 0.0f, 0.30f);
}

static void test_current_fault_threshold_edges_and_recovery(void)
{
    current_fault_state_t fault;

    current_fault_init(&fault);
    current_fault_update(&fault,
                         CURRENT_FAULT_MODE_DRIVE,
                         75.0f,
                         true,
                         CURRENT_SENSOR_REASON_OK,
                         20u);
    EXPECT_TRUE(fault.warning);
    EXPECT_FALSE(fault.pending);
    EXPECT_FALSE(fault.confirmed);
    EXPECT_TRUE(fault.reason == CURRENT_FAULT_REASON_DISCHARGE_WARNING);

    current_fault_update(&fault,
                         CURRENT_FAULT_MODE_DRIVE,
                         0.0f,
                         true,
                         CURRENT_SENSOR_REASON_OK,
                         20u);
    EXPECT_FALSE(fault.warning);
    EXPECT_FALSE(fault.pending);
    EXPECT_FALSE(fault.confirmed);
    EXPECT_FALSE(fault.latched);

    current_fault_init(&fault);
    for(uint8_t i = 0u; i < 4u; i++)
    {
        current_fault_update(&fault,
                             CURRENT_FAULT_MODE_DRIVE,
                             125.0f,
                             true,
                             CURRENT_SENSOR_REASON_OK,
                             20u);
    }
    EXPECT_TRUE(fault.pending);
    EXPECT_FALSE(fault.confirmed);
    current_fault_update(&fault,
                         CURRENT_FAULT_MODE_DRIVE,
                         125.0f,
                         true,
                         CURRENT_SENSOR_REASON_OK,
                         20u);
    EXPECT_TRUE(fault.confirmed);
    EXPECT_TRUE(fault.latched);
    EXPECT_TRUE(fault.latched_reason == CURRENT_FAULT_REASON_DISCHARGE_FAST_OVERCURRENT);

    current_fault_reset_latch(&fault);
    EXPECT_FALSE(fault.latched);
    EXPECT_TRUE(fault.latched_reason == CURRENT_FAULT_REASON_NONE);

    current_fault_init(&fault);
    for(uint8_t i = 0u; i < 25u; i++)
    {
        current_fault_update(&fault,
                             CURRENT_FAULT_MODE_CHARGE,
                             -12.5f,
                             true,
                             CURRENT_SENSOR_REASON_OK,
                             20u);
    }
    EXPECT_TRUE(fault.confirmed);
    EXPECT_TRUE(fault.latched_reason == CURRENT_FAULT_REASON_CHARGE_OVERCURRENT);

    current_fault_init(&fault);
    current_fault_update(&fault,
                         CURRENT_FAULT_MODE_DRIVE,
                         -6.0f,
                         true,
                         CURRENT_SENSOR_REASON_OK,
                         20u);
    EXPECT_TRUE(fault.warning);
    EXPECT_FALSE(fault.confirmed);
    EXPECT_FALSE(fault.latched);
    EXPECT_TRUE(fault.reason == CURRENT_FAULT_REASON_REGEN_UNEXPECTED);

    current_fault_init(&fault);
    for(uint8_t i = 0u; i < 25u; i++)
    {
        current_fault_update(&fault,
                             CURRENT_FAULT_MODE_DRIVE,
                             -26.0f,
                             true,
                             CURRENT_SENSOR_REASON_OK,
                             20u);
    }
    EXPECT_FALSE(fault.warning);
    EXPECT_TRUE(fault.confirmed);
    EXPECT_TRUE(fault.latched);
    EXPECT_TRUE(fault.latched_reason == CURRENT_FAULT_REASON_REGEN_OVERCURRENT);
}

static void unit_fill_voltage_acc(accumulator_t *acc,
                                  uint16_t min_mv,
                                  uint16_t max_mv,
                                  bool startup_done,
                                  bool full_usable,
                                  bool full_updated)
{
    if(acc == NULL)
    {
        return;
    }

    memset(acc, 0, sizeof(*acc));
    acc->smb.num_ics = NSMBS;
    acc->smb.ics_capacity = NSMBS;
    acc->smb.ics = acc->smb_ics;
    acc->smb.monitored_cell_count = NCELLS;
    acc->voltage_startup_scan_complete = startup_done;
    acc->voltage_full_usable = full_usable;
    acc->voltage_full_updated = full_updated;
    acc->usable_voltage_count = full_usable ? AMS_EXPECTED_CELL_COUNT : (AMS_EXPECTED_CELL_COUNT - 1u);
    acc->updated_voltage_count = full_updated ? AMS_EXPECTED_CELL_COUNT : (AMS_EXPECTED_CELL_COUNT - 1u);
    acc->max_voltage_mv = max_mv;
    acc->min_voltage_mv = min_mv;
    acc->max_voltage_seg = 2u;
    acc->max_voltage_cell = 7u;
    acc->min_voltage_seg = 4u;
    acc->min_voltage_cell = 14u;
}

static void test_voltage_fault_thresholds_latch_and_reset(void)
{
    voltage_fault_state_t vf;
    accumulator_t v_acc;

    voltage_fault_init(&vf);
    unit_fill_voltage_acc(&v_acc, 3300u, 4149u, true, true, true);
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.voltage_valid);
    EXPECT_FALSE(vf.warning);
    EXPECT_FALSE(vf.confirmed);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_NONE);

    unit_fill_voltage_acc(&v_acc, 3300u, CELL_OV_WARN_MV, true, true, true);
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.warning);
    EXPECT_FALSE(vf.charge_stop);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_OV_WARNING);

    unit_fill_voltage_acc(&v_acc, 3300u, CELL_CHARGE_STOP_MV, true, true, true);
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.warning);
    EXPECT_TRUE(vf.charge_stop);
    EXPECT_FALSE(vf.confirmed);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_CHARGE_STOP);

    unit_fill_voltage_acc(&v_acc, 3300u, CELL_OV_HARD_MV, true, true, true);
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.confirmed);
    EXPECT_TRUE(vf.latched);
    EXPECT_TRUE(vf.overvoltage_fault);
    EXPECT_TRUE(vf.latched_reason == VOLTAGE_FAULT_REASON_OV_HARD);
    EXPECT_TRUE(vf.max_cell_segment == 2u);
    EXPECT_TRUE(vf.max_cell_index == 7u);

    voltage_fault_reset_latch(&vf);
    EXPECT_FALSE(vf.latched);
    EXPECT_TRUE(vf.latched_reason == VOLTAGE_FAULT_REASON_NONE);

    unit_fill_voltage_acc(&v_acc, 3300u, CELL_OV_SEVERE_MV, true, true, true);
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.latched_reason == VOLTAGE_FAULT_REASON_OV_SEVERE);

    voltage_fault_reset_latch(&vf);
    unit_fill_voltage_acc(&v_acc, CELL_UV_SOFT_MV, 4100u, true, true, true);
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.warning);
    EXPECT_FALSE(vf.confirmed);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_UV_SOFT);

    unit_fill_voltage_acc(&v_acc, CELL_UV_HARD_MV, 4100u, true, true, true);
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.confirmed);
    EXPECT_TRUE(vf.undervoltage_fault);
    EXPECT_TRUE(vf.latched_reason == VOLTAGE_FAULT_REASON_UV_HARD);

    voltage_fault_reset_latch(&vf);
    unit_fill_voltage_acc(&v_acc, CELL_UV_SEVERE_MV, 4100u, true, true, true);
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.latched_reason == VOLTAGE_FAULT_REASON_UV_SEVERE);
}

static void test_voltage_fault_read_failure_precedence_and_strings(void)
{
    voltage_fault_state_t vf;
    accumulator_t v_acc;

    voltage_fault_init(&vf);
    voltage_fault_update(&vf, NULL);
    EXPECT_TRUE(vf.confirmed);
    EXPECT_TRUE(vf.read_fault);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_NOT_READY);

    unit_fill_voltage_acc(&v_acc, 3300u, 4100u, false, false, false);
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.confirmed);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_NOT_READY);

    unit_fill_voltage_acc(&v_acc, 3300u, 4100u, true, false, false);
    v_acc.stale_voltage_count = 1u;
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.confirmed);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_STALE_SCAN);

    unit_fill_voltage_acc(&v_acc, 3300u, 4100u, true, false, false);
    v_acc.pec_fail_cell_count = 3u;
    voltage_fault_update(&vf, &v_acc);
    EXPECT_TRUE(vf.confirmed);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_PEC_FAILURE);
    EXPECT_TRUE(vf.pec_fail_cell_count == 3u);

    unit_fill_voltage_acc(&v_acc, 3300u, 4100u, true, true, false);
    v_acc.pec_fail_cell_count = 1u;
    voltage_fault_update(&vf, &v_acc);
    EXPECT_FALSE(vf.voltage_valid);
    EXPECT_TRUE(vf.read_fault);
    EXPECT_TRUE(vf.confirmed);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_PEC_FAILURE);

    EXPECT_TRUE(strcmp(voltage_fault_reason_str(VOLTAGE_FAULT_REASON_CHARGE_STOP), "charge_stop") == 0);
    EXPECT_TRUE(strcmp(voltage_fault_reason_str(VOLTAGE_FAULT_REASON_OPEN_WIRE_RESERVED), "open_wire_reserved") == 0);
}

static void unit_fill_single_smb_voltage_image(accumulator_t *acc,
                                                uint16_t nominal_mv,
                                                uint32_t now_ms)
{
    if(acc == NULL)
    {
        return;
    }

    memset(acc, 0, sizeof(*acc));
    acc->smb.num_ics = 1;
    acc->smb.ics_capacity = NSMBS;
    acc->smb.ics = acc->smb_ics;
    acc->smb.monitored_cell_count = NCELLS;
    acc->voltage_startup_scan_complete = true;
    acc->voltage_full_updated = true;
    acc->voltage_full_usable = true;
    acc->updated_voltage_mask[0] = 0x7FFFu;
    acc->usable_voltage_mask[0] = 0x7FFFu;
    acc->updated_voltage_count = NCELLS;
    acc->usable_voltage_count = NCELLS;
    acc->min_voltage_mv = nominal_mv;
    acc->max_voltage_mv = nominal_mv;
    for(uint8_t cell = 0u; cell < NCELLS; cell++)
    {
        acc->cell_voltage_mv[0][cell] = nominal_mv;
        acc->cell_voltage_valid[0][cell] = true;
        acc->cell_voltage_last_update_ms[0][cell] = now_ms;
    }
}

static void test_voltage_fault_hardware_status_crosscheck(void)
{
    accumulator_t acc;
    voltage_fault_state_t vf;

    voltage_fault_init(&vf);
    unit_fill_single_smb_voltage_image(&acc, 3500u, 1000u);
    acc.smb.diag[0].statd_valid = true;

    /* Corrupted topology metadata must fail before any cell/status array walk. */
    acc.smb.monitored_cell_count = (uint8_t)(NCELLS + 1u);
    voltage_fault_update(&vf, &acc);
    EXPECT_TRUE(vf.read_fault);
    EXPECT_TRUE(vf.confirmed);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_NOT_READY);
    unit_fill_single_smb_voltage_image(&acc, 3500u, 1000u);
    acc.smb.diag[0].statd_valid = true;

    /* The unused C16 comparator must never produce a 15-cell warning. */
    acc.smb.diag[0].cell_ov_mask = 0x8000u;
    acc.smb.diag[0].cell_uv_mask = 0x8000u;
    voltage_fault_update(&vf, &acc);
    EXPECT_TRUE(vf.voltage_valid);
    EXPECT_FALSE(vf.hardware_warning);
    EXPECT_FALSE(vf.hardware_disagreement);
    EXPECT_TRUE(vf.hardware_ov_mask[0] == 0u);
    EXPECT_TRUE(vf.hardware_uv_mask[0] == 0u);

    /* A hardware-only monitored-cell OV flag is diagnostic evidence, but the
     * fresh software C value remains the hard-fault source of truth. */
    acc.smb.diag[0].cell_ov_mask = 0x0001u;
    acc.smb.diag[0].cell_uv_mask = 0u;
    voltage_fault_update(&vf, &acc);
    EXPECT_TRUE(vf.hardware_warning);
    EXPECT_TRUE(vf.hardware_disagreement);
    EXPECT_TRUE(vf.hardware_disagreement_mask[0] == 0x0001u);
    EXPECT_FALSE(vf.confirmed);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_HW_STATUS_DISAGREEMENT);

    /* A Status-D flag inside the non-atomic comparison margin remains a
     * visible warning even though it is not classified as a disagreement. */
    voltage_fault_init(&vf);
    unit_fill_single_smb_voltage_image(&acc,
                                       (uint16_t)(ADBMS_UV_WARN_MV + 5u),
                                       1000u);
    acc.smb.diag[0].statd_valid = true;
    acc.smb.diag[0].cell_ov_mask = 0u;
    acc.smb.diag[0].cell_uv_mask = 0x0001u;
    voltage_fault_update(&vf, &acc);
    EXPECT_TRUE(vf.hardware_warning);
    EXPECT_FALSE(vf.hardware_disagreement);
    EXPECT_TRUE(vf.warning);
    EXPECT_TRUE(vf.reason == VOLTAGE_FAULT_REASON_HW_STATUS_WARNING);

    /* A threshold crossing inside the non-atomic comparison margin must not
     * create a spurious disagreement simply because Status-D was older. */
    voltage_fault_init(&vf);
    acc.cell_voltage_mv[0][0] = (uint16_t)(CELL_OV_HARD_MV + 5u);
    acc.max_voltage_mv = acc.cell_voltage_mv[0][0];
    acc.smb.diag[0].cell_ov_mask = 0u;
    acc.smb.diag[0].cell_uv_mask = 0u;
    voltage_fault_update(&vf, &acc);
    EXPECT_TRUE(vf.confirmed);
    EXPECT_FALSE(vf.hardware_disagreement);

    /* Well outside the margin, matching software and hardware masks agree. */
    voltage_fault_init(&vf);
    acc.cell_voltage_mv[0][0] =
        (uint16_t)(CELL_OV_HARD_MV + ADBMS_OVUV_COMPARE_MARGIN_MV);
    acc.max_voltage_mv = acc.cell_voltage_mv[0][0];
    acc.smb.diag[0].cell_ov_mask = 0x0001u;
    acc.smb.diag[0].cell_uv_mask = 0u;
    voltage_fault_update(&vf, &acc);
    EXPECT_TRUE(vf.software_ov_mask[0] == 0x0001u);
    EXPECT_TRUE(vf.hardware_ov_mask[0] == 0x0001u);
    EXPECT_FALSE(vf.hardware_disagreement);
}

static void test_parallel_connection_observer_advisory(void)
{
    accumulator_t acc;
    ams_parallel_connection_observer_t observer;
    uint32_t now = 1000u;

    unit_fill_single_smb_voltage_image(&acc, 3500u, now);
    ams_parallel_connection_observer_init(&observer);
    EXPECT_FALSE(observer.target_validated);

    /* A partially initialized or corrupted topology must never allow an
     * advisory comparison against implicit zero-filled segments. */
    acc.smb.num_ics = 0;
    ams_parallel_connection_observer_step(&observer,
                                          &acc,
                                          0.0f,
                                          true,
                                          false,
                                          now);
    EXPECT_FALSE(observer.input_valid);
    EXPECT_FALSE(observer.initialized);
    EXPECT_TRUE(observer.reason == AMS_PARALLEL_OBSERVER_INPUT_INVALID);
    acc.smb.num_ics = 1;

    ams_parallel_connection_observer_step(&observer,
                                          &acc,
                                          0.0f,
                                          true,
                                          false,
                                          now);
    EXPECT_TRUE(observer.initialized);
    EXPECT_TRUE(observer.reason == AMS_PARALLEL_OBSERVER_WAITING);

    /* Five repeatable 20 A steps: ordinary groups move 2 mV (0.1 mOhm)
     * while cell 4 moves 30 mV (1.5 mOhm).  This can only become an advisory
     * possible-connection-degradation result, never a hard/fuse identity. */
    for(uint8_t event = 0u; event < AMS_PARALLEL_OBSERVER_CONFIRM_EVENTS; event++)
    {
        bool loaded = ((event & 1u) == 0u);
        uint16_t ordinary_mv = loaded ? 3498u : 3500u;
        uint16_t outlier_mv = loaded ? 3470u : 3500u;
        float current_a = loaded ? 20.0f : 0.0f;

        now += 100u;
        for(uint8_t cell = 0u; cell < NCELLS; cell++)
        {
            acc.cell_voltage_mv[0][cell] = ordinary_mv;
            acc.cell_voltage_last_update_ms[0][cell] = now;
        }
        acc.cell_voltage_mv[0][3] = outlier_mv;
        ams_parallel_connection_observer_step(&observer,
                                              &acc,
                                              current_a,
                                              true,
                                              false,
                                              now);
    }

    EXPECT_TRUE(observer.accepted_step_count ==
                AMS_PARALLEL_OBSERVER_CONFIRM_EVENTS);
    EXPECT_TRUE((observer.candidate_mask[0] & (1u << 3u)) != 0u);
    EXPECT_TRUE((observer.suspect_mask[0] & (1u << 3u)) != 0u);
    EXPECT_TRUE(observer.suspect);
    EXPECT_TRUE(observer.advisory_valid);
    EXPECT_TRUE(observer.reason == AMS_PARALLEL_OBSERVER_SUSPECT);

    /* Invalid or balancing-contaminated samples break the step pair rather
     * than comparing a future value against a stale baseline. */
    ams_parallel_connection_observer_step(&observer,
                                          &acc,
                                          0.0f,
                                          false,
                                          false,
                                          now + 10u);
    EXPECT_FALSE(observer.initialized);
    EXPECT_FALSE(observer.advisory_valid);
    EXPECT_TRUE(observer.reason == AMS_PARALLEL_OBSERVER_INPUT_INVALID);

    for(uint8_t cell = 0u; cell < NCELLS; cell++)
    {
        acc.cell_voltage_last_update_ms[0][cell] = now + 20u;
    }
    ams_parallel_connection_observer_step(&observer,
                                          &acc,
                                          0.0f,
                                          true,
                                          true,
                                          now + 20u);
    EXPECT_FALSE(observer.initialized);
    EXPECT_TRUE(observer.reason == AMS_PARALLEL_OBSERVER_BALANCING_ACTIVE);
    EXPECT_TRUE(strcmp(ams_parallel_observer_reason_str(
        AMS_PARALLEL_OBSERVER_SUSPECT),
        "possible_parallel_connection_degradation") == 0);
}

static void unit_set_main_fuse_inputs_fresh(ams_air_monitor_t *air,
                                             ams_air_monitor_inputs_t *inputs,
                                             uint32_t now_ms)
{
    memset(air, 0, sizeof(*air));
    memset(inputs, 0, sizeof(*inputs));
    air->feature_enabled = true;
    air->configuration_valid = true;
    air->command_valid = true;
    air->feedback_valid = true;
    air->voltage_valid = true;
    air->boot_open_verified = true;
    air->transition_authorized = true;
    air->last_update_tick = now_ms;
    air->phase = AMS_AIR_PHASE_RUN;
    air->steady_state_valid = true;
    air->permit = true;
    air->precharge_complete = true;
    inputs->now_tick = now_ms;
    inputs->command.valid = true;
    inputs->command.phase = AMS_AIR_PHASE_RUN;
    inputs->command.update_tick = now_ms;
    inputs->pos_aux.valid = true;
    inputs->pos_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs->pos_aux.update_tick = now_ms;
    inputs->neg_aux.valid = true;
    inputs->neg_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs->neg_aux.update_tick = now_ms;
    inputs->pack_voltage.valid = true;
    inputs->pack_voltage.millivolts = 300000u;
    inputs->pack_voltage.update_tick = now_ms;
    inputs->load_voltage.valid = true;
    inputs->load_voltage.millivolts = 295000u;
    inputs->load_voltage.update_tick = now_ms;
}

static void test_main_fuse_monitor_plausibility_and_clear(void)
{
    ams_main_fuse_monitor_t monitor;
    ams_air_monitor_t air;
    ams_air_monitor_inputs_t inputs;

    ams_main_fuse_monitor_init(&monitor);
    EXPECT_TRUE(monitor.reason == AMS_MAIN_FUSE_MONITOR_UNAVAILABLE);

    unit_set_main_fuse_inputs_fresh(&air, &inputs, 1000u);
    ams_main_fuse_monitor_step(&monitor, &air, &inputs,
                               5.0f, true, 1000u);
#if AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED && \
    AMS_ENABLE_AIR_AUX_FEEDBACK && \
    AMS_CURRENT_CALIBRATION_VALIDATED && \
    AMS_MAIN_FUSE_LOAD_VOLTAGE_TARGET_VALIDATED
    EXPECT_TRUE(monitor.reason == AMS_MAIN_FUSE_MONITOR_HEALTHY);
    EXPECT_TRUE(monitor.authority_valid);

    inputs.load_voltage.millivolts = 20000u;
    inputs.load_voltage.update_tick = 1100u;
    inputs.pack_voltage.update_tick = 1100u;
    inputs.command.update_tick = 1100u;
    inputs.pos_aux.update_tick = 1100u;
    inputs.neg_aux.update_tick = 1100u;
    air.last_update_tick = 1100u;
    ams_main_fuse_monitor_step(&monitor, &air, &inputs,
                               0.0f, true, 1100u);
    EXPECT_TRUE(monitor.suspect_open);
    EXPECT_TRUE(monitor.reason == AMS_MAIN_FUSE_MONITOR_SUSPECT_OPEN);

    inputs.load_voltage.update_tick = 1700u;
    inputs.pack_voltage.update_tick = 1700u;
    inputs.command.update_tick = 1700u;
    inputs.pos_aux.update_tick = 1700u;
    inputs.neg_aux.update_tick = 1700u;
    air.last_update_tick = 1700u;
    ams_main_fuse_monitor_step(&monitor, &air, &inputs,
                               0.0f, true, 1700u);
    EXPECT_TRUE(monitor.confirmed_open);
    EXPECT_TRUE(monitor.latched);
    EXPECT_TRUE(monitor.reason == AMS_MAIN_FUSE_MONITOR_CONFIRMED_OPEN);

    EXPECT_FALSE(ams_main_fuse_monitor_request_clear(&monitor,
                                                      &inputs,
                                                      0.0f,
                                                      true,
                                                      1700u));

    inputs.command.phase = AMS_AIR_PHASE_OFF;
    inputs.command.update_tick = 1800u;
    inputs.pack_voltage.update_tick = 1800u;
    inputs.load_voltage.millivolts = 0u;
    inputs.load_voltage.update_tick = 1800u;
    EXPECT_TRUE(ams_main_fuse_monitor_request_clear(&monitor,
                                                     &inputs,
                                                     0.0f,
                                                     true,
                                                     1800u));
    EXPECT_FALSE(monitor.confirmed_open);
    EXPECT_FALSE(monitor.latched);
    EXPECT_TRUE(monitor.reason == AMS_MAIN_FUSE_MONITOR_IDLE);
#else
    /* Bench/unvalidated builds retain raw telemetry but must never create a
     * suspect or confirmed fuse claim. */
    EXPECT_FALSE(monitor.authority_valid);
    EXPECT_FALSE(monitor.suspect_open);
    EXPECT_FALSE(monitor.confirmed_open);
    EXPECT_TRUE(monitor.reason == AMS_MAIN_FUSE_MONITOR_UNAVAILABLE);

    inputs.load_voltage.millivolts = 20000u;
    inputs.load_voltage.update_tick = 1700u;
    inputs.pack_voltage.update_tick = 1700u;
    inputs.command.update_tick = 1700u;
    inputs.pos_aux.update_tick = 1700u;
    inputs.neg_aux.update_tick = 1700u;
    air.last_update_tick = 1700u;
    ams_main_fuse_monitor_step(&monitor, &air, &inputs,
                               0.0f, true, 1700u);
    EXPECT_FALSE(monitor.suspect_open);
    EXPECT_FALSE(monitor.confirmed_open);
    EXPECT_FALSE(monitor.latched);
    EXPECT_TRUE(monitor.reason == AMS_MAIN_FUSE_MONITOR_UNAVAILABLE);
#endif

    EXPECT_TRUE(strcmp(ams_main_fuse_monitor_reason_str(
        AMS_MAIN_FUSE_MONITOR_CONFIRMED_OPEN),
        "confirmed_open_hv_path") == 0);
}

static void unit_adbms_make_valid_read_packet(uint8_t *dst, uint8_t seed, uint8_t cmd_counter, bool corrupt_pec)
{
    for(uint8_t i = 0u; i < (RX_DATA - 2u); i++)
    {
        dst[i] = (uint8_t)(seed + i);
    }

    dst[RX_DATA - 2u] = (uint8_t)(cmd_counter << 2u);
    dst[RX_DATA - 1u] = 0u;

    uint16_t pec = pec10_calc(1u, RX_DATA - 2u, dst);
    dst[RX_DATA - 2u] = (uint8_t)((cmd_counter << 2u) | ((pec >> 8u) & 0x03u));
    dst[RX_DATA - 1u] = (uint8_t)pec;

    if(corrupt_pec)
    {
        dst[RX_DATA - 1u] ^= 0x5Au;
    }
}

static void unit_adbms_make_read_packet_from_data(uint8_t *dst, const uint8_t payload[TX_DATA], uint8_t cmd_counter, bool corrupt_pec)
{
    memcpy(dst, payload, TX_DATA);
    dst[RX_DATA - 2u] = (uint8_t)(cmd_counter << 2u);
    dst[RX_DATA - 1u] = 0u;

    uint16_t pec = pec10_calc(1u, RX_DATA - 2u, dst);
    dst[RX_DATA - 2u] = (uint8_t)((cmd_counter << 2u) | ((pec >> 8u) & 0x03u));
    dst[RX_DATA - 1u] = (uint8_t)pec;

    if(corrupt_pec)
    {
        dst[RX_DATA - 1u] ^= 0xA5u;
    }
}

static void unit_adbms_make_statc_ccts_payload(uint8_t payload[TX_DATA], uint16_t ccts)
{
    uint16_t ct;

    memset(payload, 0, TX_DATA);
    ccts &= 0x1FFFu;
    ct = (uint16_t)(ccts >> 2u);
    payload[2] = (uint8_t)((ct >> 6u) & 0x1Fu);
    payload[3] = (uint8_t)(((ct & 0x3Fu) << 2u) | (ccts & 0x03u));
}

static void unit_adbms_make_cfga_mute_packet(uint8_t *dst, bool muted, uint8_t cmd_counter)
{
    uint8_t payload[TX_DATA] = {0u};
    payload[5] = muted ? 0x10u : 0x00u;
    unit_adbms_make_read_packet_from_data(dst, payload, cmd_counter, false);
}

static void unit_adbms2950_make_flag_payload(uint8_t payload[TX_DATA],
                                             uint16_t i1_count,
                                             uint8_t i1_phase)
{
    memset(payload, 0, TX_DATA);
    i1_count &= 0x07FFu;
    payload[2] = (uint8_t)((i1_count >> 6u) & 0x1Fu);
    payload[3] = (uint8_t)(((i1_count & 0x3Fu) << 2u) |
                           (i1_phase & 0x03u));
}

static void unit_adbms_put_s16(uint8_t payload[TX_DATA], uint8_t slot, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    payload[slot * 2u] = (uint8_t)raw;
    payload[(slot * 2u) + 1u] = (uint8_t)(raw >> 8u);
}

static void unit_adbms_make_status_sequence(const uint8_t stata[TX_DATA],
                                             const uint8_t statb[TX_DATA],
                                             const uint8_t statc[TX_DATA],
                                             const uint8_t statd[TX_DATA],
                                             const uint8_t state[TX_DATA],
                                             uint8_t cmd_counter)
{
    const uint8_t *payloads[5] = {stata, statb, statc, statd, state};

    unit_spi_txrx_sequence_count = 5u;
    for(uint8_t group = 0u; group < 5u; group++)
    {
        unit_adbms_make_read_packet_from_data(
            &unit_spi_txrx_sequence[group][CMDSZ + PEC15SZ],
            payloads[group],
            cmd_counter,
            false);
    }
}

static void unit_adbms_make_comm_result(uint8_t *dst,
                                        uint8_t cmd_counter,
                                        bool address_ack,
                                        bool data_ack,
                                        bool corrupt_pec)
{
    uint8_t payload[TX_DATA] =
    {
        (uint8_t)((ICOMM_START_ << 4u) | (address_ack ? 0x07u : 0x0Fu)),
        0u,
        (uint8_t)((0x07u << 4u) | (data_ack ? 0x07u : 0x0Fu)),
        0u,
        (uint8_t)((ICOMM_STOP_ << 4u) | FCOMM_NACK_STOP_),
        0u
    };

    unit_adbms_make_read_packet_from_data(dst, payload, cmd_counter, corrupt_pec);
}

static void unit_adbms_make_aux_result(uint8_t *dst,
                                       uint8_t gpio_ch,
                                       uint16_t raw,
                                       uint8_t cmd_counter,
                                       bool corrupt_pec)
{
    uint8_t payload[TX_DATA] = {0u};
    uint8_t byte_lo = (uint8_t)(gpio_ch * 2u);

    payload[byte_lo] = (uint8_t)raw;
    payload[byte_lo + 1u] = (uint8_t)(raw >> 8u);
    unit_adbms_make_read_packet_from_data(dst, payload, cmd_counter, corrupt_pec);
}

static void unit_adbms_make_s_group_result(uint8_t *dst,
                                            int16_t code0,
                                            int16_t code1,
                                            int16_t code2,
                                            uint8_t cmd_counter,
                                            bool corrupt_pec)
{
    uint8_t payload[TX_DATA];
    const int16_t codes[3] = {code0, code1, code2};

    for(uint8_t slot = 0u; slot < 3u; slot++)
    {
        uint16_t raw = (uint16_t)codes[slot];
        payload[slot * 2u] = (uint8_t)raw;
        payload[(slot * 2u) + 1u] = (uint8_t)(raw >> 8u);
    }
    unit_adbms_make_read_packet_from_data(dst, payload, cmd_counter, corrupt_pec);
}

static void unit_adbms_init_driver(adbms6830_driver_t *dev,
                                   adbms6830_asic *ics,
                                   SPI_HandleTypeDef *spi,
                                   GPIO_TypeDef *gpio_a,
                                   GPIO_TypeDef *gpio_b,
                                   uint8_t num_ics)
{
    const uint8_t sid[TX_DATA] =
    {
        0xA5u, (uint8_t)(ADBMS6830B_DEVICE_ID << 1u),
        0x12u, 0x34u, 0x56u, 0x78u
    };

    memset(&unit_delay_timer_instance, 0, sizeof(unit_delay_timer_instance));
    memset(&unit_delay_timer, 0, sizeof(unit_delay_timer));
    unit_delay_timer.Instance = &unit_delay_timer_instance;
    unit_delay_timer_advances = true;
    unit_spi_reset();
    for(uint8_t ic = 0u; ic < num_ics; ic++)
    {
        unit_adbms_make_read_packet_from_data(
            &unit_spi_txrx_response[CMDSZ + PEC15SZ + ((uint16_t)ic * RX_DATA)],
            sid,
            0u,
            false);
    }
    EXPECT_TRUE(adBms6830_init(dev,
                        num_ics,
                        num_ics,
                        ics,
                        num_ics,
                        spi,
                        gpio_a,
                        gpio_b,
                        3u,
                        4u,
                        STRING_B,
                        &unit_delay_timer) == HAL_OK);
    (void)adbms6830_set_monitored_cell_count(dev, 15u);
    unit_spi_reset();
    /* Production writes are deliberately constrained to the configured
     * write-owner path (String B).  Keep the unit fixture on that owner by
     * default; individual read-direction tests can still select String A
     * explicitly when that behavior is under test. */
    dev->string = dev->write_string;
    adbms6830_spi_debug_clear(dev);
}

static void test_adbms_topology_and_delay_guards(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[2];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;

    memset(&dev, 0xA5, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    memset(&unit_delay_timer_instance, 0, sizeof(unit_delay_timer_instance));
    memset(&unit_delay_timer, 0, sizeof(unit_delay_timer));
    unit_delay_timer.Instance = &unit_delay_timer_instance;
    unit_delay_timer_advances = true;
    unit_spi_reset();

    /* Lock the documented wake/reference/conversion timing contract into
     * the real driver translation unit. */
    EXPECT_TRUE(WAKEUP_US_DELAY > ADBMS6830_CORE_WAKE_MAX_US);
    EXPECT_TRUE(WAKEUP_BW_DELAY > ADBMS6830_CORE_WAKE_MAX_US);
    EXPECT_TRUE(WAKEUP_US_DELAY < ADBMS_ISOSPI_IDLE_MIN_US);
    EXPECT_TRUE(WAKEUP_BW_DELAY < ADBMS_ISOSPI_IDLE_MIN_US);
    EXPECT_TRUE((ADBMS6830_REFERENCE_PRECONVERSION_WAIT_US +
                 WAKEUP_US_DELAY + WAKEUP_BW_DELAY) >
                ADBMS6830_REFERENCE_WAKE_MAX_US);
    EXPECT_TRUE(ADBMS6830_REDUNDANT_CONVERSION_WAIT_US > 16000u);
    EXPECT_TRUE(ADBMS6830_AUX_CONVERSION_WAIT_US >= 18000u);

    EXPECT_TRUE(adBms6830_init(NULL,
                         1u,
                         1u,
                         ics,
                         1u,
                         &spi,
                         &gpio_a,
                         &gpio_b,
                         3u,
                         4u,
                         STRING_B,
                         &unit_delay_timer) == HAL_ERROR);
    EXPECT_TRUE(adBms6830_init(&dev,
                         2u,
                         2u,
                         ics,
                         1u,
                         &spi,
                         &gpio_a,
                         &gpio_b,
                         3u,
                         4u,
                         STRING_B,
                         &unit_delay_timer) == HAL_ERROR);
    EXPECT_TRUE(dev.num_ics == 0);
    EXPECT_TRUE(unit_gpio_write_calls == 0u);

    EXPECT_TRUE(adBms6830_init(&dev,
                              2u,
                              1u,
                              ics,
                              2u,
                              &spi,
                              &gpio_a,
                              &gpio_b,
                              3u,
                              4u,
                              STRING_B,
                              &unit_delay_timer) == HAL_ERROR);
    EXPECT_TRUE(dev.num_ics == 0);
    EXPECT_TRUE(dev.physical_chain_count == 0u);

    memset(ics, 0xA5, sizeof(ics));
    unit_spi_reset();
    unit_spi_tx_status = HAL_TIMEOUT;
    EXPECT_TRUE(adBms6830_init(&dev,
                              2u,
                              2u,
                              ics,
                              2u,
                              &spi,
                              &gpio_a,
                              &gpio_b,
                              3u,
                              4u,
                              STRING_B,
                              &unit_delay_timer) == HAL_TIMEOUT);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(ics[0].tx_cfgb.dcc == 0u);
    EXPECT_TRUE(ics[1].tx_cfgb.dcc == 0u);
    unit_spi_tx_status = HAL_OK;

    /* A PEC-valid RDSID from the wrong product must stop startup after the
     * one global reset and before either configuration write. */
    {
        const uint8_t wrong_product_sid[TX_DATA] =
        {
            0xA5u, (uint8_t)(ADBMS2950B_DEVICE_ID << 1u),
            0x12u, 0x34u, 0x56u, 0x78u
        };
        unit_spi_reset();
        for(uint8_t ic = 0u; ic < 2u; ic++)
        {
            unit_adbms_make_read_packet_from_data(
                &unit_spi_txrx_response[CMDSZ + PEC15SZ + ((uint16_t)ic * RX_DATA)],
                wrong_product_sid,
                0u,
                false);
        }
        EXPECT_TRUE(adBms6830_init(&dev,
                                  2u,
                                  2u,
                                  ics,
                                  2u,
                                  &spi,
                                  &gpio_a,
                                  &gpio_b,
                                  3u,
                                  4u,
                                  STRING_B,
                                  &unit_delay_timer) == HAL_ERROR);
        EXPECT_TRUE(unit_spi_tx_calls == 1u);
        EXPECT_TRUE(unit_spi_txrx_calls == 1u);
        EXPECT_TRUE(dev.health.sid_valid_ic_mask == 0u);
        EXPECT_TRUE(dev.health.sid_identity_mismatch_ic_mask == 0x0003u);
        EXPECT_TRUE(dev.health.sticky_sid_identity_mismatch_ic_mask == 0x0003u);
    }

    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 2u);
	/* APM SNAP/UNSNAP commands are broadcast through the same ring. Keep the
	 * five-SMB counter prediction aligned, including the 63 -> 1 rollover. */
	dev.spi_debug.cmd_counter_expected_mask = 0x0003u;
	dev.spi_debug.expected_cmd_counter[0] = 60u;
	dev.spi_debug.expected_cmd_counter[1] = 61u;
	adbms6830_note_external_counter_increments(&dev, 3u);
	EXPECT_TRUE(dev.spi_debug.expected_cmd_counter[0] == 63u);
	EXPECT_TRUE(dev.spi_debug.expected_cmd_counter[1] == 1u);
	adbms6830_resync_command_counter_tracking(&dev);
	EXPECT_TRUE(dev.spi_debug.cmd_counter_expected_mask == 0u);
	EXPECT_TRUE(dev.spi_debug.cmd_counter_mismatch_mask == 0u);
	EXPECT_TRUE(dev.health.last_cmd_counter_mismatch_mask == 0u);
    dev.physical_chain_count = 3u;
    unit_spi_reset();
    EXPECT_TRUE(adbms6830_wakeup_checked(&dev) == HAL_OK);
    EXPECT_TRUE(unit_gpio_write_calls == 6u);
    dev.physical_chain_count = 2u;
    unit_spi_reset();
    dev.ics_capacity = 1u;
    EXPECT_TRUE(adbms6830_wrcfgb_checked(&dev) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);

    dev.ics_capacity = 2u;
    unit_delay_timer_advances = false;
    uint32_t timeout_count = dev.delay_timeout_count;
    EXPECT_TRUE(adbms6830_us_delay(&dev, 10u) == HAL_TIMEOUT);
    EXPECT_TRUE(dev.delay_last_status == HAL_TIMEOUT);
    EXPECT_TRUE(dev.delay_timeout_count == timeout_count + 1u);

    unit_spi_reset();
    EXPECT_TRUE(adbms6830_wakeup_checked(&dev) == HAL_TIMEOUT);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);
    EXPECT_TRUE(adbms6830_read_cell_voltages(&dev) == HAL_TIMEOUT);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);

    unit_delay_timer_advances = true;
    EXPECT_TRUE(adbms6830_us_delay(&dev, 10u) == HAL_OK);
    EXPECT_TRUE(dev.delay_last_status == HAL_OK);

    for(uint8_t ic = 0u; ic < 2u; ic++)
    {
        dev.last_cell_updated_mask[ic] = UINT16_MAX;
        dev.last_cell_pec_mask[ic] = UINT16_MAX;
    }
    unit_spi_reset();
    unit_spi_txrx_status = HAL_TIMEOUT;
    EXPECT_TRUE(adbms6830_read_cell_voltages(&dev) == HAL_TIMEOUT);
    /* A critical raw transport failure stops the epoch immediately and the
     * coherent reader retries exactly once. */
    EXPECT_TRUE(unit_spi_txrx_calls == 2u);
    EXPECT_TRUE(dev.last_cell_updated_mask[0] == 0u);
    EXPECT_TRUE(dev.last_cell_updated_mask[1] == 0u);
    EXPECT_TRUE(dev.last_cell_pec_mask[0] == 0u);
    EXPECT_TRUE(dev.last_cell_pec_mask[1] == 0u);
    unit_spi_txrx_status = HAL_OK;

    dev.htim = NULL;
    EXPECT_TRUE(adbms6830_us_delay(&dev, 10u) == HAL_ERROR);
}


static void test_adbms_v05_session_filter_and_mute_contract(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint32_t wakes_before;

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    EXPECT_TRUE(dev.ics[0].tx_cfga.fc == (AMS_ADBMS_IIR_FC & 0x07u));
    EXPECT_FALSE(dev.filtered_voltage_ready);

    adbms6830_bind_runtime_hooks(&dev, NULL, unit_adbms_time_us, NULL);
    unit_adbms_fake_time_us = 1000u;
    adbms6830_session_open(&dev);
    dev.session.last_activity_us = unit_adbms_fake_time_us;
    wakes_before = dev.session.full_wake_count;

    unit_adbms_fake_time_us = 1000u + AMS_ADBMS_SESSION_GUARD_US - 1u;
    EXPECT_TRUE(adbms6830_session_require_awake(&dev) == HAL_OK);
    EXPECT_TRUE(dev.session.full_wake_count == wakes_before);

    dev.session.coherent_snapshot_active = true;
    unit_adbms_fake_time_us = 1000u + AMS_ADBMS_SESSION_GUARD_US + 10u;
    EXPECT_TRUE(adbms6830_session_require_awake(&dev) == HAL_BUSY);
    EXPECT_TRUE(dev.session.guard_expiry_count == 1u);
    EXPECT_TRUE(dev.session.full_wake_count == wakes_before);

    dev.session.coherent_snapshot_active = false;
    dev.session.last_activity_us = 1000u;
    EXPECT_TRUE(adbms6830_session_require_awake(&dev) == HAL_OK);
    EXPECT_TRUE(dev.session.full_wake_count == wakes_before + 1u);
    EXPECT_TRUE(dev.session.guard_rewake_count == 1u);
    adbms6830_session_close(&dev);

    unit_spi_reset();
    adbms6830_resync_command_counter_tracking(&dev);
    unit_adbms_make_cfga_mute_packet(
        &unit_spi_txrx_response[CMDSZ + PEC15SZ], true, 1u);
    EXPECT_TRUE(adbms6830_mute_checked(&dev) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(unit_spi_txrx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_tx[0] == 0x00u);
    EXPECT_TRUE(unit_spi_last_tx[1] == 0x28u);
    EXPECT_TRUE(dev.ics[0].rx_cfga.mute_st == 1u);
    EXPECT_TRUE(dev.health.mute_count == 1u);
    EXPECT_TRUE(dev.health.mute_verify_fail_count == 0u);

    unit_spi_reset();
    unit_adbms_make_cfga_mute_packet(
        &unit_spi_txrx_response[CMDSZ + PEC15SZ], false, 2u);
    EXPECT_TRUE(adbms6830_unmute_checked(&dev) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(unit_spi_txrx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_tx[0] == 0x00u);
    EXPECT_TRUE(unit_spi_last_tx[1] == 0x29u);
    EXPECT_TRUE(dev.ics[0].rx_cfga.mute_st == 0u);
    EXPECT_TRUE(dev.health.unmute_count == 1u);
    EXPECT_TRUE(dev.health.unmute_verify_fail_count == 0u);
}


static void test_adbms_v05_products_aux2_post_and_diagnostic_freshness(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t packet[RX_DATA];
    uint8_t payload[TX_DATA];
    int16_t aux2_raw[ADBMS6830_MAX_TRACKED_ICS] = {0};
    uint16_t aux2_valid = 0u;
    uint16_t failed = 0u;
    uint16_t unexpected = 0u;
    const uint8_t sensor = 5u;
    const uint8_t mux_idx = sensor / SENSORS_PER_MUX;
    const uint8_t sw_pos = sensor % SENSORS_PER_MUX;
    const uint8_t gpio_ch = GPIO_AUX_IDX[mux_idx];
    const uint32_t sensor_bit = (uint32_t)(1UL << sensor);

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    /* Bit-exact product parsing: AVG8 and IIR must remain distinct from raw C
     * storage, with exactly the source 16-bit code preserved. */
    memset(payload, 0, sizeof(payload));
    payload[0] = 0x34u; payload[1] = 0x12u;
    payload[2] = 0x78u; payload[3] = 0x56u;
    payload[4] = 0xBCu; payload[5] = 0x1Au;
    unit_adbms_make_read_packet_from_data(packet, payload, 0u, false);
    dev.health.last_cmd_counter_mismatch_mask = 0u;
    adbms6830_parse_cell_product(&dev, packet, A, false);
    EXPECT_TRUE(dev.ics[0].acell.ac_codes[0] == (int16_t)0x1234);
    EXPECT_TRUE(dev.ics[0].acell.ac_codes[1] == (int16_t)0x5678);
    EXPECT_TRUE(dev.ics[0].acell.ac_codes[2] == (int16_t)0x1ABC);
    EXPECT_TRUE((dev.last_acell_updated_mask[0] & 0x0007u) == 0x0007u);
    EXPECT_TRUE((dev.last_acell_pec_mask[0] & 0x0007u) == 0u);
    EXPECT_TRUE(dev.ics[0].cell.c_codes[0] != dev.ics[0].acell.ac_codes[0]);

    memset(payload, 0, sizeof(payload));
    payload[0] = 0x11u; payload[1] = 0x21u;
    payload[2] = 0x22u; payload[3] = 0x32u;
    payload[4] = 0x33u; payload[5] = 0x43u;
    unit_adbms_make_read_packet_from_data(packet, payload, 0u, false);
    adbms6830_parse_cell_product(&dev, packet, B, true);
    EXPECT_TRUE(dev.ics[0].fcell.fc_codes[3] == (int16_t)0x2111);
    EXPECT_TRUE(dev.ics[0].fcell.fc_codes[4] == (int16_t)0x3222);
    EXPECT_TRUE(dev.ics[0].fcell.fc_codes[5] == (int16_t)0x4333);
    EXPECT_TRUE((dev.last_fcell_updated_mask[0] & 0x0038u) == 0x0038u);
    EXPECT_TRUE((dev.last_fcell_pec_mask[0] & 0x0038u) == 0u);

    /* The diagnostic AUX path must never clear or overwrite the primary
     * temperature freshness image merely because publish_sample=false. */
    dev.mux_selection_valid_mask[mux_idx] = 0x0001u;
    dev.mux_selected_channel[0][mux_idx] = sw_pos;
    dev.last_temp_updated_mask[0] = sensor_bit;
    dev.ics[0].temp.raw[sensor] = (int16_t)0x2222;
    unit_spi_reset();
    unit_adbms_make_aux_result(&unit_spi_txrx_response[CMDSZ + PEC15SZ],
                               gpio_ch,
                               0x2345u,
                               1u,
                               false);
    EXPECT_TRUE(adbms6830_capture_aux_gpio_for_sensor(&dev,
                                                       sensor,
                                                       true,
                                                       false) == HAL_OK);
    EXPECT_TRUE((dev.last_temp_updated_mask[0] & sensor_bit) != 0u);
    EXPECT_TRUE(dev.ics[0].temp.raw[sensor] == (int16_t)0x2222);

    /* AUX2 is an independent GPIO result product.  A selected channel must be
     * captured into raux without replacing primary AUX temperature ownership. */
    dev.mux_selection_valid_mask[mux_idx] = 0x0001u;
    dev.mux_selected_channel[0][mux_idx] = sw_pos;
    adbms6830_resync_command_counter_tracking(&dev);
    unit_spi_reset();
    unit_adbms_make_aux_result(&unit_spi_txrx_response[CMDSZ + PEC15SZ],
                               gpio_ch,
                               0x2456u,
                               1u,
                               false);
    EXPECT_TRUE(adbms6830_capture_aux2_gpio_for_sensor(&dev,
                                                        sensor,
                                                        aux2_raw,
                                                        &aux2_valid) == HAL_OK);
    EXPECT_TRUE(aux2_valid == 0x0001u);
    EXPECT_TRUE(aux2_raw[0] == (int16_t)0x2456);
    EXPECT_TRUE(dev.ics[0].raux.ra_codes[gpio_ch] == (int16_t)0x2456);
    EXPECT_TRUE(dev.ics[0].temp.raw[sensor] == (int16_t)0x2222);
    EXPECT_TRUE(adbms6830_code_delta_mv(1000, 1100) == 15);
    EXPECT_TRUE(adbms6830_code_delta_mv(1100, 1000) == 15);

    /* FLAG_D POST matching is intentionally a reporting-path test. Lock the
     * expected Status-C interpretation independently of the transport script. */
    memset(&dev.diag[0], 0, sizeof(dev.diag[0]));
    dev.diag[0].statc_valid = true;
    dev.diag[0].oscchk = 1u;
    EXPECT_TRUE(adbms6830_post_stage_matches(&dev,
                                              ADBMS6830_POST_OSC_FAST,
                                              &failed,
                                              &unexpected));
    EXPECT_TRUE(failed == 0u && unexpected == 0u);

    memset(&dev.diag[0], 0, sizeof(dev.diag[0]));
    dev.diag[0].statc_valid = true;
    dev.diag[0].ced = 1u;
    dev.diag[0].sed = 1u;
    EXPECT_TRUE(adbms6830_post_stage_matches(&dev,
                                              ADBMS6830_POST_NVM_ED,
                                              &failed,
                                              &unexpected));

    memset(&dev.diag[0], 0, sizeof(dev.diag[0]));
    dev.diag[0].statc_valid = true;
    dev.diag[0].spiflt = 1u;
    EXPECT_TRUE(adbms6830_post_stage_matches(&dev,
                                              ADBMS6830_POST_SPIFLT,
                                              &failed,
                                              &unexpected));
    EXPECT_TRUE(failed == 0u && unexpected == 0u);

    memset(&dev.diag[0], 0, sizeof(dev.diag[0]));
    dev.diag[0].statc_valid = true;
    dev.diag[0].oscchk = 1u;
    dev.diag[0].thsd = 1u;
    EXPECT_FALSE(adbms6830_post_stage_matches(&dev,
                                               ADBMS6830_POST_OSC_SLOW,
                                               &failed,
                                               &unexpected));
    EXPECT_TRUE(unexpected == 0x0001u);

    /* The coherent C conversion counter is an independent freshness proof.
     * Current Rev5 restarts ADCV every scan, so zero after the conversion
     * wait is invalid while any nonzero CCTS is accepted. */
    {
        uint8_t ccts_packet[RX_DATA];
        uint8_t ccts_payload[TX_DATA];
        memset(ccts_packet, 0, sizeof(ccts_packet));
        unit_adbms_make_statc_ccts_payload(ccts_payload, 0u);
        unit_adbms_make_read_packet_from_data(ccts_packet, ccts_payload, 0u, false);
        dev.health.last_cmd_counter_mismatch_mask = 0u;
        EXPECT_TRUE(adbms6830_capture_coherent_cadc_counter(&dev, ccts_packet) == HAL_ERROR);
        EXPECT_TRUE(dev.health.cadc_ccts_fault_ic_mask == 0x0001u);

        unit_adbms_make_statc_ccts_payload(ccts_payload, 4u);
        unit_adbms_make_read_packet_from_data(ccts_packet, ccts_payload, 0u, false);
        EXPECT_TRUE(adbms6830_capture_coherent_cadc_counter(&dev, ccts_packet) == HAL_OK);
        EXPECT_TRUE(dev.health.cadc_ccts_valid_ic_mask == 0x0001u);
        EXPECT_TRUE(dev.health.cadc_ccts_fault_ic_mask == 0u);
        EXPECT_TRUE(dev.health.sticky_cadc_ccts_fault_ic_mask == 0x0001u);
    }

    /* AVG8 is an optional estimator/diagnostic product. A corrupt AVG8 group
     * must withdraw AVG8 only; it must not revoke a clean coherent RAW C
     * safety epoch. */
    {
        uint8_t coherent_statc[TX_DATA] = {0u};
        uint8_t clean_statd[TX_DATA] = {0u};
        unit_spi_reset();
        adbms6830_resync_command_counter_tracking(&dev);
        unit_spi_txrx_sequence_count = 9u;
        for(uint8_t group = 0u; group < 6u; group++)
        {
            unit_adbms_make_valid_read_packet(
                &unit_spi_txrx_sequence[group][CMDSZ + PEC15SZ],
                (uint8_t)(0x10u + group * 0x10u),
                5u,
                false);
        }
        unit_adbms_make_statc_ccts_payload(coherent_statc, 4u);
        unit_adbms_make_read_packet_from_data(
            &unit_spi_txrx_sequence[6u][CMDSZ + PEC15SZ], coherent_statc, 5u, false);
        unit_adbms_make_read_packet_from_data(
            &unit_spi_txrx_sequence[7u][CMDSZ + PEC15SZ], clean_statd, 5u, false);
        unit_adbms_make_valid_read_packet(
            &unit_spi_txrx_sequence[8u][CMDSZ + PEC15SZ], 0x70u, 5u, true);

        EXPECT_TRUE(adbms6830_read_cell_voltage_products(&dev, true, false) == HAL_OK);
        EXPECT_TRUE((dev.last_cell_updated_mask[0] & 0x7FFFu) == 0x7FFFu);
        EXPECT_TRUE(dev.last_acell_updated_mask[0] == 0u);
        EXPECT_TRUE(dev.health.avg8_read_fail_count == 1u);
        EXPECT_TRUE(dev.health.coherent_statc_read_count >= 1u);
        EXPECT_TRUE(dev.health.coherent_statd_read_count >= 1u);
        EXPECT_TRUE(unit_spi_txrx_calls == 9u);
    }

    /* Service fault injection is one-shot and cannot accept unbounded gaps. */
#if AMS_ENABLE_SERVICE_CLI
    EXPECT_TRUE(adbms6830_session_inject_gap_once(&dev, 6000u, false) == HAL_OK);
    EXPECT_TRUE(dev.session.inject_gap_us_once == 6000u);
    EXPECT_FALSE(dev.session.inject_bypass_guard_once);
    EXPECT_TRUE(adbms6830_session_inject_gap_once(&dev, 100001u, false) == HAL_ERROR);
#endif
}

static void test_adbms_spi_debug_write_and_full_duplex_paths(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[2];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t tx[4] = {0xAAu, 0x55u, 0x12u, 0x34u};
    uint8_t rx[8];

    unit_spi_reset();
    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 2u);

    EXPECT_TRUE(adbms6830_spi_write(&dev, tx, sizeof(tx), 1u) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_tx_len == sizeof(tx));
    EXPECT_TRUE(unit_gpio_states[dev.cs_pin[dev.write_string]] == GPIO_PIN_SET);
    EXPECT_TRUE(dev.spi_debug.tx_count == 1u);
    EXPECT_TRUE(dev.spi_debug.rx_count == 0u);
    EXPECT_TRUE(dev.spi_debug.last_tx_len == sizeof(tx));
    EXPECT_TRUE(dev.spi_debug.last_rx_len == 0u);
    EXPECT_TRUE(dev.spi_debug.last_status == HAL_OK);
    EXPECT_TRUE(memcmp(dev.spi_debug.last_tx_preview, tx, sizeof(tx)) == 0);

    unit_spi_tx_status = HAL_TIMEOUT;
    EXPECT_TRUE(adbms6830_spi_write(&dev, tx, sizeof(tx), 1u) == HAL_TIMEOUT);
    EXPECT_TRUE(dev.spi_debug.error_count == 1u);
    EXPECT_TRUE(dev.spi_debug.last_status == HAL_TIMEOUT);

    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    memset(rx, 0, sizeof(rx));
    for(uint8_t i = 0u; i < sizeof(rx); i++)
    {
        unit_spi_txrx_response[sizeof(tx) + i] = (uint8_t)(0xC0u + i);
    }

    EXPECT_TRUE(adbms6830_spi_write_read(&dev, tx, sizeof(tx), rx, sizeof(rx), 1u) == HAL_OK);
    EXPECT_TRUE(unit_spi_txrx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_txrx_len == (sizeof(tx) + sizeof(rx)));
    EXPECT_TRUE(memcmp(unit_spi_last_txrx_tx, tx, sizeof(tx)) == 0);
    for(uint8_t i = 0u; i < sizeof(rx); i++)
    {
        EXPECT_TRUE(unit_spi_last_txrx_tx[sizeof(tx) + i] == 0xFFu);
        EXPECT_TRUE(rx[i] == (uint8_t)(0xC0u + i));
    }
    EXPECT_TRUE(dev.spi_debug.tx_count == 1u);
    EXPECT_TRUE(dev.spi_debug.rx_count == 1u);
    EXPECT_TRUE(dev.spi_debug.last_total_len == (sizeof(tx) + sizeof(rx)));
    EXPECT_TRUE(memcmp(dev.spi_debug.last_rx_preview, rx, sizeof(rx)) == 0);

    unit_spi_txrx_status = HAL_ERROR;
    memset(rx, 0xA5, sizeof(rx));
    EXPECT_TRUE(adbms6830_spi_write_read(&dev, tx, sizeof(tx), rx, sizeof(rx), 1u) == HAL_ERROR);
    for(uint8_t i = 0u; i < sizeof(rx); i++)
    {
        EXPECT_TRUE(rx[i] == 0u);
    }
    EXPECT_TRUE(dev.spi_debug.error_count == 1u);
    EXPECT_TRUE(dev.spi_debug.last_status == HAL_ERROR);

    unit_spi_reset();
    EXPECT_TRUE(adbms6830_spi_write_read(&dev,
                                         tx,
                                         1u,
                                         rx,
                                         UINT16_MAX,
                                         1u) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);

    dev.string = (adbms_string)-1;
    adbms6830_set_cs(&dev, 0u);
    EXPECT_TRUE(unit_gpio_write_calls == 0u);
    EXPECT_TRUE(adbms6830_spi_write(&dev, tx, sizeof(tx), 1u) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(adbms6830_spi_probe_rdcfga_on_string(&dev,
                                                     (adbms_string)-1) == HAL_ERROR);
    dev.string = STRING_B;
}

static void test_adbms_spi_debug_rd48_pec_masks_and_clear(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[2];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t rx[RX_DATA * 2u];

    unit_spi_reset();
    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 2u);

    unit_adbms_make_valid_read_packet(&unit_spi_txrx_response[CMDSZ + PEC15SZ], 0x10u, 3u, false);
    unit_adbms_make_valid_read_packet(&unit_spi_txrx_response[CMDSZ + PEC15SZ + RX_DATA], 0x20u, 4u, false);
    adbms6830_rd48(&dev, RDCFGA, rx);

    EXPECT_TRUE(unit_spi_txrx_calls == 1u);
    EXPECT_TRUE(dev.spi_debug.last_op == ADBMS6830_SPI_OP_RD48);
    EXPECT_TRUE(dev.spi_debug.last_cmd[0] == RDCFGA[0]);
    EXPECT_TRUE(dev.spi_debug.last_cmd[1] == RDCFGA[1]);
    EXPECT_TRUE(dev.spi_debug.last_read_pec_pass_mask == 0x0003u);
    EXPECT_TRUE(dev.spi_debug.last_read_pec_fail_mask == 0x0000u);
    EXPECT_TRUE(dev.spi_debug.last_cmd_counter[0] == 3u);
    EXPECT_TRUE(dev.spi_debug.last_cmd_counter[1] == 4u);
    EXPECT_TRUE(dev.spi_debug.error_count == 0u);

    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_adbms_make_valid_read_packet(&unit_spi_txrx_response[CMDSZ + PEC15SZ], 0x30u, 1u, false);
    unit_adbms_make_valid_read_packet(&unit_spi_txrx_response[CMDSZ + PEC15SZ + RX_DATA], 0x40u, 2u, true);
    adbms6830_rd48(&dev, RDCFGA, rx);

    EXPECT_TRUE(dev.spi_debug.last_read_pec_pass_mask == 0x0001u);
    EXPECT_TRUE(dev.spi_debug.last_read_pec_fail_mask == 0x0002u);
    EXPECT_TRUE(dev.spi_debug.error_count == 1u);

    adbms6830_spi_debug_enable(&dev, false);
    adbms6830_spi_debug_clear(&dev);
    EXPECT_FALSE(dev.spi_debug.enabled);
    EXPECT_TRUE(dev.spi_debug.last_status == HAL_OK);
    adbms6830_spi_debug_enable(&dev, true);
    EXPECT_TRUE(dev.spi_debug.enabled);
    EXPECT_TRUE(strcmp(adbms6830_spi_op_str(ADBMS6830_SPI_OP_PROBE), "probe") == 0);
}

static void test_adbms_spi_scope_activity(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    const uint8_t expected_pattern[8] =
    {
        0xAAu, 0x55u, 0xFFu, 0x00u, 0x69u, 0x96u, 0x12u, 0x34u
    };

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    dev.string = STRING_A;
    EXPECT_TRUE(adbms6830_scope_activity(&dev, STRING_B, ADBMS6830_SCOPE_PATTERN, 3u) == HAL_OK);
    EXPECT_TRUE(dev.string == STRING_A);
    EXPECT_TRUE(dev.spi_debug.enabled);
    EXPECT_TRUE(dev.spi_debug.last_op == ADBMS6830_SPI_OP_SCOPE);
    EXPECT_TRUE(dev.spi_debug.last_string == STRING_B);
    EXPECT_TRUE(dev.spi_debug.tx_count == 3u);
    EXPECT_TRUE(unit_spi_tx_calls == 3u);
    EXPECT_TRUE(unit_spi_last_tx_len == sizeof(expected_pattern));
    EXPECT_TRUE(memcmp(unit_spi_last_tx, expected_pattern, sizeof(expected_pattern)) == 0);
    EXPECT_TRUE(memcmp(dev.spi_debug.last_tx_preview, expected_pattern, sizeof(expected_pattern)) == 0);

    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    EXPECT_TRUE(adbms6830_scope_activity(&dev, STRING_B, ADBMS6830_SCOPE_CMD, 2u) == HAL_OK);
    EXPECT_TRUE(dev.string == STRING_A);
    EXPECT_TRUE(unit_spi_tx_calls == 2u);
    EXPECT_TRUE(dev.spi_debug.tx_count == 2u);
    EXPECT_TRUE(dev.spi_debug.last_op == ADBMS6830_SPI_OP_SCOPE);
    EXPECT_TRUE(dev.spi_debug.last_cmd[0] == RDCFGA[0]);
    EXPECT_TRUE(dev.spi_debug.last_cmd[1] == RDCFGA[1]);

    EXPECT_TRUE(adbms6830_scope_activity(&dev, STRING_B, ADBMS6830_SCOPE_READ, 0u) == HAL_ERROR);
}

static void test_adbms_spi_sid_status_and_counter_mismatch(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[2];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t rx[RX_DATA * 2u];
    const uint8_t sid0[TX_DATA] = {0x10u, 0x06u, 0x12u, 0x13u, 0x14u, 0x15u};
    const uint8_t sid1[TX_DATA] = {0x20u, 0x06u, 0x22u, 0x23u, 0x24u, 0x25u};
    const uint8_t statc[TX_DATA] = {0x05u, 0x80u, 0x12u, 0x34u, 0xC3u, 0x5Du};
    const uint8_t statd[TX_DATA] = {0x03u, 0x0Cu, 0x30u, 0xC0u, 0xFFu, 0x3Cu};
    const uint8_t state[TX_DATA] = {0xFFu, 0xFFu, 0xFFu, 0xFEu, 0xA5u, 0xB2u};

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 2u);

    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_response[CMDSZ + PEC15SZ], sid0, 7u, false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_response[CMDSZ + PEC15SZ + RX_DATA], sid1, 7u, false);
    EXPECT_TRUE(adbms6830_read_sid(&dev) == HAL_OK);
    EXPECT_TRUE(dev.spi_debug.last_op == ADBMS6830_SPI_OP_READ_SID);
    EXPECT_TRUE(dev.diag[0].sid_valid);
    EXPECT_TRUE(dev.diag[1].sid_valid);
    EXPECT_TRUE(dev.diag[0].device_id == ADBMS6830B_DEVICE_ID);
    EXPECT_TRUE(dev.diag[1].device_id == ADBMS6830B_DEVICE_ID);
    EXPECT_TRUE(dev.health.sid_valid_ic_mask == 0x0003u);
    EXPECT_TRUE(dev.health.sid_identity_mismatch_ic_mask == 0u);
    EXPECT_TRUE(memcmp(dev.diag[0].sid, sid0, TX_DATA) == 0);
    EXPECT_TRUE(memcmp(dev.diag[1].sid, sid1, TX_DATA) == 0);
    EXPECT_TRUE(dev.spi_debug.cmd_counter_seen_mask == 0x0003u);

    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], statc, 7u, false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ + RX_DATA], statc, 7u, false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], statd, 7u, false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ + RX_DATA], statd, 7u, false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], state, 7u, false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ + RX_DATA], state, 7u, false);

    EXPECT_TRUE(adbms6830_read_status(&dev, false) == HAL_OK);
    EXPECT_TRUE(unit_spi_txrx_calls == 3u);
    EXPECT_TRUE(dev.spi_debug.last_op == ADBMS6830_SPI_OP_READ_STATUS);
    EXPECT_TRUE(dev.diag[0].statc_valid);
    EXPECT_TRUE(dev.diag[0].cs_flt_mask == 0x8005u);
    EXPECT_TRUE(dev.diag[0].va_ov == 1u);
    EXPECT_TRUE(dev.diag[0].va_uv == 1u);
    EXPECT_TRUE(dev.diag[0].spiflt == 1u);
    EXPECT_TRUE(dev.diag[0].sleep == 1u);
    EXPECT_TRUE(dev.diag[0].thsd == 1u);
    EXPECT_TRUE(dev.diag[0].oscchk == 1u);
    EXPECT_TRUE(dev.diag[0].statd_valid);
    EXPECT_TRUE(dev.diag[0].cell_ov_mask == 0x8421u);
    EXPECT_TRUE(dev.diag[0].cell_uv_mask == 0x8421u);
    EXPECT_TRUE(dev.diag[0].osc_counter == 0x3Cu);
    EXPECT_TRUE(dev.diag[0].state_valid);
    EXPECT_TRUE(dev.diag[0].gpi_mask == 0x2A5u);
    EXPECT_TRUE(dev.diag[0].revision == 0x0Bu);

    unit_spi_reset();
    unit_adbms_make_valid_read_packet(&unit_spi_txrx_response[CMDSZ + PEC15SZ], 0x70u, 9u, false);
    unit_adbms_make_valid_read_packet(&unit_spi_txrx_response[CMDSZ + PEC15SZ + RX_DATA], 0x80u, 7u, false);
    adbms6830_rd48(&dev, RDCFGA, rx);
    EXPECT_TRUE(dev.spi_debug.cmd_counter_mismatch_mask == 0x0001u);
    EXPECT_TRUE(dev.spi_debug.cmd_counter_error_count == 1u);
    EXPECT_TRUE(dev.spi_debug.error_count == 1u);
}

static void test_adbms_startup_reference_and_full_status_policy(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t stata[TX_DATA] = {0u};
    uint8_t statb[TX_DATA] = {0u};
    uint8_t statc[TX_DATA] = {0u};
    uint8_t statd[TX_DATA] = {0u};
    uint8_t state[TX_DATA] = {0u};

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    /* 3.000 V VREF2/VRES, 3.300 V VD, approximately 5.000 V VA and
     * 25.0 deg C die temperature. */
    unit_adbms_put_s16(stata, 0u, 10000);
    unit_adbms_put_s16(stata, 1u, 4900);
    unit_adbms_put_s16(statb, 0u, 12000);
    unit_adbms_put_s16(statb, 1u, 23333);
    unit_adbms_put_s16(statb, 2u, 10000);
    statd[5] = 60u;

    unit_adbms_make_status_sequence(stata, statb, statc, statd, state, 7u);
    EXPECT_TRUE(adbms6830_refresh_diagnostics(&dev) == HAL_OK);
    EXPECT_TRUE(dev.diag[0].stata_valid);
    EXPECT_TRUE(dev.diag[0].statb_valid);
    EXPECT_TRUE(dev.diag[0].reference_values_valid);
    EXPECT_TRUE(dev.diag[0].vref2_mv == 3000);
    EXPECT_TRUE(dev.diag[0].vd_mv == 3300);
    EXPECT_TRUE(dev.diag[0].va_mv == 4999);
    EXPECT_TRUE(dev.diag[0].vres_mv == 3000);
    EXPECT_TRUE(dev.diag[0].die_temp_deci_c == 250);
    EXPECT_TRUE(dev.health.status_invalid_ic_mask == 0u);
    EXPECT_TRUE(dev.health.status_fault_ic_mask == 0u);
    EXPECT_TRUE(dev.health.reference_invalid_ic_mask == 0u);
    EXPECT_TRUE(dev.health.reference_fault_ic_mask == 0u);
    EXPECT_FALSE(adbms6830_safety_diagnostics_ok(&dev));

    /* The startup gate is independent from a merely readable status image. */
    dev.health.startup_baseline_passed = true;
    EXPECT_TRUE(adbms6830_safety_diagnostics_ok(&dev));

    /* COMP only reports that the redundant C/S comparison is active. It is
     * expected during RD_ON operation; CSxFLT carries the actual mismatch. */
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    statc[5] = 0x20u;
    unit_adbms_make_status_sequence(stata, statb, statc, statd, state, 8u);
    EXPECT_TRUE(adbms6830_refresh_diagnostics(&dev) == HAL_OK);
    EXPECT_TRUE(dev.diag[0].comp == 1u);
    EXPECT_TRUE(dev.health.digital_fault_ic_mask == 0u);
    EXPECT_TRUE(dev.health.oscillator_counter_fault_ic_mask == 0u);
    EXPECT_TRUE(dev.health.status_fault_ic_mask == 0u);
    statc[5] = 0u;

    /* Reset/clear sentinel values must not be interpreted as plausible
     * measurements, even when their PEC and command counter are valid. */
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_adbms_put_s16(stata, 0u, INT16_MAX);
    unit_adbms_make_status_sequence(stata, statb, statc, statd, state, 8u);
    EXPECT_TRUE(adbms6830_refresh_diagnostics(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.health.reference_invalid_ic_mask == 0x0001u);
    EXPECT_FALSE(adbms6830_safety_diagnostics_ok(&dev));

    /* Restore a valid reference image, then assert every Status C fault
     * category.  Bits 6 and 7 are both gated despite the VDE/VDEL naming
     * conflict between the datasheet and ADI reference parser. */
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_adbms_put_s16(stata, 0u, 10000);
    statc[0] = 0x01u;
    statc[4] = 0xFFu;
    statc[5] = 0xFFu;
    unit_adbms_make_status_sequence(stata, statb, statc, statd, state, 9u);
    EXPECT_TRUE(adbms6830_refresh_diagnostics(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.diag[0].vde == 1u);
    EXPECT_TRUE(dev.diag[0].vdel == 1u);
    EXPECT_TRUE(dev.health.cs_fault_ic_mask == 0x0001u);
    EXPECT_TRUE(dev.health.supply_flag_fault_ic_mask == 0x0001u);
    EXPECT_TRUE(dev.health.memory_fault_ic_mask == 0x0001u);
    EXPECT_TRUE(dev.health.digital_fault_ic_mask == 0x0001u);
    EXPECT_TRUE(dev.health.status_fault_ic_mask == 0x0001u);

    /* The raw oscillator count is an independent diagnostic. An out-of-range
     * count cannot pass merely because the OSCCHK flag was clear. */
    memset(statc, 0, sizeof(statc));
    memset(statd, 0, sizeof(statd));
    statd[5] = ADBMS6830_OSC_COUNTER_MIN - 1u;
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_adbms_make_status_sequence(stata, statb, statc, statd, state, 10u);
    EXPECT_TRUE(adbms6830_refresh_diagnostics(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.health.oscillator_counter_fault_ic_mask == 0x0001u);
    EXPECT_TRUE(dev.health.status_fault_ic_mask == 0x0001u);

    /* C16 is unpopulated in the 15-cell SMB and must be masked; C15 remains
     * safety relevant. */
    statd[5] = 60u;
    statd[3] = 0xC0u;
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_adbms_make_status_sequence(stata, statb, statc, statd, state, 10u);
    EXPECT_TRUE(adbms6830_refresh_diagnostics(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.cell_ovuv_fault_ic_mask == 0u);

    statd[3] = 0x30u;
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_adbms_make_status_sequence(stata, statb, statc, statd, state, 11u);
    /* Monitored-cell OV/UV is owned by voltage_fault.c.  A clean Status-D
     * transaction with an asserted comparator remains a valid monitor image,
     * not a generic ADBMS transport/silicon failure. */
    EXPECT_TRUE(adbms6830_refresh_diagnostics(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.cell_ovuv_fault_ic_mask == 0x0001u);
    EXPECT_TRUE(adbms6830_non_cs_diagnostics_ok(&dev));

    /* Startup remains inhibited if even the first flag-clear transfer fails. */
    memset(statd, 0, sizeof(statd));
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_spi_tx_status_sequence_count = 1u;
    unit_spi_tx_status_sequence[0] = HAL_TIMEOUT;
    EXPECT_TRUE(adbms6830_establish_diagnostic_baseline(&dev) == HAL_TIMEOUT);
    EXPECT_FALSE(dev.health.startup_baseline_passed);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);

    /* CLOVUV requires one six-byte data record per IC.  Stop on that second
     * write so the test can inspect the actual framed transaction rather than
     * accepting a command-header-only mock. */
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_spi_tx_status_sequence_count = 2u;
    unit_spi_tx_status_sequence[0] = HAL_OK;
    unit_spi_tx_status_sequence[1] = HAL_TIMEOUT;
    EXPECT_TRUE(adbms6830_establish_diagnostic_baseline(&dev) == HAL_TIMEOUT);
    EXPECT_FALSE(dev.health.startup_baseline_passed);
    EXPECT_TRUE(unit_spi_tx_calls == 2u);
    EXPECT_TRUE(unit_spi_last_tx_len == (CMDSZ + PEC15SZ + TX_DATA + DPECSZ));
    EXPECT_TRUE(unit_spi_last_tx[0] == CLOVUV[0]);
    EXPECT_TRUE(unit_spi_last_tx[1] == CLOVUV[1]);
    EXPECT_TRUE(unit_spi_last_tx[4] == 0xFFu);
    EXPECT_TRUE(unit_spi_last_tx[5] == 0xFFu);
    EXPECT_TRUE(unit_spi_last_tx[6] == 0xFFu);
    EXPECT_TRUE(unit_spi_last_tx[7] == 0xFFu);
    EXPECT_TRUE(unit_spi_last_tx[8] == 0x00u);
    EXPECT_TRUE(unit_spi_last_tx[9] == 0x00u);
    EXPECT_TRUE((((uint16_t)unit_spi_last_tx[10] << 8u) |
                 unit_spi_last_tx[11]) ==
                (uint16_t)pec10_calc_modular(&unit_spi_last_tx[4], PEC10_WRITE));

    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    statd[5] = 60u;
    unit_adbms_make_status_sequence(stata, statb, statc, statd, state, 12u);
    EXPECT_TRUE(adbms6830_establish_diagnostic_baseline(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.startup_baseline_passed);
    EXPECT_TRUE(adbms6830_safety_diagnostics_ok(&dev));
    EXPECT_TRUE(unit_spi_txrx_calls == 5u);
}

static void test_adbms_counter_mismatch_rejects_stale_data(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    const uint8_t statc[TX_DATA] = {0u};
    const uint8_t statd[TX_DATA] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0u, 0xA5u};
    const uint8_t state[TX_DATA] = {0u};

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    /* RDSTATC establishes the expected count. RDSTATD has a valid PEC but
     * the wrong count and therefore must neither overwrite the old parsed
     * status nor be marked valid. RDSTATE demonstrates recovery after the
     * counter tracker resynchronizes. */
    dev.diag[0].cell_ov_mask = 0x1234u;
    dev.diag[0].cell_uv_mask = 0x5678u;
    dev.diag[0].osc_counter = 0x3Cu;
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], statc, 7u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], statd, 8u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], state, 8u, false);

    EXPECT_TRUE(adbms6830_read_status(&dev, false) == HAL_ERROR);
    EXPECT_TRUE(dev.diag[0].statc_valid);
    EXPECT_FALSE(dev.diag[0].statd_valid);
    EXPECT_TRUE(dev.diag[0].state_valid);
    EXPECT_TRUE(dev.diag[0].cell_ov_mask == 0x1234u);
    EXPECT_TRUE(dev.diag[0].cell_uv_mask == 0x5678u);
    EXPECT_TRUE(dev.diag[0].osc_counter == 0x3Cu);
    EXPECT_TRUE(dev.health.sticky_cmd_counter_mismatch_mask == 0x0001u);

    /* The same rule applies to voltage groups. A counter mismatch is now a
     * critical coherent-epoch failure: stop immediately, retry the whole SNAP
     * epoch once, and never refresh the mismatched group's old cell values. */
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    adbms6830_resync_command_counter_tracking(&dev);
    ics[0].cell.c_codes[3] = 1111;
    ics[0].cell.c_codes[4] = 2222;
    ics[0].cell.c_codes[5] = 3333;
    unit_spi_txrx_sequence_count = 4u;
    unit_adbms_make_valid_read_packet(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], 0x10u, 5u, false);
    unit_adbms_make_valid_read_packet(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], 0x20u, 6u, false);
    /* First cleanup UNSNAP advances 6 -> 7 and retry SNAP advances 7 -> 8. */
    unit_adbms_make_valid_read_packet(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], 0x30u, 8u, false);
    unit_adbms_make_valid_read_packet(
        &unit_spi_txrx_sequence[3][CMDSZ + PEC15SZ], 0x40u, 9u, false);

    EXPECT_TRUE(adbms6830_read_cell_voltages(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.last_cell_updated_mask[0] == 0x0007u);
    EXPECT_TRUE(dev.last_cell_pec_mask[0] == 0u);
    EXPECT_TRUE(ics[0].cell.c_codes[3] == 1111);
    EXPECT_TRUE(ics[0].cell.c_codes[4] == 2222);
    EXPECT_TRUE(ics[0].cell.c_codes[5] == 3333);
    EXPECT_TRUE(dev.health.sticky_cmd_counter_mismatch_mask == 0x0001u);
    EXPECT_TRUE(unit_spi_txrx_calls == 4u);
}

static void test_adbms_cell_adc_conversion_diagnostic(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t stata[TX_DATA] = {0u};
    uint8_t statb[TX_DATA] = {0u};
    uint8_t statc[TX_DATA] = {0u};
    uint8_t coherent_statc[TX_DATA] = {0u};
    uint8_t statd[TX_DATA] = {0u};
    uint8_t state[TX_DATA] = {0u};
    const uint8_t *status_payloads[5] = {stata, statb, statc, statd, state};

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    /* ADCV then SNAP put the tracked command counter at 10 in this fixture.
     * The coherent raw epoch now reads six C groups plus Status C (CCTS
     * freshness) and Status D (OV/UV image), all at that same counter.  After
     * UNSNAP and the diagnostic ADAX it advances to 12 for Status A-E. */
    unit_spi_txrx_sequence_count = 13u;
    for(uint8_t group = 0u; group < 6u; group++)
    {
        unit_adbms_make_valid_read_packet(
            &unit_spi_txrx_sequence[group][CMDSZ + PEC15SZ],
            (uint8_t)(0x10u + (group * 0x10u)),
            10u,
            false);
    }
    unit_adbms_make_statc_ccts_payload(coherent_statc, 4u);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[6u][CMDSZ + PEC15SZ],
        coherent_statc, 10u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[7u][CMDSZ + PEC15SZ],
        statd, 10u, false);

    unit_adbms_put_s16(stata, 0u, 10000); /* VREF2 = 3.000 V */
    unit_adbms_put_s16(stata, 1u, 4900);  /* ITMP = 25.0 C */
    unit_adbms_put_s16(statb, 0u, 12000); /* VD = 3.300 V */
    unit_adbms_put_s16(statb, 1u, 23333); /* VA ~= 5.000 V */
    unit_adbms_put_s16(statb, 2u, 10000); /* VRES = 3.000 V */
    statd[5] = 60u;
    for(uint8_t group = 0u; group < 5u; group++)
    {
        unit_adbms_make_read_packet_from_data(
            &unit_spi_txrx_sequence[8u + group][CMDSZ + PEC15SZ],
            status_payloads[group],
            12u,
            false);
    }

    EXPECT_TRUE(adbms6830_run_cell_adc_self_test(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.cell_adc_self_test_count == 1u);
    EXPECT_TRUE((dev.last_cell_updated_mask[0] & 0x7FFFu) == 0x7FFFu);
    EXPECT_TRUE(dev.health.cadc_ccts_valid_ic_mask == 0x0001u);
    EXPECT_TRUE(dev.health.cadc_ccts_fault_ic_mask == 0u);
    EXPECT_TRUE(dev.health.status_fault_ic_mask == 0u);
    EXPECT_TRUE(unit_spi_txrx_calls == 13u);

    /* A plausible stale payload with a bad data PEC must fail the coherent
     * diagnostic.  The reader retries exactly once; the retry uses the
     * command count after the first attempt's cleanup/re-SNAP. */
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    adbms6830_resync_command_counter_tracking(&dev);
    unit_spi_txrx_sequence_count = 6u;
    for(uint8_t group = 0u; group < 3u; group++)
    {
        unit_adbms_make_valid_read_packet(
            &unit_spi_txrx_sequence[group][CMDSZ + PEC15SZ],
            (uint8_t)(0x20u + (group * 0x10u)),
            20u,
            group == 2u);
        unit_adbms_make_valid_read_packet(
            &unit_spi_txrx_sequence[3u + group][CMDSZ + PEC15SZ],
            (uint8_t)(0x50u + (group * 0x10u)),
            22u,
            group == 2u);
    }
    EXPECT_TRUE(adbms6830_run_cell_adc_self_test(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.health.cell_adc_self_test_count == 2u);
    EXPECT_TRUE(unit_spi_txrx_calls == 6u);
}

static void test_adbms_spi_coldwake_and_clear_flags(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    unit_spi_reset();
    adbms6830_wakeup_cold(&dev);
    EXPECT_TRUE(dev.spi_debug.last_op == ADBMS6830_SPI_OP_COLD_WAKE);
    EXPECT_TRUE(unit_gpio_write_calls == 4u);
    EXPECT_TRUE(unit_gpio_states[dev.cs_pin[dev.write_string]] == GPIO_PIN_SET);

    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    EXPECT_TRUE(adbms6830_clear_all_flags(&dev) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(dev.spi_debug.last_op == ADBMS6830_SPI_OP_CLEAR_FLAGS);
    EXPECT_TRUE(ics[0].clrflag.tx_data[0] == 0xFFu);
    EXPECT_TRUE(ics[0].clrflag.tx_data[1] == 0xFFu);
    EXPECT_TRUE(ics[0].clrflag.tx_data[4] == 0xFFu);
    EXPECT_TRUE(ics[0].clrflag.tx_data[5] == 0xDFu);
}

static void test_adbms_pwm_write_packing(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint16_t frame_len = (uint16_t)(CMDSZ + PEC15SZ + TX_DATA + DPECSZ);

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    ics[0].PwmA.pwma[0] = PWM_33_0_PCT;
    ics[0].PwmA.pwma[1] = PWM_66_0_PCT;
    ics[0].PwmA.pwma[10] = PWM_19_8_PCT;
    ics[0].PwmA.pwma[11] = PWM_26_4_PCT;
    unit_spi_reset();
    EXPECT_TRUE(adbms6830_wrpwma_checked(&dev) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_tx_len == frame_len);
    EXPECT_TRUE(unit_spi_last_tx[0] == 0x00u);
    EXPECT_TRUE(unit_spi_last_tx[1] == 0x20u);
    EXPECT_TRUE(unit_spi_last_tx[4] == ((PWM_66_0_PCT << 4) | PWM_33_0_PCT));
    EXPECT_TRUE(unit_spi_last_tx[9] == ((PWM_26_4_PCT << 4) | PWM_19_8_PCT));

    ics[0].PwmB.pwmb[0] = PWM_39_6_PCT;
    ics[0].PwmB.pwmb[1] = PWM_46_2_PCT;
    ics[0].PwmB.pwmb[2] = PWM_52_8_PCT;
    ics[0].PwmB.pwmb[3] = PWM_59_4_PCT;
    unit_spi_reset();
    EXPECT_TRUE(adbms6830_wrpwmb_checked(&dev) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_tx_len == frame_len);
    EXPECT_TRUE(unit_spi_last_tx[0] == 0x00u);
    EXPECT_TRUE(unit_spi_last_tx[1] == 0x21u);
    EXPECT_TRUE(unit_spi_last_tx[4] == ((PWM_46_2_PCT << 4) | PWM_39_6_PCT));
    EXPECT_TRUE(unit_spi_last_tx[5] == ((PWM_59_4_PCT << 4) | PWM_52_8_PCT));
    EXPECT_TRUE(unit_spi_last_tx[6] == 0u);
    EXPECT_TRUE(unit_spi_last_tx[9] == 0u);
}

static void test_adbms6830_final_ring_subset_write_owner(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[5];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    const uint16_t five_packet_frame =
        (uint16_t)(CMDSZ + PEC15SZ + (5u * (TX_DATA + DPECSZ)));

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 5u);
    dev.physical_chain_count = 6u;

    /* A -> five 6830s -> one 2950 -> B. ADI explicitly permits ending a
     * write at an eight-byte device boundary, so the SMB subset is 44 bytes. */
    unit_spi_reset();
    EXPECT_TRUE(adbms6830_wrcfgb_checked(&dev) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_tx_len == five_packet_frame);

    /* The same five-packet subset from the non-owner String A is forbidden;
     * the current production write owner is String B. */
    dev.string = STRING_A;
    unit_spi_reset();
    EXPECT_TRUE(adbms6830_wrcfgb_checked(&dev) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);
    EXPECT_TRUE(unit_gpio_write_calls == 0u);
}

static void test_adbms6830_cfgb_timer_write_guard(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    const adbms6830_cfgb_write_event_t *event;

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);
    dev.string = dev.write_string;
    /* The common fixture intentionally preserves the initialization write.
     * Reset only this test's private audit ring so the assertions below are
     * relative to the mutation/recovery sequence under test. */
    memset(dev.cfgb_write_history, 0, sizeof(dev.cfgb_write_history));
    dev.cfgb_write_history_count = 0u;
    dev.cfgb_write_history_index = 0u;
    dev.cfgb_write_total_count = 0u;

    /* Recreate the observed nonzero CFGBR3 image.  A balance path must reject
     * it before any SPI clocks, preserve the exact payload in history, and
     * restore the software shadow to the reviewed timer-disabled policy. */
    ics[0].tx_cfgb.dtmen = 0u;
    ics[0].tx_cfgb.dtrng = 0u;
    ics[0].tx_cfgb.dcto = 41u;
    ics[0].tx_cfgb.dcc = 0u;
    unit_spi_reset();
    EXPECT_TRUE(adbms6830_wrcfgb_checked_reason(
                    &dev, ADBMS6830_CFGB_WRITE_BALANCE_APPLY) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(dev.health.config_write_guard_fault_mask == 0x0001u);
    EXPECT_TRUE(dev.health.sticky_config_write_guard_fault_mask == 0x0001u);
    EXPECT_TRUE(dev.health.config_write_guard_reject_count[0] == 1u);
    EXPECT_TRUE(ics[0].tx_cfgb.dtmen == 0u);
    EXPECT_TRUE(ics[0].tx_cfgb.dtrng == 0u);
    EXPECT_TRUE(ics[0].tx_cfgb.dcto == 0u);
    EXPECT_TRUE(dev.cfgb_write_history_count == 1u);
    event = &dev.cfgb_write_history[0];
    EXPECT_TRUE(event->reason == ADBMS6830_CFGB_WRITE_BALANCE_APPLY);
    EXPECT_TRUE(event->payload[0][3] == 0x29u);
    EXPECT_TRUE(event->timer_nonzero_mask == 0x0001u);
    EXPECT_TRUE(event->rejected_mask == 0x0001u);
    EXPECT_TRUE(event->status == HAL_ERROR);

    /* The next safe clear writes exactly the sanitized image. */
    unit_spi_reset();
    EXPECT_TRUE(adbms6830_wrcfgb_checked_reason(
                    &dev, ADBMS6830_CFGB_WRITE_BALANCE_CLEAR) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_tx[0] == 0x00u);
    EXPECT_TRUE(unit_spi_last_tx[1] == 0x24u);
    EXPECT_TRUE(unit_spi_last_tx[7] == 0x00u);
    EXPECT_TRUE(dev.cfgb_write_history_count == 2u);
    event = &dev.cfgb_write_history[1];
    EXPECT_TRUE(event->payload[0][3] == 0x00u);
    EXPECT_TRUE(event->rejected_mask == 0u);
    EXPECT_TRUE(event->status == HAL_OK);

    /* Clearing ordinary SPI counters must not erase the CFGB audit trail. */
    adbms6830_spi_debug_clear(&dev);
    EXPECT_TRUE(dev.cfgb_write_history_count == 2u);
    EXPECT_TRUE(dev.cfgb_write_total_count == 2u);

    /* The guard remains active until a transport-clean readback proves the
     * actual device image matches the safe shadow. */
    adbms6830_pack_cfga(&dev);
    adbms6830_pack_cfgb(&dev);
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 2u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ],
        ics[0].configa.tx_data,
        4u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ],
        ics[0].configb.tx_data,
        4u,
        false);
    EXPECT_TRUE(adbms6830_verify_config_readback(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.config_write_guard_fault_mask == 0u);
    EXPECT_TRUE(dev.health.sticky_config_write_guard_fault_mask == 0x0001u);
}

static void test_adbms_config_and_balance_readback(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t bad_pwma[TX_DATA];

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    adbms6830_pack_cfga(&dev);
    adbms6830_pack_cfgb(&dev);
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_spi_txrx_sequence_count = 2u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ],
        ics[0].configa.tx_data,
        4u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ],
        ics[0].configb.tx_data,
        4u,
        false);
    EXPECT_TRUE(adbms6830_verify_config_readback(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.config_mismatch_mask == 0u);

    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_spi_txrx_sequence_count = 2u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ],
        ics[0].configa.tx_data,
        5u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ],
        ics[0].configb.tx_data,
        5u,
        true);
    EXPECT_TRUE(adbms6830_verify_config_readback(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.health.configb_mismatch_mask == 0x0001u);
    EXPECT_TRUE(dev.health.config_mismatch_mask == 0x0001u);

    ics[0].PwmA.pwma[0] = PWM_33_0_PCT;
    ics[0].PwmA.pwma[11] = PWM_26_4_PCT;
    ics[0].PwmB.pwmb[0] = PWM_39_6_PCT;
    adbms6830_pack_cfgb(&dev);
    adbms6830_pack_pwma(&dev);
    adbms6830_pack_pwmb(&dev);

    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ],
        ics[0].configb.tx_data,
        6u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ],
        ics[0].pwma.tx_data,
        6u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ],
        ics[0].pwmb.tx_data,
        6u,
        false);
    EXPECT_TRUE(adbms6830_verify_balance_readback(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.balance_mismatch_mask == 0u);
    EXPECT_TRUE(dev.health.last_op == ADBMS6830_SPI_OP_BALANCE_CHECK);
    EXPECT_TRUE(unit_spi_txrx_calls == 3u);

    memcpy(bad_pwma, ics[0].pwma.tx_data, sizeof(bad_pwma));
    bad_pwma[0] ^= 0x01u;
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ],
        ics[0].configb.tx_data,
        7u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ],
        bad_pwma,
        7u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ],
        ics[0].pwmb.tx_data,
        7u,
        false);
    EXPECT_TRUE(adbms6830_verify_balance_readback(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.health.balance_cfgb_mismatch_mask == 0u);
    EXPECT_TRUE(dev.health.balance_pwma_mismatch_mask == 0x0001u);
    EXPECT_TRUE(dev.health.balance_pwmb_mismatch_mask == 0u);
    EXPECT_TRUE(dev.health.balance_mismatch_count[0] >= 1u);

    /* Counter integrity remains safety-active when verbose SPI logging is off. */
    unit_spi_reset();
    adbms6830_spi_debug_enable(&dev, false);
    adbms6830_spi_debug_clear(&dev);
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ],
        ics[0].configb.tx_data,
        8u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ],
        ics[0].pwma.tx_data,
        9u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ],
        ics[0].pwmb.tx_data,
        9u,
        false);
    EXPECT_TRUE(adbms6830_verify_balance_readback(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.health.balance_pwma_mismatch_mask == 0x0001u);
    EXPECT_TRUE(dev.health.sticky_cmd_counter_mismatch_mask == 0x0001u);
    adbms6830_spi_debug_enable(&dev, true);

    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_spi_txrx_status = HAL_TIMEOUT;
    EXPECT_TRUE(adbms6830_verify_balance_readback(&dev) == HAL_TIMEOUT);
    EXPECT_TRUE(dev.health.balance_mismatch_mask == 0x0001u);
    EXPECT_TRUE(unit_spi_txrx_calls == 3u);
    unit_spi_txrx_status = HAL_OK;
}


static void test_adbms_i2c_comm_releases_sda_for_slave_ack(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio;

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio, 0, sizeof(gpio));
    dev.num_ics = 1;
    dev.physical_chain_count = 1u;
    dev.ics_capacity = 1u;
    dev.ics = ics;
    dev.hspi = &spi;
    dev.string = STRING_A;
    dev.write_string = STRING_A;
    dev.cs_port[STRING_A] = &gpio;
    dev.cs_pin[STRING_A] = 1u;

    ics[0].comm.icomm[0] = ICOMM_START_;
    ics[0].comm.fcomm[0] = FCOMM_RELEASE_FOR_SLAVE_ACK_;
    ics[0].comm.data[0] = 0x98u;
    ics[0].comm.icomm[1] = ICOMM_BLANK_;
    ics[0].comm.fcomm[1] = FCOMM_RELEASE_FOR_SLAVE_ACK_;
    ics[0].comm.data[1] = 0x00u;
    ics[0].comm.icomm[2] = ICOMM_STOP_;
    ics[0].comm.fcomm[2] = FCOMM_NACK_STOP_;
    ics[0].comm.data[2] = 0xFFu;

    adbms6830_pack_comm(&dev);

    EXPECT_TRUE(ics[0].com.tx_data[0] == 0x68u);
    EXPECT_TRUE(ics[0].com.tx_data[1] == 0x98u);
    EXPECT_TRUE(ics[0].com.tx_data[2] == 0x08u);
    EXPECT_TRUE(ics[0].com.tx_data[3] == 0x00u);
    EXPECT_TRUE(ics[0].com.tx_data[4] == 0x19u);
    EXPECT_TRUE(ics[0].com.tx_data[5] == 0xFFu);

    /* 0x60/0x00 was the invalid v0.3.2 probe pattern: it made the master
     * generate ACK itself instead of releasing SDA for the slave. */
    EXPECT_TRUE(ics[0].com.tx_data[0] != 0x60u);
    EXPECT_TRUE(ics[0].com.tx_data[2] != 0x00u);
}

static void test_adbms_mux_ack_and_temperature_freshness(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    const uint8_t sensor = 5u;
    const uint32_t sensor_bit = (uint32_t)(1UL << sensor);
    int16_t original_raw;

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    unit_adbms_make_comm_result(&unit_spi_txrx_response[CMDSZ + PEC15SZ],
                                2u,
                                true,
                                true,
                                false);
    EXPECT_TRUE(mux_set_channel(&dev, sensor) == 0);
    EXPECT_TRUE(unit_spi_tx_calls == 2u);
    EXPECT_TRUE(unit_spi_txrx_calls == 1u);
    EXPECT_TRUE(dev.mux_selection_valid_mask[0] == 0x0001u);
    EXPECT_TRUE(dev.mux_selected_channel[0][0] == sensor);

    unit_spi_reset();
    unit_adbms_make_aux_result(&unit_spi_txrx_response[CMDSZ + PEC15SZ],
                               0u,
                               0x1234u,
                               3u,
                               false);
    EXPECT_TRUE(mux_read_gpio_voltage(&dev, sensor) == 0);
    EXPECT_TRUE(dev.ics[0].temp.raw[sensor] == (int16_t)0x1234);
    EXPECT_TRUE((dev.last_temp_updated_mask[0] & sensor_bit) != 0u);
    original_raw = dev.ics[0].temp.raw[sensor];

    /* A plausible old value must not be republished after a PEC failure. */
    unit_spi_reset();
    unit_adbms_make_aux_result(&unit_spi_txrx_response[CMDSZ + PEC15SZ],
                               0u,
                               0x5678u,
                               4u,
                               true);
    EXPECT_TRUE(mux_read_gpio_voltage(&dev, sensor) == -1);
    EXPECT_TRUE(dev.ics[0].temp.raw[sensor] == original_raw);
    EXPECT_TRUE((dev.last_temp_updated_mask[0] & sensor_bit) == 0u);

    /* ADBMS invalid-code sentinels likewise remain stale. */
    unit_spi_reset();
    unit_adbms_make_aux_result(&unit_spi_txrx_response[CMDSZ + PEC15SZ],
                               0u,
                               0xFFFFu,
                               5u,
                               false);
    EXPECT_TRUE(mux_read_gpio_voltage(&dev, sensor) == -1);
    EXPECT_TRUE(dev.ics[0].temp.raw[sensor] == original_raw);
    EXPECT_TRUE((dev.last_temp_updated_mask[0] & sensor_bit) == 0u);

    unit_spi_reset();
    unit_spi_txrx_status = HAL_TIMEOUT;
    EXPECT_TRUE(mux_read_gpio_voltage(&dev, sensor) == -1);
    EXPECT_TRUE(dev.ics[0].temp.raw[sensor] == original_raw);
    EXPECT_TRUE((dev.last_temp_updated_mask[0] & sensor_bit) == 0u);

    /* A different requested sensor cannot reuse the previous selection. */
    unit_spi_reset();
    EXPECT_TRUE(mux_read_gpio_voltage(&dev, (uint8_t)(sensor - 1u)) == -1);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);
}

static void test_adbms_mux_transport_and_ack_failures(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    const uint8_t sensor = 9u;

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    unit_adbms_make_comm_result(&unit_spi_txrx_response[CMDSZ + PEC15SZ],
                                2u,
                                false,
                                true,
                                false);
    EXPECT_TRUE(mux_set_channel(&dev, sensor) == -1);
    EXPECT_TRUE(dev.mux_selection_valid_mask[1] == 0u);
    EXPECT_TRUE(dev.mux_selected_channel[0][1] == UINT8_MAX);

    unit_spi_reset();
    unit_adbms_make_comm_result(&unit_spi_txrx_response[CMDSZ + PEC15SZ],
                                4u,
                                true,
                                true,
                                true);
    EXPECT_TRUE(mux_set_channel(&dev, sensor) == -1);
    EXPECT_TRUE(dev.mux_selection_valid_mask[1] == 0u);

    /* WRCOMM success followed by STCOMM failure must not reach RDCOMM. */
    unit_spi_reset();
    unit_spi_tx_status_sequence_count = 2u;
    unit_spi_tx_status_sequence[0] = HAL_OK;
    unit_spi_tx_status_sequence[1] = HAL_TIMEOUT;
    EXPECT_TRUE(mux_set_channel(&dev, sensor) == -1);
    EXPECT_TRUE(unit_spi_tx_calls == 2u);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);
    EXPECT_TRUE(dev.mux_selection_valid_mask[1] == 0u);

    unit_spi_reset();
    unit_spi_tx_status_sequence_count = 1u;
    unit_spi_tx_status_sequence[0] = HAL_ERROR;
    EXPECT_TRUE(mux_set_channel(&dev, sensor) == -1);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);
    EXPECT_TRUE(dev.mux_selection_valid_mask[1] == 0u);
}

static void unit_adbms_fill_open_wire_phase(uint32_t first_sequence,
                                             uint8_t counter,
                                             int16_t nominal_code)
{
    for(uint32_t group = 0u; group < 5u; group++)
    {
        unit_adbms_make_s_group_result(
            &unit_spi_txrx_sequence[first_sequence + group][CMDSZ + PEC15SZ],
            nominal_code,
            nominal_code,
            nominal_code,
            counter,
            false);
    }
}

static void test_adbms_open_wire_full_measurement_and_fault_injection(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    const int16_t baseline_code = 12200;   /* 3.33 V. */
    const int16_t stimulated_code = 10000; /* 3.00 V, about 10% attenuation. */

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    EXPECT_TRUE(dev.monitored_cell_count == 15u);
    EXPECT_FALSE(adbms6830_set_monitored_cell_count(&dev, 0u));
    EXPECT_FALSE(adbms6830_set_monitored_cell_count(&dev, (uint8_t)(CELL + 1u)));
    EXPECT_TRUE(dev.monitored_cell_count == 15u);

    /* ADI's complete sequence is baseline, even stimulus, then odd stimulus.
     * A 15-cell SMB reads only S groups A..E in each phase. */
    unit_spi_txrx_sequence_count = 15u;
    unit_adbms_fill_open_wire_phase(0u, 7u, baseline_code);
    unit_adbms_fill_open_wire_phase(5u, 8u, stimulated_code);
    unit_adbms_fill_open_wire_phase(10u, 9u, stimulated_code);
    EXPECT_TRUE(adbms6830_run_open_wire_diagnostic(&dev) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 3u);
    EXPECT_TRUE(unit_spi_txrx_calls == 15u);
    EXPECT_TRUE(unit_spi_last_tx_len == (CMDSZ + PEC15SZ));
    EXPECT_TRUE(unit_spi_last_tx[0] == 0x01u);
    EXPECT_TRUE(unit_spi_last_tx[1] == 0x6Au); /* ADSV, single, DCP=0, odd OW. */
    EXPECT_TRUE(dev.health.open_wire_full_count == 1u);
    EXPECT_TRUE(dev.health.open_wire_baseline_count == 1u);
    EXPECT_TRUE(dev.health.open_wire_even_count == 1u);
    EXPECT_TRUE(dev.health.open_wire_odd_count == 1u);
    EXPECT_TRUE(dev.health.open_wire_even_valid_ic_mask == 0x0001u);
    EXPECT_TRUE(dev.health.open_wire_odd_valid_ic_mask == 0x0001u);
    EXPECT_TRUE(dev.health.open_wire_incomplete_ic_mask == 0u);
    EXPECT_TRUE(dev.health.open_wire_fault_ic_mask == 0u);
    EXPECT_TRUE(dev.health.open_wire_baseline_valid_ic_mask == 0x0001u);
    EXPECT_TRUE(ics[0].owcell.cell_ow_even[14] == stimulated_code);
    EXPECT_TRUE(ics[0].owcell.cell_ow_odd[14] == stimulated_code);
    EXPECT_TRUE(dev.diag[0].open_wire_even_attenuation_fault_mask == 0u);
    EXPECT_TRUE(dev.diag[0].open_wire_odd_attenuation_fault_mask == 0u);

    /* A plausible absolute voltage is not sufficient: no response to the
     * internal stimulus is a latent diagnostic-switch/open-wire-path fault. */
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 10u;
    unit_adbms_fill_open_wire_phase(0u, 10u, baseline_code);
    unit_adbms_fill_open_wire_phase(5u, 11u, baseline_code);
    EXPECT_TRUE(adbms6830_run_open_wire_check(&dev, false) == HAL_ERROR);
    EXPECT_TRUE(dev.diag[0].open_wire_even_valid);
    EXPECT_TRUE(dev.diag[0].open_wire_even_attenuation_fault_mask != 0u);
    EXPECT_TRUE(dev.health.open_wire_fault_ic_mask == 0x0001u);

    /* The S-ADC data-sheet gain floor is 85%.  A still-plausible 2.664 V
     * result is about 20% below this 3.33 V baseline and must therefore fail
     * the diagnostic-divider plausibility check. */
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 10u;
    unit_adbms_fill_open_wire_phase(0u, 12u, baseline_code);
    unit_adbms_fill_open_wire_phase(5u, 13u, 7760);
    EXPECT_TRUE(adbms6830_run_open_wire_check(&dev, false) == HAL_ERROR);
    EXPECT_TRUE(dev.diag[0].open_wire_even_valid);
    EXPECT_TRUE(dev.diag[0].open_wire_even_attenuation_fault_mask != 0u);

    /* In the even-channel phase, ADI channel C2 is zero-based index 1. A
     * 1.5 V result is below the 2.0 V open-wire threshold and must latch the
     * exact cell bit while remaining a structurally valid measurement. */
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 10u;
    unit_adbms_fill_open_wire_phase(0u, 14u, baseline_code);
    unit_adbms_fill_open_wire_phase(5u, 15u, stimulated_code);
    unit_adbms_make_s_group_result(
        &unit_spi_txrx_sequence[5][CMDSZ + PEC15SZ],
        stimulated_code,
        0,
        stimulated_code,
        15u,
        false);
    EXPECT_TRUE(adbms6830_run_open_wire_check(&dev, false) == HAL_ERROR);
    EXPECT_TRUE(dev.diag[0].open_wire_even_valid);
    EXPECT_TRUE(dev.diag[0].open_wire_even_fault_mask == (uint16_t)(1u << 1u));
    EXPECT_TRUE(dev.health.open_wire_fault_ic_mask == 0x0001u);
    EXPECT_TRUE(dev.health.sticky_open_wire_fault_ic_mask == 0x0001u);

    dev.last_cell_updated_mask[0] = 0x7FFFu;
    adbms6830_mask_open_wire_cells(&dev);
    EXPECT_FALSE((dev.last_cell_updated_mask[0] & (uint16_t)(1u << 1u)) != 0u);
    EXPECT_TRUE((dev.last_cell_updated_mask[0] & (uint16_t)(1u << 0u)) != 0u);

    /* A PEC-bad group must not overwrite its previous raw values and must
     * invalidate the phase even when every other group is valid. */
    ics[0].owcell.cell_ow_odd[0] = 777;
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 10u;
    unit_adbms_fill_open_wire_phase(0u, 16u, baseline_code);
    unit_adbms_fill_open_wire_phase(5u, 17u, stimulated_code);
    unit_adbms_make_s_group_result(
        &unit_spi_txrx_sequence[5][CMDSZ + PEC15SZ],
        stimulated_code,
        stimulated_code,
        stimulated_code,
        17u,
        true);
    EXPECT_TRUE(adbms6830_run_open_wire_check(&dev, true) == HAL_ERROR);
    EXPECT_TRUE(ics[0].owcell.cell_ow_odd[0] == 777);
    EXPECT_FALSE(dev.diag[0].open_wire_odd_valid);
    EXPECT_TRUE(dev.health.open_wire_incomplete_ic_mask == 0x0001u);
    EXPECT_TRUE(dev.health.sticky_pec_fail_mask == 0x0001u);

    /* A valid PEC with the wrong command counter is equally unusable and
     * also must not publish the affected group's data as current. */
    ics[0].owcell.cell_ow_odd[0] = 888;
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 5u;
    unit_adbms_fill_open_wire_phase(0u, 17u, baseline_code);
    EXPECT_TRUE(adbms6830_run_open_wire_check(&dev, true) == HAL_ERROR);
    EXPECT_TRUE(ics[0].owcell.cell_ow_odd[0] == 888);
    EXPECT_FALSE(dev.diag[0].open_wire_odd_valid);
    EXPECT_TRUE(dev.health.sticky_cmd_counter_mismatch_mask == 0x0001u);

    /* A frozen 1 MHz delay timer must time out before any S-register read. */
    unit_spi_reset();
    unit_delay_timer_advances = false;
    EXPECT_TRUE(adbms6830_run_open_wire_check(&dev, false) == HAL_TIMEOUT);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);
    EXPECT_FALSE(dev.diag[0].open_wire_even_valid);
    unit_delay_timer_advances = true;

    /* The present Rev5 S2N-S15N routing defect requires the same complete
     * diagnostic through the C path.  ADI documents about 10/12 of baseline
     * for an intact C path, so 2.775 V from a 3.33 V baseline is nominal. */
    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 15u;
    unit_adbms_fill_open_wire_phase(0u, 7u, baseline_code);
    unit_adbms_fill_open_wire_phase(5u, 8u, 8500);
    unit_adbms_fill_open_wire_phase(10u, 9u, 8500);
    EXPECT_TRUE(adbms6830_run_open_wire_diagnostic_path(
        &dev, ADBMS6830_OPEN_WIRE_PATH_C) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 3u);
    EXPECT_TRUE(unit_spi_txrx_calls == 15u);
    EXPECT_TRUE(unit_spi_last_tx[0] == 0x02u);
    EXPECT_TRUE(unit_spi_last_tx[1] == 0x62u); /* ADCV, RD_OFF, odd OW. */
    EXPECT_TRUE(dev.health.open_wire_c_full_count == 1u);
    EXPECT_TRUE(dev.health.open_wire_s_full_count == 0u);
    EXPECT_TRUE(dev.health.open_wire_last_path == ADBMS6830_OPEN_WIRE_PATH_C);
    EXPECT_TRUE(dev.diag[0].open_wire_path == ADBMS6830_OPEN_WIRE_PATH_C);
    EXPECT_TRUE(dev.health.open_wire_incomplete_ic_mask == 0u);
    EXPECT_TRUE(dev.health.open_wire_fault_ic_mask == 0u);
    EXPECT_TRUE(dev.diag[0].open_wire_even_attenuation_fault_mask == 0u);
    EXPECT_TRUE(dev.diag[0].open_wire_odd_attenuation_fault_mask == 0u);
}


static void unit_adbms2950_init_driver(adbms2950_driver_t *dev,
                                       adbms2950_asic *ics,
                                       SPI_HandleTypeDef *spi,
                                       GPIO_TypeDef *gpio_a,
                                       GPIO_TypeDef *gpio_b,
                                       uint8_t num_ics)
{
    memset(dev, 0, sizeof(*dev));
    memset(ics, 0, sizeof(adbms2950_asic) * num_ics);
    memset(&unit_delay_timer_instance, 0, sizeof(unit_delay_timer_instance));
    memset(&unit_delay_timer, 0, sizeof(unit_delay_timer));
    unit_delay_timer.Instance = &unit_delay_timer_instance;
    dev->num_ics = num_ics;
    dev->ics_capacity = num_ics;
    dev->ics = ics;
    dev->hspi = spi;
    dev->cs_port[0] = gpio_a;
    dev->cs_port[1] = gpio_b;
    dev->cs_pin[0] = 5u;
    dev->cs_pin[1] = 6u;
    dev->string = STRING_A;
    dev->write_string = STRING_A;
    dev->htim = &unit_delay_timer;
    adbms2950_spi_debug_clear(dev);
    adbms2950_spi_debug_enable(dev, true);
    unit_spi_reset();
}

static void test_adbms2950_pec_write_is_bounded_and_reference_equal(void)
{
    uint8_t payload[TX_DATA] = {0x10u, 0x23u, 0x45u, 0x67u, 0x89u, 0xABu};
    uint8_t original[TX_DATA];
    uint16_t modular;
    uint16_t reference;

    memcpy(original, payload, sizeof(payload));
    modular = pec10_calc_modular(payload, PEC10_WRITE);
    reference = pec10_calc(0u, TX_DATA, payload);

    EXPECT_TRUE(modular == reference);
    EXPECT_TRUE(memcmp(payload, original, sizeof(payload)) == 0);
    EXPECT_TRUE(pec10_calc_modular(NULL, PEC10_WRITE) == 0xFFFFu);
}

static void test_adbms2950_spi_debug_write_and_full_duplex_paths(void)
{
    adbms2950_driver_t dev;
    adbms2950_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t tx[4] = {0x11u, 0x22u, 0x33u, 0x44u};
    uint8_t rx[6];

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms2950_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    EXPECT_TRUE(adbms2950_spi_write(&dev, tx, sizeof(tx), 1u) == HAL_OK);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_tx_len == sizeof(tx));
    EXPECT_TRUE(unit_gpio_states[5u] == GPIO_PIN_SET);
    EXPECT_TRUE(dev.spi_debug.tx_count == 1u);
    EXPECT_TRUE(dev.spi_debug.rx_count == 0u);
    EXPECT_TRUE(dev.spi_debug.last_tx_len == sizeof(tx));
    EXPECT_TRUE(dev.spi_debug.last_rx_len == 0u);
    EXPECT_TRUE(dev.spi_debug.last_status == HAL_OK);
    EXPECT_TRUE(memcmp(dev.spi_debug.last_tx_preview, tx, sizeof(tx)) == 0);

    unit_spi_tx_status = HAL_BUSY;
    EXPECT_TRUE(adbms2950_spi_write(&dev, tx, sizeof(tx), 1u) == HAL_BUSY);
    EXPECT_TRUE(dev.spi_debug.error_count == 1u);
    EXPECT_TRUE(dev.spi_debug.last_status == HAL_BUSY);

    unit_spi_reset();
    adbms2950_spi_debug_clear(&dev);
    memset(rx, 0, sizeof(rx));
    for(uint8_t i = 0u; i < sizeof(rx); i++)
    {
        unit_spi_txrx_response[sizeof(tx) + i] = (uint8_t)(0x90u + i);
    }

    EXPECT_TRUE(adbms2950_spi_write_read(&dev, tx, sizeof(tx), rx, sizeof(rx), 1u) == HAL_OK);
    EXPECT_TRUE(unit_spi_txrx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_txrx_len == (sizeof(tx) + sizeof(rx)));
    EXPECT_TRUE(memcmp(unit_spi_last_txrx_tx, tx, sizeof(tx)) == 0);
    for(uint8_t i = 0u; i < sizeof(rx); i++)
    {
        EXPECT_TRUE(unit_spi_last_txrx_tx[sizeof(tx) + i] == 0xFFu);
        EXPECT_TRUE(rx[i] == (uint8_t)(0x90u + i));
    }
    EXPECT_TRUE(dev.spi_debug.tx_count == 1u);
    EXPECT_TRUE(dev.spi_debug.rx_count == 1u);
    EXPECT_TRUE(dev.spi_debug.last_total_len == (sizeof(tx) + sizeof(rx)));
    EXPECT_TRUE(memcmp(dev.spi_debug.last_rx_preview, rx, sizeof(rx)) == 0);

    unit_spi_txrx_status = HAL_TIMEOUT;
    memset(rx, 0xA5, sizeof(rx));
    EXPECT_TRUE(adbms2950_spi_write_read(&dev, tx, sizeof(tx), rx, sizeof(rx), 1u) == HAL_TIMEOUT);
    for(uint8_t i = 0u; i < sizeof(rx); i++)
    {
        EXPECT_TRUE(rx[i] == 0u);
    }
    EXPECT_TRUE(dev.spi_debug.error_count == 1u);
    EXPECT_TRUE(dev.spi_debug.last_status == HAL_TIMEOUT);

    unit_spi_reset();
    EXPECT_TRUE(adbms2950_spi_write_read(&dev,
                                         tx,
                                         1u,
                                         rx,
                                         UINT16_MAX,
                                         1u) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);

    dev.string = (adbms_string)-1;
    adbms2950_set_cs(&dev, 0u);
    EXPECT_TRUE(unit_gpio_write_calls == 0u);
    EXPECT_TRUE(adbms2950_spi_write(&dev, tx, sizeof(tx), 1u) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    dev.string = STRING_B;
}

static void test_adbms2950_final_ring_subset_write_owner(void)
{
    adbms2950_driver_t dev;
    adbms2950_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    const uint16_t one_packet_frame =
        (uint16_t)(CMDSZ + PEC15SZ + TX_DATA + DPECSZ);

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms2950_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);
    dev.write_string = STRING_B;

    /* String A's leading device is an SMB, not this APM. */
    dev.string = STRING_A;
    unit_spi_reset();
    adbms2950_wrcfga(&dev);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(unit_gpio_write_calls == 0u);

    /* String B owns the one-device APM subset: 4 command + 8 device bytes. */
    dev.string = STRING_B;
    unit_spi_reset();
    adbms2950_wrcfga(&dev);
    EXPECT_TRUE(unit_spi_tx_calls == 1u);
    EXPECT_TRUE(unit_spi_last_tx_len == one_packet_frame);
}

static void test_adbms2950_spi_probe_pec_masks_and_clear(void)
{
    adbms2950_driver_t dev;
    adbms2950_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms2950_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    unit_adbms_make_valid_read_packet(&unit_spi_txrx_response[CMDSZ + PEC15SZ], 0x50u, 6u, false);
    EXPECT_TRUE(adbms2950_spi_probe_rdcfga(&dev) == HAL_OK);
    EXPECT_TRUE(unit_spi_txrx_calls == 1u);
    EXPECT_TRUE(dev.spi_debug.last_cmd[0] == 0x00u);
    EXPECT_TRUE(dev.spi_debug.last_cmd[1] == 0x02u);
    EXPECT_TRUE(dev.spi_debug.last_read_pec_pass_mask == 0x0001u);
    EXPECT_TRUE(dev.spi_debug.last_read_pec_fail_mask == 0x0000u);
    EXPECT_TRUE(dev.spi_debug.last_cmd_counter[0] == 6u);
    EXPECT_TRUE(dev.ics[0].rx_cmd_cntr == 6u);
    EXPECT_TRUE(dev.ics[0].rx_pec_error == 0u);
    EXPECT_TRUE(dev.spi_debug.error_count == 0u);

    unit_spi_reset();
    adbms2950_spi_debug_clear(&dev);
    unit_adbms_make_valid_read_packet(&unit_spi_txrx_response[CMDSZ + PEC15SZ], 0x60u, 2u, true);
    EXPECT_TRUE(adbms2950_spi_probe_rdcfga(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.spi_debug.last_read_pec_pass_mask == 0x0000u);
    EXPECT_TRUE(dev.spi_debug.last_read_pec_fail_mask == 0x0001u);
    EXPECT_TRUE(dev.ics[0].rx_pec_error == 1u);
    EXPECT_TRUE(dev.spi_debug.error_count == 1u);

    adbms2950_spi_debug_enable(&dev, false);
    adbms2950_spi_debug_clear(&dev);
    EXPECT_FALSE(dev.spi_debug.enabled);
    EXPECT_TRUE(dev.spi_debug.last_status == HAL_OK);
    adbms2950_spi_debug_enable(&dev, true);
    EXPECT_TRUE(dev.spi_debug.enabled);
    EXPECT_TRUE(strcmp(adbms2950_spi_op_str(ADBMS2950_SPI_OP_PROBE), "probe") == 0);
    EXPECT_TRUE(strcmp(adbms2950_spi_op_str((adbms2950_spi_op_t)99), "unknown") == 0);
}

static void test_adbms2950_mixed_chain_init_identity_and_readback(void)
{
    adbms2950_driver_t dev;
    adbms2950_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t sid[TX_DATA] = {0x11u, 0x22u, 0x33u, 0x44u, 0x55u,
                            (uint8_t)(ADBMS2950B_DEVICE_ID << 1u)};
    const uint8_t refup_cfga[TX_DATA] = {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x10u};
    const uint8_t cfga[TX_DATA] = {0x00u, 0x00u, 0x00u, 0x3Cu, 0x3Cu, 0x01u};
    const uint8_t cfgb[TX_DATA] = {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0xF0u};
    const uint8_t status_payload[TX_DATA] = {0u, 0x40u, 0u, 0u, 0u, 0x10u};
    uint8_t initialization_flag_payload[TX_DATA];
    uint8_t continuous_flag_payload[TX_DATA];

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    memset(&unit_delay_timer_instance, 0, sizeof(unit_delay_timer_instance));
    memset(&unit_delay_timer, 0, sizeof(unit_delay_timer));
    unit_delay_timer.Instance = &unit_delay_timer_instance;
    unit_delay_timer_advances = true;
    unit_spi_reset();

    unit_adbms2950_make_flag_payload(initialization_flag_payload, 150u, 0u);
    unit_adbms2950_make_flag_payload(continuous_flag_payload, 2u, 0u);

    /* Current initialization proves REFUP before any APM-specific write.
     * Reads do not advance CCNT, so SID and REFUP share epoch 1; the two
     * configuration writes advance the readback epoch to 3. */
    unit_spi_txrx_sequence_count = 8u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], sid, 1u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], refup_cfga, 1u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], cfga, 3u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[3][CMDSZ + PEC15SZ], cfgb, 3u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[4][CMDSZ + PEC15SZ], status_payload, 4u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[5][CMDSZ + PEC15SZ],
        initialization_flag_payload,
        4u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[6][CMDSZ + PEC15SZ], status_payload, 5u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[7][CMDSZ + PEC15SZ],
        continuous_flag_payload,
        5u,
        false);

    EXPECT_TRUE(adbms2950_init_mixed_chain(&dev,
                                           1u,
                                           ics,
                                           1u,
                                           &spi,
                                           &gpio_a,
                                           &gpio_b,
                                           5u,
                                           6u,
                                           &unit_delay_timer,
                                           STRING_B,
                                           false,
                                           false) == HAL_OK);
    EXPECT_TRUE(dev.string == STRING_B);
    EXPECT_TRUE(dev.num_ics == 1u);
    EXPECT_TRUE(dev.ics_capacity == 1u);
    EXPECT_TRUE(dev.health.initialized);
    EXPECT_TRUE(dev.health.sid_valid);
    EXPECT_TRUE(dev.health.config_valid);
    EXPECT_TRUE(dev.health.i1_calibrated);
    EXPECT_TRUE(dev.health.i1_continuous_ready);
    EXPECT_TRUE(dev.health.i1_conversion_count == 2u);
    EXPECT_TRUE(dev.health.counter_seen);
    EXPECT_TRUE(dev.health.device_id == ADBMS2950B_DEVICE_ID);
    EXPECT_FALSE(dev.health.hv_dividers_enabled);
    EXPECT_TRUE(unit_spi_tx_calls == 4u); /* Configs + init ADI1 + continuous ADI1. */
    EXPECT_TRUE(unit_spi_txrx_calls == 8u); /* SID, REFUP, configs, then two STAT/FLAG pairs. */
    EXPECT_TRUE(dev.ics[0].tx_cfga.gpo1c == GPO_CLR);
    EXPECT_TRUE(dev.ics[0].tx_cfga.gpo2c == GPO_CLR);

    /* If the requested configuration cannot be written, initialization
     * remains failed and a second bounded write forces both divider enables
     * low.  A cleanup success must never hide the original error. */
    unit_spi_reset();
    unit_spi_tx_status_sequence_count = 2u;
    unit_spi_tx_status_sequence[0] = HAL_TIMEOUT;
    unit_spi_tx_status_sequence[1] = HAL_OK;
    unit_spi_txrx_sequence_count = 2u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], sid, 1u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], refup_cfga, 1u, false);
    EXPECT_TRUE(adbms2950_init_mixed_chain(&dev,
                                           1u,
                                           ics,
                                           1u,
                                           &spi,
                                           &gpio_a,
                                           &gpio_b,
                                           5u,
                                           6u,
                                           &unit_delay_timer,
                                           STRING_B,
                                           false,
                                           true) == HAL_TIMEOUT);
    EXPECT_TRUE(unit_spi_tx_calls == 2u);
    EXPECT_FALSE(dev.health.initialized);
    EXPECT_FALSE(dev.health.config_valid);
    EXPECT_FALSE(dev.health.hv_dividers_enabled);
    EXPECT_TRUE(dev.ics[0].tx_cfga.gpo1c == GPO_CLR);
    EXPECT_TRUE(dev.ics[0].tx_cfga.gpo2c == GPO_CLR);

    unit_spi_reset();
    EXPECT_TRUE(adbms2950_init_mixed_chain(&dev,
                                           1u,
                                           ics,
                                           0u,
                                           &spi,
                                           &gpio_a,
                                           &gpio_b,
                                           5u,
                                           6u,
                                           &unit_delay_timer,
                                           STRING_B,
                                           false,
                                           false) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);

    /* A compatible RDSID response from the wrong product must be rejected
     * before any APM-specific write reaches the mixed chain. */
    unit_spi_reset();
    sid[5] = 0x02u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_response[CMDSZ + PEC15SZ], sid, 1u, false);
    EXPECT_TRUE(adbms2950_init_mixed_chain(&dev,
                                           1u,
                                           ics,
                                           1u,
                                           &spi,
                                           &gpio_a,
                                           &gpio_b,
                                           5u,
                                           6u,
                                           &unit_delay_timer,
                                           STRING_B,
                                           false,
                                           false) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(unit_spi_txrx_calls == 1u);
    EXPECT_FALSE(dev.health.initialized);
    EXPECT_FALSE(dev.health.sid_valid);
}

static void test_adbms2950_sid_probe_and_primary_sample_integrity(void)
{
    adbms2950_driver_t dev;
    adbms2950_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t sid[TX_DATA] = {1u, 2u, 3u, 4u, 5u,
                            (uint8_t)(ADBMS2950B_DEVICE_ID << 1u)};
    uint8_t status_payload[TX_DATA] = {0u, 0x40u, 0u, 0u, 0u, 0xA0u};
    uint8_t sample_payload[TX_DATA] = {0x39u, 0x30u, 0x00u, 0xFFu, 0x50u, 0x46u};
    uint8_t flag_payload[TX_DATA];
    float prior_current;
    int32_t prior_raw;

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms2950_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);
    dev.string = STRING_B;

    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_response[CMDSZ + PEC15SZ], sid, 2u, false);
    EXPECT_TRUE(adbms2950_spi_probe_sid(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.sid_valid);
    EXPECT_TRUE(dev.health.device_id == ADBMS2950B_DEVICE_ID);
    EXPECT_TRUE(memcmp(dev.health.sid, sid, RSID) == 0);

    unit_spi_reset();
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_response[CMDSZ + PEC15SZ], sid, 2u, true);
    EXPECT_TRUE(adbms2950_spi_probe_sid(&dev) == HAL_ERROR);
    EXPECT_FALSE(dev.health.sid_valid);
    EXPECT_TRUE(dev.health.pec_error_count == 1u);

    unit_spi_reset();
    dev.health.initialized = true;
    dev.health.i1_continuous_ready = true;
    dev.health.sid_valid = true;
    dev.health.config_valid = true;
    dev.health.hv_dividers_enabled = false;
    unit_adbms2950_make_flag_payload(flag_payload, 10u, 0u);
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], status_payload, 5u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], sample_payload, 5u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], flag_payload, 5u, false);
    adbms2950_resync_command_counter_tracking(&dev);
    EXPECT_TRUE(adbms2950_read_primary_sample(&dev, 1234u) == HAL_OK);
    EXPECT_TRUE(dev.health.sample_valid);
    EXPECT_TRUE(dev.health.current_valid);
    EXPECT_FALSE(dev.health.pack_voltage_valid);
    EXPECT_TRUE(dev.health.i1_calibrated);
    EXPECT_TRUE(dev.health.revision == 0x0Au);
    EXPECT_TRUE(dev.health.i1_raw == 12345);
    EXPECT_TRUE(dev.health.vb1_raw == 18000);
    EXPECT_NEAR(dev.health.current_a, 123.45f, 0.001f);
    EXPECT_NEAR(dev.health.pack_voltage_v,
                18000.0f * VBAT1_SCALE * VBAT_DIV_SCALE,
                0.01f);
    EXPECT_TRUE(dev.health.last_update_ms == 1234u);
    EXPECT_TRUE(dev.health.sample_count == 1u);
    EXPECT_TRUE(dev.health.i1_conversion_count == 10u);
    EXPECT_TRUE(dev.health.i1_conversion_phase == 0u);
    EXPECT_TRUE(dev.health.last_i1cntpha == 40u);
    prior_current = dev.health.current_a;
    prior_raw = dev.health.i1_raw;

    /* Without a new compatible ADI1 epoch, the same I1CNT/I1PHA value proves
     * this is a stale prior conversion even when all packets look plausible. */
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], status_payload, 5u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], sample_payload, 5u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], flag_payload, 5u, false);
    adbms2950_resync_command_counter_tracking(&dev);
    EXPECT_TRUE(adbms2950_read_primary_sample(&dev, 1500u) == HAL_ERROR);
    EXPECT_FALSE(dev.health.sample_valid);
    EXPECT_FALSE(dev.health.current_valid);
    EXPECT_FALSE(dev.health.counter_advanced);
    EXPECT_TRUE(dev.health.counter_stall_count == 1u);
    EXPECT_TRUE(dev.health.sample_error_count == 1u);
    EXPECT_TRUE(dev.health.last_update_ms == 1234u);
    EXPECT_TRUE(dev.health.current_a == prior_current);
    EXPECT_TRUE(dev.health.i1_raw == prior_raw);

    /* A successful compatible ADCV/ADI1 resets the conversion counter.  The
     * same numeric count is fresh in the new epoch if it is nonzero. */
    adbms2950_note_compatible_adi1(&dev);
    EXPECT_FALSE(dev.health.counter_seen);
    EXPECT_FALSE(dev.health.sample_valid);
    EXPECT_FALSE(dev.health.current_valid);
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], status_payload, 6u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], sample_payload, 6u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], flag_payload, 6u, false);
    adbms2950_resync_command_counter_tracking(&dev);
    EXPECT_TRUE(adbms2950_read_primary_sample(&dev, 1600u) == HAL_OK);
    EXPECT_TRUE(dev.health.counter_seen);
    EXPECT_TRUE(dev.health.counter_advanced);
    EXPECT_TRUE(dev.health.sample_count == 2u);

    /* A transport-success/DPEC-failure packet must not overwrite the prior
     * plausible sample. */
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 2u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], status_payload, 6u, false);
    sample_payload[0] = 0x01u;
    sample_payload[1] = 0x00u;
    sample_payload[2] = 0x00u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], sample_payload, 7u, true);
    adbms2950_resync_command_counter_tracking(&dev);
    EXPECT_TRUE(adbms2950_read_primary_sample(&dev, 2000u) == HAL_ERROR);
    EXPECT_FALSE(dev.health.sample_valid);
    EXPECT_FALSE(dev.health.current_valid);
    EXPECT_TRUE(dev.health.current_a == prior_current);
    EXPECT_TRUE(dev.health.i1_raw == prior_raw);
    EXPECT_TRUE(dev.health.last_update_ms == 1600u);
    EXPECT_TRUE(dev.health.sample_error_count == 2u);

    /* Both reads must describe the same command-counter epoch. */
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], status_payload, 7u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], sample_payload, 8u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], flag_payload, 8u, false);
    adbms2950_resync_command_counter_tracking(&dev);
    EXPECT_TRUE(adbms2950_read_primary_sample(&dev, 2500u) == HAL_ERROR);
    EXPECT_FALSE(dev.health.sample_valid);
    EXPECT_TRUE(dev.health.current_a == prior_current);
    EXPECT_TRUE(dev.health.i1_raw == prior_raw);
    EXPECT_TRUE(dev.health.counter_mismatch_count == 1u);
    EXPECT_TRUE(dev.health.sample_error_count == 3u);

    /* Uncalibrated status stops the operation before RDIVB1 is read. */
    unit_spi_reset();
    status_payload[1] = 0u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_response[CMDSZ + PEC15SZ], status_payload, 8u, false);
    adbms2950_resync_command_counter_tracking(&dev);
    EXPECT_TRUE(adbms2950_read_primary_sample(&dev, 3000u) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_txrx_calls == 1u);
    EXPECT_FALSE(dev.health.i1_calibrated);

    /* Reset/clear sentinel codes are invalid even with a correct PEC. */
    unit_spi_reset();
    status_payload[1] = 0x40u;
    sample_payload[0] = 0xFFu;
    sample_payload[1] = 0xFFu;
    sample_payload[2] = 0x03u;
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], status_payload, 8u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], sample_payload, 8u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], flag_payload, 8u, false);
    adbms2950_resync_command_counter_tracking(&dev);
    EXPECT_TRUE(adbms2950_read_primary_sample(&dev, 4000u) == HAL_ERROR);
    EXPECT_FALSE(dev.health.sample_valid);
    EXPECT_TRUE(dev.health.current_a == prior_current);
}

static void test_adbms2950_calibration_profiles_and_phase_freshness(void)
{
    adbms2950_driver_t dev;
    adbms2950_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    adbms2950_calibration_t custom;
    uint8_t status_payload[TX_DATA] = {0u, 0x40u, 0u, 0u, 0u, 0xA0u};
    uint8_t sample_payload[TX_DATA] = {0x39u, 0x30u, 0x00u, 0xFFu, 0x50u, 0x46u};
    uint8_t flag_payload[TX_DATA];

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms2950_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);
    dev.string = STRING_B;
    dev.write_string = STRING_B;

    EXPECT_TRUE(adbms2950_set_calibration_profile(
        &dev, ADBMS2950_CAL_PROFILE_DER_APM) == HAL_OK);
    EXPECT_NEAR(dev.calibration.shunt_resistance_ohm, 100.0e-6f, 1.0e-9f);
    EXPECT_NEAR(dev.calibration.vb1_divider_ratio,
                ADBMS2950_DER_VB1_DIVIDER_RATIO, 0.001f);
    EXPECT_TRUE(adbms2950_set_calibration_profile(
        &dev, ADBMS2950_CAL_PROFILE_EVAL_BASIC) == HAL_OK);
    EXPECT_NEAR(dev.calibration.shunt_resistance_ohm, 50.0e-6f, 1.0e-9f);
    EXPECT_NEAR(dev.calibration.vb1_divider_ratio,
                ADBMS2950_EVAL_VB1_DIVIDER_RATIO, 0.001f);
    EXPECT_TRUE(strcmp(adbms2950_calibration_profile_str(
        dev.calibration.profile), "EVAL_BASIC_50uR") == 0);

    custom = dev.calibration;
    custom.profile = ADBMS2950_CAL_PROFILE_CUSTOM;
    custom.shunt_resistance_ohm = 0.0f;
    EXPECT_TRUE(adbms2950_set_calibration(&dev, &custom) == HAL_ERROR);

    dev.health.initialized = true;
    dev.health.i1_continuous_ready = true;
    dev.health.counter_seen = false;
    unit_adbms2950_make_flag_payload(flag_payload, 10u, 0u);
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], status_payload, 4u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], sample_payload, 4u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], flag_payload, 4u, false);
    EXPECT_TRUE(adbms2950_read_primary_sample(&dev, 100u) == HAL_OK);
    EXPECT_NEAR(dev.health.current_a, 246.90f, 0.01f);
    EXPECT_NEAR(dev.health.pack_voltage_v,
                18000.0f * VBAT1_SCALE * ADBMS2950_EVAL_VB1_DIVIDER_RATIO,
                0.02f);

    /* I1PHA can advance four times inside one 1 ms conversion.  A phase-only
     * change is not a new published I1 result and must remain stale. */
    unit_spi_reset();
    unit_adbms2950_make_flag_payload(flag_payload, 10u, 1u);
    unit_spi_txrx_sequence_count = 3u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], status_payload, 7u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], sample_payload, 7u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], flag_payload, 7u, false);
    EXPECT_TRUE(adbms2950_read_primary_sample(&dev, 101u) == HAL_ERROR);
    EXPECT_FALSE(dev.health.counter_advanced);
    EXPECT_TRUE(dev.health.counter_stall_count == 1u);
}

static void test_adbms2950_masked_config_readback(void)
{
    adbms2950_driver_t dev;
    adbms2950_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t cfga[TX_DATA];
    uint8_t cfgb[TX_DATA];

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms2950_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);
    dev.string = STRING_B;
    dev.write_string = STRING_B;
    adbms2950_reset_cfg_regs(&dev);
    adbms2950_pack_cfga(&dev);
    adbms2950_pack_cfgb(&dev);
    memcpy(cfga, ics[0].configa.tx_data, TX_DATA);
    memcpy(cfgb, ics[0].configb.tx_data, TX_DATA);

    /* Live SNAPST/REFUP and reserved bits can differ without changing the
     * writable configuration. */
    cfga[1] |= 0x20u;
    cfga[3] |= 0x80u;
    cfga[5] |= 0x30u;
    cfgb[0] |= 0x80u;
    cfgb[1] |= 0x80u;
    cfgb[2] |= 0x80u;
    cfgb[3] |= 0xF4u;
    unit_spi_txrx_sequence_count = 2u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], cfga, 3u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], cfgb, 3u, false);
    EXPECT_TRUE(adbms2950_verify_config_readback(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.configa_mismatch_ic_mask == 0u);
    EXPECT_TRUE(dev.health.configb_mismatch_ic_mask == 0u);

    unit_spi_reset();
    cfga[0] ^= 0x01u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_response[CMDSZ + PEC15SZ], cfga, 3u, false);
    EXPECT_TRUE(adbms2950_verify_config_readback(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.health.configa_mismatch_ic_mask == 0x0001u);
}

static void test_adbms2950_comm_ack_release_and_eeprom_probe(void)
{
    adbms2950_driver_t dev;
    adbms2950_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    adbms2950_i2c_probe_result_t result;
    uint8_t pre[TX_DATA] = {0x68u, 0xA0u, 0x08u, 0x00u, 0x19u, 0xFFu};
    uint8_t post[TX_DATA] = {0x67u, 0xA0u, 0x77u, 0x00u, 0x1Fu, 0xFFu};

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms2950_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);
    dev.string = STRING_B;
    dev.write_string = STRING_B;
    unit_spi_txrx_sequence_count = 2u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], pre, 6u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], post, 7u, false);

    EXPECT_TRUE(adbms2950_i2c_write_probe(&dev, 0x50u, 0x00u, &result) == HAL_OK);
    EXPECT_TRUE(result.transport_status == HAL_OK);
    EXPECT_TRUE(result.address_ack);
    EXPECT_TRUE(result.data_ack);
    EXPECT_TRUE(memcmp(result.pre_rdcomm, pre, TX_DATA) == 0);
    EXPECT_TRUE(memcmp(result.post_rdcomm, post, TX_DATA) == 0);
    EXPECT_TRUE(unit_spi_tx_calls == 2u); /* WRCOMM + dynamically built STCOMM. */
    EXPECT_TRUE(unit_spi_txrx_calls == 2u);
    EXPECT_TRUE(unit_spi_last_tx_len == 13u);
    EXPECT_TRUE(unit_spi_last_tx[0] == 0x07u);
    EXPECT_TRUE(unit_spi_last_tx[1] == 0x23u);

    /* Address ACK with data NACK is reported separately and cannot pass. */
    unit_spi_reset();
    post[2] = 0x7Fu;
    unit_spi_txrx_sequence_count = 2u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], pre, 8u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], post, 9u, false);
    EXPECT_TRUE(adbms2950_i2c_write_probe(&dev, 0x50u, 0x00u, &result) == HAL_ERROR);
    EXPECT_TRUE(result.transport_status == HAL_OK);
    EXPECT_TRUE(result.address_ack);
    EXPECT_FALSE(result.data_ack);
    EXPECT_TRUE(dev.health.last_stage == ADBMS2950_STAGE_EEPROM);
    EXPECT_TRUE(dev.health.last_reason == ADBMS2950_REASON_PERIPHERAL_NACK);
}

static void test_adbms2950_redundant_diagnostic_and_cs_timing(void)
{
    adbms2950_driver_t dev;
    adbms2950_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t status_cal[TX_DATA] = {0u, 0xC0u, 0u, 0u, 0u, 0x10u};
    uint8_t rdi[TX_DATA] = {0xE8u, 0x03u, 0x00u, 0x18u, 0xFCu, 0xFFu};
    uint8_t rdvb[TX_DATA] = {0u, 0u, 0x10u, 0x27u, 0x0Bu, 0xD2u};
    uint8_t flag[TX_DATA];
    uint8_t tx[2] = {0xAAu, 0x55u};

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms2950_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);
    dev.string = STRING_B;
    dev.write_string = STRING_B;
    EXPECT_TRUE(adbms2950_set_calibration_profile(
        &dev, ADBMS2950_CAL_PROFILE_DER_APM) == HAL_OK);
    dev.health.initialized = true;
    dev.health.i1_continuous_ready = true;
    dev.health.i1_calibrated = true;
    unit_adbms2950_make_flag_payload(flag, 20u, 0u);
    unit_spi_txrx_sequence_count = 5u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], status_cal, 3u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], status_cal, 5u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], rdi, 5u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[3][CMDSZ + PEC15SZ], rdvb, 5u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[4][CMDSZ + PEC15SZ], flag, 5u, false);
    EXPECT_TRUE(adbms2950_read_redundant_sample(&dev, 777u) == HAL_OK);
    EXPECT_TRUE(dev.redundant_sample.valid);
    EXPECT_NEAR(dev.redundant_sample.current1_a, 10.0f, 0.001f);
    EXPECT_NEAR(dev.redundant_sample.current2_a, 10.0f, 0.001f);
    EXPECT_NEAR(dev.redundant_sample.current_disagreement_a, 0.0f, 0.001f);
    EXPECT_NEAR(dev.redundant_sample.pack_voltage_disagreement_v, 0.0f, 0.05f);
    EXPECT_FALSE(dev.health.i2_continuous_ready);
    EXPECT_FALSE(dev.health.counter_seen);

    /* A failed restore leaves the normal primary mode unproven and must
     * withdraw both the redundant result and I1 continuous readiness. */
    unit_spi_reset();
    dev.health.initialized = true;
    dev.health.i1_continuous_ready = true;
    dev.health.i1_calibrated = true;
    unit_spi_tx_status_sequence_count = 5u;
    unit_spi_tx_status_sequence[0] = HAL_OK;
    unit_spi_tx_status_sequence[1] = HAL_OK;
    unit_spi_tx_status_sequence[2] = HAL_OK;
    unit_spi_tx_status_sequence[3] = HAL_OK;
    unit_spi_tx_status_sequence[4] = HAL_TIMEOUT;
    unit_spi_txrx_sequence_count = 5u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], status_cal, 8u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], status_cal, 10u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], rdi, 10u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[3][CMDSZ + PEC15SZ], rdvb, 10u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[4][CMDSZ + PEC15SZ], flag, 10u, false);
    EXPECT_TRUE(adbms2950_read_redundant_sample(&dev, 778u) == HAL_TIMEOUT);
    EXPECT_FALSE(dev.redundant_sample.valid);
    EXPECT_FALSE(dev.health.i1_continuous_ready);
    EXPECT_FALSE(dev.health.i2_continuous_ready);

    unit_spi_reset();
    dev.htim = NULL;
    EXPECT_TRUE(adbms2950_spi_write(&dev, tx, sizeof(tx), 1u) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(unit_gpio_states[6u] == GPIO_PIN_SET);
}


static void test_adbms2950_refup_snapshot_flags_and_counter_reset(void)
{
    adbms2950_driver_t dev;
    adbms2950_asic ics[1];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    adbms2950_core_snapshot_t snapshot;
    uint8_t cfga[TX_DATA] = {0u, 0u, 0u, 0u, 0u, 0x10u};
    uint8_t cfgb[TX_DATA] = {0u};
    uint8_t stat[TX_DATA] = {0u, 0xC0u, 0u, 0u, 0u, 0x10u};
    uint8_t flag[TX_DATA] = {0u};
    uint8_t sid[TX_DATA] = {1u, 2u, 3u, 4u, 5u,
                            (uint8_t)(ADBMS2950B_DEVICE_ID << 1u)};

    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms2950_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);
    dev.string = STRING_B;
    dev.write_string = STRING_B;

    /* REFUP is a required readiness gate before ADC commands. */
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_response[CMDSZ + PEC15SZ], cfga, 2u, false);
    EXPECT_TRUE(adbms2950_verify_refup(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.refup_valid);
    EXPECT_TRUE(dev.health.refup);

    /* FLAG decoding keeps raw bytes and exposes reset/fault latches. */
    unit_spi_reset();
    flag[5] = 0x08u; /* RESET latch. */
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_response[CMDSZ + PEC15SZ], flag, 2u, false);
    EXPECT_TRUE(adbms2950_read_flag(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.flag_valid);
    EXPECT_TRUE((dev.health.fault_mask & (1u << 23)) != 0u);

    /* All core groups in one debug snapshot must report one CCNT epoch. */
    unit_spi_reset();
    unit_spi_txrx_sequence_count = 5u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], cfga, 9u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], cfgb, 9u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], stat, 9u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[3][CMDSZ + PEC15SZ], flag, 9u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[4][CMDSZ + PEC15SZ], sid, 9u, false);
    adbms2950_resync_command_counter_tracking(&dev);
    EXPECT_TRUE(adbms2950_read_core_snapshot(&dev, &snapshot) == HAL_OK);
    EXPECT_TRUE(snapshot.valid);
    EXPECT_TRUE(snapshot.cfga_ccnt == 9u);
    EXPECT_TRUE(snapshot.sid_ccnt == 9u);

    /* A zero CCNT after an established nonzero epoch is an unexpected reset. */
    unit_spi_reset();
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_response[CMDSZ + PEC15SZ], sid, 0u, false);
    EXPECT_TRUE(adbms2950_spi_probe_sid(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.health.unexpected_counter_reset_count == 1u);
    EXPECT_TRUE(dev.spi_debug.sticky_unexpected_counter_reset_mask == 0x0001u);
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
#if defined(AMS_HOST_ONLY_ADBMS2950_HARDENING_TEST)
    run_test("ADBMS2950 calibration/phase freshness", test_adbms2950_calibration_profiles_and_phase_freshness);
    run_test("ADBMS2950 masked config readback", test_adbms2950_masked_config_readback);
    run_test("ADBMS2950 COMM ACK/EEPROM probe", test_adbms2950_comm_ack_release_and_eeprom_probe);
    run_test("ADBMS2950 redundant diagnostic/CS timing", test_adbms2950_redundant_diagnostic_and_cs_timing);
    run_test("ADBMS2950 REFUP/snapshot/flags/CCNT reset", test_adbms2950_refup_snapshot_flags_and_counter_reset);
#else
    run_test("lut basic ranges", test_lut_basic_ranges);
    run_test("lut clamp behavior", test_lut_clamps_out_of_range_queries);
    run_test("pack config and init", test_pack_config_and_init);
    run_test("segment config layout", test_segment_config_layout);
    run_test("even split configuration", test_even_split_configuration);
    run_test("bad config rejection", test_bad_config_rejected);
    run_test("R0 initialization clamp", test_r0_initialization_clamp);
    run_test("R0 observability/accounting", test_r0_observability_and_accounting);
    run_test("fixed-basis acquisition/consensus", test_fixed_basis_acquisition_and_consensus);
    run_test("dynamic acquisition no false rest anchor", test_dynamic_acquisition_without_false_rest_anchor);
    run_test("full covariance measurement update", test_full_covariance_measurement_update);
    run_test("covariance temperature floors/topology R", test_covariance_temperature_floors_and_topology_r);
    run_test("SoH invalid until acquisition complete", test_soh_advisory_invalid_until_acquisition_complete);
    run_test("single-step nominal pack", test_single_step_nominal_pack);
    run_test("EKF innovation gate/dt observability", test_ekf_innovation_gate_and_dt_observability);
    run_test("invalid step inputs", test_invalid_step_inputs);
    run_test("200-step numerical stability", test_200_step_numerical_stability);
    run_test("estimator summary aggregation", test_estimator_summary_aggregation);
    run_test("estimator status flags", test_estimator_status_flags);
    run_test("coulomb count baseline", test_coulomb_count_baseline);
    run_test("voltage fault thresholds/latch", test_voltage_fault_thresholds_latch_and_reset);
    run_test("voltage fault read failure/strings", test_voltage_fault_read_failure_precedence_and_strings);
    run_test("voltage hardware Status-D crosscheck", test_voltage_fault_hardware_status_crosscheck);
    run_test("parallel connection observer advisory", test_parallel_connection_observer_advisory);
    run_test("main fuse plausibility/controlled clear", test_main_fuse_monitor_plausibility_and_clear);
    run_test("ADBMS topology/delay guards", test_adbms_topology_and_delay_guards);
    run_test("ADBMS v0.5 session/filter/MUTE contract", test_adbms_v05_session_filter_and_mute_contract);
    run_test("ADBMS v0.5 products/AUX2/POST diagnostics", test_adbms_v05_products_aux2_post_and_diagnostic_freshness);
    run_test("ADBMS SPI debug write/full-duplex", test_adbms_spi_debug_write_and_full_duplex_paths);
    run_test("ADBMS SPI rd48 PEC masks", test_adbms_spi_debug_rd48_pec_masks_and_clear);
    run_test("ADBMS SPI scope activity", test_adbms_spi_scope_activity);
    run_test("ADBMS SPI SID/status/counter diagnostics", test_adbms_spi_sid_status_and_counter_mismatch);
    run_test("ADBMS startup/reference/full status safety policy", test_adbms_startup_reference_and_full_status_policy);
    run_test("ADBMS command counter rejects stale data", test_adbms_counter_mismatch_rejects_stale_data);
    run_test("ADBMS cell ADC conversion diagnostic", test_adbms_cell_adc_conversion_diagnostic);
    run_test("ADBMS SPI cold wake and clear flags", test_adbms_spi_coldwake_and_clear_flags);
    run_test("ADBMS PWM write packing", test_adbms_pwm_write_packing);
    run_test("ADBMS final-ring subset write owner", test_adbms6830_final_ring_subset_write_owner);
    run_test("ADBMS CFGB timer write guard", test_adbms6830_cfgb_timer_write_guard);
    run_test("ADBMS config/balance readback", test_adbms_config_and_balance_readback);
    run_test("ADBMS I2C COMM releases SDA for slave ACK", test_adbms_i2c_comm_releases_sda_for_slave_ack);
    run_test("ADBMS mux ACK/temperature freshness", test_adbms_mux_ack_and_temperature_freshness);
    run_test("ADBMS mux transport/ACK failures", test_adbms_mux_transport_and_ack_failures);
    run_test("ADBMS full open-wire measurement/fault injection", test_adbms_open_wire_full_measurement_and_fault_injection);
	run_test("ADBMS2950 bounded write PEC", test_adbms2950_pec_write_is_bounded_and_reference_equal);
    run_test("ADBMS2950 SPI write/full-duplex", test_adbms2950_spi_debug_write_and_full_duplex_paths);
    run_test("ADBMS2950 final-ring subset write owner", test_adbms2950_final_ring_subset_write_owner);
    run_test("ADBMS2950 SPI probe PEC masks", test_adbms2950_spi_probe_pec_masks_and_clear);
    run_test("ADBMS2950 mixed-chain init/identity/readback", test_adbms2950_mixed_chain_init_identity_and_readback);
    run_test("ADBMS2950 SID/sample integrity", test_adbms2950_sid_probe_and_primary_sample_integrity);
    run_test("ADBMS2950 calibration/phase freshness", test_adbms2950_calibration_profiles_and_phase_freshness);
    run_test("ADBMS2950 masked config readback", test_adbms2950_masked_config_readback);
    run_test("ADBMS2950 COMM ACK/EEPROM probe", test_adbms2950_comm_ack_release_and_eeprom_probe);
    run_test("ADBMS2950 redundant diagnostic/CS timing", test_adbms2950_redundant_diagnostic_and_cs_timing);
    run_test("current sensor conversion/range", test_current_sensor_conversion_zero_and_range_selection);
    run_test("current sensor invalid conditions", test_current_sensor_invalid_conditions);
    run_test("current sensor fresh pair/channel mapping", test_current_sensor_requires_fresh_pair_and_channel_mapping);
    run_test("current sensor zero calibration/hysteresis", test_current_sensor_zero_cal_and_hysteresis);
    run_test("current sensor calibration record integrity", test_current_sensor_calibration_record_integrity);
    run_test("current sensor ADC status path", test_current_sensor_read_adc_status_path);
    run_test("current fault policy", test_current_fault_policy);
    run_test("current fault threshold edges/recovery", test_current_fault_threshold_edges_and_recovery);
#endif

    if (g_failures != 0)
    {
        printf("AMS UNIT TESTS FAILED: %d failure(s)\n", g_failures);
        return 1;
    }

    printf("ALL AMS UNIT TESTS PASSED\n");
    return 0;
}
