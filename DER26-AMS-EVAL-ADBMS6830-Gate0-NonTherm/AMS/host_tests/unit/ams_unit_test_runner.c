
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

    ekf.step_count = 1u;
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
    ekf.step_count = 1u;
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
    EXPECT_TRUE(soh.observation_maturity_pct == 0u);

    ams_ekf_init(&ekf, &cfg);
    ekf.step_count = 1u;
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
    EXPECT_TRUE(soh.observation_maturity_pct == 100u);
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
        (uint8_t)((ICOMM_BLANK_ << 4u) | (data_ack ? 0x07u : 0x0Fu)),
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
                        &unit_delay_timer,
                        ADBMS6830_INIT_FULL_CONFIG) == HAL_OK);
    (void)adbms6830_set_monitored_cell_count(dev, 15u);
    unit_spi_reset();
    dev->string = STRING_A;
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
                         &unit_delay_timer,
                         ADBMS6830_INIT_FULL_CONFIG) == HAL_ERROR);
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
                         &unit_delay_timer,
                         ADBMS6830_INIT_FULL_CONFIG) == HAL_ERROR);
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
                              &unit_delay_timer,
                              ADBMS6830_INIT_FULL_CONFIG) == HAL_ERROR);
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
                              &unit_delay_timer,
                              ADBMS6830_INIT_FULL_CONFIG) == HAL_TIMEOUT);
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
                                  &unit_delay_timer,
                                  ADBMS6830_INIT_FULL_CONFIG) == HAL_ERROR);
        EXPECT_TRUE(unit_spi_tx_calls == 1u);
        EXPECT_TRUE(unit_spi_txrx_calls == 1u);
        EXPECT_TRUE(dev.health.sid_valid_ic_mask == 0u);
        EXPECT_TRUE(dev.health.sid_identity_mismatch_ic_mask == 0x0003u);
        EXPECT_TRUE(dev.health.sticky_sid_identity_mismatch_ic_mask == 0x0003u);
    }

    /* The eval fixture uses the same checked reset/SID path, but it must stop
     * before CFGA/CFGB writes and remain on String B. */
    {
        const uint8_t valid_sid[TX_DATA] =
        {
            0xA5u, (uint8_t)(ADBMS6830B_DEVICE_ID << 1u),
            0x12u, 0x34u, 0x56u, 0x78u
        };
        unit_spi_reset();
        unit_adbms_make_read_packet_from_data(
            &unit_spi_txrx_response[CMDSZ + PEC15SZ],
            valid_sid,
            0u,
            false);
        EXPECT_TRUE(adBms6830_init(&dev,
                                  1u,
                                  1u,
                                  ics,
                                  1u,
                                  &spi,
                                  &gpio_a,
                                  &gpio_b,
                                  3u,
                                  4u,
                                  &unit_delay_timer,
                                  ADBMS6830_INIT_MONITOR_ONLY) == HAL_OK);
        EXPECT_TRUE(dev.string == STRING_B);
        EXPECT_TRUE(dev.write_string == STRING_B);
        EXPECT_TRUE(dev.num_ics == 1u);
        EXPECT_TRUE(dev.physical_chain_count == 1u);
        EXPECT_TRUE(dev.health.sid_valid_ic_mask == 0x0001u);
        EXPECT_TRUE(dev.health.sid_identity_mismatch_ic_mask == 0u);
        EXPECT_TRUE(unit_spi_tx_calls == 1u);   /* SRST only. */
        EXPECT_TRUE(unit_spi_txrx_calls == 1u); /* Checked RDSID only. */
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
    EXPECT_TRUE(unit_spi_txrx_calls == 6u);
    EXPECT_TRUE(dev.last_cell_updated_mask[0] == 0u);
    EXPECT_TRUE(dev.last_cell_updated_mask[1] == 0u);
    EXPECT_TRUE(dev.last_cell_pec_mask[0] == 0u);
    EXPECT_TRUE(dev.last_cell_pec_mask[1] == 0u);
    unit_spi_txrx_status = HAL_OK;

    dev.htim = NULL;
    EXPECT_TRUE(adbms6830_us_delay(&dev, 10u) == HAL_ERROR);
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
    EXPECT_TRUE(unit_gpio_states[3u] == GPIO_PIN_SET);
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
    EXPECT_TRUE(adbms6830_refresh_diagnostics(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.health.cell_ovuv_fault_ic_mask == 0x0001u);

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

    /* The same rule applies to voltage groups. Reject only the mismatched
     * group, retain its previous values, and make the complete scan fail. */
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    ics[0].cell.c_codes[3] = 1111;
    ics[0].cell.c_codes[4] = 2222;
    ics[0].cell.c_codes[5] = 3333;
    unit_spi_txrx_sequence_count = 6u;
    for(uint8_t group = 0u; group < 6u; group++)
    {
        uint8_t counter = (group == 0u) ? 5u : 6u;
        unit_adbms_make_valid_read_packet(
            &unit_spi_txrx_sequence[group][CMDSZ + PEC15SZ],
            (uint8_t)(0x10u + (group * 0x10u)),
            counter,
            false);
    }

    EXPECT_TRUE(adbms6830_read_cell_voltages(&dev) == HAL_ERROR);
    EXPECT_TRUE(dev.last_cell_updated_mask[0] == 0xFFC7u);
    EXPECT_TRUE(dev.last_cell_pec_mask[0] == 0u);
    EXPECT_TRUE(ics[0].cell.c_codes[3] == 1111);
    EXPECT_TRUE(ics[0].cell.c_codes[4] == 2222);
    EXPECT_TRUE(ics[0].cell.c_codes[5] == 3333);
    EXPECT_TRUE(dev.health.sticky_cmd_counter_mismatch_mask == 0x0001u);
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
    uint8_t statd[TX_DATA] = {0u};
    uint8_t state[TX_DATA] = {0u};
    const uint8_t *status_payloads[5] = {stata, statb, statc, statd, state};

    memset(&dev, 0, sizeof(dev));
    memset(ics, 0, sizeof(ics));
    memset(&spi, 0, sizeof(spi));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    unit_adbms_init_driver(&dev, ics, &spi, &gpio_a, &gpio_b, 1u);

    /* Six cell groups establish command counter 10.  UNSNAP and the
     * subsequent ADAX advance it twice, so the fresh Status A-E image must
     * report counter 12. */
    unit_spi_txrx_sequence_count = 11u;
    for(uint8_t group = 0u; group < 6u; group++)
    {
        unit_adbms_make_valid_read_packet(
            &unit_spi_txrx_sequence[group][CMDSZ + PEC15SZ],
            (uint8_t)(0x10u + (group * 0x10u)),
            10u,
            false);
    }

    unit_adbms_put_s16(stata, 0u, 10000); /* VREF2 = 3.000 V */
    unit_adbms_put_s16(stata, 1u, 4900);  /* ITMP = 25.0 C */
    unit_adbms_put_s16(statb, 0u, 12000); /* VD = 3.300 V */
    unit_adbms_put_s16(statb, 1u, 23333); /* VA ~= 5.000 V */
    unit_adbms_put_s16(statb, 2u, 10000); /* VRES = 3.000 V */
    statd[5] = 60u;
    for(uint8_t group = 0u; group < 5u; group++)
    {
        unit_adbms_make_read_packet_from_data(
            &unit_spi_txrx_sequence[6u + group][CMDSZ + PEC15SZ],
            status_payloads[group],
            12u,
            false);
    }

    EXPECT_TRUE(adbms6830_run_cell_adc_self_test(&dev) == HAL_OK);
    EXPECT_TRUE(dev.health.cell_adc_self_test_count == 1u);
    EXPECT_TRUE((dev.last_cell_updated_mask[0] & 0x7FFFu) == 0x7FFFu);
    EXPECT_TRUE(dev.health.status_fault_ic_mask == 0u);
    EXPECT_TRUE(unit_spi_txrx_calls == 11u);

    /* A plausible stale payload with a bad data PEC must fail the diagnostic
     * before Status A-E is consulted. */
    unit_spi_reset();
    adbms6830_spi_debug_clear(&dev);
    unit_spi_txrx_sequence_count = 6u;
    for(uint8_t group = 0u; group < 6u; group++)
    {
        unit_adbms_make_valid_read_packet(
            &unit_spi_txrx_sequence[group][CMDSZ + PEC15SZ],
            (uint8_t)(0x20u + (group * 0x10u)),
            20u,
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
    EXPECT_TRUE(unit_gpio_states[3u] == GPIO_PIN_SET);

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

    /* The same five packets from String B would target the APM and four SMBs. */
    dev.string = STRING_B;
    unit_spi_reset();
    EXPECT_TRUE(adbms6830_wrcfgb_checked(&dev) == HAL_ERROR);
    EXPECT_TRUE(unit_spi_tx_calls == 0u);
    EXPECT_TRUE(unit_spi_txrx_calls == 0u);
    EXPECT_TRUE(unit_gpio_write_calls == 0u);
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

    unit_spi_txrx_sequence_count = 7u;
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[0][CMDSZ + PEC15SZ], sid, 1u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[1][CMDSZ + PEC15SZ], cfga, 3u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[2][CMDSZ + PEC15SZ], cfgb, 3u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[3][CMDSZ + PEC15SZ], status_payload, 4u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[4][CMDSZ + PEC15SZ],
        initialization_flag_payload,
        4u,
        false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[5][CMDSZ + PEC15SZ], status_payload, 5u, false);
    unit_adbms_make_read_packet_from_data(
        &unit_spi_txrx_sequence[6][CMDSZ + PEC15SZ],
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
    EXPECT_TRUE(unit_spi_txrx_calls == 7u); /* SID, configs, then two STAT/FLAG pairs. */
    EXPECT_TRUE(dev.ics[0].tx_cfga.gpo1c == GPO_CLR);
    EXPECT_TRUE(dev.ics[0].tx_cfga.gpo2c == GPO_CLR);

    /* If the requested configuration cannot be written, initialization
     * remains failed and a second bounded write forces both divider enables
     * low.  A cleanup success must never hide the original error. */
    unit_spi_reset();
    unit_spi_tx_status_sequence_count = 2u;
    unit_spi_tx_status_sequence[0] = HAL_TIMEOUT;
    unit_spi_tx_status_sequence[1] = HAL_OK;
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
    EXPECT_TRUE(adbms2950_read_primary_sample(&dev, 4000u) == HAL_ERROR);
    EXPECT_FALSE(dev.health.sample_valid);
    EXPECT_TRUE(dev.health.current_a == prior_current);
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
    run_test("R0 observability/accounting", test_r0_observability_and_accounting);
    run_test("single-step nominal pack", test_single_step_nominal_pack);
    run_test("invalid step inputs", test_invalid_step_inputs);
    run_test("200-step numerical stability", test_200_step_numerical_stability);
    run_test("estimator summary aggregation", test_estimator_summary_aggregation);
    run_test("estimator status flags", test_estimator_status_flags);
    run_test("coulomb count baseline", test_coulomb_count_baseline);
    run_test("voltage fault thresholds/latch", test_voltage_fault_thresholds_latch_and_reset);
    run_test("voltage fault read failure/strings", test_voltage_fault_read_failure_precedence_and_strings);
    run_test("ADBMS topology/delay guards", test_adbms_topology_and_delay_guards);
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
    run_test("ADBMS config/balance readback", test_adbms_config_and_balance_readback);
    run_test("ADBMS mux ACK/temperature freshness", test_adbms_mux_ack_and_temperature_freshness);
    run_test("ADBMS mux transport/ACK failures", test_adbms_mux_transport_and_ack_failures);
    run_test("ADBMS full open-wire measurement/fault injection", test_adbms_open_wire_full_measurement_and_fault_injection);
	run_test("ADBMS2950 bounded write PEC", test_adbms2950_pec_write_is_bounded_and_reference_equal);
    run_test("ADBMS2950 SPI write/full-duplex", test_adbms2950_spi_debug_write_and_full_duplex_paths);
    run_test("ADBMS2950 final-ring subset write owner", test_adbms2950_final_ring_subset_write_owner);
    run_test("ADBMS2950 SPI probe PEC masks", test_adbms2950_spi_probe_pec_masks_and_clear);
    run_test("ADBMS2950 mixed-chain init/identity/readback", test_adbms2950_mixed_chain_init_identity_and_readback);
    run_test("ADBMS2950 SID/sample integrity", test_adbms2950_sid_probe_and_primary_sample_integrity);
    run_test("current sensor conversion/range", test_current_sensor_conversion_zero_and_range_selection);
    run_test("current sensor invalid conditions", test_current_sensor_invalid_conditions);
    run_test("current sensor fresh pair/channel mapping", test_current_sensor_requires_fresh_pair_and_channel_mapping);
    run_test("current sensor zero calibration/hysteresis", test_current_sensor_zero_cal_and_hysteresis);
    run_test("current sensor calibration record integrity", test_current_sensor_calibration_record_integrity);
    run_test("current sensor ADC status path", test_current_sensor_read_adc_status_path);
    run_test("current fault policy", test_current_fault_policy);
    run_test("current fault threshold edges/recovery", test_current_fault_threshold_edges_and_recovery);

    if (g_failures != 0)
    {
        printf("AMS UNIT TESTS FAILED: %d failure(s)\n", g_failures);
        return 1;
    }

    printf("ALL AMS UNIT TESTS PASSED\n");
    return 0;
}
