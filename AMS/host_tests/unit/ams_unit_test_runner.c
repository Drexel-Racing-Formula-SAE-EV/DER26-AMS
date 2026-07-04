
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
static uint8_t unit_spi_txrx_sequence[4][BUFSZ];
static uint16_t unit_spi_last_tx_len = 0u;
static uint16_t unit_spi_last_txrx_len = 0u;
static uint32_t unit_spi_tx_calls = 0u;
static uint32_t unit_spi_txrx_calls = 0u;
static uint32_t unit_spi_txrx_sequence_count = 0u;
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
    unit_gpio_write_calls = 0u;
    unit_spi_tx_status = HAL_OK;
    unit_spi_txrx_status = HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)Timeout;
    if((hspi == NULL) || (pData == NULL) || (Size > BUFSZ))
    {
        return HAL_ERROR;
    }

    memcpy(unit_spi_last_tx, pData, Size);
    unit_spi_last_tx_len = Size;
    unit_spi_tx_calls++;
    return unit_spi_tx_status;
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
    EXPECT_TRUE(vf.voltage_valid);
    EXPECT_TRUE(vf.warning);
    EXPECT_FALSE(vf.confirmed);
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

static void unit_adbms_init_driver(adbms6830_driver_t *dev,
                                   adbms6830_asic *ics,
                                   SPI_HandleTypeDef *spi,
                                   GPIO_TypeDef *gpio_a,
                                   GPIO_TypeDef *gpio_b,
                                   uint8_t num_ics)
{
    adBms6830_init(dev, num_ics, ics, spi, gpio_a, gpio_b, 3u, 4u, NULL);
    unit_spi_reset();
    dev->string = STRING_B;
    adbms6830_spi_debug_clear(dev);
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
    EXPECT_TRUE(unit_gpio_states[4u] == GPIO_PIN_SET);
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

static void test_adbms_spi_sid_status_and_counter_mismatch(void)
{
    adbms6830_driver_t dev;
    adbms6830_asic ics[2];
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio_a;
    GPIO_TypeDef gpio_b;
    uint8_t rx[RX_DATA * 2u];
    const uint8_t sid0[TX_DATA] = {0x10u, 0x11u, 0x12u, 0x13u, 0x14u, 0x15u};
    const uint8_t sid1[TX_DATA] = {0x20u, 0x21u, 0x22u, 0x23u, 0x24u, 0x25u};
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
    EXPECT_TRUE(unit_gpio_states[4u] == GPIO_PIN_SET);

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


static void unit_adbms2950_init_driver(adbms2950_driver_t *dev,
                                       adbms2950_asic *ics,
                                       SPI_HandleTypeDef *spi,
                                       GPIO_TypeDef *gpio_a,
                                       GPIO_TypeDef *gpio_b,
                                       uint8_t num_ics)
{
    memset(dev, 0, sizeof(*dev));
    memset(ics, 0, sizeof(adbms2950_asic) * num_ics);
    dev->num_ics = num_ics;
    dev->ics = ics;
    dev->hspi = spi;
    dev->cs_port[0] = gpio_a;
    dev->cs_port[1] = gpio_b;
    dev->cs_pin[0] = 5u;
    dev->cs_pin[1] = 6u;
    dev->string = STRING_A;
    adbms2950_spi_debug_clear(dev);
    adbms2950_spi_debug_enable(dev, true);
    unit_spi_reset();
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
    EXPECT_TRUE(adbms2950_spi_probe_rdcfga(&dev) == HAL_OK);
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
    run_test("voltage fault thresholds/latch", test_voltage_fault_thresholds_latch_and_reset);
    run_test("voltage fault read failure/strings", test_voltage_fault_read_failure_precedence_and_strings);
    run_test("ADBMS SPI debug write/full-duplex", test_adbms_spi_debug_write_and_full_duplex_paths);
    run_test("ADBMS SPI rd48 PEC masks", test_adbms_spi_debug_rd48_pec_masks_and_clear);
    run_test("ADBMS SPI SID/status/counter diagnostics", test_adbms_spi_sid_status_and_counter_mismatch);
    run_test("ADBMS SPI cold wake and clear flags", test_adbms_spi_coldwake_and_clear_flags);
    run_test("ADBMS2950 SPI write/full-duplex", test_adbms2950_spi_debug_write_and_full_duplex_paths);
    run_test("ADBMS2950 SPI probe PEC masks", test_adbms2950_spi_probe_pec_masks_and_clear);
    run_test("current sensor conversion/range", test_current_sensor_conversion_zero_and_range_selection);
    run_test("current sensor invalid conditions", test_current_sensor_invalid_conditions);
    run_test("current sensor fresh pair/channel mapping", test_current_sensor_requires_fresh_pair_and_channel_mapping);
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
