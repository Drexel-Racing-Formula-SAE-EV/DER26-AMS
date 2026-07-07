/*
 * main.c — ESP32 75s6p P42A accumulator plant node
 *
 * This keeps the existing working ESP32 architecture:
 *   - FreeRTOS plant task at 100 ms
 *   - MCP2515 CAN TX task at 100 ms
 *   - reset command on CAN ID 0x300
 *   - shared plant struct protected by plant_mutex
 *
 * Only the plant source changed:
 *   old: frozen DFN-aged replay arrays
 *   new: generated Simulink electrothermal 75s6p P42A accumulator model
 *
 * CAN frame layout, Classic CAN 2.0:
 *
 *   0x200 — MEAS frame
 *     [0:1]  V_pack  uint16  10 mV    (0–655.35 V, supports 75s pack)
 *     [2:3]  I_pack  int16   10 mA    (+discharge)
 *     [4:5]  T_surf  int16   0.01 C
 *     [6]    counter uint8
 *     [7]    reserved 0x00
 *
 *   0x201 — TRUTH frame
 *     [0:1]  SoC_true uint16  0.01%   (0–10000 => 0.00–100.00%)
 *     [2:3]  T_core   int16   0.01 C
 *     [4]    counter  uint8
 *     [5:7]  step24   uint24  plant step counter, big-endian
 *
 *   0x202 — AMS summary frame
 *     [0:1]  V_min    uint16  1 mV     min group voltage
 *     [2:3]  V_max    uint16  1 mV     max group voltage
 *     [4:5]  T_max    int16   0.01 C
 *     [6:7]  T_avg    int16   0.01 C
 *
 *   0x210 — ADBMS replacement cell triplet
 *     [0]    segment  uint8   0..4
 *     [1]    first    uint8   first cell index in this triplet
 *     [2:3]  cell0    uint16  1 mV
 *     [4:5]  cell1    uint16  1 mV
 *     [6:7]  cell2    uint16  1 mV
 *
 *   0x211 — ADBMS replacement temperature triplet
 *     [0]    segment  uint8   0..4
 *     [1]    first    uint8   first thermistor index in this triplet
 *     [2:3]  temp0    int16   0.1 C
 *     [4:5]  temp1    int16   0.1 C
 *     [6:7]  temp2    int16   0.1 C
 *
 * IMPORTANT: 0x200 voltage scaling is now 10 mV/bit, not 1 mV/bit.
 * The old 1 mV scale would saturate above 65.535 V and cannot represent a
 * 75s accumulator. Update the receiver decode to V_pack = raw * 0.01f.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "mcp2515_driver.h"
#include "plant_shared.h"
#include "drive_profiles.h"
#include "drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.h"

static const char *TAG = "P42A_PLANT";

/* CAN frame IDs */
#define CAN_ID_MEAS         0x200U
#define CAN_ID_TRUTH        0x201U
#define CAN_ID_AMS_SUMMARY  0x202U
#define CAN_ID_CELL_SAMPLE  0x210U
#define CAN_ID_TEMP_SAMPLE  0x211U
#define CAN_ID_PLANT_CTRL   0x300U

#define PLANT_CMD_RESET0    0xA5U
#define PLANT_CMD_RESET1    0x5AU
#define PLANT_CMD_RESET2    0x52U  /* 'R' */

#define PLANT_PERIOD_MS     100U
#define PROFILE_AMBIENT_C   25.0f

/* Current profile selection */
#define PROFILE_SYNTH_HPPC  0
#define PROFILE_UDDS_25C    1
#define PROFILE_US06_25C    2
#define PROFILE_LA92_25C    3

#ifndef PLANT_PROFILE_MODE
#define PLANT_PROFILE_MODE  PROFILE_US06_25C
#endif

#define PROFILE_LOOP_ENABLE 1
#define US06_ALT_ENABLE     0
#define US06_ALT_OFFSET_STEPS 1000U

/* Optional measurement perturbations. Keep zero for estimator bring-up. */
#define MEAS_I_BIAS_A       0.0f
#define MEAS_V_NOISE_STD_V  0.0f

/* 75s pack cannot fit uint16 at 1 mV/bit. Use 10 mV/bit for V_pack. */
#define PACK_VOLTAGE_COUNTS_PER_V 100.0f
#define GROUP_VOLTAGE_COUNTS_PER_V 1000.0f
#define TEMP_COUNTS_PER_C 100.0f
#define CURRENT_COUNTS_PER_A 100.0f
#define SOC_COUNTS_PER_UNIT 10000.0f

#define CELL_SAMPLE_STRIDE 3U
#define TEMP_SAMPLE_STRIDE 3U

/* Generated Simulink model state. */
static RT_MODEL_drev_75s6p_p42a_accu_T s_plant_M;
static DW_drev_75s6p_p42a_accumulato_T s_plant_DW;

/* Generated model output buffers. Static to avoid large task stack usage. */
static real32_T s_y_v_pack = 0.0F;
static real32_T s_y_t_core = 25.0F;
static real32_T s_y_t_surf = 25.0F;
static real32_T s_y_soc_true = 1.0F;
static real32_T s_y_v_group[PLANT_NUM_GROUPS];
static real32_T s_y_v_segment[PLANT_NUM_SEGMENTS];
static real32_T s_y_t_sensor[PLANT_NUM_THERMISTORS];
static real32_T s_y_soc_group[PLANT_NUM_GROUPS];
static real32_T s_y_v_min = 0.0F;
static real32_T s_y_v_max = 0.0F;
static real32_T s_y_t_max = 25.0F;
static real32_T s_y_t_avg = 25.0F;

SemaphoreHandle_t plant_mutex = NULL;
plant_shared_t plant_shared = { 0 };

static volatile bool plant_reset_requested = false;

#ifdef __GNUC__
#define PLANT_MAYBE_UNUSED __attribute__((unused))
#else
#define PLANT_MAYBE_UNUSED
#endif

static uint32_t noise_lcg_state = 0x12345678U;

typedef struct {
    float V_pack;
    float I_pack;
    float T_surf;
    float T_core;
    float SoC_true;
    float V_group[PLANT_NUM_GROUPS];
    float T_sensor[PLANT_NUM_THERMISTORS];
    float V_min;
    float V_max;
    float T_max;
    float T_avg;
    uint8_t counter;
    uint32_t step;
} plant_can_snapshot_t;

static float noise_uniform_0_1(void)
{
    noise_lcg_state = (1664525U * noise_lcg_state) + 1013904223U;
    uint32_t r = noise_lcg_state >> 8;
    return ((float)r) * (1.0f / 16777216.0f);
}

static float noise_gaussian_unit(void)
{
    float sum = 0.0f;

    for (uint32_t i = 0U; i < 12U; i++)
    {
        sum += noise_uniform_0_1();
    }

    return sum - 6.0f;
}

static void poll_control_rx(void)
{
    uint16_t id = 0U;
    uint8_t len = 0U;
    uint8_t data[8] = {0};

    for (int n = 0; n < 4; n++)
    {
        esp_err_t err = mcp2515_read_frame(&id, data, &len);
        if (err != ESP_OK)
        {
            break;
        }

        if ((id == CAN_ID_PLANT_CTRL) && (len >= 3U) &&
            (data[0] == PLANT_CMD_RESET0) &&
            (data[1] == PLANT_CMD_RESET1) &&
            (data[2] == PLANT_CMD_RESET2))
        {
            plant_reset_requested = true;
            printf("PLANT RESET CMD RX\n");
        }
    }
}

static PLANT_MAYBE_UNUSED float get_I_pack_synth_hppc(uint32_t step)
{
    uint32_t s = step % 600U;   /* 60 s repeating cycle at 10 Hz */

    if      (s < 100U) return  0.0f;    /* 0.0–9.9 s rest */
    else if (s < 110U) return  100.0f;  /* 10.0–10.9 s discharge pulse */
    else if (s < 150U) return  0.0f;    /* 11.0–14.9 s rest */
    else if (s < 160U) return -40.0f;   /* 15.0–15.9 s regen */
    else if (s < 260U) return  60.0f;   /* 16.0–25.9 s sustained discharge */
    else if (s < 310U) return  0.0f;    /* 26.0–30.9 s rest */
    else if (s < 360U) return  120.0f;  /* 31.0–35.9 s heavy discharge */
    else if (s < 400U) return  0.0f;    /* 36.0–39.9 s rest */
    else if (s < 440U) return -60.0f;   /* 40.0–43.9 s regen */
    else               return  0.0f;    /* 44.0–59.9 s rest */
}

static float replay_i_10ma(const int16_t *arr, uint32_t len, uint32_t step)
{
    if ((arr == NULL) || (len == 0U))
    {
        return 0.0f;
    }

#if PROFILE_LOOP_ENABLE
    uint32_t idx = step % len;
#else
    if (step >= len)
    {
        return 0.0f;   /* one-shot profile, then rest */
    }

    uint32_t idx = step;
#endif

    return ((float)arr[idx]) * 0.01f;
}

static float get_I_pack(uint32_t step)
{
#if (PLANT_PROFILE_MODE == PROFILE_SYNTH_HPPC)
    return get_I_pack_synth_hppc(step);
#elif (PLANT_PROFILE_MODE == PROFILE_UDDS_25C)
    return replay_i_10ma(udds25_i_10ma, UDDS25_LEN, step);
#elif (PLANT_PROFILE_MODE == PROFILE_US06_25C)
#if US06_ALT_ENABLE
    if (US06_25_LEN == 0U)
    {
        return 0.0f;
    }
    uint32_t idx = (step + US06_ALT_OFFSET_STEPS) % US06_25_LEN;
    return ((float)us06_25_i_10ma[idx]) * 0.01f;
#else
    return replay_i_10ma(us06_25_i_10ma, US06_25_LEN, step);
#endif
#elif (PLANT_PROFILE_MODE == PROFILE_LA92_25C)
    return replay_i_10ma(la92_25_i_10ma, LA92_25_LEN, step);
#else
    return get_I_pack_synth_hppc(step);
#endif
}

static inline uint16_t sat_u16(float v)
{
    if (!isfinite(v) || (v <= 0.0f)) {
        return 0U;
    }
    if (v >= 65535.0f) {
        return 65535U;
    }
    return (uint16_t)(v + 0.5f);
}

static inline int16_t sat_i16(float v)
{
    if (!isfinite(v)) {
        return 0;
    }
    if (v >= 32767.0f) {
        return INT16_MAX;
    }
    if (v <= -32768.0f) {
        return INT16_MIN;
    }
    return (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

static void pack_meas(const plant_can_snapshot_t *d, uint8_t *buf)
{
    if ((d == NULL) || (buf == NULL)) {
        return;
    }

    float V_meas = d->V_pack + (MEAS_V_NOISE_STD_V * noise_gaussian_unit());
    float I_meas = d->I_pack + MEAS_I_BIAS_A;

    uint16_t v_10mV = sat_u16(V_meas * PACK_VOLTAGE_COUNTS_PER_V);
    int16_t i_10mA  = sat_i16(I_meas * CURRENT_COUNTS_PER_A);
    int16_t ts_cC   = sat_i16(d->T_surf * TEMP_COUNTS_PER_C);

    buf[0] = (uint8_t)(v_10mV >> 8);
    buf[1] = (uint8_t)(v_10mV & 0xFFU);

    buf[2] = (uint8_t)((uint16_t)i_10mA >> 8);
    buf[3] = (uint8_t)((uint16_t)i_10mA & 0xFFU);

    buf[4] = (uint8_t)((uint16_t)ts_cC >> 8);
    buf[5] = (uint8_t)((uint16_t)ts_cC & 0xFFU);

    buf[6] = d->counter;
    buf[7] = 0x00U;
}

static void pack_truth(const plant_can_snapshot_t *d, uint8_t *buf)
{
    if ((d == NULL) || (buf == NULL)) {
        return;
    }

    uint16_t soc_d2 = sat_u16(d->SoC_true * SOC_COUNTS_PER_UNIT);
    int16_t tc_cC = sat_i16(d->T_core * TEMP_COUNTS_PER_C);
    uint32_t step24 = d->step & 0x00FFFFFFU;

    buf[0] = (uint8_t)(soc_d2 >> 8);
    buf[1] = (uint8_t)(soc_d2 & 0xFFU);

    buf[2] = (uint8_t)((uint16_t)tc_cC >> 8);
    buf[3] = (uint8_t)((uint16_t)tc_cC & 0xFFU);

    buf[4] = d->counter;
    buf[5] = (uint8_t)(step24 >> 16);
    buf[6] = (uint8_t)(step24 >> 8);
    buf[7] = (uint8_t)(step24);
}

static void pack_ams_summary(const plant_can_snapshot_t *d, uint8_t *buf)
{
    if ((d == NULL) || (buf == NULL)) {
        return;
    }

    uint16_t vmin_mV = sat_u16(d->V_min * GROUP_VOLTAGE_COUNTS_PER_V);
    uint16_t vmax_mV = sat_u16(d->V_max * GROUP_VOLTAGE_COUNTS_PER_V);
    int16_t tmax_cC = sat_i16(d->T_max * TEMP_COUNTS_PER_C);
    int16_t tavg_cC = sat_i16(d->T_avg * TEMP_COUNTS_PER_C);

    buf[0] = (uint8_t)(vmin_mV >> 8);
    buf[1] = (uint8_t)(vmin_mV & 0xFFU);

    buf[2] = (uint8_t)(vmax_mV >> 8);
    buf[3] = (uint8_t)(vmax_mV & 0xFFU);

    buf[4] = (uint8_t)((uint16_t)tmax_cC >> 8);
    buf[5] = (uint8_t)((uint16_t)tmax_cC & 0xFFU);

    buf[6] = (uint8_t)((uint16_t)tavg_cC >> 8);
    buf[7] = (uint8_t)((uint16_t)tavg_cC & 0xFFU);
}

static void pack_cell_sample(const plant_can_snapshot_t *d,
                             uint8_t seg,
                             uint8_t first_cell,
                             uint8_t *buf)
{
    if ((d == NULL) || (buf == NULL)) {
        return;
    }

    buf[0] = seg;
    buf[1] = first_cell;

    for (uint8_t n = 0U; n < CELL_SAMPLE_STRIDE; n++)
    {
        uint8_t cell = (uint8_t)(first_cell + n);
        uint16_t mv = 0U;

        if ((seg < PLANT_NUM_SEGMENTS) && (cell < 15U))
        {
            uint32_t group = ((uint32_t)seg * 15U) + cell;
            if (group < PLANT_NUM_GROUPS)
            {
                mv = sat_u16(d->V_group[group] * GROUP_VOLTAGE_COUNTS_PER_V);
            }
        }

        uint8_t off = (uint8_t)(2U + (2U * n));
        buf[off] = (uint8_t)(mv >> 8);
        buf[off + 1U] = (uint8_t)(mv & 0xFFU);
    }
}

static void pack_temp_sample(const plant_can_snapshot_t *d,
                             uint8_t seg,
                             uint8_t first_sensor,
                             uint8_t *buf)
{
    if ((d == NULL) || (buf == NULL)) {
        return;
    }

    buf[0] = seg;
    buf[1] = first_sensor;

    for (uint8_t n = 0U; n < TEMP_SAMPLE_STRIDE; n++)
    {
        uint8_t sensor = (uint8_t)(first_sensor + n);
        int16_t deci_c = 0;

        if ((seg < PLANT_NUM_SEGMENTS) && (sensor < 24U))
        {
            uint32_t therm = ((uint32_t)seg * 24U) + sensor;
            if (therm < PLANT_NUM_THERMISTORS)
            {
                deci_c = sat_i16(d->T_sensor[therm] * 10.0f);
            }
        }

        uint8_t off = (uint8_t)(2U + (2U * n));
        buf[off] = (uint8_t)((uint16_t)deci_c >> 8);
        buf[off + 1U] = (uint8_t)((uint16_t)deci_c & 0xFFU);
    }
}

static void plant_model_reset(void)
{
    memset(&s_plant_DW, 0, sizeof(s_plant_DW));
    s_plant_M.dwork = &s_plant_DW;

    drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_initialize(&s_plant_M);

    memset(s_y_v_group, 0, sizeof(s_y_v_group));
    memset(s_y_v_segment, 0, sizeof(s_y_v_segment));
    memset(s_y_t_sensor, 0, sizeof(s_y_t_sensor));
    memset(s_y_soc_group, 0, sizeof(s_y_soc_group));

    s_y_v_pack = 0.0F;
    s_y_t_core = PROFILE_AMBIENT_C;
    s_y_t_surf = PROFILE_AMBIENT_C;
    s_y_soc_true = 1.0F;
    s_y_v_min = 0.0F;
    s_y_v_max = 0.0F;
    s_y_t_max = PROFILE_AMBIENT_C;
    s_y_t_avg = PROFILE_AMBIENT_C;
}

static void publish_model_outputs(float I_pack, uint32_t step)
{
    if (xSemaphoreTake(plant_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        plant_shared.V_pack = (float)s_y_v_pack;
        plant_shared.I_pack = I_pack;
        plant_shared.T_surf = (float)s_y_t_surf;
        plant_shared.T_core = (float)s_y_t_core;
        plant_shared.SoC_true = (float)s_y_soc_true;

        memcpy(plant_shared.V_group, s_y_v_group, sizeof(plant_shared.V_group));
        memcpy(plant_shared.V_segment, s_y_v_segment, sizeof(plant_shared.V_segment));
        memcpy(plant_shared.T_sensor, s_y_t_sensor, sizeof(plant_shared.T_sensor));
        memcpy(plant_shared.SoC_group, s_y_soc_group, sizeof(plant_shared.SoC_group));

        plant_shared.V_min = (float)s_y_v_min;
        plant_shared.V_max = (float)s_y_v_max;
        plant_shared.T_max = (float)s_y_t_max;
        plant_shared.T_avg = (float)s_y_t_avg;

        plant_shared.counter++;
        plant_shared.step = step;

        xSemaphoreGive(plant_mutex);
    }
}

static bool read_can_snapshot(plant_can_snapshot_t *snap)
{
    if (snap == NULL)
    {
        return false;
    }

    if (xSemaphoreTake(plant_mutex, pdMS_TO_TICKS(5)) != pdTRUE)
    {
        return false;
    }

    snap->V_pack = plant_shared.V_pack;
    snap->I_pack = plant_shared.I_pack;
    snap->T_surf = plant_shared.T_surf;
    snap->T_core = plant_shared.T_core;
    snap->SoC_true = plant_shared.SoC_true;
    memcpy(snap->V_group, plant_shared.V_group, sizeof(snap->V_group));
    memcpy(snap->T_sensor, plant_shared.T_sensor, sizeof(snap->T_sensor));
    snap->V_min = plant_shared.V_min;
    snap->V_max = plant_shared.V_max;
    snap->T_max = plant_shared.T_max;
    snap->T_avg = plant_shared.T_avg;
    snap->counter = plant_shared.counter;
    snap->step = plant_shared.step;

    xSemaphoreGive(plant_mutex);
    return true;
}

static void plant_task(void *pvParameters)
{
    (void)pvParameters;

    plant_model_reset();

    const TickType_t period = pdMS_TO_TICKS(PLANT_PERIOD_MS);
    TickType_t xLastWake = xTaskGetTickCount();
    uint32_t step = 0U;

    printf("75s6p P42A Simulink plant started, profile mode=%d, ambient=%.1f C\n",
           PLANT_PROFILE_MODE,
           (double)PROFILE_AMBIENT_C);

    for (;;)
    {
        if (plant_reset_requested)
        {
            plant_reset_requested = false;
            plant_model_reset();
            step = 0U;
            xLastWake = xTaskGetTickCount();

            if (xSemaphoreTake(plant_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                memset(&plant_shared, 0, sizeof(plant_shared));
                plant_shared.T_surf = PROFILE_AMBIENT_C;
                plant_shared.T_core = PROFILE_AMBIENT_C;
                plant_shared.T_max = PROFILE_AMBIENT_C;
                plant_shared.T_avg = PROFILE_AMBIENT_C;
                plant_shared.SoC_true = 1.0f;
                plant_shared.step = 0U;
                xSemaphoreGive(plant_mutex);
            }

            printf("PLANT MODEL RESET DONE\n");
            vTaskDelayUntil(&xLastWake, period);
            continue;
        }

        float I_pack = get_I_pack(step);

        drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_step(
            &s_plant_M,
            (real32_T)I_pack,
            (real32_T)PROFILE_AMBIENT_C,
            &s_y_v_pack,
            &s_y_t_core,
            &s_y_t_surf,
            &s_y_soc_true,
            s_y_v_group,
            s_y_v_segment,
            s_y_t_sensor,
            s_y_soc_group,
            &s_y_v_min,
            &s_y_v_max,
            &s_y_t_max,
            &s_y_t_avg);

        publish_model_outputs(I_pack, step);

        if ((step % 100U) == 0U)
        {
            plant_can_snapshot_t snap;
            if (read_can_snapshot(&snap))
            {
                printf("PLANT step=%lu I=%.2f V=%.3f SoC=%.5f Vmin=%.4f Vmax=%.4f Ts=%.2f Tc=%.2f Tmax=%.2f\n",
                       (unsigned long)snap.step,
                       (double)snap.I_pack,
                       (double)snap.V_pack,
                       (double)snap.SoC_true,
                       (double)snap.V_min,
                       (double)snap.V_max,
                       (double)snap.T_surf,
                       (double)snap.T_core,
                       (double)snap.T_max);
            }
        }

        if (step < 0x00FFFFFFU)
        {
            step++;
        }

        vTaskDelayUntil(&xLastWake, period);
    }
}

static void can_tx_task(void *pvParameters)
{
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(PLANT_PERIOD_MS);
    TickType_t xLastWake = xTaskGetTickCount();

    uint8_t meas_buf[8] = {0};
    uint8_t truth_buf[8] = {0};
    uint8_t ams_buf[8] = {0};
    uint8_t cell_buf[8] = {0};
    uint8_t temp_buf[8] = {0};

    uint32_t tx_count = 0U;

    printf("can_tx_task started\n");

    for (;;)
    {
        poll_control_rx();

        plant_can_snapshot_t snap;

        if (read_can_snapshot(&snap))
        {
            pack_meas(&snap, meas_buf);
            pack_truth(&snap, truth_buf);
            pack_ams_summary(&snap, ams_buf);

            esp_err_t e1 = mcp2515_send_frame(CAN_ID_MEAS, meas_buf, 8);
            esp_err_t e2 = mcp2515_send_frame(CAN_ID_TRUTH, truth_buf, 8);
            esp_err_t e3 = mcp2515_send_frame(CAN_ID_AMS_SUMMARY, ams_buf, 8);
            bool image_ok = true;

            for (uint8_t seg = 0U; seg < PLANT_NUM_SEGMENTS; seg++)
            {
                for (uint8_t first = 0U; first < 15U; first = (uint8_t)(first + CELL_SAMPLE_STRIDE))
                {
                    pack_cell_sample(&snap, seg, first, cell_buf);
                    if (mcp2515_send_frame(CAN_ID_CELL_SAMPLE, cell_buf, 8) != ESP_OK)
                    {
                        image_ok = false;
                    }
                }
            }

            for (uint8_t seg = 0U; seg < PLANT_NUM_SEGMENTS; seg++)
            {
                for (uint8_t first = 0U; first < 24U; first = (uint8_t)(first + TEMP_SAMPLE_STRIDE))
                {
                    pack_temp_sample(&snap, seg, first, temp_buf);
                    if (mcp2515_send_frame(CAN_ID_TEMP_SAMPLE, temp_buf, 8) != ESP_OK)
                    {
                        image_ok = false;
                    }
                }
            }

            if ((e1 == ESP_OK) && (e2 == ESP_OK) && (e3 == ESP_OK) && image_ok)
            {
                tx_count++;

                if ((tx_count % 20U) == 0U)
                {
                    uint32_t step24 =
                        ((uint32_t)truth_buf[5] << 16) |
                        ((uint32_t)truth_buf[6] << 8)  |
                        ((uint32_t)truth_buf[7]);

                    printf("CAN TX alive count=%lu ctr=%u step=%lu Vraw10mV=%u\n",
                           (unsigned long)tx_count,
                           (unsigned int)meas_buf[6],
                           (unsigned long)step24,
                           (unsigned int)(((uint16_t)meas_buf[0] << 8) | meas_buf[1]));
                }
            }
            else
            {
                printf("TX FAIL e1=%d e2=%d e3=%d image_ok=%d\n", e1, e2, e3, image_ok ? 1 : 0);
            }
        }
        else
        {
            printf("TX mutex timeout\n");
        }

        vTaskDelayUntil(&xLastWake, period);
    }
}

void app_main(void)
{
    plant_mutex = xSemaphoreCreateMutex();
    configASSERT(plant_mutex);

    esp_err_t can_err = ESP_FAIL;

    for (int attempt = 0; attempt < 5; attempt++)
    {
        can_err = mcp2515_init();
        if (can_err == ESP_OK)
        {
            break;
        }

        ESP_LOGW(TAG, "MCP2515 init attempt %d failed: %d", attempt + 1, can_err);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (can_err != ESP_OK)
    {
        ESP_LOGE(TAG, "MCP2515 init failed — CAN TX will still attempt sends");
    }

    BaseType_t ok;

    ok = xTaskCreatePinnedToCore(plant_task,
                                 "p42a_plant",
                                 6144,
                                 NULL,
                                 5,
                                 NULL,
                                 0);
    configASSERT(ok == pdPASS);

    ok = xTaskCreatePinnedToCore(can_tx_task,
                                 "can_tx",
                                 4096,
                                 NULL,
                                 4,
                                 NULL,
                                 0);
    configASSERT(ok == pdPASS);
}
