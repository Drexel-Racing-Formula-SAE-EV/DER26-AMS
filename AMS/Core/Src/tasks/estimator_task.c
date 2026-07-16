/*
 * estimator_task.c
 * Author: Mahad Faisal (2026)
 *
 * Runs the advisory P42A DAEKF estimator at 10 Hz. The estimator is deliberately
 * non-authoritative: it does not control BMS_OK, AIRs, charging, shutdown, or
 * balancing. It only publishes state into app.estimator for telemetry/debug.
 */

#include "tasks/estimator_task.h"

#include "estimator/ams_soc_ekf.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

void estimator_task_fn(void *argument);

#define ESTIMATOR_STACK_WORDS       AMS_STACK_ESTIMATOR_WORDS
#define ESTIMATOR_HIL_TIMEOUT_TICKS 500U
#define ESTIMATOR_HW_MIN_TEMP_C     (-40.0f)
#define ESTIMATOR_HW_MAX_TEMP_C     120.0f

static float adc_code_to_cell_v(int16_t code)
{
    return ((float)code + 10000.0f) * 0.000150f;
}

static bool cell_code_valid(int16_t code, float *voltage_v)
{
    if ((code == 0) || (code == INT16_MIN))
    {
        return false;
    }

    float v = adc_code_to_cell_v(code);
    if ((!isfinite(v)) || (v < 0.5f) || (v > 5.0f))
    {
        return false;
    }

    if (voltage_v != NULL)
    {
        *voltage_v = v;
    }
    return true;
}

static bool temp_raw_to_c(int16_t raw, float *temp_c)
{
    if ((raw == 0) || (raw == -1) || (raw == INT16_MIN))
    {
        return false;
    }

    float voltage = adc_code_to_cell_v(raw);
    if ((!isfinite(voltage)) || (voltage <= 0.0f) || (voltage >= 5.0f))
    {
        return false;
    }

    float resistance = 10000.0f * (5.0f - voltage) / voltage;
    if ((!isfinite(resistance)) || (resistance <= 0.0f))
    {
        return false;
    }

    float x = logf(resistance / 10000.0f);
    float t = (1.0f / (3.354016435e-3f + (2.565235509e-4f * x))) - 273.15f;
    if ((!isfinite(t)) || (t < ESTIMATOR_HW_MIN_TEMP_C) || (t > ESTIMATOR_HW_MAX_TEMP_C))
    {
        return false;
    }

    if (temp_c != NULL)
    {
        *temp_c = t;
    }
    return true;
}

static bool collect_group_voltage(const app_data_t *data,
                                  const ams_ekf_config_t *cfg,
                                  float *v_meas_V,
                                  uint16_t *valid_count)
{
    if ((data == NULL) || (cfg == NULL) || (v_meas_V == NULL) || (valid_count == NULL))
    {
        return false;
    }

    float sum_v = 0.0f;
    uint16_t count = 0U;
    uint8_t ic_count = accumulator_configured_smb_count(&data->acc);

    for (uint16_t n = 0U; n < cfg->series_group_count; n++)
    {
        uint16_t group = (uint16_t)(cfg->first_series_group + n);
        uint8_t seg = (uint8_t)(group / NCELLS);
        uint8_t cell = (uint8_t)(group % NCELLS);

        if ((seg >= ic_count) || (cell >= NCELLS))
        {
            continue;
        }

        float v = 0.0f;
        if (cell_code_valid(data->acc.smb.ics[seg].cell.c_codes[cell], &v))
        {
            sum_v += v;
            count++;
        }
    }

    *v_meas_V = sum_v;
    *valid_count = count;
    return (count == cfg->series_group_count);
}

static bool collect_group_temp(const app_data_t *data,
                               const ams_ekf_config_t *cfg,
                               float *temp_c)
{
    if ((data == NULL) || (cfg == NULL) || (temp_c == NULL))
    {
        return false;
    }

    float sum = 0.0f;
    uint16_t count = 0U;
    uint8_t ic_count = accumulator_configured_smb_count(&data->acc);

    uint8_t first_seg = (uint8_t)(cfg->first_series_group / NCELLS);
    uint8_t last_seg = (uint8_t)((cfg->first_series_group + cfg->series_group_count - 1U) / NCELLS);

    for (uint8_t seg = first_seg; (seg <= last_seg) && (seg < ic_count); seg++)
    {
        for (uint8_t sensor = 0U; sensor < NTEMPS; sensor++)
        {
            float t = 0.0f;
            if (temp_raw_to_c(data->acc.smb.ics[seg].temp.raw[sensor], &t))
            {
                sum += t;
                count++;
            }
        }
    }

    if (count > 0U)
    {
        *temp_c = sum / (float)count;
        return true;
    }

    if (isfinite(data->avg_temp) &&
        (data->avg_temp >= ESTIMATOR_HW_MIN_TEMP_C) &&
        (data->avg_temp <= ESTIMATOR_HW_MAX_TEMP_C))
    {
        *temp_c = data->avg_temp;
        return true;
    }

    *temp_c = 25.0f;
    return false;
}

#if AMS_ENABLE_HIL_CAN
static bool hil_meas_fresh(const app_data_t *data, uint32_t now)
{
    if (data == NULL)
    {
        return false;
    }

    return ((data->hil.meas.fresh != 0U) &&
            ((now - data->hil.meas.last_rx_tick) <= ESTIMATOR_HIL_TIMEOUT_TICKS) &&
            isfinite(data->hil.meas.v_pack_V) &&
            isfinite(data->hil.meas.i_pack_A) &&
            isfinite(data->hil.meas.t_surf_C));
}
#endif

TaskHandle_t estimator_task_start(app_data_t *data)
{
    TaskHandle_t handle = NULL;

    if (data == NULL)
    {
        return NULL;
    }

    xTaskCreate(estimator_task_fn,
                "estimator task",
                ESTIMATOR_STACK_WORDS,
                (void *)data,
                EST_PRIO,
                &handle);
    return handle;
}

void estimator_task_fn(void *argument)
{
    app_data_t *data = (app_data_t *)argument;
    if (data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    ams_estimator_init_default(&data->estimator);
#if !AMS_EVAL_ESTIMATOR_ENABLED
    /* The production estimator is configured for the 75s6p DER pack.  Do not
     * reinterpret a one-board 16-channel resistor ladder as that pack. */
    data->estimator.enabled = 0u;
    data->estimator.instance_count = 0u;
    data->estimator.fault_flags = AMS_EKF_FAULT_DISABLED;
#endif

    uint32_t last_entry = osKernelGetTickCount();
    uint32_t entry;
    for (;;)
    {
        entry = osKernelGetTickCount();
        float cc_dt_s = (float)(entry - last_entry) / 1000.0f;
        last_entry = entry;

        if ((data->estimator.enabled == 0U) || (data->estimator.instance_count == 0U))
        {
            osDelayUntil(entry + (1000U / ESTIMATOR_FREQ));
            continue;
        }

        ams_estimator_input_source_t source = AMS_ESTIMATOR_INPUT_HARDWARE;
#if AMS_ENABLE_HIL_CAN
        bool use_hil = hil_meas_fresh(data, entry);
        if (use_hil)
        {
            source = AMS_ESTIMATOR_INPUT_HIL_CAN;
        }
        float cc_current_A = use_hil ? data->hil.meas.i_pack_A : data->current;
#else
        const bool use_hil = false;
        float cc_current_A = data->current;
#endif
        (void)ams_estimator_cc_step(&data->estimator, cc_current_A, cc_dt_s);

        for (uint8_t i = 0U; (i < data->estimator.instance_count) && (i < AMS_EKF_MAX_INSTANCES); i++)
        {
            ams_ekf_instance_t *inst = &data->estimator.inst[i];
            if (inst->cfg.enabled == 0U)
            {
                continue;
            }

            float v_meas = 0.0f;
            float i_pack = data->current;
            float t_surf = 25.0f;
            bool input_ok = false;

#if AMS_ENABLE_HIL_CAN
            if (use_hil &&
                (inst->cfg.first_series_group == 0U) &&
                (inst->cfg.series_group_count == AMS_EKF_PACK_SERIES_GROUPS))
            {
                v_meas = data->hil.meas.v_pack_V;
                i_pack = data->hil.meas.i_pack_A;
                t_surf = data->hil.meas.t_surf_C;
                input_ok = true;
            }
            else
#endif
            {
                uint16_t valid_v = 0U;
                bool voltage_ok = collect_group_voltage(data, &inst->cfg, &v_meas, &valid_v);
                bool temp_ok = collect_group_temp(data, &inst->cfg, &t_surf);
                (void)temp_ok;
                input_ok = voltage_ok && isfinite(i_pack);
            }

            if (input_ok)
            {
                (void)ams_ekf_step(inst, i_pack, v_meas, t_surf, inst->cfg.sample_time_s);
            }
            else
            {
                inst->valid = 0U;
                inst->fault_flags |= use_hil ? AMS_EKF_FAULT_STALE_INPUT : AMS_EKF_FAULT_BAD_INPUT;
            }
        }

        ams_estimator_refresh_summary(&data->estimator, source, entry);
        data->estimator_fault = (data->estimator.fault_flags != AMS_EKF_FAULT_NONE);
        osDelayUntil(entry + (1000U / ESTIMATOR_FREQ));
    }
}
