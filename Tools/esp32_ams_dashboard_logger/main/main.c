#include "ams_can_decode.h"
#include "mcp2515_driver.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DASH_WIFI_SSID          "DER26_AMS_DASH"
#define DASH_WIFI_PASS          "der26amslogger"
#define DASH_WIFI_CHANNEL       6
#define DASH_WIFI_MAX_CONN      4
#define DASH_STALE_TIMEOUT_MS   1500u
#define DASH_POWER_STALE_MS      250u
#define DASH_JSON_BUF_BYTES     16384u

static const char *TAG = "AMS_DASH";

static ams_dash_state_t g_state;
static SemaphoreHandle_t g_state_lock;

static bool flag_set(uint8_t flags, uint8_t bit)
{
    return (flags & (uint8_t)(1u << bit)) != 0u;
}

#if defined(__GNUC__)
__attribute__((format(printf, 4, 5)))
#endif
static void json_append(char *buf, size_t cap, size_t *off,
                        const char *fmt, ...)
{
    if((buf == NULL) || (off == NULL) || (*off >= cap))
    {
        return;
    }

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(&buf[*off], cap - *off, fmt, ap);
    va_end(ap);

    if(n <= 0)
    {
        return;
    }

    size_t written = (size_t)n;
    if(written >= (cap - *off))
    {
        *off = cap - 1u;
        buf[*off] = '\0';
    }
    else
    {
        *off += written;
    }
}

static void append_cell_array(char *buf, size_t cap, size_t *off, const ams_dash_state_t *s)
{
    json_append(buf, cap, off, "\"cells_mv\":[");
    for(uint8_t seg = 0u; seg < AMS_DASH_SEGMENTS; seg++)
    {
        json_append(buf, cap, off, "%s[", (seg == 0u) ? "" : ",");
        for(uint8_t cell = 0u; cell < AMS_DASH_CELLS_PER_SEG; cell++)
        {
            json_append(buf, cap, off, "%s%u", (cell == 0u) ? "" : ",", s->cell_mv[seg][cell]);
        }
        json_append(buf, cap, off, "]");
    }
    json_append(buf, cap, off, "]");
}

static void append_temp_array(char *buf, size_t cap, size_t *off, const ams_dash_state_t *s)
{
    json_append(buf, cap, off, "\"temps_dC\":[");
    for(uint8_t seg = 0u; seg < AMS_DASH_SEGMENTS; seg++)
    {
        json_append(buf, cap, off, "%s[", (seg == 0u) ? "" : ",");
        for(uint8_t sensor = 0u; sensor < AMS_DASH_TEMPS_PER_SEG; sensor++)
        {
            int16_t temp = s->temp_dC[seg][sensor];
            json_append(buf, cap, off, "%s%d", (sensor == 0u) ? "" : ",", (int)temp);
        }
        json_append(buf, cap, off, "]");
    }
    json_append(buf, cap, off, "]");
}

static void append_mask_array_u16(char *buf,
                                  size_t cap,
                                  size_t *off,
                                  const char *name,
                                  const uint16_t masks[AMS_DASH_SEGMENTS])
{
    json_append(buf, cap, off, "\"%s\":[", name);
    for(uint8_t seg = 0u; seg < AMS_DASH_SEGMENTS; seg++)
    {
        json_append(buf, cap, off, "%s%u", (seg == 0u) ? "" : ",", masks[seg]);
    }
    json_append(buf, cap, off, "]");
}

static void append_mask_array_u24(char *buf,
                                  size_t cap,
                                  size_t *off,
                                  const char *name,
                                  const uint32_t masks[AMS_DASH_SEGMENTS])
{
    json_append(buf, cap, off, "\"%s\":[", name);
    for(uint8_t seg = 0u; seg < AMS_DASH_SEGMENTS; seg++)
    {
        json_append(buf, cap, off, "%s%lu", (seg == 0u) ? "" : ",", (unsigned long)(masks[seg] & 0x00FFFFFFu));
    }
    json_append(buf, cap, off, "]");
}

static void build_json_state(char *buf, size_t cap, const ams_dash_state_t *s)
{
    size_t off = 0u;
    uint32_t now_ms = (uint32_t)(esp_log_timestamp());
    bool stale = ams_dash_data_stale(s, now_ms, DASH_STALE_TIMEOUT_MS);
    bool power_stale = ams_dash_power_data_stale(
        s, now_ms, DASH_POWER_STALE_MS);
    bool binding_metadata_fresh =
        ((s->power_counter_seen_mask & (uint8_t)(1u << 5u)) != 0u) &&
        ((uint32_t)(now_ms - s->power_last_rx_ms[5]) <= DASH_POWER_STALE_MS) &&
        !power_stale &&
        (s->power_counter[5] == s->power_counter[0]) &&
        (s->power_counter[0] == s->power_counter[1]) &&
        (s->power_counter[0] == s->power_counter[2]) &&
        (s->power_counter[0] == s->power_counter[3]);

    json_append(buf, cap, &off, "{");
    json_append(buf, cap, &off,
                "\"rx_frames\":%lu,\"logger_frames\":%lu,\"unknown_frames\":%lu,"
                "\"ignored_frames\":%lu,\"malformed_frames\":%lu,"
                "\"stale\":%s,\"last_rx_ms\":%lu,\"last_heartbeat_ms\":%lu,",
                (unsigned long)s->rx_frames,
                (unsigned long)s->logger_frames,
                (unsigned long)s->unknown_frames,
                (unsigned long)s->ignored_frames,
                (unsigned long)s->malformed_frames,
                stale ? "true" : "false",
                (unsigned long)s->last_rx_ms,
                (unsigned long)s->last_heartbeat_ms);

    json_append(buf, cap, &off,
                "\"heartbeat\":{\"version\":%u,\"seq\":%u,\"state\":%u,"
                "\"bms_ok\":%s,\"air\":%s,\"imd_ok\":%s,\"hard_fault\":%s,"
                "\"soft_fault\":%s,\"voltage_valid\":%s,\"current_valid\":%s,"
                "\"temp_valid\":%s,\"uptime_s\":%u},",
                s->protocol_version,
                s->sequence,
                s->state,
                flag_set(s->status_flags, 0u) ? "true" : "false",
                flag_set(s->status_flags, 1u) ? "true" : "false",
                flag_set(s->status_flags, 2u) ? "true" : "false",
                flag_set(s->status_flags, 3u) ? "true" : "false",
                flag_set(s->status_flags, 4u) ? "true" : "false",
                flag_set(s->validity_flags, 0u) ? "true" : "false",
                flag_set(s->validity_flags, 1u) ? "true" : "false",
                flag_set(s->validity_flags, 2u) ? "true" : "false",
                s->ams_uptime_s);

    json_append(buf, cap, &off,
                "\"pack\":{\"voltage_V\":%.1f,\"current_A\":%.1f,"
                "\"min_cell_mV\":%u,\"max_cell_mV\":%u},",
                (double)s->pack_voltage_dV / 10.0,
                (double)s->current_dA / 10.0,
                s->min_cell_mv,
                s->max_cell_mv);

    json_append(buf, cap, &off,
                "\"temp\":{\"max_C\":%.1f,\"min_C\":%.1f,\"avg_C\":%.1f,"
                "\"filtered_max_C\":%.1f,\"max_rate_C_per_s\":%.1f,"
                "\"max_fan_percent\":%u,\"fan_command_percent\":%u,"
                "\"fan_reason\":%u,\"flags\":%u,\"diag_flags\":%u,"
                "\"fan_diag_flags\":%u},",
                (double)s->max_temp_dC / 10.0,
                (double)s->min_temp_dC / 10.0,
                (double)s->avg_temp_dC / 10.0,
                (double)s->temp_filtered_max_dC / 10.0,
                (double)s->temp_max_rate_dC_per_s / 10.0,
                s->max_fan_percent,
                s->fan_command_percent,
                s->fan_control_reason,
                s->temp_flags,
                s->temp_diag_flags,
                s->fan_diag_flags);

    json_append(buf, cap, &off,
                "\"faults\":{\"voltage_reason\":%u,\"voltage_latched\":%u,"
                "\"temp_reason\":%u,\"temp_pending\":%u,\"temp_latched\":%u,"
                "\"current_reason\":%u,\"current_latched\":%u,\"current_mode\":%u},",
                s->voltage_reason,
                s->voltage_latched_reason,
                s->temp_reason,
                s->temp_pending_reason,
                s->temp_latched_reason,
                s->current_reason,
                s->current_latched_reason,
                s->current_mode);

    json_append(buf, cap, &off,
                "\"health\":{\"voltage_usable\":%u,\"voltage_updated\":%u,"
                "\"voltage_stale\":%u,\"voltage_pec_fail\":%u,"
                "\"temp_usable\":%u,\"temp_updated\":%u,\"temp_stale\":%u,"
                "\"temp_invalid\":%u,\"max_v_seg\":%u,\"max_v_cell\":%u,"
                "\"min_v_seg\":%u,\"min_v_cell\":%u,\"max_t_seg\":%u,"
                "\"max_t_sensor\":%u,\"min_t_seg\":%u,\"min_t_sensor\":%u},",
                s->voltage_usable_count,
                s->voltage_updated_count,
                s->voltage_stale_count,
                s->voltage_pec_fail_count,
                s->temp_usable_count,
                s->temp_updated_count,
                s->temp_stale_count,
                s->temp_invalid_count,
                s->max_voltage_seg,
                s->max_voltage_cell,
                s->min_voltage_seg,
                s->min_voltage_cell,
                s->max_temp_seg,
                s->max_temp_sensor,
                s->min_temp_seg,
                s->min_temp_sensor);

    json_append(buf, cap, &off,
                "\"charger\":{\"target_voltage_V\":%.1f,\"target_current_A\":%.1f,"
                "\"read_voltage_V\":%.1f,\"read_current_A\":%.1f,"
                "\"flags\":%u,\"raw_flags\":%u,\"disable_reason_mask\":%u,"
                "\"last_tx_status\":%u,\"tx_count\":%u,\"rx_count\":%u,"
                "\"tx_fail_count\":%u},",
                (double)s->charger_target_voltage_dV / 10.0,
                (double)s->charger_target_current_dA / 10.0,
                (double)s->charger_read_voltage_dV / 10.0,
                (double)s->charger_read_current_dA / 10.0,
                s->charger_flags,
                s->charger_raw_flags,
                s->charger_disable_reason_mask,
                s->charger_last_tx_status,
                s->charger_tx_count,
                s->charger_rx_count,
                s->charger_tx_fail_count);

    json_append(buf, cap, &off,
                "\"adbms\":{\"smb_status\":%u,\"smb_xfer\":%u,\"smb_op\":%u,"
                "\"smb_errors\":%u,\"smb_pec_fail_mask\":%u,"
                "\"smb_counter_mismatch_mask\":%u,\"smb_counter_errors\":%u,"
                "\"apm_status\":%u,\"apm_xfer\":%u,\"apm_op\":%u,"
                "\"apm_errors\":%u,\"apm_pec_fail_mask\":%u,"
                "\"scan_count\":%u,\"status_diag_count\":%u,"
                "\"config_diag_count\":%u,\"open_wire_diag_count\":%u,"
                "\"last_diag_status\":%u,\"diag_flags\":%u,\"hil_flags\":%u},",
                s->adbms6830_last_status,
                s->adbms6830_last_xfer_status,
                s->adbms6830_last_op,
                s->adbms6830_error_count,
                s->adbms6830_pec_fail_mask,
                s->adbms6830_counter_mismatch_mask,
                s->adbms6830_counter_error_count,
                s->adbms2950_last_status,
                s->adbms2950_last_xfer_status,
                s->adbms2950_last_op,
                s->adbms2950_error_count_u8,
                s->adbms2950_pec_fail_mask,
                s->adbms_scan_count,
                s->adbms_status_diag_count,
                s->adbms_config_diag_count,
                s->adbms_open_wire_diag_count,
                s->adbms_last_diag_status,
                s->adbms_diag_flags,
                s->adbms_hil_flags);

    json_append(buf, cap, &off,
                "\"current_adc\":{\"high_count\":%u,\"low_count\":%u,"
                "\"flags\":%u,\"high_fresh\":%s,\"low_fresh\":%s,"
                "\"last_read_ok\":%s,\"current_valid\":%s,"
                "\"sensor_fault\":%s,\"zero_calibrated\":%s,\"zero_cal_count\":%u},",
                s->current_adc_high_count,
                s->current_adc_low_count,
                s->current_adc_flags,
                flag_set(s->current_adc_flags, 0u) ? "true" : "false",
                flag_set(s->current_adc_flags, 1u) ? "true" : "false",
                flag_set(s->current_adc_flags, 2u) ? "true" : "false",
                flag_set(s->current_adc_flags, 3u) ? "true" : "false",
                flag_set(s->current_adc_flags, 4u) ? "true" : "false",
                flag_set(s->current_adc_flags, 5u) ? "true" : "false",
                s->current_zero_cal_count);

    json_append(buf, cap, &off,
                "\"tasks\":{\"stale_mask\":%u,\"seen_mask\":%u,"
                "\"safety_stale_mask\":%u,\"task_heartbeat_fault\":%s,"
                "\"logger_heartbeat_fault\":%s,\"logger_count\":%u},",
                s->heartbeat_stale_mask,
                s->heartbeat_seen_mask,
                s->heartbeat_safety_stale_mask,
                flag_set(s->task_health_flags, 0u) ? "true" : "false",
                flag_set(s->task_health_flags, 1u) ? "true" : "false",
                s->logger_heartbeat_count);

    json_append(buf, cap, &off,
                "\"can\":{\"error_code\":%lu,\"busoff_count\":%u,"
                "\"error_count\":%u,\"recover_count\":%u,"
                "\"busoff_fault\":%s,\"recover_pending\":%s},",
                (unsigned long)s->can_error_code,
                s->can_busoff_count,
                s->can_error_count,
                s->can_recover_count,
                flag_set(s->can_diag_flags, 0u) ? "true" : "false",
                flag_set(s->can_diag_flags, 1u) ? "true" : "false");

    json_append(buf, cap, &off,
                "\"safety\":{\"reset_flags\":%lu,\"last_panic_reason\":%u,"
                "\"panic_count\":%u,\"bms_block_count\":%u,"
                "\"panic_active\":%s,\"bms_output_inhibit\":%s,"
                "\"balance_inhibit\":%s,\"bms_state\":%s},",
                (unsigned long)s->safety_reset_flags,
                s->safety_last_panic_reason,
                s->safety_panic_count,
                s->safety_bms_block_count,
                flag_set(s->safety_flags, 0u) ? "true" : "false",
                flag_set(s->safety_flags, 1u) ? "true" : "false",
                flag_set(s->safety_flags, 2u) ? "true" : "false",
                flag_set(s->safety_flags, 3u) ? "true" : "false");

    json_append(buf, cap, &off,
                "\"watchdog\":{\"runtime_enabled\":%s,\"hw_started\":%s,"
                "\"feed_gate_ok\":%s,\"last_block_reason\":%u,"
                "\"feed_count\":%u,\"block_count\":%u,"
                "\"last_feed_age_s\":%.1f},",
                flag_set(s->watchdog_flags, 0u) ? "true" : "false",
                flag_set(s->watchdog_flags, 1u) ? "true" : "false",
                flag_set(s->watchdog_flags, 2u) ? "true" : "false",
                s->watchdog_last_block_reason,
                s->watchdog_feed_count,
                s->watchdog_block_count,
                (s->watchdog_last_feed_age_ds == 0xFFFFu) ? -1.0 : ((double)s->watchdog_last_feed_age_ds / 10.0));

    json_append(buf, cap, &off,
                "\"rtos\":{\"heap_free_B\":%lu,\"heap_min_B\":%lu,"
                "\"stack_warn_mask\":%u,\"min_stack_highwater_words\":%u,"
                "\"flags\":%u,\"fault\":%s,\"stack_warn\":%s,\"heap_warn\":%s},",
                (unsigned long)s->rtos_heap_free_div16 * 16UL,
                (unsigned long)s->rtos_heap_min_div16 * 16UL,
                s->rtos_stack_warn_mask,
                s->rtos_min_stack_high_water_words,
                s->rtos_flags,
                flag_set(s->rtos_flags, 0u) ? "true" : "false",
                flag_set(s->rtos_flags, 1u) ? "true" : "false",
                flag_set(s->rtos_flags, 2u) ? "true" : "false");

    json_append(buf, cap, &off,
                "\"estimator\":{\"last_rx_ms\":%lu,\"active_index\":%u,"
                "\"flags\":%u,\"soc_pct\":%.2f,"
                "\"innovation_mV\":%d,\"r0_mOhm\":%.2f},",
                (unsigned long)s->estimator_last_rx_ms,
                s->estimator_active_index,
                s->estimator_flags,
                (double)s->estimator_soc_centi_pct / 100.0,
                (int)s->estimator_innovation_mV,
                (double)s->estimator_r0_0p01_mohm / 100.0);

    json_append(buf, cap, &off,
                "\"power\":{\"stale\":%s,\"dcl_A\":%.1f,\"dcl_W\":%lu,"
                "\"dcl_flags\":%u,\"dcl_binding\":%u,\"dcl_segment\":%u,"
                "\"ccl_A\":%.1f,\"ccl_W\":%lu,\"ccl_flags\":%u,"
                "\"ccl_binding\":%u,\"ccl_segment\":%u,"
                "\"discharge_0p1s_A\":%u,\"discharge_10s_A\":%u,"
                "\"discharge_30s_A\":%u,\"charge_0p1s_A\":%u,"
                "\"charge_10s_A\":%u,\"charge_30s_A\":%u,"
                "\"binding_metadata_fresh\":%s,"
                "\"discharge_binding_by_horizon\":[%u,%u,%u],"
                "\"charge_binding_by_horizon\":[%u,%u,%u],"
                "\"discharge_segment_by_horizon\":[%u,%u,%u],"
                "\"charge_segment_by_horizon\":[%u,%u,%u],"
                "\"capacity_soh_pct\":%u,\"capacity_lower_pct\":%u,"
                "\"resistance_growth_upper_pct\":%u,\"combined_soh_pct\":%u,"
                "\"capacity_confidence_pct\":%u,"
                "\"resistance_confidence_pct\":%u,"
                "\"capacity_valid\":%s,\"resistance_valid\":%s,"
                "\"mission_profile\":%u,\"mission_horizon\":%u,"
                "\"mission_fallback\":%s,\"limp_latched\":%s,"
                "\"fuse_utilization_pct\":%u,"
                "\"fuse_authority_valid\":%s,"
                "\"minimum_core_temp_C\":%d,"
                "\"thermal_energy_to_target_Wh\":%.1f,"
                "\"thermal_ready\":%s,"
                "\"r0_bootstrap_progress_pct\":%u,"
                "\"crc_errors\":%lu,\"counter_errors\":%lu,"
                "\"version_errors\":%lu},",
                power_stale ? "true" : "false",
                (double)s->dcl_current_dA / 10.0,
                (unsigned long)s->dcl_power_10W * 10UL,
                s->dcl_flags,
                s->dcl_binding,
                s->dcl_limiting_segment,
                (double)s->ccl_current_dA / 10.0,
                (unsigned long)s->ccl_power_10W * 10UL,
                s->ccl_flags,
                s->ccl_binding,
                s->ccl_limiting_segment,
                s->envelope_discharge_a[0],
                s->envelope_discharge_a[1],
                s->envelope_discharge_a[2],
                s->envelope_charge_a[0],
                s->envelope_charge_a[1],
                s->envelope_charge_a[2],
                binding_metadata_fresh ? "true" : "false",
                s->envelope_discharge_binding[0],
                s->envelope_discharge_binding[1],
                s->envelope_discharge_binding[2],
                s->envelope_charge_binding[0],
                s->envelope_charge_binding[1],
                s->envelope_charge_binding[2],
                s->envelope_discharge_segment[0],
                s->envelope_discharge_segment[1],
                s->envelope_discharge_segment[2],
                s->envelope_charge_segment[0],
                s->envelope_charge_segment[1],
                s->envelope_charge_segment[2],
                s->capacity_soh_pct,
                s->capacity_soh_lower_pct,
                s->resistance_growth_upper_pct,
                s->combined_soh_pct,
                s->capacity_confidence_pct,
                s->resistance_confidence_pct,
                s->capacity_soh_valid ? "true" : "false",
                s->resistance_soh_valid ? "true" : "false",
                s->mission_profile,
                s->mission_horizon_index,
                s->mission_fallback ? "true" : "false",
                s->limp_latched ? "true" : "false",
                s->fuse_utilization_pct,
                s->fuse_authority_valid ? "true" : "false",
                (int)s->minimum_core_temp_c,
                (double)s->thermal_energy_to_target_dWh / 10.0,
                s->thermal_ready ? "true" : "false",
                s->r0_bootstrap_progress_pct,
                (unsigned long)s->power_crc_error_count,
                (unsigned long)s->power_counter_error_count,
                (unsigned long)s->power_version_error_count);

    json_append(buf, cap, &off,
                "\"hil\":{\"last_rx_ms\":%lu,\"flags\":%u,"
                "\"pack_voltage_V\":%.2f,\"current_A\":%.2f,"
                "\"surface_temp_C\":%.2f,\"soc_pct\":%.2f,"
                "\"core_temp_C\":%.2f,\"min_cell_mV\":%u,"
                "\"max_cell_mV\":%u,\"max_temp_C\":%.2f,"
                "\"avg_temp_C\":%.2f,\"meas_counter\":%u,"
                "\"truth_counter\":%u,\"plant_step\":%lu},",
                (unsigned long)s->hil_last_rx_ms,
                s->hil_flags,
                (double)s->hil_pack_voltage_cV / 100.0,
                (double)s->hil_current_cA / 100.0,
                (double)s->hil_surface_temp_cC / 100.0,
                (double)s->hil_soc_centi_pct / 100.0,
                (double)s->hil_core_temp_cC / 100.0,
                s->hil_min_cell_mv,
                s->hil_max_cell_mv,
                (double)s->hil_max_temp_cC / 100.0,
                (double)s->hil_avg_temp_cC / 100.0,
                s->hil_meas_counter,
                s->hil_truth_counter,
                (unsigned long)s->hil_plant_step);

    append_cell_array(buf, cap, &off, s);
    json_append(buf, cap, &off, ",");
    append_temp_array(buf, cap, &off, s);
    json_append(buf, cap, &off, ",");
    append_mask_array_u16(buf, cap, &off, "voltage_usable_mask", s->voltage_usable_mask);
    json_append(buf, cap, &off, ",");
    append_mask_array_u16(buf, cap, &off, "voltage_pec_mask", s->voltage_pec_mask);
    json_append(buf, cap, &off, ",");
    append_mask_array_u24(buf, cap, &off, "temp_usable_mask", s->temp_usable_mask);
    json_append(buf, cap, &off, ",");
    append_mask_array_u24(buf, cap, &off, "temp_invalid_mask", s->temp_invalid_mask);
    json_append(buf, cap, &off, ",");
    append_mask_array_u16(buf, cap, &off, "voltage_jump_mask", s->voltage_jump_mask);
    json_append(buf, cap, &off, ",");
    append_mask_array_u16(buf, cap, &off, "voltage_stuck_mask", s->voltage_stuck_mask);
    json_append(buf, cap, &off, ",");
    append_mask_array_u24(buf, cap, &off, "temp_open_mask", s->temp_open_mask);
    json_append(buf, cap, &off, ",");
    append_mask_array_u24(buf, cap, &off, "temp_short_mask", s->temp_short_mask);
    json_append(buf, cap, &off, ",");
    append_mask_array_u24(buf, cap, &off, "temp_jump_mask", s->temp_jump_mask);
    json_append(buf, cap, &off, ",");
    append_mask_array_u24(buf, cap, &off, "temp_rate_rise_mask", s->temp_rate_rise_mask);
    json_append(buf, cap, &off, "}");
}

static esp_err_t api_state_handler(httpd_req_t *req)
{
    char *json = malloc(DASH_JSON_BUF_BYTES);
    if(json == NULL)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no json buffer");
        return ESP_FAIL;
    }

    ams_dash_state_t snapshot;
    xSemaphoreTake(g_state_lock, portMAX_DELAY);
    snapshot = g_state;
    xSemaphoreGive(g_state_lock);

    build_json_state(json, DASH_JSON_BUF_BYTES, &snapshot);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t err = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    free(json);
    return err;
}

static esp_err_t csv_handler(httpd_req_t *req)
{
    ams_dash_state_t s;
    xSemaphoreTake(g_state_lock, portMAX_DELAY);
    s = g_state;
    xSemaphoreGive(g_state_lock);

    char line[1800];
    int n = snprintf(line,
                     sizeof(line),
                     "rx_frames,stale,bms_ok,state,pack_V,current_A,min_cell_mV,max_cell_mV,max_temp_C,min_temp_C,temp_filtered_max_C,temp_max_rate_C_s,fan_command_percent,fan_reason,temp_diag_flags,fan_diag_flags,temp_valid,voltage_valid,current_valid,smb_errors,smb_pec_fail_mask,heartbeat_stale_mask,heartbeat_safety_stale_mask,task_flags,can_error_code,can_busoff_count,can_recover_count,panic_reason,wdg_block_reason,estimator_soc_pct,hil_flags,current_adc_high,current_adc_low,current_adc_flags,current_zero_cal_count,charger_read_current_A,charger_disable_mask,charger_tx_fail_count,rtos_heap_free_B,rtos_heap_min_B,rtos_stack_warn_mask,rtos_flags,power_stale,dcl_A,dcl_W,dcl_flags,ccl_A,ccl_W,ccl_flags,capacity_soh_pct,capacity_lower_pct,resistance_growth_upper_pct,combined_soh_pct,capacity_confidence_pct,resistance_confidence_pct,power_crc_errors,power_counter_errors,power_version_errors\n"
                     "%lu,%d,%d,%u,%.1f,%.1f,%u,%u,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%d,%d,%d,%u,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%.2f,%u,%u,%u,%u,%u,%.1f,%u,%u,%lu,%lu,%u,%u,%d,%.1f,%lu,%u,%.1f,%lu,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu\n",
                     (unsigned long)s.rx_frames,
                     ams_dash_data_stale(&s, (uint32_t)esp_log_timestamp(), DASH_STALE_TIMEOUT_MS) ? 1 : 0,
                     flag_set(s.status_flags, 0u) ? 1 : 0,
                     s.state,
                     (double)s.pack_voltage_dV / 10.0,
                     (double)s.current_dA / 10.0,
                     s.min_cell_mv,
                     s.max_cell_mv,
                     (double)s.max_temp_dC / 10.0,
                     (double)s.min_temp_dC / 10.0,
                     (double)s.temp_filtered_max_dC / 10.0,
                     (double)s.temp_max_rate_dC_per_s / 10.0,
                     s.fan_command_percent,
                     s.fan_control_reason,
                     s.temp_diag_flags,
                     s.fan_diag_flags,
                     flag_set(s.validity_flags, 2u) ? 1 : 0,
                     flag_set(s.validity_flags, 0u) ? 1 : 0,
                     flag_set(s.validity_flags, 1u) ? 1 : 0,
                     s.adbms6830_error_count,
                     s.adbms6830_pec_fail_mask,
                     s.heartbeat_stale_mask,
                     s.heartbeat_safety_stale_mask,
                     s.task_health_flags,
                     (unsigned long)s.can_error_code,
                     s.can_busoff_count,
                     s.can_recover_count,
                     s.safety_last_panic_reason,
                     s.watchdog_last_block_reason,
                     (double)s.estimator_soc_centi_pct / 100.0,
                     s.hil_flags,
                     s.current_adc_high_count,
                     s.current_adc_low_count,
                     s.current_adc_flags,
                     s.current_zero_cal_count,
                     (double)s.charger_read_current_dA / 10.0,
                     s.charger_disable_reason_mask,
                     s.charger_tx_fail_count,
                     (unsigned long)s.rtos_heap_free_div16 * 16UL,
                     (unsigned long)s.rtos_heap_min_div16 * 16UL,
                     s.rtos_stack_warn_mask,
                     s.rtos_flags,
                     ams_dash_power_data_stale(
                         &s, (uint32_t)esp_log_timestamp(),
                         DASH_POWER_STALE_MS) ? 1 : 0,
                     (double)s.dcl_current_dA / 10.0,
                     (unsigned long)s.dcl_power_10W * 10UL,
                     s.dcl_flags,
                     (double)s.ccl_current_dA / 10.0,
                     (unsigned long)s.ccl_power_10W * 10UL,
                     s.ccl_flags,
                     s.capacity_soh_pct,
                     s.capacity_soh_lower_pct,
                     s.resistance_growth_upper_pct,
                     s.combined_soh_pct,
                     s.capacity_confidence_pct,
                     s.resistance_confidence_pct,
                     (unsigned long)s.power_crc_error_count,
                     (unsigned long)s.power_counter_error_count,
                     (unsigned long)s.power_version_error_count);

    if(n < 0)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "csv format failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, line, HTTPD_RESP_USE_STRLEN);
}

static const char INDEX_HTML[] =
"<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>DER26 AMS Dashboard</title><style>"
":root{font-family:system-ui,-apple-system,Segoe UI,sans-serif;color:#e9eef5;background:#0f141b}"
"body{margin:0;padding:18px}.top{display:flex;gap:12px;align-items:center;justify-content:space-between}"
"h1{font-size:22px;margin:0}.pill{padding:5px 10px;border-radius:999px;background:#26313f}"
".bad{background:#70232a}.ok{background:#1f5c3a}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px;margin-top:14px}"
".card{border:1px solid #2a3544;border-radius:8px;padding:12px;background:#151c25}.label{color:#9aa9ba;font-size:12px}.val{font-size:26px;font-weight:700;margin-top:2px}"
"table{border-collapse:collapse;width:100%;font-size:12px}td,th{border-bottom:1px solid #2a3544;padding:4px;text-align:right}th:first-child,td:first-child{text-align:left}"
"a{color:#8cc7ff}</style></head><body><div class='top'><h1>DER26 AMS Dashboard</h1><span id='stale' class='pill bad'>waiting</span></div>"
"<div class='grid'>"
"<div class='card'><div class='label'>BMS OK</div><div id='bms' class='val'>-</div></div>"
"<div class='card'><div class='label'>Pack Voltage</div><div id='packv' class='val'>-</div></div>"
"<div class='card'><div class='label'>Current</div><div id='cur' class='val'>-</div></div>"
"<div class='card'><div class='label'>Cell Min / Max</div><div id='cells' class='val'>-</div></div>"
"<div class='card'><div class='label'>Temp Min / Max</div><div id='temps' class='val'>-</div></div>"
"<div class='card'><div class='label'>Fan</div><div id='fan' class='val'>-</div></div>"
"<div class='card'><div class='label'>DCL (1 s)</div><div id='dcl' class='val'>-</div></div>"
"<div class='card'><div class='label'>CCL (1 s)</div><div id='ccl' class='val'>-</div></div>"
"<div class='card'><div class='label'>SoH lower bound</div><div id='soh' class='val'>-</div></div>"
"</div><div class='grid'>"
"<div class='card'><div class='label'>Fault Reasons</div><pre id='faults'></pre></div>"
"<div class='card'><div class='label'>ADBMS Link</div><pre id='adbms'></pre></div>"
"<div class='card'><div class='label'>Health</div><pre id='health'></pre></div>"
"<div class='card'><div class='label'>Charger</div><pre id='charger'></pre></div>"
"<div class='card'><div class='label'>Current ADC</div><pre id='currentadc'></pre></div>"
"<div class='card'><div class='label'>Tasks</div><pre id='tasks'></pre></div>"
"<div class='card'><div class='label'>CAN</div><pre id='can'></pre></div>"
"<div class='card'><div class='label'>Safety</div><pre id='safety'></pre></div>"
"<div class='card'><div class='label'>Watchdog</div><pre id='watchdog'></pre></div>"
"<div class='card'><div class='label'>RTOS</div><pre id='rtos'></pre></div>"
"<div class='card'><div class='label'>Estimator</div><pre id='estimator'></pre></div>"
"<div class='card'><div class='label'>Power / SoH</div><pre id='power'></pre></div>"
"<div class='card'><div class='label'>HIL</div><pre id='hil'></pre></div>"
"</div><div class='card'><div class='label'>Browser Log</div><div id='logcount'>0 samples</div>"
"<button onclick='downloadLog()'>Download CSV log</button> <button onclick='rows=[];drawLogCount()'>Clear</button></div>"
"<p><a href='/api/state'>JSON</a> | <a href='/api/snapshot.csv'>CSV snapshot</a></p>"
"<script>"
"let rows=[];"
"function drawLogCount(){document.getElementById('logcount').textContent=rows.length+' samples'}"
"function csvCell(x){return String(x).replace(/\"/g,'\"\"')}"
"function addRow(s){rows.push([Date.now(),s.stale,s.heartbeat.bms_ok,s.heartbeat.state,s.pack.voltage_V,s.pack.current_A,s.pack.min_cell_mV,s.pack.max_cell_mV,s.temp.min_C,s.temp.max_C,s.temp.fan_command_percent,s.health.voltage_usable,s.health.temp_usable,s.faults.voltage_latched,s.faults.temp_latched,s.faults.current_latched,s.can.busoff_count,s.can.recover_count,s.safety.last_panic_reason,s.watchdog.last_block_reason,s.rtos.heap_free_B,s.rtos.heap_min_B,s.rtos.stack_warn_mask,s.rtos.flags,s.estimator.soc_pct,s.power.stale,s.power.dcl_A,s.power.ccl_A,s.power.capacity_lower_pct,s.power.resistance_growth_upper_pct,s.power.crc_errors,s.power.counter_errors,s.hil.flags].map(csvCell));if(rows.length>7200)rows.shift();drawLogCount()}"
"function downloadLog(){let h='epoch_ms,stale,bms_ok,state,pack_V,current_A,min_cell_mV,max_cell_mV,min_temp_C,max_temp_C,fan_percent,voltage_usable,temp_usable,voltage_latched,temp_latched,current_latched,can_busoff_count,can_recover_count,panic_reason,wdg_block_reason,rtos_heap_free_B,rtos_heap_min_B,rtos_stack_warn_mask,rtos_flags,estimator_soc_pct,power_stale,dcl_A,ccl_A,capacity_lower_pct,resistance_growth_upper_pct,power_crc_errors,power_counter_errors,hil_flags\\n';let blob=new Blob([h+rows.map(r=>r.join(',')).join('\\n')+'\\n'],{type:'text/csv'});let a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='der26_ams_dashboard_log.csv';a.click();URL.revokeObjectURL(a.href)}"
"function f(x,d=1){return Number(x).toFixed(d)}"
"async function tick(){try{let r=await fetch('/api/state');let s=await r.json();"
"let stale=document.getElementById('stale');stale.textContent=s.stale?'STALE':'LIVE';stale.className='pill '+(s.stale?'bad':'ok');"
"document.getElementById('bms').textContent=s.heartbeat.bms_ok?'OK':'LOW';"
"document.getElementById('packv').textContent=f(s.pack.voltage_V)+' V';"
"document.getElementById('cur').textContent=f(s.pack.current_A)+' A';"
"document.getElementById('cells').textContent=s.pack.min_cell_mV+' / '+s.pack.max_cell_mV+' mV';"
"document.getElementById('temps').textContent=f(s.temp.min_C)+' / '+f(s.temp.max_C)+' C';"
"document.getElementById('fan').textContent=s.temp.fan_command_percent+'% r'+s.temp.fan_reason;"
"document.getElementById('dcl').textContent=s.power.stale?'STALE':f(s.power.dcl_A)+' A';"
"document.getElementById('ccl').textContent=s.power.stale?'STALE':f(s.power.ccl_A)+' A';"
"document.getElementById('soh').textContent=s.power.capacity_lower_pct+'%';"
"document.getElementById('faults').textContent=JSON.stringify(s.faults,null,2);"
"document.getElementById('adbms').textContent=JSON.stringify(s.adbms,null,2);"
"document.getElementById('health').textContent=JSON.stringify(s.health,null,2);"
"document.getElementById('charger').textContent=JSON.stringify(s.charger,null,2);"
"document.getElementById('currentadc').textContent=JSON.stringify(s.current_adc,null,2);"
"document.getElementById('tasks').textContent=JSON.stringify(s.tasks,null,2);"
"document.getElementById('can').textContent=JSON.stringify(s.can,null,2);"
"document.getElementById('safety').textContent=JSON.stringify(s.safety,null,2);"
"document.getElementById('watchdog').textContent=JSON.stringify(s.watchdog,null,2);"
"document.getElementById('rtos').textContent=JSON.stringify(s.rtos,null,2);"
"document.getElementById('estimator').textContent=JSON.stringify(s.estimator,null,2);"
"document.getElementById('power').textContent=JSON.stringify(s.power,null,2);"
"document.getElementById('hil').textContent=JSON.stringify(s.hil,null,2);"
"addRow(s);"
"}catch(e){document.getElementById('stale').textContent='offline';document.getElementById('stale').className='pill bad';}}"
"setInterval(tick,500);tick();</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    const httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler };
    const httpd_uri_t state_uri = { .uri = "/api/state", .method = HTTP_GET, .handler = api_state_handler };
    const httpd_uri_t csv_uri = { .uri = "/api/snapshot.csv", .method = HTTP_GET, .handler = csv_handler };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &state_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &csv_uri));
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = DASH_WIFI_SSID,
            .ssid_len = sizeof(DASH_WIFI_SSID) - 1u,
            .channel = DASH_WIFI_CHANNEL,
            .password = DASH_WIFI_PASS,
            .max_connection = DASH_WIFI_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if(strlen(DASH_WIFI_PASS) == 0u)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: SSID=%s password=%s", DASH_WIFI_SSID, DASH_WIFI_PASS);
    ESP_LOGI(TAG, "Open http://192.168.4.1/");
}

static void can_rx_task(void *arg)
{
    (void)arg;

    for(;;)
    {
        mcp2515_frame_t raw;
        esp_err_t err = mcp2515_read_frame(&raw);
        if(err == ESP_OK)
        {
            ams_can_frame_t frame = {
                .id = raw.id,
                .extended = raw.extended,
                .dlc = raw.dlc,
            };
            memcpy(frame.data, raw.data, sizeof(frame.data));

            uint32_t now_ms = (uint32_t)esp_log_timestamp();
            xSemaphoreTake(g_state_lock, portMAX_DELAY);
            (void)ams_dash_decode_frame(&g_state, &frame, now_ms);
            xSemaphoreGive(g_state_lock);
        }
        else if(err != ESP_ERR_NOT_FOUND)
        {
            ESP_LOGW(TAG, "CAN read error: %d", err);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

void app_main(void)
{
    esp_err_t nvs_err = nvs_flash_init();
    if((nvs_err == ESP_ERR_NVS_NO_FREE_PAGES) || (nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    ams_dash_state_init(&g_state);
    g_state_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(g_state_lock == NULL ? ESP_FAIL : ESP_OK);

    ESP_ERROR_CHECK(mcp2515_init());
    wifi_init_softap();
    start_http_server();

    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "AMS dashboard/logger running");
}
