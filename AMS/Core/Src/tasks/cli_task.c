/**
* @file cli_task.c
* @author Cole Bardin (cab572@drexel.edu)
* @author Mahad Faisal (major firmware updates, 2026)
* @brief
* @version 0.1
* @date 2023-10-24
*
* @copyright Copyright (c) 2023
*
*/

#include "tasks/cli_task.h"
#include "tasks/adbms_task.h"
#include "main.h"
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ext_drivers/cli.h"
#include "ext_drivers/ams_rtos_diag.h"
#include "ext_drivers/thermistor_model.h"

/**
* @brief Actual CLI task function
*
* @param arg App_data struct pointer converted to void pointer
*/
void cli_task_fn(void *arg);
static StaticTask_t cli_task_tcb;
static StackType_t cli_task_stack[AMS_STACK_CLI_WORDS];
static TaskHandle_t cli_task_handle = NULL;

int cli_handle_cmd(int argc, char *argv[]);
int cmd_not_found(int argc, char *argv[]);

int help(int argc, char *argv[]);
int get_status(int argc, char *argv[]);
int get_faults(int argc, char *argv[]);
int get_version(int argc, char *argv[]);
int get_voltage(int argc, char *argv[]);
int get_temperature(int argc, char *argv[]);
int get_temperature_sensor(int argc, char *argv[]);
int get_temp_bus_debug(int argc, char *argv[]);
int get_fan_diag(int argc, char *argv[]);


int get_current(int argc, char *argv[]);
int get_estimator_diag(int argc, char *argv[]);
int get_power_diag(int argc, char *argv[]);
int get_charger(int argc, char *argv[]);
int get_can_diag(int argc, char *argv[]);
int watchdog_control(int argc, char *argv[]);
int get_rtos_diag(int argc, char *argv[]);
int get_uart_diag(int argc, char *argv[]);
int get_spi_debug(int argc, char *argv[]);
int get_apm_debug(int argc, char *argv[]);
int get_bringup(int argc, char *argv[]);
int bmsok_control(int argc, char *argv[]);
int balance_control(int argc, char *argv[]);
int set_state(int argc, char *argv[]);
int cause_fault(int argc, char *argv[]);

char outline[CLI_LINESZ];
app_data_t *data;
cli_device_t *cli;
static adbms_string cli_adbms_scope_default_string = ACCUMULATOR_SMB_STRING;
static adbms6830_scope_mode_t cli_adbms_scope_default_mode = ADBMS6830_SCOPE_READ;
static uint16_t cli_adbms_scope_default_repeat = 20u;
static uint8_t cli_adbms_scope_preset_index = 0u;

/* Snapshot the bounded CFGB audit ring under the ADBMS owner mutex, then print
 * after releasing it so UART output never blocks safety-critical bus traffic. */
static adbms6830_cfgb_write_event_t
    cli_cfgb_write_history_snapshot[ADBMS6830_CFGB_WRITE_HISTORY_DEPTH];

/* Kept in BSS rather than on the 512-word CLI task stack. */
static int32_t cli_srepeat_min_uv[ADBMS6830_MAX_TRACKED_ICS][CELL];
static int32_t cli_srepeat_max_uv[ADBMS6830_MAX_TRACKED_ICS][CELL];
static int64_t cli_srepeat_sum_uv[ADBMS6830_MAX_TRACKED_ICS][CELL];
static uint16_t cli_srepeat_valid_count[ADBMS6830_MAX_TRACKED_ICS][CELL];
static uint32_t cli_srepeat_hal_fail_count[ADBMS6830_MAX_TRACKED_ICS];
static uint32_t cli_srepeat_pec_fail_count[ADBMS6830_MAX_TRACKED_ICS];
static uint32_t cli_srepeat_counter_fail_count[ADBMS6830_MAX_TRACKED_ICS];

/* C-ADC soak state is kept in BSS so a long diagnostic never consumes the
 * deliberately small CLI task stack. */
static int32_t cli_csoak_min_uv[ADBMS6830_MAX_TRACKED_ICS][CELL];
static int32_t cli_csoak_max_uv[ADBMS6830_MAX_TRACKED_ICS][CELL];
static int64_t cli_csoak_sum_uv[ADBMS6830_MAX_TRACKED_ICS][CELL];
static uint16_t cli_csoak_valid_count[ADBMS6830_MAX_TRACKED_ICS][CELL];
static int32_t cli_csoak_prev_uv[ADBMS6830_MAX_TRACKED_ICS][CELL];
static int32_t cli_csoak_max_jump_uv[ADBMS6830_MAX_TRACKED_ICS][CELL];
static bool cli_csoak_prev_valid[ADBMS6830_MAX_TRACKED_ICS][CELL];
static uint32_t cli_csoak_hal_fail_count[ADBMS6830_MAX_TRACKED_ICS];
static uint32_t cli_csoak_pec_fail_count[ADBMS6830_MAX_TRACKED_ICS];
static uint32_t cli_csoak_counter_fail_count[ADBMS6830_MAX_TRACKED_ICS];

/* Guided cell-map verification keeps a known C-ADC baseline in BSS.  It is
 * intentionally single-SMB and operator-driven so it cannot alter safety
 * state or infer mapping from stale data. */
static bool cli_mapcheck_baseline_valid;
static uint16_t cli_mapcheck_baseline_mask;
static int32_t cli_mapcheck_baseline_uv[CELL];
static uint32_t cli_mapcheck_baseline_tick;

static bool cli_parse_scope_repeat(const char *arg, uint16_t *repeat_out);
static bool cli_parse_u16_range(const char *arg, uint16_t minimum,
                                uint16_t maximum, uint16_t *value_out);
static const char *cli_voltage_mode_str(void);
static int cli_print_cadc_dump(adbms6830_driver_t *smb,
                                HAL_StatusTypeDef capture_status);
static int cli_run_cadc_soak(adbms6830_driver_t *smb, uint16_t repeat_count);
static int cli_run_conversion_timing(adbms6830_driver_t *smb,
                                     int argc, char *argv[]);
static int cli_run_config_repeat(adbms6830_driver_t *smb, uint16_t repeat_count);
static int cli_run_raw_dump(adbms6830_driver_t *smb);
static int cli_run_snapshot(adbms6830_driver_t *smb);
static int cli_run_recovery(adbms6830_driver_t *smb, uint16_t idle_ms);
static int cli_print_adbms_fault_classes(const app_data_t *app);
static int cli_print_adbms_lifecycle(const app_data_t *app);
static int cli_print_adbms_ages(const app_data_t *app);
static int cli_print_adbms_authority(const app_data_t *app);
static int cli_print_adbms_events(void);
static int cli_print_adbms_lockdiag(const app_data_t *app);
static int cli_handle_adbms_injection(int argc, char *argv[]);
static int cli_handle_mapcheck(adbms6830_driver_t *smb, int argc, char *argv[]);
static int cli_run_cadc_stream(int argc, char *argv[]);
static int cli_run_temperature_emulator(int argc, char *argv[]);
static int get_temperature_sensor_locked(int argc, char *argv[]);
static int get_temp_bus_debug_locked(int argc, char *argv[]);
static int get_spi_debug_locked(int argc, char *argv[]);
static int get_apm_debug_locked(int argc, char *argv[]);

static int cli_clear_balance_recorded(void)
{
    int result;

    if(data == NULL)
    {
        return -1;
    }

    adbms_spi_lock();
    result = accumulator_clear_balance(&data->acc);
    adbms_spi_unlock();
    (void)adbms_record_balance_write_result(data, result);
    return result;
}

static int cli_service_action_refused(const char *action)
{
#if AMS_ENABLE_SERVICE_CLI
    (void)action;
    return 0;
#else
    snprintf(outline, CLI_LINESZ,
             "%s refused: service CLI is compiled out (use AMS_HW_BRINGUP=1 or AMS_ENABLE_SERVICE_CLI=1 on a controlled bench)",
             (action != NULL) ? action : "service action");
    return cli_printline(cli, outline);
#endif
}
command_t cmds[] =
{
	{"help", &help, "print help menu"},
	{"status", &get_status, "compact bring-up status banner"},
	{"fault", &get_faults, "gets the faults of the system"},
	{"ver", &get_version, "gets the firmware version"},
	{"volt", &get_voltage, "gets cell voltages for all SMBs"},
	{"temp", &get_temperature, "gets sensor temperatures for all SMBs"},
	{"tempsns", &get_temperature_sensor, "one explicit mux attempt: tempsns <ic> <sensor 0-23>"},
	{"tempbus", &get_temp_bus_debug, "temperature bus debug: tempbus [idle|scan]"},
	{"fan", &get_fan_diag, "fan control telemetry: fan"},
	{"current", &get_current, "gets current sensor raw counts/voltages/status"},
		{"estimator", &get_estimator_diag, "estimator timing and advisory R0/SoH confidence"},
		{"power", &get_power_diag, "predictive SoP/SoH limits, bindings and freshness"},
	{"charger", &get_charger, "gets charger CAN command/status/debug state"},
	{"can", &get_can_diag, "CAN diagnostics: can [diag|recover]"},
	{"wdg", &watchdog_control, "watchdog diagnostics/control: wdg [status|enable]"},
	{"rtos", &get_rtos_diag, "RTOS stack/heap diagnostics: rtos"},
	{"uart", &get_uart_diag, "CLI UART diagnostics/recovery: uart [status|recover|clear]"},
	{"spi", &get_spi_debug, "ADBMS6830: spi [status|snapshot|faults|lifecycle|ages|authority|events|lockdiag|tempemu|cdump|csoak|cstream|mapcheck|inject|timing|cfgrepeat|recovery|rawdump]"},
	{"apm", &get_apm_debug, "ADBMS2950/APM: apm help (status/refup/config/flags/raw/sample/redundant/eeprom/scope/recover)"},
	{"bringup", &get_bringup, "bench bring-up summaries: bringup [help|board|adbms6830|apm2950|charger-lv|charger-battery|ready|snapshot|evidence]"},
	{"bmsok", &bmsok_control, "BMS_OK control: bmsok [status|release|inhibit]"},
	{"balance", &balance_control, "balancing control: balance [status|shadow|inhibit|release|clear]"},
	{"state", &set_state, "gets or sets the AMS state [charge|discharge]"},
	{"cause_fault", &cause_fault, "cause BMS fault for tech"},
};

char *state_str[] =
{
    "NULL",
    "start",
    "charge",
    "discharge",
    "balance",
    "error"
};

static const char *ams_state_to_str(state_t state)
{
    size_t count = sizeof(state_str) / sizeof(state_str[0]);

    if((state < 0) || ((size_t)state >= count))
    {
        return "invalid";
    }

    return state_str[state];
}


static uint8_t smb_ic_count(const adbms6830_driver_t *smb)
{
    if((smb == NULL) || (smb->num_ics <= 0))
    {
        return 0u;
    }

    return (smb->num_ics > NSMBS) ? (uint8_t)NSMBS : (uint8_t)smb->num_ics;
}

static bool cli_parse_int_range(const char *arg, int min_value, int max_value, int *value_out)
{
    char *end = NULL;
    long parsed;

    if((arg == NULL) || (value_out == NULL) || (min_value > max_value))
    {
        return false;
    }

    errno = 0;
    parsed = strtol(arg, &end, 10);
    if((errno == ERANGE) || (end == arg) || (*end != '\0') ||
       (parsed < (long)min_value) || (parsed > (long)max_value))
    {
        return false;
    }

    *value_out = (int)parsed;
    return true;
}

static int cli_scaled_int(float value, int scale)
{
    double scaled;

    if(!isfinite(value) || (scale <= 0))
    {
        return 0;
    }

    scaled = round((double)value * (double)scale);
    if(scaled >= (double)INT_MAX)
    {
        return INT_MAX;
    }
    if(scaled <= (double)INT_MIN)
    {
        return INT_MIN;
    }

    return (int)scaled;
}

static void cli_fixed1(float value, int *whole, int *decimal)
{
    if((whole == NULL) || (decimal == NULL))
    {
        return;
    }

    int scaled = cli_scaled_int(value, 10);
    *whole = scaled / 10;
    *decimal = abs(scaled % 10);
}

static void cli_fixed3(float value, int *whole, int *decimal)
{
    if((whole == NULL) || (decimal == NULL))
    {
        return;
    }

    int scaled = cli_scaled_int(value, 1000);
    *whole = scaled / 1000;
    *decimal = abs(scaled % 1000);
}


static const char *cli_hal_status_str(HAL_StatusTypeDef status)
{
    switch(status)
    {
    case HAL_OK:      return "OK";
    case HAL_ERROR:   return "ERROR";
    case HAL_BUSY:    return "BUSY";
    case HAL_TIMEOUT: return "TIMEOUT";
    default:          return "UNKNOWN";
    }
}

static const char *cli_spi_polarity_str(uint32_t polarity)
{
    return (polarity == SPI_POLARITY_HIGH) ? "HIGH" : "LOW";
}

static const char *cli_spi_phase_str(uint32_t phase)
{
    return (phase == SPI_PHASE_2EDGE) ? "2EDGE" : "1EDGE";
}

static int cli_print_hex_preview(const char *label, const uint8_t *buf, uint16_t len)
{
    char hexbuf[72];
    size_t off = 0u;
    uint16_t count;

    if((label == NULL) || (buf == NULL))
    {
        return cli_printline(cli, "hex preview unavailable");
    }

    count = (len > ADBMS6830_SPI_DEBUG_PREVIEW_BYTES) ? ADBMS6830_SPI_DEBUG_PREVIEW_BYTES : len;
    off += (size_t)snprintf(hexbuf + off, sizeof(hexbuf) - off, "%s", label);
    for(uint16_t i = 0u; (i < count) && (off < sizeof(hexbuf)); i++)
    {
        off += (size_t)snprintf(hexbuf + off, sizeof(hexbuf) - off, " %02X", buf[i]);
    }

    return cli_printline(cli, hexbuf);
}

static bool cli_raw_temp_to_values(int16_t raw, float *voltage_out, float *temp_out)
{
    if((voltage_out == NULL) || (temp_out == NULL))
    {
        return false;
    }

    thermistor_result_t result = thermistor_from_adbms_raw(
        raw, THERMISTOR_NOMINAL_VREG_V);
    *voltage_out = result.divider_voltage_v;
    *temp_out = result.valid ? result.temperature_c : 0.0f;
    return result.valid;
}

TaskHandle_t cli_task_start(app_data_t *data)
{
    if(data == NULL)
    {
        return NULL;
    }

    if(cli_task_handle == NULL)
    {
        cli_task_handle = xTaskCreateStatic(cli_task_fn,
                                            "CLI task",
                                            AMS_STACK_CLI_WORDS,
                                            (void *)data,
                                            CLI_PRIO,
                                            cli_task_stack,
                                            &cli_task_tcb);
    }

    return cli_task_handle;
}

void cli_task_fn(void *arg)
{
    data = (app_data_t *)arg;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    cli_device_t *local_cli = &data->board.cli;
    cli = local_cli;

    uint32_t entry;
    char buf[CLI_LINESZ] = {0};
    char *tokens[MAXTOKS];
    int n;
    int ret = 0;

    snprintf(outline, CLI_LINESZ, "~~~~~~~~~~ DER AMS FW V%d.%d.%d ~~~~~~~~~~", VER_MAJOR, VER_MINOR, VER_BUG);
	cli_printline(local_cli, outline);
    snprintf(outline, CLI_LINESZ,
             "Build:%s estimator_topology:%u service_cli:%d hil_can:%d APM2950:%d inhibit:%d",
             AMS_BUILD_PROFILE_NAME,
             AMS_ESTIMATOR_DEFAULT_TOPOLOGY,
             AMS_ENABLE_SERVICE_CLI,
             AMS_ENABLE_HIL_CAN,
             AMS_ENABLE_APM_2950,
             data->bms_output_inhibit);
    cli_printline(local_cli, outline);
#if AMS_ENABLE_BENCH_PASSIVE_RING_ESTIMATOR
    cli_printline(local_cli,
        "PASSIVE RING OBSERVER: open-ring zero-current/25C fallback; SoH/SoP authority disabled");
#endif
    /* Keep provenance fields on bounded independent lines.  Revision strings
     * are build inputs and may grow; combining all of them in one 128-byte CLI
     * buffer silently truncated exactly the metadata needed to identify a
     * flashed image. */
    snprintf(outline, CLI_LINESZ,
             "Manifest schema:%u commit:%s",
             (unsigned)AMS_BUILD_MANIFEST_SCHEMA,
             AMS_BUILD_GIT_COMMIT);
    cli_printline(local_cli, outline);
    snprintf(outline, CLI_LINESZ,
             "Manifest current:%s CAN:%s",
             AMS_CURRENT_CALIBRATION_REVISION,
             AMS_CAN_CONTRACT_REVISION);
    cli_printline(local_cli, outline);
    snprintf(outline, CLI_LINESZ,
             "Manifest thresholds:%s estimator:%s",
             AMS_THRESHOLD_REVISION,
             AMS_ESTIMATOR_MODEL_REVISION);
    cli_printline(local_cli, outline);
    cli_printline(local_cli, "ADBMS6822 SPI6 expected: mode3 CPOL HIGH CPHA 2EDGE");
	cli_printline(local_cli, "Type 'help' for list of commands");

	for(;;)
	{
		entry = osKernelGetTickCount();

        /* Self-heal USART3 RX if an overrun, debugger halt, transient power
         * disturbance, or failed HAL re-arm left reception disabled. */
        (void)cli_uart_service_rx(local_cli);

		if(local_cli->msg_pending == true)
		{
			size_t len;

			taskENTER_CRITICAL();
			len = cli_bounded_strlen(local_cli->line, CLI_LINESZ - 1u);
			memcpy(buf, local_cli->line, len);
			buf[len] = '\0';
			memset(local_cli->line, 0, sizeof(local_cli->line));
			local_cli->msg_pending = false;
			local_cli->msg_proc++;
			taskEXIT_CRITICAL();

			n = tokenize(buf, tokens, MAXTOKS, " \t");
			ret = cli_handle_cmd(n, tokens);
			data->cli_fault = (ret != 0);
		}
		osDelayUntil(entry + (1000 / CLI_FREQ));
	}
}

int cli_handle_cmd(int argc, char *argv[])
{
	int i;
	int ret = 0;
	bool cmd_found = false;
	int num_cmds = sizeof(cmds) / sizeof(command_t);

    if((argc <= 0) || (argv[0] == NULL))
    {
        return 0;
    }

	for(i = 0; i < num_cmds; i++)
	{
		if(!strncmp(cmds[i].name, argv[0], CLI_LINESZ))
		{
			ret = cmds[i].func(argc, argv);
			cli->msg_valid++;
			cmd_found = true;
			break;
		}
	}
	if(!cmd_found) return cmd_not_found(argc, argv);
	cli->ret = ret;
	return ret;
}

int cmd_not_found(int argc, char *argv[])
{
	int ret = 0;
	snprintf(outline, CLI_LINESZ, "Command not found: \'%s\'", argv[0]);
	ret |= cli_printline(cli, outline);
	ret |= cli_printline(cli, "Type 'help' for list of commands");
	return ret;
}

int help(int argc, char *argv[])
{
	int num_cmds;
	int i;
	int ret = 0;

	ret |= cli_printline(cli, "---------- Help Menu ----------");
	num_cmds = sizeof(cmds) / sizeof(command_t);
	for(i = 0; i < num_cmds; i++)
	{
		snprintf(outline, CLI_LINESZ, "%s - %s", cmds[i].name, cmds[i].desc);
		ret |= cli_printline(cli, outline);
	}
	return ret;
}

int get_status(int argc, char *argv[])
{
    int ret = 0;
    int max_temp_whole;
    int max_temp_decimal;
    SPI_HandleTypeDef *hspi = data->acc.smb.hspi;
    ams_air_monitor_t air_monitor;

    taskENTER_CRITICAL();
    air_monitor = data->air_monitor;
    taskEXIT_CRITICAL();

    cli_fixed1(data->max_temp, &max_temp_whole, &max_temp_decimal);

	snprintf(outline, CLI_LINESZ,
	             "FW v%d.%d.%d build:%s service:%d hil:%d state:%s BMS_OK:%d inhibit:%d ready:%d balance_inhibit:%d blocked:%lu",
	             VER_MAJOR,
	             VER_MINOR,
	             VER_BUG,
	             AMS_BUILD_PROFILE_NAME,
	             AMS_ENABLE_SERVICE_CLI,
	             AMS_ENABLE_HIL_CAN,
	             ams_state_to_str(data->state),
	             data->bms_state,
	             data->bms_output_inhibit,
	             data->bms_supervisor_ready,
	             data->balance_inhibit,
	             (unsigned long)data->bms_output_block_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Voltage authority:%s degraded:%d S_policy:%s",
             cli_voltage_mode_str(),
             data->adbms_voltage_redundancy_degraded,
             data->adbms_voltage_redundancy_degraded ? "diagnostic_only" : "required");
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "ADBMS faults active:0x%04X latched:0x%04X attempted:%d last_scan_ok:%d comm_fail:%lu",
             data->adbms_fault_active_mask,
             data->adbms_fault_latched_mask,
             data->adbms_voltage_scan_attempted,
             data->adbms_last_voltage_scan_ok,
             (unsigned long)data->adbms_comm_failure_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Safety current valid:%d fault:%d voltage valid:%d fault:%d temp valid:%d fault:%d imd valid:%d ok:%d fault:%d hard:%d",
             data->current_valid,
             data->current_fault,
             data->voltage_valid,
             data->voltage_fault,
             data->temp_valid,
             data->temp_fault,
             data->imd_valid,
             data->imd_ok,
             data->imd_fault,
             data->hard_fault);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "AIR control_sense:%d aux_feature:%d cfg:%d command:%d input:%d",
             data->air_state,
             AMS_ENABLE_AIR_AUX_FEEDBACK,
             air_monitor.configuration_valid,
             air_monitor.command_valid,
             air_monitor.feedback_valid);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "AIR phase:%s permit:%d steady:%d pending:%d boot_open:%d",
             ams_air_phase_str(air_monitor.phase),
             air_monitor.permit,
             air_monitor.steady_state_valid,
             air_monitor.transition_pending,
             air_monitor.boot_open_verified);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "AIR AUX pos:%s neg:%s precharge:%s precharge_done:%d",
             ams_air_contact_state_str(air_monitor.pos_aux),
             ams_air_contact_state_str(air_monitor.neg_aux),
             ams_air_contact_state_str(air_monitor.precharge_aux),
             air_monitor.precharge_complete);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "AIR fault:%d latched:%d reason:%s active:0x%04lX latched_mask:0x%04lX",
             air_monitor.fault,
             air_monitor.fault_latched,
             ams_air_fault_reason_str(air_monitor.reason),
             (unsigned long)air_monitor.active_fault_mask,
             (unsigned long)air_monitor.latched_fault_mask);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Cells usable:%u updated:%u stale:%u fan:%d max_temp:%d.%01dC",
             (unsigned)data->voltage_usable_cell_count,
             (unsigned)data->voltage_updated_cell_count,
             (unsigned)data->voltage_stale_cell_count,
             data->fan_state,
             max_temp_whole,
             max_temp_decimal);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Temps usable:%u updated:%u stale:%u invalid:%u warn:%d fanmax:%d chargestop:%d pending:%s %lums reason:%s",
             (unsigned)data->temp_usable_sensor_count,
             (unsigned)data->temp_updated_sensor_count,
             (unsigned)data->temp_stale_sensor_count,
             (unsigned)data->temp_invalid_sensor_count,
             data->temp_warning,
             data->temp_fan_max,
             data->temp_charge_stop,
             temperature_fault_reason_str(data->temp_fault_pending_reason),
             (unsigned long)data->temp_fault_pending_ms,
             temperature_fault_reason_str(data->temp_fault_reason));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Main fuse/HV path authority:%d suspect:%d confirmed:%d reason:%s",
             data->main_fuse_monitor.authority_valid,
             data->main_fuse_monitor.suspect_open,
             data->main_fuse_monitor.confirmed_open,
             ams_main_fuse_monitor_reason_str(data->main_fuse_monitor.reason));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Parallel connection advisory valid:%d target_validated:%d suspect:%d steps:%lu reason:%s",
             data->parallel_connection_observer.advisory_valid,
             data->parallel_connection_observer.target_validated,
             data->parallel_connection_observer.suspect,
             (unsigned long)data->parallel_connection_observer.accepted_step_count,
             ams_parallel_observer_reason_str(data->parallel_connection_observer.reason));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Heartbeat seen:0x%04X stale:0x%04X safety:0x%04X logger:0x%04X fault:%d logger_fault:%d",
             data->heartbeat.seen_mask,
             data->heartbeat.stale_mask,
             data->heartbeat.safety_stale_mask,
             data->heartbeat.logger_stale_mask,
             data->task_heartbeat_fault,
             data->logger_heartbeat_fault);
    ret |= cli_printline(cli, outline);

    if(hspi != NULL)
    {
        snprintf(outline, CLI_LINESZ,
                 "SPI6 CPOL:%s CPHA:%s prescaler:%lu APM2950:%d",
                 cli_spi_polarity_str(hspi->Init.CLKPolarity),
                 cli_spi_phase_str(hspi->Init.CLKPhase),
                 (unsigned long)hspi->Init.BaudRatePrescaler,
                 AMS_ENABLE_APM_2950);
        ret |= cli_printline(cli, outline);
    }
    else
    {
        ret |= cli_printline(cli, "SPI6 handle unavailable");
    }

    ret |= cli_printline(cli, "Bring-up order: spi clear -> spi preset normal -> spi scope -> spi probe -> spi status -> volt -> current -> bmsok release");
    return ret;
}

int get_faults(int argc, char *argv[])
{
	int ret = 0;

    if((argc >= 2) && (argv[1] != NULL))
    {
        if(!strcmp(argv[1], "resetcause"))
        {
            char flags[96];
            ams_safety_format_reset_flags(data->reset_flags, flags, sizeof(flags));
            snprintf(outline, CLI_LINESZ, "reset: %s", flags);
            return cli_printline(cli, outline);
        }

        if(!strcmp(argv[1], "panic"))
        {
            const ams_panic_record_t *panic = ams_safety_panic_record();
            snprintf(outline, CLI_LINESZ,
                     "panic: reason:%s(%lu) count:%lu cfsr:0x%08lX hfsr:0x%08lX mmfar:0x%08lX bfar:0x%08lX",
                     ams_safety_panic_reason_str(panic->panic_reason),
                     (unsigned long)panic->panic_reason,
                     (unsigned long)panic->reset_count,
                     (unsigned long)panic->cfsr,
                     (unsigned long)panic->hfsr,
                     (unsigned long)panic->mmfar,
                     (unsigned long)panic->bfar);
            return cli_printline(cli, outline);
        }

        if(!strcmp(argv[1], "fuse"))
        {
            if((argc >= 3) && (argv[2] != NULL) && !strcmp(argv[2], "clear"))
            {
#if AMS_ENABLE_SERVICE_CLI
                bool cleared;
                uint32_t now = osKernelGetTickCount();

                if(!data->bms_output_inhibit || data->bms_state)
                {
                    return cli_printline(cli,
                        "main-fuse/HV-path clear refused: require BMS_OK inhibited and physically low");
                }

                taskENTER_CRITICAL();
                cleared = ams_main_fuse_monitor_request_clear(
                    &data->main_fuse_monitor,
                    &data->air_monitor_inputs,
                    data->current,
                    data->current_valid,
                    now);
                if(cleared)
                {
                    data->fuse_fault = false;
                }
                taskEXIT_CRITICAL();

                return cli_printline(cli,
                    cleared ?
                        "main-fuse/HV-path plausibility latch cleared under safe OFF/discharged conditions" :
                        "main-fuse/HV-path clear refused: require fresh OFF/SHUTDOWN command, discharged load bus and near-zero current");
#else
                return cli_service_action_refused("main-fuse/HV-path clear");
#endif
            }

            snprintf(outline, CLI_LINESZ,
                     "main-fuse/HV-path authority:%d suspect:%d confirmed:%d latched:%d reason:%s pack:%lumV load:%lumV current:%.2fA",
                     data->main_fuse_monitor.authority_valid,
                     data->main_fuse_monitor.suspect_open,
                     data->main_fuse_monitor.confirmed_open,
                     data->main_fuse_monitor.latched,
                     ams_main_fuse_monitor_reason_str(
                         data->main_fuse_monitor.reason),
                     (unsigned long)data->main_fuse_monitor.pack_mv,
                     (unsigned long)data->main_fuse_monitor.load_mv,
                     (double)data->main_fuse_monitor.current_a);
            ret |= cli_printline(cli, outline);
            ret |= cli_printline(cli,
                "Current hardware has no authoritative AIR auxiliary/load-side voltage adapter; unavailable means no fuse_fault assertion");
            return ret;
        }

        if(!strcmp(argv[1], "log"))
        {
            if((argc >= 3) && (argv[2] != NULL) && !strcmp(argv[2], "clear"))
            {
#if AMS_ENABLE_SERVICE_CLI
                ams_fault_log_clear();
                return cli_printline(cli, "fault log cleared");
#else
                return cli_service_action_refused("fault log clear");
#endif
            }

            const ams_fault_log_t *log = ams_fault_log_get();
            snprintf(outline, CLI_LINESZ,
                     "fault log schema:%u boot:%lu count:%lu write:%lu next:%lu",
                     (unsigned)log->version,
                     (unsigned long)log->boot_sequence,
                     (unsigned long)log->count,
                     (unsigned long)log->write_index,
                     (unsigned long)log->next_sequence);
            ret |= cli_printline(cli, outline);

            for(uint32_t i = 0u; i < log->count; i++)
            {
                uint32_t idx = (log->write_index + AMS_FAULT_LOG_DEPTH - log->count + i) % AMS_FAULT_LOG_DEPTH;
                const ams_fault_log_entry_t *e = &log->entry[idx];
                snprintf(outline, CLI_LINESZ,
                         "%02lu boot:%lu seq:%lu tick:%lu event:%s reason:%u arg0:0x%08lX arg1:0x%08lX",
                         (unsigned long)i,
                         (unsigned long)e->boot_sequence,
                         (unsigned long)e->sequence,
                         (unsigned long)e->tick,
                         ams_fault_log_event_str(e->event),
                         (unsigned)e->reason,
                         (unsigned long)e->arg0,
                         (unsigned long)e->arg1);
                ret |= cli_printline(cli, outline);
            }
            return ret;
        }

#if AMS_FAULT_INJECTION_CLI && AMS_ENABLE_SERVICE_CLI
        if(!strcmp(argv[1], "inject") && (argc >= 3) && (argv[2] != NULL))
        {
            if(!strcmp(argv[2], "hardfault"))
            {
                ams_safety_fault_inject_hardfault();
                return cli_printline(cli, "fault injection hardfault panic recorded; BMS_OK forced low");
            }
            if(!strcmp(argv[2], "busfault"))
            {
                ams_safety_fault_inject_busfault();
                return cli_printline(cli, "fault injection busfault panic recorded; BMS_OK forced low");
            }
            if(!strcmp(argv[2], "canbusoff"))
            {
                data->can_error_code = HAL_CAN_ERROR_BOF;
                data->can_busoff_fault = true;
                data->can_recover_pending = true;
                data->canbus_fault = true;
                data->can_last_error_tick = osKernelGetTickCount();
                data->can_busoff_count++;
                ams_fault_log_event(AMS_FAULT_LOG_CAN_BUS_OFF, 0u, HAL_CAN_ERROR_BOF, data->can_busoff_count);
                return cli_printline(cli, "fault injection CAN bus-off recorded");
            }
        }
#endif

        ret |= cli_printline(cli, "Usage: fault [resetcause|panic|fuse|fuse clear|log|log clear]");
        return ret;
    }

	ret |= cli_printline(cli, "System faults:");
	snprintf(outline, CLI_LINESZ, "hard:   %d", data->hard_fault);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ, "soft:   %d", data->soft_fault);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ, "  cli:    %d", data->cli_fault);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ, " bms:    %d", data->bms_state);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ, "  fan:    %d", data->fan_fault);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ, "  canbus: %d", data->canbus_fault);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ, "  charger: %d", data->charger_fault);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ,
             "  adbms: diag:%d cfg:%d status:%d openwire:%d balance_write:%d failures:%lu",
             data->adbms_diag_fault,
             data->adbms_config_fault,
             data->adbms_status_fault,
             data->adbms_open_wire_fault,
             data->adbms_balance_write_fault,
             (unsigned long)data->adbms_balance_write_fail_count);
	ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "  adbms classes active:0x%04X latched:0x%04X mode:%s degraded:%d scan_attempted:%d last_scan_ok:%d comm_fail:%lu",
             data->adbms_fault_active_mask, data->adbms_fault_latched_mask,
             cli_voltage_mode_str(), data->adbms_voltage_redundancy_degraded,
             data->adbms_voltage_scan_attempted,
             data->adbms_last_voltage_scan_ok,
             (unsigned long)data->adbms_comm_failure_count);
    ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ, "  heartbeat: critical:%d logger:%d seen:0x%04X stale:0x%04X",
             data->task_heartbeat_fault,
             data->logger_heartbeat_fault,
             data->heartbeat_seen_mask,
             data->heartbeat_stale_mask);
	ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ, "  voltage: fault:%d valid:%d warn:%d stop:%d reason:%s",
             data->voltage_fault,
             data->voltage_valid,
             data->voltage_warning,
             data->charge_voltage_stop,
             voltage_fault_reason_str(data->voltage_fault_reason));
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ, "  temp: fault:%d valid:%d warn:%d fanmax:%d stop:%d pending:%s %lums reason:%s latched:%s",
             data->temp_fault,
             data->temp_valid,
             data->temp_warning,
             data->temp_fan_max,
             data->temp_charge_stop,
             temperature_fault_reason_str(data->temp_fault_pending_reason),
             (unsigned long)data->temp_fault_pending_ms,
             temperature_fault_reason_str(data->temp_fault_reason),
             temperature_fault_reason_str(data->temp_fault_latched_reason));
    ret |= cli_printline(cli, outline);
	return ret;
}

int get_voltage(int argc, char *argv[])
{
    int ret = 0;
    adbms6830_driver_t *smb = &data->acc.smb;

    snprintf(outline, CLI_LINESZ,
             "Voltage valid:%d fault:%d read:%d pending:%d streak:%u/%u warn:%d charge_stop:%d reason:%s",
             data->voltage_valid,
             data->voltage_fault,
             data->voltage_read_fault,
             data->voltage_read_fault_pending,
             (unsigned)data->voltage_read_fault_streak,
             (unsigned)VOLTAGE_READ_FAULT_CONFIRM_SCANS,
             data->voltage_warning,
             data->charge_voltage_stop,
             voltage_fault_reason_str(data->voltage_fault_reason));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Cells usable:%u updated:%u stale:%u pec:%u",
             (unsigned)data->acc.usable_voltage_count,
             (unsigned)data->acc.updated_voltage_count,
             (unsigned)data->acc.stale_voltage_count,
             (unsigned)data->acc.pec_fail_cell_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Voltage diag jump:%u stuck:%u max_delta:%umV at S%u/C%u",
             (unsigned)data->voltage_jump_cell_count,
             (unsigned)data->voltage_stuck_cell_count,
             (unsigned)data->voltage_max_delta_mv,
             (unsigned)(data->voltage_max_delta_seg + 1u),
             (unsigned)(data->voltage_max_delta_cell + 1u));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Max S%u C%u:%umV Min S%u C%u:%umV",
             (unsigned)(data->acc.max_voltage_seg + 1u),
             (unsigned)(data->acc.max_voltage_cell + 1u),
             (unsigned)data->acc.max_voltage_mv,
             (unsigned)(data->acc.min_voltage_seg + 1u),
             (unsigned)(data->acc.min_voltage_cell + 1u),
             (unsigned)data->acc.min_voltage_mv);
    ret |= cli_printline(cli, outline);

    for (uint8_t ic = 0; ic < smb_ic_count(smb); ic++)
    {
        snprintf(outline, CLI_LINESZ, "--- SMB %d usable:0x%04x updated:0x%04x stale:0x%04x pec:0x%04x jump:0x%04x stuck:0x%04x ---",
                 ic,
                 data->acc.usable_voltage_mask[ic],
                 data->acc.updated_voltage_mask[ic],
                 data->acc.stale_voltage_mask[ic],
                 data->acc.pec_fail_voltage_mask[ic],
                 data->acc.voltage_jump_mask[ic],
                 data->acc.voltage_stuck_mask[ic]);
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "    SW_OV:0x%04X SW_UV:0x%04X HW_OV:0x%04X HW_UV:0x%04X disagree:0x%04X",
                 data->voltage_fault_state.software_ov_mask[ic],
                 data->voltage_fault_state.software_uv_mask[ic],
                 data->voltage_fault_state.hardware_ov_mask[ic],
                 data->voltage_fault_state.hardware_uv_mask[ic],
                 data->voltage_fault_state.hardware_disagreement_mask[ic]);
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "    sense-path-open:0x%04X sticky:0x%04X bond_candidate:0x%04X bond_suspect:0x%04X",
                 data->adbms_sense_path_open_mask[ic],
                 data->adbms_sense_path_open_sticky_mask[ic],
                 data->parallel_connection_observer.candidate_mask[ic],
                 data->parallel_connection_observer.suspect_mask[ic]);
        ret |= cli_printline(cli, outline);

        for (int cell = 0; cell < NCELLS; cell++)
        {
            uint16_t mv = accumulator_cell_voltage_mv(&data->acc, ic, (uint8_t)cell);
            if(mv != 0u)
            {
                snprintf(outline, CLI_LINESZ, "  C%-2d: %u.%03u V",
                         cell + 1,
                         (unsigned)(mv / 1000u),
                         (unsigned)(mv % 1000u));
            }
            else
            {
                snprintf(outline, CLI_LINESZ, "  C%-2d: unavailable", cell + 1);
            }
            ret |= cli_printline(cli, outline);
        }
    }
    return ret;
}


int get_temperature(int argc, char *argv[])
{
    int ret = 0;
    adbms6830_driver_t *smb = &data->acc.smb;

    snprintf(outline, CLI_LINESZ,
             "Temp HW pullups_validated:%d auto_scan:%d (Rev5 100-ohm pull-ups require hardware rework; timing cannot compensate)",
             AMS_TEMP_PULLUPS_TARGET_VALIDATED,
             AMS_ENABLE_AUTO_TEMP_MUX_SCAN);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Temp valid:%d fault:%d warn:%d fanmax:%d stop:%d pending:%s %lums reason:%s",
             data->temp_valid,
             data->temp_fault,
             data->temp_warning,
             data->temp_fan_max,
             data->temp_charge_stop,
             temperature_fault_reason_str(data->temp_fault_pending_reason),
             (unsigned long)data->temp_fault_pending_ms,
             temperature_fault_reason_str(data->temp_fault_reason));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Temp counts usable:%u updated:%u stale:%u invalid:%u open:%u short:%u jump:%u rate:%u",
             (unsigned)data->temp_usable_sensor_count,
             (unsigned)data->temp_updated_sensor_count,
             (unsigned)data->temp_stale_sensor_count,
             (unsigned)data->temp_invalid_sensor_count,
             (unsigned)data->temp_open_sensor_count,
             (unsigned)data->temp_short_sensor_count,
             (unsigned)data->temp_jump_sensor_count,
             (unsigned)data->temp_rate_rise_sensor_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Temp filtered max:%.1f avg:%.1f max_rate:%.1fC/s at SMB%u/S%u",
             (double)data->temp_filtered_max,
             (double)data->temp_filtered_avg,
             (double)data->temp_max_rate_c_per_s,
             (unsigned)data->temp_max_rate_seg,
             (unsigned)data->temp_max_rate_sensor);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Temperature max SMB%u/S%u min SMB%u/S%u",
             (unsigned)data->max_temp_seg,
             (unsigned)data->max_temp_sensor,
             (unsigned)data->min_temp_seg,
             (unsigned)data->min_temp_sensor);
    ret |= cli_printline(cli, outline);

    for (uint8_t ic = 0; ic < smb_ic_count(smb); ic++)
    {

        snprintf(outline, CLI_LINESZ, "--- SMB %d upd:0x%06lx usable:0x%06lx invalid:0x%06lx ---",
                 ic,
                 (unsigned long)data->acc.updated_temp_mask[ic],
                 (unsigned long)data->acc.usable_temp_mask[ic],
                 (unsigned long)data->acc.invalid_temp_mask[ic]);
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ, "    diag open:0x%06lx short:0x%06lx jump:0x%06lx rate:0x%06lx",
                 (unsigned long)data->acc.temp_open_mask[ic],
                 (unsigned long)data->acc.temp_short_mask[ic],
                 (unsigned long)data->acc.temp_jump_mask[ic],
                 (unsigned long)data->acc.temp_rate_rise_mask[ic]);
        ret |= cli_printline(cli, outline);

        for (int sensor = 0; sensor < NTEMPS; sensor++)
        {
            float volt = 0.0f;
            float T = 0.0f;
            int16_t raw = smb->ics[ic].temp.raw[sensor];

            if(cli_raw_temp_to_values(raw, &volt, &T))
            {
                int whole   = (int)volt;
                int decimal = (int)((volt - (float)whole) * 10000.0f);
                int T_whole   = (int)T;
                int T_decimal = (int)roundf((T - (float)T_whole) * 10.0f);

                snprintf(outline, CLI_LINESZ, "SMB %d | Sensor %d: %-2d.%04d V, %d.%d C %s", ic, sensor, whole, decimal, T_whole, T_decimal,
                         accumulator_temp_sensor_usable(&data->acc, ic, (uint8_t)sensor) ? "usable" : "not_usable");
            }
            else
            {
                snprintf(outline, CLI_LINESZ, "SMB %d | Sensor %d: raw %d invalid", ic, sensor, raw);
            }

            ret |= cli_printline(cli, outline);
        }
    }
    return ret;
}

int get_fan_diag(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    int ret = 0;
    snprintf(outline, CLI_LINESZ,
             "Fan cmd:%.1f%% reason:%s state:%d fault:%d set_fail:%lu last_tick:%lu",
             (double)data->fan_command_percent,
             fan_control_reason_str(data->fan_control_reason),
             data->fan_state,
             data->fan_fault,
             (unsigned long)data->fan_set_fail_count,
             (unsigned long)data->fan_last_update_tick);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Fan temp raw_max:%.1fC filt_max:%.1fC fanmax:%d temp_fault:%d valid:%d",
             (double)data->max_temp,
             (double)data->temp_filtered_max,
             data->temp_fan_max,
             data->temp_fault,
             data->temp_valid);
    ret |= cli_printline(cli, outline);

    for(int i = 0; i < NFANS; i++)
    {
		const fan_t *fan = &data->board.fans[i];
		snprintf(outline, CLI_LINESZ,
				 "  fan%-2d duty:%.1f%% initialized:%d status:%s",
				 i,
				 (double)fan->duty_cycle,
				 fan->initialized,
				 cli_hal_status_str(fan->init_status));
        ret |= cli_printline(cli, outline);
    }

    return ret;
}

static void cli_microvolts_mv_string(int32_t microvolts,
                                        char *buffer,
                                        size_t buffer_size)
{
    uint32_t magnitude;

    if((buffer == NULL) || (buffer_size == 0u))
    {
        return;
    }

    magnitude = (microvolts < 0) ? (uint32_t)(-microvolts) :
                                   (uint32_t)microvolts;
    snprintf(buffer,
             buffer_size,
             "%s%lu.%03lu",
             (microvolts < 0) ? "-" : "",
             (unsigned long)(magnitude / 1000u),
             (unsigned long)(magnitude % 1000u));
}

static void cli_temp_raw_mv_string(int16_t raw, char *buffer, size_t buffer_size)
{
    int32_t microvolts;
    uint32_t magnitude;

    if((buffer == NULL) || (buffer_size == 0u))
    {
        return;
    }

    /* ADBMS6830 AUX codes use the signed -10000 code offset: a raw code
     * of 0 represents 1.500 V. */
    microvolts = ((int32_t)raw + 10000) * 150;
    magnitude = (microvolts < 0) ? (uint32_t)(-microvolts) : (uint32_t)microvolts;

    snprintf(buffer,
             buffer_size,
             "%s%lu.%03lu",
             (microvolts < 0) ? "-" : "",
             (unsigned long)(magnitude / 1000u),
             (unsigned long)(magnitude % 1000u));
}

static const char *cli_temp_debug_reason(const adbms6830_temp_debug_t *dbg,
                                         uint16_t ic_bit)
{
    if(dbg == NULL)
    {
        return "no_debug_state";
    }
    if(dbg->wrc_status != HAL_OK)
    {
        return "WRCOMM_transport";
    }
    if(dbg->pre_rdcomm_status != HAL_OK)
    {
        return "RDCOMM_pre_transport";
    }
    if((dbg->pre_comm_pec_fail_mask & ic_bit) != 0u)
    {
        return "RDCOMM_pre_PEC";
    }
    if((dbg->pre_comm_counter_mismatch_mask & ic_bit) != 0u)
    {
        return "RDCOMM_pre_counter";
    }
    if((dbg->pre_comm_match_mask & ic_bit) == 0u)
    {
        return "WRCOMM_readback_mismatch";
    }
    if(dbg->stcomm_status != HAL_OK)
    {
        return "STCOMM_transport";
    }
    if(dbg->rdcomm_status != HAL_OK)
    {
        return "RDCOMM_transport";
    }
    if((dbg->comm_pec_fail_mask & ic_bit) != 0u)
    {
        return "RDCOMM_PEC";
    }
    if((dbg->comm_counter_mismatch_mask & ic_bit) != 0u)
    {
        return "RDCOMM_counter";
    }
    if((dbg->comm_transport_valid_mask & ic_bit) == 0u)
    {
        return "RDCOMM_invalid";
    }
    if((dbg->address_ack_mask & ic_bit) == 0u)
    {
        return "ADG728_address_NACK";
    }
    if((dbg->data_ack_mask & ic_bit) == 0u)
    {
        return "ADG728_data_NACK";
    }
    if((dbg->selected_mask & ic_bit) == 0u)
    {
        return "mux_selection_unverified";
    }
    if(dbg->wake_status != HAL_OK)
    {
        return "AUX_wakeup";
    }
    if(dbg->adax_status != HAL_OK)
    {
        return "ADAX_command";
    }
    if(dbg->rdaux_status != HAL_OK)
    {
        return "RDAUXA_transport";
    }
    if((dbg->aux_pec_fail_mask & ic_bit) != 0u)
    {
        return "RDAUXA_PEC";
    }
    if((dbg->aux_counter_mismatch_mask & ic_bit) != 0u)
    {
        return "RDAUXA_counter";
    }
    if((dbg->aux_transport_valid_mask & ic_bit) == 0u)
    {
        return "RDAUXA_invalid";
    }
    if((dbg->aux_code_valid_mask & ic_bit) == 0u)
    {
        return "AUX_invalid_code";
    }
    if((dbg->updated_mask & ic_bit) == 0u)
    {
        return "sample_not_publishable";
    }
    return "none";
}

static int cli_print_temp_debug(const adbms6830_driver_t *smb, uint8_t requested_ic)
{
    int ret = 0;
    const adbms6830_temp_debug_t *dbg;
    uint8_t ic_count;

    if(smb == NULL)
    {
        return cli_printline(cli, "TEMPDBG unavailable: null driver");
    }

    dbg = &smb->temp_debug;
    if(!dbg->valid)
    {
        return cli_printline(cli, "TEMPDBG unavailable: no capture");
    }

    snprintf(outline,
             CLI_LINESZ,
             "TEMPDBG sensor:%u mux:U%u addr:0x%02X switch:%u mask:0x%02X gpio:GPIO%u force_aux:%u",
             (unsigned)dbg->sensor_num,
             (unsigned)(dbg->mux_idx + 2u),
             (unsigned)dbg->mux_address,
             (unsigned)dbg->switch_index,
             (unsigned)dbg->switch_mask,
             (unsigned)(dbg->gpio_channel + 1u),
             dbg->forced_aux_capture ? 1u : 0u);
    ret |= cli_printline(cli, outline);

    snprintf(outline,
             CLI_LINESZ,
             "TEMPDBG mode:explicit_once auto_scan:%u WRCOMM_attempts:1 STCOMM_attempts:%u",
             (unsigned)AMS_ENABLE_AUTO_TEMP_MUX_SCAN,
             dbg->stcomm_attempted ? 1u : 0u);
    ret |= cli_printline(cli, outline);

    snprintf(outline,
             CLI_LINESZ,
             "TEMPDBG stage select:%s WRCOMM:%s preRD:%s STCOMM:%s",
             cli_hal_status_str(dbg->select_status),
             cli_hal_status_str(dbg->wrc_status),
             cli_hal_status_str(dbg->pre_rdcomm_status),
             cli_hal_status_str(dbg->stcomm_status));
    ret |= cli_printline(cli, outline);
    snprintf(outline,
             CLI_LINESZ,
             "TEMPDBG stage postRD:%s ADAX:%s RDAUXA:%s overall:%s",
             cli_hal_status_str(dbg->rdcomm_status),
             cli_hal_status_str(dbg->adax_status),
             cli_hal_status_str(dbg->rdaux_status),
             cli_hal_status_str(dbg->overall_status));
    ret |= cli_printline(cli, outline);

    snprintf(outline,
             CLI_LINESZ,
             "TEMPDBG pre exp:%04X pecP:%04X pecF:%04X ctr:%04X match:%04X",
             dbg->expected_ic_mask,
             dbg->pre_comm_pec_pass_mask,
             dbg->pre_comm_pec_fail_mask,
             dbg->pre_comm_counter_mismatch_mask,
             dbg->pre_comm_match_mask);
    ret |= cli_printline(cli, outline);

    snprintf(outline,
             CLI_LINESZ,
             "TEMPDBG post pecP:%04X pecF:%04X ctr:%04X valid:%04X addrACK:%04X dataACK:%04X ack:%04X",
             dbg->comm_pec_pass_mask,
             dbg->comm_pec_fail_mask,
             dbg->comm_counter_mismatch_mask,
             dbg->comm_transport_valid_mask,
             dbg->address_ack_mask,
             dbg->data_ack_mask,
             dbg->acknowledged_mask);
    ret |= cli_printline(cli, outline);

    snprintf(outline,
             CLI_LINESZ,
             "TEMPDBG aux pecP:%04X pecF:%04X ctr:%04X valid:%04X code:%04X selected:%04X updated:%04X",
             dbg->aux_pec_pass_mask,
             dbg->aux_pec_fail_mask,
             dbg->aux_counter_mismatch_mask,
             dbg->aux_transport_valid_mask,
             dbg->aux_code_valid_mask,
             dbg->selected_mask,
             dbg->updated_mask);
    ret |= cli_printline(cli, outline);

    ic_count = smb_ic_count(smb);
    if(requested_ic >= ic_count)
    {
        requested_ic = 0u;
    }

    {
        const uint8_t *tx = dbg->wrcomm_payload[requested_ic];
        const uint8_t *pre = dbg->pre_rdcomm_packet[requested_ic];
        const uint8_t *comm = dbg->rdcomm_packet[requested_ic];
        uint16_t ic_bit = (uint16_t)(1u << requested_ic);

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPDBG IC%u WRCOMM_TX:%02X %02X %02X %02X %02X %02X",
                 (unsigned)requested_ic,
                 tx[0], tx[1], tx[2], tx[3], tx[4], tx[5]);
        ret |= cli_printline(cli, outline);

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPDBG IC%u preRDCOMM:%02X %02X %02X %02X %02X %02X %02X %02X",
                 (unsigned)requested_ic,
                 pre[0], pre[1], pre[2], pre[3],
                 pre[4], pre[5], pre[6], pre[7]);
        ret |= cli_printline(cli, outline);

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPDBG IC%u postRDCOMM:%02X %02X %02X %02X %02X %02X %02X %02X reason:%s",
                 (unsigned)requested_ic,
                 comm[0], comm[1], comm[2], comm[3],
                 comm[4], comm[5], comm[6], comm[7],
                 cli_temp_debug_reason(dbg, ic_bit));
        ret |= cli_printline(cli, outline);

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPDBG IC%u POST slots [I:%X F:%X D:%02X] [I:%X F:%X D:%02X] [I:%X F:%X D:%02X]",
                 (unsigned)requested_ic,
                 (unsigned)(comm[0] >> 4u), (unsigned)(comm[0] & 0x0Fu), comm[1],
                 (unsigned)(comm[2] >> 4u), (unsigned)(comm[2] & 0x0Fu), comm[3],
                 (unsigned)(comm[4] >> 4u), (unsigned)(comm[4] & 0x0Fu), comm[5]);
        ret |= cli_printline(cli, outline);
    }

    {
        const uint8_t *aux = dbg->rdaux_packet[requested_ic];
        int16_t raw_gpio[3];
        char mv0[20];
        char mv1[20];
        char mv2[20];

        raw_gpio[0] = (int16_t)((uint16_t)aux[0] | ((uint16_t)aux[1] << 8u));
        raw_gpio[1] = (int16_t)((uint16_t)aux[2] | ((uint16_t)aux[3] << 8u));
        raw_gpio[2] = (int16_t)((uint16_t)aux[4] | ((uint16_t)aux[5] << 8u));
        cli_temp_raw_mv_string(raw_gpio[0], mv0, sizeof(mv0));
        cli_temp_raw_mv_string(raw_gpio[1], mv1, sizeof(mv1));
        cli_temp_raw_mv_string(raw_gpio[2], mv2, sizeof(mv2));

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPDBG IC%u RDAUXA:%02X %02X %02X %02X %02X %02X %02X %02X",
                 (unsigned)requested_ic,
                 aux[0], aux[1], aux[2], aux[3],
                 aux[4], aux[5], aux[6], aux[7]);
        ret |= cli_printline(cli, outline);

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPDBG IC%u GPIO1:%d/%smV GPIO2:%d/%smV GPIO3:%d/%smV",
                 (unsigned)requested_ic,
                 (int)raw_gpio[0], mv0,
                 (int)raw_gpio[1], mv1,
                 (int)raw_gpio[2], mv2);
        ret |= cli_printline(cli, outline);
    }

    if((dbg->select_status != HAL_OK) &&
       (dbg->rdaux_status == HAL_OK))
    {
        ret |= cli_printline(
            cli,
            "TEMPDBG raw AUX captured after failed select; channel identity not trusted");
    }

    return ret;
}

int get_temperature_sensor(int argc, char *argv[])
{
    int ret;

    adbms_spi_lock();
    ret = get_temperature_sensor_locked(argc, argv);
    adbms_spi_unlock();
    return ret;
}

static int get_temperature_sensor_locked(int argc, char *argv[])
{
    int ret = 0;
    adbms6830_driver_t *smb = &data->acc.smb;

    /* Validate argument count */
    if (argc != 3)
    {
        ret |= cli_printline(cli, "Usage: tempsns <ic> <sensor 0-23>");
        return ret;
    }

    uint8_t ic_count = smb_ic_count(smb);
    int ic;
    int sensor;

    /* Reject partial strings (for example "1x") and overflow instead of
     * silently treating malformed service input as channel zero. */
    if(!cli_parse_int_range(argv[1], 0, (int)ic_count - 1, &ic))
    {
        snprintf(outline, CLI_LINESZ, "Error: ic must be 0 to %u", (unsigned)((ic_count > 0u) ? (ic_count - 1u) : 0u));
        ret |= cli_printline(cli, outline);
        return ret;
    }

    if(!cli_parse_int_range(argv[2], 0, NTEMPS - 1, &sensor))
    {
        ret |= cli_printline(cli, "Error: sensor out of range");
        return ret;
    }

    /* Capture every stage, including raw AUX data even if the mux write
     * NACKs. A forced sample is diagnostic only and never becomes usable. */
    HAL_StatusTypeDef capture_status =
        adbms6830_temp_debug_capture(smb, (uint8_t)sensor, true);

    ret |= cli_print_temp_debug(smb, (uint8_t)ic);

    /* Read back the raw value and convert only when the strict path updated
     * the requested sensor under an acknowledged mux selection. */
    uint32_t sensor_bit = (uint32_t)(1UL << (uint8_t)sensor);
    uint16_t ic_bit = (uint16_t)(1u << (uint8_t)ic);
    bool publishable = (capture_status == HAL_OK) &&
                       ((smb->temp_debug.updated_mask & ic_bit) != 0u) &&
                       ((smb->last_temp_updated_mask[ic] & sensor_bit) != 0u);

    if(publishable)
    {
        float volt = 0.0f;
        float T = 0.0f;
        int16_t raw = smb->ics[ic].temp.raw[sensor];

        if(cli_raw_temp_to_values(raw, &volt, &T))
        {
            int whole;
            int decimal;
            int T_whole;
            int T_decimal;

            cli_fixed3(volt, &whole, &decimal);
            cli_fixed1(T, &T_whole, &T_decimal);
            snprintf(outline,
                     CLI_LINESZ,
                     "SMB %d | Sensor %d: raw:%d %d.%03d V, %d.%d C VALID",
                     ic,
                     sensor,
                     raw,
                     whole,
                     decimal,
                     T_whole,
                     T_decimal);
        }
        else
        {
            snprintf(outline,
                     CLI_LINESZ,
                     "SMB %d | Sensor %d: raw:%d conversion_invalid",
                     ic,
                     sensor,
                     raw);
        }
    }
    else
    {
        snprintf(outline,
                 CLI_LINESZ,
                 "SMB %d | Sensor %d: UNAVAILABLE (see TEMPDBG reason above)",
                 ic,
                 sensor);
    }

    ret |= cli_printline(cli, outline);
    return ret;
}


static const char *cli_temp_bus_level(int16_t raw, bool valid)
{
    int32_t microvolts;

    if(!valid)
    {
        return "INVALID";
    }

    microvolts = ((int32_t)raw + 10000) * 150;
    if(microvolts >= 4000000)
    {
        return "HIGH";
    }
    if(microvolts <= 500000)
    {
        return "LOW";
    }
    return "MID";
}

int get_temp_bus_debug(int argc, char *argv[])
{
    int ret;

    adbms_spi_lock();
    ret = get_temp_bus_debug_locked(argc, argv);
    adbms_spi_unlock();
    return ret;
}

static int get_temp_bus_debug_locked(int argc, char *argv[])
{
    int ret = 0;
    adbms6830_driver_t *smb = &data->acc.smb;
    uint8_t ic_count;

    if((argc != 2) || (argv[1] == NULL))
    {
        return cli_printline(cli, "Usage: tempbus [idle|scan]");
    }

    if(!strcmp(argv[1], "idle"))
    {
        adbms6830_temp_bus_debug_t *dbg;
        HAL_StatusTypeDef status;

        status = adbms6830_temp_bus_idle_capture(smb);
        dbg = &smb->temp_bus_debug;

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPBUS idle non_driving:1 WRCOMM:0 STCOMM:0 auto_scan:%u overall:%s",
                 (unsigned)AMS_ENABLE_AUTO_TEMP_MUX_SCAN,
                 cli_hal_status_str(status));
        ret |= cli_printline(cli, outline);

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPBUS stage wake:%s ADAX_ALL:%s RDAUXB:%s",
                 cli_hal_status_str(dbg->wake_status),
                 cli_hal_status_str(dbg->adax_status),
                 cli_hal_status_str(dbg->rdauxb_status));
        ret |= cli_printline(cli, outline);

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPBUS masks exp:%04X pecP:%04X pecF:%04X ctr:%04X valid:%04X G4:%04X G5:%04X",
                 dbg->expected_ic_mask,
                 dbg->pec_pass_mask,
                 dbg->pec_fail_mask,
                 dbg->counter_mismatch_mask,
                 dbg->transport_valid_mask,
                 dbg->gpio4_code_valid_mask,
                 dbg->gpio5_code_valid_mask);
        ret |= cli_printline(cli, outline);

        ic_count = smb_ic_count(smb);
        for(uint8_t ic = 0u; ic < ic_count; ic++)
        {
            uint16_t bit = (uint16_t)(1u << ic);
            const uint8_t *packet = dbg->rdauxb_packet[ic];
            char gpio4_mv[20];
            char gpio5_mv[20];
            bool gpio4_valid = (dbg->gpio4_code_valid_mask & bit) != 0u;
            bool gpio5_valid = (dbg->gpio5_code_valid_mask & bit) != 0u;

            cli_temp_raw_mv_string(dbg->gpio4_raw[ic], gpio4_mv, sizeof(gpio4_mv));
            cli_temp_raw_mv_string(dbg->gpio5_raw[ic], gpio5_mv, sizeof(gpio5_mv));

            snprintf(outline,
                     CLI_LINESZ,
                     "TEMPBUS IC%u RDAUXB:%02X %02X %02X %02X %02X %02X %02X %02X",
                     (unsigned)ic,
                     packet[0], packet[1], packet[2], packet[3],
                     packet[4], packet[5], packet[6], packet[7]);
            ret |= cli_printline(cli, outline);

            snprintf(outline,
                     CLI_LINESZ,
                     "TEMPBUS IC%u GPIO4/SDA raw:%d %smV %s | GPIO5/SCL raw:%d %smV %s",
                     (unsigned)ic,
                     (int)dbg->gpio4_raw[ic],
                     gpio4_mv,
                     cli_temp_bus_level(dbg->gpio4_raw[ic], gpio4_valid),
                     (int)dbg->gpio5_raw[ic],
                     gpio5_mv,
                     cli_temp_bus_level(dbg->gpio5_raw[ic], gpio5_valid));
            ret |= cli_printline(cli, outline);
        }

        ret |= cli_printline(
            cli,
            "TEMPBUS note: idle HIGH proves pull-ups/continuity only; it does not prove the ADBMS6830 can pull the lines LOW");
        return ret;
    }

    if(!strcmp(argv[1], "scan"))
    {
#if !AMS_ENABLE_SERVICE_CLI
        return cli_service_action_refused("temperature-bus address scan");
#else
        adbms6830_temp_bus_scan_t *scan;
        HAL_StatusTypeDef status;

        status = adbms6830_temp_bus_scan_capture(smb);
        scan = &smb->temp_bus_scan;
        ic_count = smb_ic_count(smb);

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPBUS scan active:1 range:0x%02X-0x%02X data:0x%02X all_switches_open:1 auto_scan:%u transport_overall:%s",
                 scan->first_address,
                 (unsigned)(scan->first_address + scan->address_count - 1u),
                 scan->data_byte,
                 (unsigned)AMS_ENABLE_AUTO_TEMP_MUX_SCAN,
                 cli_hal_status_str(status));
        ret |= cli_printline(cli, outline);

        snprintf(outline,
                 CLI_LINESZ,
                 "TEMPBUS scan exp:%04X any_address_ack_bitmap:0x%02X full_write_ack_bitmap:0x%02X",
                 scan->expected_ic_mask,
                 scan->any_ack_address_bitmap,
                 scan->full_ack_address_bitmap);
        ret |= cli_printline(cli, outline);

        for(uint8_t index = 0u;
            index < ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT;
            index++)
        {
            uint8_t address = (uint8_t)(scan->first_address + index);
            const char *result;

            if(scan->acknowledged_mask[index] == scan->expected_ic_mask)
            {
                result = "ACK";
            }
            else if(scan->address_ack_mask[index] != 0u)
            {
                result = "PARTIAL_OR_DATA_NACK";
            }
            else
            {
                result = "ADDRESS_NACK";
            }

            snprintf(outline,
                     CLI_LINESZ,
                     "TEMPBUS addr:0x%02X transport:%s probe:%s WRCOMM:%s preRD:%s STCOMM:%s postRD:%s result:%s",
                     address,
                     cli_hal_status_str(scan->transport_status[index]),
                     cli_hal_status_str(scan->probe_status[index]),
                     cli_hal_status_str(scan->wrc_status[index]),
                     cli_hal_status_str(scan->pre_rdcomm_status[index]),
                     cli_hal_status_str(scan->stcomm_status[index]),
                     cli_hal_status_str(scan->rdcomm_status[index]),
                     result);
            ret |= cli_printline(cli, outline);

            snprintf(outline,
                     CLI_LINESZ,
                     "TEMPBUS addr:0x%02X masks preMatch:%04X pecP:%04X pecF:%04X ctr:%04X valid:%04X addrACK:%04X dataACK:%04X fullACK:%04X",
                     address,
                     scan->pre_comm_match_mask[index],
                     scan->comm_pec_pass_mask[index],
                     scan->comm_pec_fail_mask[index],
                     scan->comm_counter_mismatch_mask[index],
                     scan->comm_transport_valid_mask[index],
                     scan->address_ack_mask[index],
                     scan->data_ack_mask[index],
                     scan->acknowledged_mask[index]);
            ret |= cli_printline(cli, outline);

            for(uint8_t ic = 0u; ic < ic_count; ic++)
            {
                const uint8_t *packet = scan->rdcomm_packet[index][ic];

                snprintf(outline,
                         CLI_LINESZ,
                         "TEMPBUS addr:0x%02X IC%u RDCOMM:%02X %02X %02X %02X %02X %02X %02X %02X",
                         address,
                         (unsigned)ic,
                         packet[0], packet[1], packet[2], packet[3],
                         packet[4], packet[5], packet[6], packet[7]);
                ret |= cli_printline(cli, outline);
            }
        }

        ret |= cli_printline(
            cli,
            "TEMPBUS scan note: writes 0x00 to 0x4C-0x4F, opens all mux switches, publishes no temperature, and invalidates cached mux selections");
        return ret;
#endif
    }

    return cli_printline(cli, "Usage: tempbus [idle|scan]");
}



static bool cli_adbms_scan_busy(void)
{
    return (data != NULL) && data->adbms_scan_active;
}

static bool cli_adbms_open_wire_state_allowed(void)
{
    return (data != NULL) &&
           ((data->state == STATE_START) ||
            (data->state == STATE_CHARGE) ||
            (data->state == STATE_DISCARGE) ||
            (data->state == STATE_BALANCE)) &&
           !data->bms_state &&
           !data->adbms_balance_active &&
           !accumulator_balance_shadow_active(&data->acc) &&
           data->acc.smb_transport_ready &&
           data->voltage_valid;
}

static const char *cli_passfail(bool ok)
{
    return ok ? "PASS" : "FAIL";
}

static const char *cli_passblock(bool ok)
{
    return ok ? "PASS" : "BLOCKED";
}

static bool cli_spi6_mode3_ok(const SPI_HandleTypeDef *hspi)
{
    return (hspi != NULL) &&
           (hspi->Init.CLKPolarity == SPI_POLARITY_HIGH) &&
           (hspi->Init.CLKPhase == SPI_PHASE_2EDGE) &&
           (hspi->Init.FirstBit == SPI_FIRSTBIT_MSB);
}

static bool cli_preview_all_value(const uint8_t *buf, uint16_t len, uint8_t value)
{
    if((buf == NULL) || (len == 0u))
    {
        return false;
    }

    for(uint16_t i = 0u; i < len; i++)
    {
        if(buf[i] != value)
        {
            return false;
        }
    }

    return true;
}

static uint16_t cli_expected_ic_mask(uint8_t count)
{
    uint16_t mask = 0u;
    uint8_t bounded = (count > NSMBS) ? NSMBS : count;

    for(uint8_t i = 0u; i < bounded; i++)
    {
        mask |= (uint16_t)(1u << i);
    }

    return mask;
}

static uint16_t cli_sid_valid_mask(const adbms6830_driver_t *smb, uint8_t count)
{
    uint16_t mask = 0u;

    if(smb == NULL)
    {
        return 0u;
    }

    for(uint8_t i = 0u; i < count; i++)
    {
        if(smb->diag[i].sid_valid)
        {
            mask |= (uint16_t)(1u << i);
        }
    }

    return mask;
}

static uint16_t cli_stat_valid_mask(const adbms6830_driver_t *smb, uint8_t count)
{
    uint16_t mask = 0u;

    if(smb == NULL)
    {
        return 0u;
    }

    for(uint8_t i = 0u; i < count; i++)
    {
        if(smb->diag[i].statc_valid && smb->diag[i].statd_valid && smb->diag[i].state_valid)
        {
            mask |= (uint16_t)(1u << i);
        }
    }

    return mask;
}

static bool cli_charger_hw_fault(const charger_t *ccs)
{
    return (ccs != NULL) &&
           (ccs->hardware_fail || ccs->overtemp_fail ||
            ccs->input_volt_fail || ccs->voltage_sense_fail ||
            ccs->communication_fail || ccs->tx_fail);
}

static uint32_t cli_charger_rx_age_ms(const charger_t *ccs)
{
    if((ccs == NULL) || (ccs->last_rx_tick == 0u))
    {
        return 0xFFFFFFFFu;
    }

    return osKernelGetTickCount() - ccs->last_rx_tick;
}

static bool cli_adbms_refuse_active_scan(const char *name)
{
    if(!cli_adbms_scan_busy())
    {
        return false;
    }

    snprintf(outline, CLI_LINESZ,
             "%s refused: ADBMS task scan active, retry after current scan",
             (name != NULL) ? name : "ADBMS command");
    (void)cli_printline(cli, outline);
    return true;
}

static bool cli_parse_adbms_string(const char *arg, adbms_string *string_out)
{
    if((arg == NULL) || (string_out == NULL))
    {
        return false;
    }

    if((!strcmp(arg, "a")) || (!strcmp(arg, "A")) ||
       (!strcmp(arg, "cs_a")) || (!strcmp(arg, "CSA")) ||
       (!strcmp(arg, "stringa")))
    {
        *string_out = STRING_A;
        return true;
    }

    if((!strcmp(arg, "b")) || (!strcmp(arg, "B")) ||
       (!strcmp(arg, "cs_b")) || (!strcmp(arg, "CSB")) ||
       (!strcmp(arg, "stringb")))
    {
        *string_out = STRING_B;
        return true;
    }

    return false;
}

static bool cli_parse_scope_mode(const char *arg, adbms6830_scope_mode_t *mode_out)
{
    if((arg == NULL) || (mode_out == NULL))
    {
        return false;
    }

    if(!strcmp(arg, "wake"))
    {
        *mode_out = ADBMS6830_SCOPE_WAKE;
        return true;
    }

    if(!strcmp(arg, "cmd"))
    {
        *mode_out = ADBMS6830_SCOPE_CMD;
        return true;
    }

    if(!strcmp(arg, "read"))
    {
        *mode_out = ADBMS6830_SCOPE_READ;
        return true;
    }

    if(!strcmp(arg, "pattern"))
    {
        *mode_out = ADBMS6830_SCOPE_PATTERN;
        return true;
    }

    return false;
}

static const char *cli_scope_mode_str(adbms6830_scope_mode_t mode)
{
    switch(mode)
    {
    case ADBMS6830_SCOPE_WAKE:    return "wake";
    case ADBMS6830_SCOPE_CMD:     return "cmd";
    case ADBMS6830_SCOPE_READ:    return "read";
    case ADBMS6830_SCOPE_PATTERN: return "pattern";
    default:                      return "unknown";
    }
}

static const char *cli_adbms_string_str(adbms_string string)
{
    return (string == STRING_A) ? "CS_A" : "CS_B";
}

static const char *cli_gpio_state_str(GPIO_PinState state)
{
    return (state == GPIO_PIN_SET) ? "HIGH" : "LOW";
}

static const char *cli_cs_active_str(GPIO_PinState state)
{
    return (state == GPIO_PIN_RESET) ? "ACTIVE" : "IDLE";
}

static bool cli_parse_cs_level(const char *arg, GPIO_PinState *state_out)
{
    if((arg == NULL) || (state_out == NULL))
    {
        return false;
    }

    if(!strcmp(arg, "low") || !strcmp(arg, "active") || !strcmp(arg, "assert"))
    {
        *state_out = GPIO_PIN_RESET;
        return true;
    }

    if(!strcmp(arg, "high") || !strcmp(arg, "idle") ||
       !strcmp(arg, "inactive") || !strcmp(arg, "deassert"))
    {
        *state_out = GPIO_PIN_SET;
        return true;
    }

    return false;
}

static int cli_print_adbms_pin_report(const adbms6830_driver_t *smb)
{
    int ret = 0;
    GPIO_PinState cs_a_state = HAL_GPIO_ReadPin(CS_A_GPIO_Port, CS_A_Pin);
    GPIO_PinState cs_b_state = HAL_GPIO_ReadPin(CS_B_GPIO_Port, CS_B_Pin);

    ret |= cli_printline(cli, "ADBMS6822/6830 bench pin map from Cube/MCU breakout:");
    ret |= cli_printline(cli, "SPI6 SCK:PG13 MOSI:PG14 MISO:PG12 mode3 CPOL_HIGH CPHA_2EDGE");
    ret |= cli_printline(cli, "CS_A:PE2 active_low, CS_B:PE4 active_low");

    snprintf(outline, CLI_LINESZ,
             "firmware CS_A port:PE pin:2 state:%s %s",
             cli_gpio_state_str(cs_a_state),
             cli_cs_active_str(cs_a_state));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "firmware CS_B port:PE pin:4 state:%s %s",
             cli_gpio_state_str(cs_b_state),
             cli_cs_active_str(cs_b_state));
    ret |= cli_printline(cli, outline);

    if(smb != NULL)
    {
        snprintf(outline, CLI_LINESZ,
                 "driver cs pointers present A:%d B:%d pins A:0x%04X B:0x%04X",
                 smb->cs_port[STRING_A] != NULL,
                 smb->cs_port[STRING_B] != NULL,
                 (unsigned)smb->cs_pin[STRING_A],
                 (unsigned)smb->cs_pin[STRING_B]);
        ret |= cli_printline(cli, outline);
    }

    ret |= cli_printline(cli, "bench: scope PE4 for CS_B; PF4 is a stale/conflicting schematic note");
    return ret;
}

static int cli_set_one_cs(adbms6830_driver_t *smb, adbms_string string, GPIO_PinState state)
{
    if((smb == NULL) || (string > STRING_B) || (smb->cs_port[string] == NULL))
    {
        return cli_printline(cli, "ERROR: CS target unavailable");
    }

    HAL_GPIO_WritePin(smb->cs_port[string], smb->cs_pin[string], state);
    snprintf(outline, CLI_LINESZ,
             "%s set %s (%s)",
             cli_adbms_string_str(string),
             cli_gpio_state_str(state),
             cli_cs_active_str(state));
    return cli_printline(cli, outline);
}

static int cli_pulse_one_cs(adbms6830_driver_t *smb, adbms_string string, uint16_t count)
{
    int ret = 0;

    if((smb == NULL) || (string > STRING_B) || (smb->cs_port[string] == NULL))
    {
        return cli_printline(cli, "ERROR: CS target unavailable");
    }

    for(uint16_t i = 0u; i < count; i++)
    {
        HAL_GPIO_WritePin(smb->cs_port[string], smb->cs_pin[string], GPIO_PIN_RESET);
        adbms6830_us_delay(smb, 100u);
        HAL_GPIO_WritePin(smb->cs_port[string], smb->cs_pin[string], GPIO_PIN_SET);
        adbms6830_us_delay(smb, 100u);
    }

    snprintf(outline, CLI_LINESZ,
             "%s pulsed active-low %u time(s), left IDLE/HIGH",
             cli_adbms_string_str(string),
             (unsigned)count);
    ret |= cli_printline(cli, outline);
    return ret;
}

static void cli_config_pf4_as_output(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &gpio);
}

static void cli_restore_pf4_as_analog(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &gpio);
}

static void cli_pulse_candidate_pin(GPIO_TypeDef *port, uint16_t pin, uint16_t count)
{
    for(uint16_t i = 0u; i < count; i++)
    {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        osDelay(1u);
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        osDelay(1u);
    }
}

static int cli_handle_spi_candidate_pins(int argc, char *argv[])
{
    int ret = 0;
    const char *mode = "alt";
    uint16_t count = 10u;

    if((argc >= 3) && (argv[2] != NULL))
    {
        mode = argv[2];
    }

    if(argc >= 4)
    {
        if(!cli_parse_scope_repeat(argv[3], &count))
        {
            ret |= cli_printline(cli, "ERROR: count must be 1-100");
            return ret;
        }
    }

    if(strcmp(mode, "alt") && strcmp(mode, "both") &&
       strcmp(mode, "pe4") && strcmp(mode, "pf4"))
    {
        ret |= cli_printline(cli, "Usage: spi cspins [alt|both|pe4|pf4] [count 1-100]");
        return ret;
    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
    cli_config_pf4_as_output();
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);

    ret |= cli_printline(cli, "CS candidate pin test: active-low pulses, both pins left HIGH/IDLE");
    ret |= cli_printline(cli, "Probe PE4 and PF4. This does not change runtime CS_B mapping.");

    if(!strcmp(mode, "pe4"))
    {
        ret |= cli_printline(cli, "pulsing PE4 only");
        cli_pulse_candidate_pin(GPIOE, GPIO_PIN_4, count);
    }
    else if(!strcmp(mode, "pf4"))
    {
        ret |= cli_printline(cli, "pulsing PF4 only");
        cli_pulse_candidate_pin(GPIOF, GPIO_PIN_4, count);
    }
    else if(!strcmp(mode, "both"))
    {
        ret |= cli_printline(cli, "pulsing PE4 block, then PF4 block");
        cli_pulse_candidate_pin(GPIOE, GPIO_PIN_4, count);
        osDelay(5u);
        cli_pulse_candidate_pin(GPIOF, GPIO_PIN_4, count);
    }
    else
    {
        ret |= cli_printline(cli, "alternating PE4 then PF4");
        for(uint16_t i = 0u; i < count; i++)
        {
            cli_pulse_candidate_pin(GPIOE, GPIO_PIN_4, 1u);
            cli_pulse_candidate_pin(GPIOF, GPIO_PIN_4, 1u);
        }
    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
    cli_restore_pf4_as_analog();

    snprintf(outline, CLI_LINESZ,
             "cspins mode:%s count:%u done; PE4/PF4 left idle high",
             mode,
             (unsigned)count);
    ret |= cli_printline(cli, outline);
    return ret;
}

static int cli_handle_spi_cs(int argc, char *argv[], adbms6830_driver_t *smb)
{
    int ret = 0;
    adbms_string string;
    GPIO_PinState state;
    uint16_t count = 1u;

    if((argc < 3) || (argv[2] == NULL) || !strcmp(argv[2], "status"))
    {
        return cli_print_adbms_pin_report(smb);
    }

    if((argc >= 4) && (!strcmp(argv[2], "all") || !strcmp(argv[2], "both")))
    {
        if(!cli_parse_cs_level(argv[3], &state))
        {
            ret |= cli_printline(cli, "Usage: spi cs [a|b|all] [low|high|pulse] [count 1-100]");
            return ret;
        }

        ret |= cli_set_one_cs(smb, STRING_A, state);
        ret |= cli_set_one_cs(smb, STRING_B, state);
        return ret;
    }

    if(!cli_parse_adbms_string(argv[2], &string) || (argc < 4) || (argv[3] == NULL))
    {
        ret |= cli_printline(cli, "Usage: spi cs [a|b|all] [low|high|pulse] [count 1-100]");
        return ret;
    }

    if(!strcmp(argv[3], "pulse"))
    {
        if(argc >= 5)
        {
            if(!cli_parse_scope_repeat(argv[4], &count))
            {
                ret |= cli_printline(cli, "ERROR: count must be 1-100");
                return ret;
            }
        }
        ret |= cli_pulse_one_cs(smb, string, count);
        ret |= cli_print_adbms_pin_report(smb);
        return ret;
    }

    if(!cli_parse_cs_level(argv[3], &state))
    {
        ret |= cli_printline(cli, "Usage: spi cs [a|b|all] [low|high|pulse] [count 1-100]");
        return ret;
    }

    ret |= cli_set_one_cs(smb, string, state);
    ret |= cli_print_adbms_pin_report(smb);
    return ret;
}

static bool cli_parse_scope_repeat(const char *arg, uint16_t *repeat_out)
{
    int parsed;

    if((arg == NULL) || (repeat_out == NULL))
    {
        return false;
    }

    if(!cli_parse_int_range(arg, 1, 100, &parsed))
    {
        return false;
    }

    *repeat_out = (uint16_t)parsed;
    return true;
}

static bool cli_parse_u16_range(const char *arg, uint16_t minimum,
                                uint16_t maximum, uint16_t *value_out)
{
    int parsed;

    if((arg == NULL) || (value_out == NULL) || (minimum > maximum))
    {
        return false;
    }
    if(!cli_parse_int_range(arg, (int)minimum, (int)maximum, &parsed))
    {
        return false;
    }
    *value_out = (uint16_t)parsed;
    return true;
}

static const char *cli_voltage_mode_str(void)
{
#if AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP
    return "C_ONLY_MVP";
#else
    return "REDUNDANT_CS";
#endif
}

static void cli_adbms_scope_apply_preset(uint8_t preset)
{
    cli_adbms_scope_preset_index = (uint8_t)(preset % 4u);

    switch(cli_adbms_scope_preset_index)
    {
    case 0u:
		cli_adbms_scope_default_string = ACCUMULATOR_SMB_STRING;
        cli_adbms_scope_default_mode = ADBMS6830_SCOPE_READ;
        cli_adbms_scope_default_repeat = 20u;
        break;
    case 1u:
		cli_adbms_scope_default_string = ACCUMULATOR_SMB_STRING;
        cli_adbms_scope_default_mode = ADBMS6830_SCOPE_CMD;
        cli_adbms_scope_default_repeat = 50u;
        break;
    case 2u:
		cli_adbms_scope_default_string = ACCUMULATOR_SMB_STRING;
        cli_adbms_scope_default_mode = ADBMS6830_SCOPE_PATTERN;
        cli_adbms_scope_default_repeat = 20u;
        break;
    default:
		cli_adbms_scope_default_string = STRING_B;
        cli_adbms_scope_default_mode = ADBMS6830_SCOPE_READ;
        cli_adbms_scope_default_repeat = 20u;
        break;
    }
}

static int cli_print_adbms_scope_preset(void)
{
    int ret = 0;

    snprintf(outline, CLI_LINESZ,
             "scope preset string:%s mode:%s repeat:%u",
             cli_adbms_string_str(cli_adbms_scope_default_string),
             cli_scope_mode_str(cli_adbms_scope_default_mode),
             (unsigned)cli_adbms_scope_default_repeat);
    ret |= cli_printline(cli, outline);
    ret |= cli_printline(cli, "preset choices: normal|cmd|pattern|a|b|toggle|repeat <1-100>");
    ret |= cli_printline(cli, "run selected preset with: spi scope");
    return ret;
}

static int cli_print_sadc_dump(adbms6830_driver_t *smb,
                                HAL_StatusTypeDef capture_status)
{
    const adbms6830_sadc_debug_t *dbg;
    uint8_t ic_count;
    uint16_t monitored_mask;
    int ret = 0;

    if(smb == NULL)
    {
        return cli_printline(cli, "Standalone S-ADC state unavailable");
    }

    dbg = &smb->sadc_debug;
    ic_count = smb_ic_count(smb);
    monitored_mask = (smb->monitored_cell_count == 16u) ? UINT16_MAX :
                     (uint16_t)((1UL << smb->monitored_cell_count) - 1UL);

    snprintf(outline, CLI_LINESZ,
             "Standalone ADSV status:%s wake:%s cmd:%s wait:%s groups:%u safety_state_unchanged:1",
             cli_hal_status_str(capture_status),
             cli_hal_status_str(dbg->wake_status),
             cli_hal_status_str(dbg->command_status),
             cli_hal_status_str(dbg->delay_status),
             (unsigned)dbg->group_count);
    ret |= cli_printline(cli, outline);

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        for(uint8_t group = 0u; group < dbg->group_count; group++)
        {
            const uint8_t *packet = dbg->packet[ic][group];
            uint16_t ic_bit = (uint16_t)(1u << ic);
            bool pec_ok = (dbg->pec_pass_mask[group] & ic_bit) != 0u;
            bool counter_mismatch =
                (dbg->counter_mismatch_mask[group] & ic_bit) != 0u;
            const char *counter_state =
                (dbg->group_read_status[group] != HAL_OK) ? "NA" :
                (!pec_ok ? "NA" : (counter_mismatch ? "FAIL" : "PASS"));
            uint8_t command_counter = (uint8_t)(packet[6] >> 2u);

            snprintf(outline, CLI_LINESZ,
                     "IC%u RDSV%c raw:%02X %02X %02X %02X %02X %02X %02X %02X HAL:%s PEC:%s CTR:%s cnt:%u",
                     (unsigned)ic,
                     (char)('A' + group),
                     packet[0], packet[1], packet[2], packet[3],
                     packet[4], packet[5], packet[6], packet[7],
                     cli_hal_status_str(dbg->group_read_status[group]),
                     pec_ok ? "PASS" : "FAIL",
                     counter_state,
                     (unsigned)command_counter);
            ret |= cli_printline(cli, outline);
        }

        snprintf(outline, CLI_LINESZ,
                 "IC%u Svalid:0x%04X Spec:0x%04X expected_cells:0x%04X",
                 (unsigned)ic,
                 (unsigned)(smb->last_scell_updated_mask[ic] & monitored_mask),
                 (unsigned)(smb->last_scell_pec_mask[ic] & monitored_mask),
                 (unsigned)monitored_mask);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli, "Cell   raw_code     S-ADC mV      valid");

        for(uint8_t cell = 0u; cell < smb->monitored_cell_count; cell++)
        {
            uint16_t bit = (uint16_t)(1u << cell);
            bool valid = (smb->last_scell_updated_mask[ic] & bit) != 0u;

            if(valid)
            {
                int16_t raw = smb->ics[ic].scell.sc_codes[cell];
                int32_t microvolts = ((int32_t)raw + 10000) * 150;
                char mv[20];

                cli_microvolts_mv_string(microvolts, mv, sizeof(mv));
                snprintf(outline, CLI_LINESZ,
                         "S%02u    %7d      %9smV   yes",
                         (unsigned)(cell + 1u),
                         (int)raw,
                         mv);
            }
            else
            {
                snprintf(outline, CLI_LINESZ,
                         "S%02u    unavailable                 no",
                         (unsigned)(cell + 1u));
            }
            ret |= cli_printline(cli, outline);
        }
    }

    return ret;
}

static int cli_run_sadc_repeat(adbms6830_driver_t *smb, uint16_t repeat_count)
{
    uint8_t ic_count;
    uint16_t monitored_mask;
    int ret = 0;

    if((smb == NULL) || (repeat_count == 0u))
    {
        return cli_printline(cli, "Standalone S-ADC repeat parameters invalid");
    }

    ic_count = smb_ic_count(smb);
    monitored_mask = (smb->monitored_cell_count == 16u) ? UINT16_MAX :
                     (uint16_t)((1UL << smb->monitored_cell_count) - 1UL);

    memset(cli_srepeat_sum_uv, 0, sizeof(cli_srepeat_sum_uv));
    memset(cli_srepeat_valid_count, 0, sizeof(cli_srepeat_valid_count));
    memset(cli_srepeat_hal_fail_count, 0, sizeof(cli_srepeat_hal_fail_count));
    memset(cli_srepeat_pec_fail_count, 0, sizeof(cli_srepeat_pec_fail_count));
    memset(cli_srepeat_counter_fail_count, 0, sizeof(cli_srepeat_counter_fail_count));

    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        for(uint8_t cell = 0u; cell < CELL; cell++)
        {
            cli_srepeat_min_uv[ic][cell] = INT32_MAX;
            cli_srepeat_max_uv[ic][cell] = INT32_MIN;
        }
    }

    for(uint16_t sample = 0u; sample < repeat_count; sample++)
    {
        HAL_StatusTypeDef status = adbms6830_capture_s_adc(smb);
        const adbms6830_sadc_debug_t *dbg = &smb->sadc_debug;

        for(uint8_t ic = 0u; ic < ic_count; ic++)
        {
            uint16_t ic_bit = (uint16_t)(1u << ic);

            if(status != HAL_OK)
            {
                cli_srepeat_hal_fail_count[ic]++;
            }

            for(uint8_t group = 0u; group < dbg->group_count; group++)
            {
                if((dbg->pec_fail_mask[group] & ic_bit) != 0u)
                {
                    cli_srepeat_pec_fail_count[ic]++;
                }
                if((dbg->counter_mismatch_mask[group] & ic_bit) != 0u)
                {
                    cli_srepeat_counter_fail_count[ic]++;
                }
            }

            for(uint8_t cell = 0u; cell < smb->monitored_cell_count; cell++)
            {
                uint16_t bit = (uint16_t)(1u << cell);
                int32_t microvolts;

                if((smb->last_scell_updated_mask[ic] & bit) == 0u)
                {
                    continue;
                }

                microvolts =
                    ((int32_t)smb->ics[ic].scell.sc_codes[cell] + 10000) * 150;
                if(microvolts < cli_srepeat_min_uv[ic][cell])
                {
                    cli_srepeat_min_uv[ic][cell] = microvolts;
                }
                if(microvolts > cli_srepeat_max_uv[ic][cell])
                {
                    cli_srepeat_max_uv[ic][cell] = microvolts;
                }
                cli_srepeat_sum_uv[ic][cell] += microvolts;
                cli_srepeat_valid_count[ic][cell]++;
            }
        }
    }

    snprintf(outline, CLI_LINESZ,
             "Standalone ADSV repeat:%u monitored_cells:%u safety_state_unchanged:1",
             (unsigned)repeat_count,
             (unsigned)smb->monitored_cell_count);
    ret |= cli_printline(cli, outline);

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        snprintf(outline, CLI_LINESZ,
                 "IC%u capture_errors:%lu PEC_group_failures:%lu counter_group_mismatches:%lu",
                 (unsigned)ic,
                 (unsigned long)cli_srepeat_hal_fail_count[ic],
                 (unsigned long)cli_srepeat_pec_fail_count[ic],
                 (unsigned long)cli_srepeat_counter_fail_count[ic]);
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "IC%u final_valid:0x%04X expected:0x%04X",
                 (unsigned)ic,
                 (unsigned)(smb->last_scell_updated_mask[ic] & monitored_mask),
                 (unsigned)monitored_mask);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli, "Cell   valid/N    min mV       avg mV       max mV");

        for(uint8_t cell = 0u; cell < smb->monitored_cell_count; cell++)
        {
            uint16_t valid_count = cli_srepeat_valid_count[ic][cell];

            if(valid_count == 0u)
            {
                snprintf(outline, CLI_LINESZ,
                         "S%02u      0/%u     UNAVAILABLE",
                         (unsigned)(cell + 1u),
                         (unsigned)repeat_count);
            }
            else
            {
                int32_t average_uv =
                    (int32_t)(cli_srepeat_sum_uv[ic][cell] / valid_count);
                char min_mv[20];
                char avg_mv[20];
                char max_mv[20];

                cli_microvolts_mv_string(cli_srepeat_min_uv[ic][cell],
                                          min_mv,
                                          sizeof(min_mv));
                cli_microvolts_mv_string(average_uv, avg_mv, sizeof(avg_mv));
                cli_microvolts_mv_string(cli_srepeat_max_uv[ic][cell],
                                          max_mv,
                                          sizeof(max_mv));
                snprintf(outline, CLI_LINESZ,
                         "S%02u    %3u/%u    %9s    %9s    %9s",
                         (unsigned)(cell + 1u),
                         (unsigned)valid_count,
                         (unsigned)repeat_count,
                         min_mv,
                         avg_mv,
                         max_mv);
            }
            ret |= cli_printline(cli, outline);
        }
    }

    return ret;
}

static int cli_print_cadc_dump(adbms6830_driver_t *smb,
                                HAL_StatusTypeDef capture_status)
{
    const adbms6830_cadc_debug_t *dbg;
    uint8_t ic_count;
    uint16_t monitored_mask;
    int ret = 0;

    if(smb == NULL)
    {
        return cli_printline(cli, "Standalone C-ADC state unavailable");
    }

    dbg = &smb->cadc_debug;
    ic_count = smb_ic_count(smb);
    monitored_mask = (smb->monitored_cell_count == 16u) ? UINT16_MAX :
                     (uint16_t)((1UL << smb->monitored_cell_count) - 1UL);

    snprintf(outline, CLI_LINESZ,
             "Standalone ADCV(C-only) status:%s wake:%s cmd:%s poll:%s complete:%d",
             cli_hal_status_str(capture_status),
             cli_hal_status_str(dbg->wake_status),
             cli_hal_status_str(dbg->command_status),
             cli_hal_status_str(dbg->poll_status),
             dbg->poll_complete);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "Standalone ADCV time:%luus clocks:%lu groups:%u safety_state_unchanged:1",
             (unsigned long)dbg->conversion_time_us,
             (unsigned long)dbg->poll_clock_bytes,
             (unsigned)dbg->group_count);
    ret |= cli_printline(cli, outline);

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        for(uint8_t group = 0u; group < dbg->group_count; group++)
        {
            const uint8_t *packet = dbg->packet[ic][group];
            uint16_t ic_bit = (uint16_t)(1u << ic);
            bool pec_ok = (dbg->pec_pass_mask[group] & ic_bit) != 0u;
            bool counter_mismatch =
                (dbg->counter_mismatch_mask[group] & ic_bit) != 0u;
            const char *counter_state =
                (dbg->group_read_status[group] != HAL_OK) ? "NA" :
                (!pec_ok ? "NA" : (counter_mismatch ? "FAIL" : "PASS"));
            uint8_t command_counter = (uint8_t)(packet[6] >> 2u);

            snprintf(outline, CLI_LINESZ,
                     "IC%u RDCV%c raw:%02X %02X %02X %02X %02X %02X %02X %02X HAL:%s PEC:%s CTR:%s cnt:%u",
                     (unsigned)ic,
                     (char)('A' + group),
                     packet[0], packet[1], packet[2], packet[3],
                     packet[4], packet[5], packet[6], packet[7],
                     cli_hal_status_str(dbg->group_read_status[group]),
                     pec_ok ? "PASS" : "FAIL",
                     counter_state,
                     (unsigned)command_counter);
            ret |= cli_printline(cli, outline);
        }

        snprintf(outline, CLI_LINESZ,
                 "IC%u Cvalid:0x%04X Cpec:0x%04X expected_cells:0x%04X",
                 (unsigned)ic,
                 (unsigned)(smb->last_cell_updated_mask[ic] & monitored_mask),
                 (unsigned)(smb->last_cell_pec_mask[ic] & monitored_mask),
                 (unsigned)monitored_mask);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli, "Cell   raw_code     C-ADC mV      valid");

        for(uint8_t cell = 0u; cell < smb->monitored_cell_count; cell++)
        {
            uint16_t bit = (uint16_t)(1u << cell);
            bool valid = (smb->last_cell_updated_mask[ic] & bit) != 0u;

            if(valid)
            {
                int16_t raw = smb->ics[ic].cell.c_codes[cell];
                int32_t microvolts = ((int32_t)raw + 10000) * 150;
                char mv[20];

                cli_microvolts_mv_string(microvolts, mv, sizeof(mv));
                snprintf(outline, CLI_LINESZ,
                         "C%02u    %7d      %9smV   yes",
                         (unsigned)(cell + 1u), (int)raw, mv);
            }
            else
            {
                snprintf(outline, CLI_LINESZ,
                         "C%02u          --             --      no",
                         (unsigned)(cell + 1u));
            }
            ret |= cli_printline(cli, outline);
        }
    }

    return ret;
}

static int cli_run_cadc_soak(adbms6830_driver_t *smb, uint16_t repeat_count)
{
    uint8_t ic_count;
    uint16_t monitored_mask;
    uint32_t timing_min_us = UINT32_MAX;
    uint32_t timing_max_us = 0u;
    uint64_t timing_sum_us = 0u;
    uint32_t timing_valid = 0u;
    uint32_t capture_errors = 0u;
    int ret = 0;

    if((smb == NULL) || (repeat_count == 0u))
    {
        return cli_printline(cli, "C-ADC soak state unavailable");
    }

    ic_count = smb_ic_count(smb);
    monitored_mask = (smb->monitored_cell_count == 16u) ? UINT16_MAX :
                     (uint16_t)((1UL << smb->monitored_cell_count) - 1UL);

    memset(cli_csoak_sum_uv, 0, sizeof(cli_csoak_sum_uv));
    memset(cli_csoak_valid_count, 0, sizeof(cli_csoak_valid_count));
    memset(cli_csoak_prev_uv, 0, sizeof(cli_csoak_prev_uv));
    memset(cli_csoak_max_jump_uv, 0, sizeof(cli_csoak_max_jump_uv));
    memset(cli_csoak_prev_valid, 0, sizeof(cli_csoak_prev_valid));
    memset(cli_csoak_hal_fail_count, 0, sizeof(cli_csoak_hal_fail_count));
    memset(cli_csoak_pec_fail_count, 0, sizeof(cli_csoak_pec_fail_count));
    memset(cli_csoak_counter_fail_count, 0, sizeof(cli_csoak_counter_fail_count));

    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        for(uint8_t cell = 0u; cell < CELL; cell++)
        {
            cli_csoak_min_uv[ic][cell] = INT32_MAX;
            cli_csoak_max_uv[ic][cell] = INT32_MIN;
        }
    }

    for(uint16_t sample = 0u; sample < repeat_count; sample++)
    {
        HAL_StatusTypeDef status;
        const adbms6830_cadc_debug_t *dbg;

        adbms_spi_lock();
        status = adbms6830_capture_c_adc(smb);
        dbg = &smb->cadc_debug;

        if(status != HAL_OK)
        {
            capture_errors++;
        }
        if(dbg->poll_complete && (dbg->poll_status == HAL_OK))
        {
            if(dbg->conversion_time_us < timing_min_us)
            {
                timing_min_us = dbg->conversion_time_us;
            }
            if(dbg->conversion_time_us > timing_max_us)
            {
                timing_max_us = dbg->conversion_time_us;
            }
            timing_sum_us += dbg->conversion_time_us;
            timing_valid++;
        }

        for(uint8_t ic = 0u; ic < ic_count; ic++)
        {
            uint16_t ic_bit = (uint16_t)(1u << ic);

            for(uint8_t group = 0u; group < dbg->group_count; group++)
            {
                if(dbg->group_read_status[group] != HAL_OK)
                {
                    cli_csoak_hal_fail_count[ic]++;
                }
                if((dbg->pec_fail_mask[group] & ic_bit) != 0u)
                {
                    cli_csoak_pec_fail_count[ic]++;
                }
                if((dbg->counter_mismatch_mask[group] & ic_bit) != 0u)
                {
                    cli_csoak_counter_fail_count[ic]++;
                }
            }

            for(uint8_t cell = 0u; cell < smb->monitored_cell_count; cell++)
            {
                uint16_t bit = (uint16_t)(1u << cell);
                bool valid = ((smb->last_cell_updated_mask[ic] & bit) != 0u) &&
                             ((smb->last_cell_pec_mask[ic] & bit) == 0u);

                if(valid)
                {
                    int16_t raw = smb->ics[ic].cell.c_codes[cell];
                    int32_t uv = ((int32_t)raw + 10000) * 150;

                    if(uv < cli_csoak_min_uv[ic][cell])
                    {
                        cli_csoak_min_uv[ic][cell] = uv;
                    }
                    if(uv > cli_csoak_max_uv[ic][cell])
                    {
                        cli_csoak_max_uv[ic][cell] = uv;
                    }
                    cli_csoak_sum_uv[ic][cell] += uv;
                    if(cli_csoak_valid_count[ic][cell] != UINT16_MAX)
                    {
                        cli_csoak_valid_count[ic][cell]++;
                    }
                    if(cli_csoak_prev_valid[ic][cell])
                    {
                        int32_t jump = uv - cli_csoak_prev_uv[ic][cell];
                        if(jump < 0)
                        {
                            jump = -jump;
                        }
                        if(jump > cli_csoak_max_jump_uv[ic][cell])
                        {
                            cli_csoak_max_jump_uv[ic][cell] = jump;
                        }
                    }
                    cli_csoak_prev_uv[ic][cell] = uv;
                    cli_csoak_prev_valid[ic][cell] = true;
                }
                else
                {
                    cli_csoak_prev_valid[ic][cell] = false;
                }
            }
        }
        adbms_spi_unlock();
    }

    snprintf(outline, CLI_LINESZ,
             "C-ADC soak requested:%u capture_errors:%lu timing_valid:%lu safety_state_unchanged:1",
             (unsigned)repeat_count,
             (unsigned long)capture_errors,
             (unsigned long)timing_valid);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "C-ADC timing min:%luus avg:%luus max:%luus",
             (unsigned long)((timing_valid != 0u) ? timing_min_us : 0u),
             (unsigned long)((timing_valid != 0u) ?
                 (uint32_t)(timing_sum_us / timing_valid) : 0u),
             (unsigned long)((timing_valid != 0u) ? timing_max_us : 0u));
    ret |= cli_printline(cli, outline);

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        int64_t pack_avg_uv = 0;
        int32_t cell_avg_min_uv = INT32_MAX;
        int32_t cell_avg_max_uv = INT32_MIN;
        uint16_t complete_cells = 0u;

        snprintf(outline, CLI_LINESZ,
                 "IC%u transport group_fail HAL:%lu PEC:%lu CTR:%lu final_valid:0x%04X expected:0x%04X",
                 (unsigned)ic,
                 (unsigned long)cli_csoak_hal_fail_count[ic],
                 (unsigned long)cli_csoak_pec_fail_count[ic],
                 (unsigned long)cli_csoak_counter_fail_count[ic],
                 (unsigned)(smb->last_cell_updated_mask[ic] & monitored_mask),
                 (unsigned)monitored_mask);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli,
            "Cell valid/N    min mV       avg mV       max mV       p-p mV      max_jump mV");

        for(uint8_t cell = 0u; cell < smb->monitored_cell_count; cell++)
        {
            uint16_t valid = cli_csoak_valid_count[ic][cell];
            if(valid != 0u)
            {
                int32_t avg_uv = (int32_t)(cli_csoak_sum_uv[ic][cell] / valid);
                char min_mv[20], avg_mv[20], max_mv[20], pp_mv[20], jump_mv[20];
                int32_t pp_uv = cli_csoak_max_uv[ic][cell] - cli_csoak_min_uv[ic][cell];

                cli_microvolts_mv_string(cli_csoak_min_uv[ic][cell], min_mv, sizeof(min_mv));
                cli_microvolts_mv_string(avg_uv, avg_mv, sizeof(avg_mv));
                cli_microvolts_mv_string(cli_csoak_max_uv[ic][cell], max_mv, sizeof(max_mv));
                cli_microvolts_mv_string(pp_uv, pp_mv, sizeof(pp_mv));
                cli_microvolts_mv_string(cli_csoak_max_jump_uv[ic][cell], jump_mv, sizeof(jump_mv));
                snprintf(outline, CLI_LINESZ,
                         "C%02u  %4u/%-4u  %10s  %10s  %10s  %10s  %10s",
                         (unsigned)(cell + 1u), (unsigned)valid,
                         (unsigned)repeat_count, min_mv, avg_mv, max_mv,
                         pp_mv, jump_mv);

                pack_avg_uv += avg_uv;
                if(avg_uv < cell_avg_min_uv)
                {
                    cell_avg_min_uv = avg_uv;
                }
                if(avg_uv > cell_avg_max_uv)
                {
                    cell_avg_max_uv = avg_uv;
                }
                if(valid == repeat_count)
                {
                    complete_cells++;
                }
            }
            else
            {
                snprintf(outline, CLI_LINESZ,
                         "C%02u     0/%-4u          --          --          --          --          --",
                         (unsigned)(cell + 1u), (unsigned)repeat_count);
            }
            ret |= cli_printline(cli, outline);
        }

        if(cell_avg_min_uv != INT32_MAX)
        {
            char pack_mv[20], min_mv[20], max_mv[20], delta_mv[20];
            cli_microvolts_mv_string((int32_t)pack_avg_uv, pack_mv, sizeof(pack_mv));
            cli_microvolts_mv_string(cell_avg_min_uv, min_mv, sizeof(min_mv));
            cli_microvolts_mv_string(cell_avg_max_uv, max_mv, sizeof(max_mv));
            cli_microvolts_mv_string(cell_avg_max_uv - cell_avg_min_uv,
                                      delta_mv, sizeof(delta_mv));
            snprintf(outline, CLI_LINESZ,
                     "IC%u C plausibility complete_cells:%u/%u pack_sum_avg:%smV diagnostic_only:1",
                     (unsigned)ic, (unsigned)complete_cells,
                     (unsigned)smb->monitored_cell_count, pack_mv);
            ret |= cli_printline(cli, outline);
            snprintf(outline, CLI_LINESZ,
                     "IC%u C cell_avg min:%smV max:%smV delta:%smV",
                     (unsigned)ic, min_mv, max_mv, delta_mv);
            ret |= cli_printline(cli, outline);
        }
    }

    return ret;
}

static int cli_run_conversion_timing(adbms6830_driver_t *smb,
                                     int argc, char *argv[])
{
    uint16_t repeat = 20u;
    bool selected[3] = {true, true, true};
    uint32_t min_us[3] = {UINT32_MAX, UINT32_MAX, UINT32_MAX};
    uint32_t max_us[3] = {0u, 0u, 0u};
    uint64_t sum_us[3] = {0u, 0u, 0u};
    uint64_t sum_bytes[3] = {0u, 0u, 0u};
    uint32_t valid[3] = {0u, 0u, 0u};
    uint32_t failures[3] = {0u, 0u, 0u};
    uint32_t busy_not_seen[3] = {0u, 0u, 0u};
    int ret = 0;

    if(smb == NULL)
    {
        return cli_printline(cli, "Conversion timing state unavailable");
    }

    if(argc >= 3)
    {
        selected[0] = selected[1] = selected[2] = false;
        if(!strcmp(argv[2], "all"))
        {
            selected[0] = selected[1] = selected[2] = true;
        }
        else if(!strcmp(argv[2], "c"))
        {
            selected[ADBMS6830_TIMING_C_ADC] = true;
        }
        else if(!strcmp(argv[2], "s"))
        {
            selected[ADBMS6830_TIMING_S_ADC] = true;
        }
        else if(!strcmp(argv[2], "aux"))
        {
            selected[ADBMS6830_TIMING_AUX_ADC] = true;
        }
        else
        {
            return cli_printline(cli, "Usage: spi timing [all|c|s|aux] [1-1000]");
        }
    }
    if((argc >= 4) && !cli_parse_u16_range(argv[3], 1u, 1000u, &repeat))
    {
        return cli_printline(cli, "Usage: spi timing [all|c|s|aux] [1-1000]");
    }

    for(uint8_t kind = 0u; kind < 3u; kind++)
    {
        if(!selected[kind])
        {
            continue;
        }
        for(uint16_t sample = 0u; sample < repeat; sample++)
        {
            adbms6830_timing_result_t result;
            HAL_StatusTypeDef status = adbms6830_profile_conversion_timing(
                smb, (adbms6830_timing_kind_t)kind, &result);

            if((status == HAL_OK) && result.observed_busy && result.complete)
            {
                if(result.elapsed_us < min_us[kind])
                {
                    min_us[kind] = result.elapsed_us;
                }
                if(result.elapsed_us > max_us[kind])
                {
                    max_us[kind] = result.elapsed_us;
                }
                sum_us[kind] += result.elapsed_us;
                sum_bytes[kind] += result.poll_clock_bytes;
                valid[kind]++;
            }
            else
            {
                if(!result.observed_busy)
                {
                    busy_not_seen[kind]++;
                }
                failures[kind]++;
            }
        }
    }

    ret |= cli_printline(cli,
        "PLADC/PLSADC/PLAUX1 timing requires a witnessed BUSY->READY transition; no readiness/latch changes");
    for(uint8_t kind = 0u; kind < 3u; kind++)
    {
        if(!selected[kind])
        {
            continue;
        }
        snprintf(outline, CLI_LINESZ,
                 "%s requested:%u valid:%lu failures:%lu busy_not_seen:%lu",
                 adbms6830_timing_kind_name((adbms6830_timing_kind_t)kind),
                 (unsigned)repeat,
                 (unsigned long)valid[kind],
                 (unsigned long)failures[kind],
                 (unsigned long)busy_not_seen[kind]);
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "%s min:%luus avg:%luus max:%luus avg_poll_bytes:%lu",
                 adbms6830_timing_kind_name((adbms6830_timing_kind_t)kind),
                 (unsigned long)((valid[kind] != 0u) ? min_us[kind] : 0u),
                 (unsigned long)((valid[kind] != 0u) ?
                     (uint32_t)(sum_us[kind] / valid[kind]) : 0u),
                 (unsigned long)((valid[kind] != 0u) ? max_us[kind] : 0u),
                 (unsigned long)((valid[kind] != 0u) ?
                     (uint32_t)(sum_bytes[kind] / valid[kind]) : 0u));
        ret |= cli_printline(cli, outline);
    }
    return ret;
}

static int cli_run_config_repeat(adbms6830_driver_t *smb, uint16_t repeat_count)
{
    uint32_t write_a_fail = 0u;
    uint32_t write_b_fail = 0u;
    uint32_t readback_fail = 0u;
    uint32_t overall_fail = 0u;
    uint16_t cfga_mismatch_or = 0u;
    uint16_t cfgb_mismatch_or = 0u;
    uint16_t discharge_nonzero_or = 0u;
    int ret = 0;

    if((smb == NULL) || (repeat_count == 0u))
    {
        return cli_printline(cli, "Configuration stress state unavailable");
    }
    if(accumulator_balance_shadow_active(&data->acc))
    {
        return cli_printline(cli,
            "cfgrepeat refused: balancing/PWM shadow is nonzero; clear balancing first");
    }

    for(uint16_t cycle = 0u; cycle < repeat_count; cycle++)
    {
        adbms6830_config_cycle_result_t result;
        HAL_StatusTypeDef status =
            adbms6830_config_write_readback_cycle(smb, &result);

        if(result.write_cfga_status != HAL_OK)
        {
            write_a_fail++;
        }
        if(result.write_cfgb_status != HAL_OK)
        {
            write_b_fail++;
        }
        if(result.readback_status != HAL_OK)
        {
            readback_fail++;
        }
        if(status != HAL_OK)
        {
            overall_fail++;
        }
        cfga_mismatch_or |= result.cfga_mismatch_mask;
        cfgb_mismatch_or |= result.cfgb_mismatch_mask;
        discharge_nonzero_or |= result.discharge_nonzero_mask;

        if(result.discharge_nonzero_mask != 0u)
        {
            break;
        }
    }

    snprintf(outline, CLI_LINESZ,
             "CFGA/CFGB stress requested:%u overall_fail:%lu writeA_fail:%lu writeB_fail:%lu readback_fail:%lu",
             (unsigned)repeat_count,
             (unsigned long)overall_fail,
             (unsigned long)write_a_fail,
             (unsigned long)write_b_fail,
             (unsigned long)readback_fail);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "cfg mismatch OR A:0x%04X B:0x%04X discharge/PWM_nonzero:0x%04X balancing_commanded:0 safety_state_unchanged:1",
             cfga_mismatch_or, cfgb_mismatch_or, discharge_nonzero_or);
    ret |= cli_printline(cli, outline);
    return ret;
}

static int cli_run_raw_dump(adbms6830_driver_t *smb)
{
    uint8_t ic_count;
    uint32_t failures = 0u;
    int ret = 0;

    if(smb == NULL)
    {
        return cli_printline(cli, "Raw register state unavailable");
    }
    ic_count = smb_ic_count(smb);

    ret |= cli_printline(cli,
        "Raw ADBMS6830 register dump: six data bytes + command-counter/PEC bytes; values are not decoded or promoted");
    for(uint8_t reg = 0u; reg < ADBMS6830_RAW_REGISTER_COUNT; reg++)
    {
        adbms6830_raw_read_t result;
        HAL_StatusTypeDef status = adbms6830_read_raw_register(
            smb, (adbms6830_raw_register_t)reg, &result);
        if(status != HAL_OK)
        {
            failures++;
        }

        for(uint8_t ic = 0u; ic < ic_count; ic++)
        {
            const uint8_t *packet = result.packet[ic];
            uint16_t bit = (uint16_t)(1u << ic);
            snprintf(outline, CLI_LINESZ,
                     "%s IC%u raw:%02X %02X %02X %02X %02X %02X %02X %02X HAL:%s PEC:%s CTR:%s",
                     adbms6830_raw_register_name((adbms6830_raw_register_t)reg),
                     (unsigned)ic,
                     packet[0], packet[1], packet[2], packet[3],
                     packet[4], packet[5], packet[6], packet[7],
                     cli_hal_status_str(status),
                     ((result.pec_pass_mask & bit) != 0u) ? "PASS" : "FAIL",
                     ((result.counter_mismatch_mask & bit) != 0u) ? "FAIL" : "PASS");
            ret |= cli_printline(cli, outline);
        }
    }
    snprintf(outline, CLI_LINESZ,
             "Raw dump registers:%u failures:%lu safety_state_unchanged:1",
             (unsigned)ADBMS6830_RAW_REGISTER_COUNT,
             (unsigned long)failures);
    ret |= cli_printline(cli, outline);
    return ret;
}

static int cli_print_adbms_fault_classes(const app_data_t *app)
{
    static const struct
    {
        uint16_t mask;
        const char *name;
    } classes[] =
    {
        {AMS_ADBMS_FAULT_COMMUNICATION, "COMMUNICATION"},
        {AMS_ADBMS_FAULT_PEC, "PEC"},
        {AMS_ADBMS_FAULT_COMMAND_COUNTER, "COMMAND_COUNTER"},
        {AMS_ADBMS_FAULT_CONFIG_READBACK, "CONFIG_READBACK"},
        {AMS_ADBMS_FAULT_REFERENCE, "REFERENCE"},
        {AMS_ADBMS_FAULT_S_REDUNDANCY, "S_REDUNDANCY"},
        {AMS_ADBMS_FAULT_TEMP_UNAVAILABLE, "TEMP_UNAVAILABLE"},
        {AMS_ADBMS_FAULT_CELL_DATA_STALE, "CELL_DATA_STALE"},
        {AMS_ADBMS_FAULT_OPEN_WIRE, "OPEN_WIRE"},
        {AMS_ADBMS_FAULT_BALANCE_WRITE, "BALANCE_WRITE"},
        {AMS_ADBMS_FAULT_STATUS, "STATUS"},
        {AMS_ADBMS_FAULT_IDENTITY, "IDENTITY"},
        {AMS_ADBMS_FAULT_C_DATA_INVALID, "C_DATA_INVALID"},
        {AMS_ADBMS_FAULT_TOPOLOGY, "TOPOLOGY"},
        {AMS_ADBMS_FAULT_VOLTAGE_DEGRADED, "VOLTAGE_DEGRADED"},
        {AMS_ADBMS_FAULT_DEVICE_RESET, "DEVICE_RESET"}
    };
    int ret = 0;

    if(app == NULL)
    {
        return cli_printline(cli, "ADBMS fault classification unavailable");
    }

    snprintf(outline, CLI_LINESZ,
             "ADBMS faults active:0x%04X latched:0x%04X first:0x%04X transitions:%lu",
             app->adbms_fault_active_mask,
             app->adbms_fault_latched_mask,
             app->adbms_first_fault_mask,
             (unsigned long)app->adbms_fault_transition_count);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "fault ticks first:%lu last:%lu recovery:%lu comm_failures:%lu scan_ok:%d/%d",
             (unsigned long)app->adbms_first_fault_tick,
             (unsigned long)app->adbms_last_fault_tick,
             (unsigned long)app->adbms_last_recovery_tick,
             (unsigned long)app->adbms_comm_failure_count,
             app->adbms_last_voltage_scan_ok,
             app->adbms_voltage_scan_attempted);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "injection:0x%04X cell:%u device_resets:%lu mask:0x%04X C_authority:0x%04X valid:%d",
             app->adbms_fault_injection_mask,
             (unsigned)app->adbms_fault_injection_cell,
             (unsigned long)app->adbms_device_reset_count,
             app->adbms_last_device_reset_mask,
             app->adbms_c_authority_mask,
             app->adbms_c_authority_valid);
    ret |= cli_printline(cli, outline);
    for(size_t index = 0u; index < (sizeof(classes) / sizeof(classes[0])); index++)
    {
        if(((app->adbms_fault_active_mask | app->adbms_fault_latched_mask) &
            classes[index].mask) != 0u)
        {
            snprintf(outline, CLI_LINESZ,
                     "  %-20s active:%d latched:%d",
                     classes[index].name,
                     (app->adbms_fault_active_mask & classes[index].mask) != 0u,
                     (app->adbms_fault_latched_mask & classes[index].mask) != 0u);
            ret |= cli_printline(cli, outline);
        }
    }
    return ret;
}


static int cli_print_adbms_lifecycle(const app_data_t *app)
{
    uint32_t now;
    bool transition_valid;
    int ret = 0;

    if(app == NULL)
    {
        return cli_printline(cli, "ADBMS lifecycle unavailable");
    }

    now = osKernelGetTickCount();
    transition_valid = app->adbms_lifecycle_transition_count != 0u;
    snprintf(outline, CLI_LINESZ,
             "ADBMS lifecycle current:%s previous:%s reason:%s transitions:%lu",
             ams_adbms_state_str(app->adbms_lifecycle_state),
             ams_adbms_state_str(app->adbms_lifecycle_previous_state),
             ams_adbms_state_reason_str(app->adbms_lifecycle_reason),
             (unsigned long)app->adbms_lifecycle_transition_count);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "lifecycle now:%lu last_transition:%lu age:%lu valid:%d",
             (unsigned long)now,
             (unsigned long)app->adbms_lifecycle_last_transition_tick,
             (unsigned long)(transition_valid ?
                             (now - app->adbms_lifecycle_last_transition_tick) : 0u),
             transition_valid);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "first_fault:0x%04X at:%lu last_fault:%lu recovery:%lu",
             app->adbms_first_fault_mask,
             (unsigned long)app->adbms_first_fault_tick,
             (unsigned long)app->adbms_last_fault_tick,
             (unsigned long)app->adbms_last_recovery_tick);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "device resets:%lu last_mask:0x%04X fault_transitions:%lu injection:0x%04X cell:%u",
             (unsigned long)app->adbms_device_reset_count,
             app->adbms_last_device_reset_mask,
             (unsigned long)app->adbms_fault_transition_count,
             app->adbms_fault_injection_mask,
             (unsigned)app->adbms_fault_injection_cell);
    ret |= cli_printline(cli, outline);
    return ret;
}

static int cli_print_adbms_ages(const app_data_t *app)
{
    static const struct
    {
        const char *name;
        size_t offset;
    } fields[] =
    {
        {"C", offsetof(app_data_t, adbms_c_last_valid_tick)},
        {"S", offsetof(app_data_t, adbms_s_last_valid_tick)},
        {"TEMP", offsetof(app_data_t, adbms_temp_last_valid_tick)},
        {"STATUS", offsetof(app_data_t, adbms_status_last_valid_tick)},
        {"CONFIG", offsetof(app_data_t, adbms_config_last_valid_tick)},
        {"IDENTITY", offsetof(app_data_t, adbms_identity_last_valid_tick)}
    };
    uint32_t now;
    int ret = 0;

    if(app == NULL)
    {
        return cli_printline(cli, "ADBMS measurement ages unavailable");
    }

    now = osKernelGetTickCount();
    snprintf(outline, CLI_LINESZ, "ADBMS ages now:%lu units:kernel_ticks", (unsigned long)now);
    ret |= cli_printline(cli, outline);
    for(size_t index = 0u; index < (sizeof(fields) / sizeof(fields[0])); index++)
    {
        const uint32_t *tick_ptr =
            (const uint32_t *)((const uint8_t *)app + fields[index].offset);
        uint32_t tick = *tick_ptr;

        if(tick == 0u)
        {
            snprintf(outline, CLI_LINESZ, "  %-8s last:never age:INVALID", fields[index].name);
        }
        else
        {
            snprintf(outline, CLI_LINESZ, "  %-8s last:%lu age:%lu",
                     fields[index].name,
                     (unsigned long)tick,
                     (unsigned long)(now - tick));
        }
        ret |= cli_printline(cli, outline);
    }
    return ret;
}

static int cli_print_adbms_authority(const app_data_t *app)
{
    static const struct
    {
        uint16_t mask;
        const char *name;
    } checks[] =
    {
        {AMS_ADBMS_C_AUTH_TRANSPORT, "TRANSPORT"},
        {AMS_ADBMS_C_AUTH_PEC, "PEC"},
        {AMS_ADBMS_C_AUTH_COUNTER, "COUNTER"},
        {AMS_ADBMS_C_AUTH_CODES, "CODES"},
        {AMS_ADBMS_C_AUTH_RANGE, "RANGE"},
        {AMS_ADBMS_C_AUTH_SLEW, "SLEW"},
        {AMS_ADBMS_C_AUTH_FRESH, "FRESH"},
        {AMS_ADBMS_C_AUTH_CONFIG, "CONFIG"},
        {AMS_ADBMS_C_AUTH_REFERENCE, "REFERENCE"},
        {AMS_ADBMS_C_AUTH_IDENTITY, "IDENTITY"},
        {AMS_ADBMS_C_AUTH_TOPOLOGY, "TOPOLOGY"}
    };
    int ret = 0;

    if(app == NULL)
    {
        return cli_printline(cli, "ADBMS C authority unavailable");
    }

    snprintf(outline, CLI_LINESZ,
             "C authority mask:0x%04X required:0x%04X valid:%d voltage_mode:%s",
             app->adbms_c_authority_mask,
             AMS_ADBMS_C_AUTH_REQUIRED_MASK,
             app->adbms_c_authority_valid,
             cli_voltage_mode_str());
    ret |= cli_printline(cli, outline);
    for(size_t index = 0u; index < (sizeof(checks) / sizeof(checks[0])); index++)
    {
        snprintf(outline, CLI_LINESZ, "  %-12s %s",
                 checks[index].name,
                 ((app->adbms_c_authority_mask & checks[index].mask) != 0u) ?
                     "PASS" : "FAIL");
        ret |= cli_printline(cli, outline);
    }
    ret |= cli_printline(cli,
        "C_AUTHORITY_VALID is separate from REDUNDANT_VOLTAGE_VALID; no automatic fallback exists");
    return ret;
}

static bool cli_is_adbms_event(uint16_t event)
{
    return (event == AMS_FAULT_LOG_ADBMS_DIAG_FAIL) ||
           (event == AMS_FAULT_LOG_ADBMS_FAULT_CHANGE) ||
           (event == AMS_FAULT_LOG_ADBMS_STATE_TRANSITION) ||
           (event == AMS_FAULT_LOG_ADBMS_DEVICE_RESET) ||
           (event == AMS_FAULT_LOG_ADBMS_FAULT_INJECTION);
}

static int cli_print_adbms_events(void)
{
    const ams_fault_log_t *log = ams_fault_log_get();
    const adbms6830_driver_t *smb = (data != NULL) ? &data->acc.smb : NULL;
    uint32_t shown = 0u;
    int ret = 0;

    if(log == NULL)
    {
        return cli_printline(cli, "ADBMS event history unavailable");
    }

    snprintf(outline, CLI_LINESZ,
             "ADBMS retained events log_count:%lu depth:%u boot:%lu",
             (unsigned long)log->count,
             (unsigned)AMS_FAULT_LOG_DEPTH,
             (unsigned long)log->boot_sequence);
    ret |= cli_printline(cli, outline);
    for(uint32_t i = 0u; i < log->count; i++)
    {
        uint32_t idx =
            (log->write_index + AMS_FAULT_LOG_DEPTH - log->count + i) %
            AMS_FAULT_LOG_DEPTH;
        const ams_fault_log_entry_t *entry = &log->entry[idx];

        if(!cli_is_adbms_event(entry->event))
        {
            continue;
        }
        snprintf(outline, CLI_LINESZ,
                 "seq:%lu tick:%lu event:%s reason:0x%04X arg0:0x%08lX arg1:0x%08lX",
                 (unsigned long)entry->sequence,
                 (unsigned long)entry->tick,
                 ams_fault_log_event_str(entry->event),
                 entry->reason,
                 (unsigned long)entry->arg0,
                 (unsigned long)entry->arg1);
        ret |= cli_printline(cli, outline);
        shown++;
    }
    if(shown == 0u)
    {
        ret |= cli_printline(cli, "No retained ADBMS events");
    }

    if(smb != NULL)
    {
        uint32_t total_count;
        uint8_t count;
        uint8_t write_index;
        uint16_t guard_mask;
        uint16_t sticky_guard_mask;

        adbms_spi_lock();
        total_count = smb->cfgb_write_total_count;
        count = smb->cfgb_write_history_count;
        write_index = smb->cfgb_write_history_index;
        guard_mask = smb->health.config_write_guard_fault_mask;
        sticky_guard_mask = smb->health.sticky_config_write_guard_fault_mask;
        memcpy(cli_cfgb_write_history_snapshot,
               smb->cfgb_write_history,
               sizeof(cli_cfgb_write_history_snapshot));
        adbms_spi_unlock();

        if(count > ADBMS6830_CFGB_WRITE_HISTORY_DEPTH)
        {
            count = ADBMS6830_CFGB_WRITE_HISTORY_DEPTH;
        }
        snprintf(outline, CLI_LINESZ,
                 "WRCFGB history total:%lu count:%u depth:%u guard:0x%04X sticky:0x%04X",
                 (unsigned long)total_count,
                 (unsigned)count,
                 (unsigned)ADBMS6830_CFGB_WRITE_HISTORY_DEPTH,
                 guard_mask,
                 sticky_guard_mask);
        ret |= cli_printline(cli, outline);

        uint8_t start = (uint8_t)((write_index +
                                  ADBMS6830_CFGB_WRITE_HISTORY_DEPTH - count) %
                                 ADBMS6830_CFGB_WRITE_HISTORY_DEPTH);
        for(uint8_t i = 0u; i < count; i++)
        {
            uint8_t index = (uint8_t)((start + i) %
                                      ADBMS6830_CFGB_WRITE_HISTORY_DEPTH);
            const adbms6830_cfgb_write_event_t *event =
                &cli_cfgb_write_history_snapshot[index];

            snprintf(outline, CLI_LINESZ,
                     "cfgb seq:%lu tick:%lu reason:%s status:%s string:%u timer:0x%04X balance:0x%04X rejected:0x%04X",
                     (unsigned long)event->sequence,
                     (unsigned long)event->tick_ms,
                     adbms6830_cfgb_write_reason_str(event->reason),
                     cli_hal_status_str(event->status),
                     (unsigned)event->string,
                     event->timer_nonzero_mask,
                     event->balance_shadow_mask,
                     event->rejected_mask);
            ret |= cli_printline(cli, outline);

            for(uint8_t ic = 0u; ic < event->ic_count; ic++)
            {
                const uint8_t *p = event->payload[ic];
                snprintf(outline, CLI_LINESZ,
                         "  IC%u CFGB:%02X %02X %02X %02X %02X %02X DTMEN:%u DTRNG:%u DCTO:%u DCC:0x%04X",
                         (unsigned)ic,
                         p[0], p[1], p[2], p[3], p[4], p[5],
                         (unsigned)((p[3] >> 7u) & 0x01u),
                         (unsigned)((p[3] >> 6u) & 0x01u),
                         (unsigned)(p[3] & 0x3Fu),
                         (unsigned)((uint16_t)p[4] |
                                    ((uint16_t)p[5] << 8u)));
                ret |= cli_printline(cli, outline);
            }
        }
    }
    ret |= cli_printline(cli, "Use 'fault log' for all system events; clearing is intentionally global/service-only");
    return ret;
}

static int cli_print_adbms_lockdiag(const app_data_t *app)
{
    int ret = 0;

    if(app == NULL)
    {
        return cli_printline(cli, "ADBMS SPI owner diagnostics unavailable");
    }

    snprintf(outline, CLI_LINESZ,
             "SPI owner lock acquires:%lu contention:%lu violations:%lu",
             (unsigned long)app->adbms_spi_lock_acquire_count,
             (unsigned long)app->adbms_spi_lock_contention_count,
             (unsigned long)app->adbms_spi_lock_violation_count);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "SPI owner max_wait:%lu ticks max_hold:%lu ticks scan_active:%d",
             (unsigned long)app->adbms_spi_lock_max_wait_ticks,
             (unsigned long)app->adbms_spi_lock_max_hold_ticks,
             app->adbms_scan_active);
    ret |= cli_printline(cli, outline);
    ret |= cli_printline(cli,
        "All periodic/CLI ADBMS traffic uses the recursive owner mutex; CLI printing occurs after transactions");
    return ret;
}

static int cli_handle_adbms_injection(int argc, char *argv[])
{
#if AMS_ENABLE_ADBMS_FAULT_INJECTION
    static const struct
    {
        const char *name;
        uint16_t mask;
    } faults[] =
    {
        {"comm", AMS_ADBMS_FAULT_COMMUNICATION},
        {"pec", AMS_ADBMS_FAULT_PEC},
        {"counter", AMS_ADBMS_FAULT_COMMAND_COUNTER},
        {"config", AMS_ADBMS_FAULT_CONFIG_READBACK},
        {"reference", AMS_ADBMS_FAULT_REFERENCE},
        {"s", AMS_ADBMS_FAULT_S_REDUNDANCY},
        {"temp", AMS_ADBMS_FAULT_TEMP_UNAVAILABLE},
        {"stale", AMS_ADBMS_FAULT_CELL_DATA_STALE},
        {"openwire", AMS_ADBMS_FAULT_OPEN_WIRE},
        {"balance", AMS_ADBMS_FAULT_BALANCE_WRITE},
        {"status", AMS_ADBMS_FAULT_STATUS},
        {"identity", AMS_ADBMS_FAULT_IDENTITY},
        {"invalidc", AMS_ADBMS_FAULT_C_DATA_INVALID},
        {"topology", AMS_ADBMS_FAULT_TOPOLOGY},
        {"reset", AMS_ADBMS_FAULT_DEVICE_RESET}
    };
    uint16_t mask = 0u;
    uint8_t cell = 0u;
    int ret = 0;

    if((argc < 3) || (argv[2] == NULL) || !strcmp(argv[2], "status"))
    {
        snprintf(outline, CLI_LINESZ,
                 "ADBMS software injection mask:0x%04X cell:%u compiled:1",
                 data->adbms_fault_injection_mask,
                 (unsigned)data->adbms_fault_injection_cell);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli,
            "Usage: spi inject [clear|comm|pec|counter|config|reference|s|temp|stale|openwire|balance|status|identity|invalidc [1-15]|topology|reset]");
        return ret;
    }

    if(!strcmp(argv[2], "clear"))
    {
        taskENTER_CRITICAL();
        data->adbms_fault_injection_mask = 0u;
        data->adbms_fault_injection_cell = 0u;
        taskEXIT_CRITICAL();
        ams_fault_log_event(AMS_FAULT_LOG_ADBMS_FAULT_INJECTION, 0u, 0u, 0u);
        ret |= cli_printline(cli,
            "ADBMS software injection cleared; normal recovery still requires valid fresh diagnostics");
        return ret;
    }

    for(size_t index = 0u; index < (sizeof(faults) / sizeof(faults[0])); index++)
    {
        if(!strcmp(argv[2], faults[index].name))
        {
            mask = faults[index].mask;
            break;
        }
    }
    if(mask == 0u)
    {
        ret |= cli_printline(cli, "Unknown injection fault; use 'spi inject status'");
        return ret;
    }

    if(mask == AMS_ADBMS_FAULT_C_DATA_INVALID)
    {
        int parsed_cell = 1;
        if((argc >= 4) &&
           !cli_parse_int_range(argv[3], 1, (int)data->acc.smb.monitored_cell_count,
                                &parsed_cell))
        {
            ret |= cli_printline(cli, "Usage: spi inject invalidc [cell 1-15]");
            return ret;
        }
        cell = (uint8_t)parsed_cell;
    }

    taskENTER_CRITICAL();
    data->adbms_fault_injection_mask = mask;
    data->adbms_fault_injection_cell = cell;
    data->adbms_diag_fault = true;
    data->adbms_last_diag_status = HAL_ERROR;
    taskEXIT_CRITICAL();
    set_bms(false);
    ams_fault_log_event(AMS_FAULT_LOG_ADBMS_FAULT_INJECTION,
                        mask,
                        cell,
                        1u);
    snprintf(outline, CLI_LINESZ,
             "Injected software validation fault 0x%04X cell:%u; bus traffic is unmodified and BMS_OK forced low",
             mask, (unsigned)cell);
    ret |= cli_printline(cli, outline);
    ret |= cli_printline(cli,
        "Fault becomes classified on the next ADBMS scan; clear only removes injection, not real or latched evidence");
    return ret;
#else
    (void)argc;
    (void)argv;
    return cli_printline(cli,
        "ADBMS fault injection is compiled out in this profile (AMS_ENABLE_ADBMS_FAULT_INJECTION=0)");
#endif
}

static int cli_print_cs_snapshot_table(adbms6830_driver_t *smb)
{
    uint8_t ic_count = smb_ic_count(smb);
    uint16_t monitored_mask = (smb->monitored_cell_count == 16u) ? UINT16_MAX :
                              (uint16_t)((1UL << smb->monitored_cell_count) - 1UL);
    int ret = 0;

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        uint16_t c_valid = smb->last_cell_updated_mask[ic] & monitored_mask;
        uint16_t s_valid = smb->last_scell_updated_mask[ic] & monitored_mask;
        uint16_t csflt = smb->diag[ic].cs_flt_mask & monitored_mask;
        snprintf(outline, CLI_LINESZ,
                 "IC%u Cvalid:0x%04X Svalid:0x%04X CSFLT:0x%04X",
                 (unsigned)ic, c_valid, s_valid, csflt);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli,
            "Cell      C-ADC mV       S-ADC mV       delta mV     CSFLT");
        for(uint8_t cell = 0u; cell < smb->monitored_cell_count; cell++)
        {
            uint16_t bit = (uint16_t)(1u << cell);
            bool c_ok = (c_valid & bit) != 0u;
            bool s_ok = (s_valid & bit) != 0u;
            if(c_ok && s_ok)
            {
                int32_t c_uv = ((int32_t)smb->ics[ic].cell.c_codes[cell] + 10000) * 150;
                int32_t s_uv = ((int32_t)smb->ics[ic].scell.sc_codes[cell] + 10000) * 150;
                int32_t delta_uv = c_uv - s_uv;
                char c_mv[20], s_mv[20], delta_mv[20];
                if(delta_uv < 0)
                {
                    delta_uv = -delta_uv;
                }
                cli_microvolts_mv_string(c_uv, c_mv, sizeof(c_mv));
                cli_microvolts_mv_string(s_uv, s_mv, sizeof(s_mv));
                cli_microvolts_mv_string(delta_uv, delta_mv, sizeof(delta_mv));
                snprintf(outline, CLI_LINESZ,
                         "C%02u    %11s    %11s    %11s       %u",
                         (unsigned)(cell + 1u), c_mv, s_mv, delta_mv,
                         (unsigned)((csflt & bit) != 0u));
            }
            else
            {
                snprintf(outline, CLI_LINESZ,
                         "C%02u    C:%s S:%s                              %u",
                         (unsigned)(cell + 1u), c_ok ? "valid" : "invalid",
                         s_ok ? "valid" : "invalid",
                         (unsigned)((csflt & bit) != 0u));
            }
            ret |= cli_printline(cli, outline);
        }
    }
    return ret;
}

static int cli_run_snapshot(adbms6830_driver_t *smb)
{
    HAL_StatusTypeDef sid_status;
    HAL_StatusTypeDef config_status;
    HAL_StatusTypeDef cs_status;
    HAL_StatusTypeDef diag_status;
    HAL_StatusTypeDef tempbus_status;
    bool status_transport_ok;
    bool status_non_cs_ok;
    bool status_full_ok;
    bool s_redundancy_ok;
    const adbms6830_diag_health_t *health;
    uint8_t ic_count;
    int ret = 0;

    if(smb == NULL)
    {
        return cli_printline(cli, "ADBMS snapshot unavailable");
    }

    sid_status = adbms6830_read_sid(smb);
    config_status = adbms6830_verify_config_readback(smb);
    cs_status = adbms6830_capture_cs_comparison(smb);
    diag_status = adbms6830_refresh_diagnostics(smb);
    tempbus_status = adbms6830_temp_bus_idle_capture(smb);
    health = adbms6830_diag_health_get(smb);
    ic_count = smb_ic_count(smb);
    status_transport_ok = adbms6830_diagnostic_transport_ok(smb);
    status_non_cs_ok = adbms6830_non_cs_diagnostics_ok(smb);
    status_full_ok = adbms6830_safety_diagnostics_ok(smb);
    s_redundancy_ok = (health != NULL) && (health->cs_fault_ic_mask == 0u);

    snprintf(outline, CLI_LINESZ,
             "ADBMS snapshot voltage_mode:%s degraded:%d S_policy:%s auto_temp_scan:%u",
             cli_voltage_mode_str(),
             (AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP),
             (AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP) ?
                 "diagnostic_only" : "required",
             (unsigned)AMS_ENABLE_AUTO_TEMP_MUX_SCAN);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "snapshot stages SID:%s CFG:%s CS:%s STATUS_XFER:%s TEMPBUS_IDLE:%s safety_state_unchanged:1",
             cli_hal_status_str(sid_status), cli_hal_status_str(config_status),
             cli_hal_status_str(cs_status), status_transport_ok ? "OK" : "ERROR",
             cli_hal_status_str(tempbus_status));
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "snapshot status api:%s non_cs:%s S_redundancy:%s full_safety:%s",
             cli_hal_status_str(diag_status),
             status_non_cs_ok ? "OK" : "ERROR",
             s_redundancy_ok ? "OK" : "ERROR",
             status_full_ok ? "OK" : "ERROR");
    ret |= cli_printline(cli, outline);

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        const adbms6830_ic_diag_t *diag = &smb->diag[ic];
        snprintf(outline, CLI_LINESZ,
                 "IC%u SID:%s id:0x%02X serial:%02X%02X%02X%02X%02X%02X refs:%d VREF2:%dmV VD:%dmV VA:%dmV VRES:%dmV die:%ddC",
                 (unsigned)ic, diag->sid_valid ? "ok" : "bad", diag->device_id,
                 diag->sid[5], diag->sid[4], diag->sid[3],
                 diag->sid[2], diag->sid[1], diag->sid[0],
                 diag->reference_values_valid, (int)diag->vref2_mv,
                 (int)diag->vd_mv, (int)diag->va_mv,
                 (int)diag->vres_mv, (int)diag->die_temp_deci_c);
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "IC%u status cs:0x%04X ov:0x%04X uv:0x%04X supply:%u/%u/%u/%u memory:%u/%u/%u/%u digital:%u spi:%u sleep:%u osc:%u",
                 (unsigned)ic, diag->cs_flt_mask, diag->cell_ov_mask,
                 diag->cell_uv_mask, diag->va_ov, diag->va_uv,
                 diag->vd_ov, diag->vd_uv, diag->ced, diag->cmed,
                 diag->sed, diag->smed, diag->vde, diag->spiflt,
                 diag->sleep, diag->oscchk);
        ret |= cli_printline(cli, outline);
    }
    ret |= cli_print_cs_snapshot_table(smb);

    const adbms6830_temp_bus_debug_t *tbus = &smb->temp_bus_debug;
    snprintf(outline, CLI_LINESZ,
             "TEMPBUS idle valid:0x%04X G4/SDA:0x%04X G5/SCL:0x%04X PECfail:0x%04X CTR:0x%04X",
             tbus->transport_valid_mask, tbus->gpio4_code_valid_mask,
             tbus->gpio5_code_valid_mask, tbus->pec_fail_mask,
             tbus->counter_mismatch_mask);
    ret |= cli_printline(cli, outline);
    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        char g4_mv[20], g5_mv[20];
        cli_temp_raw_mv_string(tbus->gpio4_raw[ic], g4_mv, sizeof(g4_mv));
        cli_temp_raw_mv_string(tbus->gpio5_raw[ic], g5_mv, sizeof(g5_mv));
        snprintf(outline, CLI_LINESZ,
                 "IC%u GPIO4/SDA:%smV %s GPIO5/SCL:%smV %s",
                 (unsigned)ic, g4_mv,
                 cli_temp_bus_level(tbus->gpio4_raw[ic],
                     (tbus->gpio4_code_valid_mask & (uint16_t)(1u << ic)) != 0u),
                 g5_mv,
                 cli_temp_bus_level(tbus->gpio5_raw[ic],
                     (tbus->gpio5_code_valid_mask & (uint16_t)(1u << ic)) != 0u));
        ret |= cli_printline(cli, outline);
    }

    if(health != NULL)
    {
        snprintf(outline, CLI_LINESZ,
                 "health PEC last:0x%04X sticky:0x%04X counter_last:0x%04X sticky:0x%04X cfg:0x%04X ref:0x%04X status:0x%04X",
                 health->last_pec_fail_mask, health->sticky_pec_fail_mask,
                 health->last_cmd_counter_mismatch_mask,
                 health->sticky_cmd_counter_mismatch_mask,
                 health->config_mismatch_mask,
                 (uint16_t)(health->reference_invalid_ic_mask |
                            health->reference_fault_ic_mask),
                 (uint16_t)(health->status_invalid_ic_mask |
                            health->status_fault_ic_mask));
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "health unexpected_counter_reset current:0x%04X sticky:0x%04X IC0_count:%lu",
                 health->unexpected_counter_reset_mask,
                 health->sticky_unexpected_counter_reset_mask,
                 (unsigned long)health->unexpected_counter_reset_count[0]);
        ret |= cli_printline(cli, outline);
    }

    uint32_t expected_fp = adbms6830_config_expected_fingerprint(smb);
    uint32_t readback_fp = adbms6830_config_readback_fingerprint(smb);
    snprintf(outline, CLI_LINESZ,
             "config fingerprint expected:0x%08lX readback:0x%08lX match:%d",
             (unsigned long)expected_fp,
             (unsigned long)readback_fp,
             (expected_fp != 0u) && (expected_fp == readback_fp));
    ret |= cli_printline(cli, outline);
    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        snprintf(outline, CLI_LINESZ,
                 "balance shadow SMB%u planned:0x%04X age:%lu applied:0",
                 (unsigned)seg,
                 data->adbms_balance_shadow_plan[seg],
                 (unsigned long)((data->adbms_balance_shadow_plan_tick == 0u) ?
                                 0u : (osKernelGetTickCount() -
                                       data->adbms_balance_shadow_plan_tick)));
        ret |= cli_printline(cli, outline);
    }
    ret |= cli_print_adbms_fault_classes(data);
    ret |= cli_print_adbms_lifecycle(data);
    ret |= cli_print_adbms_ages(data);
    ret |= cli_print_adbms_authority(data);
    ret |= cli_print_adbms_lockdiag(data);
    return ret;
}

static int cli_run_recovery(adbms6830_driver_t *smb, uint16_t idle_ms)
{
    const adbms6830_recovery_debug_t *dbg;
    HAL_StatusTypeDef status;
    int ret = 0;

    if(smb == NULL)
    {
        return cli_printline(cli, "ADBMS recovery state unavailable");
    }

    ams_adbms_transition_state(data,
                               AMS_ADBMS_STATE_RECOVERING,
                               AMS_ADBMS_STATE_REASON_RECOVERY_REQUEST,
                               osKernelGetTickCount());
    snprintf(outline, CLI_LINESZ,
             "Recovery test holding ADBMS bus lock idle for %ums; periodic scan is intentionally paused",
             (unsigned)idle_ms);
    ret |= cli_printline(cli, outline);
    osDelay(idle_ms);
    /* This diagnostic deliberately creates an interval long enough for the
     * remote chain or bridge to lose command-counter state.  Do not classify
     * that expected uncertainty as an uncommanded brownout; let the first
     * PEC-valid recovery packet establish the new counter baseline. */
    adbms6830_resync_command_counter_tracking(smb);
    status = adbms6830_recovery_check(smb);
    ams_adbms_transition_state(data,
                               (status == HAL_OK) ? AMS_ADBMS_STATE_IDENTIFIED :
                                                    AMS_ADBMS_STATE_FAULTED,
                               AMS_ADBMS_STATE_REASON_RECOVERY_RESULT,
                               osKernelGetTickCount());
    dbg = &smb->recovery_debug;

    snprintf(outline, CLI_LINESZ,
             "Recovery status:%s wake:%s SID:%s writeA:%s writeB:%s",
             cli_hal_status_str(status), cli_hal_status_str(dbg->wake_status),
             cli_hal_status_str(dbg->sid_status),
             cli_hal_status_str(dbg->write_cfga_status),
             cli_hal_status_str(dbg->write_cfgb_status));
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "Recovery config:%s diag:%s C:%s readiness_latches_unchanged:1",
             cli_hal_status_str(dbg->config_status),
             cli_hal_status_str(dbg->diagnostic_status),
             cli_hal_status_str(dbg->cadc_status));
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "Recovery masks SID:0x%04X CFG:0x%04X REF:0x%04X STATUS:0x%04X CvalidIC:0x%04X",
             dbg->sid_valid_mask, dbg->config_mismatch_mask,
             dbg->reference_fault_mask, dbg->status_fault_mask,
             dbg->cadc_valid_ic_mask);
    ret |= cli_printline(cli, outline);
    ret |= cli_printline(cli,
        "Recovery command-counter tracking resynchronized for intentional idle/disconnect interval");
    return ret;
}




static int cli_run_temperature_emulator(int argc, char *argv[])
{
    static const int preset_c[] = {-20, 0, 25, 60, 100, 120};
    int single_temp = 0;
    bool one = false;
    int ret = 0;

    if((argc >= 3) && (argv[2] != NULL))
    {
        if(!cli_parse_int_range(argv[2], -20, 120, &single_temp))
        {
            return cli_printline(cli, "Usage: spi tempemu [temperature_C -20..120]");
        }
        one = true;
    }

    ret |= cli_printline(cli,
        "TEMP_EMU software-only: thermistor forward model -> ADBMS raw -> production inverse LUT");
    size_t count = one ? 1u : (sizeof(preset_c) / sizeof(preset_c[0]));
    for(size_t index = 0u; index < count; index++)
    {
        int input_c = one ? single_temp : preset_c[index];
        int16_t raw = 0;
        bool encoded = thermistor_adbms_raw_from_temperature_c(
            (float)input_c,
            THERMISTOR_NOMINAL_VREG_V,
            &raw);
        thermistor_result_t decoded = encoded ?
            thermistor_from_adbms_raw(raw, THERMISTOR_NOMINAL_VREG_V) :
            thermistor_from_adbms_raw(THERMISTOR_ADBMS_CLEAR_CODE,
                                      THERMISTOR_NOMINAL_VREG_V);
        int decoded_deci = decoded.valid ?
            cli_scaled_int(decoded.temperature_c, 10) : 0;
        int voltage_mv = decoded.valid ?
            cli_scaled_int(decoded.divider_voltage_v, 1000) : 0;
        int decoded_abs = (decoded_deci < 0) ? -decoded_deci : decoded_deci;
        char decoded_c[16];
        (void)snprintf(decoded_c, sizeof(decoded_c), "%s%d.%d",
                       (decoded_deci < 0) ? "-" : "",
                       decoded_abs / 10,
                       decoded_abs % 10);

        snprintf(outline, CLI_LINESZ,
                 "TEMP_EMU input:%dC encoded:%d raw:%d voltage:%dmV decoded:%sC status:%s valid:%d",
                 input_c,
                 encoded,
                 (int)raw,
                 voltage_mv,
                 decoded_c,
                 thermistor_status_str(decoded.status),
                 decoded.valid);
        ret |= cli_printline(cli, outline);
    }

    const int16_t fault_raw[] =
    {
        THERMISTOR_ADBMS_CLEAR_CODE,
        THERMISTOR_ADBMS_RESET_CODE,
        (int16_t)-10000,
        (int16_t)23000
    };
    const char *fault_name[] = {"clear_sentinel", "reset_sentinel", "open_like", "short_like"};
    for(size_t index = 0u; index < (sizeof(fault_raw) / sizeof(fault_raw[0])); index++)
    {
        thermistor_result_t result =
            thermistor_from_adbms_raw(fault_raw[index], THERMISTOR_NOMINAL_VREG_V);
        snprintf(outline, CLI_LINESZ,
                 "TEMP_EMU fault:%s raw:%d status:%s valid:%d",
                 fault_name[index],
                 (int)fault_raw[index],
                 thermistor_status_str(result.status),
                 result.valid);
        ret |= cli_printline(cli, outline);
    }

    uint8_t route_failures = 0u;
    for(uint8_t sensor = 0u; sensor < ADBMS6830_TEMP_SENSOR_COUNT; sensor++)
    {
        adbms6830_temp_route_t route;
        bool route_ok = adbms6830_temp_sensor_route(sensor, &route);
        bool expected = route_ok &&
                        (route.mux_idx == (sensor / 8u)) &&
                        (route.mux_address == (uint8_t)(0x4Cu + (sensor / 8u))) &&
                        (route.switch_index == (sensor % 8u)) &&
                        (route.switch_mask == (uint8_t)(1u << (sensor % 8u))) &&
                        (route.gpio_channel == (sensor / 8u));
        if(!expected)
        {
            route_failures++;
        }
        snprintf(outline, CLI_LINESZ,
                 "TEMP_EMU_ROUTE,sensor,%u,mux,%u,addr,0x%02X,switch,%u,mask,0x%02X,gpio,%u,result,%s",
                 (unsigned)sensor,
                 route_ok ? (unsigned)route.mux_idx : 0u,
                 route_ok ? (unsigned)route.mux_address : 0u,
                 route_ok ? (unsigned)route.switch_index : 0u,
                 route_ok ? (unsigned)route.switch_mask : 0u,
                 route_ok ? (unsigned)(route.gpio_channel + 1u) : 0u,
                 expected ? "PASS" : "FAIL");
        ret |= cli_printline(cli, outline);
    }

    uint8_t ack_packet[RX_DATA] = {0};
    ack_packet[0] = 0x67u; /* START + slave ACK */
    ack_packet[2] = 0x77u; /* blank/SDA high + slave ACK (real hardware) */
    bool ack_high_ok = adbms6830_comm_write_acknowledged(ack_packet);
    ack_packet[2] = 0x07u; /* blank/SDA low + slave ACK (also valid) */
    bool ack_low_ok = adbms6830_comm_write_acknowledged(ack_packet);
    ack_packet[0] = 0x6Fu; /* synthetic address NACK */
    bool address_nack_rejected =
        !adbms6830_comm_write_acknowledged(ack_packet);
    ack_packet[0] = 0x67u;
    ack_packet[2] = 0x7Fu; /* blank/SDA high + synthetic data NACK */
    bool data_nack_rejected =
        !adbms6830_comm_write_acknowledged(ack_packet);
    snprintf(outline, CLI_LINESZ,
             "TEMP_EMU_ACK,ack_high,%s,ack_low,%s,address_nack_rejected,%s,data_nack_rejected,%s",
             ack_high_ok ? "PASS" : "FAIL",
             ack_low_ok ? "PASS" : "FAIL",
             address_nack_rejected ? "PASS" : "FAIL",
             data_nack_rejected ? "PASS" : "FAIL");
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "TEMP_EMU_CHAIN,mux0,ACK,mux1,NACK,mux2,ACK,publishable_mux_mask,0x%02X,route_failures,%u",
             0x05u,
             (unsigned)route_failures);
    ret |= cli_printline(cli, outline);
    ret |= cli_printline(cli,
        "TEMP_EMU stale/open/short/NACK cases remain non-publishable; one failed mux must not relabel previous AUX data");
    ret |= cli_printline(cli,
        "TEMP_EMU does not drive WRCOMM/STCOMM and does not replace physical ACK or resistor-rework validation");
    return ret;
}

static int cli_handle_mapcheck(adbms6830_driver_t *smb, int argc, char *argv[])
{
    uint16_t monitored_mask;
    HAL_StatusTypeDef status;
    int ret = 0;

    if((smb == NULL) || (smb_ic_count(smb) != 1u))
    {
        return cli_printline(cli, "mapcheck requires exactly one logical SMB");
    }
    if(data->adbms_balance_active)
    {
        return cli_printline(cli,
            "mapcheck refused while balancing is active; inhibit/clear balancing first");
    }

    monitored_mask = (smb->monitored_cell_count == 16u) ? UINT16_MAX :
                     (uint16_t)((1UL << smb->monitored_cell_count) - 1UL);

    if((argc < 3) || (argv[2] == NULL))
    {
        ret |= cli_printline(cli,
            "Usage: spi mapcheck [baseline|verify <cell 1-15> [min_delta_mV 1-500]|clear]");
        return ret;
    }

    if(!strcmp(argv[2], "clear"))
    {
        cli_mapcheck_baseline_valid = false;
        cli_mapcheck_baseline_mask = 0u;
        cli_mapcheck_baseline_tick = 0u;
        memset(cli_mapcheck_baseline_uv, 0, sizeof(cli_mapcheck_baseline_uv));
        return cli_printline(cli, "C-channel mapping baseline cleared");
    }

    if(!strcmp(argv[2], "baseline"))
    {
        status = adbms6830_capture_c_adc(smb);
        uint16_t valid_mask =
            (uint16_t)(smb->last_cell_updated_mask[0] &
                       (uint16_t)~smb->last_cell_pec_mask[0] &
                       monitored_mask);

        if((status != HAL_OK) || (valid_mask != monitored_mask))
        {
            snprintf(outline, CLI_LINESZ,
                     "mapcheck baseline FAILED status:%s valid:0x%04X expected:0x%04X",
                     cli_hal_status_str(status), valid_mask, monitored_mask);
            return cli_printline(cli, outline);
        }

        for(uint8_t cell = 0u; cell < smb->monitored_cell_count; cell++)
        {
            cli_mapcheck_baseline_uv[cell] =
                ((int32_t)smb->ics[0].cell.c_codes[cell] + 10000) * 150;
        }
        cli_mapcheck_baseline_mask = valid_mask;
        cli_mapcheck_baseline_tick = osKernelGetTickCount();
        cli_mapcheck_baseline_valid = true;

        snprintf(outline, CLI_LINESZ,
                 "mapcheck baseline stored cells:%u mask:0x%04X tick:%lu safety_state_unchanged:1",
                 (unsigned)smb->monitored_cell_count,
                 valid_mask,
                 (unsigned long)cli_mapcheck_baseline_tick);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli,
            "Now change exactly one low-voltage cell-simulator channel, then run spi mapcheck verify <cell>");
        return ret;
    }

    if(!strcmp(argv[2], "verify"))
    {
        int expected_cell;
        int min_delta_mv = 20;
        uint8_t largest_cell = 0u;
        int32_t largest_delta_uv = 0;
        uint8_t unexpected_count = 0u;
        uint16_t valid_mask;

        if(!cli_mapcheck_baseline_valid)
        {
            return cli_printline(cli, "mapcheck verify refused: capture a baseline first");
        }
        if((argc < 4) ||
           !cli_parse_int_range(argv[3], 1, (int)smb->monitored_cell_count,
                                &expected_cell) ||
           ((argc >= 5) &&
            !cli_parse_int_range(argv[4], 1, 500, &min_delta_mv)))
        {
            return cli_printline(cli,
                "Usage: spi mapcheck verify <cell 1-15> [min_delta_mV 1-500]");
        }

        status = adbms6830_capture_c_adc(smb);
        valid_mask =
            (uint16_t)(smb->last_cell_updated_mask[0] &
                       (uint16_t)~smb->last_cell_pec_mask[0] &
                       monitored_mask);
        if((status != HAL_OK) ||
           (valid_mask != monitored_mask) ||
           (cli_mapcheck_baseline_mask != monitored_mask))
        {
            snprintf(outline, CLI_LINESZ,
                     "mapcheck verify FAILED status:%s valid:0x%04X baseline:0x%04X expected:0x%04X",
                     cli_hal_status_str(status), valid_mask,
                     cli_mapcheck_baseline_mask, monitored_mask);
            return cli_printline(cli, outline);
        }

        snprintf(outline, CLI_LINESZ,
                 "mapcheck expected:C%02d threshold:%dmV baseline_age:%lu ticks",
                 expected_cell,
                 min_delta_mv,
                 (unsigned long)(osKernelGetTickCount() - cli_mapcheck_baseline_tick));
        ret |= cli_printline(cli, outline);
        for(uint8_t cell = 0u; cell < smb->monitored_cell_count; cell++)
        {
            int32_t current_uv =
                ((int32_t)smb->ics[0].cell.c_codes[cell] + 10000) * 150;
            int32_t delta_uv = current_uv - cli_mapcheck_baseline_uv[cell];
            int32_t magnitude_uv = (delta_uv < 0) ? -delta_uv : delta_uv;
            char delta_mv[20];

            if(magnitude_uv > largest_delta_uv)
            {
                largest_delta_uv = magnitude_uv;
                largest_cell = (uint8_t)(cell + 1u);
            }
            if(((int)(cell + 1u) != expected_cell) &&
               (magnitude_uv >= ((int32_t)min_delta_mv * 1000)))
            {
                unexpected_count++;
            }
            cli_microvolts_mv_string(delta_uv, delta_mv, sizeof(delta_mv));
            snprintf(outline, CLI_LINESZ,
                     "  C%02u delta:%smV%s",
                     (unsigned)(cell + 1u),
                     delta_mv,
                     ((int)(cell + 1u) == expected_cell) ? " expected" : "");
            ret |= cli_printline(cli, outline);
        }

        bool pass = ((int)largest_cell == expected_cell) &&
                    (largest_delta_uv >= ((int32_t)min_delta_mv * 1000)) &&
                    (unexpected_count == 0u);
        snprintf(outline, CLI_LINESZ,
                 "mapcheck result:%s largest:C%02u %lduV unexpected_channels:%u",
                 pass ? "PASS" : "REVIEW",
                 (unsigned)largest_cell,
                 (long)largest_delta_uv,
                 (unsigned)unexpected_count);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli,
            "Mapping result is diagnostic only; it never changes readiness, cell authority or balancing");
        return ret;
    }

    return cli_printline(cli,
        "Usage: spi mapcheck [baseline|verify <cell 1-15> [min_delta_mV 1-500]|clear]");
}

static int cli_run_cadc_stream(int argc, char *argv[])
{
#if AMS_ENABLE_SERVICE_CLI
    adbms6830_driver_t *smb;
    int samples = 60;
    int interval_ms = 1000;
    int ret = 0;

    if((argc >= 3) && !cli_parse_int_range(argv[2], 1, 3600, &samples))
    {
        return cli_printline(cli, "Usage: spi cstream [samples 1-3600] [interval_ms 10-60000]");
    }
    if((argc >= 4) && !cli_parse_int_range(argv[3], 10, 60000, &interval_ms))
    {
        return cli_printline(cli, "Usage: spi cstream [samples 1-3600] [interval_ms 10-60000]");
    }
    if(data == NULL)
    {
        return cli_printline(cli, "C stream unavailable");
    }

    smb = &data->acc.smb;
    if(smb_ic_count(smb) != 1u)
    {
        return cli_printline(cli, "C stream currently requires one logical SMB");
    }

    snprintf(outline, CLI_LINESZ,
             "CSTREAM_BEGIN,samples,%d,interval_ms,%d,cells,%u,mode,%s",
             samples, interval_ms,
             (unsigned)smb->monitored_cell_count,
             cli_voltage_mode_str());
    ret |= cli_printline(cli, outline);

    for(int sample = 0; sample < samples; sample++)
    {
        HAL_StatusTypeDef status;
        uint32_t tick;
        uint32_t conversion_us;
        uint16_t valid_mask;
        uint16_t pec_mask;
        uint16_t counter_mask = 0u;
        int32_t cell_uv[CELL] = {0};
        uint8_t cell_count;

        adbms_spi_lock();
        status = adbms6830_capture_c_adc(smb);
        tick = osKernelGetTickCount();
        conversion_us = smb->cadc_debug.conversion_time_us;
        cell_count = smb->monitored_cell_count;
        valid_mask = smb->last_cell_updated_mask[0];
        pec_mask = smb->last_cell_pec_mask[0];
        for(uint8_t group = 0u; group < smb->cadc_debug.group_count; group++)
        {
            counter_mask |= smb->cadc_debug.counter_mismatch_mask[group];
        }
        for(uint8_t cell = 0u; cell < cell_count; cell++)
        {
            cell_uv[cell] =
                ((int32_t)smb->ics[0].cell.c_codes[cell] + 10000) * 150;
        }
        adbms_spi_unlock();

        snprintf(outline, CLI_LINESZ,
                 "CSTREAM_META,%d,%lu,%s,%lu,0x%04X,0x%04X,0x%04X",
                 sample,
                 (unsigned long)tick,
                 cli_hal_status_str(status),
                 (unsigned long)conversion_us,
                 valid_mask,
                 pec_mask,
                 counter_mask);
        ret |= cli_printline(cli, outline);

        for(uint8_t cell = 0u; cell < cell_count; cell++)
        {
            snprintf(outline, CLI_LINESZ,
                     "CSTREAM_C,%d,C%u,%ld",
                     sample,
                     (unsigned)(cell + 1u),
                     (long)cell_uv[cell]);
            ret |= cli_printline(cli, outline);
        }

        if(sample + 1 < samples)
        {
            osDelay((uint32_t)interval_ms);
        }
    }

    ret |= cli_printline(cli, "CSTREAM_END,safety_state_unchanged,1");
    return ret;
#else
    (void)argc;
    (void)argv;
    return cli_service_action_refused("C-ADC stream");
#endif
}

int get_spi_debug(int argc, char *argv[])
{
    int ret;

    /* Streaming intentionally releases the ADBMS owner lock between samples
     * so the periodic safety task is not starved for the duration of a long
     * CSV capture. Every individual conversion remains serialized. */
    if((argc >= 2) && (argv[1] != NULL) && !strcmp(argv[1], "cstream"))
    {
        return cli_run_cadc_stream(argc, argv);
    }
    if((argc >= 2) && (argv[1] != NULL) && !strcmp(argv[1], "csoak"))
    {
#if AMS_ENABLE_SERVICE_CLI
        uint16_t repeat_count = 1000u;
        if((argc >= 3) &&
           !cli_parse_u16_range(argv[2], 1u, 1000u, &repeat_count))
        {
            return cli_printline(cli, "Usage: spi csoak [1-1000]");
        }
        return cli_run_cadc_soak(&data->acc.smb, repeat_count);
#else
        return cli_service_action_refused("C-ADC soak");
#endif
    }

    adbms_spi_lock();
    ret = get_spi_debug_locked(argc, argv);
    adbms_spi_unlock();
    return ret;
}

static int get_spi_debug_locked(int argc, char *argv[])
{
    int ret = 0;

#if !AMS_ENABLE_SERVICE_CLI
    /* Production keeps read-only SPI health/pin reporting, but blocks every
     * command that toggles chip selects, transmits diagnostic traffic, clears
     * evidence, or changes debug instrumentation while the safety task runs. */
    if((argc >= 2) && (argv[1] != NULL) &&
       strcmp(argv[1], "status") && strcmp(argv[1], "pins") &&
       strcmp(argv[1], "faults") && strcmp(argv[1], "lifecycle") &&
       strcmp(argv[1], "ages") && strcmp(argv[1], "authority") &&
       strcmp(argv[1], "events") && strcmp(argv[1], "lockdiag") &&
       strcmp(argv[1], "tempemu"))
    {
        return cli_service_action_refused("ADBMS SPI service action");
    }
#endif

    adbms6830_driver_t *smb = &data->acc.smb;
    const adbms6830_spi_debug_t *dbg;
    const adbms6830_diag_health_t *health;
    HAL_StatusTypeDef probe_status;
    SPI_HandleTypeDef *hspi = smb->hspi;
    uint8_t ic_count = smb_ic_count(smb);

	    if((argc >= 2) && (argv[1] != NULL))
	    {
        if(!strcmp(argv[1], "clear"))
        {
            adbms6830_spi_debug_clear(smb);
            ret |= cli_printline(cli, "ADBMS SPI debug counters cleared");
        }
        else if(!strcmp(argv[1], "enable"))
        {
            adbms6830_spi_debug_enable(smb, true);
            ret |= cli_printline(cli, "ADBMS SPI debug enabled");
        }
        else if(!strcmp(argv[1], "disable"))
        {
            adbms6830_spi_debug_enable(smb, false);
            ret |= cli_printline(cli, "ADBMS SPI debug disabled");
        }
        else if(!strcmp(argv[1], "pins"))
        {
            ret |= cli_print_adbms_pin_report(smb);
        }
        else if(!strcmp(argv[1], "cspins"))
        {
            if(cli_adbms_refuse_active_scan("spi cspins"))
            {
                return ret;
            }
            ret |= cli_handle_spi_candidate_pins(argc, argv);
        }
        else if(!strcmp(argv[1], "cs"))
        {
            if(cli_adbms_refuse_active_scan("spi cs"))
            {
                return ret;
            }
            ret |= cli_handle_spi_cs(argc, argv, smb);
        }
        else if(!strcmp(argv[1], "preset") || !strcmp(argv[1], "toggle"))
        {
            if(!strcmp(argv[1], "toggle"))
            {
                cli_adbms_scope_apply_preset((uint8_t)(cli_adbms_scope_preset_index + 1u));
                ret |= cli_print_adbms_scope_preset();
            }
            else if((argc < 3) || !strcmp(argv[2], "status"))
            {
                ret |= cli_print_adbms_scope_preset();
            }
            else if(!strcmp(argv[2], "normal"))
            {
                cli_adbms_scope_apply_preset(0u);
                ret |= cli_print_adbms_scope_preset();
            }
            else if(!strcmp(argv[2], "a"))
            {
                cli_adbms_scope_preset_index = 0u;
                cli_adbms_scope_default_string = STRING_A;
                cli_adbms_scope_default_mode = ADBMS6830_SCOPE_READ;
                cli_adbms_scope_default_repeat = 20u;
                ret |= cli_print_adbms_scope_preset();
            }
            else if(!strcmp(argv[2], "cmd"))
            {
                cli_adbms_scope_apply_preset(1u);
                ret |= cli_print_adbms_scope_preset();
            }
            else if(!strcmp(argv[2], "pattern"))
            {
                cli_adbms_scope_apply_preset(2u);
                ret |= cli_print_adbms_scope_preset();
            }
			else if(!strcmp(argv[2], "b"))
            {
                cli_adbms_scope_apply_preset(3u);
                ret |= cli_print_adbms_scope_preset();
            }
            else if(!strcmp(argv[2], "toggle"))
            {
                cli_adbms_scope_apply_preset((uint8_t)(cli_adbms_scope_preset_index + 1u));
                ret |= cli_print_adbms_scope_preset();
            }
            else if(!strcmp(argv[2], "repeat") || !strcmp(argv[2], "count"))
            {
                if((argc < 4) || !cli_parse_scope_repeat(argv[3], &cli_adbms_scope_default_repeat))
                {
                    ret |= cli_printline(cli, "ERROR: repeat must be 1-100");
                    return ret;
                }
                ret |= cli_print_adbms_scope_preset();
            }
            else
            {
                ret |= cli_printline(cli, "Usage: spi preset [status|normal|cmd|pattern|a|b|toggle|repeat <1-100>]");
                return ret;
            }
        }
	        else if(!strcmp(argv[1], "probe"))
	        {
	            if(cli_adbms_refuse_active_scan("spi probe"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_spi_probe_rdcfga(smb);
	            adbms6830_resync_command_counter_tracking(smb);
	            snprintf(outline, CLI_LINESZ, "RDCFGA probe status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
	        else if(!strcmp(argv[1], "probea"))
	        {
	            if(cli_adbms_refuse_active_scan("spi probea"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_spi_probe_rdcfga_on_string(smb, STRING_A);
	            adbms6830_resync_command_counter_tracking(smb);
	            snprintf(outline, CLI_LINESZ, "RDCFGA CS_A/stringA probe status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
	        else if(!strcmp(argv[1], "probeb"))
	        {
	            if(cli_adbms_refuse_active_scan("spi probeb"))
	            {
	                return ret;
	            }
	            /* The final-ring leading device from String B is the ADBMS2950,
	             * not an SMB.  Use its one-packet identity path so this service
	             * probe cannot parse APM-plus-SMB packets into the five-SMB
	             * image.  Any standalone String-B command makes the next SMB
	             * packet the only safe command-counter synchronization point. */
	            probe_status = adbms2950_spi_probe_sid(&data->acc.apm);
	            adbms6830_resync_command_counter_tracking(smb);
	            snprintf(outline, CLI_LINESZ,
	                     "ADBMS2950 RDSID CS_B/stringB probe status: %s",
	                     cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
	        else if(!strcmp(argv[1], "scope"))
	        {
	            adbms_string string = cli_adbms_scope_default_string;
	            adbms6830_scope_mode_t scope_mode = cli_adbms_scope_default_mode;
	            uint16_t repeat = cli_adbms_scope_default_repeat;

	            if(cli_adbms_refuse_active_scan("spi scope"))
	            {
	                return ret;
	            }

	            if((argc >= 3) && !cli_parse_adbms_string(argv[2], &string))
	            {
	                ret |= cli_printline(cli, "Usage: spi scope [a|b] [wake|cmd|read|pattern] [count 1-100]");
	                return ret;
	            }

	            if((argc >= 4) && !cli_parse_scope_mode(argv[3], &scope_mode))
	            {
	                ret |= cli_printline(cli, "Usage: spi scope [a|b] [wake|cmd|read|pattern] [count 1-100]");
	                return ret;
	            }

	            if(argc >= 5)
	            {
	                if(!cli_parse_scope_repeat(argv[4], &repeat))
	                {
	                    ret |= cli_printline(cli, "ERROR: count must be 1-100");
	                    return ret;
	                }
	            }

	            probe_status = adbms6830_scope_activity(smb, string, scope_mode, repeat);
	            /* Scope traffic intentionally runs outside the coordinated scan
	             * and may contain command-only or invalid pattern frames.  Never
	             * predict which remote devices consumed it. */
	            adbms6830_resync_command_counter_tracking(smb);
	            snprintf(outline, CLI_LINESZ,
	                     "scope string:%s mode:%s repeat:%u status:%s",
	                     (string == STRING_A) ? "CS_A" : "CS_B",
	                     cli_scope_mode_str(scope_mode),
	                     (unsigned)repeat,
	                     cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	            ret |= cli_printline(cli, "Probe MCU: SCK PG13, MOSI PG14, MISO PG12, CS_A PE2, CS_B PE4");
	            ret |= cli_printline(cli, "Then probe ADBMS6822 IP/IM and SMB transformer pins for matching activity");
	        }
	        else if(!strcmp(argv[1], "sid"))
	        {
	            if(cli_adbms_refuse_active_scan("spi sid"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_read_sid(smb);
	            snprintf(outline, CLI_LINESZ, "RDSID status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
	        else if(!strcmp(argv[1], "stat"))
	        {
	            const adbms6830_diag_health_t *health;
	            bool transport_ok;
	            bool non_cs_ok;
	            bool s_ok;

	            if(cli_adbms_refuse_active_scan("spi stat"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_refresh_diagnostics(smb);
	            health = adbms6830_diag_health_get(smb);
	            transport_ok = adbms6830_diagnostic_transport_ok(smb);
	            non_cs_ok = adbms6830_non_cs_diagnostics_ok(smb);
	            s_ok = (health != NULL) && (health->cs_fault_ic_mask == 0u);
	            snprintf(outline, CLI_LINESZ,
	                     "ADAX + RDSTATA-E api:%s transport:%s non_cs:%s S_redundancy:%s",
	                     cli_hal_status_str(probe_status),
	                     transport_ok ? "OK" : "ERROR",
	                     non_cs_ok ? "OK" : "ERROR",
	                     s_ok ? "OK" : "ERROR");
	            ret |= cli_printline(cli, outline);
	        }
	        else if(!strcmp(argv[1], "staterr"))
	        {
	            if(cli_adbms_refuse_active_scan("spi staterr"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_read_status(smb, true);
	            snprintf(outline, CLI_LINESZ, "RDSTATCERR/D/E status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
	        else if(!strcmp(argv[1], "wake"))
	        {
	            if(cli_adbms_refuse_active_scan("spi wake"))
	            {
	                return ret;
	            }
	            if(smb->spi_debug.enabled)
	            {
	                smb->spi_debug.last_op = ADBMS6830_SPI_OP_WAKE;
	            }
	            adbms6830_wakeup(smb);
	            ret |= cli_printline(cli, "ADBMS wake pulses sent");
	        }
	        else if(!strcmp(argv[1], "coldwake"))
	        {
	            if(cli_adbms_refuse_active_scan("spi coldwake"))
	            {
	                return ret;
	            }
	            adbms6830_wakeup_cold(smb);
	            ret |= cli_printline(cli, "ADBMS cold wake pulse train sent");
	        }
	        else if(!strcmp(argv[1], "clrflag"))
	        {
	            if(cli_adbms_refuse_active_scan("spi clrflag"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_clear_all_flags(smb);
	            snprintf(outline, CLI_LINESZ, "CLRFLAG all status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
        else if(!strcmp(argv[1], "diagclear"))
        {
            if(cli_adbms_refuse_active_scan("spi diagclear"))
            {
                return ret;
            }
            adbms6830_diag_health_clear(smb);
            ret |= cli_printline(cli, "ADBMS diagnostic counters cleared; safety latches require reset");
        }
	        else if(!strcmp(argv[1], "cfgchk"))
	        {
	            if(cli_adbms_refuse_active_scan("spi cfgchk"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_verify_config_readback(smb);
	            snprintf(outline, CLI_LINESZ, "CFGA/CFGB readback check status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
	        else if(!strcmp(argv[1], "cellst"))
	        {
	            if(cli_adbms_refuse_active_scan("spi cellst"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_run_cell_adc_self_test(smb);
	            snprintf(outline, CLI_LINESZ, "Cell ADC conversion diagnostic status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
        else if(!strcmp(argv[1], "csdump"))
        {
            if(cli_adbms_refuse_active_scan("spi csdump"))
            {
                return ret;
            }

            probe_status = adbms6830_capture_cs_comparison(smb);
            snprintf(outline, CLI_LINESZ,
                     "Fresh startup-style C/S capture status: %s",
                     cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);

            for(uint8_t ic = 0u; ic < ic_count; ic++)
            {
                uint16_t monitored_mask =
                    (smb->monitored_cell_count == 16u) ? UINT16_MAX :
                    (uint16_t)((1UL << smb->monitored_cell_count) - 1UL);
                uint16_t c_valid = smb->last_cell_updated_mask[ic] & monitored_mask;
                uint16_t s_valid = smb->last_scell_updated_mask[ic] & monitored_mask;
                uint16_t c_pec = smb->last_cell_pec_mask[ic] & monitored_mask;
                uint16_t s_pec = smb->last_scell_pec_mask[ic] & monitored_mask;
                uint16_t csflt = smb->diag[ic].cs_flt_mask & monitored_mask;

                snprintf(outline, CLI_LINESZ,
                         "IC%u Cvalid:0x%04X Svalid:0x%04X Cpec:0x%04X Spec:0x%04X CSFLT:0x%04X",
                         (unsigned)ic,
                         (unsigned)c_valid,
                         (unsigned)s_valid,
                         (unsigned)c_pec,
                         (unsigned)s_pec,
                         (unsigned)csflt);
                ret |= cli_printline(cli, outline);
                ret |= cli_printline(cli, "Cell     C-ADC mV     S-ADC mV     |delta| mV   CSFLT");

                for(uint8_t cell = 0u; cell < smb->monitored_cell_count; cell++)
                {
                    uint16_t bit = (uint16_t)(1u << cell);
                    bool c_ok = (c_valid & bit) != 0u;
                    bool s_ok = (s_valid & bit) != 0u;

                    if(c_ok && s_ok)
                    {
                        int32_t c_uv =
                            ((int32_t)smb->ics[ic].cell.c_codes[cell] + 10000) * 150;
                        int32_t s_uv =
                            ((int32_t)smb->ics[ic].scell.sc_codes[cell] + 10000) * 150;
                        int32_t delta_uv = c_uv - s_uv;

                        if(delta_uv < 0)
                        {
                            delta_uv = -delta_uv;
                        }

                        char c_mv[20];
                        char s_mv[20];
                        char delta_mv[20];
                        cli_microvolts_mv_string(c_uv, c_mv, sizeof(c_mv));
                        cli_microvolts_mv_string(s_uv, s_mv, sizeof(s_mv));
                        cli_microvolts_mv_string(delta_uv, delta_mv, sizeof(delta_mv));
                        snprintf(outline, CLI_LINESZ,
                                 "C%02u   %11s     %11s     %11s       %u",
                                 (unsigned)(cell + 1u), c_mv, s_mv, delta_mv,
                                 (unsigned)((csflt & bit) != 0u));
                    }
                    else
                    {
                        snprintf(outline, CLI_LINESZ,
                                 "C%02u   C:%s S:%s                         %u",
                                 (unsigned)(cell + 1u),
                                 c_ok ? "valid" : "invalid",
                                 s_ok ? "valid" : "invalid",
                                 (unsigned)((csflt & bit) != 0u));
                    }
                    ret |= cli_printline(cli, outline);
                }
            }
        }
        else if(!strcmp(argv[1], "sdump"))
        {
            if(cli_adbms_refuse_active_scan("spi sdump"))
            {
                return ret;
            }

            probe_status = adbms6830_capture_s_adc(smb);
            ret |= cli_print_sadc_dump(smb, probe_status);
        }
        else if(!strcmp(argv[1], "srepeat"))
        {
            uint16_t repeat_count = 20u;

            if(cli_adbms_refuse_active_scan("spi srepeat"))
            {
                return ret;
            }
            if((argc >= 3) && !cli_parse_scope_repeat(argv[2], &repeat_count))
            {
                ret |= cli_printline(cli, "Usage: spi srepeat [1-100]");
                return ret;
            }

            ret |= cli_run_sadc_repeat(smb, repeat_count);
        }
        else if(!strcmp(argv[1], "cdump"))
        {
            if(cli_adbms_refuse_active_scan("spi cdump"))
            {
                return ret;
            }
            probe_status = adbms6830_capture_c_adc(smb);
            ret |= cli_print_cadc_dump(smb, probe_status);
        }
        else if(!strcmp(argv[1], "csoak"))
        {
            uint16_t repeat_count = 1000u;

            if(cli_adbms_refuse_active_scan("spi csoak"))
            {
                return ret;
            }
            if((argc >= 3) &&
               !cli_parse_u16_range(argv[2], 1u, 1000u, &repeat_count))
            {
                ret |= cli_printline(cli, "Usage: spi csoak [1-1000]");
                return ret;
            }
            ret |= cli_run_cadc_soak(smb, repeat_count);
        }
        else if(!strcmp(argv[1], "sessiongap"))
        {
#if AMS_ENABLE_SERVICE_CLI
            int gap_us = 6000;
            bool raw = false;
            if((argc >= 3) &&
               !cli_parse_int_range(argv[2], 1, 100000, &gap_us))
            {
                ret |= cli_printline(cli, "Usage: spi sessiongap [1-100000 us] [raw]");
                return ret;
            }
            if((argc >= 4) && (argv[3] != NULL))
            {
                if(strcmp(argv[3], "raw"))
                {
                    ret |= cli_printline(cli, "Usage: spi sessiongap [1-100000 us] [raw]");
                    return ret;
                }
                raw = true;
            }
            probe_status = adbms6830_session_inject_gap_once(
                smb, (uint32_t)gap_us, raw);
            snprintf(outline, CLI_LINESZ,
                     "ADBMS next-session gap injection:%dus mode:%s status:%s",
                     gap_us, raw ? "raw-bypass-guard" : "guarded-restart",
                     cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);
#else
            ret |= cli_service_action_refused("ADBMS session gap injection");
#endif
        }
        else if(!strcmp(argv[1], "timing"))
        {
            if(cli_adbms_refuse_active_scan("spi timing"))
            {
                return ret;
            }
            ret |= cli_run_conversion_timing(smb, argc, argv);
        }
        else if(!strcmp(argv[1], "cfgrepeat"))
        {
            uint16_t repeat_count = 100u;

            if(cli_adbms_refuse_active_scan("spi cfgrepeat"))
            {
                return ret;
            }
            if((argc >= 3) &&
               !cli_parse_u16_range(argv[2], 1u, 1000u, &repeat_count))
            {
                ret |= cli_printline(cli, "Usage: spi cfgrepeat [1-1000]");
                return ret;
            }
            ret |= cli_run_config_repeat(smb, repeat_count);
        }
        else if(!strcmp(argv[1], "snapshot"))
        {
            if(cli_adbms_refuse_active_scan("spi snapshot"))
            {
                return ret;
            }
            ret |= cli_run_snapshot(smb);
        }
        else if(!strcmp(argv[1], "rawdump"))
        {
            if(cli_adbms_refuse_active_scan("spi rawdump"))
            {
                return ret;
            }
            ret |= cli_run_raw_dump(smb);
        }
        else if(!strcmp(argv[1], "recovery"))
        {
            uint16_t idle_ms = 2000u;

            if(cli_adbms_refuse_active_scan("spi recovery"))
            {
                return ret;
            }
            if((argc >= 3) &&
               !cli_parse_u16_range(argv[2], 0u, 10000u, &idle_ms))
            {
                ret |= cli_printline(cli, "Usage: spi recovery [idle_ms 0-10000]");
                return ret;
            }
            ret |= cli_run_recovery(smb, idle_ms);
        }
        else if(!strcmp(argv[1], "faults"))
        {
            ret |= cli_print_adbms_fault_classes(data);
        }
        else if(!strcmp(argv[1], "lifecycle"))
        {
            ret |= cli_print_adbms_lifecycle(data);
        }
        else if(!strcmp(argv[1], "ages"))
        {
            ret |= cli_print_adbms_ages(data);
        }
        else if(!strcmp(argv[1], "authority"))
        {
            ret |= cli_print_adbms_authority(data);
        }
        else if(!strcmp(argv[1], "events"))
        {
            ret |= cli_print_adbms_events();
        }
        else if(!strcmp(argv[1], "lockdiag"))
        {
            ret |= cli_print_adbms_lockdiag(data);
        }
        else if(!strcmp(argv[1], "tempemu"))
        {
            ret |= cli_run_temperature_emulator(argc, argv);
        }
        else if(!strcmp(argv[1], "inject"))
        {
            ret |= cli_handle_adbms_injection(argc, argv);
        }
        else if(!strcmp(argv[1], "mapcheck"))
        {
            if(cli_adbms_refuse_active_scan("spi mapcheck"))
            {
                return ret;
            }
            ret |= cli_handle_mapcheck(smb, argc, argv);
        }
	        else if(!strcmp(argv[1], "owcheck") ||
                 !strcmp(argv[1], "owc"))
        {
            adbms6830_open_wire_result_t ow = {0};
            int ow_rc;

            if(cli_adbms_refuse_active_scan("spi owc"))
            {
                return ret;
            }
            if(!cli_adbms_open_wire_state_allowed())
            {
                ret |= cli_printline(cli,
                    "C open-wire refused: require BMS off, balancing off, transport ready and a valid normal C image");
                return ret;
            }

            ow_rc = accumulator_run_c_open_wire_diagnostic(&data->acc, &ow);
            probe_status = (ow_rc == 0) ? HAL_OK : HAL_ERROR;
            taskENTER_CRITICAL();
            data->adbms_open_wire_last_path = ow.path;
            data->adbms_open_wire_restore_fault = !ow.restored_normal_c_image;
            data->adbms_last_diag_status = probe_status;
            if(data->adbms_open_wire_diag_count != UINT32_MAX)
            {
                data->adbms_open_wire_diag_count++;
            }
            for(uint8_t ic = 0u; ic < (uint8_t)smb->num_ics; ic++)
            {
                data->adbms_sense_path_open_mask[ic] = ow.cell_fault_mask[ic];
                data->adbms_sense_path_open_sticky_mask[ic] |=
                    ow.cell_fault_mask[ic];
            }
            if(probe_status != HAL_OK)
            {
                data->adbms_open_wire_fault = true;
                data->adbms_diag_fault = true;
            }
            taskEXIT_CRITICAL();
            if(probe_status != HAL_OK)
            {
                set_bms(false);
            }

            snprintf(outline, CLI_LINESZ,
                     "C-path sense-open diag:%s restore:%s complete:%d incompleteIC:0x%04X faultIC:0x%04X",
                     cli_hal_status_str(ow.diagnostic_status),
                     cli_hal_status_str(ow.restore_status),
                     ow.complete,
                     ow.incomplete_ic_mask,
                     ow.fault_ic_mask);
            ret |= cli_printline(cli, outline);
            for(uint8_t ic = 0u; ic < (uint8_t)smb->num_ics; ic++)
            {
                snprintf(outline, CLI_LINESZ,
                         "IC%u sense-path-open cells:0x%04X sticky:0x%04X",
                         (unsigned)ic,
                         ow.cell_fault_mask[ic],
                         data->adbms_sense_path_open_sticky_mask[ic]);
                ret |= cli_printline(cli, outline);
            }
            ret |= cli_printline(cli,
                "Meaning: electrical cell-tap path open; inspect 1 A fuse, wire, connector, tab, trace, filter resistor and solder joint");
            ret |= cli_printline(cli,
                "The diagnostic C image was discarded; restore must PASS before normal C authority can return");
        }
        else if(!strcmp(argv[1], "ows"))
        {
            if(cli_adbms_refuse_active_scan("spi ows"))
            {
                return ret;
            }
            if(!cli_adbms_open_wire_state_allowed())
            {
                ret |= cli_printline(cli,
                    "S open-wire refused: require BMS off, balancing off, transport ready and a valid normal C image");
                return ret;
            }
            probe_status = adbms6830_run_open_wire_diagnostic_path(
                smb,
                ADBMS6830_OPEN_WIRE_PATH_S);
            snprintf(outline, CLI_LINESZ,
                     "S-path open-wire result:%s (not meaningful until S2N-S15N hardware is repaired)",
                     cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "oweven"))
        {
            if(cli_adbms_refuse_active_scan("spi oweven"))
            {
                return ret;
            }
            if(!cli_adbms_open_wire_state_allowed())
            {
                ret |= cli_printline(cli, "Open-wire refused by service interlock");
                return ret;
            }
            probe_status = adbms6830_run_open_wire_check_path(
                smb,
                ADBMS6830_OPEN_WIRE_PATH_S,
                false);
            snprintf(outline, CLI_LINESZ,
                     "S-path even-channel result:%s",
                     cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "owodd"))
        {
            if(cli_adbms_refuse_active_scan("spi owodd"))
            {
                return ret;
            }
            if(!cli_adbms_open_wire_state_allowed())
            {
                ret |= cli_printline(cli, "Open-wire refused by service interlock");
                return ret;
            }
            probe_status = adbms6830_run_open_wire_check_path(
                smb,
                ADBMS6830_OPEN_WIRE_PATH_S,
                true);
            snprintf(outline, CLI_LINESZ,
                     "S-path odd-channel result:%s",
                     cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "auxdiag"))
        {
            if(cli_adbms_refuse_active_scan("spi auxdiag"))
            {
                return ret;
            }
            probe_status = adbms6830_run_aux_gpio_diagnostic(smb);
            snprintf(outline, CLI_LINESZ,
                     "AUX/GPIO diagnostic hook status: %s",
                     cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);
        }
        else if(strcmp(argv[1], "status"))
        {
            ret |= cli_printline(cli,
                    "Usage: spi [status|snapshot|faults|lifecycle|ages|authority|events|lockdiag|inject|mapcheck|tempemu|cdump|csoak|cstream|csdump|sdump|srepeat|sessiongap|timing|cfgrepeat|recovery|rawdump|pins|cspins|cs|preset|toggle|probe|probea|probeb|scope|sid|stat|staterr|cfgchk|cellst|owc|ows|owcheck|oweven|owodd|auxdiag|wake|coldwake|clrflag|clear|diagclear|enable|disable]");
            return ret;
        }
    }

    dbg = adbms6830_spi_debug_get(smb);
    if(dbg == NULL)
    {
        ret |= cli_printline(cli, "ADBMS SPI debug unavailable");
        return ret;
    }

    if(hspi != NULL)
    {
        snprintf(outline, CLI_LINESZ,
                 "SPI6 mode CPOL:%s CPHA:%s prescaler:%lu firstbit:%s",
                 cli_spi_polarity_str(hspi->Init.CLKPolarity),
                 cli_spi_phase_str(hspi->Init.CLKPhase),
                 (unsigned long)hspi->Init.BaudRatePrescaler,
                 (hspi->Init.FirstBit == SPI_FIRSTBIT_MSB) ? "MSB" : "LSB");
        ret |= cli_printline(cli, outline);
    }
    else
    {
        ret |= cli_printline(cli, "SPI handle is NULL");
    }

    snprintf(outline, CLI_LINESZ,
             "dbg en:%d op:%s string:%u status:%s tx:%lu rx:%lu err:%lu",
             dbg->enabled,
             adbms6830_spi_op_str(dbg->last_op),
             (unsigned)dbg->last_string,
             cli_hal_status_str(dbg->last_status),
             (unsigned long)dbg->tx_count,
             (unsigned long)dbg->rx_count,
             (unsigned long)dbg->error_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "voltage mode:%s degraded:%d S_policy:%s active_faults:0x%04X latched_faults:0x%04X",
             cli_voltage_mode_str(),
             (AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP),
             (AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP) ?
                 "diagnostic_only" : "required",
             data->adbms_fault_active_mask,
             data->adbms_fault_latched_mask);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "balance timing verified on:%lums off-to-voltage:%lums apply_tick:%lu",
             (unsigned long)data->adbms_last_balance_on_ms,
             (unsigned long)data->adbms_last_balance_off_ms,
             (unsigned long)data->adbms_balance_apply_tick);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "scan active:%d count:%lu diag fault:%d cfg:%d stat:%d ow:%d balance_fault:%d balance_fail:%lu last:%s",
             data->adbms_scan_active,
             (unsigned long)data->adbms_scan_count,
             data->adbms_diag_fault,
             data->adbms_config_fault,
             data->adbms_status_fault,
             data->adbms_open_wire_fault,
             data->adbms_balance_write_fault,
             (unsigned long)data->adbms_balance_write_fail_count,
             cli_hal_status_str(data->adbms_last_diag_status));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "timing last:%lums max:%lums interval:%lums misses:%lu balance_active:%d recoveries:%lu",
             (unsigned long)data->adbms_last_scan_duration_ms,
             (unsigned long)data->adbms_max_scan_duration_ms,
             (unsigned long)data->adbms_last_schedule_interval_ms,
             (unsigned long)data->adbms_scan_deadline_miss_count,
             data->adbms_balance_active,
             (unsigned long)data->adbms_balance_recovery_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "timing nonyield-wall last/max:%lu/%luus yielded:%lu/%luus guard:%uus SPIdiv:/%u",
             (unsigned long)data->adbms_last_scan_cpu_us,
             (unsigned long)data->adbms_max_scan_cpu_us,
             (unsigned long)data->adbms_last_scan_yield_us,
             (unsigned long)data->adbms_max_scan_yield_us,
             (unsigned)AMS_ADBMS_SESSION_GUARD_US,
             (unsigned)AMS_ADBMS_SPI_PRESCALER_DIV);
    ret |= cli_printline(cli, outline);

    {
        const adbms6830_session_health_t *session = adbms6830_session_health_get(smb);
        if(session != NULL)
        {
            snprintf(outline, CLI_LINESZ,
                     "session count:%lu wakes_scan:%lu gap last/max:%lu/%luus",
                     (unsigned long)session->session_count,
                     (unsigned long)session->wake_count_last_scan,
                     (unsigned long)session->last_gap_us,
                     (unsigned long)session->max_gap_us);
            ret |= cli_printline(cli, outline);
            snprintf(outline, CLI_LINESZ,
                     "session expiry:%lu rewake:%lu restart:%lu restart_fail:%lu inject:%lu",
                     (unsigned long)session->guard_expiry_count,
                     (unsigned long)session->guard_rewake_count,
                     (unsigned long)session->coherent_restart_count,
                     (unsigned long)session->coherent_restart_fail_count,
                     (unsigned long)session->injected_gap_count);
            ret |= cli_printline(cli, outline);
            snprintf(outline, CLI_LINESZ,
                     "session waits:%lu interrupted:%lu requested:%lluus duration last/max:%lu/%luus",
                     (unsigned long)session->long_wait_count,
                     (unsigned long)session->long_wait_interrupted_count,
                     (unsigned long long)session->long_wait_requested_us,
                     (unsigned long)session->last_duration_us,
                     (unsigned long)session->max_duration_us);
            ret |= cli_printline(cli, outline);
        }
    }

    snprintf(outline, CLI_LINESZ,
             "task gaps ms CAN:%lu/%lu EST:%lu/%lu FAN:%lu/%lu IMD:%lu/%lu",
             (unsigned long)data->heartbeat.last_gap_ms[AMS_HEARTBEAT_CAN],
             (unsigned long)data->heartbeat.max_gap_ms[AMS_HEARTBEAT_CAN],
             (unsigned long)data->heartbeat.last_gap_ms[AMS_HEARTBEAT_ESTIMATOR],
             (unsigned long)data->heartbeat.max_gap_ms[AMS_HEARTBEAT_ESTIMATOR],
             (unsigned long)data->heartbeat.last_gap_ms[AMS_HEARTBEAT_FAN],
             (unsigned long)data->heartbeat.max_gap_ms[AMS_HEARTBEAT_FAN],
             (unsigned long)data->heartbeat.last_gap_ms[AMS_HEARTBEAT_IMD],
             (unsigned long)data->heartbeat.max_gap_ms[AMS_HEARTBEAT_IMD]);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "task gaps ms CUR:%lu/%lu TEMP:%lu/%lu ADBMS:%lu/%lu",
             (unsigned long)data->heartbeat.last_gap_ms[AMS_HEARTBEAT_CURRENT],
             (unsigned long)data->heartbeat.max_gap_ms[AMS_HEARTBEAT_CURRENT],
             (unsigned long)data->heartbeat.last_gap_ms[AMS_HEARTBEAT_TEMP],
             (unsigned long)data->heartbeat.max_gap_ms[AMS_HEARTBEAT_TEMP],
             (unsigned long)data->heartbeat.last_gap_ms[AMS_HEARTBEAT_ADBMS],
             (unsigned long)data->heartbeat.max_gap_ms[AMS_HEARTBEAT_ADBMS]);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "task kicks CAN:%lu EST:%lu FAN:%lu IMD:%lu ADBMS:%lu CUR:%lu TEMP:%lu",
             (unsigned long)data->heartbeat.count[AMS_HEARTBEAT_CAN],
             (unsigned long)data->heartbeat.count[AMS_HEARTBEAT_ESTIMATOR],
             (unsigned long)data->heartbeat.count[AMS_HEARTBEAT_FAN],
             (unsigned long)data->heartbeat.count[AMS_HEARTBEAT_IMD],
             (unsigned long)data->heartbeat.count[AMS_HEARTBEAT_ADBMS],
             (unsigned long)data->heartbeat.count[AMS_HEARTBEAT_CURRENT],
             (unsigned long)data->heartbeat.count[AMS_HEARTBEAT_TEMP]);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "balance mute:%d durable_zero:%d reason:%u urgent req/service/fail:%lu/%lu/%lu",
             data->adbms_mute_asserted,
             data->adbms_balance_durable_zero_verified,
             (unsigned)data->adbms_balance_inhibit_reason,
             (unsigned long)data->adbms_urgent_mute_request_count,
             (unsigned long)data->adbms_urgent_mute_service_count,
             (unsigned long)data->adbms_urgent_mute_fail_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "voltage products FC:%u ready:%d estimator_src:%d avg_mask0:0x%04X iir_mask0:0x%04X",
             (unsigned)AMS_ADBMS_IIR_FC,
             smb->filtered_voltage_ready,
             (int)AMS_ESTIMATOR_VOLTAGE_SOURCE,
             data->acc.avg8_usable_voltage_mask[0],
             data->acc.iir_usable_voltage_mask[0]);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "POST pass:%d stage:%u attempts:%u runs:%lu fail:%lu bad:0x%04X extra:0x%04X",
             smb->post.passed, (unsigned)smb->post.stage,
             (unsigned)smb->post.attempts,
             (unsigned long)smb->post.run_count,
             (unsigned long)smb->post.fail_count,
             smb->post.failed_ic_mask, smb->post.unexpected_ic_mask);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "AUX2 sensor:%u valid:0x%04X disagree:0x%04X count/fail:%lu/%lu app:%lu/%lu",
             (unsigned)smb->aux2_health.sensor, smb->aux2_health.valid_mask,
             smb->aux2_health.disagree_mask,
             (unsigned long)smb->aux2_health.count,
             (unsigned long)smb->aux2_health.fail_count,
             (unsigned long)data->adbms_aux2_diag_count,
             (unsigned long)data->adbms_aux2_diag_fail_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "thermOW sensor:%u valid:0x%04X suspect:0x%04X count/fail:%lu/%lu restore_fail:%lu",
             (unsigned)smb->therm_ow_health.sensor, smb->therm_ow_health.valid_mask,
             smb->therm_ow_health.suspect_mask,
             (unsigned long)smb->therm_ow_health.count,
             (unsigned long)smb->therm_ow_health.fail_count,
             (unsigned long)smb->therm_ow_health.config_restore_fail_count);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "thermOW app count/fail:%lu/%lu",
             (unsigned long)data->adbms_therm_ow_diag_count,
             (unsigned long)data->adbms_therm_ow_diag_fail_count);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "thermOW IC0 raw base/down/up/recover:%d/%d/%d/%d delta_mV down/up/recover:%d/%d/%d",
             (int)smb->therm_ow_health.baseline_raw[0],
             (int)smb->therm_ow_health.pulldown_raw[0],
             (int)smb->therm_ow_health.pullup_raw[0],
             (int)smb->therm_ow_health.recovery_raw[0],
             (int)smb->therm_ow_health.pulldown_delta_mv[0],
             (int)smb->therm_ow_health.pullup_delta_mv[0],
             (int)smb->therm_ow_health.recovery_delta_mv[0]);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "diag periodic status:%lu cfg:%lu openwire:%lu transport_ready:%d safety_ready:%d",
             (unsigned long)data->adbms_status_diag_count,
             (unsigned long)data->adbms_config_diag_count,
             (unsigned long)data->adbms_open_wire_diag_count,
             data->acc.smb_transport_ready,
             data->acc.smb_ready);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "diag init:%s timer_ready:%d timer_status:%s",
             cli_hal_status_str(data->acc.smb_init_status),
             data->acc.delay_timer_ready,
             cli_hal_status_str(data->acc.delay_timer_status));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "topology logical:%u physical:%u monitored_cells:%u string:%u write_string:%u final_ring:%d",
             (unsigned)smb->num_ics,
             (unsigned)smb->physical_chain_count,
             (unsigned)smb->monitored_cell_count,
             (unsigned)smb->string,
             (unsigned)smb->write_string,
             accumulator_final_ring_topology_valid(&data->acc));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "cmd:%02X %02X len tx:%u rx:%u total:%u",
             dbg->last_cmd[0],
             dbg->last_cmd[1],
             (unsigned)dbg->last_tx_len,
             (unsigned)dbg->last_rx_len,
             (unsigned)dbg->last_total_len);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "HAL tx:%s rx:%s xfer:%s PEC pass:0x%04X fail:0x%04X",
             cli_hal_status_str(dbg->last_tx_status),
             cli_hal_status_str(dbg->last_rx_status),
             cli_hal_status_str(dbg->last_xfer_status),
             dbg->last_read_pec_pass_mask,
             dbg->last_read_pec_fail_mask);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "cmdcnt IC0:%u IC1:%u IC2:%u IC3:%u IC4:%u",
             dbg->last_cmd_counter[0],
             dbg->last_cmd_counter[1],
             dbg->last_cmd_counter[2],
             dbg->last_cmd_counter[3],
             dbg->last_cmd_counter[4]);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "cmdcnt seen:0x%04X expect:0x%04X mismatch:0x%04X errors:%lu",
             dbg->cmd_counter_seen_mask,
             dbg->cmd_counter_expected_mask,
             dbg->cmd_counter_mismatch_mask,
             (unsigned long)dbg->cmd_counter_error_count);
    ret |= cli_printline(cli, outline);

    health = adbms6830_diag_health_get(smb);
    if(health != NULL)
    {
        snprintf(outline, CLI_LINESZ,
                 "diag op:%s status:%s cfgA:0x%04X cfgB:0x%04X cfg:0x%04X",
                 adbms6830_spi_op_str(health->last_op),
                 cli_hal_status_str(health->last_status),
                 health->configa_mismatch_mask,
                 health->configb_mismatch_mask,
                 health->config_mismatch_mask);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "diag CFGB write guard current:0x%04X sticky:0x%04X IC0_rejects:%lu",
                 health->config_write_guard_fault_mask,
                 health->sticky_config_write_guard_fault_mask,
                 (unsigned long)health->config_write_guard_reject_count[0]);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "diag PEC last pass:0x%04X fail:0x%04X sticky:0x%04X cmd sticky:0x%04X",
                 health->last_pec_pass_mask,
                 health->last_pec_fail_mask,
                 health->sticky_pec_fail_mask,
                 health->sticky_cmd_counter_mismatch_mask);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "diag SID valid:0x%04X identity_mismatch:0x%04X sticky:0x%04X expected_id:0x%02X",
                 health->sid_valid_ic_mask,
                 health->sid_identity_mismatch_ic_mask,
                 health->sticky_sid_identity_mismatch_ic_mask,
                 ADBMS6830B_DEVICE_ID);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "diag startup:%d refresh:%lu start:%lu status invalid:0x%04X fault:0x%04X",
                 health->startup_baseline_passed,
                 (unsigned long)health->diagnostic_refresh_count,
                 (unsigned long)health->startup_baseline_count,
                 health->status_invalid_ic_mask,
                 health->status_fault_ic_mask);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "diag reference invalid:0x%04X fault:0x%04X sticky status:0x%04X ref:0x%04X",
                 health->reference_invalid_ic_mask,
                 health->reference_fault_ic_mask,
                 health->sticky_status_fault_ic_mask,
                 health->sticky_reference_fault_ic_mask);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "diag classes cs:0x%04X supply:0x%04X memory:0x%04X digital:0x%04X osc:0x%04X cell:0x%04X",
                 health->cs_fault_ic_mask,
                 health->supply_flag_fault_ic_mask,
                 health->memory_fault_ic_mask,
                 health->digital_fault_ic_mask,
                 health->oscillator_counter_fault_ic_mask,
                 health->cell_ovuv_fault_ic_mask);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "diag counts cfg:%lu cell:%lu owfull:%lu owbase:%lu owe:%lu owo:%lu",
                 (unsigned long)health->config_readback_count,
                 (unsigned long)health->cell_adc_self_test_count,
                 (unsigned long)health->open_wire_full_count,
                 (unsigned long)health->open_wire_baseline_count,
                 (unsigned long)health->open_wire_even_count,
                 (unsigned long)health->open_wire_odd_count);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "diag counts balance:%lu aux:%lu mute ok/fail/verify:%lu/%lu/%lu",
                 (unsigned long)health->balance_readback_count,
                 (unsigned long)health->aux_gpio_diag_count,
                 (unsigned long)health->mute_count,
                 (unsigned long)health->mute_fail_count,
                 (unsigned long)health->mute_verify_fail_count);
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "diag unmute ok/fail/verify:%lu/%lu/%lu",
                 (unsigned long)health->unmute_count,
                 (unsigned long)health->unmute_fail_count,
                 (unsigned long)health->unmute_verify_fail_count);
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "diag products avg:%lu/%lu filt:%lu/%lu silicon:%lu Sdiag:%lu/%lu",
                 (unsigned long)health->avg8_read_count,
                 (unsigned long)health->avg8_read_fail_count,
                 (unsigned long)health->filtered_read_count,
                 (unsigned long)health->filtered_read_fail_count,
                 (unsigned long)health->silicon_health_sweep_count,
                 (unsigned long)health->s_periodic_diag_count,
                 (unsigned long)health->s_periodic_diag_fail_count);
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "diag coherent STATC:%lu/%lu STATD:%lu/%lu CCTS valid/fault/sticky:0x%04X/0x%04X/0x%04X",
                 (unsigned long)health->coherent_statc_read_count,
                 (unsigned long)health->coherent_statc_read_fail_count,
                 (unsigned long)health->coherent_statd_read_count,
                 (unsigned long)health->coherent_statd_read_fail_count,
                 health->cadc_ccts_valid_ic_mask,
                 health->cadc_ccts_fault_ic_mask,
                 health->sticky_cadc_ccts_fault_ic_mask);
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "diag CCTS IC0:%u IC1:%u IC2:%u IC3:%u IC4:%u",
                 (unsigned)health->cadc_ccts_last[0],
                 (unsigned)health->cadc_ccts_last[1],
                 (unsigned)health->cadc_ccts_last[2],
                 (unsigned)health->cadc_ccts_last[3],
                 (unsigned)health->cadc_ccts_last[4]);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "openwire cells:%u base_valid:0x%04X even_valid:0x%04X odd_valid:0x%04X",
                 (unsigned)smb->monitored_cell_count,
                 health->open_wire_baseline_valid_ic_mask,
                 health->open_wire_even_valid_ic_mask,
                 health->open_wire_odd_valid_ic_mask);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "openwire incomplete:0x%04X fault_ic:0x%04X sticky:0x%04X",
                 health->open_wire_incomplete_ic_mask,
                 health->open_wire_fault_ic_mask,
                 health->sticky_open_wire_fault_ic_mask);
        ret |= cli_printline(cli, outline);
    }

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        const adbms6830_ic_diag_t *diag = &smb->diag[ic];
        if(health != NULL)
        {
            snprintf(outline, CLI_LINESZ,
                     "IC%u health PEC pass:%lu fail:%lu cmd_mis:%lu cfg_mis:%lu",
                     (unsigned)ic,
                     (unsigned long)health->pec_pass_count[ic],
                     (unsigned long)health->pec_fail_count[ic],
                     (unsigned long)health->cmd_counter_mismatch_count[ic],
                     (unsigned long)health->config_mismatch_count[ic]);
            ret |= cli_printline(cli, outline);
        }

        snprintf(outline, CLI_LINESZ,
                 "IC%u SID:%s device_id:0x%02X %02X%02X%02X%02X%02X%02X",
                 (unsigned)ic,
                 diag->sid_valid ? "ok" : "--",
                 diag->device_id,
                 diag->sid[5], diag->sid[4], diag->sid[3],
                 diag->sid[2], diag->sid[1], diag->sid[0]);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "IC%u refs A:%d B:%d valid:%d VREF2:%dmV VD:%dmV VA:%dmV VRES:%dmV die:%ddC",
                 (unsigned)ic,
                 diag->stata_valid,
                 diag->statb_valid,
                 diag->reference_values_valid,
                 (int)diag->vref2_mv,
                 (int)diag->vd_mv,
                 (int)diag->va_mv,
                 (int)diag->vres_mv,
                 (int)diag->die_temp_deci_c);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "IC%u status C:%d D:%d E:%d cs:0x%04X ov:0x%04X uv:0x%04X osc:%u gpi:0x%03X rev:%u",
                 (unsigned)ic,
                 diag->statc_valid,
                 diag->statd_valid,
                 diag->state_valid,
                 diag->cs_flt_mask,
                 diag->cell_ov_mask,
                 diag->cell_uv_mask,
                 diag->osc_counter,
                 diag->gpi_mask,
                 diag->revision);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "IC%u supply VA:%u/%u VD:%u/%u memory C:%u CM:%u S:%u SM:%u",
                 (unsigned)ic,
                 diag->va_ov,
                 diag->va_uv,
                 diag->vd_ov,
                 diag->vd_uv,
                 diag->ced,
                 diag->cmed,
                 diag->sed,
                 diag->smed);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "IC%u digital VDE:%u VDEL:%u comp:%u spi:%u sleep:%u thsd:%u tmod:%u osc:%u",
                 (unsigned)ic,
                 diag->vde,
                 diag->vdel,
                 diag->comp,
                 diag->spiflt,
                 diag->sleep,
                 diag->thsd,
                 diag->tmodchk,
                 diag->oscchk);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "IC%u openwire base:%d even:%d/0x%04X odd:%d/0x%04X combined:0x%04X",
                 (unsigned)ic,
                 diag->open_wire_baseline_valid,
                 diag->open_wire_even_valid,
                 diag->open_wire_even_fault_mask,
                 diag->open_wire_odd_valid,
                 diag->open_wire_odd_fault_mask,
                 diag->open_wire_fault_mask);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "IC%u openwire attenuation even:0x%04X odd:0x%04X",
                 (unsigned)ic,
                 diag->open_wire_even_attenuation_fault_mask,
                 diag->open_wire_odd_attenuation_fault_mask);
        ret |= cli_printline(cli, outline);
    }

    ret |= cli_print_hex_preview("TX:", dbg->last_tx_preview, ADBMS6830_SPI_DEBUG_PREVIEW_BYTES);
    ret |= cli_print_hex_preview("RX:", dbg->last_rx_preview, ADBMS6830_SPI_DEBUG_PREVIEW_BYTES);

    return ret;
}



int get_apm_debug(int argc, char *argv[])
{
    int ret;

    adbms_spi_lock();
    ret = get_apm_debug_locked(argc, argv);
    adbms_spi_unlock();
    return ret;
}

static void cli_apm_resync_smb_tracking(void)
{
#if !AMS_APM_STANDALONE_EVAL_BENCH
    adbms6830_resync_command_counter_tracking(&data->acc.smb);
#endif
}

static int get_apm_debug_locked(int argc, char *argv[])
{
    int ret = 0;
    adbms2950_driver_t *apm = &data->acc.apm;
    const adbms2950_spi_debug_t *dbg;
    const adbms2950_health_t *health;
    const adbms2950_calibration_t *calibration;
    const adbms2950_redundant_sample_t *redundant;
    HAL_StatusTypeDef action_status = HAL_OK;
    SPI_HandleTypeDef *hspi = apm->hspi;

#if !AMS_ENABLE_SERVICE_CLI
    if((argc >= 2) && (argv[1] != NULL) &&
       strcmp(argv[1], "status") && strcmp(argv[1], "health") &&
       strcmp(argv[1], "help"))
    {
        return cli_service_action_refused("ADBMS2950 service action");
    }
#endif

    if((argc >= 2) && (argv[1] != NULL))
    {
        if(!strcmp(argv[1], "help"))
        {
            ret |= cli_printline(cli, "apm status|health              - complete APM diagnostic state");
            ret |= cli_printline(cli, "apm sid|probe                  - identity/transport read");
            ret |= cli_printline(cli, "apm refup                      - verify core/reference readiness");
            ret |= cli_printline(cli, "apm config                     - masked CFGA/CFGB readback");
            ret |= cli_printline(cli, "apm flags                      - decode STAT/FLAG and faults");
            ret |= cli_printline(cli, "apm raw                        - coherent raw core snapshot");
            ret |= cli_printline(cli, "apm sample                     - coherent I1/VB1 sample");
            ret |= cli_printline(cli, "apm redundant                  - I1/I2 and VB1/VB2 cross-check");
            ret |= cli_printline(cli, "apm eeprom                     - COMM/I2C ACK probe at 0x50");
            ret |= cli_printline(cli, "apm scope [sid|sample] [1-100] - bounded repeat/soak");
            ret |= cli_printline(cli, "apm recover                    - bounded APM reinitialization");
            ret |= cli_printline(cli, "apm profile der|eval           - 100uR or 50uR scaling");
            ret |= cli_printline(cli, "apm trace on|off               - SPI previews/counters");
            ret |= cli_printline(cli, "apm clear                      - clear diagnostic counters");
            return ret;
        }
        else if(!strcmp(argv[1], "clear"))
        {
            adbms2950_spi_debug_clear(apm);
            adbms2950_health_clear_counters(apm);
            ret |= cli_printline(cli, "ADBMS2950 SPI and health counters cleared");
        }
        else if(!strcmp(argv[1], "enable") ||
                (!strcmp(argv[1], "trace") && (argc >= 3) &&
                 (argv[2] != NULL) && !strcmp(argv[2], "on")))
        {
            adbms2950_spi_debug_enable(apm, true);
            ret |= cli_printline(cli, "ADBMS2950 SPI trace enabled");
        }
        else if(!strcmp(argv[1], "disable") ||
                (!strcmp(argv[1], "trace") && (argc >= 3) &&
                 (argv[2] != NULL) && !strcmp(argv[2], "off")))
        {
            adbms2950_spi_debug_enable(apm, false);
            ret |= cli_printline(cli, "ADBMS2950 SPI trace disabled");
        }
        else if(!strcmp(argv[1], "trace"))
        {
            ret |= cli_printline(cli, "Usage: apm trace [on|off]");
            return ret;
        }
        else if(!strcmp(argv[1], "probe") || !strcmp(argv[1], "sid"))
        {
            if(cli_adbms_refuse_active_scan("apm sid")) return ret;
            action_status = adbms2950_spi_probe_sid(apm);
            cli_apm_resync_smb_tracking();
            snprintf(outline, CLI_LINESZ, "ADBMS2950 RDSID: %s",
                     cli_hal_status_str(action_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "refup"))
        {
            if(cli_adbms_refuse_active_scan("apm refup")) return ret;
            action_status = adbms2950_verify_refup(apm);
            cli_apm_resync_smb_tracking();
            snprintf(outline, CLI_LINESZ, "ADBMS2950 REFUP verification: %s",
                     cli_hal_status_str(action_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "config"))
        {
            if(cli_adbms_refuse_active_scan("apm config")) return ret;
            action_status = adbms2950_verify_config_readback(apm);
            cli_apm_resync_smb_tracking();
            snprintf(outline, CLI_LINESZ, "ADBMS2950 config readback: %s",
                     cli_hal_status_str(action_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "flags"))
        {
            if(cli_adbms_refuse_active_scan("apm flags")) return ret;
            action_status = adbms2950_read_status(apm);
            if(action_status == HAL_OK) action_status = adbms2950_read_flag(apm);
            cli_apm_resync_smb_tracking();
            snprintf(outline, CLI_LINESZ, "ADBMS2950 STAT+FLAG: %s",
                     cli_hal_status_str(action_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "raw"))
        {
            adbms2950_core_snapshot_t snapshot;
            if(cli_adbms_refuse_active_scan("apm raw")) return ret;
            action_status = adbms2950_read_core_snapshot(apm, &snapshot);
            cli_apm_resync_smb_tracking();
            snprintf(outline, CLI_LINESZ,
                     "ADBMS2950 raw snapshot:%s valid:%d CCNT:%u/%u/%u/%u/%u",
                     cli_hal_status_str(action_status), snapshot.valid,
                     snapshot.cfga_ccnt, snapshot.cfgb_ccnt,
                     snapshot.status_ccnt, snapshot.flag_ccnt,
                     snapshot.sid_ccnt);
            ret |= cli_printline(cli, outline);
            ret |= cli_print_hex_preview("APM CFGA:", snapshot.cfga, TX_DATA);
            ret |= cli_print_hex_preview("APM CFGB:", snapshot.cfgb, TX_DATA);
            ret |= cli_print_hex_preview("APM STAT:", snapshot.status, TX_DATA);
            ret |= cli_print_hex_preview("APM FLAG:", snapshot.flag, TX_DATA);
            ret |= cli_print_hex_preview("APM SID:", snapshot.sid, TX_DATA);
        }
        else if(!strcmp(argv[1], "sample"))
        {
            if(cli_adbms_refuse_active_scan("apm sample")) return ret;
            action_status = adbms2950_read_primary_sample(apm,
                                                          osKernelGetTickCount());
            cli_apm_resync_smb_tracking();
            snprintf(outline, CLI_LINESZ,
                     "ADBMS2950 coherent I1/VB1 sample: %s",
                     cli_hal_status_str(action_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "profile"))
        {
            adbms2950_calibration_profile_t profile;
            if((argc < 3) || (argv[2] == NULL))
            {
                ret |= cli_printline(cli, "Usage: apm profile [der|eval]");
                return ret;
            }
            if(!strcmp(argv[2], "der"))
                profile = ADBMS2950_CAL_PROFILE_DER_APM;
            else if(!strcmp(argv[2], "eval"))
                profile = ADBMS2950_CAL_PROFILE_EVAL_BASIC;
            else
            {
                ret |= cli_printline(cli, "Usage: apm profile [der|eval]");
                return ret;
            }
            action_status = adbms2950_set_calibration_profile(apm, profile);
            snprintf(outline, CLI_LINESZ,
                     "ADBMS2950 calibration:%s status:%s",
                     adbms2950_calibration_profile_str(profile),
                     cli_hal_status_str(action_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "redundant"))
        {
            if(cli_adbms_refuse_active_scan("apm redundant")) return ret;
            action_status = adbms2950_read_redundant_sample(
                apm, osKernelGetTickCount());
            cli_apm_resync_smb_tracking();
            redundant = adbms2950_redundant_sample_get(apm);
            snprintf(outline, CLI_LINESZ,
                     "ADBMS2950 I1/I2+VB1/VB2:%s valid:%d",
                     cli_hal_status_str(action_status),
                     (redundant != NULL) ? redundant->valid : 0);
            ret |= cli_printline(cli, outline);
            if(redundant != NULL)
            {
                snprintf(outline, CLI_LINESZ,
                         "I1:%ld %.3fA I2:%ld %.3fA delta:%.3fA",
                         (long)redundant->i1_raw,
                         (double)redundant->current1_a,
                         (long)redundant->i2_raw,
                         (double)redundant->current2_a,
                         (double)redundant->current_disagreement_a);
                ret |= cli_printline(cli, outline);
                snprintf(outline, CLI_LINESZ,
                         "VB1:%d %.2fV VB2:%d %.2fV delta:%.2fV",
                         (int)redundant->vb1_raw,
                         (double)redundant->pack_voltage1_v,
                         (int)redundant->vb2_raw,
                         (double)redundant->pack_voltage2_v,
                         (double)redundant->pack_voltage_disagreement_v);
                ret |= cli_printline(cli, outline);
            }
        }
        else if(!strcmp(argv[1], "eeprom"))
        {
            adbms2950_i2c_probe_result_t probe;
            if(cli_adbms_refuse_active_scan("apm eeprom")) return ret;
            action_status = adbms2950_i2c_write_probe(apm, 0x50u, 0x00u, &probe);
            cli_apm_resync_smb_tracking();
            snprintf(outline, CLI_LINESZ,
                     "ADBMS2950 EEPROM 0x50:%s addrACK:%d dataACK:%d",
                     cli_hal_status_str(probe.transport_status),
                     probe.address_ack, probe.data_ack);
            ret |= cli_printline(cli, outline);
            ret |= cli_print_hex_preview("preRDCOMM:", probe.pre_rdcomm, TX_DATA);
            ret |= cli_print_hex_preview("postRDCOMM:", probe.post_rdcomm, TX_DATA);
        }
        else if(!strcmp(argv[1], "recover"))
        {
            if(cli_adbms_refuse_active_scan("apm recover")) return ret;
            action_status = adbms2950_recover(
                apm, (AMS_APM_STANDALONE_EVAL_BENCH != 0));
            data->acc.apm_init_status = action_status;
            data->acc.apm_ready = (action_status == HAL_OK);
            cli_apm_resync_smb_tracking();
            snprintf(outline, CLI_LINESZ,
                     "ADBMS2950 recovery reset:%d status:%s",
                     (AMS_APM_STANDALONE_EVAL_BENCH != 0),
                     cli_hal_status_str(action_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "scope"))
        {
            uint16_t repeat = 20u;
            uint16_t completed = 0u;
            const char *mode = "sid";
            uint8_t repeat_arg = 2u;
            float min_i = 0.0f, max_i = 0.0f;
            float min_v = 0.0f, max_v = 0.0f;

            if(cli_adbms_refuse_active_scan("apm scope")) return ret;
            if((argc >= 3) && (argv[2] != NULL) &&
               (!strcmp(argv[2], "sid") || !strcmp(argv[2], "sample")))
            {
                mode = argv[2];
                repeat_arg = 3u;
            }
            if((argc > repeat_arg) &&
               !cli_parse_scope_repeat(argv[repeat_arg], &repeat))
            {
                ret |= cli_printline(cli,
                    "Usage: apm scope [sid|sample] [1-100]");
                return ret;
            }
            if((argc == 3) && (repeat_arg == 2u) &&
               !cli_parse_scope_repeat(argv[2], &repeat))
            {
                ret |= cli_printline(cli,
                    "Usage: apm scope [sid|sample] [1-100]");
                return ret;
            }
            for(completed = 0u; completed < repeat; completed++)
            {
                if(!strcmp(mode, "sample"))
                {
                    action_status = adbms2950_read_primary_sample(
                        apm, osKernelGetTickCount());
                    if(action_status == HAL_OK)
                    {
                        const adbms2950_health_t *h = adbms2950_health_get(apm);
                        if(completed == 0u)
                        {
                            min_i = max_i = h->current_a;
                            min_v = max_v = h->pack_voltage_v;
                        }
                        else
                        {
                            if(h->current_a < min_i) min_i = h->current_a;
                            if(h->current_a > max_i) max_i = h->current_a;
                            if(h->pack_voltage_v < min_v) min_v = h->pack_voltage_v;
                            if(h->pack_voltage_v > max_v) max_v = h->pack_voltage_v;
                        }
                    }
                }
                else
                {
                    action_status = adbms2950_spi_probe_sid(apm);
                }
                if(action_status != HAL_OK) break;
            }
            cli_apm_resync_smb_tracking();
            snprintf(outline, CLI_LINESZ,
                     "ADBMS2950 scope %s requested:%u completed:%u status:%s",
                     mode, (unsigned)repeat, (unsigned)completed,
                     cli_hal_status_str(action_status));
            ret |= cli_printline(cli, outline);
            if(!strcmp(mode, "sample") && (completed > 0u))
            {
                snprintf(outline, CLI_LINESZ,
                         "scope range current:%.3f..%.3fA pack:%.2f..%.2fV",
                         (double)min_i, (double)max_i,
                         (double)min_v, (double)max_v);
                ret |= cli_printline(cli, outline);
            }
        }
        else if(strcmp(argv[1], "status") && strcmp(argv[1], "health"))
        {
            ret |= cli_printline(cli, "Usage: apm help");
            return ret;
        }
    }

    dbg = adbms2950_spi_debug_get(apm);
    health = adbms2950_health_get(apm);
    calibration = adbms2950_calibration_get(apm);
    redundant = adbms2950_redundant_sample_get(apm);
    if((dbg == NULL) || (health == NULL) || (calibration == NULL))
    {
        ret |= cli_printline(cli, "ADBMS2950 diagnostics unavailable");
        return ret;
    }

    snprintf(outline, CLI_LINESZ,
             "APM topology:%s selected:%s enabled:%d ready:%d init:%s",
#if AMS_APM_STANDALONE_EVAL_BENCH
             "standalone_eval_1x2950",
#else
             "final_ring_5x6830+1x2950",
#endif
             (apm->string == STRING_B) ? "CS_B" : "CS_A",
             AMS_ENABLE_APM_2950, data->acc.apm_ready,
             cli_hal_status_str(data->acc.apm_init_status));
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "APM safety: ADVISORY_NON_GATING; HV divider build:%d active:%d",
             AMS_APM_ENABLE_HV_DIVIDERS,
             health->hv_dividers_enabled);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "stage:%s reason:%s status:%s init:%d refup:%d/%d cfg:%d",
             adbms2950_stage_str(health->last_stage),
             adbms2950_reason_str(health->last_reason),
             cli_hal_status_str(health->last_status),
             health->initialized, health->refup_valid, health->refup,
             health->config_valid);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "identity sid:%d devid:0x%02X expected:0x%02X DER:%u rev:%u",
             health->sid_valid, health->device_id, ADBMS2950B_DEVICE_ID,
             (unsigned)health->derivative, (unsigned)health->revision);
    ret |= cli_printline(cli, outline);
    ret |= cli_print_hex_preview("SID:", health->sid, RSID);

    snprintf(outline, CLI_LINESZ,
             "STAT valid:%d I1CAL:%d I2CAL:%d I1cont:%d I2cont:%d",
             health->status_valid, health->i1_calibrated,
             health->i2_calibrated, health->i1_continuous_ready,
             health->i2_continuous_ready);
    ret |= cli_printline(cli, outline);
    ret |= cli_print_hex_preview("STAT:", health->raw_status, TX_DATA);
    snprintf(outline, CLI_LINESZ,
             "FLAG valid:%d faultmask:0x%08lX reset:%d ref_fail:%lu",
             health->flag_valid, (unsigned long)health->fault_mask,
             ((health->fault_mask & (1u << 23)) != 0u),
             (unsigned long)health->refup_failure_count);
    ret |= cli_printline(cli, outline);
    ret |= cli_print_hex_preview("FLAG:", health->raw_flag, TX_DATA);

    snprintf(outline, CLI_LINESZ,
             "cfg mismatch CFGA:0x%04X CFGB:0x%04X dividers:%s",
             health->configa_mismatch_ic_mask,
             health->configb_mismatch_ic_mask,
             health->hv_dividers_enabled ? "ON" : "OFF");
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "sample valid:%d I:%d V:%d rawI:%ld %.3fA rawVB:%d %.2fV",
             health->sample_valid, health->current_valid,
             health->pack_voltage_valid, (long)health->i1_raw,
             (double)health->current_a, (int)health->vb1_raw,
             (double)health->pack_voltage_v);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "fresh I1cnt:%u last:%u phase:%u seen:%d advanced:%d age:%lums",
             (unsigned)health->i1_conversion_count,
             (unsigned)health->last_i1_conversion_count,
             (unsigned)health->i1_conversion_phase,
             health->counter_seen, health->counter_advanced,
             (unsigned long)(health->sample_count ?
                 (osKernelGetTickCount() - health->last_update_ms) : 0u));
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "counts sample:%lu fail:%lu PEC:%lu CCmis:%lu stall:%lu reset:%lu",
             (unsigned long)health->sample_count,
             (unsigned long)health->sample_error_count,
             (unsigned long)health->pec_error_count,
             (unsigned long)health->counter_mismatch_count,
             (unsigned long)health->counter_stall_count,
             (unsigned long)health->unexpected_counter_reset_count);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "recovery pass:%lu fail:%lu",
             (unsigned long)health->recovery_count,
             (unsigned long)health->recovery_failure_count);
    ret |= cli_printline(cli, outline);

    if(hspi != NULL)
    {
        snprintf(outline, CLI_LINESZ,
                 "SPI6 CPOL:%s CPHA:%s prescaler:%lu first:%s",
                 cli_spi_polarity_str(hspi->Init.CLKPolarity),
                 cli_spi_phase_str(hspi->Init.CLKPhase),
                 (unsigned long)hspi->Init.BaudRatePrescaler,
                 (hspi->Init.FirstBit == SPI_FIRSTBIT_MSB) ? "MSB" : "LSB");
        ret |= cli_printline(cli, outline);
    }
    else ret |= cli_printline(cli, "SPI handle is NULL");

    snprintf(outline, CLI_LINESZ,
             "spi trace:%d op:%s status:%s tx:%lu rx:%lu err:%lu",
             dbg->enabled, adbms2950_spi_op_str(dbg->last_op),
             cli_hal_status_str(dbg->last_status),
             (unsigned long)dbg->tx_count, (unsigned long)dbg->rx_count,
             (unsigned long)dbg->error_count);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "HAL tx:%s rx:%s xfer:%s PEC pass:0x%04X fail:0x%04X",
             cli_hal_status_str(dbg->last_tx_status),
             cli_hal_status_str(dbg->last_rx_status),
             cli_hal_status_str(dbg->last_xfer_status),
             dbg->last_read_pec_pass_mask, dbg->last_read_pec_fail_mask);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "CCNT seen:0x%04X expect:0x%04X mismatch:0x%04X sticky:0x%04X",
             dbg->cmd_counter_seen_mask, dbg->cmd_counter_expected_mask,
             dbg->cmd_counter_mismatch_mask,
             dbg->sticky_cmd_counter_mismatch_mask);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "CCNT IC0 observed:%u expected:%u resets:0x%04X sticky:0x%04X",
             dbg->last_cmd_counter[0], dbg->expected_cmd_counter[0],
             dbg->unexpected_counter_reset_mask,
             dbg->sticky_unexpected_counter_reset_mask);
    ret |= cli_printline(cli, outline);
    ret |= cli_print_hex_preview("APM TX:", dbg->last_tx_preview,
                                 ADBMS2950_SPI_DEBUG_PREVIEW_BYTES);
    ret |= cli_print_hex_preview("APM RX:", dbg->last_rx_preview,
                                 ADBMS2950_SPI_DEBUG_PREVIEW_BYTES);

    snprintf(outline, CLI_LINESZ,
             "cal:%s shunt:%.1fuOhm gain:%.6f offset:%.3fuV polarity:%d",
             adbms2950_calibration_profile_str(calibration->profile),
             (double)(calibration->shunt_resistance_ohm * 1.0e6f),
             (double)calibration->current_gain,
             (double)calibration->current_offset_uv,
             (int)calibration->current_polarity);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ, "VB ratios VB1:%.3f VB2:%.3f",
             (double)calibration->vb1_divider_ratio,
             (double)calibration->vb2_divider_ratio);
    ret |= cli_printline(cli, outline);
    if(redundant != NULL)
    {
        snprintf(outline, CLI_LINESZ,
                 "redundant valid:%d status:%s dI:%.3fA dV:%.2fV",
                 redundant->valid,
                 cli_hal_status_str(redundant->last_status),
                 (double)redundant->current_disagreement_a,
                 (double)redundant->pack_voltage_disagreement_v);
        ret |= cli_printline(cli, outline);
    }
    return ret;
}


int get_current(int argc, char *argv[])
{
    int ret = 0;
    int whole = 0;
    int decimal = 0;
    current_sensor_t *cs = &data->board.current_sensor;
    current_sensor_t cs_snapshot;

    if((argc >= 2) && (argv[1] != NULL) && !strcmp(argv[1], "zero"))
    {
        if((argc >= 3) && (argv[2] != NULL) && !strcmp(argv[2], "clear"))
        {
#if AMS_ENABLE_SERVICE_CLI
            if((!data->bms_output_inhibit) || data->bms_state ||
               (data->state == STATE_CHARGE) ||
               (data->state == STATE_DISCARGE))
            {
                return cli_printline(cli,
                    "current zero clear refused: require BMS_OK inhibited, BMS low, and non-charge/non-drive state");
            }
            ams_current_window_lock();
            if((!data->bms_output_inhibit) || data->bms_state ||
               (data->state == STATE_CHARGE) ||
               (data->state == STATE_DISCARGE))
            {
                ams_current_window_unlock();
                return cli_printline(cli,
                    "current zero clear refused: safety state changed before clear");
            }
            current_sensor_zero_clear(cs);
            uint32_t change_tick = osKernelGetTickCount();
            ams_current_window_update(&data->current_window,
                                      change_tick,
                                      cs->current,
                                      cs->current_filtered,
                                      false,
                                      false,
                                      0u);
            taskENTER_CRITICAL();
            data->current_valid = false;
            data->current_meas_reason =
                CURRENT_SENSOR_REASON_CALIBRATION_CHANGED;
            taskEXIT_CRITICAL();
            ams_current_window_unlock();
            ret |= cli_printline(cli, "current zero: cleared");
            return ret;
#else
            return cli_service_action_refused("current zero clear");
#endif
        }

        if((argc >= 3) && (argv[2] != NULL) && !strcmp(argv[2], "status"))
        {
            ams_current_window_lock();
            cs_snapshot = *cs;
            ams_current_window_unlock();
            cli_fixed1(cs_snapshot.zero_offset_50a, &whole, &decimal);
            snprintf(outline, CLI_LINESZ,
                     "current zero: calibrated:%d captures:%lu offset50:%d.%01d A",
                     cs_snapshot.zero_calibrated,
                     (unsigned long)cs_snapshot.zero_cal_count,
                     whole,
                     decimal);
            ret |= cli_printline(cli, outline);

            cli_fixed1(cs_snapshot.zero_offset_800a, &whole, &decimal);
            snprintf(outline, CLI_LINESZ, "current zero: offset800:%d.%01d A", whole, decimal);
            ret |= cli_printline(cli, outline);
            snprintf(outline, CLI_LINESZ,
                     "current calibration: restored:%d id:%lu record_confident:%d",
                     cs_snapshot.calibration_loaded_from_record,
                     (unsigned long)cs_snapshot.calibration_id,
                     current_sensor_calibration_confident(&cs_snapshot));
            ret |= cli_printline(cli, outline);
            return ret;
        }

#if !AMS_ENABLE_SERVICE_CLI
        return cli_service_action_refused("current zero calibration");
#endif

        if((!data->bms_output_inhibit) || data->bms_state ||
           (data->state == STATE_CHARGE) || (data->state == STATE_DISCARGE))
        {
            ret |= cli_printline(cli, "current zero refused: require BMS_OK inhibited, BMS low, and non-charge/non-drive state");
            return ret;
        }

        ams_current_window_lock();
        if((!data->bms_output_inhibit) || data->bms_state ||
           (data->state == STATE_CHARGE) || (data->state == STATE_DISCARGE))
        {
            ams_current_window_unlock();
            ret |= cli_printline(cli, "current zero refused: safety state changed before capture");
            return ret;
        }

        if(!current_sensor_read_adc(cs))
        {
            ams_current_window_unlock();
            ret |= cli_printline(cli, "current zero refused: ADC read failed");
            return ret;
        }

        (void)current_sensor_convert(cs);
        bool zero_captured = current_sensor_zero_calibrate(cs);
        if(zero_captured)
        {
            (void)current_sensor_convert(cs);
            uint32_t change_tick = osKernelGetTickCount();
            ams_current_window_update(&data->current_window,
                                      change_tick,
                                      cs->current,
                                      cs->current_filtered,
                                      false,
                                      false,
                                      0u);
            cs->current_valid = false;
            cs->selected_range = CURRENT_SENSOR_RANGE_UNKNOWN;
            cs->reason = CURRENT_SENSOR_REASON_CALIBRATION_CHANGED;
            taskENTER_CRITICAL();
            data->current_valid = false;
            data->current_meas_reason =
                CURRENT_SENSOR_REASON_CALIBRATION_CHANGED;
            taskEXIT_CRITICAL();
        }
        cs_snapshot = *cs;
        ams_current_window_unlock();

        if(zero_captured)
        {
            cli_fixed1(cs_snapshot.zero_offset_50a, &whole, &decimal);
            snprintf(outline, CLI_LINESZ, "current zero captured: offset50:%d.%01d A", whole, decimal);
            ret |= cli_printline(cli, outline);
            cli_fixed1(cs_snapshot.zero_offset_800a, &whole, &decimal);
            snprintf(outline, CLI_LINESZ, "current zero captured: offset800:%d.%01d A", whole, decimal);
            ret |= cli_printline(cli, outline);
        }
        else
        {
            snprintf(outline, CLI_LINESZ,
                     "current zero refused: raw50/800 not near zero or sensor invalid reason:%s",
                     current_sensor_reason_str(cs_snapshot.reason));
            ret |= cli_printline(cli, outline);
        }
        return ret;
    }

    ams_current_window_lock();
    cs_snapshot = *cs;
    ams_current_window_unlock();
    cs = &cs_snapshot;

    snprintf(outline, CLI_LINESZ, "Current valid:%d fault:%d sensor:%d oc:%d latch:%d",
             data->current_valid,
             data->current_fault,
             data->current_sensor_fault,
             data->current_overcurrent_fault,
             data->current_fault_latched);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ, "Meas:%s range:%s mode:%s fault:%s",
             current_sensor_reason_str(data->current_meas_reason),
             current_sensor_range_str(data->current_selected_range),
             current_fault_mode_str(data->current_fault_mode),
             current_fault_reason_str(data->current_fault_reason));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ, "Current warn:%d pending:%d pend_ms:%lu latch_reason:%s",
             data->current_overcurrent_warning,
             data->current_overcurrent_pending,
             (unsigned long)data->current_fault_state.pending_ms,
             current_fault_reason_str(data->current_fault_latched_reason));
    ret |= cli_printline(cli, outline);

    ret |= cli_printline(cli, "ADC map L:PC0 ADC2_IN10 50A H:PA3 ADC1_IN3 800A");

    snprintf(outline, CLI_LINESZ, "ADC raw H:%u L:%u", cs->count_high, cs->count_low);
    ret |= cli_printline(cli, outline);

    cli_fixed3(cs->voltage_high, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "ADC H: %d.%03d V", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed3(cs->voltage_low, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "ADC L: %d.%03d V", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed3(cs->sensor_voltage_high, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "DHAB H: %d.%03d V", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed3(cs->sensor_voltage_low, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "DHAB L: %d.%03d V", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(cs->current_50a_raw, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "I_50A_raw: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(cs->current_800a_raw, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "I_800A_raw: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(cs->current_50a, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "I_50A: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(cs->current_800a, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "I_800A: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(cs->current, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "I_selected: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(cs->current_filtered, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "I_filtered_telemetry: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(cs->zero_offset_50a, &whole, &decimal);
    snprintf(outline, CLI_LINESZ,
             "Zero calibrated:%d captures:%lu offset50:%d.%01d A",
             cs->zero_calibrated,
             (unsigned long)cs->zero_cal_count,
             whole,
             decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(cs->zero_offset_800a, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "Zero offset800:%d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Calibration restored:%d id:%lu restores:%lu record_confident:%d",
             cs->calibration_loaded_from_record,
             (unsigned long)cs->calibration_id,
             (unsigned long)cs->calibration_restore_count,
             current_sensor_calibration_confident(cs));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Calibration time:%lu temp_dC:%d uncertainty_mA:%u/%u",
             (unsigned long)cs->calibration_capture_time_s,
             (int)cs->calibration_temp_deci_c,
             (unsigned)cs->calibration_uncertainty_50a_mA,
             (unsigned)cs->calibration_uncertainty_800a_mA);
    ret |= cli_printline(cli, outline);

    cli_fixed3(cs->adc_vref_v, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "Ref adc:%d.%03d V", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed3(cs->sensor_supply_v, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "Ref dhab_supply:%d.%03d V", whole, decimal);
    ret |= cli_printline(cli, outline);

    return ret;
}

int get_estimator_diag(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    if((data == NULL) || (data->estimator.enabled == 0u) ||
       (data->estimator.instance_count == 0u) ||
       (data->estimator.active_index >= data->estimator.instance_count) ||
       (data->estimator.active_index >= AMS_EKF_MAX_INSTANCES))
    {
        return cli_printline(cli, "Estimator unavailable or misconfigured");
    }

    int ret = 0;
    uint8_t active = data->estimator.active_index;
    const ams_ekf_instance_t *inst = &data->estimator.inst[active];
    const ams_resistance_soh_t *soh =
        &data->estimator.resistance_soh[active];

    snprintf(outline, CLI_LINESZ,
             "Estimator source:%u active:%u/%u valid:%u flags:0x%08lX model:0x%02X",
             (unsigned)data->estimator.input_source,
             (unsigned)active,
             (unsigned)data->estimator.instance_count,
             (unsigned)inst->valid,
             (unsigned long)data->estimator.fault_flags,
             (unsigned)data->estimator.model_domain_flags);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Epoch seq:%lu repeat:%lu missed:%lu timing_fault:%lu voltage_tick:%lu",
             (unsigned long)data->estimator.last_consumed_measurement_sequence,
             (unsigned long)data->estimator.repeated_measurement_count,
             (unsigned long)data->estimator.missed_measurement_count,
             (unsigned long)data->estimator.epoch_timing_fault_count,
             (unsigned long)data->estimator.last_voltage_tick);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Measurement publication drops:%lu",
             (unsigned long)data->measurement_store.publication_drop_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "SoC:%.4f cell_R0:%.5f ohm pack_R0:%.5f ohm innovation:%.4f V p_R0:%.7f",
             (double)data->estimator.pack_soc,
             (double)data->estimator.representative_cell_r0_ohm,
             (double)data->estimator.estimated_pack_r0_ohm,
             (double)data->estimator.pack_innovation_V,
             (double)inst->p_r0);
    ret |= cli_printline(cli, outline);

#if AMS_ENABLE_BENCH_PASSIVE_RING_ESTIMATOR
    snprintf(outline, CLI_LINESZ,
             "Passive ring current:%s temperature:%s; advisory SoC only",
             data->current_valid ? "DHAB" : "assumed_zero",
             data->temp_valid ? "measured" : "fixed_25C");
    ret |= cli_printline(cli, outline);
#endif

    for(uint8_t segment = 0u;
        (segment < data->estimator.instance_count) &&
        (segment < AMS_EKF_MAX_INSTANCES);
        segment++)
    {
        const ams_ekf_instance_t *segment_inst =
            &data->estimator.inst[segment];
        snprintf(outline, CLI_LINESZ,
                 "Segment %u SoC:%.4f valid:%u acq:%u reason:%u samples:%u",
                 (unsigned)segment,
                 (double)segment_inst->soc,
                 (unsigned)segment_inst->valid,
                 (unsigned)segment_inst->acquisition.state,
                 (unsigned)segment_inst->acquisition.reason,
                 (unsigned)segment_inst->acquisition.sample_count);
        ret |= cli_printline(cli, outline);
    }

    {
        const uint16_t compare_bit = (uint16_t)(1u << active);
        const bool raw_ok =
            (data->estimator.voltage_raw_valid_mask & compare_bit) != 0u;
        const bool avg_ok =
            (data->estimator.voltage_avg8_valid_mask & compare_bit) != 0u;
        const bool iir_ok =
            (data->estimator.voltage_iir_valid_mask & compare_bit) != 0u;
        const double raw_v = (double)data->estimator.voltage_raw_V[active];
        const double avg_v = (double)data->estimator.voltage_avg8_V[active];
        const double iir_v = (double)data->estimator.voltage_iir_V[active];

        snprintf(outline, CLI_LINESZ,
                 "Vcmp seq:%lu src:%u valid R/A/F:%u/%u/%u counts:%u/%u/%u",
                 (unsigned long)data->estimator.voltage_compare_sequence,
                 (unsigned)AMS_ESTIMATOR_VOLTAGE_SOURCE,
                 (unsigned)raw_ok, (unsigned)avg_ok, (unsigned)iir_ok,
                 (unsigned)data->estimator.voltage_raw_valid_count[active],
                 (unsigned)data->estimator.voltage_avg8_valid_count[active],
                 (unsigned)data->estimator.voltage_iir_valid_count[active]);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "Vcmp R/A/F:%.4f/%.4f/%.4f V dA-R:%.1f dF-R:%.1f mV",
                 raw_v, avg_v, iir_v,
                 (avg_ok && raw_ok) ? (avg_v - raw_v) * 1000.0 : 0.0,
                 (iir_ok && raw_ok) ? (iir_v - raw_v) * 1000.0 : 0.0);
        ret |= cli_printline(cli, outline);
    }

    snprintf(outline, CLI_LINESZ,
             "EKF reject innovation:%lu dt_clamp:%lu CC_dt_clamp:%lu CC_last_clamped:%u",
             (unsigned long)inst->innovation_reject_count,
             (unsigned long)inst->dt_clamp_count,
             (unsigned long)data->estimator.cc_dt_clamp_count,
             (unsigned)data->estimator.cc_last_dt_clamped);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "R0-SoH ADVISORY status:0x%02X confidence:%u%% persisted:%u last_reject:0x%03lX",
             (unsigned)soh->status_flags,
             (unsigned)soh->observation_confidence_pct,
             (unsigned)soh->persistence_valid,
             (unsigned long)soh->last_reject_flags);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "R0 est:%.6f ref:%.6f ohm growth:%.4f accepted:%lu rejected:%lu",
             (double)soh->estimated_cell_r0_ohm,
             (double)soh->reference_cell_r0_ohm,
             (double)soh->resistance_growth_ratio,
             (unsigned long)soh->accepted_count,
             (unsigned long)soh->rejected_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Reject epoch:%lu calibration:%lu lowI:%lu step:%lu recovery:%lu domain:%lu",
             (unsigned long)soh->reject_epoch_count,
             (unsigned long)soh->reject_current_calibration_count,
             (unsigned long)soh->reject_low_current_count,
             (unsigned long)soh->reject_low_current_step_count,
             (unsigned long)soh->reject_balance_recovery_count,
             (unsigned long)soh->reject_model_domain_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Reject innovation:%lu clamp:%lu numeric:%lu estimator:%lu last_accept_age:%lums",
             (unsigned long)soh->reject_innovation_count,
             (unsigned long)soh->reject_r0_clamp_count,
             (unsigned long)soh->reject_numeric_count,
             (unsigned long)soh->reject_estimator_count,
             (unsigned long)((soh->accepted_count != 0u) ?
                 (osKernelGetTickCount() - soh->last_accept_tick) : UINT32_MAX));
    ret |= cli_printline(cli, outline);

    ret |= cli_printline(cli,
        "R0 observations feed SoP only through a confidence-bounded upper resistance; missing confidence selects the conservative prior");
    return ret;
}

int get_power_diag(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    if(data == NULL)
    {
        return cli_printline(cli, "Power estimator unavailable");
    }

    ams_power_can_snapshot_t snapshot;
    ams_soh_result_t soh;
    uint32_t updates;
    uint32_t valid_count;
    uint32_t invalid_count;
    uint32_t numeric_count;
    uint32_t solver_status;
    bool power_limit_fault;
    ams_mission_request_state_t mission_request;
    taskENTER_CRITICAL();
    snapshot = data->power_can_snapshot;
    soh = data->power_state.soh.result;
    updates = data->power_state.update_count;
    valid_count = data->power_state.valid_count;
    invalid_count = data->power_state.invalid_count;
    numeric_count = data->power_state.numeric_failure_count;
    solver_status = data->power_state.last_solver_status;
    power_limit_fault = data->power_limit_fault;
    mission_request = data->mission_request;
    taskEXIT_CRITICAL();

    int ret = 0;
    const uint32_t now = osKernelGetTickCount();
    snprintf(outline, CLI_LINESZ,
             "SoP valid:%u authority:%u fault:%u status:%lu generation:%lu seq:%lu",
             (unsigned)snapshot.valid,
             (unsigned)snapshot.authority_valid,
             (unsigned)power_limit_fault,
             (unsigned long)solver_status,
             (unsigned long)snapshot.generation,
             (unsigned long)snapshot.measurement_sequence);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "Measurement age:%lums solve age:%lums",
             (unsigned long)(now - snapshot.measurement_timestamp_ms),
             (unsigned long)(now - snapshot.solve_timestamp_ms));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Mission:%u horizon:%u limp:%u request_fresh:%u strategy:0x%04X",
             (unsigned)snapshot.mission_profile,
             (unsigned)snapshot.mission_horizon_index,
             (unsigned)snapshot.limp_latched,
             (unsigned)ams_mission_request_fresh(&mission_request, now),
             (unsigned)snapshot.strategy_reason_flags);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "Fuse utilization:%.2f authority:%u thermal core min:%.1fC target energy:%.1fWh ready:%u R0 bootstrap:%u%%",
             (double)snapshot.fuse_utilization,
             (unsigned)snapshot.fuse_authority_valid,
             (double)snapshot.minimum_core_temp_c,
             (double)snapshot.thermal_energy_to_target_wh,
             (unsigned)snapshot.thermal_ready,
             (unsigned)snapshot.r0_bootstrap_progress_pct);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Reasons:0x%08lX updates:%lu valid:%lu invalid:%lu numeric:%lu",
             (unsigned long)snapshot.reason_flags,
             (unsigned long)updates,
             (unsigned long)valid_count,
             (unsigned long)invalid_count,
             (unsigned long)numeric_count);
    ret |= cli_printline(cli, outline);

    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        snprintf(outline, CLI_LINESZ,
                 "H%.1fs DCL:%.1fA CCL:%.1fA",
                 (double)data->power_state.sop_config.horizons_s[h],
                 (double)snapshot.discharge_current_a[h],
                 (double)snapshot.charge_current_a[h]);
        ret |= cli_printline(cli, outline);
    }

    snprintf(outline, CLI_LINESZ,
             "1s DPL:%.0fW CPL:%.0fW bind:%s/s%u %s/s%u",
             (double)snapshot.discharge_power_w_1s,
             (double)snapshot.charge_power_w_1s,
             ams_sop_binding_name((ams_sop_binding_t)
                                  snapshot.discharge_binding[1]),
             (unsigned)snapshot.discharge_limiting_segment[1],
             ams_sop_binding_name((ams_sop_binding_t)
                                  snapshot.charge_binding[1]),
             (unsigned)snapshot.charge_limiting_segment[1]);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Capacity SoH:%.3f lower:%.3f sigma:%.2fAh confidence:%u%% valid:%u windows:%lu/%lu",
             (double)soh.capacity_soh,
             (double)soh.capacity_soh_lower,
             (double)soh.capacity_sigma_ah,
             (unsigned)soh.capacity_confidence_pct,
             (unsigned)soh.capacity_valid,
             (unsigned long)soh.accepted_capacity_windows,
             (unsigned long)soh.rejected_capacity_windows);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Resistance growth upper:%.3f confidence:%u%% valid:%u combined SoH:%.3f persist:%u",
             (double)soh.resistance_growth_upper,
             (unsigned)soh.resistance_confidence_pct,
             (unsigned)soh.resistance_valid,
             (double)soh.combined_soh,
             (unsigned)soh.persistence_valid);
    ret |= cli_printline(cli, outline);
    return ret;
}

int get_can_diag(int argc, char *argv[])
{
    int ret = 0;

    if((argc >= 2) && (argv[1] != NULL) && !strcmp(argv[1], "recover"))
    {
#if AMS_ENABLE_SERVICE_CLI
        HAL_StatusTypeDef status = canbus_recover(&data->board.canbus);
        if(status == HAL_OK)
        {
            data->can_recover_count++;
            data->can_busoff_fault = false;
            data->can_recover_pending = false;
            data->can_error_code = HAL_CAN_ERROR_NONE;
            data->canbus_fault = false;
            ams_fault_log_event(AMS_FAULT_LOG_CAN_RECOVERED, 0u, data->can_recover_count, 0u);
        }
        snprintf(outline, CLI_LINESZ, "CAN recover: %s", cli_hal_status_str(status));
        ret |= cli_printline(cli, outline);
#else
        ret |= cli_service_action_refused("manual CAN recovery");
#endif
    }
    else if((argc >= 2) && (argv[1] != NULL) && strcmp(argv[1], "diag"))
    {
        ret |= cli_printline(cli, "Usage: can [diag|recover]");
        return ret;
    }

    snprintf(outline, CLI_LINESZ,
             "CAN err:0x%08lX %s busoff:%d pending:%d counts err:%lu busoff:%lu recover:%lu last_tick:%lu",
             (unsigned long)data->can_error_code,
             canbus_error_str(data->can_error_code),
             data->can_busoff_fault,
             data->can_recover_pending,
             (unsigned long)data->can_error_count,
             (unsigned long)data->can_busoff_count,
             (unsigned long)data->can_recover_count,
             (unsigned long)data->can_last_error_tick);
    ret |= cli_printline(cli, outline);

    canbus_device_t *cdev = &data->board.canbus;
    const ams_can_tx_scheduler_t *sched = &cdev->tx_scheduler;
    uint16_t rx_queued = canbus_rx_queue_count(cdev);
    uint32_t now = osKernelGetTickCount();
    uint32_t feedback_age = (cdev->rx_feedback_count != 0u) ?
        (uint32_t)(now - cdev->ecu_feedback_last_tick) : UINT32_MAX;

    snprintf(outline, CLI_LINESZ,
             "CAN RX isr:%lu proc:%lu filt:%lu q:%u high:%u drop:%lu hal:%lu",
             (unsigned long)cdev->rx_isr_count,
             (unsigned long)cdev->rx_processed_count,
             (unsigned long)cdev->rx_filtered_count,
             (unsigned)rx_queued,
             (unsigned)cdev->rx_queue_high_water,
             (unsigned long)cdev->rx_queue_drop_count,
             (unsigned long)cdev->rx_hal_error_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN RX reject rtr:%lu malformed:%lu fifo_ovr:%lu feedback:%lu age:%lu",
             (unsigned long)cdev->rx_remote_reject_count,
             (unsigned long)cdev->rx_malformed_count,
             (unsigned long)cdev->rx_fifo_overrun_count,
             (unsigned long)cdev->rx_feedback_count,
             (unsigned long)feedback_age);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN TX publish critical:%lu/%lu protected:%lu/%lu detail:%lu/%lu",
             (unsigned long)data->can_tx_critical_attempt_count,
             (unsigned long)data->can_tx_critical_fail_count,
             (unsigned long)data->can_tx_compact_bundle_count,
             (unsigned long)data->can_tx_compact_bundle_fail_count,
             (unsigned long)data->can_tx_detail_phase_count,
             (unsigned long)data->can_tx_detail_phase_fail_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN sched prot gen:%lu sup:%lu deadline:%lu detail gen:%lu done:%lu sup:%lu",
             (unsigned long)sched->protected_generated,
             (unsigned long)sched->protected_superseded,
             (unsigned long)sched->protected_deadline_miss,
             (unsigned long)sched->detail_generated,
             (unsigned long)sched->detail_completed,
             (unsigned long)sched->detail_superseded);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ, "CAN tuning shed:%lu recovery_wait:%d refresh_wait:%d",
             (unsigned long)sched->detail_tuning_shed,
             cdev->tx_recovery_pending, cdev->tx_refresh_pending);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN protected required complete:%lu latency_ms last:%lu max:%lu over50:%lu",
             (unsigned long)sched->protected_required_complete_count,
             (unsigned long)sched->protected_required_latency_last_ms,
             (unsigned long)sched->protected_required_latency_max_ms,
             (unsigned long)sched->protected_required_latency_over_50ms);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN TX load err total:%lu c:%lu p:%lu d:%lu tme:%lu",
             (unsigned long)cdev->tx_hal_load_error_count,
             (unsigned long)cdev->tx_hal_load_error_critical_count,
             (unsigned long)cdev->tx_hal_load_error_protected_count,
             (unsigned long)cdev->tx_hal_load_error_detail_count,
             (unsigned long)cdev->tx_irq_mask_error_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN build reject overflow:%lu detail:%lu class:%lu commit:%lu",
             (unsigned long)cdev->tx_build_overflow_count,
             (unsigned long)cdev->tx_build_detail_overflow_count,
             (unsigned long)cdev->tx_build_class_reject_count,
             (unsigned long)cdev->tx_build_commit_reject_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN TX complete:%lu abort req:%lu done:%lu race:%lu fail:%lu unexpected:%lu",
             (unsigned long)cdev->tx_complete_count,
             (unsigned long)cdev->tx_abort_request_count,
             (unsigned long)cdev->tx_abort_complete_count,
             (unsigned long)cdev->tx_abort_race_complete_count,
             (unsigned long)cdev->tx_abort_request_fail_count,
             (unsigned long)cdev->tx_unexpected_callback_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN pump kick:%lu defer:%lu epoch:%lu suspended:%d tx_latch:%d",
             (unsigned long)cdev->tx_pump_kick_count,
             (unsigned long)cdev->tx_pump_deferred_kick_count,
             (unsigned long)sched->controller_epoch,
             cdev->tx_suspended,
             cdev->tx_latched_inhibit);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN pending req:%u adv:%u detail:%u detail_recovery_drop:%lu",
             (unsigned)ams_can_tx_pending_count(sched,
                 AMS_CAN_TX_CLASS_PROTECTED_REQUIRED),
             (unsigned)ams_can_tx_pending_count(sched,
                 AMS_CAN_TX_CLASS_PROTECTED_ADVISORY),
             (unsigned)ams_can_tx_pending_count(sched,
                 AMS_CAN_TX_CLASS_DETAIL),
             (unsigned long)sched->detail_discarded_on_recovery);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN ECUfb ver:%u prot_seq:%u snap:%u flags:0x%02X rxdiag:%u",
             (unsigned)cdev->ecu_feedback_version,
             (unsigned)cdev->ecu_feedback_protected_sequence,
             (unsigned)cdev->ecu_feedback_snapshot_sequence,
             (unsigned)cdev->ecu_feedback_flags,
             (unsigned)cdev->ecu_feedback_rx_diag);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN task cycles:%lu deadline:%lu duration_ms last:%lu max:%lu budget:%u",
             (unsigned long)data->can_task_cycle_count,
             (unsigned long)data->can_task_deadline_miss_count,
             (unsigned long)data->can_task_last_duration_ms,
             (unsigned long)data->can_task_max_duration_ms,
             (unsigned)AMS_CAN_ECU_FAST_PERIOD_MS);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN %s %uk init:%s start:%s notify:%s active:%d",
             DER26_CAN_CONTRACT_NAME,
             (unsigned)DER26_CAN_BITRATE_KBPS,
             cli_hal_status_str(cdev->init_status),
             cli_hal_status_str(cdev->start_status),
             cli_hal_status_str(cdev->notification_status),
             cdev->notification_active);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN fault:%d charger:%d busoff_window:%u/3 ABOM:1 app_tx_latch:%d",
             data->canbus_fault,
             data->charger_fault,
             (unsigned)cdev->busoff_window_count,
             cdev->tx_latched_inhibit);
    ret |= cli_printline(cli, outline);

	return ret;
}

int watchdog_control(int argc, char *argv[])
{
    int ret = 0;

    if((argc >= 2) && (argv[1] != NULL))
    {
        if(!strcmp(argv[1], "enable"))
        {
#if AMS_ENABLE_SERVICE_CLI
#if AMS_ENABLE_IWDG
            ams_safety_watchdog_enable_runtime(data, true);
            ret |= cli_printline(cli, "watchdog runtime feed gate enabled");
#else
            ret |= cli_printline(cli, "watchdog compile flag disabled; rebuild with AMS_ENABLE_IWDG=1");
#endif
#else
            ret |= cli_service_action_refused("watchdog runtime enable");
#endif
        }
#if AMS_FAULT_INJECTION_CLI && AMS_ENABLE_SERVICE_CLI
        else if(!strcmp(argv[1], "stopfeed"))
        {
            ams_safety_watchdog_stop_feed_for_test(true);
            ret |= cli_printline(cli, "watchdog feed intentionally stopped for fault injection");
        }
        else if(!strcmp(argv[1], "feedok"))
        {
            ams_safety_watchdog_stop_feed_for_test(false);
            ret |= cli_printline(cli, "watchdog feed stop injection cleared");
        }
#endif
        else if(strcmp(argv[1], "status"))
        {
            ret |= cli_printline(cli, "Usage: wdg [status|enable]");
            return ret;
        }
    }

    snprintf(outline, CLI_LINESZ,
             "WDG compile:%d runtime:%d hw:%d ok_now:%d feeds:%lu blocks:%lu last_feed:%lu last_block:%s(%lu)",
             AMS_ENABLE_IWDG,
             data->watchdog_runtime_enabled,
             data->watchdog_hw_started,
             ams_safety_watchdog_ok(data),
             (unsigned long)data->watchdog_feed_count,
             (unsigned long)data->watchdog_block_count,
             (unsigned long)data->watchdog_last_feed_tick,
             ams_safety_watchdog_block_reason_str(data->watchdog_last_block_reason),
             (unsigned long)data->watchdog_last_block_reason);
    ret |= cli_printline(cli, outline);

    return ret;
}

int get_uart_diag(int argc, char *argv[])
{
    int ret = 0;
    UART_HandleTypeDef *huart = cli->huart;

    if((argc >= 2) && (argv[1] != NULL))
    {
        if(!strcmp(argv[1], "recover"))
        {
            HAL_StatusTypeDef status = cli_uart_force_recover(cli);
            snprintf(outline, CLI_LINESZ, "UART RX recovery: %s", cli_hal_status_str(status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "clear"))
        {
            cli_uart_diag_clear(cli);
            ret |= cli_printline(cli, "UART diagnostics cleared");
        }
        else if(strcmp(argv[1], "status"))
        {
            return cli_printline(cli, "Usage: uart [status|recover|clear]");
        }
    }

    if((huart == NULL) || (huart->Instance == NULL))
    {
        return ret | cli_printline(cli, "UART3 handle unavailable");
    }

    snprintf(outline, CLI_LINESZ,
             "UART3 gstate:0x%02lX rxstate:0x%02lX error:0x%08lX last:0x%08lX",
             (unsigned long)huart->gState,
             (unsigned long)huart->RxState,
             (unsigned long)HAL_UART_GetError(huart),
             (unsigned long)cli->uart_last_error);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "UART3 ISR:0x%08lX CR1:0x%08lX CR3:0x%08lX last_rx:%s",
             (unsigned long)huart->Instance->ISR,
             (unsigned long)huart->Instance->CR1,
             (unsigned long)huart->Instance->CR3,
             cli_hal_status_str(cli->rx_last_status));
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "UART errors total:%lu ORE:%lu FE:%lu NE:%lu PE:%lu RTO:%lu DMA:%lu",
             (unsigned long)cli->uart_error_count,
             (unsigned long)cli->uart_ore_count,
             (unsigned long)cli->uart_fe_count,
             (unsigned long)cli->uart_ne_count,
             (unsigned long)cli->uart_pe_count,
             (unsigned long)cli->uart_rto_count,
             (unsigned long)cli->uart_dma_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "UART RX arm:%lu recover:%lu fail:%lu busy:%lu dropped:%lu",
             (unsigned long)cli->rx_arm_count,
             (unsigned long)cli->rx_recovery_count,
             (unsigned long)cli->rx_rearm_fail_count,
             (unsigned long)cli->rx_busy_count,
             (unsigned long)cli->rx_drop_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CLI msg count:%u processed:%u valid:%u pending:%d index:%u",
             cli->msg_count,
             cli->msg_proc,
             cli->msg_valid,
             cli->msg_pending,
             cli->index);
    ret |= cli_printline(cli, outline);

    return ret;
}

int get_rtos_diag(int argc, char *argv[])
{
    int ret = 0;
    (void)argc;
    (void)argv;

    ams_rtos_diag_update(data);

    snprintf(outline, CLI_LINESZ,
             "RTOS heap free:%lu min:%lu warn<%u fault:%d stack_warn:%d stack_crit:%d heap_warn:%d",
             (unsigned long)data->rtos_heap_free_bytes,
             (unsigned long)data->rtos_heap_min_ever_free_bytes,
             AMS_RTOS_HEAP_WARN_BYTES,
             data->rtos_fault,
             data->rtos_stack_warning,
             data->rtos_stack_critical,
             data->rtos_heap_warning);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "RTOS flags:0x%04X last:%s task:%u tick:%lu",
             data->rtos_fault_flags,
             ams_rtos_fault_reason_str(data->rtos_last_fault_reason),
             data->rtos_last_fault_task,
             (unsigned long)data->rtos_last_fault_tick);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "RTOS counts malloc:%lu stack_ovf:%lu assert:%lu assert_line:%lu",
             (unsigned long)data->rtos_malloc_fail_count,
             (unsigned long)data->rtos_stack_overflow_count,
             (unsigned long)data->rtos_assert_fail_count,
             (unsigned long)data->rtos_last_assert_line);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "RTOS min stack high-water:%u words warn_mask:0x%04X crit_mask:0x%04X",
             data->rtos_min_stack_high_water_words,
             data->rtos_stack_warn_mask,
             data->rtos_stack_critical_mask);
    ret |= cli_printline(cli, outline);

    for(uint8_t i = 0u; i < (uint8_t)AMS_RTOS_TASK_COUNT; i++)
    {
        ams_rtos_task_id_t id = (ams_rtos_task_id_t)i;
        snprintf(outline, CLI_LINESZ,
                 "  %u %-9s prio:%u stack:%u highwater:%u warn<%u crit<%u words",
                 (unsigned)i,
                 ams_rtos_task_name(id),
                 ams_rtos_task_priority(id),
                 data->rtos_stack_config_words[i],
                 data->rtos_stack_high_water_words[i],
                 ams_rtos_task_stack_warn_words(id),
                 ams_rtos_task_stack_critical_words(id));
        ret |= cli_printline(cli, outline);
    }

    return ret;
}

int get_charger(int argc, char *argv[])
{
    int ret = 0;
    int whole = 0;
    int decimal = 0;
    charger_t *ccs = &data->board.charger;
    uint32_t now = osKernelGetTickCount();
    uint32_t age_ms = (ccs->last_rx_tick == 0u) ? 0xFFFFFFFFu : (now - ccs->last_rx_tick);

    (void)argc;
    (void)argv;

    cli_fixed1(ccs->target_voltage, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "Charger target: %d.%01d V", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(ccs->target_current, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "Charger current: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(ccs->read_voltage, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "Charger read V: %d.%01d V", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(ccs->read_current, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "Charger read I: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Charger flags raw:0x%02X hw:%d ot:%d input:%d sense:%d rx_comm:%d tx_fail:%d",
             ccs->flags,
             ccs->hardware_fail,
             ccs->overtemp_fail,
             ccs->input_volt_fail,
             ccs->voltage_sense_fail,
             ccs->communication_fail,
             ccs->tx_fail);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Charger counts tx:%lu rx:%lu txfail:%lu last_tx:%d rx_age_ms:%lu",
             (unsigned long)ccs->tx_count,
             (unsigned long)ccs->rx_count,
             (unsigned long)ccs->tx_fail_count,
             (int)ccs->last_tx_status,
             (unsigned long)age_ms);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Charger disable_mask:0x%04X cmd_period:%ums rx_timeout:%ums",
             ccs->disable_reason_mask,
             CHARGER_COMMAND_PERIOD_MS,
             CHARGER_RX_TIMEOUT_MS);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Charger CAN tx:0x%08lX rx:0x%08lX BYTE5/data[4] enable:%u disable:%u",
             (unsigned long)CCS_CANBUS_ID,
             (unsigned long)CHARGER_RX_ID,
             CHARGER_CMD_ENABLE,
             CHARGER_CMD_DISABLE);
    ret |= cli_printline(cli, outline);

    return ret;
}

int get_bringup(int argc, char *argv[])
{
    int ret = 0;
    const char *mode = ((argc >= 2) && (argv != NULL) && (argv[1] != NULL)) ? argv[1] : "help";
    adbms6830_driver_t *smb = &data->acc.smb;
    adbms2950_driver_t *apm = &data->acc.apm;
    charger_t *ccs = &data->board.charger;
    const adbms6830_spi_debug_t *smb_dbg = adbms6830_spi_debug_get(smb);
    const adbms6830_diag_health_t *smb_health = adbms6830_diag_health_get(smb);
    const adbms2950_spi_debug_t *apm_dbg = adbms2950_spi_debug_get(apm);
    uint8_t smb_count = smb_ic_count(smb);
    uint16_t expected_mask = cli_expected_ic_mask(smb_count);

    if(!strcmp(mode, "help"))
    {
        ret |= cli_printline(cli, "bringup board          - LV board-only checklist, no accumulator required");
        ret |= cli_printline(cli, "bringup adbms6830      - SMB chain SPI/CS/PEC/SID/status summary");
		ret |= cli_printline(cli, "bringup apm2950        - standalone/final-ring ADBMS2950 advisory summary");
        ret |= cli_printline(cli, "bringup charger-lv     - charger CAN low-voltage sniffer checklist");
        ret |= cli_printline(cli, "bringup charger-battery - stricter charger test once battery path is safe");
        ret |= cli_printline(cli, "bringup ready          - BMS_OK release checklist; does not release output");
        ret |= cli_printline(cli, "bringup snapshot       - compact state snapshot");
        ret |= cli_printline(cli, "bringup evidence       - bench evidence to capture before changing phase");
#if (AMS_BUILD_PROFILE == AMS_PROFILE_BENCH_VALIDATION) && !AMS_BENCH_VALIDATION_SINGLE_SMB
        ret |= cli_printline(cli, "bench chain: five SMBs on CS_A/stringA; APM disabled");
#else
        ret |= cli_printline(cli, "bench chain: one SMB through ADBMS6822 eval board on CS_B/stringB; APM disabled");
#endif
        return ret;
    }

    if(!strcmp(mode, "board") || !strcmp(mode, "snapshot"))
    {
        SPI_HandleTypeDef *hspi = smb->hspi;
        current_sensor_t current_snapshot;
        ams_current_window_lock();
        current_snapshot = data->board.current_sensor;
        ams_current_window_unlock();
        const current_sensor_t *cs = &current_snapshot;
        int sensor_high_whole;
        int sensor_high_decimal;
        int sensor_low_whole;
        int sensor_low_decimal;
        bool spi_ok = cli_spi6_mode3_ok(hspi);
        bool current_alive = data->current_valid && cs->last_read_ok &&
                             (fabsf(cs->sensor_voltage_high - 2.5f) <= 0.35f) &&
                             (fabsf(cs->sensor_voltage_low - 2.5f) <= 0.35f) &&
                             (fabsf(data->current) <= 5.0f);

        cli_fixed3(cs->sensor_voltage_high, &sensor_high_whole, &sensor_high_decimal);
        cli_fixed3(cs->sensor_voltage_low, &sensor_low_whole, &sensor_low_decimal);

        snprintf(outline, CLI_LINESZ,
                 "BRINGUP BOARD build:%s state:%s BMS_OK:%d inhibit:%d",
                 AMS_BUILD_PROFILE_NAME,
                 ams_state_to_str(data->state),
                 data->bms_state,
                 data->bms_output_inhibit);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "spi6=%s CPOL:%s CPHA:%s first:%s",
                 cli_passfail(spi_ok),
                 (hspi != NULL) ? cli_spi_polarity_str(hspi->Init.CLKPolarity) : "NULL",
                 (hspi != NULL) ? cli_spi_phase_str(hspi->Init.CLKPhase) : "NULL",
                 ((hspi != NULL) && (hspi->Init.FirstBit == SPI_FIRSTBIT_MSB)) ? "MSB" : "not_MSB");
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "current_zero=%s valid:%d reason:%s adcH:%u adcL:%u sensorH:%d.%03d sensorL:%d.%03d",
                 current_alive ? "PASS" : "WARN",
                 data->current_valid,
                 current_sensor_reason_str(data->current_meas_reason),
                 cs->count_high,
                 cs->count_low,
                 sensor_high_whole,
                 sensor_high_decimal,
                 sensor_low_whole,
                 sensor_low_decimal);
        ret |= cli_printline(cli, outline);

        ret |= cli_printline(cli, "current_zero note: PASS if DHAB is LV-powered; WARN can be OK if harness omits DHAB");

        snprintf(outline, CLI_LINESZ,
                 "no_accumulator voltage:%s temp:%s expected_not_ready_without_cells",
                 data->voltage_valid ? "present" : "not_ready",
                 data->temp_valid ? "present" : "not_ready");
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "heartbeat seen:0x%04X stale:0x%04X safety:0x%04X",
                 data->heartbeat.seen_mask,
                 data->heartbeat.stale_mask,
                 data->heartbeat.safety_stale_mask);
        ret |= cli_printline(cli, outline);

        ret |= cli_printline(cli, "pin_check: run spi pins; use spi cspins both 10 to compare PE4 vs PF4");
        ret |= cli_printline(cli, "scope_check: run spi cs a pulse 10, then spi scope a read 20");

        if(!strcmp(mode, "snapshot"))
        {
            snprintf(outline, CLI_LINESZ,
                     "faults hard:%d voltage:%d temp:%d current:%d charger:%d adbms_diag:%d",
                     data->hard_fault,
                     data->voltage_fault,
                     data->temp_fault,
                     data->current_fault,
                     data->charger_fault,
                     data->adbms_diag_fault);
            ret |= cli_printline(cli, outline);
        }
        return ret;
    }

    if(!strcmp(mode, "adbms6830") || !strcmp(mode, "chain"))
    {
        uint16_t sid_mask = cli_sid_valid_mask(smb, smb_count);
        uint16_t stat_mask = cli_stat_valid_mask(smb, smb_count);
        bool mode_ok = cli_spi6_mode3_ok(smb->hspi);
        bool hal_ok = (smb_dbg != NULL) && (smb_dbg->last_status == HAL_OK) &&
                      (smb_dbg->last_xfer_status == HAL_OK);
        bool pec_ok = (smb_dbg != NULL) &&
                      ((smb_dbg->last_read_pec_fail_mask & expected_mask) == 0u) &&
                      (((smb_dbg->last_read_pec_pass_mask & expected_mask) == expected_mask) ||
                       (smb_dbg->rx_count == 0u));
        bool rx_all_zero = (smb_dbg != NULL) &&
                           cli_preview_all_value(smb_dbg->last_rx_preview,
                                                 ADBMS6830_SPI_DEBUG_PREVIEW_BYTES,
                                                 0x00u);
        bool rx_all_ff = (smb_dbg != NULL) &&
                         cli_preview_all_value(smb_dbg->last_rx_preview,
                                               ADBMS6830_SPI_DEBUG_PREVIEW_BYTES,
                                               0xFFu);

        snprintf(outline, CLI_LINESZ,
                 "BRINGUP ADBMS6830 ic:%u expected_mask:0x%04X scan:%d debug:%d",
                 smb_count,
                 expected_mask,
                 data->adbms_scan_active,
                 (smb_dbg != NULL) ? smb_dbg->enabled : 0);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "startup=%s init:%s timer=%s timer_status:%s",
                 cli_passfail(data->acc.smb_ready && data->acc.delay_timer_ready),
                 cli_hal_status_str(data->acc.smb_init_status),
                 cli_passfail(data->acc.delay_timer_ready),
                 cli_hal_status_str(data->acc.delay_timer_status));
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "mode=%s last_op:%s string:%s status:%s tx:%lu rx:%lu err:%lu",
                 cli_passfail(mode_ok),
                 (smb_dbg != NULL) ? adbms6830_spi_op_str(smb_dbg->last_op) : "none",
                 ((smb_dbg != NULL) && (smb_dbg->last_string == STRING_A)) ? "CS_A" : "CS_B",
                 (smb_dbg != NULL) ? cli_hal_status_str(smb_dbg->last_status) : "NULL",
                 (unsigned long)((smb_dbg != NULL) ? smb_dbg->tx_count : 0u),
                 (unsigned long)((smb_dbg != NULL) ? smb_dbg->rx_count : 0u),
                 (unsigned long)((smb_dbg != NULL) ? smb_dbg->error_count : 0u));
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "response=%s hal=%s pec=%s pass:0x%04X fail:0x%04X cmd_mis:0x%04X",
                 ((smb_dbg == NULL) || (smb_dbg->rx_count == 0u)) ? "NO_READ" :
                     (rx_all_zero ? "FAIL all_zero" : (rx_all_ff ? "FAIL all_ff" : "PASS changing")),
                 cli_passfail(hal_ok),
                 cli_passfail(pec_ok),
                 (smb_dbg != NULL) ? smb_dbg->last_read_pec_pass_mask : 0u,
                 (smb_dbg != NULL) ? smb_dbg->last_read_pec_fail_mask : 0u,
                 (smb_dbg != NULL) ? smb_dbg->cmd_counter_mismatch_mask : 0u);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "sid=%s mask:0x%04X stat=%s mask:0x%04X diag_fault:%d cfg:%d stat:%d ow:%d balance:%d",
                 cli_passfail((sid_mask & expected_mask) == expected_mask),
                 sid_mask,
                 cli_passfail((stat_mask & expected_mask) == expected_mask),
                 stat_mask,
                 data->adbms_diag_fault,
                 data->adbms_config_fault,
                 data->adbms_status_fault,
                 data->adbms_open_wire_fault,
                 data->adbms_balance_write_fault);
        ret |= cli_printline(cli, outline);

        if(smb_health != NULL)
        {
            snprintf(outline, CLI_LINESZ,
                     "health status:%s cfg:0x%04X sticky_pec:0x%04X sticky_cmd:0x%04X",
                     cli_hal_status_str(smb_health->last_status),
                     smb_health->config_mismatch_mask,
                     smb_health->sticky_pec_fail_mask,
                     smb_health->sticky_cmd_counter_mismatch_mask);
            ret |= cli_printline(cli, outline);
        }

        ret |= cli_printline(cli, "next: spi pins -> spi cspins both 10 -> spi cs a pulse 10 -> spi preset normal -> spi scope a read 20");
		ret |= cli_printline(cli, "then: spi probea -> spi sid -> spi stat; use apm sid for the String-B ADBMS2950");
        return ret;
    }

    if(!strcmp(mode, "apm2950") || !strcmp(mode, "apm"))
    {
        const adbms2950_calibration_t *apm_calibration =
            adbms2950_calibration_get(apm);
        bool initialized = data->acc.apm_ready && apm->health.initialized;
        bool rx_all_zero = (apm_dbg != NULL) &&
                           cli_preview_all_value(apm_dbg->last_rx_preview,
                                                 ADBMS2950_SPI_DEBUG_PREVIEW_BYTES,
                                                 0x00u);
        bool rx_all_ff = (apm_dbg != NULL) &&
                         cli_preview_all_value(apm_dbg->last_rx_preview,
                                               ADBMS2950_SPI_DEBUG_PREVIEW_BYTES,
                                               0xFFu);

        snprintf(outline, CLI_LINESZ,
                 "%s APM2950 initialized:%d build_enabled:%d ADVISORY_NON_GATING",
#if AMS_APM_STANDALONE_EVAL_BENCH
                 "STANDALONE_EVAL",
#else
                 "FINAL_RING",
#endif
                 initialized,
                 AMS_ENABLE_APM_2950);
        ret |= cli_printline(cli, outline);

		snprintf(outline, CLI_LINESZ,
		         "init:%s sid:%d cfg:%d dividers:%s",
		         cli_hal_status_str(data->acc.apm_init_status),
		         apm->health.sid_valid,
		         apm->health.config_valid,
		         apm->health.hv_dividers_enabled ? "ON" : "OFF");
		ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "mode=%s op:%s status:%s tx:%lu rx:%lu err:%lu ics:%u",
                 cli_passfail(cli_spi6_mode3_ok(apm->hspi)),
                 (apm_dbg != NULL) ? adbms2950_spi_op_str(apm_dbg->last_op) : "none",
                 (apm_dbg != NULL) ? cli_hal_status_str(apm_dbg->last_status) : "NULL",
                 (unsigned long)((apm_dbg != NULL) ? apm_dbg->tx_count : 0u),
                 (unsigned long)((apm_dbg != NULL) ? apm_dbg->rx_count : 0u),
                 (unsigned long)((apm_dbg != NULL) ? apm_dbg->error_count : 0u),
                 (unsigned)apm->num_ics);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "response=%s pec_pass:0x%04X pec_fail:0x%04X profile:%s scaling=BENCH_UNVALIDATED",
                 ((apm_dbg == NULL) || (apm_dbg->rx_count == 0u)) ? "NO_READ" :
                     (rx_all_zero ? "FAIL all_zero" : (rx_all_ff ? "FAIL all_ff" : "PASS changing")),
                 (apm_dbg != NULL) ? apm_dbg->last_read_pec_pass_mask : 0u,
                 (apm_dbg != NULL) ? apm_dbg->last_read_pec_fail_mask : 0u,
                 (apm_calibration != NULL) ?
                     adbms2950_calibration_profile_str(apm_calibration->profile) :
                     "INVALID");
        ret |= cli_printline(cli, outline);
		ret |= cli_printline(cli, "next EVAL: apm profile eval -> apm status -> apm sid -> apm config -> apm eeprom -> apm sample -> apm redundant; keep HV dividers disabled");
        return ret;
    }

    if(!strcmp(mode, "charger-lv") || !strcmp(mode, "charger"))
    {
        uint16_t v_deci = (uint16_t)roundf(CHARGE_MAX_VOLTAGE * 10.0f);
        uint16_t i_deci = (uint16_t)roundf(CHARGE_MAX_CURRENT * 10.0f);
        uint32_t age_ms = cli_charger_rx_age_ms(ccs);

        ret |= cli_printline(cli, "BRINGUP CHARGER_LV battery_required=NO sniffer_required=YES");
        snprintf(outline, CLI_LINESZ,
                 "protocol tx:0x%08lX rx:0x%08lX BYTE5/data[4] 0=allow 1=disable",
                 (unsigned long)CCS_CANBUS_ID,
                 (unsigned long)CHARGER_RX_ID);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "allow_frame:%02X %02X %02X %02X %02X disable_frame:%02X %02X %02X %02X %02X",
                 (unsigned)((v_deci >> 8) & 0xFFu),
                 (unsigned)(v_deci & 0xFFu),
                 (unsigned)((i_deci >> 8) & 0xFFu),
                 (unsigned)(i_deci & 0xFFu),
                 CHARGER_CMD_ENABLE,
                 (unsigned)((v_deci >> 8) & 0xFFu),
                 (unsigned)(v_deci & 0xFFu),
                 (unsigned)((i_deci >> 8) & 0xFFu),
                 (unsigned)(i_deci & 0xFFu),
                 CHARGER_CMD_DISABLE);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "counts tx:%lu rx:%lu txfail:%lu rx_age:%s%lu hw_fault:%d disable_mask:0x%04X",
                 (unsigned long)ccs->tx_count,
                 (unsigned long)ccs->rx_count,
                 (unsigned long)ccs->tx_fail_count,
                 (age_ms == 0xFFFFFFFFu) ? "never/" : "",
                 (unsigned long)((age_ms == 0xFFFFFFFFu) ? 0u : age_ms),
                 cli_charger_hw_fault(ccs),
                 ccs->disable_reason_mask);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "exit_shutdown pending:%d remaining:%u requests:%lu sent:%lu failed:%lu last:%s",
                 ccs->shutdown_pending,
                 (unsigned)ccs->shutdown_frames_remaining,
                 (unsigned long)ccs->shutdown_request_count,
                 (unsigned long)ccs->shutdown_tx_count,
                 (unsigned long)ccs->shutdown_tx_fail_count,
                 cli_hal_status_str(ccs->last_shutdown_status));
        ret |= cli_printline(cli, outline);

        ret |= cli_printline(cli, "timeout_test=TODO send valid frames, stop them, confirm charger shuts output off near 5s");
        ret |= cli_printline(cli, "exit_test: leave charge -> verify three prioritized 00 00 00 00 01 frames; HAL queue success is not charger acknowledgement");
        ret |= cli_printline(cli, "next: state charge -> sniff 0x1806E5F4 -> charger -> bmsok inhibit -> verify BYTE5/data[4]=01");
        return ret;
    }

    if(!strcmp(mode, "charger-battery"))
    {
        uint32_t age_ms = cli_charger_rx_age_ms(ccs);
        bool rx_fresh = (ccs->last_rx_tick != 0u) &&
                        (age_ms <= CHARGER_RX_TIMEOUT_MS) &&
                        !ccs->communication_fail;
        bool voltage_ready = data->voltage_valid && !data->voltage_fault && !data->voltage_fault_latched;
        bool temp_ready = data->temp_valid && !data->temp_fault && !data->temp_fault_latched && !data->temp_charge_stop;
        bool current_ready = data->current_valid && !data->current_fault && !data->current_fault_latched;
        bool charger_clean = rx_fresh && !cli_charger_hw_fault(ccs) &&
                             (ccs->disable_reason_mask == CHARGER_DISABLE_REASON_NONE);
        bool ready = (data->state == STATE_CHARGE) && data->bms_state &&
                     voltage_ready && temp_ready && current_ready && charger_clean;

        snprintf(outline, CLI_LINESZ,
                 "BRINGUP CHARGER_BATTERY verdict=%s state:%s BMS_OK:%d rx_fresh:%d",
                 cli_passblock(ready),
                 ams_state_to_str(data->state),
                 data->bms_state,
                 rx_fresh);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "gates voltage:%d temp:%d current:%d charger_clean:%d rx_age_ms:%lu mask:0x%04X",
                 voltage_ready,
                 temp_ready,
                 current_ready,
                 charger_clean,
                 (unsigned long)((age_ms == 0xFFFFFFFFu) ? 0u : age_ms),
                 ccs->disable_reason_mask);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli, "requires: LV charger CAN proven, safe battery/accumulator path, contactor/charging procedure approved");
        return ret;
    }

    if(!strcmp(mode, "ready"))
    {
        bool voltage_ready = data->voltage_valid && !data->voltage_fault && !data->voltage_fault_latched;
        bool temp_ready = data->temp_valid && !data->temp_fault && !data->temp_fault_latched &&
                          ((data->state != STATE_CHARGE) || !data->temp_charge_stop);
        bool current_ready = data->current_valid && !data->current_fault && !data->current_fault_latched;
        bool heartbeat_ready = !data->task_heartbeat_fault;
        bool hard_ready = !data->hard_fault && !data->fuse_fault && !data->adbms_diag_fault;
        bool release_ok = voltage_ready && temp_ready && current_ready && heartbeat_ready && hard_ready;

        snprintf(outline, CLI_LINESZ,
                 "BRINGUP READY release_allowed=%s output_inhibit:%d BMS_OK:%d",
                 release_ok ? "YES" : "NO",
                 data->bms_output_inhibit,
                 data->bms_state);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "gates voltage:%d temp:%d current:%d heartbeat:%d hard:%d charger_fault:%d",
                 voltage_ready,
                 temp_ready,
                 current_ready,
                 heartbeat_ready,
                 hard_ready,
                 data->charger_fault);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "counts cells:%u/%u temps:%u/%u charge_stop:%d",
                 (unsigned)data->voltage_usable_cell_count,
                 (unsigned)AMS_EXPECTED_CELL_COUNT,
                 (unsigned)data->temp_usable_sensor_count,
                 (unsigned)AMS_EXPECTED_TEMP_SENSOR_COUNT,
                 data->charge_voltage_stop);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli, "note: this command does not run bmsok release");
        return ret;
    }

    if(!strcmp(mode, "evidence"))
    {
        ret |= cli_printline(cli, "BRINGUP EVIDENCE capture before phase changes:");
        ret |= cli_printline(cli, "1 status; bringup board; bmsok status; fault");
        ret |= cli_printline(cli, "2 spi pins; spi cspins both 10; scope CS_A/PE2 and candidate CS_B PE4/PF4 pins");
        ret |= cli_printline(cli, "3 spi clear; spi enable; spi cs a pulse 10; spi preset normal; spi scope a read 20");
		ret |= cli_printline(cli, "4 spi probea; spi sid; spi stat; apm sid; apm sample");
        ret |= cli_printline(cli, "5 current; volt; temp; bringup ready");
        ret |= cli_printline(cli, "6 charger; bringup charger-lv plus CAN sniffer frame screenshots/logs");
        ret |= cli_printline(cli, "7 for battery/charger only: bringup charger-battery after approved safe setup");
        return ret;
    }

    ret |= cli_printline(cli, "Usage: bringup [help|board|adbms6830|apm2950|charger-lv|charger-battery|ready|snapshot|evidence]");
    return ret;
}

int get_version(int argc, char *argv[])
{
	int ret = 0;
	snprintf(outline, CLI_LINESZ,
	         "v%d.%d.%d profile:%s service_cli:%d hil_can:%d features:0x%04x",
	         VER_MAJOR,
	         VER_MINOR,
	         VER_BUG,
	         AMS_BUILD_PROFILE_NAME,
	         AMS_ENABLE_SERVICE_CLI,
	         AMS_ENABLE_HIL_CAN,
	         (unsigned)AMS_BUILD_FEATURE_FLAGS_VALUE);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ,
	         "manifest schema:%u commit:%s cfg:0x%08lX",
	         (unsigned)AMS_BUILD_MANIFEST_SCHEMA,
	         AMS_BUILD_GIT_COMMIT,
	         (unsigned long)AMS_BUILD_CONFIG_FINGERPRINT);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ,
	         "manifest current:%s CAN:%s",
	         AMS_CURRENT_CALIBRATION_REVISION,
	         AMS_CAN_CONTRACT_REVISION);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ,
	         "manifest thresholds:%s estimator:%s",
	         AMS_THRESHOLD_REVISION,
	         AMS_ESTIMATOR_MODEL_REVISION);
	ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "build date:%s time:%s voltage_mode:%s topology:%u SMB cells:%u string:%u",
             ams_build_manifest.build_date, ams_build_manifest.build_time, cli_voltage_mode_str(),
             (unsigned)NSMBS,
             (unsigned)data->acc.smb.monitored_cell_count,
             (unsigned)data->acc.smb.string);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "diagnostics service_cli:%d adbms_injection:%d auto_temp_scan:%d HIL_replace:%d discharge_timer:%d",
             AMS_ENABLE_SERVICE_CLI,
             AMS_ENABLE_ADBMS_FAULT_INJECTION,
             AMS_ENABLE_AUTO_TEMP_MUX_SCAN,
             AMS_HIL_REPLACE_ADBMS,
             AMS_ENABLE_ADBMS_DISCHARGE_TIMER);
    ret |= cli_printline(cli, outline);
    snprintf(outline, CLI_LINESZ,
             "config fingerprints expected:0x%08lX readback:0x%08lX match:%d",
             (unsigned long)data->adbms_config_expected_fingerprint,
             (unsigned long)data->adbms_config_readback_fingerprint,
             (data->adbms_config_expected_fingerprint != 0u) &&
             (data->adbms_config_expected_fingerprint ==
              data->adbms_config_readback_fingerprint));
    ret |= cli_printline(cli, outline);
	return ret;
}

int bmsok_control(int argc, char *argv[])
{
    int ret = 0;

    if((argc >= 2) && (argv[1] != NULL))
    {
        if(!strcmp(argv[1], "release") || !strcmp(argv[1], "enable"))
        {
#if !AMS_PROFILE_BMS_RUNTIME_AUTHORITY_ALLOWED
            data->bms_output_inhibit = true;
            set_bms(0);
            ret |= cli_printline(cli,
                "BMS_OK release REFUSED: build-profile source lock is immutable");
#elif AMS_ENABLE_SERVICE_CLI
            data->bms_output_inhibit = false;
            ret |= cli_printline(cli, "BMS_OK output release enabled; safety gates still apply");
#else
            ret |= cli_service_action_refused("BMS_OK release");
#endif
        }
        else if(!strcmp(argv[1], "inhibit") || !strcmp(argv[1], "disable"))
        {
            data->bms_output_inhibit = true;
            set_bms(0);
            ret |= cli_printline(cli, "BMS_OK output inhibited and forced low");
        }
        else if(strcmp(argv[1], "status"))
        {
            ret |= cli_printline(cli, "Usage: bmsok [status|release|inhibit]");
            return ret;
        }
    }

    snprintf(outline, CLI_LINESZ,
             "BMS_OK state:%d inhibit:%d supervisor_ready:%d blocked_assertions:%lu",
             data->bms_state,
             data->bms_output_inhibit,
             data->bms_supervisor_ready,
             (unsigned long)data->bms_output_block_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "ready gates voltage:%d current:%d temp:%d imd:%d adbms:%d heartbeat:%d charger:%d hard:%d",
             data->voltage_valid && !data->voltage_fault,
             data->current_valid && !data->current_fault,
             data->temp_valid && !data->temp_fault &&
                 ((data->state != STATE_CHARGE) || !data->temp_charge_stop),
             data->imd_valid && data->imd_ok && !data->imd_fault,
             !data->adbms_diag_fault,
             !data->task_heartbeat_fault,
             !data->charger_fault,
             !data->hard_fault);
    ret |= cli_printline(cli, outline);

    return ret;
}

int balance_control(int argc, char *argv[])
{
    int ret = 0;

    if((argc >= 2) && (argv[1] != NULL))
    {
        if(!strcmp(argv[1], "shadow"))
        {
            uint32_t now = osKernelGetTickCount();
            snprintf(outline, CLI_LINESZ,
                     "balance shadow tick:%lu age:%lu actual_apply:%s temp_valid:%d voltage_valid:%d",
                     (unsigned long)data->adbms_balance_shadow_plan_tick,
                     (unsigned long)((data->adbms_balance_shadow_plan_tick == 0u) ?
                                     0u : (now - data->adbms_balance_shadow_plan_tick)),
                     data->balance_inhibit ? "INHIBITED" : "gated_by_policy",
                     data->temp_valid,
                     data->voltage_valid);
            ret |= cli_printline(cli, outline);
            for(uint8_t seg = 0u; seg < NSMBS; seg++)
            {
                uint16_t mask = data->adbms_balance_shadow_plan[seg];
                uint8_t planned_count = 0u;
                for(uint8_t cell = 0u; cell < data->acc.smb.monitored_cell_count; cell++)
                {
                    if((mask & (uint16_t)(1u << cell)) != 0u)
                    {
                        planned_count++;
                    }
                }
                snprintf(outline, CLI_LINESZ,
                         "SMB%u shadow mask:0x%04X planned_cells:%u",
                         (unsigned)seg,
                         mask,
                         (unsigned)planned_count);
                ret |= cli_printline(cli, outline);
                for(uint8_t cell = 0u; cell < data->acc.smb.monitored_cell_count; cell++)
                {
                    if((mask & (uint16_t)(1u << cell)) != 0u)
                    {
                        snprintf(outline, CLI_LINESZ,
                                 "  SMB%u C%u would_balance:1 hardware_write:0",
                                 (unsigned)seg,
                                 (unsigned)(cell + 1u));
                        ret |= cli_printline(cli, outline);
                    }
                }
            }
            ret |= cli_printline(cli,
                "Shadow mode uses the production selection policy but never writes DCC/PWM bits");
            return ret;
        }
        if(!strcmp(argv[1], "inhibit") || !strcmp(argv[1], "disable"))
        {
            data->balance_inhibit = true;
            int clear_ret = cli_clear_balance_recorded();
            ret |= cli_printline(cli,
                                 (clear_ret == 0) ?
                                 "Balancing inhibited and PWM/DCC cleared" :
                                 "Balancing inhibited; WARNING clear write failed");
        }
        else if(!strcmp(argv[1], "release") || !strcmp(argv[1], "enable"))
        {
#if !AMS_PROFILE_BALANCE_RUNTIME_AUTHORITY_ALLOWED
            data->balance_inhibit = true;
            (void)cli_clear_balance_recorded();
            ret |= cli_printline(cli,
                "Balancing release REFUSED: build-profile source lock is immutable");
#elif AMS_ENABLE_SERVICE_CLI
            data->balance_inhibit = false;
            ret |= cli_printline(cli, "Balancing release enabled; safety gates still apply");
#else
            ret |= cli_service_action_refused("balancing release");
#endif
        }
        else if(!strcmp(argv[1], "clear"))
        {
            int clear_ret = cli_clear_balance_recorded();
            ret |= cli_printline(cli,
                                 (clear_ret == 0) ?
                                 "Balancing PWM/DCC cleared" :
                                 "WARNING balance clear write failed");
        }
        else if(strcmp(argv[1], "status"))
        {
            ret |= cli_printline(cli, "Usage: balance [status|shadow|inhibit|release|clear]");
            return ret;
        }
    }

    snprintf(outline, CLI_LINESZ,
             "balance inhibit:%d state:%s note:%s",
             data->balance_inhibit,
             ams_state_to_str(data->state),
             data->balance_inhibit ? "set for resistor-ladder/bench bring-up" : "charge-state safety gates control PWM");
    ret |= cli_printline(cli, outline);
    return ret;
}

int set_state(int argc, char *argv[])
{
    int ret = 0;

    if(argc == 1)
    {
        snprintf(outline, CLI_LINESZ, "AMS State: %s", ams_state_to_str(data->state));
        ret |= cli_printline(cli, outline);
        snprintf(outline, CLI_LINESZ,
                 "previous:%s reason:%s count:%lu last_tick:%lu transition:%d charger_shutdown:%d/%u",
                 ams_state_to_str(data->state_previous),
                 ams_state_transition_reason_str(data->state_transition_reason),
                 (unsigned long)data->state_transition_count,
                 (unsigned long)data->state_transition_last_tick,
                 data->state_transition_in_progress,
                 data->board.charger.shutdown_pending,
                 (unsigned)data->board.charger.shutdown_frames_remaining);
        ret |= cli_printline(cli, outline);
    }
    else if(argc == 2)
    {
#if !AMS_ENABLE_SERVICE_CLI
        return cli_service_action_refused("AMS state change");
#else
        state_t requested_state;

        if(!strcmp(argv[1], "charge"))
        {
            requested_state = STATE_CHARGE;
        }
        else if(!strcmp(argv[1], "discharge"))
        {
            requested_state = STATE_DISCARGE;
        }
        else
        {
            snprintf(outline, CLI_LINESZ, "ERROR: unrecognized state: %s", argv[1]);
            cli_printline(cli, outline);
            cli_printline(cli, "Usage: state [charge|discharge]");
            return 1;
        }

        uint32_t transition_tick = osKernelGetTickCount();
        ams_state_transition_result_t transition_result;

        /* Publish the new policy and the in-progress gate atomically with the
         * BMS_OK drop.  The gate remains set across the synchronous ADBMS
         * cleanup, so the higher-priority supervisor cannot reassert BMS_OK
         * while bleed outputs are still being cleared. */
        taskENTER_CRITICAL();
        set_bms(false);
        transition_result = ams_state_transition_begin(
            data,
            requested_state,
            AMS_STATE_TRANSITION_SERVICE_COMMAND,
            transition_tick);

        if((transition_result != AMS_STATE_TRANSITION_REJECTED) &&
           (requested_state == STATE_CHARGE))
        {
            /* A CLI state change must not fabricate charger freshness.  Force
             * the charge path to wait for a real charger status frame. */
            data->board.charger.last_rx_tick = 0u;
            data->board.charger.communication_fail = true;
            data->charger_fault = true;
        }
        taskEXIT_CRITICAL();

        /* Waiting for the next periodic ADBMS scan leaves a window where
         * bleed resistors can remain active in the new state. */
        int clear_result = cli_clear_balance_recorded();

        taskENTER_CRITICAL();
        ams_state_transition_finish(data);
        taskEXIT_CRITICAL();

        if(transition_result == AMS_STATE_TRANSITION_REJECTED)
        {
            ret |= cli_printline(cli,
                                 "ERROR state transition rejected; reset required from NULL/ERROR state");
            return 1;
        }

        if(transition_result == AMS_STATE_TRANSITION_APPLIED)
        {
            ams_fault_log_event(AMS_FAULT_LOG_STATE_TRANSITION,
                                (uint16_t)data->state_transition_reason,
                                (((uint32_t)(uint16_t)data->state_previous) << 16u) |
                                    (uint32_t)(uint16_t)data->state,
                                data->state_transition_count);
        }

        if(clear_result != 0)
        {
            ret |= cli_printline(cli, "WARNING state changed with balance-clear write failure; BMS_OK held low");
        }

        snprintf(outline, CLI_LINESZ, "AMS State: %s", ams_state_to_str(data->state));
        ret |= cli_printline(cli, outline);

        /* State changes do not reach into the CAN HAL.  CAN start,
         * notifications and recovery remain owned by the CAN service, which
         * already records and retries transport failures. */
#endif
    }
    else
    {
        cli_printline(cli, "ERROR: too many arguments");
        cli_printline(cli, "Usage: state [charge|discharge]");
        return 1;
    }

    return ret;
}

int cause_fault(int argc, char *argv[])
{
	int ret = 0;

#if AMS_ENABLE_SERVICE_CLI
	set_bms(0);
	ret |= cli_printline(cli, "BMS_OK forced low by service command");
#else
	ret |= cli_service_action_refused("cause_fault");
#endif

	return ret;
};
