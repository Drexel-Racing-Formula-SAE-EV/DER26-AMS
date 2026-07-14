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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ext_drivers/cli.h"
#include "ext_drivers/ams_rtos_diag.h"

/**
* @brief Actual CLI task function
*
* @param arg App_data struct pointer converted to void pointer
*/
void cli_task_fn(void *arg);
int cli_handle_cmd(int argc, char *argv[]);
int cmd_not_found(int argc, char *argv[]);

int help(int argc, char *argv[]);
int get_status(int argc, char *argv[]);
int get_faults(int argc, char *argv[]);
int get_version(int argc, char *argv[]);
int get_voltage(int argc, char *argv[]);
int get_temperature(int argc, char *argv[]);
int get_temperature_sensor(int argc, char *argv[]);
int get_fan_diag(int argc, char *argv[]);


int get_current(int argc, char *argv[]);
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
static adbms_string cli_adbms_scope_default_string = STRING_B;
static adbms6830_scope_mode_t cli_adbms_scope_default_mode = ADBMS6830_SCOPE_READ;
static uint16_t cli_adbms_scope_default_repeat = 20u;
static uint8_t cli_adbms_scope_preset_index = 0u;
static bool cli_parse_scope_repeat(const char *arg, uint16_t *repeat_out);
static int get_temperature_sensor_locked(int argc, char *argv[]);
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
	{"tempsns", &get_temperature_sensor, "gets one sensor: tempsns <ic> <sensor 0-23>"},
	{"fan", &get_fan_diag, "fan control telemetry: fan"},
	{"current", &get_current, "gets current sensor raw counts/voltages/status"},
	{"charger", &get_charger, "gets charger CAN command/status/debug state"},
	{"can", &get_can_diag, "CAN diagnostics: can [diag|recover]"},
	{"wdg", &watchdog_control, "watchdog diagnostics/control: wdg [status|enable]"},
	{"rtos", &get_rtos_diag, "RTOS stack/heap diagnostics: rtos"},
	{"uart", &get_uart_diag, "CLI UART diagnostics/recovery: uart [status|recover|clear]"},
	{"spi", &get_spi_debug, "ADBMS6830 SPI debug: spi [status|pins|cspins|cs|preset|toggle|probe|probea|probeb|scope|sid|stat|staterr|cfgchk|cellst|oweven|owodd|auxdiag|wake|coldwake|clrflag|clear|diagclear|enable|disable]"},
	{"apm", &get_apm_debug, "ADBMS2950/APM debug: apm [status|probe|clear|enable|disable]"},
	{"bringup", &get_bringup, "bench bring-up summaries: bringup [help|board|adbms6830|apm2950|charger-lv|charger-battery|ready|snapshot|evidence]"},
	{"bmsok", &bmsok_control, "BMS_OK control: bmsok [status|release|inhibit]"},
	{"balance", &balance_control, "balancing control: balance [status|inhibit|release|clear]"},
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
    float volt = ((float)raw + 10000.0f) * 0.000150f;

    if((voltage_out == NULL) || (temp_out == NULL))
    {
        return false;
    }

    *voltage_out = volt;
    *temp_out = 0.0f;

    if((volt <= 0.0f) || (volt >= 5.0f))
    {
        return false;
    }

    float resistance = 10000.0f * (5.0f - volt) / volt;
    if(resistance <= 0.0f)
    {
        return false;
    }

    float x = logf(resistance / 10000.0f);
    float denom = 3.354016435e-3f +
                  (2.565235509e-4f * x) +
                  (2.605970121e-6f * x * x) +
                  (6.329261265e-8f * x * x * x);

    if(denom == 0.0f)
    {
        return false;
    }

    float temp = (1.0f / denom) - 273.15f;
    if(!isfinite(temp) || (temp < -40.0f) || (temp > 150.0f))
    {
        return false;
    }

    *temp_out = temp;
    return true;
}

TaskHandle_t cli_task_start(app_data_t *data)
{
    TaskHandle_t handle = NULL;

    if(data == NULL)
    {
        return NULL;
    }

    xTaskCreate(cli_task_fn, "CLI task", AMS_STACK_CLI_WORDS, (void *)data, CLI_PRIO, &handle);
    return handle;
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
             "Build:%s service_cli:%d hil_can:%d APM2950:%d BMS_OK_inhibit:%d",
             AMS_HW_BRINGUP ? "hw-bringup" : "normal",
             AMS_ENABLE_SERVICE_CLI,
             AMS_ENABLE_HIL_CAN,
             AMS_ENABLE_APM_2950_DEBUG,
             data->bms_output_inhibit);
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

    cli_fixed1(data->max_temp, &max_temp_whole, &max_temp_decimal);

	snprintf(outline, CLI_LINESZ,
	             "FW v%d.%d.%d build:%s service:%d hil:%d state:%s BMS_OK:%d inhibit:%d ready:%d balance_inhibit:%d blocked:%lu",
	             VER_MAJOR,
	             VER_MINOR,
	             VER_BUG,
	             AMS_HW_BRINGUP ? "hw-bringup" : "normal",
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
                 "SPI6 CPOL:%s CPHA:%s prescaler:%lu APM2950_debug:%d",
                 cli_spi_polarity_str(hspi->Init.CLKPolarity),
                 cli_spi_phase_str(hspi->Init.CLKPhase),
                 (unsigned long)hspi->Init.BaudRatePrescaler,
                 AMS_ENABLE_APM_2950_DEBUG);
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
            snprintf(outline, CLI_LINESZ, "fault log count:%lu write:%lu",
                     (unsigned long)log->count,
                     (unsigned long)log->write_index);
            ret |= cli_printline(cli, outline);

            for(uint32_t i = 0u; i < log->count; i++)
            {
                uint32_t idx = (log->write_index + AMS_FAULT_LOG_DEPTH - log->count + i) % AMS_FAULT_LOG_DEPTH;
                const ams_fault_log_entry_t *e = &log->entry[idx];
                snprintf(outline, CLI_LINESZ,
                         "%02lu tick:%lu event:%s reason:%u arg0:0x%08lX arg1:0x%08lX",
                         (unsigned long)i,
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

        ret |= cli_printline(cli, "Usage: fault [resetcause|panic|log|log clear]");
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
             "Voltage valid:%d fault:%d warn:%d charge_stop:%d reason:%s",
             data->voltage_valid,
             data->voltage_fault,
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

    /* Poll just the requested sensor through the mux */
//    mux_read_gpio_voltage(smb, sensor);

    if(mux_set_channel(smb, (uint8_t)sensor) != 0)
    {
        ret |= cli_printline(cli, "Error: failed to select temp mux channel");
        return ret;
    }

    adbms6830_us_delay(smb, 2000u);

    adbms6830_wakeup(smb);
    if(mux_read_gpio_voltage(smb, (uint8_t)sensor) != 0)
    {
        ret |= cli_printline(cli, "Error: failed to read temp mux channel");
        return ret;
    }

    adbms6830_us_delay(smb, 2000u);

    /* Read back the raw value and convert to voltage/temp after validation. */
    float volt = 0.0f;
    float T = 0.0f;
    int16_t raw = smb->ics[ic].temp.raw[sensor];

    if(cli_raw_temp_to_values(raw, &volt, &T))
    {
        int whole   = (int)volt;
        int decimal = (int)((volt - (float)whole) * 10000.0f);
        int T_whole   = (int)T;
        int T_decimal = (int)roundf((T - (float)T_whole) * 10.0f);

        snprintf(outline, CLI_LINESZ, "SMB %d | Sensor %d: %d.%04d V, %d.%d C", ic, sensor, whole, decimal, T_whole, T_decimal);
    }
    else
    {
        snprintf(outline, CLI_LINESZ, "SMB %d | Sensor %d: raw %d invalid", ic, sensor, raw);
    }

    ret |= cli_printline(cli, outline);

    return ret;
}



static bool cli_adbms_scan_busy(void)
{
    return (data != NULL) && data->adbms_scan_active;
}

static bool cli_adbms_open_wire_state_allowed(void)
{
    return (data != NULL) &&
           ((data->state == STATE_CHARGE) || (data->state == STATE_BALANCE));
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

static void cli_adbms_scope_apply_preset(uint8_t preset)
{
    cli_adbms_scope_preset_index = (uint8_t)(preset % 4u);

    switch(cli_adbms_scope_preset_index)
    {
    case 0u:
        cli_adbms_scope_default_string = STRING_B;
        cli_adbms_scope_default_mode = ADBMS6830_SCOPE_READ;
        cli_adbms_scope_default_repeat = 20u;
        break;
    case 1u:
        cli_adbms_scope_default_string = STRING_B;
        cli_adbms_scope_default_mode = ADBMS6830_SCOPE_CMD;
        cli_adbms_scope_default_repeat = 50u;
        break;
    case 2u:
        cli_adbms_scope_default_string = STRING_B;
        cli_adbms_scope_default_mode = ADBMS6830_SCOPE_PATTERN;
        cli_adbms_scope_default_repeat = 20u;
        break;
    default:
        cli_adbms_scope_default_string = STRING_A;
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

int get_spi_debug(int argc, char *argv[])
{
    int ret;

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
       strcmp(argv[1], "status") && strcmp(argv[1], "pins"))
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
            else if(!strcmp(argv[2], "normal") || !strcmp(argv[2], "b"))
            {
                cli_adbms_scope_apply_preset(0u);
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
            else if(!strcmp(argv[2], "a"))
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
	            snprintf(outline, CLI_LINESZ, "RDCFGA CS_A/stringA probe status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
	        else if(!strcmp(argv[1], "probeb"))
	        {
	            if(cli_adbms_refuse_active_scan("spi probeb"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_spi_probe_rdcfga_on_string(smb, STRING_B);
	            snprintf(outline, CLI_LINESZ, "RDCFGA CS_B/stringB probe status: %s", cli_hal_status_str(probe_status));
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
	            if(cli_adbms_refuse_active_scan("spi stat"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_read_status(smb, false);
	            snprintf(outline, CLI_LINESZ, "RDSTATC/D/E status: %s", cli_hal_status_str(probe_status));
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
            adbms6830_diag_health_clear(smb);
            ret |= cli_printline(cli, "ADBMS diagnostic health counters cleared");
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
	            snprintf(outline, CLI_LINESZ, "Cell ADC diagnostic hook status: %s", cli_hal_status_str(probe_status));
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
	                ret |= cli_printline(cli, "Open-wire refused: use only in charge/balance service state");
	                return ret;
	            }
	            probe_status = adbms6830_run_open_wire_check(smb, false);
	            snprintf(outline, CLI_LINESZ, "Open-wire even-channel command status: %s", cli_hal_status_str(probe_status));
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
	                ret |= cli_printline(cli, "Open-wire refused: use only in charge/balance service state");
	                return ret;
	            }
	            probe_status = adbms6830_run_open_wire_check(smb, true);
	            snprintf(outline, CLI_LINESZ, "Open-wire odd-channel command status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
	        else if(!strcmp(argv[1], "auxdiag"))
	        {
	            if(cli_adbms_refuse_active_scan("spi auxdiag"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_run_aux_gpio_diagnostic(smb);
	            snprintf(outline, CLI_LINESZ, "AUX/GPIO diagnostic hook status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
	        else if(strcmp(argv[1], "status"))
	        {
	            ret |= cli_printline(cli, "Usage: spi [status|pins|cspins|cs|preset|toggle|probe|probea|probeb|scope|sid|stat|staterr|cfgchk|cellst|oweven|owodd|auxdiag|wake|coldwake|clrflag|clear|diagclear|enable|disable]");
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
	             "scan active:%d count:%lu diag fault:%d cfg:%d stat:%d ow:%d balance:%d balance_fail:%lu last:%s",
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
	             "diag periodic status:%lu cfg:%lu openwire:%lu timer_ready:%d timer_status:%s",
	             (unsigned long)data->adbms_status_diag_count,
	             (unsigned long)data->adbms_config_diag_count,
	             (unsigned long)data->adbms_open_wire_diag_count,
	             data->acc.delay_timer_ready,
	             cli_hal_status_str(data->acc.delay_timer_status));
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
                 "diag PEC last pass:0x%04X fail:0x%04X sticky:0x%04X cmd sticky:0x%04X",
                 health->last_pec_pass_mask,
                 health->last_pec_fail_mask,
                 health->sticky_pec_fail_mask,
                 health->sticky_cmd_counter_mismatch_mask);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "diag counts cfg:%lu cell:%lu owe:%lu owo:%lu aux:%lu",
                 (unsigned long)health->config_readback_count,
                 (unsigned long)health->cell_adc_self_test_count,
                 (unsigned long)health->open_wire_even_count,
                 (unsigned long)health->open_wire_odd_count,
                 (unsigned long)health->aux_gpio_diag_count);
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
                 "IC%u SID:%s %02X%02X%02X%02X%02X%02X STATC:%d csflt:0x%04X sleep:%u spi:%u thsd:%u osc:%u",
                 (unsigned)ic,
                 diag->sid_valid ? "ok" : "--",
                 diag->sid[5], diag->sid[4], diag->sid[3],
                 diag->sid[2], diag->sid[1], diag->sid[0],
                 diag->statc_valid,
                 diag->cs_flt_mask,
                 diag->sleep,
                 diag->spiflt,
                 diag->thsd,
                 diag->oscchk);
        ret |= cli_printline(cli, outline);

        snprintf(outline, CLI_LINESZ,
                 "IC%u STATD:%d ov:0x%04X uv:0x%04X osc_cnt:%u STATE:%d gpi:0x%03X rev:%u",
                 (unsigned)ic,
                 diag->statd_valid,
                 diag->cell_ov_mask,
                 diag->cell_uv_mask,
                 diag->osc_counter,
                 diag->state_valid,
                 diag->gpi_mask,
                 diag->revision);
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

static int get_apm_debug_locked(int argc, char *argv[])
{
    int ret = 0;

#if !AMS_ENABLE_SERVICE_CLI
    if((argc >= 2) && (argv[1] != NULL) && strcmp(argv[1], "status"))
    {
        return cli_service_action_refused("ADBMS2950 service action");
    }
#endif

    adbms2950_driver_t *apm = &data->acc.apm;
    const adbms2950_spi_debug_t *dbg;
    HAL_StatusTypeDef probe_status;
    SPI_HandleTypeDef *hspi = apm->hspi;

    if((apm->hspi == NULL) || (apm->num_ics == 0u))
    {
        ret |= cli_printline(cli, "ADBMS2950/APM not initialized");
        ret |= cli_printline(cli, "Build with AMS_ENABLE_APM_2950_DEBUG=1 for CLI-only APM probing");
        dbg = adbms2950_spi_debug_get(apm);
        if(dbg == NULL)
        {
            return ret;
        }
    }
    else if((argc >= 2) && (argv[1] != NULL))
    {
        if(!strcmp(argv[1], "clear"))
        {
            adbms2950_spi_debug_clear(apm);
            ret |= cli_printline(cli, "ADBMS2950 SPI debug counters cleared");
        }
        else if(!strcmp(argv[1], "enable"))
        {
            adbms2950_spi_debug_enable(apm, true);
            ret |= cli_printline(cli, "ADBMS2950 SPI debug enabled");
        }
        else if(!strcmp(argv[1], "disable"))
        {
            adbms2950_spi_debug_enable(apm, false);
            ret |= cli_printline(cli, "ADBMS2950 SPI debug disabled");
        }
	        else if(!strcmp(argv[1], "probe"))
	        {
	            if(cli_adbms_refuse_active_scan("apm probe"))
	            {
	                return ret;
	            }
	            probe_status = adbms2950_spi_probe_rdcfga(apm);
	            snprintf(outline, CLI_LINESZ, "ADBMS2950 RDCFGA probe status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
        else if(strcmp(argv[1], "status"))
        {
            ret |= cli_printline(cli, "Usage: apm [status|probe|clear|enable|disable]");
            return ret;
        }
    }

    dbg = adbms2950_spi_debug_get(apm);
    if(dbg == NULL)
    {
        ret |= cli_printline(cli, "ADBMS2950 SPI debug unavailable");
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
	             "apm dbg en:%d op:%s string:%u status:%s tx:%lu rx:%lu err:%lu ics:%u",
             dbg->enabled,
             adbms2950_spi_op_str(dbg->last_op),
             (unsigned)dbg->last_string,
             cli_hal_status_str(dbg->last_status),
             (unsigned long)dbg->tx_count,
             (unsigned long)dbg->rx_count,
             (unsigned long)dbg->error_count,
             (unsigned)apm->num_ics);
	    ret |= cli_printline(cli, outline);
	    ret |= cli_printline(cli, "APM safety: debug-only, non-gating until bench CS/SPI/PEC/scaling/shunt polarity are proven");

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
             "cmdcnt IC0:%u IC1:%u IC2:%u IC3:%u",
             dbg->last_cmd_counter[0],
             dbg->last_cmd_counter[1],
             dbg->last_cmd_counter[2],
             dbg->last_cmd_counter[3]);
    ret |= cli_printline(cli, outline);

    ret |= cli_print_hex_preview("APM TX:", dbg->last_tx_preview, ADBMS2950_SPI_DEBUG_PREVIEW_BYTES);
    ret |= cli_print_hex_preview("APM RX:", dbg->last_rx_preview, ADBMS2950_SPI_DEBUG_PREVIEW_BYTES);

    return ret;
}


int get_current(int argc, char *argv[])
{
    int ret = 0;
    int whole = 0;
    int decimal = 0;
    current_sensor_t *cs = &data->board.current_sensor;

    if((argc >= 2) && (argv[1] != NULL) && !strcmp(argv[1], "zero"))
    {
        if((argc >= 3) && (argv[2] != NULL) && !strcmp(argv[2], "clear"))
        {
#if AMS_ENABLE_SERVICE_CLI
            current_sensor_zero_clear(cs);
            ret |= cli_printline(cli, "current zero: cleared");
            return ret;
#else
            return cli_service_action_refused("current zero clear");
#endif
        }

        if((argc >= 3) && (argv[2] != NULL) && !strcmp(argv[2], "status"))
        {
            cli_fixed1(cs->zero_offset_50a, &whole, &decimal);
            snprintf(outline, CLI_LINESZ,
                     "current zero: calibrated:%d captures:%lu offset50:%d.%01d A",
                     cs->zero_calibrated,
                     (unsigned long)cs->zero_cal_count,
                     whole,
                     decimal);
            ret |= cli_printline(cli, outline);

            cli_fixed1(cs->zero_offset_800a, &whole, &decimal);
            snprintf(outline, CLI_LINESZ, "current zero: offset800:%d.%01d A", whole, decimal);
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

        if(!current_sensor_read_adc(cs))
        {
            ret |= cli_printline(cli, "current zero refused: ADC read failed");
            return ret;
        }

        (void)current_sensor_convert(cs);
        if(current_sensor_zero_calibrate(cs))
        {
            (void)current_sensor_convert(cs);
            cli_fixed1(cs->zero_offset_50a, &whole, &decimal);
            snprintf(outline, CLI_LINESZ, "current zero captured: offset50:%d.%01d A", whole, decimal);
            ret |= cli_printline(cli, outline);
            cli_fixed1(cs->zero_offset_800a, &whole, &decimal);
            snprintf(outline, CLI_LINESZ, "current zero captured: offset800:%d.%01d A", whole, decimal);
            ret |= cli_printline(cli, outline);
        }
        else
        {
            snprintf(outline, CLI_LINESZ,
                     "current zero refused: raw50/800 not near zero or sensor invalid reason:%s",
                     current_sensor_reason_str(cs->reason));
            ret |= cli_printline(cli, outline);
        }
        return ret;
    }

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

    cli_fixed3(cs->adc_vref_v, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "Ref adc:%d.%03d V", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed3(cs->sensor_supply_v, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "Ref dhab_supply:%d.%03d V", whole, decimal);
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

    uint16_t rx_queued = canbus_rx_queue_count(&data->board.canbus);
    snprintf(outline, CLI_LINESZ,
             "CAN RX isr:%lu processed:%lu queued:%u high:%u dropped:%lu hal_err:%lu",
             (unsigned long)data->board.canbus.rx_isr_count,
             (unsigned long)data->board.canbus.rx_processed_count,
             (unsigned)rx_queued,
             (unsigned)data->board.canbus.rx_queue_high_water,
             (unsigned long)data->board.canbus.rx_queue_drop_count,
             (unsigned long)data->board.canbus.rx_hal_error_count);
    ret |= cli_printline(cli, outline);

	snprintf(outline, CLI_LINESZ,
			 "CAN init:%s start:%s notify:%s started:%d active:%d",
			 cli_hal_status_str(data->board.canbus.init_status),
			 cli_hal_status_str(data->board.canbus.start_status),
			 cli_hal_status_str(data->board.canbus.notification_status),
			 data->board.canbus.started,
			 data->board.canbus.notification_active);
	ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "CAN fault:%d charger_fault:%d HIL_ADBMS:%d cooldown_ms:%u",
             data->canbus_fault,
             data->charger_fault,
             AMS_HIL_REPLACE_ADBMS,
             AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS);
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
             "RTOS heap free:%lu min:%lu warn<%u fault:%d stack_warn:%d heap_warn:%d",
             (unsigned long)data->rtos_heap_free_bytes,
             (unsigned long)data->rtos_heap_min_ever_free_bytes,
             AMS_RTOS_HEAP_WARN_BYTES,
             data->rtos_fault,
             data->rtos_stack_warning,
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
             "RTOS min stack high-water:%u words warn_mask:0x%04X warn<%u words",
             data->rtos_min_stack_high_water_words,
             data->rtos_stack_warn_mask,
             AMS_RTOS_STACK_WARN_WORDS);
    ret |= cli_printline(cli, outline);

    for(uint8_t i = 0u; i < (uint8_t)AMS_RTOS_TASK_COUNT; i++)
    {
        ams_rtos_task_id_t id = (ams_rtos_task_id_t)i;
        snprintf(outline, CLI_LINESZ,
                 "  %u %-9s prio:%u stack:%u words highwater:%u words",
                 (unsigned)i,
                 ams_rtos_task_name(id),
                 ams_rtos_task_priority(id),
                 data->rtos_stack_config_words[i],
                 data->rtos_stack_high_water_words[i]);
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
        ret |= cli_printline(cli, "bringup apm2950        - ADBMS2950/APM debug-only summary");
        ret |= cli_printline(cli, "bringup charger-lv     - charger CAN low-voltage sniffer checklist");
        ret |= cli_printline(cli, "bringup charger-battery - stricter charger test once battery path is safe");
        ret |= cli_printline(cli, "bringup ready          - BMS_OK release checklist; does not release output");
        ret |= cli_printline(cli, "bringup snapshot       - compact state snapshot");
        ret |= cli_printline(cli, "bringup evidence       - bench evidence to capture before changing phase");
        ret |= cli_printline(cli, "bench ADBMS start: spi pins -> spi cspins both 10 -> spi cs b pulse 10 -> spi scope b read 20");
        return ret;
    }

    if(!strcmp(mode, "board") || !strcmp(mode, "snapshot"))
    {
        SPI_HandleTypeDef *hspi = smb->hspi;
        current_sensor_t *cs = &data->board.current_sensor;
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
                 AMS_HW_BRINGUP ? "hw-bringup" : "normal",
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
        ret |= cli_printline(cli, "scope_check: run spi cs b pulse 10, then spi scope b read 20");

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

        ret |= cli_printline(cli, "next: spi pins -> spi cspins both 10 -> spi cs b pulse 10 -> spi preset normal -> spi scope b read 20");
        ret |= cli_printline(cli, "then: spi probeb -> spi sid -> spi stat; use probea only to validate string-A wiring");
        return ret;
    }

    if(!strcmp(mode, "apm2950") || !strcmp(mode, "apm"))
    {
        bool initialized = (apm->hspi != NULL) && (apm->num_ics > 0u);
        bool rx_all_zero = (apm_dbg != NULL) &&
                           cli_preview_all_value(apm_dbg->last_rx_preview,
                                                 ADBMS2950_SPI_DEBUG_PREVIEW_BYTES,
                                                 0x00u);
        bool rx_all_ff = (apm_dbg != NULL) &&
                         cli_preview_all_value(apm_dbg->last_rx_preview,
                                               ADBMS2950_SPI_DEBUG_PREVIEW_BYTES,
                                               0xFFu);

        snprintf(outline, CLI_LINESZ,
                 "BRINGUP APM2950 initialized:%d build_debug:%d DEBUG_ONLY_NON_GATING",
                 initialized,
                 AMS_ENABLE_APM_2950_DEBUG);
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
                 "response=%s pec_pass:0x%04X pec_fail:0x%04X scaling=UNPROVEN shunt_polarity=UNPROVEN",
                 ((apm_dbg == NULL) || (apm_dbg->rx_count == 0u)) ? "NO_READ" :
                     (rx_all_zero ? "FAIL all_zero" : (rx_all_ff ? "FAIL all_ff" : "PASS changing")),
                 (apm_dbg != NULL) ? apm_dbg->last_read_pec_pass_mask : 0u,
                 (apm_dbg != NULL) ? apm_dbg->last_read_pec_fail_mask : 0u);
        ret |= cli_printline(cli, outline);
        ret |= cli_printline(cli, "next: enable AMS_ENABLE_APM_2950_DEBUG only for intentional APM probing");
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

        ret |= cli_printline(cli, "timeout_test=TODO send valid frames, stop them, confirm charger shuts output off near 5s");
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
        ret |= cli_printline(cli, "2 spi pins; spi cspins both 10; scope PE4 and PF4 candidate CS_B pins");
        ret |= cli_printline(cli, "3 spi clear; spi enable; spi cs b pulse 10; spi preset normal; spi scope b read 20");
        ret |= cli_printline(cli, "4 spi probeb; spi sid; spi stat; bringup adbms6830");
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
	         "v%d.%d.%d %s service_cli:%d hil_can:%d",
	         VER_MAJOR,
	         VER_MINOR,
	         VER_BUG,
	         AMS_HW_BRINGUP ? "hw-bringup" : "normal",
	         AMS_ENABLE_SERVICE_CLI,
	         AMS_ENABLE_HIL_CAN);
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
#if AMS_ENABLE_SERVICE_CLI
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
#if AMS_ENABLE_SERVICE_CLI
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
            ret |= cli_printline(cli, "Usage: balance [status|inhibit|release|clear]");
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

        /* Drop BMS_OK before changing modes and synchronously clear all
         * balance outputs.  Waiting for the next periodic ADBMS scan leaves a
         * window where bleed resistors can remain active in the new state. */
        set_bms(false);
        int clear_result = cli_clear_balance_recorded();
        data->state = requested_state;

        if(requested_state == STATE_CHARGE)
        {
            /* A CLI state change must not fabricate charger freshness.  Force
             * the charge path to wait for a real charger status frame. */
            data->board.charger.last_rx_tick = 0u;
            data->board.charger.communication_fail = true;
            data->charger_fault = true;
        }

        if(clear_result != 0)
        {
            ret |= cli_printline(cli, "WARNING state changed with balance-clear write failure; BMS_OK held low");
        }

        snprintf(outline, CLI_LINESZ, "AMS State: %s", ams_state_to_str(data->state));
        ret |= cli_printline(cli, outline);

        if(data->board.canbus.hcan != NULL)
        {
            HAL_CAN_ActivateNotification(data->board.canbus.hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        }
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
