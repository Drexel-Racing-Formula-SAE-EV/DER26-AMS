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
#include "main.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ext_drivers/cli.h"

#ifndef AMS_RENODE_FAKE_ADBMS
#define AMS_RENODE_FAKE_ADBMS AMS_RENODE
#endif

#ifndef AMS_RENODE_FAKE_CURRENT
#define AMS_RENODE_FAKE_CURRENT AMS_RENODE
#endif

#ifndef AMS_RENODE_FAKE_CHARGER
#define AMS_RENODE_FAKE_CHARGER AMS_RENODE
#endif

#ifndef AMS_RENODE_FAKE_TEMP
#define AMS_RENODE_FAKE_TEMP AMS_RENODE
#endif

#ifndef AMS_RENODE_CAN_CAPTURE
#define AMS_RENODE_CAN_CAPTURE AMS_RENODE
#endif

#if AMS_RENODE
#define AMS_BUILD_MODE_STR "renode"
#elif AMS_HW_BRINGUP
#define AMS_BUILD_MODE_STR "hw-bringup"
#else
#define AMS_BUILD_MODE_STR "normal"
#endif

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


int get_current(int argc, char *argv[]);
int get_charger(int argc, char *argv[]);
int get_spi_debug(int argc, char *argv[]);
int get_apm_debug(int argc, char *argv[]);
int get_adbms_fake(int argc, char *argv[]);
int get_current_fake(int argc, char *argv[]);
int get_charger_fake(int argc, char *argv[]);
int get_temp_fake(int argc, char *argv[]);
int get_canlog(int argc, char *argv[]);
int get_scenario(int argc, char *argv[]);
int get_bringup(int argc, char *argv[]);
int bmsok_control(int argc, char *argv[]);
int set_state(int argc, char *argv[]);
int cause_fault(int argc, char *argv[]);

static void cli_reset_fault_domain(const char *domain);
static void cli_set_state_value(state_t state);

char outline[CLI_LINESZ];
app_data_t *data;
cli_device_t *cli;
command_t cmds[] =
{
	{"help", &help, "print help menu"},
	{"status", &get_status, "compact bring-up status banner"},
	{"fault", &get_faults, "gets the faults of the system"},
	{"ver", &get_version, "gets the firmware version"},
	{"volt", &get_voltage, "gets cell voltages for all SMBs"},
	{"temp", &get_temperature, "gets sensor temperatures for all SMBs"},
	{"tempsns", &get_temperature_sensor, "gets one sensor: tempsns <ic> <sensor 0-23>"},
	{"current", &get_current, "gets current sensor raw counts/voltages/status"},
	{"charger", &get_charger, "gets charger CAN command/status/debug state"},
	{"spi", &get_spi_debug, "ADBMS6830 SPI debug: spi [status|probe|probea|probeb|sid|stat|staterr|cfgchk|cellst|oweven|owodd|auxdiag|wake|coldwake|clrflag|clear|diagclear|enable|disable]"},
	{"apm", &get_apm_debug, "ADBMS2950/APM debug: apm [status|probe|clear|enable|disable]"},
	{"adbmsfake", &get_adbms_fake, "Renode fake ADBMS6830 controls: adbmsfake [status|reset|healthy|all|cell|ov|uv|aux|pec|missing|counter]"},
	{"currentfake", &get_current_fake, "Renode fake DHAB current sensor controls"},
	{"chargerfake", &get_charger_fake, "Renode fake charger CAN controls"},
	{"tempfake", &get_temp_fake, "Renode fake SMB thermistor controls"},
	{"canlog", &get_canlog, "Renode CAN TX capture summary: canlog [status|clear]"},
	{"scenario", &get_scenario, "Renode scenario setup: scenario [healthy|charge-ready|ov|uv|hot|charger-timeout|current-trip]"},
	{"bringup", &get_bringup, "bench bring-up summaries: bringup [help|board|adbms6830|apm2950|charger-lv|charger-battery|ready|snapshot|evidence]"},
	{"bmsok", &bmsok_control, "BMS_OK control: bmsok [status|release|inhibit]"},
	{"state", &set_state, "gets or sets the AMS state [start|charge|discharge|balance|error]"},
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

static void cli_fixed1(float value, int *whole, int *decimal)
{
    int scaled = (int)roundf(value * 10.0f);

    if(whole == NULL || decimal == NULL)
    {
        return;
    }

    *whole = scaled / 10;
    *decimal = abs(scaled % 10);
}

static void cli_fixed3(float value, int *whole, int *decimal)
{
    int scaled = (int)roundf(value * 1000.0f);

    if(whole == NULL || decimal == NULL)
    {
        return;
    }

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

    xTaskCreate(cli_task_fn, "CLI task", 256, (void *)data, CLI_PRIO, &handle);
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
             "Build:%s APM2950:%d BMS_OK_inhibit:%d",
             AMS_BUILD_MODE_STR,
             AMS_ENABLE_APM_2950_DEBUG,
             data->bms_output_inhibit);
    cli_printline(local_cli, outline);
    cli_printline(local_cli, "ADBMS6822 SPI6 expected: mode3 CPOL HIGH CPHA 2EDGE");
	cli_printline(local_cli, "Type 'help' for list of commands");

	for(;;)
	{
		entry = osKernelGetTickCount();
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
    SPI_HandleTypeDef *hspi = data->acc.smb.hspi;

    snprintf(outline, CLI_LINESZ,
             "FW v%d.%d.%d build:%s state:%s BMS_OK:%d inhibit:%d blocked:%lu",
             VER_MAJOR,
             VER_MINOR,
             VER_BUG,
             AMS_BUILD_MODE_STR,
             ams_state_to_str(data->state),
             data->bms_state,
             data->bms_output_inhibit,
             (unsigned long)data->bms_output_block_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Safety current valid:%d fault:%d voltage valid:%d fault:%d temp valid:%d fault:%d hard:%d",
             data->current_valid,
             data->current_fault,
             data->voltage_valid,
             data->voltage_fault,
             data->temp_valid,
             data->temp_fault,
             data->hard_fault);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "Cells usable:%u updated:%u stale:%u fan:%d max_temp:%d.%01dC",
             (unsigned)data->voltage_usable_cell_count,
             (unsigned)data->voltage_updated_cell_count,
             (unsigned)data->voltage_stale_cell_count,
             data->fan_state,
             (int)data->max_temp,
             abs((int)roundf((data->max_temp - (float)((int)data->max_temp)) * 10.0f)));
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

    ret |= cli_printline(cli, "Bring-up order: spi clear -> spi probe -> spi status -> volt -> current -> bmsok release");
    return ret;
}

int get_faults(int argc, char *argv[])
{
	int ret = 0;

    if(argc >= 2)
    {
        if(!strcmp(argv[1], "help"))
        {
            ret |= cli_printline(cli, "Usage: fault [reset-current|reset-voltage|reset-temp|reset-charger|reset-all]");
            return ret;
        }
        if(!strcmp(argv[1], "reset-current") ||
           !strcmp(argv[1], "reset-voltage") ||
           !strcmp(argv[1], "reset-temp") ||
           !strcmp(argv[1], "reset-charger") ||
           !strcmp(argv[1], "reset-all") ||
           !strcmp(argv[1], "current") ||
           !strcmp(argv[1], "voltage") ||
           !strcmp(argv[1], "temp") ||
           !strcmp(argv[1], "charger") ||
           !strcmp(argv[1], "all"))
        {
            cli_reset_fault_domain(argv[1]);
            snprintf(outline, CLI_LINESZ, "Fault reset command applied: %s; BMS_OK forced low", argv[1]);
            ret |= cli_printline(cli, outline);
        }
        else
        {
            ret |= cli_printline(cli, "Usage: fault [reset-current|reset-voltage|reset-temp|reset-charger|reset-all]");
            return ret;
        }
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
        snprintf(outline, CLI_LINESZ, "--- SMB %d usable:0x%04x updated:0x%04x stale:0x%04x pec:0x%04x ---",
                 ic,
                 data->acc.usable_voltage_mask[ic],
                 data->acc.updated_voltage_mask[ic],
                 data->acc.stale_voltage_mask[ic],
                 data->acc.pec_fail_voltage_mask[ic]);
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
             "Temp counts usable:%u updated:%u stale:%u invalid:%u",
             (unsigned)data->temp_usable_sensor_count,
             (unsigned)data->temp_updated_sensor_count,
             (unsigned)data->temp_stale_sensor_count,
             (unsigned)data->temp_invalid_sensor_count);
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

        snprintf(outline, CLI_LINESZ, "--- SMB %d ---", ic);
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

int get_temperature_sensor(int argc, char *argv[])
{
    int ret = 0;
    adbms6830_driver_t *smb = &data->acc.smb;

    /* Validate argument count */
    if (argc != 3)
    {
        ret |= cli_printline(cli, "Usage: tempsns <ic> <sensor 0-23>");
        return ret;
    }

    int ic     = atoi(argv[1]);
    int sensor = atoi(argv[2]);

    /* Validate IC index */
    uint8_t ic_count = smb_ic_count(smb);
    if (ic < 0 || ic >= (int)ic_count)
    {
        snprintf(outline, CLI_LINESZ, "Error: ic must be 0 to %u", (unsigned)((ic_count > 0u) ? (ic_count - 1u) : 0u));
        ret |= cli_printline(cli, outline);
        return ret;
    }

    /* Validate sensor index (0–23 per requirements) */
    if ((sensor < 0) || (sensor >= NTEMPS))
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

static bool cli_refuse_renode_physical_spi(const char *name)
{
#if AMS_RENODE && !AMS_RENODE_FAKE_ADBMS
    snprintf(outline, CLI_LINESZ,
             "%s unavailable in Renode: no physical ADBMS/SPI chain is modeled; use status/bringup summaries",
             (name != NULL) ? name : "SPI command");
    (void)cli_printline(cli, outline);
    return true;
#else
    (void)name;
    return false;
#endif
}

static bool cli_parse_u16_arg(const char *text, uint16_t *out)
{
    char *end = NULL;
    unsigned long value;

    if((text == NULL) || (out == NULL))
    {
        return false;
    }

    value = strtoul(text, &end, 0);
    if((end == text) || ((end != NULL) && (*end != '\0')) || (value > 0xFFFFul))
    {
        return false;
    }

    *out = (uint16_t)value;
    return true;
}

static bool cli_parse_u32_arg(const char *text, uint32_t *out)
{
    char *end = NULL;
    unsigned long value;

    if((text == NULL) || (out == NULL))
    {
        return false;
    }

    value = strtoul(text, &end, 0);
    if((end == text) || ((end != NULL) && (*end != '\0')))
    {
        return false;
    }

    *out = (uint32_t)value;
    return true;
}

static bool cli_parse_i16_arg(const char *text, int16_t *out)
{
    char *end = NULL;
    long value;

    if((text == NULL) || (out == NULL))
    {
        return false;
    }

    value = strtol(text, &end, 0);
    if((end == text) || ((end != NULL) && (*end != '\0')) ||
       (value < INT16_MIN) || (value > INT16_MAX))
    {
        return false;
    }

    *out = (int16_t)value;
    return true;
}

static bool cli_parse_float_arg(const char *text, float *out)
{
    char *end = NULL;
    float value;

    if((text == NULL) || (out == NULL))
    {
        return false;
    }

    value = strtof(text, &end);
    if((end == text) || ((end != NULL) && (*end != '\0')) || !isfinite(value))
    {
        return false;
    }

    *out = value;
    return true;
}

static bool cli_parse_bool_arg(const char *text, bool *out)
{
    if((text == NULL) || (out == NULL))
    {
        return false;
    }

    if(!strcmp(text, "1") || !strcmp(text, "on") || !strcmp(text, "true") || !strcmp(text, "yes"))
    {
        *out = true;
        return true;
    }

    if(!strcmp(text, "0") || !strcmp(text, "off") || !strcmp(text, "false") || !strcmp(text, "no"))
    {
        *out = false;
        return true;
    }

    return false;
}

static bool cli_parse_fake_mask_arg(const char *text, uint16_t *out)
{
    if((text != NULL) && (!strcmp(text, "none") || !strcmp(text, "clear")))
    {
        if(out != NULL)
        {
            *out = 0u;
        }
        return true;
    }

    return cli_parse_u16_arg(text, out);
}

static void cli_publish_voltage_fault_state(void)
{
    voltage_fault_state_t *fault = &data->voltage_fault_state;

    data->voltage_valid = fault->voltage_valid;
    data->voltage_read_fault = fault->read_fault;
    data->voltage_warning = fault->warning;
    data->charge_voltage_stop = fault->charge_stop;
    data->overvoltage_fault = fault->overvoltage_fault;
    data->undervoltage_fault = fault->undervoltage_fault;
    data->voltage_fault_latched = fault->latched;
    data->voltage_fault_reason = fault->reason;
    data->voltage_fault_latched_reason = fault->latched_reason;
    data->voltage_fault = (fault->read_fault ||
                           fault->overvoltage_fault ||
                           fault->undervoltage_fault ||
                           fault->latched);
}

static void cli_publish_temperature_fault_state(void)
{
    temperature_fault_state_t *fault = &data->temp_fault_state;

    data->temp_valid = fault->temp_valid;
    data->temp_read_fault = fault->read_fault;
    data->temp_warning = fault->warning;
    data->temp_fan_max = fault->fan_max;
    data->temp_charge_stop = fault->charge_stop;
    data->temp_overtemp_pending = fault->pending;
    data->overtemp_fault = fault->overtemp_fault;
    data->severe_overtemp_fault = fault->severe_overtemp_fault;
    data->temp_fault_latched = fault->latched;
    data->temp_fault_reason = fault->reason;
    data->temp_fault_pending_reason = fault->pending_reason;
    data->temp_fault_latched_reason = fault->latched_reason;
    data->temp_fault_pending_ms = fault->pending_ms;
    data->temp_fault = (fault->read_fault ||
                        fault->overtemp_fault ||
                        fault->latched);
}

static void cli_publish_current_fault_state(void)
{
    current_fault_state_t *fault = &data->current_fault_state;

    data->current_sensor_fault = fault->sensor_fault;
    data->current_overcurrent_warning = fault->warning;
    data->current_overcurrent_pending = fault->pending;
    data->current_overcurrent_fault = fault->confirmed;
    data->current_fault_latched = fault->latched;
    data->current_fault_reason = fault->reason;
    data->current_fault_latched_reason = fault->latched_reason;
    data->current_fault_mode = fault->mode;
    data->current_fault = (data->current_sensor_fault ||
                           data->current_overcurrent_fault ||
                           data->current_fault_latched);
}

static void cli_reset_fault_domain(const char *domain)
{
    if((domain == NULL) || !strcmp(domain, "reset-all") || !strcmp(domain, "all"))
    {
        voltage_fault_reset_latch(&data->voltage_fault_state);
        temperature_fault_reset_latch(&data->temp_fault_state);
        current_fault_reset_latch(&data->current_fault_state);
        data->hard_fault = false;
        data->soft_fault = false;
        data->charger_fault = false;
        data->board.charger.tx_fail = false;
        data->board.charger.communication_fail = false;
        data->board.charger.disable_reason_mask = CHARGER_DISABLE_REASON_NONE;
    }
    else if(!strcmp(domain, "reset-current") || !strcmp(domain, "current"))
    {
        current_fault_reset_latch(&data->current_fault_state);
    }
    else if(!strcmp(domain, "reset-voltage") || !strcmp(domain, "voltage"))
    {
        voltage_fault_reset_latch(&data->voltage_fault_state);
    }
    else if(!strcmp(domain, "reset-temp") || !strcmp(domain, "temp"))
    {
        temperature_fault_reset_latch(&data->temp_fault_state);
    }
    else if(!strcmp(domain, "reset-charger") || !strcmp(domain, "charger"))
    {
        data->charger_fault = false;
        data->board.charger.tx_fail = false;
        data->board.charger.communication_fail = false;
        data->board.charger.disable_reason_mask = CHARGER_DISABLE_REASON_NONE;
    }

    cli_publish_voltage_fault_state();
    cli_publish_temperature_fault_state();
    cli_publish_current_fault_state();
    set_bms(0);
}

static int cli_print_adbms_fake_status(void)
{
    adbms6830_fake_status_t status = adbms6830_fake_get_status();
    int ret = 0;

    snprintf(outline, CLI_LINESZ,
             "ADBMSFAKE enabled:%d initialized:%d model_ics:%u zero_based_indices",
             status.enabled,
             status.initialized,
             status.ic_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "fault_masks pec:0x%04X missing:0x%04X counter:0x%04X",
             status.pec_fail_mask,
             status.missing_mask,
             status.counter_fault_mask);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "cells min:%umV ic:%u cell:%u max:%umV ic:%u cell:%u",
             status.min_cell_mv,
             status.min_cell_ic,
             status.min_cell_index,
             status.max_cell_mv,
             status.max_cell_ic,
             status.max_cell_index);
    ret |= cli_printline(cli, outline);

    return ret;
}

int get_adbms_fake(int argc, char *argv[])
{
    int ret = 0;
    uint16_t a = 0u;
    uint16_t b = 0u;
    uint16_t c = 0u;
    int16_t raw = 0;

    if(!adbms6830_fake_enabled())
    {
        ret |= cli_printline(cli, "adbmsfake unavailable: build with AMS_RENODE_FAKE_ADBMS=1");
        return ret;
    }

    if((argc < 2) || (argv[1] == NULL) || !strcmp(argv[1], "status"))
    {
        return cli_print_adbms_fake_status();
    }

    if(!strcmp(argv[1], "help"))
    {
        ret |= cli_printline(cli, "Usage:");
        ret |= cli_printline(cli, "  adbmsfake status");
        ret |= cli_printline(cli, "  adbmsfake reset|healthy");
        ret |= cli_printline(cli, "  adbmsfake all <mv>");
        ret |= cli_printline(cli, "  adbmsfake cell <ic 0-15> <cell 0-14> <mv>");
        ret |= cli_printline(cli, "  adbmsfake ov|uv <ic 0-15> <cell 0-14>");
        ret |= cli_printline(cli, "  adbmsfake aux <ic 0-15> <gpio 0-2> <raw>");
        ret |= cli_printline(cli, "  adbmsfake pec|missing|counter <mask|none>");
        ret |= cli_printline(cli, "Then observe through: spi probe/sid/stat/cfgchk, volt, temp, fault, bringup adbms6830");
        return ret;
    }

    if(!strcmp(argv[1], "reset") || !strcmp(argv[1], "healthy"))
    {
        adbms6830_fake_reset();
        ret |= cli_printline(cli, "ADBMS fake reset to healthy nominal 3.70V cells and valid PEC/counters");
        ret |= cli_print_adbms_fake_status();
        return ret;
    }

    if(!strcmp(argv[1], "all"))
    {
        if((argc < 3) || !cli_parse_u16_arg(argv[2], &a) ||
           (adbms6830_fake_set_all_cells_mv(a) != 0))
        {
            ret |= cli_printline(cli, "Usage: adbmsfake all <mv 1000-5000>");
            return ret;
        }
        snprintf(outline, CLI_LINESZ, "ADBMS fake all cells set to %umV", a);
        ret |= cli_printline(cli, outline);
        return ret;
    }

    if(!strcmp(argv[1], "cell"))
    {
        if((argc < 5) ||
           !cli_parse_u16_arg(argv[2], &a) ||
           !cli_parse_u16_arg(argv[3], &b) ||
           !cli_parse_u16_arg(argv[4], &c) ||
           (adbms6830_fake_set_cell_mv((uint8_t)a, (uint8_t)b, c) != 0))
        {
            ret |= cli_printline(cli, "Usage: adbmsfake cell <ic 0-15> <cell 0-14> <mv 1000-5000>");
            return ret;
        }
        snprintf(outline, CLI_LINESZ, "ADBMS fake ic:%u cell:%u set to %umV", a, b, c);
        ret |= cli_printline(cli, outline);
        return ret;
    }

    if(!strcmp(argv[1], "ov") || !strcmp(argv[1], "uv"))
    {
        uint16_t mv = !strcmp(argv[1], "ov") ? 4250u : 2300u;

        if((argc < 4) ||
           !cli_parse_u16_arg(argv[2], &a) ||
           !cli_parse_u16_arg(argv[3], &b) ||
           (adbms6830_fake_set_cell_mv((uint8_t)a, (uint8_t)b, mv) != 0))
        {
            ret |= cli_printline(cli, "Usage: adbmsfake ov|uv <ic 0-15> <cell 0-14>");
            return ret;
        }
        snprintf(outline, CLI_LINESZ, "ADBMS fake ic:%u cell:%u set to %umV %s injection",
                 a, b, mv, !strcmp(argv[1], "ov") ? "OV" : "UV");
        ret |= cli_printline(cli, outline);
        return ret;
    }

    if(!strcmp(argv[1], "aux"))
    {
        if((argc < 5) ||
           !cli_parse_u16_arg(argv[2], &a) ||
           !cli_parse_u16_arg(argv[3], &b) ||
           !cli_parse_i16_arg(argv[4], &raw) ||
           (adbms6830_fake_set_aux_raw((uint8_t)a, (uint8_t)b, raw) != 0))
        {
            ret |= cli_printline(cli, "Usage: adbmsfake aux <ic 0-15> <gpio 0-2> <raw -32768..32767>");
            return ret;
        }
        snprintf(outline, CLI_LINESZ, "ADBMS fake ic:%u gpio:%u AUX raw set to %d", a, b, raw);
        ret |= cli_printline(cli, outline);
        return ret;
    }

    if(!strcmp(argv[1], "pec") || !strcmp(argv[1], "missing") || !strcmp(argv[1], "counter"))
    {
        if((argc < 3) || !cli_parse_fake_mask_arg(argv[2], &a))
        {
            ret |= cli_printline(cli, "Usage: adbmsfake pec|missing|counter <mask|none>");
            return ret;
        }

        if(!strcmp(argv[1], "pec"))
        {
            adbms6830_fake_set_pec_fail_mask(a);
        }
        else if(!strcmp(argv[1], "missing"))
        {
            adbms6830_fake_set_missing_mask(a);
        }
        else
        {
            adbms6830_fake_set_counter_fault_mask(a);
        }

        snprintf(outline, CLI_LINESZ, "ADBMS fake %s mask set to 0x%04X", argv[1], a);
        ret |= cli_printline(cli, outline);
        ret |= cli_print_adbms_fake_status();
        return ret;
    }

    ret |= cli_printline(cli, "Usage: adbmsfake [help|status|reset|healthy|all|cell|ov|uv|aux|pec|missing|counter]");
    return ret;
}

static int cli_print_current_fake_status(void)
{
    current_sensor_fake_status_t status = current_sensor_fake_get_status();
    int ret = 0;
    int whole = 0;
    int decimal = 0;

    snprintf(outline, CLI_LINESZ,
             "CURRENTFAKE enabled:%d adc_fail:%d raw_override:%d reads:%lu",
             status.enabled,
             status.adc_fail,
             status.raw_override,
             (unsigned long)status.read_count);
    ret |= cli_printline(cli, outline);

    cli_fixed1(status.requested_current_a, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "requested:%d.%01dA positive=discharge negative=charge/regen",
             whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(status.channel_50a_current_a, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "model 50A:%d.%01dA adcL:%u", whole, decimal, status.count_low);
    ret |= cli_printline(cli, outline);

    cli_fixed1(status.channel_800a_current_a, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "model 800A:%d.%01dA adcH:%u", whole, decimal, status.count_high);
    ret |= cli_printline(cli, outline);

    ret |= cli_printline(cli, "Observe firmware acceptance with: current, fault, status, bringup board");
    return ret;
}

int get_current_fake(int argc, char *argv[])
{
    int ret = 0;
    float a = 0.0f;
    float b = 0.0f;
    uint16_t raw_low = 0u;
    uint16_t raw_high = 0u;
    int whole = 0;
    int decimal = 0;
    bool on = false;

    if(!current_sensor_fake_enabled())
    {
        ret |= cli_printline(cli, "currentfake unavailable: build with AMS_RENODE_FAKE_CURRENT=1");
        return ret;
    }

    if((argc < 2) || (argv[1] == NULL) || !strcmp(argv[1], "status"))
    {
        return cli_print_current_fake_status();
    }

    if(!strcmp(argv[1], "help"))
    {
        ret |= cli_printline(cli, "Usage:");
        ret |= cli_printline(cli, "  currentfake status");
        ret |= cli_printline(cli, "  currentfake reset|zero");
        ret |= cli_printline(cli, "  currentfake amps <signed_A>       positive=discharge, negative=charge/regen");
        ret |= cli_printline(cli, "  currentfake charge <positive_A>   shortcut for amps -A");
        ret |= cli_printline(cli, "  currentfake mismatch <50A_A> <800A_A>");
        ret |= cli_printline(cli, "  currentfake raw <low_adc> <high_adc>");
        ret |= cli_printline(cli, "  currentfake clearraw");
        ret |= cli_printline(cli, "  currentfake rail <low|high>");
        ret |= cli_printline(cli, "  currentfake fail <on|off>");
        return ret;
    }

    if(!strcmp(argv[1], "reset") || !strcmp(argv[1], "zero"))
    {
        current_sensor_fake_reset();
        (void)current_sensor_fake_set_current_a(0.0f);
        ret |= cli_printline(cli, "Current fake reset to DHAB mid-scale zero-current");
        return cli_print_current_fake_status();
    }

    if(!strcmp(argv[1], "amps"))
    {
        if((argc < 3) || !cli_parse_float_arg(argv[2], &a) ||
           (current_sensor_fake_set_current_a(a) != 0))
        {
            ret |= cli_printline(cli, "Usage: currentfake amps <signed_A -900..900>");
            return ret;
        }
        cli_fixed1(a, &whole, &decimal);
        snprintf(outline, CLI_LINESZ, "Current fake set to %d.%01dA", whole, decimal);
        ret |= cli_printline(cli, outline);
        return cli_print_current_fake_status();
    }

    if(!strcmp(argv[1], "charge"))
    {
        if((argc < 3) || !cli_parse_float_arg(argv[2], &a) || (a < 0.0f) ||
           (current_sensor_fake_set_current_a(-a) != 0))
        {
            ret |= cli_printline(cli, "Usage: currentfake charge <positive_A>");
            return ret;
        }
        cli_fixed1(-a, &whole, &decimal);
        snprintf(outline, CLI_LINESZ, "Current fake set to charge current %d.%01dA", whole, decimal);
        ret |= cli_printline(cli, outline);
        return cli_print_current_fake_status();
    }

    if(!strcmp(argv[1], "mismatch"))
    {
        if((argc < 4) || !cli_parse_float_arg(argv[2], &a) || !cli_parse_float_arg(argv[3], &b) ||
           (current_sensor_fake_set_channel_currents_a(a, b) != 0))
        {
            ret |= cli_printline(cli, "Usage: currentfake mismatch <50A_channel_A> <800A_channel_A>");
            return ret;
        }
        ret |= cli_printline(cli, "Current fake channel mismatch injected");
        return cli_print_current_fake_status();
    }

    if(!strcmp(argv[1], "raw"))
    {
        if((argc < 4) || !cli_parse_u16_arg(argv[2], &raw_low) ||
           !cli_parse_u16_arg(argv[3], &raw_high) ||
           (current_sensor_fake_set_raw_counts(raw_low, raw_high) != 0))
        {
            ret |= cli_printline(cli, "Usage: currentfake raw <low_adc 0-4095> <high_adc 0-4095>");
            return ret;
        }
        ret |= cli_printline(cli, "Current fake raw ADC override set");
        return cli_print_current_fake_status();
    }

    if(!strcmp(argv[1], "clearraw"))
    {
        current_sensor_fake_clear_raw_override();
        ret |= cli_printline(cli, "Current fake raw ADC override cleared");
        return cli_print_current_fake_status();
    }

    if(!strcmp(argv[1], "rail"))
    {
        if((argc < 3) || (argv[2] == NULL))
        {
            ret |= cli_printline(cli, "Usage: currentfake rail <low|high>");
            return ret;
        }
        if(!strcmp(argv[2], "low"))
        {
            (void)current_sensor_fake_set_raw_counts(0u, 0u);
        }
        else if(!strcmp(argv[2], "high"))
        {
            (void)current_sensor_fake_set_raw_counts(4095u, 4095u);
        }
        else
        {
            ret |= cli_printline(cli, "Usage: currentfake rail <low|high>");
            return ret;
        }
        ret |= cli_printline(cli, "Current fake ADC rail fault injected");
        return cli_print_current_fake_status();
    }

    if(!strcmp(argv[1], "fail"))
    {
        if((argc < 3) || !cli_parse_bool_arg(argv[2], &on))
        {
            ret |= cli_printline(cli, "Usage: currentfake fail <on|off>");
            return ret;
        }
        current_sensor_fake_set_adc_fail(on);
        snprintf(outline, CLI_LINESZ, "Current fake ADC failure %s", on ? "enabled" : "disabled");
        ret |= cli_printline(cli, outline);
        return cli_print_current_fake_status();
    }

    ret |= cli_printline(cli, "Usage: currentfake [help|status|reset|zero|amps|charge|mismatch|raw|clearraw|rail|fail]");
    return ret;
}

static int cli_print_charger_fake_status(void)
{
    charger_fake_status_t status = charger_fake_get_status();
    int ret = 0;
    int whole = 0;
    int decimal = 0;

    snprintf(outline, CLI_LINESZ,
             "CHARGERFAKE enabled:%d online:%d auto_reply:%d tx_fail:%d hold_timeout:%d flags:0x%02X",
             status.enabled,
             status.online,
             status.auto_reply,
             status.force_tx_fail,
             status.hold_timeout,
             status.flags);
    ret |= cli_printline(cli, outline);

    cli_fixed1(status.command_voltage, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "last command V:%d.%01d", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(status.command_current, &whole, &decimal);
    snprintf(outline, CLI_LINESZ,
             "last command I:%d.%01d BYTE5/data[4]:%u allow:%u disable:%u",
             whole,
             decimal,
             status.last_control,
             CHARGER_CMD_ENABLE,
             CHARGER_CMD_DISABLE);
    ret |= cli_printline(cli, outline);

    cli_fixed1(status.readback_voltage, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "manual readback V:%d.%01d", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(status.readback_current, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "manual readback I:%d.%01d", whole, decimal);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "counts cmd:%lu reply:%lu fake_txfail:%lu payload:%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned long)status.command_count,
             (unsigned long)status.reply_count,
             (unsigned long)status.tx_fail_count,
             status.last_payload[0],
             status.last_payload[1],
             status.last_payload[2],
             status.last_payload[3],
             status.last_payload[4],
             status.last_payload[5],
             status.last_payload[6],
             status.last_payload[7]);
    ret |= cli_printline(cli, outline);
    ret |= cli_printline(cli, "Observe firmware state with: charger, fault, status, bringup charger-lv");
    return ret;
}

int get_charger_fake(int argc, char *argv[])
{
    int ret = 0;
    bool on = false;
    uint16_t flags = 0u;
    float voltage = 0.0f;
    float current = 0.0f;

    if(!charger_fake_enabled())
    {
        ret |= cli_printline(cli, "chargerfake unavailable: build with AMS_RENODE_FAKE_CHARGER=1");
        return ret;
    }

    if((argc < 2) || (argv[1] == NULL) || !strcmp(argv[1], "status"))
    {
        return cli_print_charger_fake_status();
    }

    if(!strcmp(argv[1], "help"))
    {
        ret |= cli_printline(cli, "Usage:");
        ret |= cli_printline(cli, "  chargerfake status");
        ret |= cli_printline(cli, "  chargerfake reset|healthy");
        ret |= cli_printline(cli, "  chargerfake online <on|off>");
        ret |= cli_printline(cli, "  chargerfake autoreply <on|off>");
        ret |= cli_printline(cli, "  chargerfake txfail <on|off>");
        ret |= cli_printline(cli, "  chargerfake timeout <on|off>");
        ret |= cli_printline(cli, "  chargerfake flags <0x00-0x0F>");
        ret |= cli_printline(cli, "  chargerfake readback <voltage_V> <current_A>");
        ret |= cli_printline(cli, "  chargerfake rxgood");
        return ret;
    }

    if(!strcmp(argv[1], "reset") || !strcmp(argv[1], "healthy"))
    {
        charger_fake_reset();
        data->board.charger.flags = 0u;
        data->board.charger.communication_fail = false;
        ret |= cli_printline(cli, "Charger fake reset to online healthy auto-reply");
        return cli_print_charger_fake_status();
    }

    if(!strcmp(argv[1], "online") || !strcmp(argv[1], "autoreply") ||
       !strcmp(argv[1], "txfail") || !strcmp(argv[1], "timeout"))
    {
        if((argc < 3) || !cli_parse_bool_arg(argv[2], &on))
        {
            ret |= cli_printline(cli, "Usage: chargerfake online|autoreply|txfail|timeout <on|off>");
            return ret;
        }

        if(!strcmp(argv[1], "online"))
        {
            charger_fake_set_online(on);
        }
        else if(!strcmp(argv[1], "autoreply"))
        {
            charger_fake_set_auto_reply(on);
        }
        else if(!strcmp(argv[1], "txfail"))
        {
            charger_fake_set_tx_fail(on);
        }
        else
        {
            charger_fake_set_timeout(on);
            if(on)
            {
                data->board.charger.last_rx_tick = osKernelGetTickCount() - CHARGER_RX_TIMEOUT_MS - 1u;
                data->board.charger.communication_fail = true;
            }
            else
            {
                data->board.charger.communication_fail = false;
                data->board.charger.last_rx_tick = osKernelGetTickCount();
                charger_fake_set_online(true);
            }
        }

        snprintf(outline, CLI_LINESZ, "Charger fake %s set to %s", argv[1], on ? "on" : "off");
        ret |= cli_printline(cli, outline);
        return cli_print_charger_fake_status();
    }

    if(!strcmp(argv[1], "flags"))
    {
        if((argc < 3) || !cli_parse_u16_arg(argv[2], &flags) || (flags > 0xFFu))
        {
            ret |= cli_printline(cli, "Usage: chargerfake flags <0x00-0xFF>");
            return ret;
        }
        charger_fake_set_flags((uint8_t)flags);
        snprintf(outline, CLI_LINESZ, "Charger fake flags set to 0x%02X", flags);
        ret |= cli_printline(cli, outline);
        return cli_print_charger_fake_status();
    }

    if(!strcmp(argv[1], "readback"))
    {
        if((argc < 4) || !cli_parse_float_arg(argv[2], &voltage) ||
           !cli_parse_float_arg(argv[3], &current) ||
           (charger_fake_set_readback(voltage, current) != 0))
        {
            ret |= cli_printline(cli, "Usage: chargerfake readback <voltage_V 0-500> <current_A 0-100>");
            return ret;
        }
        ret |= cli_printline(cli, "Charger fake manual readback set");
        return cli_print_charger_fake_status();
    }

    if(!strcmp(argv[1], "rxgood"))
    {
        charger_fake_set_online(true);
        charger_fake_set_timeout(false);
        charger_fake_set_flags(0u);
        (void)charger_fake_set_readback(CHARGE_MAX_VOLTAGE, CHARGE_MAX_CURRENT);
        data->board.charger.communication_fail = false;
        data->board.charger.last_rx_tick = osKernelGetTickCount();
        ret |= cli_printline(cli, "Charger fake prepared for healthy RX on next charge command");
        return cli_print_charger_fake_status();
    }

    ret |= cli_printline(cli, "Usage: chargerfake [help|status|reset|healthy|online|autoreply|txfail|timeout|flags|readback|rxgood]");
    return ret;
}

static void cli_refresh_temperature_from_accumulator(void)
{
    accumulator_update_temp_stats_at(&data->acc, osKernelGetTickCount());
    data->max_temp = data->acc.max_temp;
    data->avg_temp = data->acc.avg_temp;
    temperature_fault_update_with_period(&data->temp_fault_state,
                                         &data->acc,
                                         (1000u / ADBMS_FREQ));
    cli_publish_temperature_fault_state();
}

static int cli_print_temp_fake_status(void)
{
    accumulator_temp_fake_status_t status = accumulator_temp_fake_get_status(&data->acc);
    int ret = 0;
    int whole = 0;
    int decimal = 0;

    snprintf(outline, CLI_LINESZ,
             "TEMPFAKE enabled:%d initialized:%d hold_missing:%d applies:%lu",
             status.enabled,
             status.initialized,
             status.hold_missing,
             (unsigned long)status.apply_count);
    ret |= cli_printline(cli, outline);

    cli_fixed1(status.min_temp_c, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "min:%d.%01dC SMB%u/S%u",
             whole, decimal, status.min_seg, status.min_sensor);
    ret |= cli_printline(cli, outline);

    cli_fixed1(status.max_temp_c, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "max:%d.%01dC SMB%u/S%u",
             whole, decimal, status.max_seg, status.max_sensor);
    ret |= cli_printline(cli, outline);

    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        snprintf(outline, CLI_LINESZ,
                 "SMB%u missing:0x%06lX invalid:0x%06lX",
                 seg,
                 (unsigned long)status.missing_mask[seg],
                 (unsigned long)status.invalid_mask[seg]);
        ret |= cli_printline(cli, outline);
    }

    ret |= cli_printline(cli, "Observe firmware acceptance with: temp, fault, status, bringup ready");
    return ret;
}

int get_temp_fake(int argc, char *argv[])
{
    int ret = 0;
    float temp_c = 0.0f;
    uint16_t seg = 0u;
    uint16_t sensor = 0u;
    uint32_t mask = 0u;
    bool on = false;

    if(!accumulator_temp_fake_enabled())
    {
        ret |= cli_printline(cli, "tempfake unavailable: build with AMS_RENODE_FAKE_TEMP=1");
        return ret;
    }

    if((argc < 2) || (argv[1] == NULL) || !strcmp(argv[1], "status"))
    {
        return cli_print_temp_fake_status();
    }

    if(!strcmp(argv[1], "help"))
    {
        ret |= cli_printline(cli, "Usage:");
        ret |= cli_printline(cli, "  tempfake status");
        ret |= cli_printline(cli, "  tempfake reset|healthy");
        ret |= cli_printline(cli, "  tempfake all <C>");
        ret |= cli_printline(cli, "  tempfake sensor <smb 0-4> <sensor 0-23> <C>");
        ret |= cli_printline(cli, "  tempfake hot <smb> <sensor> [C=65]");
        ret |= cli_printline(cli, "  tempfake cold <smb> <sensor> [C=-5]");
        ret |= cli_printline(cli, "  tempfake missing <smb> <mask24|none>");
        ret |= cli_printline(cli, "  tempfake invalid <smb> <mask24|none>");
        ret |= cli_printline(cli, "  tempfake holdmissing <on|off>");
        return ret;
    }

    if(!strcmp(argv[1], "reset") || !strcmp(argv[1], "healthy"))
    {
        accumulator_temp_fake_reset(&data->acc);
        cli_refresh_temperature_from_accumulator();
        ret |= cli_printline(cli, "Temp fake reset to full healthy 25C SMB thermistors");
        return cli_print_temp_fake_status();
    }

    if(!strcmp(argv[1], "all"))
    {
        if((argc < 3) || !cli_parse_float_arg(argv[2], &temp_c) ||
           (accumulator_temp_fake_set_all(&data->acc, temp_c) != 0))
        {
            ret |= cli_printline(cli, "Usage: tempfake all <C -40..150>");
            return ret;
        }
        cli_refresh_temperature_from_accumulator();
        ret |= cli_printline(cli, "Temp fake all sensors updated");
        return cli_print_temp_fake_status();
    }

    if(!strcmp(argv[1], "sensor") || !strcmp(argv[1], "hot") || !strcmp(argv[1], "cold"))
    {
        if((argc < 4) ||
           !cli_parse_u16_arg(argv[2], &seg) ||
           !cli_parse_u16_arg(argv[3], &sensor))
        {
            ret |= cli_printline(cli, "Usage: tempfake sensor|hot|cold <smb 0-4> <sensor 0-23> [C]");
            return ret;
        }

        if(!strcmp(argv[1], "hot"))
        {
            temp_c = 65.0f;
            if((argc >= 5) && !cli_parse_float_arg(argv[4], &temp_c))
            {
                ret |= cli_printline(cli, "Usage: tempfake hot <smb> <sensor> [C]");
                return ret;
            }
        }
        else if(!strcmp(argv[1], "cold"))
        {
            temp_c = -5.0f;
            if((argc >= 5) && !cli_parse_float_arg(argv[4], &temp_c))
            {
                ret |= cli_printline(cli, "Usage: tempfake cold <smb> <sensor> [C]");
                return ret;
            }
        }
        else if((argc < 5) || !cli_parse_float_arg(argv[4], &temp_c))
        {
            ret |= cli_printline(cli, "Usage: tempfake sensor <smb 0-4> <sensor 0-23> <C>");
            return ret;
        }

        if(accumulator_temp_fake_set_sensor(&data->acc, (uint8_t)seg, (uint8_t)sensor, temp_c) != 0)
        {
            ret |= cli_printline(cli, "ERROR: tempfake sensor out of range or invalid temperature");
            return ret;
        }
        cli_refresh_temperature_from_accumulator();
        ret |= cli_printline(cli, "Temp fake sensor updated");
        return cli_print_temp_fake_status();
    }

    if(!strcmp(argv[1], "missing") || !strcmp(argv[1], "invalid"))
    {
        if((argc < 4) || !cli_parse_u16_arg(argv[2], &seg))
        {
            ret |= cli_printline(cli, "Usage: tempfake missing|invalid <smb 0-4> <mask24|none>");
            return ret;
        }

        if(!strcmp(argv[3], "none") || !strcmp(argv[3], "clear"))
        {
            mask = 0u;
        }
        else if(!cli_parse_u32_arg(argv[3], &mask))
        {
            ret |= cli_printline(cli, "Usage: tempfake missing|invalid <smb 0-4> <mask24|none>");
            return ret;
        }

        if(seg >= NSMBS)
        {
            ret |= cli_printline(cli, "ERROR: smb must be 0-4");
            return ret;
        }

        if(!strcmp(argv[1], "missing"))
        {
            accumulator_temp_fake_set_missing_mask(&data->acc, (uint8_t)seg, mask);
        }
        else
        {
            accumulator_temp_fake_set_invalid_mask(&data->acc, (uint8_t)seg, mask);
        }
        cli_refresh_temperature_from_accumulator();
        ret |= cli_printline(cli, "Temp fake mask updated");
        return cli_print_temp_fake_status();
    }

    if(!strcmp(argv[1], "holdmissing"))
    {
        if((argc < 3) || !cli_parse_bool_arg(argv[2], &on))
        {
            ret |= cli_printline(cli, "Usage: tempfake holdmissing <on|off>");
            return ret;
        }
        accumulator_temp_fake_set_hold_missing(&data->acc, on);
        cli_refresh_temperature_from_accumulator();
        snprintf(outline, CLI_LINESZ, "Temp fake holdmissing %s", on ? "on" : "off");
        ret |= cli_printline(cli, outline);
        return cli_print_temp_fake_status();
    }

    ret |= cli_printline(cli, "Usage: tempfake [help|status|reset|healthy|all|sensor|hot|cold|missing|invalid|holdmissing]");
    return ret;
}

int get_canlog(int argc, char *argv[])
{
    int ret = 0;
    canbus_capture_status_t status;

    if(!canbus_capture_enabled())
    {
        ret |= cli_printline(cli, "canlog unavailable: build with AMS_RENODE_CAN_CAPTURE=1");
        return ret;
    }

    if((argc >= 2) && (argv[1] != NULL) && !strcmp(argv[1], "clear"))
    {
        canbus_capture_clear();
        ret |= cli_printline(cli, "CAN capture counters cleared");
    }
    else if((argc >= 2) && (argv[1] != NULL) && strcmp(argv[1], "status"))
    {
        ret |= cli_printline(cli, "Usage: canlog [status|clear]");
        return ret;
    }

    status = canbus_capture_get_status();
    snprintf(outline, CLI_LINESZ,
             "CANLOG enabled:%d total:%lu std:%lu ext:%lu",
             status.enabled,
             (unsigned long)status.total_tx,
             (unsigned long)status.std_tx,
             (unsigned long)status.ext_tx);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "buckets ecu:%lu est:%lu logger:%lu charger:%lu other:%lu",
             (unsigned long)status.ecu_tx,
             (unsigned long)status.estimator_tx,
             (unsigned long)status.logger_tx,
             (unsigned long)status.charger_tx,
             (unsigned long)status.other_tx);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "last ide:%lu id:0x%08lX data:%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned long)status.last_ide,
             (unsigned long)status.last_id,
             status.last_payload[0],
             status.last_payload[1],
             status.last_payload[2],
             status.last_payload[3],
             status.last_payload[4],
             status.last_payload[5],
             status.last_payload[6],
             status.last_payload[7]);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "logger 690:%lu 691:%lu 692:%lu 693:%lu 694:%lu 695:%lu",
             (unsigned long)status.logger_id_count[0],
             (unsigned long)status.logger_id_count[1],
             (unsigned long)status.logger_id_count[2],
             (unsigned long)status.logger_id_count[3],
             (unsigned long)status.logger_id_count[4],
             (unsigned long)status.logger_id_count[5]);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "logger 696:%lu 697:%lu 698:%lu 699:%lu 69A:%lu 69B:%lu",
             (unsigned long)status.logger_id_count[6],
             (unsigned long)status.logger_id_count[7],
             (unsigned long)status.logger_id_count[8],
             (unsigned long)status.logger_id_count[9],
             (unsigned long)status.logger_id_count[10],
             (unsigned long)status.logger_id_count[11]);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "logger detail 6A0:%lu 6A1:%lu 6A2:%lu 6A3:%lu 6A4:%lu 6A5:%lu",
             (unsigned long)status.logger_id_count[12],
             (unsigned long)status.logger_id_count[13],
             (unsigned long)status.logger_id_count[14],
             (unsigned long)status.logger_id_count[15],
             (unsigned long)status.logger_id_count[16],
             (unsigned long)status.logger_id_count[17]);
    ret |= cli_printline(cli, outline);
    return ret;
}

static void cli_set_state_value(state_t state)
{
    data->state = state;
    if(state == STATE_CHARGE)
    {
        data->board.charger.last_rx_tick = osKernelGetTickCount();
    }
    if(data->board.canbus.hcan != NULL)
    {
        HAL_CAN_ActivateNotification(data->board.canbus.hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

static void cli_scenario_common_healthy(void)
{
    adbms6830_fake_reset();
    (void)current_sensor_fake_set_current_a(0.0f);
    accumulator_temp_fake_reset(&data->acc);
    charger_fake_reset();
    canbus_capture_clear();
    cli_reset_fault_domain("reset-all");
    cli_set_state_value(STATE_DISCARGE);
    cli_refresh_temperature_from_accumulator();
}

int get_scenario(int argc, char *argv[])
{
    int ret = 0;
    const char *name = ((argc >= 2) && (argv != NULL) && (argv[1] != NULL)) ? argv[1] : "help";

    if(!strcmp(name, "help"))
    {
        ret |= cli_printline(cli, "Usage: scenario [healthy|charge-ready|ov|uv|hot|cold|charger-timeout|charger-txfail|current-trip]");
        ret |= cli_printline(cli, "Scenarios set fake inputs only; observe with status/fault/volt/temp/current/charger/canlog");
        return ret;
    }

    if(!strcmp(name, "healthy"))
    {
        cli_scenario_common_healthy();
    }
    else if(!strcmp(name, "charge-ready"))
    {
        cli_scenario_common_healthy();
        cli_set_state_value(STATE_CHARGE);
        charger_fake_set_auto_reply(true);
        charger_fake_set_online(true);
        charger_fake_set_timeout(false);
    }
    else if(!strcmp(name, "ov"))
    {
        cli_scenario_common_healthy();
        (void)adbms6830_fake_set_cell_mv(0u, 3u, 4250u);
    }
    else if(!strcmp(name, "uv"))
    {
        cli_scenario_common_healthy();
        (void)adbms6830_fake_set_cell_mv(2u, 7u, 2300u);
    }
    else if(!strcmp(name, "hot"))
    {
        cli_scenario_common_healthy();
        (void)accumulator_temp_fake_set_sensor(&data->acc, 0u, 0u, 65.0f);
        cli_refresh_temperature_from_accumulator();
    }
    else if(!strcmp(name, "cold"))
    {
        cli_scenario_common_healthy();
        (void)accumulator_temp_fake_set_sensor(&data->acc, 0u, 0u, -5.0f);
        cli_refresh_temperature_from_accumulator();
    }
    else if(!strcmp(name, "charger-timeout"))
    {
        cli_scenario_common_healthy();
        cli_set_state_value(STATE_CHARGE);
        charger_fake_set_timeout(true);
        data->board.charger.last_rx_tick = osKernelGetTickCount() - CHARGER_RX_TIMEOUT_MS - 1u;
        data->board.charger.communication_fail = true;
    }
    else if(!strcmp(name, "charger-txfail"))
    {
        cli_scenario_common_healthy();
        cli_set_state_value(STATE_CHARGE);
        charger_fake_set_tx_fail(true);
    }
    else if(!strcmp(name, "current-trip"))
    {
        cli_scenario_common_healthy();
        cli_set_state_value(STATE_START);
        (void)current_sensor_fake_set_current_a(70.0f);
    }
    else
    {
        ret |= cli_printline(cli, "Usage: scenario [healthy|charge-ready|ov|uv|hot|cold|charger-timeout|charger-txfail|current-trip]");
        return ret;
    }

    snprintf(outline, CLI_LINESZ, "Scenario applied: %s", name);
    ret |= cli_printline(cli, outline);
    ret |= cli_printline(cli, "Wait a task cycle, then run: status; fault; volt; temp; current; charger; canlog");
    return ret;
}

int get_spi_debug(int argc, char *argv[])
{
    int ret = 0;
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
	        else if(!strcmp(argv[1], "probe"))
	        {
	            if(cli_refuse_renode_physical_spi("spi probe"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi probea"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi probeb"))
	            {
	                return ret;
	            }
	            if(cli_adbms_refuse_active_scan("spi probeb"))
	            {
	                return ret;
	            }
	            probe_status = adbms6830_spi_probe_rdcfga_on_string(smb, STRING_B);
	            snprintf(outline, CLI_LINESZ, "RDCFGA CS_B/stringB probe status: %s", cli_hal_status_str(probe_status));
	            ret |= cli_printline(cli, outline);
	        }
	        else if(!strcmp(argv[1], "sid"))
	        {
	            if(cli_refuse_renode_physical_spi("spi sid"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi stat"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi staterr"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi wake"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi coldwake"))
	            {
	                return ret;
	            }
	            if(cli_adbms_refuse_active_scan("spi coldwake"))
	            {
	                return ret;
	            }
	            adbms6830_wakeup_cold(smb);
	            ret |= cli_printline(cli, "ADBMS cold wake pulse train sent");
	        }
	        else if(!strcmp(argv[1], "clrflag"))
	        {
	            if(cli_refuse_renode_physical_spi("spi clrflag"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi cfgchk"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi cellst"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi oweven"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi owodd"))
	            {
	                return ret;
	            }
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
	            if(cli_refuse_renode_physical_spi("spi auxdiag"))
	            {
	                return ret;
	            }
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
	            ret |= cli_printline(cli, "Usage: spi [status|probe|probea|probeb|sid|stat|staterr|cfgchk|cellst|oweven|owodd|auxdiag|wake|coldwake|clrflag|clear|diagclear|enable|disable]");
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
	             "scan active:%d count:%lu diag fault:%d cfg:%d stat:%d ow:%d last:%s",
	             data->adbms_scan_active,
	             (unsigned long)data->adbms_scan_count,
	             data->adbms_diag_fault,
	             data->adbms_config_fault,
	             data->adbms_status_fault,
	             data->adbms_open_wire_fault,
	             cli_hal_status_str(data->adbms_last_diag_status));
	    ret |= cli_printline(cli, outline);

	    snprintf(outline, CLI_LINESZ,
	             "diag periodic status:%lu cfg:%lu openwire:%lu",
	             (unsigned long)data->adbms_status_diag_count,
	             (unsigned long)data->adbms_config_diag_count,
	             (unsigned long)data->adbms_open_wire_diag_count);
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
    int ret = 0;
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
	            if(cli_refuse_renode_physical_spi("apm probe"))
	            {
	                return ret;
	            }
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

    (void)argc;
    (void)argv;

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

    cli_fixed1(cs->current_50a, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "I_50A: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(cs->current_800a, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "I_800A: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

    cli_fixed1(cs->current, &whole, &decimal);
    snprintf(outline, CLI_LINESZ, "I_selected: %d.%01d A", whole, decimal);
    ret |= cli_printline(cli, outline);

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
        return ret;
    }

    if(!strcmp(mode, "board") || !strcmp(mode, "snapshot"))
    {
        SPI_HandleTypeDef *hspi = smb->hspi;
        current_sensor_t *cs = &data->board.current_sensor;
        bool spi_ok = cli_spi6_mode3_ok(hspi);
        bool current_alive = data->current_valid && cs->last_read_ok &&
                             (fabsf(cs->sensor_voltage_high - 2.5f) <= 0.35f) &&
                             (fabsf(cs->sensor_voltage_low - 2.5f) <= 0.35f) &&
                             (fabsf(data->current) <= 5.0f);

        snprintf(outline, CLI_LINESZ,
                 "BRINGUP BOARD build:%s state:%s BMS_OK:%d inhibit:%d",
                 AMS_BUILD_MODE_STR,
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
                 (int)cs->sensor_voltage_high,
                 abs((int)roundf((cs->sensor_voltage_high - (float)((int)cs->sensor_voltage_high)) * 1000.0f)),
                 (int)cs->sensor_voltage_low,
                 abs((int)roundf((cs->sensor_voltage_low - (float)((int)cs->sensor_voltage_low)) * 1000.0f)));
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
                 "sid=%s mask:0x%04X stat=%s mask:0x%04X diag_fault:%d cfg:%d stat:%d ow:%d",
                 cli_passfail((sid_mask & expected_mask) == expected_mask),
                 sid_mask,
                 cli_passfail((stat_mask & expected_mask) == expected_mask),
                 stat_mask,
                 data->adbms_diag_fault,
                 data->adbms_config_fault,
                 data->adbms_status_fault,
                 data->adbms_open_wire_fault);
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

        ret |= cli_printline(cli, "next: spi coldwake -> spi probea/probeb -> spi sid -> spi stat -> bringup adbms6830");
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
        ret |= cli_printline(cli, "2 spi clear; spi enable; spi coldwake; spi probea/probeb; spi sid; spi stat; bringup adbms6830");
        ret |= cli_printline(cli, "3 current; volt; temp; bringup ready");
        ret |= cli_printline(cli, "4 charger; bringup charger-lv plus CAN sniffer frame screenshots/logs");
        ret |= cli_printline(cli, "5 for battery/charger only: bringup charger-battery after approved safe setup");
        return ret;
    }

    ret |= cli_printline(cli, "Usage: bringup [help|board|adbms6830|apm2950|charger-lv|charger-battery|ready|snapshot|evidence]");
    return ret;
}

int get_version(int argc, char *argv[])
{
	int ret = 0;
	snprintf(outline, CLI_LINESZ, "v%d.%d.%d %s", VER_MAJOR, VER_MINOR, VER_BUG, AMS_BUILD_MODE_STR);
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
            data->bms_output_inhibit = false;
            ret |= cli_printline(cli, "BMS_OK output release enabled; safety gates still apply");
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
             "BMS_OK state:%d inhibit:%d blocked_assertions:%lu",
             data->bms_state,
             data->bms_output_inhibit,
             (unsigned long)data->bms_output_block_count);
    ret |= cli_printline(cli, outline);

    snprintf(outline, CLI_LINESZ,
             "ready gates voltage:%d current:%d hard:%d temp:%d charger:%d",
             data->voltage_valid && !data->voltage_fault,
             data->current_valid && !data->current_fault,
             data->hard_fault,
             data->temp_valid && !data->temp_fault &&
                 ((data->state != STATE_CHARGE) || !data->temp_charge_stop),
             data->charger_fault);
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
        if(!strcmp(argv[1], "start")) cli_set_state_value(STATE_START);
        else if(!strcmp(argv[1], "charge")) cli_set_state_value(STATE_CHARGE);
        else if(!strcmp(argv[1], "discharge")) cli_set_state_value(STATE_DISCARGE);
        else if(!strcmp(argv[1], "balance")) cli_set_state_value(STATE_BALANCE);
        else if(!strcmp(argv[1], "error")) cli_set_state_value(STATE_ERROR);
        else if(!strcmp(argv[1], "null")) cli_set_state_value(STATE_NULL);
        else
        {
            snprintf(outline, CLI_LINESZ, "ERROR: unrecognized state: %s", argv[1]);
            cli_printline(cli, outline);
            cli_printline(cli, "Usage: state [start|charge|discharge|balance|error|null]");
            return 1;
        }

        snprintf(outline, CLI_LINESZ, "AMS State: %s", ams_state_to_str(data->state));
        ret |= cli_printline(cli, outline);
    }
    else
    {
        cli_printline(cli, "ERROR: too many arguments");
        cli_printline(cli, "Usage: state [start|charge|discharge|balance|error|null]");
        return 1;
    }

    return ret;
}

int cause_fault(int argc, char *argv[])
{
	int ret = 0;

	set_bms(0);

	return ret;
};
