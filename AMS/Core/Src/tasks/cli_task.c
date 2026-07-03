/**
* @file cli_task.c
* @author Cole Bardin (cab572@drexel.edu)
* @brief
* @version 0.1
* @date 2023-10-24
*
* @copyright Copyright (c) 2023
*
*/

#include "tasks/cli_task.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ext_drivers/cli.h"

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
int get_spi_debug(int argc, char *argv[]);
int get_apm_debug(int argc, char *argv[]);
int bmsok_control(int argc, char *argv[]);
int set_state(int argc, char *argv[]);
int cause_fault(int argc, char *argv[]);

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
	{"spi", &get_spi_debug, "ADBMS6830 SPI debug: spi [status|probe|sid|stat|staterr|wake|coldwake|clrflag|clear|enable|disable]"},
	{"apm", &get_apm_debug, "ADBMS2950/APM debug: apm [status|probe|clear|enable|disable]"},
	{"bmsok", &bmsok_control, "BMS_OK control: bmsok [status|release|inhibit]"},
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
             AMS_HW_BRINGUP ? "hw-bringup" : "normal",
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
             AMS_HW_BRINGUP ? "hw-bringup" : "normal",
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



int get_spi_debug(int argc, char *argv[])
{
    int ret = 0;
    adbms6830_driver_t *smb = &data->acc.smb;
    const adbms6830_spi_debug_t *dbg;
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
            probe_status = adbms6830_spi_probe_rdcfga(smb);
            snprintf(outline, CLI_LINESZ, "RDCFGA probe status: %s", cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "sid"))
        {
            probe_status = adbms6830_read_sid(smb);
            snprintf(outline, CLI_LINESZ, "RDSID status: %s", cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "stat"))
        {
            probe_status = adbms6830_read_status(smb, false);
            snprintf(outline, CLI_LINESZ, "RDSTATC/D/E status: %s", cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "staterr"))
        {
            probe_status = adbms6830_read_status(smb, true);
            snprintf(outline, CLI_LINESZ, "RDSTATCERR/D/E status: %s", cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);
        }
        else if(!strcmp(argv[1], "wake"))
        {
            if(smb->spi_debug.enabled)
            {
                smb->spi_debug.last_op = ADBMS6830_SPI_OP_WAKE;
            }
            adbms6830_wakeup(smb);
            ret |= cli_printline(cli, "ADBMS wake pulses sent");
        }
        else if(!strcmp(argv[1], "coldwake"))
        {
            adbms6830_wakeup_cold(smb);
            ret |= cli_printline(cli, "ADBMS cold wake pulse train sent");
        }
        else if(!strcmp(argv[1], "clrflag"))
        {
            probe_status = adbms6830_clear_all_flags(smb);
            snprintf(outline, CLI_LINESZ, "CLRFLAG all status: %s", cli_hal_status_str(probe_status));
            ret |= cli_printline(cli, outline);
        }
        else if(strcmp(argv[1], "status"))
        {
            ret |= cli_printline(cli, "Usage: spi [status|probe|sid|stat|staterr|wake|coldwake|clrflag|clear|enable|disable]");
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

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        const adbms6830_ic_diag_t *diag = &smb->diag[ic];
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

int get_version(int argc, char *argv[])
{
	int ret = 0;
	snprintf(outline, CLI_LINESZ, "v%d.%d.%d %s", VER_MAJOR, VER_MINOR, VER_BUG, AMS_HW_BRINGUP ? "hw-bringup" : "normal");
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
        if(!strcmp(argv[1], "charge")){
        	data->state = STATE_CHARGE;
        	data->board.charger.last_rx_tick = osKernelGetTickCount();
        }
        else if(!strcmp(argv[1], "discharge")) data->state = STATE_DISCARGE;
        else
        {
            snprintf(outline, CLI_LINESZ, "ERROR: unrecognized state: %s", argv[1]);
            cli_printline(cli, outline);
            cli_printline(cli, "Usage: state [charge|discharge]");
            return 1;
        }

        snprintf(outline, CLI_LINESZ, "AMS State: %s", ams_state_to_str(data->state));
        ret |= cli_printline(cli, outline);

        if(data->board.canbus.hcan != NULL)
        {
            HAL_CAN_ActivateNotification(data->board.canbus.hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        }
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

	set_bms(0);

	return ret;
};
