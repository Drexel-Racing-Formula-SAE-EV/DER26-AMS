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
int get_faults(int argc, char *argv[]);
int get_version(int argc, char *argv[]);
int get_voltage(int argc, char *argv[]);
int get_temperature(int argc, char *argv[]);
int get_temperature_sensor(int argc, char *argv[]);
int get_current(int argc, char *argv[]);
int set_state(int argc, char *argv[]);
int cause_fault(int argc, char *argv[]);

char outline[CLI_LINESZ];
app_data_t *data;
cli_device_t *cli;
command_t cmds[] =
{
	{"help", &help, "print help menu"},
	{"fault", &get_faults, "gets the faults of the system"},
	{"ver", &get_version, "gets the firmware version"},
	{"volt", &get_voltage, "gets cell voltages for all SMBs"},
	{"temp", &get_temperature, "gets sensor temperatures for all SMBs"},
	{"tempsns", &get_temperature_sensor, "gets one sensor: tempsns <ic> <sensor 0-23>"},
	{"current", &get_current, "gets current sensor raw counts/voltages/status"},
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

    snprintf(outline, CLI_LINESZ, "~~~~~~~~~~ DER AMS FW V%d.%d ~~~~~~~~~~", VER_MAJOR, VER_MINOR);
	cli_printline(local_cli, outline);
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

                snprintf(outline, CLI_LINESZ, "SMB %d | Sensor %d: %-2d.%04d V, %d.%d C", ic, sensor, whole, decimal, T_whole, T_decimal);
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
	snprintf(outline, CLI_LINESZ, "v%d.%d", VER_MAJOR, VER_MINOR);
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
