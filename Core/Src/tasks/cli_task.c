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
#include <string.h>
#include <stdio.h>
#include "ext_drivers/cli.h"
#include <math.h>
#include <stdlib.h>

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
};

TaskHandle_t cli_task_start(app_data_t *data)
{
   TaskHandle_t handle;
   xTaskCreate(cli_task_fn, "CLI task", 256, (void *)data, CLI_PRIO, &handle);
   return handle;
}

void cli_task_fn(void *arg)
{
    data = (app_data_t *)arg;
    cli = &data->board.cli;
    uint32_t entry;
    char buf[CLI_LINESZ] = {0};
    char *tokens[MAXTOKS];
    int n;
    int ret = 0;

    snprintf(outline, CLI_LINESZ, "~~~~~~~~~~ DER AMS FW V%d.%d ~~~~~~~~~~", VER_MAJOR, VER_MINOR);
	cli_printline(cli, outline);
	cli_printline(cli, "Type 'help' for list of commands");

	for(;;)
	{
		entry = osKernelGetTickCount();
		if(cli->msg_pending == true)
		{
			taskENTER_CRITICAL();
			memcpy(buf, cli->line, strlen(cli->line) + 1);
			memset(cli->line, 0, strlen(cli->line) + 1);
			n = tokenize(buf, tokens, MAXTOKS, " \t");
			ret = cli_handle_cmd(n, tokens);
			taskEXIT_CRITICAL();
			data->cli_fault = ret;
			cli->msg_pending = false;
			cli->msg_proc++;
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
	snprintf(outline, CLI_LINESZ, "  fan:    %d", data->fan_fault);
	ret |= cli_printline(cli, outline);
	snprintf(outline, CLI_LINESZ, "  canbus: %d", data->canbus_fault);
	ret |= cli_printline(cli, outline);
	return ret;
}

int get_voltage(int argc, char *argv[])
{
    int ret = 0;
    adbms6830_driver_t *smb = &data->acc.smb;

    for (int ic = 0; ic < smb->num_ics; ic++)
    {
        snprintf(outline, CLI_LINESZ, "--- SMB %d ---", ic);
        ret |= cli_printline(cli, outline);

        for (int cell = 0; cell < NCELLS; cell++)
        {
            float volt = (smb->ics[ic].cell.c_codes[cell] + 10000) * 0.000150f;
            int whole   = (int)volt;
            int decimal = (int)((volt - whole) * 10000);
            snprintf(outline, CLI_LINESZ, "  C%-2d: %d.%04d V", cell + 1, whole, decimal);
            ret |= cli_printline(cli, outline);
        }
    }
    return ret;
}


int get_temperature(int argc, char *argv[])
{
    int ret = 0;
    adbms6830_driver_t *smb = &data->acc.smb;






    for (int ic = 0; ic < smb->num_ics; ic++)
    {

	//Purely For Testing function
//	for (uint8_t sensor = 0; sensor < 24u; sensor++)
//	{
//	    mux_read_gpio_voltage(smb, sensor);
//	    adbms6830_us_delay(smb, 50000u);
//	}
	//Finished Testing

        snprintf(outline, CLI_LINESZ, "--- SMB %d ---", ic);
        ret |= cli_printline(cli, outline);

        for (int sensor = 0; sensor < NTEMPS; sensor++)
        {
            float volt = (smb->ics[ic].temp.raw[sensor] + 10000) * 0.000150f;
            int whole   = (int)volt;
            int decimal = (int)((volt - whole) * 10000);

            float R = 10000.0f * (5.0f - volt) / volt;
            float x = log(R / 10000.0f);
            float T = 1.0f / (3.354016435e-3f + 2.565235509e-4f*x + 2.605970121e-6f*x*x + 6.329261265e-8f*x*x*x) - 273.15f;

            int T_whole   = (int)T;                          // e.g. -12  or  23
            int T_decimal = (int)roundf((T - T_whole) * 10); // e.g.  4   or   7

            snprintf(outline, CLI_LINESZ, "SMB %d | Sensor %d: %-2d.%04d V, %d.%d C", ic, sensor, whole, decimal, T_whole, T_decimal);
//            snprintf(outline, CLI_LINESZ, "  Sensor %-2d: %d.%04d V", sensor, whole, decimal);
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
    if (ic < 0 || ic >= smb->num_ics)
    {
        snprintf(outline, CLI_LINESZ, "Error: ic must be 0 to %d", smb->num_ics - 1);
        ret |= cli_printline(cli, outline);
        return ret;
    }

    /* Validate sensor index (0–23 per requirements) */
    if (sensor < 0 || sensor > 23)
    {
        ret |= cli_printline(cli, "Error: sensor must be 0 to 23");
        return ret;
    }

    /* Poll just the requested sensor through the mux */
    mux_read_gpio_voltage(smb, sensor);

    /* Read back the raw value and convert to voltage */
    float volt  = (smb->ics[ic].temp.raw[sensor] + 10000) * 0.000150f;
    int whole   = (int)volt;
    int decimal = (int)((volt - whole) * 10000);

    float R = 10000.0f * (5.0f - volt) / volt;
    float x = log(R / 10000.0f);
    float T = 1.0f / (3.354016435e-3f + 2.565235509e-4f*x + 2.605970121e-6f*x*x + 6.329261265e-8f*x*x*x) - 273.15f;

    int T_whole   = (int)T;                          // e.g. -12  or  23
    int T_decimal = (int)roundf((T - T_whole) * 10); // e.g.  4   or   7

    snprintf(outline, CLI_LINESZ, "SMB %d | Sensor %d: %d.%04d V, %d.%d C", ic, sensor, whole, decimal, T_whole, T_decimal);
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

