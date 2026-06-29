/**
 * @file cli.c
 * @author Cole Bardin (cab572@drexel.edu)
 * @brief
 * @version 0.1
 * @date 2023-10-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <string.h>

#include "cmsis_os.h"

#include "ext_drivers/cli.h"

void cli_device_init(cli_device_t *dev, UART_HandleTypeDef *huart)
{
    if(dev == NULL)
    {
        return;
    }

    dev->huart = huart;
    dev->index = 0;
    dev->msg_pending = false;
    dev->msg_count = 0;
    dev->msg_proc = 0;
    dev->msg_valid = 0;
    dev->ret = 0;
}

int cli_printline(cli_device_t *dev, char *line)
{
	static char nl[] = "\r\n";
	HAL_StatusTypeDef ret = 0;

    if((dev == NULL) || (dev->huart == NULL) || (line == NULL))
    {
        return HAL_ERROR;
    }

    size_t line_len = cli_bounded_strlen(line, CLI_LINESZ - 1u);

	if(xPortIsInsideInterrupt())
	{
		ret |= HAL_UART_Transmit_IT(dev->huart, (uint8_t *)line, (uint16_t)line_len);
		ret |= HAL_UART_Transmit_IT(dev->huart, (uint8_t*)nl, 2u);
	}
	else
	{
		//while(osMutexAcquire(app.board.stm32f767.uart3_mutex, 0) != osOK) osDelay(5);
		ret |= HAL_UART_Transmit(dev->huart, (uint8_t *)line, (uint16_t)line_len, HAL_MAX_DELAY);
		ret |= HAL_UART_Transmit(dev->huart, (uint8_t *)nl, 2u, HAL_MAX_DELAY);
		//osMutexRelease(app.board.stm32f767.uart3_mutex);
	}
	return ret;
}

int tokenize(char *s, char *toks[], int maxtoks, char *delim)
{
    int count = 0;

    if((s == NULL) || (toks == NULL) || (delim == NULL) || (maxtoks <= 0))
    {
        return 0;
    }

    toks[0] = NULL;
    if(maxtoks <= 1)
    {
        return 0;
    }

    char *tok = strtok(s, delim);
    while((tok != NULL) && (count < (maxtoks - 1)))
    {
        toks[count++] = tok;
        tok = strtok(NULL, delim);
    }
    toks[count] = NULL;

    return count;
}
