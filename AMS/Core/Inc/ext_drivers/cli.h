/**
 * @file cli.h
 * @author Cole Bardin (cab572@drexel.edu)
 * @author Mahad Faisal (major firmware updates, 2026)
 * @brief
 * @version 0.1
 * @date 2023-10-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CLI_H_
#define __CLI_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "stm32f7xx_hal.h"

#define CLI_LINESZ 128
#define MAXTOKS (CLI_LINESZ / 2)


static inline size_t cli_bounded_strlen(const char *s, size_t max_len)
{
    size_t len = 0u;

    if(s == NULL)
    {
        return 0u;
    }

    while((len < max_len) && (s[len] != '\0'))
    {
        len++;
    }

    return len;
}

typedef struct {
    uint8_t c;
    volatile unsigned int index;
	UART_HandleTypeDef *huart;
    volatile bool msg_pending;
    volatile unsigned int msg_count;
    volatile unsigned int msg_proc;
    volatile unsigned int msg_valid;
    char line[CLI_LINESZ];
    HAL_StatusTypeDef ret;

    /* USART3 RX health and recovery diagnostics. These fields are shared
     * between interrupt and task context, so keep them volatile. */
    volatile uint32_t uart_error_count;
    volatile uint32_t uart_ore_count;
    volatile uint32_t uart_fe_count;
    volatile uint32_t uart_ne_count;
    volatile uint32_t uart_pe_count;
    volatile uint32_t uart_rto_count;
    volatile uint32_t uart_dma_count;
    volatile uint32_t uart_last_error;
    volatile uint32_t rx_arm_count;
    volatile uint32_t rx_recovery_count;
    volatile uint32_t rx_rearm_fail_count;
    volatile uint32_t rx_busy_count;
    volatile uint32_t rx_drop_count;
    volatile HAL_StatusTypeDef rx_last_status;
} cli_device_t;

typedef struct {
    char *name;
    int (*func)(int argc, char *argv[]);
    char *desc;
} command_t;

void cli_device_init(cli_device_t *dev, UART_HandleTypeDef *huart);
HAL_StatusTypeDef cli_uart_start_rx(cli_device_t *dev);
HAL_StatusTypeDef cli_uart_service_rx(cli_device_t *dev);
HAL_StatusTypeDef cli_uart_force_recover(cli_device_t *dev);
void cli_uart_note_error(cli_device_t *dev, uint32_t error_code);
void cli_uart_diag_clear(cli_device_t *dev);
int cli_printline(cli_device_t *dev, char *line);
int tokenize(char *s, char *toks[], int maktoks, char *delim);

#endif
