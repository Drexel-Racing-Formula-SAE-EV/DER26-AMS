/**
 * @file cli.c
 * @author Cole Bardin (cab572@drexel.edu)
 * @author Mahad Faisal (major firmware updates, 2026)
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

#define CLI_TX_TIMEOUT_MS 20u

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
    dev->ret = HAL_OK;

    cli_uart_diag_clear(dev);
}

void cli_uart_diag_clear(cli_device_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

    dev->uart_error_count = 0u;
    dev->uart_ore_count = 0u;
    dev->uart_fe_count = 0u;
    dev->uart_ne_count = 0u;
    dev->uart_pe_count = 0u;
    dev->uart_rto_count = 0u;
    dev->uart_dma_count = 0u;
    dev->uart_last_error = HAL_UART_ERROR_NONE;
    dev->rx_arm_count = 0u;
    dev->rx_recovery_count = 0u;
    dev->rx_rearm_fail_count = 0u;
    dev->rx_busy_count = 0u;
    dev->rx_drop_count = 0u;
    dev->rx_last_status = HAL_OK;
}

void cli_uart_note_error(cli_device_t *dev, uint32_t error_code)
{
    if((dev == NULL) || (error_code == HAL_UART_ERROR_NONE))
    {
        return;
    }

    dev->uart_error_count++;
    dev->uart_last_error = error_code;

    if((error_code & HAL_UART_ERROR_ORE) != 0u)
    {
        dev->uart_ore_count++;
    }
    if((error_code & HAL_UART_ERROR_FE) != 0u)
    {
        dev->uart_fe_count++;
    }
    if((error_code & HAL_UART_ERROR_NE) != 0u)
    {
        dev->uart_ne_count++;
    }
    if((error_code & HAL_UART_ERROR_PE) != 0u)
    {
        dev->uart_pe_count++;
    }
    if((error_code & HAL_UART_ERROR_RTO) != 0u)
    {
        dev->uart_rto_count++;
    }
    if((error_code & HAL_UART_ERROR_DMA) != 0u)
    {
        dev->uart_dma_count++;
    }
}

HAL_StatusTypeDef cli_uart_start_rx(cli_device_t *dev)
{
    HAL_StatusTypeDef status;

    if((dev == NULL) || (dev->huart == NULL))
    {
        return HAL_ERROR;
    }

    status = HAL_UART_Receive_IT(dev->huart, &dev->c, 1u);
    dev->rx_last_status = status;

    if(status == HAL_OK)
    {
        dev->rx_arm_count++;
    }
    else if(status == HAL_BUSY)
    {
        dev->rx_busy_count++;
    }
    else
    {
        dev->rx_rearm_fail_count++;
    }

    return status;
}

HAL_StatusTypeDef cli_uart_force_recover(cli_device_t *dev)
{
    UART_HandleTypeDef *huart;
    HAL_StatusTypeDef abort_status;
    HAL_StatusTypeDef arm_status;

    if((dev == NULL) || (dev->huart == NULL))
    {
        return HAL_ERROR;
    }

    huart = dev->huart;

    /* Preserve a completed command that is waiting for the CLI task, but
     * discard any partially received line because it may contain a damaged
     * byte from the UART fault. */
    if(dev->msg_pending == false)
    {
        dev->index = 0u;
        dev->line[0] = '\0';
    }

    abort_status = HAL_UART_AbortReceive(huart);
    if(abort_status != HAL_OK)
    {
        dev->rx_last_status = abort_status;
        dev->rx_rearm_fail_count++;
        return abort_status;
    }

    __HAL_UART_CLEAR_FLAG(huart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    arm_status = cli_uart_start_rx(dev);
    if(arm_status == HAL_OK)
    {
        dev->rx_recovery_count++;
    }

    return arm_status;
}

HAL_StatusTypeDef cli_uart_service_rx(cli_device_t *dev)
{
    UART_HandleTypeDef *huart;
    uint32_t error_code;
    bool rx_irq_enabled;
    bool error_irq_enabled;

    if((dev == NULL) || (dev->huart == NULL))
    {
        return HAL_ERROR;
    }

    huart = dev->huart;
    error_code = HAL_UART_GetError(huart);
    rx_irq_enabled = ((huart->Instance->CR1 & USART_CR1_RXNEIE) != 0u);
    error_irq_enabled = ((huart->Instance->CR3 & USART_CR3_EIE) != 0u);

    if(error_code != HAL_UART_ERROR_NONE)
    {
        /* Normally the HAL error callback records and clears this. Reaching
         * here means the callback did not complete recovery, so recover from
         * task context as a second line of defense. */
        cli_uart_note_error(dev, error_code);
        return cli_uart_force_recover(dev);
    }

    if((huart->RxState == HAL_UART_STATE_BUSY_RX) &&
       rx_irq_enabled && error_irq_enabled)
    {
        return HAL_OK;
    }

    /* READY means no one-byte receive is armed. BUSY_RX with disabled RX/error
     * interrupts is also inconsistent. Abort and re-arm either condition. */
    return cli_uart_force_recover(dev);
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
		/* Do not use UART TX from ISR context. HAL_UART_Transmit_IT() is not a
		 * print queue, and blocking UART TX from an IRQ is forbidden for timing. */
		return HAL_BUSY;
	}
	else
	{
		//while(osMutexAcquire(app.board.stm32f767.uart3_mutex, 0) != osOK) osDelay(5);
		ret |= HAL_UART_Transmit(dev->huart, (uint8_t *)line, (uint16_t)line_len, CLI_TX_TIMEOUT_MS);
		ret |= HAL_UART_Transmit(dev->huart, (uint8_t *)nl, 2u, CLI_TX_TIMEOUT_MS);
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
