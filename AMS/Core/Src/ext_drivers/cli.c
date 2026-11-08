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

#define CLI_TX_STALL_MS 250u
#define CLI_TX_ENQUEUE_MS 50u

static uint32_t cli_lock(void)
{
#if defined(AMS_HOST_TEST) && AMS_HOST_TEST
    return 0u;
#else
    return portSET_INTERRUPT_MASK_FROM_ISR();
#endif
}
static void cli_unlock(uint32_t mask)
{
#if defined(AMS_HOST_TEST) && AMS_HOST_TEST
    (void)mask;
#else
    portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
#endif
}
static void cli_tx_kick(cli_device_t *d)
{
    if(d->tx_discard_pending && d->tx_active_len==0u)
    {
        d->tx_head=d->tx_tail=d->tx_used=0u;d->tx_discard_pending=false;
    }
    if(d->tx_active_len || !d->tx_used)return;
    uint16_t n=(uint16_t)(CLI_TX_QUEUE_BYTES-d->tx_tail);
    if(n>d->tx_used)n=d->tx_used;
    if(n>64u)n=64u;
    d->tx_active_len=n;
    if(HAL_UART_Transmit_IT(d->huart,&d->tx_queue[d->tx_tail],n)!=HAL_OK)
    {d->tx_active_len=0u;d->tx_errors++;}
    else d->tx_start_tick=osKernelGetTickCount();
}
void cli_uart_tx_complete(cli_device_t *d)
{
    if(!d)return;
    uint32_t mask=cli_lock();uint16_t n=d->tx_active_len;
    d->tx_tail=(uint16_t)((d->tx_tail+n)%CLI_TX_QUEUE_BYTES);
    d->tx_used-=n;d->tx_bytes+=n;d->tx_active_len=0u;
    cli_tx_kick(d);cli_unlock(mask);
}
void cli_uart_service_tx(cli_device_t *d)
{
    if(!d || !d->huart)return;
    uint32_t mask=cli_lock();
    if(d->tx_used && (uint32_t)(osKernelGetTickCount()-d->tx_start_tick)>CLI_TX_STALL_MS)
    {
        (void)HAL_UART_AbortTransmit(d->huart);
        d->tx_active_len=0u;d->tx_head=d->tx_tail=d->tx_used=0u;
        d->tx_aborts++;d->report_cancel=true;
    }
    cli_tx_kick(d);cli_unlock(mask);
}

void cli_device_init(cli_device_t *dev, UART_HandleTypeDef *huart)
{
    if(dev == NULL)
    {
        return;
    }

    memset(dev,0,sizeof(*dev));dev->huart=huart;
    dev->index = 0;
    dev->msg_pending = false;
    dev->msg_count = 0;
    dev->msg_proc = 0;
    dev->msg_valid = 0;
    dev->ret = HAL_OK;
    dev->report_buffer = NULL;
    dev->report_capacity = 0u;
    dev->report_used = 0u;
    dev->report_truncated = false;

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
    dev->index = 0u;
    dev->line[0] = '\0';
    dev->rx_discard_line = true;

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
    cli_uart_service_tx(dev);
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

#if !(defined(AMS_HOST_TEST) && AMS_HOST_TEST)
	if(xPortIsInsideInterrupt())
	{
		/* Do not use UART TX from ISR context. HAL_UART_Transmit_IT() is not a
		 * print queue, and blocking UART TX from an IRQ is forbidden for timing. */
		return HAL_BUSY;
	}
#endif
    if(dev->report_cancel)return HAL_BUSY;
	if(dev->report_buffer != NULL)
	{
		/* A complete line is either stored or dropped. The CLI task flushes
		 * these NUL-delimited lines after it releases the SPI owner lock. */
		if((line_len + 1u) > (dev->report_capacity - dev->report_used))
		{
			dev->report_truncated = true;
			return HAL_ERROR;
		}
		memcpy(&dev->report_buffer[dev->report_used], line, line_len);
		dev->report_buffer[dev->report_used + line_len] = '\0';
		dev->report_used += line_len + 1u;
		return HAL_OK;
	}
	else
	{
        uint32_t start=osKernelGetTickCount();
        for(;;)
        {
            cli_uart_service_tx(dev);
            uint32_t mask=cli_lock();
            if(dev->report_cancel){cli_unlock(mask);return HAL_BUSY;}
            if(CLI_TX_QUEUE_BYTES-dev->tx_used>=line_len+2u)
            {
                for(size_t i=0;i<line_len+2u;i++)
                {
                    dev->tx_queue[dev->tx_head]=(i<line_len)?(uint8_t)line[i]:(uint8_t)nl[i-line_len];
                    dev->tx_head=(uint16_t)((dev->tx_head+1u)%CLI_TX_QUEUE_BYTES);
                }
                if(dev->tx_used==0u)dev->tx_start_tick=osKernelGetTickCount();
                dev->tx_used+=(uint16_t)(line_len+2u);
                if(dev->tx_used>dev->tx_high_water)dev->tx_high_water=dev->tx_used;
                cli_tx_kick(dev);cli_unlock(mask);break;
            }
            cli_unlock(mask);
            if((uint32_t)(osKernelGetTickCount()-start)>=CLI_TX_ENQUEUE_MS)
            {dev->tx_dropped++;dev->report_cancel=true;return HAL_TIMEOUT;}
            osDelay(1u);
        }
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
