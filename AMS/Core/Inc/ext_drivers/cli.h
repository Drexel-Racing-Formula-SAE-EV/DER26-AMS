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
#include <string.h>

#include "stm32f7xx_hal.h"
#include "ams_build_profile.h"

#define CLI_LINESZ 128
#define MAXTOKS (CLI_LINESZ / 2)
#define CLI_RX_QUEUE_DEPTH 4u
#define CLI_TX_QUEUE_BYTES 4096u


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
    char rx_queue[CLI_RX_QUEUE_DEPTH][CLI_LINESZ];
    volatile uint8_t rx_head, rx_tail, rx_count;
    volatile bool rx_discard_line;
    volatile uint32_t rx_line_overflow, rx_queue_overflow;
    volatile bool stop_requested, stop_ack_pending, report_cancel;
    volatile uint32_t stop_count, stop_applied_tick;
    uint32_t command_stop_count;
    uint32_t command_last_ms, command_max_ms;
    uint8_t tx_queue[CLI_TX_QUEUE_BYTES];
    volatile uint16_t tx_head, tx_tail, tx_used, tx_active_len;
    volatile uint32_t tx_start_tick, tx_bytes, tx_dropped, tx_errors, tx_aborts, tx_high_water;
    volatile bool tx_discard_pending;
    volatile bool rx_dma_recover_pending;
    volatile uint32_t rx_dma_events, rx_dma_overruns;
    HAL_StatusTypeDef ret;
    /* CLI-task-only bounded report staging. Never transmit UART while the
     * CLI owns the ADBMS transport lock. */
    char *report_buffer;
    size_t report_capacity;
    size_t report_used;
    bool report_truncated;

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

/* ISR producer / critical-section task consumer. A stop line is a bounded
 * lexical request; only the supervisor applies control policy. */
static inline bool cli_stop_line(const char *s)
{
    while(*s==' ' || *s=='\t')s++;
    if(strncmp(s,"state",5u)!=0)return false;
    s+=5; if(*s!=' ' && *s!='\t')return false;
    while(*s==' ' || *s=='\t')s++;
    if(strncmp(s,"stop",4u)!=0)return false;
    s+=4; while(*s==' ' || *s=='\t')s++;
    return *s=='\0';
}
static inline void cli_rx_byte(cli_device_t *d, uint8_t c)
{
    if(c=='\r' || c=='\n')
    {
        if(d->rx_discard_line){d->rx_discard_line=false;d->index=0u;return;}
        if(d->index==0u)return;
        d->line[d->index]='\0'; d->index=0u; d->msg_count++;
        if(cli_stop_line(d->line))
        {
            d->stop_requested=true;d->report_cancel=true;d->tx_discard_pending=true;
            d->rx_head=d->rx_tail=d->rx_count=0u;d->msg_pending=false;
            return;
        }
        if(d->rx_count==CLI_RX_QUEUE_DEPTH){d->rx_queue_overflow++;d->rx_drop_count++;return;}
        memcpy(d->rx_queue[d->rx_head],d->line,CLI_LINESZ);
        d->rx_head=(uint8_t)((d->rx_head+1u)%CLI_RX_QUEUE_DEPTH);
        d->rx_count++;d->msg_pending=true;
    }
    else if(d->rx_discard_line)return;
    else if(c<32u && c!='\t' && c!='\b')
    {d->rx_discard_line=true;d->rx_drop_count++;}
    else if(c=='\b' || c==127u){if(d->index)d->index--;}
    else if(d->index<CLI_LINESZ-1u)d->line[d->index++]=(char)c;
    else {d->rx_discard_line=true;d->rx_line_overflow++;d->rx_drop_count++;}
}
static inline bool cli_take_line(cli_device_t *d, char out[CLI_LINESZ])
{
    if(d->rx_count==0u || d->stop_requested)return false;
    memcpy(out,d->rx_queue[d->rx_tail],CLI_LINESZ);
    d->rx_tail=(uint8_t)((d->rx_tail+1u)%CLI_RX_QUEUE_DEPTH);
    d->rx_count--;d->msg_pending=d->rx_count!=0u;d->msg_proc++;
    return true;
}

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
void cli_uart_tx_complete(cli_device_t *dev);
#if AMS_CLI_UART_DMA
void cli_uart_dma_rx_event(cli_device_t *dev);
void cli_uart_dma_irq(cli_device_t *dev);
void cli_uart_dma_tx_irq(cli_device_t *dev);
#endif
void cli_uart_service_tx(cli_device_t *dev);
int tokenize(char *s, char *toks[], int maktoks, char *delim);

#endif
