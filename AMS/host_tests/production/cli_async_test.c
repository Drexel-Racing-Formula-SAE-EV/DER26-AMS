#include "app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick, waits, aborts;
static const uint8_t *pending;
static uint16_t pending_n;
static bool drain_on_delay, fail_start;
static unsigned char wire[200000];
static size_t wire_n;
static cli_device_t dev;
static UART_HandleTypeDef uart;
static USART_TypeDef regs;
uint32_t osKernelGetTickCount(void){return tick;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h,const uint8_t *p,uint16_t n)
{(void)h;if(fail_start)return HAL_BUSY;assert(!pending);pending=p;pending_n=n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *h)
{(void)h;pending=NULL;pending_n=0;aborts++;return HAL_OK;}
static void complete(void)
{
    if(!pending)return;
    assert(wire_n+pending_n<sizeof(wire));
    memcpy(wire+wire_n,pending,pending_n);wire_n+=pending_n;
    pending=NULL;pending_n=0;cli_uart_tx_complete(&dev);
}
osStatus_t osDelay(uint32_t ms)
{tick+=ms;waits++;if(drain_on_delay)complete();return osOK;}
/* Real implementation, including queue ownership and timeout behavior. */
#include "Core/Src/ext_drivers/cli.c"

static void reset(void)
{
    memset(&uart,0,sizeof(uart));uart.Instance=&regs;
    pending=NULL;pending_n=0;tick=waits=aborts=0;wire_n=0;
    drain_on_delay=true;fail_start=false;cli_device_init(&dev,&uart);
}
static void feed(const char *s){while(*s)cli_rx_byte(&dev,(uint8_t)*s++);}
int main(void)
{
    reset();char message[]="123456789012345678901234567890";
    for(unsigned i=0;i<1000;i++)assert(cli_printline(&dev,message)==HAL_OK);
    while(pending)complete();
    assert(wire_n==32000u && waits>0 && dev.tx_bytes==wire_n);
    for(size_t i=0;i<wire_n;i+=32u)
    {assert(memcmp(wire+i,message,30u)==0);assert(wire[i+30]=='\r' && wire[i+31]=='\n');}
    puts("PASS real CLI IT TX wrap, ordering, CRLF and cooperative backpressure");
    reset();feed("one\r\ntwo\rthree\nfour\rfive\r");
    assert(dev.rx_count==4 && dev.rx_queue_overflow==1);
    char line[CLI_LINESZ];assert(cli_take_line(&dev,line) && !strcmp(line,"one"));
    assert(cli_take_line(&dev,line) && !strcmp(line,"two"));
    for(unsigned i=0;i<140;i++)cli_rx_byte(&dev,'x');
    feed("\r\n");assert(dev.rx_line_overflow==1);
    assert(dev.rx_count==2);
    feed("state stop\r");assert(dev.stop_requested && dev.report_cancel);
    assert(dev.rx_count==0 && !cli_take_line(&dev,line));
    puts("PASS bounded RX, CRLF, burst rejection, overflow rejection and stop bypass");
    reset();assert(cli_printline(&dev,message)==HAL_OK);feed("state\tstop  \r");
    assert(cli_printline(&dev,message)==HAL_BUSY);complete();
    assert(dev.tx_used==0 && !dev.tx_discard_pending);
    reset();drain_on_delay=false;
    while(cli_printline(&dev,message)==HAL_OK){}
    assert(waits>0 && tick==50 && dev.tx_dropped==1 && dev.report_cancel);
    tick=251;cli_uart_service_tx(&dev);assert(aborts==1 && dev.tx_used==0);
    puts("PASS cancellation and stalled TX have finite cooperative waits/recovery");
    reset();fail_start=true;assert(cli_printline(&dev,message)==HAL_OK);
    tick=251;cli_uart_service_tx(&dev);assert(aborts==1 && !dev.tx_used);
    reset();feed("state stopX\r");assert(!dev.stop_requested && dev.rx_count==1);
    reset();feed("state stop");cli_rx_byte(&dev,0u);feed("x\r");
    assert(!dev.stop_requested && dev.rx_count==0);
    puts("PASS failed IT start recovery and exact stop recognition");
    return 0;
}
