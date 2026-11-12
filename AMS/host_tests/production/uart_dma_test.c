#include "app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static USART_TypeDef uart_regs;
static DMA_TypeDef dma_regs;
static DMA_Stream_TypeDef rx_regs,tx_regs;
static SCB_Type scb_regs;
#undef USART3
#undef DMA1
#undef DMA1_Stream1
#undef DMA1_Stream3
#undef SCB
#define USART3 (&uart_regs)
#define DMA1 (&dma_regs)
#define DMA1_Stream1 (&rx_regs)
#define DMA1_Stream3 (&tx_regs)
#define SCB (&scb_regs)
#undef __HAL_RCC_DMA1_CLK_ENABLE
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)
#undef __HAL_DMA_CLEAR_FLAG
#define __HAL_DMA_CLEAR_FLAG(h,f) (DMA1->LIFCR=(f))
#define __DMB() ((void)0)
static unsigned cleans,invalidates,cleaninvalidates;
#define SCB_CleanDCache_by_Addr(a,n) do { assert(((uintptr_t)(a)&31u)==0 && n==64);cleans++; }while(0)
#define SCB_InvalidateDCache_by_Addr(a,n) do { assert(((uintptr_t)(a)&31u)==0 && n==256);invalidates++; }while(0)
#define SCB_CleanInvalidateDCache_by_Addr(a,n) do { assert(((uintptr_t)(a)&31u)==0 && n==256);cleaninvalidates++; }while(0)
static bool stuck_stream;
#define CLI_DMA_DISABLE_STREAM(s) do { if(!stuck_stream) (s)->CR &= ~DMA_SxCR_EN; }while(0)
#include "Core/Src/ext_drivers/cli.c"
static cli_device_t d;
static UART_HandleTypeDef h;
static uint32_t tick;
static const uint8_t *pending;
static uint16_t pending_n;
static unsigned char wire[50000];
static size_t wire_n;
static bool fail_start;
uint32_t osKernelGetTickCount(void){return tick;}
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *dma) {
    assert(dma->Init.Channel==DMA_CHANNEL_4);
    assert(dma->Instance==DMA1_Stream1 || dma->Instance==DMA1_Stream3);
    return HAL_OK;
}
void HAL_NVIC_SetPriority(IRQn_Type i,uint32_t p,uint32_t sub){assert(p==5);}
void HAL_NVIC_EnableIRQ(IRQn_Type i){}
void HAL_NVIC_DisableIRQ(IRQn_Type i){}
void HAL_NVIC_ClearPendingIRQ(IRQn_Type i){}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n) {
    assert(p==cli_dma_rx && n==256 && u->hdmarx->Init.Mode==DMA_CIRCULAR);
    u->RxState=HAL_UART_STATE_BUSY_RX;u->Instance->CR3|=USART_CR3_DMAR|USART_CR3_EIE;
    u->Instance->CR1|=USART_CR1_IDLEIE;rx_regs.NDTR=256;rx_regs.CR=DMA_SxCR_EN;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n) {
    if(fail_start)return HAL_BUSY;
    assert(!pending && p==cli_dma_tx && n<=64);pending=p;pending_n=n;
    u->gState=HAL_UART_STATE_BUSY_TX;tx_regs.CR=DMA_SxCR_EN;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u) {
    assert(!(tx_regs.CR&DMA_SxCR_EN));pending=NULL;pending_n=0;u->gState=HAL_UART_STATE_READY;return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) {
    assert(!(rx_regs.CR&DMA_SxCR_EN));u->RxState=HAL_UART_STATE_READY;return HAL_OK;
}
uint32_t HAL_UART_GetError(const UART_HandleTypeDef *u){return u->ErrorCode;}
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *dma){assert(dma==&cli_dma_rx_handle);cli_uart_dma_rx_event(&d);}
static void complete(void){
    if(!pending)return;
    memcpy(wire+wire_n,pending,pending_n);wire_n+=pending_n;pending=NULL;pending_n=0;
    h.gState=HAL_UART_STATE_READY;tx_regs.CR=0;cli_uart_tx_complete(&d);
}
osStatus_t osDelay(uint32_t ms){tick+=ms;complete();return osOK;}
static void reset(void){
    memset(&h,0,sizeof(h));memset(&d,0,sizeof(d));memset(&uart_regs,0,sizeof(uart_regs));
    memset(&rx_regs,0,sizeof(rx_regs));memset(&tx_regs,0,sizeof(tx_regs));
    stuck_stream=false;cli_dma_initialized=cli_dma_quarantined=false;cli_dma_rx_position=0;
    h.Instance=USART3;h.gState=h.RxState=HAL_UART_STATE_READY;
    pending=NULL;wire_n=0;tick=0;fail_start=false;scb_regs.CCR=SCB_CCR_DC_Msk;
    cli_device_init(&d,&h);assert(cli_uart_start_rx(&d)==HAL_OK);
}
static unsigned producer;
static void feed(const char *s){
    while(*s){cli_dma_rx[producer++%256]=(uint8_t)*s++;}
    rx_regs.NDTR=256-producer%256;cli_uart_dma_rx_event(&d);
}
int main(void){
    reset();producer=0;char out[CLI_LINESZ];
    feed("one\r\ntwo\r");assert(d.rx_count==2);
    cli_uart_dma_rx_event(&d);assert(d.rx_count==2); /* delayed callback no replay */
    assert(cli_take_line(&d,out) && !strcmp(out,"one"));
    assert(cli_take_line(&d,out) && !strcmp(out,"two"));
    for(unsigned i=0;i<100;i++) {feed("test\r");assert(cli_take_line(&d,out) && !strcmp(out,"test"));}
    assert(producer>256 && d.rx_count==0);
    feed("state stop\r");assert(d.stop_requested && d.report_cancel && d.rx_count==0);
    assert(invalidates>100 && cleaninvalidates);
    puts("PASS real circular RX parser: wrap, IDLE/HT duplicate suppression, stop bypass, cache alignment");
    reset();producer=0;feed("partial");dma_regs.LISR=DMA_LISR_HTIF1|DMA_LISR_TCIF1;
    cli_uart_dma_rx_event(&d); /* IDLE wins the race against delayed DMA IRQ */
    assert(d.rx_dma_overruns==1 && d.rx_discard_line && d.index==0);
    cli_uart_dma_irq(&d);dma_regs.LISR=0;
    assert(d.rx_dma_overruns==1 && d.rx_discard_line && d.index==0);
    feed("damaged\rvalid\r");assert(cli_take_line(&d,out) && !strcmp(out,"valid"));
    d.rx_dma_recover_pending=true;assert(cli_uart_service_rx(&d)==HAL_OK);
    assert(!d.rx_dma_recover_pending && d.rx_recovery_count==1 && d.rx_discard_line);
    rx_regs.CR=0;assert(cli_uart_service_rx(&d)==HAL_OK && d.rx_recovery_count==2);
    puts("PASS ambiguous delayed RX discarded through delimiter; task-side error/stopped-stream recovery");
    reset();char msg[]="123456789012345678901234567890";
    for(unsigned i=0;i<1000;i++) assert(cli_printline(&d,msg)==HAL_OK);
    while(pending)complete();
    assert(wire_n==32000 && d.tx_bytes==wire_n && cleans>0);
    for(unsigned i=0;i<32000;i+=32)assert(!memcmp(wire+i,msg,30) && wire[i+30]=='\r' && wire[i+31]=='\n');
    assert(cli_printline(&d,msg)==HAL_OK);tick+=251;cli_uart_service_tx(&d);
    assert(!pending && !d.tx_used && d.tx_aborts==1);
    /* A late completion after abort cannot consume queue data. */
    cli_uart_tx_complete(&d);assert(d.tx_used==0);
    cli_dma_quarantined=true;assert(cli_printline(&d,msg)==HAL_BUSY); /* canceled report */
    puts("PASS real DMA TX queue: 32KB ordering, wrap/backpressure, finite stall shutdown and late callback");
    reset();rx_regs.CR=DMA_SxCR_EN;stuck_stream=true;d.rx_dma_recover_pending=true;
    assert(cli_uart_service_rx(&d)==HAL_ERROR && cli_dma_quarantined);
    assert(cli_dma_tx_start(&d,(const uint8_t *)msg,30)==HAL_BUSY);
    puts("PASS failed bounded stream shutdown quarantines DMA and forbids buffer reuse");
    return 0;
}
