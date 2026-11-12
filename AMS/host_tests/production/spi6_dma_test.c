/* Execute actual transport logic against fake MMIO and scheduler/IRQ events. */
#include <assert.h>
#include <stdio.h>
#include "ext_drivers/ams_spi6_transport.h"
#include "FreeRTOS.h"
static SPI_TypeDef spi_regs;
static DMA_TypeDef dma_regs;
static DMA_Stream_TypeDef tx_regs,rx_regs;
#undef SPI6
#undef DMA2
#undef DMA2_Stream5
#undef DMA2_Stream6
#define SPI6 (&spi_regs)
#define DMA2 (&dma_regs)
#define DMA2_Stream5 (&tx_regs)
#define DMA2_Stream6 (&rx_regs)
#define __DSB() ((void)0)
#define __DMB() ((void)0)
#define __get_IPSR() 0
#define __get_PRIMASK() 0
#define __get_BASEPRI() 0
static unsigned cache_clean,cache_clean_invalidate,cache_invalidate;
#define CACHE_CHECK(a,b) do { assert(((uintptr_t)(a)&31u)==0 && (b)==512); } while(0)
#define SCB_CleanDCache_by_Addr(a,b) do { CACHE_CHECK(a,b); cache_clean++; } while(0)
#define SCB_CleanInvalidateDCache_by_Addr(a,b) do { CACHE_CHECK(a,b); cache_clean_invalidate++; } while(0)
#define SCB_InvalidateDCache_by_Addr(a,b) do { CACHE_CHECK(a,b); cache_invalidate++; } while(0)
#undef __HAL_RCC_DMA2_CLK_ENABLE
#undef __HAL_RCC_SPI6_FORCE_RESET
#undef __HAL_RCC_SPI6_RELEASE_RESET
#define __HAL_RCC_DMA2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI6_FORCE_RESET() ((void)0)
#define __HAL_RCC_SPI6_RELEASE_RESET() ((void)0)

/* Avoid host dereference of Cortex system registers. */
#undef SCB
#define SCB fake_scb
static SCB_Type fake_scb_storage;
static SCB_Type *fake_scb=&fake_scb_storage;
#include "Core/Src/ext_drivers/ams_spi6_transport.c"
static unsigned now,polls,blocks,mode;
static BaseType_t scheduler_state=taskSCHEDULER_RUNNING;
uint32_t HAL_GetTick(void) { return now; }
void HAL_NVIC_SetPriority(IRQn_Type n,uint32_t p,uint32_t sub) { assert(p>=5); }
void HAL_NVIC_EnableIRQ(IRQn_Type n) {}
void HAL_NVIC_ClearPendingIRQ(IRQn_Type n) {}
BaseType_t xTaskGetSchedulerState(void) { return scheduler_state; }
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *s) { s->token=0; return s; }
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t s,BaseType_t *higher) { s->token=1; return pdTRUE; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t s,TickType_t ticks) {
    if(s->token) { s->token=0; return pdTRUE; }
    if(!ticks) return pdFALSE;
    blocks++;
    assert(active && (SPI6->CR2 & SPI_CR2_RXDMAEN));
    if(mode==1) { now+=ticks; return pdFALSE; }
    DMA2->HISR=(mode==2) ? DMA_HISR_TEIF6 : DONE_FLAGS;
    if(mode==5 && tx_regs.NDTR) { DMA2->HISR=DMA_HISR_TCIF5; tx_regs.NDTR=0; }
    else { tx_regs.NDTR=rx_regs.NDTR=0; }
    if(mode==3) rx_regs.NDTR=1;
    if(mode==4) spi_regs.SR=SPI_SR_BSY;
    memset(dma_rx,0x5a,sizeof(dma_rx));
    ams_spi6_dma_irq();
    /* Real hardware clears HISR by the W1C HIFCR write. */
    DMA2->HISR=0;
    return pdTRUE;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*h,uint8_t*t,uint16_t n,uint32_t ms) { polls++; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*h,uint8_t*t,uint8_t*r,uint16_t n,uint32_t ms) { polls++; return HAL_OK; }
int main(void) {
    SPI_HandleTypeDef h={0}; h.Instance=SPI6; h.State=HAL_SPI_STATE_READY;
    h.Init.DataSize=SPI_DATASIZE_8BIT;h.Init.Direction=SPI_DIRECTION_2LINES;h.Init.Mode=SPI_MODE_MASTER;
    uint8_t tx[44]={0},rx[44]={0};
    ams_spi6_dma_init();
    assert(ams_spi6_transfer(&h,tx,rx,4,3)==HAL_OK && polls==1);
    assert(ams_spi6_transfer(&h,tx,rx,44,3)==HAL_OK && blocks==1 && rx[0]==0x5a);
    mode=1; memset(rx,0xa5,sizeof(rx));
    assert(ams_spi6_transfer(&h,tx,rx,44,3)==HAL_TIMEOUT);
    assert(!active && h.State==HAL_SPI_STATE_READY && rx[0]==0xa5);
    assert(!(tx_regs.CR & DMA_SxCR_EN) && !(rx_regs.CR & DMA_SxCR_EN));
    /* A late interrupt after cleanup cannot authorize a subsequent transfer. */
    DMA2->HISR=DONE_FLAGS; ams_spi6_dma_irq(); DMA2->HISR=0;
    mode=2; assert(ams_spi6_transfer(&h,tx,rx,44,3)==HAL_ERROR);
    mode=0; assert(ams_spi6_transfer(&h,tx,rx,44,3)==HAL_OK);
    assert(ams_spi6_transfer(&h,tx,NULL,44,3)==HAL_OK);
    mode=3; assert(ams_spi6_transfer(&h,tx,rx,44,3)==HAL_ERROR);
    mode=4; assert(ams_spi6_transfer(&h,tx,rx,44,3)==HAL_ERROR);
    spi_regs.SR=0;
    mode=5; assert(ams_spi6_transfer(&h,tx,rx,44,3)==HAL_OK);
    scheduler_state=0; assert(ams_spi6_transfer(&h,tx,rx,44,3)==HAL_OK);
    scheduler_state=taskSCHEDULER_RUNNING;
    active=true; assert(ams_spi6_transfer(&h,tx,rx,4,3)==HAL_ERROR); active=false;
    ams_spi6_dma_stats_t st=ams_spi6_dma_stats();
    assert(st.started==8 && st.completed==4 && st.errors==3 && st.timeouts==1);
    mode=0; fake_scb->CCR=SCB_CCR_DC_Msk;
    assert(ams_spi6_transfer(&h,tx,rx,44,3)==HAL_OK);
    assert(cache_clean==1 && cache_clean_invalidate==1 && cache_invalidate==1);
    quarantined=true; assert(ams_spi6_transfer(&h,tx,rx,4,3)==HAL_ERROR);
    puts("PASS DMA success, suspension, timeout, error, late/split IRQ, reuse, writes, NDTR, BSY, pre-kernel fallback, counters and quarantine");
}
