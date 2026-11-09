/* Exclusive SPI6 DMA2 stream 5 TX / stream 6 RX, channel 1 (RM0410).
 * Called under the existing recursive SPI mutex, including CS ownership.
 * No HAL SPI DMA callbacks: their completion paths can poll in interrupt context.
 * DMA IRQ only records flags and releases a static semaphore. */
#include "ext_drivers/ams_spi6_transport.h"
#if AMS_ADBMS_SPI_DMA
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdbool.h>
#if AMS_BUILD_PROFILE != AMS_PROFILE_TESTDAY
#error "SPI DMA is a bench-only candidate; use vehicle comms profile 4"
#endif
#define TX_FLAGS (DMA_HISR_FEIF5|DMA_HISR_DMEIF5|DMA_HISR_TEIF5|DMA_HISR_HTIF5|DMA_HISR_TCIF5)
#define RX_FLAGS (DMA_HISR_FEIF6|DMA_HISR_DMEIF6|DMA_HISR_TEIF6|DMA_HISR_HTIF6|DMA_HISR_TCIF6)
#define ERR_FLAGS (DMA_HISR_FEIF5|DMA_HISR_DMEIF5|DMA_HISR_TEIF5|DMA_HISR_FEIF6|DMA_HISR_DMEIF6|DMA_HISR_TEIF6)
#define DONE_FLAGS (DMA_HISR_TCIF5|DMA_HISR_TCIF6)
/* Dedicated SRAM, whole cache lines. Linker excludes this bank from heap/stack. */
static uint8_t dma_tx[512] __attribute__((section(".spi_dma"),aligned(32)));
static uint8_t dma_rx[512] __attribute__((section(".spi_dma"),aligned(32)));
static StaticSemaphore_t completion_storage;
static SemaphoreHandle_t completion;
static volatile uint32_t events;
static volatile bool active;
static bool quarantined;
static ams_spi6_dma_stats_t stats;

ams_spi6_dma_stats_t ams_spi6_dma_stats(void)
{
    taskENTER_CRITICAL();
    ams_spi6_dma_stats_t copy=stats;
    copy.quarantined=quarantined;
    taskEXIT_CRITICAL();
    return copy;
}

void ams_spi6_dma_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    completion = xSemaphoreCreateBinaryStatic(&completion_storage);
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn,6,0);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn,6,0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

void ams_spi6_dma_irq(void)
{
    uint32_t flags = DMA2->HISR & (TX_FLAGS|RX_FLAGS);
    DMA2->HIFCR = flags;
    if(active && flags) {
        events |= flags;
        if(flags & ERR_FLAGS) SPI6->CR2 &= ~(SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN);
        BaseType_t higher = pdFALSE;
        xSemaphoreGiveFromISR(completion,&higher);
        portYIELD_FROM_ISR(higher);
    }
}

/* Finite stream shutdown: never return buffers to service while EN remains set.
 * Quarantine persists until MCU reset rather than resetting unrelated DMA2 users. */
static bool stop_streams(void)
{
    SPI6->CR2 &= ~(SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN);
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    for(unsigned i=0;i<1024;i++)
        if(!((DMA2_Stream5->CR|DMA2_Stream6->CR)&DMA_SxCR_EN)) return true;
    quarantined=true;
    return false;
}

HAL_StatusTypeDef ams_spi6_transfer(SPI_HandleTypeDef *h, uint8_t *tx,
    uint8_t *rx, uint16_t n, uint32_t timeout)
{
    if(!h || !tx || !n || n>sizeof(dma_tx)) return HAL_ERROR;
    if(h->Instance==SPI6 && (quarantined || active)) return HAL_ERROR;
    /* Four-byte commands and pre-kernel calls retain the proven polling path. */
    if(h->Instance!=SPI6 || n<16 || xTaskGetSchedulerState()!=taskSCHEDULER_RUNNING) {
        stats.polling++;
        return rx ? HAL_SPI_TransmitReceive(h,tx,rx,n,timeout) : HAL_SPI_Transmit(h,tx,n,timeout);
    }
    if(!completion || quarantined || active || h->State!=HAL_SPI_STATE_READY ||
       __get_IPSR()!=0 || __get_PRIMASK()!=0 || __get_BASEPRI()!=0) return HAL_ERROR;
    if(h->Init.DataSize!=SPI_DATASIZE_8BIT ||
       h->Init.Direction!=SPI_DIRECTION_2LINES || h->Init.Mode!=SPI_MODE_MASTER) return HAL_ERROR;
    if(!stop_streams()) return HAL_ERROR;
    memcpy(dma_tx,tx,n);
    memset(dma_rx,0,n);
    if(SCB->CCR & SCB_CCR_DC_Msk) {
        SCB_CleanDCache_by_Addr((uint32_t*)dma_tx,512);
        SCB_CleanInvalidateDCache_by_Addr((uint32_t*)dma_rx,512);
    }
    __DSB();
    while(xSemaphoreTake(completion,0)==pdTRUE) {}
    DMA2->HIFCR=TX_FLAGS|RX_FLAGS;
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream6_IRQn);
    uint32_t cr1=SPI6->CR1, cr2=SPI6->CR2;
    /* Full duplex byte transfers; RX drains the FIFO even for writes. */
    DMA2_Stream5->CR=(1u<<DMA_SxCR_CHSEL_Pos)|DMA_SxCR_DIR_0|DMA_SxCR_MINC|DMA_SxCR_TCIE|DMA_SxCR_TEIE|DMA_SxCR_DMEIE;
    DMA2_Stream6->CR=(1u<<DMA_SxCR_CHSEL_Pos)|DMA_SxCR_MINC|DMA_SxCR_TCIE|DMA_SxCR_TEIE|DMA_SxCR_DMEIE;
    DMA2_Stream5->FCR=0; DMA2_Stream6->FCR=0;
    DMA2_Stream5->PAR=(uint32_t)&SPI6->DR; DMA2_Stream6->PAR=(uint32_t)&SPI6->DR;
    DMA2_Stream5->M0AR=(uint32_t)dma_tx; DMA2_Stream6->M0AR=(uint32_t)dma_rx;
    DMA2_Stream5->NDTR=n; DMA2_Stream6->NDTR=n;
    events=0; active=true; stats.started++; h->State=HAL_SPI_STATE_BUSY_TX_RX;
    __DMB();
    uint32_t start=HAL_GetTick();
    SPI6->CR2=cr2|SPI_CR2_FRXTH;
    SPI6->CR1=cr1|SPI_CR1_SPE;
    DMA2_Stream6->CR |= DMA_SxCR_EN;
    DMA2_Stream5->CR |= DMA_SxCR_EN;
    SPI6->CR2 |= SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN;
    HAL_StatusTypeDef result=HAL_TIMEOUT;
    for(;;) {
        uint32_t e=events;
        if(e & ERR_FLAGS) { result=HAL_ERROR; break; }
        if((e & DONE_FLAGS)==DONE_FLAGS) { result=HAL_OK; break; }
        uint32_t elapsed=HAL_GetTick()-start;
        if(elapsed>=timeout) break;
        TickType_t ticks=pdMS_TO_TICKS(timeout-elapsed)+1;
        (void)xSemaphoreTake(completion,ticks);
    }
    /* RX TC follows the final received byte, but BSY may lag a few cycles.
     * Bounded task-side tail; never the HAL 100 ms IRQ polling loop. */
    if(result==HAL_OK) {
        unsigned i;
        for(i=0;i<1024 && (SPI6->SR & SPI_SR_BSY);i++) {}
        if((SPI6->SR & (SPI_SR_BSY|SPI_SR_OVR|SPI_SR_MODF))!=0) result=HAL_ERROR;
    }
    taskENTER_CRITICAL();
    active=false;
    if((events & ERR_FLAGS) || (DMA2->HISR & ERR_FLAGS) ||
       (result==HAL_OK && (DMA2_Stream5->NDTR || DMA2_Stream6->NDTR))) result=HAL_ERROR;
    bool stopped=stop_streams();
    DMA2->HIFCR=TX_FLAGS|RX_FLAGS;
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream6_IRQn);
    taskEXIT_CRITICAL();
    if(!stopped) result=HAL_ERROR;
    if(result!=HAL_OK) {
        __HAL_RCC_SPI6_FORCE_RESET(); __DSB(); __HAL_RCC_SPI6_RELEASE_RESET();
        SPI6->CR1=cr1 & ~SPI_CR1_SPE;
    }
    SPI6->CR2=cr2 & ~(SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN);
    h->State=HAL_SPI_STATE_READY;
    if(result==HAL_OK) stats.completed++;
    else if(result==HAL_TIMEOUT) stats.timeouts++;
    else stats.errors++;
    if(result==HAL_OK && rx) {
        __DSB();
        if(SCB->CCR & SCB_CCR_DC_Msk) SCB_InvalidateDCache_by_Addr((uint32_t*)dma_rx,512);
        memcpy(rx,dma_rx,n);
    }
    return result;
}
#endif
