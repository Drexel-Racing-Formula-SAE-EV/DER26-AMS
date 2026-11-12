/* Real 6830 leaf paths + real DMA transport; synthetic MMIO/IRQ/packet engine. */
#define main unused_wake_fixture_main
#include "adbms_wake_policy_test.c"
#undef main
#include "FreeRTOS.h"
static DMA_TypeDef dma_regs;
static DMA_Stream_TypeDef dma_tx_regs,dma_rx_regs;
#undef DMA2
#undef DMA2_Stream5
#undef DMA2_Stream6
#define DMA2 (&dma_regs)
#define DMA2_Stream5 (&dma_tx_regs)
#define DMA2_Stream6 (&dma_rx_regs)
#define __DSB() ((void)0)
#define __DMB() ((void)0)
#define __get_IPSR() 0
#define __get_PRIMASK() 0
#define __get_BASEPRI() 0
#define SCB_CleanDCache_by_Addr(a,b) ((void)0)
#define SCB_CleanInvalidateDCache_by_Addr(a,b) ((void)0)
#define SCB_InvalidateDCache_by_Addr(a,b) ((void)0)
#undef SCB
static SCB_Type scb_regs;
#define SCB (&scb_regs)
#undef __HAL_RCC_DMA2_CLK_ENABLE
#define __HAL_RCC_DMA2_CLK_ENABLE() ((void)0)
#include "Core/Src/ext_drivers/ams_spi6_transport.c"
void HAL_NVIC_SetPriority(IRQn_Type n,uint32_t p,uint32_t sub) { CHECK(p>=5); }
void HAL_NVIC_EnableIRQ(IRQn_Type n) {}
void HAL_NVIC_ClearPendingIRQ(IRQn_Type n) {}
BaseType_t xTaskGetSchedulerState(void) { return taskSCHEDULER_RUNNING; }
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *s) { s->token=0; return s; }
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t s,BaseType_t *higher) { s->token=1; return pdTRUE; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t s,TickType_t ticks) {
    if(s->token) { s->token=0; return pdTRUE; }
    if(!ticks) return pdFALSE;
    CHECK(active && test_lock_depth>0 && last_cs==GPIO_PIN_RESET);
    HAL_StatusTypeDef rc=HAL_SPI_TransmitReceive(&spi,dma_tx,dma_rx,(uint16_t)dma_rx_regs.NDTR,ticks);
    dma_rx_regs.NDTR=dma_tx_regs.NDTR=0;
    dma_regs.HISR=(rc==HAL_OK) ? DONE_FLAGS : DMA_HISR_TEIF6;
    ams_spi6_dma_irq(); dma_regs.HISR=0;
    return pdTRUE;
}
int main(void) {
    ams_spi6_dma_init();
    const uint32_t rates[]={SPI_BAUDRATEPRESCALER_64,SPI_BAUDRATEPRESCALER_128};
    for(unsigned i=0;i<2;i++) {
        reset_dev(5); spi.Init.Mode=SPI_MODE_MASTER;
        spi.Init.Direction=SPI_DIRECTION_2LINES;
        spi.Init.BaudRatePrescaler=rates[i]; spi_regs.CR1=rates[i]|SPI_CR1_SPE;
        read_clean();
        ams_spi6_dma_stats_t before=ams_spi6_dma_stats();
        CHECK(before.completed>0);
        CHECK(adbms6830_spi_write(&dev,STCOMM,sizeof(STCOMM),1)==HAL_OK);
        adbms6830_note_counter_increment(&dev); /* production mux caller owns this */
        ams_spi6_dma_stats_t after=ams_spi6_dma_stats();
        CHECK(after.started==before.started && after.polling==before.polling+1);
        CHECK(spi.Init.BaudRatePrescaler==rates[i]);
        CHECK((spi_regs.CR1 & SPI_CR1_BR)==rates[i]);
        read_clean();
        fail_spi=true;
        CHECK(adbms6830_spi_write(&dev,STCOMM,sizeof(STCOMM),1)==HAL_TIMEOUT);
        CHECK(spi.Init.BaudRatePrescaler==rates[i] && spi.State==HAL_SPI_STATE_READY);
        fail_spi=false;
        read_clean();
        CHECK(test_lock_depth==0 && last_cs==GPIO_PIN_SET && !active);
    }
    puts("PASS integrated fast DMA reads / slow polling mux / fast DMA reads, including mux timeout recovery");
    return failures ? 1 : 0;
}
