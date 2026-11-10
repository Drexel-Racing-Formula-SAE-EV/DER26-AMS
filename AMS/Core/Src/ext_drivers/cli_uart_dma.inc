/* Private implementation included by cli.c: one USART3 owner, DMA1 CH4.
 * RX circular 256B, HT/TC + IDLE; TX keeps the existing 64B cancellation bound.
 * DMA buffers occupy dedicated, whole cache lines in the existing SRAM bank.
 * No CPU writes to RX after start; invalidate before every CPU consumption.
 */
#define CLI_DMA_RX_BYTES 256u
#ifndef CLI_DMA_DISABLE_STREAM
#define CLI_DMA_DISABLE_STREAM(stream) ((stream)->CR &= ~DMA_SxCR_EN)
#endif
static HAL_StatusTypeDef cli_dma_stop(bool tx);
static uint8_t cli_dma_rx[CLI_DMA_RX_BYTES] __attribute__((section(".spi_dma"),aligned(32)));
static uint8_t cli_dma_tx[64] __attribute__((section(".spi_dma"),aligned(32)));
static DMA_HandleTypeDef cli_dma_rx_handle, cli_dma_tx_handle;
static bool cli_dma_initialized, cli_dma_quarantined, cli_dma_overrun_latched;
static uint16_t cli_dma_rx_position;
static UART_HandleTypeDef *cli_dma_owner;

static void cli_dma_invalidate_rx(void)
{
    if(SCB->CCR & SCB_CCR_DC_Msk)
        SCB_InvalidateDCache_by_Addr((uint32_t *)cli_dma_rx,sizeof(cli_dma_rx));
    __DMB();
}
static HAL_StatusTypeDef cli_dma_init(cli_device_t *d)
{
    if(cli_dma_quarantined) return HAL_ERROR;
    if(d->huart->Instance!=USART3) return HAL_ERROR;
    if(cli_dma_initialized) return d->huart==cli_dma_owner ? HAL_OK : HAL_ERROR;
    __HAL_RCC_DMA1_CLK_ENABLE();
    if(cli_dma_stop(true)!=HAL_OK || cli_dma_stop(false)!=HAL_OK) return HAL_ERROR;
    cli_dma_rx_handle.Instance=DMA1_Stream1;
    cli_dma_rx_handle.Init.Channel=DMA_CHANNEL_4;
    cli_dma_rx_handle.Init.Direction=DMA_PERIPH_TO_MEMORY;
    cli_dma_rx_handle.Init.PeriphInc=DMA_PINC_DISABLE;
    cli_dma_rx_handle.Init.MemInc=DMA_MINC_ENABLE;
    cli_dma_rx_handle.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
    cli_dma_rx_handle.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
    cli_dma_rx_handle.Init.Mode=DMA_CIRCULAR;
    cli_dma_rx_handle.Init.Priority=DMA_PRIORITY_HIGH;
    cli_dma_rx_handle.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
    cli_dma_tx_handle=cli_dma_rx_handle;
    cli_dma_tx_handle.Instance=DMA1_Stream3;
    cli_dma_tx_handle.Init.Direction=DMA_MEMORY_TO_PERIPH;
    cli_dma_tx_handle.Init.Mode=DMA_NORMAL;
    cli_dma_tx_handle.Init.Priority=DMA_PRIORITY_LOW;
    if(HAL_DMA_Init(&cli_dma_rx_handle)!=HAL_OK ||
       HAL_DMA_Init(&cli_dma_tx_handle)!=HAL_OK) return HAL_ERROR;
    __HAL_LINKDMA(d->huart,hdmarx,cli_dma_rx_handle);
    __HAL_LINKDMA(d->huart,hdmatx,cli_dma_tx_handle);
    /* Same preemption priority as USART3: cursor/parser cannot re-enter. */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn,5,0);
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn,5,0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    cli_dma_owner=d->huart;cli_dma_initialized=true;
    return HAL_OK;
}
static HAL_StatusTypeDef cli_dma_rx_start(cli_device_t *d)
{
    if(cli_dma_init(d)!=HAL_OK) return HAL_ERROR;
    if(d->huart->RxState!=HAL_UART_STATE_READY) return HAL_BUSY;
    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    __HAL_DMA_CLEAR_FLAG(&cli_dma_rx_handle,DMA_FLAG_FEIF1_5|DMA_FLAG_DMEIF1_5|
        DMA_FLAG_TEIF1_5|DMA_FLAG_HTIF1_5|DMA_FLAG_TCIF1_5);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);
    cli_dma_rx_position=0;cli_dma_overrun_latched=false;
    /* Clean only while DMA is stopped; no dirty line can later overwrite RX. */
    if(SCB->CCR & SCB_CCR_DC_Msk)
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)cli_dma_rx,sizeof(cli_dma_rx));
    HAL_StatusTypeDef result=HAL_UARTEx_ReceiveToIdle_DMA(d->huart,cli_dma_rx,sizeof(cli_dma_rx));
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    return result;
}
static HAL_StatusTypeDef cli_dma_tx_start(cli_device_t *d,const uint8_t *data,uint16_t n)
{
    if(cli_dma_init(d)!=HAL_OK || n>sizeof(cli_dma_tx) ||
       d->huart->gState!=HAL_UART_STATE_READY) return HAL_BUSY;
    memcpy(cli_dma_tx,data,n);
    if(SCB->CCR & SCB_CCR_DC_Msk)
        SCB_CleanDCache_by_Addr((uint32_t *)cli_dma_tx,sizeof(cli_dma_tx));
    __DMB();
    return HAL_UART_Transmit_DMA(d->huart,cli_dma_tx,n);
}
/* Check from BOTH IDLE and DMA entry points. An IDLE IRQ can beat a delayed
 * DMA IRQ; it must not parse bytes before the DMA overrun is recognized. */
static bool cli_dma_rx_ambiguous(cli_device_t *d,uint32_t remaining)
{
    uint32_t flags=DMA1->LISR;
    bool ambiguous=(flags & (DMA_LISR_HTIF1|DMA_LISR_TCIF1))==
                           (DMA_LISR_HTIF1|DMA_LISR_TCIF1);
    if(flags & (DMA_LISR_TEIF1|DMA_LISR_DMEIF1|DMA_LISR_FEIF1)) {
        d->rx_dma_recover_pending=true;ambiguous=true;
    }
    if(ambiguous) {
        if(!cli_dma_overrun_latched) {d->rx_dma_overruns++;d->rx_drop_count++;}
        cli_dma_overrun_latched=true;d->index=0;d->rx_discard_line=true;
        if(remaining<=CLI_DMA_RX_BYTES)
            cli_dma_rx_position=(uint16_t)((CLI_DMA_RX_BYTES-remaining)%CLI_DMA_RX_BYTES);
    } else cli_dma_overrun_latched=false;
    return ambiguous;
}
void cli_uart_dma_rx_event(cli_device_t *d)
{
    if(!d || !d->huart || d->rx_dma_recover_pending ||
       d->huart->RxState!=HAL_UART_STATE_BUSY_RX) return;
    uint32_t remaining=__HAL_DMA_GET_COUNTER(&cli_dma_rx_handle);
    if(remaining>CLI_DMA_RX_BYTES) {
        d->rx_dma_recover_pending=true;d->rx_discard_line=true;d->index=0;return;
    }
    uint16_t position=(uint16_t)((CLI_DMA_RX_BYTES-remaining)%CLI_DMA_RX_BYTES);
    /* Live NDTR, not callback Size: a delayed HT callback after IDLE must not
     * rewind the cursor and replay already parsed input. */
    cli_dma_invalidate_rx();
    if(cli_dma_rx_ambiguous(d,remaining)) return;
    for(unsigned count=0;cli_dma_rx_position!=position && count<CLI_DMA_RX_BYTES;count++) {
        uint8_t c=cli_dma_rx[cli_dma_rx_position];
        cli_dma_rx_position=(cli_dma_rx_position+1u)%CLI_DMA_RX_BYTES;
        cli_rx_byte(d,c);
    }
    d->rx_dma_events++;
}
void cli_uart_dma_irq(cli_device_t *d)
{
    uint32_t flags=DMA1->LISR;
    (void)cli_dma_rx_ambiguous(d,__HAL_DMA_GET_COUNTER(&cli_dma_rx_handle));
    if(flags & (DMA_LISR_TEIF1|DMA_LISR_DMEIF1|DMA_LISR_FEIF1)) {
        if(cli_dma_stop(false)!=HAL_OK) {
            HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);return;
        }
    }
    HAL_DMA_IRQHandler(&cli_dma_rx_handle);
}

/* Avoid HAL's tick-based DMA abort loop while CLI critical sections mask the
 * tick. A failed finite shutdown permanently quarantines these buffers. */
static HAL_StatusTypeDef cli_dma_stop(bool tx)
{
    DMA_Stream_TypeDef *stream=tx ? DMA1_Stream3 : DMA1_Stream1;
    CLI_DMA_DISABLE_STREAM(stream);
    for(unsigned i=0;i<1024;i++) if(!(stream->CR & DMA_SxCR_EN)) return HAL_OK;
    cli_dma_quarantined=true;
    return HAL_ERROR;
}

void cli_uart_dma_tx_irq(cli_device_t *d)
{
    if(DMA1->LISR & (DMA_LISR_TEIF3|DMA_LISR_DMEIF3|DMA_LISR_FEIF3)) {
        d->report_cancel=true;
        if(cli_dma_stop(true)!=HAL_OK) {
            HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);return;
        }
    }
    HAL_DMA_IRQHandler(&cli_dma_tx_handle);
}
