#ifndef AMS_SPI6_TRANSPORT_H
#define AMS_SPI6_TRANSPORT_H
#include "main.h"
#include "ams_build_profile.h"
#ifndef AMS_ADBMS_SPI_DMA
#define AMS_ADBMS_SPI_DMA 0
#endif
typedef struct {
    uint32_t started, completed, errors, timeouts, polling, quarantined;
} ams_spi6_dma_stats_t;
#if AMS_ADBMS_SPI_DMA
ams_spi6_dma_stats_t ams_spi6_dma_stats(void);
void ams_spi6_dma_init(void);
void ams_spi6_dma_irq(void);
HAL_StatusTypeDef ams_spi6_transfer(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
#else
static inline ams_spi6_dma_stats_t ams_spi6_dma_stats(void)
{
    ams_spi6_dma_stats_t empty={0}; return empty;
}
static inline HAL_StatusTypeDef ams_spi6_transfer(SPI_HandleTypeDef *h,
    uint8_t *tx, uint8_t *rx, uint16_t n, uint32_t timeout)
{
    return rx ? HAL_SPI_TransmitReceive(h,tx,rx,n,timeout) : HAL_SPI_Transmit(h,tx,n,timeout);
}
#endif
#endif
