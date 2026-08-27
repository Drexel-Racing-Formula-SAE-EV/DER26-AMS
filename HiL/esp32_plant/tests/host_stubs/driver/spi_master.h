#ifndef HOST_STUB_DRIVER_SPI_MASTER_H_
#define HOST_STUB_DRIVER_SPI_MASTER_H_

#include <stddef.h>

#include "esp_err.h"

#define SPI2_HOST 2
#define SPI_DMA_CH_AUTO 0

typedef void *spi_device_handle_t;

typedef struct {
    int mosi_io_num;
    int miso_io_num;
    int sclk_io_num;
    int quadwp_io_num;
    int quadhd_io_num;
} spi_bus_config_t;

typedef struct {
    int clock_speed_hz;
    int mode;
    int spics_io_num;
    int queue_size;
} spi_device_interface_config_t;

typedef struct {
    size_t length;
    const void *tx_buffer;
    void *rx_buffer;
} spi_transaction_t;

esp_err_t spi_bus_initialize(
    int host,
    const spi_bus_config_t *configuration,
    int dma_channel);
esp_err_t spi_bus_add_device(
    int host,
    const spi_device_interface_config_t *configuration,
    spi_device_handle_t *device);
esp_err_t spi_device_transmit(
    spi_device_handle_t device,
    spi_transaction_t *transaction);

#endif /* HOST_STUB_DRIVER_SPI_MASTER_H_ */
