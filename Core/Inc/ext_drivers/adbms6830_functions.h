/*
 * adbms6830_functions.h
 *
 *  Created on: May 13, 2025
 *      Author: realb
 */

#ifndef INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_
#define INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_

#include <stdint.h>
#include "ext_drivers/adbms6830_data.h"

void adBms6830_init(adbms6830_driver_t* dev,
                    uint8_t num_ics,
                    adbms6830_asic* ics,
                    SPI_HandleTypeDef* hspi,
                    GPIO_TypeDef* cs_port_a,
                    GPIO_TypeDef* cs_port_b,
                    uint16_t cs_pin_a,
                    uint16_t cs_pin_b);

void adbms6830_reset_cfg(adbms6830_driver_t *dev);
void adbms6830_srst(adbms6830_driver_t *dev);
void adbms6830_wrcfga(adbms6830_driver_t *dev);
void adbms6830_wrcfgb(adbms6830_driver_t *dev);
void adbms6830_rdcfga(adbms6830_driver_t *dev);
void adbms6830_rdcfgb(adbms6830_driver_t *dev);
void adbms6830_adcv(adbms6830_driver_t *dev, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs);

void adbms6830_wakeup(adbms6830_driver_t* dev);

void adbms6830_us_delay(adbms6830_driver_t* dev, uint16_t microseconds);

#endif /* INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_ */
