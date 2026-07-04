/*
 * adbms6830_functions.h
 *
 *  Created on: May 13, 2025
 *      Author: realb
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_
#define INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_

#include <stdint.h>
#include <stdbool.h>
#include "ext_drivers/adbms6830_data.h"

void adBms6830_init(adbms6830_driver_t* dev,
				    uint8_t num_ics,
				    adbms6830_asic* ics,
				    SPI_HandleTypeDef* hspi,
				    GPIO_TypeDef* cs_port_a,
				    GPIO_TypeDef* cs_port_b,
				    uint16_t cs_pin_a,
				    uint16_t cs_pin_b,
					TIM_HandleTypeDef *htim);

void adbms6830_reset_cfg(adbms6830_driver_t *dev);
void adbms6830_srst(adbms6830_driver_t *dev);
void adbms6830_wrcfga(adbms6830_driver_t *dev);
void adbms6830_wrcfgb(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_wrcfgb_checked(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_wrpwma_checked(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_wrpwmb_checked(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_write_pwm_checked(adbms6830_driver_t *dev);
void adbms6830_rdcfga(adbms6830_driver_t *dev);
void adbms6830_rdcfgb(adbms6830_driver_t *dev);
void adbms6830_adcv(adbms6830_driver_t *dev, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs);

void adbms6830_wakeup(adbms6830_driver_t* dev);
void adbms6830_wakeup_cold(adbms6830_driver_t* dev);

void adbms6830_us_delay(adbms6830_driver_t* dev, uint16_t microseconds);

void adbms6830_adcv(adbms6830_driver_t *dev, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs);
void adbms6830_start_adc_cell_voltage_measurement(adbms6830_driver_t *dev);
void adbms6830_parse_cell(adbms6830_driver_t *dev, uint8_t *data, GRP grp);
void adbms6830_read_cell_voltages(adbms6830_driver_t *dev);


void adbms6830_spi_debug_enable(adbms6830_driver_t *dev, bool enable);
void adbms6830_spi_debug_clear(adbms6830_driver_t *dev);
const adbms6830_spi_debug_t *adbms6830_spi_debug_get(const adbms6830_driver_t *dev);
const char *adbms6830_spi_op_str(adbms6830_spi_op_t op);
HAL_StatusTypeDef adbms6830_spi_probe_rdcfga(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_spi_probe_rdcfga_on_string(adbms6830_driver_t *dev, adbms_string string);
HAL_StatusTypeDef adbms6830_read_sid(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_read_status(adbms6830_driver_t *dev, bool inject_spiflt);
HAL_StatusTypeDef adbms6830_clear_all_flags(adbms6830_driver_t *dev);
const adbms6830_diag_health_t *adbms6830_diag_health_get(const adbms6830_driver_t *dev);
void adbms6830_diag_health_clear(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_verify_config_readback(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_run_cell_adc_self_test(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_run_open_wire_check(adbms6830_driver_t *dev, bool odd_channels);
HAL_StatusTypeDef adbms6830_run_aux_gpio_diagnostic(adbms6830_driver_t *dev);

///* I2C COMM primitives */
//void adbms6830_i2c_write(adbms6830_driver_t *dev, uint8_t slave_addr, uint8_t data);
//void adbms6830_i2c_read(adbms6830_driver_t *dev, uint8_t slave_addr, uint8_t *rx_data);
//
///* ADG728 mux control */
//void adbms6830_mux_write(adbms6830_driver_t *dev, uint8_t slave_addr, uint8_t switch_mask);
//void adbms6830_mux_select_channel(adbms6830_driver_t *dev, uint8_t slave_addr, uint8_t channel);
//void adbms6830_mux_disable_all(adbms6830_driver_t *dev);
//
///* Aux voltage read — reads RDAUXA into dev->ics[].aux.a_codes[] */
//void adbms6830_adax(adbms6830_driver_t *dev, uint8_t gpio_ch);
//void adbms6830_parse_aux(adbms6830_driver_t *dev, uint8_t *data);
//void adbms6830_read_aux_voltage(adbms6830_driver_t *dev, uint8_t gpio_ch);
//
///* Temperature read — selects mux channel, triggers ADAX, returns raw aux code */
//void adbms6830_read_temp_raw(adbms6830_driver_t *dev, uint8_t sensor_num, int16_t *out);
//void adbms6830_read_all_temps(adbms6830_driver_t *dev, uint8_t ic_num, uint8_t num_temps);
//float adbms6830_convert_temp(adbms6830_driver_t *dev, uint8_t ic_num, uint8_t sensor_num, float vntc);

//int  mux_read_gpio_voltage(adbms6830_driver_t *dev, uint8_t ic_idx, uint8_t sensor_num);
int mux_read_gpio_voltage(adbms6830_driver_t *dev, uint8_t sensor_num);
int  adbms6830_read_temp_raw(adbms6830_driver_t *dev, uint8_t ic_idx, int16_t *out_raw);
float adbms6830_convert_temp(adbms6830_driver_t *dev, uint8_t ic_idx, uint8_t sensor_num, float vref);
float voltage_to_temp(float v);
int mux_set_channel(adbms6830_driver_t *dev, uint8_t sensor_num);

#endif /* INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_ */
