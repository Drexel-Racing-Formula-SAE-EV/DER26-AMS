/*
 * adbms2950.h
 *
 *  Created on: May 13, 2025
 *      Author: cole
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef INC_EXT_DRIVERS_ADBMS2950_H_
#define INC_EXT_DRIVERS_ADBMS2950_H_

#include "ext_drivers/adbms_shared.h"
#include "ext_drivers/adbms2950_defs.h"
#include "stm32f7xx_hal.h"
#include <stdbool.h>

#define ADBMS2950_MAX_TRACKED_ICS 16u
#define ADBMS2950_SPI_DEBUG_PREVIEW_BYTES 16u
#define ADBMS2950B_DEVICE_ID 0x06u
#define ADBMS2950_I1_RESET_CODE 0x03FFFFu
#define ADBMS2950_I1_CLEAR_CODE 0xFC0000u
#define ADBMS2950_VB1_RESET_CODE 0x7FFFu
#define ADBMS2950_VB1_CLEAR_CODE 0x8000u

typedef enum
{
  ADBMS2950_SPI_OP_NONE = 0,
  ADBMS2950_SPI_OP_CMD,
  ADBMS2950_SPI_OP_WR48,
  ADBMS2950_SPI_OP_RD48,
  ADBMS2950_SPI_OP_PROBE
} adbms2950_spi_op_t;

typedef struct
{
  bool enabled;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t error_count;
  adbms2950_spi_op_t last_op;
  adbms_string last_string;
  HAL_StatusTypeDef last_status;
  HAL_StatusTypeDef last_tx_status;
  HAL_StatusTypeDef last_rx_status;
  HAL_StatusTypeDef last_xfer_status;
  uint8_t last_cmd[2];
  uint16_t last_tx_len;
  uint16_t last_rx_len;
  uint16_t last_total_len;
  uint16_t last_read_pec_pass_mask;
  uint16_t last_read_pec_fail_mask;
  uint8_t last_cmd_counter[ADBMS2950_MAX_TRACKED_ICS];
  uint8_t last_tx_preview[ADBMS2950_SPI_DEBUG_PREVIEW_BYTES];
  uint8_t last_rx_preview[ADBMS2950_SPI_DEBUG_PREVIEW_BYTES];
} adbms2950_spi_debug_t;

/* The APM path is intentionally advisory until its shunt polarity, divider
 * ratio and fault response are validated on the final hardware.  Keeping its
 * health separate prevents an unproven sensor from silently becoming a BMS_OK
 * input while still making every transport and sample failure observable. */
typedef struct
{
  bool initialized;
  bool sid_valid;
  bool config_valid;
  bool sample_valid;
  bool current_valid;
  bool pack_voltage_valid;
  bool i1_calibrated;
  bool hv_dividers_enabled;
  bool counter_seen;
  bool counter_advanced;
  HAL_StatusTypeDef last_status;
  uint8_t device_id;
  uint8_t revision;
  uint8_t sid[RSID];
  uint8_t last_cmd_counter;
  uint32_t sample_count;
  uint32_t sample_error_count;
  uint32_t pec_error_count;
  uint32_t counter_mismatch_count;
  uint32_t counter_stall_count;
  uint32_t last_update_ms;
  int32_t i1_raw;
  int16_t vb1_raw;
  float current_a;
  float pack_voltage_v;
} adbms2950_health_t;

/*!< ADBMS2950 IC main structure */
typedef struct
{
  cfa_ tx_cfga;
  cfa_ rx_cfga;
  cfb_ tx_cfgb;
  cfb_ rx_cfgb;
  flag_ flag;
  flag_ clflag;
  crg_ i;
  iacc_ iacc;
  vbat_ vbat;
  vbacc_ vbacc;
  i_vbat_ ivbat;
  i_vbacc_ i_vbacc;
  vr_  vr;
  rvr_ rvr;
  oc_ oc;
  auxa_ auxa;
  auxb_ auxb;
  auxc_ auxc;
  rdalli_ rdalli;
  rdalla_ rdalla;
  rdallc_ rdallc;
  rdallv_ rdallv;
  rdallr_ rdallr;
  rdallx_ rdallx;
  state_ state;
  com_ tx_comm;
  com_ rx_comm;
  sid_ sid;
  ic_register_ configa;
  ic_register_ configb;
  ic_register_ clrflag;
  ic_register_ reg;
  ic_register_ axa;
  ic_register_ axb;
  ic_register_ axc;
  ic_register_ flg;
  ic_register_ ste;
  ic_register_ rdlli;
  ic_register_ rdlla;
  ic_register_ rdllc;
  ic_register_ rdllv;
  ic_register_ rdllr;
  ic_register_ rdllx;
  ic_register_ com;
  ic_register_ rsid;
  cmdcnt_pec_ cccrc;
  uint32_t pladc_count;
  uint32_t OCTicks;
  uint32_t cal_count;
  tm48_ tm48;
  uint8_t Result;
  uint8_t ResultLoc;
  uint8_t OC_PWM_Result;
  uint8_t rx_pec_error;
  uint8_t rx_cmd_cntr;
} adbms2950_asic;

/* adbms2950 main driver */
typedef struct
{
	float vbat[NVBATS]; // Actual battery voltage
	float current[NVBATS]; // Actual current value
	float temps[NAPMTEMPS]; // Current sensor NTC temps

	float vbat_adc[NVBATS]; // VBAT ADC voltage
	float vi_adc[NVIS]; // VI ADC voltage
  float vtemp_adc[NAPMTEMPS];
  uint8_t num_ics;
  uint8_t ics_capacity;
  adbms2950_asic *ics;
  loop_manager_2950_t loop_manager;
  pladc_manager_t pladc_manager;
  adc_configuration_t config;
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port[2];
	uint16_t cs_pin[2];
	adbms_string string;
	TIM_HandleTypeDef *htim;
	HAL_StatusTypeDef delay_last_status;
	uint32_t delay_timeout_count;
	adbms2950_spi_debug_t spi_debug;
	adbms2950_health_t health;
} adbms2950_driver_t;

/* Legacy standalone initializer retained for source compatibility.  New AMS
 * code must use adbms2950_init_mixed_chain() so reset ownership and chain
 * direction are explicit. */
void adbms2950_init(adbms2950_driver_t* dev,
					uint8_t num_asics,
					adbms2950_asic* ics,
					SPI_HandleTypeDef* hspi,
					GPIO_TypeDef* CSA_Port,
					GPIO_TypeDef* CSB_Port,
					uint16_t CSA_Pin,
					uint16_t CSB_Pin,
					TIM_HandleTypeDef* htim);

HAL_StatusTypeDef adbms2950_init_mixed_chain(adbms2950_driver_t *dev,
                                             uint8_t num_asics,
                                             adbms2950_asic *ics,
                                             uint8_t ics_capacity,
                                             SPI_HandleTypeDef *hspi,
                                             GPIO_TypeDef *CSA_Port,
                                             GPIO_TypeDef *CSB_Port,
                                             uint16_t CSA_Pin,
                                             uint16_t CSB_Pin,
                                             TIM_HandleTypeDef *htim,
                                             adbms_string primary_string,
                                             bool issue_chain_reset,
                                             bool enable_hv_dividers);

// Configuration
void adbms2950_reset_cfg_regs(adbms2950_driver_t* dev);
void adbms2950_wrcfga(adbms2950_driver_t* dev);
void adbms2950_wrcfgb(adbms2950_driver_t* dev);
void adbms2950_rdcfga(adbms2950_driver_t* dev);
void adbms2950_rdcfgb(adbms2950_driver_t* dev);

// Operational Commands
void adbms2950_srst(adbms2950_driver_t* dev);
void adbms2950_adi1(adbms2950_driver_t* dev, adi1_* arg); //cmd, starts i1adc, vb1adc
void adbms2950_adi2(adbms2950_driver_t* dev, adi2_* arg); //cmd, starts i2adc, vb2adc
void adbms2950_adv(adbms2950_driver_t* dev, adv_* arg); //cmd, starts v1adc, v2adc

void adbms2950_plv(adbms2950_driver_t* dev); //cmd, polls v1adc, v2adc

void adbms2950_rdvb(adbms2950_driver_t* dev); //rd48, reads vb1adc, vb2adc
void adbms2950_rdi(adbms2950_driver_t* dev); //rd48, reads i1adc, i2adc
void adbms2950_rdv1d(adbms2950_driver_t* dev); //rd48, reads v1adc for v7a, v2adc for v9b

HAL_StatusTypeDef adbms2950_read_sid(adbms2950_driver_t *dev);
HAL_StatusTypeDef adbms2950_read_status(adbms2950_driver_t *dev);
HAL_StatusTypeDef adbms2950_read_primary_sample(adbms2950_driver_t *dev,
                                                uint32_t now_ms);
HAL_StatusTypeDef adbms2950_verify_config_readback(adbms2950_driver_t *dev);
const adbms2950_health_t *adbms2950_health_get(const adbms2950_driver_t *dev);
void adbms2950_health_clear_counters(adbms2950_driver_t *dev);

void adbms2950_gpo_set(adbms2950_driver_t* dev, GPO gpo, CFGA_GPO state);

// Control
void adbms2950_wakeup(adbms2950_driver_t *dev);
HAL_StatusTypeDef adbms2950_wakeup_checked(adbms2950_driver_t *dev);
void adbms2950_set_cs(adbms2950_driver_t *dev, uint8_t state);
// Utility
HAL_StatusTypeDef adbms2950_us_delay(adbms2950_driver_t* dev, uint16_t microseconds);

void adbms2950_spi_debug_enable(adbms2950_driver_t *dev, bool enable);
void adbms2950_spi_debug_clear(adbms2950_driver_t *dev);
const adbms2950_spi_debug_t *adbms2950_spi_debug_get(const adbms2950_driver_t *dev);
const char *adbms2950_spi_op_str(adbms2950_spi_op_t op);
HAL_StatusTypeDef adbms2950_spi_probe_rdcfga(adbms2950_driver_t *dev);
HAL_StatusTypeDef adbms2950_spi_probe_sid(adbms2950_driver_t *dev);

/* Low-level SPI helpers are exposed for APM bring-up/debug and host tests. */
HAL_StatusTypeDef adbms2950_spi_write(adbms2950_driver_t *dev, uint8_t *data, uint16_t len, uint8_t use_cs);
HAL_StatusTypeDef adbms2950_spi_write_read(adbms2950_driver_t *dev, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t rx_len, uint8_t use_cs);

#endif /* INC_EXT_DRIVERS_ADBMS2950_H_ */
