/*
 * adbms2950.h
 *
 *  Created on: May 13, 2025
 *      Author: cole
 */

#ifndef INC_EXT_DRIVERS_ADBMS2950_H_
#define INC_EXT_DRIVERS_ADBMS2950_H_

#include "ext_drivers/adbms_shared.h"
#include "ext_drivers/adbms2950_defs.h"
#include "stm32f7xx_hal.h"

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
  adbms2950_asic *ics;
  loop_manager_2950_t loop_manager;
  pladc_manager_t pladc_manager;
  adc_configuration_t config;
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port[2];
	uint16_t cs_pin[2];
	adbms_string string;
	TIM_HandleTypeDef *htim;
} adbms2950_driver_t;

void adbms2950_init(adbms2950_driver_t* dev,
					uint8_t num_asics,
					adbms2950_asic* ics,
					SPI_HandleTypeDef* hspi,
					GPIO_TypeDef* CSA_Port,
					GPIO_TypeDef* CSB_Port,
					uint16_t CSA_Pin,
					uint16_t CSB_Pin,
					TIM_HandleTypeDef* htim);

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

void adbms2950_gpo_set(adbms2950_driver_t* dev, GPO gpo, CFGA_GPO state);

// Control
void adbms2950_wakeup(adbms2950_driver_t *dev);
// Utility
void adbms2950_us_delay(adbms2950_driver_t* dev, uint16_t microseconds);

#endif /* INC_EXT_DRIVERS_ADBMS2950_H_ */
