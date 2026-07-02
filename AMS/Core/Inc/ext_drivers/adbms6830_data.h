/*
 * adbms6830.h
 *
 *  Created on: May 13, 2025
 *      Author: realb
 */

#ifndef INC_EXT_DRIVERS_ADBMS6830_DATA_H_
#define INC_EXT_DRIVERS_ADBMS6830_DATA_H_

#include "ext_drivers/adbms_shared.h"
#include <stdint.h>
#include "stm32f7xx_hal.h"

#define ADBMS6830_MAX_TRACKED_ICS 16u

/* BMS ic main structure */
typedef struct
{
  cfa6830_ tx_cfga;
  cfa6830_ rx_cfga;
  cfb6830_ tx_cfgb;
  cfb6830_ rx_cfgb;
  clrflag_ clflag;
  cv_  cell;
  acv_ acell;
  scv_ scell;
  fcv_ fcell;
  ax_  aux;
  rax_ raux;
  temp_ temp;
  sta_ stata;
  stb_ statb;
  stc_ statc;
  std_ statd;
  ste_ state;
  com_ comm;
  pwma_ PwmA;
  pwmb_ PwmB;
  sid_ sid;
  ic_register_ configa;
  ic_register_ configb;
  ic_register_ clrflag;
  ic_register_ stat;
  ic_register_ com;
  ic_register_ pwma;
  ic_register_ pwmb;
  ic_register_ rsid;
  cmdcnt_pec_6830_ cccrc;
  aux_ow_ gpio;
  cell_ow_ owcell;
  diag_test_6830_ diag_result;
} adbms6830_asic;

typedef struct
{
  int num_ics;
  adbms6830_asic *ics;
  adbms_string string;
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port[2];
  uint16_t cs_pin[2];

  adc_command_config_t adc_config;
  threshold_config_t thresholds;
  loop_manager_6830_t loop_manager;

  TIM_HandleTypeDef *htim;

  /* Last cell-voltage read status. One bit per cell index for each IC.
   * last_cell_updated_mask is set only when the register group PEC passed and
   * the cell code was valid for the most recent read_cell_voltages() call.
   * last_cell_pec_mask marks cells inside any register group whose PEC failed.
   */
  uint16_t last_cell_updated_mask[ADBMS6830_MAX_TRACKED_ICS];
  uint16_t last_cell_pec_mask[ADBMS6830_MAX_TRACKED_ICS];
} adbms6830_driver_t;

#endif /* INC_EXT_DRIVERS_ADBMS6830_DATA_H_ */
