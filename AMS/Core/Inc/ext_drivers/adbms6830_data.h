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
#include <stdbool.h>
#include "stm32f7xx_hal.h"

#define ADBMS6830_MAX_TRACKED_ICS 16u

#define ADBMS6830_SPI_DEBUG_PREVIEW_BYTES 16u

typedef enum
{
  ADBMS6830_SPI_OP_NONE = 0,
  ADBMS6830_SPI_OP_CMD,
  ADBMS6830_SPI_OP_WR48,
  ADBMS6830_SPI_OP_RD48,
  ADBMS6830_SPI_OP_STCOMM,
  ADBMS6830_SPI_OP_PROBE,
  ADBMS6830_SPI_OP_WAKE,
  ADBMS6830_SPI_OP_COLD_WAKE,
  ADBMS6830_SPI_OP_READ_SID,
  ADBMS6830_SPI_OP_READ_STATUS,
  ADBMS6830_SPI_OP_CLEAR_FLAGS,
  ADBMS6830_SPI_OP_CONFIG_CHECK,
  ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST,
  ADBMS6830_SPI_OP_OPEN_WIRE_EVEN,
  ADBMS6830_SPI_OP_OPEN_WIRE_ODD,
  ADBMS6830_SPI_OP_AUX_GPIO_DIAG
} adbms6830_spi_op_t;

typedef struct
{
  bool enabled;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t error_count;
  adbms6830_spi_op_t last_op;
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
  uint16_t cmd_counter_seen_mask;
  uint16_t cmd_counter_expected_mask;
  uint16_t cmd_counter_mismatch_mask;
  uint32_t cmd_counter_error_count;
  uint8_t last_cmd_counter[ADBMS6830_MAX_TRACKED_ICS];
  uint8_t expected_cmd_counter[ADBMS6830_MAX_TRACKED_ICS];
  uint8_t last_tx_preview[ADBMS6830_SPI_DEBUG_PREVIEW_BYTES];
  uint8_t last_rx_preview[ADBMS6830_SPI_DEBUG_PREVIEW_BYTES];
} adbms6830_spi_debug_t;

typedef struct
{
  bool sid_valid;
  bool statc_valid;
  bool statd_valid;
  bool state_valid;

  uint8_t sid[RSID];

  uint16_t cs_flt_mask;
  uint16_t cadc_counter;
  uint8_t cadc_subcounter;
  uint8_t va_ov;
  uint8_t va_uv;
  uint8_t vd_ov;
  uint8_t vd_uv;
  uint8_t ced;
  uint8_t cmed;
  uint8_t sed;
  uint8_t smed;
  uint8_t vde;
  uint8_t vdel;
  uint8_t comp;
  uint8_t spiflt;
  uint8_t sleep;
  uint8_t thsd;
  uint8_t tmodchk;
  uint8_t oscchk;

  uint16_t cell_ov_mask;
  uint16_t cell_uv_mask;
  uint8_t osc_counter;

  uint16_t gpi_mask;
  uint8_t revision;
} adbms6830_ic_diag_t;

typedef struct
{
  adbms6830_spi_op_t last_op;
  HAL_StatusTypeDef last_status;

  uint16_t last_pec_pass_mask;
  uint16_t last_pec_fail_mask;
  uint16_t sticky_pec_fail_mask;
  uint16_t last_cmd_counter_mismatch_mask;
  uint16_t sticky_cmd_counter_mismatch_mask;

  uint16_t configa_mismatch_mask;
  uint16_t configb_mismatch_mask;
  uint16_t config_mismatch_mask;

  uint32_t pec_pass_count[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t pec_fail_count[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t cmd_counter_mismatch_count[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t config_mismatch_count[ADBMS6830_MAX_TRACKED_ICS];

  uint32_t config_readback_count;
  uint32_t cell_adc_self_test_count;
  uint32_t open_wire_even_count;
  uint32_t open_wire_odd_count;
  uint32_t aux_gpio_diag_count;
} adbms6830_diag_health_t;


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

  /* Last temperature-mux read status. One bit per temp sensor index for each
   * IC. The mux scan only reads three of twenty-four sensors per AMS cycle, so
   * safety policy needs an explicit freshness mask instead of treating old raw
   * thermistor codes as newly updated.
   */
  uint32_t last_temp_updated_mask[ADBMS6830_MAX_TRACKED_ICS];

  adbms6830_spi_debug_t spi_debug;
  adbms6830_ic_diag_t diag[ADBMS6830_MAX_TRACKED_ICS];
  adbms6830_diag_health_t health;
} adbms6830_driver_t;

#endif /* INC_EXT_DRIVERS_ADBMS6830_DATA_H_ */
