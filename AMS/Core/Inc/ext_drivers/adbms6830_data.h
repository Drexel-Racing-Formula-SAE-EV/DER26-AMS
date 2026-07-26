/*
 * adbms6830.h
 *
 *  Created on: May 13, 2025
 *      Author: realb
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef INC_EXT_DRIVERS_ADBMS6830_DATA_H_
#define INC_EXT_DRIVERS_ADBMS6830_DATA_H_

#include "ext_drivers/adbms_shared.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"

#define ADBMS6830_MAX_TRACKED_ICS 16u
#define ADBMS6830_MAX_PHYSICAL_DEVICES 16u
#define ADBMS6830B_DEVICE_ID 0x03u
#define ADBMS6830_MUX_COUNT 3u
#define ADBMS6830_TEMP_SENSOR_COUNT 24u

/* The S-ADC open-wire result for an intact, normally charged cell remains
 * well above this threshold after the monitor's internal diagnostic divider.
 * ADI's reference application uses 2000 mV.  The application must explicitly
 * configure how many populated cell channels are monitored so an unpopulated
 * C16 input is never reported as an open wire. */
#define ADBMS6830_OPEN_WIRE_THRESHOLD_MV 2000u
#define ADBMS6830_OPEN_WIRE_CONVERSION_WAIT_US 9000u
/* The ADBMS6830B S-ADC is specified at 85% to 95% gain while the open-wire
 * switch is active.  Compare each stimulated sample with a fresh unstimulated
 * S-ADC baseline and require the corresponding 5% to 15% attenuation. */
#define ADBMS6830_OPEN_WIRE_MIN_ATTENUATION_PERMILLE 50u
#define ADBMS6830_OPEN_WIRE_MAX_ATTENUATION_PERMILLE 150u

/* Status A/B plausibility limits.  The VREF2 limit deliberately includes
 * margin around the datasheet's normal 2.988 V to 3.012 V range.  These are
 * safety-screening limits, not calibration constants. */
#define ADBMS6830_VREF2_MIN_MV 2980
#define ADBMS6830_VREF2_MAX_MV 3020
#define ADBMS6830_VD_MIN_MV 2700
#define ADBMS6830_VD_MAX_MV 3600
#define ADBMS6830_VA_MIN_MV 4500
#define ADBMS6830_VA_MAX_MV 5500
#define ADBMS6830_VRES_MAX_DELTA_MV 50
#define ADBMS6830_DIE_TEMP_MIN_DECI_C (-500)
#define ADBMS6830_DIE_TEMP_MAX_DECI_C 1500
#define ADBMS6830_OSC_COUNTER_MIN 52u
#define ADBMS6830_OSC_COUNTER_MAX 71u
/* ADAX AUX_ALL performs 18 sequential 1 ms conversions.  Keep enough margin
 * for the complete sequence before any Status/AUX result is accepted. */
#define ADBMS6830_AUX_CONVERSION_WAIT_US 20000u

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
  ADBMS6830_SPI_OP_DIAGNOSTIC_REFRESH,
  ADBMS6830_SPI_OP_STARTUP_BASELINE,
  ADBMS6830_SPI_OP_CLEAR_FLAGS,
  ADBMS6830_SPI_OP_CONFIG_CHECK,
  ADBMS6830_SPI_OP_BALANCE_CHECK,
  ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST,
  ADBMS6830_SPI_OP_OPEN_WIRE_BASELINE,
  ADBMS6830_SPI_OP_OPEN_WIRE_EVEN,
  ADBMS6830_SPI_OP_OPEN_WIRE_ODD,
  ADBMS6830_SPI_OP_OPEN_WIRE_FULL,
  ADBMS6830_SPI_OP_AUX_GPIO_DIAG,
  ADBMS6830_SPI_OP_SCOPE
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
  bool stata_valid;
  bool statb_valid;
  bool statc_valid;
  bool statd_valid;
  bool state_valid;
  bool reference_values_valid;

  uint8_t sid[RSID];
  uint8_t device_id;

  int16_t vref2_raw;
  int16_t itmp_raw;
  int16_t vd_raw;
  int16_t va_raw;
  int16_t vres_raw;
  int16_t vref2_mv;
  int16_t die_temp_deci_c;
  int16_t vd_mv;
  int16_t va_mv;
  int16_t vres_mv;

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

  bool open_wire_even_valid;
  bool open_wire_odd_valid;
  bool open_wire_baseline_valid;
  uint16_t open_wire_even_fault_mask;
  uint16_t open_wire_odd_fault_mask;
  uint16_t open_wire_even_attenuation_fault_mask;
  uint16_t open_wire_odd_attenuation_fault_mask;
  uint16_t open_wire_fault_mask;
  uint16_t open_wire_baseline_mv[CELL];
} adbms6830_ic_diag_t;

/* End-to-end result of the most recent checked 48-bit register read.
 * Keep this separate from HAL_StatusTypeDef so diagnostics can distinguish a
 * local transport failure from a packet that arrived but failed PEC10 and/or
 * command-counter validation.  Public checked APIs still fail closed through
 * HAL_ERROR for any non-OK result. */
typedef enum
{
  ADBMS6830_READ_RESULT_NONE = 0,
  ADBMS6830_READ_RESULT_OK,
  ADBMS6830_READ_RESULT_TOPOLOGY_ERROR,
  ADBMS6830_READ_RESULT_TRANSPORT_ERROR,
  ADBMS6830_READ_RESULT_PEC_ERROR,
  ADBMS6830_READ_RESULT_COUNTER_ERROR,
  ADBMS6830_READ_RESULT_PEC_AND_COUNTER_ERROR
} adbms6830_read_result_t;

typedef struct
{
  adbms6830_spi_op_t last_op;
  HAL_StatusTypeDef last_status;
  adbms6830_read_result_t last_read_result;

  uint16_t last_pec_pass_mask;
  uint16_t last_pec_fail_mask;
  uint16_t sticky_pec_fail_mask;
  uint16_t last_cmd_counter_mismatch_mask;
  uint16_t sticky_cmd_counter_mismatch_mask;

  /* RDSID byte 1 bits [6:1] identify an ADBMS6830B as 0x03.  Keep
   * transport validity separate from product identity so a PEC-valid packet
   * from the ADBMS2950 at the wrong end cannot establish SMB readiness. */
  uint16_t sid_valid_ic_mask;
  uint16_t sid_identity_mismatch_ic_mask;
  uint16_t sticky_sid_identity_mismatch_ic_mask;

  uint16_t configa_mismatch_mask;
  uint16_t configb_mismatch_mask;
  uint16_t config_mismatch_mask;

  uint16_t balance_cfgb_mismatch_mask;
  uint16_t balance_pwma_mismatch_mask;
  uint16_t balance_pwmb_mismatch_mask;
  uint16_t balance_mismatch_mask;

  uint32_t pec_pass_count[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t pec_fail_count[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t cmd_counter_mismatch_count[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t config_mismatch_count[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t balance_mismatch_count[ADBMS6830_MAX_TRACKED_ICS];

  uint32_t config_readback_count;
  uint32_t balance_readback_count;
  uint32_t diagnostic_refresh_count;
  uint32_t startup_baseline_count;
  uint32_t cell_adc_self_test_count;
  uint32_t open_wire_baseline_count;
  uint32_t open_wire_even_count;
  uint32_t open_wire_odd_count;
  uint32_t open_wire_full_count;
  uint32_t aux_gpio_diag_count;

  bool startup_baseline_passed;
  uint16_t status_invalid_ic_mask;
  uint16_t status_fault_ic_mask;
  uint16_t reference_invalid_ic_mask;
  uint16_t reference_fault_ic_mask;
  uint16_t cs_fault_ic_mask;
  uint16_t supply_flag_fault_ic_mask;
  uint16_t memory_fault_ic_mask;
  uint16_t digital_fault_ic_mask;
  uint16_t oscillator_counter_fault_ic_mask;
  uint16_t cell_ovuv_fault_ic_mask;
  uint16_t sticky_status_fault_ic_mask;
  uint16_t sticky_reference_fault_ic_mask;

  uint16_t open_wire_baseline_valid_ic_mask;
  uint16_t open_wire_even_valid_ic_mask;
  uint16_t open_wire_odd_valid_ic_mask;
  uint16_t open_wire_incomplete_ic_mask;
  uint16_t open_wire_fault_ic_mask;
  uint16_t sticky_open_wire_fault_ic_mask;
  uint16_t open_wire_cell_fault_mask[ADBMS6830_MAX_TRACKED_ICS];
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
  uint8_t physical_chain_count;
  uint8_t ics_capacity;
  adbms6830_asic *ics;
  uint8_t monitored_cell_count;
  adbms_string string;
  /* Register writes may intentionally stop after the leading five devices.
   * write_string records which physical end owns that leading subset; read
   * probes may temporarily use the other end, but writes must never do so. */
  adbms_string write_string;
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port[2];
  uint16_t cs_pin[2];

  adc_command_config_t adc_config;
  threshold_config_t thresholds;
  loop_manager_6830_t loop_manager;

  TIM_HandleTypeDef *htim;

  HAL_StatusTypeDef delay_last_status;
  uint32_t delay_timeout_count;

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

  /* A temperature is publishable only when the corresponding ADG728 channel
   * selection was acknowledged by that IC's local mux.  RDCOMM supplies the
   * per-IC ACK result after STCOMM; retaining it here prevents a failed mux
   * write from relabelling the previous physical channel as a new sensor. */
  uint16_t mux_selection_valid_mask[ADBMS6830_MUX_COUNT];
  uint8_t mux_selected_channel[ADBMS6830_MAX_TRACKED_ICS][ADBMS6830_MUX_COUNT];

  adbms6830_spi_debug_t spi_debug;
  adbms6830_ic_diag_t diag[ADBMS6830_MAX_TRACKED_ICS];
  adbms6830_diag_health_t health;
} adbms6830_driver_t;

#endif /* INC_EXT_DRIVERS_ADBMS6830_DATA_H_ */
