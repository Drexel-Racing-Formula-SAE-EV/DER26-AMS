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
#define ADBMS6830_TEMP_BUS_SCAN_FIRST_ADDR 0x4Cu
#define ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT 4u
#define ADBMS6830_TEMP_BUS_SCAN_DATA_BYTE 0x00u
#define ADBMS6830_SADC_GROUP_COUNT 6u
#define ADBMS6830_CADC_GROUP_COUNT 6u
#define ADBMS6830_RAW_REGISTER_COUNT 26u
#define ADBMS6830_CFGB_WRITE_HISTORY_DEPTH 8u

/* The S-ADC open-wire result for an intact, normally charged cell remains
 * well above this threshold after the monitor's internal diagnostic divider.
 * ADI's reference application uses 2000 mV.  The application must explicitly
 * configure how many populated cell channels are monitored so an unpopulated
 * C16 input is never reported as an open wire. */
#define ADBMS6830_OPEN_WIRE_THRESHOLD_MV 2000u
#define ADBMS6830_OPEN_WIRE_CONVERSION_WAIT_US 9000u
/* S-path open-wire gain is specified at 85% to 95%, corresponding to 5% to
 * 15% attenuation from a fresh baseline.  The C path uses the external 200-ohm
 * network against the monitor's nominal 1.75-kohm diagnostic resistance; the
 * expected attenuation is about 16.7%.  Keep a deliberately wider 10% to 25%
 * C window until target characterization is complete. */
#define ADBMS6830_S_OPEN_WIRE_MIN_ATTENUATION_PERMILLE 50u
#define ADBMS6830_S_OPEN_WIRE_MAX_ATTENUATION_PERMILLE 150u
#define ADBMS6830_C_OPEN_WIRE_MIN_ATTENUATION_PERMILLE 100u
#define ADBMS6830_C_OPEN_WIRE_MAX_ATTENUATION_PERMILLE 250u

typedef enum
{
  ADBMS6830_OPEN_WIRE_PATH_C = 0,
  ADBMS6830_OPEN_WIRE_PATH_S = 1
} adbms6830_open_wire_path_t;

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
  ADBMS6830_SPI_OP_CS_COMPARE,
  ADBMS6830_SPI_OP_S_ADC_DUMP,
  ADBMS6830_SPI_OP_C_ADC_DUMP,
  ADBMS6830_SPI_OP_CONVERSION_TIMING,
  ADBMS6830_SPI_OP_CONFIG_STRESS,
  ADBMS6830_SPI_OP_RECOVERY,
  ADBMS6830_SPI_OP_RAW_DUMP,
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

/* Pure temperature-channel routing description used by both the physical
 * ADG728 transaction path and the software-only bench emulator. */
typedef struct
{
  uint8_t sensor_num;
  uint8_t mux_idx;
  uint8_t mux_address;
  uint8_t switch_index;
  uint8_t switch_mask;
  uint8_t gpio_channel;
} adbms6830_temp_route_t;

/* Last service-level temperature transaction. This is diagnostic state only:
 * normal temperature publication still requires an acknowledged ADG728 mux
 * selection plus a fresh, PEC-valid AUX sample. */
typedef struct
{
  bool valid;
  bool forced_aux_capture;
  bool stcomm_attempted;

  uint8_t sensor_num;
  uint8_t mux_idx;
  uint8_t mux_address;
  uint8_t switch_index;
  uint8_t switch_mask;
  uint8_t gpio_channel;

  uint16_t expected_ic_mask;

  HAL_StatusTypeDef select_status;
  HAL_StatusTypeDef wrc_status;
  HAL_StatusTypeDef pre_rdcomm_status;
  HAL_StatusTypeDef stcomm_status;
  HAL_StatusTypeDef rdcomm_status;
  HAL_StatusTypeDef wake_status;
  HAL_StatusTypeDef adax_status;
  HAL_StatusTypeDef rdaux_status;
  HAL_StatusTypeDef overall_status;

  /* WRCOMM verification is intentionally captured before STCOMM so a bad
   * register write cannot drive the external GPIO/I2C bus. */
  uint16_t pre_comm_pec_pass_mask;
  uint16_t pre_comm_pec_fail_mask;
  uint16_t pre_comm_counter_mismatch_mask;
  uint16_t pre_comm_match_mask;

  uint16_t comm_pec_pass_mask;
  uint16_t comm_pec_fail_mask;
  uint16_t comm_counter_mismatch_mask;
  uint16_t comm_transport_valid_mask;
  uint16_t address_ack_mask;
  uint16_t data_ack_mask;
  uint16_t acknowledged_mask;
  uint16_t selected_mask;

  uint16_t aux_pec_pass_mask;
  uint16_t aux_pec_fail_mask;
  uint16_t aux_counter_mismatch_mask;
  uint16_t aux_transport_valid_mask;
  uint16_t aux_code_valid_mask;
  uint16_t updated_mask;

  uint8_t wrcomm_payload[ADBMS6830_MAX_TRACKED_ICS][TX_DATA];
  uint8_t pre_rdcomm_packet[ADBMS6830_MAX_TRACKED_ICS][RX_DATA];
  uint8_t rdcomm_packet[ADBMS6830_MAX_TRACKED_ICS][RX_DATA];
  uint8_t rdaux_packet[ADBMS6830_MAX_TRACKED_ICS][RX_DATA];
} adbms6830_temp_debug_t;

/* Non-driving temperature-bus observation.  The capture only runs the AUX
 * ADC and reads GPIO4/SDA and GPIO5/SCL; it never issues WRCOMM or STCOMM. */
typedef struct
{
  bool valid;
  HAL_StatusTypeDef wake_status;
  HAL_StatusTypeDef adax_status;
  HAL_StatusTypeDef rdauxb_status;
  HAL_StatusTypeDef overall_status;

  uint16_t expected_ic_mask;
  uint16_t pec_pass_mask;
  uint16_t pec_fail_mask;
  uint16_t counter_mismatch_mask;
  uint16_t transport_valid_mask;
  uint16_t gpio4_code_valid_mask;
  uint16_t gpio5_code_valid_mask;

  uint8_t rdauxb_packet[ADBMS6830_MAX_TRACKED_ICS][RX_DATA];
  int16_t gpio4_raw[ADBMS6830_MAX_TRACKED_ICS];
  int16_t gpio5_raw[ADBMS6830_MAX_TRACKED_ICS];
} adbms6830_temp_bus_debug_t;

/* Active service scan of the four legal ADG728 addresses. Each probe writes
 * control byte 0x00 (all switches open), records the raw COMM image, and
 * distinguishes transport completion from slave ACK. The scan never publishes
 * a temperature sample and invalidates all cached mux selections afterward. */
typedef struct
{
  bool valid;
  uint8_t first_address;
  uint8_t address_count;
  uint8_t data_byte;
  uint8_t any_ack_address_bitmap;
  uint8_t full_ack_address_bitmap;

  uint16_t expected_ic_mask;
  HAL_StatusTypeDef overall_status;
  HAL_StatusTypeDef probe_status[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  HAL_StatusTypeDef transport_status[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  HAL_StatusTypeDef wrc_status[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  HAL_StatusTypeDef pre_rdcomm_status[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  HAL_StatusTypeDef stcomm_status[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  HAL_StatusTypeDef rdcomm_status[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];

  uint16_t pre_comm_match_mask[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  uint16_t comm_pec_pass_mask[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  uint16_t comm_pec_fail_mask[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  uint16_t comm_counter_mismatch_mask[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  uint16_t comm_transport_valid_mask[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  uint16_t address_ack_mask[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  uint16_t data_ack_mask[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];
  uint16_t acknowledged_mask[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT];

  uint8_t rdcomm_packet[ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT]
                       [ADBMS6830_MAX_TRACKED_ICS]
                       [RX_DATA];
} adbms6830_temp_bus_scan_t;

/* Standalone S-ADC bench capture.  This state is diagnostic-only and never
 * promotes a cell voltage into the normal measurement/safety publication.
 * Each raw packet retains the six data bytes plus command-counter/PEC bytes. */
typedef struct
{
  bool valid;
  uint8_t group_count;
  uint16_t expected_ic_mask;

  HAL_StatusTypeDef wake_status;
  HAL_StatusTypeDef command_status;
  HAL_StatusTypeDef delay_status;
  HAL_StatusTypeDef group_read_status[ADBMS6830_SADC_GROUP_COUNT];
  HAL_StatusTypeDef overall_status;

  uint16_t pec_pass_mask[ADBMS6830_SADC_GROUP_COUNT];
  uint16_t pec_fail_mask[ADBMS6830_SADC_GROUP_COUNT];
  uint16_t counter_mismatch_mask[ADBMS6830_SADC_GROUP_COUNT];
  uint16_t transport_valid_mask[ADBMS6830_SADC_GROUP_COUNT];

  uint8_t packet[ADBMS6830_MAX_TRACKED_ICS]
                [ADBMS6830_SADC_GROUP_COUNT]
                [RX_DATA];
} adbms6830_sadc_debug_t;

/* Standalone C-ADC bench capture. This uses ADCV with redundancy disabled and
 * polls PLCADC to measure actual conversion completion. Like the S capture, it
 * is diagnostic-only and never changes application readiness or latches. */
typedef struct
{
  bool valid;
  uint8_t group_count;
  uint16_t expected_ic_mask;

  HAL_StatusTypeDef wake_status;
  HAL_StatusTypeDef command_status;
  HAL_StatusTypeDef poll_status;
  HAL_StatusTypeDef group_read_status[ADBMS6830_CADC_GROUP_COUNT];
  HAL_StatusTypeDef overall_status;

  uint32_t conversion_time_us;
  uint32_t poll_clock_bytes;
  bool poll_complete;

  uint16_t pec_pass_mask[ADBMS6830_CADC_GROUP_COUNT];
  uint16_t pec_fail_mask[ADBMS6830_CADC_GROUP_COUNT];
  uint16_t counter_mismatch_mask[ADBMS6830_CADC_GROUP_COUNT];
  uint16_t transport_valid_mask[ADBMS6830_CADC_GROUP_COUNT];

  uint8_t packet[ADBMS6830_MAX_TRACKED_ICS]
                [ADBMS6830_CADC_GROUP_COUNT]
                [RX_DATA];
} adbms6830_cadc_debug_t;

typedef enum
{
  ADBMS6830_TIMING_C_ADC = 0,
  ADBMS6830_TIMING_S_ADC,
  ADBMS6830_TIMING_AUX_ADC
} adbms6830_timing_kind_t;

/* Every CFGB write is classified by intent.  This makes the discharge-timer
 * policy auditable and prevents a generic balance/configuration path from
 * silently arming DCTO. */
typedef enum
{
  ADBMS6830_CFGB_WRITE_UNSPECIFIED = 0,
  ADBMS6830_CFGB_WRITE_INITIALIZATION,
  ADBMS6830_CFGB_WRITE_BALANCE_APPLY,
  ADBMS6830_CFGB_WRITE_BALANCE_CLEAR,
  ADBMS6830_CFGB_WRITE_BALANCE_RECOVERY,
  ADBMS6830_CFGB_WRITE_CONFIG_STRESS,
  ADBMS6830_CFGB_WRITE_DISCHARGE_TIMER_CONFIG
} adbms6830_cfgb_write_reason_t;

typedef struct
{
  uint32_t sequence;
  uint32_t tick_ms;
  adbms6830_cfgb_write_reason_t reason;
  HAL_StatusTypeDef status;
  adbms_string string;
  uint8_t ic_count;
  uint16_t timer_nonzero_mask;
  uint16_t balance_shadow_mask;
  uint16_t rejected_mask;
  uint8_t payload[ADBMS6830_MAX_TRACKED_ICS][TX_DATA];
} adbms6830_cfgb_write_event_t;

typedef struct
{
  adbms6830_timing_kind_t kind;
  HAL_StatusTypeDef wake_status;
  HAL_StatusTypeDef command_status;
  HAL_StatusTypeDef poll_status;
  HAL_StatusTypeDef overall_status;
  uint32_t elapsed_us;
  uint32_t poll_clock_bytes;
  bool observed_busy;
  bool complete;
} adbms6830_timing_result_t;

typedef struct
{
  HAL_StatusTypeDef write_cfga_status;
  HAL_StatusTypeDef write_cfgb_status;
  HAL_StatusTypeDef readback_status;
  HAL_StatusTypeDef overall_status;
  uint16_t cfga_mismatch_mask;
  uint16_t cfgb_mismatch_mask;
  uint16_t discharge_nonzero_mask;
} adbms6830_config_cycle_result_t;

typedef struct
{
  HAL_StatusTypeDef wake_status;
  HAL_StatusTypeDef sid_status;
  HAL_StatusTypeDef write_cfga_status;
  HAL_StatusTypeDef write_cfgb_status;
  HAL_StatusTypeDef config_status;
  HAL_StatusTypeDef diagnostic_status;
  HAL_StatusTypeDef cadc_status;
  HAL_StatusTypeDef overall_status;
  uint16_t sid_valid_mask;
  uint16_t config_mismatch_mask;
  uint16_t reference_fault_mask;
  uint16_t status_fault_mask;
  uint16_t cadc_valid_ic_mask;
} adbms6830_recovery_debug_t;

typedef enum
{
  ADBMS6830_RAW_CFGA = 0,
  ADBMS6830_RAW_CFGB,
  ADBMS6830_RAW_CVA,
  ADBMS6830_RAW_CVB,
  ADBMS6830_RAW_CVC,
  ADBMS6830_RAW_CVD,
  ADBMS6830_RAW_CVE,
  ADBMS6830_RAW_CVF,
  ADBMS6830_RAW_SVA,
  ADBMS6830_RAW_SVB,
  ADBMS6830_RAW_SVC,
  ADBMS6830_RAW_SVD,
  ADBMS6830_RAW_SVE,
  ADBMS6830_RAW_SVF,
  ADBMS6830_RAW_AUXA,
  ADBMS6830_RAW_AUXB,
  ADBMS6830_RAW_AUXC,
  ADBMS6830_RAW_AUXD,
  ADBMS6830_RAW_STATA,
  ADBMS6830_RAW_STATB,
  ADBMS6830_RAW_STATC,
  ADBMS6830_RAW_STATD,
  ADBMS6830_RAW_STATE,
  ADBMS6830_RAW_COMM,
  ADBMS6830_RAW_PWMA,
  ADBMS6830_RAW_PWMB
} adbms6830_raw_register_t;

typedef struct
{
  adbms6830_raw_register_t reg;
  HAL_StatusTypeDef status;
  uint16_t pec_pass_mask;
  uint16_t pec_fail_mask;
  uint16_t counter_mismatch_mask;
  uint8_t packet[ADBMS6830_MAX_TRACKED_ICS][RX_DATA];
} adbms6830_raw_read_t;

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

  adbms6830_open_wire_path_t open_wire_path;
  bool open_wire_even_valid;
  bool open_wire_odd_valid;
  bool open_wire_baseline_valid;
  uint16_t open_wire_even_fault_mask;
  uint16_t open_wire_odd_fault_mask;
  uint16_t open_wire_even_attenuation_fault_mask;
  uint16_t open_wire_odd_attenuation_fault_mask;
  uint16_t open_wire_fault_mask;
  uint16_t open_wire_baseline_mv[CELL];
  uint16_t open_wire_even_mv[CELL];
  uint16_t open_wire_odd_mv[CELL];
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
  /* A PEC-valid observed command counter of zero while a nonzero count was
   * expected indicates that the remote monitor lost state without the MCU
   * issuing SRST/RSTCC or explicitly resynchronizing.  This is useful evidence
   * for brownout/VREG-collapse and unexpected sleep/reset investigations. */
  uint16_t unexpected_counter_reset_mask;
  uint16_t sticky_unexpected_counter_reset_mask;
  uint16_t config_write_guard_fault_mask;
  uint16_t sticky_config_write_guard_fault_mask;
  uint32_t unexpected_counter_reset_count[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t config_mismatch_count[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t config_write_guard_reject_count[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t balance_mismatch_count[ADBMS6830_MAX_TRACKED_ICS];

  uint32_t config_readback_count;
  uint32_t balance_readback_count;
  uint32_t diagnostic_refresh_count;
  uint32_t startup_baseline_count;
  uint32_t cell_adc_self_test_count;
  adbms6830_open_wire_path_t open_wire_last_path;
  uint32_t open_wire_c_full_count;
  uint32_t open_wire_s_full_count;
  uint32_t open_wire_restore_count;
  uint32_t open_wire_restore_fail_count;
  HAL_StatusTypeDef open_wire_last_restore_status;
  uint32_t open_wire_baseline_count;
  uint32_t open_wire_even_count;
  uint32_t open_wire_odd_count;
  uint32_t open_wire_full_count;
  uint32_t aux_gpio_diag_count;
  uint32_t mute_count;
  uint32_t mute_fail_count;
  uint32_t mute_verify_fail_count;
  uint32_t unmute_count;
  uint32_t unmute_fail_count;
  uint32_t unmute_verify_fail_count;
  uint32_t filtered_read_count;
  uint32_t filtered_read_fail_count;
  uint32_t avg8_read_count;
  uint32_t avg8_read_fail_count;
  uint32_t coherent_statc_read_count;
  uint32_t coherent_statc_read_fail_count;
  uint32_t coherent_statd_read_count;
  uint32_t coherent_statd_read_fail_count;
  uint32_t silicon_health_sweep_count;
  uint32_t s_periodic_diag_count;
  uint32_t s_periodic_diag_fail_count;

  /* Status-C CCTS is coherent to C-cell data when both are read inside one
   * SNAP epoch.  It is therefore a useful independent proof that the primary
   * C converter is actually advancing instead of repeatedly returning a
   * plausible but stale register image. */
  uint16_t cadc_ccts_last[ADBMS6830_MAX_TRACKED_ICS];
  uint16_t cadc_ccts_previous[ADBMS6830_MAX_TRACKED_ICS];
  uint16_t cadc_ccts_initialized_mask;
  uint16_t cadc_ccts_valid_ic_mask;
  uint16_t cadc_ccts_fault_ic_mask;
  uint16_t sticky_cadc_ccts_fault_ic_mask;
  uint32_t cadc_ccts_fault_count[ADBMS6830_MAX_TRACKED_ICS];

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


typedef HAL_StatusTypeDef (*adbms6830_cooperative_wait_fn_t)(void *ctx, uint32_t wait_us);
typedef uint32_t (*adbms6830_time_us_fn_t)(void *ctx);

typedef struct
{
  bool active;
  bool coherent_snapshot_active;
  uint32_t session_id;
  uint32_t start_us;
  uint32_t last_activity_us;
  uint32_t last_gap_us;
  uint32_t max_gap_us;
  uint32_t last_duration_us;
  uint32_t max_duration_us;
  uint32_t session_count;
  uint32_t full_wake_count;
  uint32_t wake_count_scan_start;
  uint32_t wake_count_last_scan;
  uint32_t guard_expiry_count;
  uint32_t guard_rewake_count;
  uint32_t coherent_restart_count;
  uint32_t coherent_restart_fail_count;
  uint32_t long_wait_count;
  uint32_t long_wait_interrupted_count;
  uint64_t long_wait_requested_us;
  uint32_t injected_gap_count;
  uint32_t last_injected_gap_us;
  uint32_t inject_gap_us_once;
  bool inject_bypass_guard_once;
} adbms6830_session_health_t;

typedef enum
{
  ADBMS6830_POST_IDLE = 0,
  ADBMS6830_POST_BASELINE,
  ADBMS6830_POST_OSC_FAST,
  ADBMS6830_POST_OSC_SLOW,
  ADBMS6830_POST_SUPPLY_UV,
  ADBMS6830_POST_SUPPLY_OV,
  ADBMS6830_POST_THSD_FLAG,
  ADBMS6830_POST_NVM_ED,
  ADBMS6830_POST_NVM_MED,
  ADBMS6830_POST_TMOD,
  ADBMS6830_POST_SPIFLT,
  ADBMS6830_POST_RESTORE_CONFIG,
  ADBMS6830_POST_FINAL_BASELINE,
  ADBMS6830_POST_PASS,
  ADBMS6830_POST_FAIL
} adbms6830_post_stage_t;

typedef struct
{
  adbms6830_post_stage_t stage;
  HAL_StatusTypeDef last_status;
  bool passed;
  uint8_t attempts;
  uint8_t expected_flag_d;
  uint16_t failed_ic_mask;
  uint16_t unexpected_ic_mask;
  uint32_t run_count;
  uint32_t fail_count;
} adbms6830_post_health_t;

typedef struct
{
  uint8_t sensor;
  uint16_t valid_mask;
  uint16_t disagree_mask;
  int16_t aux_raw[ADBMS6830_MAX_TRACKED_ICS];
  int16_t aux2_raw[ADBMS6830_MAX_TRACKED_ICS];
  int16_t delta_mv[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t count;
  uint32_t fail_count;
} adbms6830_aux2_health_t;

typedef struct
{
  uint8_t sensor;
  uint16_t valid_mask;
  uint16_t suspect_mask;
  int16_t baseline_raw[ADBMS6830_MAX_TRACKED_ICS];
  int16_t pulldown_raw[ADBMS6830_MAX_TRACKED_ICS];
  int16_t pullup_raw[ADBMS6830_MAX_TRACKED_ICS];
  int16_t recovery_raw[ADBMS6830_MAX_TRACKED_ICS];
  int16_t pulldown_delta_mv[ADBMS6830_MAX_TRACKED_ICS];
  int16_t pullup_delta_mv[ADBMS6830_MAX_TRACKED_ICS];
  int16_t recovery_delta_mv[ADBMS6830_MAX_TRACKED_ICS];
  uint32_t count;
  uint32_t fail_count;
  uint32_t config_restore_fail_count;
} adbms6830_therm_ow_health_t;

typedef struct
{
  adbms6830_open_wire_path_t path;
  HAL_StatusTypeDef diagnostic_status;
  HAL_StatusTypeDef restore_status;
  uint16_t incomplete_ic_mask;
  uint16_t fault_ic_mask;
  uint16_t cell_fault_mask[ADBMS6830_MAX_TRACKED_ICS];
  bool complete;
  bool restored_normal_c_image;
} adbms6830_open_wire_result_t;

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

  /* Last redundant S-ADC comparison capture status.  One bit per cell is
   * set only when the corresponding RDSV group passed PEC/command-counter
   * validation and contained a non-sentinel result.  This is deliberately
   * separate from normal cell publication: it exists for the bench csdump
   * diagnostic and must never make a primary cell value usable by itself. */
  uint16_t last_scell_updated_mask[ADBMS6830_MAX_TRACKED_ICS];
  uint16_t last_scell_pec_mask[ADBMS6830_MAX_TRACKED_ICS];

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

  adbms6830_temp_debug_t temp_debug;
  adbms6830_temp_bus_debug_t temp_bus_debug;
  adbms6830_temp_bus_scan_t temp_bus_scan;
  adbms6830_sadc_debug_t sadc_debug;
  adbms6830_cadc_debug_t cadc_debug;
  adbms6830_recovery_debug_t recovery_debug;
  adbms6830_spi_debug_t spi_debug;
  adbms6830_cfgb_write_event_t cfgb_write_history[ADBMS6830_CFGB_WRITE_HISTORY_DEPTH];
  uint8_t cfgb_write_history_count;
  uint8_t cfgb_write_history_index;
  uint32_t cfgb_write_total_count;
  adbms6830_ic_diag_t diag[ADBMS6830_MAX_TRACKED_ICS];

  /* Runtime transport hooks are bound by the ADBMS task after the scheduler
   * starts.  Startup code can still use the existing timer busy-wait fallback. */
  adbms6830_cooperative_wait_fn_t cooperative_wait_fn;
  adbms6830_time_us_fn_t time_us_fn;
  void *runtime_hook_ctx;
  adbms6830_session_health_t session;
  adbms6830_post_health_t post;
  adbms6830_aux2_health_t aux2_health;
  adbms6830_therm_ow_health_t therm_ow_health;

  uint16_t last_acell_updated_mask[ADBMS6830_MAX_TRACKED_ICS];
  uint16_t last_acell_pec_mask[ADBMS6830_MAX_TRACKED_ICS];
  uint16_t last_fcell_updated_mask[ADBMS6830_MAX_TRACKED_ICS];
  uint16_t last_fcell_pec_mask[ADBMS6830_MAX_TRACKED_ICS];
  bool filtered_voltage_ready;
  uint8_t filtered_successful_epoch_count;
  uint32_t filter_ready_after_ms;

  /* Only meaningful in the post-S-ECO build. Any ADCV issued by a diagnostic
   * invalidates this state; the normal acquisition path re-establishes the
   * production continuous-C command before it trusts subsequent products. */
  bool continuous_c_running;

  adbms6830_diag_health_t health;
} adbms6830_driver_t;

#endif /* INC_EXT_DRIVERS_ADBMS6830_DATA_H_ */
