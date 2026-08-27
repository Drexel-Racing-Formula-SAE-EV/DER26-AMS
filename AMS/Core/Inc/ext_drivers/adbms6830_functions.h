/*
 * adbms6830_functions.h
 *
 *  Created on: May 13, 2025
 *      Author: realb
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_
#define INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "ext_drivers/adbms6830_data.h"

typedef enum
{
    ADBMS6830_SCOPE_WAKE = 0,
    ADBMS6830_SCOPE_CMD,
    ADBMS6830_SCOPE_READ,
    ADBMS6830_SCOPE_PATTERN
} adbms6830_scope_mode_t;

HAL_StatusTypeDef adBms6830_init(adbms6830_driver_t* dev,
                    uint8_t num_ics,
                    uint8_t physical_chain_count,
                    adbms6830_asic* ics,
                    uint8_t ics_capacity,
				    SPI_HandleTypeDef* hspi,
				    GPIO_TypeDef* cs_port_a,
				    GPIO_TypeDef* cs_port_b,
				    uint16_t cs_pin_a,
				    uint16_t cs_pin_b,
                    adbms_string selected_string,
					TIM_HandleTypeDef *htim);

void adbms6830_reset_cfg(adbms6830_driver_t *dev);
void adbms6830_srst(adbms6830_driver_t *dev);
void adbms6830_wrcfga(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_wrcfga_checked(adbms6830_driver_t *dev);
void adbms6830_wrcfgb(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_wrcfgb_checked(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_wrcfgb_checked_reason(
    adbms6830_driver_t *dev,
    adbms6830_cfgb_write_reason_t reason);
void adbms6830_disable_discharge_timer_shadow(adbms6830_driver_t *dev);
const char *adbms6830_cfgb_write_reason_str(adbms6830_cfgb_write_reason_t reason);
HAL_StatusTypeDef adbms6830_wrpwma_checked(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_wrpwmb_checked(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_write_pwm_checked(adbms6830_driver_t *dev);
void adbms6830_rdcfga(adbms6830_driver_t *dev);
void adbms6830_rdcfgb(adbms6830_driver_t *dev);
void adbms6830_adcv(adbms6830_driver_t *dev, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs);

void adbms6830_wakeup(adbms6830_driver_t* dev);
HAL_StatusTypeDef adbms6830_wakeup_checked(adbms6830_driver_t* dev);
void adbms6830_wakeup_cold(adbms6830_driver_t* dev);

HAL_StatusTypeDef adbms6830_us_delay(adbms6830_driver_t* dev, uint16_t microseconds);
HAL_StatusTypeDef adbms6830_wait_cooperative(adbms6830_driver_t *dev, uint32_t microseconds);
void adbms6830_bind_runtime_hooks(adbms6830_driver_t *dev,
                                  adbms6830_cooperative_wait_fn_t wait_fn,
                                  adbms6830_time_us_fn_t time_fn,
                                  void *ctx);
const adbms6830_session_health_t *adbms6830_session_health_get(const adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_session_inject_gap_once(adbms6830_driver_t *dev,
                                                       uint32_t gap_us,
                                                       bool bypass_guard);
void adbms6830_session_begin_scan(adbms6830_driver_t *dev);
void adbms6830_session_end_scan(adbms6830_driver_t *dev);

HAL_StatusTypeDef adbms6830_start_adc_cell_voltage_measurement(adbms6830_driver_t *dev);
bool adbms6830_set_monitored_cell_count(adbms6830_driver_t *dev, uint8_t cell_count);
void adbms6830_parse_cell(adbms6830_driver_t *dev, uint8_t *data, GRP grp);
HAL_StatusTypeDef adbms6830_read_cell_voltages(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_read_cell_voltage_products(adbms6830_driver_t *dev,
                                                       bool read_avg8,
                                                       bool read_filtered);
HAL_StatusTypeDef adbms6830_mute_checked(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_unmute_checked(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_run_startup_post(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_run_aux2_redundancy(adbms6830_driver_t *dev, uint8_t sensor_num);
HAL_StatusTypeDef adbms6830_run_thermistor_open_wire(adbms6830_driver_t *dev, uint8_t sensor_num);
HAL_StatusTypeDef adbms6830_run_s_periodic_diagnostic(adbms6830_driver_t *dev);


void adbms6830_spi_debug_enable(adbms6830_driver_t *dev, bool enable);
void adbms6830_spi_debug_clear(adbms6830_driver_t *dev);
const adbms6830_spi_debug_t *adbms6830_spi_debug_get(const adbms6830_driver_t *dev);
const char *adbms6830_spi_op_str(adbms6830_spi_op_t op);
HAL_StatusTypeDef adbms6830_spi_probe_rdcfga(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_spi_probe_rdcfga_on_string(adbms6830_driver_t *dev, adbms_string string);
HAL_StatusTypeDef adbms6830_scope_activity(adbms6830_driver_t *dev,
                                           adbms_string string,
                                           adbms6830_scope_mode_t mode,
                                           uint16_t repeat_count);
HAL_StatusTypeDef adbms6830_read_sid(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_read_status(adbms6830_driver_t *dev, bool inject_spiflt);
HAL_StatusTypeDef adbms6830_refresh_diagnostics(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_establish_diagnostic_baseline(adbms6830_driver_t *dev);
/* Transport/integrity validity is intentionally separate from the diagnostic
 * fault policy.  A clean Status A-E/SID image can be transported correctly
 * while CSxFLT legitimately reports the known S-routing defect. */
bool adbms6830_diagnostic_transport_ok(const adbms6830_driver_t *dev);
/* True when all status/reference classes other than C-vs-S redundancy are
 * healthy.  CSxFLT is reported separately and remains safety-gating in the
 * normal REDUNDANT_CS build. */
bool adbms6830_non_cs_diagnostics_ok(const adbms6830_driver_t *dev);
bool adbms6830_safety_diagnostics_ok(const adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_clear_all_flags(adbms6830_driver_t *dev);
const adbms6830_diag_health_t *adbms6830_diag_health_get(const adbms6830_driver_t *dev);
void adbms6830_diag_health_clear(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_verify_config_readback(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_verify_balance_readback(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_run_cell_adc_self_test(adbms6830_driver_t *dev);
/* Bench diagnostic: recreate the startup redundant conversion, snapshot both
 * primary C-ADC and redundant S-ADC registers, then read the fresh CSxFLT
 * image.  A HAL_OK result means the capture is complete and transport-clean;
 * CSxFLT may still be nonzero and is reported through dev->diag[]. */
HAL_StatusTypeDef adbms6830_capture_cs_comparison(adbms6830_driver_t *dev);
/* Bench diagnostic: issue one standalone ADSV conversion with open-wire and
 * discharge switches disabled, then retain raw RDSVA-RDSVF packets and decode
 * only transport-valid S-cell codes.  It does not change application safety
 * readiness, BMS_OK, balancing state, or primary C-ADC publication. */
HAL_StatusTypeDef adbms6830_capture_s_adc(adbms6830_driver_t *dev);
/* Standalone C-ADC capture with PLCADC completion polling. */
HAL_StatusTypeDef adbms6830_capture_c_adc(adbms6830_driver_t *dev);
/* Profile one C, S or AUX conversion using the corresponding polling command. */
HAL_StatusTypeDef adbms6830_profile_conversion_timing(
    adbms6830_driver_t *dev,
    adbms6830_timing_kind_t kind,
    adbms6830_timing_result_t *result);
/* One safe configuration write/readback cycle. The caller must ensure all
 * discharge/PWM shadows are zero before using this service diagnostic. */
HAL_StatusTypeDef adbms6830_config_write_readback_cycle(
    adbms6830_driver_t *dev,
    adbms6830_config_cycle_result_t *result);
/* Re-establish identity, configuration, references and one C-ADC image after
 * an externally imposed idle/disconnect interval. Application safety latches
 * and readiness are not modified by this diagnostic sequence. */
HAL_StatusTypeDef adbms6830_recovery_check(adbms6830_driver_t *dev);
/* Deterministic FNV-1a fingerprints over the per-IC CFGA/CFGB desired image
 * and the most recent readback image.  These are diagnostics, not cryptographic
 * integrity checks; byte-by-byte comparison remains the safety decision. */
uint32_t adbms6830_config_expected_fingerprint(const adbms6830_driver_t *dev);
uint32_t adbms6830_config_readback_fingerprint(const adbms6830_driver_t *dev);
/* Read one named register group without decoding it. */
HAL_StatusTypeDef adbms6830_read_raw_register(adbms6830_driver_t *dev,
                                              adbms6830_raw_register_t reg,
                                              adbms6830_raw_read_t *result);
const char *adbms6830_raw_register_name(adbms6830_raw_register_t reg);
const char *adbms6830_timing_kind_name(adbms6830_timing_kind_t kind);
/* Explicit open-wire path selection. The C path is used while the known SMB
 * S2N-S15N routing defect is under repair; callers must restore a normal C
 * conversion before publishing voltage authority. Legacy APIs remain S-path
 * wrappers for compatibility. */
HAL_StatusTypeDef adbms6830_run_open_wire_check_path(
    adbms6830_driver_t *dev,
    adbms6830_open_wire_path_t path,
    bool odd_channels);
HAL_StatusTypeDef adbms6830_run_open_wire_diagnostic_path(
    adbms6830_driver_t *dev,
    adbms6830_open_wire_path_t path);
HAL_StatusTypeDef adbms6830_run_open_wire_check(adbms6830_driver_t *dev, bool odd_channels);
HAL_StatusTypeDef adbms6830_run_open_wire_diagnostic(adbms6830_driver_t *dev);
HAL_StatusTypeDef adbms6830_run_aux_gpio_diagnostic(adbms6830_driver_t *dev);
/* Commands sent from the other end of a reversible mixed chain still advance
 * every compatible device's command counter. Record a known count, or discard
 * the prediction after an ambiguous transport failure. */
void adbms6830_note_external_counter_increments(adbms6830_driver_t *dev,
                                                 uint8_t increment_count);
void adbms6830_resync_command_counter_tracking(adbms6830_driver_t *dev);

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
int adbms6830_read_temp_raw(adbms6830_driver_t *dev,
                            uint8_t ic_idx,
                            uint8_t sensor_num,
                            int16_t *out_raw);
float adbms6830_convert_temp(adbms6830_driver_t *dev, uint8_t ic_idx, uint8_t sensor_num, float vref);
/* Legacy wrapper: input is the signed DER26 ADBMS AUX code. */
float voltage_to_temp(float v);
int mux_set_channel(adbms6830_driver_t *dev, uint8_t sensor_num);

/* Pure helpers shared by the live ADG728 path and software-only validation.
 * Keeping them inline makes the host test and target use the exact same route
 * and ACK interpretation without introducing a second implementation. */
static inline bool adbms6830_temp_sensor_route(
    uint8_t sensor_num,
    adbms6830_temp_route_t *route_out)
{
    if((sensor_num >= ADBMS6830_TEMP_SENSOR_COUNT) || (route_out == NULL))
    {
        return false;
    }

    route_out->sensor_num = sensor_num;
    route_out->mux_idx = (uint8_t)(sensor_num / 8u);
    route_out->mux_address = (uint8_t)(0x4Cu + route_out->mux_idx);
    route_out->switch_index = (uint8_t)(sensor_num % 8u);
    route_out->switch_mask = (uint8_t)(1u << route_out->switch_index);
    route_out->gpio_channel = route_out->mux_idx;
    return true;
}

static inline bool adbms6830_comm_address_acknowledged(
    const uint8_t packet[RX_DATA])
{
    /* ADBMS6830B Table 35 post-STCOMM readback:
     * ICOMM0=0x6 means the master generated START and FCOMM0=0x7 means
     * the slave generated ACK on the address byte. */
    return (packet != NULL) &&
           ((packet[0] >> 4u) == 0x06u) &&
           ((packet[0] & 0x0Fu) == 0x07u);
}

static inline bool adbms6830_comm_data_acknowledged(
    const uint8_t packet[RX_DATA])
{
    uint8_t icomm1;

    if(packet == NULL)
    {
        return false;
    }

    /* After STCOMM, ICOMM1 is a readback of the actual inter-byte SDA
     * state, not necessarily the 0x0 blank code that was written:
     *   0x0 = blank, SDA held low between bytes
     *   0x7 = blank, SDA held high between bytes
     * Both are valid blank readback states. FCOMM1=0x7 is the actual
     * slave-generated ACK for the data byte. Real ADG728 hardware returns
     * 0x77 here when SDA rises between the address and data bytes. */
    icomm1 = (uint8_t)(packet[2] >> 4u);
    return ((icomm1 == 0x00u) || (icomm1 == 0x07u)) &&
           ((packet[2] & 0x0Fu) == 0x07u);
}

static inline bool adbms6830_comm_write_acknowledged(
    const uint8_t packet[RX_DATA])
{
    return adbms6830_comm_address_acknowledged(packet) &&
           adbms6830_comm_data_acknowledged(packet);
}

/* Bench-only diagnostic capture. It records raw RDCOMM and RDAUXA packets,
 * decoded ACK masks, stage statuses, and optionally performs an AUX capture
 * even when the ADG728 write did not acknowledge. Forced samples are never
 * marked publishable unless the normal acknowledged selection path passed. */
HAL_StatusTypeDef adbms6830_temp_debug_capture(adbms6830_driver_t *dev,
                                                uint8_t sensor_num,
                                                bool force_aux_capture);

/* Non-driving bench diagnostic for the ADG728 bus. It samples GPIO4/SDA and
 * GPIO5/SCL through RDAUXB without issuing WRCOMM or STCOMM. */
HAL_StatusTypeDef adbms6830_temp_bus_idle_capture(adbms6830_driver_t *dev);

/* Active service diagnostic for the complete ADG728 address range 0x4C-0x4F.
 * It writes control byte 0x00 (all switches open), records per-address ACK and
 * transport evidence, never publishes temperature data, and invalidates cached
 * mux selections before returning. */
HAL_StatusTypeDef adbms6830_temp_bus_scan_capture(adbms6830_driver_t *dev);

#endif /* INC_EXT_DRIVERS_ADBMS6830_FUNCTIONS_H_ */
