/*
 * current_sensor.h
 *
 *  Created on: Mar 3th, 2024
 *      Author: Justin Nguyen
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef INC_CURRENT_SENSOR_H_
#define INC_CURRENT_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>
#include <stm32f7xx_hal.h>

typedef enum
{
    CURRENT_SENSOR_RANGE_UNKNOWN = 0,
    CURRENT_SENSOR_RANGE_50A,
    CURRENT_SENSOR_RANGE_800A
} current_sensor_range_t;

typedef enum
{
    CURRENT_SENSOR_REASON_OK = 0,
    CURRENT_SENSOR_REASON_NULL,
    CURRENT_SENSOR_REASON_ADC_READ,
    CURRENT_SENSOR_REASON_ADC_IMPLAUSIBLE,
    CURRENT_SENSOR_REASON_SENSOR_SATURATION,
    CURRENT_SENSOR_REASON_CHANNEL_MISMATCH,
    CURRENT_SENSOR_REASON_NOT_MAPPED,
    CURRENT_SENSOR_REASON_ZERO_CAL_REJECTED,
    CURRENT_SENSOR_REASON_CALIBRATION_CHANGED
} current_sensor_reason_t;

#define CURRENT_SENSOR_CALIBRATION_MAGIC       0x4943414Cu /* 'ICAL' */
#define CURRENT_SENSOR_CALIBRATION_SCHEMA      1u
#define CURRENT_SENSOR_CALIBRATION_RECORD_SIZE 44u
#define CURRENT_SENSOR_CALIBRATION_TIME_UNKNOWN 0u
#define CURRENT_SENSOR_CALIBRATION_UNCERTAINTY_UNKNOWN UINT16_MAX

/*
 * Fixed-width, versioned object for a future board-owned flash/EEPROM adapter.
 * The CRC is calculated field-by-field in a defined little-endian order, so
 * compiler padding is never part of the integrity check.  No current firmware
 * path writes this object to nonvolatile memory.
 */
typedef struct
{
    uint32_t magic;
    uint16_t schema;
    uint16_t size;
    uint32_t calibration_id;
    uint32_t capture_time_s;
    int32_t zero_offset_50a_mA;
    int32_t zero_offset_800a_mA;
    uint32_t adc_vref_uV;
    uint32_t sensor_supply_uV;
    int16_t calibration_temp_deci_c;
    uint16_t uncertainty_50a_mA;
    uint16_t uncertainty_800a_mA;
    uint16_t reserved;
    uint32_t crc32;
} current_sensor_calibration_record_t;

typedef struct
{
    uint32_t calibration_id;
    uint32_t capture_time_s;
    int16_t calibration_temp_deci_c;
    uint16_t uncertainty_50a_mA;
    uint16_t uncertainty_800a_mA;
} current_sensor_calibration_metadata_t;

typedef struct
{
    float current;

    /* MCU-side ADC voltages after analog conditioning/divider. */
    float voltage_high;
    float voltage_low;

    /* DHAB sensor-side voltages after undoing the AMS input divider. */
    float sensor_voltage_high;
    float sensor_voltage_low;

    /* Legacy H/L fields retained for compatibility. C_SENSE_L is design-mapped to the 50 A DHAB channel. */
    float current_high;
    float current_low;

    /* Explicit range-based values used by new code. */
    float current_50a;
    float current_800a;

    /* Raw range currents before software zero-offset/deadband correction. */
    float current_50a_raw;
    float current_800a_raw;

    /* Telemetry-only filtered values. Safety fault logic uses current/current_50a/current_800a, not these. */
    float current_50a_filtered;
    float current_800a_filtered;
    float current_filtered;
    bool filter_initialized;

    /* Software zero-current offset captured only by an explicit service/bench command. */
    float zero_offset_50a;
    float zero_offset_800a;
    bool zero_calibrated;
    uint32_t zero_cal_count;

    /* Persistent-calibration provenance. A service zero capture alone does
     * not set loaded_from_record and is therefore insufficient for formal
     * resistance-SoH confidence. */
    bool calibration_loaded_from_record;
    uint32_t calibration_id;
    uint32_t calibration_capture_time_s;
    int16_t calibration_temp_deci_c;
    uint16_t calibration_uncertainty_50a_mA;
    uint16_t calibration_uncertainty_800a_mA;
    uint32_t calibration_restore_count;

    /* Voltage-reference parameters. Defaults match AMS Rev3.1: STM32 ADC at 3.3 V, DHAB at 5 V. */
    float adc_vref_v;
    float sensor_supply_v;

    uint16_t count_high;
    uint16_t count_low;
    bool count_high_fresh;
    bool count_low_fresh;

    bool last_read_ok;
    bool current_valid;
    current_sensor_range_t selected_range;
    current_sensor_reason_t reason;

    ADC_HandleTypeDef *hadc_high;
    ADC_HandleTypeDef *hadc_low;
    uint32_t channel_high;
    uint32_t channel_low;
} current_sensor_t;

void current_sensor_init(current_sensor_t *dev,
                         ADC_HandleTypeDef *hadc_low,
                         ADC_HandleTypeDef *hadc_high,
                         uint32_t channel_low,
                         uint32_t channel_high);
float current_sensor_current_read(current_sensor_t *dev);
float current_sensor_convert(current_sensor_t *dev);
bool current_sensor_read_adc(current_sensor_t *dev);
void current_sensor_set_reference_voltages(current_sensor_t *dev,
                                           float adc_vref_v,
                                           float sensor_supply_v);
bool current_sensor_zero_calibrate(current_sensor_t *dev);
void current_sensor_zero_clear(current_sensor_t *dev);
bool current_sensor_calibration_record_create(
    const current_sensor_t *dev,
    const current_sensor_calibration_metadata_t *metadata,
    current_sensor_calibration_record_t *record);
bool current_sensor_calibration_record_valid(
    const current_sensor_calibration_record_t *record);
bool current_sensor_calibration_apply(
    current_sensor_t *dev,
    const current_sensor_calibration_record_t *record,
    bool zero_current_proven);
bool current_sensor_calibration_confident(const current_sensor_t *dev);
const char *current_sensor_reason_str(current_sensor_reason_t reason);
const char *current_sensor_range_str(current_sensor_range_t range);

#endif /* INC_CURRENT_SENSOR_H_ */
