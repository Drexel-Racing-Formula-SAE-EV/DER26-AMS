/*
 * current_sensor.h
 *
 *  Created on: Mar 3th, 2024
 *      Author: Justin Nguyen
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
    CURRENT_SENSOR_REASON_NOT_MAPPED
} current_sensor_reason_t;

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

    uint16_t count_high;
    uint16_t count_low;

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
const char *current_sensor_reason_str(current_sensor_reason_t reason);
const char *current_sensor_range_str(current_sensor_range_t range);

#endif /* INC_CURRENT_SENSOR_H_ */
