/*
 * current_sensor.c
 *
 *  Created on: Mar 3th, 2024
 *      Author: Justin Nguyen
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#include <ext_drivers/current_sensor.h>
#include <ext_drivers/stm32f767z.h>
#include <math.h>

#define CURRENT_ADC_NOMINAL_VREF_V              3.3f
#define CURRENT_SENSOR_NOMINAL_SUPPLY_V         5.0f
#define CURRENT_ADC_MAX_COUNT                   4095.0f
#define CURRENT_ADC_TIMEOUT_MS                  5u

#define DHAB_NOMINAL_OFFSET_V                   2.5f
#define DHAB_CH_50A_SENS_V_PER_A_AT_5V          0.040f
#define DHAB_CH_800A_SENS_V_PER_A_AT_5V         0.0025f

#define SENSOR_DIVIDER_TOP_OHM                  100000.0f
#define SENSOR_DIVIDER_BOTTOM_OHM               150000.0f
#define SENSOR_DIVIDER_GAIN                     (SENSOR_DIVIDER_BOTTOM_OHM / (SENSOR_DIVIDER_TOP_OHM + SENSOR_DIVIDER_BOTTOM_OHM))

#define CURRENT_ADC_IMPLAUS_LOW_COUNT           100u
#define CURRENT_ADC_IMPLAUS_HIGH_COUNT          3800u

#define DHAB_SENSOR_VALID_MIN_V                 0.20f
#define DHAB_SENSOR_VALID_MAX_V                 4.80f
#define DHAB_SENSOR_CLAMP_LOW_V                 0.30f
#define DHAB_SENSOR_CLAMP_HIGH_V                4.70f

#define CURRENT_50A_USE_LIMIT_A                 45.0f
#define CURRENT_800A_RETURN_TO_50A_LIMIT_A      38.0f
#define CURRENT_CHANNEL_COMPARE_MIN_A           10.0f
#define CURRENT_CHANNEL_AGREE_ABS_A             7.5f
#define CURRENT_CHANNEL_AGREE_PCT               0.15f

#define CURRENT_ZERO_CAPTURE_MAX_50A_A          5.0f
#define CURRENT_ZERO_CAPTURE_MAX_800A_A         25.0f
#define CURRENT_ZERO_OFFSET_MAX_50A_A           5.0f
#define CURRENT_ZERO_OFFSET_MAX_800A_A          25.0f

#define CURRENT_50A_DEADBAND_A                  0.25f
#define CURRENT_800A_DEADBAND_A                 2.0f
#define CURRENT_FILTER_ALPHA                    0.25f

#define CURRENT_ADC_VREF_MIN_V                  2.8f
#define CURRENT_ADC_VREF_MAX_V                  3.6f
#define CURRENT_SENSOR_SUPPLY_MIN_V             4.5f
#define CURRENT_SENSOR_SUPPLY_MAX_V             5.5f

/*
 * Design-file mapping:
 * - C_SENSE_L is buffered outward to the BSPD Hall-effect sensor input.
 * - BSPD math uses V = I * 0.04 + 2.5, which is the DHAB CH1 / +/-50 A
 *   output.
 * Therefore C_SENSE_L is the 50 A channel and C_SENSE_H is the 800 A
 * channel. Keep the macro explicit so a physical harness mismatch can still
 * be corrected quickly after continuity/bench checks.
 */
#define CURRENT_SENSOR_50A_CHANNEL_IS_HIGH 0u

static bool finite_in_range(float value, float min_value, float max_value)
{
    return isfinite(value) && (value >= min_value) && (value <= max_value);
}

static float current_sensor_adc_count_to_voltage(const current_sensor_t *dev, uint16_t count)
{
    float adc_vref = CURRENT_ADC_NOMINAL_VREF_V;

    if((dev != NULL) && finite_in_range(dev->adc_vref_v, CURRENT_ADC_VREF_MIN_V, CURRENT_ADC_VREF_MAX_V))
    {
        adc_vref = dev->adc_vref_v;
    }

    return ((float)count * adc_vref) / CURRENT_ADC_MAX_COUNT;
}

static float current_sensor_adc_voltage_to_sensor_voltage(float adc_voltage)
{
    return adc_voltage / SENSOR_DIVIDER_GAIN;
}

static float current_sensor_supply_v(const current_sensor_t *dev)
{
    if((dev != NULL) && finite_in_range(dev->sensor_supply_v,
                                        CURRENT_SENSOR_SUPPLY_MIN_V,
                                        CURRENT_SENSOR_SUPPLY_MAX_V))
    {
        return dev->sensor_supply_v;
    }

    return CURRENT_SENSOR_NOMINAL_SUPPLY_V;
}

static float current_sensor_offset_voltage(const current_sensor_t *dev)
{
    return current_sensor_supply_v(dev) * 0.5f;
}

static float current_sensor_scaled_sensitivity(const current_sensor_t *dev, float sensitivity_at_5v)
{
    return sensitivity_at_5v * (current_sensor_supply_v(dev) / CURRENT_SENSOR_NOMINAL_SUPPLY_V);
}

static float current_sensor_voltage_to_current(const current_sensor_t *dev,
                                               float sensor_voltage,
                                               float sensitivity_v_per_a_at_5v)
{
    float sensitivity = current_sensor_scaled_sensitivity(dev, sensitivity_v_per_a_at_5v);

    if((sensitivity <= 0.0f) || !isfinite(sensitivity))
    {
        return 0.0f;
    }

    return (sensor_voltage - current_sensor_offset_voltage(dev)) / sensitivity;
}

static float current_sensor_apply_deadband(float current_a, float deadband_a)
{
    if(!isfinite(current_a))
    {
        return current_a;
    }

    return (fabsf(current_a) < deadband_a) ? 0.0f : current_a;
}

static float current_sensor_filter(float previous, float sample)
{
    return previous + (CURRENT_FILTER_ALPHA * (sample - previous));
}

static bool current_sensor_adc_count_implausible(uint16_t count)
{
    return (count < CURRENT_ADC_IMPLAUS_LOW_COUNT) ||
           (count > CURRENT_ADC_IMPLAUS_HIGH_COUNT);
}

static bool current_sensor_voltage_outside_sensor_range(float sensor_voltage)
{
    return (sensor_voltage < DHAB_SENSOR_VALID_MIN_V) ||
           (sensor_voltage > DHAB_SENSOR_VALID_MAX_V) ||
           !isfinite(sensor_voltage);
}

static bool current_sensor_voltage_at_clamp(float sensor_voltage)
{
    return (sensor_voltage <= DHAB_SENSOR_CLAMP_LOW_V) ||
           (sensor_voltage >= DHAB_SENSOR_CLAMP_HIGH_V);
}

static void current_sensor_set_invalid(current_sensor_t *dev, current_sensor_reason_t reason)
{
    if(dev == NULL)
    {
        return;
    }

    dev->current_valid = false;
    dev->selected_range = CURRENT_SENSOR_RANGE_UNKNOWN;
    dev->reason = reason;
}

static bool current_sensor_offsets_usable(const current_sensor_t *dev)
{
    if((dev == NULL) || !dev->zero_calibrated)
    {
        return false;
    }

    return finite_in_range(dev->zero_offset_50a,
                           -CURRENT_ZERO_OFFSET_MAX_50A_A,
                           CURRENT_ZERO_OFFSET_MAX_50A_A) &&
           finite_in_range(dev->zero_offset_800a,
                           -CURRENT_ZERO_OFFSET_MAX_800A_A,
                           CURRENT_ZERO_OFFSET_MAX_800A_A);
}

static void current_sensor_update_filter(current_sensor_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

    if(!dev->filter_initialized)
    {
        dev->current_50a_filtered = dev->current_50a;
        dev->current_800a_filtered = dev->current_800a;
        dev->current_filtered = dev->current;
        dev->filter_initialized = true;
        return;
    }

    dev->current_50a_filtered = current_sensor_filter(dev->current_50a_filtered, dev->current_50a);
    dev->current_800a_filtered = current_sensor_filter(dev->current_800a_filtered, dev->current_800a);
    dev->current_filtered = current_sensor_filter(dev->current_filtered, dev->current);
}

const char *current_sensor_reason_str(current_sensor_reason_t reason)
{
    switch(reason)
    {
        case CURRENT_SENSOR_REASON_OK:                return "ok";
        case CURRENT_SENSOR_REASON_NULL:              return "null";
        case CURRENT_SENSOR_REASON_ADC_READ:          return "adc_read";
        case CURRENT_SENSOR_REASON_ADC_IMPLAUSIBLE:   return "adc_implausible";
        case CURRENT_SENSOR_REASON_SENSOR_SATURATION: return "sensor_saturation";
        case CURRENT_SENSOR_REASON_CHANNEL_MISMATCH:  return "channel_mismatch";
        case CURRENT_SENSOR_REASON_NOT_MAPPED:        return "not_mapped";
        case CURRENT_SENSOR_REASON_ZERO_CAL_REJECTED: return "zero_cal_rejected";
        default:                                      return "unknown";
    }
}

const char *current_sensor_range_str(current_sensor_range_t range)
{
    switch(range)
    {
        case CURRENT_SENSOR_RANGE_50A:     return "50A";
        case CURRENT_SENSOR_RANGE_800A:    return "800A";
        case CURRENT_SENSOR_RANGE_UNKNOWN:
        default:                           return "unknown";
    }
}

void current_sensor_set_reference_voltages(current_sensor_t *dev,
                                           float adc_vref_v,
                                           float sensor_supply_v)
{
    if(dev == NULL)
    {
        return;
    }

    if(finite_in_range(adc_vref_v, CURRENT_ADC_VREF_MIN_V, CURRENT_ADC_VREF_MAX_V))
    {
        dev->adc_vref_v = adc_vref_v;
    }

    if(finite_in_range(sensor_supply_v, CURRENT_SENSOR_SUPPLY_MIN_V, CURRENT_SENSOR_SUPPLY_MAX_V))
    {
        dev->sensor_supply_v = sensor_supply_v;
    }
}

void current_sensor_init(current_sensor_t *dev,
                         ADC_HandleTypeDef *hadc_low,
                         ADC_HandleTypeDef *hadc_high,
                         uint32_t channel_low,
                         uint32_t channel_high)
{
    if(dev == NULL)
    {
        return;
    }

    dev->hadc_low = hadc_low;
    dev->hadc_high = hadc_high;
    dev->channel_low = channel_low;
    dev->channel_high = channel_high;
    dev->voltage_high = 0.0f;
    dev->voltage_low = 0.0f;
    dev->sensor_voltage_high = 0.0f;
    dev->sensor_voltage_low = 0.0f;
    dev->current_low = 0.0f;
    dev->current_high = 0.0f;
    dev->current_50a = 0.0f;
    dev->current_800a = 0.0f;
    dev->current_50a_raw = 0.0f;
    dev->current_800a_raw = 0.0f;
    dev->current_50a_filtered = 0.0f;
    dev->current_800a_filtered = 0.0f;
    dev->current_filtered = 0.0f;
    dev->filter_initialized = false;
    dev->zero_offset_50a = 0.0f;
    dev->zero_offset_800a = 0.0f;
    dev->zero_calibrated = false;
    dev->zero_cal_count = 0u;
    dev->adc_vref_v = CURRENT_ADC_NOMINAL_VREF_V;
    dev->sensor_supply_v = CURRENT_SENSOR_NOMINAL_SUPPLY_V;
    dev->current = 0.0f;
    dev->count_high = 0u;
    dev->count_low = 0u;
    dev->count_high_fresh = false;
    dev->count_low_fresh = false;
    dev->last_read_ok = false;
    dev->current_valid = false;
    dev->selected_range = CURRENT_SENSOR_RANGE_UNKNOWN;
    dev->reason = CURRENT_SENSOR_REASON_ADC_READ;
}

float current_sensor_convert(current_sensor_t *dev)
{
    float sensor_voltage_50a;
    float sensor_voltage_800a;
    float agreement_limit;
    current_sensor_range_t previous_range;
    float low_range_limit;

    if(dev == NULL)
    {
        return 0.0f;
    }

    previous_range = dev->selected_range;

    if((!dev->last_read_ok) ||
       (!dev->count_high_fresh) ||
       (!dev->count_low_fresh))
    {
        current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_READ);
        return dev->current;
    }

    dev->voltage_low  = current_sensor_adc_count_to_voltage(dev, dev->count_low);
    dev->voltage_high = current_sensor_adc_count_to_voltage(dev, dev->count_high);

    /*
     * AMS Rev3.1 conditions both DHAB current sensor outputs with a
     * 100k/150k divider before the STM32 ADC, scaling the sensor's 0..5 V
     * output range to roughly 0..3 V at the MCU. Undo that divider before
     * applying the DHAB offset/sensitivity math.
     */
    dev->sensor_voltage_low = current_sensor_adc_voltage_to_sensor_voltage(dev->voltage_low);
    dev->sensor_voltage_high = current_sensor_adc_voltage_to_sensor_voltage(dev->voltage_high);

    if(current_sensor_adc_count_implausible(dev->count_low) ||
       current_sensor_adc_count_implausible(dev->count_high))
    {
        current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_IMPLAUSIBLE);
        return dev->current;
    }

    if(current_sensor_voltage_outside_sensor_range(dev->sensor_voltage_low) ||
       current_sensor_voltage_outside_sensor_range(dev->sensor_voltage_high))
    {
        current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_IMPLAUSIBLE);
        return dev->current;
    }

#if CURRENT_SENSOR_50A_CHANNEL_IS_HIGH
    sensor_voltage_50a = dev->sensor_voltage_high;
    sensor_voltage_800a = dev->sensor_voltage_low;
#else
    sensor_voltage_50a = dev->sensor_voltage_low;
    sensor_voltage_800a = dev->sensor_voltage_high;
#endif

    dev->current_50a_raw = current_sensor_voltage_to_current(dev,
                                                             sensor_voltage_50a,
                                                             DHAB_CH_50A_SENS_V_PER_A_AT_5V);
    dev->current_800a_raw = current_sensor_voltage_to_current(dev,
                                                              sensor_voltage_800a,
                                                              DHAB_CH_800A_SENS_V_PER_A_AT_5V);

    dev->current_50a = dev->current_50a_raw;
    dev->current_800a = dev->current_800a_raw;

    if(current_sensor_offsets_usable(dev))
    {
        dev->current_50a -= dev->zero_offset_50a;
        dev->current_800a -= dev->zero_offset_800a;
    }

    dev->current_50a = current_sensor_apply_deadband(dev->current_50a, CURRENT_50A_DEADBAND_A);
    dev->current_800a = current_sensor_apply_deadband(dev->current_800a, CURRENT_800A_DEADBAND_A);

#if CURRENT_SENSOR_50A_CHANNEL_IS_HIGH
    dev->current_high = dev->current_50a;
    dev->current_low = dev->current_800a;
#else
    dev->current_high = dev->current_800a;
    dev->current_low = dev->current_50a;
#endif

    low_range_limit = (previous_range == CURRENT_SENSOR_RANGE_800A) ?
                      CURRENT_800A_RETURN_TO_50A_LIMIT_A :
                      CURRENT_50A_USE_LIMIT_A;

    if((fabsf(dev->current_50a) <= low_range_limit) &&
       !current_sensor_voltage_at_clamp(sensor_voltage_50a))
    {
        dev->current = dev->current_50a;
        dev->selected_range = CURRENT_SENSOR_RANGE_50A;

        if((fabsf(dev->current_50a) >= CURRENT_CHANNEL_COMPARE_MIN_A) &&
           !current_sensor_voltage_at_clamp(sensor_voltage_800a))
        {
            agreement_limit = CURRENT_CHANNEL_AGREE_ABS_A +
                              (CURRENT_CHANNEL_AGREE_PCT * fabsf(dev->current_50a));

            if(fabsf(dev->current_50a - dev->current_800a) > agreement_limit)
            {
                current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_CHANNEL_MISMATCH);
                return dev->current;
            }
        }
    }
    else
    {
        if(current_sensor_voltage_at_clamp(sensor_voltage_800a))
        {
            current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_SENSOR_SATURATION);
            return dev->current;
        }

        dev->current = dev->current_800a;
        dev->selected_range = CURRENT_SENSOR_RANGE_800A;
    }

    if(!isfinite(dev->current))
    {
        current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_IMPLAUSIBLE);
        return dev->current;
    }

    dev->current_valid = true;
    dev->reason = CURRENT_SENSOR_REASON_OK;
    current_sensor_update_filter(dev);
    return dev->current;
}

bool current_sensor_zero_calibrate(current_sensor_t *dev)
{
    float offset_50a;
    float offset_800a;

    if(dev == NULL)
    {
        return false;
    }

    if((!dev->last_read_ok) ||
       (!dev->count_high_fresh) ||
       (!dev->count_low_fresh) ||
       !isfinite(dev->current_50a_raw) ||
       !isfinite(dev->current_800a_raw))
    {
        dev->reason = CURRENT_SENSOR_REASON_ZERO_CAL_REJECTED;
        return false;
    }

    offset_50a = dev->current_50a_raw;
    offset_800a = dev->current_800a_raw;

    if((fabsf(offset_50a) > CURRENT_ZERO_CAPTURE_MAX_50A_A) ||
       (fabsf(offset_800a) > CURRENT_ZERO_CAPTURE_MAX_800A_A))
    {
        dev->reason = CURRENT_SENSOR_REASON_ZERO_CAL_REJECTED;
        return false;
    }

    dev->zero_offset_50a = offset_50a;
    dev->zero_offset_800a = offset_800a;
    dev->zero_calibrated = true;
    dev->zero_cal_count++;
    dev->filter_initialized = false;
    return true;
}

void current_sensor_zero_clear(current_sensor_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

    dev->zero_offset_50a = 0.0f;
    dev->zero_offset_800a = 0.0f;
    dev->zero_calibrated = false;
    dev->filter_initialized = false;
}

bool current_sensor_read_adc(current_sensor_t *dev)
{
    stm32f767z_adc_read_result_t high_result;
    stm32f767z_adc_read_result_t low_result;

    if(dev == NULL)
    {
        return false;
    }

    dev->count_high_fresh = false;
    dev->count_low_fresh = false;
    dev->last_read_ok = false;

    if((dev->hadc_high == NULL) || (dev->hadc_low == NULL))
    {
        dev->last_read_ok = false;
        current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_READ);
        return false;
    }

    if(stm32f767z_adc_switch_channel(dev->hadc_high, dev->channel_high) != HAL_OK)
    {
        dev->last_read_ok = false;
        current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_READ);
        return false;
    }

    high_result = stm32f767z_adc_read_checked(dev->hadc_high, CURRENT_ADC_TIMEOUT_MS);
    if(high_result.status == HAL_OK)
    {
        dev->count_high = high_result.count;
        dev->count_high_fresh = true;
    }
    if(high_result.status != HAL_OK)
    {
        dev->last_read_ok = false;
        current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_READ);
        return false;
    }

    if(stm32f767z_adc_switch_channel(dev->hadc_low, dev->channel_low) != HAL_OK)
    {
        dev->last_read_ok = false;
        current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_READ);
        return false;
    }

    low_result = stm32f767z_adc_read_checked(dev->hadc_low, CURRENT_ADC_TIMEOUT_MS);
    if(low_result.status == HAL_OK)
    {
        dev->count_low = low_result.count;
        dev->count_low_fresh = true;
    }
    if(low_result.status != HAL_OK)
    {
        dev->last_read_ok = false;
        current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_READ);
        return false;
    }

    dev->last_read_ok = true;
    return true;
}

float current_sensor_current_read(current_sensor_t *dev)
{
    if(dev == NULL)
    {
        return 0.0f;
    }

    if(current_sensor_read_adc(dev))
    {
        return current_sensor_convert(dev);
    }

    return dev->current;
}
