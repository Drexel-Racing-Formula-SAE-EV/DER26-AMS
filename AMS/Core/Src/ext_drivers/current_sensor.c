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

#define CURRENT_CAL_TEMP_MIN_DECI_C             (-400)
#define CURRENT_CAL_TEMP_MAX_DECI_C             1200
#define CURRENT_CAL_CONFIDENT_50A_UNCERTAINTY_MA 500u
#define CURRENT_CAL_CONFIDENT_800A_UNCERTAINTY_MA 5000u

_Static_assert(sizeof(current_sensor_calibration_record_t) ==
               CURRENT_SENSOR_CALIBRATION_RECORD_SIZE,
               "current calibration record layout changed");

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

static uint32_t current_cal_crc32_byte(uint32_t crc, uint8_t value)
{
    crc ^= value;
    for(uint8_t bit = 0u; bit < 8u; bit++)
    {
        uint32_t mask = 0u - (crc & 1u);
        crc = (crc >> 1u) ^ (0xEDB88320u & mask);
    }
    return crc;
}

static uint32_t current_cal_crc32_u16(uint32_t crc, uint16_t value)
{
    crc = current_cal_crc32_byte(crc, (uint8_t)(value & 0xFFu));
    return current_cal_crc32_byte(crc, (uint8_t)(value >> 8u));
}

static uint32_t current_cal_crc32_u32(uint32_t crc, uint32_t value)
{
    for(uint8_t byte = 0u; byte < 4u; byte++)
    {
        crc = current_cal_crc32_byte(
            crc, (uint8_t)(value >> (8u * byte)));
    }
    return crc;
}

static uint32_t current_cal_record_crc(
    const current_sensor_calibration_record_t *record)
{
    if(record == NULL)
    {
        return 0u;
    }

    uint32_t crc = UINT32_MAX;
    crc = current_cal_crc32_u32(crc, record->magic);
    crc = current_cal_crc32_u16(crc, record->schema);
    crc = current_cal_crc32_u16(crc, record->size);
    crc = current_cal_crc32_u32(crc, record->calibration_id);
    crc = current_cal_crc32_u32(crc, record->capture_time_s);
    crc = current_cal_crc32_u32(crc,
                                (uint32_t)record->zero_offset_50a_mA);
    crc = current_cal_crc32_u32(crc,
                                (uint32_t)record->zero_offset_800a_mA);
    crc = current_cal_crc32_u32(crc, record->adc_vref_uV);
    crc = current_cal_crc32_u32(crc, record->sensor_supply_uV);
    crc = current_cal_crc32_u16(
        crc, (uint16_t)record->calibration_temp_deci_c);
    crc = current_cal_crc32_u16(crc, record->uncertainty_50a_mA);
    crc = current_cal_crc32_u16(crc, record->uncertainty_800a_mA);
    crc = current_cal_crc32_u16(crc, record->reserved);
    return ~crc;
}

static void current_sensor_clear_calibration_provenance(current_sensor_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

    dev->calibration_loaded_from_record = false;
    dev->calibration_id = 0u;
    dev->calibration_capture_time_s = CURRENT_SENSOR_CALIBRATION_TIME_UNKNOWN;
    dev->calibration_temp_deci_c = 0;
    dev->calibration_uncertainty_50a_mA =
        CURRENT_SENSOR_CALIBRATION_UNCERTAINTY_UNKNOWN;
    dev->calibration_uncertainty_800a_mA =
        CURRENT_SENSOR_CALIBRATION_UNCERTAINTY_UNKNOWN;
}

static void current_sensor_mark_calibration_changed(current_sensor_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

    /* A value converted under the previous references/offsets is stale even
     * when its ADC counts are fresh. The owner must run conversion again
     * before publishing current as valid. */
    dev->current_valid = false;
    dev->selected_range = CURRENT_SENSOR_RANGE_UNKNOWN;
    dev->reason = CURRENT_SENSOR_REASON_CALIBRATION_CHANGED;
    dev->filter_initialized = false;
}

static bool current_sensor_live_zero_for_record(
    const current_sensor_t *dev,
    const current_sensor_calibration_record_t *record,
    float *live_50a_A,
    float *live_800a_A)
{
    if((dev == NULL) || (record == NULL) || (live_50a_A == NULL) ||
       (live_800a_A == NULL) || !dev->last_read_ok ||
       !dev->count_high_fresh || !dev->count_low_fresh ||
       (dev->count_high < CURRENT_ADC_IMPLAUS_LOW_COUNT) ||
       (dev->count_high > CURRENT_ADC_IMPLAUS_HIGH_COUNT) ||
       (dev->count_low < CURRENT_ADC_IMPLAUS_LOW_COUNT) ||
       (dev->count_low > CURRENT_ADC_IMPLAUS_HIGH_COUNT))
    {
        return false;
    }

    float adc_vref_v = (float)record->adc_vref_uV / 1000000.0f;
    float sensor_supply_v = (float)record->sensor_supply_uV / 1000000.0f;
    float sensor_low_v =
        (((float)dev->count_low * adc_vref_v) / CURRENT_ADC_MAX_COUNT) /
        SENSOR_DIVIDER_GAIN;
    float sensor_high_v =
        (((float)dev->count_high * adc_vref_v) / CURRENT_ADC_MAX_COUNT) /
        SENSOR_DIVIDER_GAIN;

    if(!finite_in_range(sensor_low_v,
                        DHAB_SENSOR_VALID_MIN_V,
                        DHAB_SENSOR_VALID_MAX_V) ||
       !finite_in_range(sensor_high_v,
                        DHAB_SENSOR_VALID_MIN_V,
                        DHAB_SENSOR_VALID_MAX_V))
    {
        return false;
    }

    float offset_v = sensor_supply_v * 0.5f;
    float sensitivity_50a = DHAB_CH_50A_SENS_V_PER_A_AT_5V *
                            (sensor_supply_v /
                             CURRENT_SENSOR_NOMINAL_SUPPLY_V);
    float sensitivity_800a = DHAB_CH_800A_SENS_V_PER_A_AT_5V *
                             (sensor_supply_v /
                              CURRENT_SENSOR_NOMINAL_SUPPLY_V);
    if(!finite_in_range(sensitivity_50a, 0.001f, 1.0f) ||
       !finite_in_range(sensitivity_800a, 0.0001f, 1.0f))
    {
        return false;
    }

#if CURRENT_SENSOR_50A_CHANNEL_IS_HIGH
    float sensor_50a_v = sensor_high_v;
    float sensor_800a_v = sensor_low_v;
#else
    float sensor_50a_v = sensor_low_v;
    float sensor_800a_v = sensor_high_v;
#endif

    *live_50a_A = (sensor_50a_v - offset_v) / sensitivity_50a;
    *live_800a_A = (sensor_800a_v - offset_v) / sensitivity_800a;
    return finite_in_range(*live_50a_A,
                           -CURRENT_ZERO_CAPTURE_MAX_50A_A,
                           CURRENT_ZERO_CAPTURE_MAX_50A_A) &&
           finite_in_range(*live_800a_A,
                           -CURRENT_ZERO_CAPTURE_MAX_800A_A,
                           CURRENT_ZERO_CAPTURE_MAX_800A_A);
}

static bool current_sensor_record_matches_live_zero(
    const current_sensor_t *dev,
    const current_sensor_calibration_record_t *record)
{
    if((dev == NULL) || (record == NULL))
    {
        return false;
    }

    float live_50a_A = 0.0f;
    float live_800a_A = 0.0f;
    if(!current_sensor_live_zero_for_record(dev,
                                            record,
                                            &live_50a_A,
                                            &live_800a_A))
    {
        return false;
    }

    float stored_50a_A = (float)record->zero_offset_50a_mA / 1000.0f;
    float stored_800a_A = (float)record->zero_offset_800a_mA / 1000.0f;
    float tolerance_50a_A =
        (float)record->uncertainty_50a_mA / 1000.0f;
    float tolerance_800a_A =
        (float)record->uncertainty_800a_mA / 1000.0f;

    /* Never claim sub-resolution agreement, and never let an intentionally
     * low-confidence record enlarge the restore window beyond the confidence
     * policy. A record may remain structurally valid for diagnostics while
     * still being refused for live use. */
    if(tolerance_50a_A < CURRENT_50A_DEADBAND_A)
    {
        tolerance_50a_A = CURRENT_50A_DEADBAND_A;
    }
    if(tolerance_800a_A < CURRENT_800A_DEADBAND_A)
    {
        tolerance_800a_A = CURRENT_800A_DEADBAND_A;
    }
    if(tolerance_50a_A >
       ((float)CURRENT_CAL_CONFIDENT_50A_UNCERTAINTY_MA / 1000.0f))
    {
        tolerance_50a_A =
            (float)CURRENT_CAL_CONFIDENT_50A_UNCERTAINTY_MA / 1000.0f;
    }
    if(tolerance_800a_A >
       ((float)CURRENT_CAL_CONFIDENT_800A_UNCERTAINTY_MA / 1000.0f))
    {
        tolerance_800a_A =
            (float)CURRENT_CAL_CONFIDENT_800A_UNCERTAINTY_MA / 1000.0f;
    }

    return fabsf(live_50a_A - stored_50a_A) <= tolerance_50a_A &&
           fabsf(live_800a_A - stored_800a_A) <= tolerance_800a_A;
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
        case CURRENT_SENSOR_REASON_CALIBRATION_CHANGED:
                                                        return "calibration_changed";
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
    bool changed = false;

    if(dev == NULL)
    {
        return;
    }

    if(finite_in_range(adc_vref_v, CURRENT_ADC_VREF_MIN_V, CURRENT_ADC_VREF_MAX_V))
    {
        changed = changed || (dev->adc_vref_v != adc_vref_v);
        dev->adc_vref_v = adc_vref_v;
    }

    if(finite_in_range(sensor_supply_v, CURRENT_SENSOR_SUPPLY_MIN_V, CURRENT_SENSOR_SUPPLY_MAX_V))
    {
        changed = changed || (dev->sensor_supply_v != sensor_supply_v);
        dev->sensor_supply_v = sensor_supply_v;
    }

    /* Offsets are expressed in amperes using these references. Changing a
     * reference after capture invalidates the calibration rather than silently
     * applying an offset derived under a different scale. */
    if(changed && dev->zero_calibrated)
    {
        dev->zero_offset_50a = 0.0f;
        dev->zero_offset_800a = 0.0f;
        dev->zero_calibrated = false;
        dev->filter_initialized = false;
        current_sensor_clear_calibration_provenance(dev);
    }
    if(changed)
    {
        current_sensor_mark_calibration_changed(dev);
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
    current_sensor_clear_calibration_provenance(dev);
    dev->calibration_restore_count = 0u;
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
       (!dev->current_valid) ||
       (dev->reason != CURRENT_SENSOR_REASON_OK) ||
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
    if(dev->zero_cal_count != UINT32_MAX)
    {
        dev->zero_cal_count++;
    }
    current_sensor_clear_calibration_provenance(dev);
    current_sensor_mark_calibration_changed(dev);
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
    current_sensor_clear_calibration_provenance(dev);
    current_sensor_mark_calibration_changed(dev);
}

bool current_sensor_calibration_record_create(
    const current_sensor_t *dev,
    const current_sensor_calibration_metadata_t *metadata,
    current_sensor_calibration_record_t *record)
{
    if((dev == NULL) || (metadata == NULL) || (record == NULL) ||
       !current_sensor_offsets_usable(dev) ||
       (metadata->calibration_id == 0u) ||
       (metadata->calibration_temp_deci_c < CURRENT_CAL_TEMP_MIN_DECI_C) ||
       (metadata->calibration_temp_deci_c > CURRENT_CAL_TEMP_MAX_DECI_C) ||
       (metadata->uncertainty_50a_mA == 0u) ||
       (metadata->uncertainty_50a_mA ==
        CURRENT_SENSOR_CALIBRATION_UNCERTAINTY_UNKNOWN) ||
       (metadata->uncertainty_800a_mA == 0u) ||
       (metadata->uncertainty_800a_mA ==
        CURRENT_SENSOR_CALIBRATION_UNCERTAINTY_UNKNOWN) ||
       !finite_in_range(dev->adc_vref_v,
                        CURRENT_ADC_VREF_MIN_V,
                        CURRENT_ADC_VREF_MAX_V) ||
       !finite_in_range(dev->sensor_supply_v,
                        CURRENT_SENSOR_SUPPLY_MIN_V,
                        CURRENT_SENSOR_SUPPLY_MAX_V))
    {
        return false;
    }

    current_sensor_calibration_record_t next = {0};
    next.magic = CURRENT_SENSOR_CALIBRATION_MAGIC;
    next.schema = CURRENT_SENSOR_CALIBRATION_SCHEMA;
    next.size = CURRENT_SENSOR_CALIBRATION_RECORD_SIZE;
    next.calibration_id = metadata->calibration_id;
    next.capture_time_s = metadata->capture_time_s;
    next.zero_offset_50a_mA =
        (int32_t)lroundf(dev->zero_offset_50a * 1000.0f);
    next.zero_offset_800a_mA =
        (int32_t)lroundf(dev->zero_offset_800a * 1000.0f);
    next.adc_vref_uV = (uint32_t)lroundf(dev->adc_vref_v * 1000000.0f);
    next.sensor_supply_uV =
        (uint32_t)lroundf(dev->sensor_supply_v * 1000000.0f);
    next.calibration_temp_deci_c = metadata->calibration_temp_deci_c;
    next.uncertainty_50a_mA = metadata->uncertainty_50a_mA;
    next.uncertainty_800a_mA = metadata->uncertainty_800a_mA;
    next.reserved = 0u;
    next.crc32 = current_cal_record_crc(&next);
    *record = next;
    return true;
}

bool current_sensor_calibration_record_valid(
    const current_sensor_calibration_record_t *record)
{
    if((record == NULL) ||
       (record->magic != CURRENT_SENSOR_CALIBRATION_MAGIC) ||
       (record->schema != CURRENT_SENSOR_CALIBRATION_SCHEMA) ||
       (record->size != CURRENT_SENSOR_CALIBRATION_RECORD_SIZE) ||
       (record->calibration_id == 0u) ||
       (record->reserved != 0u) ||
       (record->calibration_temp_deci_c < CURRENT_CAL_TEMP_MIN_DECI_C) ||
       (record->calibration_temp_deci_c > CURRENT_CAL_TEMP_MAX_DECI_C) ||
       (record->uncertainty_50a_mA == 0u) ||
       (record->uncertainty_50a_mA ==
        CURRENT_SENSOR_CALIBRATION_UNCERTAINTY_UNKNOWN) ||
       (record->uncertainty_800a_mA == 0u) ||
       (record->uncertainty_800a_mA ==
        CURRENT_SENSOR_CALIBRATION_UNCERTAINTY_UNKNOWN) ||
       (record->zero_offset_50a_mA <
        -(int32_t)(CURRENT_ZERO_OFFSET_MAX_50A_A * 1000.0f)) ||
       (record->zero_offset_50a_mA >
        (int32_t)(CURRENT_ZERO_OFFSET_MAX_50A_A * 1000.0f)) ||
       (record->zero_offset_800a_mA <
        -(int32_t)(CURRENT_ZERO_OFFSET_MAX_800A_A * 1000.0f)) ||
       (record->zero_offset_800a_mA >
        (int32_t)(CURRENT_ZERO_OFFSET_MAX_800A_A * 1000.0f)) ||
       (record->adc_vref_uV <
        (uint32_t)(CURRENT_ADC_VREF_MIN_V * 1000000.0f)) ||
       (record->adc_vref_uV >
        (uint32_t)(CURRENT_ADC_VREF_MAX_V * 1000000.0f)) ||
       (record->sensor_supply_uV <
        (uint32_t)(CURRENT_SENSOR_SUPPLY_MIN_V * 1000000.0f)) ||
       (record->sensor_supply_uV >
        (uint32_t)(CURRENT_SENSOR_SUPPLY_MAX_V * 1000000.0f)))
    {
        return false;
    }

    return record->crc32 == current_cal_record_crc(record);
}

bool current_sensor_calibration_apply(
    current_sensor_t *dev,
    const current_sensor_calibration_record_t *record,
    bool zero_current_proven)
{
    if((dev == NULL) || !zero_current_proven ||
       !current_sensor_calibration_record_valid(record) ||
       !current_sensor_record_matches_live_zero(dev, record))
    {
        if(dev != NULL)
        {
            dev->reason = CURRENT_SENSOR_REASON_ZERO_CAL_REJECTED;
        }
        return false;
    }

    dev->adc_vref_v = (float)record->adc_vref_uV / 1000000.0f;
    dev->sensor_supply_v = (float)record->sensor_supply_uV / 1000000.0f;
    dev->zero_offset_50a =
        (float)record->zero_offset_50a_mA / 1000.0f;
    dev->zero_offset_800a =
        (float)record->zero_offset_800a_mA / 1000.0f;
    dev->zero_calibrated = true;
    dev->calibration_loaded_from_record = true;
    dev->calibration_id = record->calibration_id;
    dev->calibration_capture_time_s = record->capture_time_s;
    dev->calibration_temp_deci_c = record->calibration_temp_deci_c;
    dev->calibration_uncertainty_50a_mA = record->uncertainty_50a_mA;
    dev->calibration_uncertainty_800a_mA = record->uncertainty_800a_mA;
    if(dev->calibration_restore_count != UINT32_MAX)
    {
        dev->calibration_restore_count++;
    }
    current_sensor_mark_calibration_changed(dev);
    return true;
}

bool current_sensor_calibration_confident(const current_sensor_t *dev)
{
    return current_sensor_offsets_usable(dev) &&
           dev->calibration_loaded_from_record &&
           (dev->calibration_id != 0u) &&
           (dev->calibration_capture_time_s !=
            CURRENT_SENSOR_CALIBRATION_TIME_UNKNOWN) &&
           (dev->calibration_temp_deci_c >= CURRENT_CAL_TEMP_MIN_DECI_C) &&
           (dev->calibration_temp_deci_c <= CURRENT_CAL_TEMP_MAX_DECI_C) &&
           finite_in_range(dev->adc_vref_v,
                           CURRENT_ADC_VREF_MIN_V,
                           CURRENT_ADC_VREF_MAX_V) &&
           finite_in_range(dev->sensor_supply_v,
                           CURRENT_SENSOR_SUPPLY_MIN_V,
                           CURRENT_SENSOR_SUPPLY_MAX_V) &&
           (dev->calibration_uncertainty_50a_mA > 0u) &&
           (dev->calibration_uncertainty_800a_mA > 0u) &&
           (dev->calibration_uncertainty_50a_mA <=
            CURRENT_CAL_CONFIDENT_50A_UNCERTAINTY_MA) &&
           (dev->calibration_uncertainty_800a_mA <=
            CURRENT_CAL_CONFIDENT_800A_UNCERTAINTY_MA);
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
