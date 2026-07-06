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

#ifndef AMS_RENODE
#define AMS_RENODE 0
#endif

#ifndef AMS_RENODE_FAKE_CURRENT
#define AMS_RENODE_FAKE_CURRENT AMS_RENODE
#endif

#define CURRENT_ADC_VREF_V              3.3f
#define CURRENT_ADC_MAX_COUNT           4095.0f
#define CURRENT_ADC_TIMEOUT_MS          5u

#define DHAB_OFFSET_V                   2.5f
#define DHAB_CH_50A_SENS_V_PER_A        0.040f
#define DHAB_CH_800A_SENS_V_PER_A       0.0025f

#define SENSOR_DIVIDER_TOP_OHM          100000.0f
#define SENSOR_DIVIDER_BOTTOM_OHM       150000.0f
#define SENSOR_DIVIDER_GAIN             (SENSOR_DIVIDER_BOTTOM_OHM / (SENSOR_DIVIDER_TOP_OHM + SENSOR_DIVIDER_BOTTOM_OHM))

#define CURRENT_ADC_IMPLAUS_LOW_COUNT   100u
#define CURRENT_ADC_IMPLAUS_HIGH_COUNT  3800u

#define DHAB_SENSOR_VALID_MIN_V         0.20f
#define DHAB_SENSOR_VALID_MAX_V         4.80f
#define DHAB_SENSOR_CLAMP_LOW_V         0.30f
#define DHAB_SENSOR_CLAMP_HIGH_V        4.70f

#define CURRENT_50A_USE_LIMIT_A         45.0f
#define CURRENT_CHANNEL_COMPARE_MIN_A   10.0f
#define CURRENT_CHANNEL_AGREE_ABS_A     7.5f
#define CURRENT_CHANNEL_AGREE_PCT       0.15f

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

#if AMS_RENODE_FAKE_CURRENT
typedef struct
{
    bool initialized;
    bool enabled;
    bool adc_fail;
    bool raw_override;
    float requested_current_a;
    float current_50a_a;
    float current_800a_a;
    uint16_t count_low;
    uint16_t count_high;
    uint32_t read_count;
} current_sensor_fake_model_t;

static current_sensor_fake_model_t current_sensor_fake;
#endif

static float current_sensor_adc_count_to_voltage(uint16_t count)
{
    return ((float)count * CURRENT_ADC_VREF_V) / CURRENT_ADC_MAX_COUNT;
}

static uint16_t current_sensor_adc_voltage_to_count(float adc_voltage)
{
    float count;

    if(!isfinite(adc_voltage) || (adc_voltage <= 0.0f))
    {
        return 0u;
    }

    count = (adc_voltage * CURRENT_ADC_MAX_COUNT) / CURRENT_ADC_VREF_V;
    if(count >= CURRENT_ADC_MAX_COUNT)
    {
        return (uint16_t)CURRENT_ADC_MAX_COUNT;
    }

    return (uint16_t)(count + 0.5f);
}

static float current_sensor_adc_voltage_to_sensor_voltage(float adc_voltage)
{
    return adc_voltage / SENSOR_DIVIDER_GAIN;
}

static float current_sensor_voltage_to_current(float sensor_voltage, float sensitivity_v_per_a)
{
    return (sensor_voltage - DHAB_OFFSET_V) / sensitivity_v_per_a;
}

static float current_sensor_current_to_sensor_voltage(float current_a, float sensitivity_v_per_a)
{
    return DHAB_OFFSET_V + (current_a * sensitivity_v_per_a);
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

#if AMS_RENODE_FAKE_CURRENT
static void current_sensor_fake_ensure(void)
{
    if(current_sensor_fake.initialized)
    {
        return;
    }

    current_sensor_fake.initialized = true;
    current_sensor_fake.enabled = true;
    current_sensor_fake.adc_fail = false;
    current_sensor_fake.raw_override = false;
    current_sensor_fake.requested_current_a = 0.0f;
    current_sensor_fake.current_50a_a = 0.0f;
    current_sensor_fake.current_800a_a = 0.0f;
    current_sensor_fake.count_low = current_sensor_adc_voltage_to_count(DHAB_OFFSET_V * SENSOR_DIVIDER_GAIN);
    current_sensor_fake.count_high = current_sensor_fake.count_low;
    current_sensor_fake.read_count = 0u;
}

static uint16_t current_sensor_fake_count_for_current(float current_a, float sensitivity_v_per_a)
{
    float sensor_voltage = current_sensor_current_to_sensor_voltage(current_a, sensitivity_v_per_a);
    float adc_voltage;

    if(sensor_voltage <= DHAB_SENSOR_CLAMP_LOW_V)
    {
        sensor_voltage = DHAB_SENSOR_CLAMP_LOW_V;
    }
    else if(sensor_voltage >= DHAB_SENSOR_CLAMP_HIGH_V)
    {
        sensor_voltage = DHAB_SENSOR_CLAMP_HIGH_V;
    }

    adc_voltage = sensor_voltage * SENSOR_DIVIDER_GAIN;
    return current_sensor_adc_voltage_to_count(adc_voltage);
}

static void current_sensor_fake_update_counts_from_currents(void)
{
    uint16_t count_50a = current_sensor_fake_count_for_current(current_sensor_fake.current_50a_a,
                                                               DHAB_CH_50A_SENS_V_PER_A);
    uint16_t count_800a = current_sensor_fake_count_for_current(current_sensor_fake.current_800a_a,
                                                                DHAB_CH_800A_SENS_V_PER_A);

#if CURRENT_SENSOR_50A_CHANNEL_IS_HIGH
    current_sensor_fake.count_high = count_50a;
    current_sensor_fake.count_low = count_800a;
#else
    current_sensor_fake.count_low = count_50a;
    current_sensor_fake.count_high = count_800a;
#endif
}

bool current_sensor_fake_enabled(void)
{
    current_sensor_fake_ensure();
    return current_sensor_fake.enabled;
}

void current_sensor_fake_reset(void)
{
    current_sensor_fake.initialized = false;
    current_sensor_fake_ensure();
}

int current_sensor_fake_set_current_a(float current_a)
{
    current_sensor_fake_ensure();
    if(!isfinite(current_a) || (current_a < -900.0f) || (current_a > 900.0f))
    {
        return -1;
    }

    current_sensor_fake.enabled = true;
    current_sensor_fake.adc_fail = false;
    current_sensor_fake.raw_override = false;
    current_sensor_fake.requested_current_a = current_a;
    current_sensor_fake.current_50a_a = current_a;
    current_sensor_fake.current_800a_a = current_a;
    current_sensor_fake_update_counts_from_currents();
    return 0;
}

int current_sensor_fake_set_channel_currents_a(float current_50a, float current_800a)
{
    current_sensor_fake_ensure();
    if(!isfinite(current_50a) || !isfinite(current_800a) ||
       (current_50a < -900.0f) || (current_50a > 900.0f) ||
       (current_800a < -900.0f) || (current_800a > 900.0f))
    {
        return -1;
    }

    current_sensor_fake.enabled = true;
    current_sensor_fake.adc_fail = false;
    current_sensor_fake.raw_override = false;
    current_sensor_fake.requested_current_a = current_50a;
    current_sensor_fake.current_50a_a = current_50a;
    current_sensor_fake.current_800a_a = current_800a;
    current_sensor_fake_update_counts_from_currents();
    return 0;
}

int current_sensor_fake_set_raw_counts(uint16_t count_low, uint16_t count_high)
{
    current_sensor_fake_ensure();
    if((count_low > (uint16_t)CURRENT_ADC_MAX_COUNT) ||
       (count_high > (uint16_t)CURRENT_ADC_MAX_COUNT))
    {
        return -1;
    }

    current_sensor_fake.enabled = true;
    current_sensor_fake.adc_fail = false;
    current_sensor_fake.raw_override = true;
    current_sensor_fake.count_low = count_low;
    current_sensor_fake.count_high = count_high;
    return 0;
}

void current_sensor_fake_clear_raw_override(void)
{
    current_sensor_fake_ensure();
    current_sensor_fake.raw_override = false;
    current_sensor_fake_update_counts_from_currents();
}

void current_sensor_fake_set_adc_fail(bool fail)
{
    current_sensor_fake_ensure();
    current_sensor_fake.enabled = true;
    current_sensor_fake.adc_fail = fail;
}

current_sensor_fake_status_t current_sensor_fake_get_status(void)
{
    current_sensor_fake_status_t status;

    current_sensor_fake_ensure();
    status.enabled = current_sensor_fake.enabled;
    status.adc_fail = current_sensor_fake.adc_fail;
    status.raw_override = current_sensor_fake.raw_override;
    status.requested_current_a = current_sensor_fake.requested_current_a;
    status.channel_50a_current_a = current_sensor_fake.current_50a_a;
    status.channel_800a_current_a = current_sensor_fake.current_800a_a;
    status.count_low = current_sensor_fake.count_low;
    status.count_high = current_sensor_fake.count_high;
    status.read_count = current_sensor_fake.read_count;
    return status;
}
#else
bool current_sensor_fake_enabled(void)
{
    return false;
}

void current_sensor_fake_reset(void)
{
}

int current_sensor_fake_set_current_a(float current_a)
{
    (void)current_a;
    return -1;
}

int current_sensor_fake_set_channel_currents_a(float current_50a, float current_800a)
{
    (void)current_50a;
    (void)current_800a;
    return -1;
}

int current_sensor_fake_set_raw_counts(uint16_t count_low, uint16_t count_high)
{
    (void)count_low;
    (void)count_high;
    return -1;
}

void current_sensor_fake_clear_raw_override(void)
{
}

void current_sensor_fake_set_adc_fail(bool fail)
{
    (void)fail;
}

current_sensor_fake_status_t current_sensor_fake_get_status(void)
{
    return (current_sensor_fake_status_t){0};
}
#endif

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

    if(dev == NULL)
    {
        return 0.0f;
    }

    if((!dev->last_read_ok) ||
       (!dev->count_high_fresh) ||
       (!dev->count_low_fresh))
    {
        current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_READ);
        return dev->current;
    }

    dev->voltage_low  = current_sensor_adc_count_to_voltage(dev->count_low);
    dev->voltage_high = current_sensor_adc_count_to_voltage(dev->count_high);

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

    dev->current_50a = current_sensor_voltage_to_current(sensor_voltage_50a,
                                                          DHAB_CH_50A_SENS_V_PER_A);
    dev->current_800a = current_sensor_voltage_to_current(sensor_voltage_800a,
                                                           DHAB_CH_800A_SENS_V_PER_A);

#if CURRENT_SENSOR_50A_CHANNEL_IS_HIGH
    dev->current_high = dev->current_50a;
    dev->current_low = dev->current_800a;
#else
    dev->current_high = dev->current_800a;
    dev->current_low = dev->current_50a;
#endif

    if((fabsf(dev->current_50a) <= CURRENT_50A_USE_LIMIT_A) &&
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
    return dev->current;
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

#if AMS_RENODE_FAKE_CURRENT
    current_sensor_fake_ensure();
    if(current_sensor_fake.enabled)
    {
        current_sensor_fake.read_count++;
        if(current_sensor_fake.adc_fail)
        {
            current_sensor_set_invalid(dev, CURRENT_SENSOR_REASON_ADC_READ);
            return false;
        }

        if(!current_sensor_fake.raw_override)
        {
            current_sensor_fake_update_counts_from_currents();
        }

        dev->count_high = current_sensor_fake.count_high;
        dev->count_low = current_sensor_fake.count_low;
        dev->count_high_fresh = true;
        dev->count_low_fresh = true;
        dev->last_read_ok = true;
        return true;
    }
#endif

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
