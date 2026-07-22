/*
 * thermistor_model.c
 *
 * Shared DER26 thermistor conversion for Vishay NTCLE350E4103FHB0.
 */
#include "ext_drivers/thermistor_model.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#include "thermistor_lut_generated.h"

/* The exact part identity, source hash, R25, B25/85, model range, and
 * full Vishay equations come from generated artifacts. The generator validates
 * their cross-file consistency before emitting the headers. */
_Static_assert(THERMISTOR_LUT_COUNT == 281u,
               "Unexpected thermistor LUT point count");

static thermistor_result_t thermistor_invalid_result(thermistor_status_t status)
{
    thermistor_result_t result;
    result.temperature_c = NAN;
    result.resistance_ohm = NAN;
    result.divider_voltage_v = NAN;
    result.status = status;
    result.valid = false;
    result.model_clamped = false;
    return result;
}

static bool thermistor_vreg_valid(float vreg_v)
{
    return isfinite(vreg_v) &&
           (vreg_v >= THERMISTOR_MIN_VREG_V) &&
           (vreg_v <= THERMISTOR_MAX_VREG_V);
}

float thermistor_temperature_lut_c(float resistance_ohm)
{
    uint16_t low;
    uint16_t high;

    if(!isfinite(resistance_ohm) || (resistance_ohm <= 0.0f))
    {
        return NAN;
    }

    if(resistance_ohm >= THERMISTOR_LUT_COLD_R_OHM)
    {
        return THERMISTOR_LUT_MIN_TEMP_C;
    }
    if(resistance_ohm <= THERMISTOR_LUT_HOT_R_OHM)
    {
        return THERMISTOR_LUT_MAX_TEMP_C;
    }

    /* The table is strictly descending with increasing temperature. Find
     * low such that R[low] >= resistance >= R[low + 1]. */
    low = 0u;
    high = (uint16_t)(THERMISTOR_LUT_COUNT - 1u);
    while((uint16_t)(high - low) > 1u)
    {
        uint16_t mid = (uint16_t)(low + ((high - low) / 2u));
        if(resistance_ohm <= thermistor_lut_resistance_ohm[mid])
        {
            low = mid;
        }
        else
        {
            high = mid;
        }
    }

    float r_cold = thermistor_lut_resistance_ohm[low];
    float r_hot = thermistor_lut_resistance_ohm[low + 1u];
    float delta_r = r_cold - r_hot;
    if(!isfinite(delta_r) || (delta_r <= 0.0f))
    {
        return NAN;
    }

    float fraction = (r_cold - resistance_ohm) / delta_r;
    float temperature_c = THERMISTOR_LUT_MIN_TEMP_C +
                          (((float)low + fraction) * THERMISTOR_LUT_STEP_C);
    return isfinite(temperature_c) ? temperature_c : NAN;
}

float thermistor_temperature_steinhart_hart_c(float resistance_ohm)
{
    if(!isfinite(resistance_ohm) || (resistance_ohm <= 0.0f))
    {
        return NAN;
    }

    float x = logf(resistance_ohm / THERMISTOR_R25_OHM);
    float x2 = x * x;
    float denominator = THERMISTOR_SH_A1 +
                        (THERMISTOR_SH_B1 * x) +
                        (THERMISTOR_SH_C1 * x2) +
                        (THERMISTOR_SH_D1 * x2 * x);

    if(!isfinite(denominator) || (denominator <= 0.0f))
    {
        return NAN;
    }

    float temperature_c = (1.0f / denominator) - 273.15f;
    return isfinite(temperature_c) ? temperature_c : NAN;
}

float thermistor_resistance_from_temperature_c(float temperature_c)
{
    float temperature_k;
    float temperature_k2;
    float exponent;
    float resistance_ohm;

    if(!isfinite(temperature_c) || (temperature_c <= -273.15f))
    {
        return NAN;
    }

    temperature_k = temperature_c + 273.15f;
    temperature_k2 = temperature_k * temperature_k;
    exponent = THERMISTOR_FWD_A +
               (THERMISTOR_FWD_B / temperature_k) +
               (THERMISTOR_FWD_C / temperature_k2) +
               (THERMISTOR_FWD_D / (temperature_k2 * temperature_k));
    resistance_ohm = THERMISTOR_R25_OHM * expf(exponent);

    return (isfinite(resistance_ohm) && (resistance_ohm > 0.0f)) ?
           resistance_ohm : NAN;
}

thermistor_result_t thermistor_from_divider_voltage(float divider_voltage_v,
                                                    float vreg_v)
{
    thermistor_result_t result = thermistor_invalid_result(THERMISTOR_STATUS_NUMERIC_FAULT);

    if(!isfinite(divider_voltage_v) || !isfinite(vreg_v))
    {
        return result;
    }
    if(!thermistor_vreg_valid(vreg_v))
    {
        return thermistor_invalid_result(THERMISTOR_STATUS_REFERENCE_OUT_OF_RANGE);
    }

    result.divider_voltage_v = divider_voltage_v;

    if((divider_voltage_v < 0.0f) || (divider_voltage_v > vreg_v))
    {
        result.status = THERMISTOR_STATUS_VOLTAGE_OUT_OF_RANGE;
        return result;
    }

    float ratio = divider_voltage_v / vreg_v;
    if(ratio <= THERMISTOR_OPEN_RATIO_MAX)
    {
        result.status = THERMISTOR_STATUS_OPEN_CIRCUIT;
        return result;
    }
    if(ratio >= THERMISTOR_SHORT_RATIO_MIN)
    {
        result.status = THERMISTOR_STATUS_SHORT_CIRCUIT;
        return result;
    }
    if(divider_voltage_v <= 0.0f)
    {
        result.status = THERMISTOR_STATUS_OPEN_CIRCUIT;
        return result;
    }

    result.resistance_ohm = THERMISTOR_PULLDOWN_OHM *
                            (vreg_v - divider_voltage_v) /
                            divider_voltage_v;
    if(!isfinite(result.resistance_ohm) || (result.resistance_ohm <= 0.0f))
    {
        result.status = THERMISTOR_STATUS_RESISTANCE_INVALID;
        return result;
    }

    if(result.resistance_ohm > THERMISTOR_LUT_COLD_R_OHM)
    {
        result.temperature_c = THERMISTOR_MODEL_MIN_TEMP_C;
        result.status = THERMISTOR_STATUS_CLAMPED_COLD;
        result.valid = true;
        result.model_clamped = true;
        return result;
    }
    if(result.resistance_ohm < THERMISTOR_LUT_HOT_R_OHM)
    {
        result.temperature_c = THERMISTOR_MODEL_MAX_TEMP_C;
        result.status = THERMISTOR_STATUS_CLAMPED_HOT;
        result.valid = true;
        result.model_clamped = true;
        return result;
    }

    result.temperature_c = thermistor_temperature_lut_c(result.resistance_ohm);
    if(!isfinite(result.temperature_c))
    {
        result.status = THERMISTOR_STATUS_NUMERIC_FAULT;
        return result;
    }

    result.status = THERMISTOR_STATUS_OK;
    result.valid = true;
    result.model_clamped = false;
    return result;
}

thermistor_result_t thermistor_from_adbms_raw(int16_t raw_code,
                                              float vreg_v)
{
    thermistor_result_t result;

    if((raw_code == THERMISTOR_ADBMS_RESET_CODE) ||
       (raw_code == THERMISTOR_ADBMS_CLEAR_CODE))
    {
        return thermistor_invalid_result(THERMISTOR_STATUS_ADC_SENTINEL);
    }

    float voltage_v = ((float)raw_code + THERMISTOR_ADBMS_CODE_OFFSET) *
                      THERMISTOR_ADBMS_LSB_V;
    result = thermistor_from_divider_voltage(voltage_v, vreg_v);
    result.divider_voltage_v = voltage_v;
    return result;
}

bool thermistor_adbms_raw_from_temperature_c(float temperature_c,
                                             float vreg_v,
                                             int16_t *raw_code_out)
{
    if((raw_code_out == NULL) || !thermistor_vreg_valid(vreg_v) ||
       !isfinite(temperature_c) ||
       (temperature_c < THERMISTOR_MODEL_MIN_TEMP_C) ||
       (temperature_c > THERMISTOR_MODEL_MAX_TEMP_C))
    {
        return false;
    }

    float resistance_ohm = thermistor_resistance_from_temperature_c(temperature_c);
    if(!isfinite(resistance_ohm) || (resistance_ohm <= 0.0f))
    {
        return false;
    }

    float voltage_v = vreg_v * THERMISTOR_PULLDOWN_OHM /
                      (resistance_ohm + THERMISTOR_PULLDOWN_OHM);
    float raw_f = (voltage_v / THERMISTOR_ADBMS_LSB_V) -
                  THERMISTOR_ADBMS_CODE_OFFSET;
    if(!isfinite(raw_f) ||
       (raw_f < (float)INT16_MIN) ||
       (raw_f > (float)INT16_MAX))
    {
        return false;
    }

    long raw_l = lroundf(raw_f);
    if((raw_l < (long)INT16_MIN) || (raw_l > (long)INT16_MAX) ||
       ((int16_t)raw_l == THERMISTOR_ADBMS_RESET_CODE) ||
       ((int16_t)raw_l == THERMISTOR_ADBMS_CLEAR_CODE))
    {
        return false;
    }

    *raw_code_out = (int16_t)raw_l;
    return true;
}

bool thermistor_status_is_model_valid(thermistor_status_t status)
{
    return status == THERMISTOR_STATUS_OK;
}

const char *thermistor_status_str(thermistor_status_t status)
{
    switch(status)
    {
        case THERMISTOR_STATUS_OK:                       return "ok";
        case THERMISTOR_STATUS_CLAMPED_COLD:             return "clamped_cold";
        case THERMISTOR_STATUS_CLAMPED_HOT:              return "clamped_hot";
        case THERMISTOR_STATUS_OPEN_CIRCUIT:             return "open";
        case THERMISTOR_STATUS_SHORT_CIRCUIT:            return "short";
        case THERMISTOR_STATUS_ADC_SENTINEL:             return "adc_sentinel";
        case THERMISTOR_STATUS_REFERENCE_OUT_OF_RANGE:   return "vreg_out_of_range";
        case THERMISTOR_STATUS_VOLTAGE_OUT_OF_RANGE:     return "voltage_out_of_range";
        case THERMISTOR_STATUS_RESISTANCE_INVALID:       return "resistance_invalid";
        case THERMISTOR_STATUS_NUMERIC_FAULT:             return "numeric_fault";
        default:                                          return "unknown";
    }
}
