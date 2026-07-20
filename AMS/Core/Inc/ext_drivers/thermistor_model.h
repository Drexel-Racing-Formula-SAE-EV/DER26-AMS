/*
 * thermistor_model.h
 *
 * Shared DER26 SMB thermistor model for Vishay NTCLE350E4103FHB0.
 *
 * Production conversion uses the manufacturer R/T LUT generated from the
 * Vishay NTC R/T Calculator CSV.  The full extended Steinhart-Hart equations
 * remain available as an independent reference and as the HIL inverse model.
 */
#ifndef INC_EXT_DRIVERS_THERMISTOR_MODEL_H_
#define INC_EXT_DRIVERS_THERMISTOR_MODEL_H_

#include <stdbool.h>
#include <stdint.h>

#include "ext_drivers/thermistor_model_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

#define THERMISTOR_MODEL_REVISION                 1u
/* SMB BOM: Panasonic EXB38V103JV, four isolated 10 kOhm elements,
 * +/-5% resistance tolerance and +/-200 ppm/K TCR. Runtime conversion uses
 * the nominal value; physical/tolerance validation remains separate. */
#define THERMISTOR_PULLDOWN_OHM                   10000.0f
#define THERMISTOR_PULLDOWN_TOLERANCE_PERCENT     5.0f
#define THERMISTOR_PULLDOWN_TCR_PPM_PER_K         200.0f
#define THERMISTOR_NOMINAL_VREG_V                 5.0f
#define THERMISTOR_MIN_VREG_V                     4.5f
#define THERMISTOR_MAX_VREG_V                     5.5f

#define THERMISTOR_ADBMS_LSB_V                    0.000150f
#define THERMISTOR_ADBMS_CODE_OFFSET              10000.0f
#define THERMISTOR_ADBMS_RESET_CODE               ((int16_t)-1)       /* 0xFFFF */
#define THERMISTOR_ADBMS_CLEAR_CODE               INT16_MIN           /* 0x8000 */


/* These ratios preserve the existing DER26 100 mV / 4.9 V thresholds when
 * VREG is 5.0 V, while remaining coherent if a measured VREG is supplied. */
#define THERMISTOR_OPEN_RATIO_MAX                 0.02f
#define THERMISTOR_SHORT_RATIO_MIN                0.98f

typedef enum
{
    THERMISTOR_STATUS_OK = 0,
    THERMISTOR_STATUS_CLAMPED_COLD,
    THERMISTOR_STATUS_CLAMPED_HOT,
    THERMISTOR_STATUS_OPEN_CIRCUIT,
    THERMISTOR_STATUS_SHORT_CIRCUIT,
    THERMISTOR_STATUS_ADC_SENTINEL,
    THERMISTOR_STATUS_REFERENCE_OUT_OF_RANGE,
    THERMISTOR_STATUS_VOLTAGE_OUT_OF_RANGE,
    THERMISTOR_STATUS_RESISTANCE_INVALID,
    THERMISTOR_STATUS_NUMERIC_FAULT
} thermistor_status_t;

typedef struct
{
    float temperature_c;
    float resistance_ohm;
    float divider_voltage_v;
    thermistor_status_t status;
    bool valid;
    bool model_clamped;
} thermistor_result_t;

/* Manufacturer-table implementation used by the production conversion.
 * Resistance above/below the exported table returns the nearest endpoint. */
float thermistor_temperature_lut_c(float resistance_ohm);

/* Independent full Vishay extended Steinhart-Hart inverse equation.
 * This mathematical reference is not automatically a board-level accuracy
 * claim and must remain tied to THERMISTOR_MODEL_SOURCE_SHA256. */
float thermistor_temperature_steinhart_hart_c(float resistance_ohm);

/* Full Vishay forward equation used for HIL and reference generation. */
float thermistor_resistance_from_temperature_c(float temperature_c);

/* Convert the physical SMB divider node to a nominal thermistor result.
 * Electrically valid values beyond the exported LUT range are conservatively
 * clamped to -20 C / 120 C and flagged with CLAMPED_COLD / CLAMPED_HOT. */
thermistor_result_t thermistor_from_divider_voltage(float divider_voltage_v,
                                                    float vreg_v);

/* Convert the signed ADBMS AUX result representation used by DER26:
 *   V = (raw + 10000) * 150 uV
 *
 * Raw code 0 is intentionally accepted. It represents approximately 6.7 C
 * for the current divider and is not a reset/clear sentinel. */
thermistor_result_t thermistor_from_adbms_raw(int16_t raw_code,
                                              float vreg_v);

/* Generate a nominal ADBMS raw code from temperature using the full Vishay
 * forward equation. Intended for HIL and deterministic host tests. */
bool thermistor_adbms_raw_from_temperature_c(float temperature_c,
                                             float vreg_v,
                                             int16_t *raw_code_out);

bool thermistor_status_is_model_valid(thermistor_status_t status);
const char *thermistor_status_str(thermistor_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* INC_EXT_DRIVERS_THERMISTOR_MODEL_H_ */
