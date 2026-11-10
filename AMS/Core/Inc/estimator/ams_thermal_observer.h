#ifndef AMS_THERMAL_OBSERVER_H
#define AMS_THERMAL_OBSERVER_H
#include <stdbool.h>
#include <stdint.h>
/* Shadow only: representative cell core/surface, never protection authority. */
#define AMS_THERMAL_SEGMENTS 5u
#define AMS_THERMAL_SENSORS 24u
#define AMS_THERMAL_UNCALIBRATED (1u << 0)
#define AMS_THERMAL_BOUNDARY_ASSUMED (1u << 1)
#define AMS_THERMAL_CURRENT_INVALID (1u << 2)
#define AMS_THERMAL_SURFACE_INVALID (1u << 3)
#define AMS_THERMAL_BOUNDARY_INVALID (1u << 4)
#define AMS_THERMAL_TIMING_INVALID (1u << 5)
#define AMS_THERMAL_NUMERIC (1u << 6)
#define AMS_THERMAL_CURRENT_UNCALIBRATED (1u << 7)
#define AMS_THERMAL_MODEL_INVALID (1u << 8)
#define AMS_THERMAL_INNOVATION_REJECT (1u << 9)
typedef struct {
    float cc, cs, rcs, rsa; /* per-cell J/K and K/W, same convention as SoP */
    float process_variance_rate, measurement_variance;
    float model_margin_c, innovation_gate_sigma;
    uint32_t max_step_ms, max_surface_age_ms, sensor_mask;
    uint8_t minimum_sensors;
    bool calibrated;
} ams_thermal_config_t;
typedef struct {
    uint32_t now_ms, source_ms;
    float heat_w, boundary_c;
    bool heat_valid, current_valid, current_calibrated, boundary_valid, boundary_assumed;
    uint32_t surface_mask, surface_tick[AMS_THERMAL_SENSORS];
    float surface_c[AMS_THERMAL_SENSORS];
} ams_thermal_input_t;
typedef struct {
    float core_c, surface_c, p00, p01, p11;
    float core_upper_c, innovation_c, heat_w;
    uint32_t tick_ms, surface_tick[AMS_THERMAL_SENSORS], seen_mask;
    uint32_t flags, predictions, corrections, rejected, duplicate_count;
    uint8_t usable_sensors;
    bool initialized, available;
} ams_thermal_observer_t;
void ams_thermal_default_config(ams_thermal_config_t *c);
void ams_thermal_init(ams_thermal_observer_t *o);
void ams_thermal_step(ams_thermal_observer_t *o,
                      const ams_thermal_config_t *c,
                      const ams_thermal_input_t *in);
#endif
