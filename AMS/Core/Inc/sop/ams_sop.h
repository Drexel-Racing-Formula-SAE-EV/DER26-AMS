/*
 * ams_sop.h
 *
 * Robust finite-horizon State-of-Power estimator for the DER26 75s6p P42A
 * accumulator.  The engine is deterministic, heap-free, and independent of
 * HAL/FreeRTOS.  It consumes coherent DADEKF and measurement states and solves
 * the largest constant-current pulse that satisfies every cell, SOC, thermal,
 * current-path, and model-domain constraint.
 */

#ifndef INC_SOP_AMS_SOP_H_
#define INC_SOP_AMS_SOP_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_SOP_SEGMENTS             5u
#define AMS_SOP_CELLS_PER_SEGMENT   15u
#define AMS_SOP_TOTAL_CELLS         75u
#define AMS_SOP_HORIZONS             4u
#define AMS_SOP_BISECTION_ITERS     16u
#define AMS_SOP_FULL_CELL_MASK       0x7FFFu
#define AMS_SOP_INVALID_INDEX        0xFFu

typedef enum
{
    AMS_SOP_OK = 0,
    AMS_SOP_BAD_ARGUMENT,
    AMS_SOP_INVALID_CONFIGURATION,
    AMS_SOP_INVALID_INPUT,
    AMS_SOP_NUMERIC_FAILURE
} ams_sop_status_t;

typedef enum
{
    AMS_SOP_MODE_IDLE = 0,
    AMS_SOP_MODE_DRIVE,
    AMS_SOP_MODE_CHARGE
} ams_sop_operating_mode_t;

typedef enum
{
    AMS_SOP_BIND_NONE = 0,
    AMS_SOP_BIND_CELL_UV,
    AMS_SOP_BIND_CELL_OV,
    AMS_SOP_BIND_SOC_LOW,
    AMS_SOP_BIND_SOC_HIGH,
    AMS_SOP_BIND_CORE_TEMP,
    AMS_SOP_BIND_SURFACE_TEMP,
    AMS_SOP_BIND_CHARGE_TEMP_LOW,
    AMS_SOP_BIND_CURRENT_PATH,
    AMS_SOP_BIND_DIRECTION_INHIBIT,
    AMS_SOP_BIND_MODEL_DOMAIN,
    AMS_SOP_BIND_INVALID_INPUT,
    AMS_SOP_BIND_HORIZON_ENVELOPE,
    AMS_SOP_BIND_FUSE_THERMAL,
    AMS_SOP_BIND_MISSION_PROFILE
} ams_sop_binding_t;

/* Input/result reason flags are retained even for valid degraded-prior solves. */
#define AMS_SOP_REASON_NONE                    0x00000000u
#define AMS_SOP_REASON_MEASUREMENT_INVALID     0x00000001u
#define AMS_SOP_REASON_ESTIMATOR_INVALID       0x00000002u
#define AMS_SOP_REASON_CURRENT_UNCALIBRATED    0x00000004u
#define AMS_SOP_REASON_CURRENT_POLARITY        0x00000008u
#define AMS_SOP_REASON_MEASUREMENT_STALE       0x00000010u
#define AMS_SOP_REASON_INCOMPLETE_TOPOLOGY     0x00000020u
#define AMS_SOP_REASON_MODEL_DOMAIN            0x00000040u
#define AMS_SOP_REASON_SOH_CAPACITY_PRIOR      0x00000080u
#define AMS_SOP_REASON_SOH_RESISTANCE_PRIOR    0x00000100u
#define AMS_SOP_REASON_CURRENT_UNCERTAINTY      0x00000200u
#define AMS_SOP_REASON_AMBIENT_PROXY            0x00000400u
#define AMS_SOP_REASON_BALANCE_RECOVERY         0x00000800u
#define AMS_SOP_REASON_DISCHARGE_INHIBITED      0x00001000u
#define AMS_SOP_REASON_REGEN_INHIBITED          0x00002000u
#define AMS_SOP_REASON_CHARGE_INHIBITED         0x00004000u
#define AMS_SOP_REASON_CONFIGURATION            0x00008000u
#define AMS_SOP_REASON_NUMERIC                  0x00010000u
#define AMS_SOP_REASON_ZERO_CURRENT_INFEASIBLE  0x00020000u
#define AMS_SOP_REASON_LIMIT_SLEWED              0x00040000u
#define AMS_SOP_REASON_FUSE_SHADOW               0x00080000u
#define AMS_SOP_REASON_FUSE_DERATED              0x00100000u
#define AMS_SOP_REASON_MISSION_DERATED           0x00200000u
#define AMS_SOP_REASON_MISSION_FALLBACK          0x00400000u
#define AMS_SOP_REASON_LIMP_HOME                 0x00800000u
#define AMS_SOP_REASON_RECOVERY_VOLTAGE          0x01000000u
#define AMS_SOP_REASON_RECOVERY_THERMAL          0x02000000u
#define AMS_SOP_REASON_RECOVERY_SOC_HOLD         0x04000000u
#define AMS_SOP_REASON_RECOVERY_CURRENT_PATH     0x08000000u
/* State-estimator acquisition is an explicit authority prerequisite. The
 * constrained dynamic estimator may continue tracking while this bit is set,
 * but SoP must fail zero until startup state ambiguity has been resolved by a
 * qualified acquisition. */
#define AMS_SOP_REASON_ESTIMATOR_UNACQUIRED       0x10000000u

typedef struct
{
    float soc;
    float vp1_v;
    float vp2_v;
    float r0_ohm;
    float core_temp_c;
    float surface_max_temp_c;

    /* DADEKF covariance and residual terms, all per representative cell except
     * innovation_v, which is the voltage residual for this segment. */
    float p_soc;
    float p_vp1;
    float p_vp2;
    float p_r0;
    float innovation_v;

    /* Conservative lower/upper confidence bounds supplied by the SoH layer. */
    float capacity_soh_lower;
    float resistance_soh_upper;

    float cell_voltage_v[AMS_SOP_CELLS_PER_SEGMENT];
    uint32_t max_cell_age_ms;
    uint16_t cell_usable_mask;
    uint8_t estimator_valid;
    uint8_t model_domain_flags;
    uint8_t capacity_soh_valid;
    uint8_t resistance_soh_valid;
} ams_sop_segment_input_t;

typedef struct
{
    uint32_t measurement_sequence;
    uint32_t measurement_timestamp_ms;
    uint32_t now_ms;
    float pack_current_a;                 /* positive = accumulator discharge */
    float pack_current_uncertainty_a;     /* absolute, >= 0 */
    float ambient_temp_c;

    ams_sop_segment_input_t segment[AMS_SOP_SEGMENTS];

    ams_sop_operating_mode_t operating_mode;
    uint8_t measurement_valid;
    uint8_t estimator_valid;
    uint8_t estimator_acquired;
    uint8_t estimator_segment_topology;
    uint8_t current_calibrated;
    uint8_t current_polarity_validated;
    uint8_t ambient_measured;
    uint8_t balance_recovered;
    uint8_t discharge_authorized;
    uint8_t charger_authorized;
    uint8_t regen_authorized;
} ams_sop_input_t;

typedef struct
{
    /* Operational limits are intentionally inside the absolute P42A/BMS
     * limits.  Uncertainty margins are added again by the predictor. */
    float cell_uv_operating_v;
    float cell_ov_operating_v;
    float soc_min;
    float soc_max;
    float discharge_core_temp_max_c;
    float discharge_surface_temp_max_c;
    float charge_core_temp_max_c;
    float charge_surface_temp_max_c;
    float charge_temp_min_c;

    /* Horizon-specific pack-current path limits.  These are DER26 system
     * limits, not the much larger 6p cell-array capability. */
    float discharge_current_max_a[AMS_SOP_HORIZONS];
    float charge_current_max_a[AMS_SOP_HORIZONS];
    float horizons_s[AMS_SOP_HORIZONS];

    float cell_capacity_ah;
    float parallel_cells;
    float r2_ohm;
    float c2_f;

    /* P42A two-node thermal network used by the validated HIL plant. */
    float core_thermal_capacity_j_per_k;
    float surface_thermal_capacity_j_per_k;
    float core_surface_resistance_k_per_w;
    float surface_ambient_resistance_k_per_w;

    float sigma_multiplier;
    float cell_voltage_measurement_uncertainty_v;
    float model_voltage_margin_v;
    float temperature_measurement_uncertainty_c;
    float model_temperature_margin_c;
    float current_uncertainty_floor_a;
    float max_innovation_per_cell_v;
    float max_measurement_age_ms;
    float default_capacity_soh_lower;
    float default_resistance_soh_upper;

    /* Prediction grid and published-limit increase rates.  Decreases are
     * immediate; increases are rate limited outside the pure solve. */
    float fine_step_s;
    float medium_step_s;
    float coarse_step_s;
    float discharge_rise_rate_a_per_s;
    float charge_rise_rate_a_per_s;
    float discharge_voltage_recovery_a_per_s;
    float charge_voltage_recovery_a_per_s;
    float discharge_thermal_recovery_a_per_s;
    float charge_thermal_recovery_a_per_s;
    float discharge_current_path_recovery_a_per_s;
    float charge_current_path_recovery_a_per_s;
    float soc_recovery_rate_a_per_s;
    float soc_recovery_delta;
    float soc_recovery_charge_as;
} ams_sop_config_t;

typedef struct
{
    uint8_t discharge_soc_recovered;
    uint8_t charge_soc_recovered;
    uint8_t fuse_state_valid;
    float fuse_utilization;
} ams_sop_recovery_context_t;

typedef struct
{
    float minimum_cell_voltage_v;
    float maximum_cell_voltage_v;
    float minimum_soc;
    float maximum_soc;
    float maximum_core_temp_c;
    float maximum_surface_temp_c;
    float pack_voltage_v;
} ams_sop_prediction_extrema_t;

typedef struct
{
    ams_sop_prediction_extrema_t extrema;
    ams_sop_binding_t binding;
    uint8_t limiting_segment;
    uint8_t limiting_cell;
    uint32_t prediction_steps;
    uint8_t feasible;
} ams_sop_evaluation_t;

typedef struct
{
    /* Raw electrothermal capability and direction-authorized published target.
     * Charge values are negative by the project current-sign convention. */
    float model_discharge_current_a[AMS_SOP_HORIZONS];
    float model_charge_current_a[AMS_SOP_HORIZONS];
    float discharge_current_a[AMS_SOP_HORIZONS];
    float charge_current_a[AMS_SOP_HORIZONS];
    float discharge_power_w[AMS_SOP_HORIZONS];
    float charge_power_w[AMS_SOP_HORIZONS];

    ams_sop_binding_t discharge_binding[AMS_SOP_HORIZONS];
    ams_sop_binding_t charge_binding[AMS_SOP_HORIZONS];
    uint8_t discharge_limiting_segment[AMS_SOP_HORIZONS];
    uint8_t discharge_limiting_cell[AMS_SOP_HORIZONS];
    uint8_t charge_limiting_segment[AMS_SOP_HORIZONS];
    uint8_t charge_limiting_cell[AMS_SOP_HORIZONS];
    ams_sop_prediction_extrema_t discharge_extrema[AMS_SOP_HORIZONS];
    ams_sop_prediction_extrema_t charge_extrema[AMS_SOP_HORIZONS];

    uint32_t measurement_sequence;
    uint32_t measurement_timestamp_ms;
    uint32_t solve_timestamp_ms;
    uint32_t reason_flags;
    uint32_t feasibility_evaluations;
    uint32_t prediction_steps;
    uint8_t valid;
    uint8_t authority_valid;
    uint8_t fallback_active;
} ams_sop_result_t;

void ams_sop_default_config(ams_sop_config_t *cfg);
bool ams_sop_config_valid(const ams_sop_config_t *cfg);
uint32_t ams_sop_input_reason_flags(const ams_sop_input_t *input,
                                    const ams_sop_config_t *cfg);
ams_sop_status_t ams_sop_solve(const ams_sop_input_t *input,
                               const ams_sop_config_t *cfg,
                               ams_sop_result_t *result);
ams_sop_status_t ams_sop_evaluate_current(const ams_sop_input_t *input,
                                          const ams_sop_config_t *cfg,
                                          float pack_current_a,
                                          float horizon_s,
                                          ams_sop_evaluation_t *evaluation);

/* Apply the production publication policy: reductions and invalidation are
 * immediate, while recovering/increasing limits are bounded. */
void ams_sop_apply_slew(const ams_sop_result_t *raw,
                        const ams_sop_result_t *previous,
                        const ams_sop_config_t *cfg,
                        float elapsed_s,
                        ams_sop_result_t *published);

/* Cause-scheduled recovery. Reductions remain immediate. Voltage recovery is
 * fast, thermal/current-path recovery is slower, and an SoC-bound limit is
 * held until independently observed net charge/discharge has restored SoC. */
void ams_sop_apply_recovery(const ams_sop_result_t *raw,
                            const ams_sop_result_t *previous,
                            const ams_sop_config_t *cfg,
                            const ams_sop_recovery_context_t *context,
                            float elapsed_s,
                            ams_sop_result_t *published);

const char *ams_sop_binding_name(ams_sop_binding_t binding);

#ifdef __cplusplus
}
#endif

#endif /* INC_SOP_AMS_SOP_H_ */
