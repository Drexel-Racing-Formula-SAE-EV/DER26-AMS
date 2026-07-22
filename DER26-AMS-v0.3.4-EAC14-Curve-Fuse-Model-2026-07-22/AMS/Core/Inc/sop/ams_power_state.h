/* Production integration boundary for DADEKF -> SoH -> SoP -> CAN. */

#ifndef INC_SOP_AMS_POWER_STATE_H_
#define INC_SOP_AMS_POWER_STATE_H_

#include <stdbool.h>
#include <stdint.h>

#include "estimator/ams_soc_ekf.h"
#include "measurement/ams_measurement.h"
#include "soh/ams_soh.h"
#include "sop/ams_fuse_observer.h"
#include "sop/ams_power_strategy.h"
#include "sop/ams_sop.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    ams_sop_operating_mode_t operating_mode;
    uint8_t discharge_authorized;
    uint8_t charger_authorized;
    uint8_t regen_authorized;
    uint8_t current_calibrated;
    uint8_t current_polarity_validated;
    ams_mission_profile_t requested_mission;
    uint8_t mission_request_valid;
    uint8_t stationary_confirmed;
    uint8_t fuse_model_validated;
} ams_power_policy_t;

/* Small immutable transport snapshot. The application publishes/copies this
 * under its existing short RTOS critical sections rather than sharing the full
 * solver state across task priorities. */
typedef struct
{
    uint32_t generation;
    uint32_t measurement_sequence;
    uint32_t measurement_timestamp_ms;
    uint32_t solve_timestamp_ms;
    uint32_t reason_flags;
    float discharge_current_a[AMS_SOP_HORIZONS];
    float charge_current_a[AMS_SOP_HORIZONS];
    float discharge_power_w_1s;
    float charge_power_w_1s;
    float capacity_soh;
    float capacity_soh_lower;
    float resistance_growth_upper;
    uint8_t discharge_binding[AMS_SOP_HORIZONS];
    uint8_t charge_binding[AMS_SOP_HORIZONS];
    uint8_t discharge_limiting_segment[AMS_SOP_HORIZONS];
    uint8_t charge_limiting_segment[AMS_SOP_HORIZONS];
    uint8_t capacity_confidence_pct;
    uint8_t resistance_confidence_pct;
    uint8_t capacity_valid;
    uint8_t resistance_valid;
    float fuse_utilization;
    float minimum_core_temp_c;
    float thermal_energy_to_target_wh;
    uint16_t strategy_reason_flags;
    uint8_t mission_profile;
    uint8_t mission_horizon_index;
    uint8_t thermal_ready;
    uint8_t r0_bootstrap_progress_pct;
    uint8_t fuse_authority_valid;
    uint8_t limp_latched;
    uint8_t valid;
    uint8_t authority_valid;
} ams_power_can_snapshot_t;

typedef struct
{
    ams_sop_config_t sop_config;
    ams_soh_config_t soh_config;
    ams_fuse_observer_config_t fuse_config;
    ams_power_strategy_config_t strategy_config;
    ams_soh_estimator_t soh;
    ams_fuse_observer_t fuse;
    ams_power_strategy_state_t strategy;
    ams_fuse_observer_result_t fuse_result;
    ams_power_strategy_result_t strategy_result;
    ams_sop_result_t raw_result;
    ams_sop_result_t strategy_limited_result;
    ams_sop_result_t published_result;
    ams_power_can_snapshot_t can_snapshot;
    uint32_t update_count;
    uint32_t valid_count;
    uint32_t invalid_count;
    uint32_t numeric_failure_count;
    uint32_t last_update_tick;
    uint32_t last_valid_tick;
    uint32_t last_solver_status;
    float discharge_soc_hold_reference;
    float charge_soc_hold_reference;
    double discharge_soc_hold_charge_as;
    double charge_soc_hold_charge_as;
    uint8_t discharge_soc_hold_active;
    uint8_t charge_soc_hold_active;
} ams_power_state_t;

void ams_power_state_init(ams_power_state_t *state);
void ams_power_state_invalidate(ams_power_state_t *state,
                                uint32_t now_ms,
                                uint32_t reason_flags);
bool ams_power_state_update(ams_power_state_t *state,
                            const ams_measurement_snapshot_t *measurement,
                            const ams_estimator_t *estimator,
                            const ams_power_policy_t *policy,
                            uint32_t now_ms,
                            float elapsed_s);

#ifdef __cplusplus
}
#endif

#endif /* INC_SOP_AMS_POWER_STATE_H_ */
