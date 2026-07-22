/* Mission arbitration and non-authoritative readiness diagnostics. */

#ifndef INC_SOP_AMS_POWER_STRATEGY_H_
#define INC_SOP_AMS_POWER_STRATEGY_H_

#include <stdbool.h>
#include <stdint.h>

#include "sop/ams_fuse_observer.h"
#include "sop/ams_sop.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_MISSION_CAN_REQUEST_ID       0x688u
#define AMS_MISSION_CAN_STATUS_ID        0x689u
#define AMS_MISSION_PROTOCOL_VERSION     1u
#define AMS_MISSION_REQUEST_MAX_AGE_MS 250u

#define AMS_MISSION_REQUEST_FLAG_STATIONARY (1u << 0u)

#define AMS_STRATEGY_REASON_NONE              0x0000u
#define AMS_STRATEGY_REASON_REQUEST_STALE     0x0001u
#define AMS_STRATEGY_REASON_REQUEST_INVALID   0x0002u
#define AMS_STRATEGY_REASON_QUALIFY_BLOCKED   0x0004u
#define AMS_STRATEGY_REASON_AUTO_LIMP         0x0008u
#define AMS_STRATEGY_REASON_MISSION_DERATED   0x0010u
#define AMS_STRATEGY_REASON_FUSE_SHADOW       0x0020u
#define AMS_STRATEGY_REASON_FUSE_DERATED      0x0040u
#define AMS_STRATEGY_REASON_THERMAL_NOT_READY 0x0080u
#define AMS_STRATEGY_REASON_R0_UNQUALIFIED    0x0100u

typedef enum
{
    AMS_MISSION_ENDURANCE = 0,
    AMS_MISSION_QUALIFY = 1,
    AMS_MISSION_LIMP_HOME = 2
} ams_mission_profile_t;

typedef struct
{
    ams_mission_profile_t requested_profile;
    uint32_t last_rx_ms;
    uint32_t accepted_count;
    uint32_t rejected_count;
    uint8_t counter;
    uint8_t good_streak;
    uint8_t stationary_confirmed;
    uint8_t valid;
    uint8_t seen;
} ams_mission_request_state_t;

typedef struct
{
    float limp_trigger_soc_lower;
    float limp_current_max_a;
    float thermal_ready_target_c;
    float core_capacity_j_per_k_per_cell;
    float surface_capacity_j_per_k_per_cell;
    float cells_per_segment;
    uint8_t resistance_confidence_required_pct;
} ams_power_strategy_config_t;

typedef struct
{
    ams_mission_profile_t active_profile;
    uint32_t update_count;
    uint8_t limp_latched;
} ams_power_strategy_state_t;

typedef struct
{
    ams_mission_profile_t requested_profile;
    float minimum_segment_soc_lower;
    float segment_core_temp_c[AMS_SOP_SEGMENTS];
    float segment_surface_temp_c[AMS_SOP_SEGMENTS];
    float segment_r0_ohm[AMS_SOP_SEGMENTS];
    float pack_current_a;
    uint8_t request_valid;
    uint8_t stationary_confirmed;
    uint8_t resistance_confidence_pct;
} ams_power_strategy_input_t;

typedef struct
{
    ams_mission_profile_t active_profile;
    float recommended_discharge_current_a;
    float minimum_core_temp_c;
    float thermal_energy_to_target_wh;
    float estimated_self_heat_w;
    float estimated_self_heat_time_s;
    float fuse_utilization;
    uint16_t reason_flags;
    uint8_t recommended_horizon_index;
    uint8_t thermal_ready;
    uint8_t resistance_bootstrap_progress_pct;
    uint8_t fuse_authority_valid;
    uint8_t limp_latched;
} ams_power_strategy_result_t;

void ams_mission_request_init(ams_mission_request_state_t *state);
void ams_mission_request_encode(ams_mission_profile_t profile,
                                uint8_t counter,
                                bool stationary_confirmed,
                                uint8_t payload[8]);
bool ams_mission_request_ingest(ams_mission_request_state_t *state,
                                const uint8_t payload[8],
                                uint32_t now_ms);
bool ams_mission_request_fresh(const ams_mission_request_state_t *state,
                               uint32_t now_ms);
void ams_power_strategy_default_config(ams_power_strategy_config_t *cfg);
bool ams_power_strategy_config_valid(const ams_power_strategy_config_t *cfg);
void ams_power_strategy_init(ams_power_strategy_state_t *state);
bool ams_power_strategy_update(ams_power_strategy_state_t *state,
                               const ams_power_strategy_config_t *cfg,
                               const ams_power_strategy_input_t *input,
                               const ams_fuse_observer_result_t *fuse,
                               const ams_sop_result_t *hard_result,
                               ams_sop_result_t *limited_result,
                               ams_power_strategy_result_t *strategy_result);

#ifdef __cplusplus
}
#endif

#endif /* INC_SOP_AMS_POWER_STRATEGY_H_ */
