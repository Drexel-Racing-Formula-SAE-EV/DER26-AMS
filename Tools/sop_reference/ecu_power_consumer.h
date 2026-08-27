/* Portable reference consumer for the DER26 AMS SoP/SoH CAN contract. */

#ifndef DER26_ECU_POWER_CONSUMER_H_
#define DER26_ECU_POWER_CONSUMER_H_

#include <stdbool.h>
#include <stdint.h>

#define DER26_POWER_DCL_ID       0x684u
#define DER26_POWER_CCL_ID       0x685u
#define DER26_POWER_SOH_ID       0x686u
#define DER26_POWER_ENVELOPE_ID  0x687u
#define DER26_MISSION_REQUEST_ID 0x688u
#define DER26_POWER_STRATEGY_ID  0x689u
#define DER26_POWER_BINDINGS_ID  0x68Au

#define DER26_POWER_PROTOCOL_VERSION 2u
#define DER26_MISSION_PROTOCOL_VERSION 1u
#define DER26_POWER_MAX_AGE_MS 250u
#define DER26_POWER_MAX_BUNDLE_SKEW_MS 50u
#define DER26_POWER_REQUIRED_GOOD_BUNDLES 2u
#define DER26_POWER_DCL_MAX_DA 1200u
#define DER26_POWER_CCL_MAX_DA 150u
#define DER26_POWER_DPL_MAX_10W 4000u
#define DER26_POWER_CPL_MAX_10W 500u
#define DER26_POWER_BINDING_MAX 14u
#define DER26_POWER_CORE_FRAME_COUNT 4u

#define DER26_POWER_FLAG_VALID             (1u << 0u)
#define DER26_POWER_FLAG_AUTHORITY_VALID   (1u << 1u)
#define DER26_POWER_FLAG_DIRECTION_INHIBIT (1u << 6u)
#define DER26_POWER_FLAG_FALLBACK          (1u << 7u)

#define DER26_MISSION_ENDURANCE 0u
#define DER26_MISSION_QUALIFY   1u
#define DER26_MISSION_LIMP_HOME 2u
#define DER26_MISSION_FLAG_STATIONARY (1u << 0u)

typedef enum
{
    DER26_POWER_HORIZON_0P1_S = 0,
    DER26_POWER_HORIZON_10_S,
    DER26_POWER_HORIZON_30_S,
    DER26_POWER_WIRE_HORIZON_COUNT
} der26_power_wire_horizon_t;

/* This is the only interface intended for the final torque-transmit clamp. */
typedef struct
{
    float current_limit_a;       /* Non-negative magnitude. */
    float power_limit_w;         /* Non-negative magnitude. */
    uint8_t flags;
    uint8_t binding;
    uint8_t limiting_segment;
    uint8_t authorized;
} der26_power_direction_authority_t;

typedef struct
{
    der26_power_direction_authority_t discharge;
    der26_power_direction_authority_t charge_regen;
    uint32_t received_ms;
    uint8_t counter;
    uint8_t valid;
} der26_power_immediate_authority_t;

typedef struct
{
    /* Constant-current feasibility from the present battery state.  These
     * three values are not a torque/current schedule and must not be treated
     * as pointwise limits that reset on every ECU control cycle. */
    float discharge_constant_current_feasible_a
        [DER26_POWER_WIRE_HORIZON_COUNT];
    float charge_constant_current_feasible_a
        [DER26_POWER_WIRE_HORIZON_COUNT];

    /* Optional metadata from 0x68A.  Missing metadata never invalidates the
     * immediate scalar authority, but these arrays must not be consumed unless
     * binding_metadata_valid is true. */
    uint8_t discharge_binding[DER26_POWER_WIRE_HORIZON_COUNT];
    uint8_t charge_binding[DER26_POWER_WIRE_HORIZON_COUNT];
    uint8_t discharge_limiting_segment[DER26_POWER_WIRE_HORIZON_COUNT];
    uint8_t charge_limiting_segment[DER26_POWER_WIRE_HORIZON_COUNT];
    uint32_t received_ms;
    uint32_t binding_metadata_received_ms;
    uint8_t counter;
    uint8_t binding_metadata_valid;
} der26_power_feasibility_envelope_t;

typedef struct
{
    float capacity_soh;
    float capacity_soh_lower;
    float resistance_growth_upper;
    float combined_soh;
    uint32_t received_ms;
    uint8_t counter;
    uint8_t capacity_confidence_pct;
    uint8_t resistance_confidence_pct;
    uint8_t capacity_valid;
    uint8_t resistance_valid;
} der26_power_soh_t;

typedef struct
{
    /* 0x689 is AMS-owned resource/strategy state.  The ECU may predict it
     * forward inside an optimizer, but must resynchronize from this value and
     * may never use a local model to increase the AMS scalar authority. */
    float fuse_utilization;
    float minimum_core_temp_c;
    float thermal_energy_to_target_wh;
    uint32_t received_ms;
    uint8_t counter;
    uint8_t mission_profile;
    uint8_t mission_horizon_index; /* AMS internal 0.1/1/10/30 index. */
    uint8_t thermal_ready;
    uint8_t fuse_authority_valid;
    uint8_t limp_latched;
    uint8_t mission_fallback;
    uint8_t r0_bootstrap_progress_pct;
    uint8_t valid;
} der26_power_resource_state_t;

typedef struct
{
    uint8_t discharge_binding[DER26_POWER_WIRE_HORIZON_COUNT];
    uint8_t charge_binding[DER26_POWER_WIRE_HORIZON_COUNT];
    uint8_t discharge_limiting_segment[DER26_POWER_WIRE_HORIZON_COUNT];
    uint8_t charge_limiting_segment[DER26_POWER_WIRE_HORIZON_COUNT];
    uint32_t received_ms;
    uint8_t counter;
    uint8_t valid;
} der26_power_binding_metadata_t;

typedef struct
{
    uint8_t payload[DER26_POWER_CORE_FRAME_COUNT][8];
    uint32_t stage_started_ms;
    uint32_t last_complete_ms;
    uint32_t accepted_bundle_count;
    uint32_t crc_error_count;
    uint32_t version_error_count;
    uint32_t counter_error_count;
    uint32_t malformed_count;
    uint32_t duplicate_count;
    uint32_t semantic_error_count;
    uint32_t advisory_crc_error_count;
    uint32_t advisory_version_error_count;
    uint32_t advisory_malformed_count;
    uint32_t advisory_semantic_error_count;
    der26_power_immediate_authority_t active_immediate;
    der26_power_feasibility_envelope_t active_envelope;
    der26_power_soh_t active_soh;
    der26_power_resource_state_t resource;
    der26_power_binding_metadata_t binding_metadata;
    uint8_t stage_mask;
    uint8_t stage_counter;
    uint8_t last_complete_counter;
    uint8_t good_bundle_streak;
    uint8_t stage_active;
    uint8_t complete_seen;
    uint8_t active_valid;
} der26_power_consumer_t;

void der26_power_consumer_init(der26_power_consumer_t *consumer);
bool der26_power_consumer_ingest(der26_power_consumer_t *consumer,
                                 uint16_t can_id,
                                 bool extended,
                                 bool remote,
                                 uint8_t dlc,
                                 const uint8_t payload[8],
                                 uint32_t now_ms);

/* Safe default API for the final torque clamp.  Output is always zeroed on
 * failure or staleness. */
bool der26_power_consumer_get_immediate_authority(
    const der26_power_consumer_t *consumer,
    uint32_t now_ms,
    der26_power_immediate_authority_t *authority);

/* Diagnostic/planning envelope.  The wire contains exactly three horizons:
 * 0.1 s, 10 s, and 30 s.  There is no phantom 1 s array slot. */
bool der26_power_consumer_get_feasibility_envelope(
    const der26_power_consumer_t *consumer,
    uint32_t now_ms,
    der26_power_feasibility_envelope_t *envelope);

/* Returns false unless 0x689 is fresh and counter-synchronized to the active
 * four-frame authority bundle. */
bool der26_power_consumer_get_resource_state(
    const der26_power_consumer_t *consumer,
    uint32_t now_ms,
    der26_power_resource_state_t *resource);

bool der26_power_consumer_get_soh(const der26_power_consumer_t *consumer,
                                  uint32_t now_ms,
                                  der26_power_soh_t *soh);

uint8_t der26_power_crc8(uint16_t can_id, const uint8_t payload[7]);
bool der26_mission_request_encode(uint8_t profile,
                                  uint8_t counter,
                                  bool stationary_confirmed,
                                  uint8_t payload[8]);

#endif /* DER26_ECU_POWER_CONSUMER_H_ */
