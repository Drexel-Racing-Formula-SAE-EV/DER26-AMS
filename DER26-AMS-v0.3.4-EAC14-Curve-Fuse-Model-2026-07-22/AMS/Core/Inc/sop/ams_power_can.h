/* Versioned Classic-CAN contract for AMS dynamic power limits and SoH. */

#ifndef INC_SOP_AMS_POWER_CAN_H_
#define INC_SOP_AMS_POWER_CAN_H_

#include <stdbool.h>
#include <stdint.h>

#include "sop/ams_power_state.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_POWER_CAN_PROTOCOL_VERSION 2u
#define AMS_POWER_CAN_DCL_ID       0x684u
#define AMS_POWER_CAN_CCL_ID       0x685u
#define AMS_POWER_CAN_SOH_ID       0x686u
#define AMS_POWER_CAN_ENVELOPE_ID  0x687u
#define AMS_POWER_CAN_STRATEGY_ID  0x689u
#define AMS_POWER_CAN_BINDINGS_ID  0x68Au
#define AMS_POWER_CAN_MAX_AGE_MS   250u

/* The wire envelope intentionally omits the internal 1 s horizon because the
 * active 1 s / mission-selected scalar is carried at higher resolution in the
 * DCL and CCL frames.  Never index this type with AMS_SOP_HORIZONS. */
typedef enum
{
    AMS_POWER_CAN_HORIZON_0P1_S = 0,
    AMS_POWER_CAN_HORIZON_10_S,
    AMS_POWER_CAN_HORIZON_30_S,
    AMS_POWER_CAN_WIRE_HORIZON_COUNT
} ams_power_can_wire_horizon_t;

#define AMS_POWER_CAN_FLAG_VALID             (1u << 0u)
#define AMS_POWER_CAN_FLAG_AUTHORITY_VALID   (1u << 1u)
#define AMS_POWER_CAN_FLAG_CAPACITY_PRIOR    (1u << 2u)
#define AMS_POWER_CAN_FLAG_RESISTANCE_PRIOR  (1u << 3u)
#define AMS_POWER_CAN_FLAG_AMBIENT_PROXY     (1u << 4u)
#define AMS_POWER_CAN_FLAG_SLEWED             (1u << 5u)
#define AMS_POWER_CAN_FLAG_DIRECTION_INHIBIT (1u << 6u)
#define AMS_POWER_CAN_FLAG_FALLBACK           (1u << 7u)

typedef struct
{
    float current_limit_a;   /* magnitude; DCL/CCL direction comes from ID */
    float power_limit_w;
    uint8_t counter;
    uint8_t flags;
    uint8_t binding;
    uint8_t limiting_segment;
} ams_power_can_limit_t;

typedef struct
{
    float capacity_soh;
    float capacity_soh_lower;
    float resistance_growth_upper;
    float combined_soh;
    uint8_t capacity_confidence_pct;
    uint8_t resistance_confidence_pct;
    uint8_t capacity_valid;
    uint8_t resistance_valid;
    uint8_t counter;
} ams_power_can_soh_t;

typedef struct
{
    /* Constant-current feasibility from the current battery state.  These
     * values are not a pointwise current schedule and must not be repeatedly
     * reset by a downstream controller. */
    float discharge_constant_current_a[AMS_POWER_CAN_WIRE_HORIZON_COUNT];
    float charge_constant_current_a[AMS_POWER_CAN_WIRE_HORIZON_COUNT];
    uint8_t counter;
} ams_power_can_envelope_t;

typedef struct
{
    uint8_t discharge_binding[AMS_POWER_CAN_WIRE_HORIZON_COUNT];
    uint8_t charge_binding[AMS_POWER_CAN_WIRE_HORIZON_COUNT];
    uint8_t discharge_limiting_segment[AMS_POWER_CAN_WIRE_HORIZON_COUNT];
    uint8_t charge_limiting_segment[AMS_POWER_CAN_WIRE_HORIZON_COUNT];
    uint8_t counter;
} ams_power_can_bindings_t;

typedef struct
{
    float fuse_utilization;
    float minimum_core_temp_c;
    float thermal_energy_to_target_wh;
    uint8_t counter;
    uint8_t mission_profile;
    uint8_t mission_horizon_index;
    uint8_t thermal_ready;
    uint8_t fuse_authority_valid;
    uint8_t limp_latched;
    uint8_t request_fallback;
    uint8_t r0_bootstrap_progress_pct;
} ams_power_can_strategy_t;

uint8_t ams_power_can_crc8(uint16_t can_id, const uint8_t payload[7]);
bool ams_power_can_frame_valid(uint16_t can_id, const uint8_t payload[8]);
void ams_power_can_encode_dcl(const ams_power_can_snapshot_t *snapshot,
                              uint8_t counter,
                              uint32_t now_ms,
                              uint8_t payload[8]);
void ams_power_can_encode_ccl(const ams_power_can_snapshot_t *snapshot,
                              uint8_t counter,
                              uint32_t now_ms,
                              uint8_t payload[8]);
void ams_power_can_encode_soh(const ams_power_can_snapshot_t *snapshot,
                              uint8_t counter,
                              uint32_t now_ms,
                              uint8_t payload[8]);
void ams_power_can_encode_envelope(const ams_power_can_snapshot_t *snapshot,
                                   uint8_t counter,
                                   uint32_t now_ms,
                                   uint8_t payload[8]);
void ams_power_can_encode_strategy(const ams_power_can_snapshot_t *snapshot,
                                   uint8_t counter,
                                   uint32_t now_ms,
                                   uint8_t payload[8]);
void ams_power_can_encode_bindings(const ams_power_can_snapshot_t *snapshot,
                                   uint8_t counter,
                                   uint32_t now_ms,
                                   uint8_t payload[8]);
bool ams_power_can_decode_limit(uint16_t can_id,
                                const uint8_t payload[8],
                                ams_power_can_limit_t *limit);
bool ams_power_can_decode_soh(const uint8_t payload[8],
                              ams_power_can_soh_t *soh);
bool ams_power_can_decode_envelope(const uint8_t payload[8],
                                   ams_power_can_envelope_t *envelope);
bool ams_power_can_decode_strategy(const uint8_t payload[8],
                                   ams_power_can_strategy_t *strategy);
bool ams_power_can_decode_bindings(const uint8_t payload[8],
                                   ams_power_can_bindings_t *bindings);

#ifdef __cplusplus
}
#endif

#endif /* INC_SOP_AMS_POWER_CAN_H_ */
