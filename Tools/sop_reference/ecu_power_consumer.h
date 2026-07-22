/* Portable reference consumer for the DER26 AMS SoP/SoH CAN bundle. */

#ifndef DER26_ECU_POWER_CONSUMER_H_
#define DER26_ECU_POWER_CONSUMER_H_

#include <stdbool.h>
#include <stdint.h>

#define DER26_POWER_DCL_ID      0x684u
#define DER26_POWER_CCL_ID      0x685u
#define DER26_POWER_SOH_ID      0x686u
#define DER26_POWER_ENVELOPE_ID 0x687u
#define DER26_MISSION_REQUEST_ID 0x688u
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

#define DER26_POWER_FLAG_VALID             (1u << 0u)
#define DER26_POWER_FLAG_AUTHORITY_VALID   (1u << 1u)
#define DER26_POWER_FLAG_DIRECTION_INHIBIT (1u << 6u)
#define DER26_POWER_FLAG_FALLBACK           (1u << 7u)

#define DER26_MISSION_ENDURANCE 0u
#define DER26_MISSION_QUALIFY   1u
#define DER26_MISSION_LIMP_HOME 2u
#define DER26_MISSION_FLAG_STATIONARY (1u << 0u)

typedef struct
{
    float discharge_current_limit_a;
    float charge_current_limit_a;
    float discharge_power_limit_w;
    float charge_power_limit_w;
    float discharge_envelope_a[3]; /* 0.1, 10, 30 s */
    float charge_envelope_a[3];    /* 0.1, 10, 30 s magnitudes */
    float capacity_soh;
    float capacity_soh_lower;
    float resistance_growth_upper;
    float combined_soh;
    uint32_t received_ms;
    uint8_t counter;
    uint8_t discharge_flags;
    uint8_t charge_flags;
    uint8_t discharge_binding;
    uint8_t charge_binding;
    uint8_t discharge_limiting_segment;
    uint8_t charge_limiting_segment;
    uint8_t capacity_confidence_pct;
    uint8_t resistance_confidence_pct;
    uint8_t capacity_valid;
    uint8_t resistance_valid;
    uint8_t discharge_valid;
    uint8_t charge_valid;
} der26_power_limits_t;

typedef struct
{
    uint8_t payload[4][8];
    uint32_t stage_started_ms;
    uint32_t last_complete_ms;
    uint32_t accepted_bundle_count;
    uint32_t crc_error_count;
    uint32_t version_error_count;
    uint32_t counter_error_count;
    uint32_t malformed_count;
    uint32_t duplicate_count;
    uint32_t semantic_error_count;
    der26_power_limits_t active;
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
bool der26_power_consumer_get(const der26_power_consumer_t *consumer,
                              uint32_t now_ms,
                              der26_power_limits_t *limits);
uint8_t der26_power_crc8(uint16_t can_id, const uint8_t payload[7]);
bool der26_mission_request_encode(uint8_t profile,
                                  uint8_t counter,
                                  bool stationary_confirmed,
                                  uint8_t payload[8]);

#endif /* DER26_ECU_POWER_CONSUMER_H_ */
