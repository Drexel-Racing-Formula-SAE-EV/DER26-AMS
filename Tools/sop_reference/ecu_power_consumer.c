#include "ecu_power_consumer.h"

#include <string.h>

#define DER26_POWER_FULL_MASK 0x0Fu

static bool known_id(uint16_t can_id)
{
    return (can_id >= DER26_POWER_DCL_ID) &&
           (can_id <= DER26_POWER_ENVELOPE_ID);
}

static uint8_t frame_slot(uint16_t can_id)
{
    return (uint8_t)(can_id - DER26_POWER_DCL_ID);
}

static uint16_t be_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8u) | data[1]);
}

uint8_t der26_power_crc8(uint16_t can_id, const uint8_t payload[7])
{
    if(payload == NULL)
    {
        return 0u;
    }
    uint8_t bytes[9];
    bytes[0] = (uint8_t)(can_id >> 8u);
    bytes[1] = (uint8_t)can_id;
    memcpy(&bytes[2], payload, 7u);
    uint8_t crc = 0xFFu;
    for(uint8_t index = 0u; index < sizeof(bytes); index++)
    {
        crc ^= bytes[index];
        for(uint8_t bit = 0u; bit < 8u; bit++)
        {
            crc = ((crc & 0x80u) != 0u) ?
                (uint8_t)((crc << 1u) ^ 0x1Du) :
                (uint8_t)(crc << 1u);
        }
    }
    return (uint8_t)(crc ^ 0xFFu);
}

bool der26_mission_request_encode(uint8_t profile,
                                  uint8_t counter,
                                  bool stationary_confirmed,
                                  uint8_t payload[8])
{
    if((payload == NULL) || (profile > DER26_MISSION_LIMP_HOME))
    {
        return false;
    }
    memset(payload, 0, 8u);
    payload[0] = (uint8_t)((DER26_MISSION_PROTOCOL_VERSION << 4u) |
                           (counter & 0x0Fu));
    payload[1] = profile;
    payload[2] = stationary_confirmed ?
        DER26_MISSION_FLAG_STATIONARY : 0u;
    payload[7] = der26_power_crc8(DER26_MISSION_REQUEST_ID, payload);
    return true;
}

void der26_power_consumer_init(der26_power_consumer_t *consumer)
{
    if(consumer != NULL)
    {
        memset(consumer, 0, sizeof(*consumer));
    }
}

static void invalidate_transport(der26_power_consumer_t *consumer)
{
    consumer->active_valid = 0u;
    consumer->good_bundle_streak = 0u;
    consumer->stage_active = 0u;
    consumer->stage_mask = 0u;
}

static bool direction_valid(uint8_t flags)
{
    const uint8_t required = DER26_POWER_FLAG_VALID |
                             DER26_POWER_FLAG_AUTHORITY_VALID;
    return ((flags & required) == required) &&
           ((flags & (DER26_POWER_FLAG_DIRECTION_INHIBIT |
                      DER26_POWER_FLAG_FALLBACK)) == 0u);
}

static bool limit_frame_semantic_valid(uint16_t can_id,
                                       const uint8_t payload[8])
{
    const bool valid =
        (payload[1] & DER26_POWER_FLAG_VALID) != 0u;
    const bool authority =
        (payload[1] & DER26_POWER_FLAG_AUTHORITY_VALID) != 0u;
    const bool inhibited =
        (payload[1] & DER26_POWER_FLAG_DIRECTION_INHIBIT) != 0u;
    const bool fallback =
        (payload[1] & DER26_POWER_FLAG_FALLBACK) != 0u;
    const uint16_t current_da = be_u16(&payload[2]);
    const uint16_t power_10w = be_u16(&payload[4]);
    const uint8_t binding = payload[6] >> 4u;
    const uint8_t segment = payload[6] & 0x0Fu;
    const uint16_t current_max = (can_id == DER26_POWER_DCL_ID) ?
        DER26_POWER_DCL_MAX_DA : DER26_POWER_CCL_MAX_DA;
    const uint16_t power_max = (can_id == DER26_POWER_DCL_ID) ?
        DER26_POWER_DPL_MAX_10W : DER26_POWER_CPL_MAX_10W;

    if((valid != authority) || (valid && fallback) ||
       (current_da > current_max) || (power_10w > power_max) ||
       (binding > DER26_POWER_BINDING_MAX) ||
       !((segment <= 4u) || (segment == 0x0Fu)))
    {
        return false;
    }
    if((!valid || inhibited || fallback) &&
       ((current_da != 0u) || (power_10w != 0u)))
    {
        return false;
    }
    return true;
}

static bool soh_frame_semantic_valid(const uint8_t payload[8])
{
    const bool capacity_valid = (payload[5] & 0x80u) != 0u;
    const bool resistance_valid = (payload[6] & 0x80u) != 0u;
    const uint8_t capacity_confidence = payload[5] & 0x7Fu;
    const uint8_t resistance_confidence = payload[6] & 0x7Fu;

    if((payload[1] > 110u) || (payload[2] > 100u) ||
       (payload[4] > 100u) ||
       (capacity_confidence > 100u) ||
       (resistance_confidence > 100u))
    {
        return false;
    }
    if(capacity_valid &&
       ((payload[1] < 50u) || (payload[2] < 50u) ||
        (payload[2] > payload[1])))
    {
        return false;
    }
    if(resistance_valid && (payload[3] < 100u))
    {
        return false;
    }
    return true;
}

static bool envelope_frame_semantic_valid(const uint8_t payload[8])
{
    return (payload[1] <= (DER26_POWER_DCL_MAX_DA / 10u)) &&
           (payload[2] <= (DER26_POWER_DCL_MAX_DA / 10u)) &&
           (payload[3] <= (DER26_POWER_DCL_MAX_DA / 10u)) &&
           (payload[4] <= (DER26_POWER_CCL_MAX_DA / 10u)) &&
           (payload[5] <= (DER26_POWER_CCL_MAX_DA / 10u)) &&
           (payload[6] <= (DER26_POWER_CCL_MAX_DA / 10u)) &&
           (payload[1] >= payload[2]) && (payload[2] >= payload[3]) &&
           (payload[4] >= payload[5]) && (payload[5] >= payload[6]);
}

static bool frame_semantic_valid(uint16_t can_id, const uint8_t payload[8])
{
    if((can_id == DER26_POWER_DCL_ID) ||
       (can_id == DER26_POWER_CCL_ID))
    {
        return limit_frame_semantic_valid(can_id, payload);
    }
    if(can_id == DER26_POWER_SOH_ID)
    {
        return soh_frame_semantic_valid(payload);
    }
    return envelope_frame_semantic_valid(payload);
}

static bool bundle_semantic_valid(const der26_power_consumer_t *consumer)
{
    const uint8_t *dcl = consumer->payload[0];
    const uint8_t *ccl = consumer->payload[1];
    const uint8_t *envelope = consumer->payload[3];
    const uint16_t dcl_da = be_u16(&dcl[2]);
    const uint16_t ccl_da = be_u16(&ccl[2]);

    return (((uint16_t)envelope[1] * 10u + 5u) >= dcl_da) &&
           (((uint16_t)envelope[2] * 10u) <= (dcl_da + 5u)) &&
           (((uint16_t)envelope[4] * 10u + 5u) >= ccl_da) &&
           (((uint16_t)envelope[5] * 10u) <= (ccl_da + 5u));
}

static bool complete_bundle(der26_power_consumer_t *consumer,
                            uint32_t now_ms)
{
    const uint8_t *dcl = consumer->payload[0];
    const uint8_t *ccl = consumer->payload[1];
    const uint8_t *soh = consumer->payload[2];
    const uint8_t *envelope = consumer->payload[3];
    der26_power_limits_t decoded;
    memset(&decoded, 0, sizeof(decoded));

    if(!bundle_semantic_valid(consumer))
    {
        consumer->semantic_error_count++;
        invalidate_transport(consumer);
        return false;
    }

    decoded.counter = consumer->stage_counter;
    decoded.received_ms = now_ms;
    decoded.discharge_flags = dcl[1];
    decoded.charge_flags = ccl[1];
    decoded.discharge_binding = dcl[6] >> 4u;
    decoded.discharge_limiting_segment = dcl[6] & 0x0Fu;
    decoded.charge_binding = ccl[6] >> 4u;
    decoded.charge_limiting_segment = ccl[6] & 0x0Fu;
    decoded.discharge_valid = direction_valid(dcl[1]) ? 1u : 0u;
    decoded.charge_valid = direction_valid(ccl[1]) ? 1u : 0u;
    if(decoded.discharge_valid != 0u)
    {
        decoded.discharge_current_limit_a =
            (float)be_u16(&dcl[2]) / 10.0f;
        decoded.discharge_power_limit_w = (float)be_u16(&dcl[4]) * 10.0f;
    }
    if(decoded.charge_valid != 0u)
    {
        decoded.charge_current_limit_a =
            (float)be_u16(&ccl[2]) / 10.0f;
        decoded.charge_power_limit_w = (float)be_u16(&ccl[4]) * 10.0f;
    }
    decoded.capacity_soh = (float)soh[1] / 100.0f;
    decoded.capacity_soh_lower = (float)soh[2] / 100.0f;
    decoded.resistance_growth_upper = (float)soh[3] / 100.0f;
    decoded.combined_soh = (float)soh[4] / 100.0f;
    decoded.capacity_confidence_pct = soh[5] & 0x7Fu;
    decoded.resistance_confidence_pct = soh[6] & 0x7Fu;
    decoded.capacity_valid = ((soh[5] & 0x80u) != 0u) ? 1u : 0u;
    decoded.resistance_valid = ((soh[6] & 0x80u) != 0u) ? 1u : 0u;
    for(uint8_t index = 0u; index < 3u; index++)
    {
        decoded.discharge_envelope_a[index] = (float)envelope[index + 1u];
        decoded.charge_envelope_a[index] = (float)envelope[index + 4u];
    }

    if(consumer->complete_seen != 0u)
    {
        const uint8_t expected =
            (uint8_t)((consumer->last_complete_counter + 1u) & 0x0Fu);
        if(consumer->stage_counter != expected)
        {
            consumer->counter_error_count++;
            consumer->good_bundle_streak = 0u;
        }
    }
    consumer->last_complete_counter = consumer->stage_counter;
    consumer->complete_seen = 1u;
    if(consumer->good_bundle_streak < UINT8_MAX)
    {
        consumer->good_bundle_streak++;
    }
    consumer->accepted_bundle_count++;
    consumer->last_complete_ms = now_ms;
    consumer->active = decoded;
    consumer->active_valid =
        (consumer->good_bundle_streak >= DER26_POWER_REQUIRED_GOOD_BUNDLES) ?
            1u : 0u;
    consumer->stage_active = 0u;
    consumer->stage_mask = 0u;
    return true;
}

bool der26_power_consumer_ingest(der26_power_consumer_t *consumer,
                                 uint16_t can_id,
                                 bool extended,
                                 bool remote,
                                 uint8_t dlc,
                                 const uint8_t payload[8],
                                 uint32_t now_ms)
{
    if((consumer == NULL) || (payload == NULL) || !known_id(can_id))
    {
        return false;
    }
    if(extended || remote || (dlc != 8u))
    {
        consumer->malformed_count++;
        invalidate_transport(consumer);
        return false;
    }
    if((payload[0] >> 4u) != DER26_POWER_PROTOCOL_VERSION)
    {
        consumer->version_error_count++;
        invalidate_transport(consumer);
        return false;
    }
    if(payload[7] != der26_power_crc8(can_id, payload))
    {
        consumer->crc_error_count++;
        invalidate_transport(consumer);
        return false;
    }
    if(!frame_semantic_valid(can_id, payload))
    {
        consumer->semantic_error_count++;
        invalidate_transport(consumer);
        return false;
    }

    const uint8_t counter = payload[0] & 0x0Fu;
    const uint8_t slot = frame_slot(can_id);
    const uint8_t slot_mask = (uint8_t)(1u << slot);
    if((consumer->stage_active == 0u) ||
       (counter != consumer->stage_counter))
    {
        if((consumer->stage_active != 0u) &&
           (consumer->stage_mask != DER26_POWER_FULL_MASK))
        {
            consumer->counter_error_count++;
            consumer->active_valid = 0u;
            consumer->good_bundle_streak = 0u;
        }
        consumer->stage_active = 1u;
        consumer->stage_counter = counter;
        consumer->stage_mask = 0u;
        consumer->stage_started_ms = now_ms;
    }
    if((uint32_t)(now_ms - consumer->stage_started_ms) >
       DER26_POWER_MAX_BUNDLE_SKEW_MS)
    {
        consumer->counter_error_count++;
        invalidate_transport(consumer);
        return false;
    }
    if((consumer->stage_mask & slot_mask) != 0u)
    {
        consumer->duplicate_count++;
        invalidate_transport(consumer);
        return false;
    }

    memcpy(consumer->payload[slot], payload, 8u);
    consumer->stage_mask |= slot_mask;
    if(consumer->stage_mask == DER26_POWER_FULL_MASK)
    {
        return complete_bundle(consumer, now_ms);
    }
    return true;
}

bool der26_power_consumer_get(const der26_power_consumer_t *consumer,
                              uint32_t now_ms,
                              der26_power_limits_t *limits)
{
    if(limits == NULL)
    {
        return false;
    }
    memset(limits, 0, sizeof(*limits));
    if((consumer == NULL) || (consumer->active_valid == 0u) ||
       ((uint32_t)(now_ms - consumer->active.received_ms) >
        DER26_POWER_MAX_AGE_MS) ||
       ((consumer->stage_active != 0u) &&
        ((uint32_t)(now_ms - consumer->stage_started_ms) >
         DER26_POWER_MAX_BUNDLE_SKEW_MS)))
    {
        return false;
    }
    *limits = consumer->active;
    return true;
}
