#include "ecu_power_consumer.h"

#include <string.h>

#define DER26_POWER_FULL_MASK 0x0Fu

static bool core_id(uint16_t can_id)
{
    return (can_id >= DER26_POWER_DCL_ID) &&
           (can_id <= DER26_POWER_ENVELOPE_ID);
}

static bool advisory_id(uint16_t can_id)
{
    return (can_id == DER26_POWER_STRATEGY_ID) ||
           (can_id == DER26_POWER_BINDINGS_ID);
}

static uint8_t frame_slot(uint16_t can_id)
{
    return (uint8_t)(can_id - DER26_POWER_DCL_ID);
}

static uint16_t be_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8u) | data[1]);
}

static bool segment_nibble_valid(uint8_t segment)
{
    return (segment <= 4u) || (segment == 0x0Fu);
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
       !segment_nibble_valid(segment))
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

static bool core_frame_semantic_valid(uint16_t can_id,
                                      const uint8_t payload[8])
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

static bool strategy_frame_semantic_valid(const uint8_t payload[8])
{
    return ((payload[1] & 0x03u) <= DER26_MISSION_LIMP_HOME) &&
           (((payload[1] >> 2u) & 0x03u) <= 3u) &&
           (payload[6] <= 100u);
}

static bool bindings_frame_semantic_valid(const uint8_t payload[8])
{
    const uint8_t binding[6] = {
        (uint8_t)(payload[1] >> 4u),
        (uint8_t)(payload[1] & 0x0Fu),
        (uint8_t)(payload[2] >> 4u),
        (uint8_t)(payload[2] & 0x0Fu),
        (uint8_t)(payload[3] >> 4u),
        (uint8_t)(payload[3] & 0x0Fu)
    };
    const uint8_t segment[6] = {
        (uint8_t)(payload[4] >> 4u),
        (uint8_t)(payload[4] & 0x0Fu),
        (uint8_t)(payload[5] >> 4u),
        (uint8_t)(payload[5] & 0x0Fu),
        (uint8_t)(payload[6] >> 4u),
        (uint8_t)(payload[6] & 0x0Fu)
    };

    for(uint8_t index = 0u; index < 6u; index++)
    {
        if((binding[index] > DER26_POWER_BINDING_MAX) ||
           !segment_nibble_valid(segment[index]))
        {
            return false;
        }
    }
    return true;
}

static bool bundle_semantic_valid(const der26_power_consumer_t *consumer)
{
    const uint8_t *dcl = consumer->payload[0];
    const uint8_t *ccl = consumer->payload[1];
    const uint8_t *envelope = consumer->payload[3];
    const uint16_t dcl_da = be_u16(&dcl[2]);
    const uint16_t ccl_da = be_u16(&ccl[2]);

    /* Byte quantization allows a 0.5 A tolerance around the higher-resolution
     * active scalar.  0.1 s must be >= active scalar and 10 s must be <= it. */
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
    der26_power_immediate_authority_t immediate;
    der26_power_feasibility_envelope_t feasibility;
    der26_power_soh_t decoded_soh;
    memset(&immediate, 0, sizeof(immediate));
    memset(&feasibility, 0, sizeof(feasibility));
    memset(&decoded_soh, 0, sizeof(decoded_soh));

    if(!bundle_semantic_valid(consumer))
    {
        consumer->semantic_error_count++;
        invalidate_transport(consumer);
        return false;
    }

    immediate.counter = consumer->stage_counter;
    immediate.received_ms = now_ms;
    immediate.discharge.flags = dcl[1];
    immediate.charge_regen.flags = ccl[1];
    immediate.discharge.binding = dcl[6] >> 4u;
    immediate.discharge.limiting_segment = dcl[6] & 0x0Fu;
    immediate.charge_regen.binding = ccl[6] >> 4u;
    immediate.charge_regen.limiting_segment = ccl[6] & 0x0Fu;
    immediate.discharge.authorized = direction_valid(dcl[1]) ? 1u : 0u;
    immediate.charge_regen.authorized = direction_valid(ccl[1]) ? 1u : 0u;
    if(immediate.discharge.authorized != 0u)
    {
        immediate.discharge.current_limit_a =
            (float)be_u16(&dcl[2]) / 10.0f;
        immediate.discharge.power_limit_w =
            (float)be_u16(&dcl[4]) * 10.0f;
    }
    if(immediate.charge_regen.authorized != 0u)
    {
        immediate.charge_regen.current_limit_a =
            (float)be_u16(&ccl[2]) / 10.0f;
        immediate.charge_regen.power_limit_w =
            (float)be_u16(&ccl[4]) * 10.0f;
    }

    decoded_soh.counter = consumer->stage_counter;
    decoded_soh.received_ms = now_ms;
    decoded_soh.capacity_soh = (float)soh[1] / 100.0f;
    decoded_soh.capacity_soh_lower = (float)soh[2] / 100.0f;
    decoded_soh.resistance_growth_upper = (float)soh[3] / 100.0f;
    decoded_soh.combined_soh = (float)soh[4] / 100.0f;
    decoded_soh.capacity_confidence_pct = soh[5] & 0x7Fu;
    decoded_soh.resistance_confidence_pct = soh[6] & 0x7Fu;
    decoded_soh.capacity_valid = ((soh[5] & 0x80u) != 0u) ? 1u : 0u;
    decoded_soh.resistance_valid = ((soh[6] & 0x80u) != 0u) ? 1u : 0u;

    feasibility.counter = consumer->stage_counter;
    feasibility.received_ms = now_ms;
    for(uint8_t index = 0u; index < DER26_POWER_WIRE_HORIZON_COUNT;
        index++)
    {
        feasibility.discharge_constant_current_feasible_a[index] =
            (float)envelope[index + 1u];
        feasibility.charge_constant_current_feasible_a[index] =
            (float)envelope[index + 4u];
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
    consumer->active_immediate = immediate;
    consumer->active_envelope = feasibility;
    consumer->active_soh = decoded_soh;
    consumer->active_valid =
        (consumer->good_bundle_streak >= DER26_POWER_REQUIRED_GOOD_BUNDLES) ?
            1u : 0u;
    consumer->stage_active = 0u;
    consumer->stage_mask = 0u;
    return true;
}

static bool ingest_strategy(der26_power_consumer_t *consumer,
                            const uint8_t payload[8],
                            uint32_t now_ms)
{
    if(!strategy_frame_semantic_valid(payload))
    {
        consumer->advisory_semantic_error_count++;
        return false;
    }

    der26_power_resource_state_t resource;
    memset(&resource, 0, sizeof(resource));
    resource.counter = payload[0] & 0x0Fu;
    resource.received_ms = now_ms;
    resource.mission_profile = payload[1] & 0x03u;
    resource.mission_horizon_index = (payload[1] >> 2u) & 0x03u;
    resource.thermal_ready = ((payload[1] & 0x10u) != 0u) ? 1u : 0u;
    resource.fuse_authority_valid =
        ((payload[1] & 0x20u) != 0u) ? 1u : 0u;
    resource.limp_latched = ((payload[1] & 0x40u) != 0u) ? 1u : 0u;
    resource.mission_fallback = ((payload[1] & 0x80u) != 0u) ? 1u : 0u;
    resource.fuse_utilization = (float)payload[2] / 100.0f;
    resource.minimum_core_temp_c = (float)payload[3] - 40.0f;
    resource.thermal_energy_to_target_wh =
        (float)be_u16(&payload[4]) / 10.0f;
    resource.r0_bootstrap_progress_pct = payload[6];
    resource.valid = 1u;
    consumer->resource = resource;
    return true;
}

static bool ingest_bindings(der26_power_consumer_t *consumer,
                            const uint8_t payload[8],
                            uint32_t now_ms)
{
    if(!bindings_frame_semantic_valid(payload))
    {
        consumer->advisory_semantic_error_count++;
        return false;
    }

    der26_power_binding_metadata_t metadata;
    memset(&metadata, 0, sizeof(metadata));
    metadata.counter = payload[0] & 0x0Fu;
    metadata.received_ms = now_ms;
    metadata.valid = 1u;

    metadata.discharge_binding[DER26_POWER_HORIZON_0P1_S] =
        payload[1] >> 4u;
    metadata.discharge_binding[DER26_POWER_HORIZON_10_S] =
        payload[1] & 0x0Fu;
    metadata.discharge_binding[DER26_POWER_HORIZON_30_S] =
        payload[2] >> 4u;
    metadata.charge_binding[DER26_POWER_HORIZON_0P1_S] =
        payload[2] & 0x0Fu;
    metadata.charge_binding[DER26_POWER_HORIZON_10_S] =
        payload[3] >> 4u;
    metadata.charge_binding[DER26_POWER_HORIZON_30_S] =
        payload[3] & 0x0Fu;

    metadata.discharge_limiting_segment[DER26_POWER_HORIZON_0P1_S] =
        payload[4] >> 4u;
    metadata.discharge_limiting_segment[DER26_POWER_HORIZON_10_S] =
        payload[4] & 0x0Fu;
    metadata.discharge_limiting_segment[DER26_POWER_HORIZON_30_S] =
        payload[5] >> 4u;
    metadata.charge_limiting_segment[DER26_POWER_HORIZON_0P1_S] =
        payload[5] & 0x0Fu;
    metadata.charge_limiting_segment[DER26_POWER_HORIZON_10_S] =
        payload[6] >> 4u;
    metadata.charge_limiting_segment[DER26_POWER_HORIZON_30_S] =
        payload[6] & 0x0Fu;

    consumer->binding_metadata = metadata;
    return true;
}

static void invalidate_advisory(der26_power_consumer_t *consumer,
                                uint16_t can_id)
{
    if(can_id == DER26_POWER_STRATEGY_ID)
    {
        consumer->resource.valid = 0u;
    }
    else
    {
        consumer->binding_metadata.valid = 0u;
    }
}

static bool ingest_advisory(der26_power_consumer_t *consumer,
                            uint16_t can_id,
                            bool extended,
                            bool remote,
                            uint8_t dlc,
                            const uint8_t payload[8],
                            uint32_t now_ms)
{
    /* A newly received bad advisory must not leave an older same-counter
     * advisory visible after modulo-16 wrap or retransmission.  Invalidate
     * only that optional channel; scalar DCL/CCL authority remains intact. */
    invalidate_advisory(consumer, can_id);

    if(extended || remote || (dlc != 8u))
    {
        consumer->advisory_malformed_count++;
        return false;
    }
    if((payload[0] >> 4u) != DER26_POWER_PROTOCOL_VERSION)
    {
        consumer->advisory_version_error_count++;
        return false;
    }
    if(payload[7] != der26_power_crc8(can_id, payload))
    {
        consumer->advisory_crc_error_count++;
        return false;
    }

    return (can_id == DER26_POWER_STRATEGY_ID) ?
        ingest_strategy(consumer, payload, now_ms) :
        ingest_bindings(consumer, payload, now_ms);
}

bool der26_power_consumer_ingest(der26_power_consumer_t *consumer,
                                 uint16_t can_id,
                                 bool extended,
                                 bool remote,
                                 uint8_t dlc,
                                 const uint8_t payload[8],
                                 uint32_t now_ms)
{
    if((consumer == NULL) || (payload == NULL))
    {
        return false;
    }
    if(advisory_id(can_id))
    {
        /* Advisory corruption cannot revoke an already valid scalar DCL/CCL
         * bundle.  The advisory getter simply remains unavailable. */
        return ingest_advisory(consumer, can_id, extended, remote, dlc,
                               payload, now_ms);
    }
    if(!core_id(can_id))
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
    if(!core_frame_semantic_valid(can_id, payload))
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

static bool active_bundle_fresh(const der26_power_consumer_t *consumer,
                                uint32_t now_ms)
{
    return (consumer != NULL) && (consumer->active_valid != 0u) &&
           ((uint32_t)(now_ms - consumer->active_immediate.received_ms) <=
            DER26_POWER_MAX_AGE_MS) &&
           !((consumer->stage_active != 0u) &&
             ((uint32_t)(now_ms - consumer->stage_started_ms) >
              DER26_POWER_MAX_BUNDLE_SKEW_MS));
}

bool der26_power_consumer_get_immediate_authority(
    const der26_power_consumer_t *consumer,
    uint32_t now_ms,
    der26_power_immediate_authority_t *authority)
{
    if(authority == NULL)
    {
        return false;
    }
    memset(authority, 0, sizeof(*authority));
    if(!active_bundle_fresh(consumer, now_ms))
    {
        return false;
    }

    *authority = consumer->active_immediate;
    authority->valid = 1u;
    return true;
}

bool der26_power_consumer_get_feasibility_envelope(
    const der26_power_consumer_t *consumer,
    uint32_t now_ms,
    der26_power_feasibility_envelope_t *envelope)
{
    if(envelope == NULL)
    {
        return false;
    }
    memset(envelope, 0, sizeof(*envelope));
    if(!active_bundle_fresh(consumer, now_ms))
    {
        return false;
    }

    *envelope = consumer->active_envelope;
    if((consumer->binding_metadata.valid != 0u) &&
       (consumer->binding_metadata.counter == envelope->counter) &&
       ((uint32_t)(now_ms - consumer->binding_metadata.received_ms) <=
        DER26_POWER_MAX_AGE_MS) &&
       ((uint32_t)(consumer->binding_metadata.received_ms -
                   consumer->active_immediate.received_ms) <=
        DER26_POWER_MAX_BUNDLE_SKEW_MS))
    {
        memcpy(envelope->discharge_binding,
               consumer->binding_metadata.discharge_binding,
               sizeof(envelope->discharge_binding));
        memcpy(envelope->charge_binding,
               consumer->binding_metadata.charge_binding,
               sizeof(envelope->charge_binding));
        memcpy(envelope->discharge_limiting_segment,
               consumer->binding_metadata.discharge_limiting_segment,
               sizeof(envelope->discharge_limiting_segment));
        memcpy(envelope->charge_limiting_segment,
               consumer->binding_metadata.charge_limiting_segment,
               sizeof(envelope->charge_limiting_segment));
        envelope->binding_metadata_received_ms =
            consumer->binding_metadata.received_ms;
        envelope->binding_metadata_valid = 1u;
    }
    return true;
}

bool der26_power_consumer_get_resource_state(
    const der26_power_consumer_t *consumer,
    uint32_t now_ms,
    der26_power_resource_state_t *resource)
{
    if(resource == NULL)
    {
        return false;
    }
    memset(resource, 0, sizeof(*resource));
    if(!active_bundle_fresh(consumer, now_ms) ||
       (consumer->resource.valid == 0u) ||
       (consumer->resource.counter != consumer->active_immediate.counter) ||
       ((uint32_t)(now_ms - consumer->resource.received_ms) >
        DER26_POWER_MAX_AGE_MS) ||
       ((uint32_t)(consumer->resource.received_ms -
                   consumer->active_immediate.received_ms) >
        DER26_POWER_MAX_BUNDLE_SKEW_MS))
    {
        return false;
    }

    *resource = consumer->resource;
    resource->valid = 1u;
    return true;
}

bool der26_power_consumer_get_soh(const der26_power_consumer_t *consumer,
                                  uint32_t now_ms,
                                  der26_power_soh_t *soh)
{
    if(soh == NULL)
    {
        return false;
    }
    memset(soh, 0, sizeof(*soh));
    if(!active_bundle_fresh(consumer, now_ms))
    {
        return false;
    }

    *soh = consumer->active_soh;
    return true;
}
