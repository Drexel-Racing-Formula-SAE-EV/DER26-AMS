#include "sop/ams_power_can.h"

#include <math.h>
#include <string.h>

static uint16_t sat_u16(float value)
{
    if(!isfinite(value) || (value <= 0.0f))
    {
        return 0u;
    }
    if(value >= 65535.0f)
    {
        return UINT16_MAX;
    }
    return (uint16_t)(value + 0.5f);
}

static uint8_t sat_u8(float value)
{
    if(!isfinite(value) || (value <= 0.0f))
    {
        return 0u;
    }
    if(value >= 255.0f)
    {
        return UINT8_MAX;
    }
    return (uint8_t)(value + 0.5f);
}

static void put_u16(uint8_t payload[8], uint8_t offset, uint16_t value)
{
    payload[offset] = (uint8_t)(value >> 8u);
    payload[offset + 1u] = (uint8_t)value;
}

static uint16_t get_u16(const uint8_t payload[8], uint8_t offset)
{
    return (uint16_t)(((uint16_t)payload[offset] << 8u) |
                      payload[offset + 1u]);
}

static bool snapshot_fresh(const ams_power_can_snapshot_t *snapshot,
                           uint32_t now_ms)
{
    return (snapshot != NULL) && (snapshot->generation != 0u) &&
           (snapshot->valid != 0u) && (snapshot->authority_valid != 0u) &&
           ((uint32_t)(now_ms - snapshot->solve_timestamp_ms) <=
            AMS_POWER_CAN_MAX_AGE_MS) &&
           ((uint32_t)(now_ms - snapshot->measurement_timestamp_ms) <=
            AMS_POWER_CAN_MAX_AGE_MS);
}

static uint8_t encode_flags(const ams_power_can_snapshot_t *snapshot,
                            bool fresh,
                            bool direction_inhibited)
{
    uint8_t flags = 0u;
    if(fresh)
    {
        flags |= AMS_POWER_CAN_FLAG_VALID |
                 AMS_POWER_CAN_FLAG_AUTHORITY_VALID;
    }
    if(snapshot != NULL)
    {
        if((snapshot->reason_flags & AMS_SOP_REASON_SOH_CAPACITY_PRIOR) != 0u)
        {
            flags |= AMS_POWER_CAN_FLAG_CAPACITY_PRIOR;
        }
        if((snapshot->reason_flags & AMS_SOP_REASON_SOH_RESISTANCE_PRIOR) != 0u)
        {
            flags |= AMS_POWER_CAN_FLAG_RESISTANCE_PRIOR;
        }
        if((snapshot->reason_flags & AMS_SOP_REASON_AMBIENT_PROXY) != 0u)
        {
            flags |= AMS_POWER_CAN_FLAG_AMBIENT_PROXY;
        }
        if((snapshot->reason_flags & AMS_SOP_REASON_LIMIT_SLEWED) != 0u)
        {
            flags |= AMS_POWER_CAN_FLAG_SLEWED;
        }
        if(!fresh || (snapshot->valid == 0u) ||
           (snapshot->authority_valid == 0u))
        {
            flags |= AMS_POWER_CAN_FLAG_FALLBACK;
        }
    }
    else
    {
        flags |= AMS_POWER_CAN_FLAG_FALLBACK;
    }
    if(direction_inhibited)
    {
        flags |= AMS_POWER_CAN_FLAG_DIRECTION_INHIBIT;
    }
    return flags;
}

uint8_t ams_power_can_crc8(uint16_t can_id, const uint8_t payload[7])
{
    if(payload == NULL)
    {
        return 0u;
    }

    /* CRC-8/SAE-J1850: poly 0x1D, init/xorout 0xFF. Include the 11-bit ID so
     * a valid payload cannot be accepted under the wrong power-frame ID. */
    uint8_t bytes[9];
    bytes[0] = (uint8_t)(can_id >> 8u);
    bytes[1] = (uint8_t)can_id;
    memcpy(&bytes[2], payload, 7u);
    uint8_t crc = 0xFFu;
    for(uint8_t i = 0u; i < sizeof(bytes); i++)
    {
        crc ^= bytes[i];
        for(uint8_t bit = 0u; bit < 8u; bit++)
        {
            crc = ((crc & 0x80u) != 0u) ?
                (uint8_t)((crc << 1u) ^ 0x1Du) :
                (uint8_t)(crc << 1u);
        }
    }
    return (uint8_t)(crc ^ 0xFFu);
}

bool ams_power_can_frame_valid(uint16_t can_id, const uint8_t payload[8])
{
    if(payload == NULL)
    {
        return false;
    }
    if((can_id != AMS_POWER_CAN_DCL_ID) &&
       (can_id != AMS_POWER_CAN_CCL_ID) &&
       (can_id != AMS_POWER_CAN_SOH_ID) &&
       (can_id != AMS_POWER_CAN_ENVELOPE_ID) &&
       (can_id != AMS_POWER_CAN_STRATEGY_ID) &&
       (can_id != AMS_POWER_CAN_BINDINGS_ID))
    {
        return false;
    }
    if((payload[0] >> 4u) != AMS_POWER_CAN_PROTOCOL_VERSION)
    {
        return false;
    }
    return payload[7] == ams_power_can_crc8(can_id, payload);
}

static void encode_limit(const ams_power_can_snapshot_t *snapshot,
                         uint16_t can_id,
                         uint8_t counter,
                         uint32_t now_ms,
                         bool discharge,
                         uint8_t payload[8])
{
    memset(payload, 0, 8u);
    const bool fresh = snapshot_fresh(snapshot, now_ms);
    const bool inhibited = (snapshot == NULL) ||
        (discharge ?
         ((snapshot->reason_flags & AMS_SOP_REASON_DISCHARGE_INHIBITED) != 0u) :
         ((snapshot->reason_flags &
           (AMS_SOP_REASON_REGEN_INHIBITED |
            AMS_SOP_REASON_CHARGE_INHIBITED)) != 0u));
    const float current_a = (fresh && !inhibited) ?
        (discharge ? snapshot->discharge_current_a[1] :
                     -snapshot->charge_current_a[1]) : 0.0f;
    const float power_w = (fresh && !inhibited) ?
        (discharge ? snapshot->discharge_power_w_1s :
                     snapshot->charge_power_w_1s) : 0.0f;
    const uint8_t binding = (snapshot != NULL) ?
        (discharge ? snapshot->discharge_binding[1] :
                     snapshot->charge_binding[1]) :
        (uint8_t)AMS_SOP_BIND_INVALID_INPUT;
    uint8_t segment = (snapshot != NULL) ?
        (discharge ? snapshot->discharge_limiting_segment[1] :
                     snapshot->charge_limiting_segment[1]) :
        AMS_SOP_INVALID_INDEX;
    if(segment >= AMS_SOP_SEGMENTS)
    {
        segment = 0x0Fu;
    }

    payload[0] = (uint8_t)((AMS_POWER_CAN_PROTOCOL_VERSION << 4u) |
                           (counter & 0x0Fu));
    payload[1] = encode_flags(snapshot, fresh, inhibited);
    put_u16(payload, 2u, sat_u16(current_a * 10.0f));
    put_u16(payload, 4u, sat_u16(power_w / 10.0f));
    payload[6] = (uint8_t)(((binding & 0x0Fu) << 4u) |
                           (segment & 0x0Fu));
    payload[7] = ams_power_can_crc8(can_id, payload);
}

void ams_power_can_encode_dcl(const ams_power_can_snapshot_t *snapshot,
                              uint8_t counter,
                              uint32_t now_ms,
                              uint8_t payload[8])
{
    encode_limit(snapshot, AMS_POWER_CAN_DCL_ID, counter, now_ms, true,
                 payload);
}

void ams_power_can_encode_ccl(const ams_power_can_snapshot_t *snapshot,
                              uint8_t counter,
                              uint32_t now_ms,
                              uint8_t payload[8])
{
    encode_limit(snapshot, AMS_POWER_CAN_CCL_ID, counter, now_ms, false,
                 payload);
}

void ams_power_can_encode_soh(const ams_power_can_snapshot_t *snapshot,
                              uint8_t counter,
                              uint32_t now_ms,
                              uint8_t payload[8])
{
    memset(payload, 0, 8u);
    const bool fresh = snapshot_fresh(snapshot, now_ms);
    payload[0] = (uint8_t)((AMS_POWER_CAN_PROTOCOL_VERSION << 4u) |
                           (counter & 0x0Fu));
    if(snapshot != NULL)
    {
        payload[1] = sat_u8(snapshot->capacity_soh * 100.0f);
        payload[2] = sat_u8(snapshot->capacity_soh_lower * 100.0f);
        payload[3] = sat_u8(snapshot->resistance_growth_upper * 100.0f);
        const float resistance_health =
            (snapshot->resistance_growth_upper > 0.0f) ?
            1.0f / snapshot->resistance_growth_upper : 0.0f;
        const float combined = (snapshot->capacity_soh_lower <
                                resistance_health) ?
            snapshot->capacity_soh_lower : resistance_health;
        payload[4] = sat_u8(combined * 100.0f);
        payload[5] = (uint8_t)((snapshot->capacity_confidence_pct & 0x7Fu) |
            ((fresh && (snapshot->capacity_valid != 0u)) ? 0x80u : 0u));
        payload[6] = (uint8_t)((snapshot->resistance_confidence_pct & 0x7Fu) |
            ((fresh && (snapshot->resistance_valid != 0u)) ? 0x80u : 0u));
    }
    payload[7] = ams_power_can_crc8(AMS_POWER_CAN_SOH_ID, payload);
}

void ams_power_can_encode_envelope(const ams_power_can_snapshot_t *snapshot,
                                   uint8_t counter,
                                   uint32_t now_ms,
                                   uint8_t payload[8])
{
    memset(payload, 0, 8u);
    const bool fresh = snapshot_fresh(snapshot, now_ms);
    payload[0] = (uint8_t)((AMS_POWER_CAN_PROTOCOL_VERSION << 4u) |
                           (counter & 0x0Fu));
    if(fresh)
    {
        payload[1] = sat_u8(snapshot->discharge_current_a[0]);
        payload[2] = sat_u8(snapshot->discharge_current_a[2]);
        payload[3] = sat_u8(snapshot->discharge_current_a[3]);
        payload[4] = sat_u8(-snapshot->charge_current_a[0]);
        payload[5] = sat_u8(-snapshot->charge_current_a[2]);
        payload[6] = sat_u8(-snapshot->charge_current_a[3]);
    }
    payload[7] = ams_power_can_crc8(AMS_POWER_CAN_ENVELOPE_ID, payload);
}

static uint8_t binding_nibble(uint8_t binding)
{
    return (binding <= (uint8_t)AMS_SOP_BIND_MISSION_PROFILE) ?
        binding : (uint8_t)AMS_SOP_BIND_INVALID_INPUT;
}

static uint8_t segment_nibble(uint8_t segment)
{
    return (segment < AMS_SOP_SEGMENTS) ? segment : 0x0Fu;
}

static uint8_t pack_nibbles(uint8_t high, uint8_t low)
{
    return (uint8_t)(((high & 0x0Fu) << 4u) | (low & 0x0Fu));
}

void ams_power_can_encode_strategy(const ams_power_can_snapshot_t *snapshot,
                                   uint8_t counter,
                                   uint32_t now_ms,
                                   uint8_t payload[8])
{
    memset(payload, 0, 8u);
    const bool fresh = snapshot_fresh(snapshot, now_ms);
    payload[0] = (uint8_t)((AMS_POWER_CAN_PROTOCOL_VERSION << 4u) |
                           (counter & 0x0Fu));
    if(snapshot != NULL)
    {
        payload[1] = (uint8_t)(snapshot->mission_profile & 0x03u);
        payload[1] |= (uint8_t)((snapshot->mission_horizon_index & 0x03u)
                                << 2u);
        if(fresh && (snapshot->thermal_ready != 0u))
        {
            payload[1] |= 0x10u;
        }
        if(fresh && (snapshot->fuse_authority_valid != 0u))
        {
            payload[1] |= 0x20u;
        }
        if(snapshot->limp_latched != 0u)
        {
            payload[1] |= 0x40u;
        }
        if(!fresh ||
           ((snapshot->reason_flags & AMS_SOP_REASON_MISSION_FALLBACK) != 0u))
        {
            payload[1] |= 0x80u;
        }
        payload[2] = sat_u8(snapshot->fuse_utilization * 100.0f);
        payload[3] = sat_u8(snapshot->minimum_core_temp_c + 40.0f);
        put_u16(payload, 4u,
                sat_u16(snapshot->thermal_energy_to_target_wh * 10.0f));
        payload[6] = (snapshot->r0_bootstrap_progress_pct <= 100u) ?
            snapshot->r0_bootstrap_progress_pct : 100u;
    }
    payload[7] = ams_power_can_crc8(AMS_POWER_CAN_STRATEGY_ID, payload);
}

void ams_power_can_encode_bindings(const ams_power_can_snapshot_t *snapshot,
                                   uint8_t counter,
                                   uint32_t now_ms,
                                   uint8_t payload[8])
{
    memset(payload, 0, 8u);
    const bool fresh = snapshot_fresh(snapshot, now_ms);
    payload[0] = (uint8_t)((AMS_POWER_CAN_PROTOCOL_VERSION << 4u) |
                           (counter & 0x0Fu));

    if(fresh)
    {
        const uint8_t db0 = binding_nibble(snapshot->discharge_binding[0]);
        const uint8_t db10 = binding_nibble(snapshot->discharge_binding[2]);
        const uint8_t db30 = binding_nibble(snapshot->discharge_binding[3]);
        const uint8_t cb0 = binding_nibble(snapshot->charge_binding[0]);
        const uint8_t cb10 = binding_nibble(snapshot->charge_binding[2]);
        const uint8_t cb30 = binding_nibble(snapshot->charge_binding[3]);
        const uint8_t ds0 =
            segment_nibble(snapshot->discharge_limiting_segment[0]);
        const uint8_t ds10 =
            segment_nibble(snapshot->discharge_limiting_segment[2]);
        const uint8_t ds30 =
            segment_nibble(snapshot->discharge_limiting_segment[3]);
        const uint8_t cs0 =
            segment_nibble(snapshot->charge_limiting_segment[0]);
        const uint8_t cs10 =
            segment_nibble(snapshot->charge_limiting_segment[2]);
        const uint8_t cs30 =
            segment_nibble(snapshot->charge_limiting_segment[3]);

        payload[1] = pack_nibbles(db0, db10);
        payload[2] = pack_nibbles(db30, cb0);
        payload[3] = pack_nibbles(cb10, cb30);
        payload[4] = pack_nibbles(ds0, ds10);
        payload[5] = pack_nibbles(ds30, cs0);
        payload[6] = pack_nibbles(cs10, cs30);
    }
    else
    {
        /* An all-invalid binding map is safer than stale metadata.  The core
         * DCL/CCL frames already fail to zero when the snapshot is stale. */
        payload[1] = pack_nibbles((uint8_t)AMS_SOP_BIND_INVALID_INPUT,
                                  (uint8_t)AMS_SOP_BIND_INVALID_INPUT);
        payload[2] = pack_nibbles((uint8_t)AMS_SOP_BIND_INVALID_INPUT,
                                  (uint8_t)AMS_SOP_BIND_INVALID_INPUT);
        payload[3] = pack_nibbles((uint8_t)AMS_SOP_BIND_INVALID_INPUT,
                                  (uint8_t)AMS_SOP_BIND_INVALID_INPUT);
        payload[4] = 0xFFu;
        payload[5] = 0xFFu;
        payload[6] = 0xFFu;
    }

    payload[7] = ams_power_can_crc8(AMS_POWER_CAN_BINDINGS_ID, payload);
}

bool ams_power_can_decode_limit(uint16_t can_id,
                                const uint8_t payload[8],
                                ams_power_can_limit_t *limit)
{
    if((limit == NULL) ||
       ((can_id != AMS_POWER_CAN_DCL_ID) &&
        (can_id != AMS_POWER_CAN_CCL_ID)) ||
       !ams_power_can_frame_valid(can_id, payload))
    {
        return false;
    }

    memset(limit, 0, sizeof(*limit));
    limit->counter = payload[0] & 0x0Fu;
    limit->flags = payload[1];
    limit->current_limit_a = (float)get_u16(payload, 2u) / 10.0f;
    limit->power_limit_w = (float)get_u16(payload, 4u) * 10.0f;
    limit->binding = payload[6] >> 4u;
    limit->limiting_segment = payload[6] & 0x0Fu;
    return true;
}

bool ams_power_can_decode_soh(const uint8_t payload[8],
                              ams_power_can_soh_t *soh)
{
    if((soh == NULL) ||
       !ams_power_can_frame_valid(AMS_POWER_CAN_SOH_ID, payload))
    {
        return false;
    }
    memset(soh, 0, sizeof(*soh));
    soh->counter = payload[0] & 0x0Fu;
    soh->capacity_soh = (float)payload[1] / 100.0f;
    soh->capacity_soh_lower = (float)payload[2] / 100.0f;
    soh->resistance_growth_upper = (float)payload[3] / 100.0f;
    soh->combined_soh = (float)payload[4] / 100.0f;
    soh->capacity_confidence_pct = payload[5] & 0x7Fu;
    soh->resistance_confidence_pct = payload[6] & 0x7Fu;
    soh->capacity_valid = ((payload[5] & 0x80u) != 0u) ? 1u : 0u;
    soh->resistance_valid = ((payload[6] & 0x80u) != 0u) ? 1u : 0u;
    return true;
}

bool ams_power_can_decode_envelope(const uint8_t payload[8],
                                   ams_power_can_envelope_t *envelope)
{
    if((envelope == NULL) ||
       !ams_power_can_frame_valid(AMS_POWER_CAN_ENVELOPE_ID, payload))
    {
        return false;
    }
    memset(envelope, 0, sizeof(*envelope));
    envelope->counter = payload[0] & 0x0Fu;
    envelope->discharge_constant_current_a[AMS_POWER_CAN_HORIZON_0P1_S] =
        (float)payload[1];
    envelope->discharge_constant_current_a[AMS_POWER_CAN_HORIZON_10_S] =
        (float)payload[2];
    envelope->discharge_constant_current_a[AMS_POWER_CAN_HORIZON_30_S] =
        (float)payload[3];
    envelope->charge_constant_current_a[AMS_POWER_CAN_HORIZON_0P1_S] =
        (float)payload[4];
    envelope->charge_constant_current_a[AMS_POWER_CAN_HORIZON_10_S] =
        (float)payload[5];
    envelope->charge_constant_current_a[AMS_POWER_CAN_HORIZON_30_S] =
        (float)payload[6];
    return true;
}

bool ams_power_can_decode_strategy(const uint8_t payload[8],
                                   ams_power_can_strategy_t *strategy)
{
    if((strategy == NULL) ||
       !ams_power_can_frame_valid(AMS_POWER_CAN_STRATEGY_ID, payload))
    {
        return false;
    }
    memset(strategy, 0, sizeof(*strategy));
    strategy->counter = payload[0] & 0x0Fu;
    strategy->mission_profile = payload[1] & 0x03u;
    strategy->mission_horizon_index = (payload[1] >> 2u) & 0x03u;
    strategy->thermal_ready = ((payload[1] & 0x10u) != 0u) ? 1u : 0u;
    strategy->fuse_authority_valid =
        ((payload[1] & 0x20u) != 0u) ? 1u : 0u;
    strategy->limp_latched = ((payload[1] & 0x40u) != 0u) ? 1u : 0u;
    strategy->request_fallback = ((payload[1] & 0x80u) != 0u) ? 1u : 0u;
    strategy->fuse_utilization = (float)payload[2] / 100.0f;
    strategy->minimum_core_temp_c = (float)payload[3] - 40.0f;
    strategy->thermal_energy_to_target_wh =
        (float)get_u16(payload, 4u) / 10.0f;
    strategy->r0_bootstrap_progress_pct = payload[6];
    return (strategy->mission_profile <= (uint8_t)AMS_MISSION_LIMP_HOME) &&
           (strategy->r0_bootstrap_progress_pct <= 100u);
}

bool ams_power_can_decode_bindings(const uint8_t payload[8],
                                   ams_power_can_bindings_t *bindings)
{
    if((bindings == NULL) ||
       !ams_power_can_frame_valid(AMS_POWER_CAN_BINDINGS_ID, payload))
    {
        return false;
    }

    const uint8_t packed_binding[6] = {
        (uint8_t)(payload[1] >> 4u),
        (uint8_t)(payload[1] & 0x0Fu),
        (uint8_t)(payload[2] >> 4u),
        (uint8_t)(payload[2] & 0x0Fu),
        (uint8_t)(payload[3] >> 4u),
        (uint8_t)(payload[3] & 0x0Fu)
    };
    const uint8_t packed_segment[6] = {
        (uint8_t)(payload[4] >> 4u),
        (uint8_t)(payload[4] & 0x0Fu),
        (uint8_t)(payload[5] >> 4u),
        (uint8_t)(payload[5] & 0x0Fu),
        (uint8_t)(payload[6] >> 4u),
        (uint8_t)(payload[6] & 0x0Fu)
    };

    for(uint8_t index = 0u; index < 6u; index++)
    {
        if((packed_binding[index] > (uint8_t)AMS_SOP_BIND_MISSION_PROFILE) ||
           !((packed_segment[index] < AMS_SOP_SEGMENTS) ||
             (packed_segment[index] == 0x0Fu)))
        {
            return false;
        }
    }

    memset(bindings, 0, sizeof(*bindings));
    bindings->counter = payload[0] & 0x0Fu;
    bindings->discharge_binding[AMS_POWER_CAN_HORIZON_0P1_S] =
        packed_binding[0];
    bindings->discharge_binding[AMS_POWER_CAN_HORIZON_10_S] =
        packed_binding[1];
    bindings->discharge_binding[AMS_POWER_CAN_HORIZON_30_S] =
        packed_binding[2];
    bindings->charge_binding[AMS_POWER_CAN_HORIZON_0P1_S] =
        packed_binding[3];
    bindings->charge_binding[AMS_POWER_CAN_HORIZON_10_S] =
        packed_binding[4];
    bindings->charge_binding[AMS_POWER_CAN_HORIZON_30_S] =
        packed_binding[5];
    bindings->discharge_limiting_segment[AMS_POWER_CAN_HORIZON_0P1_S] =
        packed_segment[0];
    bindings->discharge_limiting_segment[AMS_POWER_CAN_HORIZON_10_S] =
        packed_segment[1];
    bindings->discharge_limiting_segment[AMS_POWER_CAN_HORIZON_30_S] =
        packed_segment[2];
    bindings->charge_limiting_segment[AMS_POWER_CAN_HORIZON_0P1_S] =
        packed_segment[3];
    bindings->charge_limiting_segment[AMS_POWER_CAN_HORIZON_10_S] =
        packed_segment[4];
    bindings->charge_limiting_segment[AMS_POWER_CAN_HORIZON_30_S] =
        packed_segment[5];
    return true;
}

