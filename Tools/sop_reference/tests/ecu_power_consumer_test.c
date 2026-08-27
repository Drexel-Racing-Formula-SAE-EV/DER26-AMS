#include "ecu_power_consumer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expression) do { if(!(expression)) { \
    fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #expression); \
    exit(1); } } while(0)

static void put_u16(uint8_t payload[8], uint8_t offset, uint16_t value)
{
    payload[offset] = (uint8_t)(value >> 8u);
    payload[offset + 1u] = (uint8_t)value;
}

static void finalize(uint16_t can_id, uint8_t payload[8])
{
    payload[7] = der26_power_crc8(can_id, payload);
}

static void make_bundle(uint8_t counter,
                        uint8_t payload[DER26_POWER_CORE_FRAME_COUNT][8],
                        uint8_t dcl_flags,
                        uint8_t ccl_flags)
{
    memset(payload, 0, DER26_POWER_CORE_FRAME_COUNT * 8u);
    for(uint8_t slot = 0u; slot < DER26_POWER_CORE_FRAME_COUNT; slot++)
    {
        payload[slot][0] = (uint8_t)((DER26_POWER_PROTOCOL_VERSION << 4u) |
                                     (counter & 0x0Fu));
    }

    payload[0][1] = dcl_flags;
    put_u16(payload[0], 2u, 800u);
    put_u16(payload[0], 4u, 2200u);
    payload[0][6] = 0xE2u; /* Mission-profile binding, segment 2. */

    payload[1][1] = ccl_flags;
    put_u16(payload[1], 2u, 100u);
    put_u16(payload[1], 4u, 300u);
    payload[1][6] = 0xD3u; /* Fuse-thermal binding, segment 3. */

    payload[2][1] = 95u;
    payload[2][2] = 90u;
    payload[2][3] = 120u;
    payload[2][4] = 83u;
    payload[2][5] = (uint8_t)(0x80u | 75u);
    payload[2][6] = (uint8_t)(0x80u | 80u);

    payload[3][1] = 118u;
    payload[3][2] = 70u;
    payload[3][3] = 65u;
    payload[3][4] = 11u;
    payload[3][5] = 10u;
    payload[3][6] = 9u;

    for(uint8_t slot = 0u; slot < DER26_POWER_CORE_FRAME_COUNT; slot++)
    {
        finalize((uint16_t)(DER26_POWER_DCL_ID + slot), payload[slot]);
    }
}

static void make_zero_bundle(
    uint8_t counter,
    uint8_t payload[DER26_POWER_CORE_FRAME_COUNT][8])
{
    make_bundle(counter, payload,
                DER26_POWER_FLAG_FALLBACK,
                DER26_POWER_FLAG_FALLBACK);
    put_u16(payload[0], 2u, 0u);
    put_u16(payload[0], 4u, 0u);
    put_u16(payload[1], 2u, 0u);
    put_u16(payload[1], 4u, 0u);
    memset(&payload[3][1], 0, 6u);
    finalize(DER26_POWER_DCL_ID, payload[0]);
    finalize(DER26_POWER_CCL_ID, payload[1]);
    finalize(DER26_POWER_ENVELOPE_ID, payload[3]);
}

static void make_strategy(uint8_t counter, uint8_t payload[8])
{
    memset(payload, 0, 8u);
    payload[0] = (uint8_t)((DER26_POWER_PROTOCOL_VERSION << 4u) |
                           (counter & 0x0Fu));
    payload[1] = (uint8_t)(DER26_MISSION_LIMP_HOME | (3u << 2u) |
                           0x10u | 0x20u | 0x40u);
    payload[2] = 76u;
    payload[3] = 62u; /* 22 C after subtracting 40. */
    put_u16(payload, 4u, 264u);
    payload[6] = 65u;
    finalize(DER26_POWER_STRATEGY_ID, payload);
}

static void make_bindings(uint8_t counter, uint8_t payload[8])
{
    memset(payload, 0, 8u);
    payload[0] = (uint8_t)((DER26_POWER_PROTOCOL_VERSION << 4u) |
                           (counter & 0x0Fu));
    payload[1] = 0x15u; /* D: 0.1 s UV, 10 s core temperature. */
    payload[2] = 0xE2u; /* D: 30 s mission; C: 0.1 s OV. */
    payload[3] = 0x64u; /* C: 10 s surface temperature, 30 s high SoC. */
    payload[4] = 0x02u; /* D segments 0, 2. */
    payload[5] = 0x41u; /* D30 segment 4; C0.1 segment 1. */
    payload[6] = 0x3Fu; /* C10 segment 3; C30 none. */
    finalize(DER26_POWER_BINDINGS_ID, payload);
}

static void ingest_bundle_order(
    der26_power_consumer_t *consumer,
    uint8_t payload[DER26_POWER_CORE_FRAME_COUNT][8],
    const uint8_t order[DER26_POWER_CORE_FRAME_COUNT],
    uint32_t now_ms)
{
    for(uint8_t index = 0u; index < DER26_POWER_CORE_FRAME_COUNT; index++)
    {
        const uint8_t slot = order[index];
        CHECK(der26_power_consumer_ingest(
            consumer, (uint16_t)(DER26_POWER_DCL_ID + slot), false, false, 8u,
            payload[slot], now_ms + index));
    }
}

static void ingest_bundle(
    der26_power_consumer_t *consumer,
    uint8_t payload[DER26_POWER_CORE_FRAME_COUNT][8],
    uint32_t now_ms)
{
    static const uint8_t natural_order[DER26_POWER_CORE_FRAME_COUNT] =
        { 0u, 1u, 2u, 3u };
    ingest_bundle_order(consumer, payload, natural_order, now_ms);
}

int main(void)
{
    const uint8_t valid_flags = DER26_POWER_FLAG_VALID |
                                DER26_POWER_FLAG_AUTHORITY_VALID;
    der26_power_consumer_t consumer;
    der26_power_immediate_authority_t authority;
    der26_power_feasibility_envelope_t envelope;
    der26_power_resource_state_t resource;
    der26_power_soh_t soh;
    uint8_t payload[DER26_POWER_CORE_FRAME_COUNT][8];
    uint8_t advisory[8];

    CHECK(DER26_POWER_WIRE_HORIZON_COUNT == 3u);
    der26_power_consumer_init(&consumer);

    uint8_t mission[8];
    CHECK(der26_mission_request_encode(DER26_MISSION_QUALIFY, 5u, true,
                                       mission));
    CHECK(mission[0] == 0x15u);
    CHECK(mission[1] == DER26_MISSION_QUALIFY);
    CHECK(mission[2] == DER26_MISSION_FLAG_STATIONARY);
    CHECK(mission[7] == der26_power_crc8(DER26_MISSION_REQUEST_ID,
                                         mission));
    CHECK(!der26_mission_request_encode(3u, 6u, false, mission));

    /* Every ordering of the four required frames must assemble atomically. */
    static const uint8_t permutations[24][DER26_POWER_CORE_FRAME_COUNT] = {
        {0u,1u,2u,3u}, {0u,1u,3u,2u}, {0u,2u,1u,3u},
        {0u,2u,3u,1u}, {0u,3u,1u,2u}, {0u,3u,2u,1u},
        {1u,0u,2u,3u}, {1u,0u,3u,2u}, {1u,2u,0u,3u},
        {1u,2u,3u,0u}, {1u,3u,0u,2u}, {1u,3u,2u,0u},
        {2u,0u,1u,3u}, {2u,0u,3u,1u}, {2u,1u,0u,3u},
        {2u,1u,3u,0u}, {2u,3u,0u,1u}, {2u,3u,1u,0u},
        {3u,0u,1u,2u}, {3u,0u,2u,1u}, {3u,1u,0u,2u},
        {3u,1u,2u,0u}, {3u,2u,0u,1u}, {3u,2u,1u,0u}
    };
    for(uint8_t permutation = 0u; permutation < 24u; permutation++)
    {
        der26_power_consumer_init(&consumer);
        make_bundle(1u, payload, valid_flags, valid_flags);
        ingest_bundle_order(&consumer, payload, permutations[permutation],
                            10u);
        make_bundle(2u, payload, valid_flags, valid_flags);
        ingest_bundle_order(&consumer, payload, permutations[permutation],
                            20u);
        CHECK(der26_power_consumer_get_immediate_authority(
            &consumer, 24u, &authority));
        CHECK(authority.counter == 2u);
    }
    der26_power_consumer_init(&consumer);

    /* Duplicate, skew, and malformed core traffic fail closed. */
    make_bundle(1u, payload, valid_flags, valid_flags);
    CHECK(der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID, false,
                                      false, 8u, payload[0], 40u));
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID, false,
                                       false, 8u, payload[0], 41u));
    CHECK(consumer.duplicate_count == 1u);
    CHECK(!der26_power_consumer_get_immediate_authority(&consumer, 41u,
                                                        &authority));

    der26_power_consumer_init(&consumer);
    CHECK(der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID, false,
                                      false, 8u, payload[0], 50u));
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_CCL_ID, false,
                                       false, 8u, payload[1], 101u));
    CHECK(consumer.counter_error_count == 1u);

    /* Two coherent bundles are required before scalar authority is exposed. */
    make_bundle(1u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 100u);
    CHECK(!der26_power_consumer_get_immediate_authority(&consumer, 104u,
                                                        &authority));
    CHECK(authority.discharge.current_limit_a == 0.0f);

    make_bundle(2u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 200u);
    CHECK(der26_power_consumer_get_immediate_authority(&consumer, 204u,
                                                       &authority));
    CHECK(authority.valid);
    CHECK(authority.counter == 2u);
    CHECK(authority.discharge.authorized);
    CHECK(authority.charge_regen.authorized);
    CHECK(authority.discharge.current_limit_a == 80.0f);
    CHECK(authority.charge_regen.current_limit_a == 10.0f);
    CHECK(authority.discharge.power_limit_w == 22000.0f);
    CHECK(authority.discharge.binding == 14u);
    CHECK(authority.charge_regen.binding == 13u);

    CHECK(der26_power_consumer_get_feasibility_envelope(&consumer, 204u,
                                                        &envelope));
    CHECK(envelope.discharge_constant_current_feasible_a
          [DER26_POWER_HORIZON_0P1_S] == 118.0f);
    CHECK(envelope.discharge_constant_current_feasible_a
          [DER26_POWER_HORIZON_10_S] == 70.0f);
    CHECK(envelope.discharge_constant_current_feasible_a
          [DER26_POWER_HORIZON_30_S] == 65.0f);
    CHECK(!envelope.binding_metadata_valid);
    CHECK(!der26_power_consumer_get_resource_state(&consumer, 204u,
                                                    &resource));
    CHECK(der26_power_consumer_get_soh(&consumer, 204u, &soh));
    CHECK(soh.capacity_soh_lower == 0.90f);

    /* Advisory resource and binding metadata are accepted only when their
     * counter matches the active authority bundle. */
    make_strategy(2u, advisory);
    CHECK(der26_power_consumer_ingest(&consumer, DER26_POWER_STRATEGY_ID,
                                      false, false, 8u, advisory, 205u));
    make_bindings(2u, advisory);
    CHECK(der26_power_consumer_ingest(&consumer, DER26_POWER_BINDINGS_ID,
                                      false, false, 8u, advisory, 206u));
    CHECK(der26_power_consumer_get_resource_state(&consumer, 207u,
                                                  &resource));
    CHECK(resource.valid);
    CHECK(resource.counter == 2u);
    CHECK(resource.fuse_authority_valid);
    CHECK(resource.fuse_utilization == 0.76f);
    CHECK(resource.minimum_core_temp_c == 22.0f);
    CHECK(resource.thermal_energy_to_target_wh == 26.4f);

    CHECK(der26_power_consumer_get_feasibility_envelope(&consumer, 207u,
                                                        &envelope));
    CHECK(envelope.binding_metadata_valid);
    CHECK(envelope.discharge_binding[DER26_POWER_HORIZON_0P1_S] == 1u);
    CHECK(envelope.discharge_binding[DER26_POWER_HORIZON_10_S] == 5u);
    CHECK(envelope.discharge_binding[DER26_POWER_HORIZON_30_S] == 14u);
    CHECK(envelope.charge_binding[DER26_POWER_HORIZON_30_S] == 4u);
    CHECK(envelope.charge_limiting_segment
          [DER26_POWER_HORIZON_30_S] == 0x0Fu);

    /* A bad optional frame cannot revoke valid scalar authority. */
    make_strategy(2u, advisory);
    advisory[2] ^= 0x01u;
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_STRATEGY_ID,
                                       false, false, 8u, advisory, 220u));
    CHECK(consumer.advisory_crc_error_count == 1u);
    CHECK(der26_power_consumer_get_immediate_authority(&consumer, 221u,
                                                       &authority));
    CHECK(authority.discharge.current_limit_a == 80.0f);
    CHECK(!der26_power_consumer_get_resource_state(&consumer, 221u,
                                                    &resource));

    /* Unrelated traffic is ignored without disturbing scalar authority. */
    memset(advisory, 0, sizeof(advisory));
    CHECK(!der26_power_consumer_ingest(&consumer, 0x555u, false, false, 8u,
                                       advisory, 222u));
    CHECK(der26_power_consumer_get_immediate_authority(&consumer, 222u,
                                                       &authority));

    /* A syntactically valid but mismatched resource counter is unavailable,
     * not silently reused against a newer authority bundle. */
    make_strategy(3u, advisory);
    CHECK(der26_power_consumer_ingest(&consumer, DER26_POWER_STRATEGY_ID,
                                      false, false, 8u, advisory, 225u));
    CHECK(!der26_power_consumer_get_resource_state(&consumer, 226u,
                                                    &resource));
    CHECK(resource.fuse_utilization == 0.0f);
    CHECK(der26_power_consumer_get_immediate_authority(&consumer, 226u,
                                                       &authority));

    /* Counter equality alone is insufficient after modulo-16 wrap.  Advisory
     * metadata outside the same-cycle skew window is rejected. */
    make_strategy(2u, advisory);
    CHECK(der26_power_consumer_ingest(&consumer, DER26_POWER_STRATEGY_ID,
                                      false, false, 8u, advisory, 260u));
    CHECK(!der26_power_consumer_get_resource_state(&consumer, 261u,
                                                    &resource));

    /* A newer valid zero-authority bundle immediately overrides the old
     * nonzero scalar.  No horizon value is available through this API path. */
    make_zero_bundle(3u, payload);
    ingest_bundle(&consumer, payload, 300u);
    CHECK(der26_power_consumer_get_immediate_authority(&consumer, 304u,
                                                       &authority));
    CHECK(!authority.discharge.authorized);
    CHECK(!authority.charge_regen.authorized);
    CHECK(authority.discharge.current_limit_a == 0.0f);
    CHECK(authority.charge_regen.current_limit_a == 0.0f);

    /* Core corruption invalidates authority and requires requalification. */
    make_bundle(4u, payload, valid_flags, valid_flags);
    payload[0][3] ^= 0x01u;
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID,
                                       false, false, 8u, payload[0], 400u));
    CHECK(consumer.crc_error_count == 1u);
    CHECK(!der26_power_consumer_get_immediate_authority(&consumer, 400u,
                                                        &authority));

    /* Wrong-DLC use of a required ID is also an authority fault. */
    der26_power_consumer_init(&consumer);
    make_bundle(4u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 410u);
    make_bundle(5u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 420u);
    CHECK(der26_power_consumer_get_immediate_authority(&consumer, 424u,
                                                       &authority));
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID, false,
                                       false, 7u, payload[0], 425u));
    CHECK(!der26_power_consumer_get_immediate_authority(&consumer, 425u,
                                                        &authority));

    der26_power_consumer_init(&consumer);
    make_bundle(4u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 500u);
    make_bundle(6u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 600u);
    CHECK(consumer.counter_error_count == 1u);
    CHECK(!der26_power_consumer_get_immediate_authority(&consumer, 604u,
                                                        &authority));
    make_bundle(7u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 700u);
    CHECK(der26_power_consumer_get_immediate_authority(&consumer, 704u,
                                                       &authority));

    /* Direction authorization remains independent. */
    der26_power_consumer_init(&consumer);
    for(uint8_t counter = 8u; counter <= 9u; counter++)
    {
        make_bundle(counter, payload, valid_flags,
                    (uint8_t)(valid_flags |
                              DER26_POWER_FLAG_DIRECTION_INHIBIT));
        put_u16(payload[1], 2u, 0u);
        put_u16(payload[1], 4u, 0u);
        payload[3][4] = 0u;
        payload[3][5] = 0u;
        payload[3][6] = 0u;
        finalize(DER26_POWER_CCL_ID, payload[1]);
        finalize(DER26_POWER_ENVELOPE_ID, payload[3]);
        ingest_bundle(&consumer, payload, 800u + (uint32_t)counter * 10u);
    }
    CHECK(der26_power_consumer_get_immediate_authority(&consumer, 904u,
                                                       &authority));
    CHECK(authority.discharge.authorized);
    CHECK(!authority.charge_regen.authorized);
    CHECK(authority.charge_regen.current_limit_a == 0.0f);

    /* Optional semantic corruption is contained to metadata. */
    make_bindings(9u, advisory);
    advisory[6] = 0x3Eu; /* Segment 14 is invalid. */
    finalize(DER26_POWER_BINDINGS_ID, advisory);
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_BINDINGS_ID,
                                       false, false, 8u, advisory, 910u));
    CHECK(consumer.advisory_semantic_error_count == 1u);
    CHECK(der26_power_consumer_get_immediate_authority(&consumer, 911u,
                                                       &authority));

    /* Staleness zeroes every getter output. */
    CHECK(!der26_power_consumer_get_immediate_authority(&consumer, 1200u,
                                                        &authority));
    CHECK(authority.discharge.current_limit_a == 0.0f);
    CHECK(!der26_power_consumer_get_feasibility_envelope(&consumer, 1200u,
                                                         &envelope));
    CHECK(envelope.discharge_constant_current_feasible_a[0] == 0.0f);

    /* Counter wrap remains valid. */
    der26_power_consumer_init(&consumer);
    make_bundle(15u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 1300u);
    make_bundle(0u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 1400u);
    CHECK(der26_power_consumer_get_immediate_authority(&consumer, 1404u,
                                                       &authority));
    CHECK(authority.counter == 0u);

    puts("ecu_power_consumer_test PASS");
    return 0;
}
