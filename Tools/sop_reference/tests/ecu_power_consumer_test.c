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

static void make_bundle(uint8_t counter, uint8_t payload[4][8],
                        uint8_t dcl_flags, uint8_t ccl_flags)
{
    memset(payload, 0, 4u * 8u);
    for(uint8_t slot = 0u; slot < 4u; slot++)
    {
        payload[slot][0] = (uint8_t)((DER26_POWER_PROTOCOL_VERSION << 4u) |
                                     counter);
    }
    payload[0][1] = dcl_flags;
    put_u16(payload[0], 2u, 800u);
    put_u16(payload[0], 4u, 2200u);
    payload[0][6] = 0xE2u; /* mission-profile binding */
    payload[1][1] = ccl_flags;
    put_u16(payload[1], 2u, 100u);
    put_u16(payload[1], 4u, 300u);
    payload[1][6] = 0xD3u; /* fuse-thermal binding */
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
    for(uint8_t slot = 0u; slot < 4u; slot++)
    {
        payload[slot][7] = der26_power_crc8(
            (uint16_t)(DER26_POWER_DCL_ID + slot), payload[slot]);
    }
}

static void ingest_bundle(der26_power_consumer_t *consumer,
                          uint8_t payload[4][8], uint32_t now_ms)
{
    for(uint8_t slot = 0u; slot < 4u; slot++)
    {
        CHECK(der26_power_consumer_ingest(
            consumer, (uint16_t)(DER26_POWER_DCL_ID + slot), false, false, 8u,
            payload[slot], now_ms + slot));
    }
}

int main(void)
{
    der26_power_consumer_t consumer;
    der26_power_limits_t limits;
    uint8_t payload[4][8];
    const uint8_t valid_flags = DER26_POWER_FLAG_VALID |
                                DER26_POWER_FLAG_AUTHORITY_VALID;
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

    make_bundle(1u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 100u);
    CHECK(!der26_power_consumer_get(&consumer, 104u, &limits));
    make_bundle(2u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 200u);
    CHECK(der26_power_consumer_get(&consumer, 204u, &limits));
    CHECK(limits.discharge_valid && limits.charge_valid);
    CHECK(limits.discharge_current_limit_a == 80.0f);
    CHECK(limits.charge_current_limit_a == 10.0f);
    CHECK(limits.discharge_power_limit_w == 22000.0f);
    CHECK(limits.capacity_soh_lower == 0.90f);
    CHECK(limits.discharge_envelope_a[2] == 65.0f);
    CHECK(!der26_power_consumer_get(&consumer, 454u, &limits));

    make_bundle(3u, payload, valid_flags, valid_flags);
    payload[0][3] ^= 0x01u;
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID,
                                       false, false, 8u, payload[0], 300u));
    CHECK(consumer.crc_error_count == 1u);
    CHECK(!der26_power_consumer_get(&consumer, 300u, &limits));

    der26_power_consumer_init(&consumer);
    make_bundle(4u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 400u);
    make_bundle(6u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 500u);
    CHECK(consumer.counter_error_count == 1u);
    CHECK(!der26_power_consumer_get(&consumer, 504u, &limits));
    make_bundle(7u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 600u);
    CHECK(der26_power_consumer_get(&consumer, 604u, &limits));

    der26_power_consumer_init(&consumer);
    make_bundle(8u, payload,
                valid_flags,
                (uint8_t)(valid_flags | DER26_POWER_FLAG_DIRECTION_INHIBIT));
    put_u16(payload[1], 2u, 0u);
    put_u16(payload[1], 4u, 0u);
    payload[3][4] = 0u;
    payload[3][5] = 0u;
    payload[3][6] = 0u;
    payload[1][7] = der26_power_crc8(DER26_POWER_CCL_ID, payload[1]);
    payload[3][7] = der26_power_crc8(DER26_POWER_ENVELOPE_ID, payload[3]);
    ingest_bundle(&consumer, payload, 700u);
    make_bundle(9u, payload,
                valid_flags,
                (uint8_t)(valid_flags | DER26_POWER_FLAG_DIRECTION_INHIBIT));
    put_u16(payload[1], 2u, 0u);
    put_u16(payload[1], 4u, 0u);
    payload[3][4] = 0u;
    payload[3][5] = 0u;
    payload[3][6] = 0u;
    payload[1][7] = der26_power_crc8(DER26_POWER_CCL_ID, payload[1]);
    payload[3][7] = der26_power_crc8(DER26_POWER_ENVELOPE_ID, payload[3]);
    ingest_bundle(&consumer, payload, 800u);
    CHECK(der26_power_consumer_get(&consumer, 804u, &limits));
    CHECK(limits.discharge_valid && !limits.charge_valid);
    CHECK(limits.discharge_current_limit_a == 80.0f);
    CHECK(limits.charge_current_limit_a == 0.0f);

    der26_power_consumer_init(&consumer);
    make_bundle(10u, payload, valid_flags, valid_flags);
    CHECK(der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID,
                                      false, false, 8u, payload[0], 900u));
    CHECK(!der26_power_consumer_get(&consumer, 951u, &limits));

    der26_power_consumer_init(&consumer);
    make_bundle(11u, payload, valid_flags, valid_flags);
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID,
                                       false, true, 8u, payload[0], 1000u));
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID,
                                       true, false, 8u, payload[0], 1001u));
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID,
                                       false, false, 7u, payload[0], 1002u));
    CHECK(consumer.malformed_count == 3u);

    der26_power_consumer_init(&consumer);
    make_bundle(12u, payload, valid_flags, valid_flags);
    put_u16(payload[0], 2u, 2000u);
    payload[0][7] = der26_power_crc8(DER26_POWER_DCL_ID, payload[0]);
    CHECK(!der26_power_consumer_ingest(&consumer, DER26_POWER_DCL_ID,
                                       false, false, 8u, payload[0], 1050u));
    CHECK(consumer.semantic_error_count == 1u);

    der26_power_consumer_init(&consumer);
    make_bundle(13u, payload, valid_flags, valid_flags);
    payload[3][2] = 90u; /* 10 s above the 80 A 1 s value. */
    payload[3][7] = der26_power_crc8(DER26_POWER_ENVELOPE_ID, payload[3]);
    for(uint8_t slot = 0u; slot < 3u; slot++)
    {
        CHECK(der26_power_consumer_ingest(
            &consumer, (uint16_t)(DER26_POWER_DCL_ID + slot), false,
            false, 8u, payload[slot], 1070u + slot));
    }
    CHECK(!der26_power_consumer_ingest(&consumer,
                                       DER26_POWER_ENVELOPE_ID, false,
                                       false, 8u, payload[3], 1073u));
    CHECK(consumer.semantic_error_count == 1u);

    der26_power_consumer_init(&consumer);
    make_bundle(15u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 1100u);
    make_bundle(0u, payload, valid_flags, valid_flags);
    ingest_bundle(&consumer, payload, 1200u);
    CHECK(der26_power_consumer_get(&consumer, 1204u, &limits));
    CHECK(limits.counter == 0u);

    puts("ecu_power_consumer_test PASS");
    return 0;
}
