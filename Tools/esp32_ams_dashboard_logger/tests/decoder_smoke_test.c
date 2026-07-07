#include "ams_can_decode.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expr)                                                        \
    do {                                                                   \
        if(!(expr)) {                                                       \
            fprintf(stderr, "CHECK failed at %s:%d: %s\n",                 \
                    __FILE__, __LINE__, #expr);                            \
            exit(1);                                                       \
        }                                                                  \
    } while(0)

static void be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8u);
    p[1] = (uint8_t)(v & 0xFFu);
}

static ams_can_frame_t frame(uint32_t id, const uint8_t data[8])
{
    ams_can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    f.dlc = 8u;
    memcpy(f.data, data, 8u);
    return f;
}

static void test_summary_frames(void)
{
    ams_dash_state_t s;
    ams_dash_state_init(&s);

    const uint8_t heartbeat[8] = {
        AMS_LOGGER_PROTOCOL_VERSION, 42u, 3u, 0x19u, 0x07u, 0x00u, 0x04u, 0xD2u
    };
    ams_can_frame_t hb = frame(AMS_LOGGER_CAN_ID_HEARTBEAT, heartbeat);
    CHECK(ams_dash_decode_frame(&s, &hb, 100u));
    CHECK(s.protocol_version == AMS_LOGGER_PROTOCOL_VERSION);
    CHECK(s.sequence == 42u);
    CHECK(s.status_flags == 0x19u);
    CHECK(s.validity_flags == 0x07u);
    CHECK(s.ams_uptime_s == 1234u);
    CHECK(!ams_dash_data_stale(&s, 1500u, 1500u));
    CHECK(ams_dash_data_stale(&s, 1601u, 1500u));

    uint8_t pack[8] = { 0u };
    be16(&pack[0], 3124u);
    be16(&pack[2], (uint16_t)-123);
    be16(&pack[4], 3201u);
    be16(&pack[6], 4099u);
    ams_can_frame_t pf = frame(AMS_LOGGER_CAN_ID_PACK_ELECTRICAL, pack);
    CHECK(ams_dash_decode_frame(&s, &pf, 120u));
    CHECK(s.pack_voltage_dV == 3124u);
    CHECK(s.current_dA == -123);
    CHECK(s.min_cell_mv == 3201u);
    CHECK(s.max_cell_mv == 4099u);

    uint8_t temp[8] = { 0u };
    be16(&temp[0], 444u);
    be16(&temp[2], 250u);
    be16(&temp[4], 300u);
    temp[6] = 75u;
    temp[7] = 0xA5u;
    ams_can_frame_t tf = frame(AMS_LOGGER_CAN_ID_TEMP_FAN, temp);
    CHECK(ams_dash_decode_frame(&s, &tf, 140u));
    CHECK(s.max_temp_dC == 444);
    CHECK(s.min_temp_dC == 250);
    CHECK(s.avg_temp_dC == 300);
    CHECK(s.max_fan_percent == 75u);
    CHECK(s.temp_flags == 0xA5u);
}

static void test_detail_and_masks(void)
{
    ams_dash_state_t s;
    ams_dash_state_init(&s);

    for(uint8_t seg = 0u; seg < AMS_DASH_SEGMENTS; seg++)
    {
        for(uint8_t sensor = 0u; sensor < AMS_DASH_TEMPS_PER_SEG; sensor++)
        {
            CHECK(s.temp_dC[seg][sensor] == AMS_DASH_TEMP_INVALID);
        }
    }

    uint8_t cells[8] = { 4u, 12u, 0u, 0u, 0u, 0u, 0u, 0u };
    be16(&cells[2], 4012u);
    be16(&cells[4], 4013u);
    be16(&cells[6], 4014u);
    ams_can_frame_t cf = frame(AMS_LOGGER_CAN_ID_CELL_DETAIL, cells);
    CHECK(ams_dash_decode_frame(&s, &cf, 10u));
    CHECK(s.cell_mv[4][12] == 4012u);
    CHECK(s.cell_mv[4][13] == 4013u);
    CHECK(s.cell_mv[4][14] == 4014u);

    uint8_t temps[8] = { 2u, 3u, 0u, 0u, 0u, 0u, 0u, 0u };
    be16(&temps[2], 251u);
    be16(&temps[4], (uint16_t)AMS_DASH_TEMP_INVALID);
    be16(&temps[6], 253u);
    ams_can_frame_t tdf = frame(AMS_LOGGER_CAN_ID_TEMP_DETAIL, temps);
    CHECK(ams_dash_decode_frame(&s, &tdf, 20u));
    CHECK(s.temp_dC[2][3] == 251);
    CHECK(s.temp_dC[2][4] == AMS_DASH_TEMP_INVALID);
    CHECK(s.temp_dC[2][5] == 253);

    const uint8_t mask_a[8] = { 2u, 0x00u, 0xFFu, 0x00u, 0x00u, 0x0Fu, 0x0Fu, 0u };
    ams_can_frame_t maf = frame(AMS_LOGGER_CAN_ID_TEMP_MASKS_A, mask_a);
    CHECK(ams_dash_decode_frame(&s, &maf, 30u));
    CHECK(s.temp_updated_mask[2] == 0x0000FF00u);
    CHECK(s.temp_usable_mask[2] == 0x00000F0Fu);

    const uint8_t mask_b[8] = { 2u, 0x00u, 0x00u, 0x20u, 0x00u, 0x00u, 0x10u, 0u };
    ams_can_frame_t mbf = frame(AMS_LOGGER_CAN_ID_TEMP_MASKS_B, mask_b);
    CHECK(ams_dash_decode_frame(&s, &mbf, 40u));
    CHECK(s.temp_stale_mask[2] == 0x00000020u);
    CHECK(s.temp_invalid_mask[2] == 0x00000010u);

    const uint8_t pec[8] = { 3u, 0x01u, 0x01u, 7u, 0u, 0u, 0u, 0u };
    ams_can_frame_t vf = frame(AMS_LOGGER_CAN_ID_VOLTAGE_PEC, pec);
    CHECK(ams_dash_decode_frame(&s, &vf, 50u));
    CHECK(s.voltage_pec_mask[3] == 0x0101u);
}

static void test_task_health_frame(void)
{
    ams_dash_state_t s;
    ams_dash_state_init(&s);

    const uint8_t task_health[8] = {
        0x12u, 0x34u, 0x00u, 0x1Fu, 0x00u, 0x05u, 0x03u, 0x77u
    };
    ams_can_frame_t th = frame(AMS_LOGGER_CAN_ID_TASK_HEALTH, task_health);
    CHECK(ams_dash_decode_frame(&s, &th, 60u));
    CHECK(s.heartbeat_stale_mask == 0x1234u);
    CHECK(s.heartbeat_seen_mask == 0x001Fu);
    CHECK(s.heartbeat_safety_stale_mask == 0x0005u);
    CHECK(s.task_health_flags == 0x03u);
    CHECK(s.logger_heartbeat_count == 0x77u);
    CHECK(s.logger_frames == 1u);
    CHECK(s.unknown_frames == 0u);
}

static void test_rejects_non_contract_frames(void)
{
    ams_dash_state_t s;
    ams_dash_state_init(&s);

    const uint8_t data[8] = { 0u };
    ams_can_frame_t ext = frame(AMS_LOGGER_CAN_ID_HEARTBEAT, data);
    ext.extended = true;
    CHECK(!ams_dash_decode_frame(&s, &ext, 10u));
    CHECK(s.unknown_frames == 1u);

    ams_can_frame_t short_frame = frame(AMS_LOGGER_CAN_ID_HEARTBEAT, data);
    short_frame.dlc = 7u;
    CHECK(!ams_dash_decode_frame(&s, &short_frame, 20u));
    CHECK(s.unknown_frames == 2u);

    ams_can_frame_t unknown = frame(0x123u, data);
    CHECK(!ams_dash_decode_frame(&s, &unknown, 30u));
    CHECK(s.rx_frames == 1u);
    CHECK(s.unknown_frames == 3u);
    CHECK(s.logger_frames == 0u);
}

int main(void)
{
    test_summary_frames();
    test_detail_and_masks();
    test_task_health_frame();
    test_rejects_non_contract_frames();
    puts("decoder_smoke_test PASS");
    return 0;
}
