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

static void be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)((v >> 24u) & 0xFFu);
    p[1] = (uint8_t)((v >> 16u) & 0xFFu);
    p[2] = (uint8_t)((v >> 8u) & 0xFFu);
    p[3] = (uint8_t)(v & 0xFFu);
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


static void test_can_diag_frame(void)
{
    ams_dash_state_t s;
    ams_dash_state_init(&s);

    const uint8_t can_diag[8] = {
        0x00u, 0x00u, 0x00u, 0x04u, 0x02u, 0x09u, 0x01u, 0x03u
    };
    ams_can_frame_t cf = frame(AMS_LOGGER_CAN_ID_CAN_DIAG, can_diag);
    CHECK(ams_dash_decode_frame(&s, &cf, 70u));
    CHECK(s.can_error_code == 0x00000004u);
    CHECK(s.can_busoff_count == 2u);
    CHECK(s.can_error_count == 9u);
    CHECK(s.can_recover_count == 1u);
    CHECK(s.can_diag_flags == 0x03u);
    CHECK(s.logger_frames == 1u);
    CHECK(s.unknown_frames == 0u);
}

static void test_extended_diagnostic_frames(void)
{
    ams_dash_state_t s;
    ams_dash_state_init(&s);

    uint8_t safety[8] = { 0u };
    be32(&safety[0], 0xA5A55A5Au);
    safety[4] = 6u;
    safety[5] = 2u;
    safety[6] = 9u;
    safety[7] = 0x5Bu;
    ams_can_frame_t sf = frame(AMS_LOGGER_CAN_ID_SAFETY_DIAG, safety);
    CHECK(ams_dash_decode_frame(&s, &sf, 80u));
    CHECK(s.safety_reset_flags == 0xA5A55A5Au);
    CHECK(s.safety_last_panic_reason == 6u);
    CHECK(s.safety_panic_count == 2u);
    CHECK(s.safety_bms_block_count == 9u);
    CHECK(s.safety_flags == 0x5Bu);

    uint8_t wdg[8] = { 0x07u, 0x08u, 0x00u, 0x2Au, 0x00u, 0x03u, 0x01u, 0xF4u };
    ams_can_frame_t wf = frame(AMS_LOGGER_CAN_ID_WATCHDOG_DIAG, wdg);
    CHECK(ams_dash_decode_frame(&s, &wf, 90u));
    CHECK(s.watchdog_flags == 0x07u);
    CHECK(s.watchdog_last_block_reason == 8u);
    CHECK(s.watchdog_feed_count == 42u);
    CHECK(s.watchdog_block_count == 3u);
    CHECK(s.watchdog_last_feed_age_ds == 500u);

    uint8_t adbms[8] = { 0x01u, 0x23u, 0x04u, 0x05u, 0x06u, 0x03u, 0x3Fu, 0x07u };
    ams_can_frame_t af = frame(AMS_LOGGER_CAN_ID_ADBMS_DIAG, adbms);
    CHECK(ams_dash_decode_frame(&s, &af, 100u));
    CHECK(s.adbms_scan_count == 0x0123u);
    CHECK(s.adbms_status_diag_count == 4u);
    CHECK(s.adbms_config_diag_count == 5u);
    CHECK(s.adbms_open_wire_diag_count == 6u);
    CHECK(s.adbms_last_diag_status == 3u);
    CHECK(s.adbms_diag_flags == 0x3Fu);
    CHECK(s.adbms_hil_flags == 0x07u);

    uint8_t adc[8] = { 0x0Au, 0xBCu, 0x01u, 0x23u, 0x02u, 0x06u, 0x3Fu, 0x02u };
    ams_can_frame_t adcf = frame(AMS_LOGGER_CAN_ID_CURRENT_ADC, adc);
    CHECK(ams_dash_decode_frame(&s, &adcf, 105u));
    CHECK(s.current_adc_high_count == 0x0ABCu);
    CHECK(s.current_adc_low_count == 0x0123u);
    CHECK(s.current_selected_range == 2u);
    CHECK(s.current_meas_reason == 6u);
    CHECK(s.current_adc_flags == 0x3Fu);
    CHECK(s.current_zero_cal_count == 2u);

    uint8_t charger[8] = { 0xFFu, 0x9Cu, 0x04u, 0x00u, 0x03u, 0x09u, 0x0Au, 0x02u };
    ams_can_frame_t chf = frame(AMS_LOGGER_CAN_ID_CHARGER_DETAIL, charger);
    CHECK(ams_dash_decode_frame(&s, &chf, 106u));
    CHECK(s.charger_read_current_dA == -100);
    CHECK(s.charger_disable_reason_mask == 0x0400u);
    CHECK(s.charger_last_tx_status == 3u);
    CHECK(s.charger_tx_count == 9u);
    CHECK(s.charger_rx_count == 10u);
    CHECK(s.charger_tx_fail_count == 2u);
}

static void test_estimator_and_hil_frames(void)
{
    ams_dash_state_t s;
    ams_dash_state_init(&s);

    uint8_t est[8] = { 2u, 0xA5u, 0x26u, 0xF2u, 0xFFu, 0x9Cu, 0x04u, 0xD2u };
    ams_can_frame_t ef = frame(AMS_DASH_CAN_ID_ESTIMATOR_STATUS, est);
    CHECK(ams_dash_decode_frame(&s, &ef, 110u));
    CHECK(s.estimator_active_index == 2u);
    CHECK(s.estimator_flags == 0xA5u);
    CHECK(s.estimator_soc_centi_pct == 9970u);
    CHECK(s.estimator_innovation_mV == -100);
    CHECK(s.estimator_r0_0p01_mohm == 1234u);
    CHECK(s.estimator_last_rx_ms == 110u);

    uint8_t meas[8] = { 0x79u, 0x18u, 0xFBu, 0x2Eu, 0x09u, 0xC4u, 0x11u, 0x00u };
    ams_can_frame_t mf = frame(AMS_DASH_CAN_ID_HIL_MEAS, meas);
    CHECK(ams_dash_decode_frame(&s, &mf, 120u));
    CHECK(s.hil_pack_voltage_cV == 31000u);
    CHECK(s.hil_current_cA == -1234);
    CHECK(s.hil_surface_temp_cC == 2500);
    CHECK((s.hil_flags & 0x01u) != 0u);

    uint8_t truth[8] = { 0x22u, 0xB8u, 0x0Au, 0x28u, 0x12u, 0x00u, 0x10u, 0x20u };
    ams_can_frame_t tf = frame(AMS_DASH_CAN_ID_HIL_TRUTH, truth);
    CHECK(ams_dash_decode_frame(&s, &tf, 130u));
    CHECK(s.hil_soc_centi_pct == 8888u);
    CHECK(s.hil_core_temp_cC == 2600);
    CHECK(s.hil_plant_step == 0x001020u);
    CHECK((s.hil_flags & 0x02u) != 0u);

    uint8_t summary[8] = { 0x0Cu, 0x81u, 0x0Fu, 0xA0u, 0x11u, 0x94u, 0x0Bu, 0xB8u };
    ams_can_frame_t hf = frame(AMS_DASH_CAN_ID_HIL_SUMMARY, summary);
    CHECK(ams_dash_decode_frame(&s, &hf, 140u));
    CHECK(s.hil_min_cell_mv == 3201u);
    CHECK(s.hil_max_cell_mv == 4000u);
    CHECK(s.hil_max_temp_cC == 4500);
    CHECK(s.hil_avg_temp_cC == 3000);
    CHECK((s.hil_flags & 0x04u) != 0u);
}

static void test_rejects_non_contract_frames(void)
{
    ams_dash_state_t s;
    ams_dash_state_init(&s);

    const uint8_t data[8] = { 0u };
    ams_can_frame_t ext = frame(AMS_LOGGER_CAN_ID_HEARTBEAT, data);
    ext.extended = true;
    CHECK(!ams_dash_decode_frame(&s, &ext, 10u));
    CHECK(s.ignored_frames == 1u);
    CHECK(s.unknown_frames == 0u);

    ams_can_frame_t short_frame = frame(AMS_LOGGER_CAN_ID_HEARTBEAT, data);
    short_frame.dlc = 7u;
    CHECK(!ams_dash_decode_frame(&s, &short_frame, 20u));
    CHECK(s.malformed_frames == 1u);

    ams_can_frame_t ignored = frame(AMS_DASH_CAN_ID_ECU_AMS, data);
    CHECK(!ams_dash_decode_frame(&s, &ignored, 25u));
    CHECK(s.ignored_frames == 2u);

    ams_can_frame_t unknown = frame(0x123u, data);
    CHECK(!ams_dash_decode_frame(&s, &unknown, 30u));
    CHECK(s.rx_frames == 4u);
    CHECK(s.unknown_frames == 1u);
    CHECK(s.logger_frames == 0u);
}

int main(void)
{
    test_summary_frames();
    test_detail_and_masks();
    test_task_health_frame();
    test_can_diag_frame();
    test_extended_diagnostic_frames();
    test_estimator_and_hil_frames();
    test_rejects_non_contract_frames();
    puts("decoder_smoke_test PASS");
    return 0;
}
