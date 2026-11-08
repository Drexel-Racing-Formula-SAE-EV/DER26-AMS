#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "diagnostics/ams_flight_recorder.h"

static unsigned failures;

#define CHECK(expr) do { \
    if(!(expr)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while(0)

static ams_flight_record_v1_t sample(uint32_t tick, int32_t current_mA)
{
    ams_flight_record_v1_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.tick_ms = tick;
    rec.measurement_sequence = tick / 50u;
    rec.pack_voltage_mV = 300000u + tick;
    rec.pack_current_mA = current_mA;
    rec.min_cell_mV = 3900u;
    rec.max_cell_mV = 4100u;
    rec.min_cell_index = 1u;
    rec.max_cell_index = 74u;
    rec.max_temp_deci_C = 425;
    rec.max_temp_index = 12u;
    rec.soc_permille = 750u;
    rec.current_uncertainty_mA = 250u;
    rec.ams_state = 3u;
    rec.bms_ok = 1u;
    rec.can_authority = 1u;
    rec.estimator_valid = 1u;
    return rec;
}

static void test_init_and_crc(void)
{
    ams_flight_recorder_t recorder;
    ams_flight_record_v1_t rec = sample(50u, 1234);
    ams_flight_record_v1_t out;

    ams_flight_recorder_init(&recorder, 7u, 0x12345678u);
    CHECK(ams_flight_recorder_header_valid(&recorder.header));
    CHECK(recorder.header.boot_sequence == 7u);
    CHECK(recorder.header.runtime_manifest_crc == 0x12345678u);
    CHECK(recorder.header.valid_count == 0u);
    CHECK(!ams_flight_recorder_copy_chronological(&recorder, 0u, &out));

    CHECK(ams_flight_recorder_push(&recorder, &rec));
    CHECK(recorder.header.valid_count == 1u);
    CHECK(ams_flight_recorder_copy_chronological(&recorder, 0u, &out));
    CHECK(out.sequence == 0u);
    CHECK(out.tick_ms == 50u);
    CHECK(out.pack_current_mA == 1234);
    CHECK(ams_flight_record_valid(&out));

    recorder.record[0].pack_current_mA ^= 1;
    CHECK(!ams_flight_record_valid(&recorder.record[0]));
    recorder.header.valid_count ^= 1u;
    CHECK(!ams_flight_recorder_header_valid(&recorder.header));
}

static void test_ring_wrap_chronology(void)
{
    ams_flight_recorder_t recorder;
    ams_flight_record_v1_t rec;
    ams_flight_record_v1_t out;
    const uint32_t total = AMS_FLIGHT_RECORDER_CAPACITY + 17u;

    ams_flight_recorder_init(&recorder, 1u, 0u);
    for(uint32_t i = 0u; i < total; i++)
    {
        rec = sample(i * 50u, (int32_t)i);
        CHECK(ams_flight_recorder_push(&recorder, &rec));
    }

    CHECK(recorder.header.valid_count == AMS_FLIGHT_RECORDER_CAPACITY);
    CHECK(recorder.header.write_sequence == total);
    for(uint32_t i = 0u; i < AMS_FLIGHT_RECORDER_CAPACITY; i++)
    {
        CHECK(ams_flight_recorder_copy_chronological(&recorder, i, &out));
        CHECK(out.sequence == (total - AMS_FLIGHT_RECORDER_CAPACITY + i));
        CHECK(out.pack_current_mA ==
              (int32_t)(total - AMS_FLIGHT_RECORDER_CAPACITY + i));
    }
}

static void test_first_trigger_post_window_freeze(void)
{
    ams_flight_recorder_t recorder;
    ams_flight_record_v1_t rec;
    ams_flight_record_v1_t out;
    ams_flight_recorder_status_t status;

    ams_flight_recorder_init(&recorder, 1u, 0u);

    /* Fill a complete rolling buffer before the event. The post-trigger ten
     * records must replace only the oldest ten, preserving exactly 100 pre
     * records plus 10 post records. */
    for(uint32_t i = 0u; i < AMS_FLIGHT_RECORDER_CAPACITY; i++)
    {
        rec = sample(i * 50u, (int32_t)i);
        CHECK(ams_flight_recorder_push(&recorder, &rec));
    }

    CHECK(ams_flight_recorder_trigger(&recorder,
                                      AMS_FLIGHT_EVENT_MANUAL,
                                      0x55AAu,
                                      5500u));
    CHECK(!ams_flight_recorder_trigger(&recorder,
                                       AMS_FLIGHT_EVENT_HARD_FAULT,
                                       0xAAAAu,
                                       5550u));

    for(uint32_t i = 0u; i < AMS_FLIGHT_RECORDER_POST_SAMPLES; i++)
    {
        rec = sample(5550u + i * 50u, 1000 + (int32_t)i);
        CHECK(ams_flight_recorder_push(&recorder, &rec));
    }

    ams_flight_recorder_get_status(&recorder, &status);
    CHECK(status.triggered);
    CHECK(status.frozen);
    CHECK(status.trigger_sequence == AMS_FLIGHT_RECORDER_CAPACITY);
    CHECK(status.trigger_tick_ms == 5500u);
    CHECK(status.trigger_event == AMS_FLIGHT_EVENT_MANUAL);
    CHECK(status.trigger_reason == 0x55AAu);
    CHECK(status.post_trigger_remaining == 0u);
    CHECK(status.valid_count == AMS_FLIGHT_RECORDER_CAPACITY);

    CHECK(ams_flight_recorder_copy_chronological(&recorder, 0u, &out));
    CHECK(out.sequence == AMS_FLIGHT_RECORDER_POST_SAMPLES);
    CHECK(ams_flight_recorder_copy_chronological(
        &recorder, AMS_FLIGHT_RECORDER_PRE_SAMPLES - 1u, &out));
    CHECK(out.sequence == AMS_FLIGHT_RECORDER_CAPACITY - 1u);
    CHECK(ams_flight_recorder_copy_chronological(
        &recorder, AMS_FLIGHT_RECORDER_PRE_SAMPLES, &out));
    CHECK(out.sequence == AMS_FLIGHT_RECORDER_CAPACITY);
    CHECK(out.pack_current_mA == 1000);
    CHECK(ams_flight_recorder_copy_chronological(
        &recorder, AMS_FLIGHT_RECORDER_CAPACITY - 1u, &out));
    CHECK(out.sequence == AMS_FLIGHT_RECORDER_CAPACITY +
                          AMS_FLIGHT_RECORDER_POST_SAMPLES - 1u);

    rec = sample(9999u, 9999);
    CHECK(!ams_flight_recorder_push(&recorder, &rec));
}

static void test_clear_rearms(void)
{
    ams_flight_recorder_t recorder;
    ams_flight_record_v1_t rec = sample(1u, 1);
    ams_flight_recorder_status_t status;

    ams_flight_recorder_init(&recorder, 9u, 0xABCDEF12u);
    CHECK(ams_flight_recorder_trigger(&recorder,
                                      AMS_FLIGHT_EVENT_HARD_FAULT,
                                      1u, 10u));
    for(uint32_t i = 0u; i < AMS_FLIGHT_RECORDER_POST_SAMPLES; i++)
    {
        CHECK(ams_flight_recorder_push(&recorder, &rec));
    }
    CHECK(!ams_flight_recorder_trigger(&recorder,
                                       AMS_FLIGHT_EVENT_MANUAL, 0u, 20u));

    ams_flight_recorder_clear(&recorder);
    ams_flight_recorder_get_status(&recorder, &status);
    CHECK(!status.triggered);
    CHECK(!status.frozen);
    CHECK(status.valid_count == 0u);
    CHECK(recorder.header.boot_sequence == 9u);
    CHECK(recorder.header.runtime_manifest_crc == 0xABCDEF12u);
    CHECK(ams_flight_recorder_header_valid(&recorder.header));
    CHECK(ams_flight_recorder_trigger(&recorder,
                                      AMS_FLIGHT_EVENT_MANUAL, 2u, 30u));
}

static void test_sequence_wrap(void)
{
    ams_flight_recorder_t recorder;
    ams_flight_record_v1_t rec = sample(UINT32_MAX - 1u, 1);
    ams_flight_record_v1_t out;

    ams_flight_recorder_init(&recorder, 1u, 0u);
    recorder.header.write_sequence = UINT32_MAX;
    /* Keep header self-consistency out of this internal wrap test; push owns
     * record sequence and restores header CRC afterward. */
    CHECK(ams_flight_recorder_push(&recorder, &rec));
    CHECK(recorder.header.write_sequence == 0u);
    CHECK(ams_flight_recorder_copy_chronological(&recorder, 0u, &out));
    CHECK(out.sequence == UINT32_MAX);
    CHECK(ams_flight_recorder_header_valid(&recorder.header));
}

int main(void)
{
    test_init_and_crc();
    test_ring_wrap_chronology();
    test_first_trigger_post_window_freeze();
    test_clear_rearms();
    test_sequence_wrap();

    if(failures != 0u)
    {
        fprintf(stderr, "FAIL %u flight recorder checks\n", failures);
        return 1;
    }

    printf("PASS flight recorder core: wrap/trigger/post-window/CRC/clear/sequence-wrap\n");
    return 0;
}
