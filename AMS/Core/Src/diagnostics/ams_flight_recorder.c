#include "diagnostics/ams_flight_recorder.h"

#include <stddef.h>
#include <string.h>

static uint32_t flight_crc32_bytes(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;

    if(data == NULL)
    {
        return 0u;
    }

    for(size_t i = 0u; i < length; i++)
    {
        crc ^= (uint32_t)data[i];
        for(uint8_t bit = 0u; bit < 8u; bit++)
        {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1u));
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }

    return ~crc;
}

uint32_t ams_flight_record_crc32(const ams_flight_record_v1_t *record)
{
    if(record == NULL)
    {
        return 0u;
    }

    return flight_crc32_bytes((const uint8_t *)record,
                              offsetof(ams_flight_record_v1_t, crc32));
}

bool ams_flight_record_valid(const ams_flight_record_v1_t *record)
{
    return (record != NULL) &&
           (record->commit == AMS_FLIGHT_RECORD_COMMIT) &&
           (record->crc32 == ams_flight_record_crc32(record));
}

uint32_t ams_flight_recorder_header_crc32(
    const ams_flight_recorder_header_t *header)
{
    if(header == NULL)
    {
        return 0u;
    }

    return flight_crc32_bytes((const uint8_t *)header,
                              offsetof(ams_flight_recorder_header_t,
                                       header_crc32));
}

bool ams_flight_recorder_header_valid(
    const ams_flight_recorder_header_t *header)
{
    return (header != NULL) &&
           (header->magic == AMS_FLIGHT_RECORDER_MAGIC) &&
           (header->schema_version == AMS_FLIGHT_RECORDER_SCHEMA_VERSION) &&
           (header->record_size == sizeof(ams_flight_record_v1_t)) &&
           (header->commit == AMS_FLIGHT_HEADER_COMMIT) &&
           (header->header_crc32 == ams_flight_recorder_header_crc32(header));
}

static void flight_header_commit(ams_flight_recorder_header_t *header)
{
    if(header == NULL)
    {
        return;
    }

    header->commit = 0u;
    header->header_crc32 = ams_flight_recorder_header_crc32(header);
    header->commit = AMS_FLIGHT_HEADER_COMMIT;
}

void ams_flight_recorder_init(ams_flight_recorder_t *recorder,
                              uint32_t boot_sequence,
                              uint32_t runtime_manifest_crc)
{
    if(recorder == NULL)
    {
        return;
    }

    memset(recorder, 0, sizeof(*recorder));
    recorder->header.magic = AMS_FLIGHT_RECORDER_MAGIC;
    recorder->header.schema_version = AMS_FLIGHT_RECORDER_SCHEMA_VERSION;
    recorder->header.record_size = (uint16_t)sizeof(ams_flight_record_v1_t);
    recorder->header.boot_sequence = boot_sequence;
    recorder->header.runtime_manifest_crc = runtime_manifest_crc;
    flight_header_commit(&recorder->header);
}

void ams_flight_recorder_clear(ams_flight_recorder_t *recorder)
{
    uint32_t boot_sequence;
    uint32_t manifest_crc;

    if(recorder == NULL)
    {
        return;
    }

    boot_sequence = recorder->header.boot_sequence;
    manifest_crc = recorder->header.runtime_manifest_crc;
    ams_flight_recorder_init(recorder, boot_sequence, manifest_crc);
}

bool ams_flight_recorder_trigger(ams_flight_recorder_t *recorder,
                                 uint16_t event,
                                 uint16_t reason,
                                 uint32_t tick_ms)
{
    if((recorder == NULL) ||
       ((recorder->header.flags & AMS_FLIGHT_FLAG_TRIGGERED) != 0u))
    {
        return false;
    }

    recorder->header.trigger_sequence = recorder->header.write_sequence;
    recorder->header.trigger_tick_ms = tick_ms;
    recorder->header.trigger_event = event;
    recorder->header.trigger_reason = reason;
    recorder->header.post_trigger_remaining = AMS_FLIGHT_RECORDER_POST_SAMPLES;
    recorder->header.flags |= AMS_FLIGHT_FLAG_TRIGGERED;
    flight_header_commit(&recorder->header);
    return true;
}

bool ams_flight_recorder_push(ams_flight_recorder_t *recorder,
                              const ams_flight_record_v1_t *sample)
{
    ams_flight_record_v1_t *dst;

    if((recorder == NULL) || (sample == NULL) ||
       ((recorder->header.flags & AMS_FLIGHT_FLAG_FROZEN) != 0u))
    {
        return false;
    }

    dst = &recorder->record[recorder->header.write_index];
    *dst = *sample;
    dst->sequence = recorder->header.write_sequence;
    dst->commit = 0u;
    dst->crc32 = ams_flight_record_crc32(dst);
    dst->commit = AMS_FLIGHT_RECORD_COMMIT;

    recorder->header.write_sequence++;
    recorder->header.write_index++;
    if(recorder->header.write_index >= AMS_FLIGHT_RECORDER_CAPACITY)
    {
        recorder->header.write_index = 0u;
    }
    if(recorder->header.valid_count < AMS_FLIGHT_RECORDER_CAPACITY)
    {
        recorder->header.valid_count++;
    }

    if((recorder->header.flags & AMS_FLIGHT_FLAG_TRIGGERED) != 0u)
    {
        if(recorder->header.post_trigger_remaining > 0u)
        {
            recorder->header.post_trigger_remaining--;
        }
        if(recorder->header.post_trigger_remaining == 0u)
        {
            recorder->header.flags |= AMS_FLIGHT_FLAG_FROZEN;
        }
    }

    flight_header_commit(&recorder->header);
    return true;
}

void ams_flight_recorder_get_status(const ams_flight_recorder_t *recorder,
                                    ams_flight_recorder_status_t *status)
{
    if(status == NULL)
    {
        return;
    }

    memset(status, 0, sizeof(*status));
    if(recorder == NULL)
    {
        return;
    }

    status->write_sequence = recorder->header.write_sequence;
    status->valid_count = recorder->header.valid_count;
    status->write_index = recorder->header.write_index;
    status->trigger_sequence = recorder->header.trigger_sequence;
    status->trigger_tick_ms = recorder->header.trigger_tick_ms;
    status->post_trigger_remaining = recorder->header.post_trigger_remaining;
    status->trigger_event = recorder->header.trigger_event;
    status->trigger_reason = recorder->header.trigger_reason;
    status->triggered =
        (recorder->header.flags & AMS_FLIGHT_FLAG_TRIGGERED) != 0u;
    status->frozen =
        (recorder->header.flags & AMS_FLIGHT_FLAG_FROZEN) != 0u;
}

bool ams_flight_recorder_copy_chronological(
    const ams_flight_recorder_t *recorder,
    uint32_t start,
    ams_flight_record_v1_t *record_out)
{
    uint32_t oldest_index;
    uint32_t index;

    if((recorder == NULL) || (record_out == NULL) ||
       (start >= recorder->header.valid_count) ||
       (recorder->header.valid_count > AMS_FLIGHT_RECORDER_CAPACITY))
    {
        return false;
    }

    if(recorder->header.valid_count < AMS_FLIGHT_RECORDER_CAPACITY)
    {
        oldest_index = 0u;
    }
    else
    {
        oldest_index = recorder->header.write_index;
    }

    index = (oldest_index + start) % AMS_FLIGHT_RECORDER_CAPACITY;
    *record_out = recorder->record[index];
    return ams_flight_record_valid(record_out);
}
