/*
 * ams_flight_recorder.h
 *
 * Portable, allocation-free pre/post-fault recorder for DER26 AMS.
 * Stage A intentionally uses ordinary SRAM. Retained .noinit placement is a
 * later opt-in once RAM/map and torn-write recovery are target validated.
 */

#ifndef INC_DIAGNOSTICS_AMS_FLIGHT_RECORDER_H_
#define INC_DIAGNOSTICS_AMS_FLIGHT_RECORDER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_FLIGHT_RECORDER_SCHEMA_VERSION 1u
#define AMS_FLIGHT_RECORDER_SAMPLE_HZ 20u
#define AMS_FLIGHT_RECORDER_PRE_SAMPLES 100u
#define AMS_FLIGHT_RECORDER_POST_SAMPLES 10u
#define AMS_FLIGHT_RECORDER_CAPACITY \
    (AMS_FLIGHT_RECORDER_PRE_SAMPLES + AMS_FLIGHT_RECORDER_POST_SAMPLES)

#define AMS_FLIGHT_RECORDER_MAGIC 0x414D5346u /* "AMSF" */
#define AMS_FLIGHT_RECORD_COMMIT 0x46524331u  /* "FRC1" */
#define AMS_FLIGHT_HEADER_COMMIT 0x46484431u  /* "FHD1" */

#define AMS_FLIGHT_FLAG_TRIGGERED (1u << 0u)
#define AMS_FLIGHT_FLAG_FROZEN    (1u << 1u)

/* Compact safety-state bits. These are recorder schema, not application fault
 * ownership. Once published they must retain their meaning for schema v1. */
#define AMS_FLIGHT_FAULT_HARD             (1u << 0u)
#define AMS_FLIGHT_FAULT_VOLTAGE          (1u << 1u)
#define AMS_FLIGHT_FAULT_TEMPERATURE      (1u << 2u)
#define AMS_FLIGHT_FAULT_CURRENT          (1u << 3u)
#define AMS_FLIGHT_FAULT_FUSE             (1u << 4u)
#define AMS_FLIGHT_FAULT_IMD              (1u << 5u)
#define AMS_FLIGHT_FAULT_CHARGER          (1u << 6u)
#define AMS_FLIGHT_FAULT_ADBMS            (1u << 7u)
#define AMS_FLIGHT_FAULT_CAN_HARD         (1u << 8u)
#define AMS_FLIGHT_FAULT_HEARTBEAT        (1u << 9u)
#define AMS_FLIGHT_FAULT_RTOS             (1u << 10u)
#define AMS_FLIGHT_FAULT_ESTIMATOR        (1u << 11u)
#define AMS_FLIGHT_FAULT_POWER_LIMIT      (1u << 12u)

#define AMS_FLIGHT_CAN_BUSOFF_ACTIVE      (1u << 0u)
#define AMS_FLIGHT_CAN_RECOVER_PENDING    (1u << 1u)
#define AMS_FLIGHT_CAN_AUTHORITY_READY    (1u << 2u)
#define AMS_FLIGHT_CAN_HARD_LATCH         (1u << 3u)
#define AMS_FLIGHT_CAN_TX_LATCHED         (1u << 4u)
#define AMS_FLIGHT_CAN_AUTH_REFRESH       (1u << 5u)
#define AMS_FLIGHT_CAN_GENERIC_FAULT      (1u << 6u)

/* Event ids are intentionally small and stable. Add; do not renumber. */
typedef enum
{
    AMS_FLIGHT_EVENT_NONE = 0,
    AMS_FLIGHT_EVENT_MANUAL = 1,
    AMS_FLIGHT_EVENT_HARD_FAULT = 2,
    AMS_FLIGHT_EVENT_CAN_BUSOFF_HARD = 3,
    AMS_FLIGHT_EVENT_WATCHDOG_BLOCK = 4,
    AMS_FLIGHT_EVENT_PANIC = 5
} ams_flight_event_t;

/* Exactly 80 bytes. The layout is ordered so every member is naturally
 * aligned on 32-bit ARM/GCC while remaining straightforward to serialize as
 * explicit little-endian fixed-width fields off-target. */
typedef struct
{
    uint32_t sequence;
    uint32_t tick_ms;
    uint32_t measurement_sequence;
    uint32_t pack_voltage_mV;
    int32_t pack_current_mA;
    int32_t discharge_power_limit_W;
    int32_t charge_power_limit_W;
    uint32_t measurement_validity_flags;
    uint32_t safety_fault_mask;
    uint32_t adbms_fault_mask;
    uint32_t can_status_flags;

    uint16_t min_cell_mV;
    uint16_t max_cell_mV;
    uint16_t min_cell_index;
    uint16_t max_cell_index;
    int16_t max_temp_deci_C;
    uint16_t max_temp_index;
    uint16_t soc_permille;
    uint16_t current_uncertainty_mA;
    uint16_t heartbeat_stale_mask;

    uint8_t current_range;
    uint8_t ams_state;
    uint8_t bms_ok;
    uint8_t can_authority;
    uint8_t can_busoff_active;
    uint8_t balancing_active;
    uint8_t estimator_valid;
    uint8_t reserved_u8;
    uint8_t reserved_align[2];

    uint32_t crc32;
    uint32_t commit;
} ams_flight_record_v1_t;

typedef struct
{
    uint32_t magic;
    uint16_t schema_version;
    uint16_t record_size;
    uint32_t boot_sequence;
    uint32_t write_sequence;
    uint32_t write_index;
    uint32_t valid_count;
    uint32_t trigger_sequence;
    uint32_t trigger_tick_ms;
    uint16_t trigger_event;
    uint16_t trigger_reason;
    uint32_t post_trigger_remaining;
    uint32_t runtime_manifest_crc;
    uint32_t flags;
    uint32_t header_crc32;
    uint32_t commit;
} ams_flight_recorder_header_t;

typedef struct
{
    ams_flight_recorder_header_t header;
    ams_flight_record_v1_t record[AMS_FLIGHT_RECORDER_CAPACITY];
} ams_flight_recorder_t;

_Static_assert(sizeof(ams_flight_record_v1_t) == 80u,
               "flight recorder v1 record schema must remain 80 bytes");
_Static_assert(sizeof(ams_flight_recorder_header_t) == 56u,
               "flight recorder header schema must remain 56 bytes");

typedef struct
{
    uint32_t write_sequence;
    uint32_t valid_count;
    uint32_t write_index;
    uint32_t trigger_sequence;
    uint32_t trigger_tick_ms;
    uint32_t post_trigger_remaining;
    uint16_t trigger_event;
    uint16_t trigger_reason;
    bool triggered;
    bool frozen;
} ams_flight_recorder_status_t;

void ams_flight_recorder_init(ams_flight_recorder_t *recorder,
                              uint32_t boot_sequence,
                              uint32_t runtime_manifest_crc);
void ams_flight_recorder_clear(ams_flight_recorder_t *recorder);

/* First trigger wins. This function performs only bounded scalar writes and is
 * suitable for an ISR once the owning integration guarantees single-core
 * atomic 32-bit accesses and, where needed, wraps task-context calls in the
 * platform's short interrupt mask. */
bool ams_flight_recorder_trigger(ams_flight_recorder_t *recorder,
                                 uint16_t event,
                                 uint16_t reason,
                                 uint32_t tick_ms);

/* Sole-writer call. The caller supplies an engineering-state sample; sequence,
 * CRC and commit are owned by the recorder. Returns false only when disabled
 * by caller policy, frozen, or given invalid arguments. */
bool ams_flight_recorder_push(ams_flight_recorder_t *recorder,
                              const ams_flight_record_v1_t *sample);

void ams_flight_recorder_get_status(const ams_flight_recorder_t *recorder,
                                    ams_flight_recorder_status_t *status);

/* Copy records in chronological order. start=0 is the oldest retained record.
 * Returns false for an out-of-range index or invalid/torn record. */
bool ams_flight_recorder_copy_chronological(
    const ams_flight_recorder_t *recorder,
    uint32_t start,
    ams_flight_record_v1_t *record_out);

uint32_t ams_flight_record_crc32(const ams_flight_record_v1_t *record);
bool ams_flight_record_valid(const ams_flight_record_v1_t *record);
uint32_t ams_flight_recorder_header_crc32(
    const ams_flight_recorder_header_t *header);
bool ams_flight_recorder_header_valid(
    const ams_flight_recorder_header_t *header);

#ifdef __cplusplus
}
#endif

#endif /* INC_DIAGNOSTICS_AMS_FLIGHT_RECORDER_H_ */
