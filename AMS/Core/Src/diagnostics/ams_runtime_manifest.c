#include "diagnostics/ams_runtime_manifest.h"

#include <stddef.h>
#include <string.h>

#include "ams_build_profile.h"
#include "ams_version.h"
#include "app.h"
#include "ext_drivers/accumulator.h"
#include "ext_drivers/thermistor_model.h"
#include "sop/ams_fuse_observer.h"

static uint32_t runtime_crc32_byte(uint32_t crc, uint8_t value)
{
    crc ^= (uint32_t)value;
    for(uint8_t bit = 0u; bit < 8u; bit++)
    {
        uint32_t mask = (uint32_t)(-(int32_t)(crc & 1u));
        crc = (crc >> 1u) ^ (0xEDB88320u & mask);
    }
    return crc;
}

static uint32_t runtime_crc32_u16(uint32_t crc, uint16_t value)
{
    crc = runtime_crc32_byte(crc, (uint8_t)(value & 0xFFu));
    return runtime_crc32_byte(crc, (uint8_t)(value >> 8u));
}

static uint32_t runtime_crc32_u32(uint32_t crc, uint32_t value)
{
    for(uint8_t shift = 0u; shift < 32u; shift += 8u)
    {
        crc = runtime_crc32_byte(crc, (uint8_t)(value >> shift));
    }
    return crc;
}

static uint32_t runtime_crc32_i16(uint32_t crc, int16_t value)
{
    return runtime_crc32_u16(crc, (uint16_t)value);
}

static uint32_t runtime_crc32_float(uint32_t crc, float value)
{
    uint32_t bits = 0u;
    _Static_assert(sizeof(bits) == sizeof(value),
                   "runtime manifest assumes 32-bit float");
    memcpy(&bits, &value, sizeof(bits));
    return runtime_crc32_u32(crc, bits);
}

static uint32_t runtime_crc32_string_update(uint32_t crc, const char *text)
{
    if(text == NULL)
    {
        return runtime_crc32_byte(crc, 0u);
    }

    while(*text != '\0')
    {
        crc = runtime_crc32_byte(crc, (uint8_t)*text);
        text++;
    }
    /* Preserve string boundaries when multiple fields are hashed. */
    return runtime_crc32_byte(crc, 0u);
}

static uint32_t runtime_string_crc32(const char *text)
{
    return ~runtime_crc32_string_update(0xFFFFFFFFu, text);
}

uint32_t ams_runtime_manifest_current_calibration_crc(
    const current_sensor_t *current_sensor)
{
    uint32_t crc = 0xFFFFFFFFu;

    if(current_sensor == NULL)
    {
        return 0u;
    }

    crc = runtime_crc32_u32(crc, current_sensor->calibration_id);
    crc = runtime_crc32_u32(crc, current_sensor->calibration_capture_time_s);
    crc = runtime_crc32_i16(crc, current_sensor->calibration_temp_deci_c);
    crc = runtime_crc32_u16(crc,
                            current_sensor->calibration_uncertainty_50a_mA);
    crc = runtime_crc32_u16(crc,
                            current_sensor->calibration_uncertainty_800a_mA);
    crc = runtime_crc32_byte(
        crc, current_sensor->calibration_loaded_from_record ? 1u : 0u);
    crc = runtime_crc32_byte(crc, current_sensor->zero_calibrated ? 1u : 0u);
    crc = runtime_crc32_u32(crc, current_sensor->zero_cal_count);
    crc = runtime_crc32_float(crc, current_sensor->zero_offset_50a);
    crc = runtime_crc32_float(crc, current_sensor->zero_offset_800a);
    crc = runtime_crc32_float(crc, current_sensor->adc_vref_v);
    crc = runtime_crc32_float(crc, current_sensor->sensor_supply_v);
    return ~crc;
}

static uint32_t runtime_source_identity_crc(void)
{
    uint32_t crc = 0xFFFFFFFFu;
    crc = runtime_crc32_string_update(crc, AMS_SOURCE_REVISION);
    crc = runtime_crc32_string_update(crc, AMS_BUILD_GIT_COMMIT);
    crc = runtime_crc32_u32(crc, AMS_BUILD_CONFIG_FINGERPRINT);
    return ~crc;
}

static uint32_t runtime_topology_crc(void)
{
    uint32_t crc = 0xFFFFFFFFu;
    crc = runtime_crc32_u16(crc, (uint16_t)NSMBS);
    crc = runtime_crc32_u16(crc, (uint16_t)NCELLS);
    crc = runtime_crc32_u16(crc, (uint16_t)NTEMPS);
    crc = runtime_crc32_u16(crc, (uint16_t)ACCUMULATOR_PHYSICAL_CHAIN_COUNT);
    crc = runtime_crc32_u16(crc, (uint16_t)THERMISTOR_MODEL_REVISION);
    return ~crc;
}

uint32_t ams_runtime_manifest_crc32(const ams_runtime_manifest_t *manifest)
{
    uint32_t crc = 0xFFFFFFFFu;

    if(manifest == NULL)
    {
        return 0u;
    }

#define CRC_U32(member) crc = runtime_crc32_u32(crc, manifest->member)
#define CRC_U16(member) crc = runtime_crc32_u16(crc, manifest->member)
    CRC_U32(magic);
    CRC_U16(schema_version);
    CRC_U16(size);
    CRC_U32(firmware_version);
    CRC_U32(source_identity_crc);
    CRC_U32(build_profile);
    CRC_U32(build_config_fingerprint);
    CRC_U32(build_feature_flags);
    CRC_U32(current_calibration_id);
    CRC_U32(current_calibration_crc);
    CRC_U32(estimator_model_crc);
    CRC_U32(sop_model_crc);
    CRC_U32(soh_model_crc);
    CRC_U32(fuse_model_crc);
    CRC_U32(thresholds_crc);
    CRC_U32(can_contract_crc);
    CRC_U32(topology_crc);
    CRC_U16(nsmbs);
    CRC_U16(ncells);
    CRC_U16(ntemps);
    CRC_U16(thermistor_model_revision);
#undef CRC_U32
#undef CRC_U16

    return ~crc;
}

void ams_runtime_manifest_build(ams_runtime_manifest_t *manifest,
                                const current_sensor_t *current_sensor)
{
    if(manifest == NULL)
    {
        return;
    }

    memset(manifest, 0, sizeof(*manifest));
    manifest->magic = AMS_RUNTIME_MANIFEST_MAGIC;
    manifest->schema_version = AMS_RUNTIME_MANIFEST_SCHEMA;
    manifest->size = (uint16_t)sizeof(*manifest);
    manifest->firmware_version =
        ((uint32_t)AMS_VERSION_MAJOR << 16u) |
        ((uint32_t)AMS_VERSION_MINOR << 8u) |
        (uint32_t)AMS_VERSION_PATCH;
    manifest->source_identity_crc = runtime_source_identity_crc();
    manifest->build_profile = (uint32_t)AMS_BUILD_PROFILE;
    manifest->build_config_fingerprint = AMS_BUILD_CONFIG_FINGERPRINT;
    manifest->build_feature_flags = (uint32_t)AMS_BUILD_FEATURE_FLAGS_VALUE;

    if(current_sensor != NULL)
    {
        manifest->current_calibration_id = current_sensor->calibration_id;
        manifest->current_calibration_crc =
            ams_runtime_manifest_current_calibration_crc(current_sensor);
    }

    manifest->estimator_model_crc =
        runtime_string_crc32(AMS_ESTIMATOR_MODEL_REVISION);
    manifest->sop_model_crc = runtime_string_crc32(AMS_SOP_MODEL_REVISION);
    manifest->soh_model_crc = runtime_string_crc32(AMS_SOH_MODEL_REVISION);
    manifest->fuse_model_crc = runtime_string_crc32(AMS_FUSE_MODEL_REVISION);
    manifest->thresholds_crc = runtime_string_crc32(AMS_THRESHOLD_REVISION);
    manifest->can_contract_crc = runtime_string_crc32(AMS_CAN_CONTRACT_REVISION);
    manifest->topology_crc = runtime_topology_crc();
    manifest->nsmbs = (uint16_t)NSMBS;
    manifest->ncells = (uint16_t)NCELLS;
    manifest->ntemps = (uint16_t)NTEMPS;
    manifest->thermistor_model_revision = (uint16_t)THERMISTOR_MODEL_REVISION;
    manifest->manifest_crc = ams_runtime_manifest_crc32(manifest);
}

bool ams_runtime_manifest_valid(const ams_runtime_manifest_t *manifest)
{
    return (manifest != NULL) &&
           (manifest->magic == AMS_RUNTIME_MANIFEST_MAGIC) &&
           (manifest->schema_version == AMS_RUNTIME_MANIFEST_SCHEMA) &&
           (manifest->size == sizeof(*manifest)) &&
           (manifest->manifest_crc == ams_runtime_manifest_crc32(manifest));
}

void ams_runtime_identity_refresh(struct app_data_t *data,
                                  bool reset_flight_recorder)
{
    uint32_t boot_sequence = 1u;

    if(data == NULL)
    {
        return;
    }

    ams_runtime_manifest_build(&data->runtime_manifest,
                               &data->board.current_sensor);

#if AMS_ENABLE_FLIGHT_RECORDER
    if(!reset_flight_recorder)
    {
        return;
    }

    if(data->flight_recorder.header.boot_sequence != 0u)
    {
        boot_sequence = data->flight_recorder.header.boot_sequence;
    }

    /* Identity changes intentionally start a new evidence epoch. Mixing
     * samples created under different calibration/configuration identities
     * behind one recorder-header manifest CRC would be ambiguous. */
    ams_flight_recorder_init(&data->flight_recorder,
                             boot_sequence,
                             data->runtime_manifest.manifest_crc);
#else
    (void)reset_flight_recorder;
#endif
}
