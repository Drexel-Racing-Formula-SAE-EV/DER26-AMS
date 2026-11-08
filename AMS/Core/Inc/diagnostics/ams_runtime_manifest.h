/*
 * ams_runtime_manifest.h
 *
 * Compact, fixed-width runtime identity for DER26 incident evidence.  This is
 * diagnostic provenance only: it never grants measurement or BMS authority.
 */

#ifndef INC_DIAGNOSTICS_AMS_RUNTIME_MANIFEST_H_
#define INC_DIAGNOSTICS_AMS_RUNTIME_MANIFEST_H_

#include <stdbool.h>
#include <stdint.h>

#include "ext_drivers/current_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_RUNTIME_MANIFEST_MAGIC 0x414D524Du /* "AMRM" */
#define AMS_RUNTIME_MANIFEST_SCHEMA 1u

/* firmware_version = major[23:16] | minor[15:8] | patch[7:0]. */
typedef struct
{
    uint32_t magic;
    uint16_t schema_version;
    uint16_t size;

    uint32_t firmware_version;
    uint32_t source_identity_crc;
    uint32_t build_profile;
    uint32_t build_config_fingerprint;
    uint32_t build_feature_flags;

    uint32_t current_calibration_id;
    uint32_t current_calibration_crc;
    uint32_t estimator_model_crc;
    uint32_t sop_model_crc;
    uint32_t soh_model_crc;
    uint32_t fuse_model_crc;
    uint32_t thresholds_crc;
    uint32_t can_contract_crc;
    uint32_t topology_crc;

    uint16_t nsmbs;
    uint16_t ncells;
    uint16_t ntemps;
    uint16_t thermistor_model_revision;

    uint32_t manifest_crc;
} ams_runtime_manifest_t;

_Static_assert(sizeof(ams_runtime_manifest_t) == 76u,
               "runtime manifest schema must remain fixed-width");

void ams_runtime_manifest_build(ams_runtime_manifest_t *manifest,
                                const current_sensor_t *current_sensor);
uint32_t ams_runtime_manifest_crc32(const ams_runtime_manifest_t *manifest);
bool ams_runtime_manifest_valid(const ams_runtime_manifest_t *manifest);

/* Exposed for regression tests and service-time identity refresh.  The
 * fingerprint represents the calibration actually applied to the runtime
 * sensor object; it does not imply that the calibration is authoritative. */
uint32_t ams_runtime_manifest_current_calibration_crc(
    const current_sensor_t *current_sensor);

struct app_data_t;
/* Rebuild runtime identity from the live application configuration.  When
 * reset_flight_recorder is true, start a new evidence epoch bound to the new
 * manifest CRC while preserving the current boot sequence. */
void ams_runtime_identity_refresh(struct app_data_t *data,
                                  bool reset_flight_recorder);

#ifdef __cplusplus
}
#endif

#endif /* INC_DIAGNOSTICS_AMS_RUNTIME_MANIFEST_H_ */
