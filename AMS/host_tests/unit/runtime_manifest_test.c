#include <stdio.h>
#include <string.h>

#include "diagnostics/ams_runtime_manifest.h"
#include "ams_version.h"
#include "ext_drivers/current_sensor.h"

static unsigned failures;
#define CHECK(expr) do { if(!(expr)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    failures++; } } while(0)

int main(void)
{
    current_sensor_t sensor;
    ams_runtime_manifest_t manifest;
    ams_runtime_manifest_t changed;
    uint32_t base_cal_crc;

    memset(&sensor, 0, sizeof(sensor));
    sensor.adc_vref_v = 3.3f;
    sensor.sensor_supply_v = 5.0f;
    ams_runtime_manifest_build(&manifest, &sensor);

    CHECK(ams_runtime_manifest_valid(&manifest));
    CHECK(manifest.magic == AMS_RUNTIME_MANIFEST_MAGIC);
    CHECK(manifest.schema_version == AMS_RUNTIME_MANIFEST_SCHEMA);
    CHECK(manifest.firmware_version ==
          (((uint32_t)AMS_VERSION_MAJOR << 16u) |
           ((uint32_t)AMS_VERSION_MINOR << 8u) |
           (uint32_t)AMS_VERSION_PATCH));
    CHECK(manifest.nsmbs > 0u);
    CHECK(manifest.ncells > 0u);
    CHECK(manifest.ntemps > 0u);
    CHECK(manifest.estimator_model_crc != 0u);
    CHECK(manifest.sop_model_crc != 0u);
    CHECK(manifest.soh_model_crc != 0u);
    CHECK(manifest.fuse_model_crc != 0u);
    CHECK(manifest.thresholds_crc != 0u);
    CHECK(manifest.can_contract_crc != 0u);
    CHECK(manifest.topology_crc != 0u);

    base_cal_crc = manifest.current_calibration_crc;
    sensor.zero_calibrated = true;
    sensor.zero_cal_count = 1u;
    sensor.zero_offset_50a = 0.125f;
    sensor.zero_offset_800a = -0.25f;
    sensor.calibration_id = 42u;
    sensor.calibration_loaded_from_record = true;
    sensor.calibration_uncertainty_50a_mA = 100u;
    sensor.calibration_uncertainty_800a_mA = 500u;
    ams_runtime_manifest_build(&changed, &sensor);
    CHECK(ams_runtime_manifest_valid(&changed));
    CHECK(changed.current_calibration_id == 42u);
    CHECK(changed.current_calibration_crc != base_cal_crc);
    CHECK(changed.manifest_crc != manifest.manifest_crc);

    changed.manifest_crc ^= 1u;
    CHECK(!ams_runtime_manifest_valid(&changed));

    if(failures != 0u)
    {
        return 1;
    }
    puts("PASS runtime manifest tests");
    return 0;
}
