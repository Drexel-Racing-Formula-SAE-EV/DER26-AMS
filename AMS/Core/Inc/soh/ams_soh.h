/*
 * Capacity and resistance State-of-Health estimator for DER26.
 *
 * Capacity is learned only from calibrated coulomb throughput between two
 * independently stable rest anchors with sufficient SOC excursion. Resistance
 * health aggregates the DADEKF outer-loop R0 observations. No heuristic ageing
 * drift is applied when the pack is unobservable.
 */

#ifndef INC_SOH_AMS_SOH_H_
#define INC_SOH_AMS_SOH_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_SOH_SEGMENTS 5u
#define AMS_SOH_PERSIST_MAGIC 0x534F4833u /* "SOH3" */
#define AMS_SOH_PERSIST_SCHEMA 3u
#define AMS_SOH_ALL_SEGMENTS_MASK ((1u << AMS_SOH_SEGMENTS) - 1u)

/* Resistance ageing is deliberately slow state.  Fresh R0 estimates often
 * arrive as a correlated burst during one excitation episode.  Do not commit
 * the leading edge of that burst as permanent ageing: collect a bounded episode
 * and only evaluate its robust median after qualified R0 observations have
 * stopped for a short gap. */
#define AMS_SOH_RESISTANCE_EPISODE_MIN_OBSERVATIONS 9u
#define AMS_SOH_RESISTANCE_EPISODE_MAX_OBSERVATIONS 33u
#define AMS_SOH_RESISTANCE_EPISODE_GAP_MS 2500u

#define AMS_SOH_PERSIST_CAPACITY_VALID   (1u << 0u)
#define AMS_SOH_PERSIST_RESISTANCE_VALID (1u << 1u)

#define AMS_SOH_REASON_NONE                  0x00000000u
#define AMS_SOH_REASON_BAD_ARGUMENT          0x00000001u
#define AMS_SOH_REASON_STALE                 0x00000002u
#define AMS_SOH_REASON_MEASUREMENT           0x00000004u
#define AMS_SOH_REASON_ESTIMATOR             0x00000008u
#define AMS_SOH_REASON_CURRENT_CALIBRATION   0x00000010u
#define AMS_SOH_REASON_CURRENT_POLARITY      0x00000020u
#define AMS_SOH_REASON_BALANCING             0x00000040u
#define AMS_SOH_REASON_NOT_RESTED             0x00000080u
#define AMS_SOH_REASON_TEMPERATURE            0x00000100u
#define AMS_SOH_REASON_CELL_SPREAD            0x00000200u
#define AMS_SOH_REASON_SOC_EXCURSION          0x00000400u
#define AMS_SOH_REASON_THROUGHPUT              0x00000800u
#define AMS_SOH_REASON_DIRECTION               0x00001000u
#define AMS_SOH_REASON_CAPACITY_RANGE          0x00002000u
#define AMS_SOH_REASON_CAPACITY_OUTLIER        0x00004000u
#define AMS_SOH_REASON_RESISTANCE_INCOMPLETE   0x00008000u
#define AMS_SOH_REASON_NUMERIC                 0x00010000u
#define AMS_SOH_REASON_PERSISTENCE             0x00020000u
#define AMS_SOH_REASON_SOC_UNCERTAINTY          0x00040000u
#define AMS_SOH_REASON_REST_INNOVATION          0x00080000u
#define AMS_SOH_REASON_REST_POLARIZATION        0x00100000u

typedef struct
{
    float nominal_pack_capacity_ah;
    float rest_current_max_a;
    float required_rest_time_s;
    float minimum_soc_excursion;
    float minimum_throughput_ah;
    float minimum_temperature_c;
    float maximum_temperature_c;
    float maximum_cell_spread_v;
    float maximum_soc_sigma;
    float maximum_rest_innovation_v_per_cell;
    float maximum_rest_polarization_v;
    float minimum_capacity_soh;
    float maximum_capacity_soh;
    float maximum_relative_outlier;
    float capacity_sigma_floor_ah;
    float confidence_sigma_multiplier;
    float prior_capacity_soh_lower;
    float prior_resistance_soh_upper;
    float resistance_uncertainty_floor;
    uint32_t maximum_measurement_age_ms;
    uint8_t minimum_capacity_observations;
    uint8_t minimum_resistance_confidence_pct;
} ams_soh_config_t;

typedef struct
{
    uint32_t measurement_sequence;
    uint32_t measurement_timestamp_ms;
    uint32_t now_ms;
    float elapsed_s;
    float pack_current_a;
    float pack_current_uncertainty_a;
    double total_charge_as;
    float pack_soc;
    float average_cell_temp_c;
    float cell_voltage_spread_v;
    float maximum_soc_sigma;
    float maximum_abs_innovation_v_per_cell;
    float maximum_abs_polarization_v;
    float segment_resistance_growth_ratio[AMS_SOH_SEGMENTS];
    uint8_t segment_resistance_confidence_pct[AMS_SOH_SEGMENTS];
    uint8_t segment_resistance_valid[AMS_SOH_SEGMENTS];
    uint8_t measurement_valid;
    uint8_t estimator_valid;
    uint8_t current_calibrated;
    uint8_t current_polarity_validated;
    uint8_t balance_recovered;
} ams_soh_input_t;

typedef struct
{
    uint32_t accepted_capacity_windows;
    uint32_t rejected_capacity_windows;
    uint32_t last_reason_flags;
    uint32_t last_update_tick;
    uint32_t last_accept_tick;
    float capacity_ah;
    float capacity_sigma_ah;
    float capacity_soh;
    float capacity_soh_lower;
    float resistance_growth_ratio;
    float resistance_growth_upper;
    float combined_soh;
    uint8_t capacity_confidence_pct;
    uint8_t resistance_confidence_pct;
    uint8_t capacity_valid;
    uint8_t resistance_valid;
    uint8_t persistence_valid;
} ams_soh_result_t;

typedef struct
{
    ams_soh_result_t result;
    double capacity_mean_ah;
    double capacity_m2_ah2;
    float segment_resistance_growth_ratio[AMS_SOH_SEGMENTS];
    float segment_resistance_growth_upper[AMS_SOH_SEGMENTS];
    uint8_t segment_resistance_confidence_pct[AMS_SOH_SEGMENTS];
    uint8_t segment_resistance_valid_mask;
    float segment_resistance_episode_ratio[AMS_SOH_SEGMENTS][AMS_SOH_RESISTANCE_EPISODE_MAX_OBSERVATIONS];
    uint32_t segment_resistance_episode_last_fresh_ms[AMS_SOH_SEGMENTS];
    uint8_t segment_resistance_episode_count[AMS_SOH_SEGMENTS];
    uint8_t segment_resistance_episode_write_index[AMS_SOH_SEGMENTS];
    uint8_t segment_resistance_episode_min_confidence[AMS_SOH_SEGMENTS];
    float rest_elapsed_s;
    float anchor_soc;
    float anchor_temp_c;
    double anchor_total_charge_as;
    uint32_t anchor_tick;
    uint8_t anchor_valid;
    uint8_t rest_latched;
} ams_soh_estimator_t;

typedef struct
{
    uint32_t magic;
    uint16_t schema;
    uint16_t size;
    uint32_t generation;
    uint32_t accepted_capacity_windows;
    double capacity_mean_ah;
    double capacity_m2_ah2;
    float segment_resistance_growth_upper[AMS_SOH_SEGMENTS];
    uint8_t segment_resistance_confidence_pct[AMS_SOH_SEGMENTS];
    uint8_t segment_resistance_valid_mask;
    uint8_t validity_flags;
    uint8_t reserved[1];
    uint32_t crc32;
} ams_soh_persist_record_t;

void ams_soh_default_config(ams_soh_config_t *cfg);
bool ams_soh_config_valid(const ams_soh_config_t *cfg);
void ams_soh_init(ams_soh_estimator_t *estimator,
                  const ams_soh_config_t *cfg);
bool ams_soh_update(ams_soh_estimator_t *estimator,
                    const ams_soh_config_t *cfg,
                    const ams_soh_input_t *input);
bool ams_soh_export_record(const ams_soh_estimator_t *estimator,
                           uint32_t generation,
                           ams_soh_persist_record_t *record);
bool ams_soh_import_record(ams_soh_estimator_t *estimator,
                           const ams_soh_config_t *cfg,
                           const ams_soh_persist_record_t *record);
bool ams_soh_select_newest_record(const ams_soh_config_t *cfg,
                                  const ams_soh_persist_record_t *first,
                                  const ams_soh_persist_record_t *second,
                                  ams_soh_persist_record_t *selected);
uint32_t ams_soh_record_crc32(const ams_soh_persist_record_t *record);

#ifdef __cplusplus
}
#endif

#endif /* INC_SOH_AMS_SOH_H_ */
