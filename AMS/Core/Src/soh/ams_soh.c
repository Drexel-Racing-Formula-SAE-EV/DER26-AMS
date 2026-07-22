#include "soh/ams_soh.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(ams_soh_persist_record_t) == 64u,
               "SoH persistence schema-3 size changed");
_Static_assert(offsetof(ams_soh_persist_record_t, crc32) == 60u,
               "SoH persistence schema-3 CRC offset changed");

static float clampf_local(float value, float lower, float upper)
{
    if(value < lower)
    {
        return lower;
    }
    if(value > upper)
    {
        return upper;
    }
    return value;
}

static uint32_t saturating_increment(uint32_t value)
{
    return (value == UINT32_MAX) ? UINT32_MAX : value + 1u;
}

static bool finite_positive(float value)
{
    return isfinite(value) && (value > 0.0f);
}

void ams_soh_default_config(ams_soh_config_t *cfg)
{
    if(cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->nominal_pack_capacity_ah = 25.2f; /* 4.2 Ah x 6p */
    cfg->rest_current_max_a = 0.50f;
    cfg->required_rest_time_s = 60.0f;
    cfg->minimum_soc_excursion = 0.15f;
    cfg->minimum_throughput_ah = 3.0f;
    cfg->minimum_temperature_c = 10.0f;
    cfg->maximum_temperature_c = 40.0f;
    cfg->maximum_cell_spread_v = 0.050f;
    cfg->maximum_soc_sigma = 0.015f;
    cfg->maximum_rest_innovation_v_per_cell = 0.015f;
    cfg->maximum_rest_polarization_v = 0.020f;
    cfg->minimum_capacity_soh = 0.50f;
    cfg->maximum_capacity_soh = 1.10f;
    cfg->maximum_relative_outlier = 0.20f;
    cfg->capacity_sigma_floor_ah = 0.50f;
    cfg->confidence_sigma_multiplier = 3.0f;
    cfg->prior_capacity_soh_lower = 0.80f;
    cfg->prior_resistance_soh_upper = 1.25f;
    cfg->resistance_uncertainty_floor = 0.05f;
    cfg->maximum_measurement_age_ms = 250u;
    cfg->minimum_capacity_observations = 2u;
    cfg->minimum_resistance_confidence_pct = 50u;
}

bool ams_soh_config_valid(const ams_soh_config_t *cfg)
{
    return (cfg != NULL) &&
        finite_positive(cfg->nominal_pack_capacity_ah) &&
        finite_positive(cfg->rest_current_max_a) &&
        finite_positive(cfg->required_rest_time_s) &&
        finite_positive(cfg->minimum_soc_excursion) &&
        (cfg->minimum_soc_excursion < 1.0f) &&
        finite_positive(cfg->minimum_throughput_ah) &&
        isfinite(cfg->minimum_temperature_c) &&
        isfinite(cfg->maximum_temperature_c) &&
        (cfg->minimum_temperature_c < cfg->maximum_temperature_c) &&
        finite_positive(cfg->maximum_cell_spread_v) &&
        finite_positive(cfg->maximum_soc_sigma) &&
        finite_positive(cfg->maximum_rest_innovation_v_per_cell) &&
        finite_positive(cfg->maximum_rest_polarization_v) &&
        isfinite(cfg->minimum_capacity_soh) &&
        isfinite(cfg->maximum_capacity_soh) &&
        (cfg->minimum_capacity_soh > 0.0f) &&
        (cfg->minimum_capacity_soh < cfg->maximum_capacity_soh) &&
        finite_positive(cfg->maximum_relative_outlier) &&
        finite_positive(cfg->capacity_sigma_floor_ah) &&
        finite_positive(cfg->confidence_sigma_multiplier) &&
        isfinite(cfg->prior_capacity_soh_lower) &&
        (cfg->prior_capacity_soh_lower >= cfg->minimum_capacity_soh) &&
        (cfg->prior_capacity_soh_lower <= 1.0f) &&
        isfinite(cfg->prior_resistance_soh_upper) &&
        (cfg->prior_resistance_soh_upper >= 1.0f) &&
        finite_positive(cfg->resistance_uncertainty_floor) &&
        (cfg->maximum_measurement_age_ms > 0u) &&
        (cfg->minimum_capacity_observations > 0u) &&
        (cfg->minimum_resistance_confidence_pct <= 100u);
}

static void refresh_capacity_result(ams_soh_estimator_t *estimator,
                                    const ams_soh_config_t *cfg)
{
    ams_soh_result_t *result = &estimator->result;
    const uint32_t count = result->accepted_capacity_windows;
    float sigma_ah = cfg->capacity_sigma_floor_ah;

    if(count > 1u)
    {
        const double sample_variance = estimator->capacity_m2_ah2 /
                                       (double)(count - 1u);
        if(isfinite(sample_variance) && (sample_variance >= 0.0))
        {
            const float standard_error =
                (float)sqrt(sample_variance / (double)count);
            if(standard_error > sigma_ah)
            {
                sigma_ah = standard_error;
            }
        }
    }

    result->capacity_ah = (count > 0u) ?
        (float)estimator->capacity_mean_ah : cfg->nominal_pack_capacity_ah;
    result->capacity_sigma_ah = sigma_ah;
    result->capacity_soh = clampf_local(
        result->capacity_ah / cfg->nominal_pack_capacity_ah,
        cfg->minimum_capacity_soh, cfg->maximum_capacity_soh);
    result->capacity_valid =
        (count >= (uint32_t)cfg->minimum_capacity_observations) ? 1u : 0u;

    if(result->capacity_valid != 0u)
    {
        const float lower_ah = result->capacity_ah -
            cfg->confidence_sigma_multiplier * sigma_ah;
        result->capacity_soh_lower = clampf_local(
            lower_ah / cfg->nominal_pack_capacity_ah,
            cfg->minimum_capacity_soh, 1.0f);
    }
    else
    {
        result->capacity_soh_lower = cfg->prior_capacity_soh_lower;
    }

    uint32_t confidence = count * 25u;
    if(confidence > 100u)
    {
        confidence = 100u;
    }
    result->capacity_confidence_pct = (uint8_t)confidence;
}

static void refresh_resistance_summary(ams_soh_estimator_t *estimator,
                                       const ams_soh_config_t *cfg)
{
    float worst_ratio = 1.0f;
    float worst_upper = 1.0f;
    uint8_t minimum_confidence = 100u;
    uint8_t valid_count = 0u;

    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        const uint8_t bit = (uint8_t)(1u << segment);
        const bool valid =
            (estimator->segment_resistance_valid_mask & bit) != 0u;
        const float ratio = valid ?
            estimator->segment_resistance_growth_ratio[segment] : 1.0f;
        const float upper = valid ?
            estimator->segment_resistance_growth_upper[segment] :
            cfg->prior_resistance_soh_upper;
        const uint8_t confidence = valid ?
            estimator->segment_resistance_confidence_pct[segment] : 0u;
        if(ratio > worst_ratio)
        {
            worst_ratio = ratio;
        }
        if(upper > worst_upper)
        {
            worst_upper = upper;
        }
        if(valid)
        {
            if(confidence < minimum_confidence)
            {
                minimum_confidence = confidence;
            }
            valid_count++;
        }
    }

    estimator->result.resistance_valid =
        (estimator->segment_resistance_valid_mask ==
         AMS_SOH_ALL_SEGMENTS_MASK) ? 1u : 0u;
    estimator->result.resistance_growth_ratio = worst_ratio;
    estimator->result.resistance_growth_upper = clampf_local(
        worst_upper, 1.0f, 3.0f);
    estimator->result.resistance_confidence_pct = (valid_count > 0u) ?
        minimum_confidence : 0u;
}

static void refresh_resistance_result(ams_soh_estimator_t *estimator,
                                      const ams_soh_config_t *cfg,
                                      const ams_soh_input_t *input)
{
    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        const float ratio = input->segment_resistance_growth_ratio[segment];
        const uint8_t confidence =
            input->segment_resistance_confidence_pct[segment];
        if((input->segment_resistance_valid[segment] == 0u) ||
           !isfinite(ratio) || (ratio < 0.50f) || (ratio > 3.0f) ||
           (confidence < cfg->minimum_resistance_confidence_pct))
        {
            continue;
        }

        const float uncertainty = cfg->resistance_uncertainty_floor +
            (0.20f * (100.0f - (float)confidence) / 100.0f);
        const float upper = clampf_local(ratio + uncertainty, 1.0f, 3.0f);
        const uint8_t bit = (uint8_t)(1u << segment);
        const bool already_valid =
            (estimator->segment_resistance_valid_mask & bit) != 0u;

        /* Normalized cell resistance is treated as an ageing state. A lower
         * later observation cannot erase a previously qualified conservative
         * bound; a battery/segment replacement requires an explicit service
         * reset and new calibration record. */
        if(!already_valid ||
           (upper > estimator->segment_resistance_growth_upper[segment]))
        {
            estimator->segment_resistance_growth_ratio[segment] = ratio;
            estimator->segment_resistance_growth_upper[segment] = upper;
        }
        if(!already_valid ||
           (confidence >
            estimator->segment_resistance_confidence_pct[segment]))
        {
            estimator->segment_resistance_confidence_pct[segment] = confidence;
        }
        estimator->segment_resistance_valid_mask |= bit;
    }

    refresh_resistance_summary(estimator, cfg);
    if(estimator->result.resistance_valid == 0u)
    {
        estimator->result.last_reason_flags |=
            AMS_SOH_REASON_RESISTANCE_INCOMPLETE;
    }
}

static void refresh_combined_result(ams_soh_estimator_t *estimator)
{
    const float resistance_health =
        (estimator->result.resistance_growth_upper > 0.0f) ?
        (1.0f / estimator->result.resistance_growth_upper) : 0.0f;
    estimator->result.combined_soh =
        (estimator->result.capacity_soh_lower < resistance_health) ?
        estimator->result.capacity_soh_lower : resistance_health;
    estimator->result.combined_soh = clampf_local(
        estimator->result.combined_soh, 0.0f, 1.0f);
}

void ams_soh_init(ams_soh_estimator_t *estimator,
                  const ams_soh_config_t *cfg)
{
    if(estimator == NULL)
    {
        return;
    }

    memset(estimator, 0, sizeof(*estimator));
    if(!ams_soh_config_valid(cfg))
    {
        estimator->result.last_reason_flags = AMS_SOH_REASON_BAD_ARGUMENT;
        return;
    }

    estimator->capacity_mean_ah = cfg->nominal_pack_capacity_ah;
    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        estimator->segment_resistance_growth_ratio[segment] = 1.0f;
        estimator->segment_resistance_growth_upper[segment] =
            cfg->prior_resistance_soh_upper;
    }
    refresh_resistance_summary(estimator, cfg);
    refresh_capacity_result(estimator, cfg);
    refresh_combined_result(estimator);
}

static uint32_t input_reasons(const ams_soh_config_t *cfg,
                              const ams_soh_input_t *input)
{
    uint32_t reasons = AMS_SOH_REASON_NONE;
    if((input->measurement_sequence == 0u) ||
       ((uint32_t)(input->now_ms - input->measurement_timestamp_ms) >
        cfg->maximum_measurement_age_ms))
    {
        reasons |= AMS_SOH_REASON_STALE;
    }
    if(input->measurement_valid == 0u)
    {
        reasons |= AMS_SOH_REASON_MEASUREMENT;
    }
    if(input->estimator_valid == 0u)
    {
        reasons |= AMS_SOH_REASON_ESTIMATOR;
    }
    if(input->current_calibrated == 0u)
    {
        reasons |= AMS_SOH_REASON_CURRENT_CALIBRATION;
    }
    if(input->current_polarity_validated == 0u)
    {
        reasons |= AMS_SOH_REASON_CURRENT_POLARITY;
    }
    if(input->balance_recovered == 0u)
    {
        reasons |= AMS_SOH_REASON_BALANCING;
    }
    if(!finite_positive(input->elapsed_s) ||
       !isfinite(input->pack_current_a) ||
       !isfinite(input->pack_current_uncertainty_a) ||
       (input->pack_current_uncertainty_a < 0.0f) ||
       !isfinite(input->total_charge_as) ||
       !isfinite(input->pack_soc) || (input->pack_soc < 0.0f) ||
       (input->pack_soc > 1.0f) ||
       !isfinite(input->average_cell_temp_c) ||
       !isfinite(input->cell_voltage_spread_v) ||
       (input->cell_voltage_spread_v < 0.0f) ||
       !isfinite(input->maximum_soc_sigma) ||
       (input->maximum_soc_sigma < 0.0f) ||
       !isfinite(input->maximum_abs_innovation_v_per_cell) ||
       (input->maximum_abs_innovation_v_per_cell < 0.0f) ||
       !isfinite(input->maximum_abs_polarization_v) ||
       (input->maximum_abs_polarization_v < 0.0f))
    {
        reasons |= AMS_SOH_REASON_NUMERIC;
    }
    return reasons;
}

static bool rest_observable(const ams_soh_config_t *cfg,
                            const ams_soh_input_t *input,
                            uint32_t *reasons)
{
    bool observable = true;
    if((fabsf(input->pack_current_a) + input->pack_current_uncertainty_a) >
       cfg->rest_current_max_a)
    {
        *reasons |= AMS_SOH_REASON_NOT_RESTED;
        observable = false;
    }
    if((input->average_cell_temp_c < cfg->minimum_temperature_c) ||
       (input->average_cell_temp_c > cfg->maximum_temperature_c))
    {
        *reasons |= AMS_SOH_REASON_TEMPERATURE;
        observable = false;
    }
    if(input->cell_voltage_spread_v > cfg->maximum_cell_spread_v)
    {
        *reasons |= AMS_SOH_REASON_CELL_SPREAD;
        observable = false;
    }
    if(input->maximum_soc_sigma > cfg->maximum_soc_sigma)
    {
        *reasons |= AMS_SOH_REASON_SOC_UNCERTAINTY;
        observable = false;
    }
    if(input->maximum_abs_innovation_v_per_cell >
       cfg->maximum_rest_innovation_v_per_cell)
    {
        *reasons |= AMS_SOH_REASON_REST_INNOVATION;
        observable = false;
    }
    if(input->maximum_abs_polarization_v >
       cfg->maximum_rest_polarization_v)
    {
        *reasons |= AMS_SOH_REASON_REST_POLARIZATION;
        observable = false;
    }
    return observable;
}

static void capture_or_evaluate_anchor(ams_soh_estimator_t *estimator,
                                       const ams_soh_config_t *cfg,
                                       const ams_soh_input_t *input)
{
    if(estimator->anchor_valid != 0u)
    {
        const float delta_soc = input->pack_soc - estimator->anchor_soc;
        const double delta_charge_as = input->total_charge_as -
                                       estimator->anchor_total_charge_as;
        const float throughput_ah = (float)(fabs(delta_charge_as) / 3600.0);
        uint32_t reject = AMS_SOH_REASON_NONE;

        if(fabsf(delta_soc) < cfg->minimum_soc_excursion)
        {
            reject |= AMS_SOH_REASON_SOC_EXCURSION;
        }
        if(throughput_ah < cfg->minimum_throughput_ah)
        {
            reject |= AMS_SOH_REASON_THROUGHPUT;
        }
        if(((double)delta_soc * delta_charge_as) >= 0.0)
        {
            /* Positive pack current integrates positive charge throughput while
             * SOC must fall; charge and SOC deltas therefore have opposite signs. */
            reject |= AMS_SOH_REASON_DIRECTION;
        }

        float candidate_ah = 0.0f;
        if(reject == AMS_SOH_REASON_NONE)
        {
            candidate_ah = throughput_ah / fabsf(delta_soc);
            const float lower_ah = cfg->nominal_pack_capacity_ah *
                                   cfg->minimum_capacity_soh;
            const float upper_ah = cfg->nominal_pack_capacity_ah *
                                   cfg->maximum_capacity_soh;
            if(!isfinite(candidate_ah) || (candidate_ah < lower_ah) ||
               (candidate_ah > upper_ah))
            {
                reject |= AMS_SOH_REASON_CAPACITY_RANGE;
            }
            else if((estimator->result.accepted_capacity_windows > 0u) &&
                    (fabs(candidate_ah - estimator->capacity_mean_ah) /
                     estimator->capacity_mean_ah >
                     cfg->maximum_relative_outlier))
            {
                reject |= AMS_SOH_REASON_CAPACITY_OUTLIER;
            }
        }

        if(reject == AMS_SOH_REASON_NONE)
        {
            const uint32_t next_count = saturating_increment(
                estimator->result.accepted_capacity_windows);
            if(next_count != estimator->result.accepted_capacity_windows)
            {
                const double delta = (double)candidate_ah -
                                     estimator->capacity_mean_ah;
                estimator->capacity_mean_ah += delta / (double)next_count;
                const double delta2 = (double)candidate_ah -
                                      estimator->capacity_mean_ah;
                estimator->capacity_m2_ah2 += delta * delta2;
                estimator->result.accepted_capacity_windows = next_count;
                estimator->result.last_accept_tick = input->now_ms;
            }
        }
        else
        {
            estimator->result.rejected_capacity_windows = saturating_increment(
                estimator->result.rejected_capacity_windows);
            estimator->result.last_reason_flags |= reject;
        }
    }

    estimator->anchor_soc = input->pack_soc;
    estimator->anchor_temp_c = input->average_cell_temp_c;
    estimator->anchor_total_charge_as = input->total_charge_as;
    estimator->anchor_tick = input->now_ms;
    estimator->anchor_valid = 1u;
}

bool ams_soh_update(ams_soh_estimator_t *estimator,
                    const ams_soh_config_t *cfg,
                    const ams_soh_input_t *input)
{
    if((estimator == NULL) || (input == NULL) ||
       !ams_soh_config_valid(cfg))
    {
        if(estimator != NULL)
        {
            estimator->result.last_reason_flags =
                AMS_SOH_REASON_BAD_ARGUMENT;
        }
        return false;
    }

    uint32_t reasons = input_reasons(cfg, input);
    const uint32_t fatal = AMS_SOH_REASON_STALE |
                           AMS_SOH_REASON_MEASUREMENT |
                           AMS_SOH_REASON_ESTIMATOR |
                           AMS_SOH_REASON_CURRENT_CALIBRATION |
                           AMS_SOH_REASON_CURRENT_POLARITY |
                           AMS_SOH_REASON_BALANCING |
                           AMS_SOH_REASON_NUMERIC;
    estimator->result.last_reason_flags = reasons;
    estimator->result.last_update_tick = input->now_ms;
    refresh_resistance_result(estimator, cfg, input);

    if((reasons & fatal) != 0u)
    {
        estimator->rest_elapsed_s = 0.0f;
        estimator->rest_latched = 0u;
        refresh_capacity_result(estimator, cfg);
        refresh_combined_result(estimator);
        return false;
    }

    bool at_rest = rest_observable(cfg, input,
                                   &estimator->result.last_reason_flags);
    if(at_rest)
    {
        estimator->rest_elapsed_s += input->elapsed_s;
        if(!isfinite(estimator->rest_elapsed_s))
        {
            estimator->rest_elapsed_s = 0.0f;
            estimator->result.last_reason_flags |= AMS_SOH_REASON_NUMERIC;
            refresh_capacity_result(estimator, cfg);
            refresh_combined_result(estimator);
            return false;
        }
        if((estimator->rest_elapsed_s >= cfg->required_rest_time_s) &&
           (estimator->rest_latched == 0u))
        {
            capture_or_evaluate_anchor(estimator, cfg, input);
            estimator->rest_latched = 1u;
        }
    }
    else
    {
        estimator->rest_elapsed_s = 0.0f;
        estimator->rest_latched = 0u;
    }

    refresh_capacity_result(estimator, cfg);
    refresh_combined_result(estimator);
    return true;
}

static uint32_t crc32_bytes(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for(size_t i = 0u; i < length; i++)
    {
        crc ^= data[i];
        for(uint8_t bit = 0u; bit < 8u; bit++)
        {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

uint32_t ams_soh_record_crc32(const ams_soh_persist_record_t *record)
{
    if(record == NULL)
    {
        return 0u;
    }
    return crc32_bytes((const uint8_t *)record,
                       offsetof(ams_soh_persist_record_t, crc32));
}

bool ams_soh_export_record(const ams_soh_estimator_t *estimator,
                           uint32_t generation,
                           ams_soh_persist_record_t *record)
{
    if((estimator == NULL) || (record == NULL) || (generation == 0u) ||
       ((estimator->result.accepted_capacity_windows == 0u) &&
        (estimator->segment_resistance_valid_mask == 0u)) ||
       !isfinite(estimator->capacity_mean_ah) ||
       !isfinite(estimator->capacity_m2_ah2) ||
       (estimator->capacity_m2_ah2 < 0.0) ||
       ((estimator->segment_resistance_valid_mask &
         (uint8_t)~AMS_SOH_ALL_SEGMENTS_MASK) != 0u))
    {
        return false;
    }

    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        if(!isfinite(estimator->segment_resistance_growth_upper[segment]) ||
           (estimator->segment_resistance_growth_upper[segment] < 1.0f) ||
           (estimator->segment_resistance_growth_upper[segment] > 3.0f) ||
           (estimator->segment_resistance_confidence_pct[segment] > 100u))
        {
            return false;
        }
    }

    memset(record, 0, sizeof(*record));
    record->magic = AMS_SOH_PERSIST_MAGIC;
    record->schema = AMS_SOH_PERSIST_SCHEMA;
    record->size = (uint16_t)sizeof(*record);
    record->generation = generation;
    record->accepted_capacity_windows =
        estimator->result.accepted_capacity_windows;
    record->capacity_mean_ah = estimator->capacity_mean_ah;
    record->capacity_m2_ah2 = estimator->capacity_m2_ah2;
    memcpy(record->segment_resistance_growth_upper,
           estimator->segment_resistance_growth_upper,
           sizeof(record->segment_resistance_growth_upper));
    memcpy(record->segment_resistance_confidence_pct,
           estimator->segment_resistance_confidence_pct,
           sizeof(record->segment_resistance_confidence_pct));
    record->segment_resistance_valid_mask =
        estimator->segment_resistance_valid_mask;
    if(estimator->result.capacity_valid != 0u)
    {
        record->validity_flags |= AMS_SOH_PERSIST_CAPACITY_VALID;
    }
    if(estimator->segment_resistance_valid_mask ==
       AMS_SOH_ALL_SEGMENTS_MASK)
    {
        record->validity_flags |= AMS_SOH_PERSIST_RESISTANCE_VALID;
    }
    record->crc32 = ams_soh_record_crc32(record);
    return true;
}

bool ams_soh_import_record(ams_soh_estimator_t *estimator,
                           const ams_soh_config_t *cfg,
                           const ams_soh_persist_record_t *record)
{
    if((estimator == NULL) || !ams_soh_config_valid(cfg) ||
       (record == NULL) || (record->magic != AMS_SOH_PERSIST_MAGIC) ||
       (record->schema != AMS_SOH_PERSIST_SCHEMA) ||
       (record->size != sizeof(*record)) ||
       (record->generation == 0u) ||
       (record->crc32 != ams_soh_record_crc32(record)) ||
       !isfinite(record->capacity_mean_ah) ||
       !isfinite(record->capacity_m2_ah2) ||
       (record->capacity_m2_ah2 < 0.0) ||
       ((record->accepted_capacity_windows == 0u) &&
        (record->segment_resistance_valid_mask == 0u)) ||
       ((record->segment_resistance_valid_mask &
         (uint8_t)~AMS_SOH_ALL_SEGMENTS_MASK) != 0u) ||
       ((record->validity_flags &
         (uint8_t)~(AMS_SOH_PERSIST_CAPACITY_VALID |
                    AMS_SOH_PERSIST_RESISTANCE_VALID)) != 0u) ||
       (((record->validity_flags & AMS_SOH_PERSIST_CAPACITY_VALID) != 0u) &&
        (record->accepted_capacity_windows <
         cfg->minimum_capacity_observations)) ||
       ((((record->validity_flags & AMS_SOH_PERSIST_RESISTANCE_VALID) != 0u) ?
          1u : 0u) !=
        ((record->segment_resistance_valid_mask ==
          AMS_SOH_ALL_SEGMENTS_MASK) ? 1u : 0u)))
    {
        if(estimator != NULL)
        {
            estimator->result.last_reason_flags |=
                AMS_SOH_REASON_PERSISTENCE;
        }
        return false;
    }

    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        if(!isfinite(record->segment_resistance_growth_upper[segment]) ||
           (record->segment_resistance_growth_upper[segment] < 1.0f) ||
           (record->segment_resistance_growth_upper[segment] > 3.0f) ||
           (record->segment_resistance_confidence_pct[segment] > 100u))
        {
            estimator->result.last_reason_flags |=
                AMS_SOH_REASON_PERSISTENCE;
            return false;
        }
    }

    const float minimum_ah = cfg->nominal_pack_capacity_ah *
                             cfg->minimum_capacity_soh;
    const float maximum_ah = cfg->nominal_pack_capacity_ah *
                             cfg->maximum_capacity_soh;
    if((record->capacity_mean_ah < (double)minimum_ah) ||
       (record->capacity_mean_ah > (double)maximum_ah))
    {
        estimator->result.last_reason_flags |= AMS_SOH_REASON_PERSISTENCE;
        return false;
    }

    ams_soh_init(estimator, cfg);
    estimator->capacity_mean_ah = record->capacity_mean_ah;
    estimator->capacity_m2_ah2 = record->capacity_m2_ah2;
    estimator->result.accepted_capacity_windows =
        record->accepted_capacity_windows;
    estimator->segment_resistance_valid_mask =
        record->segment_resistance_valid_mask;
    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        const uint8_t bit = (uint8_t)(1u << segment);
        if((record->segment_resistance_valid_mask & bit) != 0u)
        {
            estimator->segment_resistance_growth_ratio[segment] =
                record->segment_resistance_growth_upper[segment];
            estimator->segment_resistance_growth_upper[segment] =
                record->segment_resistance_growth_upper[segment];
            estimator->segment_resistance_confidence_pct[segment] =
                record->segment_resistance_confidence_pct[segment];
        }
    }
    refresh_resistance_summary(estimator, cfg);
    estimator->result.persistence_valid = 1u;
    refresh_capacity_result(estimator, cfg);
    if((record->validity_flags & AMS_SOH_PERSIST_CAPACITY_VALID) == 0u)
    {
        /* A persisted observation count is not itself authority. Preserve the
         * qualification state recorded at export so a reboot cannot create
         * confidence that did not exist before the write. */
        estimator->result.capacity_valid = 0u;
        estimator->result.capacity_soh_lower =
            cfg->prior_capacity_soh_lower;
    }
    refresh_combined_result(estimator);
    return true;
}

bool ams_soh_select_newest_record(const ams_soh_config_t *cfg,
                                  const ams_soh_persist_record_t *first,
                                  const ams_soh_persist_record_t *second,
                                  ams_soh_persist_record_t *selected)
{
    if((selected == NULL) || !ams_soh_config_valid(cfg))
    {
        return false;
    }

    ams_soh_estimator_t scratch;
    const bool first_valid = (first != NULL) &&
        ams_soh_import_record(&scratch, cfg, first);
    const bool second_valid = (second != NULL) &&
        ams_soh_import_record(&scratch, cfg, second);
    if(!first_valid && !second_valid)
    {
        return false;
    }
    if(first_valid && !second_valid)
    {
        *selected = *first;
        return true;
    }
    if(second_valid && !first_valid)
    {
        *selected = *second;
        return true;
    }

    /* Signed modular comparison is safe for adjacent wear-level generations;
     * generation zero is reserved as invalid. */
    *selected = ((int32_t)(first->generation - second->generation) > 0) ?
        *first : *second;
    return true;
}
