/* Passive, immutable DADEKF/SoP/fuse tuning snapshot transport. */
#ifndef INC_EXT_DRIVERS_AMS_TUNING_TELEMETRY_H_
#define INC_EXT_DRIVERS_AMS_TUNING_TELEMETRY_H_

#include <stdbool.h>
#include <stdint.h>

#include "estimator/ams_soc_ekf.h"
#include "sop/ams_sop.h"

typedef struct
{
    float soc, vp1_v, vp2_v, r0_ohm, t_core_c;
    float p_soc, p_vp1, p_vp2, p_r0, r_meas_v2;
    /* Full [SoC,Vp1,Vp2] covariance cross terms. SoP deliberately keeps
     * its existing L1 sigma margin, which remains conservative for any
     * admissible correlation, while these terms support estimator
     * consistency diagnostics and offline NEES. */
    float p_soc_vp1, p_soc_vp2, p_vp1_vp2;
    float v_pred_v, innovation_v, innovation_variance_v2;
    float measured_v, current_a, surface_temp_c;
    float voltage_raw_v, voltage_avg8_v, voltage_iir_v;
    float reference_r0_ohm, resistance_growth_ratio, r0_variance_ohm2;
    float acquisition_candidate_soc, acquisition_ocv_cell_v;
    float acquisition_vp1_finish_v, acquisition_vp2_finish_v;
    float acquisition_fit_rmse_mv_cell, acquisition_fit_rcond;
    float acquisition_consensus_soc;
    uint32_t step_count, innovation_reject_count, dt_clamp_count;
    uint32_t covariance_repair_count;
    uint32_t acquisition_dynamic_step_count, acquisition_dynamic_update_count;
    uint32_t acquisition_anchor_count;
    uint32_t fault_flags, measurement_sequence, current_sequence;
    uint32_t measurement_age_ms, current_age_ms, soh_reject_flags;
    uint16_t fresh_temp_count;
    uint8_t voltage_valid_flags, model_domain_flags, valid;
    uint8_t acquisition_state, acquisition_reason;
    uint8_t acquisition_sample_count, acquisition_reject_count;
    uint8_t soh_confidence_pct, soh_status_flags;
    uint8_t soh_accepted_count, soh_rejected_count;
} ams_tuning_segment_t;

typedef struct
{
    float raw_model_discharge_a, raw_model_charge_a;
    float strategy_discharge_a, strategy_charge_a;
    float final_discharge_a, final_charge_a;
    float discharge_power_w, charge_power_w;
    float discharge_min_cell_v, charge_max_cell_v;
    uint8_t discharge_binding, charge_binding;
    uint8_t discharge_segment, discharge_cell;
    uint8_t charge_segment, charge_cell;
} ams_tuning_sop_horizon_t;

typedef struct
{
    uint32_t snapshot_sequence;
    uint32_t measurement_sequence;
    uint32_t estimator_step;
    uint32_t source_tick_ms;
    uint32_t reason_flags;
    ams_tuning_segment_t segment[AMS_EKF_MAX_INSTANCES];
    ams_tuning_sop_horizon_t horizon[AMS_SOP_HORIZONS];
    float fuse_utilization, fuse_temperature_c, fuse_derating;
    float fuse_effective_current_a, fuse_equivalent_current_a;
    float fuse_typical_melt_time_s, fuse_usable_melt_time_s;
    float fuse_cap_a[AMS_SOP_HORIZONS];
    float hardware_discharge_cap_a[AMS_SOP_HORIZONS];
    uint16_t fuse_reason_flags;
    uint8_t instance_count, power_valid, power_authority_valid;
    uint8_t fuse_valid, fuse_authority_valid, fuse_budget_exhausted;
} ams_tuning_snapshot_t;

typedef struct
{
    ams_tuning_snapshot_t buffer[2];
    uint16_t reader_count[2];
    uint32_t next_sequence;
    uint32_t publication_drop_count;
    uint8_t published_index;
    bool published;
} ams_tuning_store_t;

#endif /* INC_EXT_DRIVERS_AMS_TUNING_TELEMETRY_H_ */
