/*
 * ams_soc_ekf.h
 * Author: Mahad Faisal (2026)
 *
 * Physics-only adaptive dual EKF for the DER AMS.
 *
 * Matches the working RA8M1 estimator architecture: 3-state inner EKF,
 * scalar outer R0 loop, adaptive measurement covariance, feed-forward thermal
 * observer, LUT OCV/R0/C1/tau1, and fixed slow R2/C2 branch.
 *
 * Version 1 intentionally has no NN residual layer and no authority over
 * BMS_OK, AIRs, charging, shutdown, or balancing. It is an advisory estimator
 * that can run one full-pack instance now and scale to segment/sub-segment
 * instances later.
 */

#ifndef INC_ESTIMATOR_AMS_SOC_EKF_H_
#define INC_ESTIMATOR_AMS_SOC_EKF_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_EKF_MAX_INSTANCES       10U
#define AMS_EKF_ADAPT_WIN           10U

#define AMS_EKF_PACK_SERIES_GROUPS  75U
#define AMS_EKF_PACK_PARALLEL_CELLS 6.0f
#define AMS_EKF_CELL_CAPACITY_AH    4.2f
#define AMS_EKF_DEFAULT_DT_S        0.1f

#define AMS_EKF_DEFAULT_SOC_INIT    1.0f
#define AMS_EKF_DEFAULT_R0_INIT_OHM 0.0147f

#define AMS_HIL_CAN_ID_MEAS         0x200U
#define AMS_HIL_CAN_ID_TRUTH        0x201U
#define AMS_HIL_CAN_ID_SUMMARY      0x202U
#define AMS_HIL_CAN_ID_CELL_SAMPLE  0x210U
#define AMS_HIL_CAN_ID_TEMP_SAMPLE  0x211U
#define AMS_HIL_CAN_ID_CTRL         0x300U
#define AMS_ESTIMATOR_STATUS_CAN_ID 0x421U

#define AMS_EKF_FAULT_NONE          0x00000000UL
#define AMS_EKF_FAULT_DISABLED      0x00000001UL
#define AMS_EKF_FAULT_BAD_CONFIG    0x00000002UL
#define AMS_EKF_FAULT_BAD_INPUT     0x00000004UL
#define AMS_EKF_FAULT_BAD_VOLTAGE   0x00000008UL
#define AMS_EKF_FAULT_BAD_CURRENT   0x00000010UL
#define AMS_EKF_FAULT_BAD_TEMP      0x00000020UL
#define AMS_EKF_FAULT_STALE_INPUT   0x00000040UL
#define AMS_EKF_FAULT_CLAMPED       0x00000080UL

#define AMS_EKF_FLAG_VALID          0x01U
#define AMS_EKF_FLAG_HIL_SOURCE     0x02U
#define AMS_EKF_FLAG_FAULTED        0x04U
#define AMS_EKF_FLAG_STALE          0x08U
#define AMS_EKF_FLAG_CLAMPED        0x10U
#define AMS_EKF_FLAG_CC_FALLBACK    0x20U

typedef enum
{
    AMS_ESTIMATOR_INPUT_NONE = 0,
    AMS_ESTIMATOR_INPUT_HARDWARE = 1,
    AMS_ESTIMATOR_INPUT_HIL_CAN = 2
} ams_estimator_input_source_t;

typedef struct
{
    uint8_t  enabled;
    uint16_t first_series_group;    /* 0..74 linear index across 5x15 groups */
    uint16_t series_group_count;    /* 75 for pack, 15 for one segment, 7/8 for split */
    float    parallel_cell_count;   /* 6 for the DER pack */
    float    cell_capacity_Ah;      /* P42A nominal cell capacity */
    float    sample_time_s;
    float    soc_init;
    float    r0_init_ohm;
} ams_ekf_config_t;

typedef struct
{
    ams_ekf_config_t cfg;

    float soc;          /* [0,1] */
    float vp1_V;        /* [V/cell] */
    float vp2_V;        /* [V/cell] */
    float r0_ohm;       /* [ohm/cell] */
    float t_core_C;     /* estimated representative core temperature */

    float p_soc;
    float p_vp1;
    float p_vp2;
    float p_r0;

    float r_meas_V2;
    float innov_hist[AMS_EKF_ADAPT_WIN];
    uint8_t innov_idx;

    float v_pred_V;
    float innovation_V;
    float last_i_pack_A;
    float last_v_meas_V;
    float last_t_surf_C;
    uint32_t step_count;
    uint32_t fault_flags;
    uint8_t valid;
} ams_ekf_instance_t;

typedef struct
{
    uint8_t enabled;
    uint8_t instance_count;
    uint8_t active_index;
    ams_estimator_input_source_t input_source;
    uint32_t last_update_tick;
    uint32_t step_count;
    uint32_t fault_flags;

    float pack_soc;
    float pack_r0_ohm;
    float pack_v_pred_V;
    float pack_innovation_V;
    float pack_t_core_C;
    float cc_soc;
    uint8_t cc_valid;
    uint32_t cc_step_count;

    ams_ekf_instance_t inst[AMS_EKF_MAX_INSTANCES];
} ams_estimator_t;

typedef struct
{
    uint8_t fresh;
    uint8_t counter;
    uint32_t last_rx_tick;
    float v_pack_V;
    float i_pack_A;
    float t_surf_C;
} ams_hil_meas_t;

typedef struct
{
    uint8_t fresh;
    uint8_t counter;
    uint32_t last_rx_tick;
    uint32_t plant_step;
    float soc_true;
    float t_core_C;
} ams_hil_truth_t;

typedef struct
{
    uint8_t fresh;
    uint32_t last_rx_tick;
    float v_min_V;
    float v_max_V;
    float t_max_C;
    float t_avg_C;
} ams_hil_summary_t;

typedef struct
{
    ams_hil_meas_t meas;
    ams_hil_truth_t truth;
    ams_hil_summary_t summary;
} ams_hil_input_t;

void ams_ekf_make_pack_config(ams_ekf_config_t *cfg);
void ams_ekf_make_segment_config(ams_ekf_config_t *cfg, uint8_t segment_index);
void ams_ekf_make_group_range_config(ams_ekf_config_t *cfg,
                                     uint16_t first_series_group,
                                     uint16_t series_group_count);

void ams_ekf_init(ams_ekf_instance_t *ekf, const ams_ekf_config_t *cfg);
bool ams_ekf_step(ams_ekf_instance_t *ekf,
                  float i_pack_A,
                  float v_meas_V,
                  float t_surf_C,
                  float dt_s);

void ams_estimator_init_default(ams_estimator_t *est);
bool ams_estimator_configure_pack(ams_estimator_t *est);
bool ams_estimator_configure_segments(ams_estimator_t *est);
bool ams_estimator_configure_even_split(ams_estimator_t *est, uint8_t instance_count);
void ams_estimator_cc_reset(ams_estimator_t *est, float soc_init);
bool ams_estimator_cc_step(ams_estimator_t *est, float i_pack_A, float dt_s);
void ams_estimator_refresh_summary(ams_estimator_t *est,
                                   ams_estimator_input_source_t source,
                                   uint32_t tick);
uint8_t ams_estimator_status_flags(const ams_estimator_t *est);

#ifdef __cplusplus
}
#endif

#endif /* INC_ESTIMATOR_AMS_SOC_EKF_H_ */
