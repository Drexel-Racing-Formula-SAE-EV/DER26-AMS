/*
 * app.h
 *
 *  Created on: Jan 29, 2024
 *      Author: cole
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include <stdbool.h>
#include <stdint.h>

#include "ams_build_profile.h"
#include "stm32f7xx_hal.h"
#include "main.h"
#include "board.h"
#include "ext_drivers/accumulator.h"
#include "estimator/ams_soc_ekf.h"
#include "ext_drivers/current_fault.h"
#include "ext_drivers/voltage_fault.h"
#include "ext_drivers/temperature_fault.h"
#include "ext_drivers/air_monitor.h"
#include "ext_drivers/ams_safety.h"
#include "measurement/ams_measurement.h"
#include "sop/ams_power_state.h"

#define VER_MAJOR 0
#define VER_MINOR 3
#define VER_BUG   0

#define AMS_BUILD_MANIFEST_MAGIC 0x414D5342u /* 'AMSB' */
#define AMS_BUILD_MANIFEST_SCHEMA 4u

#define AMS_BUILD_FEATURE_HIL_CAN      (1u << 0u)
#define AMS_BUILD_FEATURE_HIL_ADBMS    (1u << 1u)
#define AMS_BUILD_FEATURE_SERVICE_CLI  (1u << 2u)
#define AMS_BUILD_FEATURE_IMD          (1u << 3u)
#define AMS_BUILD_FEATURE_IWDG         (1u << 4u)
#define AMS_BUILD_FEATURE_AIR_AUX      (1u << 5u)
#define AMS_BUILD_FEATURE_APM_2950     (1u << 6u)
#define AMS_BUILD_FEATURE_HW_BRINGUP   (1u << 7u)
#define AMS_BUILD_FEATURE_MISSION_CAN  (1u << 8u)
#define AMS_BUILD_FEATURE_FUSE_MODEL   (1u << 9u)

#define AMS_BUILD_FEATURE_FLAGS_VALUE ( \
    (AMS_ENABLE_HIL_CAN ? AMS_BUILD_FEATURE_HIL_CAN : 0u) | \
    (AMS_HIL_REPLACE_ADBMS ? AMS_BUILD_FEATURE_HIL_ADBMS : 0u) | \
    (AMS_ENABLE_SERVICE_CLI ? AMS_BUILD_FEATURE_SERVICE_CLI : 0u) | \
    (AMS_ENABLE_IMD ? AMS_BUILD_FEATURE_IMD : 0u) | \
    (AMS_ENABLE_IWDG ? AMS_BUILD_FEATURE_IWDG : 0u) | \
    (AMS_ENABLE_AIR_AUX_FEEDBACK ? AMS_BUILD_FEATURE_AIR_AUX : 0u) | \
    (AMS_ENABLE_APM_2950 ? AMS_BUILD_FEATURE_APM_2950 : 0u) | \
    (AMS_HW_BRINGUP ? AMS_BUILD_FEATURE_HW_BRINGUP : 0u) | \
    (AMS_ENABLE_MISSION_CAN ? AMS_BUILD_FEATURE_MISSION_CAN : 0u) | \
    (AMS_FUSE_MODEL_VALIDATED ? AMS_BUILD_FEATURE_FUSE_MODEL : 0u))

#ifndef AMS_HW_BRINGUP
#define AMS_HW_BRINGUP 0
#endif

#ifndef AMS_HW_BRINGUP_BMS_OK_RELEASED_DEFAULT
#define AMS_HW_BRINGUP_BMS_OK_RELEASED_DEFAULT 0
#endif

#ifndef AMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT
#define AMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT \
    (AMS_HW_BRINGUP || AMS_PROFILE_BALANCE_INHIBIT_DEFAULT)
#endif

#ifndef AMS_HIL_REPLACE_ADBMS
#define AMS_HIL_REPLACE_ADBMS 0
#endif

/* HIL CAN injection is excluded from normal/production firmware unless an
 * explicit HIL build enables it.  This prevents standard CAN frames from
 * overwriting authoritative accumulator measurements in vehicle firmware. */
#ifndef AMS_ENABLE_HIL_CAN
#define AMS_ENABLE_HIL_CAN 0
#endif

/* Service CLI actions are disabled in normal/production firmware.  The
 * dedicated hardware-bring-up profile enables them by default, and a build
 * may still override this macro explicitly when required on a controlled
 * bench. */
#ifndef AMS_ENABLE_SERVICE_CLI
#define AMS_ENABLE_SERVICE_CLI AMS_HW_BRINGUP
#endif

/* The PCB/Cube configuration routes IMD_PWM to TIM2_CH1.  Keep the feature
 * opt-in until that external PWM/status wiring and polarity have been checked
 * on the target.  With it disabled, the supervisor deliberately retains the
 * initialized IMD fault and cannot assert BMS_OK. */
#ifndef AMS_ENABLE_IMD
#define AMS_ENABLE_IMD 0
#endif

/*
 * Future AIR+/AIR-/precharge auxiliary-contact supervision.  The current PCB
 * only provides AIR_CONTROL_MCU, which senses voltage on the common control net
 * and is not physical contactor feedback.  Keep this gate disabled until the
 * protected auxiliary inputs, pin mappings, polarities, line supervision and
 * manufacturer timing limits have been reviewed and physically validated.
 *
 * Enabling this flag before one board adapter supplies fresh classified inputs
 * to ams_air_monitor_step() is deliberately fail-closed: BMS_OK remains low.
 * The target profile must also supply reviewed ams_air_monitor_config_t values;
 * there are intentionally no generic timing or voltage defaults.
 */
#ifndef AMS_ENABLE_AIR_AUX_FEEDBACK
#define AMS_ENABLE_AIR_AUX_FEEDBACK 0
#endif

/* A target build must explicitly assert that the board adapter, schematic,
 * pin polarity and monitor period have been reviewed. Host/SIL builds may
 * enable the logic gate without claiming that physical inputs exist. */
#ifndef AMS_AIR_AUX_BOARD_ADAPTER_READY
#define AMS_AIR_AUX_BOARD_ADAPTER_READY 0
#endif

#ifndef AMS_AIR_MONITOR_PERIOD_MS
#define AMS_AIR_MONITOR_PERIOD_MS 0u
#endif

#ifndef AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS
#define AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS 0u
#endif

#if AMS_ENABLE_AIR_AUX_FEEDBACK && !defined(AMS_HOST_TEST)
#if !AMS_AIR_AUX_BOARD_ADAPTER_READY
#error "AIR auxiliary feedback requires a reviewed board adapter"
#endif
#if AMS_AIR_MONITOR_PERIOD_MS == 0u
#error "AIR auxiliary feedback requires a reviewed nonzero monitor period"
#endif
#if AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS < AMS_AIR_MONITOR_PERIOD_MS
#error "AIR publication timeout must be at least one monitor period"
#endif
#if AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS > 0x7FFFFFFFu
#error "AIR publication timeout must be tick-wrap safe"
#endif
#endif

#if AMS_HIL_REPLACE_ADBMS && !AMS_ENABLE_HIL_CAN
#error "AMS_HIL_REPLACE_ADBMS requires AMS_ENABLE_HIL_CAN=1"
#endif

#define AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS 500u

#define ERR_FREQ 20
#define CLI_FREQ 20
#define AIR_FREQ 2 /* legacy control-sense path; future monitor uses its period macro */
#define CURRENT_FREQ 50

/* ADBMS acquisition profile.  The 1 Hz setting is intentionally limited to
 * hardware bring-up, where BMS_OK and balancing are inhibited by default.
 * Normal firmware restores the original 10 Hz acquisition intent.  A build
 * may override AMS_ADBMS_SCAN_HZ, but it must remain a whole-Hz rate that can
 * be represented by the 1 ms RTOS tick. */
#ifndef AMS_ADBMS_SCAN_HZ
#if AMS_HW_BRINGUP
#define AMS_ADBMS_SCAN_HZ 1u
#else
#define AMS_ADBMS_SCAN_HZ 10u
#endif
#endif

#if (AMS_ADBMS_SCAN_HZ == 0u) || (AMS_ADBMS_SCAN_HZ > 1000u)
#error "AMS_ADBMS_SCAN_HZ must be between 1 and 1000 Hz"
#endif

#define ADBMS_FREQ AMS_ADBMS_SCAN_HZ
#define AMS_ADBMS_TASK_PERIOD_MS ((1000u + ADBMS_FREQ - 1u) / ADBMS_FREQ)

/* Passive balancing is disabled before a measurement so the cell inputs can
 * recover from bleed load.  When balancing is re-applied, hold it on for a
 * real, measurable interval before beginning the next recovery/scan cycle.
 * Both values are deliberately overrideable for target characterization. */
#ifndef AMS_ADBMS_BALANCE_RECOVERY_MS
#define AMS_ADBMS_BALANCE_RECOVERY_MS 100u
#endif
#ifndef AMS_ADBMS_BALANCE_MIN_ON_MS
#define AMS_ADBMS_BALANCE_MIN_ON_MS 100u
#endif

#if (AMS_ADBMS_BALANCE_RECOVERY_MS == 0u) || \
    (AMS_ADBMS_BALANCE_RECOVERY_MS > 0x7FFFFFFFu)
#error "AMS_ADBMS_BALANCE_RECOVERY_MS must be nonzero and tick-wrap safe"
#endif
#if (AMS_ADBMS_BALANCE_MIN_ON_MS == 0u) || \
    (AMS_ADBMS_BALANCE_MIN_ON_MS > 0x7FFFFFFFu)
#error "AMS_ADBMS_BALANCE_MIN_ON_MS must be nonzero and tick-wrap safe"
#endif
#define IMD_FREQ 10
#define FAN_FREQ 5
#define CAN_FREQ 2
#define ESTIMATOR_FREQ 10

#define AMS_HEARTBEAT_STARTUP_GRACE_MS 3000u
#define AMS_HEARTBEAT_ADBMS_TIMEOUT_MS 3000u
#define AMS_HEARTBEAT_CURRENT_TIMEOUT_MS 200u
#define AMS_HEARTBEAT_TEMP_TIMEOUT_MS 3000u
#define AMS_HEARTBEAT_CAN_TIMEOUT_MS 2000u
#define AMS_HEARTBEAT_LOGGER_TIMEOUT_MS 2000u
#define AMS_HEARTBEAT_IMD_TIMEOUT_MS 500u
#define AMS_HEARTBEAT_FAN_TIMEOUT_MS 1000u
#define AMS_HEARTBEAT_ESTIMATOR_TIMEOUT_MS 500u

/* CMSIS-RTOS mutex timeouts are expressed in kernel ticks.  The generated
 * FreeRTOS configuration uses a 1 kHz kernel tick, so this is 500 ms on the
 * target.  A bounded wait is essential here: the caller's panic path drops
 * BMS_OK, whereas osWaitForever could leave it asserted after a deadlock. */
#define AMS_ADBMS_MUTEX_TIMEOUT_TICKS 500u
#define AMS_CURRENT_WINDOW_MUTEX_TIMEOUT_TICKS 20u

/* FreeRTOS priority policy: larger number means higher priority.
 * Safety supervisor stays highest. Blocking bench/service CLI stays below all
 * measurement/control tasks so long UART prints cannot starve ADBMS/current/CAN.
 */
#define ERR_PRIO    17
#define CUR_PRIO    12
#define ADBMS_PRIO  11
#define CAN_PRIO    10
#define EST_PRIO     8
#define FAN_PRIO     7
#define AIR_PRIO     7
#define IMD_PRIO     6
#define CLI_PRIO     4

/* Static FreeRTOS task stack sizes are in StackType_t words, not bytes.
 * Cortex-M7 uses 32-bit StackType_t, so multiply by 4 for bytes.
 */
#define AMS_STACK_ERROR_WORDS       256u
#define AMS_STACK_CURRENT_WORDS     256u
#define AMS_STACK_ADBMS_WORDS      1536u
#define AMS_STACK_CAN_WORDS         512u
#define AMS_STACK_ESTIMATOR_WORDS  1536u
#define AMS_STACK_FAN_WORDS         192u
#define AMS_STACK_AIR_WORDS         192u
#define AMS_STACK_IMD_WORDS         192u
#define AMS_STACK_CLI_WORDS         512u

#define AMS_RTOS_STACK_WARN_WORDS    96u
#define AMS_RTOS_HEAP_WARN_BYTES   2048u

#define ECU_CANBUS_ID 0x69u

/* Compact AMS -> ECU safety/status contract. These frames are intentionally
 * separate from the legacy paged 0x069 telemetry stream and the passive logger
 * 0x690+ stream. ECU should use these for runtime gating; full cell/temp data
 * can stay slow/logger-side during staged bench bring-up. */
#define AMS_ECU_CAN_ID_STATUS      0x680u
#define AMS_ECU_CAN_ID_ELECTRICAL  0x681u
#define AMS_ECU_CAN_ID_THERMAL     0x682u
#define AMS_ECU_CAN_ID_HEALTH      0x683u
#define AMS_ECU_CAN_ID_SOP_DCL     0x684u
#define AMS_ECU_CAN_ID_SOP_CCL     0x685u
#define AMS_ECU_CAN_ID_SOH         0x686u
#define AMS_ECU_CAN_ID_SOP_ENVELOPE 0x687u
#define AMS_ECU_CAN_ID_MISSION_REQUEST 0x688u
#define AMS_ECU_CAN_ID_STRATEGY_STATUS 0x689u
#define AMS_ECU_CAN_ID_SOP_BINDINGS  0x68Au
#define AMS_CAN_ECU_FAST_FREQ_HZ   10u
#define AMS_CAN_ECU_FAST_PERIOD_MS (1000u / AMS_CAN_ECU_FAST_FREQ_HZ)

#define TO_LSB16(x) ((uint16_t)x & 0xff)
#define TO_MSB16(x) ((((uint16_t)x & 0xff00) >> 8) & 0xff)

// TODO: confirm temp thresholds against accumulator design/rules.
#define TEMP_THRESH_H TEMP_HOT_HARD_C
#define TEMP_THRESH_L TEMP_FAN_RAMP_START_C
/* Voltage thresholds are defined in ext_drivers/voltage_fault.h in mV. */

typedef enum
{
	STATE_NULL,
	STATE_START,
	STATE_CHARGE,
	STATE_DISCARGE,
	STATE_BALANCE,
	STATE_ERROR
} state_t;

typedef struct
{
    uint32_t magic;
    uint16_t schema;
    uint8_t profile;
    uint16_t feature_flags;
    uint8_t estimator_topology;
    const char *profile_name;
    const char *git_commit;
	const char *estimator_model_revision;
	const char *sop_model_revision;
	const char *soh_model_revision;
    const char *current_calibration_revision;
    const char *can_contract_revision;
    const char *threshold_revision;
} ams_build_manifest_t;

extern const ams_build_manifest_t ams_build_manifest;

typedef enum
{
	AMS_STATE_TRANSITION_BOOT = 0,
	AMS_STATE_TRANSITION_STARTUP_READY,
	AMS_STATE_TRANSITION_SERVICE_COMMAND,
	AMS_STATE_TRANSITION_CORRUPT_CURRENT_STATE,
	AMS_STATE_TRANSITION_INVALID_REQUEST
} ams_state_transition_reason_t;

typedef enum
{
	AMS_STATE_TRANSITION_REJECTED = 0,
	AMS_STATE_TRANSITION_NO_CHANGE,
	AMS_STATE_TRANSITION_APPLIED
} ams_state_transition_result_t;

static inline bool ams_state_is_valid(state_t state)
{
	return (state >= STATE_NULL) && (state <= STATE_ERROR);
}

static inline bool ams_state_allows_bms_ok(state_t state)
{
	switch(state)
	{
	case STATE_CHARGE:
	case STATE_DISCARGE:
	case STATE_BALANCE:
		return true;
	case STATE_START:
	case STATE_NULL:
	case STATE_ERROR:
	default:
		return false;
	}
}

static inline bool ams_state_transition_allowed(
	state_t previous,
	state_t target,
	ams_state_transition_reason_t reason)
{
	if(!ams_state_is_valid(previous) || !ams_state_is_valid(target))
	{
		return false;
	}

	switch(reason)
	{
	case AMS_STATE_TRANSITION_STARTUP_READY:
		return (previous == STATE_START) && (target == STATE_DISCARGE);

	case AMS_STATE_TRANSITION_SERVICE_COMMAND:
		return ((previous == STATE_START) ||
		        (previous == STATE_CHARGE) ||
		        (previous == STATE_DISCARGE) ||
		        (previous == STATE_BALANCE)) &&
		       ((target == STATE_CHARGE) ||
		        (target == STATE_DISCARGE));

	case AMS_STATE_TRANSITION_CORRUPT_CURRENT_STATE:
	case AMS_STATE_TRANSITION_INVALID_REQUEST:
		return target == STATE_ERROR;

	case AMS_STATE_TRANSITION_BOOT:
	default:
		return false;
	}
}

typedef enum
{
	AMS_HEARTBEAT_ADBMS = 0,
	AMS_HEARTBEAT_CURRENT,
	AMS_HEARTBEAT_TEMP,
	AMS_HEARTBEAT_CAN,
	AMS_HEARTBEAT_LOGGER,
	AMS_HEARTBEAT_IMD,
	AMS_HEARTBEAT_FAN,
	AMS_HEARTBEAT_ESTIMATOR,
	AMS_HEARTBEAT_COUNT
} ams_heartbeat_id_t;


typedef enum
{
    AMS_RTOS_TASK_ERROR = 0,
    AMS_RTOS_TASK_CURRENT,
    AMS_RTOS_TASK_ADBMS,
    AMS_RTOS_TASK_CAN,
    AMS_RTOS_TASK_ESTIMATOR,
    AMS_RTOS_TASK_FAN,
    AMS_RTOS_TASK_AIR,
    AMS_RTOS_TASK_IMD,
    AMS_RTOS_TASK_CLI,
    AMS_RTOS_TASK_COUNT
} ams_rtos_task_id_t;

#define AMS_RTOS_TASK_BIT(id) ((uint16_t)(1u << (uint16_t)(id)))

typedef enum
{
    AMS_RTOS_FAULT_NONE = 0,
    AMS_RTOS_FAULT_STACK_OVERFLOW,
    AMS_RTOS_FAULT_MALLOC_FAILED,
    AMS_RTOS_FAULT_ASSERT_FAILED
} ams_rtos_fault_reason_t;

#define AMS_RTOS_FAULT_FLAG_STACK_OVERFLOW (1u << 0u)
#define AMS_RTOS_FAULT_FLAG_MALLOC_FAILED  (1u << 1u)
#define AMS_RTOS_FAULT_FLAG_ASSERT_FAILED  (1u << 2u)
#define AMS_RTOS_FAULT_FLAG_LOW_STACK_WARN (1u << 3u)
#define AMS_RTOS_FAULT_FLAG_LOW_HEAP_WARN  (1u << 4u)

#define AMS_HEARTBEAT_BIT(id) ((uint16_t)(1u << (uint16_t)(id)))
#define AMS_HEARTBEAT_SAFETY_MASK (AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_ADBMS) | \
                                   AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_CURRENT) | \
                                   AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_TEMP) | \
                                   AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_CAN) | \
                                   AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_FAN) | \
                                   (AMS_SOP_AUTHORITY_REQUIRED ? AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_ESTIMATOR) : 0u) | \
                                   (AMS_ENABLE_IMD ? AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_IMD) : 0u))
#define AMS_HEARTBEAT_LOGGER_MASK AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_LOGGER)

typedef struct
{
	uint32_t boot_tick;
	uint32_t last_tick[AMS_HEARTBEAT_COUNT];
	uint32_t count[AMS_HEARTBEAT_COUNT];
	uint16_t seen_mask;
	uint16_t stale_mask;
	uint16_t safety_stale_mask;
	uint16_t logger_stale_mask;
} ams_heartbeat_monitor_t;

struct app_data_t
{
	float total_voltage;
	float max_voltage;
	float min_voltage;
	float max_temp;
	float avg_temp;
	float current;
	bool current_valid;
	uint32_t current_sample_tick;
	uint32_t current_sample_sequence;
	current_sensor_range_t current_selected_range;
	current_sensor_reason_t current_meas_reason;

	bool hard_fault;
	bool soft_fault;

	uint32_t reset_flags;
	uint32_t last_panic_reason;
	uint32_t safety_panic_count;

	bool watchdog_runtime_enabled;
	bool watchdog_hw_started;
	uint32_t watchdog_feed_count;
	uint32_t watchdog_block_count;
	uint32_t watchdog_last_feed_tick;
	uint32_t watchdog_last_block_reason;
	uint32_t watchdog_last_logged_block_reason;

	uint32_t can_error_code;
	uint32_t can_busoff_count;
	uint32_t can_error_count;
	uint32_t can_recover_count;
	uint32_t can_last_error_tick;
	bool can_busoff_fault;
	bool can_recover_pending;
	/* Single-writer CAN-task diagnostics.  Keep safety/charger command,
	 * compact ECU heartbeat, and best-effort detail failures distinguishable. */
	uint32_t can_tx_critical_attempt_count;
	uint32_t can_tx_critical_fail_count;
	uint32_t can_tx_compact_bundle_count;
	uint32_t can_tx_compact_bundle_fail_count;
	uint32_t can_tx_detail_phase_count;
	uint32_t can_tx_detail_phase_fail_count;
	uint32_t can_tx_detail_suppressed_count;
	uint32_t can_task_cycle_count;
	uint32_t can_task_deadline_miss_count;
	uint32_t can_task_last_duration_ms;
	uint32_t can_task_max_duration_ms;

    uint32_t rtos_heap_free_bytes;
    uint32_t rtos_heap_min_ever_free_bytes;
    uint32_t rtos_malloc_fail_count;
    uint32_t rtos_stack_overflow_count;
    uint32_t rtos_assert_fail_count;
    uint32_t rtos_last_assert_line;
    uint32_t rtos_last_fault_tick;
    uint16_t rtos_stack_high_water_words[AMS_RTOS_TASK_COUNT];
    uint16_t rtos_stack_config_words[AMS_RTOS_TASK_COUNT];
    uint16_t rtos_min_stack_high_water_words;
    uint16_t rtos_stack_warn_mask;
    uint16_t rtos_fault_flags;
    uint8_t rtos_last_fault_reason;
    uint8_t rtos_last_fault_task;
    bool rtos_fault;
    bool rtos_stack_warning;
    bool rtos_heap_warning;

	bool fan_fault;
	bool cli_fault;
	bool canbus_fault;
	bool current_fault;
	bool current_sensor_fault;
	bool current_overcurrent_warning;
	bool current_overcurrent_pending;
	bool current_overcurrent_fault;
	bool current_fault_latched;
	current_fault_reason_t current_fault_reason;
	current_fault_reason_t current_fault_latched_reason;
	current_fault_mode_t current_fault_mode;
	current_fault_state_t current_fault_state;
	bool fuse_fault;
	bool temp_fault;
	bool temp_valid;
	bool temp_read_fault;
	bool temp_warning;
	bool temp_fan_max;
	bool temp_charge_stop;
	bool temp_overtemp_pending;
	bool overtemp_fault;
	bool severe_overtemp_fault;
	bool temp_fault_latched;
	temperature_fault_reason_t temp_fault_reason;
	temperature_fault_reason_t temp_fault_pending_reason;
	temperature_fault_reason_t temp_fault_latched_reason;
	temperature_fault_state_t temp_fault_state;
	uint32_t temp_fault_pending_ms;
	uint16_t temp_usable_sensor_count;
	uint16_t temp_updated_sensor_count;
	uint16_t temp_stale_sensor_count;
	uint16_t temp_invalid_sensor_count;
	uint16_t temp_open_sensor_count;
	uint16_t temp_short_sensor_count;
	uint16_t temp_jump_sensor_count;
	uint16_t temp_rate_rise_sensor_count;
	float temp_filtered_max;
	float temp_filtered_avg;
	float temp_max_rate_c_per_s;
	uint8_t max_temp_seg;
	uint8_t max_temp_sensor;
	uint8_t min_temp_seg;
	uint8_t min_temp_sensor;
	uint8_t temp_max_rate_seg;
	uint8_t temp_max_rate_sensor;
	bool voltage_fault;
	bool voltage_valid;
	bool voltage_read_fault;
	bool voltage_warning;
	bool charge_voltage_stop;
	bool overvoltage_fault;
	bool undervoltage_fault;
	bool voltage_fault_latched;
	voltage_fault_reason_t voltage_fault_reason;
	voltage_fault_reason_t voltage_fault_latched_reason;
	voltage_fault_state_t voltage_fault_state;
	uint16_t voltage_usable_cell_count;
	uint16_t voltage_updated_cell_count;
	uint16_t voltage_stale_cell_count;
	uint16_t voltage_pec_fail_cell_count;
	uint16_t voltage_jump_cell_count;
	uint16_t voltage_stuck_cell_count;
	uint16_t voltage_max_delta_mv;
	uint8_t max_voltage_seg;
	uint8_t max_voltage_cell;
	uint8_t min_voltage_seg;
	uint8_t min_voltage_cell;
	uint8_t voltage_max_delta_seg;
	uint8_t voltage_max_delta_cell;
	bool estimator_fault;
	bool power_limit_fault;

	/* Legacy telemetry name: this is the conditioned AIR_CONTROL_MCU voltage
	 * sense, not proof of AIR+, AIR- or precharge contact position. */
	bool air_state;
	ams_air_monitor_t air_monitor;
	bool imd_ok;
	bool imd_valid;
	bool imd_fault;
	imd_status_t imd_status;
	uint32_t imd_last_valid_tick;
    bool fan_state;
    float fan_command_percent;
    uint8_t fan_control_reason;
    uint32_t fan_set_fail_count;
    uint32_t fan_last_update_tick;

	bool charger_fault;
	bool adbms_diag_fault;
	bool adbms_config_fault;
	bool adbms_status_fault;
	/* Safety latch: a failed/incomplete open-wire diagnostic remains asserted
	 * until reset. A later pass may restore measurement freshness, but cannot
	 * silently restore BMS permission in the same boot. */
	bool adbms_open_wire_fault;
	bool adbms_balance_write_fault;
	bool adbms_scan_active;
	bool adbms_balance_active;
	uint32_t adbms_scan_count;
	uint32_t adbms_status_diag_count;
	uint32_t adbms_config_diag_count;
	uint32_t adbms_open_wire_diag_count;
	uint32_t adbms_balance_write_fail_count;
	uint32_t adbms_balance_recovery_count;
	uint32_t adbms_scan_deadline_miss_count;
	uint32_t adbms_last_scan_duration_ms;
	uint32_t adbms_max_scan_duration_ms;
	uint32_t adbms_last_schedule_interval_ms;
	uint32_t adbms_last_balance_on_ms;
	uint32_t adbms_last_balance_off_ms;
	uint32_t adbms_balance_apply_tick;
	HAL_StatusTypeDef adbms_last_diag_status;
	bool task_heartbeat_fault;
	bool logger_heartbeat_fault;
	uint16_t heartbeat_stale_mask;
	uint16_t heartbeat_seen_mask;
    bool bms_state;
    bool bms_output_inhibit;
    bool bms_supervisor_ready;
    uint32_t bms_output_block_count;
	bool balance_inhibit;

	state_t state;
	state_t state_previous;
	ams_state_transition_reason_t state_transition_reason;
	uint32_t state_transition_count;
	uint32_t state_transition_last_tick;
	bool state_transition_in_progress;

	board_t board;
	accumulator_t acc;
	ams_estimator_t estimator;
	ams_power_state_t power_state;
	ams_power_can_snapshot_t power_can_snapshot;
	ams_mission_request_state_t mission_request;
	ams_current_window_accumulator_t current_window;
	ams_measurement_store_t measurement_store;
	ams_hil_input_t hil;
	ams_heartbeat_monitor_t heartbeat;

	TaskHandle_t fan_task;
	TaskHandle_t cli_task;
	TaskHandle_t error_task;
	TaskHandle_t canbus_task;
	TaskHandle_t air_task;
	TaskHandle_t imd_task;
	TaskHandle_t current_task;
	TaskHandle_t adbms_task;
	TaskHandle_t estimator_task;
};

/* Begin a mode transition while the caller owns a short RTOS critical
 * section.  The in-progress gate is intentionally set before any potentially
 * blocking exit cleanup.  The safety supervisor therefore cannot reassert
 * BMS_OK while balancing is being cleared or a charger shutdown is pending.
 *
 * A corrupted current state or invalid target is contained as STATE_ERROR.
 * Leaving charge (or entering ERROR conservatively) schedules repeated
 * zero-demand charger-disable frames through the CAN task, the sole CAN TX
 * owner.  Call ams_state_transition_finish() only after synchronous local
 * cleanup has completed. */
static inline ams_state_transition_result_t
ams_state_transition_begin(app_data_t *data,
                           state_t requested,
                           ams_state_transition_reason_t reason,
                           uint32_t now)
{
	state_t previous;
	state_t target;

	if(data == NULL)
	{
		return AMS_STATE_TRANSITION_REJECTED;
	}

	previous = data->state;
	target = requested;

	if(!ams_state_is_valid(previous))
	{
		target = STATE_ERROR;
		reason = AMS_STATE_TRANSITION_CORRUPT_CURRENT_STATE;
	}
	else if(!ams_state_is_valid(target))
	{
		target = STATE_ERROR;
		reason = AMS_STATE_TRANSITION_INVALID_REQUEST;
	}
	else if(!ams_state_transition_allowed(previous, target, reason))
	{
		return AMS_STATE_TRANSITION_REJECTED;
	}

	data->state_transition_in_progress = true;
	data->bms_supervisor_ready = false;

	if(previous == target)
	{
		return AMS_STATE_TRANSITION_NO_CHANGE;
	}

	data->state_previous = previous;
	data->state = target;
	data->state_transition_reason = reason;
	data->state_transition_last_tick = now;
	if(data->state_transition_count != UINT32_MAX)
	{
		data->state_transition_count++;
	}

	if((previous == STATE_CHARGE) ||
	   (target == STATE_ERROR) ||
	   !ams_state_is_valid(previous))
	{
		charger_t *charger = &data->board.charger;
		charger->shutdown_pending = true;
		charger->shutdown_frames_remaining = CHARGER_EXIT_DISABLE_FRAMES;
		charger->disable_reason_mask |= CHARGER_DISABLE_REASON_STATE_EXIT;
		if(charger->shutdown_request_count != UINT32_MAX)
		{
			charger->shutdown_request_count++;
		}
		/* Keep every BMS_OK-capable destination inhibited until the CAN
		 * owner has queued the complete shutdown burst. */
		data->charger_fault = true;
	}

	return AMS_STATE_TRANSITION_APPLIED;
}

static inline void ams_state_transition_finish(app_data_t *data)
{
	if(data != NULL)
	{
		data->state_transition_in_progress = false;
	}
}

static inline const char *ams_state_transition_reason_str(
	ams_state_transition_reason_t reason)
{
	switch(reason)
	{
	case AMS_STATE_TRANSITION_BOOT: return "boot";
	case AMS_STATE_TRANSITION_STARTUP_READY: return "startup_ready";
	case AMS_STATE_TRANSITION_SERVICE_COMMAND: return "service";
	case AMS_STATE_TRANSITION_CORRUPT_CURRENT_STATE: return "corrupt_state";
	case AMS_STATE_TRANSITION_INVALID_REQUEST: return "invalid_request";
	default: return "unknown";
	}
}

void app_create(void);
void set_bms(bool state);
void adbms_spi_lock(void);
void adbms_spi_unlock(void);
void ams_current_window_lock(void);
void ams_current_window_unlock(void);
void ams_heartbeat_init(app_data_t *data, uint32_t now);
void ams_heartbeat_kick(app_data_t *data, ams_heartbeat_id_t id, uint32_t now);
uint16_t ams_heartbeat_update(app_data_t *data, uint32_t now);
uint32_t ams_heartbeat_timeout_ms(ams_heartbeat_id_t id);
const char *ams_heartbeat_name(ams_heartbeat_id_t id);

#endif /* INC_APP_H_ */
