#ifndef AMS_AIR_MONITOR_H_
#define AMS_AIR_MONITOR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Contactor feedback is intentionally separate from AIR_CONTROL_MCU.
 * AIR_CONTROL_MCU only senses voltage on the existing common AIR control net;
 * it does not prove that any main power contact opened or closed.
 *
 * The present PCB has no AIR_POS_AUX, AIR_NEG_AUX or PRECHARGE_AUX inputs. This
 * module is therefore a hardware-independent, fail-closed evaluator for the
 * future revision. It deliberately contains no pin mapping, input polarity or
 * generic timing defaults. The board adapter must provide classified samples
 * and a configuration derived from the reviewed schematic, harness, contactor
 * data sheet and precharge design.
 */

#define AMS_AIR_PERMILLE_SCALE       1000u
#define AMS_AIR_MAX_INTERVAL_MS      0x7FFFFFFFu

typedef enum
{
    AMS_AIR_PHASE_UNKNOWN = 0,
    AMS_AIR_PHASE_OFF,
    AMS_AIR_PHASE_PRECHARGE,
    AMS_AIR_PHASE_RUN,
    AMS_AIR_PHASE_SHUTDOWN
} ams_air_phase_t;

typedef enum
{
    AMS_AIR_CONTACT_UNKNOWN = 0,
    AMS_AIR_CONTACT_OPEN,
    AMS_AIR_CONTACT_CLOSED,
    AMS_AIR_CONTACT_LINE_FAULT
} ams_air_contact_state_t;

typedef enum
{
    AMS_AIR_FAULT_NONE = 0,
    AMS_AIR_FAULT_FEATURE_DISABLED,
    AMS_AIR_FAULT_WAITING_FOR_INPUTS,
    AMS_AIR_FAULT_CONFIG_INVALID,
    AMS_AIR_FAULT_COMMAND_INVALID,
    AMS_AIR_FAULT_COMMAND_STALE,
    AMS_AIR_FAULT_INPUT_STALE,
    AMS_AIR_FAULT_VOLTAGE_STALE,
    AMS_AIR_FAULT_TRANSITION_PENDING,
    AMS_AIR_FAULT_BOOT_OPEN_NOT_VERIFIED,
    AMS_AIR_FAULT_INVALID_TRANSITION,
    AMS_AIR_FAULT_PRECHARGE_INCOMPLETE,
    AMS_AIR_FAULT_CONTACT_UNSTABLE,
    AMS_AIR_FAULT_POS_WELDED,
    AMS_AIR_FAULT_NEG_WELDED,
    AMS_AIR_FAULT_POS_DID_NOT_CLOSE,
    AMS_AIR_FAULT_NEG_DID_NOT_CLOSE,
    AMS_AIR_FAULT_PRECHARGE_WELDED,
    AMS_AIR_FAULT_PRECHARGE_DID_NOT_CLOSE,
    AMS_AIR_FAULT_CONTACT_DISAGREEMENT,
    AMS_AIR_FAULT_LINE_SUPERVISION,
    AMS_AIR_FAULT_BUS_VOLTAGE_PLAUSIBILITY,
    AMS_AIR_FAULT_PRECHARGE_TIMEOUT,
    /* Appended to preserve the numeric values of the existing diagnostics. */
    AMS_AIR_FAULT_REARM_REQUIRED,
    AMS_AIR_FAULT_SAMPLE_INCOHERENT
} ams_air_fault_reason_t;

typedef enum
{
    AMS_AIR_FAULT_BIT_CONFIG              = (1u << 0),
    AMS_AIR_FAULT_BIT_COMMAND             = (1u << 1),
    AMS_AIR_FAULT_BIT_FEEDBACK_STALE      = (1u << 2),
    AMS_AIR_FAULT_BIT_VOLTAGE_STALE       = (1u << 3),
    AMS_AIR_FAULT_BIT_BOOT_OPEN           = (1u << 4),
    AMS_AIR_FAULT_BIT_TRANSITION          = (1u << 5),
    AMS_AIR_FAULT_BIT_CONTACT_UNSTABLE    = (1u << 6),
    AMS_AIR_FAULT_BIT_POS_CONTACT         = (1u << 7),
    AMS_AIR_FAULT_BIT_NEG_CONTACT         = (1u << 8),
    AMS_AIR_FAULT_BIT_PRECHARGE_CONTACT   = (1u << 9),
    AMS_AIR_FAULT_BIT_CONTACT_DISAGREE    = (1u << 10),
    AMS_AIR_FAULT_BIT_LINE_SUPERVISION    = (1u << 11),
    AMS_AIR_FAULT_BIT_BUS_VOLTAGE         = (1u << 12),
    AMS_AIR_FAULT_BIT_PRECHARGE_TIMEOUT   = (1u << 13),
    AMS_AIR_FAULT_BIT_SAMPLE_INCOHERENT   = (1u << 14)
} ams_air_fault_bit_t;

typedef struct
{
    bool valid;
    /* Effective hardware phase after local shutdown permissions are applied,
     * not merely a remote controller's requested phase. If BMS_OK/shutdown is
     * forced low, the adapter must report OFF or SHUTDOWN as appropriate. */
    ams_air_phase_t phase;
    uint32_t update_tick;
} ams_air_command_sample_t;

typedef struct
{
    bool valid;
    ams_air_contact_state_t state;
    uint32_t update_tick;
} ams_air_contact_sample_t;

typedef struct
{
    bool valid;
    uint32_t millivolts;
    uint32_t update_tick;
} ams_air_voltage_sample_t;

typedef struct
{
    uint32_t now_tick;
    ams_air_command_sample_t command;
    ams_air_contact_sample_t pos_aux;
    ams_air_contact_sample_t neg_aux;
    ams_air_contact_sample_t precharge_aux;
    ams_air_voltage_sample_t pack_voltage;
    ams_air_voltage_sample_t load_voltage;
} ams_air_monitor_inputs_t;

typedef struct
{
    uint32_t command_timeout_ms;
    uint32_t contact_sample_timeout_ms;
    uint32_t voltage_sample_timeout_ms;
    uint32_t max_sample_skew_ms;
    uint32_t debounce_ms;

    uint32_t pos_make_timeout_ms;
    uint32_t pos_release_timeout_ms;
    uint32_t neg_make_timeout_ms;
    uint32_t neg_release_timeout_ms;
    uint32_t precharge_make_timeout_ms;
    uint32_t precharge_release_timeout_ms;

    uint32_t precharge_max_ms;
    uint32_t run_voltage_settle_ms;
    uint32_t bus_discharge_timeout_ms;

    uint32_t minimum_pack_voltage_mv;
    uint32_t open_bus_max_mv;
    uint16_t precharge_complete_min_permille;
    uint16_t run_bus_min_permille;
    uint16_t run_bus_max_permille;

    bool require_precharge_aux;
    bool require_bus_voltage;
} ams_air_monitor_config_t;

typedef struct
{
    bool candidate_valid;
    bool debounced_valid;
    ams_air_contact_state_t candidate;
    ams_air_contact_state_t debounced;
    uint32_t candidate_since_tick;
} ams_air_debounce_state_t;

typedef struct
{
    bool feature_enabled;
    bool configuration_valid;
    bool command_valid;
    bool feedback_valid;
    bool voltage_valid;
    bool steady_state_valid;
    bool transition_pending;
    bool transition_authorized;
    bool boot_open_verified;
    bool precharge_complete;
    bool bus_plausible;
    bool contact_disagreement;
    bool permit;
    bool fault;
    bool fault_latched;

    ams_air_phase_t phase;
    ams_air_phase_t previous_phase;
    ams_air_contact_state_t pos_aux;
    ams_air_contact_state_t neg_aux;
    ams_air_contact_state_t precharge_aux;
    ams_air_fault_reason_t reason;
    ams_air_fault_reason_t latched_reason;

    uint32_t active_fault_mask;
    uint32_t latched_fault_mask;
    uint32_t last_update_tick;
    uint32_t phase_start_tick;
    bool phase_initialized;

    ams_air_debounce_state_t pos_filter;
    ams_air_debounce_state_t neg_filter;
    ams_air_debounce_state_t precharge_filter;
} ams_air_monitor_t;

static inline void ams_air_monitor_init(ams_air_monitor_t *monitor,
                                        bool feature_enabled)
{
    if(monitor == NULL)
    {
        return;
    }

    *monitor = (ams_air_monitor_t){0};
    monitor->feature_enabled = feature_enabled;
    monitor->phase = AMS_AIR_PHASE_UNKNOWN;
    monitor->previous_phase = AMS_AIR_PHASE_UNKNOWN;
    monitor->pos_aux = AMS_AIR_CONTACT_UNKNOWN;
    monitor->neg_aux = AMS_AIR_CONTACT_UNKNOWN;
    monitor->precharge_aux = AMS_AIR_CONTACT_UNKNOWN;
    monitor->reason = feature_enabled ?
                      AMS_AIR_FAULT_WAITING_FOR_INPUTS :
                      AMS_AIR_FAULT_FEATURE_DISABLED;
    monitor->fault = feature_enabled;
}

static inline bool ams_air_monitor_ready(const ams_air_monitor_t *monitor)
{
    return (monitor != NULL) &&
           monitor->feature_enabled &&
           monitor->configuration_valid &&
           monitor->command_valid &&
           monitor->feedback_valid &&
           monitor->boot_open_verified &&
           ((monitor->phase == AMS_AIR_PHASE_OFF) ||
            (monitor->phase == AMS_AIR_PHASE_PRECHARGE) ||
            (monitor->phase == AMS_AIR_PHASE_RUN)) &&
           monitor->transition_authorized &&
           (monitor->steady_state_valid || monitor->transition_pending) &&
           monitor->permit &&
           !monitor->fault &&
           !monitor->fault_latched &&
           (monitor->active_fault_mask == 0u) &&
           (monitor->latched_fault_mask == 0u);
}

bool ams_air_monitor_config_valid(const ams_air_monitor_config_t *config);

/* Validate that task evaluation and supervisor publication-liveness timing
 * cannot outlive any safety-relevant input or contact-transition deadline. */
bool ams_air_monitor_schedule_valid(const ams_air_monitor_config_t *config,
                                    uint32_t evaluation_period_ms,
                                    uint32_t publication_timeout_ms);

void ams_air_monitor_step(ams_air_monitor_t *monitor,
                          const ams_air_monitor_config_t *config,
                          const ams_air_monitor_inputs_t *inputs);

/*
 * Explicit controlled-clear request. The function re-evaluates the supplied
 * inputs and only clears latches in OFF/SHUTDOWN after all required contacts
 * are stably open and the load-side bus is safe when voltage proof is enabled.
 */
bool ams_air_monitor_request_clear(ams_air_monitor_t *monitor,
                                   const ams_air_monitor_config_t *config,
                                   const ams_air_monitor_inputs_t *inputs);

static inline const char *ams_air_phase_str(ams_air_phase_t phase)
{
    switch(phase)
    {
    case AMS_AIR_PHASE_OFF:       return "off";
    case AMS_AIR_PHASE_PRECHARGE: return "precharge";
    case AMS_AIR_PHASE_RUN:       return "run";
    case AMS_AIR_PHASE_SHUTDOWN:  return "shutdown";
    case AMS_AIR_PHASE_UNKNOWN:
    default:                      return "unknown";
    }
}

static inline const char *ams_air_contact_state_str(ams_air_contact_state_t state)
{
    switch(state)
    {
    case AMS_AIR_CONTACT_OPEN:       return "open";
    case AMS_AIR_CONTACT_CLOSED:     return "closed";
    case AMS_AIR_CONTACT_LINE_FAULT: return "line_fault";
    case AMS_AIR_CONTACT_UNKNOWN:
    default:                         return "unknown";
    }
}

static inline const char *ams_air_fault_reason_str(ams_air_fault_reason_t reason)
{
    switch(reason)
    {
    case AMS_AIR_FAULT_NONE:                    return "none";
    case AMS_AIR_FAULT_FEATURE_DISABLED:        return "feature_disabled";
    case AMS_AIR_FAULT_WAITING_FOR_INPUTS:      return "waiting_for_inputs";
    case AMS_AIR_FAULT_CONFIG_INVALID:          return "config_invalid";
    case AMS_AIR_FAULT_COMMAND_INVALID:         return "command_invalid";
    case AMS_AIR_FAULT_COMMAND_STALE:           return "command_stale";
    case AMS_AIR_FAULT_INPUT_STALE:             return "input_stale";
    case AMS_AIR_FAULT_VOLTAGE_STALE:           return "voltage_stale";
    case AMS_AIR_FAULT_TRANSITION_PENDING:      return "transition_pending";
    case AMS_AIR_FAULT_BOOT_OPEN_NOT_VERIFIED:  return "boot_open_not_verified";
    case AMS_AIR_FAULT_INVALID_TRANSITION:      return "invalid_transition";
    case AMS_AIR_FAULT_PRECHARGE_INCOMPLETE:    return "precharge_incomplete";
    case AMS_AIR_FAULT_CONTACT_UNSTABLE:        return "contact_unstable";
    case AMS_AIR_FAULT_POS_WELDED:              return "air_pos_welded";
    case AMS_AIR_FAULT_NEG_WELDED:              return "air_neg_welded";
    case AMS_AIR_FAULT_POS_DID_NOT_CLOSE:       return "air_pos_did_not_close";
    case AMS_AIR_FAULT_NEG_DID_NOT_CLOSE:       return "air_neg_did_not_close";
    case AMS_AIR_FAULT_PRECHARGE_WELDED:        return "precharge_welded";
    case AMS_AIR_FAULT_PRECHARGE_DID_NOT_CLOSE: return "precharge_did_not_close";
    case AMS_AIR_FAULT_CONTACT_DISAGREEMENT:    return "contact_disagreement";
    case AMS_AIR_FAULT_LINE_SUPERVISION:         return "line_supervision";
    case AMS_AIR_FAULT_BUS_VOLTAGE_PLAUSIBILITY: return "bus_voltage_plausibility";
    case AMS_AIR_FAULT_PRECHARGE_TIMEOUT:         return "precharge_timeout";
    case AMS_AIR_FAULT_REARM_REQUIRED:            return "rearm_required";
    case AMS_AIR_FAULT_SAMPLE_INCOHERENT:          return "sample_incoherent";
    default:                                      return "unknown";
    }
}

#endif /* AMS_AIR_MONITOR_H_ */
