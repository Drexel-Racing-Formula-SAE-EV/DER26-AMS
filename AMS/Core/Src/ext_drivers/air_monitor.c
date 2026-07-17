#include "ext_drivers/air_monitor.h"

typedef struct
{
    ams_air_contact_state_t pos;
    ams_air_contact_state_t neg;
    ams_air_contact_state_t precharge;
} ams_air_expected_contacts_t;

static uint32_t air_elapsed(uint32_t now, uint32_t then)
{
    return (uint32_t)(now - then);
}

static bool air_interval_valid(uint32_t interval)
{
    return (interval > 0u) && (interval <= AMS_AIR_MAX_INTERVAL_MS);
}

static bool air_phase_valid(ams_air_phase_t phase)
{
    return (phase == AMS_AIR_PHASE_OFF) ||
           (phase == AMS_AIR_PHASE_PRECHARGE) ||
           (phase == AMS_AIR_PHASE_RUN) ||
           (phase == AMS_AIR_PHASE_SHUTDOWN);
}

static bool air_contact_sample_state_valid(ams_air_contact_state_t state)
{
    return (state == AMS_AIR_CONTACT_OPEN) ||
           (state == AMS_AIR_CONTACT_CLOSED) ||
           (state == AMS_AIR_CONTACT_LINE_FAULT);
}

static bool air_sample_fresh(bool valid,
                             uint32_t now,
                             uint32_t update_tick,
                             uint32_t timeout)
{
    return valid && (air_elapsed(now, update_tick) <= timeout);
}

static void air_filter_update(ams_air_debounce_state_t *filter,
                              ams_air_contact_state_t sample,
                              uint32_t now,
                              uint32_t debounce_ms)
{
    if(filter == NULL)
    {
        return;
    }

    if(!filter->candidate_valid || (filter->candidate != sample))
    {
        filter->candidate_valid = true;
        filter->candidate = sample;
        filter->candidate_since_tick = now;
    }

    if((debounce_ms == 0u) ||
       (air_elapsed(now, filter->candidate_since_tick) >= debounce_ms))
    {
        filter->debounced = filter->candidate;
        filter->debounced_valid = true;
    }
}

static bool air_transition_allowed(ams_air_phase_t from,
                                   ams_air_phase_t to,
                                   bool boot_open_verified)
{
    if(from == to)
    {
        return true;
    }

    /* A shutdown/open request is always allowed. */
    if((to == AMS_AIR_PHASE_OFF) || (to == AMS_AIR_PHASE_SHUTDOWN))
    {
        return true;
    }

    if(!boot_open_verified)
    {
        return false;
    }

    if(((from == AMS_AIR_PHASE_OFF) || (from == AMS_AIR_PHASE_SHUTDOWN)) &&
       (to == AMS_AIR_PHASE_PRECHARGE))
    {
        return true;
    }

    return (from == AMS_AIR_PHASE_PRECHARGE) &&
           (to == AMS_AIR_PHASE_RUN);
}

static ams_air_expected_contacts_t air_expected_contacts(ams_air_phase_t phase)
{
    ams_air_expected_contacts_t expected = {
        AMS_AIR_CONTACT_OPEN,
        AMS_AIR_CONTACT_OPEN,
        AMS_AIR_CONTACT_OPEN
    };

    if(phase == AMS_AIR_PHASE_PRECHARGE)
    {
        expected.neg = AMS_AIR_CONTACT_CLOSED;
        expected.precharge = AMS_AIR_CONTACT_CLOSED;
    }
    else if(phase == AMS_AIR_PHASE_RUN)
    {
        expected.pos = AMS_AIR_CONTACT_CLOSED;
        expected.neg = AMS_AIR_CONTACT_CLOSED;
    }

    return expected;
}

static uint32_t air_contact_timeout(const ams_air_monitor_config_t *config,
                                    unsigned int contact,
                                    ams_air_contact_state_t expected)
{
    bool closing = (expected == AMS_AIR_CONTACT_CLOSED);

    if(contact == 0u)
    {
        return closing ? config->pos_make_timeout_ms :
                         config->pos_release_timeout_ms;
    }
    if(contact == 1u)
    {
        return closing ? config->neg_make_timeout_ms :
                         config->neg_release_timeout_ms;
    }
    return closing ? config->precharge_make_timeout_ms :
                     config->precharge_release_timeout_ms;
}

static ams_air_fault_reason_t air_contact_fault_reason(
    unsigned int contact,
    ams_air_contact_state_t expected)
{
    bool expected_open = (expected == AMS_AIR_CONTACT_OPEN);

    if(contact == 0u)
    {
        return expected_open ? AMS_AIR_FAULT_POS_WELDED :
                               AMS_AIR_FAULT_POS_DID_NOT_CLOSE;
    }
    if(contact == 1u)
    {
        return expected_open ? AMS_AIR_FAULT_NEG_WELDED :
                               AMS_AIR_FAULT_NEG_DID_NOT_CLOSE;
    }
    return expected_open ? AMS_AIR_FAULT_PRECHARGE_WELDED :
                           AMS_AIR_FAULT_PRECHARGE_DID_NOT_CLOSE;
}

static uint32_t air_contact_fault_bit(unsigned int contact)
{
    if(contact == 0u)
    {
        return AMS_AIR_FAULT_BIT_POS_CONTACT;
    }
    if(contact == 1u)
    {
        return AMS_AIR_FAULT_BIT_NEG_CONTACT;
    }
    return AMS_AIR_FAULT_BIT_PRECHARGE_CONTACT;
}

static void air_mark_fault(ams_air_monitor_t *monitor,
                           uint32_t bit,
                           ams_air_fault_reason_t reason,
                           bool latch)
{
    monitor->active_fault_mask |= bit;
    if((monitor->reason == AMS_AIR_FAULT_NONE) ||
       (monitor->reason == AMS_AIR_FAULT_WAITING_FOR_INPUTS) ||
       (monitor->reason == AMS_AIR_FAULT_TRANSITION_PENDING))
    {
        monitor->reason = reason;
    }

    if(latch)
    {
        monitor->latched_fault_mask |= bit;
        if(monitor->latched_reason == AMS_AIR_FAULT_NONE)
        {
            monitor->latched_reason = reason;
        }
    }
}

static void air_finalize(ams_air_monitor_t *monitor)
{
    monitor->fault_latched = (monitor->latched_fault_mask != 0u);
    monitor->fault = (monitor->active_fault_mask != 0u) ||
                     monitor->fault_latched;

    if(monitor->active_fault_mask != 0u)
    {
        monitor->permit = false;
    }
    else if(monitor->fault_latched)
    {
        monitor->reason = monitor->latched_reason;
        monitor->permit = false;
    }
}

static bool air_ratio_at_least(uint32_t numerator_mv,
                               uint32_t denominator_mv,
                               uint16_t threshold_permille)
{
    return ((uint64_t)numerator_mv * AMS_AIR_PERMILLE_SCALE) >=
           ((uint64_t)denominator_mv * threshold_permille);
}

static bool air_ratio_at_most(uint32_t numerator_mv,
                              uint32_t denominator_mv,
                              uint16_t threshold_permille)
{
    return ((uint64_t)numerator_mv * AMS_AIR_PERMILLE_SCALE) <=
           ((uint64_t)denominator_mv * threshold_permille);
}

bool ams_air_monitor_config_valid(const ams_air_monitor_config_t *config)
{
    if(config == NULL)
    {
        return false;
    }

    if(!air_interval_valid(config->command_timeout_ms) ||
       !air_interval_valid(config->contact_sample_timeout_ms) ||
       (config->debounce_ms > AMS_AIR_MAX_INTERVAL_MS) ||
       !air_interval_valid(config->pos_make_timeout_ms) ||
       !air_interval_valid(config->pos_release_timeout_ms) ||
       !air_interval_valid(config->neg_make_timeout_ms) ||
       !air_interval_valid(config->neg_release_timeout_ms) ||
       !air_interval_valid(config->precharge_max_ms))
    {
        return false;
    }

    if((config->debounce_ms > config->pos_make_timeout_ms) ||
       (config->debounce_ms > config->pos_release_timeout_ms) ||
       (config->debounce_ms > config->neg_make_timeout_ms) ||
       (config->debounce_ms > config->neg_release_timeout_ms))
    {
        return false;
    }

    if(config->require_precharge_aux)
    {
        if(!air_interval_valid(config->precharge_make_timeout_ms) ||
           !air_interval_valid(config->precharge_release_timeout_ms) ||
           (config->debounce_ms > config->precharge_make_timeout_ms) ||
           (config->debounce_ms > config->precharge_release_timeout_ms))
        {
            return false;
        }
    }

    if((config->precharge_max_ms < config->pos_release_timeout_ms) ||
       (config->precharge_max_ms < config->neg_make_timeout_ms) ||
       (config->require_precharge_aux &&
        (config->precharge_max_ms < config->precharge_make_timeout_ms)))
    {
        return false;
    }

    if(config->require_bus_voltage)
    {
        if(!air_interval_valid(config->voltage_sample_timeout_ms) ||
           !air_interval_valid(config->run_voltage_settle_ms) ||
           !air_interval_valid(config->bus_discharge_timeout_ms) ||
           (config->minimum_pack_voltage_mv == 0u) ||
           (config->open_bus_max_mv == 0u) ||
           (config->open_bus_max_mv >= config->minimum_pack_voltage_mv) ||
           (config->precharge_complete_min_permille == 0u) ||
           (config->precharge_complete_min_permille > AMS_AIR_PERMILLE_SCALE) ||
           (config->run_bus_min_permille == 0u) ||
           (config->run_bus_min_permille > AMS_AIR_PERMILLE_SCALE) ||
           (config->run_bus_min_permille > config->run_bus_max_permille) ||
           (config->precharge_complete_min_permille > config->run_bus_max_permille) ||
           (config->run_bus_max_permille > 2000u))
        {
            return false;
        }
    }

    return true;
}

void ams_air_monitor_step(ams_air_monitor_t *monitor,
                          const ams_air_monitor_config_t *config,
                          const ams_air_monitor_inputs_t *inputs)
{
    ams_air_expected_contacts_t expected;
    ams_air_contact_state_t actual[3];
    ams_air_contact_state_t wanted[3];
    bool required[3];
    bool contacts_match = true;
    bool voltage_phase_ok = true;
    bool prior_permit;
    bool prior_steady_state;
    bool prior_precharge_complete;
    uint32_t phase_age;

    if(monitor == NULL)
    {
        return;
    }

    if(!monitor->feature_enabled)
    {
        ams_air_monitor_init(monitor, false);
        return;
    }

    prior_permit = monitor->permit && !monitor->fault;
    prior_steady_state = monitor->steady_state_valid;
    prior_precharge_complete = monitor->precharge_complete;
    monitor->configuration_valid = false;
    monitor->command_valid = false;
    monitor->feedback_valid = false;
    monitor->voltage_valid = false;
    monitor->steady_state_valid = false;
    monitor->transition_pending = false;
    monitor->precharge_complete = false;
    monitor->bus_plausible = false;
    monitor->contact_disagreement = false;
    monitor->permit = false;
    monitor->fault = false;
    monitor->active_fault_mask = 0u;
    monitor->reason = AMS_AIR_FAULT_NONE;

    if((inputs == NULL) || !ams_air_monitor_config_valid(config))
    {
        air_mark_fault(monitor,
                       AMS_AIR_FAULT_BIT_CONFIG,
                       AMS_AIR_FAULT_CONFIG_INVALID,
                       false);
        air_finalize(monitor);
        return;
    }

    monitor->configuration_valid = true;
    monitor->last_update_tick = inputs->now_tick;

    if(!inputs->command.valid || !air_phase_valid(inputs->command.phase))
    {
        air_mark_fault(monitor,
                       AMS_AIR_FAULT_BIT_COMMAND,
                       AMS_AIR_FAULT_COMMAND_INVALID,
                       false);
        air_finalize(monitor);
        return;
    }
    if(!air_sample_fresh(true,
                         inputs->now_tick,
                         inputs->command.update_tick,
                         config->command_timeout_ms))
    {
        air_mark_fault(monitor,
                       AMS_AIR_FAULT_BIT_COMMAND,
                       AMS_AIR_FAULT_COMMAND_STALE,
                       false);
        air_finalize(monitor);
        return;
    }
    monitor->command_valid = true;

    if(!monitor->phase_initialized)
    {
        monitor->phase_initialized = true;
        monitor->previous_phase = AMS_AIR_PHASE_UNKNOWN;
        monitor->phase = inputs->command.phase;
        monitor->phase_start_tick = inputs->now_tick;
        monitor->transition_authorized = false;
    }
    else if(monitor->phase != inputs->command.phase)
    {
        bool allowed = air_transition_allowed(monitor->phase,
                                              inputs->command.phase,
                                              monitor->boot_open_verified);
        ams_air_fault_reason_t transition_reason =
            AMS_AIR_FAULT_INVALID_TRANSITION;

        if((inputs->command.phase != AMS_AIR_PHASE_OFF) &&
           (inputs->command.phase != AMS_AIR_PHASE_SHUTDOWN) &&
           !prior_steady_state)
        {
            allowed = false;
        }

        if((monitor->phase == AMS_AIR_PHASE_PRECHARGE) &&
           (inputs->command.phase == AMS_AIR_PHASE_RUN) &&
           config->require_bus_voltage &&
           !prior_precharge_complete)
        {
            allowed = false;
            transition_reason = AMS_AIR_FAULT_PRECHARGE_INCOMPLETE;
        }

        monitor->previous_phase = monitor->phase;
        monitor->phase = inputs->command.phase;
        monitor->phase_start_tick = inputs->now_tick;
        /* Closing transitions need the existing shutdown-circuit permission
         * to remain present while contacts move. Opening transitions are the
         * opposite: hold permission low until every required contact and the
         * load-side bus have reached the verified open state. */
        monitor->transition_authorized =
            prior_permit &&
            allowed &&
            (inputs->command.phase != AMS_AIR_PHASE_OFF) &&
            (inputs->command.phase != AMS_AIR_PHASE_SHUTDOWN);
        if(!allowed)
        {
            air_mark_fault(monitor,
                           AMS_AIR_FAULT_BIT_TRANSITION,
                           transition_reason,
                           true);
        }
    }

    phase_age = air_elapsed(inputs->now_tick, monitor->phase_start_tick);

    if(!air_sample_fresh(inputs->pos_aux.valid,
                         inputs->now_tick,
                         inputs->pos_aux.update_tick,
                         config->contact_sample_timeout_ms) ||
       !air_contact_sample_state_valid(inputs->pos_aux.state) ||
       !air_sample_fresh(inputs->neg_aux.valid,
                         inputs->now_tick,
                         inputs->neg_aux.update_tick,
                         config->contact_sample_timeout_ms) ||
       !air_contact_sample_state_valid(inputs->neg_aux.state) ||
       (config->require_precharge_aux &&
        (!air_sample_fresh(inputs->precharge_aux.valid,
                           inputs->now_tick,
                           inputs->precharge_aux.update_tick,
                           config->contact_sample_timeout_ms) ||
         !air_contact_sample_state_valid(inputs->precharge_aux.state))))
    {
        air_mark_fault(monitor,
                       AMS_AIR_FAULT_BIT_FEEDBACK_STALE,
                       AMS_AIR_FAULT_INPUT_STALE,
                       false);
        air_finalize(monitor);
        return;
    }

    air_filter_update(&monitor->pos_filter,
                      inputs->pos_aux.state,
                      inputs->now_tick,
                      config->debounce_ms);
    air_filter_update(&monitor->neg_filter,
                      inputs->neg_aux.state,
                      inputs->now_tick,
                      config->debounce_ms);
    if(config->require_precharge_aux)
    {
        air_filter_update(&monitor->precharge_filter,
                          inputs->precharge_aux.state,
                          inputs->now_tick,
                          config->debounce_ms);
    }

    if(!monitor->pos_filter.debounced_valid ||
       !monitor->neg_filter.debounced_valid ||
       (config->require_precharge_aux &&
        !monitor->precharge_filter.debounced_valid))
    {
        uint32_t longest_timeout = config->pos_make_timeout_ms;
        if(config->pos_release_timeout_ms > longest_timeout)
        {
            longest_timeout = config->pos_release_timeout_ms;
        }
        if(config->neg_make_timeout_ms > longest_timeout)
        {
            longest_timeout = config->neg_make_timeout_ms;
        }
        if(config->neg_release_timeout_ms > longest_timeout)
        {
            longest_timeout = config->neg_release_timeout_ms;
        }
        if(config->require_precharge_aux &&
           (config->precharge_make_timeout_ms > longest_timeout))
        {
            longest_timeout = config->precharge_make_timeout_ms;
        }
        if(config->require_precharge_aux &&
           (config->precharge_release_timeout_ms > longest_timeout))
        {
            longest_timeout = config->precharge_release_timeout_ms;
        }

        monitor->transition_pending = true;
        monitor->reason = AMS_AIR_FAULT_TRANSITION_PENDING;
        monitor->permit = monitor->transition_authorized &&
                          monitor->boot_open_verified;
        if(phase_age >= longest_timeout)
        {
            air_mark_fault(monitor,
                           AMS_AIR_FAULT_BIT_CONTACT_UNSTABLE,
                           AMS_AIR_FAULT_CONTACT_UNSTABLE,
                           true);
        }
        air_finalize(monitor);
        return;
    }

    monitor->feedback_valid = true;
    monitor->pos_aux = monitor->pos_filter.debounced;
    monitor->neg_aux = monitor->neg_filter.debounced;
    monitor->precharge_aux = config->require_precharge_aux ?
                             monitor->precharge_filter.debounced :
                             AMS_AIR_CONTACT_UNKNOWN;

    if((monitor->pos_aux == AMS_AIR_CONTACT_LINE_FAULT) ||
       (monitor->neg_aux == AMS_AIR_CONTACT_LINE_FAULT) ||
       (config->require_precharge_aux &&
        (monitor->precharge_aux == AMS_AIR_CONTACT_LINE_FAULT)))
    {
        air_mark_fault(monitor,
                       AMS_AIR_FAULT_BIT_LINE_SUPERVISION,
                       AMS_AIR_FAULT_LINE_SUPERVISION,
                       true);
    }

    if(config->require_bus_voltage)
    {
        if(!air_sample_fresh(inputs->pack_voltage.valid,
                             inputs->now_tick,
                             inputs->pack_voltage.update_tick,
                             config->voltage_sample_timeout_ms) ||
           !air_sample_fresh(inputs->load_voltage.valid,
                             inputs->now_tick,
                             inputs->load_voltage.update_tick,
                             config->voltage_sample_timeout_ms) ||
           (inputs->pack_voltage.millivolts < config->minimum_pack_voltage_mv))
        {
            air_mark_fault(monitor,
                           AMS_AIR_FAULT_BIT_VOLTAGE_STALE,
                           AMS_AIR_FAULT_VOLTAGE_STALE,
                           false);
        }
        else
        {
            monitor->voltage_valid = true;
        }
    }
    else
    {
        monitor->bus_plausible = true;
    }

    expected = air_expected_contacts(monitor->phase);
    actual[0] = monitor->pos_aux;
    actual[1] = monitor->neg_aux;
    actual[2] = monitor->precharge_aux;
    wanted[0] = expected.pos;
    wanted[1] = expected.neg;
    wanted[2] = expected.precharge;
    required[0] = true;
    required[1] = true;
    required[2] = config->require_precharge_aux;

    monitor->contact_disagreement =
        (monitor->pos_aux != AMS_AIR_CONTACT_LINE_FAULT) &&
        (monitor->neg_aux != AMS_AIR_CONTACT_LINE_FAULT) &&
        (monitor->pos_aux != monitor->neg_aux) &&
        (expected.pos == expected.neg);

    for(unsigned int contact = 0u; contact < 3u; contact++)
    {
        if(!required[contact])
        {
            continue;
        }
        if(actual[contact] != wanted[contact])
        {
            contacts_match = false;
            monitor->transition_pending = true;
            if(phase_age >= air_contact_timeout(config,
                                                contact,
                                                wanted[contact]))
            {
                air_mark_fault(monitor,
                               air_contact_fault_bit(contact),
                               air_contact_fault_reason(contact,
                                                        wanted[contact]),
                               true);
            }
        }
    }

    if(monitor->contact_disagreement &&
       (monitor->active_fault_mask &
        (AMS_AIR_FAULT_BIT_POS_CONTACT | AMS_AIR_FAULT_BIT_NEG_CONTACT)))
    {
        monitor->active_fault_mask |= AMS_AIR_FAULT_BIT_CONTACT_DISAGREE;
        monitor->latched_fault_mask |= AMS_AIR_FAULT_BIT_CONTACT_DISAGREE;
    }

    /* The command owner may not leave the precharge relay energized forever,
     * even on a design that has not implemented load-side voltage proof. */
    if((monitor->phase == AMS_AIR_PHASE_PRECHARGE) &&
       (phase_age >= config->precharge_max_ms))
    {
        air_mark_fault(monitor,
                       AMS_AIR_FAULT_BIT_PRECHARGE_TIMEOUT,
                       AMS_AIR_FAULT_PRECHARGE_TIMEOUT,
                       true);
    }

    if(config->require_bus_voltage && monitor->voltage_valid)
    {
        uint32_t pack_mv = inputs->pack_voltage.millivolts;
        uint32_t load_mv = inputs->load_voltage.millivolts;

        if(monitor->phase == AMS_AIR_PHASE_PRECHARGE)
        {
            monitor->precharge_complete =
                air_ratio_at_least(load_mv,
                                   pack_mv,
                                   config->precharge_complete_min_permille);
            monitor->bus_plausible =
                air_ratio_at_most(load_mv,
                                  pack_mv,
                                  config->run_bus_max_permille);
            if(!monitor->bus_plausible)
            {
                air_mark_fault(monitor,
                               AMS_AIR_FAULT_BIT_BUS_VOLTAGE,
                               AMS_AIR_FAULT_BUS_VOLTAGE_PLAUSIBILITY,
                               true);
            }
        }
        else if(monitor->phase == AMS_AIR_PHASE_RUN)
        {
            if(phase_age < config->run_voltage_settle_ms)
            {
                voltage_phase_ok = false;
                monitor->transition_pending = true;
            }
            else
            {
                monitor->bus_plausible =
                    air_ratio_at_least(load_mv,
                                       pack_mv,
                                       config->run_bus_min_permille) &&
                    air_ratio_at_most(load_mv,
                                      pack_mv,
                                      config->run_bus_max_permille);
                if(!monitor->bus_plausible)
                {
                    voltage_phase_ok = false;
                    air_mark_fault(monitor,
                                   AMS_AIR_FAULT_BIT_BUS_VOLTAGE,
                                   AMS_AIR_FAULT_BUS_VOLTAGE_PLAUSIBILITY,
                                   true);
                }
            }
        }
        else
        {
            if(phase_age < config->bus_discharge_timeout_ms)
            {
                voltage_phase_ok = false;
                monitor->transition_pending = true;
            }
            else
            {
                monitor->bus_plausible = (load_mv <= config->open_bus_max_mv);
                if(!monitor->bus_plausible)
                {
                    voltage_phase_ok = false;
                    air_mark_fault(monitor,
                                   AMS_AIR_FAULT_BIT_BUS_VOLTAGE,
                                   AMS_AIR_FAULT_BUS_VOLTAGE_PLAUSIBILITY,
                                   true);
                }
            }
        }
    }

    if(((monitor->phase == AMS_AIR_PHASE_PRECHARGE) ||
        (monitor->phase == AMS_AIR_PHASE_RUN)) &&
       !monitor->boot_open_verified)
    {
        air_mark_fault(monitor,
                       AMS_AIR_FAULT_BIT_BOOT_OPEN,
                       AMS_AIR_FAULT_BOOT_OPEN_NOT_VERIFIED,
                       false);
    }

    monitor->steady_state_valid = contacts_match &&
                                  voltage_phase_ok &&
                                  (monitor->active_fault_mask == 0u);

    if(monitor->steady_state_valid &&
       ((monitor->phase == AMS_AIR_PHASE_OFF) ||
        (monitor->phase == AMS_AIR_PHASE_SHUTDOWN)) &&
       (!config->require_bus_voltage || monitor->bus_plausible))
    {
        monitor->boot_open_verified = true;
    }

    if((monitor->active_fault_mask == 0u) &&
       (monitor->latched_fault_mask == 0u))
    {
        if(monitor->steady_state_valid)
        {
            monitor->transition_pending = false;
            monitor->transition_authorized = true;
            monitor->permit = monitor->boot_open_verified;
            monitor->reason = AMS_AIR_FAULT_NONE;
        }
        else
        {
            monitor->transition_pending = true;
            monitor->permit = monitor->transition_authorized &&
                              monitor->boot_open_verified;
            monitor->reason = AMS_AIR_FAULT_TRANSITION_PENDING;
        }
    }

    air_finalize(monitor);
}

bool ams_air_monitor_request_clear(ams_air_monitor_t *monitor,
                                   const ams_air_monitor_config_t *config,
                                   const ams_air_monitor_inputs_t *inputs)
{
    if((monitor == NULL) || (config == NULL) || (inputs == NULL))
    {
        return false;
    }

    ams_air_monitor_step(monitor, config, inputs);

    if(!monitor->feature_enabled ||
       !monitor->configuration_valid ||
       !monitor->command_valid ||
       !monitor->feedback_valid ||
       (monitor->active_fault_mask != 0u) ||
       !monitor->steady_state_valid ||
       !monitor->boot_open_verified ||
       ((monitor->phase != AMS_AIR_PHASE_OFF) &&
        (monitor->phase != AMS_AIR_PHASE_SHUTDOWN)) ||
       (monitor->pos_aux != AMS_AIR_CONTACT_OPEN) ||
       (monitor->neg_aux != AMS_AIR_CONTACT_OPEN) ||
       (config->require_precharge_aux &&
        (monitor->precharge_aux != AMS_AIR_CONTACT_OPEN)) ||
       (config->require_bus_voltage && !monitor->bus_plausible))
    {
        return false;
    }

    monitor->latched_fault_mask = 0u;
    monitor->latched_reason = AMS_AIR_FAULT_NONE;
    monitor->fault_latched = false;
    monitor->fault = false;
    monitor->reason = AMS_AIR_FAULT_NONE;
    monitor->permit = true;
    monitor->transition_authorized = true;
    return true;
}
