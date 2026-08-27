#include "ext_drivers/main_fuse_monitor.h"

#include <stddef.h>

#include "ams_build_profile.h"

#define AMS_MAIN_FUSE_MIN_PACK_MV             100000u
#define AMS_MAIN_FUSE_MAX_LOAD_PERMILLE       500u
#define AMS_MAIN_FUSE_NEAR_ZERO_CURRENT_A       2.0f
#define AMS_MAIN_FUSE_CONFIRM_MS               500u
#define AMS_MAIN_FUSE_INPUT_MAX_AGE_MS        1000u
#define AMS_MAIN_FUSE_CLEAR_LOAD_PERMILLE       100u
#define AMS_MAIN_FUSE_CLEAR_LOAD_MAX_MV       60000u

#ifndef AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED
#define AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED 0
#endif

static float main_fuse_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool main_fuse_sample_fresh(bool valid,
                                   uint32_t update_tick,
                                   uint32_t now_tick)
{
    return valid && ((now_tick - update_tick) <= AMS_MAIN_FUSE_INPUT_MAX_AGE_MS);
}

void ams_main_fuse_monitor_init(ams_main_fuse_monitor_t *monitor)
{
    if(monitor == NULL)
    {
        return;
    }
    *monitor = (ams_main_fuse_monitor_t){0};
    monitor->reason = AMS_MAIN_FUSE_MONITOR_UNAVAILABLE;
    monitor->latched_reason = AMS_MAIN_FUSE_MONITOR_UNAVAILABLE;
}

void ams_main_fuse_monitor_step(ams_main_fuse_monitor_t *monitor,
                                const ams_air_monitor_t *air,
                                const ams_air_monitor_inputs_t *inputs,
                                float current_a,
                                bool current_valid,
                                uint32_t now_ms)
{
    bool samples_fresh;
    bool configuration_authorized;
    bool run_proven;
    bool open_signature;

    if(monitor == NULL)
    {
        return;
    }

    monitor->last_update_tick = now_ms;
    if(monitor->evaluation_count != UINT32_MAX)
    {
        monitor->evaluation_count++;
    }

    if((air == NULL) || (inputs == NULL))
    {
        monitor->authority_valid = false;
        monitor->reason = AMS_MAIN_FUSE_MONITOR_UNAVAILABLE;
        return;
    }

    samples_fresh =
        main_fuse_sample_fresh(air->last_update_tick != 0u,
                               air->last_update_tick,
                               now_ms) &&
        main_fuse_sample_fresh(inputs->command.valid,
                               inputs->command.update_tick,
                               now_ms) &&
        main_fuse_sample_fresh(inputs->pos_aux.valid,
                               inputs->pos_aux.update_tick,
                               now_ms) &&
        main_fuse_sample_fresh(inputs->neg_aux.valid,
                               inputs->neg_aux.update_tick,
                               now_ms) &&
        main_fuse_sample_fresh(inputs->pack_voltage.valid,
                               inputs->pack_voltage.update_tick,
                               now_ms) &&
        main_fuse_sample_fresh(inputs->load_voltage.valid,
                               inputs->load_voltage.update_tick,
                               now_ms) &&
        current_valid;

    configuration_authorized =
        (AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED != 0) &&
        (AMS_ENABLE_AIR_AUX_FEEDBACK != 0) &&
        (AMS_CURRENT_CALIBRATION_VALIDATED != 0) &&
        (AMS_MAIN_FUSE_LOAD_VOLTAGE_TARGET_VALIDATED != 0);
    monitor->authority_valid = samples_fresh && configuration_authorized;

    monitor->pack_mv = inputs->pack_voltage.millivolts;
    monitor->load_mv = inputs->load_voltage.millivolts;
    monitor->current_a = current_a;

    if(!samples_fresh)
    {
        monitor->suspect_open = false;
        monitor->suspect_since_tick = 0u;
        monitor->reason = configuration_authorized &&
                          (inputs->command.valid ||
                           inputs->pack_voltage.valid ||
                           inputs->load_voltage.valid) ?
                          AMS_MAIN_FUSE_MONITOR_INPUT_STALE :
                          AMS_MAIN_FUSE_MONITOR_UNAVAILABLE;
        return;
    }

    /* Never produce suspect/confirmed fuse claims from an unevidenced build.
     * The raw values remain visible for service diagnostics, but authority and
     * state stay explicitly unavailable until every required adapter and
     * calibration gate is compiled in. */
    if(!configuration_authorized)
    {
        monitor->suspect_open = false;
        monitor->suspect_since_tick = 0u;
        monitor->reason = AMS_MAIN_FUSE_MONITOR_UNAVAILABLE;
        return;
    }

    if(inputs->command.phase == AMS_AIR_PHASE_OFF ||
       inputs->command.phase == AMS_AIR_PHASE_SHUTDOWN)
    {
        monitor->suspect_open = false;
        monitor->suspect_since_tick = 0u;
        monitor->reason = AMS_MAIN_FUSE_MONITOR_IDLE;
        return;
    }

    run_proven = ams_air_monitor_ready(air) &&
                 (inputs->command.phase == AMS_AIR_PHASE_RUN) &&
                 (air->phase == AMS_AIR_PHASE_RUN) &&
                 air->steady_state_valid &&
                 air->permit &&
                 !air->fault &&
                 !air->fault_latched &&
                 (inputs->pos_aux.state == AMS_AIR_CONTACT_CLOSED) &&
                 (inputs->neg_aux.state == AMS_AIR_CONTACT_CLOSED) &&
                 air->precharge_complete;
    if(!run_proven)
    {
        monitor->suspect_open = false;
        monitor->suspect_since_tick = 0u;
        monitor->reason = AMS_MAIN_FUSE_MONITOR_TRANSITION;
        return;
    }

    open_signature =
        (inputs->pack_voltage.millivolts >= AMS_MAIN_FUSE_MIN_PACK_MV) &&
        (((uint64_t)inputs->load_voltage.millivolts * 1000u) <
         ((uint64_t)inputs->pack_voltage.millivolts *
          AMS_MAIN_FUSE_MAX_LOAD_PERMILLE)) &&
        (main_fuse_absf(current_a) <= AMS_MAIN_FUSE_NEAR_ZERO_CURRENT_A);

    if(!open_signature)
    {
        monitor->suspect_open = false;
        monitor->suspect_since_tick = 0u;
        monitor->reason = AMS_MAIN_FUSE_MONITOR_HEALTHY;
        return;
    }

    if(!monitor->suspect_open)
    {
        monitor->suspect_open = true;
        monitor->suspect_since_tick = now_ms;
    }

    if((now_ms - monitor->suspect_since_tick) >= AMS_MAIN_FUSE_CONFIRM_MS)
    {
        monitor->confirmed_open = true;
        monitor->latched = true;
        monitor->reason = AMS_MAIN_FUSE_MONITOR_CONFIRMED_OPEN;
        monitor->latched_reason = monitor->reason;
    }
    else
    {
        monitor->reason = AMS_MAIN_FUSE_MONITOR_SUSPECT_OPEN;
    }
}

bool ams_main_fuse_monitor_request_clear(
    ams_main_fuse_monitor_t *monitor,
    const ams_air_monitor_inputs_t *inputs,
    float current_a,
    bool current_valid,
    uint32_t now_ms)
{
    bool fresh;
    bool phase_safe;
    bool bus_safe;

    if((monitor == NULL) || (inputs == NULL))
    {
        return false;
    }

    fresh = main_fuse_sample_fresh(inputs->command.valid,
                                   inputs->command.update_tick,
                                   now_ms) &&
            main_fuse_sample_fresh(inputs->pack_voltage.valid,
                                   inputs->pack_voltage.update_tick,
                                   now_ms) &&
            main_fuse_sample_fresh(inputs->load_voltage.valid,
                                   inputs->load_voltage.update_tick,
                                   now_ms) &&
            current_valid;
    phase_safe = (inputs->command.phase == AMS_AIR_PHASE_OFF) ||
                 (inputs->command.phase == AMS_AIR_PHASE_SHUTDOWN);
    bus_safe = (inputs->load_voltage.millivolts <=
                AMS_MAIN_FUSE_CLEAR_LOAD_MAX_MV) &&
               ((inputs->pack_voltage.millivolts == 0u) ||
                (((uint64_t)inputs->load_voltage.millivolts * 1000u) <=
                 ((uint64_t)inputs->pack_voltage.millivolts *
                  AMS_MAIN_FUSE_CLEAR_LOAD_PERMILLE)));

    if(!fresh || !phase_safe || !bus_safe ||
       (main_fuse_absf(current_a) > AMS_MAIN_FUSE_NEAR_ZERO_CURRENT_A))
    {
        return false;
    }

    monitor->suspect_open = false;
    monitor->confirmed_open = false;
    monitor->latched = false;
    monitor->suspect_since_tick = 0u;
    monitor->reason = AMS_MAIN_FUSE_MONITOR_IDLE;
    monitor->latched_reason = AMS_MAIN_FUSE_MONITOR_UNAVAILABLE;
    return true;
}

const char *ams_main_fuse_monitor_reason_str(
    ams_main_fuse_monitor_reason_t reason)
{
    switch(reason)
    {
    case AMS_MAIN_FUSE_MONITOR_UNAVAILABLE: return "unavailable";
    case AMS_MAIN_FUSE_MONITOR_IDLE: return "idle";
    case AMS_MAIN_FUSE_MONITOR_TRANSITION: return "transition";
    case AMS_MAIN_FUSE_MONITOR_HEALTHY: return "healthy";
    case AMS_MAIN_FUSE_MONITOR_SUSPECT_OPEN: return "suspect_open_hv_path";
    case AMS_MAIN_FUSE_MONITOR_CONFIRMED_OPEN: return "confirmed_open_hv_path";
    case AMS_MAIN_FUSE_MONITOR_INPUT_STALE: return "input_stale";
    default: return "unknown";
    }
}
