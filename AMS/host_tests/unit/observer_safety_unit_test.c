#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ext_drivers/main_fuse_monitor.h"
#include "ext_drivers/parallel_connection_observer.h"

static unsigned failures;

#define CHECK(expr)                                                        \
    do                                                                     \
    {                                                                      \
        if(!(expr))                                                        \
        {                                                                  \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
            failures++;                                                    \
        }                                                                  \
    } while(0)

static void prepare_single_active_segment(accumulator_t *acc,
                                          uint16_t nominal_mv,
                                          uint32_t now_ms)
{
    const uint16_t required_mask = (uint16_t)((1UL << NCELLS) - 1UL);

    memset(acc, 0, sizeof(*acc));
    acc->smb.num_ics = 1;
    acc->smb.ics_capacity = NSMBS;
    acc->smb.ics = acc->smb_ics;
    acc->smb.monitored_cell_count = NCELLS;
    acc->voltage_full_usable = true;
    acc->usable_voltage_mask[0] = required_mask;
    for(uint8_t cell = 0u; cell < NCELLS; cell++)
    {
        acc->cell_voltage_mv[0][cell] = nominal_mv;
        acc->cell_voltage_last_update_ms[0][cell] = now_ms;
    }
}

static void test_parallel_observer_uses_configured_segments_only(void)
{
    accumulator_t acc;
    ams_parallel_connection_observer_t observer;
    uint32_t now_ms = 1000u;

    CHECK(NSMBS >= 5);
    prepare_single_active_segment(&acc, 3500u, now_ms);
    ams_parallel_connection_observer_init(&observer);

    /* Only SMB0 is configured and fresh. The unused storage slots must not be
     * treated as implicit missing segments in a one-SMB bench topology. */
    ams_parallel_connection_observer_step(&observer,
                                          &acc,
                                          0.0f,
                                          true,
                                          false,
                                          now_ms);
    CHECK(observer.input_valid);
    CHECK(observer.initialized);
    CHECK(observer.reason == AMS_PARALLEL_OBSERVER_WAITING);

    for(uint8_t event = 0u;
        event < AMS_PARALLEL_OBSERVER_CONFIRM_EVENTS;
        event++)
    {
        bool loaded = ((event & 1u) == 0u);
        uint16_t ordinary_mv = loaded ? 3498u : 3500u;
        uint16_t outlier_mv = loaded ? 3470u : 3500u;
        float current_a = loaded ? 20.0f : 0.0f;

        now_ms += 100u;
        for(uint8_t cell = 0u; cell < NCELLS; cell++)
        {
            acc.cell_voltage_mv[0][cell] = ordinary_mv;
            acc.cell_voltage_last_update_ms[0][cell] = now_ms;
        }
        acc.cell_voltage_mv[0][3] = outlier_mv;
        ams_parallel_connection_observer_step(&observer,
                                              &acc,
                                              current_a,
                                              true,
                                              false,
                                              now_ms);
    }

    CHECK(observer.advisory_valid);
    CHECK(observer.suspect);
    CHECK((observer.suspect_mask[0] & (1u << 3u)) != 0u);
    for(uint8_t seg = 1u; seg < NSMBS; seg++)
    {
        CHECK(observer.candidate_mask[seg] == 0u);
        CHECK(observer.suspect_mask[seg] == 0u);
        CHECK(observer.last_valid_mask[seg] == 0u);
    }

    acc.smb.num_ics = 0;
    ams_parallel_connection_observer_step(&observer,
                                          &acc,
                                          0.0f,
                                          true,
                                          false,
                                          now_ms + 10u);
    CHECK(!observer.input_valid);
    CHECK(!observer.initialized);
    CHECK(!observer.advisory_valid);
    CHECK(!observer.suspect);
    CHECK(observer.reason == AMS_PARALLEL_OBSERVER_INPUT_INVALID);
}

static void prepare_main_fuse_inputs(ams_air_monitor_t *air,
                                     ams_air_monitor_inputs_t *inputs,
                                     uint32_t now_ms)
{
    memset(air, 0, sizeof(*air));
    memset(inputs, 0, sizeof(*inputs));

    air->feature_enabled = true;
    air->configuration_valid = true;
    air->command_valid = true;
    air->feedback_valid = true;
    air->voltage_valid = true;
    air->boot_open_verified = true;
    air->transition_authorized = true;
    air->last_update_tick = now_ms;
    air->phase = AMS_AIR_PHASE_RUN;
    air->steady_state_valid = true;
    air->permit = true;
    air->precharge_complete = true;

    inputs->now_tick = now_ms;
    inputs->command.valid = true;
    inputs->command.phase = AMS_AIR_PHASE_RUN;
    inputs->command.update_tick = now_ms;
    inputs->pos_aux.valid = true;
    inputs->pos_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs->pos_aux.update_tick = now_ms;
    inputs->neg_aux.valid = true;
    inputs->neg_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs->neg_aux.update_tick = now_ms;
    inputs->pack_voltage.valid = true;
    inputs->pack_voltage.millivolts = 300000u;
    inputs->pack_voltage.update_tick = now_ms;
    inputs->load_voltage.valid = true;
    inputs->load_voltage.millivolts = 295000u;
    inputs->load_voltage.update_tick = now_ms;
}

static void refresh_main_fuse_ticks(ams_air_monitor_t *air,
                                    ams_air_monitor_inputs_t *inputs,
                                    uint32_t now_ms)
{
    air->last_update_tick = now_ms;
    inputs->now_tick = now_ms;
    inputs->command.update_tick = now_ms;
    inputs->pos_aux.update_tick = now_ms;
    inputs->neg_aux.update_tick = now_ms;
    inputs->pack_voltage.update_tick = now_ms;
    inputs->load_voltage.update_tick = now_ms;
}

static void test_validated_main_fuse_monitor(void)
{
    ams_main_fuse_monitor_t monitor;
    ams_air_monitor_t air;
    ams_air_monitor_inputs_t inputs;

    ams_main_fuse_monitor_init(&monitor);
    prepare_main_fuse_inputs(&air, &inputs, 1000u);

    ams_main_fuse_monitor_step(&monitor,
                               &air,
                               &inputs,
                               5.0f,
                               true,
                               1000u);
    CHECK(monitor.authority_valid);
    CHECK(!monitor.suspect_open);
    CHECK(monitor.reason == AMS_MAIN_FUSE_MONITOR_HEALTHY);

    inputs.load_voltage.millivolts = 20000u;
    refresh_main_fuse_ticks(&air, &inputs, 1100u);
    ams_main_fuse_monitor_step(&monitor,
                               &air,
                               &inputs,
                               0.0f,
                               true,
                               1100u);
    CHECK(monitor.authority_valid);
    CHECK(monitor.suspect_open);
    CHECK(!monitor.confirmed_open);
    CHECK(monitor.reason == AMS_MAIN_FUSE_MONITOR_SUSPECT_OPEN);

    refresh_main_fuse_ticks(&air, &inputs, 1700u);
    ams_main_fuse_monitor_step(&monitor,
                               &air,
                               &inputs,
                               0.0f,
                               true,
                               1700u);
    CHECK(monitor.confirmed_open);
    CHECK(monitor.latched);
    CHECK(monitor.reason == AMS_MAIN_FUSE_MONITOR_CONFIRMED_OPEN);

    /* A stale publication cannot create authority or continue a new
     * confirmation interval. The already latched service result is retained. */
    ams_main_fuse_monitor_step(&monitor,
                               &air,
                               &inputs,
                               0.0f,
                               true,
                               2801u);
    CHECK(!monitor.authority_valid);
    CHECK(!monitor.suspect_open);
    CHECK(monitor.confirmed_open);
    CHECK(monitor.latched);
    CHECK(monitor.reason == AMS_MAIN_FUSE_MONITOR_INPUT_STALE);

    refresh_main_fuse_ticks(&air, &inputs, 2900u);
    inputs.command.phase = AMS_AIR_PHASE_OFF;
    inputs.load_voltage.millivolts = 0u;
    CHECK(ams_main_fuse_monitor_request_clear(&monitor,
                                              &inputs,
                                              0.0f,
                                              true,
                                              2900u));
    CHECK(!monitor.confirmed_open);
    CHECK(!monitor.latched);
    CHECK(monitor.reason == AMS_MAIN_FUSE_MONITOR_IDLE);
}

int main(void)
{
    test_parallel_observer_uses_configured_segments_only();
    test_validated_main_fuse_monitor();

    if(failures != 0u)
    {
        fprintf(stderr, "OBSERVER SAFETY TESTS FAILED: %u\n", failures);
        return 1;
    }

    puts("PASS configured-segment parallel observer topology");
    puts("PASS validated main-fuse plausibility/stale/clear policy");
    puts("ALL OBSERVER SAFETY TESTS PASSED");
    return 0;
}
