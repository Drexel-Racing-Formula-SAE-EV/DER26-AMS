/*
 * main_fuse_monitor.h
 *
 * Plausibility monitor for an open main accumulator fuse/HV path.  It requires
 * reviewed AIR command/auxiliary-contact feedback and independent pack/load
 * bus voltage samples.  In current hardware those inputs are unavailable, so
 * the monitor remains explicitly unavailable and cannot assert fuse_fault.
 */
#ifndef INC_EXT_DRIVERS_MAIN_FUSE_MONITOR_H_
#define INC_EXT_DRIVERS_MAIN_FUSE_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "ext_drivers/air_monitor.h"

typedef enum
{
    AMS_MAIN_FUSE_MONITOR_UNAVAILABLE = 0,
    AMS_MAIN_FUSE_MONITOR_IDLE,
    AMS_MAIN_FUSE_MONITOR_TRANSITION,
    AMS_MAIN_FUSE_MONITOR_HEALTHY,
    AMS_MAIN_FUSE_MONITOR_SUSPECT_OPEN,
    AMS_MAIN_FUSE_MONITOR_CONFIRMED_OPEN,
    AMS_MAIN_FUSE_MONITOR_INPUT_STALE
} ams_main_fuse_monitor_reason_t;

typedef struct
{
    bool authority_valid;
    bool suspect_open;
    bool confirmed_open;
    bool latched;
    ams_main_fuse_monitor_reason_t reason;
    ams_main_fuse_monitor_reason_t latched_reason;
    uint32_t suspect_since_tick;
    uint32_t last_update_tick;
    uint32_t evaluation_count;
    uint32_t pack_mv;
    uint32_t load_mv;
    float current_a;
} ams_main_fuse_monitor_t;

void ams_main_fuse_monitor_init(ams_main_fuse_monitor_t *monitor);
void ams_main_fuse_monitor_step(ams_main_fuse_monitor_t *monitor,
                                const ams_air_monitor_t *air,
                                const ams_air_monitor_inputs_t *inputs,
                                float current_a,
                                bool current_valid,
                                uint32_t now_ms);
/* Controlled service clear. A confirmed/latched open-HV-path result is
 * cleared only with fresh OFF/SHUTDOWN command, a discharged load bus and
 * near-zero current. */
bool ams_main_fuse_monitor_request_clear(
    ams_main_fuse_monitor_t *monitor,
    const ams_air_monitor_inputs_t *inputs,
    float current_a,
    bool current_valid,
    uint32_t now_ms);
const char *ams_main_fuse_monitor_reason_str(
    ams_main_fuse_monitor_reason_t reason);

#endif /* INC_EXT_DRIVERS_MAIN_FUSE_MONITOR_H_ */
