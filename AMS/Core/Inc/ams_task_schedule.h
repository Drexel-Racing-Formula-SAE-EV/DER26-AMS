#ifndef AMS_TASK_SCHEDULE_H
#define AMS_TASK_SCHEDULE_H

#include <stdint.h>
#include "cmsis_os2.h"

/* Task context only. Always block, including exact or missed deadlines.
 * CMSIS osDelayUntil returns osErrorParameter without sleeping for an expired
 * deadline. Ignoring it creates a continuously-ready high-priority task.
 * A positive relative wait also closes the race where a tick/preemption occurs
 * between reading now and submitting an absolute deadline. Normal deadlines
 * retain the remaining interval; missed slots yield one tick without replay.
 */
static inline void ams_task_delay_until(uint32_t deadline)
{
    const uint32_t now = osKernelGetTickCount();
    uint32_t delay = deadline - now;
    if((delay == 0u) || (delay >= UINT32_C(0x80000000)))
    {
        delay = 1u;
    }
    (void)osDelay(delay);
}

#endif
