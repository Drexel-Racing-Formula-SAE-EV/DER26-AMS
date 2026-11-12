#ifndef AMS_DIAG_BUDGET_H
#define AMS_DIAG_BUDGET_H
#include <stdint.h>
#include <stdbool.h>
/* Advisory work never overrides the scan budget. Lateness is reported by
 * forced (legacy field name), but does not authorize an over-budget job.
 * Successful coverage ages expose overload instead of hiding it with retries. */
static inline bool ams_diag_admit(uint32_t now, uint32_t scan_start,
    uint32_t period, uint32_t cost, uint32_t due, uint32_t max_deferral,
    bool *forced)
{
    uint32_t elapsed = now - scan_start;
    *forced = (uint32_t)(now - due) >= max_deferral;
    return (elapsed < period) && (cost <= period - elapsed);
}
/* No position can monopolize the rotation: at most two attempts, on
 * separate scheduled visits. Completed fault detections advance immediately. */
static inline void ams_diag_advance(bool completed, uint8_t *retry, uint8_t *sensor)
{
    if(!completed && *retry == 0u) *retry = 1u;
    else { *retry = 0u; *sensor = (uint8_t)((*sensor + 1u) % 24u); }
}
/* A missing result is not a pass. Zero is a valid timestamp. */
static inline uint32_t ams_diag_overdue_mask(uint32_t now, uint32_t start,
    const uint32_t completed[24], uint32_t seen, uint32_t max_age)
{
    uint32_t result = 0;
    for(unsigned i=0;i<24;i++)
        if((uint32_t)(now - ((seen & (1UL<<i)) ? completed[i] : start)) >= max_age)
            result |= 1UL<<i;
    return result;
}
#endif
