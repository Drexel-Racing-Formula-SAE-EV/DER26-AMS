#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "ams_task_schedule.h"

static uint32_t tick;
static uint32_t advance_after_read;
static uint32_t sleeps;
static uint32_t last_delay;
static uint32_t cli_opportunities;

uint32_t osKernelGetTickCount(void)
{
    uint32_t sampled = tick;
    tick += advance_after_read;
    advance_after_read = 0u;
    return sampled;
}

osStatus_t osDelay(uint32_t delay)
{
    /* A ready lower-priority service task can run only after its higher-
     * priority caller blocks. Fail immediately on zero/nonblocking waits. */
    assert(delay > 0u && delay < UINT32_C(0x80000000));
    sleeps++;
    cli_opportunities++;
    last_delay = delay;
    tick += delay;
    return osOK;
}

int main(void)
{
    tick = 100u;
    ams_task_delay_until(110u);
    assert(last_delay == 10u && tick == 110u);
    ams_task_delay_until(110u);
    assert(last_delay == 1u);
    tick = 6658u; /* Target overrun interval: first entry was tick 118. */
    ams_task_delay_until(218u);
    assert(last_delay == 1u);
    tick = UINT32_C(0xFFFFFFFC);
    ams_task_delay_until(4u);
    assert(last_delay == 8u && tick == 4u);
    tick = 4u;
    ams_task_delay_until(UINT32_C(0xFFFFFFFE));
    assert(last_delay == 1u);
    tick = 0u;
    ams_task_delay_until(UINT32_C(0x80000000));
    assert(last_delay == 1u);

    /* A preemption after sampling now cannot turn the wait into a no-op. */
    tick = 100u;
    advance_after_read = 20u;
    ams_task_delay_until(110u);
    assert(last_delay == 10u && tick == 130u);

    uint32_t previous_sleeps = sleeps;
    uint32_t previous_cli = cli_opportunities;
    for(unsigned i = 0u; i < 100u; i++)
    {
        uint32_t entry = tick;
        tick += 150u; /* Work repeatedly exceeds the 100-tick period. */
        ams_task_delay_until(entry + 100u);
    }
    assert(sleeps - previous_sleeps == 100u);
    assert(cli_opportunities - previous_cli == 100u);
    puts("PASS periodic task blocking: nominal, exact, overrun, wrap, preemption race, sustained overload");
    return 0;
}
