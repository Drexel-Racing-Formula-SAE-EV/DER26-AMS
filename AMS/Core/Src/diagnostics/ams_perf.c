/* Passive DWT accounting. Task context excludes instrumented IRQ bodies.
 * PendSV/SVC entry/exit and profiler overhead are not fully separated. */
#include "diagnostics/ams_perf.h"
#include "main.h"
#include <string.h>
#ifndef AMS_HOST_TEST
#define AMS_HOST_TEST 0
#endif
static ams_perf_row_t rows[AMS_PERF_TASKS];
static uint32_t ready_at[AMS_PERF_TASKS], slice_cycles[AMS_PERF_TASKS];
static bool ready_pending[AMS_PERF_TASKS];
static uint32_t last, depth, overflow;
static int current=-1;
static bool initialized;
static uint64_t irq_total;
#if AMS_HOST_TEST
__attribute__((weak)) uint32_t SystemCoreClock=216000000u;
__attribute__((weak)) uint32_t ams_perf_test_clock(void) { return 0; }
#define CLOCK() ams_perf_test_clock()
static uint32_t lock(void) { return 0; }
static void unlock(uint32_t state) { (void)state; }
#else
#define CLOCK() DWT->CYCCNT
static uint32_t lock(void) { uint32_t s=__get_PRIMASK(); __disable_irq(); return s; }
static void unlock(uint32_t state) { __set_PRIMASK(state); }
#endif
static void checkpoint(uint32_t now)
{
    uint32_t elapsed=now-last; last=now;
    if(depth) irq_total+=elapsed;
    else if(current>=0) { rows[current].cycles+=elapsed; slice_cycles[current]+=elapsed; }
}
static int find(const void *task, const char *name)
{
    for(unsigned i=0;i<AMS_PERF_TASKS;i++) if(rows[i].task==task) return (int)i;
    for(unsigned i=0;i<AMS_PERF_TASKS;i++) if(!rows[i].task) {
        rows[i].task=task;
        if(name) { strncpy(rows[i].name,name,15); rows[i].name[15]=0; }
        return (int)i;
    }
    overflow++; return -1;
}
void ams_perf_init(void)
{
    uint32_t s=lock();
    if(!initialized) {
#if !AMS_HOST_TEST
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
        last=CLOCK(); initialized=true;
    }
    unlock(s);
}
void ams_perf_switch_in(const void *task, const char *name)
{
    if(!initialized || !task) return;
    uint32_t s=lock(), now=CLOCK(); checkpoint(now);
    current=find(task,name);
    if(current>=0) {
        if(ready_pending[current]) {
            uint32_t delay=now-ready_at[current];
            if(delay>rows[current].max_unblock_cycles) rows[current].max_unblock_cycles=delay;
            ready_pending[current]=false;
        }
        rows[current].slices++; slice_cycles[current]=0;
    }
    unlock(s);
}
void ams_perf_clear(void)
{
    uint32_t s=lock();
    for(unsigned i=0;i<AMS_PERF_TASKS;i++) {
        rows[i].cycles=0; rows[i].slices=0;
        rows[i].max_slice_cycles=rows[i].max_unblock_cycles=0;
        ready_pending[i]=false; slice_cycles[i]=0;
    }
    if(current>=0) rows[current].slices=1;
    irq_total=0; overflow=0; last=CLOCK();
    unlock(s); /* Never reset the shared DWT counter. */
}
void ams_perf_switch_out(void)
{
    if(!initialized) return;
    uint32_t s=lock(); checkpoint(CLOCK());
    if(current>=0 && slice_cycles[current]>rows[current].max_slice_cycles)
        rows[current].max_slice_cycles=slice_cycles[current];
    current=-1; unlock(s);
}
void ams_perf_ready(const void *task, const char *name)
{
    if(!initialized || !task) return;
    uint32_t s=lock(); int i=find(task,name);
    if(i>=0 && !ready_pending[i]) { ready_at[i]=CLOCK(); ready_pending[i]=true; }
    unlock(s);
}
void ams_perf_irq_enter(void)
{
    if(!initialized) return;
    uint32_t s=lock(); checkpoint(CLOCK()); depth++; unlock(s);
}
void ams_perf_irq_exit(void)
{
    if(!initialized) return;
    uint32_t s=lock(); checkpoint(CLOCK()); if(depth) depth--; unlock(s);
}
bool ams_perf_get(unsigned index, ams_perf_row_t *out)
{
    if(index>=AMS_PERF_TASKS || !out) return false;
    uint32_t s=lock(); if(initialized) checkpoint(CLOCK());
    *out=rows[index]; bool valid=out->task!=0; unlock(s); return valid;
}
uint64_t ams_perf_irq_cycles(void)
{
    uint32_t s=lock(); if(initialized) checkpoint(CLOCK());
    uint64_t total=irq_total; unlock(s); return total;
}
uint32_t ams_perf_overflow(void) { return overflow; }
