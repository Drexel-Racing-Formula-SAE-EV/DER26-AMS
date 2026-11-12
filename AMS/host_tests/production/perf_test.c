#include <assert.h>
#include <stdio.h>
#include "diagnostics/ams_perf.h"
static uint32_t clock_now;
uint32_t ams_perf_test_clock(void) { return clock_now; }
int main(void) {
    int a,b; ams_perf_row_t row;
    ams_perf_irq_enter(); /* safe before initialization */
    ams_perf_init(); ams_perf_ready(&a,"ADBMS");
    clock_now=10; ams_perf_switch_in(&a,"ADBMS");
    clock_now=30; ams_perf_irq_enter();
    clock_now=35; ams_perf_irq_enter();
    clock_now=40; ams_perf_irq_exit();
    clock_now=50; ams_perf_irq_exit();
    clock_now=70; ams_perf_switch_out(); ams_perf_ready(&b,"CAN");
    clock_now=90; ams_perf_switch_in(&b,"CAN");
    clock_now=100; ams_perf_switch_out();
    assert(ams_perf_get(0,&row) && row.cycles==40 && row.max_slice_cycles==40 && row.max_unblock_cycles==10);
    assert(ams_perf_get(1,&row) && row.cycles==10 && row.max_unblock_cycles==20);
    assert(ams_perf_irq_cycles()==20);
    clock_now=UINT32_MAX-5; ams_perf_switch_in(&a,"ADBMS");
    clock_now=4; ams_perf_switch_out();
    assert(ams_perf_get(0,&row) && row.cycles==50 && row.slices==2);
    ams_perf_clear();
    assert(clock_now==4 && ams_perf_get(0,&row) && row.cycles==0 && row.max_slice_cycles==0);
    assert(ams_perf_irq_cycles()==0);
    int handles[20];
    for(unsigned i=0;i<20;i++) ams_perf_ready(handles+i,"bounded");
    assert(ams_perf_overflow()>0 && !ams_perf_get(16,&row));
    puts("PASS task/IRQ accounting, nested IRQ exclusion, unblock latency, wrap and bounded slots");
}
