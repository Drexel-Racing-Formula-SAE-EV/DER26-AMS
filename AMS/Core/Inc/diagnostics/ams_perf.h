#ifndef AMS_PERF_H
#define AMS_PERF_H
#include <stdint.h>
#include <stdbool.h>
#define AMS_PERF_TASKS 16u
typedef struct {
    const void *task;
    char name[16];
    uint64_t cycles;
    uint32_t slices, max_slice_cycles, max_unblock_cycles;
} ams_perf_row_t;
void ams_perf_init(void);
void ams_perf_clear(void);
void ams_perf_switch_in(const void *task, const char *name);
void ams_perf_switch_out(void);
void ams_perf_ready(const void *task, const char *name);
void ams_perf_irq_enter(void);
void ams_perf_irq_exit(void);
bool ams_perf_get(unsigned index, ams_perf_row_t *out);
uint64_t ams_perf_irq_cycles(void);
uint32_t ams_perf_overflow(void);
#endif
