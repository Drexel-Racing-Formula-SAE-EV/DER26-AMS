#ifndef HOST_STUB_TASK_H_
#define HOST_STUB_TASK_H_

#include "FreeRTOS.h"

TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t delay);
void vTaskDelayUntil(TickType_t *previous_wake, TickType_t increment);
BaseType_t xTaskCreatePinnedToCore(
    void (*task)(void *),
    const char *name,
    uint32_t stack_depth,
    void *parameters,
    unsigned int priority,
    void *task_handle,
    int core);

#endif /* HOST_STUB_TASK_H_ */
