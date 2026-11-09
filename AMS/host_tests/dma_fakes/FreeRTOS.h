#ifndef DMA_TEST_FREERTOS_H
#define DMA_TEST_FREERTOS_H
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef struct { int token; } StaticSemaphore_t;
typedef StaticSemaphore_t *SemaphoreHandle_t;
#define pdFALSE 0
#define pdTRUE 1
#define pdMS_TO_TICKS(x) (x)
#define taskSCHEDULER_RUNNING 2
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define portYIELD_FROM_ISR(x) ((void)(x))
#endif
