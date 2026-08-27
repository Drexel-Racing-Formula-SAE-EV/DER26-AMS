#ifndef HOST_STUB_SEMPHR_H_
#define HOST_STUB_SEMPHR_H_

#include "FreeRTOS.h"

typedef void *SemaphoreHandle_t;

SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t wait);
BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore);

#endif /* HOST_STUB_SEMPHR_H_ */
