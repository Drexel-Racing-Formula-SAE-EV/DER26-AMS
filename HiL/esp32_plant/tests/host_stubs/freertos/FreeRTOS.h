#ifndef HOST_STUB_FREERTOS_H_
#define HOST_STUB_FREERTOS_H_

#include <stdint.h>

typedef uint32_t TickType_t;
typedef int BaseType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdMS_TO_TICKS(value) ((TickType_t)(value))
#define portTICK_PERIOD_MS 1U
#define configASSERT(condition) ((void)(condition))

#endif /* HOST_STUB_FREERTOS_H_ */
