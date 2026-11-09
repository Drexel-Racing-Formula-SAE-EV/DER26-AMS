/* Host-only link shims for driver clock dependencies. Not target firmware. */
#include <stdint.h>
uint32_t HAL_GetTick(void) { return 0u; }
uint32_t osKernelGetTickCount(void) { return 0u; }
