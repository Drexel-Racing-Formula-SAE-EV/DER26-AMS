/* Link-only clocks for unrelated 6830 code included by the APM unit runner.
 * These tests do not validate scheduling or target timing. */
#include <stdint.h>
uint32_t HAL_GetTick(void) { return 0u; }
uint32_t osKernelGetTickCount(void) { return 0u; }
