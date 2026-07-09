#ifndef AMS_RTOS_DIAG_H_
#define AMS_RTOS_DIAG_H_

#include <stdint.h>
#include "app.h"

void ams_rtos_diag_init(app_data_t *data);
void ams_rtos_diag_update(app_data_t *data);
const char *ams_rtos_task_name(ams_rtos_task_id_t id);
uint16_t ams_rtos_task_config_stack_words(ams_rtos_task_id_t id);
uint8_t ams_rtos_task_priority(ams_rtos_task_id_t id);
const char *ams_rtos_fault_reason_str(uint8_t reason);

void ams_rtos_assert_failed(const char *file, int line);

#if AMS_HOST_TEST
void ams_rtos_host_set_heap(uint32_t free_bytes, uint32_t min_ever_free_bytes);
void ams_rtos_host_set_stack_high_water(ams_rtos_task_id_t id, uint16_t words);
void ams_rtos_host_reset_state(void);
#endif

#endif /* AMS_RTOS_DIAG_H_ */
