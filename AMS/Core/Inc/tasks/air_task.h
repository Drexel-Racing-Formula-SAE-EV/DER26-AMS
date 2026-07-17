/*
 * air_task.h
 *
 *  Created on: Apr 3, 2024
 *      Author: Cole Bardin
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef __AIR_TASK_H_
#define __AIR_TASK_H_

#include "app.h"
#include "cmsis_os.h"

TaskHandle_t air_task_start(app_data_t *data);

/* Future board adapter hooks. The weak fail-closed implementations in
 * air_task.c return false. A target may set AMS_AIR_AUX_BOARD_ADAPTER_READY=1
 * only after providing reviewed strong definitions for both hooks. */
bool ams_air_board_get_config(ams_air_monitor_config_t *config);
bool ams_air_board_read_inputs(ams_air_monitor_inputs_t *inputs,
                               uint32_t now);

#endif /* __AIR_TASK_H_ */
