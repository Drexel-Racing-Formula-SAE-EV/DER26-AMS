/**
 * @file error_task.h
 * @author Ian Kennedy (ibk24@drexel.edu)
 * @author Mahad Faisal (major firmware updates, 2026)
 * @brief
 * @version 0.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef __ERROR_TASK_H_
#define __ERROR_TASK_H_

#include "app.h"

#include "cmsis_os.h"

/**
 * @brief Starts the ERROR task
 *
 * @param data App data structure pointer
 * @return TaskHandle_t Handle used for task
 */
TaskHandle_t error_task_start(app_data_t *data);

/* Executes one safety-supervisor evaluation without delaying.  The task loop
 * calls this every ERR_FREQ period; host/SIL tests use the same function so
 * BMS_OK ownership is tested without duplicating the decision logic. */
void error_task_update(app_data_t *data, uint32_t now);

#endif
