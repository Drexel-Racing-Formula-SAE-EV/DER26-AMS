/**
 * @file error_task.h
 * @author Ian Kennedy (ibk24@drexel.edu)
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

#endif
