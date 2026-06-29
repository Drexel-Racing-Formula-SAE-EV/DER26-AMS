/*
 * fan_task.h
 *
 *  Created on: Feb 5, 2024
 *      Author: Cassius Garcia
 */

#ifndef INC_TASKS_FAN_TASK_H_
#define INC_TASKS_FAN_TASK_H_

#include "app.h"
#include "cmsis_os.h"

TaskHandle_t fan_task_start(app_data_t *data);

#endif /* INC_TASKS_FAN_TASK_H_ */
