/*
 * current_task.h
 *
 *  Created on: Apr 15th, 2024
 *      Author: Justin
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef CURRENT_TASK_H_
#define CURRENT_TASK_H_

#include "app.h"
#include "cmsis_os.h"

TaskHandle_t current_task_start(app_data_t *data);

#endif /* CURRENT_TASK_H_ */
