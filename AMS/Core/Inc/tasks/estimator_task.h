/*
 * estimator_task.h
 * Author: Mahad Faisal (2026)
 *
 * Advisory AMS SoC estimator task.
 */

#ifndef INC_TASKS_ESTIMATOR_TASK_H_
#define INC_TASKS_ESTIMATOR_TASK_H_

#include "app.h"
#include "cmsis_os.h"

TaskHandle_t estimator_task_start(app_data_t *data);
bool estimator_task_update(app_data_t *data, uint32_t now, float cc_dt_s);

#endif /* INC_TASKS_ESTIMATOR_TASK_H_ */
