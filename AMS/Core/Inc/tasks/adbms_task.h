/*
 * adbms_task.h
 *
 *  Created on: June 3, 2025
 *      Author: Cole Bardin
 */

#ifndef __ADBMS_TASK_H_
#define __ADBMS_TASK_H_

#include "app.h"
#include "cmsis_os.h"

TaskHandle_t adbms_task_start(app_data_t *data);

#endif /* __ADBMS_TASK_H_ */
