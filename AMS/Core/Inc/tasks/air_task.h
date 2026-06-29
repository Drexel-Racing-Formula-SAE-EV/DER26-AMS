/*
 * air_task.h
 *
 *  Created on: Apr 3, 2024
 *      Author: Cole Bardin
 */

#ifndef __AIR_TASK_H_
#define __AIR_TASK_H_

#include "app.h"
#include "cmsis_os.h"

TaskHandle_t air_task_start(app_data_t *data);

#endif /* __AIR_TASK_H_ */
