/*
*   imd_task.h
*   Created: 3/25/2024
*   Author: Brendan Hoag
*   Purpose: Device driver task for IR151-3204 ground fault monitoring system
*   datasheet: https://www.benderinc.com/products/ground-fault-monitoring-ungrounded/isometer-ir155-03-04-series/
*/

#ifndef INC_TASKS_IMD_TASK_H_
#define INC_TASKS_IMD_TASK_H_

#include "app.h"
#include "cmsis_os.h"
#include "ext_drivers/imd.h"

/*
* function: imd_task_start
* ------------------
*
* data - an app_data_t pointer representing the app containing the imd we want to run
*/
TaskHandle_t imd_task_start(app_data_t *data);

#endif
