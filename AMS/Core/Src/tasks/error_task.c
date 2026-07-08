/**
 * @file error_task.c
 * @author Ian Kennedy (ibk24@drexel.edu)
 * @author Mahad Faisal (major firmware updates, 2026)
 * @brief
 * @version 0.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "main.h"
#include "tasks/error_task.h"
#include "ext_drivers/ams_safety.h"

/**
 * @brief Actual ERROR task function
 *
 * @param arg App_data struct pointer converted to void pointer
 */
void error_task_fn(void *arg);

TaskHandle_t error_task_start(app_data_t *data)
{
    TaskHandle_t handle = NULL;

    if(data == NULL)
    {
        return NULL;
    }

    xTaskCreate(error_task_fn, "ERROR task", 128, (void *)data, ERR_PRIO, &handle);
    return handle;
}

void error_task_fn(void *arg)
{
	app_data_t *data = (app_data_t *)arg;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }


    uint32_t entry;

    for(;;)
    {
        entry = osKernelGetTickCount();

        data->air_state = HAL_GPIO_ReadPin(AIR_CTRL_GPIO_Port, AIR_CTRL_Pin);
        (void)ams_heartbeat_update(data, entry);


	        data->hard_fault = (data->fuse_fault ||
	                            data->temp_fault ||
	                            data->voltage_fault ||
	                            data->charger_fault ||
	                            data->adbms_diag_fault ||
	                            data->task_heartbeat_fault ||
	                            data->current_overcurrent_fault ||
	                            data->current_fault_latched);

        data->soft_fault = (data->cli_fault ||
                            data->canbus_fault ||
                            data->logger_heartbeat_fault ||
                            data->current_sensor_fault ||
                            data->current_overcurrent_warning ||
                            data->current_overcurrent_pending ||
                            data->temp_warning ||
                            data->temp_overtemp_pending ||
                            data->fan_fault);

        if(data->hard_fault)
        {
            set_bms(0);
        }

        ams_safety_watchdog_task_update(data);
//        data->cascadia_error = HAL_GPIO_ReadPin(MTR_Fault_GPIO_Port, MTR_Fault_Pin);
//		data->imd_fail = HAL_GPIO_ReadPin(IMD_Fail_GPIO_Port, IMD_Fail_Pin);
//		data->bms_fail = HAL_GPIO_ReadPin(BMS_Fail_GPIO_Port, BMS_Fail_Pin);
//		data->bspd_fail = HAL_GPIO_ReadPin(BSPD_Fail_GPIO_Port, BSPD_Fail_Pin);
//
//		if(!data->board.ams.air_state && !data->imd_fail && !data->bms_fail && !data->bspd_fail) set_ssa(100);
//		else set_ssa(0);
//
////		data->hard_fault = (data->apps_fault ||
////				            data->bse_fault ||
////							data->coolant_fault ||
////							data->cascadia_error
////						    );
////		data->hard_fault = (data->coolant_fault ||
////							data->cascadia_error
////						    );
//
//        data->soft_fault =(data->bppc_fault ||
//        				   data->cli_fault ||
//						   data->acc_fault ||
//						   data->canbus_fault ||
//						   data->dashboard_fault
//						   );
//
//        if(data->fw_override) set_ecu_ok(data->fw_override_state);
//        else set_ecu_ok(!data->coolant_fault);
//        // I believe this needs to be set low on an APPS/BSE fault (rules say disable inverter but no need to disable tractive system)
//        set_cascadia_enable(!data->hard_fault);
//
//        if(data->hard_fault){
//        	set_ecu_ok(0);
//        }
        osDelayUntil(entry + (1000 / ERR_FREQ));
    }
}
