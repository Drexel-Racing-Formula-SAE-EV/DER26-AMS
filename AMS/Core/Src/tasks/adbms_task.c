/*
 * adbms_task.h
 *
 *  Created on: June 3, 2025
 *      Author: Cole Bardin
 */

#include <tasks/adbms_task.h>

void adbms_task_fn(void *argument);

static void adbms_task_publish_voltage_state(app_data_t *data)
{
    voltage_fault_state_t *fault = &data->voltage_fault_state;

    data->voltage_valid = fault->voltage_valid;
    data->voltage_read_fault = fault->read_fault;
    data->voltage_warning = fault->warning;
    data->charge_voltage_stop = fault->charge_stop;
    data->overvoltage_fault = fault->overvoltage_fault;
    data->undervoltage_fault = fault->undervoltage_fault;
    data->voltage_fault_latched = fault->latched;
    data->voltage_fault_reason = fault->reason;
    data->voltage_fault_latched_reason = fault->latched_reason;
    data->voltage_usable_cell_count = fault->usable_cell_count;
    data->voltage_updated_cell_count = fault->updated_cell_count;
    data->voltage_stale_cell_count = fault->stale_cell_count;
    data->max_voltage_seg = fault->max_cell_segment;
    data->max_voltage_cell = fault->max_cell_index;
    data->min_voltage_seg = fault->min_cell_segment;
    data->min_voltage_cell = fault->min_cell_index;

    data->voltage_fault = (fault->read_fault ||
                           fault->overvoltage_fault ||
                           fault->undervoltage_fault ||
                           fault->latched);
}

static void adbms_task_publish_temperature_state(app_data_t *data)
{
    temperature_fault_state_t *fault = &data->temp_fault_state;

    data->temp_valid = fault->temp_valid;
    data->temp_read_fault = fault->read_fault;
    data->temp_warning = fault->warning;
    data->temp_fan_max = fault->fan_max;
    data->temp_charge_stop = fault->charge_stop;
    data->temp_overtemp_pending = fault->pending;
    data->overtemp_fault = fault->overtemp_fault;
    data->severe_overtemp_fault = fault->severe_overtemp_fault;
    data->temp_fault_latched = fault->latched;
    data->temp_fault_reason = fault->reason;
    data->temp_fault_pending_reason = fault->pending_reason;
    data->temp_fault_latched_reason = fault->latched_reason;
    data->temp_fault_pending_ms = fault->pending_ms;
    data->temp_usable_sensor_count = fault->usable_sensor_count;
    data->temp_updated_sensor_count = fault->updated_sensor_count;
    data->temp_stale_sensor_count = fault->stale_sensor_count;
    data->temp_invalid_sensor_count = fault->invalid_sensor_count;
    data->max_temp_seg = fault->max_temp_segment;
    data->max_temp_sensor = fault->max_temp_sensor;
    data->min_temp_seg = fault->min_temp_segment;
    data->min_temp_sensor = fault->min_temp_sensor;

    data->temp_fault = (fault->read_fault ||
                        fault->overtemp_fault ||
                        fault->latched);
}

TaskHandle_t adbms_task_start(app_data_t *data)
{
    TaskHandle_t handle = NULL;

    if(data == NULL)
    {
        return NULL;
    }

    xTaskCreate(adbms_task_fn, "adbms task", 1024, (void *)data, ADBMS_PRIO, &handle);
    return handle;
}

void adbms_task_fn(void *argument)
{
	app_data_t *data = (app_data_t *) argument;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

	accumulator_t *acc = &data->acc;
	uint32_t entry;

	for(;;)
	{
        entry = osKernelGetTickCount();

        /* Turn off balancing before reading so cell voltages recover from load. */
        accumulator_clear_balance(acc);
        osDelay(100);

        (void)accumulator_read_volt(acc);
        accumulator_update_voltage_stats_at(acc, osKernelGetTickCount());

        data->max_voltage = acc->max_volt;
        data->min_voltage = acc->min_volt;
        data->total_voltage = acc->total_volt;

        voltage_fault_update(&data->voltage_fault_state, acc);
        adbms_task_publish_voltage_state(data);

        if(data->voltage_fault)
        {
            set_bms(0);
        }
        ams_heartbeat_kick(data, AMS_HEARTBEAT_ADBMS, osKernelGetTickCount());

        (void)accumulator_read_temp(acc);
        accumulator_update_temp_stats_at(acc, osKernelGetTickCount());
        data->max_temp = acc->max_temp;
        data->avg_temp = acc->avg_temp;

        temperature_fault_update_with_period(&data->temp_fault_state,
                                             acc,
                                             (1000u / ADBMS_FREQ));
        adbms_task_publish_temperature_state(data);

        if(data->temp_fault)
        {
            set_bms(0);
        }
        ams_heartbeat_kick(data, AMS_HEARTBEAT_TEMP, osKernelGetTickCount());

        bool bms_ok_ready = (data->voltage_valid &&
                             !data->voltage_fault &&
                             data->temp_valid &&
                             !data->temp_fault &&
                             !data->task_heartbeat_fault &&
                             ((data->state != STATE_CHARGE) || !data->temp_charge_stop) &&
                             !data->fuse_fault &&
                             !data->charger_fault &&
                             !data->hard_fault &&
                             data->current_valid &&
                             !data->current_fault);
        set_bms(bms_ok_ready);

        /* Voltage charge-stop can still balance; hard faults/temp stop cannot. */
        if((data->state == STATE_CHARGE) &&
           data->voltage_valid &&
           !data->voltage_fault &&
           data->temp_valid &&
           !data->temp_charge_stop &&
           !data->temp_fault &&
           !data->hard_fault &&
           !data->current_fault &&
           data->current_valid &&
           data->bms_state)
        {
            (void)accumulator_set_balance(acc);
        }
        else
        {
            accumulator_clear_balance(acc);
        }

        osDelayUntil(entry + (1000 / ADBMS_FREQ));
	}
}
