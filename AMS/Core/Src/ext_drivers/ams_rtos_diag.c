#include "ext_drivers/ams_rtos_diag.h"

#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "ext_drivers/ams_safety.h"

extern app_data_t app;

#if AMS_HOST_TEST
static uint32_t g_host_heap_free = 8192u;
static uint32_t g_host_heap_min = 4096u;
static uint16_t g_host_stack_hw[AMS_RTOS_TASK_COUNT];
#endif

static uint16_t g_last_logged_stack_warn_mask;
static bool g_last_logged_heap_warn;

static const uint16_t g_stack_words[AMS_RTOS_TASK_COUNT] =
{
    [AMS_RTOS_TASK_ERROR]     = AMS_STACK_ERROR_WORDS,
    [AMS_RTOS_TASK_CURRENT]   = AMS_STACK_CURRENT_WORDS,
    [AMS_RTOS_TASK_ADBMS]     = AMS_STACK_ADBMS_WORDS,
    [AMS_RTOS_TASK_CAN]       = AMS_STACK_CAN_WORDS,
    [AMS_RTOS_TASK_ESTIMATOR] = AMS_STACK_ESTIMATOR_WORDS,
    [AMS_RTOS_TASK_FAN]       = AMS_STACK_FAN_WORDS,
    [AMS_RTOS_TASK_AIR]       = AMS_STACK_AIR_WORDS,
    [AMS_RTOS_TASK_IMD]       = AMS_STACK_IMD_WORDS,
    [AMS_RTOS_TASK_CLI]       = AMS_STACK_CLI_WORDS,
};

static const uint8_t g_task_prio[AMS_RTOS_TASK_COUNT] =
{
    [AMS_RTOS_TASK_ERROR]     = ERR_PRIO,
    [AMS_RTOS_TASK_CURRENT]   = CUR_PRIO,
    [AMS_RTOS_TASK_ADBMS]     = ADBMS_PRIO,
    [AMS_RTOS_TASK_CAN]       = CAN_PRIO,
    [AMS_RTOS_TASK_ESTIMATOR] = EST_PRIO,
    [AMS_RTOS_TASK_FAN]       = FAN_PRIO,
    [AMS_RTOS_TASK_AIR]       = AIR_PRIO,
    [AMS_RTOS_TASK_IMD]       = IMD_PRIO,
    [AMS_RTOS_TASK_CLI]       = CLI_PRIO,
};

const char *ams_rtos_task_name(ams_rtos_task_id_t id)
{
    switch(id)
    {
    case AMS_RTOS_TASK_ERROR:     return "error";
    case AMS_RTOS_TASK_CURRENT:   return "current";
    case AMS_RTOS_TASK_ADBMS:     return "adbms";
    case AMS_RTOS_TASK_CAN:       return "can";
    case AMS_RTOS_TASK_ESTIMATOR: return "estimator";
    case AMS_RTOS_TASK_FAN:       return "fan";
    case AMS_RTOS_TASK_AIR:       return "air";
    case AMS_RTOS_TASK_IMD:       return "imd";
    case AMS_RTOS_TASK_CLI:       return "cli";
    default:                      return "unknown";
    }
}

uint16_t ams_rtos_task_config_stack_words(ams_rtos_task_id_t id)
{
    return (id < AMS_RTOS_TASK_COUNT) ? g_stack_words[id] : 0u;
}

uint8_t ams_rtos_task_priority(ams_rtos_task_id_t id)
{
    return (id < AMS_RTOS_TASK_COUNT) ? g_task_prio[id] : 0u;
}

const char *ams_rtos_fault_reason_str(uint8_t reason)
{
    switch((ams_rtos_fault_reason_t)reason)
    {
    case AMS_RTOS_FAULT_NONE:           return "none";
    case AMS_RTOS_FAULT_STACK_OVERFLOW: return "stack_overflow";
    case AMS_RTOS_FAULT_MALLOC_FAILED:  return "malloc_failed";
    case AMS_RTOS_FAULT_ASSERT_FAILED:  return "assert_failed";
    default:                            return "unknown";
    }
}

static TaskHandle_t ams_rtos_task_handle(const app_data_t *data, ams_rtos_task_id_t id)
{
    if(data == NULL)
    {
        return NULL;
    }

    switch(id)
    {
    case AMS_RTOS_TASK_ERROR:     return data->error_task;
    case AMS_RTOS_TASK_CURRENT:   return data->current_task;
    case AMS_RTOS_TASK_ADBMS:     return data->adbms_task;
    case AMS_RTOS_TASK_CAN:       return data->canbus_task;
    case AMS_RTOS_TASK_ESTIMATOR: return data->estimator_task;
    case AMS_RTOS_TASK_FAN:       return data->fan_task;
    case AMS_RTOS_TASK_AIR:       return data->air_task;
    case AMS_RTOS_TASK_IMD:       return data->imd_task;
    case AMS_RTOS_TASK_CLI:       return data->cli_task;
    default:                      return NULL;
    }
}

static ams_rtos_task_id_t ams_rtos_task_id_from_handle(TaskHandle_t handle)
{
    if(handle == NULL)
    {
        return AMS_RTOS_TASK_COUNT;
    }

    for(uint8_t i = 0u; i < (uint8_t)AMS_RTOS_TASK_COUNT; i++)
    {
        if(ams_rtos_task_handle(&app, (ams_rtos_task_id_t)i) == handle)
        {
            return (ams_rtos_task_id_t)i;
        }
    }

    return AMS_RTOS_TASK_COUNT;
}

static uint32_t ams_rtos_now_tick_safe(void)
{
#if AMS_HOST_TEST
    return osKernelGetTickCount();
#else
    if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        return osKernelGetTickCount();
    }
    return HAL_GetTick();
#endif
}

static void ams_rtos_set_fault(app_data_t *data,
                               ams_rtos_fault_reason_t reason,
                               ams_rtos_task_id_t task_id,
                               uint32_t arg)
{
    app_data_t *target = (data != NULL) ? data : &app;
    uint32_t flag = 0u;

    switch(reason)
    {
    case AMS_RTOS_FAULT_STACK_OVERFLOW:
        flag = AMS_RTOS_FAULT_FLAG_STACK_OVERFLOW;
        target->rtos_stack_overflow_count++;
        break;
    case AMS_RTOS_FAULT_MALLOC_FAILED:
        flag = AMS_RTOS_FAULT_FLAG_MALLOC_FAILED;
        target->rtos_malloc_fail_count++;
        break;
    case AMS_RTOS_FAULT_ASSERT_FAILED:
        flag = AMS_RTOS_FAULT_FLAG_ASSERT_FAILED;
        target->rtos_assert_fail_count++;
        target->rtos_last_assert_line = arg;
        break;
    default:
        break;
    }

    target->rtos_fault = true;
    target->rtos_fault_flags |= (uint16_t)flag;
    target->rtos_last_fault_reason = (uint8_t)reason;
    target->rtos_last_fault_task = (uint8_t)task_id;
    target->rtos_last_fault_tick = ams_rtos_now_tick_safe();
}

void ams_rtos_diag_init(app_data_t *data)
{
    if(data == NULL)
    {
        return;
    }

    data->rtos_heap_free_bytes = 0u;
    data->rtos_heap_min_ever_free_bytes = 0u;
    data->rtos_malloc_fail_count = 0u;
    data->rtos_stack_overflow_count = 0u;
    data->rtos_assert_fail_count = 0u;
    data->rtos_last_assert_line = 0u;
    data->rtos_last_fault_tick = 0u;
    data->rtos_min_stack_high_water_words = 0xFFFFu;
    data->rtos_stack_warn_mask = 0u;
    data->rtos_fault_flags = 0u;
    data->rtos_last_fault_reason = (uint8_t)AMS_RTOS_FAULT_NONE;
    data->rtos_last_fault_task = (uint8_t)AMS_RTOS_TASK_COUNT;
    data->rtos_fault = false;
    data->rtos_stack_warning = false;
    data->rtos_heap_warning = false;

    g_last_logged_stack_warn_mask = 0u;
    g_last_logged_heap_warn = false;

    for(uint8_t i = 0u; i < (uint8_t)AMS_RTOS_TASK_COUNT; i++)
    {
        data->rtos_stack_config_words[i] = g_stack_words[i];
        data->rtos_stack_high_water_words[i] = 0u;
    }
}

void ams_rtos_diag_update(app_data_t *data)
{
    uint16_t warn_mask = 0u;
    uint16_t min_hw = 0xFFFFu;

    if(data == NULL)
    {
        return;
    }

#if AMS_HOST_TEST
    data->rtos_heap_free_bytes = g_host_heap_free;
    data->rtos_heap_min_ever_free_bytes = g_host_heap_min;
#else
    data->rtos_heap_free_bytes = (uint32_t)xPortGetFreeHeapSize();
    data->rtos_heap_min_ever_free_bytes = (uint32_t)xPortGetMinimumEverFreeHeapSize();
#endif

    for(uint8_t i = 0u; i < (uint8_t)AMS_RTOS_TASK_COUNT; i++)
    {
        ams_rtos_task_id_t id = (ams_rtos_task_id_t)i;
        TaskHandle_t handle = ams_rtos_task_handle(data, id);
        uint16_t hw = 0u;

        data->rtos_stack_config_words[i] = g_stack_words[i];
        if(handle != NULL)
        {
#if AMS_HOST_TEST
            hw = g_host_stack_hw[i];
#else
            UBaseType_t raw_hw = uxTaskGetStackHighWaterMark(handle);
            hw = (raw_hw > 0xFFFFu) ? 0xFFFFu : (uint16_t)raw_hw;
#endif
            if(hw < min_hw)
            {
                min_hw = hw;
            }
            if(hw < AMS_RTOS_STACK_WARN_WORDS)
            {
                warn_mask |= AMS_RTOS_TASK_BIT(id);
            }
        }

        data->rtos_stack_high_water_words[i] = hw;
    }

    if(min_hw == 0xFFFFu)
    {
        min_hw = 0u;
    }

    data->rtos_min_stack_high_water_words = min_hw;
    data->rtos_stack_warn_mask = warn_mask;
    data->rtos_stack_warning = (warn_mask != 0u);
    data->rtos_heap_warning = (data->rtos_heap_min_ever_free_bytes < AMS_RTOS_HEAP_WARN_BYTES);

    if(data->rtos_stack_warning)
    {
        data->rtos_fault_flags |= AMS_RTOS_FAULT_FLAG_LOW_STACK_WARN;
    }
    else
    {
        data->rtos_fault_flags &= (uint16_t)~AMS_RTOS_FAULT_FLAG_LOW_STACK_WARN;
    }

    if(data->rtos_heap_warning)
    {
        data->rtos_fault_flags |= AMS_RTOS_FAULT_FLAG_LOW_HEAP_WARN;
    }
    else
    {
        data->rtos_fault_flags &= (uint16_t)~AMS_RTOS_FAULT_FLAG_LOW_HEAP_WARN;
    }

    if(warn_mask != g_last_logged_stack_warn_mask)
    {
        g_last_logged_stack_warn_mask = warn_mask;
        if(warn_mask != 0u)
        {
            ams_fault_log_event(AMS_FAULT_LOG_RTOS_LOW_STACK_WARN,
                                0u,
                                warn_mask,
                                min_hw);
        }
    }
    if(data->rtos_heap_warning != g_last_logged_heap_warn)
    {
        g_last_logged_heap_warn = data->rtos_heap_warning;
        if(data->rtos_heap_warning)
        {
            ams_fault_log_event(AMS_FAULT_LOG_RTOS_LOW_HEAP_WARN,
                                0u,
                                data->rtos_heap_free_bytes,
                                data->rtos_heap_min_ever_free_bytes);
        }
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)pcTaskName;
    ams_rtos_task_id_t id = ams_rtos_task_id_from_handle(xTask);
    ams_rtos_set_fault(&app, AMS_RTOS_FAULT_STACK_OVERFLOW, id, 0u);
#if !AMS_HOST_TEST
    taskDISABLE_INTERRUPTS();
#endif
    ams_safety_panic(AMS_PANIC_RTOS_STACK_OVERFLOW);
    for(;;)
    {
#if !AMS_HOST_TEST
        __NOP();
#endif
    }
}

void vApplicationMallocFailedHook(void)
{
    ams_rtos_set_fault(&app, AMS_RTOS_FAULT_MALLOC_FAILED, AMS_RTOS_TASK_COUNT, 0u);
    ams_fault_log_event(AMS_FAULT_LOG_RTOS_MALLOC_FAILED, 0u, app.rtos_malloc_fail_count, 0u);
#if !AMS_HOST_TEST
    taskDISABLE_INTERRUPTS();
#endif
    ams_safety_panic(AMS_PANIC_RTOS_MALLOC_FAILED);
    for(;;)
    {
#if !AMS_HOST_TEST
        __NOP();
#endif
    }
}

void ams_rtos_assert_failed(const char *file, int line)
{
    (void)file;
    ams_rtos_set_fault(&app,
                       AMS_RTOS_FAULT_ASSERT_FAILED,
                       AMS_RTOS_TASK_COUNT,
                       (uint32_t)line);
    ams_fault_log_event(AMS_FAULT_LOG_RTOS_ASSERT_FAILED,
                        0u,
                        (uint32_t)line,
                        app.rtos_assert_fail_count);
    ams_safety_panic(AMS_PANIC_RTOS_ASSERT_FAILED);
}

#if AMS_HOST_TEST
void ams_rtos_host_set_heap(uint32_t free_bytes, uint32_t min_ever_free_bytes)
{
    g_host_heap_free = free_bytes;
    g_host_heap_min = min_ever_free_bytes;
}

void ams_rtos_host_set_stack_high_water(ams_rtos_task_id_t id, uint16_t words)
{
    if(id < AMS_RTOS_TASK_COUNT)
    {
        g_host_stack_hw[id] = words;
    }
}

void ams_rtos_host_reset_state(void)
{
    g_host_heap_free = 8192u;
    g_host_heap_min = 4096u;
    g_last_logged_stack_warn_mask = 0u;
    g_last_logged_heap_warn = false;
    for(uint8_t i = 0u; i < (uint8_t)AMS_RTOS_TASK_COUNT; i++)
    {
        g_host_stack_hw[i] = (uint16_t)(g_stack_words[i] / 2u);
    }
}
#endif
