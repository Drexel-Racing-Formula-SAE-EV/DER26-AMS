#include "ext_drivers/ams_safety.h"

#include <stdio.h>
#include <string.h>

#include "app.h"
#include "main.h"

extern app_data_t app;

#if defined(__GNUC__) && !defined(AMS_HOST_TEST)
#define AMS_NOINIT __attribute__((section(".noinit")))
#else
#define AMS_NOINIT
#endif

static ams_panic_record_t g_panic_record AMS_NOINIT;
static ams_fault_log_t g_fault_log AMS_NOINIT;
static uint32_t g_reset_flags;
static bool g_watchdog_hw_started;
static bool g_watchdog_runtime_enabled;
static bool g_watchdog_stop_feed_test;
static bool g_panic_active;

#if AMS_HOST_TEST
static uint32_t g_host_reset_csr;
static uint32_t g_host_cfsr;
static uint32_t g_host_hfsr;
static uint32_t g_host_mmfar;
static uint32_t g_host_bfar;
static bool g_host_bms_forced_low;
#endif


static void ams_safety_zero_bytes(void *ptr, size_t len)
{
    volatile uint8_t *p = (volatile uint8_t *)ptr;
    while(len-- > 0u)
    {
        *p++ = 0u;
    }
}

static void ams_panic_record_init_if_needed_raw(void)
{
    if(g_panic_record.magic != AMS_PANIC_RECORD_MAGIC)
    {
        ams_safety_zero_bytes(&g_panic_record, sizeof(g_panic_record));
        g_panic_record.magic = AMS_PANIC_RECORD_MAGIC;
    }
}

static void ams_fault_log_init_if_needed_raw(void)
{
    if((g_fault_log.magic != AMS_FAULT_LOG_MAGIC) ||
       (g_fault_log.write_index >= AMS_FAULT_LOG_DEPTH) ||
       (g_fault_log.count > AMS_FAULT_LOG_DEPTH))
    {
        ams_safety_zero_bytes(&g_fault_log, sizeof(g_fault_log));
        g_fault_log.magic = AMS_FAULT_LOG_MAGIC;
    }
}

static void ams_panic_record_init_if_needed(void)
{
    if(g_panic_record.magic != AMS_PANIC_RECORD_MAGIC)
    {
        memset(&g_panic_record, 0, sizeof(g_panic_record));
        g_panic_record.magic = AMS_PANIC_RECORD_MAGIC;
    }
}


#if !AMS_HOST_TEST
static void ams_safety_enable_gpio_clock(GPIO_TypeDef *port)
{
    if(port == GPIOA) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; }
    else if(port == GPIOB) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; }
    else if(port == GPIOC) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; }
    else if(port == GPIOD) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; }
    else if(port == GPIOE) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; }
    else if(port == GPIOF) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN; }
    else if(port == GPIOG) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN; }
    else if(port == GPIOH) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; }
#if defined(GPIOI) && defined(RCC_AHB1ENR_GPIOIEN)
    else if(port == GPIOI) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN; }
#endif
#if defined(GPIOJ) && defined(RCC_AHB1ENR_GPIOJEN)
    else if(port == GPIOJ) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN; }
#endif
#if defined(GPIOK) && defined(RCC_AHB1ENR_GPIOKEN)
    else if(port == GPIOK) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN; }
#endif
    else { /* Unsupported port: leave hardware defaults. */ }
}

static void ams_safety_force_gpio_output_low(GPIO_TypeDef *port, uint16_t pin_mask)
{
    if((port == NULL) || (pin_mask == 0u))
    {
        return;
    }

    ams_safety_enable_gpio_clock(port);
    __DSB();

    port->BSRR = ((uint32_t)pin_mask << 16u);

    for(uint32_t pos = 0u; pos < 16u; pos++)
    {
        uint32_t bit = (1u << pos);
        if(((uint32_t)pin_mask & bit) != 0u)
        {
            uint32_t shift = pos * 2u;
            port->MODER = (port->MODER & ~(0x3u << shift)) | (0x1u << shift);
            port->OTYPER &= ~bit;
            port->PUPDR &= ~(0x3u << shift);
        }
    }

    port->BSRR = ((uint32_t)pin_mask << 16u);
    __DSB();
    __ISB();
}
#endif

static void ams_fault_log_init_if_needed(void)
{
    if((g_fault_log.magic != AMS_FAULT_LOG_MAGIC) ||
       (g_fault_log.write_index >= AMS_FAULT_LOG_DEPTH) ||
       (g_fault_log.count > AMS_FAULT_LOG_DEPTH))
    {
        memset(&g_fault_log, 0, sizeof(g_fault_log));
        g_fault_log.magic = AMS_FAULT_LOG_MAGIC;
    }
}

static uint32_t ams_safety_now_tick(void)
{
#if AMS_HOST_TEST
    return osKernelGetTickCount();
#else
    return HAL_GetTick();
#endif
}

void ams_safety_force_bms_low(void)
{
#if AMS_HOST_TEST
    app.bms_state = false;
    g_host_bms_forced_low = true;
    HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, GPIO_PIN_RESET);
#else
    app.bms_state = false;
    ams_safety_force_gpio_output_low(BMS_OK_GPIO_Port, BMS_OK_Pin);
#endif
}

void ams_fault_log_event(ams_fault_log_event_t event,
                         uint16_t reason,
                         uint32_t arg0,
                         uint32_t arg1)
{
    ams_fault_log_init_if_needed();

    uint32_t index = g_fault_log.write_index % AMS_FAULT_LOG_DEPTH;
    g_fault_log.entry[index].tick = ams_safety_now_tick();
    g_fault_log.entry[index].event = (uint16_t)event;
    g_fault_log.entry[index].reason = reason;
    g_fault_log.entry[index].arg0 = arg0;
    g_fault_log.entry[index].arg1 = arg1;
    g_fault_log.write_index = (index + 1u) % AMS_FAULT_LOG_DEPTH;
    if(g_fault_log.count < AMS_FAULT_LOG_DEPTH)
    {
        g_fault_log.count++;
    }
}

const ams_fault_log_t *ams_fault_log_get(void)
{
    ams_fault_log_init_if_needed();
    return &g_fault_log;
}

void ams_fault_log_clear(void)
{
    memset(&g_fault_log, 0, sizeof(g_fault_log));
    g_fault_log.magic = AMS_FAULT_LOG_MAGIC;
}

const char *ams_fault_log_event_str(uint16_t event)
{
    switch((ams_fault_log_event_t)event)
    {
    case AMS_FAULT_LOG_BOOT: return "BOOT";
    case AMS_FAULT_LOG_RESET_CAUSE: return "RESET_CAUSE";
    case AMS_FAULT_LOG_PANIC: return "PANIC";
    case AMS_FAULT_LOG_BMS_OK_ASSERTED: return "BMS_OK_ASSERTED";
    case AMS_FAULT_LOG_BMS_OK_DROPPED: return "BMS_OK_DROPPED";
    case AMS_FAULT_LOG_VOLTAGE_LATCH: return "VOLTAGE_LATCH";
    case AMS_FAULT_LOG_TEMP_LATCH: return "TEMP_LATCH";
    case AMS_FAULT_LOG_CURRENT_LATCH: return "CURRENT_LATCH";
    case AMS_FAULT_LOG_ADBMS_DIAG_FAIL: return "ADBMS_DIAG_FAIL";
    case AMS_FAULT_LOG_CAN_BUS_OFF: return "CAN_BUS_OFF";
    case AMS_FAULT_LOG_CAN_RECOVERED: return "CAN_RECOVERED";
    case AMS_FAULT_LOG_WATCHDOG_FEED_STOPPED: return "WATCHDOG_FEED_STOPPED";
    case AMS_FAULT_LOG_FAULT_CLEAR_ACCEPTED: return "FAULT_CLEAR_ACCEPTED";
    case AMS_FAULT_LOG_FAULT_CLEAR_REJECTED: return "FAULT_CLEAR_REJECTED";
    default: return "UNKNOWN";
    }
}

static void ams_fault_log_event_raw_tick(uint32_t tick,
                                         ams_fault_log_event_t event,
                                         uint16_t reason,
                                         uint32_t arg0,
                                         uint32_t arg1)
{
    ams_fault_log_init_if_needed_raw();

    uint32_t index = g_fault_log.write_index % AMS_FAULT_LOG_DEPTH;
    g_fault_log.entry[index].tick = tick;
    g_fault_log.entry[index].event = (uint16_t)event;
    g_fault_log.entry[index].reason = reason;
    g_fault_log.entry[index].arg0 = arg0;
    g_fault_log.entry[index].arg1 = arg1;
    g_fault_log.write_index = (index + 1u) % AMS_FAULT_LOG_DEPTH;
    if(g_fault_log.count < AMS_FAULT_LOG_DEPTH)
    {
        g_fault_log.count++;
    }
}

void ams_safety_panic(ams_panic_reason_t reason)
{
    ams_safety_force_bms_low();
    ams_panic_record_init_if_needed_raw();

    g_panic_active = true;
    g_panic_record.panic_reason = (uint32_t)reason;
#if AMS_HOST_TEST
    g_panic_record.cfsr = g_host_cfsr;
    g_panic_record.hfsr = g_host_hfsr;
    g_panic_record.mmfar = g_host_mmfar;
    g_panic_record.bfar = g_host_bfar;
#else
    g_panic_record.cfsr = SCB->CFSR;
    g_panic_record.hfsr = SCB->HFSR;
    g_panic_record.mmfar = SCB->MMFAR;
    g_panic_record.bfar = SCB->BFAR;
#endif
    g_panic_record.reset_count++;

    /* Panic context may be HardFault/NMI. Do not depend on HAL, FreeRTOS,
     * mutexes, UART, or scheduler state after the BMS_OK-low write. */
    ams_fault_log_event_raw_tick(0u,
                                 AMS_FAULT_LOG_PANIC,
                                 (uint16_t)reason,
                                 g_panic_record.cfsr,
                                 g_panic_record.hfsr);
}

bool ams_safety_panic_active(void)
{
    return g_panic_active;
}

void ams_safety_unexpected_irq_panic_loop(void)
{
#if !AMS_HOST_TEST
    __disable_irq();
#endif
    ams_safety_panic(AMS_PANIC_DEFAULT_HANDLER);
    for(;;)
    {
#if !AMS_HOST_TEST
        __NOP();
#endif
    }
}

void ams_safety_enable_cpu_faults(void)
{
#if !AMS_HOST_TEST
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_USGFAULTENA_Msk;
#endif
}

void ams_safety_record_reset_cause(void)
{
    ams_panic_record_init_if_needed();
    ams_fault_log_init_if_needed();

#if AMS_HOST_TEST
    g_reset_flags = g_host_reset_csr;
#else
    g_reset_flags = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
#endif

    ams_fault_log_event(AMS_FAULT_LOG_RESET_CAUSE, 0u, g_reset_flags, g_panic_record.panic_reason);
}

void ams_safety_sync_app(app_data_t *data)
{
    if(data == NULL)
    {
        return;
    }

    ams_panic_record_init_if_needed();
    ams_fault_log_init_if_needed();

    data->reset_flags = g_reset_flags;
    data->last_panic_reason = g_panic_record.panic_reason;
    data->safety_panic_count = g_panic_record.reset_count;
    data->watchdog_runtime_enabled = g_watchdog_runtime_enabled;
    data->watchdog_hw_started = g_watchdog_hw_started;
    data->watchdog_last_block_reason = AMS_WATCHDOG_BLOCK_NONE;
}

uint32_t ams_safety_reset_flags(void)
{
    return g_reset_flags;
}

const ams_panic_record_t *ams_safety_panic_record(void)
{
    ams_panic_record_init_if_needed();
    return &g_panic_record;
}

const char *ams_safety_panic_reason_str(uint32_t reason)
{
    switch((ams_panic_reason_t)reason)
    {
    case AMS_PANIC_NONE: return "none";
    case AMS_PANIC_NMI: return "nmi";
    case AMS_PANIC_HARDFAULT: return "hardfault";
    case AMS_PANIC_MEMMANAGE: return "memmanage";
    case AMS_PANIC_BUSFAULT: return "busfault";
    case AMS_PANIC_USAGEFAULT: return "usagefault";
    case AMS_PANIC_ERROR_HANDLER: return "error_handler";
    case AMS_PANIC_ASSERT_FAILED: return "assert_failed";
    case AMS_PANIC_DEFAULT_HANDLER: return "default_handler";
    case AMS_PANIC_SCHEDULER_RETURNED: return "scheduler_returned";
    case AMS_PANIC_TASK_CREATE_FAILED: return "task_create_failed";
    case AMS_PANIC_LIBC_EXIT: return "libc_exit";
    case AMS_PANIC_FAULT_INJECTION_HARDFAULT: return "inject_hardfault";
    case AMS_PANIC_FAULT_INJECTION_BUSFAULT: return "inject_busfault";
    default: return "unknown";
    }
}

void ams_safety_format_reset_flags(uint32_t flags, char *buf, size_t len)
{
    if((buf == NULL) || (len == 0u))
    {
        return;
    }

    (void)snprintf(buf,
                   len,
                   "IWDG=%u WWDG=%u SW=%u PIN=%u BOR=%u POR=%u LPWR=%u raw=0x%08lX",
                   (unsigned)((flags & RCC_CSR_IWDGRSTF) != 0u),
                   (unsigned)((flags & RCC_CSR_WWDGRSTF) != 0u),
                   (unsigned)((flags & RCC_CSR_SFTRSTF) != 0u),
                   (unsigned)((flags & RCC_CSR_PINRSTF) != 0u),
                   (unsigned)((flags & RCC_CSR_BORRSTF) != 0u),
                   (unsigned)((flags & RCC_CSR_PORRSTF) != 0u),
                   (unsigned)((flags & RCC_CSR_LPWRRSTF) != 0u),
                   (unsigned long)flags);
}

const char *ams_safety_watchdog_block_reason_str(uint32_t reason)
{
    switch((ams_watchdog_block_reason_t)reason)
    {
    case AMS_WATCHDOG_BLOCK_NONE: return "none";
    case AMS_WATCHDOG_BLOCK_NOT_ENABLED: return "not_enabled";
    case AMS_WATCHDOG_BLOCK_PANIC: return "panic";
    case AMS_WATCHDOG_BLOCK_STARTUP_GRACE: return "startup_grace";
    case AMS_WATCHDOG_BLOCK_HEARTBEAT: return "heartbeat";
    case AMS_WATCHDOG_BLOCK_ADBMS_STALE: return "adbms_stale";
    case AMS_WATCHDOG_BLOCK_CURRENT_STALE: return "current_stale";
    case AMS_WATCHDOG_BLOCK_TEMP_STALE: return "temp_stale";
    case AMS_WATCHDOG_BLOCK_HARD_FAULT: return "hard_fault";
    case AMS_WATCHDOG_BLOCK_STOP_FEED_TEST: return "stop_feed_test";
    default: return "unknown";
    }
}

static bool watchdog_start_hw(void)
{
#if !AMS_ENABLE_IWDG
    return false;
#elif AMS_HOST_TEST
    g_watchdog_hw_started = true;
    return true;
#else
    const uint32_t prescaler_code = 4u; /* /64 */
    uint32_t reload = ((AMS_IWDG_TIMEOUT_MS * 32u) / 64u);
    if(reload == 0u)
    {
        reload = 1u;
    }
    if(reload > 4095u)
    {
        reload = 4095u;
    }
    reload -= 1u;

    IWDG->KR = 0x5555u;
    IWDG->PR = prescaler_code;
    IWDG->RLR = reload;

    for(uint32_t timeout = 0u; ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0u) && (timeout < 100000u); timeout++)
    {
        __NOP();
    }

    IWDG->KR = 0xCCCCu;
    IWDG->KR = 0xAAAAu;
    g_watchdog_hw_started = true;
    return true;
#endif
}

static void watchdog_feed_hw(void)
{
#if AMS_ENABLE_IWDG
#if AMS_HOST_TEST
    /* Host tests only count feeds through app_data_t. */
#else
    if(g_watchdog_hw_started)
    {
        IWDG->KR = 0xAAAAu;
    }
#endif
#endif
}

void ams_safety_watchdog_enable_runtime(app_data_t *data, bool enable)
{
#if AMS_ENABLE_IWDG
    /* Do not start the irreversible hardware IWDG from the CLI/control path.
     * It starts only from ams_safety_watchdog_task_update() after the error
     * task has evaluated the full health gate and is ready to feed it. */
    g_watchdog_runtime_enabled = enable;
#else
    (void)enable;
    g_watchdog_runtime_enabled = false;
#endif

    if(data != NULL)
    {
        data->watchdog_runtime_enabled = g_watchdog_runtime_enabled;
        data->watchdog_hw_started = g_watchdog_hw_started;
    }
}

bool ams_safety_watchdog_hw_started(void)
{
    return g_watchdog_hw_started;
}

bool ams_safety_watchdog_ok(const app_data_t *data)
{
    uint32_t now;
    bool startup_grace;

    if(data == NULL)
    {
        return false;
    }

    now = ams_safety_now_tick();
    startup_grace = (now - data->heartbeat.boot_tick) < AMS_HEARTBEAT_STARTUP_GRACE_MS;

    if(!g_watchdog_runtime_enabled)
    {
        return false;
    }
    if(g_panic_active)
    {
        return false;
    }
#if AMS_FAULT_INJECTION_CLI
    if(g_watchdog_stop_feed_test)
    {
        return false;
    }
#endif
    if(data->hard_fault || data->charger_fault || data->adbms_diag_fault || data->fuse_fault)
    {
        return false;
    }
    if(startup_grace)
    {
        return false;
    }
    if(data->task_heartbeat_fault || (data->heartbeat.safety_stale_mask != 0u))
    {
        return false;
    }
    if(!data->voltage_valid || data->voltage_read_fault || data->voltage_fault)
    {
        return false;
    }
    if(!data->current_valid || data->current_fault || data->current_sensor_fault)
    {
        return false;
    }
    if(!data->temp_valid || data->temp_read_fault || data->temp_fault)
    {
        return false;
    }

    return true;
}

void ams_safety_watchdog_task_update(app_data_t *data)
{
    uint32_t now;
    ams_watchdog_block_reason_t reason = AMS_WATCHDOG_BLOCK_NONE;

    if(data == NULL)
    {
        return;
    }

    now = ams_safety_now_tick();
    data->watchdog_runtime_enabled = g_watchdog_runtime_enabled;
    data->watchdog_hw_started = g_watchdog_hw_started;

    if(!g_watchdog_runtime_enabled)
    {
        reason = AMS_WATCHDOG_BLOCK_NOT_ENABLED;
    }
    else if(g_panic_active)
    {
        reason = AMS_WATCHDOG_BLOCK_PANIC;
    }
#if AMS_FAULT_INJECTION_CLI
    else if(g_watchdog_stop_feed_test)
    {
        reason = AMS_WATCHDOG_BLOCK_STOP_FEED_TEST;
    }
#endif
    else if(data->hard_fault || data->charger_fault || data->adbms_diag_fault || data->fuse_fault)
    {
        reason = AMS_WATCHDOG_BLOCK_HARD_FAULT;
    }
    else if((now - data->heartbeat.boot_tick) < AMS_HEARTBEAT_STARTUP_GRACE_MS)
    {
        reason = AMS_WATCHDOG_BLOCK_STARTUP_GRACE;
    }
    else if(data->task_heartbeat_fault || (data->heartbeat.safety_stale_mask != 0u))
    {
        reason = AMS_WATCHDOG_BLOCK_HEARTBEAT;
    }
    else if(!data->voltage_valid || data->voltage_read_fault || data->voltage_fault)
    {
        reason = AMS_WATCHDOG_BLOCK_ADBMS_STALE;
    }
    else if(!data->current_valid || data->current_fault || data->current_sensor_fault)
    {
        reason = AMS_WATCHDOG_BLOCK_CURRENT_STALE;
    }
    else if(!data->temp_valid || data->temp_read_fault || data->temp_fault)
    {
        reason = AMS_WATCHDOG_BLOCK_TEMP_STALE;
    }

    if(reason == AMS_WATCHDOG_BLOCK_NONE)
    {
        if(!g_watchdog_hw_started)
        {
            (void)watchdog_start_hw();
            data->watchdog_hw_started = g_watchdog_hw_started;
        }
        watchdog_feed_hw();
        data->watchdog_feed_count++;
        data->watchdog_last_feed_tick = now;
        data->watchdog_last_block_reason = AMS_WATCHDOG_BLOCK_NONE;
    }
    else
    {
        data->watchdog_last_block_reason = (uint32_t)reason;

        /* Disabled/default and startup-grace states are status, not fault-log
         * events. Only count/log feed stoppage after the runtime watchdog gate
         * is enabled and the block reason represents an actual health problem. */
        if((reason != AMS_WATCHDOG_BLOCK_NOT_ENABLED) &&
           (reason != AMS_WATCHDOG_BLOCK_STARTUP_GRACE))
        {
            data->watchdog_block_count++;
            if(data->watchdog_last_logged_block_reason != (uint32_t)reason)
            {
                data->watchdog_last_logged_block_reason = (uint32_t)reason;
                ams_fault_log_event(AMS_FAULT_LOG_WATCHDOG_FEED_STOPPED,
                                    (uint16_t)reason,
                                    data->heartbeat.safety_stale_mask,
                                    data->watchdog_block_count);
            }
        }
    }
}

#if AMS_FAULT_INJECTION_CLI
void ams_safety_fault_inject_hardfault(void)
{
    ams_safety_panic(AMS_PANIC_FAULT_INJECTION_HARDFAULT);
}

void ams_safety_fault_inject_busfault(void)
{
    ams_safety_panic(AMS_PANIC_FAULT_INJECTION_BUSFAULT);
}

void ams_safety_watchdog_stop_feed_for_test(bool stop)
{
    g_watchdog_stop_feed_test = stop;
}
#endif

#if AMS_HOST_TEST
void ams_safety_host_reset_state(void)
{
    memset(&g_panic_record, 0, sizeof(g_panic_record));
    memset(&g_fault_log, 0, sizeof(g_fault_log));
    g_reset_flags = 0u;
    g_watchdog_hw_started = false;
    g_watchdog_runtime_enabled = false;
    g_watchdog_stop_feed_test = false;
    g_panic_active = false;
    g_host_reset_csr = 0u;
    g_host_cfsr = 0u;
    g_host_hfsr = 0u;
    g_host_mmfar = 0u;
    g_host_bfar = 0u;
    g_host_bms_forced_low = false;
}

void ams_safety_host_set_reset_csr(uint32_t csr)
{
    g_host_reset_csr = csr;
}

void ams_safety_host_set_fault_regs(uint32_t cfsr, uint32_t hfsr, uint32_t mmfar, uint32_t bfar)
{
    g_host_cfsr = cfsr;
    g_host_hfsr = hfsr;
    g_host_mmfar = mmfar;
    g_host_bfar = bfar;
}

bool ams_safety_host_bms_forced_low(void)
{
    return g_host_bms_forced_low;
}
#endif
