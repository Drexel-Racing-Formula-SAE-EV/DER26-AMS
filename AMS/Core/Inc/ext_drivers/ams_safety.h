#ifndef AMS_SAFETY_H_
#define AMS_SAFETY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ams_build_profile.h"
#include "stm32f7xx_hal.h"

#ifndef AMS_ENABLE_IWDG
#define AMS_ENABLE_IWDG 0
#endif

#ifndef AMS_FAULT_INJECTION_CLI
#define AMS_FAULT_INJECTION_CLI 0
#endif

#ifndef AMS_IWDG_TIMEOUT_MS
#define AMS_IWDG_TIMEOUT_MS 5000u
#endif

#define AMS_PANIC_RECORD_MAGIC 0x414D5350u /* 'AMSP' */
#define AMS_FAULT_LOG_MAGIC    0x414D534Cu /* 'AMSL' */
#define AMS_FAULT_LOG_VERSION  2u
#define AMS_FAULT_LOG_ENTRY_COMMIT 0xC04D17EDu
#define AMS_FAULT_LOG_DEPTH    32u

#ifndef AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS
#define AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS 1000u
#endif

#ifndef AMS_CAN_ERROR_SOFT_HOLD_MS
#define AMS_CAN_ERROR_SOFT_HOLD_MS 1000u
#endif

typedef struct app_data_t app_data_t;

typedef enum
{
    AMS_PANIC_NONE = 0,
    AMS_PANIC_NMI,
    AMS_PANIC_HARDFAULT,
    AMS_PANIC_MEMMANAGE,
    AMS_PANIC_BUSFAULT,
    AMS_PANIC_USAGEFAULT,
    AMS_PANIC_ERROR_HANDLER,
    AMS_PANIC_ASSERT_FAILED,
    AMS_PANIC_DEFAULT_HANDLER,
    AMS_PANIC_SCHEDULER_RETURNED,
    AMS_PANIC_TASK_CREATE_FAILED,
    AMS_PANIC_LIBC_EXIT,
    AMS_PANIC_RTOS_STACK_OVERFLOW,
    AMS_PANIC_RTOS_MALLOC_FAILED,
    AMS_PANIC_RTOS_ASSERT_FAILED,
    AMS_PANIC_FAULT_INJECTION_HARDFAULT,
    AMS_PANIC_FAULT_INJECTION_BUSFAULT,
    AMS_PANIC_MUTEX_CREATE_FAILED,
    AMS_PANIC_MUTEX_ACQUIRE_FAILED,
    AMS_PANIC_MUTEX_RELEASE_FAILED,
} ams_panic_reason_t;

typedef enum
{
    AMS_WATCHDOG_BLOCK_NONE = 0,
    AMS_WATCHDOG_BLOCK_NOT_ENABLED,
    AMS_WATCHDOG_BLOCK_PANIC,
    AMS_WATCHDOG_BLOCK_STARTUP_GRACE,
    AMS_WATCHDOG_BLOCK_HEARTBEAT,
    AMS_WATCHDOG_BLOCK_ADBMS_STALE,
    AMS_WATCHDOG_BLOCK_CURRENT_STALE,
    AMS_WATCHDOG_BLOCK_TEMP_STALE,
    AMS_WATCHDOG_BLOCK_HARD_FAULT,
    AMS_WATCHDOG_BLOCK_STOP_FEED_TEST,
    AMS_WATCHDOG_BLOCK_START_FAILED,
} ams_watchdog_block_reason_t;

typedef enum
{
    AMS_FAULT_LOG_BOOT = 1,
    AMS_FAULT_LOG_RESET_CAUSE,
    AMS_FAULT_LOG_PANIC,
    AMS_FAULT_LOG_BMS_OK_ASSERTED,
    AMS_FAULT_LOG_BMS_OK_DROPPED,
    AMS_FAULT_LOG_VOLTAGE_LATCH,
    AMS_FAULT_LOG_TEMP_LATCH,
    AMS_FAULT_LOG_CURRENT_LATCH,
    AMS_FAULT_LOG_ADBMS_DIAG_FAIL,
    AMS_FAULT_LOG_CAN_BUS_OFF,
    AMS_FAULT_LOG_CAN_RECOVERED,
    AMS_FAULT_LOG_WATCHDOG_FEED_STOPPED,
    AMS_FAULT_LOG_RTOS_STACK_OVERFLOW,
    AMS_FAULT_LOG_RTOS_MALLOC_FAILED,
    AMS_FAULT_LOG_RTOS_ASSERT_FAILED,
    AMS_FAULT_LOG_RTOS_LOW_STACK_WARN,
    AMS_FAULT_LOG_RTOS_LOW_HEAP_WARN,
    AMS_FAULT_LOG_FAULT_CLEAR_ACCEPTED,
    AMS_FAULT_LOG_FAULT_CLEAR_REJECTED,
    AMS_FAULT_LOG_AIR_FAULT_LATCH,
    AMS_FAULT_LOG_STATE_TRANSITION,
    AMS_FAULT_LOG_ADBMS_FAULT_CHANGE,
    AMS_FAULT_LOG_ADBMS_STATE_TRANSITION,
    AMS_FAULT_LOG_ADBMS_DEVICE_RESET,
    AMS_FAULT_LOG_ADBMS_FAULT_INJECTION,
} ams_fault_log_event_t;

typedef struct
{
    uint32_t magic;
    uint32_t panic_reason;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t reset_count;
} ams_panic_record_t;

typedef struct
{
    uint32_t boot_sequence;
    uint32_t sequence;
    uint32_t tick;
    uint16_t event;
    uint16_t reason;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t crc32;
    uint32_t commit;
} ams_fault_log_entry_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t entry_size;
    uint32_t write_index;
    uint32_t count;
    uint32_t boot_sequence;
    uint32_t next_sequence;
    ams_fault_log_entry_t entry[AMS_FAULT_LOG_DEPTH];
} ams_fault_log_t;

void ams_safety_force_bms_low(void);
void ams_safety_panic(ams_panic_reason_t reason);
void ams_safety_unexpected_irq_panic_loop(void);
bool ams_safety_panic_active(void);
void ams_safety_enable_cpu_faults(void);
void ams_safety_record_reset_cause(void);
void ams_safety_sync_app(app_data_t *data);

uint32_t ams_safety_reset_flags(void);
const ams_panic_record_t *ams_safety_panic_record(void);
const char *ams_safety_panic_reason_str(uint32_t reason);
void ams_safety_format_reset_flags(uint32_t flags, char *buf, size_t len);

bool ams_safety_watchdog_ok(const app_data_t *data);
void ams_safety_watchdog_boot_arm(app_data_t *data);
void ams_safety_watchdog_task_update(app_data_t *data);
void ams_safety_watchdog_enable_runtime(app_data_t *data, bool enable);
bool ams_safety_watchdog_hw_started(void);
const char *ams_safety_watchdog_block_reason_str(uint32_t reason);

void ams_fault_log_event(ams_fault_log_event_t event,
                         uint16_t reason,
                         uint32_t arg0,
                         uint32_t arg1);
bool ams_fault_log_snapshot(ams_fault_log_t *out);
const ams_fault_log_t *ams_fault_log_get(void);
void ams_fault_log_clear(void);
const char *ams_fault_log_event_str(uint16_t event);

#if AMS_FAULT_INJECTION_CLI
void ams_safety_fault_inject_hardfault(void);
void ams_safety_fault_inject_busfault(void);
void ams_safety_watchdog_stop_feed_for_test(bool stop);
#endif

#if AMS_HOST_TEST
void ams_safety_host_reset_state(void);
void ams_safety_host_set_reset_csr(uint32_t csr);
void ams_safety_host_set_fault_regs(uint32_t cfsr, uint32_t hfsr, uint32_t mmfar, uint32_t bfar);
bool ams_safety_host_bms_forced_low(void);
void ams_safety_host_fault_log_invalidate_entry(uint32_t index);
void ams_safety_host_fault_log_corrupt_entry_crc(uint32_t index);
void ams_safety_host_fault_log_corrupt_metadata(uint32_t write_index,
                                                uint32_t count,
                                                uint32_t next_sequence);
#endif

#endif /* AMS_SAFETY_H_ */
