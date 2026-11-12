/*
 * Direct host test of production app.c BMS_OK ownership enforcement.
 * app.c is linked with section garbage collection so only set_bms() and its
 * direct dependencies are retained; this tests the real implementation rather
 * than the broader SIL harness shim.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app.h"

extern app_data_t app;

static TaskHandle_t fake_current_task;
static GPIO_PinState fake_bms_pin = GPIO_PIN_RESET;
static bool fake_panic_active;
static uint32_t fake_log_count;
static uint32_t fake_critical_depth;
static uint32_t fake_critical_max_depth;
static uint32_t fake_critical_enter_count;
static uint32_t fake_critical_exit_count;

void vPortEnterCritical(void)
{
    fake_critical_depth++;
    fake_critical_enter_count++;
    if(fake_critical_depth > fake_critical_max_depth)
    {
        fake_critical_max_depth = fake_critical_depth;
    }
}

void vPortExitCritical(void)
{
    if(fake_critical_depth > 0u)
    {
        fake_critical_depth--;
    }
    fake_critical_exit_count++;
}

TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    return fake_current_task;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)port;
    (void)pin;
    fake_bms_pin = state;
}

bool ams_safety_panic_active(void)
{
    return fake_panic_active;
}

void ams_fault_log_event(ams_fault_log_event_t event,
                         uint16_t reason,
                         uint32_t arg0,
                         uint32_t arg1)
{
    (void)event;
    (void)reason;
    (void)arg0;
    (void)arg1;
    fake_log_count++;
}

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if(!(condition))                                                        \
        {                                                                       \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            return 1;                                                           \
        }                                                                       \
    } while(0)

int main(void)
{
    app_data_t heartbeat_app = {0};

    app.bms_state = false;
    app.bms_output_inhibit = false;
    app.bms_output_block_count = 0u;
    app.error_task = (TaskHandle_t)(uintptr_t)0x1234u;

    /* A measurement/communication/CLI task may not assert the output. */
    fake_current_task = (TaskHandle_t)(uintptr_t)0x5678u;
    set_bms(true);
    CHECK(app.bms_state == false);
    CHECK(fake_bms_pin == GPIO_PIN_RESET);
    CHECK(app.bms_output_block_count == 1u);

    /* The registered safety supervisor may assert only in an authority-capable
     * profile. TESTDAY is compile-locked at the final hardware writer even if
     * the mutable runtime inhibit is corrupted clear. */
    fake_current_task = app.error_task;
    app.bms_output_inhibit = false;
    app.adbms_urgent_mute_requested = false;
    set_bms(true);
#if AMS_PROFILE_BMS_RUNTIME_AUTHORITY_ALLOWED
    CHECK(app.bms_state == true);
    CHECK(fake_bms_pin == GPIO_PIN_SET);
#else
    CHECK(app.bms_state == false);
    CHECK(fake_bms_pin == GPIO_PIN_RESET);
    CHECK(app.bms_output_inhibit == true);
    CHECK(app.adbms_urgent_mute_requested == true);
#endif

    /* Any context remains allowed to force the output low immediately. */
    fake_current_task = (TaskHandle_t)(uintptr_t)0x9999u;
    set_bms(false);
    CHECK(app.bms_state == false);
    CHECK(fake_bms_pin == GPIO_PIN_RESET);

#if AMS_BUILD_PROFILE == AMS_PROFILE_CHARGE_REVIEW
    /* Force a pre-existing high shadow/pin, then prove any caller can revoke. */
    app.state = STATE_START;
    app.charger_service_power_requested = true;
    app.charger_service_power_expires_tick = 60000u;
    app.bms_state = true;
    fake_bms_pin = GPIO_PIN_SET;
    fake_current_task = (TaskHandle_t)(uintptr_t)0x9999u;
    set_bms(false);
    CHECK(!app.bms_state && fake_bms_pin == GPIO_PIN_RESET);
    CHECK(!app.charger_service_power_requested);
    CHECK(app.charger_service_power_expires_tick == 0u);
#endif

    /* Repeated low is idempotent only after physical off proof. */
    app.adbms_urgent_mute_requested = false;
    app.adbms_mute_asserted = true;
    app.adbms_balance_durable_zero_verified = true;
    app.adbms_balance_write_fault = false;
    app.adbms_balance_active = false;
    uint32_t requests_before = app.adbms_urgent_mute_request_count;
    set_bms(false);
    CHECK(!app.adbms_urgent_mute_requested);
    CHECK(app.adbms_urgent_mute_request_count == requests_before);
    app.adbms_mute_asserted = false;
    set_bms(false);
    CHECK(app.adbms_urgent_mute_requested);
    CHECK(app.adbms_urgent_mute_request_count == requests_before + 1u);
    set_bms(false);
    CHECK(app.adbms_urgent_mute_request_count == requests_before + 1u);
    app.adbms_mute_asserted = true;
    app.adbms_urgent_mute_requested = false;
    app.adbms_balance_durable_zero_verified = false;
    set_bms(false);
    CHECK(app.adbms_urgent_mute_requested);
    app.adbms_balance_durable_zero_verified = true;
    app.adbms_urgent_mute_requested = false;
    app.adbms_balance_write_fault = true;
    app.adbms_urgent_mute_edge = false;
    set_bms(false);
    CHECK(app.adbms_urgent_mute_requested);
    CHECK(!app.adbms_urgent_mute_edge);
    app.adbms_balance_write_fault = false;
    app.adbms_urgent_mute_requested = false;
    app.adbms_balance_active = true;
    uint32_t edge_generation = app.adbms_urgent_mute_generation;
    set_bms(false);
    CHECK(app.adbms_urgent_mute_requested);
    CHECK(app.adbms_urgent_mute_edge);
    CHECK(app.adbms_urgent_mute_generation == edge_generation + 1u);
    app.adbms_balance_active = false;
    app.adbms_urgent_mute_requested = false;
    /* Simulate an asserted output before a fail-low edge in every profile. */
    app.bms_state = true;
    set_bms(false);
    CHECK(app.adbms_urgent_mute_requested);
    CHECK(fake_bms_pin == GPIO_PIN_RESET);
    CHECK(fake_critical_depth == 0u);
    CHECK(fake_critical_enter_count == fake_critical_exit_count);
    fake_critical_enter_count = 0u;
    fake_critical_exit_count = 0u;
    fake_critical_max_depth = 0u;

    /* Output inhibit and panic remain higher-authority gates in profiles that
     * can otherwise assert. TESTDAY has already accumulated one additional
     * compile-lock block above, so use relative counters here. */
    fake_current_task = app.error_task;
    uint32_t block_before = app.bms_output_block_count;
    app.bms_output_inhibit = true;
    set_bms(true);
    CHECK(app.bms_state == false);
    CHECK(app.bms_output_block_count > block_before);

    block_before = app.bms_output_block_count;
    app.bms_output_inhibit = false;
    fake_panic_active = true;
    set_bms(true);
    CHECK(app.bms_state == false);
    CHECK(app.bms_output_block_count > block_before);
#if AMS_PROFILE_BMS_RUNTIME_AUTHORITY_ALLOWED
    CHECK(fake_log_count >= 1u);
#else
    /* Only the simulated asserted-to-low regression edge is logged here. */
    #if AMS_BUILD_PROFILE == AMS_PROFILE_CHARGE_REVIEW
    CHECK(fake_log_count >= 2u); /* Additional forced-high service regression. */
#else
    CHECK(fake_log_count == 1u);
#endif
#endif

    /* Exercise the production heartbeat implementation, not the broad SIL
     * harness copy.  Every shared monitor mutation must be bracketed by a
     * balanced FreeRTOS critical section. */
    CHECK(fake_critical_depth == 0u);
    CHECK(fake_critical_enter_count == fake_critical_exit_count);
    fake_critical_enter_count = 0u;
    fake_critical_exit_count = 0u;
    fake_critical_max_depth = 0u;
    ams_heartbeat_init(&heartbeat_app, 100u);
    ams_heartbeat_kick(&heartbeat_app, AMS_HEARTBEAT_ADBMS, 110u);
    ams_heartbeat_kick(&heartbeat_app, AMS_HEARTBEAT_CURRENT, 111u);
    CHECK((heartbeat_app.heartbeat.seen_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_ADBMS)) != 0u);
    CHECK((heartbeat_app.heartbeat.seen_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_CURRENT)) != 0u);
    CHECK(fake_critical_enter_count == 2u);
    CHECK(fake_critical_exit_count == 2u);
    CHECK(fake_critical_depth == 0u);
    CHECK(fake_critical_max_depth == 1u);

    (void)ams_heartbeat_update(&heartbeat_app, 112u);
    CHECK(fake_critical_enter_count == 3u);
    CHECK(fake_critical_exit_count == 3u);
    CHECK(fake_critical_depth == 0u);

    ams_heartbeat_kick(&heartbeat_app, AMS_HEARTBEAT_COUNT, 113u);
    CHECK(fake_critical_enter_count == 3u);

    ams_heartbeat_kick(&heartbeat_app, (ams_heartbeat_id_t)-1, 114u);
    CHECK(fake_critical_enter_count == 3u);

    heartbeat_app.heartbeat.count[AMS_HEARTBEAT_ADBMS] = UINT32_MAX;
    ams_heartbeat_kick(&heartbeat_app, AMS_HEARTBEAT_ADBMS, 115u);
    CHECK(heartbeat_app.heartbeat.count[AMS_HEARTBEAT_ADBMS] == UINT32_MAX);
    CHECK(fake_critical_enter_count == 4u);
    CHECK(fake_critical_exit_count == 4u);

    #if AMS_BUILD_PROFILE == AMS_PROFILE_TESTDAY
    puts("PASS TESTDAY compile-locked BMS_OK writer and heartbeat critical-section enforcement");
    #elif AMS_BUILD_PROFILE == AMS_PROFILE_CHARGE_REVIEW
    puts("PASS CHARGE_REVIEW BMS_OK writer (runtime-armed) and heartbeat critical-section enforcement");
#else
    puts("PASS BMS_OK ownership and heartbeat critical-section enforcement");
#endif
    return 0;
}
