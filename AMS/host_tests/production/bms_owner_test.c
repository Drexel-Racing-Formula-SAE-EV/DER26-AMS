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

    /* The registered safety supervisor may assert it when no other gate blocks. */
    fake_current_task = app.error_task;
    set_bms(true);
    CHECK(app.bms_state == true);
    CHECK(fake_bms_pin == GPIO_PIN_SET);

    /* Any context remains allowed to force the output low immediately. */
    fake_current_task = (TaskHandle_t)(uintptr_t)0x9999u;
    set_bms(false);
    CHECK(app.bms_state == false);
    CHECK(fake_bms_pin == GPIO_PIN_RESET);

    /* Output inhibit and panic remain higher-authority gates. */
    fake_current_task = app.error_task;
    app.bms_output_inhibit = true;
    set_bms(true);
    CHECK(app.bms_state == false);
    CHECK(app.bms_output_block_count == 2u);

    app.bms_output_inhibit = false;
    fake_panic_active = true;
    set_bms(true);
    CHECK(app.bms_state == false);
    CHECK(app.bms_output_block_count == 3u);
    CHECK(fake_log_count >= 2u);

    puts("PASS BMS_OK sole-owner enforcement");
    return 0;
}
