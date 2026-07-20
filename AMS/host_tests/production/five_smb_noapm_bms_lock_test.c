/* Direct-link test for the real app.c five-SMB/no-APM BMS_OK lockout. */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app.h"

extern app_data_t app;

static TaskHandle_t fake_current_task;
static GPIO_PinState fake_bms_pin = GPIO_PIN_RESET;

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
    return false;
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
    CHECK(AMS_ACCUMULATOR_5SMB_NO_APM == 1);

    app.bms_state = false;
    /* Deliberately corrupt the mutable inhibit low. The compile-time fixture
     * gate must still prevent assertion by the correct supervisor owner. */
    app.bms_output_inhibit = false;
    app.bms_output_block_count = 0u;
    app.error_task = (TaskHandle_t)(uintptr_t)0x1234u;
    fake_current_task = app.error_task;

    set_bms(true);
    CHECK(app.bms_state == false);
    CHECK(fake_bms_pin == GPIO_PIN_RESET);
    CHECK(app.bms_output_block_count == 1u);

    set_bms(false);
    CHECK(app.bms_state == false);
    CHECK(fake_bms_pin == GPIO_PIN_RESET);

    puts("PASS real app.c five-SMB/no-APM BMS_OK compile-time lockout");
    return 0;
}
