/*
 * Author: Mahad Faisal (2026)
 * DER26 AMS host-side plug-in test runner.
 *
 * This is intentionally a host test harness, not firmware code. It compiles a
 * selected subset of the production AMS C files into a desktop executable and
 * supplies fake HAL, FreeRTOS, CAN, GPIO, ADC, UART, and ADBMS shims.
 *
 * The goal is to catch logic regressions in safety/fault handling, CAN packet
 * layout, charger handling, balancing gating, and defensive input behavior
 * before touching the STM32 board. It does not prove real SPI timing, PEC,
 * thermistor calibration, charger polarity, or hardware pin mapping.
 */
#include "host_test_config.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <setjmp.h>

#ifndef AMS_HOST_LONG_FUZZ_CYCLES
#define AMS_HOST_LONG_FUZZ_CYCLES 10000u
#endif

#ifndef AMS_HOST_CONCURRENT_FUZZ_CYCLES
#define AMS_HOST_CONCURRENT_FUZZ_CYCLES 2500u
#endif

#include "app.h"
#include "ext_drivers/adbms2950.h"
#include "ext_drivers/adbms6830_functions.h"
#include "ext_drivers/thermistor_model.h"

GPIO_TypeDef dummy_gpio;
TIM_TypeDef tim3_inst, tim4_inst, tim5_inst;
static SPI_HandleTypeDef fake_topology_spi;
static TIM_HandleTypeDef fake_topology_timer;
app_data_t app;
static uint32_t fake_tick = 0;
static GPIO_PinState bms_pin_state = GPIO_PIN_RESET;
static uint32_t tx_count = 0;
static struct { uint32_t ide, stdid, extid, dlc; uint8_t data[8]; } tx_log[AMS_HOST_TX_LOG_CAPACITY];
static uint32_t tx_free_level = 3;
static HAL_StatusTypeDef fake_can_add_tx_status = HAL_OK;
static uint32_t fake_can_add_tx_call_count = 0u;
static uint32_t fake_can_fail_on_call = 0u;
static uint32_t fake_can_advance_tick_per_tx_ms = 0u;
static uint32_t fake_can_mutate_after_tx_count = 0u;
static uint32_t fake_can_error = HAL_CAN_ERROR_NONE;
static HAL_StatusTypeDef fake_can_recover_status = HAL_OK;
static HAL_StatusTypeDef fake_can_notification_status = HAL_OK;
static HAL_StatusTypeDef fake_can_filter_status = HAL_OK;
static CAN_FilterTypeDef fake_can_filter_log[3];
static uint32_t fake_can_filter_count = 0u;
static char cli_capture[8192];
static size_t cli_capture_len = 0u;
static UART_HandleTypeDef cli_dummy_uart;
static CAN_RxHeaderTypeDef fake_rx_hdr;
static uint8_t fake_rx_data[8];
static HAL_StatusTypeDef fake_rx_status = HAL_OK;
static CAN_HandleTypeDef hil_fake_hcan;
static jmp_buf task_exit_jmp;
static int task_exit_after_delay_until = 0;
static int fake_mux_write_enable = 1;
static uint16_t fake_adc_read_counts[2] = {2048u, 2048u};
static HAL_StatusTypeDef fake_adc_read_statuses[2] = {HAL_OK, HAL_OK};
static uint32_t fake_adc_read_index = 0u;
static bool fake_adbms_use_custom_voltage_masks = false;
static uint16_t fake_adbms_updated_masks[ADBMS6830_MAX_TRACKED_ICS];
static uint16_t fake_adbms_pec_masks[ADBMS6830_MAX_TRACKED_ICS];
static HAL_StatusTypeDef fake_adbms_diag_status = HAL_OK;
static HAL_StatusTypeDef fake_adbms_start_conversion_status = HAL_OK;
static HAL_StatusTypeDef fake_adbms_read_cell_status = HAL_OK;
static HAL_StatusTypeDef fake_apm_init_status = HAL_OK;
static HAL_StatusTypeDef fake_apm_sample_status = HAL_OK;
static HAL_StatusTypeDef fake_apm_probe_status = HAL_OK;
static uint32_t fake_apm_init_call_count = 0u;
static uint32_t fake_apm_sample_call_count = 0u;
static uint32_t fake_apm_probe_call_count = 0u;
static uint32_t fake_adbms_wrcfgb_call_count = 0u;
static uint32_t fake_adbms_wrpwm_call_count = 0u;
static uint32_t fake_adbms_open_wire_call_count = 0u;
static int32_t fake_apm_i1_raw = 1234;
static int16_t fake_apm_vb1_raw = 18000;
static adbms_string fake_apm_init_string = STRING_A;
static bool fake_apm_init_requested_reset = true;
static bool fake_apm_init_enabled_dividers = true;
static uint32_t fake_external_counter_note_calls = 0u;
static uint32_t fake_external_counter_increment_total = 0u;
static uint32_t fake_counter_resync_calls = 0u;
static uint16_t fake_adbms_config_mismatch_mask = 0u;
static uint16_t fake_adbms_delay_values_us[8];
static uint32_t fake_adbms_delay_calls = 0u;
static bool fake_adbms_delay_advances_tick = false;
static uint32_t fake_adbms_lock_depth = 0u;
static uint32_t fake_adbms_lock_max_depth = 0u;
static HAL_StatusTypeDef fake_tim_base_start_status = HAL_OK;
static HAL_StatusTypeDef fake_tim_pwm_start_status = HAL_OK;
static HAL_StatusTypeDef fake_tim_ic_start_it_status = HAL_OK;
static HAL_StatusTypeDef fake_tim_ic_start_status = HAL_OK;
static uint32_t fake_tim_total_capture = 1000u;
static uint32_t fake_tim_high_capture = 500u;

static uint16_t adc_count_for_mcu_voltage(float v);
static uint16_t adc_count_for_sensor_voltage(float v);
static void fake_adc_set_two_read_sequence(uint16_t high_count, uint16_t low_count);
static void fake_adc_set_status_sequence(HAL_StatusTypeDef high_status, HAL_StatusTypeDef low_status);
static void fake_adbms_voltage_masks_full_update(void);
static void fake_adbms_voltage_masks_all_missing(bool pec_fail);
static void fake_adbms_voltage_masks_one_missing(uint8_t seg, uint8_t cell, bool pec_fail);
static void fake_adc_set_current_a(float current_a);
static void run_one_canbus_task_iteration(app_data_t *d);
static void run_one_adbms_task_iteration(app_data_t *d);
static void fill_nominal_pack(app_data_t *d, float base_v);
static void sil_publish_temp_state(app_data_t *d);
static void sil_expect_balancing_clear(const app_data_t *d);
static uint8_t sil_balance_pwm_duty(const app_data_t *d, uint8_t ic, uint8_t cell);
static void sil_prepare_cli_capture(void);
static void sil_run_can_charge_iteration(app_data_t *d, CAN_HandleTypeDef *hcan);

uint32_t osKernelGetTickCount(void){ return fake_tick; }
osStatus_t osDelay(uint32_t ticks){ fake_tick += ticks; return osOK; }
osStatus_t osDelayUntil(uint32_t ticks){ if((int32_t)(ticks - fake_tick) > 0){ fake_tick = ticks; } if(task_exit_after_delay_until){ task_exit_after_delay_until = 0; longjmp(task_exit_jmp, 1); } return osOK; }
BaseType_t xTaskCreate(TaskFunction_t fn, const char * const name, const configSTACK_DEPTH_TYPE stack, void * const arg, UBaseType_t prio, TaskHandle_t * const handle){ (void)fn;(void)name;(void)stack;(void)arg;(void)prio; if(handle) *handle=(TaskHandle_t)0x1; return pdPASS; }
TaskHandle_t xTaskCreateStatic(TaskFunction_t fn,
                               const char * const name,
                               const uint32_t stack_depth,
                               void * const arg,
                               UBaseType_t prio,
                               StackType_t * const stack_buffer,
                               StaticTask_t * const task_buffer)
{
    (void)name;
    (void)arg;
    (void)prio;
    if((fn == NULL) || (stack_depth == 0u) ||
       (stack_buffer == NULL) || (task_buffer == NULL))
    {
        return NULL;
    }
    return (TaskHandle_t)task_buffer;
}
void vTaskDelete(TaskHandle_t handle){ (void)handle; }

void vPortEnterCritical(void){}
void vPortExitCritical(void){}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan){ return hcan ? fake_can_recover_status : HAL_ERROR; }
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan){ return hcan ? fake_can_recover_status : HAL_ERROR; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan){ if(!hcan) return HAL_ERROR; if(fake_can_recover_status == HAL_OK) fake_can_error = HAL_CAN_ERROR_NONE; return fake_can_recover_status; }
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *hcan){ (void)hcan; return fake_can_error; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t notif){ (void)notif; return hcan ? fake_can_notification_status : HAL_ERROR; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan,
                                       const CAN_FilterTypeDef *filter)
{
    if((hcan == NULL) || (filter == NULL)) return HAL_ERROR;
    if(fake_can_filter_status != HAL_OK) return fake_can_filter_status;
    if(fake_can_filter_count < 3u) fake_can_filter_log[fake_can_filter_count] = *filter;
    fake_can_filter_count++;
    return HAL_OK;
}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *hcan){ (void)hcan; return tx_free_level; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *hdr, const uint8_t *data, uint32_t *mailbox){
    if(!hcan || !hdr || !data || tx_count >= AMS_HOST_TX_LOG_CAPACITY) return HAL_ERROR;
    fake_can_add_tx_call_count++;
    if(fake_can_add_tx_status != HAL_OK) return fake_can_add_tx_status;
    if((fake_can_fail_on_call != 0u) &&
       (fake_can_add_tx_call_count == fake_can_fail_on_call)) return HAL_ERROR;
    tx_log[tx_count].ide = hdr->IDE; tx_log[tx_count].stdid = hdr->StdId; tx_log[tx_count].extid = hdr->ExtId; tx_log[tx_count].dlc = hdr->DLC;
    memcpy(tx_log[tx_count].data, data, 8); if(mailbox) *mailbox=0; tx_count++;
    if((fake_can_mutate_after_tx_count != 0u) &&
       (tx_count == fake_can_mutate_after_tx_count))
    {
        for(uint8_t fan = 0u; fan < NFANS; fan++) app.board.fans[fan].duty_cycle = 100.0f;
        app.temp_warning = true;
        app.temp_fan_max = true;
        app.temp_charge_stop = true;
        app.temp_overtemp_pending = true;
        app.overtemp_fault = true;
        app.severe_overtemp_fault = true;
        app.fan_fault = true;
        app.temp_read_fault = true;
    }
    fake_tick += fake_can_advance_tick_per_tx_ms;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo, CAN_RxHeaderTypeDef *hdr, uint8_t data[]){
    (void)fifo; if(!hcan || !hdr || !data) return HAL_ERROR; *hdr = fake_rx_hdr; memcpy(data, fake_rx_data, 8); return fake_rx_status;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel){ (void)channel; return htim ? fake_tim_pwm_start_status : HAL_ERROR; }
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim){ return htim ? fake_tim_base_start_status : HAL_ERROR; }
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t channel){ (void)channel; return htim ? fake_tim_ic_start_it_status : HAL_ERROR; }
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t channel){ (void)channel; return htim ? fake_tim_ic_start_status : HAL_ERROR; }
uint32_t HAL_TIM_ReadCapturedValue(const TIM_HandleTypeDef *htim, uint32_t channel){ (void)htim; return channel == TIM_CHANNEL_1 ? fake_tim_total_capture : fake_tim_high_capture; }
uint32_t HAL_GetTick(void){ return fake_tick; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin){ (void)port; (void)pin; return GPIO_PIN_SET; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state){ (void)port; (void)pin; bms_pin_state = state; }
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin){ (void)port; (void)pin; bms_pin_state = !bms_pin_state; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init){ (void)port; (void)init; }
static void cli_capture_append(const uint8_t *pData, uint16_t Size)
{
    if((pData == NULL) || (Size == 0u))
    {
        return;
    }

    size_t remaining = (sizeof(cli_capture) - 1u) - cli_capture_len;
    size_t n = (Size < remaining) ? (size_t)Size : remaining;
    if(n > 0u)
    {
        memcpy(&cli_capture[cli_capture_len], pData, n);
        cli_capture_len += n;
        cli_capture[cli_capture_len] = '\0';
    }
}

static void cli_capture_clear(void)
{
    cli_capture_len = 0u;
    cli_capture[0] = '\0';
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout){ (void)huart;(void)Timeout; cli_capture_append(pData, Size); return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size){ (void)huart; cli_capture_append(pData, Size); return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size){ (void)huart;(void)pData;(void)Size; return HAL_OK; }
uint32_t HAL_UART_GetError(const UART_HandleTypeDef *huart){ return huart ? huart->ErrorCode : HAL_UART_ERROR_NONE; }
void adbms_spi_lock(void)
{
    fake_adbms_lock_depth++;
    if(fake_adbms_lock_depth > fake_adbms_lock_max_depth)
    {
        fake_adbms_lock_max_depth = fake_adbms_lock_depth;
    }
}

void adbms_spi_unlock(void)
{
    assert(fake_adbms_lock_depth > 0u);
    fake_adbms_lock_depth--;
}

void ams_current_window_lock(void)
{
}

void ams_current_window_unlock(void)
{
}

void set_bms(bool state){
    if(state && (app.bms_output_inhibit || ams_safety_panic_active())){
        app.bms_output_block_count++;
        app.bms_state = false;
        HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, GPIO_PIN_RESET);
        return;
    }
    app.bms_state = state;
    HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint32_t ams_heartbeat_timeout_ms(ams_heartbeat_id_t id)
{
    switch(id)
    {
        case AMS_HEARTBEAT_ADBMS: return AMS_HEARTBEAT_ADBMS_TIMEOUT_MS;
        case AMS_HEARTBEAT_CURRENT: return AMS_HEARTBEAT_CURRENT_TIMEOUT_MS;
        case AMS_HEARTBEAT_TEMP: return AMS_HEARTBEAT_TEMP_TIMEOUT_MS;
        case AMS_HEARTBEAT_CAN: return AMS_HEARTBEAT_CAN_TIMEOUT_MS;
        case AMS_HEARTBEAT_LOGGER: return AMS_HEARTBEAT_LOGGER_TIMEOUT_MS;
        case AMS_HEARTBEAT_IMD: return AMS_HEARTBEAT_IMD_TIMEOUT_MS;
        case AMS_HEARTBEAT_FAN: return AMS_HEARTBEAT_FAN_TIMEOUT_MS;
        default: return 0u;
    }
}

const char *ams_heartbeat_name(ams_heartbeat_id_t id)
{
    switch(id)
    {
        case AMS_HEARTBEAT_ADBMS: return "adbms";
        case AMS_HEARTBEAT_CURRENT: return "current";
        case AMS_HEARTBEAT_TEMP: return "temp";
        case AMS_HEARTBEAT_CAN: return "can";
        case AMS_HEARTBEAT_LOGGER: return "logger";
        case AMS_HEARTBEAT_IMD: return "imd";
        case AMS_HEARTBEAT_FAN: return "fan";
        default: return "unknown";
    }
}

void ams_heartbeat_init(app_data_t *d, uint32_t now)
{
    if(d == NULL) return;
    memset(&d->heartbeat, 0, sizeof(d->heartbeat));
    d->heartbeat.boot_tick = now;
    for(uint8_t i = 0u; i < (uint8_t)AMS_HEARTBEAT_COUNT; i++)
    {
        d->heartbeat.last_tick[i] = now;
    }
    d->task_heartbeat_fault = false;
    d->logger_heartbeat_fault = false;
    d->heartbeat_stale_mask = 0u;
    d->heartbeat_seen_mask = 0u;
}

void ams_heartbeat_kick(app_data_t *d, ams_heartbeat_id_t id, uint32_t now)
{
    if((d == NULL) || ((int)id < 0) || (id >= AMS_HEARTBEAT_COUNT)) return;
    d->heartbeat.last_tick[id] = now;
    if(d->heartbeat.count[id] != UINT32_MAX) d->heartbeat.count[id]++;
    d->heartbeat.seen_mask |= AMS_HEARTBEAT_BIT(id);
    d->heartbeat_seen_mask = d->heartbeat.seen_mask;
}

uint16_t ams_heartbeat_update(app_data_t *d, uint32_t now)
{
    uint16_t stale = 0u;
    if(d == NULL) return 0u;
    bool grace = (now - d->heartbeat.boot_tick) < AMS_HEARTBEAT_STARTUP_GRACE_MS;
    for(uint8_t i = 0u; i < (uint8_t)AMS_HEARTBEAT_COUNT; i++)
    {
        uint16_t bit = AMS_HEARTBEAT_BIT(i);
        uint32_t timeout_ms = ams_heartbeat_timeout_ms((ams_heartbeat_id_t)i);
        if((d->heartbeat.seen_mask & bit) == 0u)
        {
            if(!grace) stale |= bit;
        }
        else if((now - d->heartbeat.last_tick[i]) > timeout_ms)
        {
            stale |= bit;
        }
    }
    d->heartbeat.stale_mask = stale;
    d->heartbeat.safety_stale_mask = (uint16_t)(stale & AMS_HEARTBEAT_SAFETY_MASK);
    d->heartbeat.logger_stale_mask = (uint16_t)(stale & AMS_HEARTBEAT_LOGGER_MASK);
    d->heartbeat_stale_mask = d->heartbeat.stale_mask;
    d->heartbeat_seen_mask = d->heartbeat.seen_mask;
    d->task_heartbeat_fault = (d->heartbeat.safety_stale_mask != 0u);
    d->logger_heartbeat_fault = (d->heartbeat.logger_stale_mask != 0u);
    return stale;
}

// External driver stubs used by accumulator/CLI paths
static HAL_StatusTypeDef fake_adbms_init_status = HAL_OK;
static HAL_StatusTypeDef fake_adbms_wrcfgb_status = HAL_OK;
static HAL_StatusTypeDef fake_adbms_wrpwm_status = HAL_OK;
static HAL_StatusTypeDef fake_adbms_balance_verify_status = HAL_OK;
static int fake_adbms_wrpwm_fail_after_ok = -1;
HAL_StatusTypeDef adBms6830_init(adbms6830_driver_t* dev, uint8_t num_ics, uint8_t physical_chain_count, adbms6830_asic* ics, uint8_t ics_capacity, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port_a, GPIO_TypeDef* cs_port_b, uint16_t cs_pin_a, uint16_t cs_pin_b, TIM_HandleTypeDef *htim){ if((dev == NULL) || (num_ics == 0u) || (num_ics > ics_capacity) || (num_ics > ADBMS6830_MAX_TRACKED_ICS) || (physical_chain_count < num_ics) || (physical_chain_count > ADBMS6830_MAX_PHYSICAL_DEVICES) || (ics == NULL)){ return HAL_ERROR; } dev->num_ics=num_ics; dev->physical_chain_count=physical_chain_count; dev->ics_capacity=ics_capacity; dev->ics=ics; dev->hspi=hspi; dev->cs_port[0]=cs_port_a; dev->cs_port[1]=cs_port_b; dev->cs_pin[0]=cs_pin_a; dev->cs_pin[1]=cs_pin_b; dev->htim=htim; dev->string=STRING_A; dev->write_string=STRING_A; dev->monitored_cell_count=0u; memset(&dev->spi_debug, 0, sizeof(dev->spi_debug)); memset(&dev->health, 0, sizeof(dev->health)); memset(&dev->diag, 0, sizeof(dev->diag)); dev->spi_debug.last_status=HAL_OK; dev->spi_debug.last_tx_status=HAL_OK; dev->spi_debug.last_rx_status=HAL_OK; dev->spi_debug.last_xfer_status=HAL_OK; dev->health.last_status=HAL_OK; dev->health.sid_valid_ic_mask=(uint16_t)((1u << num_ics) - 1u); for(uint8_t ic=0; ic<ADBMS6830_MAX_TRACKED_ICS; ic++){ dev->last_cell_updated_mask[ic]=0u; dev->last_cell_pec_mask[ic]=0u; dev->last_temp_updated_mask[ic]=0u; if(ic<num_ics){ dev->diag[ic].sid_valid=true; dev->diag[ic].device_id=ADBMS6830B_DEVICE_ID; dev->diag[ic].sid[1]=(uint8_t)(ADBMS6830B_DEVICE_ID<<1u); } } return fake_adbms_init_status; }
bool adbms6830_set_monitored_cell_count(adbms6830_driver_t *dev, uint8_t cell_count)
{
    if((dev == NULL) || (dev->ics == NULL) || (dev->num_ics == 0u) ||
       (cell_count == 0u) || (cell_count > CELL))
    {
        return false;
    }
    dev->monitored_cell_count = cell_count;
    return true;
}
static HAL_StatusTypeDef fake_adbms_wrpwm_next_status(void)
{
    if(fake_adbms_wrpwm_fail_after_ok == 0)
    {
        return HAL_ERROR;
    }
    if(fake_adbms_wrpwm_fail_after_ok > 0)
    {
        fake_adbms_wrpwm_fail_after_ok--;
    }
    return fake_adbms_wrpwm_status;
}
void adbms6830_reset_cfg(adbms6830_driver_t *dev){(void)dev;} void adbms6830_srst(adbms6830_driver_t *dev){(void)dev;} void adbms6830_wrcfga(adbms6830_driver_t *dev){(void)dev;} void adbms6830_wrcfgb(adbms6830_driver_t *dev){(void)adbms6830_wrcfgb_checked(dev);} HAL_StatusTypeDef adbms6830_wrcfgb_checked(adbms6830_driver_t *dev){(void)dev; fake_adbms_wrcfgb_call_count++; return fake_adbms_wrcfgb_status;} HAL_StatusTypeDef adbms6830_wrpwma_checked(adbms6830_driver_t *dev){(void)dev; fake_adbms_wrpwm_call_count++; return fake_adbms_wrpwm_next_status();} HAL_StatusTypeDef adbms6830_wrpwmb_checked(adbms6830_driver_t *dev){(void)dev; fake_adbms_wrpwm_call_count++; return fake_adbms_wrpwm_next_status();} HAL_StatusTypeDef adbms6830_write_pwm_checked(adbms6830_driver_t *dev){(void)dev; fake_adbms_wrpwm_call_count++; return fake_adbms_wrpwm_next_status();} void adbms6830_rdcfga(adbms6830_driver_t *dev){(void)dev;} void adbms6830_rdcfgb(adbms6830_driver_t *dev){(void)dev;}
HAL_StatusTypeDef adbms6830_verify_balance_readback(adbms6830_driver_t *dev){(void)dev; return fake_adbms_balance_verify_status;}
void adbms6830_adcv(adbms6830_driver_t *dev, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs){(void)dev;(void)rd;(void)cont;(void)dcp;(void)rstf;(void)owcs;}
void adbms6830_wakeup(adbms6830_driver_t* dev){(void)dev;}
HAL_StatusTypeDef adbms6830_wakeup_checked(adbms6830_driver_t* dev){return (dev != NULL) ? HAL_OK : HAL_ERROR;}
HAL_StatusTypeDef adbms6830_us_delay(adbms6830_driver_t* dev, uint16_t microseconds)
{
    if(dev == NULL)
    {
        return HAL_ERROR;
    }
    if(fake_adbms_delay_calls < (sizeof(fake_adbms_delay_values_us) /
                                  sizeof(fake_adbms_delay_values_us[0])))
    {
        fake_adbms_delay_values_us[fake_adbms_delay_calls] = microseconds;
    }
    fake_adbms_delay_calls++;
    if(fake_adbms_delay_advances_tick)
    {
        fake_tick += ((uint32_t)microseconds + 999u) / 1000u;
    }
    return HAL_OK;
}
HAL_StatusTypeDef adbms6830_start_adc_cell_voltage_measurement(adbms6830_driver_t *dev){return (dev != NULL) ? fake_adbms_start_conversion_status : HAL_ERROR;}
void adbms6830_parse_cell(adbms6830_driver_t *dev, uint8_t *data, GRP grp){(void)dev;(void)data;(void)grp;}
void adbms6830_wakeup_cold(adbms6830_driver_t* dev){ if(dev){ dev->spi_debug.last_op = ADBMS6830_SPI_OP_COLD_WAKE; } }
HAL_StatusTypeDef adbms6830_read_cell_voltages(adbms6830_driver_t *dev){
    if(dev){
        for(uint8_t ic=0; ic<ADBMS6830_MAX_TRACKED_ICS; ic++){
            dev->last_cell_updated_mask[ic]=0u;
            dev->last_cell_pec_mask[ic]=0u;
        }
        if(fake_adbms_read_cell_status != HAL_OK){
            return fake_adbms_read_cell_status;
        }
        for(uint8_t ic=0; (dev->ics != NULL) && (ic < (uint8_t)dev->num_ics) && (ic < ADBMS6830_MAX_TRACKED_ICS); ic++){
            if(fake_adbms_use_custom_voltage_masks){
                dev->last_cell_updated_mask[ic]=(uint16_t)(fake_adbms_updated_masks[ic] & 0x7FFFu);
                dev->last_cell_pec_mask[ic]=(uint16_t)(fake_adbms_pec_masks[ic] & 0x7FFFu);
            }
            else{
                dev->last_cell_updated_mask[ic]=0x7FFFu;
                dev->last_cell_pec_mask[ic]=0u;
            }
        }
        return HAL_OK;
    }
    return HAL_ERROR;
}
void adbms6830_spi_debug_enable(adbms6830_driver_t *dev, bool enable){ if(dev) dev->spi_debug.enabled = enable; }
void adbms6830_spi_debug_clear(adbms6830_driver_t *dev){ if(dev){ bool en = dev->spi_debug.enabled; memset(&dev->spi_debug, 0, sizeof(dev->spi_debug)); dev->spi_debug.enabled = en; } }
const adbms6830_spi_debug_t *adbms6830_spi_debug_get(const adbms6830_driver_t *dev){ return dev ? &dev->spi_debug : NULL; }
const char *adbms6830_spi_op_str(adbms6830_spi_op_t op){
    switch(op){
        case ADBMS6830_SPI_OP_NONE: return "none";
        case ADBMS6830_SPI_OP_CMD: return "cmd";
        case ADBMS6830_SPI_OP_WR48: return "wr48";
        case ADBMS6830_SPI_OP_RD48: return "rd48";
        case ADBMS6830_SPI_OP_STCOMM: return "stcomm";
        case ADBMS6830_SPI_OP_PROBE: return "probe";
        case ADBMS6830_SPI_OP_WAKE: return "wake";
        case ADBMS6830_SPI_OP_COLD_WAKE: return "cold_wake";
        case ADBMS6830_SPI_OP_READ_SID: return "read_sid";
        case ADBMS6830_SPI_OP_READ_STATUS: return "read_status";
        case ADBMS6830_SPI_OP_DIAGNOSTIC_REFRESH: return "diagnostic_refresh";
        case ADBMS6830_SPI_OP_STARTUP_BASELINE: return "startup_baseline";
        case ADBMS6830_SPI_OP_CLEAR_FLAGS: return "clear_flags";
        case ADBMS6830_SPI_OP_CONFIG_CHECK: return "config_check";
        case ADBMS6830_SPI_OP_BALANCE_CHECK: return "balance_check";
        case ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST: return "cell_adc_diag";
        case ADBMS6830_SPI_OP_OPEN_WIRE_BASELINE: return "open_wire_baseline";
        case ADBMS6830_SPI_OP_OPEN_WIRE_EVEN: return "open_wire_even";
        case ADBMS6830_SPI_OP_OPEN_WIRE_ODD: return "open_wire_odd";
        case ADBMS6830_SPI_OP_OPEN_WIRE_FULL: return "open_wire_full";
        case ADBMS6830_SPI_OP_AUX_GPIO_DIAG: return "aux_gpio_diag";
        case ADBMS6830_SPI_OP_SCOPE: return "scope";
        default: return "unknown";
    }
}
HAL_StatusTypeDef adbms6830_spi_probe_rdcfga(adbms6830_driver_t *dev){
    if(dev == NULL) return HAL_ERROR;
    dev->spi_debug.last_op = ADBMS6830_SPI_OP_PROBE;
    dev->spi_debug.last_status = HAL_OK;
    dev->spi_debug.rx_count++;
    return HAL_OK;
}
HAL_StatusTypeDef adbms6830_spi_probe_rdcfga_on_string(adbms6830_driver_t *dev, adbms_string string){
    if((dev == NULL) || (string > STRING_B)) return HAL_ERROR;
    adbms_string previous = dev->string;
    dev->string = string;
    HAL_StatusTypeDef status = adbms6830_spi_probe_rdcfga(dev);
    dev->spi_debug.last_string = string;
    dev->string = previous;
    return status;
}
HAL_StatusTypeDef adbms6830_scope_activity(adbms6830_driver_t *dev, adbms_string string, adbms6830_scope_mode_t mode, uint16_t repeat_count){
    if((dev == NULL) || (string > STRING_B) || (repeat_count == 0u)) return HAL_ERROR;
    adbms_string previous = dev->string;
    uint16_t repeat = (repeat_count > 100u) ? 100u : repeat_count;
    dev->string = string;
    dev->spi_debug.enabled = true;
    dev->spi_debug.last_op = ADBMS6830_SPI_OP_SCOPE;
    dev->spi_debug.last_string = string;
    dev->spi_debug.last_status = HAL_OK;
    dev->spi_debug.last_cmd[0] = 0x00u;
    dev->spi_debug.last_cmd[1] = 0x02u;
    if(mode == ADBMS6830_SCOPE_READ){
        dev->spi_debug.rx_count += repeat;
    }
    else{
        dev->spi_debug.tx_count += repeat;
    }
    dev->string = previous;
    return HAL_OK;
}
HAL_StatusTypeDef adbms6830_read_sid(adbms6830_driver_t *dev){
    if(dev == NULL) return HAL_ERROR;
    dev->health.sid_valid_ic_mask = 0u;
    dev->health.sid_identity_mismatch_ic_mask = 0u;
    for(uint8_t ic = 0u; (dev->ics != NULL) && (ic < (uint8_t)dev->num_ics) && (ic < ADBMS6830_MAX_TRACKED_ICS); ic++){
        for(uint8_t b = 0u; b < RSID; b++){
            dev->diag[ic].sid[b] = (uint8_t)(0x10u + (ic * 0x10u) + b);
            dev->ics[ic].sid.sid[b] = dev->diag[ic].sid[b];
        }
        dev->diag[ic].sid[1] = (uint8_t)(ADBMS6830B_DEVICE_ID << 1u);
        dev->ics[ic].sid.sid[1] = dev->diag[ic].sid[1];
        dev->diag[ic].device_id = ADBMS6830B_DEVICE_ID;
        dev->diag[ic].sid_valid = true;
        dev->health.sid_valid_ic_mask |= (uint16_t)(1u << ic);
    }
    dev->spi_debug.last_op = ADBMS6830_SPI_OP_READ_SID;
    dev->spi_debug.last_status = HAL_OK;
    dev->spi_debug.rx_count++;
    return HAL_OK;
}
HAL_StatusTypeDef adbms6830_read_status(adbms6830_driver_t *dev, bool inject_spiflt){
    if(dev == NULL) return HAL_ERROR;
    for(uint8_t ic = 0u; (ic < (uint8_t)dev->num_ics) && (ic < ADBMS6830_MAX_TRACKED_ICS); ic++){
        dev->diag[ic].statc_valid = true;
        dev->diag[ic].statd_valid = true;
        dev->diag[ic].state_valid = true;
        dev->diag[ic].spiflt = inject_spiflt ? 1u : 0u;
        dev->diag[ic].sleep = 0u;
        dev->diag[ic].thsd = 0u;
        dev->diag[ic].oscchk = 0u;
        dev->diag[ic].revision = 1u;
    }
    dev->spi_debug.last_op = ADBMS6830_SPI_OP_READ_STATUS;
    dev->spi_debug.last_status = HAL_OK;
    dev->spi_debug.rx_count += 3u;
    return HAL_OK;
}
HAL_StatusTypeDef adbms6830_refresh_diagnostics(adbms6830_driver_t *dev){
    if(dev == NULL) return HAL_ERROR;
    for(uint8_t ic = 0u; (ic < (uint8_t)dev->num_ics) && (ic < ADBMS6830_MAX_TRACKED_ICS); ic++){
        dev->diag[ic].stata_valid = true;
        dev->diag[ic].statb_valid = true;
        dev->diag[ic].reference_values_valid = true;
        dev->diag[ic].vref2_mv = 3000;
        dev->diag[ic].vd_mv = 3300;
        dev->diag[ic].va_mv = 5000;
        dev->diag[ic].vres_mv = 3000;
        dev->diag[ic].die_temp_deci_c = 250;
    }
    (void)adbms6830_read_status(dev, false);
    dev->health.diagnostic_refresh_count++;
    dev->health.status_invalid_ic_mask = 0u;
    dev->health.status_fault_ic_mask = 0u;
    dev->health.reference_invalid_ic_mask = 0u;
    dev->health.reference_fault_ic_mask = 0u;
    dev->health.last_op = ADBMS6830_SPI_OP_DIAGNOSTIC_REFRESH;
    dev->health.last_status = fake_adbms_diag_status;
    return fake_adbms_diag_status;
}
HAL_StatusTypeDef adbms6830_establish_diagnostic_baseline(adbms6830_driver_t *dev){
    HAL_StatusTypeDef status;
    if(dev == NULL) return HAL_ERROR;
    dev->health.startup_baseline_count++;
    status = adbms6830_refresh_diagnostics(dev);
    dev->health.startup_baseline_passed = (status == HAL_OK);
    dev->health.last_op = ADBMS6830_SPI_OP_STARTUP_BASELINE;
    dev->health.last_status = status;
    return status;
}
bool adbms6830_safety_diagnostics_ok(const adbms6830_driver_t *dev){
    return (dev != NULL) && dev->health.startup_baseline_passed &&
           (dev->health.status_invalid_ic_mask == 0u) &&
           (dev->health.status_fault_ic_mask == 0u) &&
           (dev->health.reference_invalid_ic_mask == 0u) &&
           (dev->health.reference_fault_ic_mask == 0u);
}
void adbms6830_note_external_counter_increments(adbms6830_driver_t *dev,
                                                 uint8_t increment_count){
    if(dev == NULL) return;
    fake_external_counter_note_calls++;
    fake_external_counter_increment_total += increment_count;
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics && ic < ADBMS6830_MAX_TRACKED_ICS; ic++){
        uint16_t bit = (uint16_t)(1u << ic);
        if((dev->spi_debug.cmd_counter_expected_mask & bit) == 0u) continue;
        for(uint8_t n = 0u; n < increment_count; n++){
            uint8_t current = (uint8_t)(dev->spi_debug.expected_cmd_counter[ic] & 0x3Fu);
            dev->spi_debug.expected_cmd_counter[ic] =
                ((current == 0u) || (current >= 63u)) ? 1u : (uint8_t)(current + 1u);
        }
    }
}
void adbms6830_resync_command_counter_tracking(adbms6830_driver_t *dev){
    if(dev == NULL) return;
    fake_counter_resync_calls++;
    dev->spi_debug.cmd_counter_expected_mask = 0u;
    dev->spi_debug.cmd_counter_mismatch_mask = 0u;
    dev->health.last_cmd_counter_mismatch_mask = 0u;
    memset(dev->spi_debug.expected_cmd_counter, 0,
           sizeof(dev->spi_debug.expected_cmd_counter));
}
HAL_StatusTypeDef adbms6830_clear_all_flags(adbms6830_driver_t *dev){
    if(dev == NULL) return HAL_ERROR;
    dev->spi_debug.last_op = ADBMS6830_SPI_OP_CLEAR_FLAGS;
    dev->spi_debug.last_status = HAL_OK;
    dev->spi_debug.tx_count++;
    return HAL_OK;
}
const adbms6830_diag_health_t *adbms6830_diag_health_get(const adbms6830_driver_t *dev){ return dev ? &dev->health : NULL; }
void adbms6830_diag_health_clear(adbms6830_driver_t *dev){ if(dev){ bool startup_ok = dev->health.startup_baseline_passed; uint16_t sid_valid = dev->health.sid_valid_ic_mask; uint16_t sid_mismatch = dev->health.sid_identity_mismatch_ic_mask; memset(&dev->health, 0, sizeof(dev->health)); dev->health.last_status = HAL_OK; dev->health.startup_baseline_passed = startup_ok; dev->health.sid_valid_ic_mask = sid_valid; dev->health.sid_identity_mismatch_ic_mask = sid_mismatch; } }
HAL_StatusTypeDef adbms6830_verify_config_readback(adbms6830_driver_t *dev){
    if(dev == NULL) return HAL_ERROR;
    dev->health.config_readback_count++;
    dev->health.last_op = ADBMS6830_SPI_OP_CONFIG_CHECK;
    dev->health.last_status = fake_adbms_diag_status;
    dev->spi_debug.last_op = ADBMS6830_SPI_OP_CONFIG_CHECK;
    dev->spi_debug.last_status = fake_adbms_diag_status;
    dev->health.configa_mismatch_mask = 0u;
    dev->health.configb_mismatch_mask = fake_adbms_config_mismatch_mask;
    dev->health.config_mismatch_mask = fake_adbms_config_mismatch_mask;
    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++){
        if((fake_adbms_config_mismatch_mask & (uint16_t)(1u << ic)) != 0u){
            dev->health.config_mismatch_count[ic]++;
        }
    }
    return fake_adbms_diag_status;
}
HAL_StatusTypeDef adbms6830_run_cell_adc_self_test(adbms6830_driver_t *dev){
    if(dev == NULL) return HAL_ERROR;
    dev->health.cell_adc_self_test_count++;
    dev->health.last_op = ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST;
    dev->health.last_status = fake_adbms_diag_status;
    dev->spi_debug.last_op = ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST;
    dev->spi_debug.last_status = fake_adbms_diag_status;
    return fake_adbms_diag_status;
}

static HAL_StatusTypeDef fake_adbms6830_open_wire_baseline(adbms6830_driver_t *dev)
{
    if(dev == NULL) return HAL_ERROR;
    dev->health.open_wire_baseline_count++;
    dev->health.open_wire_baseline_valid_ic_mask = 0u;
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics && ic < ADBMS6830_MAX_TRACKED_ICS; ic++){
        uint16_t bit = (uint16_t)(1u << ic);
        dev->diag[ic].open_wire_baseline_valid = (fake_adbms_diag_status == HAL_OK);
        dev->diag[ic].open_wire_even_valid = false;
        dev->diag[ic].open_wire_odd_valid = false;
        dev->diag[ic].open_wire_even_fault_mask = 0u;
        dev->diag[ic].open_wire_odd_fault_mask = 0u;
        dev->diag[ic].open_wire_even_attenuation_fault_mask = 0u;
        dev->diag[ic].open_wire_odd_attenuation_fault_mask = 0u;
        dev->diag[ic].open_wire_fault_mask = 0u;
        if(dev->diag[ic].open_wire_baseline_valid){
            dev->health.open_wire_baseline_valid_ic_mask |= bit;
        }
    }
    dev->health.last_op = ADBMS6830_SPI_OP_OPEN_WIRE_BASELINE;
    dev->health.last_status = fake_adbms_diag_status;
    return fake_adbms_diag_status;
}

static HAL_StatusTypeDef fake_adbms6830_open_wire_phase(adbms6830_driver_t *dev, bool odd_channels){
    if(dev == NULL) return HAL_ERROR;
    dev->health.last_op = odd_channels ? ADBMS6830_SPI_OP_OPEN_WIRE_ODD : ADBMS6830_SPI_OP_OPEN_WIRE_EVEN;
    dev->health.last_status = fake_adbms_diag_status;
    dev->spi_debug.last_op = dev->health.last_op;
    dev->spi_debug.last_status = fake_adbms_diag_status;
    if(odd_channels) dev->health.open_wire_odd_count++; else dev->health.open_wire_even_count++;
    dev->health.open_wire_even_valid_ic_mask = 0u;
    dev->health.open_wire_odd_valid_ic_mask = 0u;
    dev->health.open_wire_incomplete_ic_mask = 0u;
    dev->health.open_wire_fault_ic_mask = 0u;
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics && ic < ADBMS6830_MAX_TRACKED_ICS; ic++){
        uint16_t bit = (uint16_t)(1u << ic);
        if(odd_channels){
            dev->diag[ic].open_wire_odd_valid = (fake_adbms_diag_status == HAL_OK);
            dev->diag[ic].open_wire_odd_fault_mask = 0u;
        }
        else{
            dev->diag[ic].open_wire_even_valid = (fake_adbms_diag_status == HAL_OK);
            dev->diag[ic].open_wire_even_fault_mask = 0u;
        }
        dev->diag[ic].open_wire_fault_mask =
            (uint16_t)(dev->diag[ic].open_wire_even_fault_mask |
                       dev->diag[ic].open_wire_odd_fault_mask);
        dev->health.open_wire_cell_fault_mask[ic] = dev->diag[ic].open_wire_fault_mask;
        if(dev->diag[ic].open_wire_even_valid) dev->health.open_wire_even_valid_ic_mask |= bit;
        if(dev->diag[ic].open_wire_odd_valid) dev->health.open_wire_odd_valid_ic_mask |= bit;
        if((!dev->diag[ic].open_wire_baseline_valid) ||
           (!dev->diag[ic].open_wire_even_valid) ||
           (!dev->diag[ic].open_wire_odd_valid)){
            dev->health.open_wire_incomplete_ic_mask |= bit;
        }
    }
    return fake_adbms_diag_status;
}

HAL_StatusTypeDef adbms6830_run_open_wire_check(adbms6830_driver_t *dev, bool odd_channels)
{
    fake_adbms_open_wire_call_count++;
    HAL_StatusTypeDef status = fake_adbms6830_open_wire_baseline(dev);
    if(status != HAL_OK) return status;
    return fake_adbms6830_open_wire_phase(dev, odd_channels);
}

HAL_StatusTypeDef adbms6830_run_open_wire_diagnostic(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef baseline_status;
    HAL_StatusTypeDef even_status;
    HAL_StatusTypeDef odd_status;
    HAL_StatusTypeDef result;
    if(dev == NULL) return HAL_ERROR;
    fake_adbms_open_wire_call_count++;
    dev->health.open_wire_full_count++;
    baseline_status = fake_adbms6830_open_wire_baseline(dev);
    even_status = (baseline_status == HAL_OK) ?
                  fake_adbms6830_open_wire_phase(dev, false) : baseline_status;
    odd_status = (baseline_status == HAL_OK) ?
                 fake_adbms6830_open_wire_phase(dev, true) : baseline_status;
    result = ((baseline_status == HAL_OK) &&
              (even_status == HAL_OK) && (odd_status == HAL_OK) &&
              (dev->health.open_wire_incomplete_ic_mask == 0u) &&
              (dev->health.open_wire_fault_ic_mask == 0u)) ? HAL_OK : HAL_ERROR;
    dev->health.last_op = ADBMS6830_SPI_OP_OPEN_WIRE_FULL;
    dev->health.last_status = result;
    dev->spi_debug.last_op = ADBMS6830_SPI_OP_OPEN_WIRE_FULL;
    dev->spi_debug.last_status = result;
    return result;
}
HAL_StatusTypeDef adbms6830_run_aux_gpio_diagnostic(adbms6830_driver_t *dev){
    if(dev == NULL) return HAL_ERROR;
    dev->health.aux_gpio_diag_count++;
    dev->health.last_op = ADBMS6830_SPI_OP_AUX_GPIO_DIAG;
    dev->health.last_status = fake_adbms_diag_status;
    dev->spi_debug.last_op = ADBMS6830_SPI_OP_AUX_GPIO_DIAG;
    dev->spi_debug.last_status = fake_adbms_diag_status;
    return fake_adbms_diag_status;
}

int mux_read_gpio_voltage(adbms6830_driver_t *dev, uint8_t sensor_num){
    if(fake_mux_write_enable && dev && dev->ics && dev->num_ics > 0 && sensor_num < 24){
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics && ic < ADBMS6830_MAX_TRACKED_ICS; ic++){
            dev->ics[ic].temp.raw[sensor_num] = (int16_t)((2.5f/0.000150f)-10000.0f);
            dev->last_temp_updated_mask[ic] |= (uint32_t)(1UL << sensor_num);
        }
    }
    else if(dev && sensor_num < 24){
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics && ic < ADBMS6830_MAX_TRACKED_ICS; ic++){
            dev->last_temp_updated_mask[ic] |= (uint32_t)(1UL << sensor_num);
        }
    }
    return (sensor_num < 24) ? 0 : -1;
}
int adbms6830_read_temp_raw(adbms6830_driver_t *dev, uint8_t ic_idx, uint8_t sensor_num, int16_t *out_raw){ (void)dev;(void)ic_idx;(void)sensor_num; if(out_raw) *out_raw=0; return 0; }
float adbms6830_convert_temp(adbms6830_driver_t *dev, uint8_t ic_idx, uint8_t sensor_num, float vref){ (void)dev;(void)ic_idx;(void)sensor_num;(void)vref; return 25.0f; }
float voltage_to_temp(float raw){ if(!isfinite(raw) || raw < (float)INT16_MIN || raw > (float)INT16_MAX) return NAN; thermistor_result_t result = thermistor_from_adbms_raw((int16_t)lroundf(raw), THERMISTOR_NOMINAL_VREG_V); return result.valid ? result.temperature_c : NAN; }
int mux_set_channel(adbms6830_driver_t *dev, uint8_t sensor_num){ (void)dev; return sensor_num < 24 ? 0 : -1; }

void adbms2950_gpo_set(adbms2950_driver_t *dev, GPO gp, CFGA_GPO state){(void)dev;(void)gp;(void)state;} void adbms2950_wakeup(adbms2950_driver_t *dev){(void)dev;} void adbms2950_wrcfga(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdcfga(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdvb(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdi(adbms2950_driver_t *dev){(void)dev;} void adbms2950_adv(adbms2950_driver_t *dev, adv_ *adv){(void)dev;(void)adv;} void adbms2950_plv(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdv1d(adbms2950_driver_t *dev){(void)dev;}
HAL_StatusTypeDef adbms2950_init_mixed_chain(adbms2950_driver_t *dev,
                                              uint8_t num_asics,
                                              adbms2950_asic *ics,
                                              uint8_t ics_capacity,
                                              SPI_HandleTypeDef *hspi,
                                              GPIO_TypeDef *cs_a,
                                              GPIO_TypeDef *cs_b,
                                              uint16_t pin_a,
                                              uint16_t pin_b,
                                              TIM_HandleTypeDef *htim,
                                              adbms_string primary_string,
                                              bool issue_chain_reset,
                                              bool enable_hv_dividers)
{
    fake_apm_init_call_count++;
    if((dev == NULL) || (ics == NULL) || (num_asics == 0u) ||
       (num_asics > ics_capacity) || (num_asics > ADBMS2950_MAX_TRACKED_ICS) ||
       (hspi == NULL) || (htim == NULL) || (cs_a == NULL) || (cs_b == NULL) ||
       (pin_a == 0u) || (pin_b == 0u) || (primary_string > STRING_B))
    {
        return HAL_ERROR;
    }
    memset(dev, 0, sizeof(*dev));
    memset(ics, 0, sizeof(*ics) * num_asics);
    dev->num_ics = num_asics;
    dev->ics_capacity = ics_capacity;
    dev->ics = ics;
    dev->hspi = hspi;
    dev->cs_port[STRING_A] = cs_a;
    dev->cs_port[STRING_B] = cs_b;
    dev->cs_pin[STRING_A] = pin_a;
    dev->cs_pin[STRING_B] = pin_b;
    dev->htim = htim;
    dev->string = primary_string;
    dev->write_string = primary_string;
	fake_apm_init_string = primary_string;
	fake_apm_init_requested_reset = issue_chain_reset;
	fake_apm_init_enabled_dividers = enable_hv_dividers;
    dev->spi_debug.enabled = true;
    dev->health.hv_dividers_enabled = enable_hv_dividers;
    dev->health.last_status = fake_apm_init_status;
    if(fake_apm_init_status == HAL_OK)
    {
        dev->health.initialized = true;
        dev->health.sid_valid = true;
        dev->health.config_valid = true;
        dev->health.i1_calibrated = true;
        dev->health.i1_continuous_ready = true;
        dev->health.i1_conversion_count = 136u;
        dev->health.counter_seen = true;
        dev->health.counter_advanced = true;
        dev->health.device_id = ADBMS2950B_DEVICE_ID;
        dev->health.sid[5] = (uint8_t)(ADBMS2950B_DEVICE_ID << 1u);
    }
    return fake_apm_init_status;
}
HAL_StatusTypeDef adbms2950_wakeup_checked(adbms2950_driver_t *dev){ return dev ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef adbms2950_us_delay(adbms2950_driver_t *dev, uint16_t us){ (void)us; return dev ? HAL_OK : HAL_ERROR; }
void adbms2950_spi_debug_enable(adbms2950_driver_t *dev, bool enable){ if(dev) dev->spi_debug.enabled = enable; }
void adbms2950_spi_debug_clear(adbms2950_driver_t *dev){ if(dev){ bool en = dev->spi_debug.enabled; memset(&dev->spi_debug, 0, sizeof(dev->spi_debug)); dev->spi_debug.enabled = en; } }
const adbms2950_spi_debug_t *adbms2950_spi_debug_get(const adbms2950_driver_t *dev){ return dev ? &dev->spi_debug : NULL; }
const adbms2950_health_t *adbms2950_health_get(const adbms2950_driver_t *dev){ return dev ? &dev->health : NULL; }
void adbms2950_health_clear_counters(adbms2950_driver_t *dev){ if(dev){ dev->health.sample_count=0u; dev->health.sample_error_count=0u; dev->health.pec_error_count=0u; dev->health.counter_mismatch_count=0u; dev->health.counter_stall_count=0u; } }
const char *adbms2950_spi_op_str(adbms2950_spi_op_t op){
    switch(op){
        case ADBMS2950_SPI_OP_NONE: return "none";
        case ADBMS2950_SPI_OP_CMD: return "cmd";
        case ADBMS2950_SPI_OP_WR48: return "wr48";
        case ADBMS2950_SPI_OP_RD48: return "rd48";
        case ADBMS2950_SPI_OP_PROBE: return "probe";
        default: return "unknown";
    }
}
HAL_StatusTypeDef adbms2950_spi_probe_rdcfga(adbms2950_driver_t *dev){
    if(dev == NULL) return HAL_ERROR;
    dev->spi_debug.last_op = ADBMS2950_SPI_OP_PROBE;
    dev->spi_debug.last_status = HAL_OK;
    dev->spi_debug.rx_count++;
    return HAL_OK;
}
HAL_StatusTypeDef adbms2950_read_sid(adbms2950_driver_t *dev){
    if(dev == NULL) return HAL_ERROR;
    dev->health.last_status = fake_apm_probe_status;
    dev->health.sid_valid = (fake_apm_probe_status == HAL_OK);
    if(dev->health.sid_valid){ dev->health.device_id=ADBMS2950B_DEVICE_ID; dev->health.sid[5]=(uint8_t)(ADBMS2950B_DEVICE_ID<<1u); }
    return fake_apm_probe_status;
}
HAL_StatusTypeDef adbms2950_spi_probe_sid(adbms2950_driver_t *dev){
    fake_apm_probe_call_count++; HAL_StatusTypeDef s=adbms2950_read_sid(dev); if(dev){ dev->spi_debug.last_op=ADBMS2950_SPI_OP_PROBE; dev->spi_debug.last_status=s; dev->spi_debug.rx_count++; } return s;
}
HAL_StatusTypeDef adbms2950_read_status(adbms2950_driver_t *dev){ if(!dev) return HAL_ERROR; dev->health.i1_calibrated=(fake_apm_sample_status==HAL_OK); return fake_apm_sample_status; }
HAL_StatusTypeDef adbms2950_read_primary_sample(adbms2950_driver_t *dev, uint32_t now_ms){
    fake_apm_sample_call_count++;
    if((dev == NULL) || !dev->health.initialized) return HAL_ERROR;
    dev->health.last_status=fake_apm_sample_status;
    if(fake_apm_sample_status != HAL_OK){ dev->health.sample_valid=false; dev->health.current_valid=false; dev->health.pack_voltage_valid=false; if(dev->health.sample_error_count!=UINT32_MAX) dev->health.sample_error_count++; return fake_apm_sample_status; }
    dev->health.i1_calibrated=true; dev->health.i1_continuous_ready=true; dev->health.counter_seen=true; dev->health.counter_advanced=true; dev->health.i1_conversion_count++; dev->health.last_i1cntpha=(uint16_t)(dev->health.i1_conversion_count<<2u); dev->health.sample_valid=true; dev->health.current_valid=true; dev->health.pack_voltage_valid=dev->health.hv_dividers_enabled; dev->health.i1_raw=fake_apm_i1_raw; dev->health.vb1_raw=fake_apm_vb1_raw; dev->health.current_a=(float)fake_apm_i1_raw*0.01f; dev->health.pack_voltage_v=(float)fake_apm_vb1_raw*VBAT1_SCALE*VBAT_DIV_SCALE; dev->health.last_update_ms=now_ms; if(dev->health.sample_count!=UINT32_MAX) dev->health.sample_count++; return HAL_OK;
}
void adbms2950_note_compatible_adi1(adbms2950_driver_t *dev){ if(dev){ dev->health.counter_seen=false; dev->health.counter_advanced=false; dev->health.sample_valid=false; dev->health.current_valid=false; dev->health.pack_voltage_valid=false; } }
HAL_StatusTypeDef adbms2950_verify_config_readback(adbms2950_driver_t *dev){ return dev ? fake_apm_init_status : HAL_ERROR; }
HAL_StatusTypeDef stm32f767z_adc_switch_channel(ADC_HandleTypeDef *hadc, uint32_t channel){ (void)channel; return hadc ? HAL_OK : HAL_ERROR; }
stm32f767z_adc_read_result_t stm32f767z_adc_read_checked(ADC_HandleTypeDef *hadc, uint32_t timeout_ms){
    (void)timeout_ms;
    stm32f767z_adc_read_result_t r = { HAL_ERROR, 0u };
    uint32_t idx;

    if(!hadc) return r;

    idx = fake_adc_read_index;
    if(idx > 1u) idx = 1u;

    r.status = fake_adc_read_statuses[idx];
    r.count = (r.status == HAL_OK) ? fake_adc_read_counts[idx] : 0u;
    fake_adc_read_index++;
    return r;
}
uint16_t stm32f767z_adc_read(ADC_HandleTypeDef *hadc){ return stm32f767z_adc_read_checked(hadc, 5u).count; }

// Include actual implementation files so static helpers in canbus_task are testable.
#include "Core/Src/ext_drivers/charger.c"
#include "Core/Src/ext_drivers/fans.c"
#include "Core/Src/ext_drivers/current_sensor.c"
#include "Core/Src/ext_drivers/current_fault.c"
#include "Core/Src/ext_drivers/voltage_fault.c"
#include "Core/Src/ext_drivers/temperature_fault.c"
#include "Core/Src/ext_drivers/imd.c"
#include "Core/Src/ext_drivers/accumulator.c"
#include "Core/Src/ext_drivers/ams_safety.c"
#include "Core/Src/ext_drivers/ams_rtos_diag.c"
#include "Core/Src/ext_drivers/canbus.c"
#include "Core/Src/measurement/ams_measurement.c"
#include "Core/Src/estimator/ams_estimator_lut.c"
#include "Core/Src/estimator/ams_soc_ekf.c"
int cli_printline(cli_device_t *dev, char *line)
{
    (void)dev;
    if(line == NULL)
    {
        return HAL_ERROR;
    }
    cli_capture_append((const uint8_t *)line, (uint16_t)cli_bounded_strlen(line, CLI_LINESZ - 1u));
    cli_capture_append((const uint8_t *)"\n", 1u);
    return HAL_OK;
}

void cli_device_init(cli_device_t *dev, UART_HandleTypeDef *huart)
{
    if(dev == NULL)
    {
        return;
    }
    memset(dev, 0, sizeof(*dev));
    dev->huart = huart;
}

HAL_StatusTypeDef cli_uart_start_rx(cli_device_t *dev)
{
    return (dev != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef cli_uart_service_rx(cli_device_t *dev)
{
    return (dev != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef cli_uart_force_recover(cli_device_t *dev)
{
    if(dev == NULL)
    {
        return HAL_ERROR;
    }
    dev->rx_recovery_count++;
    return HAL_OK;
}

void cli_uart_note_error(cli_device_t *dev, uint32_t error_code)
{
    if((dev != NULL) && (error_code != HAL_UART_ERROR_NONE))
    {
        dev->uart_error_count++;
        dev->uart_last_error = error_code;
    }
}

void cli_uart_diag_clear(cli_device_t *dev)
{
    if(dev != NULL)
    {
        dev->uart_error_count = 0u;
        dev->uart_last_error = HAL_UART_ERROR_NONE;
        dev->rx_recovery_count = 0u;
    }
}

int tokenize(char *s, char *toks[], int maxtoks, char *delim)
{
    int count = 0;
    if((s == NULL) || (toks == NULL) || (delim == NULL) || (maxtoks <= 0))
    {
        return 0;
    }
    char *tok = strtok(s, delim);
    while((tok != NULL) && (count < (maxtoks - 1)))
    {
        toks[count++] = tok;
        tok = strtok(NULL, delim);
    }
    toks[count] = NULL;
    return count;
}

#include "Core/Src/tasks/canbus_task.c"
#include "Core/Src/tasks/cli_task.c"
#include "Core/Src/tasks/adbms_task.c"
#include "Core/Src/tasks/error_task.c"
#include "Core/Src/tasks/fan_task.c"
#include "Core/Src/tasks/current_task.c"
#include "Core/Src/tasks/imd_task.c"
#include "Core/Src/tasks/estimator_task.c"

static void host_can_rx_isr_only(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_RxFifo0MsgPendingCallback(hcan);
}

static uint32_t host_receive_can_frame(CAN_HandleTypeDef *hcan)
{
    host_can_rx_isr_only(hcan);
    return canbus_process_rx_queue(&app.board.canbus, &app, CANBUS_RX_QUEUE_DEPTH);
}

static uint16_t word_at(uint32_t frame, uint8_t word_index){ return ((uint16_t)tx_log[frame].data[word_index*2] << 8) | tx_log[frame].data[word_index*2+1]; }
static int16_t code_for_volts(float v){ return (int16_t)((v / 0.000150f) - 10000.0f); }
static int16_t raw_for_ntc_voltage(float v){ return (int16_t)((v / 0.000150f) - 10000.0f); }
static int16_t raw_for_temp_c(float temp_c)
{
    int16_t raw = 0;
    return thermistor_adbms_raw_from_temperature_c(
               temp_c, THERMISTOR_NOMINAL_VREG_V, &raw) ?
           raw : THERMISTOR_ADBMS_RESET_CODE;
}
#define CHECK(cond) do{ if(!(cond)){ fprintf(stderr,"FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); exit(1);} }while(0)
#define HOST_LOGGER_FRAME_COUNT 120u
#define HOST_ECU_FRAME_COUNT 62u
#define HOST_ECU_COMPACT_FRAME_COUNT 4u
#define HOST_ECU_PHASE0_FRAME_COUNT 18u
#define HOST_LOGGER_PHASE0_FRAME_COUNT 41u
#define HOST_LEGACY_ECU_FRAME_OFFSET HOST_ECU_COMPACT_FRAME_COUNT
#define HOST_NONCHARGE_CAN_FRAME_COUNT \
    (HOST_ECU_COMPACT_FRAME_COUNT + HOST_ECU_PHASE0_FRAME_COUNT + \
     HOST_LOGGER_PHASE0_FRAME_COUNT)
#define HOST_CHARGE_CAN_FRAME_COUNT \
    (HOST_ECU_COMPACT_FRAME_COUNT + HOST_LOGGER_PHASE0_FRAME_COUNT + 1u)
#define HOST_CHARGER_FRAME_INDEX 0u

static void sil_mark_all_heartbeats_alive(app_data_t *d);
static uint32_t host_publish_measurement_snapshot(app_data_t *d,
                                                  uint32_t voltage_tick,
                                                  float current_A,
                                                  uint32_t validity_flags);

static void sil_bind_final_ring_topology(accumulator_t *acc)
{
    if(acc == NULL) return;
    acc->smb.hspi = &fake_topology_spi;
    acc->smb.htim = &fake_topology_timer;
    acc->smb.cs_port[STRING_A] = &dummy_gpio;
    acc->smb.cs_port[STRING_B] = &dummy_gpio;
    acc->smb.cs_pin[STRING_A] = 0x0002u;
    acc->smb.cs_pin[STRING_B] = 0x0004u;
    acc->smb.write_string = STRING_A;
    acc->apm.hspi = &fake_topology_spi;
    acc->apm.htim = &fake_topology_timer;
    acc->apm.cs_port[STRING_A] = &dummy_gpio;
    acc->apm.cs_port[STRING_B] = &dummy_gpio;
    acc->apm.cs_pin[STRING_A] = 0x0002u;
    acc->apm.cs_pin[STRING_B] = 0x0004u;
    acc->apm.write_string = STRING_B;
}

static void init_fake_app(void){ fake_tick = 0u; memset(&app,0,sizeof(app)); ams_safety_host_reset_state(); ams_rtos_host_reset_state(); ams_rtos_diag_init(&app); app.state = STATE_START; app.acc.smb.num_ics = NSMBS; app.acc.smb.physical_chain_count = (uint8_t)AMS_ADBMS_PHYSICAL_CHAIN_COUNT; app.acc.smb.ics_capacity = NSMBS; app.acc.smb.ics = app.acc.smb_ics; app.acc.smb.string = STRING_A; app.acc.smb.health.startup_baseline_passed = true; app.acc.delay_timer_ready = true; app.acc.delay_timer_status = HAL_OK; app.acc.smb_ready = true; app.acc.smb_init_status = HAL_OK; app.acc.apm.num_ics = NAPMS; app.acc.apm.ics_capacity = NAPMS; app.acc.apm.ics = app.acc.apm_ics; app.acc.apm.string = STRING_B; app.acc.apm_ready = true; app.acc.apm_init_status = HAL_OK; app.acc.apm.health.initialized = true; app.acc.apm.health.i1_calibrated = true; app.acc.apm.health.i1_continuous_ready = true; app.acc.apm.health.sid_valid = true; app.acc.apm.health.config_valid = true; app.acc.apm.health.device_id = ADBMS2950B_DEVICE_ID; app.acc.apm.health.sid[5] = (uint8_t)(ADBMS2950B_DEVICE_ID << 1u); sil_bind_final_ring_topology(&app.acc); current_fault_init(&app.current_fault_state); voltage_fault_init(&app.voltage_fault_state); temperature_fault_init(&app.temp_fault_state); ams_heartbeat_init(&app, fake_tick); ams_safety_watchdog_boot_arm(&app); app.current_meas_reason = CURRENT_SENSOR_REASON_ADC_READ; app.current_fault_reason = CURRENT_FAULT_REASON_SENSOR_NOT_READY; app.voltage_fault_reason = VOLTAGE_FAULT_REASON_NOT_READY; app.temp_fault = true; app.temp_read_fault = true; app.temp_fan_max = true; app.temp_fault_reason = TEMPERATURE_FAULT_REASON_NOT_READY; app.imd_valid = true; app.imd_ok = true; app.imd_fault = false; app.imd_status = IMD_NORMAL; app.balance_inhibit = (AMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT != 0); fake_adbms_voltage_masks_full_update(); fake_adc_read_index = 0u; fake_adbms_init_status = HAL_OK; fake_adbms_start_conversion_status = HAL_OK; fake_adbms_read_cell_status = HAL_OK; fake_apm_init_status = HAL_OK; fake_apm_sample_status = HAL_OK; fake_apm_probe_status = HAL_OK; fake_apm_init_call_count = 0u; fake_apm_sample_call_count = 0u; fake_apm_probe_call_count = 0u; fake_adbms_wrcfgb_call_count = 0u; fake_adbms_wrpwm_call_count = 0u; fake_adbms_open_wire_call_count = 0u; fake_apm_i1_raw = 1234; fake_apm_vb1_raw = 18000; fake_apm_init_string = STRING_A; fake_apm_init_requested_reset = true; fake_apm_init_enabled_dividers = true; fake_adbms_wrcfgb_status = HAL_OK; fake_adbms_wrpwm_status = HAL_OK; fake_adbms_balance_verify_status = HAL_OK; fake_adbms_wrpwm_fail_after_ok = -1; fake_adbms_diag_status = HAL_OK; fake_adbms_config_mismatch_mask = 0u; fake_adbms_delay_advances_tick = false; fake_can_add_tx_status = HAL_OK; fake_can_add_tx_call_count = 0u; fake_can_fail_on_call = 0u; fake_can_advance_tick_per_tx_ms = 0u; fake_can_mutate_after_tx_count = 0u; fake_can_error = HAL_CAN_ERROR_NONE; fake_can_recover_status = HAL_OK; fake_can_notification_status = HAL_OK; fake_can_filter_status = HAL_OK; fake_can_filter_count = 0u; memset(fake_can_filter_log, 0, sizeof(fake_can_filter_log)); fake_rx_status = HAL_OK; fake_tim_base_start_status = HAL_OK; fake_tim_pwm_start_status = HAL_OK; fake_tim_ic_start_it_status = HAL_OK; fake_tim_ic_start_status = HAL_OK; fake_tim_total_capture = 1000u; fake_tim_high_capture = 500u; memset(&fake_rx_hdr, 0, sizeof(fake_rx_hdr)); memset(fake_rx_data, 0, sizeof(fake_rx_data)); bms_pin_state = GPIO_PIN_RESET; }

static void host_mark_updated_cells(app_data_t *d)
{
    if(d == NULL) return;
    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        d->acc.smb.last_cell_updated_mask[ic] = (ic < (uint8_t)d->acc.smb.num_ics) ? 0x7FFFu : 0u;
        d->acc.smb.last_cell_pec_mask[ic] = 0u;
    }
}

static void host_mark_updated_temps(app_data_t *d, uint32_t mask)
{
    if(d == NULL) return;
    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        d->acc.smb.last_temp_updated_mask[ic] = (ic < (uint8_t)d->acc.smb.num_ics) ? (mask & ((1UL << NTEMPS) - 1UL)) : 0u;
    }
}

static void host_send_hil_cell_triplet(uint8_t seg,
                                       uint8_t first_cell,
                                       uint16_t mv0,
                                       uint16_t mv1,
                                       uint16_t mv2)
{
    memset(&fake_rx_hdr, 0, sizeof(fake_rx_hdr));
    fake_rx_hdr.IDE = CAN_ID_STD;
    fake_rx_hdr.StdId = AMS_HIL_CAN_ID_CELL_SAMPLE;
    fake_rx_hdr.DLC = 8u;
    fake_rx_data[0] = seg;
    fake_rx_data[1] = first_cell;
    fake_rx_data[2] = (uint8_t)(mv0 >> 8);
    fake_rx_data[3] = (uint8_t)(mv0 & 0xFFu);
    fake_rx_data[4] = (uint8_t)(mv1 >> 8);
    fake_rx_data[5] = (uint8_t)(mv1 & 0xFFu);
    fake_rx_data[6] = (uint8_t)(mv2 >> 8);
    fake_rx_data[7] = (uint8_t)(mv2 & 0xFFu);
    app.board.canbus.hcan = &hil_fake_hcan;
    (void)host_receive_can_frame(&hil_fake_hcan);
}

static void host_send_hil_temp_triplet(uint8_t seg,
                                       uint8_t first_sensor,
                                       int16_t t0_deci_c,
                                       int16_t t1_deci_c,
                                       int16_t t2_deci_c)
{
    memset(&fake_rx_hdr, 0, sizeof(fake_rx_hdr));
    fake_rx_hdr.IDE = CAN_ID_STD;
    fake_rx_hdr.StdId = AMS_HIL_CAN_ID_TEMP_SAMPLE;
    fake_rx_hdr.DLC = 8u;
    fake_rx_data[0] = seg;
    fake_rx_data[1] = first_sensor;
    fake_rx_data[2] = (uint8_t)((uint16_t)t0_deci_c >> 8);
    fake_rx_data[3] = (uint8_t)((uint16_t)t0_deci_c & 0xFFu);
    fake_rx_data[4] = (uint8_t)((uint16_t)t1_deci_c >> 8);
    fake_rx_data[5] = (uint8_t)((uint16_t)t1_deci_c & 0xFFu);
    fake_rx_data[6] = (uint8_t)((uint16_t)t2_deci_c >> 8);
    fake_rx_data[7] = (uint8_t)((uint16_t)t2_deci_c & 0xFFu);
    app.board.canbus.hcan = &hil_fake_hcan;
    (void)host_receive_can_frame(&hil_fake_hcan);
}

static void test_accumulator_stats_and_balance(void){
    init_fake_app();
    for(int ic=0; ic<NSMBS; ic++) for(int c=0;c<NCELLS;c++) app.acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(3.700f);
    app.acc.smb_ics[2].cell.c_codes[0] = code_for_volts(4.100f);
    for(int c=1; c<=6; c++) app.acc.smb_ics[2].cell.c_codes[c] = code_for_volts(4.140f);
    app.acc.smb_ics[4].cell.c_codes[14] = code_for_volts(3.200f);
    host_mark_updated_cells(&app);
    accumulator_update_voltage_stats(&app.acc);
    CHECK(fabsf(app.acc.max_volt - 4.140f) < 0.002f);
    CHECK(fabsf(app.acc.min_volt - 3.200f) < 0.002f);
    CHECK(app.acc.total_volt > 279.0f && app.acc.total_volt < 281.0f);
    CHECK(accumulator_set_balance(&app.acc) == 0);
    CHECK((app.acc.smb_ics[0].tx_cfgb.dcc) == 0u);
    CHECK((app.acc.smb_ics[1].tx_cfgb.dcc) == 0u);
    CHECK(app.acc.smb_ics[2].tx_cfgb.dcc == 0u);
    CHECK(sil_balance_pwm_duty(&app, 2u, 0u) == 0u);
    CHECK(sil_balance_pwm_duty(&app, 2u, 1u) == BALANCE_PWM_DUTY);
    CHECK(sil_balance_pwm_duty(&app, 2u, 2u) == BALANCE_PWM_DUTY);
    CHECK(sil_balance_pwm_duty(&app, 2u, 3u) == BALANCE_PWM_DUTY);
    CHECK(sil_balance_pwm_duty(&app, 2u, 4u) == BALANCE_PWM_DUTY);
    CHECK(sil_balance_pwm_duty(&app, 2u, 5u) == 0u);
    CHECK(sil_balance_pwm_duty(&app, 2u, 6u) == 0u);
    CHECK((app.acc.smb_ics[3].tx_cfgb.dcc) == 0u);
    CHECK(sil_balance_pwm_duty(&app, 4u, 14u) == 0u);
    fake_adbms_balance_verify_status = HAL_ERROR;
    CHECK(accumulator_set_balance(&app.acc) == -1);
    sil_expect_balancing_clear(&app);
    fake_adbms_balance_verify_status = HAL_OK;
    fake_adbms_wrpwm_status = HAL_ERROR;
    CHECK(accumulator_set_balance(&app.acc) == -1);
    sil_expect_balancing_clear(&app);
    fake_adbms_wrpwm_status = HAL_OK;
    fake_adbms_wrcfgb_status = HAL_ERROR;
    CHECK(accumulator_clear_balance(&app.acc) == -1);
    fake_adbms_wrcfgb_status = HAL_OK;
    fake_adbms_balance_verify_status = HAL_ERROR;
    CHECK(accumulator_clear_balance(&app.acc) == -1);
    fake_adbms_balance_verify_status = HAL_OK;
    CHECK(accumulator_clear_balance(&app.acc) == 0);
    sil_expect_balancing_clear(&app);

    app.acc.smb.num_ics = 99; /* Corrupt topology must fail closed and stay bounded. */
    accumulator_update_voltage_stats(&app.acc);
    CHECK(app.acc.valid_voltage_count == 0u);
    CHECK(app.acc.voltage_full_usable == false);
    CHECK(app.acc.max_volt == 0.0f && app.acc.min_volt == 0.0f);
}

static void test_adbms_voltage_scan_timing_contract(void)
{
    init_fake_app();
    memset(fake_adbms_delay_values_us, 0, sizeof(fake_adbms_delay_values_us));
    fake_adbms_delay_calls = 0u;

    CHECK(smb_read_voltage(&app.acc.smb) == 0);
    CHECK(fake_adbms_delay_calls == 2u);
    CHECK(fake_adbms_delay_values_us[0] ==
          ADBMS6830_REFERENCE_PRECONVERSION_WAIT_US);
    CHECK(fake_adbms_delay_values_us[1] ==
          ADBMS6830_REDUNDANT_CONVERSION_WAIT_US);
    CHECK(fake_adbms_delay_values_us[1] > 16000u);

    /* A past absolute deadline must never rewind the host clock. The previous
     * fake did so and hid production deadline overruns. Preserve wraparound
     * behavior while advancing only to a genuinely future tick. */
    fake_tick = 500u;
    CHECK(osDelayUntil(400u) == osOK);
    CHECK(fake_tick == 500u);
    fake_tick = UINT32_MAX - 5u;
    CHECK(osDelayUntil(4u) == osOK);
    CHECK(fake_tick == 4u);

    /* Non-balancing scans retain the configured 10 Hz schedule without an
     * unconditional recovery delay. */
    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    app.state = STATE_DISCARGE;
    fake_tick = 1000u;
    run_one_adbms_task_iteration(&app);
    CHECK(fake_tick == 1000u + AMS_ADBMS_TASK_PERIOD_MS);
    CHECK(app.adbms_balance_recovery_count == 0u);
    CHECK(app.adbms_balance_active == false);
    CHECK(app.adbms_last_schedule_interval_ms == AMS_ADBMS_TASK_PERIOD_MS);

    /* Once PWM is actually selected, each subsequent scan performs the full
     * off/recovery interval and then leaves the verified PWM command active
     * for at least the configured minimum on-time. */
    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    app.state = STATE_CHARGE;
    app.current_valid = true;
    app.bms_state = true;
    app.balance_inhibit = false;
    app.acc.smb_ics[0].cell.c_codes[0] = code_for_volts(4.100f);
    app.acc.smb_ics[0].cell.c_codes[1] = code_for_volts(4.180f);
    fake_tick = 2000u;
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_balance_active == true);
    CHECK(accumulator_balance_shadow_active(&app.acc));
    uint32_t expected_first_balance_interval =
        (AMS_ADBMS_TASK_PERIOD_MS > AMS_ADBMS_BALANCE_MIN_ON_MS) ?
            AMS_ADBMS_TASK_PERIOD_MS : AMS_ADBMS_BALANCE_MIN_ON_MS;
    CHECK(fake_tick == 2000u + expected_first_balance_interval);

    uint32_t previous_iteration_end = fake_tick;
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_balance_recovery_count == 1u);
    CHECK(app.adbms_balance_active == true);
	CHECK(app.adbms_last_balance_on_ms >= AMS_ADBMS_BALANCE_MIN_ON_MS);
	CHECK(app.adbms_last_balance_off_ms >= AMS_ADBMS_BALANCE_RECOVERY_MS);
    CHECK((uint32_t)(fake_tick - previous_iteration_end) >=
          (AMS_ADBMS_BALANCE_RECOVERY_MS + AMS_ADBMS_BALANCE_MIN_ON_MS));
    CHECK(app.adbms_last_scan_duration_ms >= AMS_ADBMS_BALANCE_RECOVERY_MS);
}

static void test_temp_stats(void){
    init_fake_app();
    for(int ic=0; ic<NSMBS; ic++) for(int s=0;s<NTEMPS;s++) app.acc.smb_ics[ic].temp.raw[s] = raw_for_ntc_voltage(2.5f);
    app.acc.smb_ics[1].temp.raw[2] = -1; // ADBMS reset sentinel
    app.acc.smb_ics[3].temp.raw[5] = INT16_MIN; // invalid skip
    host_mark_updated_temps(&app, (1UL << NTEMPS) - 1UL);
    accumulator_update_temp_stats(&app.acc);
    CHECK(app.acc.max_temp > 20.0f && app.acc.max_temp < 30.0f);
    CHECK(app.acc.avg_temp > 20.0f && app.acc.avg_temp < 30.0f);
    CHECK(app.acc.usable_temp_count == (uint16_t)(NSMBS * NTEMPS - 2));
    CHECK(app.acc.invalid_temp_count == 2u);
    CHECK(app.acc.temp_startup_scan_complete == false);
}

static void test_can_telemetry_packets(void){
    init_fake_app(); static CAN_HandleTypeDef hcan; app.board.canbus.hcan = &hcan;
    app.state = STATE_DISCARGE; app.air_state = true; app.current = -12.3f; app.imd_ok=true; app.imd_status=IMD_NORMAL; app.board.imd.duty=42.5f; app.max_temp=37.2f; app.min_voltage=3.201f; app.max_voltage=4.099f;
    for(int i=0;i<NFANS;i++) app.board.fans[i].duty_cycle = (float)(i*10);
    for(int ic=0; ic<NSMBS; ic++) for(int c=0;c<NCELLS;c++) app.acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(3.0f + 0.001f*(float)(ic*NCELLS+c));
    host_mark_updated_cells(&app);
    accumulator_update_voltage_stats_at(&app.acc, fake_tick);
    for(int ic=0; ic<NSMBS; ic++) for(int s=0;s<NTEMPS;s++) app.acc.smb_ics[ic].temp.raw[s] = raw_for_temp_c(37.2f);
    host_mark_updated_temps(&app, (1UL << NTEMPS) - 1UL);
    accumulator_update_temp_stats_at(&app.acc, fake_tick);
    sil_publish_temp_state(&app);
    tx_count=0; tx_free_level=3;
    CHECK(send_ecu_ams_status(&app.board.canbus, &app) == HAL_OK);
    CHECK(send_ecu_ams_voltages(&app.board.canbus, &app) == HAL_OK);
    CHECK(send_ecu_ams_temps(&app.board.canbus, &app) == HAL_OK);
    CHECK(send_ecu_ams_fans(&app.board.canbus, &app) == HAL_OK);
    CHECK(tx_count == 62u);
    for(uint32_t i=0;i<tx_count;i++){ CHECK(tx_log[i].ide == CAN_ID_STD); CHECK(tx_log[i].stdid == ECU_CANBUS_ID); CHECK(tx_log[i].dlc == 8u); CHECK(word_at(i,0) == i); }
    CHECK(word_at(0,1) == STATE_DISCARGE); CHECK(word_at(0,2) == 1u); CHECK((int16_t)word_at(0,3) == -123);
    CHECK(word_at(2,1) == 372u); CHECK(word_at(2,2) == 3201u); CHECK(word_at(2,3) == 4099u);
    // Last voltage packet of last segment carries cells 72,73,74, not zeroed.
    CHECK(word_at(27,1) > 3060u && word_at(27,3) > 3070u);
    // Last temp packet per segment: third word is padding zero because 17 temps/segment.
    CHECK(word_at(33,1) > 200u && word_at(33,2) > 200u && word_at(33,3) == 0u);
    // Fan packets include 6 real fans then zero padding.
    CHECK(word_at(58,1) == 0u && word_at(58,2) == 100u && word_at(58,3) == 200u);
    CHECK(word_at(59,1) == 300u && word_at(59,2) == 400u && word_at(59,3) == 500u);
    CHECK(word_at(60,1) == 0u && word_at(61,3) == 0u);

    tx_count=0; tx_free_level=0; fake_tick=0;
    CHECK(send_ecu_ams_status(&app.board.canbus, &app) == HAL_TIMEOUT);
}

static void test_can_telemetry_pacing_and_snapshot(void)
{
    static CAN_HandleTypeDef hcan;
    init_fake_app();
    app.board.canbus.hcan = &hcan;
    app.state = STATE_DISCARGE;
    fake_tick = 100u;
    fill_nominal_pack(&app, 3.900f);
    (void)host_publish_measurement_snapshot(
        &app,
        100u,
        12.0f,
        AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
        AMS_MEAS_VALID_CURRENT | AMS_MEAS_BALANCE_RECOVERED);

    ams_measurement_snapshot_t frozen;
    CHECK(ams_measurement_store_copy_latest(&app.measurement_store, &frozen));
    can_measurement_view_t view;
    can_measurement_view_build(&app, &frozen, &view);
    CHECK(view.measurement_sequence == frozen.sequence);

    /* Mutating the live driver state after publication must not change the
     * values serialized for this CAN phase. */
    app.acc.cell_voltage_mv[0][0] = 4100u;

    uint32_t total_frames = 0u;
    for(uint8_t phase = 0u; phase < CAN_TELEMETRY_PHASE_COUNT; phase++)
    {
        tx_count = 0u;
        tx_free_level = 3u;
        CHECK(send_ecu_compact_telemetry(&app.board.canbus,
                                         &app,
                                         &view,
                                         phase) == HAL_OK);
        CHECK(send_ecu_ams_phase(&app.board.canbus,
                                 &app,
                                 &view,
                                 phase) == HAL_OK);
        CHECK(send_logger_phase(&app.board.canbus,
                                &app,
                                &view,
                                phase,
                                7u) == HAL_OK);

        uint32_t expected = (phase == 0u) ? 63u : 36u;
        CHECK(tx_count == expected);
        CHECK(tx_count <= 63u);
        total_frames += tx_count;

        uint32_t meta_index =
            HOST_ECU_COMPACT_FRAME_COUNT +
            ((phase == 0u) ? HOST_ECU_PHASE0_FRAME_COUNT : 11u);
        CHECK(tx_log[meta_index].stdid == AMS_LOGGER_CAN_ID_SNAPSHOT_META);
        CHECK(tx_log[meta_index].data[1] == 7u);
        CHECK(tx_log[meta_index].data[2] == phase);
        CHECK(tx_log[meta_index].data[3] == CAN_TELEMETRY_PHASE_COUNT);
        uint32_t meta_sequence =
            ((uint32_t)tx_log[meta_index].data[4] << 24u) |
            ((uint32_t)tx_log[meta_index].data[5] << 16u) |
            ((uint32_t)tx_log[meta_index].data[6] << 8u) |
            (uint32_t)tx_log[meta_index].data[7];
        CHECK(meta_sequence == frozen.sequence);

        for(uint32_t frame = meta_index + 1u; frame < tx_count; frame++)
        {
            if((tx_log[frame].stdid == AMS_LOGGER_CAN_ID_CELL_DETAIL) ||
               (tx_log[frame].stdid == AMS_LOGGER_CAN_ID_TEMP_DETAIL))
            {
                CHECK(tx_log[frame].data[0] == phase);
            }
        }

        if(phase == 0u)
        {
            /* Compact electrical is frame 1. The phase-0 legacy voltage
             * starts after four compact + three legacy status frames. */
            CHECK(word_at(1u, 0u) == 2925u);
            CHECK(word_at(11u, 1u) == frozen.cell_mv[0][0]);
            CHECK(word_at(11u, 1u) != 4100u);
        }
    }

    CHECK(total_frames == 207u);

    /* Mutate live thermal/fan state after the first compact frame.  The
     * thermal frame later in the same four-frame bundle must still contain
     * the values captured before transmission began. */
    for(uint8_t fan = 0u; fan < NFANS; fan++)
    {
        app.board.fans[fan].duty_cycle = 10.0f;
    }
    app.temp_warning = false;
    app.temp_fan_max = false;
    app.temp_charge_stop = false;
    app.temp_overtemp_pending = false;
    app.overtemp_fault = false;
    app.severe_overtemp_fault = false;
    app.fan_fault = false;
    app.temp_read_fault = false;
    tx_count = 0u;
    fake_can_add_tx_call_count = 0u;
    fake_can_mutate_after_tx_count = 1u;
    CHECK(send_ecu_compact_telemetry(&app.board.canbus,
                                     &app,
                                     &view,
                                     9u) == HAL_OK);
    CHECK(tx_count == HOST_ECU_COMPACT_FRAME_COUNT);
    CHECK(tx_log[2].stdid == AMS_ECU_CAN_ID_THERMAL);
    CHECK(tx_log[2].data[6] == 10u);
    CHECK(tx_log[2].data[7] == 0u);
    CHECK(app.temp_warning == true);
    CHECK(app.board.fans[0].duty_cycle == 100.0f);
    fake_can_mutate_after_tx_count = 0u;
}

static void test_can_priority_metrics_and_deadlines(void)
{
    static CAN_HandleTypeDef hcan;

    /* A failed critical shutdown frame must not be mislabeled as a failed
     * compact bundle when all four compact frames and detail traffic succeed. */
    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    app.board.canbus.hcan = &hcan;
    app.state = STATE_DISCARGE;
    app.board.charger.shutdown_pending = true;
    app.board.charger.shutdown_frames_remaining = 2u;
    app.board.charger.shutdown_request_count = 1u;
    tx_count = 0u;
    tx_free_level = 3u;
    fake_can_fail_on_call = 1u;
    run_one_canbus_task_iteration(&app);
    CHECK(app.can_tx_critical_attempt_count == 1u);
    CHECK(app.can_tx_critical_fail_count == 1u);
    CHECK(app.can_tx_compact_bundle_count == 1u);
    CHECK(app.can_tx_compact_bundle_fail_count == 0u);
    CHECK(app.can_tx_detail_phase_count == 1u);
    CHECK(app.can_tx_detail_phase_fail_count == 0u);
    CHECK(app.can_tx_detail_suppressed_count == 0u);
    CHECK(app.can_task_cycle_count == 1u);
    CHECK(tx_count > HOST_ECU_COMPACT_FRAME_COUNT);

    /* Compact-heartbeat congestion suppresses best-effort detail traffic and
     * is counted independently. */
    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    app.board.canbus.hcan = &hcan;
    app.state = STATE_DISCARGE;
    fake_can_add_tx_status = HAL_ERROR;
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(&app);
    CHECK(app.can_tx_compact_bundle_count == 1u);
    CHECK(app.can_tx_compact_bundle_fail_count == 1u);
    CHECK(app.can_tx_detail_phase_count == 0u);
    CHECK(app.can_tx_detail_suppressed_count == 1u);
    CHECK(app.can_task_cycle_count == 1u);

    /* Successful sends can still overrun the 100 ms task budget.  Advance the
     * fake clock per transmitted frame to prove explicit deadline accounting. */
    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    app.board.canbus.hcan = &hcan;
    app.state = STATE_DISCARGE;
    fake_tick = 500u;
    fake_can_advance_tick_per_tx_ms = 2u;
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(&app);
    CHECK(app.can_task_cycle_count == 1u);
    CHECK(app.can_task_deadline_miss_count == 1u);
    CHECK(app.can_task_last_duration_ms > AMS_CAN_ECU_FAST_PERIOD_MS);
    CHECK(app.can_task_max_duration_ms == app.can_task_last_duration_ms);

    sil_prepare_cli_capture();
    CHECK(get_can_diag(0, NULL) == 0);
    CHECK(strstr(cli_capture, "CAN TX attempts/fail") != NULL);
    CHECK(strstr(cli_capture, "deadline_miss:1") != NULL);
}

static void test_logger_can_contract_packets(void){
    init_fake_app(); static CAN_HandleTypeDef hcan; app.board.canbus.hcan = &hcan;
    app.state = STATE_DISCARGE;
    app.bms_state = true;
    app.air_state = true;
    app.imd_ok = true;
    app.current = -12.3f;
    app.current_valid = true;
    app.current_selected_range = CURRENT_SENSOR_RANGE_50A;
    app.current_meas_reason = CURRENT_SENSOR_REASON_OK;
    app.current_fault_reason = CURRENT_FAULT_REASON_NONE;
    app.current_fault_latched_reason = CURRENT_FAULT_REASON_NONE;
    app.current_fault_mode = CURRENT_FAULT_MODE_DRIVE;
    for(int i=0;i<NFANS;i++) app.board.fans[i].duty_cycle = (float)(i * 15);

    for(int ic=0; ic<NSMBS; ic++){
        for(int c=0;c<NCELLS;c++) app.acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(3.500f + 0.001f * (float)(ic * NCELLS + c));
        for(int s=0;s<NTEMPS;s++) app.acc.smb_ics[ic].temp.raw[s] = raw_for_temp_c(25.0f + (float)s * 0.1f);
    }
    app.acc.smb_ics[4].cell.c_codes[14] = code_for_volts(4.014f);
    app.acc.smb_ics[4].temp.raw[23] = raw_for_temp_c(44.4f);
    app.acc.smb_ics[2].temp.raw[5] = -1;
    host_mark_updated_cells(&app);
    host_mark_updated_temps(&app, (1UL << NTEMPS) - 1UL);
    accumulator_update_voltage_stats_at(&app.acc, fake_tick);
    accumulator_update_temp_stats_at(&app.acc, fake_tick);
    voltage_fault_update(&app.voltage_fault_state, &app.acc);
    app.voltage_valid = app.voltage_fault_state.voltage_valid;
    app.voltage_usable_cell_count = app.voltage_fault_state.usable_cell_count;
    app.voltage_updated_cell_count = app.voltage_fault_state.updated_cell_count;
    app.voltage_stale_cell_count = app.voltage_fault_state.stale_cell_count;
    app.max_voltage_seg = app.voltage_fault_state.max_cell_segment;
    app.max_voltage_cell = app.voltage_fault_state.max_cell_index;
    app.min_voltage_seg = app.voltage_fault_state.min_cell_segment;
    app.min_voltage_cell = app.voltage_fault_state.min_cell_index;
    sil_publish_temp_state(&app);

    app.acc.smb.spi_debug.enabled = true;
    app.acc.smb.spi_debug.last_status = HAL_TIMEOUT;
    app.acc.smb.spi_debug.last_xfer_status = HAL_BUSY;
    app.acc.smb.spi_debug.last_op = ADBMS6830_SPI_OP_READ_STATUS;
    app.acc.smb.spi_debug.error_count = 7u;
    app.acc.smb.spi_debug.cmd_counter_error_count = 3u;
    app.acc.smb.spi_debug.last_read_pec_pass_mask = 0x001Fu;
    app.acc.smb.spi_debug.last_read_pec_fail_mask = 0x0004u;
    app.acc.smb.spi_debug.cmd_counter_mismatch_mask = 0x0008u;
    app.acc.smb.spi_debug.last_cmd[0] = 0x00u;
    app.acc.smb.spi_debug.last_cmd[1] = 0x32u;
    app.acc.apm.spi_debug.enabled = true;
    app.acc.apm.spi_debug.last_status = HAL_ERROR;
    app.acc.apm.spi_debug.last_xfer_status = HAL_TIMEOUT;
    app.acc.apm.spi_debug.last_op = ADBMS2950_SPI_OP_PROBE;
    app.acc.apm.spi_debug.error_count = 2u;
    app.acc.apm.spi_debug.last_read_pec_fail_mask = 0x0001u;
    app.acc.pec_fail_voltage_mask[3] = 0x0101u;

    app.reset_flags = 0xA5A55A5Au;
    app.last_panic_reason = AMS_PANIC_ERROR_HANDLER;
    app.safety_panic_count = 2u;
    app.bms_output_block_count = 9u;
    app.watchdog_runtime_enabled = true;
    app.watchdog_hw_started = true;
    app.watchdog_feed_count = 42u;
    app.watchdog_block_count = 3u;
    app.watchdog_last_block_reason = AMS_WATCHDOG_BLOCK_HARD_FAULT;
    app.watchdog_last_feed_tick = 12000u;
    app.adbms_scan_count = 0x0123u;
    app.adbms_status_diag_count = 4u;
    app.adbms_config_diag_count = 5u;
    app.adbms_open_wire_diag_count = 6u;
    app.adbms_last_diag_status = HAL_BUSY;
    app.adbms_diag_fault = true;
    app.adbms_config_fault = true;
    app.adbms_balance_write_fault = true;
    app.adbms_balance_write_fail_count = 7u;
    app.adbms_scan_active = true;
    app.hil.meas.fresh = 1u;
    app.hil.truth.fresh = 1u;
    app.hil.summary.fresh = 1u;
    app.board.current_sensor.count_high = 0x0ABCu;
    app.board.current_sensor.count_low = 0x0123u;
    app.board.current_sensor.count_high_fresh = true;
    app.board.current_sensor.count_low_fresh = true;
    app.board.current_sensor.last_read_ok = true;
    app.board.current_sensor.current_valid = true;
    app.current_selected_range = CURRENT_SENSOR_RANGE_800A;
    app.current_meas_reason = CURRENT_SENSOR_REASON_SENSOR_SATURATION;
    app.board.charger.read_current = -4.2f;
    app.board.charger.disable_reason_mask = CHARGER_DISABLE_REASON_TX_FAIL;
    app.board.charger.last_tx_status = HAL_TIMEOUT;
    app.board.charger.tx_count = 9u;
    app.board.charger.rx_count = 10u;
    app.board.charger.tx_fail_count = 2u;

    tx_count=0; tx_free_level=3; fake_tick=12345u;
    CHECK(send_logger_telemetry(&app.board.canbus, &app) == HAL_OK);
    CHECK(tx_count == HOST_LOGGER_FRAME_COUNT);

    CHECK(tx_log[0].ide == CAN_ID_STD && tx_log[0].stdid == AMS_LOGGER_CAN_ID_HEARTBEAT);
    CHECK(tx_log[0].data[0] == AMS_LOGGER_PROTOCOL_VERSION);
    CHECK(tx_log[0].data[2] == STATE_DISCARGE);
    CHECK((tx_log[0].data[3] & (1u << AMS_LOGGER_HEARTBEAT_FLAG_BMS_OK)) != 0u);
    CHECK((tx_log[0].data[4] & (1u << AMS_LOGGER_VALID_FLAG_CURRENT_VALID)) != 0u);
    CHECK(word_at(0,3) == 12u);

    CHECK(tx_log[1].stdid == AMS_LOGGER_CAN_ID_FAULT_REASONS);
    CHECK(tx_log[2].stdid == AMS_LOGGER_CAN_ID_PACK_ELECTRICAL);
    CHECK((int16_t)word_at(2,1) == -123);
    CHECK(tx_log[3].stdid == AMS_LOGGER_CAN_ID_TEMP_FAN);
    CHECK(tx_log[3].data[6] == 75u);
    CHECK(tx_log[8].stdid == AMS_LOGGER_CAN_ID_6830_LINK);
    CHECK(tx_log[8].data[0] == HAL_TIMEOUT);
    CHECK(tx_log[8].data[1] == HAL_BUSY);
    CHECK(word_at(8,2) == 0x0004u);
    CHECK(word_at(8,3) == 0x0008u);
    CHECK(tx_log[10].stdid == AMS_LOGGER_CAN_ID_2950_LINK);
    CHECK(tx_log[10].data[0] == HAL_ERROR);
    CHECK(tx_log[10].data[1] == HAL_TIMEOUT);
    CHECK(word_at(10,2) == 0x0001u);
    CHECK(tx_log[11].stdid == AMS_LOGGER_CAN_ID_TASK_HEALTH);
    CHECK(word_at(11,1) == app.heartbeat.stale_mask);
    CHECK(word_at(11,2) == app.heartbeat.seen_mask);
    CHECK(tx_log[12].stdid == AMS_LOGGER_CAN_ID_CAN_DIAG);
    CHECK(((uint32_t)tx_log[12].data[0] << 24 | (uint32_t)tx_log[12].data[1] << 16 | (uint32_t)tx_log[12].data[2] << 8 | tx_log[12].data[3]) == app.can_error_code);

    CHECK(tx_log[13].stdid == AMS_LOGGER_CAN_ID_SAFETY_DIAG);
    CHECK(((uint32_t)tx_log[13].data[0] << 24 | (uint32_t)tx_log[13].data[1] << 16 | (uint32_t)tx_log[13].data[2] << 8 | tx_log[13].data[3]) == app.reset_flags);
    CHECK(tx_log[13].data[4] == AMS_PANIC_ERROR_HANDLER);
    CHECK(tx_log[13].data[5] == 2u);
    CHECK(tx_log[13].data[6] == 9u);
    CHECK((tx_log[13].data[7] & (1u << AMS_LOGGER_SAFETY_FLAG_BMS_STATE)) != 0u);

    CHECK(tx_log[14].stdid == AMS_LOGGER_CAN_ID_WATCHDOG_DIAG);
    CHECK((tx_log[14].data[0] & (1u << AMS_LOGGER_WATCHDOG_FLAG_RUNTIME_ENABLED)) != 0u);
    CHECK((tx_log[14].data[0] & (1u << AMS_LOGGER_WATCHDOG_FLAG_HW_STARTED)) != 0u);
    CHECK(tx_log[14].data[1] == AMS_WATCHDOG_BLOCK_HARD_FAULT);
    CHECK(word_at(14,1) == 42u);
    CHECK(word_at(14,2) == 3u);

    CHECK(tx_log[15].stdid == AMS_LOGGER_CAN_ID_ADBMS_DIAG);
    CHECK(word_at(15,0) == 0x0123u);
    CHECK(tx_log[15].data[2] == 4u);
    CHECK(tx_log[15].data[3] == 5u);
    CHECK(tx_log[15].data[4] == 6u);
    CHECK(tx_log[15].data[5] == HAL_BUSY);
    CHECK((tx_log[15].data[6] & (1u << AMS_LOGGER_ADBMS_DIAG_FLAG_DIAG_FAULT)) != 0u);
    CHECK((tx_log[15].data[6] & (1u << AMS_LOGGER_ADBMS_DIAG_FLAG_CONFIG_FAULT)) != 0u);
    CHECK((tx_log[15].data[6] & (1u << AMS_LOGGER_ADBMS_DIAG_FLAG_SCAN_ACTIVE)) != 0u);
    CHECK((tx_log[15].data[6] & (1u << AMS_LOGGER_ADBMS_DIAG_FLAG_BALANCE_WRITE_FAULT)) != 0u);
    CHECK(tx_log[15].data[7] == 0x07u);

    CHECK(tx_log[16].stdid == AMS_LOGGER_CAN_ID_CURRENT_ADC);
    CHECK(word_at(16,0) == 0x0ABCu);
    CHECK(word_at(16,1) == 0x0123u);
    CHECK(tx_log[16].data[4] == CURRENT_SENSOR_RANGE_800A);
    CHECK(tx_log[16].data[5] == CURRENT_SENSOR_REASON_SENSOR_SATURATION);
    CHECK((tx_log[16].data[6] & (1u << AMS_LOGGER_CURRENT_ADC_FLAG_HIGH_FRESH)) != 0u);
    CHECK((tx_log[16].data[6] & (1u << AMS_LOGGER_CURRENT_ADC_FLAG_LOW_FRESH)) != 0u);
    CHECK((tx_log[16].data[6] & (1u << AMS_LOGGER_CURRENT_ADC_FLAG_LAST_READ_OK)) != 0u);
    CHECK((tx_log[16].data[6] & (1u << AMS_LOGGER_CURRENT_ADC_FLAG_CURRENT_VALID)) != 0u);

    CHECK(tx_log[17].stdid == AMS_LOGGER_CAN_ID_CHARGER_DETAIL);
    CHECK((int16_t)word_at(17,0) == -42);
    CHECK(word_at(17,1) == CHARGER_DISABLE_REASON_TX_FAIL);
    CHECK(tx_log[17].data[4] == HAL_TIMEOUT);
    CHECK(tx_log[17].data[5] == 9u);
    CHECK(tx_log[17].data[6] == 10u);
    CHECK(tx_log[17].data[7] == 2u);

    CHECK(tx_log[18].stdid == AMS_LOGGER_CAN_ID_RTOS_DIAG);
    CHECK(word_at(18,0) == (app.rtos_heap_free_bytes / 16u));
    CHECK(word_at(18,1) == (app.rtos_heap_min_ever_free_bytes / 16u));
    CHECK(word_at(18,2) == app.rtos_stack_warn_mask);

    uint32_t detail_base = 19u;
    uint32_t last_cell_frame = detail_base + (4u * 5u) + 4u;
    CHECK(tx_log[last_cell_frame].stdid == AMS_LOGGER_CAN_ID_CELL_DETAIL);
    CHECK(tx_log[last_cell_frame].data[0] == 4u);
    CHECK(tx_log[last_cell_frame].data[1] == 12u);
    CHECK(word_at(last_cell_frame,3) == 4014u);

    uint32_t invalid_temp_frame = detail_base + 25u + (2u * 8u) + 1u;
    CHECK(tx_log[invalid_temp_frame].stdid == AMS_LOGGER_CAN_ID_TEMP_DETAIL);
    CHECK(tx_log[invalid_temp_frame].data[0] == 2u);
    CHECK(tx_log[invalid_temp_frame].data[1] == 3u);
    CHECK(word_at(invalid_temp_frame,3) == AMS_LOGGER_TEMP_INVALID_DECI_C);

    uint32_t last_temp_frame = detail_base + 25u + (4u * 8u) + 7u;
    CHECK(tx_log[last_temp_frame].stdid == AMS_LOGGER_CAN_ID_TEMP_DETAIL);
    CHECK(tx_log[last_temp_frame].data[0] == 4u);
    CHECK(tx_log[last_temp_frame].data[1] == 21u);
    CHECK((int16_t)word_at(last_temp_frame,3) == 444);

    uint32_t voltage_pec_seg3 = detail_base + 25u + 40u + (3u * 4u) + 1u;
    CHECK(tx_log[voltage_pec_seg3].stdid == AMS_LOGGER_CAN_ID_VOLTAGE_PEC);
    CHECK(tx_log[voltage_pec_seg3].data[0] == 3u);
    CHECK(((uint16_t)tx_log[voltage_pec_seg3].data[1] << 8 | tx_log[voltage_pec_seg3].data[2]) == 0x0101u);
    CHECK(tx_log[voltage_pec_seg3].data[3] == 2u);

    uint32_t temp_mask_b_seg2 = detail_base + 25u + 40u + (2u * 4u) + 3u;
    CHECK(tx_log[temp_mask_b_seg2].stdid == AMS_LOGGER_CAN_ID_TEMP_MASKS_B);
    CHECK(tx_log[temp_mask_b_seg2].data[0] == 2u);
    CHECK(tx_log[temp_mask_b_seg2].data[4] == 0u);
    CHECK(tx_log[temp_mask_b_seg2].data[5] == 0u);
    CHECK(tx_log[temp_mask_b_seg2].data[6] == (1u << 5));
}

static void test_charger_rx_and_tx(void){
    init_fake_app(); static CAN_HandleTypeDef hcan; app.board.canbus.hcan=&hcan; charger_init(&app.board.charger, &app.board.canbus);
    memset(&fake_rx_hdr,0,sizeof(fake_rx_hdr)); fake_rx_hdr.IDE=CAN_ID_EXT; fake_rx_hdr.ExtId=CHARGER_RX_ID; fake_rx_hdr.DLC=5; fake_rx_data[0]=0x0C; fake_rx_data[1]=0x34; fake_rx_data[2]=0x00; fake_rx_data[3]=0x2A; fake_rx_data[4]=0x0B; fake_tick=1234;
    (void)host_receive_can_frame(&hcan);
    CHECK(fabsf(app.board.charger.read_voltage - 312.4f) < 0.01f);
    CHECK(fabsf(app.board.charger.read_current - 4.2f) < 0.01f);
    CHECK(app.board.charger.hardware_fail == true);
    CHECK(app.board.charger.overtemp_fail == true);
    CHECK(app.board.charger.input_volt_fail == false);
    CHECK(app.board.charger.voltage_sense_fail == true);
    CHECK(app.board.charger.rx_count == 1u);

    app.state=STATE_CHARGE; app.current_valid=true; app.hard_fault=false; app.voltage_fault=false; app.temp_fault=false; app.bms_state=true; app.board.charger.hardware_fail=false; app.board.charger.overtemp_fail=false; app.board.charger.input_volt_fail=false; app.board.charger.voltage_sense_fail=false; app.board.charger.communication_fail=false; app.board.charger.last_rx_tick=fake_tick;
    tx_count=0; tx_free_level=3; CHECK(canbus_send(&app.board.canbus, CAN_ID_EXT, CCS_CANBUS_ID, (uint8_t[8]){0x0C,0x30,0,0x64,0,0,0,0}) == HAL_OK);
    CHECK(tx_count == 1u && tx_log[0].ide == CAN_ID_EXT && tx_log[0].extid == CCS_CANBUS_ID);
}


static void run_one_canbus_task_iteration(app_data_t *d){
    if(setjmp(task_exit_jmp) == 0){ task_exit_after_delay_until = 1; canbus_task_fn(d); }
}
static void run_one_adbms_task_iteration(app_data_t *d){
    if(setjmp(task_exit_jmp) == 0){ task_exit_after_delay_until = 1; adbms_task_fn(d); }
}
static void run_one_error_task_iteration(app_data_t *d){
    if(setjmp(task_exit_jmp) == 0){ task_exit_after_delay_until = 1; error_task_fn(d); }
}
static void run_one_fan_task_iteration(app_data_t *d){
    if(setjmp(task_exit_jmp) == 0){ task_exit_after_delay_until = 1; fan_task_fn(d); }
}
static void run_one_current_task_iteration(app_data_t *d){
    if(setjmp(task_exit_jmp) == 0){ task_exit_after_delay_until = 1; current_task_fn(d); }
}
static void run_one_estimator_task_iteration(app_data_t *d){
    if(setjmp(task_exit_jmp) == 0){ task_exit_after_delay_until = 1; estimator_task_fn(d); }
}

static void fill_nominal_pack(app_data_t *d, float base_v){
    d->acc.smb.num_ics = NSMBS; d->acc.smb.ics = d->acc.smb_ics;
    for(int ic=0; ic<NSMBS; ic++){
        for(int c=0;c<NCELLS;c++) d->acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(base_v);
        d->acc.smb.last_cell_updated_mask[ic] = 0x7FFFu;
        d->acc.smb.last_cell_pec_mask[ic] = 0u;
    }
    for(int ic=0; ic<NSMBS; ic++) for(int s=0;s<NTEMPS;s++) d->acc.smb_ics[ic].temp.raw[s] = raw_for_ntc_voltage(2.5f);
    accumulator_update_voltage_stats_at(&d->acc, fake_tick);
    voltage_fault_update(&d->voltage_fault_state, &d->acc);
    d->voltage_valid = d->voltage_fault_state.voltage_valid;
    d->voltage_fault = d->voltage_fault_state.read_fault || d->voltage_fault_state.overvoltage_fault || d->voltage_fault_state.undervoltage_fault || d->voltage_fault_state.latched;
    host_mark_updated_temps(d, (1UL << NTEMPS) - 1UL);
    accumulator_update_temp_stats_at(&d->acc, fake_tick);
    d->max_temp = d->acc.max_temp;
    d->avg_temp = d->acc.avg_temp;
    sil_publish_temp_state(d);
}

static uint32_t host_publish_measurement_snapshot(app_data_t *d,
                                                  uint32_t voltage_tick,
                                                  float current_A,
                                                  uint32_t validity_flags)
{
    CHECK(d != NULL);

    if((validity_flags & AMS_MEAS_VALID_CURRENT) != 0u)
    {
        /* Synthetic SIL current belongs to one exact epoch. */
        validity_flags |= AMS_MEAS_CURRENT_TIMING_VALID;
    }

    ams_measurement_snapshot_t previous;
    bool have_previous =
        ams_measurement_store_copy_latest(&d->measurement_store, &previous);
    uint32_t interval_ms = have_previous ?
        (uint32_t)(voltage_tick - previous.voltage_complete_tick) : 100u;

    ams_current_window_t current = {0};
    current.start_tick = voltage_tick - interval_ms;
    current.end_tick = voltage_tick;
    current.latest_sample_tick = voltage_tick;
    current.sample_count =
        ((validity_flags & AMS_MEAS_VALID_CURRENT) != 0u) ? 5u : 0u;
    current.invalid_sample_count =
        ((validity_flags & AMS_MEAS_VALID_CURRENT) != 0u) ? 0u : 1u;
    current.latest_A = current_A;
    current.filtered_A = current_A;
    current.average_A = current_A;
    current.rms_A = fabsf(current_A);
    current.min_A = current_A;
    current.max_A = current_A;
    current.charge_As =
        ((validity_flags & AMS_MEAS_VALID_CURRENT) != 0u) ?
        ((double)current_A * ((double)interval_ms / 1000.0)) : 0.0;
    current.absolute_charge_As = fabs(current.charge_As);
    current.total_charge_As =
        (have_previous ? previous.current.total_charge_As : 0.0) +
        current.charge_As;
    current.total_absolute_charge_As =
        (have_previous ? previous.current.total_absolute_charge_As : 0.0) +
        current.absolute_charge_As;
    current.total_invalid_sample_count =
        (have_previous ? previous.current.total_invalid_sample_count : 0u) +
        current.invalid_sample_count;
    current.valid = ((validity_flags & AMS_MEAS_VALID_CURRENT) != 0u);

    uint16_t balance_masks[NSMBS] = {0};
    ams_measurement_snapshot_t *snapshot =
        ams_measurement_store_begin_write(&d->measurement_store);
    CHECK(snapshot != NULL);
    ams_measurement_snapshot_prepare(snapshot,
                                     &d->acc,
                                     &current,
                                     voltage_tick - interval_ms,
                                     voltage_tick,
                                     voltage_tick,
                                     balance_masks,
                                     0u,
                                     validity_flags);
    uint32_t sequence =
        ams_measurement_store_publish(&d->measurement_store, snapshot);
    CHECK(sequence != 0u);
    return sequence;
}

static ADC_HandleTypeDef sil_adc_high;
static ADC_HandleTypeDef sil_adc_low;
static TIM_HandleTypeDef sil_fan_htim;
static uint32_t sil_fan_ccr[NFANS];

static void sil_attach_current_adcs(app_data_t *d)
{
    CHECK(d != NULL);
    d->board.current_sensor.hadc_high = &sil_adc_high;
    d->board.current_sensor.hadc_low = &sil_adc_low;
}

static void sil_attach_fans(app_data_t *d)
{
    CHECK(d != NULL);
    for(uint8_t i = 0u; i < NFANS; i++)
    {
        d->board.fans[i].CCR = &sil_fan_ccr[i];
        d->board.fans[i].max_timer_val = 1000u;
        d->board.fans[i].htim = &sil_fan_htim;
		d->board.fans[i].initialized = true;
		d->board.fans[i].init_status = HAL_OK;
    }
}

static void sil_set_cell_voltage(app_data_t *d, uint8_t seg, uint8_t cell, float volts)
{
    CHECK(d != NULL);
    CHECK(seg < NSMBS);
    CHECK(cell < NCELLS);
    d->acc.smb_ics[seg].cell.c_codes[cell] = code_for_volts(volts);
}

static void sil_clear_voltage_history(app_data_t *d)
{
    CHECK(d != NULL);
    d->acc.valid_voltage_count = 0u;
    d->acc.updated_voltage_count = 0u;
    d->acc.usable_voltage_count = 0u;
    d->acc.stale_voltage_count = 0u;
    d->acc.pec_fail_cell_count = 0u;
    d->acc.voltage_full_updated = false;
    d->acc.voltage_full_usable = false;
    d->acc.voltage_startup_scan_complete = false;
    memset(d->acc.cell_voltage_mv, 0, sizeof(d->acc.cell_voltage_mv));
    memset(d->acc.cell_voltage_valid, 0, sizeof(d->acc.cell_voltage_valid));
    memset(d->acc.cell_voltage_last_update_ms, 0, sizeof(d->acc.cell_voltage_last_update_ms));
    memset(d->acc.cell_voltage_consecutive_misses, 0, sizeof(d->acc.cell_voltage_consecutive_misses));
    memset(d->acc.updated_voltage_mask, 0, sizeof(d->acc.updated_voltage_mask));
    memset(d->acc.usable_voltage_mask, 0, sizeof(d->acc.usable_voltage_mask));
    memset(d->acc.pec_fail_voltage_mask, 0, sizeof(d->acc.pec_fail_voltage_mask));
    memset(d->acc.stale_voltage_mask, 0, sizeof(d->acc.stale_voltage_mask));
}

static void sil_expect_balancing_clear(const app_data_t *d)
{
    CHECK(d != NULL);
    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        CHECK(d->acc.smb_ics[ic].tx_cfgb.dcc == 0u);
        for(uint8_t cell = 0u; cell < PWMA; cell++)
        {
            CHECK(d->acc.smb_ics[ic].PwmA.pwma[cell] == 0u);
        }
        for(uint8_t cell = 0u; cell < PWMB; cell++)
        {
            CHECK(d->acc.smb_ics[ic].PwmB.pwmb[cell] == 0u);
        }
    }
}

static uint8_t sil_balance_pwm_duty(const app_data_t *d, uint8_t ic, uint8_t cell)
{
    CHECK(d != NULL);
    CHECK(ic < NSMBS);
    CHECK(cell < CELL);

    if(cell < PWMA)
    {
        return d->acc.smb_ics[ic].PwmA.pwma[cell];
    }
    return d->acc.smb_ics[ic].PwmB.pwmb[cell - PWMA];
}

static void sil_run_current_sample(app_data_t *d, float current_a)
{
    CHECK(d != NULL);
    fake_adc_set_current_a(current_a);
    run_one_current_task_iteration(d);
}

static void sil_run_current_adc_status(app_data_t *d, HAL_StatusTypeDef high_status, HAL_StatusTypeDef low_status)
{
    CHECK(d != NULL);
    fake_adc_set_status_sequence(high_status, low_status);
    run_one_current_task_iteration(d);
}

static void sil_run_voltage_sample(app_data_t *d)
{
    CHECK(d != NULL);
    run_one_adbms_task_iteration(d);
    sil_mark_all_heartbeats_alive(d);
    error_task_update(d, fake_tick);
}

static void sil_copy_temp_state(app_data_t *d)
{
    CHECK(d != NULL);
    d->max_temp = d->acc.max_temp;
    d->avg_temp = d->acc.avg_temp;
    d->temp_valid = d->temp_fault_state.temp_valid;
    d->temp_read_fault = d->temp_fault_state.read_fault;
    d->temp_warning = d->temp_fault_state.warning;
    d->temp_fan_max = d->temp_fault_state.fan_max;
    d->temp_charge_stop = d->temp_fault_state.charge_stop;
    d->temp_overtemp_pending = d->temp_fault_state.pending;
    d->overtemp_fault = d->temp_fault_state.overtemp_fault;
    d->severe_overtemp_fault = d->temp_fault_state.severe_overtemp_fault;
    d->temp_fault_latched = d->temp_fault_state.latched;
    d->temp_fault_reason = d->temp_fault_state.reason;
    d->temp_fault_pending_reason = d->temp_fault_state.pending_reason;
    d->temp_fault_latched_reason = d->temp_fault_state.latched_reason;
    d->temp_fault_pending_ms = d->temp_fault_state.pending_ms;
    d->temp_usable_sensor_count = d->temp_fault_state.usable_sensor_count;
    d->temp_updated_sensor_count = d->temp_fault_state.updated_sensor_count;
    d->temp_stale_sensor_count = d->temp_fault_state.stale_sensor_count;
    d->temp_invalid_sensor_count = d->temp_fault_state.invalid_sensor_count;
    d->temp_open_sensor_count = d->temp_fault_state.open_sensor_count;
    d->temp_short_sensor_count = d->temp_fault_state.short_sensor_count;
    d->temp_jump_sensor_count = d->temp_fault_state.jump_sensor_count;
    d->temp_rate_rise_sensor_count = d->temp_fault_state.rate_rise_sensor_count;
    d->temp_filtered_max = (float)d->temp_fault_state.filtered_max_temp_deci_c / 10.0f;
    d->temp_filtered_avg = (float)d->temp_fault_state.filtered_avg_temp_deci_c / 10.0f;
    d->temp_max_rate_c_per_s = (float)d->temp_fault_state.max_rate_deci_c_per_s / 10.0f;
    d->temp_max_rate_seg = d->temp_fault_state.max_rate_segment;
    d->temp_max_rate_sensor = d->temp_fault_state.max_rate_sensor;
    d->max_temp_seg = d->temp_fault_state.max_temp_segment;
    d->max_temp_sensor = d->temp_fault_state.max_temp_sensor;
    d->min_temp_seg = d->temp_fault_state.min_temp_segment;
    d->min_temp_sensor = d->temp_fault_state.min_temp_sensor;
    d->temp_fault = d->temp_read_fault || d->overtemp_fault || d->temp_fault_latched;
}

static void sil_publish_temp_state(app_data_t *d)
{
    CHECK(d != NULL);
    temperature_fault_update(&d->temp_fault_state, &d->acc);
    sil_copy_temp_state(d);
}

static void sil_set_all_temps(app_data_t *d, float temp_c, uint32_t updated_mask)
{
    CHECK(d != NULL);
    int16_t raw = raw_for_temp_c(temp_c);
    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        for(uint8_t s = 0u; s < NTEMPS; s++)
        {
            d->acc.smb_ics[ic].temp.raw[s] = raw;
        }
    }
    host_mark_updated_temps(d, updated_mask);
    accumulator_update_temp_stats_at(&d->acc, fake_tick);
    d->max_temp = d->acc.max_temp;
    d->avg_temp = d->acc.avg_temp;
    sil_publish_temp_state(d);
}

static void sil_clear_temp_history(app_data_t *d)
{
    CHECK(d != NULL);
    d->acc.valid_temp_count = 0u;
    d->acc.updated_temp_count = 0u;
    d->acc.usable_temp_count = 0u;
    d->acc.stale_temp_count = 0u;
    d->acc.invalid_temp_count = 0u;
    d->acc.temp_full_updated = false;
    d->acc.temp_full_usable = false;
    d->acc.temp_startup_scan_complete = false;
    memset(d->acc.temp_deci_c, 0, sizeof(d->acc.temp_deci_c));
    memset(d->acc.temp_sensor_valid, 0, sizeof(d->acc.temp_sensor_valid));
    memset(d->acc.temp_last_update_ms, 0, sizeof(d->acc.temp_last_update_ms));
    memset(d->acc.temp_consecutive_misses, 0, sizeof(d->acc.temp_consecutive_misses));
    memset(d->acc.updated_temp_mask, 0, sizeof(d->acc.updated_temp_mask));
    memset(d->acc.usable_temp_mask, 0, sizeof(d->acc.usable_temp_mask));
    memset(d->acc.stale_temp_mask, 0, sizeof(d->acc.stale_temp_mask));
    memset(d->acc.invalid_temp_mask, 0, sizeof(d->acc.invalid_temp_mask));
    memset(d->acc.smb.last_temp_updated_mask, 0, sizeof(d->acc.smb.last_temp_updated_mask));
    temperature_fault_init(&d->temp_fault_state);
    sil_publish_temp_state(d);
}

static void sil_mark_all_heartbeats_alive(app_data_t *d)
{
    CHECK(d != NULL);
    for(uint8_t i = 0u; i < (uint8_t)AMS_HEARTBEAT_COUNT; i++)
    {
        ams_heartbeat_kick(d, (ams_heartbeat_id_t)i, fake_tick);
    }
    (void)ams_heartbeat_update(d, fake_tick);
}

static void sil_prepare_ready_system(state_t state, float current_a, float cell_v)
{
    init_fake_app();
    fake_tick = 0u;
    bms_pin_state = GPIO_PIN_RESET;
    app.state = state;
    sil_attach_current_adcs(&app);
    fill_nominal_pack(&app, cell_v);
    sil_set_all_temps(&app, 25.0f, (1UL << NTEMPS) - 1UL);
    fake_adbms_voltage_masks_full_update();
    sil_run_current_sample(&app, current_a);
    CHECK(app.current_valid == true);
    CHECK(app.current_fault == false);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.voltage_fault == false);
    CHECK(app.temp_fault == false);
    sil_mark_all_heartbeats_alive(&app);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);
}

static void test_task_iterations_with_injected_signals(void){
    static CAN_HandleTypeDef hcan;
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan = &hcan; app.state = STATE_DISCARGE;
    tx_count=0; tx_free_level=3; fake_tick=0;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == HOST_NONCHARGE_CAN_FRAME_COUNT); CHECK(app.canbus_fault == false); CHECK(app.board.charger.target_voltage == 0.0f);

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan = &hcan; app.state = STATE_CHARGE; app.current_valid=true; app.bms_state=true; app.board.charger.last_rx_tick=1000; fake_tick=1000; tx_count=0; tx_free_level=3;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT); CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].ide == CAN_ID_EXT); CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].extid == CCS_CANBUS_ID); CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 0u); CHECK(app.canbus_fault == false); CHECK(app.charger_fault == false);

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan = &hcan; app.state = STATE_CHARGE; app.bms_state=true; app.board.charger.last_rx_tick=1; fake_tick=6005; tx_count=0; tx_free_level=3; bms_pin_state = GPIO_PIN_SET;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT); CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 1u); CHECK(app.board.charger.communication_fail == true); CHECK(app.charger_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET);

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state = STATE_CHARGE; app.current_valid=true; app.bms_state=true; app.balance_inhibit=false; app.acc.smb_ics[0].cell.c_codes[0] = code_for_volts(4.100f); app.acc.smb_ics[0].cell.c_codes[1] = code_for_volts(4.140f); fake_tick=0;
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == false); CHECK(app.temp_fault == false); CHECK(app.max_voltage > 4.13f); CHECK(app.min_voltage > 3.69f); CHECK(app.acc.smb_ics[0].tx_cfgb.dcc == 0u); CHECK(sil_balance_pwm_duty(&app, 0u, 1u) == BALANCE_PWM_DUTY);

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state = STATE_DISCARGE; sil_attach_current_adcs(&app); sil_run_current_sample(&app, 0.0f); app.bms_state=false; bms_pin_state=GPIO_PIN_RESET; fake_tick=0;
    run_one_adbms_task_iteration(&app);
    sil_mark_all_heartbeats_alive(&app);
    error_task_update(&app, fake_tick);
    CHECK(app.voltage_fault == false); CHECK(app.temp_fault == false); CHECK(app.bms_state == true); CHECK(bms_pin_state == GPIO_PIN_SET);

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state = STATE_DISCARGE; app.current_fault=true; app.bms_state=false; bms_pin_state=GPIO_PIN_RESET; fake_tick=0;
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == false); CHECK(app.temp_fault == false); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET);

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state = STATE_CHARGE; app.bms_state=true; app.acc.smb_ics[2].cell.c_codes[3] = code_for_volts(2.400f); fake_tick=0; bms_pin_state=GPIO_PIN_SET;
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET); sil_expect_balancing_clear(&app);

    init_fake_app(); app.fuse_fault = true; app.temp_fault=false; app.voltage_fault=false; app.charger_fault=false; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
    run_one_error_task_iteration(&app);
    CHECK(app.hard_fault == true); CHECK(app.bms_state == false);

    init_fake_app(); for(int i=0;i<NFANS;i++){ static TIM_HandleTypeDef ht; static uint32_t ccrs[NFANS]; app.board.fans[i].CCR=&ccrs[i]; app.board.fans[i].max_timer_val=1000; app.board.fans[i].htim=&ht; app.board.fans[i].initialized=true; app.board.fans[i].init_status=HAL_OK; }
    app.max_temp = 0.0f; app.temp_valid = false; app.temp_usable_sensor_count = 0u; run_one_fan_task_iteration(&app); CHECK(app.fan_state == true); CHECK((app.heartbeat.seen_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_FAN)) != 0u); for(int i=0;i<NFANS;i++) CHECK(app.board.fans[i].duty_cycle == 100.0f);
    app.temp_valid = true; app.temp_read_fault = false; app.temp_fault = false; app.temp_fan_max = true; app.temp_usable_sensor_count = AMS_EXPECTED_TEMP_SENSOR_COUNT;
    app.max_temp = TEMP_FAN_MAX_C + 1.0f; run_one_fan_task_iteration(&app); CHECK(app.fan_state == true); for(int i=0;i<NFANS;i++) CHECK(app.board.fans[i].duty_cycle == 100.0f);
    app.temp_fan_max = false; app.max_temp = TEMP_FAN_RAMP_START_C - 1.0f; run_one_fan_task_iteration(&app); CHECK(app.fan_state == false); for(int i=0;i<NFANS;i++) CHECK(app.board.fans[i].duty_cycle == 0.0f);

    static ADC_HandleTypeDef adc1, adc2; init_fake_app(); app.board.current_sensor.hadc_high=&adc1; app.board.current_sensor.hadc_low=&adc2; fake_adc_set_two_read_sequence(adc_count_for_sensor_voltage(2.5f), adc_count_for_sensor_voltage(2.5f)); run_one_current_task_iteration(&app); CHECK(app.current_fault == false); CHECK(app.current_valid == true);
    init_fake_app(); app.board.current_sensor.hadc_high=NULL; app.board.current_sensor.hadc_low=&adc2; app.current = 12.3f; app.board.current_sensor.current = 45.6f; run_one_current_task_iteration(&app); CHECK(app.current_fault == false); CHECK(app.current_sensor_fault == false); CHECK(fabsf(app.current - 12.3f) < 0.001f);
}

static uint16_t adc_count_for_mcu_voltage(float v)
{
    return (uint16_t)((v * 4095.0f / 3.3f) + 0.5f);
}

static uint16_t adc_count_for_sensor_voltage(float v)
{
    return adc_count_for_mcu_voltage(v * 0.6f);
}

static void fake_adc_set_two_read_sequence(uint16_t high_count, uint16_t low_count)
{
    fake_adc_read_counts[0] = high_count;
    fake_adc_read_counts[1] = low_count;
    fake_adc_read_statuses[0] = HAL_OK;
    fake_adc_read_statuses[1] = HAL_OK;
    fake_adc_read_index = 0u;
}

static void fake_adc_set_status_sequence(HAL_StatusTypeDef high_status, HAL_StatusTypeDef low_status)
{
    fake_adc_read_statuses[0] = high_status;
    fake_adc_read_statuses[1] = low_status;
    fake_adc_read_counts[0] = 0u;
    fake_adc_read_counts[1] = 0u;
    fake_adc_read_index = 0u;
}

static void fake_adbms_voltage_masks_full_update(void)
{
    fake_adbms_use_custom_voltage_masks = false;
    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        fake_adbms_updated_masks[ic] = 0x7FFFu;
        fake_adbms_pec_masks[ic] = 0u;
    }
}

static void fake_adbms_voltage_masks_all_missing(bool pec_fail)
{
    fake_adbms_use_custom_voltage_masks = true;
    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        fake_adbms_updated_masks[ic] = 0u;
        fake_adbms_pec_masks[ic] = pec_fail ? 0x7FFFu : 0u;
    }
}

static void fake_adbms_voltage_masks_one_missing(uint8_t seg, uint8_t cell, bool pec_fail)
{
    fake_adbms_use_custom_voltage_masks = true;
    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        fake_adbms_updated_masks[ic] = 0x7FFFu;
        fake_adbms_pec_masks[ic] = 0u;
    }

    if((seg < ADBMS6830_MAX_TRACKED_ICS) && (cell < NCELLS))
    {
        uint16_t bit = (uint16_t)(1u << cell);
        fake_adbms_updated_masks[seg] = (uint16_t)(fake_adbms_updated_masks[seg] & (uint16_t)~bit);
        if(pec_fail)
        {
            fake_adbms_pec_masks[seg] = (uint16_t)(fake_adbms_pec_masks[seg] | bit);
        }
    }
}

static void fake_adc_set_current_a(float current_a)
{
    float sensor_high = 2.5f + (current_a * 0.0025f);
    float sensor_low = 2.5f + (current_a * 0.040f);

    if(sensor_low > 4.75f)
    {
        sensor_low = 4.75f;
    }
    else if(sensor_low < 0.25f)
    {
        sensor_low = 0.25f;
    }

    fake_adc_set_two_read_sequence(adc_count_for_sensor_voltage(sensor_high),
                                   adc_count_for_sensor_voltage(sensor_low));
}

static void host_current_sensor_mark_fresh(current_sensor_t *sensor)
{
    sensor->last_read_ok = true;
    sensor->count_high_fresh = true;
    sensor->count_low_fresh = true;
}

static void test_current_sensor_measurement_model(void)
{
    current_sensor_t cs = {0};
    float current;

    cs.count_high = adc_count_for_sensor_voltage(2.5f);
    cs.count_low = adc_count_for_sensor_voltage(2.5f);
    host_current_sensor_mark_fresh(&cs);
    current = current_sensor_convert(&cs);
    CHECK(cs.current_valid == true);
    CHECK(cs.reason == CURRENT_SENSOR_REASON_OK);
    CHECK(cs.selected_range == CURRENT_SENSOR_RANGE_50A);
    CHECK(fabsf(current) < 0.05f);
    CHECK(fabsf(cs.sensor_voltage_high - 2.5f) < 0.01f);
    CHECK(fabsf(cs.sensor_voltage_low - 2.5f) < 0.01f);

    cs = (current_sensor_t){0};
    cs.count_high = adc_count_for_sensor_voltage(2.55f);  /* +20 A on 2.5 mV/A / C_SENSE_H */
    cs.count_low = adc_count_for_sensor_voltage(3.3f);    /* +20 A on 40 mV/A / C_SENSE_L */
    host_current_sensor_mark_fresh(&cs);
    current = current_sensor_convert(&cs);
    CHECK(cs.current_valid == true);
    CHECK(cs.selected_range == CURRENT_SENSOR_RANGE_50A);
    CHECK(fabsf(current - 20.0f) < 0.25f);
    CHECK(fabsf(cs.current_50a - 20.0f) < 0.25f);
    CHECK(fabsf(cs.current_800a - 20.0f) < 1.0f);

    cs = (current_sensor_t){0};
    cs.count_high = adc_count_for_sensor_voltage(2.65f);  /* +60 A on 800 A / C_SENSE_H */
    cs.count_low = adc_count_for_sensor_voltage(4.75f);   /* 50 A channel at clamp / C_SENSE_L */
    host_current_sensor_mark_fresh(&cs);
    current = current_sensor_convert(&cs);
    CHECK(cs.current_valid == true);
    CHECK(cs.selected_range == CURRENT_SENSOR_RANGE_800A);
    CHECK(fabsf(current - 60.0f) < 1.0f);

    cs = (current_sensor_t){0};
    cs.count_high = adc_count_for_sensor_voltage(4.75f);
    cs.count_low = adc_count_for_sensor_voltage(4.75f);
    host_current_sensor_mark_fresh(&cs);
    (void)current_sensor_convert(&cs);
    CHECK(cs.current_valid == false);
    CHECK(cs.reason == CURRENT_SENSOR_REASON_SENSOR_SATURATION);

    cs = (current_sensor_t){0};
    cs.count_high = adc_count_for_sensor_voltage(2.5f);   /* 0 A on 800 A / C_SENSE_H */
    cs.count_low = adc_count_for_sensor_voltage(3.3f);    /* +20 A on 50 A / C_SENSE_L */
    host_current_sensor_mark_fresh(&cs);
    (void)current_sensor_convert(&cs);
    CHECK(cs.current_valid == false);
    CHECK(cs.reason == CURRENT_SENSOR_REASON_CHANNEL_MISMATCH);

    cs = (current_sensor_t){0};
    cs.count_high = 3900u;
    cs.count_low = adc_count_for_sensor_voltage(2.5f);
    host_current_sensor_mark_fresh(&cs);
    (void)current_sensor_convert(&cs);
    CHECK(cs.current_valid == false);
    CHECK(cs.reason == CURRENT_SENSOR_REASON_ADC_IMPLAUSIBLE);

    cs = (current_sensor_t){0};
    cs.count_high = adc_count_for_sensor_voltage(2.5f);
    cs.count_low = adc_count_for_sensor_voltage(2.5f);
    cs.last_read_ok = false;
    cs.count_high_fresh = true;
    cs.count_low_fresh = false;
    (void)current_sensor_convert(&cs);
    CHECK(cs.current_valid == false);
    CHECK(cs.reason == CURRENT_SENSOR_REASON_ADC_READ);

    CHECK(strcmp(current_sensor_reason_str(CURRENT_SENSOR_REASON_SENSOR_SATURATION), "sensor_saturation") == 0);
    CHECK(strcmp(current_sensor_range_str(CURRENT_SENSOR_RANGE_800A), "800A") == 0);
}

static void test_current_task_measurement_state(void)
{
    static ADC_HandleTypeDef adc1, adc2;

    init_fake_app();
    app.board.current_sensor.hadc_high = &adc1;
    app.board.current_sensor.hadc_low = &adc2;
    app.current = 99.0f;
    fake_adc_set_two_read_sequence(adc_count_for_sensor_voltage(2.5f),
                                   adc_count_for_sensor_voltage(2.5f));
    run_one_current_task_iteration(&app);
    CHECK(app.current_valid == true);
    CHECK(app.current_fault == false);
    CHECK(app.current_meas_reason == CURRENT_SENSOR_REASON_OK);
    CHECK(app.current_selected_range == CURRENT_SENSOR_RANGE_50A);
    CHECK(fabsf(app.current) < 0.05f);

    init_fake_app();
    app.board.current_sensor.hadc_high = &adc1;
    app.board.current_sensor.hadc_low = &adc2;
    app.current = 12.3f;
    fake_adc_set_status_sequence(HAL_TIMEOUT, HAL_OK);
    run_one_current_task_iteration(&app);
    CHECK(app.current_valid == false);
    CHECK(app.current_fault == false);
    CHECK(app.current_sensor_fault == false);
    CHECK(app.current_meas_reason == CURRENT_SENSOR_REASON_ADC_READ);
    CHECK(fabsf(app.current - 12.3f) < 0.001f);

    for(int i = 0; i < 25; i++)
    {
        fake_adc_set_status_sequence(HAL_TIMEOUT, HAL_OK);
        run_one_current_task_iteration(&app);
    }
    CHECK(app.current_sensor_fault == true);
    CHECK(app.current_fault == true);
    CHECK(app.current_fault_reason == CURRENT_FAULT_REASON_SENSOR_ADC_READ);
}


static void test_current_task_threshold_faults(void)
{
    static ADC_HandleTypeDef adc1, adc2;

    init_fake_app();
    app.state = STATE_DISCARGE;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    app.board.current_sensor.hadc_high = &adc1;
    app.board.current_sensor.hadc_low = &adc2;

    for(int i = 0; i < 25; i++)
    {
        fake_adc_set_two_read_sequence(adc_count_for_sensor_voltage(2.725f),  /* +90 A on 800 A / C_SENSE_H */
                                       adc_count_for_sensor_voltage(4.75f));   /* 50 A channel clamped / C_SENSE_L */
        run_one_current_task_iteration(&app);
    }

    CHECK(app.current_valid == true);
    CHECK(app.current_selected_range == CURRENT_SENSOR_RANGE_800A);
    CHECK(fabsf(app.current - 90.0f) < 1.0f);
    CHECK(app.current_overcurrent_fault == true);
    CHECK(app.current_fault_latched == true);
    CHECK(app.current_fault == true);
    CHECK(app.current_fault_latched_reason == CURRENT_FAULT_REASON_DISCHARGE_OVERCURRENT);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    /* The second half verifies a valid cold reading, so restore a working mux. */
    fake_mux_write_enable = 1;
    init_fake_app();
    app.state = STATE_START;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    app.board.current_sensor.hadc_high = &adc1;
    app.board.current_sensor.hadc_low = &adc2;

    for(int i = 0; i < 2; i++)
    {
        fake_adc_set_two_read_sequence(adc_count_for_sensor_voltage(2.50625f), /* +2.5 A on 800 A / C_SENSE_H */
                                       adc_count_for_sensor_voltage(2.6f));     /* +2.5 A on 50 A / C_SENSE_L */
        run_one_current_task_iteration(&app);
    }

    CHECK(app.current_valid == true);
    CHECK(fabsf(app.current - 2.5f) < 0.2f);
    CHECK(app.current_overcurrent_fault == true);
    CHECK(app.current_fault_latched == true);
    CHECK(app.current_fault_latched_reason == CURRENT_FAULT_REASON_PRECHARGE_FAST_OVERCURRENT);
    CHECK(app.bms_state == false);
}

static ams_air_monitor_config_t air_test_config(void)
{
    ams_air_monitor_config_t config = {
        .command_timeout_ms = 50u,
        .contact_sample_timeout_ms = 25u,
        .voltage_sample_timeout_ms = 25u,
        .max_sample_skew_ms = 5u,
        .debounce_ms = 2u,
        .pos_make_timeout_ms = 10u,
        .pos_release_timeout_ms = 10u,
        .neg_make_timeout_ms = 10u,
        .neg_release_timeout_ms = 10u,
        .precharge_make_timeout_ms = 10u,
        .precharge_release_timeout_ms = 10u,
        .precharge_max_ms = 40u,
        .run_voltage_settle_ms = 5u,
        .bus_discharge_timeout_ms = 5u,
        .minimum_pack_voltage_mv = 100000u,
        .open_bus_max_mv = 5000u,
        .precharge_complete_min_permille = 850u,
        .run_bus_min_permille = 850u,
        .run_bus_max_permille = 1100u,
        .require_precharge_aux = true,
        .require_bus_voltage = true
    };
    return config;
}

static ams_air_monitor_inputs_t air_test_inputs(uint32_t now,
                                                ams_air_phase_t phase,
                                                ams_air_contact_state_t pos,
                                                ams_air_contact_state_t neg,
                                                ams_air_contact_state_t precharge,
                                                uint32_t load_mv)
{
    ams_air_monitor_inputs_t inputs = {0};
    inputs.now_tick = now;
    inputs.command.valid = true;
    inputs.command.phase = phase;
    inputs.command.update_tick = now;
    inputs.pos_aux.valid = true;
    inputs.pos_aux.state = pos;
    inputs.pos_aux.update_tick = now;
    inputs.neg_aux.valid = true;
    inputs.neg_aux.state = neg;
    inputs.neg_aux.update_tick = now;
    inputs.precharge_aux.valid = true;
    inputs.precharge_aux.state = precharge;
    inputs.precharge_aux.update_tick = now;
    inputs.pack_voltage.valid = true;
    inputs.pack_voltage.millivolts = 400000u;
    inputs.pack_voltage.update_tick = now;
    inputs.load_voltage.valid = true;
    inputs.load_voltage.millivolts = load_mv;
    inputs.load_voltage.update_tick = now;
    return inputs;
}

static void air_test_refresh(ams_air_monitor_inputs_t *inputs, uint32_t now)
{
    CHECK(inputs != NULL);
    inputs->now_tick = now;
    inputs->command.update_tick = now;
    inputs->pos_aux.update_tick = now;
    inputs->neg_aux.update_tick = now;
    inputs->precharge_aux.update_tick = now;
    inputs->pack_voltage.update_tick = now;
    inputs->load_voltage.update_tick = now;
}

static void test_air_feedback_scaffold(void)
{
    ams_air_monitor_t monitor;
    ams_air_monitor_config_t config = air_test_config();
    ams_air_monitor_inputs_t inputs;

    memset(&monitor, 0xA5, sizeof(monitor));
    ams_air_monitor_init(&monitor, false);
    CHECK(monitor.feature_enabled == false);
    CHECK(monitor.feedback_valid == false);
    CHECK(monitor.fault == false);
    CHECK(monitor.reason == AMS_AIR_FAULT_FEATURE_DISABLED);
    CHECK(!ams_air_monitor_ready(&monitor));

    ams_air_monitor_init(&monitor, true);
    CHECK(monitor.feature_enabled == true);
    CHECK(monitor.feedback_valid == false);
    CHECK(monitor.fault == true);
    CHECK(monitor.reason == AMS_AIR_FAULT_WAITING_FOR_INPUTS);
    CHECK(!ams_air_monitor_ready(&monitor));

    CHECK(ams_air_monitor_config_valid(&config));
    CHECK(!ams_air_monitor_config_valid(NULL));
    CHECK(ams_air_monitor_schedule_valid(&config, 2u, 5u));
    CHECK(!ams_air_monitor_schedule_valid(&config, 0u, 5u));
    CHECK(!ams_air_monitor_schedule_valid(&config, 6u, 5u));
    CHECK(!ams_air_monitor_schedule_valid(&config, 2u, 6u));
    CHECK(!ams_air_monitor_schedule_valid(NULL, 2u, 5u));

    /* Every physical deadline must be reachable by the selected task and
     * publication schedule, including open-bus discharge. */
    config.bus_discharge_timeout_ms = 4u;
    CHECK(!ams_air_monitor_schedule_valid(&config, 2u, 5u));
    config.bus_discharge_timeout_ms = 5u;

    /* A fresh contact edge may take one evaluation to observe plus one or more
     * evaluations to satisfy debounce. Reject a schedule that would time out
     * before even an immediate hardware transition can be debounced. */
    config.pos_make_timeout_ms = 3u;
    CHECK(ams_air_monitor_config_valid(&config));
    CHECK(!ams_air_monitor_schedule_valid(&config, 2u, 3u));
    config.pos_make_timeout_ms = 10u;
    CHECK(ams_air_monitor_schedule_valid(&config, 2u, 5u));

    config.command_timeout_ms = 0u;
    CHECK(!ams_air_monitor_config_valid(&config));

    /* A GPIO-only two-contactor revision is representable without fabricating
     * precharge-AUX or voltage samples. Precharge duration remains mandatory. */
    config = air_test_config();
    config.require_precharge_aux = false;
    config.require_bus_voltage = false;
    config.precharge_make_timeout_ms = 0u;
    config.precharge_release_timeout_ms = 0u;
    config.voltage_sample_timeout_ms = 0u;
    config.run_voltage_settle_ms = 0u;
    config.bus_discharge_timeout_ms = 0u;
    config.minimum_pack_voltage_mv = 0u;
    config.open_bus_max_mv = 0u;
    config.precharge_complete_min_permille = 0u;
    config.run_bus_min_permille = 0u;
    config.run_bus_max_permille = 0u;
    CHECK(ams_air_monitor_config_valid(&config));

    inputs = air_test_inputs(100u,
                             AMS_AIR_PHASE_OFF,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             0u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 102u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.feedback_valid == true);
    CHECK(monitor.boot_open_verified == true);
    CHECK(ams_air_monitor_ready(&monitor));

    config.command_timeout_ms = 0u;
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.fault == true);
    CHECK(monitor.reason == AMS_AIR_FAULT_CONFIG_INVALID);
    CHECK(!ams_air_monitor_ready(&monitor));
    CHECK(strcmp(ams_air_phase_str(AMS_AIR_PHASE_PRECHARGE), "precharge") == 0);
    CHECK(strcmp(ams_air_contact_state_str(AMS_AIR_CONTACT_LINE_FAULT), "line_fault") == 0);
    CHECK(strcmp(ams_air_fault_reason_str(AMS_AIR_FAULT_POS_WELDED), "air_pos_welded") == 0);
    CHECK(!ams_air_monitor_ready(NULL));
    ams_air_monitor_init(NULL, true);
    ams_air_monitor_step(NULL, NULL, NULL);
    CHECK(!ams_air_monitor_request_clear(NULL, NULL, NULL));
}

static void test_air_monitor_nominal_sequence_and_weld_clear(void)
{
    ams_air_monitor_t monitor;
    ams_air_monitor_config_t config = air_test_config();
    ams_air_monitor_inputs_t inputs = air_test_inputs(100u,
                                                      AMS_AIR_PHASE_OFF,
                                                      AMS_AIR_CONTACT_OPEN,
                                                      AMS_AIR_CONTACT_OPEN,
                                                      AMS_AIR_CONTACT_OPEN,
                                                      0u);

    ams_air_monitor_init(&monitor, true);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.transition_pending == true);
    CHECK(monitor.permit == false);
    CHECK(monitor.boot_open_verified == false);

    air_test_refresh(&inputs, 102u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.feedback_valid == true);
    CHECK(monitor.transition_pending == true); /* bus-discharge proof pending */
    CHECK(monitor.boot_open_verified == false);

    air_test_refresh(&inputs, 105u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.steady_state_valid == true);
    CHECK(monitor.boot_open_verified == true);
    CHECK(ams_air_monitor_ready(&monitor));

    inputs.command.phase = AMS_AIR_PHASE_PRECHARGE;
    air_test_refresh(&inputs, 110u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.transition_pending == true);
    CHECK(monitor.permit == true);
    CHECK(monitor.fault == false);

    inputs.neg_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs.precharge_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs.load_voltage.millivolts = 100000u;
    air_test_refresh(&inputs, 111u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 113u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.steady_state_valid == true);
    CHECK(monitor.precharge_complete == false);
    CHECK(ams_air_monitor_ready(&monitor));

    inputs.load_voltage.millivolts = 360000u;
    air_test_refresh(&inputs, 120u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.precharge_complete == true);
    CHECK(ams_air_monitor_ready(&monitor));

    inputs.command.phase = AMS_AIR_PHASE_RUN;
    air_test_refresh(&inputs, 121u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.transition_pending == true);
    CHECK(monitor.permit == true);

    inputs.pos_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs.precharge_aux.state = AMS_AIR_CONTACT_OPEN;
    air_test_refresh(&inputs, 122u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 124u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.transition_pending == true); /* RUN voltage settle time */
    CHECK(monitor.permit == true);
    air_test_refresh(&inputs, 126u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.steady_state_valid == true);
    CHECK(monitor.bus_plausible == true);
    CHECK(ams_air_monitor_ready(&monitor));

    /* A shutdown transition does not fault while the manufacturer-qualified
     * release interval is still open. */
    inputs.command.phase = AMS_AIR_PHASE_SHUTDOWN;
    inputs.load_voltage.millivolts = 0u;
    air_test_refresh(&inputs, 130u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 139u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.fault == false);
    CHECK(monitor.transition_pending == true);
    CHECK(monitor.permit == false);
    CHECK(!ams_air_monitor_ready(&monitor));

    air_test_refresh(&inputs, 140u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.fault == true);
    CHECK(monitor.fault_latched == true);
    CHECK((monitor.latched_fault_mask & AMS_AIR_FAULT_BIT_POS_CONTACT) != 0u);
    CHECK((monitor.latched_fault_mask & AMS_AIR_FAULT_BIT_NEG_CONTACT) != 0u);
    CHECK(monitor.reason == AMS_AIR_FAULT_POS_WELDED);
    CHECK(!ams_air_monitor_ready(&monitor));

    /* Latches cannot be cleared while either contact remains closed. */
    CHECK(!ams_air_monitor_request_clear(&monitor, &config, &inputs));

    inputs.pos_aux.state = AMS_AIR_CONTACT_OPEN;
    inputs.neg_aux.state = AMS_AIR_CONTACT_OPEN;
    inputs.load_voltage.millivolts = 200000u;
    air_test_refresh(&inputs, 141u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 143u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK((monitor.active_fault_mask & AMS_AIR_FAULT_BIT_BUS_VOLTAGE) != 0u);
    CHECK(!ams_air_monitor_request_clear(&monitor, &config, &inputs));

    inputs.load_voltage.millivolts = 0u;
    air_test_refresh(&inputs, 144u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.active_fault_mask == 0u);
    CHECK(monitor.fault_latched == true);
    CHECK(ams_air_monitor_request_clear(&monitor, &config, &inputs));
    CHECK(monitor.fault == false);
    CHECK(monitor.fault_latched == false);
    CHECK(monitor.latched_fault_mask == 0u);
    CHECK(monitor.permit == false);
    CHECK(!ams_air_monitor_ready(&monitor));

    /* Clearing in SHUTDOWN removes the evidence only. OFF is the explicit
     * re-arm boundary, and still requires the open/bus-safe proof. */
    inputs.command.phase = AMS_AIR_PHASE_OFF;
    air_test_refresh(&inputs, 145u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(!ams_air_monitor_ready(&monitor));
    air_test_refresh(&inputs, 150u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(ams_air_monitor_ready(&monitor));
}

static void test_air_monitor_shutdown_rearm_and_live_precharge_proof(void)
{
    ams_air_monitor_t monitor;
    ams_air_monitor_config_t config = air_test_config();
    ams_air_monitor_inputs_t inputs = air_test_inputs(100u,
                                                      AMS_AIR_PHASE_OFF,
                                                      AMS_AIR_CONTACT_OPEN,
                                                      AMS_AIR_CONTACT_OPEN,
                                                      AMS_AIR_CONTACT_OPEN,
                                                      0u);

    /* Establish the only valid starting point: a fresh, debounced, discharged
     * all-open state. */
    ams_air_monitor_init(&monitor, true);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 102u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 105u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(ams_air_monitor_ready(&monitor));

    /* SHUTDOWN is a non-permitting terminal state. Reaching a proven open
     * state must not automatically reassert BMS_OK; the command owner must
     * explicitly pass through OFF before another close sequence. */
    inputs.command.phase = AMS_AIR_PHASE_SHUTDOWN;
    air_test_refresh(&inputs, 110u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 115u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.steady_state_valid == true);
    CHECK(monitor.permit == false);
    CHECK(!ams_air_monitor_ready(&monitor));

    /* A controlled clear may erase a diagnosed latch in SHUTDOWN, but it must
     * not itself be an authorization/re-arm action. */
    CHECK(ams_air_monitor_request_clear(&monitor, &config, &inputs));
    CHECK(monitor.permit == false);
    CHECK(!ams_air_monitor_ready(&monitor));

    /* SHUTDOWN -> PRECHARGE bypasses the explicit OFF re-arm and is therefore
     * a persistent sequencing fault. */
    inputs.command.phase = AMS_AIR_PHASE_PRECHARGE;
    air_test_refresh(&inputs, 116u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.fault_latched == true);
    CHECK((monitor.latched_fault_mask & AMS_AIR_FAULT_BIT_TRANSITION) != 0u);
    CHECK(!ams_air_monitor_ready(&monitor));

    /* Rebuild a valid sequence and prove that PRECHARGE -> RUN uses voltage
     * proof from the current transition snapshot, not a stale success bit from
     * the prior monitor cycle. */
    inputs = air_test_inputs(200u,
                             AMS_AIR_PHASE_OFF,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             0u);
    ams_air_monitor_init(&monitor, true);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 202u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 205u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(ams_air_monitor_ready(&monitor));

    inputs.command.phase = AMS_AIR_PHASE_PRECHARGE;
    inputs.neg_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs.precharge_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs.load_voltage.millivolts = 360000u;
    air_test_refresh(&inputs, 210u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 212u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.steady_state_valid == true);
    CHECK(monitor.precharge_complete == true);

    inputs.command.phase = AMS_AIR_PHASE_RUN;
    inputs.pos_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs.precharge_aux.state = AMS_AIR_CONTACT_OPEN;
    inputs.load_voltage.millivolts = 0u;
    air_test_refresh(&inputs, 213u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.fault_latched == true);
    CHECK(monitor.reason == AMS_AIR_FAULT_PRECHARGE_INCOMPLETE);
    CHECK(monitor.permit == false);
    CHECK(!ams_air_monitor_ready(&monitor));
}

static void air_test_prime_steady_monitor(ams_air_monitor_t *monitor,
                                          ams_air_phase_t phase)
{
    ams_air_contact_state_t pos = AMS_AIR_CONTACT_OPEN;
    ams_air_contact_state_t neg = AMS_AIR_CONTACT_OPEN;
    ams_air_contact_state_t precharge = AMS_AIR_CONTACT_OPEN;

    CHECK(monitor != NULL);
    if(phase == AMS_AIR_PHASE_PRECHARGE)
    {
        neg = AMS_AIR_CONTACT_CLOSED;
        precharge = AMS_AIR_CONTACT_CLOSED;
    }
    else if(phase == AMS_AIR_PHASE_RUN)
    {
        pos = AMS_AIR_CONTACT_CLOSED;
        neg = AMS_AIR_CONTACT_CLOSED;
    }

    ams_air_monitor_init(monitor, true);
    monitor->configuration_valid = true;
    monitor->command_valid = true;
    monitor->feedback_valid = true;
    monitor->voltage_valid = true;
    monitor->steady_state_valid = true;
    monitor->transition_authorized = (phase != AMS_AIR_PHASE_SHUTDOWN);
    monitor->boot_open_verified = true;
    monitor->permit = (phase != AMS_AIR_PHASE_SHUTDOWN);
    monitor->fault = false;
    monitor->fault_latched = false;
    monitor->phase = phase;
    monitor->previous_phase = phase;
    monitor->phase_initialized = true;
    monitor->pos_aux = pos;
    monitor->neg_aux = neg;
    monitor->precharge_aux = precharge;
    monitor->pos_filter = (ams_air_debounce_state_t){
        .candidate_valid = true,
        .debounced_valid = true,
        .candidate = pos,
        .debounced = pos,
        .candidate_since_tick = 0u
    };
    monitor->neg_filter = (ams_air_debounce_state_t){
        .candidate_valid = true,
        .debounced_valid = true,
        .candidate = neg,
        .debounced = neg,
        .candidate_since_tick = 0u
    };
    monitor->precharge_filter = (ams_air_debounce_state_t){
        .candidate_valid = true,
        .debounced_valid = true,
        .candidate = precharge,
        .debounced = precharge,
        .candidate_since_tick = 0u
    };
}

static void test_air_monitor_transition_state_space_and_stale_recovery(void)
{
    static const ams_air_phase_t phases[4] = {
        AMS_AIR_PHASE_OFF,
        AMS_AIR_PHASE_PRECHARGE,
        AMS_AIR_PHASE_RUN,
        AMS_AIR_PHASE_SHUTDOWN
    };
    static const bool allowed[4][4] = {
        /* to: OFF,  PRECHARGE, RUN,  SHUTDOWN */
        { true,  true,       false, true  }, /* OFF */
        { true,  true,       true,  true  }, /* PRECHARGE */
        { true,  false,      true,  true  }, /* RUN */
        { true,  false,      false, true  }  /* SHUTDOWN */
    };
    ams_air_monitor_config_t config = air_test_config();

    /* Exhaust every edge in the four-state transition graph with zero
     * debounce and no optional sensors. This independently checks that adding
     * a future phase cannot accidentally create a close-sequence bypass. */
    config.require_precharge_aux = false;
    config.require_bus_voltage = false;
    config.debounce_ms = 0u;
    config.precharge_make_timeout_ms = 0u;
    config.precharge_release_timeout_ms = 0u;
    config.voltage_sample_timeout_ms = 0u;
    config.run_voltage_settle_ms = 0u;
    config.bus_discharge_timeout_ms = 0u;
    config.minimum_pack_voltage_mv = 0u;
    config.open_bus_max_mv = 0u;
    config.precharge_complete_min_permille = 0u;
    config.run_bus_min_permille = 0u;
    config.run_bus_max_permille = 0u;
    CHECK(ams_air_monitor_config_valid(&config));

    for(unsigned int from = 0u; from < 4u; from++)
    {
        for(unsigned int to = 0u; to < 4u; to++)
        {
            ams_air_monitor_t monitor;
            ams_air_contact_state_t pos = AMS_AIR_CONTACT_OPEN;
            ams_air_contact_state_t neg = AMS_AIR_CONTACT_OPEN;
            ams_air_monitor_inputs_t inputs;

            if(phases[to] == AMS_AIR_PHASE_PRECHARGE)
            {
                neg = AMS_AIR_CONTACT_CLOSED;
            }
            else if(phases[to] == AMS_AIR_PHASE_RUN)
            {
                pos = AMS_AIR_CONTACT_CLOSED;
                neg = AMS_AIR_CONTACT_CLOSED;
            }

            air_test_prime_steady_monitor(&monitor, phases[from]);
            inputs = air_test_inputs(100u + (from * 10u) + to,
                                     phases[to],
                                     pos,
                                     neg,
                                     AMS_AIR_CONTACT_OPEN,
                                     0u);
            monitor.phase_start_tick = inputs.now_tick;
            ams_air_monitor_step(&monitor, &config, &inputs);

            if(allowed[from][to])
            {
                if(monitor.fault_latched)
                {
                    fprintf(stderr,
                            "AIR transition matrix unexpected latch from=%u to=%u reason=%u mask=0x%08lx\n",
                            from,
                            to,
                            (unsigned int)monitor.reason,
                            (unsigned long)monitor.latched_fault_mask);
                }
                CHECK(monitor.fault_latched == false);
                CHECK(monitor.active_fault_mask == 0u);
                if(phases[to] == AMS_AIR_PHASE_SHUTDOWN)
                {
                    CHECK(monitor.permit == false);
                    CHECK(!ams_air_monitor_ready(&monitor));
                }
                else
                {
                    CHECK(ams_air_monitor_ready(&monitor));
                }
            }
            else
            {
                CHECK(monitor.fault_latched == true);
                CHECK((monitor.latched_fault_mask &
                       AMS_AIR_FAULT_BIT_TRANSITION) != 0u);
                CHECK(!ams_air_monitor_ready(&monitor));
            }
        }
    }

    /* Once operation is energized, a source that exceeds its reviewed
     * freshness timeout has already caused BMS_OK to drop. It must remain
     * latched through same-phase data recovery and require verified open/clear
     * handling, rather than reasserting while the AIRs may be moving. */
    {
        ams_air_monitor_t monitor;
        ams_air_monitor_inputs_t inputs = air_test_inputs(500u,
                                                          AMS_AIR_PHASE_RUN,
                                                          AMS_AIR_CONTACT_CLOSED,
                                                          AMS_AIR_CONTACT_CLOSED,
                                                          AMS_AIR_CONTACT_OPEN,
                                                          400000u);
        air_test_prime_steady_monitor(&monitor, AMS_AIR_PHASE_RUN);
        inputs.command.update_tick =
            inputs.now_tick - config.command_timeout_ms - 1u;
        ams_air_monitor_step(&monitor, &config, &inputs);
        CHECK(monitor.reason == AMS_AIR_FAULT_COMMAND_STALE);
        CHECK(monitor.fault_latched == true);
        CHECK(!ams_air_monitor_ready(&monitor));

        air_test_refresh(&inputs, 501u);
        ams_air_monitor_step(&monitor, &config, &inputs);
        CHECK(monitor.active_fault_mask == 0u);
        CHECK(monitor.fault_latched == true);
        CHECK(monitor.permit == false);
        CHECK(!ams_air_monitor_ready(&monitor));
    }

    {
        ams_air_monitor_t monitor;
        ams_air_monitor_inputs_t inputs = air_test_inputs(600u,
                                                          AMS_AIR_PHASE_RUN,
                                                          AMS_AIR_CONTACT_CLOSED,
                                                          AMS_AIR_CONTACT_CLOSED,
                                                          AMS_AIR_CONTACT_OPEN,
                                                          400000u);
        air_test_prime_steady_monitor(&monitor, AMS_AIR_PHASE_RUN);
        monitor.phase_start_tick = inputs.now_tick;
        inputs.pos_aux.update_tick =
            inputs.now_tick - config.max_sample_skew_ms - 1u;
        ams_air_monitor_step(&monitor, &config, &inputs);
        CHECK(monitor.reason == AMS_AIR_FAULT_SAMPLE_INCOHERENT);
        CHECK((monitor.latched_fault_mask &
               AMS_AIR_FAULT_BIT_SAMPLE_INCOHERENT) != 0u);
        CHECK(monitor.fault_latched == true);
        CHECK(monitor.permit == false);
        CHECK(!ams_air_monitor_ready(&monitor));
    }

    /* Retaining authority through a close transition requires the complete
     * prior ready predicate. A corrupted/inconsistent mask may not be ignored
     * merely because the permit/fault booleans still look healthy. */
    {
        ams_air_monitor_t monitor;
        ams_air_monitor_inputs_t inputs = air_test_inputs(
            700u,
            AMS_AIR_PHASE_PRECHARGE,
            AMS_AIR_CONTACT_OPEN,
            AMS_AIR_CONTACT_CLOSED,
            AMS_AIR_CONTACT_CLOSED,
            360000u);

        air_test_prime_steady_monitor(&monitor, AMS_AIR_PHASE_OFF);
        monitor.active_fault_mask = AMS_AIR_FAULT_BIT_COMMAND;
        ams_air_monitor_step(&monitor, &config, &inputs);
        CHECK(monitor.fault_latched == true);
        CHECK(monitor.reason == AMS_AIR_FAULT_REARM_REQUIRED);
        CHECK(monitor.permit == false);
        CHECK(!ams_air_monitor_ready(&monitor));
    }
}

static void test_air_monitor_ready_snapshot_integrity(void)
{
    ams_air_monitor_t monitor;

    air_test_prime_steady_monitor(&monitor, AMS_AIR_PHASE_RUN);
    CHECK(ams_air_monitor_ready(&monitor));

    monitor.phase = AMS_AIR_PHASE_SHUTDOWN;
    CHECK(!ams_air_monitor_ready(&monitor));
    monitor.phase = AMS_AIR_PHASE_RUN;

    monitor.active_fault_mask = AMS_AIR_FAULT_BIT_COMMAND;
    CHECK(!ams_air_monitor_ready(&monitor));
    monitor.active_fault_mask = 0u;

    monitor.transition_authorized = false;
    CHECK(!ams_air_monitor_ready(&monitor));
    monitor.transition_authorized = true;

    monitor.steady_state_valid = false;
    monitor.transition_pending = false;
    CHECK(!ams_air_monitor_ready(&monitor));
    monitor.transition_pending = true;
    CHECK(ams_air_monitor_ready(&monitor));

    monitor.boot_open_verified = false;
    CHECK(!ams_air_monitor_ready(&monitor));
}

static void test_air_monitor_faults_freshness_and_tick_wrap(void)
{
    ams_air_monitor_t monitor;
    ams_air_monitor_config_t config = air_test_config();
    ams_air_monitor_inputs_t inputs = air_test_inputs(10u,
                                                      AMS_AIR_PHASE_RUN,
                                                      AMS_AIR_CONTACT_CLOSED,
                                                      AMS_AIR_CONTACT_CLOSED,
                                                      AMS_AIR_CONTACT_OPEN,
                                                      400000u);

    /* A reboot into RUN cannot trust already-closed contacts. A verified
     * all-open state is mandatory before the close sequence is authorized. */
    ams_air_monitor_init(&monitor, true);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 12u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.reason == AMS_AIR_FAULT_BOOT_OPEN_NOT_VERIFIED);
    CHECK((monitor.active_fault_mask & AMS_AIR_FAULT_BIT_BOOT_OPEN) != 0u);
    CHECK(!ams_air_monitor_ready(&monitor));

    /* Stale command and contact samples are immediate, recoverable inhibits. */
    inputs.command.phase = AMS_AIR_PHASE_OFF;
    inputs.pos_aux.state = AMS_AIR_CONTACT_OPEN;
    inputs.neg_aux.state = AMS_AIR_CONTACT_OPEN;
    inputs.load_voltage.millivolts = 0u;
    air_test_refresh(&inputs, 20u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 22u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 25u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.boot_open_verified == true);

    inputs.now_tick = 76u;
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.reason == AMS_AIR_FAULT_COMMAND_STALE);
    CHECK(monitor.fault_latched == false);

    air_test_refresh(&inputs, 77u);
    inputs.pos_aux.update_tick = 51u;
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.reason == AMS_AIR_FAULT_INPUT_STALE);
    CHECK(monitor.fault_latched == false);

    air_test_refresh(&inputs, 78u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(ams_air_monitor_ready(&monitor));

    inputs.now_tick = 79u;
    inputs.command.update_tick = 79u;
    inputs.pos_aux.update_tick = 79u;
    inputs.neg_aux.update_tick = 79u;
    inputs.precharge_aux.update_tick = 79u;
    inputs.pack_voltage.update_tick = 53u;
    inputs.load_voltage.update_tick = 79u;
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.reason == AMS_AIR_FAULT_VOLTAGE_STALE);
    CHECK(monitor.fault_latched == false);

    air_test_refresh(&inputs, 79u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(ams_air_monitor_ready(&monitor));

    /* Individually fresh samples can still be unsafe as one decision if their
     * timestamps exceed the reviewed cross-snapshot skew. OFF recovers after a
     * coherent refresh because no contact is energized. */
    air_test_refresh(&inputs, 80u);
    inputs.pos_aux.update_tick =
        inputs.now_tick - config.max_sample_skew_ms - 1u;
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.reason == AMS_AIR_FAULT_SAMPLE_INCOHERENT);
    CHECK((monitor.active_fault_mask &
           AMS_AIR_FAULT_BIT_SAMPLE_INCOHERENT) != 0u);
    CHECK(monitor.fault_latched == false);
    CHECK(!ams_air_monitor_ready(&monitor));
    air_test_refresh(&inputs, 81u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(ams_air_monitor_ready(&monitor));

    /* A direct OFF -> RUN command bypasses precharge and is latched. */
    inputs.command.phase = AMS_AIR_PHASE_RUN;
    air_test_refresh(&inputs, 82u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.reason == AMS_AIR_FAULT_INVALID_TRANSITION);
    CHECK((monitor.latched_fault_mask & AMS_AIR_FAULT_BIT_TRANSITION) != 0u);

    /* Return to a proven all-open state before an explicit clear. */
    inputs.command.phase = AMS_AIR_PHASE_OFF;
    air_test_refresh(&inputs, 83u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 88u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(ams_air_monitor_request_clear(&monitor, &config, &inputs));

    /* Missing close feedback after the configured deadline is latched. */
    inputs.command.phase = AMS_AIR_PHASE_PRECHARGE;
    air_test_refresh(&inputs, 90u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 100u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK((monitor.latched_fault_mask & AMS_AIR_FAULT_BIT_NEG_CONTACT) != 0u);
    CHECK((monitor.latched_fault_mask & AMS_AIR_FAULT_BIT_PRECHARGE_CONTACT) != 0u);

    /* RUN cannot bypass completion of the preceding steady Precharge state. */
    ams_air_monitor_init(&monitor, true);
    inputs = air_test_inputs(110u,
                             AMS_AIR_PHASE_OFF,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             0u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 112u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 115u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    inputs.command.phase = AMS_AIR_PHASE_PRECHARGE;
    inputs.neg_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs.precharge_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs.load_voltage.millivolts = 100000u;
    air_test_refresh(&inputs, 120u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 122u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.steady_state_valid == true);
    CHECK(monitor.precharge_complete == false);
    inputs.command.phase = AMS_AIR_PHASE_RUN;
    air_test_refresh(&inputs, 123u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.reason == AMS_AIR_FAULT_PRECHARGE_INCOMPLETE);
    CHECK((monitor.latched_fault_mask & AMS_AIR_FAULT_BIT_TRANSITION) != 0u);
    CHECK(!ams_air_monitor_ready(&monitor));

    /* A supervised line fault has its own persistent diagnostic. */
    ams_air_monitor_init(&monitor, true);
    inputs = air_test_inputs(200u,
                             AMS_AIR_PHASE_OFF,
                             AMS_AIR_CONTACT_LINE_FAULT,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             0u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 202u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.reason == AMS_AIR_FAULT_LINE_SUPERVISION);
    CHECK((monitor.latched_fault_mask & AMS_AIR_FAULT_BIT_LINE_SUPERVISION) != 0u);

    /* Unsafely high load-side voltage after the discharge deadline is latched
     * even when every auxiliary contact reports open. */
    ams_air_monitor_init(&monitor, true);
    inputs = air_test_inputs(300u,
                             AMS_AIR_PHASE_OFF,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             200000u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 302u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 305u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.reason == AMS_AIR_FAULT_BUS_VOLTAGE_PLAUSIBILITY);
    CHECK((monitor.latched_fault_mask & AMS_AIR_FAULT_BIT_BUS_VOLTAGE) != 0u);

    /* Remaining in PRECHARGE beyond the reviewed maximum is a latched
     * sequencing fault even if the bus has reached the completion ratio. */
    ams_air_monitor_init(&monitor, true);
    inputs = air_test_inputs(400u,
                             AMS_AIR_PHASE_OFF,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             0u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 402u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 405u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.boot_open_verified == true);
    inputs.command.phase = AMS_AIR_PHASE_PRECHARGE;
    inputs.neg_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs.precharge_aux.state = AMS_AIR_CONTACT_CLOSED;
    inputs.load_voltage.millivolts = 360000u;
    air_test_refresh(&inputs, 410u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 412u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.precharge_complete == true);
    air_test_refresh(&inputs, 449u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.fault == false);
    air_test_refresh(&inputs, 450u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.reason == AMS_AIR_FAULT_PRECHARGE_TIMEOUT);
    CHECK((monitor.latched_fault_mask & AMS_AIR_FAULT_BIT_PRECHARGE_TIMEOUT) != 0u);

    /* All ages and deadlines use unsigned subtraction and survive tick wrap. */
    ams_air_monitor_init(&monitor, true);
    inputs = air_test_inputs(UINT32_MAX - 2u,
                             AMS_AIR_PHASE_OFF,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             AMS_AIR_CONTACT_OPEN,
                             0u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    air_test_refresh(&inputs, 0u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.feedback_valid == true);
    CHECK(monitor.boot_open_verified == false);
    air_test_refresh(&inputs, 2u);
    ams_air_monitor_step(&monitor, &config, &inputs);
    CHECK(monitor.boot_open_verified == true);
    CHECK(ams_air_monitor_ready(&monitor));
}

static uint32_t air_test_rng_next(uint32_t *state)
{
    *state = (*state * 1664525u) + 1013904223u;
    return *state;
}

static void test_air_monitor_seeded_invariants(void)
{
    ams_air_monitor_t monitor;
    ams_air_monitor_config_t config = air_test_config();
    ams_air_monitor_inputs_t inputs = air_test_inputs(0u,
                                                      AMS_AIR_PHASE_OFF,
                                                      AMS_AIR_CONTACT_OPEN,
                                                      AMS_AIR_CONTACT_OPEN,
                                                      AMS_AIR_CONTACT_OPEN,
                                                      0u);
    uint32_t rng = 0xA17C0DEu;

    ams_air_monitor_init(&monitor, true);
    for(uint32_t cycle = 0u; cycle < AMS_HOST_LONG_FUZZ_CYCLES; cycle++)
    {
        uint32_t r = air_test_rng_next(&rng);
        inputs.now_tick += (r & 0x0Fu);
        inputs.command.valid = ((r & (1u << 4)) == 0u);
        inputs.command.phase = (ams_air_phase_t)((r >> 5) % 6u);
        inputs.pos_aux.valid = ((r & (1u << 8)) == 0u);
        inputs.neg_aux.valid = ((r & (1u << 9)) == 0u);
        inputs.precharge_aux.valid = ((r & (1u << 10)) == 0u);
        inputs.pos_aux.state = (ams_air_contact_state_t)((r >> 11) % 5u);
        inputs.neg_aux.state = (ams_air_contact_state_t)((r >> 14) % 5u);
        inputs.precharge_aux.state = (ams_air_contact_state_t)((r >> 17) % 5u);
        inputs.pack_voltage.valid = ((r & (1u << 20)) == 0u);
        inputs.load_voltage.valid = ((r & (1u << 21)) == 0u);
        inputs.pack_voltage.millivolts = 300000u + (r % 150001u);
        inputs.load_voltage.millivolts = (r >> 3) % 500001u;

        inputs.command.update_tick = inputs.now_tick - ((r >> 22) & 0x3Fu);
        inputs.pos_aux.update_tick = inputs.now_tick - ((r >> 12) & 0x1Fu);
        inputs.neg_aux.update_tick = inputs.now_tick - ((r >> 15) & 0x1Fu);
        inputs.precharge_aux.update_tick = inputs.now_tick - ((r >> 18) & 0x1Fu);
        inputs.pack_voltage.update_tick = inputs.now_tick - ((r >> 7) & 0x1Fu);
        inputs.load_voltage.update_tick = inputs.now_tick - ((r >> 2) & 0x1Fu);

        ams_air_monitor_step(&monitor, &config, &inputs);

        CHECK(monitor.fault_latched == (monitor.latched_fault_mask != 0u));
        CHECK(monitor.fault == ((monitor.active_fault_mask != 0u) ||
                                (monitor.latched_fault_mask != 0u)));
        CHECK(!monitor.permit || !monitor.fault);
        CHECK((monitor.phase != AMS_AIR_PHASE_SHUTDOWN) || !monitor.permit);
        CHECK(ams_air_monitor_ready(&monitor) ==
              (monitor.feature_enabled &&
               monitor.configuration_valid &&
               monitor.command_valid &&
               monitor.feedback_valid &&
               monitor.boot_open_verified &&
               ((monitor.phase == AMS_AIR_PHASE_OFF) ||
                (monitor.phase == AMS_AIR_PHASE_PRECHARGE) ||
                (monitor.phase == AMS_AIR_PHASE_RUN)) &&
               monitor.transition_authorized &&
               (monitor.steady_state_valid || monitor.transition_pending) &&
               monitor.permit &&
               !monitor.fault &&
               !monitor.fault_latched &&
               (monitor.active_fault_mask == 0u) &&
               (monitor.latched_fault_mask == 0u)));
        CHECK(!monitor.precharge_complete ||
              (monitor.phase == AMS_AIR_PHASE_PRECHARGE));
        CHECK(!monitor.feedback_valid ||
              (monitor.pos_filter.debounced_valid &&
               monitor.neg_filter.debounced_valid &&
               monitor.precharge_filter.debounced_valid));
    }
}

static void test_fan_current_and_null_guards(void){
    fan_t fan={0}; uint32_t ccr=999; static TIM_HandleTypeDef htim; CHECK(fan_init(NULL,NULL,NULL,0,NULL,1) != 0); CHECK(fan_init(&fan,NULL,&htim,1000,&ccr,1)==0); CHECK(ccr==0u); CHECK(set_fan_percent(&fan,120.0f)==0 && ccr==1000u && fabsf(fan.duty_cycle-100.0f)<0.01f); CHECK(set_fan_percent(&fan,-10.0f)==0 && ccr==0u && fabsf(fan.duty_cycle)<0.01f);
    CHECK(current_sensor_convert(NULL) == 0.0f);
    CHECK(accumulator_set_balance(NULL) == -1); CHECK(accumulator_clear_balance(NULL) == -1); accumulator_update_voltage_stats(NULL); accumulator_update_temp_stats(NULL);
    CHECK(imd_read(NULL) == 1); imd_init(NULL, 0, NULL, NULL, 0, 0, NULL, 0);
}

static void test_imd_capture_validation(void)
{
    static TIM_HandleTypeDef htim;
    static GPIO_TypeDef status_port;
    imd_t imd;

    init_fake_app();
    fake_tick = 1000u;
    fake_tim_total_capture = 100u;
    fake_tim_high_capture = 50u;
    imd_init(&imd,
             1000u,
             &htim,
             NULL,
             TIM_CHANNEL_2,
             TIM_CHANNEL_1,
             &status_port,
             1u);
    CHECK(imd.capture_started == true);
    CHECK(imd.init_status == HAL_OK);
    CHECK(imd_read_at(&imd, fake_tick) != 0);
    imd_capture_event(&imd, fake_tick);
    CHECK(imd_read(&imd) == 0);
    CHECK(imd.status == IMD_NORMAL);
    CHECK(imd.OK_HS == true);
    CHECK(fabsf(imd.freq - 10.0f) < 0.01f);
    CHECK(fabsf(imd.duty - 50.0f) < 0.01f);

    /* Live CCR changes after the interrupt cannot tear the published tuple.
     * They take effect only when the next capture event snapshots both. */
    fake_tim_high_capture = 101u;
    CHECK(imd_read(&imd) == 0);
    CHECK(imd.high_count == 50u);
    imd_capture_event(&imd, fake_tick);
    CHECK(imd_read(&imd) != 0);
    CHECK(imd.status == IMD_UNKNOWN);

    /* An in-progress/torn ISR publication is never consumed as valid. */
    imd.capture_sequence = 1u;
    CHECK(imd_read_at(&imd, fake_tick) != 0);
    CHECK(imd.status == IMD_UNKNOWN);
    imd.capture_sequence = 2u;
    CHECK(imd.OK_HS == false);
    CHECK(imd.duty == 0.0f);
    CHECK(imd.freq == 0.0f);

    /* Full-width captures must not overflow high_count * 100 before the
     * floating-point duty conversion. */
    fake_tim_total_capture = UINT32_MAX;
    fake_tim_high_capture = UINT32_MAX;
    imd_init(&imd,
             UINT32_MAX,
             &htim,
             NULL,
             TIM_CHANNEL_2,
             TIM_CHANNEL_1,
             &status_port,
             1u);
    imd_capture_event(&imd, fake_tick);
    CHECK(imd_read(&imd) == 0);
    CHECK(isfinite(imd.duty));
    CHECK(fabsf(imd.duty - 100.0f) < 0.01f);

    /* Corrupt clock/capture values can produce a frequency above INT_MAX.
     * Reject it before the status-code float-to-int conversion. */
    fake_tim_total_capture = 1u;
    fake_tim_high_capture = 0u;
    imd_init(&imd,
             UINT32_MAX,
             &htim,
             NULL,
             TIM_CHANNEL_2,
             TIM_CHANNEL_1,
             &status_port,
             1u);
    imd_capture_event(&imd, fake_tick);
    CHECK(imd_read(&imd) != 0);
    CHECK(imd.status == IMD_UNKNOWN);

    fake_tim_base_start_status = HAL_ERROR;
    imd_init(&imd,
             1000u,
             &htim,
             NULL,
             TIM_CHANNEL_2,
             TIM_CHANNEL_1,
             &status_port,
             1u);
    CHECK(imd.capture_started == false);
    CHECK(imd.init_status == HAL_ERROR);
    CHECK(imd_read(&imd) != 0);

    fake_tim_base_start_status = HAL_OK;
    imd_init(&imd,
             0u,
             &htim,
             NULL,
             TIM_CHANNEL_2,
             TIM_CHANNEL_1,
             &status_port,
             1u);
    CHECK(imd.capture_started == true);
    CHECK(imd_read(&imd) != 0);

    /* A plausible capture register value must not stay healthy after the PWM
     * edges stop, including when the RTOS tick wraps. */
    fake_tim_total_capture = 100u;
    fake_tim_high_capture = 50u;
    imd_init(&imd,
             1000u,
             &htim,
             NULL,
             TIM_CHANNEL_2,
             TIM_CHANNEL_1,
             &status_port,
             1u);
    fake_tick = UINT32_MAX - 20u;
    imd_capture_event(&imd, fake_tick);
    CHECK(imd_read_at(&imd, 30u) == 0);
    CHECK(imd_read_at(&imd,
                      (uint32_t)(fake_tick + AMS_IMD_CAPTURE_TIMEOUT_MS + 1u)) != 0);
    CHECK(imd.status == IMD_UNKNOWN);
    CHECK(imd.OK_HS == false);

    /* The task publishes a coherent fail-closed snapshot and immediately
     * drops BMS_OK when capture freshness is lost. */
    init_fake_app();
    fake_tick = 2000u;
    fake_tim_total_capture = 100u;
    fake_tim_high_capture = 50u;
    imd_init(&app.board.imd,
             1000u,
             &htim,
             NULL,
             TIM_CHANNEL_2,
             TIM_CHANNEL_1,
             &status_port,
             1u);
    imd_capture_event(&app.board.imd, fake_tick);
    CHECK(imd_task_update(&app, fake_tick));
    CHECK(app.imd_valid == true);
    CHECK(app.imd_ok == true);
    CHECK(app.imd_fault == false);
    CHECK(app.imd_last_valid_tick == fake_tick);
    CHECK((app.heartbeat.seen_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_IMD)) != 0u);

    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    fake_tick += AMS_IMD_CAPTURE_TIMEOUT_MS + 1u;
    CHECK(!imd_task_update(&app, fake_tick));
    CHECK(app.imd_valid == false);
    CHECK(app.imd_ok == false);
    CHECK(app.imd_fault == true);
    CHECK(app.imd_status == IMD_UNKNOWN);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
}



static void test_safety_panic_reset_watchdog_and_log(void)
{
    init_fake_app();
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    ams_safety_host_set_fault_regs(0x00000011u, 0x00000022u, 0x00000033u, 0x00000044u);
    ams_safety_panic(AMS_PANIC_HARDFAULT);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    CHECK(ams_safety_host_bms_forced_low() == true);
    set_bms(true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    const ams_panic_record_t *panic = ams_safety_panic_record();
    CHECK(panic->panic_reason == AMS_PANIC_HARDFAULT);
    CHECK(panic->cfsr == 0x00000011u);
    CHECK(panic->hfsr == 0x00000022u);
    CHECK(panic->mmfar == 0x00000033u);
    CHECK(panic->bfar == 0x00000044u);
    CHECK(panic->reset_count == 1u);

    ams_safety_host_set_reset_csr(RCC_CSR_IWDGRSTF | RCC_CSR_PORRSTF);
    ams_safety_record_reset_cause();
    ams_safety_sync_app(&app);
    CHECK((app.reset_flags & RCC_CSR_IWDGRSTF) != 0u);
    CHECK((app.reset_flags & RCC_CSR_PORRSTF) != 0u);
    CHECK(app.last_panic_reason == AMS_PANIC_HARDFAULT);
    CHECK(app.safety_panic_count == 1u);

    char reset_buf[128];
    ams_safety_format_reset_flags(app.reset_flags, reset_buf, sizeof(reset_buf));
    CHECK(strstr(reset_buf, "IWDG=1") != NULL);
    CHECK(strstr(reset_buf, "POR=1") != NULL);

    const ams_fault_log_t *log = ams_fault_log_get();
    CHECK(log->count >= 2u);
    ams_fault_log_clear();
    CHECK(ams_fault_log_get()->count == 0u);

    ams_fault_log_event(AMS_FAULT_LOG_BOOT, 1u, 2u, 3u);
    ams_fault_log_t snapshot;
    CHECK(ams_fault_log_snapshot(&snapshot));
    CHECK(snapshot.count == 1u);
    CHECK(snapshot.entry[0].event == AMS_FAULT_LOG_BOOT);
    ams_fault_log_event(AMS_FAULT_LOG_CAN_RECOVERED, 4u, 5u, 6u);
    CHECK(snapshot.count == 1u);
    CHECK(ams_fault_log_get()->count == 2u);
    CHECK(strcmp(ams_fault_log_event_str(AMS_FAULT_LOG_AIR_FAULT_LATCH),
                 "AIR_FAULT_LATCH") == 0);
    CHECK(strcmp(ams_fault_log_event_str(AMS_FAULT_LOG_STATE_TRANSITION),
                 "STATE_TRANSITION") == 0);
    CHECK(!ams_fault_log_snapshot(NULL));
}

static void test_retained_fault_log_integrity_recovery(void)
{
    ams_fault_log_t snapshot;

    ams_safety_host_reset_state();
    ams_safety_host_set_reset_csr(RCC_CSR_PORRSTF);
    ams_safety_record_reset_cause();
    CHECK(ams_fault_log_snapshot(&snapshot));
    CHECK(snapshot.boot_sequence == 1u);
    CHECK(snapshot.count == 1u);
    CHECK(snapshot.entry[0].boot_sequence == 1u);
    CHECK(snapshot.entry[0].event == AMS_FAULT_LOG_RESET_CAUSE);

    /* Model the next boot while retaining .noinit state. */
    ams_safety_host_set_reset_csr(RCC_CSR_IWDGRSTF);
    ams_safety_record_reset_cause();
    CHECK(ams_fault_log_snapshot(&snapshot));
    CHECK(snapshot.boot_sequence == 2u);
    CHECK(snapshot.count == 2u);
    CHECK(snapshot.entry[1].boot_sequence == 2u);

    ams_safety_host_reset_state();
    ams_fault_log_clear();
    ams_fault_log_event(AMS_FAULT_LOG_BOOT, 10u, 11u, 12u);
    ams_fault_log_event(AMS_FAULT_LOG_CAN_RECOVERED, 20u, 21u, 22u);
    ams_fault_log_event(AMS_FAULT_LOG_CURRENT_LATCH, 30u, 31u, 32u);

    CHECK(ams_fault_log_snapshot(&snapshot));
    CHECK(snapshot.magic == AMS_FAULT_LOG_MAGIC);
    CHECK(snapshot.version == AMS_FAULT_LOG_VERSION);
    CHECK(snapshot.entry_size == sizeof(ams_fault_log_entry_t));
    CHECK(snapshot.count == 3u);
    CHECK(snapshot.next_sequence == 4u);
    CHECK(snapshot.entry[0].sequence == 1u);
    CHECK(snapshot.entry[1].sequence == 2u);
    CHECK(snapshot.entry[2].sequence == 3u);
    CHECK(snapshot.entry[2].commit == AMS_FAULT_LOG_ENTRY_COMMIT);
    CHECK(snapshot.entry[2].crc32 != 0u);

    /* Model reset after a destination was invalidated but before its final
     * commit word was published. The partial record must disappear while the
     * older and newer committed records remain ordered. */
    ams_safety_host_fault_log_invalidate_entry(1u);
    CHECK(ams_fault_log_snapshot(&snapshot));
    CHECK(snapshot.count == 2u);
    CHECK(snapshot.write_index == 2u);
    CHECK(snapshot.next_sequence == 4u);
    CHECK(snapshot.entry[0].event == AMS_FAULT_LOG_BOOT);
    CHECK(snapshot.entry[0].sequence == 1u);
    CHECK(snapshot.entry[1].event == AMS_FAULT_LOG_CURRENT_LATCH);
    CHECK(snapshot.entry[1].sequence == 3u);

    ams_fault_log_event(AMS_FAULT_LOG_TEMP_LATCH, 40u, 41u, 42u);
    CHECK(ams_fault_log_snapshot(&snapshot));
    CHECK(snapshot.count == 3u);
    CHECK(snapshot.entry[2].sequence == 4u);

    /* A committed marker with a damaged payload/CRC is not trusted. */
    ams_safety_host_fault_log_corrupt_entry_crc(0u);
    CHECK(ams_fault_log_snapshot(&snapshot));
    CHECK(snapshot.count == 2u);
    CHECK(snapshot.entry[0].sequence == 3u);
    CHECK(snapshot.entry[1].sequence == 4u);
    CHECK(snapshot.next_sequence == 5u);

    /* Header progress may lag or tear independently of an entry commit. The
     * sequence scan must restore it without discarding valid records. */
    ams_safety_host_fault_log_corrupt_metadata(UINT32_MAX,
                                               UINT32_MAX,
                                               0u);
    CHECK(ams_fault_log_snapshot(&snapshot));
    CHECK(snapshot.count == 2u);
    CHECK(snapshot.write_index == 2u);
    CHECK(snapshot.next_sequence == 5u);

    ams_fault_log_clear();
    for(uint32_t index = 0u; index < (AMS_FAULT_LOG_DEPTH + 8u); index++)
    {
        ams_fault_log_event(AMS_FAULT_LOG_BOOT,
                            (uint16_t)index,
                            index,
                            ~index);
    }
    CHECK(ams_fault_log_snapshot(&snapshot));
    CHECK(snapshot.count == AMS_FAULT_LOG_DEPTH);
    CHECK(snapshot.next_sequence == (AMS_FAULT_LOG_DEPTH + 9u));
    for(uint32_t offset = 0u; offset < snapshot.count; offset++)
    {
        uint32_t ring_index =
            (snapshot.write_index + AMS_FAULT_LOG_DEPTH - snapshot.count +
             offset) % AMS_FAULT_LOG_DEPTH;
        CHECK(snapshot.entry[ring_index].reason == (uint16_t)(offset + 8u));
        CHECK(snapshot.entry[ring_index].sequence == (offset + 9u));
        CHECK(snapshot.entry[ring_index].commit ==
              AMS_FAULT_LOG_ENTRY_COMMIT);
    }
}

static void sil_make_measurement_gates_ready(app_data_t *d)
{
    d->voltage_valid = true;
    d->voltage_read_fault = false;
    d->voltage_fault = false;
    d->current_valid = true;
    d->current_fault = false;
    d->current_sensor_fault = false;
    d->temp_valid = true;
    d->temp_read_fault = false;
    d->temp_fault = false;
    d->adbms_diag_fault = false;
    d->charger_fault = false;
    d->fuse_fault = false;
    d->imd_valid = true;
    d->imd_ok = true;
    d->imd_fault = false;
    d->imd_status = IMD_NORMAL;
}

static void sil_make_watchdog_ready(app_data_t *d)
{
    sil_make_measurement_gates_ready(d);
    d->state = STATE_DISCARGE;
    d->current_fault_mode = CURRENT_FAULT_MODE_DRIVE;
    d->heartbeat.boot_tick = 0u;
    sil_mark_all_heartbeats_alive(d);
    (void)ams_heartbeat_update(d, fake_tick);
}

static void test_watchdog_feed_gate(void)
{
    init_fake_app();
    fake_tick = AMS_HEARTBEAT_STARTUP_GRACE_MS + 100u;
    app.heartbeat.boot_tick = 0u;
    ams_safety_watchdog_enable_runtime(&app, true);
    ams_safety_watchdog_task_update(&app);

#if AMS_ENABLE_IWDG
    CHECK(app.watchdog_runtime_enabled == true);
    CHECK(app.watchdog_hw_started == true);
    CHECK(app.watchdog_feed_count == 0u);
    CHECK(app.watchdog_last_block_reason != AMS_WATCHDOG_BLOCK_NONE);

    sil_make_watchdog_ready(&app);
    ams_safety_watchdog_task_update(&app);
    CHECK(app.watchdog_hw_started == true);
    CHECK(app.watchdog_feed_count == 1u);
    CHECK(app.watchdog_last_block_reason == AMS_WATCHDOG_BLOCK_NONE);

    app.fuse_fault = true;
    ams_safety_watchdog_task_update(&app);
    CHECK(app.watchdog_feed_count == 1u);
    CHECK(app.watchdog_last_block_reason == AMS_WATCHDOG_BLOCK_HARD_FAULT);
    app.fuse_fault = false;

    app.task_heartbeat_fault = true;
    app.heartbeat.safety_stale_mask = AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_ADBMS);
    ams_safety_watchdog_task_update(&app);
    CHECK(app.watchdog_feed_count == 1u);
    CHECK(app.watchdog_block_count >= 1u);
    CHECK(app.watchdog_last_block_reason == AMS_WATCHDOG_BLOCK_HEARTBEAT);
#else
    CHECK(app.watchdog_runtime_enabled == false);
    CHECK(app.watchdog_feed_count == 0u);
    CHECK(app.watchdog_last_block_reason == AMS_WATCHDOG_BLOCK_NOT_ENABLED);
#endif
}

static void test_watchdog_boot_arm_and_startup_grace(void)
{
    init_fake_app();
    fake_tick = 100u;
    ams_heartbeat_init(&app, fake_tick);
    ams_safety_watchdog_boot_arm(&app);
    ams_safety_watchdog_task_update(&app);

#if AMS_ENABLE_IWDG
    CHECK(app.watchdog_runtime_enabled == true);
    CHECK(app.watchdog_hw_started == true);
    CHECK(app.watchdog_feed_count == 1u);
    CHECK(app.watchdog_last_feed_tick == fake_tick);
    CHECK(app.watchdog_last_block_reason == AMS_WATCHDOG_BLOCK_STARTUP_GRACE);

    fake_tick += AMS_HEARTBEAT_STARTUP_GRACE_MS + 1u;
    app.hard_fault = true;
    ams_safety_watchdog_task_update(&app);
    CHECK(app.watchdog_feed_count == 1u);
    CHECK(app.watchdog_last_block_reason == AMS_WATCHDOG_BLOCK_HARD_FAULT);
#else
    CHECK(app.watchdog_runtime_enabled == false);
    CHECK(app.watchdog_hw_started == false);
    CHECK(app.watchdog_feed_count == 0u);
    CHECK(app.watchdog_last_block_reason == AMS_WATCHDOG_BLOCK_NOT_ENABLED);
#endif
}

static void test_watchdog_start_failure_is_fail_closed(void)
{
#if AMS_ENABLE_IWDG
    init_fake_app();

    /* Recreate a boot where the target's IWDG status bits never complete the
     * start handshake.  This host-only flag models that register-level fault. */
    ams_safety_host_reset_state();
    app.watchdog_runtime_enabled = false;
    app.watchdog_hw_started = false;
    app.watchdog_feed_count = 0u;
    app.watchdog_block_count = 0u;
    app.watchdog_last_logged_block_reason = AMS_WATCHDOG_BLOCK_NONE;
    g_host_watchdog_start_fail = true;

    fake_tick = AMS_HEARTBEAT_STARTUP_GRACE_MS + 100u;
    sil_make_watchdog_ready(&app);
    ams_safety_watchdog_boot_arm(&app);

    CHECK(app.watchdog_runtime_enabled == true);
    CHECK(app.watchdog_hw_started == false);
    CHECK(ams_safety_watchdog_ok(&app) == false);

    set_bms(true);
    error_task_update(&app, fake_tick);
    CHECK(app.bms_state == false);
    CHECK(app.watchdog_feed_count == 0u);
    CHECK(app.watchdog_block_count == 1u);
    CHECK(app.watchdog_last_block_reason == AMS_WATCHDOG_BLOCK_START_FAILED);

    /* Once the modeled hardware fault clears, the next healthy supervisor
     * update starts and feeds the watchdog normally. */
    g_host_watchdog_start_fail = false;
    ams_safety_watchdog_task_update(&app);
    CHECK(app.watchdog_hw_started == true);
    CHECK(app.watchdog_feed_count == 1u);
    CHECK(app.watchdog_last_block_reason == AMS_WATCHDOG_BLOCK_NONE);
#endif
}

static void test_rtos_stack_heap_diag_and_faults(void)
{
    init_fake_app();
    app.error_task = (TaskHandle_t)0x1000u;
    app.current_task = (TaskHandle_t)0x1001u;
    app.adbms_task = (TaskHandle_t)0x1002u;
    app.canbus_task = (TaskHandle_t)0x1003u;
    app.estimator_task = (TaskHandle_t)0x1004u;
    app.fan_task = (TaskHandle_t)0x1005u;
    app.cli_task = (TaskHandle_t)0x1006u;

    ams_rtos_host_set_heap(1024u, 1024u);
    ams_rtos_host_set_stack_high_water(AMS_RTOS_TASK_ERROR, 160u);
    ams_rtos_host_set_stack_high_water(AMS_RTOS_TASK_CURRENT, 140u);
    ams_rtos_host_set_stack_high_water(AMS_RTOS_TASK_ADBMS, 64u);
    ams_rtos_host_set_stack_high_water(AMS_RTOS_TASK_CAN, 180u);
    ams_rtos_host_set_stack_high_water(AMS_RTOS_TASK_ESTIMATOR, 220u);
    ams_rtos_host_set_stack_high_water(AMS_RTOS_TASK_FAN, 120u);
    ams_rtos_host_set_stack_high_water(AMS_RTOS_TASK_CLI, 200u);

    ams_rtos_diag_update(&app);
    CHECK(app.rtos_heap_warning == true);
    CHECK(app.rtos_stack_warning == true);
    CHECK(app.rtos_fault == false);
    CHECK(app.rtos_min_stack_high_water_words == 64u);
    CHECK((app.rtos_stack_warn_mask & AMS_RTOS_TASK_BIT(AMS_RTOS_TASK_ADBMS)) != 0u);
    CHECK((app.rtos_fault_flags & AMS_RTOS_FAULT_FLAG_LOW_HEAP_WARN) != 0u);
    CHECK((app.rtos_fault_flags & AMS_RTOS_FAULT_FLAG_LOW_STACK_WARN) != 0u);

    sil_make_watchdog_ready(&app);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    run_one_error_task_iteration(&app);
    CHECK(app.soft_fault == true);
    CHECK(app.hard_fault == false);
    CHECK(app.bms_state == true);

    init_fake_app();
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    fake_tick = 77u;
    ams_rtos_set_fault(&app, AMS_RTOS_FAULT_STACK_OVERFLOW, AMS_RTOS_TASK_CLI, 0u);
    CHECK(app.rtos_fault == true);
    CHECK(app.rtos_stack_overflow_count == 1u);
    CHECK(app.rtos_last_fault_reason == AMS_RTOS_FAULT_STACK_OVERFLOW);
    CHECK(app.rtos_last_fault_task == AMS_RTOS_TASK_CLI);
    run_one_error_task_iteration(&app);
    CHECK(app.hard_fault == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    init_fake_app();
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    ams_rtos_assert_failed("host", 1234);
    CHECK(app.rtos_fault == true);
    CHECK(app.rtos_assert_fail_count == 1u);
    CHECK(app.rtos_last_assert_line == 1234u);
    CHECK(app.rtos_last_fault_reason == AMS_RTOS_FAULT_ASSERT_FAILED);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    CHECK(ams_safety_panic_active() == true);
}

static void test_can_busoff_sets_fault_and_recovers(void)
{
    static CAN_HandleTypeDef hcan;

    init_fake_app();
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.state = STATE_CHARGE;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    fake_tick = 100u;
    fake_can_error = HAL_CAN_ERROR_BOF;

    canbus_poll_errors(&app.board.canbus, &app);
    CHECK(app.canbus_fault == true);
    CHECK(app.can_busoff_fault == true);
    CHECK(app.can_recover_pending == true);
    CHECK(app.can_busoff_count == 1u);
    CHECK(app.charger_fault == true);
    CHECK(app.board.charger.communication_fail == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    fake_tick += AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS + 1u;
    canbus_poll_errors(&app.board.canbus, &app);
    CHECK(app.can_busoff_fault == false);
    CHECK(app.can_recover_pending == false);
    CHECK(app.can_recover_count == 1u);
    CHECK(app.can_error_code == HAL_CAN_ERROR_NONE);
    CHECK(app.canbus_fault == false);
    CHECK(fake_can_error == HAL_CAN_ERROR_NONE);
	CHECK(app.board.canbus.started == true);
	CHECK(app.board.canbus.notification_active == true);

    init_fake_app();
    app.board.canbus.hcan = &hcan;
    app.state = STATE_CHARGE;
    fake_tick = 1000u;
    fake_can_error = HAL_CAN_ERROR_BOF;
    fake_can_recover_status = HAL_ERROR;
    canbus_poll_errors(&app.board.canbus, &app);
    uint32_t first_recovery_tick = app.can_last_error_tick;
    fake_tick += AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS + 1u;
    canbus_poll_errors(&app.board.canbus, &app);
    CHECK(app.can_busoff_fault == true);
    CHECK(app.can_recover_pending == true);
    CHECK(app.can_recover_count == 0u);
    CHECK(app.canbus_fault == true);
	CHECK(app.board.canbus.started == false);
	CHECK(app.board.canbus.notification_active == false);
    CHECK(app.can_last_error_tick > first_recovery_tick);
    uint32_t failed_recovery_tick = app.can_last_error_tick;
    fake_tick += (AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS / 2u);
    canbus_poll_errors(&app.board.canbus, &app);
    CHECK(app.can_last_error_tick == failed_recovery_tick);

	fake_can_recover_status = HAL_OK;
	fake_can_notification_status = HAL_ERROR;
    fake_tick = failed_recovery_tick + AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS + 1u;
    canbus_poll_errors(&app.board.canbus, &app);
	CHECK(app.can_busoff_fault == true);
	CHECK(app.can_recover_pending == true);
	CHECK(app.can_recover_count == 0u);
	CHECK(app.canbus_fault == true);
	CHECK(app.board.canbus.started == true);
	CHECK(app.board.canbus.notification_active == false);
	failed_recovery_tick = app.can_last_error_tick;

	fake_can_notification_status = HAL_OK;
	fake_tick = failed_recovery_tick + AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS + 1u;
	canbus_poll_errors(&app.board.canbus, &app);
    CHECK(app.can_busoff_fault == false);
    CHECK(app.can_recover_pending == false);
    CHECK(app.can_recover_count == 1u);
    CHECK(app.canbus_fault == false);
	CHECK(app.board.canbus.started == true);
	CHECK(app.board.canbus.notification_active == true);

    init_fake_app();
    app.board.canbus.hcan = &hcan;
    fake_tick = 5000u;
    fake_can_error = HAL_CAN_ERROR_ACK;
    canbus_poll_errors(&app.board.canbus, &app);
    CHECK(app.canbus_fault == true);
    CHECK(app.can_error_code == HAL_CAN_ERROR_ACK);
    CHECK(app.can_error_count == 1u);
    CHECK(fake_can_error == HAL_CAN_ERROR_NONE);
    fake_tick += (AMS_CAN_ERROR_SOFT_HOLD_MS / 2u);
    canbus_poll_errors(&app.board.canbus, &app);
    CHECK(app.canbus_fault == true);
    fake_tick += AMS_CAN_ERROR_SOFT_HOLD_MS + 1u;
    canbus_poll_errors(&app.board.canbus, &app);
    CHECK(app.canbus_fault == false);
}

static void test_fault_matrix_extra(void){
    static CAN_HandleTypeDef hcan;
    static ADC_HandleTypeDef adc1;

    // 1) Overvoltage must hard-disable BMS and clear balancing.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state=STATE_CHARGE; app.current_valid=true; app.bms_state=true; bms_pin_state=GPIO_PIN_SET; app.acc.smb_ics[1].cell.c_codes[4]=code_for_volts(4.250f);
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET);
    sil_expect_balancing_clear(&app);

    // 1b) Failure to clear balance before voltage measurement must fail closed.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state=STATE_CHARGE; app.current_valid=true; app.bms_state=true; bms_pin_state=GPIO_PIN_SET; fake_adbms_wrcfgb_status = HAL_ERROR;
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_diag_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET);
    fake_adbms_wrcfgb_status = HAL_OK;

    // 2) All invalid cell readings must fail safe. A pack with no valid cell data cannot be considered safe.
    init_fake_app(); app.acc.smb.num_ics=NSMBS; app.acc.smb.ics=app.acc.smb_ics; app.state=STATE_CHARGE; app.current_valid=true; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
    for(int ic=0; ic<NSMBS; ic++) for(int c=0;c<NCELLS;c++) app.acc.smb_ics[ic].cell.c_codes[c] = 0;
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET);

    // 3) High temperature must hard-disable BMS and prevent balancing. Drive
    // the policy with the real production scan period; this test must not
    // accidentally rely on the 1 Hz hardware-bring-up profile.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); sil_set_all_temps(&app, 66.0f, (1UL << NTEMPS) - 1UL); app.state=STATE_CHARGE; app.current_valid=true; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
    for(uint32_t guard = 0u;
        !app.temp_fault &&
        (guard <= ((TEMP_HOT_HARD_CONFIRM_MS / AMS_ADBMS_TASK_PERIOD_MS) + 1u));
        guard++)
    {
        temperature_fault_update_with_period(&app.temp_fault_state,
                                             &app.acc,
                                             AMS_ADBMS_TASK_PERIOD_MS);
        sil_copy_temp_state(&app);
    }
    run_one_adbms_task_iteration(&app);
    CHECK(app.temp_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET);
    sil_expect_balancing_clear(&app);

    // 4) Charger-reported faults must command disable in charge state.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan=&hcan; app.state=STATE_CHARGE; app.current_valid=true; app.bms_state=true; app.board.charger.last_rx_tick=1000; fake_tick=1000;
    app.board.charger.hardware_fail=true; app.board.charger.overtemp_fail=false; app.board.charger.input_volt_fail=false; app.board.charger.voltage_sense_fail=false; tx_count=0; tx_free_level=3; bms_pin_state=GPIO_PIN_SET;
    run_one_canbus_task_iteration(&app);
    CHECK(app.charger_fault == true); CHECK(app.bms_state == false); CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT); CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 1u);

    // 5) Pre-existing hard fault must command charger disable even without charger self-fault.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan=&hcan; app.state=STATE_CHARGE; app.current_valid=true; app.bms_state=true; app.hard_fault=true; app.board.charger.last_rx_tick=1000; fake_tick=1000; tx_count=0; tx_free_level=3;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT); CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 1u); CHECK(app.bms_state == false);

    // 6) CAN transmit failure must become a soft CAN fault, not silently pass.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan=&hcan; app.state=STATE_DISCARGE; tx_count=0; tx_free_level=0; fake_tick=0;
    run_one_canbus_task_iteration(&app);
    CHECK(app.canbus_fault == true);

    // 7) Fan driver failure must set fan soft fault, then error task should mark soft fault only.
    init_fake_app(); sil_make_measurement_gates_ready(&app); app.state=STATE_DISCARGE; app.current_fault_mode=CURRENT_FAULT_MODE_DRIVE; app.temp_usable_sensor_count = AMS_EXPECTED_TEMP_SENSOR_COUNT; app.max_temp = TEMP_FAN_MAX_C + 5.0f; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
    // Leave fan CCR pointers NULL, so set_fan_percent() fails.
    run_one_fan_task_iteration(&app);
    CHECK(app.fan_fault == true);
    run_one_error_task_iteration(&app);
    CHECK(app.soft_fault == true); CHECK(app.hard_fault == false); CHECK(app.bms_state == true);

    // 8) Current ADC missing is allowed one transient sample, then becomes a soft sensor fault after confirmation.
    init_fake_app(); app.temp_valid=true; app.temp_fault=false; app.temp_read_fault=false; app.board.current_sensor.hadc_high=&adc1; app.board.current_sensor.hadc_low=NULL; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
    run_one_current_task_iteration(&app);
    CHECK(app.current_fault == false);
    for(int i = 0; i < 25; i++)
    {
        run_one_current_task_iteration(&app);
    }
    CHECK(app.current_sensor_fault == true);
    CHECK(app.current_fault == true);
    run_one_error_task_iteration(&app);
    CHECK(app.soft_fault == true); CHECK(app.hard_fault == false); CHECK(app.bms_state == false);

    // 9) Error task hard-fault aggregation must drop BMS for each hard fault source.
    init_fake_app(); app.temp_fault=true; app.bms_state=true; bms_pin_state=GPIO_PIN_SET; run_one_error_task_iteration(&app); CHECK(app.hard_fault==true && app.bms_state==false);
    init_fake_app(); app.voltage_fault=true; app.bms_state=true; bms_pin_state=GPIO_PIN_SET; run_one_error_task_iteration(&app); CHECK(app.hard_fault==true && app.bms_state==false);
    init_fake_app(); app.charger_fault=true; app.bms_state=true; bms_pin_state=GPIO_PIN_SET; run_one_error_task_iteration(&app); CHECK(app.hard_fault==true && app.bms_state==false);
    init_fake_app(); app.fuse_fault=true; app.bms_state=true; bms_pin_state=GPIO_PIN_SET; run_one_error_task_iteration(&app); CHECK(app.hard_fault==true && app.bms_state==false);
}


static void test_voltage_stats_boundaries_and_fuzz(void){
    CHECK(isfinite(convert_adc_to_volt(INT_MAX)));
    CHECK(isfinite(convert_adc_to_volt(INT_MIN)));

    init_fake_app();
    app.acc.smb.num_ics = 0;
    accumulator_update_voltage_stats(&app.acc);
    CHECK(app.acc.valid_voltage_count == 0u);
    CHECK(app.acc.max_volt == 0.0f && app.acc.min_volt == 0.0f && app.acc.total_volt == 0.0f);
    CHECK(accumulator_set_balance(&app.acc) == -1);

    init_fake_app();
    app.acc.smb.num_ics = NSMBS;
    for(int ic=0; ic<NSMBS; ic++){
        for(int c=0; c<NCELLS; c++){
            int selector = (ic * NCELLS + c) % 5;
            if(selector == 0) app.acc.smb_ics[ic].cell.c_codes[c] = 0;
            else if(selector == 1) app.acc.smb_ics[ic].cell.c_codes[c] = INT16_MIN;
            else if(selector == 2) app.acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(0.49f);
            else if(selector == 3) app.acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(5.01f);
            else app.acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(3.50f + 0.001f * (float)c);
        }
    }
    host_mark_updated_cells(&app);
    accumulator_update_voltage_stats(&app.acc);
    CHECK(app.acc.valid_voltage_count == 30u);
    CHECK(fabsf(app.acc.min_volt - 1.500f) < 0.002f);
    CHECK(app.acc.max_volt > 3.49f && app.acc.max_volt < 3.52f);

    init_fake_app();
    app.acc.smb.num_ics = NSMBS;
    app.acc.smb.ics = app.acc.smb_ics;
    float expected_min = 99.0f, expected_max = -99.0f, expected_total = 0.0f;
    unsigned expected_valid = 0u;
    uint32_t lcg = 0x12345678u;
    for(int ic=0; ic<NSMBS; ic++){
        for(int c=0; c<NCELLS; c++){
            lcg = 1664525u * lcg + 1013904223u;
            int16_t code;
            switch((lcg >> 29) & 7u){
                case 0: code = 0; break;
                case 1: code = INT16_MIN; break;
                case 2: code = code_for_volts(0.1f); break;
                case 3: code = code_for_volts(6.0f); break;
                default: {
                    float v = 2.8f + (float)(lcg & 0x3ffu) / 1023.0f * 1.3f;
                    code = code_for_volts(v);
                    break;
                }
            }
            app.acc.smb_ics[ic].cell.c_codes[c] = code;
            float v = convert_adc_to_volt(code);
            if(code != INT16_MIN && v >= 0.5f && v <= 5.0f){
                if(v < expected_min) expected_min = v;
                if(v > expected_max) expected_max = v;
                expected_total += v;
                expected_valid++;
            }
        }
    }
    host_mark_updated_cells(&app);
    accumulator_update_voltage_stats(&app.acc);
    CHECK(app.acc.valid_voltage_count == expected_valid);
    CHECK(fabsf(app.acc.min_volt - expected_min) < 0.002f);
    CHECK(fabsf(app.acc.max_volt - expected_max) < 0.002f);
    CHECK(fabsf(app.acc.total_volt - expected_total) < 0.050f);

    app.acc.smb.num_ics = 255;
    accumulator_update_voltage_stats(&app.acc);
    CHECK(app.acc.valid_voltage_count == 0u);
    CHECK(app.acc.voltage_full_usable == false);
}


static void test_voltage_fault_policy_and_strict_scan_freshness(void){
    init_fake_app();
    fake_tick = 0u;
    fill_nominal_pack(&app, 3.700f);
    voltage_fault_update(&app.voltage_fault_state, &app.acc);
    CHECK(app.voltage_fault_state.voltage_valid == true);
    CHECK(app.voltage_fault_state.read_fault == false);
    CHECK(app.voltage_fault_state.reason == VOLTAGE_FAULT_REASON_NONE);

    /* ADBMS code zero is 1.500 V. It is a valid, severe-UV measurement,
     * not stale/reset data, and must enter the latched voltage-fault path. */
    init_fake_app(); fill_nominal_pack(&app, 3.700f);
    app.acc.smb_ics[0].cell.c_codes[0] = 0;
    host_mark_updated_cells(&app);
    accumulator_update_voltage_stats_at(&app.acc, 0u);
    voltage_fault_update(&app.voltage_fault_state, &app.acc);
    CHECK(app.acc.cell_voltage_mv[0][0] == 1500u);
    CHECK(app.acc.voltage_full_updated == true);
    CHECK(app.acc.voltage_full_usable == true);
    CHECK(app.voltage_fault_state.voltage_valid == true);
    CHECK(app.voltage_fault_state.undervoltage_fault == true);
    CHECK(app.voltage_fault_state.latched == true);
    CHECK(app.voltage_fault_state.reason == VOLTAGE_FAULT_REASON_UV_SEVERE);

    /* One noisy/PEC-failed group remains visible in telemetry but fails closed immediately. */
    fake_tick = 1000u;
    host_mark_updated_cells(&app);
    app.acc.smb.last_cell_updated_mask[0] &= (uint16_t)~0x0001u;
    app.acc.smb.last_cell_pec_mask[0] = 0x0001u;
    accumulator_update_voltage_stats_at(&app.acc, fake_tick);
    voltage_fault_update(&app.voltage_fault_state, &app.acc);
    CHECK(app.acc.usable_voltage_count == AMS_EXPECTED_CELL_COUNT);
    CHECK(app.acc.updated_voltage_count == (AMS_EXPECTED_CELL_COUNT - 1u));
    CHECK(app.voltage_fault_state.voltage_valid == false);
    CHECK(app.voltage_fault_state.read_fault == true);
    CHECK(app.voltage_fault_state.warning == false);
    CHECK(app.voltage_fault_state.confirmed == true);
    CHECK(app.voltage_fault_state.reason == VOLTAGE_FAULT_REASON_PEC_FAILURE);

    /* Persistent missing data stays fail-closed and eventually ages the stale mask. */
    for(fake_tick = 2000u; fake_tick <= 3000u; fake_tick += 1000u)
    {
        host_mark_updated_cells(&app);
        app.acc.smb.last_cell_updated_mask[0] &= (uint16_t)~0x0001u;
        app.acc.smb.last_cell_pec_mask[0] = 0x0001u;
        accumulator_update_voltage_stats_at(&app.acc, fake_tick);
    }
    voltage_fault_update(&app.voltage_fault_state, &app.acc);
    CHECK(app.acc.usable_voltage_count == (AMS_EXPECTED_CELL_COUNT - 1u));
    CHECK(app.voltage_fault_state.read_fault == true);
    CHECK(app.voltage_fault_state.reason == VOLTAGE_FAULT_REASON_PEC_FAILURE);

    init_fake_app(); fill_nominal_pack(&app, 3.700f);
    app.acc.smb_ics[0].cell.c_codes[0] = code_for_volts(4.180f);
    host_mark_updated_cells(&app);
    accumulator_update_voltage_stats_at(&app.acc, 0u);
    voltage_fault_update(&app.voltage_fault_state, &app.acc);
    CHECK(app.voltage_fault_state.charge_stop == true);
    CHECK(app.voltage_fault_state.overvoltage_fault == false);
    CHECK(app.voltage_fault_state.latched == false);

    init_fake_app(); fill_nominal_pack(&app, 3.700f);
    app.acc.smb_ics[1].cell.c_codes[4] = code_for_volts(4.200f);
    host_mark_updated_cells(&app);
    accumulator_update_voltage_stats_at(&app.acc, 0u);
    voltage_fault_update(&app.voltage_fault_state, &app.acc);
    CHECK(app.voltage_fault_state.overvoltage_fault == true);
    CHECK(app.voltage_fault_state.latched == true);
    CHECK(app.voltage_fault_state.latched_reason == VOLTAGE_FAULT_REASON_OV_HARD);

    init_fake_app(); fill_nominal_pack(&app, 3.700f);
    app.acc.smb_ics[2].cell.c_codes[5] = code_for_volts(2.500f);
    host_mark_updated_cells(&app);
    accumulator_update_voltage_stats_at(&app.acc, 0u);
    voltage_fault_update(&app.voltage_fault_state, &app.acc);
    CHECK(app.voltage_fault_state.undervoltage_fault == true);
    CHECK(app.voltage_fault_state.latched == true);
    CHECK(app.voltage_fault_state.latched_reason == VOLTAGE_FAULT_REASON_UV_HARD);
}

static void test_system_sil_boot_ready_and_bms_conjunction(void)
{
    init_fake_app();
    fake_tick = 0u;
    bms_pin_state = GPIO_PIN_RESET;
    app.state = STATE_DISCARGE;
    sil_attach_current_adcs(&app);
    fill_nominal_pack(&app, 3.700f);
    sil_clear_voltage_history(&app);

    sil_run_current_sample(&app, 0.0f);
    CHECK(app.current_valid == true);
    CHECK(app.current_fault == false);
    CHECK(app.bms_state == false);

    fake_adbms_voltage_masks_all_missing(false);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == false);
    CHECK(app.voltage_read_fault == true);
    CHECK(app.voltage_fault == true);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_NOT_READY);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    fake_adbms_voltage_masks_full_update();
    sil_run_current_sample(&app, 0.0f);
    sil_run_voltage_sample(&app);
    CHECK(app.current_valid == true);
    CHECK(app.voltage_valid == true);
    CHECK(app.current_fault == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);

    init_fake_app();
    fake_tick = 0u;
    app.state = STATE_DISCARGE;
    sil_attach_current_adcs(&app);
    fill_nominal_pack(&app, 3.700f);
    fake_adbms_voltage_masks_full_update();
    sil_run_current_adc_status(&app, HAL_TIMEOUT, HAL_OK);
    CHECK(app.current_valid == false);
    CHECK(app.current_fault == false);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    init_fake_app();
    fake_tick = 0u;
    app.state = STATE_DISCARGE;
    sil_attach_current_adcs(&app);
    fill_nominal_pack(&app, 3.700f);
    sil_clear_voltage_history(&app);
    fake_adbms_voltage_masks_all_missing(true);
    sil_run_current_sample(&app, 0.0f);
    sil_run_voltage_sample(&app);
    CHECK(app.current_valid == true);
    CHECK(app.voltage_valid == false);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
}

static void test_system_sil_single_pec_miss_drops_bms_then_recovers(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    fake_adbms_voltage_masks_one_missing(0u, 0u, true);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == false);
    CHECK(app.voltage_read_fault == true);
    CHECK(app.voltage_warning == false);
    CHECK(app.voltage_fault == true);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_PEC_FAILURE);
    CHECK(app.voltage_usable_cell_count == AMS_EXPECTED_CELL_COUNT);
    CHECK(app.voltage_updated_cell_count == (AMS_EXPECTED_CELL_COUNT - 1u));
    CHECK(app.voltage_stale_cell_count == 0u);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.voltage_warning == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_NONE);
    CHECK(app.voltage_updated_cell_count == AMS_EXPECTED_CELL_COUNT);
    CHECK(app.bms_state == true);
}

static void test_system_sil_persistent_voltage_stale_drops_bms_ok(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    fake_adbms_voltage_masks_one_missing(1u, 2u, true);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == false);
    CHECK(app.voltage_read_fault == true);
    CHECK(app.voltage_fault == true);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_PEC_FAILURE);
    CHECK(app.bms_state == false);

    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == false);
    CHECK(app.voltage_fault == true);
    CHECK(app.bms_state == false);

    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == false);
    CHECK(app.voltage_read_fault == true);
    CHECK(app.voltage_fault == true);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_PEC_FAILURE);
    CHECK(app.voltage_usable_cell_count == (AMS_EXPECTED_CELL_COUNT - 1u));
    CHECK(app.voltage_stale_cell_count == 1u);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.voltage_fault == false);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);
}

static void test_system_sil_charge_stop_allows_balance_before_hard_ov(void)
{
    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.balance_inhibit = false;

    sil_set_cell_voltage(&app, 0u, 0u, 4.100f);
    sil_set_cell_voltage(&app, 0u, 1u, 4.180f);
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.charge_voltage_stop == true);
    CHECK(app.overvoltage_fault == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.voltage_fault_latched == false);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_CHARGE_STOP);
    CHECK(app.bms_state == true);
    CHECK(app.acc.smb_ics[0].tx_cfgb.dcc == 0u);
    CHECK(sil_balance_pwm_duty(&app, 0u, 0u) == 0u);
    CHECK(sil_balance_pwm_duty(&app, 0u, 1u) == BALANCE_PWM_DUTY);

    app.balance_inhibit = true;
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);
    sil_expect_balancing_clear(&app);
    app.balance_inhibit = false;

    sil_set_cell_voltage(&app, 0u, 1u, 4.200f);
    sil_run_voltage_sample(&app);
    CHECK(app.overvoltage_fault == true);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.voltage_fault_latched_reason == VOLTAGE_FAULT_REASON_OV_HARD);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    sil_expect_balancing_clear(&app);

    sil_set_cell_voltage(&app, 0u, 0u, 3.700f);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.bms_state == false);
}

static void test_system_sil_balance_inhibit_ladder_bringup_lockout(void)
{
    char *balance_status_argv[] = {"balance", "status", NULL};
    char *balance_inhibit_argv[] = {"balance", "inhibit", NULL};
    char *balance_release_argv[] = {"balance", "release", NULL};
    char *balance_clear_argv[] = {"balance", "clear", NULL};

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);

#if AMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT
    CHECK(app.balance_inhibit == true);
#else
    CHECK(app.balance_inhibit == false);
#endif

    app.balance_inhibit = false;
    sil_set_cell_voltage(&app, 0u, 0u, 4.100f);
    sil_set_cell_voltage(&app, 0u, 1u, 4.180f);
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);
    CHECK(app.charge_voltage_stop == true);
    CHECK(sil_balance_pwm_duty(&app, 0u, 1u) == BALANCE_PWM_DUTY);

    app.balance_inhibit = true;
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);
    CHECK(app.charge_voltage_stop == true);
    sil_expect_balancing_clear(&app);

    app.acc.smb_ics[0].PwmA.pwma[0] = BALANCE_PWM_DUTY;
    app.acc.smb_ics[0].PwmB.pwmb[0] = BALANCE_PWM_DUTY;
    app.acc.smb_ics[0].tx_cfgb.dcc = 0x0003u;
    fake_adbms_lock_depth = 0u;
    fake_adbms_lock_max_depth = 0u;
    sil_prepare_cli_capture();
    CHECK(balance_control(2, balance_clear_argv) == 0);
    CHECK(fake_adbms_lock_depth == 0u);
    CHECK(fake_adbms_lock_max_depth >= 2u);
    CHECK(strstr(cli_capture, "Balancing PWM/DCC cleared") != NULL);
    sil_expect_balancing_clear(&app);

    CHECK(balance_control(2, balance_release_argv) == 0);
    CHECK(app.balance_inhibit == false);
    sil_run_voltage_sample(&app);
    CHECK(sil_balance_pwm_duty(&app, 0u, 1u) == BALANCE_PWM_DUTY);

    app.acc.smb_ics[0].PwmA.pwma[1] = BALANCE_PWM_DUTY;
    fake_adbms_wrpwm_status = HAL_ERROR;
    sil_prepare_cli_capture();
    CHECK(balance_control(2, balance_inhibit_argv) == 0);
    CHECK(app.balance_inhibit == true);
    CHECK(strstr(cli_capture, "WARNING clear write failed") != NULL);
    CHECK(app.acc.smb_ics[0].PwmA.pwma[1] == 0u);
    fake_adbms_wrpwm_status = HAL_OK;

    sil_prepare_cli_capture();
    CHECK(balance_control(2, balance_status_argv) == 0);
    CHECK(strstr(cli_capture, "balance inhibit:1") != NULL);
    CHECK(strstr(cli_capture, "resistor-ladder/bench") != NULL);
}

static void test_system_sil_bench_cli_abuse_and_balance_idempotence(void)
{
    char *balance_bad[] = {"balance", "wat", NULL};
    char *balance_disable[] = {"balance", "disable", NULL};
    char *balance_enable[] = {"balance", "enable", NULL};
    char *balance_clear[] = {"balance", "clear", NULL};
    char *bmsok_bad[] = {"bmsok", "wat", NULL};
    char *bmsok_status[] = {"bmsok", "status", NULL};
    char *state_bad[] = {"state", "launch", NULL};
    char *state_charge[] = {"state", "charge", NULL};
    char *state_discharge[] = {"state", "discharge", NULL};

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.balance_inhibit = false;
    app.bms_output_inhibit = true;
    bms_pin_state = GPIO_PIN_RESET;

    sil_prepare_cli_capture();
    CHECK(balance_control(2, balance_bad) == 0);
    CHECK(strstr(cli_capture, "Usage: balance") != NULL);
    CHECK(app.balance_inhibit == false);

    sil_prepare_cli_capture();
    CHECK(bmsok_control(2, bmsok_bad) == 0);
    CHECK(strstr(cli_capture, "Usage: bmsok") != NULL);
    CHECK(app.bms_output_inhibit == true);

    sil_prepare_cli_capture();
    CHECK(bmsok_control(2, bmsok_status) == 0);
    CHECK(strstr(cli_capture, "inhibit:1") != NULL);

    state_t before_state = app.state;
    sil_prepare_cli_capture();
    CHECK(set_state(2, state_bad) == 1);
    CHECK(app.state == before_state);
    CHECK(strstr(cli_capture, "Usage: state") != NULL);

    app.acc.smb_ics[0].tx_cfgb.dcc = 0x0002u;
    app.acc.smb_ics[0].PwmA.pwma[1] = BALANCE_PWM_DUTY;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    sil_prepare_cli_capture();
    CHECK(set_state(2, state_discharge) == 0);
    CHECK(app.state == STATE_DISCARGE);
    CHECK(app.state_previous == STATE_CHARGE);
    CHECK(app.state_transition_reason == AMS_STATE_TRANSITION_SERVICE_COMMAND);
    CHECK(app.state_transition_count == 1u);
    CHECK(app.state_transition_in_progress == false);
    CHECK(app.board.charger.shutdown_pending == true);
    CHECK(app.board.charger.shutdown_frames_remaining == CHARGER_EXIT_DISABLE_FRAMES);
    CHECK(app.charger_fault == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    sil_expect_balancing_clear(&app);

    app.acc.smb_ics[0].tx_cfgb.dcc = 0x0002u;
    app.acc.smb_ics[0].PwmA.pwma[1] = BALANCE_PWM_DUTY;
    fake_adbms_wrpwm_status = HAL_ERROR;
    sil_prepare_cli_capture();
    CHECK(set_state(2, state_charge) == 0);
    CHECK(app.state == STATE_CHARGE);
    CHECK(app.adbms_balance_write_fault == true);
    CHECK(app.adbms_diag_fault == true);
    CHECK(app.bms_state == false);
    CHECK(strstr(cli_capture, "balance-clear write failure") != NULL);
    sil_expect_balancing_clear(&app);

    fake_adbms_wrpwm_status = HAL_OK;
    sil_prepare_cli_capture();
    CHECK(set_state(2, state_discharge) == 0);
    CHECK(app.adbms_balance_write_fault == false);
    CHECK(app.adbms_diag_fault == false);

    for(uint8_t i = 0u; i < 12u; i++)
    {
        app.acc.smb_ics[i % NSMBS].tx_cfgb.dcc = (uint16_t)(1u << (i % NCELLS));
        app.acc.smb_ics[i % NSMBS].PwmA.pwma[i % PWMA] = BALANCE_PWM_DUTY;
        app.acc.smb_ics[i % NSMBS].PwmB.pwmb[i % PWMB] = BALANCE_PWM_DUTY;

        sil_prepare_cli_capture();
        CHECK(balance_control(2, (i & 1u) ? balance_disable : balance_clear) == 0);
        if(i & 1u)
        {
            CHECK(app.balance_inhibit == true);
        }
        sil_expect_balancing_clear(&app);

        sil_prepare_cli_capture();
        CHECK(balance_control(2, balance_enable) == 0);
        CHECK(app.balance_inhibit == false);
    }

    app.state = STATE_ERROR;
    sil_prepare_cli_capture();
    CHECK(set_state(2, state_discharge) == 1);
    CHECK(app.state == STATE_ERROR);
    CHECK(app.state_transition_in_progress == false);
    CHECK(strstr(cli_capture, "reset required") != NULL);

    ams_fault_log_t state_log;
    uint32_t state_event_count = 0u;
    CHECK(ams_fault_log_snapshot(&state_log));
    for(uint32_t i = 0u; i < AMS_FAULT_LOG_DEPTH; i++)
    {
        if(state_log.entry[i].event == AMS_FAULT_LOG_STATE_TRANSITION)
        {
            state_event_count++;
        }
    }
    CHECK(state_event_count >= 3u);
}

static void test_system_sil_bench_state_transition_balance_cleanup(void)
{
    const state_t states[] = {STATE_DISCARGE, STATE_START, STATE_BALANCE};

    for(size_t i = 0u; i < (sizeof(states) / sizeof(states[0])); i++)
    {
        sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
        app.balance_inhibit = false;
        sil_set_cell_voltage(&app, 0u, 0u, 4.100f);
        sil_set_cell_voltage(&app, 0u, 1u, 4.180f);
        fake_adbms_voltage_masks_full_update();
        sil_run_voltage_sample(&app);
        CHECK(app.bms_state == true);
        CHECK(sil_balance_pwm_duty(&app, 0u, 1u) == BALANCE_PWM_DUTY);

        app.state = states[i];
        sil_run_voltage_sample(&app);
        sil_expect_balancing_clear(&app);

        /* A mode transition must not reuse the current-policy result from the
         * prior charge state.  The supervisor holds BMS_OK low until the
         * current task has published under the new policy. */
        CHECK(app.bms_state == false);
        CHECK(bms_pin_state == GPIO_PIN_RESET);
        sil_run_current_sample(&app, 0.0f);
        sil_mark_all_heartbeats_alive(&app);
        error_task_update(&app, fake_tick);

        CHECK((states[i] != STATE_START) || (app.state == STATE_DISCARGE));
        CHECK(app.bms_state == true);
        CHECK(bms_pin_state == GPIO_PIN_SET);
    }
}

static void test_system_sil_state_transition_guard_and_audit(void)
{
    static CAN_HandleTypeDef hcan;

    /* A same-state service operation still holds the transition guard while
     * its blocking cleanup runs.  With all ordinary gates healthy, the
     * supervisor must not reassert BMS_OK until finish is published. */
    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.canbus.hcan = &hcan;
    app.board.charger.last_rx_tick = fake_tick;
    taskENTER_CRITICAL();
    set_bms(false);
    CHECK(ams_state_transition_begin(&app,
                                     STATE_CHARGE,
                                     AMS_STATE_TRANSITION_SERVICE_COMMAND,
                                     123u) == AMS_STATE_TRANSITION_NO_CHANGE);
    taskEXIT_CRITICAL();
    CHECK(app.state_transition_in_progress == true);
    sil_mark_all_heartbeats_alive(&app);
    error_task_update(&app, fake_tick);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    taskENTER_CRITICAL();
    ams_state_transition_finish(&app);
    taskEXIT_CRITICAL();
    sil_mark_all_heartbeats_alive(&app);
    error_task_update(&app, fake_tick);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);

    /* An applied charge exit creates an auditable transition and a blocking
     * charger shutdown request; current-policy refresh alone cannot reopen
     * BMS_OK before the CAN owner finishes that request. */
    taskENTER_CRITICAL();
    set_bms(false);
    CHECK(ams_state_transition_begin(&app,
                                     STATE_DISCARGE,
                                     AMS_STATE_TRANSITION_SERVICE_COMMAND,
                                     456u) == AMS_STATE_TRANSITION_APPLIED);
    taskEXIT_CRITICAL();
    CHECK(app.state == STATE_DISCARGE);
    CHECK(app.state_previous == STATE_CHARGE);
    CHECK(app.state_transition_reason == AMS_STATE_TRANSITION_SERVICE_COMMAND);
    CHECK(app.state_transition_count == 1u);
    CHECK(app.state_transition_last_tick == 456u);
    CHECK(app.board.charger.shutdown_pending == true);
    CHECK(app.board.charger.shutdown_frames_remaining == CHARGER_EXIT_DISABLE_FRAMES);
    CHECK(app.board.charger.shutdown_request_count == 1u);
    CHECK(app.charger_fault == true);
    taskENTER_CRITICAL();
    ams_state_transition_finish(&app);
    taskEXIT_CRITICAL();

    sil_run_current_sample(&app, 0.0f);
    sil_mark_all_heartbeats_alive(&app);
    error_task_update(&app, fake_tick);
    CHECK(app.current_fault_mode == CURRENT_FAULT_MODE_DRIVE);
    CHECK(app.bms_state == false);

    /* Corrupt enums are contained as ERROR, request a conservative charger
     * shutdown, and cannot wrap the transition diagnostic counter. */
    init_fake_app();
    app.state = (state_t)99;
    app.state_transition_count = UINT32_MAX;
    taskENTER_CRITICAL();
    CHECK(ams_state_transition_begin(&app,
                                     STATE_DISCARGE,
                                     AMS_STATE_TRANSITION_SERVICE_COMMAND,
                                     789u) == AMS_STATE_TRANSITION_APPLIED);
    taskEXIT_CRITICAL();
    CHECK(app.state == STATE_ERROR);
    CHECK(app.state_previous == (state_t)99);
    CHECK(app.state_transition_reason == AMS_STATE_TRANSITION_CORRUPT_CURRENT_STATE);
    CHECK(app.state_transition_count == UINT32_MAX);
    CHECK(app.board.charger.shutdown_pending == true);
    CHECK(app.charger_fault == true);
    taskENTER_CRITICAL();
    ams_state_transition_finish(&app);
    taskEXIT_CRITICAL();

    CHECK(ams_state_transition_begin(&app,
                                     STATE_DISCARGE,
                                     AMS_STATE_TRANSITION_SERVICE_COMMAND,
                                     790u) == AMS_STATE_TRANSITION_REJECTED);
    CHECK(app.state == STATE_ERROR);
    CHECK(app.state_transition_in_progress == false);
    CHECK(app.state_transition_count == UINT32_MAX);

    init_fake_app();
    app.state = STATE_DISCARGE;
    CHECK(ams_state_transition_begin(&app,
                                     (state_t)77,
                                     AMS_STATE_TRANSITION_SERVICE_COMMAND,
                                     791u) == AMS_STATE_TRANSITION_APPLIED);
    CHECK(app.state == STATE_ERROR);
    CHECK(app.state_transition_reason == AMS_STATE_TRANSITION_INVALID_REQUEST);
    CHECK(app.board.charger.shutdown_pending == true);
    ams_state_transition_finish(&app);
}

static void test_system_sil_bench_bmsok_inhibit_survives_ready_tasks(void)
{
    char *release_argv[] = {"bmsok", "release", NULL};
    char *inhibit_argv[] = {"bmsok", "inhibit", NULL};
    static CAN_HandleTypeDef hcan;

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;

    sil_prepare_cli_capture();
    CHECK(bmsok_control(2, inhibit_argv) == 0);
    CHECK(app.bms_output_inhibit == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    set_bms(true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    for(uint8_t i = 0u; i < 8u; i++)
    {
        sil_run_current_sample(&app, 0.0f);
        fake_adbms_voltage_masks_full_update();
        sil_run_voltage_sample(&app);
        sil_run_can_charge_iteration(&app, &hcan);
        run_one_error_task_iteration(&app);
        CHECK(app.current_valid == true);
        CHECK(app.voltage_valid == true);
        CHECK(app.bms_state == false);
        CHECK(bms_pin_state == GPIO_PIN_RESET);
    }

    sil_prepare_cli_capture();
    CHECK(bmsok_control(2, release_argv) == 0);
    CHECK(app.bms_output_inhibit == false);
    app.current_valid = true;
    app.current_fault = false;
    app.voltage_valid = true;
    app.voltage_fault = false;
    app.temp_valid = true;
    app.temp_fault = false;
    app.temp_charge_stop = false;
    app.adbms_diag_fault = false;
    app.task_heartbeat_fault = false;
    app.fuse_fault = false;
    app.charger_fault = false;
    app.hard_fault = false;
    set_bms(true);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);
}

static void test_system_sil_bench_adbms_write_failures_and_recovery(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    app.acc.smb_ics[0].tx_cfgb.dcc = 0x0003u;
    app.acc.smb_ics[0].PwmA.pwma[1] = BALANCE_PWM_DUTY;
    fake_adbms_wrcfgb_status = HAL_ERROR;
    sil_run_voltage_sample(&app);
    CHECK(app.adbms_diag_fault == true);
    CHECK(app.adbms_balance_write_fault == true);
    CHECK(app.adbms_balance_write_fail_count >= 1u);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    sil_expect_balancing_clear(&app);

    fake_adbms_wrcfgb_status = HAL_OK;
    for(uint8_t i = 0u; i < 12u; i++)
    {
        sil_run_voltage_sample(&app);
    }
    CHECK(app.adbms_diag_fault == false);
    CHECK(app.adbms_status_fault == false);
    CHECK(app.adbms_balance_write_fault == false);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.balance_inhibit = false;
    sil_set_cell_voltage(&app, 0u, 0u, 4.100f);
    sil_set_cell_voltage(&app, 0u, 1u, 4.180f);
    fake_adbms_voltage_masks_full_update();
    /* The scheduler no longer performs a redundant all-off write before a
     * non-balancing scan, so fail the first PWM write in the apply operation. */
    fake_adbms_wrpwm_fail_after_ok = 0;
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == false);
    CHECK(app.adbms_diag_fault == true);
    CHECK(app.adbms_balance_write_fault == true);
    CHECK(app.charge_voltage_stop == true);
    /* A partial CFG/PWM write now triggers an immediate all-off rollback. */
    sil_expect_balancing_clear(&app);

    app.state = STATE_DISCARGE;
    fake_adbms_wrpwm_fail_after_ok = -1;
    sil_run_voltage_sample(&app);
    sil_expect_balancing_clear(&app);
    CHECK(app.adbms_balance_write_fault == false);
    CHECK(app.adbms_diag_fault == false);
    CHECK(app.bms_state == false);
    sil_run_current_sample(&app, 0.0f);
    sil_mark_all_heartbeats_alive(&app);
    error_task_update(&app, fake_tick);
    CHECK(app.bms_state == true);
}

static void test_system_sil_voltage_uv_ov_severe_diagnostics_and_latch(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    sil_set_cell_voltage(&app, 2u, 3u, 2.800f);
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_warning == true);
    CHECK(app.undervoltage_fault == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_UV_SOFT);
    CHECK(app.bms_state == true);

    sil_set_cell_voltage(&app, 2u, 3u, 2.500f);
    sil_run_voltage_sample(&app);
    CHECK(app.undervoltage_fault == true);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.voltage_fault_latched_reason == VOLTAGE_FAULT_REASON_UV_HARD);
    CHECK(app.bms_state == false);

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    sil_set_cell_voltage(&app, 4u, 14u, 4.250f);
    sil_run_voltage_sample(&app);
    CHECK(app.overvoltage_fault == true);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.voltage_fault_latched_reason == VOLTAGE_FAULT_REASON_OV_SEVERE);
    CHECK(app.bms_state == false);

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    sil_set_cell_voltage(&app, 3u, 9u, 2.300f);
    sil_run_voltage_sample(&app);
    CHECK(app.undervoltage_fault == true);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.voltage_fault_latched_reason == VOLTAGE_FAULT_REASON_UV_SEVERE);
    CHECK(app.bms_state == false);
}

static void test_system_sil_current_warning_fast_trip_and_latch_persistence(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    sil_run_current_sample(&app, 75.0f);
    CHECK(app.current_valid == true);
    CHECK(app.current_overcurrent_warning == true);
    CHECK(app.current_overcurrent_fault == false);
    CHECK(app.current_fault == false);
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);
    run_one_error_task_iteration(&app);
    CHECK(app.soft_fault == true);
    CHECK(app.hard_fault == false);
    CHECK(app.bms_state == true);

    for(int i = 0; i < 5; i++)
    {
        sil_run_current_sample(&app, 130.0f);
    }
    CHECK(app.current_valid == true);
    CHECK(app.current_overcurrent_fault == true);
    CHECK(app.current_fault_latched == true);
    CHECK(app.current_fault_latched_reason == CURRENT_FAULT_REASON_DISCHARGE_FAST_OVERCURRENT);
    CHECK(app.current_fault == true);
    CHECK(app.bms_state == false);

    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.voltage_fault == false);
    CHECK(app.bms_state == false);

    for(int i = 0; i < 5; i++)
    {
        sil_run_current_sample(&app, 0.0f);
    }
    CHECK(app.current_valid == true);
    CHECK(app.current_fault_latched == true);
    CHECK(app.current_fault == true);
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == false);
}

static void test_system_sil_current_stale_adc_pair_fails_safe(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    sil_run_current_adc_status(&app, HAL_OK, HAL_TIMEOUT);
    CHECK(app.current_valid == false);
    CHECK(app.current_fault == false);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == false);

    for(int i = 0; i < 25; i++)
    {
        sil_run_current_adc_status(&app, HAL_OK, HAL_TIMEOUT);
    }
    CHECK(app.current_valid == false);
    CHECK(app.current_sensor_fault == true);
    CHECK(app.current_fault == true);
    CHECK(app.current_fault_reason == CURRENT_FAULT_REASON_SENSOR_ADC_READ);
    CHECK(app.bms_state == false);
}

static void test_system_sil_regen_and_charge_current_placeholders(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    sil_run_current_sample(&app, -6.0f);
    CHECK(app.current_valid == true);
    CHECK(app.current_overcurrent_warning == true);
    CHECK(app.current_fault_reason == CURRENT_FAULT_REASON_REGEN_UNEXPECTED);
    CHECK(app.current_fault == false);
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);

    for(int i = 0; i < 5; i++)
    {
        sil_run_current_sample(&app, -35.0f);
    }
    CHECK(app.current_fault == true);
    CHECK(app.current_fault_latched == true);
    CHECK(app.current_fault_latched_reason == CURRENT_FAULT_REASON_REGEN_FAST_OVERCURRENT);
    CHECK(app.bms_state == false);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    for(int i = 0; i < 5; i++)
    {
        sil_run_current_sample(&app, -16.0f);
    }
    CHECK(app.current_fault == true);
    CHECK(app.current_fault_latched == true);
    CHECK(app.current_fault_latched_reason == CURRENT_FAULT_REASON_CHARGE_FAST_OVERCURRENT);
    CHECK(app.bms_state == false);
}

static void test_system_sil_2950_advisory_sampling_and_cli(void)
{
    static SPI_HandleTypeDef hspi;
	uint32_t prior_sample_count;
    char *status[] = {"apm", "status"};
    char *sample[] = {"apm", "sample"};
    char *sid[] = {"apm", "sid"};
	char *config[] = {"apm", "config"};
	char *scope[] = {"apm", "scope", "3"};
	char *scope_bad[] = {"apm", "scope", "0"};

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
	fake_external_counter_note_calls = 0u;
	fake_external_counter_increment_total = 0u;
	fake_counter_resync_calls = 0u;
    hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	/* Both logical drivers share the one physical SPI6 peripheral. */
	app.acc.smb.hspi = &hspi;
    app.acc.apm.hspi = &hspi;
	prior_sample_count = app.acc.apm.health.sample_count;

    fake_apm_i1_raw = -250;
    fake_apm_vb1_raw = 18000;
    fake_tick = 1000u;
    CHECK(accumulator_read_volt(&app.acc) == 0);
    CHECK(accumulator_read_apm(&app.acc, fake_tick) == 0);
    CHECK(app.acc.apm.health.sample_valid == true);
    CHECK(app.acc.apm.health.current_valid == true);
    CHECK(app.acc.apm.health.pack_voltage_valid == false);
    CHECK(fabsf(app.acc.apm.health.current_a - (-2.5f)) < 0.001f);
    CHECK(app.acc.apm.health.last_update_ms == 1000u);
    CHECK(app.bms_state == true);
	CHECK(fake_external_counter_note_calls == 1u);
    CHECK(fake_external_counter_increment_total ==
	      ADBMS2950_SHARED_COUNTER_INCREMENTS_PER_SAMPLE);
	CHECK(fake_counter_resync_calls == 0u);
	/* The shared-counter proof is one-shot. A second APM read cannot reuse a
	 * prior SMB wake/scan token. */
	CHECK(accumulator_read_apm(&app.acc, 1500u) == -1);
	CHECK(app.acc.apm.health.sample_count == prior_sample_count + 1u);
	CHECK(app.acc.apm.health.sample_valid == false);
	CHECK(app.acc.apm.health.current_valid == false);
	CHECK(fake_external_counter_note_calls == 1u);

    /* APM failure is visible but deliberately cannot become a safety input
     * until final-board scaling/polarity/fault validation is complete. */
    fake_apm_sample_status = HAL_TIMEOUT;
    CHECK(accumulator_read_volt(&app.acc) == 0);
    CHECK(accumulator_read_apm(&app.acc, 2000u) == -1);
    CHECK(app.acc.apm.health.sample_valid == false);
    CHECK(app.acc.apm.health.sample_error_count == 1u);
	CHECK(fake_counter_resync_calls == 1u);
    CHECK(app.adbms_diag_fault == false);
    CHECK(app.bms_state == true);
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.current_valid == true);
    CHECK(app.bms_state == true);

    fake_apm_sample_status = HAL_OK;
    sil_prepare_cli_capture();
    CHECK(get_apm_debug(2, status) == 0);
    CHECK(strstr(cli_capture, "APM topology A:5x6830 B:1x2950 selected:CS_B") != NULL);
    CHECK(strstr(cli_capture, "ADVISORY_NON_GATING") != NULL);
    CHECK(strstr(cli_capture, "dividers:OFF") != NULL);

    sil_prepare_cli_capture();
    CHECK(get_apm_debug(2, sample) == 0);
    CHECK(strstr(cli_capture, "SNAP+RDSTAT+RDIVB1+RDFLAG sample: OK") != NULL);
    CHECK(strstr(cli_capture, "current:-2.500A") != NULL);
	CHECK(fake_external_counter_note_calls == 1u);
	CHECK(fake_external_counter_increment_total ==
	      ADBMS2950_SHARED_COUNTER_INCREMENTS_PER_SAMPLE);
	/* The intervening periodic voltage iteration also retries the still-failed
	 * advisory APM sample and conservatively resynchronizes the SMB counters. */
	CHECK(fake_counter_resync_calls == 3u);

    fake_apm_probe_status = HAL_OK;
    sil_prepare_cli_capture();
    CHECK(get_apm_debug(2, sid) == 0);
    CHECK(strstr(cli_capture, "RDSID identity probe: OK") != NULL);
    CHECK(app.acc.apm.health.device_id == ADBMS2950B_DEVICE_ID);
	CHECK(fake_counter_resync_calls == 4u);

	sil_prepare_cli_capture();
	CHECK(get_apm_debug(2, config) == 0);
	CHECK(strstr(cli_capture, "RDCFGA/RDCFGB readback: OK") != NULL);
	CHECK(fake_counter_resync_calls == 5u);

	sil_prepare_cli_capture();
	CHECK(get_apm_debug(3, scope) == 0);
	CHECK(strstr(cli_capture, "scope requested:3 completed:3 status:OK") != NULL);
	CHECK(fake_counter_resync_calls == 6u);

	sil_prepare_cli_capture();
	CHECK(get_apm_debug(3, scope_bad) == 0);
	CHECK(strstr(cli_capture, "Usage: apm scope [1-100]") != NULL);

    app.adbms_scan_active = true;
    sil_prepare_cli_capture();
    CHECK(get_apm_debug(2, sample) == 0);
    CHECK(strstr(cli_capture, "apm sample refused") != NULL);
    app.adbms_scan_active = false;
}

static void test_system_sil_2950_final_ring_init_ownership(void)
{
    accumulator_t acc;
    SPI_HandleTypeDef hspi;
    GPIO_TypeDef cs_a;
    GPIO_TypeDef cs_b;
    TIM_HandleTypeDef timer;

    init_fake_app();
	fake_external_counter_note_calls = 0u;
	fake_external_counter_increment_total = 0u;
	fake_counter_resync_calls = 0u;
    memset(&acc, 0, sizeof(acc));
    memset(&hspi, 0, sizeof(hspi));
    memset(&cs_a, 0, sizeof(cs_a));
    memset(&cs_b, 0, sizeof(cs_b));
    memset(&timer, 0, sizeof(timer));
    accumulator_init(&acc, &hspi, &cs_a, &cs_b, 0x0002u, 0x0004u, &timer);

    CHECK(acc.smb_ready == true);
    CHECK(acc.smb.string == STRING_A);
    CHECK(acc.smb.write_string == STRING_A);
    CHECK(acc.apm_ready == true);
    CHECK(acc.apm.string == STRING_B);
    CHECK(acc.apm.write_string == STRING_B);
    CHECK(accumulator_final_ring_topology_valid(&acc));
    CHECK(fake_apm_init_string == STRING_B);
    CHECK(fake_apm_init_requested_reset == false);
    CHECK(fake_apm_init_enabled_dividers == false);
    CHECK(acc.apm.health.hv_dividers_enabled == false);
	CHECK(fake_external_counter_note_calls == 0u);
	CHECK(fake_external_counter_increment_total == 0u);
	CHECK(fake_counter_resync_calls == 1u);

    /* APM startup failure does not erase a verified SMB chain or become a
     * hidden BMS_OK gate while the path is advisory. */
    fake_apm_init_status = HAL_TIMEOUT;
    memset(&acc, 0, sizeof(acc));
    accumulator_init(&acc, &hspi, &cs_a, &cs_b, 0x0002u, 0x0004u, &timer);
    CHECK(acc.smb_ready == true);
    CHECK(acc.apm_ready == false);
    CHECK(acc.apm_init_status == HAL_TIMEOUT);
    CHECK(accumulator_final_ring_topology_valid(&acc));
    CHECK(fake_apm_init_requested_reset == false);
	CHECK(fake_counter_resync_calls == 2u);

    fake_apm_init_status = HAL_OK;
}

static void test_system_sil_final_ring_topology_corruption_fails_closed(void)
{
    init_fake_app();
    CHECK(accumulator_final_ring_topology_valid(&app.acc));

    /* A stale diagnostic that leaves the SMB driver pointed at String B must
     * not turn its five-packet subset write/read sequence into a different
     * physical device selection. */
    app.acc.apm_full_ring_awake_token = true;
    app.acc.smb.write_string = STRING_B;
    CHECK(!accumulator_final_ring_topology_valid(&app.acc));
    CHECK(accumulator_read_volt(&app.acc) == -1);
    CHECK(app.acc.apm_full_ring_awake_token == false);
    CHECK(accumulator_set_balance(&app.acc) == -1);
    CHECK(accumulator_clear_balance(&app.acc) == -1);

    app.acc.smb.write_string = STRING_A;
    app.acc.apm.string = STRING_A;
    CHECK(!accumulator_final_ring_topology_valid(&app.acc));
    CHECK(accumulator_read_apm(&app.acc, fake_tick) == -1);

    app.acc.apm.string = STRING_B;
    CHECK(accumulator_final_ring_topology_valid(&app.acc));
}

static void test_system_sil_2950_requires_successful_full_ring_scan(void)
{
	uint32_t prior_sample_count;

	init_fake_app();
	fake_external_counter_note_calls = 0u;
	fake_external_counter_increment_total = 0u;
	fake_counter_resync_calls = 0u;
	app.acc.apm.health.counter_seen = true;
	app.acc.apm.health.counter_advanced = true;
	app.acc.apm.health.sample_valid = true;
	app.acc.apm.health.current_valid = true;
	prior_sample_count = app.acc.apm.health.sample_count;

	/* ADCV reached the mixed ring, but a later cell read failed. The APM
	 * freshness epoch must still reset, while no full-ring sample token may be
	 * published and no SNAP/UNSNAP transaction may be attempted. */
	fake_adbms_read_cell_status = HAL_TIMEOUT;
	CHECK(accumulator_read_volt(&app.acc) == -1);
	CHECK(app.acc.apm.health.counter_seen == false);
	CHECK(app.acc.apm.health.counter_advanced == false);
	CHECK(app.acc.apm.health.sample_valid == false);
	CHECK(app.acc.apm.health.current_valid == false);
	CHECK(app.acc.apm_full_ring_awake_token == false);
	CHECK(accumulator_read_apm(&app.acc, 1000u) == -1);
	CHECK(app.acc.apm.health.sample_count == prior_sample_count);
	CHECK(fake_external_counter_note_calls == 0u);
	CHECK(fake_counter_resync_calls == 0u);

	/* The periodic task applies the same rule: a failed SMB scan skips the APM
	 * transaction rather than pretending every physical device was awake. */
	run_one_adbms_task_iteration(&app);
	CHECK(app.acc.apm.health.sample_count == prior_sample_count);
	CHECK(fake_external_counter_note_calls == 0u);

	/* A complete subsequent scan re-arms exactly one coordinated APM sample. */
	fake_adbms_read_cell_status = HAL_OK;
	CHECK(accumulator_read_volt(&app.acc) == 0);
	CHECK(app.acc.apm_full_ring_awake_token == true);
	CHECK(accumulator_read_apm(&app.acc, 2000u) == 0);
	CHECK(app.acc.apm_full_ring_awake_token == false);
	CHECK(app.acc.apm.health.sample_count == prior_sample_count + 1u);
	CHECK(fake_external_counter_note_calls == 1u);
	CHECK(fake_external_counter_increment_total ==
	      ADBMS2950_SHARED_COUNTER_INCREMENTS_PER_SAMPLE);
}

static void test_system_sil_combined_fault_precedence_and_reset_path(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    for(int i = 0; i < 5; i++)
    {
        sil_run_current_sample(&app, 130.0f);
    }
    CHECK(app.current_fault_latched == true);
    CHECK(app.current_fault_latched_reason == CURRENT_FAULT_REASON_DISCHARGE_FAST_OVERCURRENT);
    CHECK(app.bms_state == false);

    sil_set_cell_voltage(&app, 1u, 1u, 4.250f);
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.current_fault_latched == true);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.voltage_fault_latched_reason == VOLTAGE_FAULT_REASON_OV_SEVERE);
    CHECK(app.bms_state == false);

    sil_set_cell_voltage(&app, 1u, 1u, 3.700f);
    voltage_fault_reset_latch(&app.voltage_fault_state);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_fault == false);
    CHECK(app.current_fault == true);
    CHECK(app.bms_state == false);

    current_fault_reset_latch(&app.current_fault_state);
    for(int i = 0; i < 3; i++)
    {
        sil_run_current_sample(&app, 0.0f);
    }
    sil_run_voltage_sample(&app);
    CHECK(app.current_fault == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.current_valid == true);
    CHECK(app.voltage_valid == true);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);
}

static void test_system_sil_harsh_timeline_no_false_enable(void)
{
    init_fake_app();
    fake_tick = 0u;
    bms_pin_state = GPIO_PIN_RESET;
    app.state = STATE_START;
    sil_attach_current_adcs(&app);
    fill_nominal_pack(&app, 3.700f);
    sil_clear_voltage_history(&app);

    fake_adbms_voltage_masks_all_missing(true);
    sil_run_current_sample(&app, 0.3f);
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == false);
    CHECK(app.voltage_fault == true);

    fake_adbms_voltage_masks_full_update();
    sil_run_current_sample(&app, 0.3f);
    sil_run_voltage_sample(&app);
    CHECK(app.state == STATE_DISCARGE);
    CHECK(app.bms_state == false);
    sil_run_current_sample(&app, 0.3f);
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);

    fake_adbms_voltage_masks_one_missing(0u, 4u, true);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == false);
    CHECK(app.voltage_read_fault == true);
    CHECK(app.voltage_warning == false);
    CHECK(app.voltage_fault == true);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_PEC_FAILURE);
    CHECK(app.bms_state == false);

    sil_run_current_sample(&app, 250.0f);
    CHECK(app.current_fault == true);
    CHECK(app.current_fault_latched == true);
    CHECK(app.current_fault_latched_reason == CURRENT_FAULT_REASON_DISCHARGE_EXTREME);
    CHECK(app.bms_state == false);

    app.state = STATE_DISCARGE;
    current_fault_reset_latch(&app.current_fault_state);
    for(int i = 0; i < 3; i++)
    {
        sil_run_current_sample(&app, 0.0f);
    }
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.current_fault == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.bms_state == true);

    fake_adbms_voltage_masks_all_missing(true);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == false);
    CHECK(app.voltage_fault == true);
    CHECK(app.bms_state == false);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == false);
    CHECK(app.bms_state == false);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_fault == true);
    CHECK(app.bms_state == false);
}


static void test_system_sil_current_invalid_immediate_bms_drop_and_recovery(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);

    sil_run_current_adc_status(&app, HAL_TIMEOUT, HAL_OK);
    CHECK(app.current_valid == false);
    CHECK(app.current_fault == false);
    CHECK(app.current_meas_reason == CURRENT_SENSOR_REASON_ADC_READ);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    sil_run_current_sample(&app, 0.0f);
    CHECK(app.current_valid == true);
    CHECK(app.current_fault == false);
    CHECK(app.bms_state == false);

    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.current_valid == true);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);
}

static void test_system_sil_hard_fault_and_corrupt_smb_config_fail_closed(void)
{
    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    CHECK(app.bms_state == true);

    app.fuse_fault = true;
    sil_set_cell_voltage(&app, 0u, 1u, 4.000f);
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.voltage_fault == false);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    sil_expect_balancing_clear(&app);

    app.fuse_fault = false;
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);

    init_fake_app();
    fake_tick = 0u;
    app.state = STATE_DISCARGE;
    sil_attach_current_adcs(&app);
    fill_nominal_pack(&app, 3.700f);
    app.acc.smb.num_ics = 2;
    app.acc.smb.ics = app.acc.smb_ics;
    fake_adbms_voltage_masks_full_update();
    sil_run_current_sample(&app, 0.0f);
    sil_run_voltage_sample(&app);
    CHECK(app.current_valid == true);
    CHECK(app.voltage_valid == false);
    CHECK(app.voltage_fault == true);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_PARTIAL_SCAN);
    CHECK(app.voltage_updated_cell_count == (uint16_t)(2u * NCELLS));
    CHECK(app.bms_state == false);

    init_fake_app();
    fake_tick = 0u;
    app.state = STATE_DISCARGE;
    sil_attach_current_adcs(&app);
    fill_nominal_pack(&app, 3.700f);
    app.acc.smb.ics = NULL;
    fake_adbms_voltage_masks_full_update();
    sil_run_current_sample(&app, 0.0f);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == false);
    CHECK(app.voltage_fault == true);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_PARTIAL_SCAN);
    CHECK(app.bms_state == false);
}

static void test_system_sil_voltage_threshold_exact_edges(void)
{
    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    sil_set_cell_voltage(&app, 0u, 0u, 4.179f);
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.charge_voltage_stop == false);
    CHECK(app.overvoltage_fault == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.bms_state == true);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    sil_set_cell_voltage(&app, 0u, 0u, 4.180f);
    sil_run_voltage_sample(&app);
    CHECK(app.charge_voltage_stop == true);
    CHECK(app.overvoltage_fault == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_CHARGE_STOP);
    CHECK(app.bms_state == true);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    sil_set_cell_voltage(&app, 0u, 0u, 4.199f);
    sil_run_voltage_sample(&app);
    CHECK(app.charge_voltage_stop == true);
    CHECK(app.overvoltage_fault == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.bms_state == true);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    sil_set_cell_voltage(&app, 0u, 0u, 4.200f);
    sil_run_voltage_sample(&app);
    CHECK(app.overvoltage_fault == true);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.voltage_fault_latched_reason == VOLTAGE_FAULT_REASON_OV_HARD);
    CHECK(app.bms_state == false);

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    sil_set_cell_voltage(&app, 1u, 1u, 2.501f);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_warning == true);
    CHECK(app.undervoltage_fault == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_UV_SOFT);
    CHECK(app.bms_state == true);

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    sil_set_cell_voltage(&app, 1u, 1u, 2.500f);
    sil_run_voltage_sample(&app);
    CHECK(app.undervoltage_fault == true);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.voltage_fault_latched_reason == VOLTAGE_FAULT_REASON_UV_HARD);
    CHECK(app.bms_state == false);
}

static void test_system_sil_charger_disable_from_dynamic_gates(void)
{
    static CAN_HandleTypeDef hcan;

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;
    sil_set_cell_voltage(&app, 0u, 0u, 4.180f);
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.charge_voltage_stop == true);
    CHECK(app.bms_state == true);
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 1u);
    CHECK((app.board.charger.disable_reason_mask & CHARGER_DISABLE_REASON_VOLTAGE_CHARGE_STOP) != 0u);
    CHECK(app.bms_state == true);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;
    sil_run_current_sample(&app, -11.0f);
    CHECK(app.current_overcurrent_warning == true);
    CHECK(app.current_fault == false);
    CHECK(app.bms_state == true);
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 0u);
    CHECK(app.bms_state == true);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;
    sil_run_current_adc_status(&app, HAL_OK, HAL_TIMEOUT);
    CHECK(app.current_valid == false);
    CHECK(app.bms_state == false);
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 1u);
    CHECK((app.board.charger.disable_reason_mask & CHARGER_DISABLE_REASON_CURRENT_INVALID) != 0u);
    CHECK(app.bms_state == false);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;
    app.bms_state = false;
    bms_pin_state = GPIO_PIN_RESET;
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 1u);
    CHECK(app.board.charger.disable_reason_mask == CHARGER_DISABLE_REASON_BMS_NOT_OK);
    CHECK(app.charger_fault == false);
    CHECK(app.bms_state == false);

    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 0u);
    CHECK(app.board.charger.disable_reason_mask == CHARGER_DISABLE_REASON_NONE);
    CHECK(app.bms_state == true);
}

static void test_system_sil_deterministic_fault_injection_invariants(void)
{
    for(uint32_t i = 0u; i < 192u; i++)
    {
        init_fake_app();
        fake_tick = 0u;
        bms_pin_state = GPIO_PIN_RESET;
        app.state = (i % 5u == 0u) ? STATE_CHARGE :
                    (i % 5u == 1u) ? STATE_START :
                    (i % 5u == 2u) ? STATE_BALANCE : STATE_DISCARGE;
        sil_attach_current_adcs(&app);
        fill_nominal_pack(&app, 3.700f);
        sil_clear_voltage_history(&app);

        switch(i % 12u)
        {
            case 0u: sil_set_cell_voltage(&app, (uint8_t)(i % NSMBS), (uint8_t)(i % NCELLS), 4.250f); break;
            case 1u: sil_set_cell_voltage(&app, (uint8_t)(i % NSMBS), (uint8_t)(i % NCELLS), 4.200f); break;
            case 2u: sil_set_cell_voltage(&app, (uint8_t)(i % NSMBS), (uint8_t)(i % NCELLS), 4.180f); break;
            case 3u: sil_set_cell_voltage(&app, (uint8_t)(i % NSMBS), (uint8_t)(i % NCELLS), 2.300f); break;
            case 4u: sil_set_cell_voltage(&app, (uint8_t)(i % NSMBS), (uint8_t)(i % NCELLS), 2.500f); break;
            case 5u: sil_set_cell_voltage(&app, (uint8_t)(i % NSMBS), (uint8_t)(i % NCELLS), 2.800f); break;
            default: break;
        }

        if((i % 17u) == 0u)
        {
            fake_adbms_voltage_masks_all_missing(true);
        }
        else if((i % 13u) == 0u)
        {
            fake_adbms_voltage_masks_one_missing((uint8_t)(i % NSMBS), (uint8_t)(i % NCELLS), true);
        }
        else
        {
            fake_adbms_voltage_masks_full_update();
        }

        if((i % 19u) == 0u)
        {
            app.hard_fault = true;
        }
        if((i % 23u) == 0u)
        {
            app.fuse_fault = true;
        }
        if((i % 29u) == 0u)
        {
            app.charger_fault = true;
        }

        if((i % 31u) == 0u)
        {
            sil_run_current_adc_status(&app, HAL_TIMEOUT, HAL_OK);
        }
        else
        {
            float current_a;
            switch(i % 10u)
            {
                case 0u: current_a = 0.0f; break;
                case 1u: current_a = 75.0f; break;
                case 2u: current_a = 130.0f; break;
                case 3u: current_a = 260.0f; break;
                case 4u: current_a = -6.0f; break;
                case 5u: current_a = -11.0f; break;
                case 6u: current_a = -16.0f; break;
                case 7u: current_a = -35.0f; break;
                case 8u: current_a = 0.9f; break;
                default: current_a = 2.5f; break;
            }
            sil_run_current_sample(&app, current_a);
        }

        sil_run_voltage_sample(&app);

        if(app.bms_state)
        {
            CHECK(bms_pin_state == GPIO_PIN_SET);
            CHECK(app.current_valid == true);
            CHECK(app.current_fault == false);
            CHECK(app.voltage_valid == true);
            CHECK(app.voltage_fault == false);
            CHECK(app.temp_fault == false);
            CHECK(app.fuse_fault == false);
            CHECK(app.charger_fault == false);
            CHECK(app.hard_fault == false);
        }
        else
        {
            CHECK(bms_pin_state == GPIO_PIN_RESET);
        }

        if(app.voltage_fault_latched || app.current_fault_latched || app.hard_fault || app.fuse_fault)
        {
            CHECK(app.bms_state == false);
        }
        if(!app.current_valid || !app.voltage_valid)
        {
            CHECK(app.bms_state == false);
        }
    }
}


#define SIL_CHECK(ctx, cond) do{ if(!(cond)){ fprintf(stderr,"FAIL %s:%d [%s]: %s\n", __FILE__, __LINE__, (ctx), #cond); exit(1);} }while(0)

static uint32_t sil_rng_next(uint32_t *state)
{
    *state = (*state * 1664525u) + 1013904223u;
    return *state;
}

static void sil_write_all_cells(app_data_t *d, float volts)
{
    CHECK(d != NULL);
    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        for(uint8_t cell = 0u; cell < NCELLS; cell++)
        {
            d->acc.smb_ics[ic].cell.c_codes[cell] = code_for_volts(volts);
        }
    }
}

static void sil_prepare_cli_capture(void)
{
    cli_capture_clear();
    cli_device_init(&app.board.cli, &cli_dummy_uart);
    cli = &app.board.cli;
    data = &app;
}

static void sil_assert_diagnostics_not_generic(const app_data_t *d, const char *ctx)
{
    SIL_CHECK(ctx, d != NULL);

    if(d->voltage_fault || !d->voltage_valid || d->voltage_fault_latched)
    {
        SIL_CHECK(ctx, d->voltage_fault_reason != VOLTAGE_FAULT_REASON_NONE ||
                       d->voltage_fault_latched_reason != VOLTAGE_FAULT_REASON_NONE);
        SIL_CHECK(ctx, strcmp(voltage_fault_reason_str(d->voltage_fault_reason), "unknown") != 0);
    }

    if(d->current_fault || !d->current_valid || d->current_fault_latched)
    {
        SIL_CHECK(ctx, d->current_fault_reason != CURRENT_FAULT_REASON_NONE ||
                       d->current_fault_latched_reason != CURRENT_FAULT_REASON_NONE ||
                       d->current_meas_reason != CURRENT_SENSOR_REASON_OK);
        SIL_CHECK(ctx, strcmp(current_fault_reason_str(d->current_fault_reason), "unknown") != 0);
    }

    if(d->temp_fault || !d->temp_valid || d->temp_fault_latched)
    {
        SIL_CHECK(ctx, d->temp_fault_reason != TEMPERATURE_FAULT_REASON_NONE ||
                       d->temp_fault_latched_reason != TEMPERATURE_FAULT_REASON_NONE);
        SIL_CHECK(ctx, strcmp(temperature_fault_reason_str(d->temp_fault_reason), "unknown") != 0);
    }
}

static void sil_assert_safety_invariants(const app_data_t *d, const char *ctx)
{
    SIL_CHECK(ctx, d != NULL);

    if(d->bms_state)
    {
        SIL_CHECK(ctx, bms_pin_state == GPIO_PIN_SET);
        SIL_CHECK(ctx, d->current_valid == true);
        SIL_CHECK(ctx, d->voltage_valid == true);
        SIL_CHECK(ctx, d->current_fault == false);
        SIL_CHECK(ctx, d->current_sensor_fault == false);
        SIL_CHECK(ctx, d->current_overcurrent_fault == false);
        SIL_CHECK(ctx, d->current_fault_latched == false);
        SIL_CHECK(ctx, d->voltage_fault == false);
        SIL_CHECK(ctx, d->voltage_read_fault == false);
        SIL_CHECK(ctx, d->overvoltage_fault == false);
        SIL_CHECK(ctx, d->undervoltage_fault == false);
        SIL_CHECK(ctx, d->voltage_fault_latched == false);
        SIL_CHECK(ctx, d->temp_valid == true);
        SIL_CHECK(ctx, d->temp_fault == false);
        SIL_CHECK(ctx, d->temp_read_fault == false);
        SIL_CHECK(ctx, d->overtemp_fault == false);
        SIL_CHECK(ctx, d->severe_overtemp_fault == false);
        SIL_CHECK(ctx, d->temp_fault_latched == false);
        SIL_CHECK(ctx, d->task_heartbeat_fault == false);
        if(d->state == STATE_CHARGE)
        {
            SIL_CHECK(ctx, d->temp_charge_stop == false);
        }
        SIL_CHECK(ctx, d->fuse_fault == false);
        SIL_CHECK(ctx, d->charger_fault == false);
        SIL_CHECK(ctx, d->hard_fault == false);
    }
    else
    {
        SIL_CHECK(ctx, bms_pin_state == GPIO_PIN_RESET);
    }

    if(!d->current_valid || !d->voltage_valid || !d->temp_valid || d->hard_fault || d->fuse_fault ||
       d->temp_fault || d->charger_fault || d->current_fault || d->current_fault_latched ||
       d->voltage_fault || d->voltage_fault_latched || d->overvoltage_fault ||
       d->undervoltage_fault || d->temp_read_fault || d->temp_fault_latched || d->overtemp_fault ||
       d->task_heartbeat_fault ||
       ((d->state == STATE_CHARGE) && d->temp_charge_stop))
    {
        SIL_CHECK(ctx, d->bms_state == false);
    }

    if(d->voltage_valid)
    {
        SIL_CHECK(ctx, d->voltage_usable_cell_count == AMS_EXPECTED_CELL_COUNT);
    }
    if(d->voltage_fault_latched)
    {
        SIL_CHECK(ctx, d->voltage_fault_latched_reason != VOLTAGE_FAULT_REASON_NONE);
    }
    if(d->current_fault_latched)
    {
        SIL_CHECK(ctx, d->current_fault_latched_reason != CURRENT_FAULT_REASON_NONE);
    }
    if(d->temp_valid)
    {
        SIL_CHECK(ctx, d->temp_usable_sensor_count == AMS_EXPECTED_TEMP_SENSOR_COUNT);
    }
    if(d->temp_fault_latched)
    {
        SIL_CHECK(ctx, d->temp_fault_latched_reason != TEMPERATURE_FAULT_REASON_NONE);
    }
    if(d->balance_inhibit)
    {
        for(uint8_t ic = 0u; ic < NSMBS; ic++)
        {
            SIL_CHECK(ctx, d->acc.smb_ics[ic].tx_cfgb.dcc == 0u);
            for(uint8_t cell = 0u; cell < PWMA; cell++)
            {
                SIL_CHECK(ctx, d->acc.smb_ics[ic].PwmA.pwma[cell] == 0u);
            }
            for(uint8_t cell = 0u; cell < PWMB; cell++)
            {
                SIL_CHECK(ctx, d->acc.smb_ics[ic].PwmB.pwmb[cell] == 0u);
            }
        }
    }

    sil_assert_diagnostics_not_generic(d, ctx);
}

static void sil_run_can_charge_iteration(app_data_t *d, CAN_HandleTypeDef *hcan)
{
    CHECK(d != NULL);
    d->board.canbus.hcan = hcan;
    if(d->state == STATE_CHARGE)
    {
        if(d->board.charger.last_rx_tick == 0u)
        {
            d->board.charger.last_rx_tick = fake_tick;
        }
    }
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(d);
}

static void sil_force_state_for_scheduler_abuse(app_data_t *d, state_t state)
{
    CHECK(d != NULL);
    d->state = state;
    if((state == STATE_CHARGE) && d->temp_charge_stop)
    {
        set_bms(false);
    }
}

static void test_system_sil_task_order_permutations_fail_closed(void)
{
    static CAN_HandleTypeDef hcan;

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;

    /* Explicit order from the bring-up concern: current kills BMS, ADBMS and charger must not re-enable it. */
    sil_run_current_adc_status(&app, HAL_TIMEOUT, HAL_OK);
    CHECK(app.current_valid == false);
    CHECK(app.bms_state == false);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.bms_state == false);
    sil_run_can_charge_iteration(&app, &hcan);
    CHECK(app.bms_state == false);
    CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT && tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 1u);
    sil_assert_safety_invariants(&app, "current_bad_then_voltage_then_charger");

    /* Same observed current fault, but swap later task order. */
    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;
    sil_run_current_adc_status(&app, HAL_TIMEOUT, HAL_OK);
    sil_run_can_charge_iteration(&app, &hcan);
    CHECK(app.bms_state == false);
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == false);
    sil_assert_safety_invariants(&app, "current_bad_then_charger_then_voltage");

    /* A hard-fault source aggregated by error_task cannot be revived by later tasks. */
    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;
    app.fuse_fault = true;
    run_one_error_task_iteration(&app);
    CHECK(app.hard_fault == true);
    CHECK(app.bms_state == false);
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == false);
    sil_run_current_sample(&app, 0.0f);
    CHECK(app.bms_state == false);
    sil_run_can_charge_iteration(&app, &hcan);
    CHECK(app.bms_state == false);
    sil_assert_safety_invariants(&app, "hard_fault_order_permutation");

    /* Latched OV cannot be cleared by a healthy current sample or charger tick. */
    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;
    sil_set_cell_voltage(&app, 0u, 0u, 4.200f);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.bms_state == false);
    sil_run_current_sample(&app, 0.0f);
    CHECK(app.bms_state == false);
    sil_run_can_charge_iteration(&app, &hcan);
    CHECK(app.bms_state == false);
    sil_assert_safety_invariants(&app, "latched_voltage_order_permutation");
}

static void test_system_sil_recovery_and_latch_reset_paths(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    sil_run_current_sample(&app, 75.0f);
    CHECK(app.current_overcurrent_warning == true);
    CHECK(app.current_fault == false);
    CHECK(app.bms_state == true);
    sil_run_current_sample(&app, 0.0f);
    CHECK(app.current_overcurrent_warning == false);
    CHECK(app.current_fault_reason == CURRENT_FAULT_REASON_NONE);
    CHECK(app.bms_state == true);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    sil_set_cell_voltage(&app, 0u, 0u, 4.180f);
    sil_run_voltage_sample(&app);
    CHECK(app.charge_voltage_stop == true);
    CHECK(app.voltage_fault == false);
    CHECK(app.bms_state == true);
    sil_set_cell_voltage(&app, 0u, 0u, 3.700f);
    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.charge_voltage_stop == false);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_NONE);
    CHECK(app.bms_state == true);

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    sil_set_cell_voltage(&app, 1u, 1u, 4.200f);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.bms_state == false);
    sil_set_cell_voltage(&app, 1u, 1u, 3.700f);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_fault_latched == true);
    CHECK(app.bms_state == false);
    voltage_fault_reset_latch(&app.voltage_fault_state);
    sil_run_current_sample(&app, 0.0f);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_fault_latched == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.bms_state == true);

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    for(uint8_t i = 0u; i < 5u; i++)
    {
        sil_run_current_sample(&app, 130.0f);
    }
    CHECK(app.current_fault_latched == true);
    CHECK(app.bms_state == false);
    for(uint8_t i = 0u; i < 3u; i++)
    {
        sil_run_current_sample(&app, 0.0f);
    }
    CHECK(app.current_fault_latched == true);
    CHECK(app.bms_state == false);
    current_fault_reset_latch(&app.current_fault_state);
    sil_run_current_sample(&app, 0.0f);
    sil_run_voltage_sample(&app);
    CHECK(app.current_fault_latched == false);
    CHECK(app.current_fault == false);
    CHECK(app.bms_state == true);
}

static void test_system_sil_current_boundary_timing_edges(void)
{
    current_fault_state_t st;
    current_fault_init(&st);

    current_fault_update(&st, CURRENT_FAULT_MODE_DRIVE, 86.0f, true,
                         CURRENT_SENSOR_REASON_OK, 499u);
    CHECK(st.pending == true);
    CHECK(st.confirmed == false);
    CHECK(st.latched == false);
    current_fault_update(&st, CURRENT_FAULT_MODE_DRIVE, 86.0f, true,
                         CURRENT_SENSOR_REASON_OK, 1u);
    CHECK(st.confirmed == true);
    CHECK(st.latched == true);
    CHECK(st.latched_reason == CURRENT_FAULT_REASON_DISCHARGE_OVERCURRENT);

    current_fault_init(&st);
    current_fault_update(&st, CURRENT_FAULT_MODE_DRIVE, 121.0f, true,
                         CURRENT_SENSOR_REASON_OK, 99u);
    CHECK(st.pending == true);
    CHECK(st.confirmed == false);
    current_fault_update(&st, CURRENT_FAULT_MODE_DRIVE, 121.0f, true,
                         CURRENT_SENSOR_REASON_OK, 1u);
    CHECK(st.confirmed == true);
    CHECK(st.latched_reason == CURRENT_FAULT_REASON_DISCHARGE_FAST_OVERCURRENT);

    current_fault_init(&st);
    current_fault_update(&st, CURRENT_FAULT_MODE_CHARGE, -12.5f, true,
                         CURRENT_SENSOR_REASON_OK, 499u);
    CHECK(st.confirmed == false);
    current_fault_update(&st, CURRENT_FAULT_MODE_CHARGE, -12.5f, true,
                         CURRENT_SENSOR_REASON_OK, 1u);
    CHECK(st.confirmed == true);
    CHECK(st.latched_reason == CURRENT_FAULT_REASON_CHARGE_OVERCURRENT);

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    for(uint8_t i = 0u; i < 4u; i++)
    {
        sil_run_current_sample(&app, 130.0f);
    }
    CHECK(app.current_overcurrent_pending == true);
    CHECK(app.current_overcurrent_fault == false);
    CHECK(app.current_fault_latched == false);
    sil_run_current_sample(&app, 130.0f);
    CHECK(app.current_overcurrent_fault == true);
    CHECK(app.current_fault_latched == true);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    for(uint8_t i = 0u; i < 24u; i++)
    {
        sil_run_current_sample(&app, -12.5f);
    }
    CHECK(app.current_overcurrent_pending == true);
    CHECK(app.current_fault_latched == false);
    sil_run_current_sample(&app, -12.5f);
    CHECK(app.current_fault_latched == true);
    CHECK(app.current_fault_latched_reason == CURRENT_FAULT_REASON_CHARGE_OVERCURRENT);
}

static void test_system_sil_cli_can_diagnostic_consistency(void)
{
    static CAN_HandleTypeDef hcan;

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    fake_adbms_voltage_masks_one_missing(2u, 3u, true);
    sil_run_voltage_sample(&app);
    sil_run_voltage_sample(&app);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_fault == true);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_PEC_FAILURE);
    CHECK(app.bms_state == false);

    sil_prepare_cli_capture();
    CHECK(get_faults(0, NULL) == 0);
    CHECK(strstr(cli_capture, "voltage: fault:1") != NULL);
    CHECK(strstr(cli_capture, "valid:0") != NULL);
    CHECK(strstr(cli_capture, "pec_failure") != NULL);

    sil_prepare_cli_capture();
    CHECK(get_voltage(0, NULL) == 0);
    CHECK(strstr(cli_capture, "Voltage valid:0 fault:1") != NULL);
    CHECK(strstr(cli_capture, "reason:pec_failure") != NULL);
    CHECK(strstr(cli_capture, "stale:") != NULL);

    app.board.canbus.hcan = &hcan;
    tx_count = 0u;
    tx_free_level = 3u;
    CHECK(send_ecu_ams_voltages(&app.board.canbus, &app) == HAL_OK);
    CHECK(tx_count == 25u);
    uint32_t frame = (uint32_t)(2u * 5u) + (uint32_t)(3u / 3u);
    uint8_t word = (uint8_t)(3u % 3u);
    CHECK(word_at(frame, word + 1u) == 0u);

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    sil_run_current_adc_status(&app, HAL_TIMEOUT, HAL_OK);
    for(uint8_t i = 0u; i < 25u; i++)
    {
        sil_run_current_adc_status(&app, HAL_TIMEOUT, HAL_OK);
    }
    CHECK(app.current_fault == true);
    CHECK(app.current_fault_reason == CURRENT_FAULT_REASON_SENSOR_ADC_READ);
    sil_prepare_cli_capture();
    CHECK(get_current(0, NULL) == 0);
    CHECK(strstr(cli_capture, "valid:0") != NULL);
    CHECK(strstr(cli_capture, "reason:adc_read") != NULL || strstr(cli_capture, "sensor_adc_read") != NULL);
    CHECK(strstr(cli_capture, "ADC map L:PC0 ADC2_IN10 50A H:PA3 ADC1_IN3 800A") != NULL);
}

static void test_current_service_calibration_boundary(void)
{
    init_fake_app();
    app.board.current_sensor.zero_calibrated = true;
    app.board.current_sensor.zero_offset_50a = 1.0f;
    app.board.current_sensor.zero_offset_800a = 2.0f;
    app.bms_output_inhibit = false;
    app.bms_state = true;
    app.state = STATE_DISCARGE;
    char *clear_args[] = {"current", "zero", "clear", NULL};

    sil_prepare_cli_capture();
    CHECK(get_current(3, clear_args) == 0);
    CHECK(strstr(cli_capture, "refused") != NULL);
    CHECK(app.board.current_sensor.zero_calibrated == true);

    app.bms_output_inhibit = true;
    app.bms_state = false;
    app.state = STATE_START;
    ams_current_window_init(&app.current_window, fake_tick);
    ams_current_window_update(&app.current_window,
                              fake_tick,
                              0.0f,
                              0.0f,
                              true,
                              true,
                              91u);
    app.current_valid = true;
    app.current_meas_reason = CURRENT_SENSOR_REASON_OK;

    sil_prepare_cli_capture();
    CHECK(get_current(3, clear_args) == 0);
    CHECK(strstr(cli_capture, "cleared") != NULL);
    CHECK(app.board.current_sensor.zero_calibrated == false);
    CHECK(app.board.current_sensor.current_valid == false);
    CHECK(app.current_valid == false);
    CHECK(app.current_meas_reason ==
          CURRENT_SENSOR_REASON_CALIBRATION_CHANGED);
    CHECK(app.current_window.active.invalid_sample_count == 1u);
    CHECK(app.current_window.last_sample_valid == false);
}

static void test_cli_numeric_and_telemetry_conversion_guards(void)
{
    static CAN_HandleTypeDef hcan;
    uint16_t repeat = 77u;
    int whole = 123;
    int decimal = 456;

    init_fake_app();
    sil_prepare_cli_capture();

    /* Malformed channel strings must not fall through atoi() as channel 0 or
     * execute any mux delays/SPI activity. */
    uint32_t tick_before = fake_tick;
    char *bad_ic[] = {"tempsns", "abc", "0", NULL};
    CHECK(get_temperature_sensor(3, bad_ic) == 0);
    CHECK(fake_tick == tick_before);
    CHECK(strstr(cli_capture, "Error: ic") != NULL);

    sil_prepare_cli_capture();
    char *bad_sensor[] = {"tempsns", "0", "1junk", NULL};
    CHECK(get_temperature_sensor(3, bad_sensor) == 0);
    CHECK(fake_tick == tick_before);
    CHECK(strstr(cli_capture, "sensor out of range") != NULL);

    sil_prepare_cli_capture();
    char *overflow_ic[] = {"tempsns", "999999999999999999999999", "0", NULL};
    CHECK(get_temperature_sensor(3, overflow_ic) == 0);
    CHECK(fake_tick == tick_before);
    CHECK(strstr(cli_capture, "Error: ic") != NULL);

    CHECK(!cli_parse_scope_repeat("12junk", &repeat));
    CHECK(repeat == 77u);
    CHECK(!cli_parse_scope_repeat("101", &repeat));
    CHECK(repeat == 77u);
    CHECK(cli_parse_scope_repeat("100", &repeat));
    CHECK(repeat == 100u);

    cli_fixed1(NAN, &whole, &decimal);
    CHECK(whole == 0 && decimal == 0);
    cli_fixed3(INFINITY, &whole, &decimal);
    CHECK(whole == 0 && decimal == 0);
    cli_fixed1(1.0e30f, &whole, &decimal);
    CHECK(whole == (INT_MAX / 10) && decimal == (INT_MAX % 10));
    cli_fixed1(-1.0e30f, &whole, &decimal);
    CHECK(whole == (INT_MIN / 10) && decimal == abs(INT_MIN % 10));

    app.board.canbus.hcan = &hcan;
    app.temp_valid = true;
    app.current = NAN;
    app.board.imd.duty = INFINITY;
    app.max_temp = NAN;
    app.min_voltage = -INFINITY;
    app.max_voltage = INFINITY;
    app.board.fans[0].duty_cycle = INFINITY;
    tx_count = 0u;
    tx_free_level = 3u;
    CHECK(send_ecu_ams_status(&app.board.canbus, &app) == HAL_OK);
    CHECK(tx_count == 3u);
    CHECK(word_at(0u, 3u) == 0u);
    CHECK(word_at(1u, 3u) == 0u);
    CHECK(word_at(2u, 1u) == ECU_TEMP_INVALID_DECI_C);
    CHECK(word_at(2u, 2u) == 0u);
    CHECK(word_at(2u, 3u) == 0u);

    tx_count = 0u;
    CHECK(send_ecu_ams_fans(&app.board.canbus, &app) == HAL_OK);
    CHECK(word_at(0u, 1u) == 0u);

    app.current = 1.0e30f;
    app.max_temp = -1.0e30f;
    app.min_voltage = 1.0e30f;
    app.max_voltage = 1.0e30f;
    tx_count = 0u;
    CHECK(send_ecu_ams_status(&app.board.canbus, &app) == HAL_OK);
    CHECK((int16_t)word_at(0u, 3u) == INT16_MAX);
    CHECK((int16_t)word_at(2u, 1u) == INT16_MIN);
    CHECK(word_at(2u, 2u) == UINT16_MAX);
    CHECK(word_at(2u, 3u) == UINT16_MAX);

    CHECK(current_task_abs_deciamps(NAN) == 0u);
    CHECK(current_task_abs_deciamps(1.0e30f) == UINT32_MAX);

    fan_t fan = {0};
    uint32_t ccr = 0u;
    static TIM_HandleTypeDef htim;
    CHECK(fan_init(&fan, NULL, &htim, UINT32_MAX, &ccr, 1) == 0);
    CHECK(set_fan_percent(&fan, 100.0f) == 0);
    CHECK(ccr == UINT32_MAX);
}

static void test_adbms6830_diagnostic_commands_and_cli_health(void)
{
    static SPI_HandleTypeDef hspi;
    char *cfgchk[] = {"spi", "cfgchk"};
    char *cellst[] = {"spi", "cellst"};
    char *stat[] = {"spi", "stat"};
    char *oweven[] = {"spi", "oweven"};
    char *owodd[] = {"spi", "owodd"};
    char *auxdiag[] = {"spi", "auxdiag"};
    char *diagclear[] = {"spi", "diagclear"};

    init_fake_app();
    hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    app.acc.smb.hspi = &hspi;
    app.acc.apm.hspi = &hspi;
    app.acc.smb.num_ics = NSMBS;
    app.acc.smb.ics = app.acc.smb_ics;
    app.acc.smb.monitored_cell_count = NCELLS;
    fake_adbms_diag_status = HAL_OK;
    fake_adbms_config_mismatch_mask = 0x0002u;

    app.acc.smb.health.sticky_pec_fail_mask = 0x0004u;
    app.acc.smb.health.last_pec_pass_mask = 0x001Bu;
    app.acc.smb.health.last_pec_fail_mask = 0x0004u;
    app.acc.smb.health.sticky_cmd_counter_mismatch_mask = 0x0008u;
    app.acc.smb.health.pec_pass_count[0] = 12u;
    app.acc.smb.health.pec_fail_count[2] = 3u;
    app.acc.smb.health.cmd_counter_mismatch_count[3] = 2u;

    sil_prepare_cli_capture();
    get_spi_debug(2, stat);
    CHECK(strstr(cli_capture, "ADAX + RDSTATA-E safety status: OK") != NULL);
    CHECK(strstr(cli_capture, "topology logical:5 physical:6 monitored_cells:15") != NULL);
    CHECK(strstr(cli_capture, "diag startup:1 refresh:1") != NULL);
    CHECK(strstr(cli_capture, "IC0 refs A:1 B:1 valid:1") != NULL);
    CHECK(app.acc.smb.health.diagnostic_refresh_count == 1u);

    sil_prepare_cli_capture();
    get_spi_debug(2, cfgchk);
    CHECK(strstr(cli_capture, "CFGA/CFGB readback check status: OK") != NULL);
    CHECK(strstr(cli_capture, "diag op:config_check") != NULL);
    CHECK(strstr(cli_capture, "cfgB:0x0002 cfg:0x0002") != NULL);
    CHECK(strstr(cli_capture, "IC1 health") != NULL);
    CHECK(app.acc.smb.health.config_readback_count == 1u);
    CHECK(app.acc.smb.health.config_mismatch_count[1] == 1u);

    sil_prepare_cli_capture();
    get_spi_debug(2, cellst);
    CHECK(strstr(cli_capture, "Cell ADC conversion diagnostic status: OK") != NULL);
    CHECK(strstr(cli_capture, "diag op:cell_adc_diag") != NULL);
    CHECK(app.acc.smb.health.cell_adc_self_test_count == 1u);

    app.state = STATE_CHARGE;
    sil_prepare_cli_capture();
    get_spi_debug(2, oweven);
    CHECK(strstr(cli_capture, "Open-wire even-channel result: OK") != NULL);
    CHECK(strstr(cli_capture, "diag op:open_wire_even") != NULL);
    CHECK(app.acc.smb.health.open_wire_even_count == 1u);
    CHECK(app.acc.smb.health.open_wire_baseline_count == 1u);
    CHECK(app.acc.smb.health.open_wire_baseline_valid_ic_mask == 0x001Fu);

    sil_prepare_cli_capture();
    get_spi_debug(2, owodd);
    CHECK(strstr(cli_capture, "Open-wire odd-channel result: OK") != NULL);
    CHECK(strstr(cli_capture, "diag op:open_wire_odd") != NULL);
    CHECK(app.acc.smb.health.open_wire_odd_count == 1u);
    CHECK(app.acc.smb.health.open_wire_baseline_count == 2u);

    sil_prepare_cli_capture();
    get_spi_debug(2, auxdiag);
    CHECK(strstr(cli_capture, "AUX/GPIO diagnostic hook status: OK") != NULL);
    CHECK(strstr(cli_capture, "diag op:aux_gpio_diag") != NULL);
    CHECK(app.acc.smb.health.aux_gpio_diag_count == 1u);

    sil_prepare_cli_capture();
    get_spi_debug(2, diagclear);
    CHECK(strstr(cli_capture, "diagnostic counters cleared") != NULL);
    CHECK(strstr(cli_capture, "safety latches require reset") != NULL);
    CHECK(app.acc.smb.health.config_readback_count == 0u);
    CHECK(app.acc.smb.health.sticky_pec_fail_mask == 0u);
    CHECK(app.acc.smb.health.last_status == HAL_OK);
    CHECK(app.acc.smb.health.startup_baseline_passed == true);

    fake_adbms_diag_status = HAL_ERROR;
    sil_prepare_cli_capture();
    get_spi_debug(2, cellst);
    CHECK(strstr(cli_capture, "Cell ADC conversion diagnostic status: ERROR") != NULL);
    CHECK(strstr(cli_capture, "status:ERROR") != NULL);
    fake_adbms_diag_status = HAL_OK;
    fake_adbms_config_mismatch_mask = 0u;
}

static void test_adbms_periodic_diagnostics_and_safe_open_wire(void)
{
    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    app.state = STATE_DISCARGE;
    app.current_valid = true;
    app.current_fault = false;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;

    /* Diagnostics are scheduled from monotonic time, independent of scan
     * frequency. The first iteration establishes the deadlines. */
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_status_diag_count == 0u);
    CHECK(app.adbms_config_diag_count == 0u);
    CHECK(app.adbms_open_wire_diag_count == 0u);
    CHECK(app.adbms_diag_fault == false);
    CHECK(app.temp_policy_last_elapsed_ms == AMS_ADBMS_TASK_PERIOD_MS);

    fake_tick = app.adbms_status_diag_next_tick + 37u;
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_status_diag_count == 1u);
    CHECK(app.adbms_config_diag_count == 0u);
    CHECK(app.adbms_open_wire_diag_count == 0u);
    CHECK(app.adbms_diag_fault == false);
    CHECK(app.adbms_diag_last_lateness_ms == 37u);
    CHECK(app.temp_policy_last_elapsed_ms > AMS_ADBMS_TASK_PERIOD_MS);

    CHECK(app.adbms_config_diag_count == 0u);

    fake_adbms_config_mismatch_mask = 0x0004u;
    fake_tick = app.adbms_config_diag_next_tick;
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_config_diag_count == 1u);
    CHECK(app.adbms_config_fault == true);
    CHECK(app.adbms_diag_fault == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    fake_adbms_config_mismatch_mask = 0u;
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_config_diag_count == 1u);
    CHECK(app.adbms_config_fault == true);
    CHECK(app.adbms_diag_fault == true);

    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    app.state = STATE_BALANCE;
    app.current_valid = true;
    app.current_fault = false;
    app.bms_state = true;
    run_one_adbms_task_iteration(&app);
    fake_tick = app.adbms_open_wire_diag_next_tick;
    fill_nominal_pack(&app, 3.700f);
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_open_wire_diag_count == 1u);
    CHECK(app.acc.smb.health.open_wire_even_count == 1u);
    CHECK(app.acc.smb.health.open_wire_odd_count == 1u);
    CHECK(app.acc.smb.health.open_wire_full_count == 1u);
    CHECK(app.acc.smb.health.open_wire_baseline_count == 1u);
    CHECK(app.acc.smb.health.open_wire_baseline_valid_ic_mask == 0x001Fu);
    CHECK(app.adbms_open_wire_fault == false);

    /* An incomplete/failed diagnostic is a reboot latch. A later clean
     * diagnostic may refresh channel data but cannot silently re-authorize the
     * pack during the same boot. */
    fake_adbms_diag_status = HAL_ERROR;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    fake_tick = app.adbms_open_wire_diag_next_tick;
    fill_nominal_pack(&app, 3.700f);
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_open_wire_fault == true);
    CHECK(app.adbms_diag_fault == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    fake_adbms_diag_status = HAL_OK;
    fake_tick = app.adbms_open_wire_diag_next_tick;
    fill_nominal_pack(&app, 3.700f);
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_open_wire_fault == true);
    CHECK(app.adbms_diag_fault == true);
    CHECK(app.bms_state == false);

	init_fake_app();
	fill_nominal_pack(&app, 3.700f);
	app.state = STATE_DISCARGE;
	app.current_valid = true;
	app.bms_state = true;
	bms_pin_state = GPIO_PIN_SET;
	app.acc.delay_timer_ready = false;
	app.acc.delay_timer_status = HAL_ERROR;
	run_one_adbms_task_iteration(&app);
	fake_tick = app.adbms_status_diag_next_tick;
	run_one_adbms_task_iteration(&app);
	CHECK(app.adbms_status_fault == true);
	CHECK(app.adbms_diag_fault == true);
	CHECK(app.bms_state == false);
	CHECK(bms_pin_state == GPIO_PIN_RESET);

	init_fake_app();
	fill_nominal_pack(&app, 3.700f);
	app.state = STATE_DISCARGE;
	app.current_valid = true;
	app.bms_state = true;
	bms_pin_state = GPIO_PIN_SET;
	app.acc.smb_ready = false;
	app.acc.smb_init_status = HAL_TIMEOUT;
	run_one_adbms_task_iteration(&app);
	fake_tick = app.adbms_status_diag_next_tick;
	run_one_adbms_task_iteration(&app);
	CHECK(app.adbms_status_fault == true);
	CHECK(app.adbms_diag_fault == true);
	CHECK(app.bms_state == false);
	CHECK(bms_pin_state == GPIO_PIN_RESET);
}

static void test_adbms_cli_scan_guard_and_cs_probe_commands(void)
{
    static SPI_HandleTypeDef hspi;
    static GPIO_TypeDef cs_a;
    static GPIO_TypeDef cs_b;
    uint32_t rx_count_before_guarded_command;
    char *probe[] = {"spi", "probe"};
    char *probea[] = {"spi", "probea"};
    char *probeb[] = {"spi", "probeb"};
    char *cspins[] = {"spi", "cspins", "both", "3"};
    char *scope[] = {"spi", "scope", "b", "read", "20"};
    char *scope_default[] = {"spi", "scope"};
    char *preset_status[] = {"spi", "preset", "status"};
    char *preset_cmd[] = {"spi", "preset", "cmd"};
    char *preset_toggle[] = {"spi", "toggle"};
    char *oweven[] = {"spi", "oweven"};
    char *owcheck[] = {"spi", "owcheck"};
    char *apm_probe[] = {"apm", "probe"};

    init_fake_app();
    hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    accumulator_init(&app.acc, &hspi, &cs_a, &cs_b, 0x0002u, 0x0004u, NULL);
    CHECK(app.acc.smb.cs_port[STRING_A] == &cs_a);
    CHECK(app.acc.smb.cs_port[STRING_B] == &cs_b);
    CHECK(app.acc.smb.cs_pin[STRING_A] == 0x0002u);
    CHECK(app.acc.smb.cs_pin[STRING_B] == 0x0004u);
	CHECK(app.acc.smb.string == STRING_A);
    app.acc.smb.hspi = &hspi;
    app.acc.smb.num_ics = NSMBS;
    app.acc.smb.ics = app.acc.smb_ics;
	app.acc.smb.string = STRING_A;
	fake_counter_resync_calls = 0u;

    app.adbms_scan_active = true;
    rx_count_before_guarded_command = app.acc.smb.spi_debug.rx_count;
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, probe) == 0);
    CHECK(strstr(cli_capture, "spi probe refused") != NULL);
    CHECK(app.acc.smb.spi_debug.rx_count == rx_count_before_guarded_command);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(5, scope) == 0);
    CHECK(strstr(cli_capture, "spi scope refused") != NULL);
    CHECK(app.acc.smb.spi_debug.rx_count == rx_count_before_guarded_command);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(4, cspins) == 0);
    CHECK(strstr(cli_capture, "spi cspins refused") != NULL);

    app.adbms_scan_active = false;
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(4, cspins) == 0);
    CHECK(strstr(cli_capture, "pulsing PE4 block, then PF4 block") != NULL);
    CHECK(strstr(cli_capture, "PE4/PF4 left idle high") != NULL);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, (char *[]){"spi", "pins"}) == 0);
    CHECK(strstr(cli_capture, "CS_B:PE4") != NULL);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, (char *[]){"spi", "cspins"}) == 0);
    CHECK(strstr(cli_capture, "alternating PE4 then PF4") != NULL);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, probea) == 0);
    CHECK(strstr(cli_capture, "CS_A/stringA probe status: OK") != NULL);
    CHECK(app.acc.smb.spi_debug.last_string == STRING_A);
	CHECK(app.acc.smb.string == STRING_A);
	CHECK(fake_counter_resync_calls == 1u);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, probeb) == 0);
	CHECK(strstr(cli_capture, "ADBMS2950 RDSID CS_B/stringB probe status: OK") != NULL);
	CHECK(app.acc.apm.spi_debug.last_op == ADBMS2950_SPI_OP_PROBE);
	CHECK(app.acc.apm.spi_debug.rx_count == 1u);
	CHECK(app.acc.smb.string == STRING_A);
	CHECK(fake_counter_resync_calls == 2u);

    adbms6830_spi_debug_clear(&app.acc.smb);
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(5, scope) == 0);
    CHECK(strstr(cli_capture, "scope string:CS_B mode:read repeat:20 status:OK") != NULL);
    CHECK(strstr(cli_capture, "Probe MCU: SCK PG13") != NULL);
    CHECK(app.acc.smb.spi_debug.last_op == ADBMS6830_SPI_OP_SCOPE);
    CHECK(app.acc.smb.spi_debug.last_string == STRING_B);
    CHECK(app.acc.smb.spi_debug.rx_count == 20u);
	CHECK(app.acc.smb.string == STRING_A);
	CHECK(fake_counter_resync_calls == 3u);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(3, preset_status) == 0);
	CHECK(strstr(cli_capture, "scope preset string:CS_A mode:read repeat:20") != NULL);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(3, preset_cmd) == 0);
	CHECK(strstr(cli_capture, "scope preset string:CS_A mode:cmd repeat:50") != NULL);

    adbms6830_spi_debug_clear(&app.acc.smb);
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, scope_default) == 0);
	CHECK(strstr(cli_capture, "scope string:CS_A mode:cmd repeat:50 status:OK") != NULL);
    CHECK(app.acc.smb.spi_debug.last_op == ADBMS6830_SPI_OP_SCOPE);
	CHECK(app.acc.smb.spi_debug.last_string == STRING_A);
    CHECK(app.acc.smb.spi_debug.tx_count == 50u);
    CHECK(app.acc.smb.spi_debug.rx_count == 0u);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, preset_toggle) == 0);
	CHECK(strstr(cli_capture, "scope preset string:CS_A mode:pattern repeat:20") != NULL);

    app.state = STATE_START;
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, oweven) == 0);
    CHECK(strstr(cli_capture, "Open-wire refused") != NULL);
    CHECK(app.acc.smb.health.open_wire_even_count == 0u);

    app.state = STATE_DISCARGE;
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, oweven) == 0);
    CHECK(strstr(cli_capture, "Open-wire even-channel result: OK") != NULL);
    CHECK(app.acc.smb.health.open_wire_even_count == 1u);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, owcheck) == 0);
    CHECK(strstr(cli_capture, "Open-wire full even+odd result: OK") != NULL);
    CHECK(app.acc.smb.health.open_wire_full_count == 1u);
    CHECK(app.acc.smb.health.open_wire_even_count == 2u);
    CHECK(app.acc.smb.health.open_wire_odd_count == 1u);
    CHECK(app.acc.smb.health.last_op == ADBMS6830_SPI_OP_OPEN_WIRE_FULL);

    fake_adbms_diag_status = HAL_ERROR;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, owcheck) == 0);
    CHECK(strstr(cli_capture, "Open-wire full even+odd result: ERROR") != NULL);
    CHECK(app.adbms_open_wire_fault == true);
    CHECK(app.adbms_diag_fault == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    fake_adbms_diag_status = HAL_OK;

    app.acc.apm.hspi = &hspi;
    app.acc.apm.num_ics = 1u;
    app.adbms_scan_active = true;
    sil_prepare_cli_capture();
    CHECK(get_apm_debug(2, apm_probe) == 0);
    CHECK(strstr(cli_capture, "apm probe refused") != NULL);
}

static void test_software_heartbeat_monitor_faults_and_recovery(void)
{
    init_fake_app();
    fake_tick = 1000u;
    ams_heartbeat_init(&app, fake_tick);
    sil_make_measurement_gates_ready(&app);
    app.state = STATE_DISCARGE;
    app.current_fault_mode = CURRENT_FAULT_MODE_DRIVE;
    app.current_overcurrent_fault = false;
    app.current_fault_latched = false;
    app.fuse_fault = false;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;

    for(uint8_t i = 0u; i < (uint8_t)AMS_HEARTBEAT_COUNT; i++)
    {
        ams_heartbeat_kick(&app, (ams_heartbeat_id_t)i, fake_tick);
    }

    run_one_error_task_iteration(&app);
    CHECK(app.task_heartbeat_fault == false);
    CHECK(app.logger_heartbeat_fault == false);
    CHECK(app.hard_fault == false);
    CHECK(app.bms_state == true);

    fake_tick = 2000u;
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_ADBMS, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_TEMP, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CAN, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_LOGGER, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_IMD, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_FAN, fake_tick);
    fake_tick += AMS_HEARTBEAT_CURRENT_TIMEOUT_MS + 1u;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;

    run_one_error_task_iteration(&app);
    CHECK(app.task_heartbeat_fault == true);
    CHECK((app.heartbeat_stale_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_CURRENT)) != 0u);
    CHECK(app.hard_fault == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CURRENT, fake_tick);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    run_one_error_task_iteration(&app);
    CHECK(app.task_heartbeat_fault == false);
    CHECK((app.heartbeat_stale_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_CURRENT)) == 0u);
    CHECK(app.hard_fault == false);

    /* A fan-task stall is independently safety critical even while the
     * temperature producer and every other critical service remain alive. */
    fake_tick += AMS_HEARTBEAT_FAN_TIMEOUT_MS + 1u;
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_ADBMS, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CURRENT, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_TEMP, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CAN, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_LOGGER, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_IMD, fake_tick);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    run_one_error_task_iteration(&app);
    CHECK(app.task_heartbeat_fault == true);
    CHECK((app.heartbeat_stale_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_FAN)) != 0u);
    CHECK(app.bms_state == false);

    ams_heartbeat_kick(&app, AMS_HEARTBEAT_FAN, fake_tick);
    run_one_error_task_iteration(&app);
    CHECK(app.task_heartbeat_fault == false);
    CHECK((app.heartbeat_stale_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_FAN)) == 0u);

    init_fake_app();
    fake_tick = 5000u;
    ams_heartbeat_init(&app, fake_tick);
    sil_make_measurement_gates_ready(&app);
    app.state = STATE_DISCARGE;
    app.current_fault_mode = CURRENT_FAULT_MODE_DRIVE;
    app.current_overcurrent_fault = false;
    app.current_fault_latched = false;
    app.fuse_fault = false;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_ADBMS, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CURRENT, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_TEMP, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CAN, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_LOGGER, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_IMD, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_FAN, fake_tick);
    fake_tick += AMS_HEARTBEAT_LOGGER_TIMEOUT_MS + 1u;
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_ADBMS, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CURRENT, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_TEMP, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CAN, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_IMD, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_FAN, fake_tick);

    run_one_error_task_iteration(&app);
    CHECK(app.task_heartbeat_fault == false);
    CHECK(app.logger_heartbeat_fault == true);
    CHECK((app.heartbeat_stale_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_LOGGER)) != 0u);
    CHECK(app.hard_fault == false);
    CHECK(app.soft_fault == true);
    CHECK(app.bms_state == true);

    init_fake_app();
    fake_tick = 10000u;
    ams_heartbeat_init(&app, fake_tick);
    sil_make_measurement_gates_ready(&app);
    app.state = STATE_DISCARGE;
    app.current_fault_mode = CURRENT_FAULT_MODE_DRIVE;
    app.current_overcurrent_fault = false;
    app.current_fault_latched = false;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    fake_tick += AMS_HEARTBEAT_STARTUP_GRACE_MS - 1u;
    run_one_error_task_iteration(&app);
    CHECK(app.task_heartbeat_fault == false);
    CHECK(app.bms_state == true);

    fake_tick = 10000u + AMS_HEARTBEAT_STARTUP_GRACE_MS + 1u;
    run_one_error_task_iteration(&app);
    CHECK(app.task_heartbeat_fault == true);
    CHECK((app.heartbeat_stale_mask & AMS_HEARTBEAT_SAFETY_MASK) == AMS_HEARTBEAT_SAFETY_MASK);
    CHECK(app.bms_state == false);
}

static void test_supervisor_rejects_non_operating_and_corrupt_states(void)
{
    static const state_t rejected_states[] =
    {
        STATE_NULL,
        STATE_ERROR,
        (state_t)0x7F
    };

    for(size_t i = 0u; i < (sizeof(rejected_states) / sizeof(rejected_states[0])); i++)
    {
        init_fake_app();
        fake_tick = 1000u;
        sil_make_measurement_gates_ready(&app);
        sil_mark_all_heartbeats_alive(&app);
        app.current_overcurrent_fault = false;
        app.current_fault_latched = false;
        app.state = rejected_states[i];
        app.bms_state = true;
        bms_pin_state = GPIO_PIN_SET;

        error_task_update(&app, fake_tick);

        CHECK(app.bms_supervisor_ready == false);
        CHECK(app.bms_state == false);
        CHECK(bms_pin_state == GPIO_PIN_RESET);
        if(i == 2u)
        {
            CHECK(app.state == STATE_ERROR);
        }
    }
}

static void test_system_sil_bringup_status_and_bmsok_inhibit(void)
{
    static SPI_HandleTypeDef hspi;
    char *status_argv[] = {"bmsok", "status", NULL};
    char *release_argv[] = {"bmsok", "release", NULL};
    char *inhibit_argv[] = {"bmsok", "inhibit", NULL};
    char *balance_status_argv[] = {"balance", "status", NULL};
    char *balance_inhibit_argv[] = {"balance", "inhibit", NULL};
    char *balance_release_argv[] = {"balance", "release", NULL};

    init_fake_app();
    hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    app.acc.smb.hspi = &hspi;
    app.acc.apm.hspi = &hspi;
    app.current_valid = true;
    app.current_fault = false;
    app.voltage_valid = true;
    app.voltage_fault = false;
    app.bms_output_inhibit = true;
    bms_pin_state = GPIO_PIN_RESET;

    set_bms(true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    CHECK(app.bms_output_block_count == 1u);

    sil_prepare_cli_capture();
    CHECK(bmsok_control(2, status_argv) == 0);
    CHECK(strstr(cli_capture, "inhibit:1") != NULL);
    CHECK(strstr(cli_capture, "blocked_assertions:1") != NULL);

    sil_prepare_cli_capture();
    CHECK(bmsok_control(2, release_argv) == 0);
    CHECK(app.bms_output_inhibit == false);
    CHECK(strstr(cli_capture, "release enabled") != NULL);
    set_bms(true);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);

    sil_prepare_cli_capture();
    CHECK(bmsok_control(2, inhibit_argv) == 0);
    CHECK(app.bms_output_inhibit == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    sil_prepare_cli_capture();
    CHECK(get_status(0, NULL) == 0);
    CHECK(strstr(cli_capture, "BMS_OK:0 inhibit:1") != NULL);
    CHECK(strstr(cli_capture, "balance_inhibit:") != NULL);
    CHECK(strstr(cli_capture, "CPOL:HIGH CPHA:2EDGE") != NULL);

    app.acc.smb_ics[0].PwmA.pwma[1] = BALANCE_PWM_DUTY;
    sil_prepare_cli_capture();
    CHECK(balance_control(2, balance_status_argv) == 0);
#if AMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT
    CHECK(strstr(cli_capture, "balance inhibit:1") != NULL);
#else
    CHECK(strstr(cli_capture, "balance inhibit:0") != NULL);
#endif

    sil_prepare_cli_capture();
    CHECK(balance_control(2, balance_inhibit_argv) == 0);
    CHECK(app.balance_inhibit == true);
    CHECK(strstr(cli_capture, "Balancing inhibited") != NULL);
    sil_expect_balancing_clear(&app);

    sil_prepare_cli_capture();
    CHECK(balance_control(2, balance_release_argv) == 0);
    CHECK(app.balance_inhibit == false);
    CHECK(strstr(cli_capture, "release enabled") != NULL);
}

static void test_bringup_cli_board_ready_and_adbms_summaries(void)
{
    static SPI_HandleTypeDef hspi;
    char *board_argv[] = {"bringup", "board", NULL};
    char *ready_argv[] = {"bringup", "ready", NULL};
    char *adbms_argv[] = {"bringup", "adbms6830", NULL};

    init_fake_app();
    hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    app.acc.smb.hspi = &hspi;
    app.acc.apm.hspi = &hspi;
    app.bms_output_inhibit = true;
    app.board.current_sensor.last_read_ok = true;
    app.board.current_sensor.sensor_voltage_high = 2.50f;
    app.board.current_sensor.sensor_voltage_low = 2.50f;
    app.board.current_sensor.count_high = 1861u;
    app.board.current_sensor.count_low = 1861u;
    app.current_valid = true;
    app.current = 0.0f;
    app.current_meas_reason = CURRENT_SENSOR_REASON_OK;

    sil_prepare_cli_capture();
    CHECK(get_bringup(2, board_argv) == 0);
    CHECK(strstr(cli_capture, "BRINGUP BOARD") != NULL);
    CHECK(strstr(cli_capture, "spi6=PASS") != NULL);
    CHECK(strstr(cli_capture, "current_zero=PASS") != NULL);
    CHECK(strstr(cli_capture, "expected_not_ready_without_cells") != NULL);

    sil_prepare_cli_capture();
    CHECK(get_bringup(2, ready_argv) == 0);
    CHECK(strstr(cli_capture, "BRINGUP READY") != NULL);
    CHECK(strstr(cli_capture, "release_allowed=NO") != NULL);
    CHECK(strstr(cli_capture, "does not run bmsok release") != NULL);
    CHECK(app.bms_output_inhibit == true);
    CHECK(app.bms_state == false);

    app.acc.smb.spi_debug.enabled = true;
    app.acc.smb.spi_debug.rx_count = 1u;
    app.acc.smb.spi_debug.last_status = HAL_OK;
    app.acc.smb.spi_debug.last_xfer_status = HAL_OK;
    memset(app.acc.smb.spi_debug.last_rx_preview, 0xFF, sizeof(app.acc.smb.spi_debug.last_rx_preview));
    app.acc.smb.spi_debug.last_read_pec_fail_mask = 0x001Fu;

    sil_prepare_cli_capture();
    CHECK(get_bringup(2, adbms_argv) == 0);
    CHECK(strstr(cli_capture, "BRINGUP ADBMS6830") != NULL);
    CHECK(strstr(cli_capture, "startup=PASS init:OK timer=PASS") != NULL);
    CHECK(strstr(cli_capture, "mode=PASS") != NULL);
    CHECK(strstr(cli_capture, "response=FAIL all_ff") != NULL);
    CHECK(strstr(cli_capture, "pec=FAIL") != NULL);

    for(uint8_t i = 0u; i < ADBMS6830_SPI_DEBUG_PREVIEW_BYTES; i++)
    {
        app.acc.smb.spi_debug.last_rx_preview[i] = (uint8_t)(0x20u + i);
    }
    app.acc.smb.spi_debug.last_read_pec_fail_mask = 0u;
    app.acc.smb.spi_debug.last_read_pec_pass_mask = 0x001Fu;
    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        app.acc.smb.diag[ic].sid_valid = true;
        app.acc.smb.diag[ic].statc_valid = true;
        app.acc.smb.diag[ic].statd_valid = true;
        app.acc.smb.diag[ic].state_valid = true;
    }

    sil_prepare_cli_capture();
    CHECK(get_bringup(2, adbms_argv) == 0);
    CHECK(strstr(cli_capture, "response=PASS changing") != NULL);
    CHECK(strstr(cli_capture, "pec=PASS") != NULL);
    CHECK(strstr(cli_capture, "sid=PASS") != NULL);
    CHECK(strstr(cli_capture, "stat=PASS") != NULL);
}

static void test_bringup_cli_apm_and_charger_phase_split(void)
{
    static SPI_HandleTypeDef hspi;
    static CAN_HandleTypeDef hcan;
    char *apm_argv[] = {"bringup", "apm2950", NULL};
    char *charger_lv_argv[] = {"bringup", "charger-lv", NULL};
    char *charger_battery_argv[] = {"bringup", "charger-battery", NULL};

    init_fake_app();
	app.acc.apm_ready = false;
	app.acc.apm_init_status = HAL_ERROR;
	app.acc.apm.health.initialized = false;
    sil_prepare_cli_capture();
    CHECK(get_bringup(2, apm_argv) == 0);
    CHECK(strstr(cli_capture, "FINAL_RING APM2950") != NULL);
    CHECK(strstr(cli_capture, "initialized:0") != NULL);
    CHECK(strstr(cli_capture, "ADVISORY_NON_GATING") != NULL);

    hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    app.acc.apm.hspi = &hspi;
    app.acc.apm.num_ics = 1u;
	app.acc.apm_ready = true;
	app.acc.apm_init_status = HAL_OK;
	app.acc.apm.health.initialized = true;
	app.acc.apm.health.sid_valid = true;
	app.acc.apm.health.config_valid = true;
    app.acc.apm.spi_debug.rx_count = 1u;
    app.acc.apm.spi_debug.last_status = HAL_OK;
    memset(app.acc.apm.spi_debug.last_rx_preview, 0xFF, sizeof(app.acc.apm.spi_debug.last_rx_preview));

    sil_prepare_cli_capture();
    CHECK(get_bringup(2, apm_argv) == 0);
    CHECK(strstr(cli_capture, "initialized:1") != NULL);
    CHECK(strstr(cli_capture, "mode=PASS") != NULL);
    CHECK(strstr(cli_capture, "response=FAIL all_ff") != NULL);
    CHECK(strstr(cli_capture, "scaling=UNPROVEN") != NULL);
	CHECK(strstr(cli_capture, "sid:1 cfg:1 dividers:OFF") != NULL);

    init_fake_app();
    charger_init(&app.board.charger, &app.board.canbus);
    sil_prepare_cli_capture();
    CHECK(get_bringup(2, charger_lv_argv) == 0);
    CHECK(strstr(cli_capture, "BRINGUP CHARGER_LV") != NULL);
    CHECK(strstr(cli_capture, "BYTE5/data[4] 0=allow 1=disable") != NULL);
    CHECK(strstr(cli_capture, "allow_frame:0C 30 00 64 00") != NULL);
    CHECK(strstr(cli_capture, "timeout_test=TODO") != NULL);

    sil_prepare_cli_capture();
    CHECK(get_bringup(2, charger_battery_argv) == 0);
    CHECK(strstr(cli_capture, "BRINGUP CHARGER_BATTERY") != NULL);
    CHECK(strstr(cli_capture, "verdict=BLOCKED") != NULL);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.rx_count = 1u;
    app.board.charger.last_rx_tick = fake_tick;
    app.board.charger.disable_reason_mask = CHARGER_DISABLE_REASON_NONE;

    sil_prepare_cli_capture();
    CHECK(get_bringup(2, charger_battery_argv) == 0);
    CHECK(strstr(cli_capture, "verdict=PASS") != NULL);
    CHECK(strstr(cli_capture, "charger_clean:1") != NULL);
}

static void test_system_sil_contradictory_dhab_vs_2950_observable_non_gating(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    app.acc.apm.current[0] = 425.0f;
    app.acc.apm.current[1] = -425.0f;
    app.acc.apm.spi_debug.enabled = true;
    app.acc.apm.spi_debug.last_status = HAL_OK;
    app.acc.apm.spi_debug.rx_count = 12u;

    sil_run_current_sample(&app, 0.0f);
    sil_run_voltage_sample(&app);
    CHECK(app.current_valid == true);
    CHECK(fabsf(app.current) < 0.05f);
    CHECK(fabsf(app.acc.apm.current[0] - app.current) > 400.0f);
    CHECK(app.acc.apm.spi_debug.rx_count == 12u);
    CHECK(app.current_fault == false);
    CHECK(app.voltage_fault == false);
    CHECK(app.bms_state == true);
    sil_assert_safety_invariants(&app, "2950_contradiction_debug_only");
}

static void test_system_sil_startup_garbage_never_enables_bms(void)
{
    uint32_t rng = 0xBADC0DEu;

    for(uint16_t iter = 0u; iter < 256u; iter++)
    {
        init_fake_app();
        fake_tick = 0u;
        bms_pin_state = GPIO_PIN_RESET;
        app.state = (iter & 1u) ? STATE_CHARGE : STATE_DISCARGE;
        sil_attach_current_adcs(&app);
        sil_clear_voltage_history(&app);

        for(uint8_t ic = 0u; ic < NSMBS; ic++)
        {
            for(uint8_t cell = 0u; cell < NCELLS; cell++)
            {
                uint32_t r = sil_rng_next(&rng);
                switch(r & 7u)
                {
                    case 0u: app.acc.smb_ics[ic].cell.c_codes[cell] = 0; break;
                    case 1u: app.acc.smb_ics[ic].cell.c_codes[cell] = INT16_MIN; break;
                    case 2u: app.acc.smb_ics[ic].cell.c_codes[cell] = code_for_volts(4.300f); break;
                    case 3u: app.acc.smb_ics[ic].cell.c_codes[cell] = code_for_volts(2.200f); break;
                    default: app.acc.smb_ics[ic].cell.c_codes[cell] = code_for_volts(3.200f + ((float)(r & 0x3FFu) / 1023.0f)); break;
                }
            }
        }

        if((iter % 3u) == 0u)
        {
            fake_adbms_voltage_masks_all_missing((iter & 4u) != 0u);
        }
        else
        {
            fake_adbms_voltage_masks_one_missing((uint8_t)(iter % NSMBS),
                                                 (uint8_t)(iter % NCELLS),
                                                 true);
        }

        if((iter % 5u) == 0u)
        {
            sil_run_current_adc_status(&app, HAL_TIMEOUT, HAL_ERROR);
        }
        else
        {
            fake_adc_set_two_read_sequence((uint16_t)(sil_rng_next(&rng) & 0x0FFFu),
                                           (uint16_t)(sil_rng_next(&rng) & 0x0FFFu));
            run_one_current_task_iteration(&app);
        }
        sil_run_voltage_sample(&app);
        CHECK(app.acc.voltage_startup_scan_complete == false);
        CHECK(app.voltage_valid == false);
        CHECK(app.bms_state == false);
        CHECK(bms_pin_state == GPIO_PIN_RESET);
        sil_assert_diagnostics_not_generic(&app, "startup_garbage");
    }
}

static void test_system_sil_long_run_seeded_fuzz_invariants(void)
{
    static CAN_HandleTypeDef hcan;
    uint32_t rng = 0x5EED1234u;
    uint32_t bms_true_seen = 0u;
    uint32_t bms_false_seen = 0u;
    uint32_t current_fault_seen = 0u;
    uint32_t voltage_fault_seen = 0u;
    uint32_t invalid_current_seen = 0u;
    uint32_t pec_voltage_seen = 0u;
    uint32_t balance_inhibit_seen = 0u;

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    sil_attach_fans(&app);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;

    for(uint32_t cycle = 0u; cycle < AMS_HOST_LONG_FUZZ_CYCLES; cycle++)
    {
        if((cycle % 125u) == 0u)
        {
            sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
            app.board.canbus.hcan = &hcan;
            charger_init(&app.board.charger, &app.board.canbus);
            app.board.charger.last_rx_tick = fake_tick;
        }

        uint32_t r = sil_rng_next(&rng);
        app.state = (r & 0x3u) == 0u ? STATE_CHARGE :
                    (r & 0x3u) == 1u ? STATE_DISCARGE :
                    (r & 0x3u) == 2u ? STATE_START : STATE_BALANCE;
        app.board.charger.last_rx_tick = fake_tick;
        app.fuse_fault = (((r >> 8) & 0x3Fu) == 0x11u) || (((r >> 16) & 0x7Fu) == 0x22u);
        app.charger_fault = false;
        app.hard_fault = false;
        app.balance_inhibit = (((r >> 28) & 0x7u) == 0x3u);
        if(app.balance_inhibit)
        {
            app.acc.smb_ics[(r >> 1) % NSMBS].tx_cfgb.dcc = (uint16_t)(1u << ((r >> 4) % 15u));
            app.acc.smb_ics[(r >> 6) % NSMBS].PwmA.pwma[(r >> 9) % PWMA] = BALANCE_PWM_DUTY;
            app.acc.smb_ics[(r >> 11) % NSMBS].PwmB.pwmb[(r >> 14) % PWMB] = BALANCE_PWM_DUTY;
        }

        sil_write_all_cells(&app, 3.700f);
        switch((r >> 20) % 12u)
        {
            case 0u: sil_set_cell_voltage(&app, (uint8_t)(r % NSMBS), (uint8_t)((r >> 4) % NCELLS), 4.250f); break;
            case 1u: sil_set_cell_voltage(&app, (uint8_t)(r % NSMBS), (uint8_t)((r >> 4) % NCELLS), 4.200f); break;
            case 2u: sil_set_cell_voltage(&app, (uint8_t)(r % NSMBS), (uint8_t)((r >> 4) % NCELLS), 4.180f); break;
            case 3u: sil_set_cell_voltage(&app, (uint8_t)(r % NSMBS), (uint8_t)((r >> 4) % NCELLS), 2.300f); break;
            case 4u: sil_set_cell_voltage(&app, (uint8_t)(r % NSMBS), (uint8_t)((r >> 4) % NCELLS), 2.500f); break;
            case 5u: sil_set_cell_voltage(&app, (uint8_t)(r % NSMBS), (uint8_t)((r >> 4) % NCELLS), 2.800f); break;
            default: break;
        }

        if((r & 0x1Fu) == 0u)
        {
            fake_adbms_voltage_masks_all_missing(true);
        }
        else if((r & 0x0Fu) == 0u)
        {
            fake_adbms_voltage_masks_one_missing((uint8_t)(r % NSMBS),
                                                 (uint8_t)((r >> 4) % NCELLS),
                                                 true);
        }
        else
        {
            fake_adbms_voltage_masks_full_update();
        }

        uint32_t csel = (r >> 5) % 16u;
        float current_a = 0.0f;
        bool inject_adc_fail = false;
        switch(csel)
        {
            case 0u: current_a = 0.0f; break;
            case 1u: current_a = 75.0f; break;
            case 2u: current_a = 86.0f; break;
            case 3u: current_a = 130.0f; break;
            case 4u: current_a = 260.0f; break;
            case 5u: current_a = -6.0f; break;
            case 6u: current_a = -12.5f; break;
            case 7u: current_a = -16.0f; break;
            case 8u: current_a = -31.0f; break;
            case 9u: current_a = -55.0f; break;
            case 10u: current_a = 0.9f; break;
            case 11u: current_a = 2.1f; break;
            case 12u: inject_adc_fail = true; break;
            default: current_a = 0.0f; break;
        }

        uint8_t order = (uint8_t)((r >> 12) % 6u);
        for(uint8_t step = 0u; step < 4u; step++)
        {
            uint8_t op;
            static const uint8_t perms[6][4] = {
                {0u, 1u, 2u, 3u}, {0u, 2u, 1u, 3u}, {1u, 0u, 2u, 3u},
                {1u, 2u, 0u, 3u}, {2u, 0u, 1u, 3u}, {2u, 1u, 0u, 3u}
            };
            op = perms[order][step];
            if(op == 0u)
            {
                if(inject_adc_fail)
                {
                    sil_run_current_adc_status(&app, HAL_TIMEOUT, HAL_OK);
                }
                else
                {
                    sil_run_current_sample(&app, current_a);
                }
            }
            else if(op == 1u)
            {
                sil_run_voltage_sample(&app);
            }
            else if(op == 2u)
            {
                sil_run_can_charge_iteration(&app, &hcan);
            }
            else
            {
                run_one_error_task_iteration(&app);
            }
        }

        sil_assert_safety_invariants(&app, "long_run_seeded_fuzz");
        bms_true_seen += app.bms_state ? 1u : 0u;
        bms_false_seen += app.bms_state ? 0u : 1u;
        current_fault_seen += app.current_fault ? 1u : 0u;
        voltage_fault_seen += app.voltage_fault ? 1u : 0u;
        invalid_current_seen += app.current_valid ? 0u : 1u;
        pec_voltage_seen += (app.voltage_fault_reason == VOLTAGE_FAULT_REASON_PEC_FAILURE) ? 1u : 0u;
        balance_inhibit_seen += app.balance_inhibit ? 1u : 0u;
    }

    CHECK(bms_true_seen > 0u);
    CHECK(bms_false_seen > 0u);
    CHECK(current_fault_seen > 0u);
    CHECK(voltage_fault_seen > 0u);
    CHECK(invalid_current_seen > 0u);
    CHECK(pec_voltage_seen > 0u);
    CHECK(balance_inhibit_seen > 0u);
}

static void test_system_sil_concurrent_heartbeat_starvation_and_recovery(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    for(uint16_t i = 0u; i < 180u; i++)
    {
        sil_run_current_sample(&app, 0.0f);
    }
    CHECK(app.current_valid == true);
    CHECK(app.bms_state == true);

    run_one_error_task_iteration(&app);
    CHECK(app.task_heartbeat_fault == true);
    CHECK((app.heartbeat_stale_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_ADBMS)) != 0u);
    CHECK((app.heartbeat_stale_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_TEMP)) != 0u);
    CHECK((app.heartbeat_stale_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_CAN)) != 0u);
    CHECK((app.heartbeat_stale_mask & AMS_HEARTBEAT_BIT(AMS_HEARTBEAT_CURRENT)) == 0u);
    CHECK(app.bms_state == false);
    sil_assert_safety_invariants(&app, "heartbeat_starved_current_only");

    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.task_heartbeat_fault == false);
    CHECK(app.voltage_valid == true);
    CHECK(app.temp_valid == true);
    /* The safety supervisor now owns assertion and is evaluated after this
     * simulated ADBMS publish, so readiness recovers in one supervisor pass. */
    CHECK(app.bms_state == true);

    run_one_error_task_iteration(&app);
    CHECK(app.hard_fault == false);
    CHECK(app.task_heartbeat_fault == false);
    CHECK(app.bms_state == true);

    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);
    sil_assert_safety_invariants(&app, "heartbeat_recovered_after_adbms");

    sil_mark_all_heartbeats_alive(&app);
    fake_tick += AMS_HEARTBEAT_LOGGER_TIMEOUT_MS + 1u;
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_ADBMS, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CURRENT, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_TEMP, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CAN, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_IMD, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_FAN, fake_tick);
    run_one_error_task_iteration(&app);
    CHECK(app.task_heartbeat_fault == false);
    CHECK(app.logger_heartbeat_fault == true);
    CHECK(app.hard_fault == false);
    CHECK(app.bms_state == true);
    sil_assert_safety_invariants(&app, "logger_heartbeat_starved_only");
}

static void test_system_sil_concurrent_charger_tx_recovery_ordering(void)
{
    static CAN_HandleTypeDef hcan;

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;

    fake_can_add_tx_status = HAL_ERROR;
    sil_run_can_charge_iteration(&app, &hcan);
    CHECK(app.board.charger.tx_fail == true);
    CHECK(app.board.charger.last_tx_status == HAL_ERROR);
    CHECK(app.board.charger.tx_fail_count == 1u);
    CHECK(app.canbus_fault == true);
    CHECK(app.charger_fault == true);
    CHECK(app.bms_state == false);
    sil_assert_safety_invariants(&app, "charger_tx_fail_drops_bms");

    fake_can_add_tx_status = HAL_OK;
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == false);
    CHECK(app.charger_fault == true);

    sil_run_can_charge_iteration(&app, &hcan);
    CHECK(app.board.charger.tx_fail == false);
    CHECK(app.board.charger.last_tx_status == HAL_OK);
    CHECK(app.charger_fault == true);
    CHECK(app.bms_state == false);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == CHARGER_CMD_DISABLE);
    sil_assert_safety_invariants(&app, "charger_tx_first_success_still_conservative");

    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == false);
    sil_run_can_charge_iteration(&app, &hcan);
    CHECK(app.charger_fault == false);
    CHECK(app.bms_state == false);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == CHARGER_CMD_DISABLE);

    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);
    sil_assert_safety_invariants(&app, "charger_tx_recovered_after_fault_clear");

    /* CAN soft-fault recovery is time-based and must not accidentally depend
     * on the selected ADBMS scan profile. */
    fake_tick = app.can_last_error_tick + AMS_CAN_ERROR_SOFT_HOLD_MS + 1u;
    app.board.charger.last_rx_tick = fake_tick;
    sil_run_can_charge_iteration(&app, &hcan);
    CHECK(app.charger_fault == false);
    CHECK(app.canbus_fault == false);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == CHARGER_CMD_ENABLE);
    CHECK(app.bms_state == true);
    sil_assert_safety_invariants(&app, "charger_tx_enable_after_recovery");
}

static void test_system_sil_concurrent_seeded_scheduler_abuse(void)
{
    static CAN_HandleTypeDef hcan;
    uint32_t rng = 0xC0A11E57u;
    uint32_t bms_true_seen = 0u;
    uint32_t bms_false_seen = 0u;
    uint32_t heartbeat_fault_seen = 0u;
    uint32_t canbus_fault_seen = 0u;
    uint32_t temp_fault_seen = 0u;
    uint32_t fan_max_seen = 0u;
    uint32_t charger_tx_fail_seen = 0u;

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;

    for(uint32_t cycle = 0u; cycle < AMS_HOST_CONCURRENT_FUZZ_CYCLES; cycle++)
    {
        if((cycle % 125u) == 0u)
        {
            sil_prepare_ready_system((cycle & 0x80u) ? STATE_CHARGE : STATE_DISCARGE, 0.0f, 3.700f);
            sil_attach_fans(&app);
            app.board.canbus.hcan = &hcan;
            charger_init(&app.board.charger, &app.board.canbus);
            app.board.charger.last_rx_tick = fake_tick;
            fake_can_add_tx_status = HAL_OK;
            tx_free_level = 3u;
        }

        uint32_t r = sil_rng_next(&rng);
        if((r & 0x1Fu) == 0u)
        {
            sil_force_state_for_scheduler_abuse(&app, STATE_CHARGE);
        }
        else if((r & 0x1Fu) == 1u)
        {
            sil_force_state_for_scheduler_abuse(&app, STATE_DISCARGE);
        }
        else if((r & 0x3Fu) == 2u)
        {
            sil_force_state_for_scheduler_abuse(&app, STATE_BALANCE);
        }

        if((r & 0x7Fu) == 0x24u)
        {
            app.board.charger.last_rx_tick = fake_tick - (CHARGER_RX_TIMEOUT_MS + 1u);
        }
        else if(app.state == STATE_CHARGE)
        {
            app.board.charger.last_rx_tick = fake_tick;
        }

        if((r & 0xFFu) == 0x5Au)
        {
            app.fuse_fault = true;
        }
        else if((cycle % 125u) == 1u)
        {
            app.fuse_fault = false;
        }

        switch((r >> 8) % 8u)
        {
            case 0u: sil_set_all_temps(&app, 25.0f, (1UL << NTEMPS) - 1UL); break;
            case 1u: sil_set_all_temps(&app, 50.0f, (1UL << NTEMPS) - 1UL); break;
            case 2u: sil_set_all_temps(&app, 61.0f, (1UL << NTEMPS) - 1UL); break;
            case 3u: sil_set_all_temps(&app, 25.0f, 0u); break;
            default: break;
        }
        if((app.state == STATE_CHARGE) &&
           (app.temp_charge_stop || app.temp_fault || !app.temp_valid))
        {
            set_bms(false);
        }

        sil_write_all_cells(&app, 3.700f);
        switch((r >> 12) % 10u)
        {
            case 0u: sil_set_cell_voltage(&app, (uint8_t)(r % NSMBS), (uint8_t)((r >> 4) % NCELLS), 4.200f); break;
            case 1u: sil_set_cell_voltage(&app, (uint8_t)(r % NSMBS), (uint8_t)((r >> 4) % NCELLS), 2.500f); break;
            case 2u: sil_set_cell_voltage(&app, (uint8_t)(r % NSMBS), (uint8_t)((r >> 4) % NCELLS), 4.180f); break;
            default: break;
        }

        if((r & 0x3Fu) == 0x11u)
        {
            fake_adbms_voltage_masks_all_missing(true);
        }
        else if((r & 0x1Fu) == 0x0Cu)
        {
            fake_adbms_voltage_masks_one_missing((uint8_t)(r % NSMBS),
                                                 (uint8_t)((r >> 4) % NCELLS),
                                                 true);
        }
        else
        {
            fake_adbms_voltage_masks_full_update();
        }

        uint8_t op = (uint8_t)((r >> 20) % 8u);
        if(op == 0u)
        {
            float current_a;
            switch((r >> 24) % 8u)
            {
                case 0u: current_a = 0.0f; break;
                case 1u: current_a = 75.0f; break;
                case 2u: current_a = 130.0f; break;
                case 3u: current_a = 260.0f; break;
                case 4u: current_a = -6.0f; break;
                case 5u: current_a = -12.5f; break;
                case 6u: current_a = -31.0f; break;
                default: current_a = -55.0f; break;
            }
            sil_run_current_sample(&app, current_a);
        }
        else if(op == 1u)
        {
            sil_run_current_adc_status(&app,
                                       ((r >> 3) & 1u) ? HAL_TIMEOUT : HAL_OK,
                                       ((r >> 4) & 1u) ? HAL_ERROR : HAL_OK);
        }
        else if(op == 2u)
        {
            sil_run_voltage_sample(&app);
        }
        else if(op == 3u)
        {
            fake_can_add_tx_status = ((r >> 5) & 1u) ? HAL_ERROR : HAL_OK;
            tx_free_level = ((r >> 6) & 1u) ? 0u : 3u;
            sil_run_can_charge_iteration(&app, &hcan);
            if(fake_can_add_tx_status != HAL_OK || tx_free_level == 0u)
            {
                charger_tx_fail_seen += (app.state == STATE_CHARGE) ? 1u : 0u;
            }
            fake_can_add_tx_status = HAL_OK;
            tx_free_level = 3u;
        }
        else if(op == 4u)
        {
            run_one_error_task_iteration(&app);
        }
        else if(op == 5u)
        {
            run_one_fan_task_iteration(&app);
        }
        else if(op == 6u)
        {
            fake_tick += (r & 1u) ? (AMS_HEARTBEAT_CURRENT_TIMEOUT_MS + 1u) :
                                    (AMS_HEARTBEAT_CAN_TIMEOUT_MS + 1u);
            run_one_error_task_iteration(&app);
        }
        else
        {
            sil_run_voltage_sample(&app);
            run_one_error_task_iteration(&app);
        }

        sil_assert_safety_invariants(&app, "concurrent_seeded_scheduler_abuse");
        bms_true_seen += app.bms_state ? 1u : 0u;
        bms_false_seen += app.bms_state ? 0u : 1u;
        heartbeat_fault_seen += app.task_heartbeat_fault ? 1u : 0u;
        canbus_fault_seen += app.canbus_fault ? 1u : 0u;
        temp_fault_seen += app.temp_fault ? 1u : 0u;
        fan_max_seen += (app.fan_state && (app.board.fans[0].duty_cycle >= 99.9f)) ? 1u : 0u;
    }

    fake_can_add_tx_status = HAL_OK;
    tx_free_level = 3u;

    CHECK(bms_true_seen > 0u);
    CHECK(bms_false_seen > 0u);
    CHECK(heartbeat_fault_seen > 0u);
    CHECK(canbus_fault_seen > 0u);
    CHECK(temp_fault_seen > 0u);
    CHECK(fan_max_seen > 0u);
    CHECK(charger_tx_fail_seen > 0u);
}

static void test_temp_invalid_and_cold_valid_fault_behavior(void){
    fake_mux_write_enable = 0;

    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    sil_clear_temp_history(&app);
    for(int ic=0; ic<NSMBS; ic++) for(int s=0; s<NTEMPS; s++) app.acc.smb_ics[ic].temp.raw[s] = -1;
    app.state = STATE_DISCARGE; app.bms_state = true; bms_pin_state = GPIO_PIN_SET; fake_tick = 0;
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == false);
    CHECK(app.temp_fault == true);
    CHECK(app.acc.valid_temp_count == 0u);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    sil_set_all_temps(&app, 0.0f, (1UL << NTEMPS) - 1UL);
    app.state = STATE_DISCARGE; app.current_valid = true; app.bms_state = true; bms_pin_state = GPIO_PIN_SET; fake_tick = 0;
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == false);
    CHECK(app.temp_fault == false);
    CHECK(app.acc.valid_temp_count == (uint16_t)(NSMBS * NTEMPS));
    CHECK(app.max_temp > -2.0f && app.max_temp < 2.0f);
    CHECK(app.temp_charge_stop == true);
    CHECK(app.temp_warning == true);
    CHECK(app.temp_fault_reason == TEMPERATURE_FAULT_REASON_COLD_CHARGE_STOP);
    CHECK(app.bms_state == true);

    fake_mux_write_enable = 1;
}

static void test_system_sil_temperature_mux_cadence_no_false_stale(void)
{
    init_fake_app();
    fake_tick = 0u;
    fill_nominal_pack(&app, 3.700f);
    sil_attach_current_adcs(&app);
    sil_clear_temp_history(&app);

    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        for(uint8_t sensor = 0u; sensor < NTEMPS; sensor++)
        {
            app.acc.smb_ics[ic].temp.raw[sensor] = raw_for_temp_c(25.0f);
        }
    }

    for(uint8_t sweep = 0u; sweep < 2u; sweep++)
    {
        for(uint8_t mux = 0u; mux < (NTEMPS / 3u); mux++)
        {
            uint32_t mask = (1UL << mux) | (1UL << (mux + 8u)) | (1UL << (mux + 16u));
            host_mark_updated_temps(&app, mask);
            accumulator_update_temp_stats_at(&app.acc, fake_tick);
            sil_publish_temp_state(&app);

            CHECK(app.temp_invalid_sensor_count == 0u);
            CHECK(app.acc.temp_startup_scan_complete == (sweep > 0u || mux == ((NTEMPS / 3u) - 1u)));
            CHECK(app.temp_fault_reason != TEMPERATURE_FAULT_REASON_INVALID_SENSOR);
            if(sweep > 0u || mux == ((NTEMPS / 3u) - 1u))
            {
                CHECK(app.temp_valid == true);
                CHECK(app.temp_fault == false);
                CHECK(app.temp_usable_sensor_count == AMS_EXPECTED_TEMP_SENSOR_COUNT);
                CHECK(app.temp_warning == false);
                CHECK(app.temp_fault_reason == TEMPERATURE_FAULT_REASON_NONE);
            }
            else
            {
                CHECK(app.temp_valid == false);
                CHECK(app.temp_fault == true);
                CHECK(app.temp_fault_reason == TEMPERATURE_FAULT_REASON_NOT_READY);
            }

            fake_tick += 1000u;
        }
    }
}

static void test_system_sil_temperature_invalid_update_overrides_history(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    CHECK(accumulator_temp_sensor_usable(&app.acc, 2u, 5u));
    app.acc.smb_ics[2].temp.raw[5] = -1;
    app.acc.smb.last_temp_updated_mask[2] = (uint32_t)(1UL << 5);
    accumulator_update_temp_stats_at(&app.acc, fake_tick);
    sil_publish_temp_state(&app);

    CHECK(!accumulator_temp_sensor_usable(&app.acc, 2u, 5u));
    CHECK(app.temp_valid == false);
    CHECK(app.temp_fault == true);
    CHECK(app.temp_read_fault == true);
    CHECK(app.temp_invalid_sensor_count > 0u);
    CHECK(app.temp_fault_reason == TEMPERATURE_FAULT_REASON_INVALID_SENSOR);
    CHECK(temp_deci_c_for_ecu(&app, 2u, 5u) == ECU_TEMP_INVALID_DECI_C);
}

static void test_system_sil_temperature_startup_partial_and_stale_fail_closed(void)
{
    init_fake_app();
    fake_tick = 0u;
    bms_pin_state = GPIO_PIN_RESET;
    app.state = STATE_DISCARGE;
    sil_attach_current_adcs(&app);
    fill_nominal_pack(&app, 3.700f);
    sil_clear_temp_history(&app);
    sil_run_current_sample(&app, 0.0f);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;

    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.current_valid == true);
    CHECK(app.temp_valid == false);
    CHECK(app.temp_fault == true);
    CHECK(app.temp_fault_reason == TEMPERATURE_FAULT_REASON_NOT_READY);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    sil_set_all_temps(&app, 25.0f, (1UL << NTEMPS) - 1UL);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    fake_tick += ACCUMULATOR_TEMP_STALE_TIMEOUT_MS + 100u;
    run_one_adbms_task_iteration(&app);
    CHECK(app.temp_valid == false);
    CHECK(app.temp_fault == true);
    CHECK(app.temp_read_fault == true);
    CHECK(app.temp_fault_reason == TEMPERATURE_FAULT_REASON_STALE_SCAN);
    CHECK(app.bms_state == false);
}

static void test_system_sil_temperature_fan_ramp_warning_and_recovery(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    sil_attach_fans(&app);

    sil_set_all_temps(&app, 34.0f, (1UL << NTEMPS) - 1UL);
    run_one_fan_task_iteration(&app);
    CHECK(app.temp_valid == true);
    CHECK(app.temp_fault == false);
    CHECK(app.fan_state == false);
    for(int i = 0; i < NFANS; i++) CHECK(app.board.fans[i].duty_cycle == 0.0f);

    sil_set_all_temps(&app, 42.5f, (1UL << NTEMPS) - 1UL);
    run_one_fan_task_iteration(&app);
    CHECK(app.temp_fault == false);
    CHECK(app.temp_warning == false);
    CHECK(app.fan_state == true);
    for(int i = 0; i < NFANS; i++) CHECK(app.board.fans[i].duty_cycle > 49.0f && app.board.fans[i].duty_cycle < 51.0f);

    sil_set_all_temps(&app, TEMP_FAN_MAX_C, (1UL << NTEMPS) - 1UL);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    run_one_fan_task_iteration(&app);
    CHECK(app.temp_fault == false);
    CHECK(app.temp_warning == true);
    CHECK(app.temp_fan_max == true);
    CHECK(app.temp_charge_stop == true);
    CHECK(app.temp_fault_reason == TEMPERATURE_FAULT_REASON_HOT_FAN_MAX);
    CHECK(app.bms_state == true);
    for(int i = 0; i < NFANS; i++) CHECK(app.board.fans[i].duty_cycle == 100.0f);

    sil_set_all_temps(&app, 25.0f, (1UL << NTEMPS) - 1UL);
    run_one_fan_task_iteration(&app);
    CHECK(app.temp_fault == false);
    CHECK(app.temp_warning == false);
    CHECK(app.temp_fan_max == false);
    CHECK(app.temp_charge_stop == false);
    CHECK(app.fan_state == false);
}

static void test_system_sil_temperature_charge_stop_thresholds(void)
{
    static CAN_HandleTypeDef hcan;

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.charger.last_rx_tick = fake_tick;

    sil_set_all_temps(&app, TEMP_CHARGE_MAX_C - 0.1f, (1UL << NTEMPS) - 1UL);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(&app);
    CHECK(app.temp_charge_stop == false);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 0u);

    sil_set_all_temps(&app, TEMP_CHARGE_MAX_C, (1UL << NTEMPS) - 1UL);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    app.board.charger.last_rx_tick = fake_tick;
    tx_count = 0u;
    run_one_canbus_task_iteration(&app);
    CHECK(app.temp_fault == false);
    CHECK(app.temp_charge_stop == true);
    CHECK(app.temp_fault_reason == TEMPERATURE_FAULT_REASON_HOT_CHARGE_STOP);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 1u);
    CHECK(app.bms_state == false);

    temperature_fault_reset_latch(&app.temp_fault_state);
    sil_set_all_temps(&app, TEMP_CHARGE_MIN_C, (1UL << NTEMPS) - 1UL);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    app.board.charger.last_rx_tick = fake_tick;
    tx_count = 0u;
    run_one_canbus_task_iteration(&app);
    CHECK(app.temp_fault == false);
    CHECK(app.temp_charge_stop == true);
    CHECK(app.temp_fault_reason == TEMPERATURE_FAULT_REASON_COLD_CHARGE_STOP);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == 1u);
    CHECK(app.bms_state == false);
}

static void test_system_sil_temperature_hard_latch_and_reset_path(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    sil_set_all_temps(&app, TEMP_HOT_HARD_C - 0.1f, (1UL << NTEMPS) - 1UL);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    run_one_adbms_task_iteration(&app);
    CHECK(app.temp_valid == true);
    CHECK(app.temp_fault == false);
    CHECK(app.bms_state == true);

    sil_set_all_temps(&app, TEMP_HOT_HARD_C, (1UL << NTEMPS) - 1UL);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    CHECK(app.temp_fault == false);
    CHECK(app.temp_overtemp_pending == true);
    CHECK(app.temp_fault_pending_reason == TEMPERATURE_FAULT_REASON_HOT_HARD);
    CHECK(app.temp_fault_pending_ms == TEMP_FAULT_DEFAULT_SAMPLE_MS);
    CHECK(app.bms_state == true);
    for(uint32_t guard = 0u;
        !app.temp_fault &&
        (guard <= ((TEMP_HOT_HARD_CONFIRM_MS / AMS_ADBMS_TASK_PERIOD_MS) + 1u));
        guard++)
    {
        temperature_fault_update_with_period(&app.temp_fault_state,
                                             &app.acc,
                                             AMS_ADBMS_TASK_PERIOD_MS);
        sil_copy_temp_state(&app);
    }
    sil_mark_all_heartbeats_alive(&app);
    error_task_update(&app, fake_tick);
    CHECK(app.temp_fault == true);
    CHECK(app.overtemp_fault == true);
    CHECK(app.temp_fault_latched == true);
    CHECK(app.temp_fault_latched_reason == TEMPERATURE_FAULT_REASON_HOT_HARD);
    CHECK(app.bms_state == false);

    sil_set_all_temps(&app, 25.0f, (1UL << NTEMPS) - 1UL);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    run_one_adbms_task_iteration(&app);
    CHECK(app.temp_valid == true);
    CHECK(app.temp_fault == true);
    CHECK(app.temp_fault_latched == true);
    CHECK(app.bms_state == false);

    temperature_fault_reset_latch(&app.temp_fault_state);
    sil_set_all_temps(&app, 25.0f, (1UL << NTEMPS) - 1UL);
    app.current_valid = true;
    app.current_fault = false;
    app.hard_fault = false;
    app.bms_state = false;
    bms_pin_state = GPIO_PIN_RESET;
    run_one_adbms_task_iteration(&app);
    sil_mark_all_heartbeats_alive(&app);
    error_task_update(&app, fake_tick);
    CHECK(app.temp_valid == true);
    CHECK(app.temp_fault == false);
    CHECK(app.temp_fault_latched == false);
    CHECK(app.bms_state == true);

    sil_set_all_temps(&app, TEMP_HOT_SEVERE_C, (1UL << NTEMPS) - 1UL);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    CHECK(app.temp_fault == false);
    CHECK(app.temp_overtemp_pending == true);
    CHECK(app.temp_fault_pending_reason == TEMPERATURE_FAULT_REASON_HOT_SEVERE);
    for(uint32_t guard = 0u;
        !app.temp_fault &&
        (guard <= ((TEMP_HOT_SEVERE_CONFIRM_MS / AMS_ADBMS_TASK_PERIOD_MS) + 1u));
        guard++)
    {
        temperature_fault_update_with_period(&app.temp_fault_state,
                                             &app.acc,
                                             AMS_ADBMS_TASK_PERIOD_MS);
        sil_copy_temp_state(&app);
    }
    sil_mark_all_heartbeats_alive(&app);
    error_task_update(&app, fake_tick);
    CHECK(app.temp_fault == true);
    CHECK(app.severe_overtemp_fault == true);
    CHECK(app.temp_fault_latched_reason == TEMPERATURE_FAULT_REASON_HOT_SEVERE);
    CHECK(app.bms_state == false);
}

static void test_system_sil_temperature_cli_can_diagnostics(void)
{
    static CAN_HandleTypeDef hcan;

    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);
    app.board.canbus.hcan = &hcan;
    sil_set_all_temps(&app, 25.0f, (1UL << NTEMPS) - 1UL);
    app.acc.smb_ics[2].temp.raw[5] = -1;
    app.acc.smb.last_temp_updated_mask[2] |= (uint32_t)(1UL << 5);
    accumulator_update_temp_stats_at(&app.acc, fake_tick);
    sil_publish_temp_state(&app);

    CHECK(app.temp_valid == false);
    CHECK(app.temp_fault == true);
    CHECK(app.temp_fault_reason == TEMPERATURE_FAULT_REASON_INVALID_SENSOR);

    sil_prepare_cli_capture();
    get_status(1, (char *[]){"status", NULL});
    CHECK(strstr(cli_capture, "Temps usable:") != NULL);
    CHECK(strstr(cli_capture, "chargestop:") != NULL);
    CHECK(strstr(cli_capture, "reason:") != NULL);
    cli_capture_clear();
    get_faults(1, (char *[]){"faults", NULL});
    CHECK(strstr(cli_capture, "temp: fault:1") != NULL);
    CHECK(strstr(cli_capture, "pending:") != NULL);

    tx_count = 0u;
    tx_free_level = 3u;
    CHECK(send_ecu_ams_temps(&app.board.canbus, &app) == HAL_OK);
    CHECK(tx_count == 30u);
    CHECK(word_at(13u, 3u) == ECU_TEMP_INVALID_DECI_C);

    tx_count = 0u;
    CHECK(send_ecu_ams_status(&app.board.canbus, &app) == HAL_OK);
    CHECK(tx_count == 3u);
    CHECK(word_at(2u, 1u) == ECU_TEMP_INVALID_DECI_C);
}

static void test_can_rx_filter_matrix(void){
    static CAN_HandleTypeDef hcan;
    init_fake_app();
    app.board.canbus.hcan = &hcan;
    charger_init(&app.board.charger, &app.board.canbus);

    CHECK(canbus_configure_rx_filters(NULL) == HAL_ERROR);
    CHECK(canbus_configure_rx_filters(&hcan) == HAL_OK);
#if AMS_ENABLE_HIL_CAN
    CHECK(fake_can_filter_count == 3u);
#else
    CHECK(fake_can_filter_count == 1u);
#endif
    CHECK(fake_can_filter_log[0].FilterBank == 0u);
    CHECK(fake_can_filter_log[0].FilterMode == CAN_FILTERMODE_IDMASK);
    CHECK(fake_can_filter_log[0].FilterScale == CAN_FILTERSCALE_32BIT);
    CHECK(fake_can_filter_log[0].FilterIdHigh ==
          ((CHARGER_RX_ID >> 13u) & 0xFFFFu));
    CHECK(fake_can_filter_log[0].FilterIdLow ==
          (((CHARGER_RX_ID << 3u) & 0xFFF8u) | CAN_ID_EXT));
    CHECK(fake_can_filter_log[0].FilterMaskIdHigh == 0xFFFFu);
    CHECK(fake_can_filter_log[0].FilterMaskIdLow == 0xFFFEu);
    CHECK(fake_can_filter_log[0].FilterFIFOAssignment == CAN_RX_FIFO0);
    CHECK(fake_can_filter_log[0].FilterActivation == ENABLE);
#if AMS_ENABLE_HIL_CAN
    CHECK(fake_can_filter_log[1].FilterBank == 1u);
    CHECK(fake_can_filter_log[1].FilterMode == CAN_FILTERMODE_IDLIST);
    CHECK(fake_can_filter_log[1].FilterScale == CAN_FILTERSCALE_16BIT);
    CHECK(fake_can_filter_log[1].FilterIdHigh ==
          ((AMS_HIL_CAN_ID_MEAS & 0x7FFu) << 5u));
    CHECK(fake_can_filter_log[1].FilterIdLow ==
          ((AMS_HIL_CAN_ID_TRUTH & 0x7FFu) << 5u));
    CHECK(fake_can_filter_log[1].FilterMaskIdHigh ==
          ((AMS_HIL_CAN_ID_SUMMARY & 0x7FFu) << 5u));
    CHECK(fake_can_filter_log[1].FilterMaskIdLow ==
          ((AMS_HIL_CAN_ID_CELL_SAMPLE & 0x7FFu) << 5u));
    CHECK(fake_can_filter_log[2].FilterBank == 2u);
    CHECK(fake_can_filter_log[2].FilterIdHigh ==
          ((AMS_HIL_CAN_ID_TEMP_SAMPLE & 0x7FFu) << 5u));
    CHECK(fake_can_filter_log[2].FilterIdLow ==
          fake_can_filter_log[2].FilterIdHigh);
    CHECK(fake_can_filter_log[2].FilterMaskIdHigh ==
          fake_can_filter_log[2].FilterIdHigh);
    CHECK(fake_can_filter_log[2].FilterMaskIdLow ==
          fake_can_filter_log[2].FilterIdHigh);
#endif

    fake_can_filter_count = 0u;
    fake_can_filter_status = HAL_ERROR;
    CHECK(canbus_configure_rx_filters(&hcan) == HAL_ERROR);
    CHECK(fake_can_filter_count == 0u);
    fake_can_filter_status = HAL_OK;

    fake_rx_status = HAL_ERROR;
    fake_rx_hdr.IDE = CAN_ID_EXT; fake_rx_hdr.ExtId = CHARGER_RX_ID; fake_rx_hdr.DLC = 8;
    memset(fake_rx_data, 0xAA, sizeof(fake_rx_data));
    (void)host_receive_can_frame(&hcan);
    CHECK(app.board.charger.rx_count == 0u);
    CHECK(app.board.charger.flags == 0u);

    fake_rx_status = HAL_OK;
    fake_rx_hdr.IDE = CAN_ID_STD; fake_rx_hdr.StdId = (uint32_t)CHARGER_RX_ID; fake_rx_hdr.DLC = 8;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.board.canbus.rx_packet.dlc == 0u);
    CHECK(app.board.canbus.rx_filtered_count == 1u);
    CHECK(app.board.charger.rx_count == 0u);

    fake_rx_hdr.IDE = CAN_ID_EXT; fake_rx_hdr.ExtId = CHARGER_RX_ID ^ 0x10u; fake_rx_hdr.DLC = 8;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.board.canbus.rx_packet.dlc == 0u);
    CHECK(app.board.canbus.rx_filtered_count == 2u);
    CHECK(app.board.charger.rx_count == 0u);

    fake_rx_hdr.IDE = CAN_ID_EXT; fake_rx_hdr.ExtId = CHARGER_RX_ID; fake_rx_hdr.DLC = 4;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.board.canbus.rx_filtered_count == 3u);
    CHECK(app.board.charger.rx_count == 0u);

    fake_rx_hdr.DLC = 8;
    fake_rx_data[0] = 0x0C; fake_rx_data[1] = 0x30; // 312.0 V
    fake_rx_data[2] = 0x00; fake_rx_data[3] = 0x0A; // 1.0 A
    fake_rx_data[4] = 0x0Fu;
    fake_tick = 7777u;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.board.charger.rx_count == 1u);
    CHECK(fabsf(app.board.charger.read_voltage - 312.0f) < 0.01f);
    CHECK(fabsf(app.board.charger.read_current - 1.0f) < 0.01f);
    CHECK(app.board.charger.hardware_fail && app.board.charger.overtemp_fail && app.board.charger.input_volt_fail && app.board.charger.voltage_sense_fail);
    CHECK(app.board.charger.communication_fail == false);
    CHECK(app.board.charger.last_rx_tick == 7777u);
}

static void test_can_rx_isr_queue_ownership_and_overflow(void)
{
    static CAN_HandleTypeDef hcan;
    static CAN_HandleTypeDef wrong_hcan;
    const uint32_t queue_capacity = CANBUS_RX_QUEUE_DEPTH - 1u;

    init_fake_app();
    canbus_device_init(&app.board.canbus, &hcan);
    charger_init(&app.board.charger, &app.board.canbus);

    fake_rx_hdr.IDE = CAN_ID_EXT;
    fake_rx_hdr.RTR = CAN_RTR_DATA;
    fake_rx_hdr.ExtId = CHARGER_RX_ID;
    fake_rx_hdr.DLC = 5u;
    fake_rx_data[0] = 0x0Cu;
    fake_rx_data[1] = 0x34u;
    fake_rx_data[2] = 0x00u;
    fake_rx_data[3] = 0x2Au;
    fake_rx_data[4] = 0x00u;
    fake_tick = 1234u;

    host_can_rx_isr_only(&hcan);
    CHECK(app.board.canbus.rx_isr_count == 1u);
    CHECK(canbus_rx_queue_count(&app.board.canbus) == 1u);
    CHECK(app.board.canbus.rx_packet.dlc == 0u);
    CHECK(app.board.charger.rx_count == 0u);
    CHECK(app.board.charger.read_voltage == 0.0f);

    /* Parsing happens in task context, and freshness uses receipt time rather
     * than a potentially delayed task-processing time. */
    fake_tick = 2000u;
    CHECK(canbus_process_rx_queue(&app.board.canbus, &app, 1u) == 1u);
    CHECK(app.board.canbus.rx_processed_count == 1u);
    CHECK(canbus_rx_queue_count(&app.board.canbus) == 0u);
    CHECK(app.board.charger.rx_count == 1u);
    CHECK(fabsf(app.board.charger.read_voltage - 312.4f) < 0.01f);
    CHECK(app.board.charger.last_rx_tick == 1234u);
    CHECK(app.board.canbus.rx_packet.tick == 1234u);

    /* A callback from a different CAN peripheral must not consume or publish
     * a frame into this device's queue. */
    host_can_rx_isr_only(&wrong_hcan);
    CHECK(app.board.canbus.rx_isr_count == 1u);
    CHECK(canbus_rx_queue_count(&app.board.canbus) == 0u);

    /* Remote request frames carry no charger status payload. */
    fake_rx_hdr.RTR = CAN_RTR_REMOTE;
    fake_tick = 2100u;
    host_can_rx_isr_only(&hcan);
    CHECK(canbus_process_rx_queue(&app.board.canbus, &app, 1u) == 0u);
    CHECK(app.board.canbus.rx_filtered_count == 1u);
    CHECK(app.board.canbus.rx_packet.rtr == CAN_RTR_DATA);
    CHECK(app.board.charger.rx_count == 1u);

#if AMS_ENABLE_HIL_CAN
    /* HIL state follows the same ownership rule: no ISR-side mutation. */
    memset(&fake_rx_hdr, 0, sizeof(fake_rx_hdr));
    memset(fake_rx_data, 0, sizeof(fake_rx_data));
    fake_rx_hdr.IDE = CAN_ID_STD;
    fake_rx_hdr.RTR = CAN_RTR_DATA;
    fake_rx_hdr.StdId = AMS_HIL_CAN_ID_MEAS;
    fake_rx_hdr.DLC = 7u;
    fake_rx_data[0] = 0x7Bu;
    fake_rx_data[1] = 0x00u;
    fake_tick = 2200u;
    host_can_rx_isr_only(&hcan);
    CHECK(app.hil.meas.fresh == 0u);
    CHECK(canbus_process_rx_queue(&app.board.canbus, &app, 1u) == 1u);
    CHECK(app.hil.meas.fresh == 1u);
    CHECK(app.hil.meas.last_rx_tick == 2200u);
#endif

    /* A HAL receive failure is counted in the ISR, then converted into an
     * application-visible soft CAN fault by task context. */
    fake_rx_status = HAL_ERROR;
    host_can_rx_isr_only(&hcan);
    CHECK(app.board.canbus.rx_hal_error_count == 1u);
    CHECK(app.canbus_fault == false);
    CHECK(canbus_process_rx_queue(&app.board.canbus, &app, 0u) == 0u);
    CHECK(app.canbus_fault == true);
    CHECK(app.can_error_count == 1u);
    CHECK((app.can_error_code & HAL_CAN_ERROR_RX_FOV0) != 0u);
    CHECK(strcmp(canbus_error_str(app.can_error_code), "rx_fifo0_overrun") == 0);

    /* Reinitialize, fill past ring capacity without task service, and verify
     * exact loss accounting plus FIFO order during bounded draining. */
    init_fake_app();
    canbus_device_init(&app.board.canbus, &hcan);
    fake_rx_hdr.IDE = CAN_ID_EXT;
    fake_rx_hdr.RTR = CAN_RTR_DATA;
    fake_rx_hdr.ExtId = CHARGER_RX_ID;
    fake_rx_hdr.DLC = 5u;

    for(uint32_t i = 0u; i < (CANBUS_RX_QUEUE_DEPTH + 5u); i++)
    {
        fake_tick = i;
        fake_rx_data[0] = (uint8_t)i;
        host_can_rx_isr_only(&hcan);
    }

    CHECK(app.board.canbus.rx_isr_count == (CANBUS_RX_QUEUE_DEPTH + 5u));
    CHECK(canbus_rx_queue_count(&app.board.canbus) == queue_capacity);
    CHECK(app.board.canbus.rx_queue_high_water == queue_capacity);
    CHECK(app.board.canbus.rx_queue_drop_count == 6u);
    CHECK(app.board.canbus.rx_packet.dlc == 0u);

    CHECK(canbus_process_rx_queue(&app.board.canbus, &app, 7u) == 7u);
    CHECK(app.board.canbus.rx_packet.data[0] == 6u);
    CHECK(canbus_rx_queue_count(&app.board.canbus) == (queue_capacity - 7u));
    CHECK(app.canbus_fault == true);
    CHECK(app.can_error_count == 6u);
    CHECK((app.can_error_code & HAL_CAN_ERROR_RX_FOV0) != 0u);

    CHECK(canbus_process_rx_queue(&app.board.canbus, &app, CANBUS_RX_QUEUE_DEPTH) ==
          (queue_capacity - 7u));
    CHECK(canbus_rx_queue_count(&app.board.canbus) == 0u);
    CHECK(app.board.canbus.rx_processed_count == queue_capacity);
    CHECK(app.board.canbus.rx_packet.data[0] == (uint8_t)(queue_capacity - 1u));

    /* Exercise the physical end of the array after both indices stopped at
     * slot 95, proving that enqueue/dequeue preserve order across wrap. */
    for(uint8_t i = 0u; i < 12u; i++)
    {
        fake_rx_data[0] = (uint8_t)(0xA0u + i);
        host_can_rx_isr_only(&hcan);
    }
    CHECK(canbus_rx_queue_count(&app.board.canbus) == 12u);
    CHECK(canbus_process_rx_queue(&app.board.canbus, &app, 12u) == 12u);
    CHECK(app.board.canbus.rx_packet.data[0] == 0xABu);
    CHECK(canbus_rx_queue_count(&app.board.canbus) == 0u);

    /* Loss counters are edge-accounted; repeatedly servicing an empty queue
     * must not count the same six dropped frames again. */
    CHECK(canbus_process_rx_queue(&app.board.canbus, &app, CANBUS_RX_QUEUE_DEPTH) == 0u);
    CHECK(app.can_error_count == 6u);
}



static float estimator_expected_voltage(const ams_ekf_config_t *cfg, float soc, float current_A, float temp_C, float r0_ohm){
    float i_cell = current_A / cfg->parallel_cell_count;
    return (float)cfg->series_group_count * (ams_p42a_ocv_v(soc, temp_C) - (r0_ohm * i_cell));
}

static void test_measurement_epoch_contract(void)
{
    ams_current_window_accumulator_t current_acc;
    ams_current_window_t completed;

    ams_current_window_init(&current_acc, 0u);
    ams_current_window_update(&current_acc,
                              10u,
                              10.0f,
                              9.0f,
                              true,
                              true,
                              42u);
    ams_current_window_update(&current_acc,
                              30u,
                              20.0f,
                              19.0f,
                              true,
                              true,
                              42u);
    CHECK(ams_current_window_rotate(&current_acc, 50u, &completed));
    CHECK(completed.valid);
    CHECK(completed.sample_count == 2u);
    CHECK(completed.invalid_sample_count == 0u);
    CHECK(fabs(completed.charge_As - 0.8) < 1.0e-9);
    CHECK(fabs(completed.total_charge_As - 0.8) < 1.0e-9);
    CHECK(fabsf(completed.average_A - 16.0f) < 1.0e-5f);
    CHECK(fabsf(completed.latest_A - 20.0f) < 1.0e-5f);
    CHECK(fabsf(completed.filtered_A - 19.0f) < 1.0e-5f);
    CHECK(completed.calibration_record_confident);
    CHECK(completed.calibration_id == 42u);

    /* The held sample from the previous boundary used record 42. A new
     * record inside this epoch must reject resistance-SoH confidence without
     * invalidating otherwise coherent current integration. */
    ams_current_window_update(&current_acc,
                              70u,
                              20.0f,
                              20.0f,
                              true,
                              true,
                              43u);
    CHECK(ams_current_window_rotate(&current_acc, 100u, &completed));
    CHECK(fabs(completed.charge_As - 1.0) < 1.0e-9);
    CHECK(fabs(completed.total_charge_As - 1.8) < 1.0e-9);
    CHECK(!completed.calibration_record_confident);
    CHECK(completed.calibration_id == 0u);

    ams_current_window_update(&current_acc,
                              110u,
                              NAN,
                              NAN,
                              false,
                              false,
                              0u);
    CHECK(!ams_current_window_rotate(&current_acc, 120u, &completed));
    CHECK(completed.invalid_sample_count > 0u);
    CHECK(completed.total_invalid_sample_count > 0u);

    /* Tick arithmetic remains valid across the 32-bit RTOS tick wrap. */
    ams_current_window_init(&current_acc, UINT32_MAX - 20u);
    ams_current_window_update(&current_acc,
                              UINT32_MAX - 10u,
                              4.0f,
                              4.0f,
                              true,
                              true,
                              7u);
    ams_current_window_update(&current_acc,
                              5u,
                              4.0f,
                              4.0f,
                              true,
                              true,
                              7u);
    CHECK(ams_current_window_rotate(&current_acc, 20u, &completed));
    CHECK(fabs(completed.charge_As - 0.164) < 1.0e-9);

    /* Current-window sequences use the same nonzero rollover convention as
     * published measurement epochs. */
    ams_current_window_init(&current_acc, 0u);
    current_acc.next_sequence = UINT32_MAX - 1u;
    ams_current_window_update(&current_acc,
                              10u,
                              1.0f,
                              1.0f,
                              true,
                              true,
                              9u);
    CHECK(ams_current_window_rotate(&current_acc, 20u, &completed));
    CHECK(completed.sequence == UINT32_MAX);
    ams_current_window_update(&current_acc,
                              30u,
                              1.0f,
                              1.0f,
                              true,
                              true,
                              9u);
    CHECK(ams_current_window_rotate(&current_acc, 40u, &completed));
    CHECK(completed.sequence == 1u);

    ams_measurement_store_t store;
    ams_measurement_store_init(&store);
    ams_measurement_snapshot_t *write =
        ams_measurement_store_begin_write(&store);
    CHECK(write != NULL);
    ams_measurement_snapshot_prepare(write,
                                     NULL,
                                     &completed,
                                     1u,
                                     2u,
                                     3u,
                                     NULL,
                                     0u,
                                     AMS_MEAS_VALID_CURRENT);
    CHECK(ams_measurement_store_publish(&store, write) == 1u);
    ams_measurement_snapshot_t copied;
    CHECK(ams_measurement_store_copy_latest(&store, &copied));
    CHECK(copied.sequence == 1u);
    CHECK(copied.voltage_complete_tick == 2u);
    CHECK(copied.current.total_invalid_sample_count ==
          completed.total_invalid_sample_count);
    CHECK(store.reader_count[0] == 0u);
    CHECK(store.reader_count[1] == 0u);

    /* A pinned inactive buffer is never overwritten.  The producer drops one
     * publication attempt and can resume once that reader releases it. */
    uint8_t inactive = (uint8_t)(store.published_index ^ 1u);
    store.reader_count[inactive] = 1u;
    CHECK(ams_measurement_store_begin_write(&store) == NULL);
    CHECK(store.publication_drop_count == 1u);
    store.reader_count[inactive] = 0u;
    write = ams_measurement_store_begin_write(&store);
    CHECK(write == &store.buffer[inactive]);
    ams_measurement_snapshot_prepare(write,
                                     NULL,
                                     &completed,
                                     4u,
                                     5u,
                                     6u,
                                     NULL,
                                     0u,
                                     AMS_MEAS_VALID_CURRENT);
    CHECK(ams_measurement_store_publish(&store, write) == 3u);
    CHECK(ams_measurement_store_copy_latest(&store, &copied));
    CHECK(copied.sequence == 3u);
    CHECK(copied.voltage_complete_tick == 5u);

    /* Sequence zero is reserved for "never published". Rollover skips it
     * while preserving a visible forward epoch on the next publication. */
    ams_measurement_store_t wrap_store;
    ams_measurement_store_init(&wrap_store);
    wrap_store.next_sequence = UINT32_MAX - 1u;
    write = ams_measurement_store_begin_write(&wrap_store);
    CHECK(write != NULL);
    ams_measurement_snapshot_prepare(write,
                                     NULL,
                                     &completed,
                                     10u,
                                     11u,
                                     12u,
                                     NULL,
                                     0u,
                                     AMS_MEAS_VALID_CURRENT);
    CHECK(ams_measurement_store_publish(&wrap_store, write) == UINT32_MAX);
    write = ams_measurement_store_begin_write(&wrap_store);
    CHECK(write != NULL);
    ams_measurement_snapshot_prepare(write,
                                     NULL,
                                     &completed,
                                     13u,
                                     14u,
                                     15u,
                                     NULL,
                                     0u,
                                     AMS_MEAS_VALID_CURRENT);
    CHECK(ams_measurement_store_publish(&wrap_store, write) == 1u);

    /* A failed producer can explicitly release its reservation. A mismatched
     * pointer cannot cancel somebody else's in-progress write. */
    write = ams_measurement_store_begin_write(&wrap_store);
    CHECK(write != NULL);
    CHECK(!ams_measurement_store_abort_write(&wrap_store,
                                              &wrap_store.buffer[wrap_store.write_index ^ 1u]));
    CHECK(wrap_store.write_in_progress);
    CHECK(ams_measurement_store_abort_write(&wrap_store, write));
    CHECK(!wrap_store.write_in_progress);
    CHECK(ams_measurement_store_publish(&wrap_store, write) == 0u);
    CHECK(ams_measurement_store_begin_write(&wrap_store) != NULL);

    /* Diagnostic masks belong to the same immutable epoch as measurements. */
    accumulator_t snapshot_acc;
    memset(&snapshot_acc, 0, sizeof(snapshot_acc));
    snapshot_acc.smb.num_ics = 1u;
    snapshot_acc.smb.ics_capacity = NSMBS;
    snapshot_acc.smb.ics = snapshot_acc.smb_ics;
    snapshot_acc.updated_voltage_mask[0] = 0x0015u;
    snapshot_acc.stale_voltage_mask[0] = 0x0020u;
    snapshot_acc.pec_fail_voltage_mask[0] = 0x0040u;
    snapshot_acc.temp_open_mask[0] = 0x000123u;
    snapshot_acc.temp_rate_rise_mask[0] = 0x000456u;
    ams_measurement_snapshot_t mask_snapshot;
    ams_measurement_snapshot_prepare(&mask_snapshot,
                                     &snapshot_acc,
                                     &completed,
                                     20u,
                                     21u,
                                     22u,
                                     NULL,
                                     0u,
                                     0u);
    snapshot_acc.updated_voltage_mask[0] = 0u;
    snapshot_acc.temp_open_mask[0] = 0u;
    CHECK(mask_snapshot.voltage_updated_mask[0] == 0x0015u);
    CHECK(mask_snapshot.voltage_stale_mask[0] == 0x0020u);
    CHECK(mask_snapshot.voltage_pec_fail_mask[0] == 0x0040u);
    CHECK(mask_snapshot.temp_open_mask[0] == 0x000123u);
    CHECK(mask_snapshot.temp_rate_rise_mask[0] == 0x000456u);
}

static void test_estimator_ra8m1_architecture_parity(void){
    /*
     * This guards the intended match to the working RA8M1 physics-only DAEKF:
     * 3-state inner EKF, scalar outer R0 adaptation, adaptive R, feed-forward
     * thermal observer, LUT OCV/R0/C1/tau1, and fixed R2/C2 slow branch.
     */
    CHECK(fabsf(AMS_EKF_INV_C2 - 8.3333333e-5f) < 1.0e-10f);
    CHECK(fabsf(AMS_EKF_INV_TAU2 - 2.08333333e-2f) < 1.0e-8f);
    CHECK(fabsf(AMS_EKF_INV_R2 - 250.0f) < 1.0e-4f);
    CHECK(fabsf(AMS_EKF_INV_CC - 1.81818176e-2f) < 1.0e-8f);
    CHECK(fabsf(AMS_EKF_INV_RCS - 6.66666687e-1f) < 1.0e-7f);
    CHECK(fabsf(AMS_EKF_R0_MIN_OHM - 0.005f) < 1.0e-8f);
    CHECK(fabsf(AMS_EKF_R0_MAX_OHM - 0.040f) < 1.0e-8f);

    ams_ekf_config_t cfg;
    ams_ekf_make_pack_config(&cfg);
    cfg.r0_init_ohm = 0.20f;
    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &cfg);
    CHECK(fabsf(ekf.r0_ohm - 0.040f) < 1.0e-7f);

    ams_estimator_t estimator;
    ams_estimator_init_default(&estimator);
    estimator.cc_step_count = UINT32_MAX;
    CHECK(ams_estimator_cc_apply_charge(&estimator, 1.0));
    CHECK(estimator.cc_step_count == UINT32_MAX);

    cfg.r0_init_ohm = 0.0147f;
    cfg.soc_init = 1.0f;
    ams_ekf_init(&ekf, &cfg);

    float v_nom = (float)cfg.series_group_count *
                  (ams_p42a_ocv_v(ekf.soc, 25.0f) - (ekf.r0_ohm * 0.0f));
    CHECK(ams_ekf_step(&ekf, 0.0f, v_nom, 25.0f, 0.1f));
    CHECK(ekf.valid == 1u);
    CHECK(ekf.soc >= 0.0f && ekf.soc <= 1.0f);
    CHECK(isfinite(ekf.vp1_V));
    CHECK(isfinite(ekf.vp2_V));
    CHECK(isfinite(ekf.r_meas_V2));
    CHECK(isfinite(ekf.t_core_C));
}

static void test_estimator_lut_and_config_matrix(void){
    float prev_ocv = ams_p42a_ocv_v(0.0f, 25.0f);
    CHECK(isfinite(prev_ocv));
    for(int k=1; k<=100; k++){
        float soc = (float)k / 100.0f;
        float ocv = ams_p42a_ocv_v(soc, 25.0f);
        CHECK(isfinite(ocv));
        CHECK(ocv >= prev_ocv - 0.001f);
        prev_ocv = ocv;
    }
    CHECK(ams_p42a_r0_ohm(0.5f, 25.0f) > 0.005f);
    CHECK(ams_p42a_r0_ohm(0.5f, 25.0f) < 0.050f);
    CHECK(ams_p42a_inv_c1(0.5f, 25.0f) > 0.0f);
    CHECK(ams_p42a_neg_inv_tau1(0.5f, 25.0f) < 0.0f);
    CHECK(ams_p42a_inv_r1_from_luts(0.0f, -0.1f) >= 0.0f);

    ams_estimator_t est;
    memset(&est, 0xA5, sizeof(est));
    ams_estimator_init_default(&est);
    CHECK(est.enabled == 1u);
    CHECK(est.instance_count == 1u);
    CHECK(est.inst[0].cfg.first_series_group == 0u);
    CHECK(est.inst[0].cfg.series_group_count == 75u);

    est.fault_flags = AMS_EKF_FAULT_BAD_TEMP;
    CHECK(ams_estimator_configure_segments(&est));
    CHECK(est.fault_flags == AMS_EKF_FAULT_NONE);
    CHECK(est.instance_count == 5u);
    for(uint8_t i=0; i<5u; i++){
        CHECK(est.inst[i].cfg.first_series_group == (uint16_t)(15u * i));
        CHECK(est.inst[i].cfg.series_group_count == 15u);
        CHECK(est.inst[i].fault_flags == AMS_EKF_FAULT_NONE);
    }

    CHECK(ams_estimator_configure_even_split(&est, 10u));
    CHECK(est.instance_count == 10u);
    uint16_t first = 0u;
    for(uint8_t i=0; i<10u; i++){
        uint16_t expected_count = (i < 5u) ? 8u : 7u;
        CHECK(est.inst[i].cfg.first_series_group == first);
        CHECK(est.inst[i].cfg.series_group_count == expected_count);
        first = (uint16_t)(first + expected_count);
    }
    CHECK(first == 75u);
    CHECK(!ams_estimator_configure_even_split(&est, 0u));
    CHECK(!ams_estimator_configure_even_split(&est, 11u));

    ams_ekf_config_t bad;
    ams_ekf_make_segment_config(&bad, 5u);
    ams_ekf_instance_t ekf;
    ams_ekf_init(&ekf, &bad);
    CHECK((ekf.fault_flags & AMS_EKF_FAULT_BAD_CONFIG) != 0u);
}

static void test_estimator_step_faults_and_scalability(void){
    ams_ekf_config_t cfg;
    ams_ekf_instance_t ekf;
    ams_ekf_make_pack_config(&cfg);
    cfg.soc_init = 0.80f;
    ams_ekf_init(&ekf, &cfg);
    float v_nom = estimator_expected_voltage(&cfg, ekf.soc, 0.0f, 25.0f, ekf.r0_ohm);
    CHECK(ams_ekf_step(&ekf, 0.0f, v_nom, 25.0f, 0.1f));
    CHECK(ekf.valid == 1u);
    CHECK(ekf.soc > 0.0f && ekf.soc <= 1.0f);
    CHECK(isfinite(ekf.v_pred_V));

    CHECK(!ams_ekf_step(&ekf, NAN, v_nom, 25.0f, 0.1f));
    CHECK((ekf.fault_flags & AMS_EKF_FAULT_BAD_INPUT) != 0u);
    CHECK(!ams_ekf_step(&ekf, 0.0f, 10.0f, 25.0f, 0.1f));
    CHECK((ekf.fault_flags & AMS_EKF_FAULT_BAD_VOLTAGE) != 0u);
    CHECK(!ams_ekf_step(&ekf, 2000.0f, v_nom, 25.0f, 0.1f));
    CHECK((ekf.fault_flags & AMS_EKF_FAULT_BAD_CURRENT) != 0u);
    CHECK(!ams_ekf_step(&ekf, 0.0f, v_nom, 130.0f, 0.1f));
    CHECK((ekf.fault_flags & AMS_EKF_FAULT_BAD_TEMP) != 0u);

    ams_ekf_init(&ekf, &cfg);
    for(int k=0; k<200; k++){
        float v = estimator_expected_voltage(&cfg, ekf.soc, 25.0f, 25.0f, ekf.r0_ohm);
        CHECK(ams_ekf_step(&ekf, 25.0f, v, 25.0f, (k % 3 == 0) ? NAN : 0.1f));
        CHECK(isfinite(ekf.soc));
        CHECK(isfinite(ekf.r0_ohm));
        CHECK(isfinite(ekf.t_core_C));
        CHECK(ekf.r0_ohm >= 0.005f && ekf.r0_ohm <= 0.040f);
    }
    CHECK(ekf.step_count == 200u);
    CHECK(ekf.soc >= 0.0f && ekf.soc <= 1.0f);

    ams_estimator_t est;
    ams_estimator_init_default(&est);
    CHECK(ams_estimator_configure_even_split(&est, 10u));
    for(uint8_t i=0; i<est.instance_count; i++){
        ams_ekf_instance_t *inst = &est.inst[i];
        float vv = estimator_expected_voltage(&inst->cfg, inst->soc, 10.0f, 25.0f, inst->r0_ohm);
        CHECK(ams_ekf_step(inst, 10.0f, vv, 25.0f, 0.1f));
        CHECK(inst->valid == 1u);
    }
    for(uint8_t i=0; i<est.instance_count; i++){
        est.inst[i].soc = 0.50f + 0.01f * (float)i;
        est.inst[i].r0_ohm = 0.010f + 0.001f * (float)i;
        est.inst[i].t_core_C = 20.0f + (float)i;
        est.inst[i].v_pred_V = (float)est.inst[i].cfg.series_group_count * 3.7f;
        est.inst[i].innovation_V = 0.1f * (float)i;
        est.inst[i].valid = 1u;
    }
    est.inst[6].fault_flags = AMS_EKF_FAULT_BAD_TEMP;
    ams_estimator_refresh_summary(&est, AMS_ESTIMATOR_INPUT_HARDWARE, 999u);
    CHECK((est.fault_flags & AMS_EKF_FAULT_BAD_TEMP) != 0u);
    CHECK(est.pack_soc > 0.50f && est.pack_soc < 0.60f);
    CHECK(est.representative_cell_r0_ohm > 0.010f &&
          est.representative_cell_r0_ohm < 0.020f);
    CHECK(fabsf(est.pack_r0_ohm - est.representative_cell_r0_ohm) < 1.0e-8f);
    CHECK(fabsf(est.estimated_pack_r0_ohm -
                (est.representative_cell_r0_ohm * 75.0f / 6.0f)) < 1.0e-6f);
    CHECK(fabsf(est.pack_v_pred_V - 277.5f) < 0.5f);
    CHECK(est.pack_innovation_V > 4.0f && est.pack_innovation_V < 5.0f);
}

static void test_estimator_epoch_sequence_and_timing(void)
{
    init_fake_app();
    fake_tick = 100u;
    fill_nominal_pack(&app, 3.90f);
    ams_estimator_init_default(&app.estimator);

    const uint32_t good_flags =
        AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
        AMS_MEAS_VALID_CURRENT | AMS_MEAS_BALANCE_RECOVERED;
    (void)host_publish_measurement_snapshot(&app, 100u, 10.0f, good_flags);
    CHECK(estimator_task_update(&app, 100u, 0.1f));
    CHECK(app.estimator.inst[0].step_count == 1u);
    CHECK(app.estimator.last_consumed_measurement_sequence == 1u);
    float cc_after_first = app.estimator.cc_soc;

    CHECK(estimator_task_update(&app, 150u, 0.05f));
    CHECK(app.estimator.inst[0].step_count == 1u);
    CHECK(app.estimator.cc_soc == cc_after_first);
    CHECK(app.estimator.repeated_measurement_count == 1u);

    uint32_t steps_before_stale = app.estimator.inst[0].step_count;
    CHECK(!estimator_task_update(&app, 601u, 0.1f));
    CHECK(app.estimator.inst[0].step_count == steps_before_stale);
    CHECK((app.estimator.fault_flags & AMS_EKF_FAULT_STALE_INPUT) != 0u);

    /* Reinitialize after the deliberate stale-time jump so the remainder of
     * this deterministic sequence keeps monotonic test time. */
    init_fake_app();
    fake_tick = 100u;
    fill_nominal_pack(&app, 3.90f);
    ams_estimator_init_default(&app.estimator);
    (void)host_publish_measurement_snapshot(&app, 100u, 10.0f, good_flags);
    CHECK(estimator_task_update(&app, 100u, 0.1f));

    (void)host_publish_measurement_snapshot(&app, 200u, 20.0f, good_flags);
    (void)host_publish_measurement_snapshot(&app, 300u, 30.0f, good_flags);
    CHECK(estimator_task_update(&app, 300u, 0.1f));
    CHECK(app.estimator.missed_measurement_count == 1u);
    CHECK(app.estimator.inst[0].step_count == 2u);
    CHECK(fabsf(app.estimator.inst[0].last_i_pack_A - 25.0f) < 1.0e-4f);

    uint32_t steps_before_bad_time = app.estimator.inst[0].step_count;
    (void)host_publish_measurement_snapshot(&app, 300u, 30.0f, good_flags);
    CHECK(!estimator_task_update(&app, 301u, 0.1f));
    CHECK(app.estimator.inst[0].step_count == steps_before_bad_time);
    CHECK(app.estimator.epoch_timing_fault_count == 1u);
    CHECK((app.estimator.fault_flags & AMS_EKF_FAULT_EPOCH_TIMING) != 0u);

    /* An invalid skipped current interval cannot be hidden by a later valid
     * window and then retroactively integrated into coulomb count. */
    init_fake_app();
    fake_tick = 100u;
    fill_nominal_pack(&app, 3.90f);
    ams_estimator_init_default(&app.estimator);
    (void)host_publish_measurement_snapshot(&app, 100u, 10.0f, good_flags);
    CHECK(estimator_task_update(&app, 100u, 0.1f));
    uint32_t steps_before_gap = app.estimator.inst[0].step_count;
    float cc_before_gap = app.estimator.cc_soc;
    (void)host_publish_measurement_snapshot(
        &app,
        200u,
        0.0f,
        AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
        AMS_MEAS_BALANCE_RECOVERED);
    (void)host_publish_measurement_snapshot(&app, 300u, 10.0f, good_flags);
    CHECK(!estimator_task_update(&app, 300u, 0.1f));
    CHECK(app.estimator.inst[0].step_count == steps_before_gap);
    CHECK(app.estimator.cc_soc == cc_before_gap);

    /* Voltage-epoch timing uses unsigned wrap-safe subtraction. */
    init_fake_app();
    fake_tick = UINT32_MAX - 49u;
    fill_nominal_pack(&app, 3.90f);
    ams_estimator_init_default(&app.estimator);
    (void)host_publish_measurement_snapshot(&app,
                                            UINT32_MAX - 49u,
                                            0.0f,
                                            good_flags);
    CHECK(estimator_task_update(&app, UINT32_MAX - 49u, 0.1f));
    (void)host_publish_measurement_snapshot(&app, 50u, 0.0f, good_flags);
    CHECK(estimator_task_update(&app, 50u, 0.1f));
    CHECK(app.estimator.epoch_timing_fault_count == 0u);
}

static void test_estimator_model_domain_flags(void)
{
    ams_estimator_t est;
    ams_estimator_init_default(&est);
    ams_ekf_instance_t *inst = &est.inst[0];
    inst->t_core_C = 0.0f;
    float v_low = estimator_expected_voltage(&inst->cfg,
                                             inst->soc,
                                             0.0f,
                                             5.0f,
                                             inst->r0_ohm);
    CHECK(ams_ekf_step(inst, 0.0f, v_low, 0.0f, 0.1f));
    ams_estimator_refresh_summary(&est, AMS_ESTIMATOR_INPUT_HARDWARE, 1u);
    CHECK((est.model_domain_flags & AMS_EKF_MODEL_DOMAIN_TEMP_LOW) != 0u);
    CHECK((ams_estimator_status_flags(&est) & AMS_EKF_FLAG_MODEL_CLAMPED) != 0u);

    inst->t_core_C = 45.0f;
    float v_high = estimator_expected_voltage(&inst->cfg,
                                              inst->soc,
                                              0.0f,
                                              40.0f,
                                              inst->r0_ohm);
    CHECK(ams_ekf_step(inst, 0.0f, v_high, 45.0f, 0.1f));
    ams_estimator_refresh_summary(&est, AMS_ESTIMATOR_INPUT_HARDWARE, 2u);
    CHECK((est.model_domain_flags & AMS_EKF_MODEL_DOMAIN_TEMP_HIGH) != 0u);
}

static void test_hil_parser_edge_cases(void){
    static CAN_HandleTypeDef hcan;
    init_fake_app(); app.board.canbus.hcan = &hcan;
    memset(&fake_rx_hdr, 0, sizeof(fake_rx_hdr));
    memset(fake_rx_data, 0xFF, sizeof(fake_rx_data));

    fake_rx_hdr.IDE = CAN_ID_EXT; fake_rx_hdr.ExtId = AMS_HIL_CAN_ID_MEAS; fake_rx_hdr.DLC = 8;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.hil.meas.fresh == 0u);

    fake_rx_hdr.IDE = CAN_ID_STD; fake_rx_hdr.StdId = AMS_HIL_CAN_ID_MEAS; fake_rx_hdr.DLC = 6;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.hil.meas.fresh == 0u);

    fake_rx_hdr.DLC = 8;
    fake_rx_data[0]=0x7B; fake_rx_data[1]=0x00; /* 314.88 V at 10 mV/count */
    fake_rx_data[2]=0xFF; fake_rx_data[3]=0x38; /* -2.00 A at 10 mA/count */
    fake_rx_data[4]=0x09; fake_rx_data[5]=0xC4; /* 25.00 C */
    fake_rx_data[6]=3u; fake_tick=44u;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.hil.meas.fresh == 1u);
    CHECK(app.hil.meas.last_rx_tick == 44u);
    CHECK(fabsf(app.hil.meas.v_pack_V - 314.88f) < 0.02f);
    CHECK(fabsf(app.hil.meas.i_pack_A + 2.00f) < 0.02f);

    fake_rx_hdr.StdId = AMS_HIL_CAN_ID_SUMMARY; fake_rx_hdr.DLC = 8;
    fake_rx_data[0]=0x0E; fake_rx_data[1]=0x10; /* 3.600 V */
    fake_rx_data[2]=0x10; fake_rx_data[3]=0x04; /* 4.100 V */
    fake_rx_data[4]=0x0A; fake_rx_data[5]=0x28; /* 26.00 C */
    fake_rx_data[6]=0x09; fake_rx_data[7]=0xC4; /* 25.00 C */
    fake_tick=55u;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.hil.summary.fresh == 1u);
    CHECK(app.hil.summary.last_rx_tick == 55u);
    CHECK(fabsf(app.hil.summary.v_min_V - 3.600f) < 0.002f);
    CHECK(fabsf(app.hil.summary.v_max_V - 4.100f) < 0.002f);
    CHECK(fabsf(app.hil.summary.t_max_C - 26.0f) < 0.02f);
}

static void test_estimator_task_hil_and_hardware_paths(void){
    static CAN_HandleTypeDef hcan;

    init_fake_app(); fake_tick = 100u; fill_nominal_pack(&app, 3.90f); app.board.canbus.hcan = &hcan;
    sil_make_measurement_gates_ready(&app);
    app.current = 0.0f; app.avg_temp = 25.0f;
    (void)host_publish_measurement_snapshot(
        &app,
        fake_tick,
        0.0f,
        AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
        AMS_MEAS_VALID_CURRENT | AMS_MEAS_BALANCE_RECOVERED);
    run_one_estimator_task_iteration(&app);
    CHECK(app.estimator.input_source == AMS_ESTIMATOR_INPUT_HARDWARE);
    CHECK(app.estimator.inst[0].valid == 1u);
    CHECK(app.estimator_fault == false);
    CHECK(app.estimator.pack_soc >= 0.0f && app.estimator.pack_soc <= 1.0f);
    CHECK(app.estimator.resistance_soh[0].accepted_count == 0u);
    CHECK(app.estimator.resistance_soh[0].rejected_count == 1u);
    CHECK(app.estimator.resistance_soh[0].reject_current_calibration_count == 1u);
    CHECK((app.estimator.resistance_soh[0].status_flags &
           AMS_SOH_STATUS_ADVISORY_VALID) == 0u);

    init_fake_app(); app.board.canbus.hcan = &hcan;
    app.hil.meas.fresh = 1u;
    app.hil.meas.last_rx_tick = 1000u;
    app.hil.meas.v_pack_V = 310.0f;
    app.hil.meas.i_pack_A = -5.0f;
    app.hil.meas.t_surf_C = 24.0f;
    fake_tick = 1100u;
    run_one_estimator_task_iteration(&app);
    CHECK(app.estimator.input_source == AMS_ESTIMATOR_INPUT_HIL_CAN);
    CHECK(app.estimator.inst[0].valid == 1u);
    CHECK((ams_estimator_status_flags(&app.estimator) & AMS_EKF_FLAG_HIL_SOURCE) != 0u);
    CHECK(app.estimator.resistance_soh[0].accepted_count == 0u);
    CHECK(app.estimator.resistance_soh[0].reject_low_current_count == 1u);

    /* A second coherent HIL epoch with known synthetic current calibration,
     * sufficient current, and a real step may update advisory R0. */
    app.hil.meas.counter = 1u;
    app.hil.meas.last_rx_tick = 1200u;
    app.hil.meas.i_pack_A = -30.0f;
    app.estimator.inst[0].soc = 0.50f;
    app.estimator.cc_soc = 0.50f;
    app.hil.meas.v_pack_V = estimator_expected_voltage(
        &app.estimator.inst[0].cfg,
        app.estimator.inst[0].soc,
        app.hil.meas.i_pack_A,
        25.0f,
        app.estimator.inst[0].r0_ohm);
    app.hil.meas.t_surf_C = 25.0f;
    CHECK(estimator_task_update(&app, 1200u, 0.1f));
    CHECK(app.estimator.resistance_soh[0].accepted_count == 1u);
    CHECK(app.estimator.resistance_soh[0].last_reject_flags == AMS_SOH_REJECT_NONE);

    sil_prepare_cli_capture();
    CHECK(get_estimator_diag(0, NULL) == 0);
    CHECK(strstr(cli_capture, "R0-SoH ADVISORY") != NULL);
    CHECK(strstr(cli_capture, "accepted:1") != NULL);
    CHECK(strstr(cli_capture, "non-authoritative") != NULL);

    init_fake_app(); app.board.canbus.hcan = &hcan;
    app.hil.meas.fresh = 1u;
    app.hil.meas.last_rx_tick = 1u;
    app.hil.meas.v_pack_V = 310.0f;
    app.hil.meas.i_pack_A = -5.0f;
    app.hil.meas.t_surf_C = 24.0f;
    fake_tick = 10000u;
    run_one_estimator_task_iteration(&app);
    CHECK(app.estimator.input_source == AMS_ESTIMATOR_INPUT_HARDWARE);
    CHECK(app.estimator.inst[0].valid == 0u);
    CHECK(app.estimator_fault == true);
}

static void test_estimator_rejects_invalid_hardware_inputs(void)
{
    float cc_before;
    uint32_t steps_before;

    init_fake_app();
    fake_tick = 100u;
    fill_nominal_pack(&app, 3.90f);
    sil_make_measurement_gates_ready(&app);
    app.current = 20.0f;
    app.avg_temp = 25.0f;
    ams_estimator_init_default(&app.estimator);
    (void)host_publish_measurement_snapshot(
        &app, 100u, 20.0f,
        AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
        AMS_MEAS_VALID_CURRENT | AMS_MEAS_BALANCE_RECOVERED);
    CHECK(estimator_task_update(&app, 100u, 0.1f));
    CHECK(app.estimator.inst[0].valid == 1u);
    cc_before = app.estimator.cc_soc;
    steps_before = app.estimator.inst[0].step_count;

    app.current_valid = false;
    app.current = 500.0f;
    (void)host_publish_measurement_snapshot(
        &app, 200u, 500.0f,
        AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
        AMS_MEAS_BALANCE_RECOVERED);
    CHECK(!estimator_task_update(&app, 200u, 0.1f));
    CHECK(app.estimator.cc_soc == cc_before);
    CHECK(app.estimator.inst[0].step_count == steps_before);
    CHECK(app.estimator.inst[0].valid == 0u);
    CHECK((app.estimator.fault_flags & AMS_EKF_FAULT_BAD_INPUT) != 0u);

    /* A valid current may advance coulomb count without an EKF correction,
     * but invalid voltage or temperature must keep the EKF result invalid. */
    app.current_valid = true;
    app.current = 20.0f;
    app.voltage_valid = false;
    (void)host_publish_measurement_snapshot(
        &app, 300u, 20.0f,
        AMS_MEAS_VALID_TEMPERATURE | AMS_MEAS_VALID_CURRENT |
        AMS_MEAS_BALANCE_RECOVERED);
    CHECK(!estimator_task_update(&app, 300u, 0.1f));
    CHECK(app.estimator.cc_soc < cc_before);
    CHECK(app.estimator.inst[0].step_count == steps_before);

    app.voltage_valid = true;
    app.temp_valid = false;
    (void)host_publish_measurement_snapshot(
        &app, 400u, 20.0f,
        AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_CURRENT |
        AMS_MEAS_BALANCE_RECOVERED);
    CHECK(!estimator_task_update(&app, 400u, 0.1f));
    CHECK(app.estimator.inst[0].step_count == steps_before);

    /* Even a globally marked-valid temperature snapshot is unusable when no
     * actual thermistor or validated aggregate temperature can be collected. */
    app.temp_valid = true;
    app.avg_temp = NAN;
    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        for(uint8_t sensor = 0u; sensor < NTEMPS; sensor++)
        {
            app.acc.smb_ics[seg].temp.raw[sensor] = -1;
        }
        app.acc.usable_temp_mask[seg] = 0u;
    }
    (void)host_publish_measurement_snapshot(
        &app, 500u, 20.0f,
        AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
        AMS_MEAS_VALID_CURRENT | AMS_MEAS_BALANCE_RECOVERED);
    CHECK(!estimator_task_update(&app, 500u, 0.1f));
    CHECK(app.estimator.inst[0].step_count == steps_before);

    app.avg_temp = 25.0f;
    fill_nominal_pack(&app, 3.90f);
    (void)host_publish_measurement_snapshot(
        &app, 600u, 20.0f,
        AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
        AMS_MEAS_VALID_CURRENT | AMS_MEAS_BALANCE_RECOVERED);
    CHECK(estimator_task_update(&app, 600u, 0.1f));
    CHECK(app.estimator.inst[0].valid == 1u);
    CHECK(app.estimator.inst[0].step_count == (steps_before + 1u));
    CHECK(app.estimator_fault == false);
}

static void test_estimator_status_packet_edges(void){
    static CAN_HandleTypeDef hcan;
    init_fake_app(); app.board.canbus.hcan = &hcan;
    ams_estimator_init_default(&app.estimator);
    tx_count = 0; tx_free_level = 3;
    CHECK(send_estimator_status(&app.board.canbus, &app) == HAL_OK);
    CHECK(tx_count == 1u);
    CHECK(tx_log[0].stdid == AMS_ESTIMATOR_STATUS_CAN_ID);
    CHECK((tx_log[0].data[1] & AMS_EKF_FLAG_VALID) == 0u);
    CHECK((tx_log[0].data[1] & AMS_EKF_FLAG_CC_FALLBACK) != 0u);
    CHECK(word_at(0,1) == 10000u);

    app.estimator.inst[0].valid = 1u;
    app.estimator.inst[0].soc = 0.4567f;
    app.estimator.inst[0].innovation_V = -1.234f;
    app.estimator.inst[0].r0_ohm = 0.01234f;
    app.estimator.input_source = AMS_ESTIMATOR_INPUT_HARDWARE;
    tx_count = 0;
    CHECK(send_estimator_status(&app.board.canbus, &app) == HAL_OK);
    CHECK(tx_count == 1u);
    CHECK(tx_log[0].stdid == AMS_ESTIMATOR_STATUS_CAN_ID);
    CHECK(tx_log[0].data[0] == 0u);
    CHECK((tx_log[0].data[1] & AMS_EKF_FLAG_VALID) != 0u);
    CHECK(word_at(0,1) == 4567u);
    CHECK((int16_t)word_at(0,2) == -1234);
    CHECK(word_at(0,3) == 1234u);

    app.estimator.inst[0].soc = 10.0f;
    app.estimator.inst[0].innovation_V = 1000.0f;
    app.estimator.inst[0].r0_ohm = 10.0f;
    tx_count = 0;
    CHECK(send_estimator_status(&app.board.canbus, &app) == HAL_OK);
    CHECK(word_at(0,1) == 65535u);
    CHECK((int16_t)word_at(0,2) == INT16_MAX);
    CHECK(word_at(0,3) == 65535u);
}

static void test_hil_parser_and_estimator_core(void){
    static CAN_HandleTypeDef hcan;
    init_fake_app(); app.board.canbus.hcan = &hcan;

    memset(&fake_rx_hdr, 0, sizeof(fake_rx_hdr));
    memset(fake_rx_data, 0, sizeof(fake_rx_data));
    fake_rx_hdr.IDE = CAN_ID_STD; fake_rx_hdr.StdId = AMS_HIL_CAN_ID_MEAS; fake_rx_hdr.DLC = 8;
    uint16_t v_10mV = 31500u; /* 315.00 V */
    int16_t i_10mA = -1234;   /* -12.34 A */
    int16_t t_cC = 2534;      /* 25.34 C */
    fake_rx_data[0]=(uint8_t)(v_10mV>>8); fake_rx_data[1]=(uint8_t)v_10mV;
    fake_rx_data[2]=(uint8_t)((uint16_t)i_10mA>>8); fake_rx_data[3]=(uint8_t)((uint16_t)i_10mA);
    fake_rx_data[4]=(uint8_t)((uint16_t)t_cC>>8); fake_rx_data[5]=(uint8_t)((uint16_t)t_cC);
    fake_rx_data[6]=77u; fake_tick=1234u;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.hil.meas.fresh == 1u);
    CHECK(app.hil.meas.counter == 77u);
    CHECK(fabsf(app.hil.meas.v_pack_V - 315.0f) < 0.02f);
    CHECK(fabsf(app.hil.meas.i_pack_A + 12.34f) < 0.02f);
    CHECK(fabsf(app.hil.meas.t_surf_C - 25.34f) < 0.02f);

    fake_rx_hdr.StdId = AMS_HIL_CAN_ID_TRUTH; fake_rx_hdr.DLC = 8;
    uint16_t soc_d2 = 9876u; int16_t tc_cC = 2601; uint32_t step = 0x00012345u;
    fake_rx_data[0]=(uint8_t)(soc_d2>>8); fake_rx_data[1]=(uint8_t)soc_d2;
    fake_rx_data[2]=(uint8_t)((uint16_t)tc_cC>>8); fake_rx_data[3]=(uint8_t)((uint16_t)tc_cC);
    fake_rx_data[4]=78u; fake_rx_data[5]=(uint8_t)(step>>16); fake_rx_data[6]=(uint8_t)(step>>8); fake_rx_data[7]=(uint8_t)step;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.hil.truth.fresh == 1u);
    CHECK(app.hil.truth.plant_step == step);
    CHECK(fabsf(app.hil.truth.soc_true - 0.9876f) < 0.0002f);
    CHECK(fabsf(app.hil.truth.t_core_C - 26.01f) < 0.02f);

    ams_estimator_init_default(&app.estimator);
    CHECK(app.estimator.instance_count == 1u);
    CHECK(ams_ekf_step(&app.estimator.inst[0], app.hil.meas.i_pack_A, app.hil.meas.v_pack_V, app.hil.meas.t_surf_C, 0.1f));
    ams_estimator_refresh_summary(&app.estimator, AMS_ESTIMATOR_INPUT_HIL_CAN, fake_tick);
    CHECK(app.estimator.inst[0].valid == 1u);
    CHECK(app.estimator.pack_soc >= 0.0f && app.estimator.pack_soc <= 1.0f);
    CHECK(app.estimator.pack_r0_ohm >= 0.005f && app.estimator.pack_r0_ohm <= 0.040f);

    tx_count = 0; tx_free_level = 3;
    CHECK(send_estimator_status(&app.board.canbus, &app) == HAL_OK);
    CHECK(tx_count == 1u);
    CHECK(tx_log[0].ide == CAN_ID_STD);
    CHECK(tx_log[0].stdid == AMS_ESTIMATOR_STATUS_CAN_ID);
    CHECK((tx_log[0].data[1] & AMS_EKF_FLAG_VALID) != 0u);
    CHECK((tx_log[0].data[1] & AMS_EKF_FLAG_HIL_SOURCE) != 0u);

    fake_rx_hdr.StdId = AMS_HIL_CAN_ID_MEAS; fake_rx_hdr.DLC = 8;
    v_10mV = 45000u; /* 450.00 V: proves 75s HIL uses 10 mV/count, not 1 mV/count. */
    i_10mA = 12345;  /* +123.45 A, positive discharge convention. */
    t_cC = -123;     /* -1.23 C signed temp handling. */
    fake_rx_data[0]=(uint8_t)(v_10mV>>8); fake_rx_data[1]=(uint8_t)v_10mV;
    fake_rx_data[2]=(uint8_t)((uint16_t)i_10mA>>8); fake_rx_data[3]=(uint8_t)((uint16_t)i_10mA);
    fake_rx_data[4]=(uint8_t)((uint16_t)t_cC>>8); fake_rx_data[5]=(uint8_t)((uint16_t)t_cC);
    fake_rx_data[6]=79u; fake_rx_data[7]=0u; fake_tick=1300u;
    (void)host_receive_can_frame(&hcan);
    CHECK(fabsf(app.hil.meas.v_pack_V - 450.0f) < 0.02f);
    CHECK(fabsf(app.hil.meas.i_pack_A - 123.45f) < 0.02f);
    CHECK(fabsf(app.hil.meas.t_surf_C + 1.23f) < 0.02f);
}

static void test_hil_adbms_image_replaces_raw_reads(void)
{
    init_fake_app();
    fake_tick = 1000u;

    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        for(uint8_t first = 0u; first < NCELLS; first = (uint8_t)(first + 3u))
        {
            uint16_t mv[3] = {3700u, 3701u, 3702u};
            for(uint8_t n = 0u; n < 3u; n++)
            {
                uint8_t cell = (uint8_t)(first + n);
                if(cell < NCELLS)
                {
                    mv[n] = (uint16_t)(3600u + ((uint16_t)seg * 15u) + cell);
                }
            }
            if((seg == 3u) && (first == 12u))
            {
                mv[2] = 4123u;
            }
            if((seg == 1u) && (first == 0u))
            {
                mv[0] = 3201u;
            }
            host_send_hil_cell_triplet(seg, first, mv[0], mv[1], mv[2]);
        }

        for(uint8_t first = 0u; first < NTEMPS; first = (uint8_t)(first + 3u))
        {
            int16_t t[3] = {250, 251, 252};
            for(uint8_t n = 0u; n < 3u; n++)
            {
                uint8_t sensor = (uint8_t)(first + n);
                if(sensor < NTEMPS)
                {
                    t[n] = (int16_t)(240 + (int16_t)seg + (int16_t)sensor);
                }
            }
            if((seg == 4u) && (first == 21u))
            {
                t[2] = 615;
            }
            if((seg == 0u) && (first == 0u))
            {
                t[0] = -55;
            }
            host_send_hil_temp_triplet(seg, first, t[0], t[1], t[2]);
        }
    }

    accumulator_hil_refresh_update_masks(&app.acc, fake_tick, AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS);
    accumulator_update_voltage_stats_at(&app.acc, fake_tick);
    accumulator_update_temp_stats_at(&app.acc, fake_tick);

    CHECK(app.acc.voltage_full_updated);
    CHECK(app.acc.voltage_full_usable);
    CHECK(app.acc.valid_voltage_count == (uint16_t)(NSMBS * NCELLS));
    CHECK(app.acc.min_voltage_mv == 3201u);
    CHECK(app.acc.max_voltage_mv == 4123u);
    CHECK(accumulator_cell_voltage_mv(&app.acc, 3u, 14u) == 4123u);

    CHECK(app.acc.temp_full_updated);
    CHECK(app.acc.temp_full_usable);
    CHECK(app.acc.valid_temp_count == (uint16_t)(NSMBS * NTEMPS));
    CHECK(app.acc.min_temp_deci_c >= -56 && app.acc.min_temp_deci_c <= -54);
    CHECK(app.acc.max_temp_deci_c >= 614 && app.acc.max_temp_deci_c <= 616);

    fake_tick += (AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS + 1u);
    accumulator_hil_refresh_update_masks(&app.acc, fake_tick, AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS);
    accumulator_update_voltage_stats_at(&app.acc, fake_tick);
    accumulator_update_temp_stats_at(&app.acc, fake_tick);

    CHECK(app.acc.updated_voltage_count == 0u);
    CHECK(app.acc.updated_temp_count == 0u);
    CHECK(app.acc.voltage_full_updated == false);
    CHECK(app.acc.temp_full_updated == false);
    CHECK(app.acc.valid_voltage_count == (uint16_t)(NSMBS * NCELLS));
    CHECK(app.acc.valid_temp_count == (uint16_t)(NSMBS * NTEMPS));

    fake_tick += ACCUMULATOR_TEMP_STALE_TIMEOUT_MS;
    accumulator_hil_refresh_update_masks(&app.acc, fake_tick, AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS);
    accumulator_update_voltage_stats_at(&app.acc, fake_tick);
    accumulator_update_temp_stats_at(&app.acc, fake_tick);

    CHECK(app.acc.valid_voltage_count == 0u);
    CHECK(app.acc.valid_temp_count == 0u);

#if AMS_HIL_REPLACE_ADBMS
    init_fake_app();
    fake_tick = 2000u;
    fake_adbms_lock_depth = 0u;
    fake_adbms_lock_max_depth = 0u;
    bms_pin_state = GPIO_PIN_RESET;
    app.state = STATE_DISCARGE;
    app.current_valid = true;
    app.current_fault = false;
    app.task_heartbeat_fault = false;

    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        for(uint8_t first = 0u; first < NCELLS; first = (uint8_t)(first + 3u))
        {
            host_send_hil_cell_triplet(seg, first, 3700u, 3700u, 3700u);
        }
        for(uint8_t first = 0u; first < NTEMPS; first = (uint8_t)(first + 3u))
        {
            host_send_hil_temp_triplet(seg, first, 250, 250, 250);
        }
    }

    CHECK(fake_adbms_lock_depth == 0u);
    CHECK(fake_adbms_lock_max_depth >= 1u);

    run_one_adbms_task_iteration(&app);

    CHECK(fake_adbms_lock_depth == 0u);

    CHECK(app.voltage_valid == true);
    CHECK(app.voltage_fault == false);
    CHECK(app.temp_valid == true);
    CHECK(app.temp_fault == false);
    CHECK(app.adbms_diag_fault == false);
    CHECK(app.acc.valid_voltage_count == (uint16_t)(NSMBS * NCELLS));
    CHECK(app.acc.valid_temp_count == (uint16_t)(NSMBS * NTEMPS));

    app.can_busoff_fault = true;
    app.can_recover_pending = true;
    app.adbms_balance_write_fault = true;
    CHECK(adbms_record_balance_write_result(&app, 0));
    CHECK(app.adbms_balance_write_fault == false);
    CHECK(app.adbms_diag_fault == true);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_diag_fault == true);
    CHECK(app.adbms_last_diag_status == HAL_ERROR);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
#endif
}

static void test_accumulator_tick_wrap_freshness(void)
{
    const uint32_t before_wrap = UINT32_MAX - 100u;
    const uint32_t after_wrap = 50u;
    const uint16_t cell_mv[3] = {3700u, 3701u, 3702u};
    const int16_t temp_deci_c[3] = {250, 251, 252};

    init_fake_app();
    CHECK(accumulator_hil_ingest_cell_triplet(&app.acc,
                                               0u,
                                               0u,
                                               cell_mv,
                                               before_wrap) == 0);
    CHECK(accumulator_hil_ingest_temp_triplet(&app.acc,
                                               0u,
                                               0u,
                                               temp_deci_c,
                                               before_wrap) == 0);

    accumulator_hil_refresh_update_masks(&app.acc,
                                          after_wrap,
                                          AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS);
    CHECK((app.acc.smb.last_cell_updated_mask[0] & 0x0007u) == 0x0007u);
    CHECK((app.acc.smb.last_temp_updated_mask[0] & 0x00000007u) == 0x00000007u);

    accumulator_hil_refresh_update_masks(
        &app.acc,
        (uint32_t)(before_wrap + AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS + 1u),
        AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS);
    CHECK((app.acc.smb.last_cell_updated_mask[0] & 0x0007u) == 0u);
    CHECK((app.acc.smb.last_temp_updated_mask[0] & 0x00000007u) == 0u);

    /* Exercise the independent cell/temperature stale timers across the same
     * wrap, not just the HIL image-age gate. */
    init_fake_app();
    app.acc.smb_ics[0].cell.c_codes[0] = code_for_volts(3.700f);
    app.acc.smb.last_cell_updated_mask[0] = 0x0001u;
    accumulator_update_voltage_stats_at(&app.acc, before_wrap);
    CHECK(app.acc.cell_voltage_valid[0][0] == true);
    app.acc.smb.last_cell_updated_mask[0] = 0u;
    accumulator_update_voltage_stats_at(&app.acc, after_wrap);
    CHECK(app.acc.cell_voltage_valid[0][0] == true);
    accumulator_update_voltage_stats_at(
        &app.acc,
        (uint32_t)(before_wrap + ACCUMULATOR_CELL_STALE_TIMEOUT_MS + 1u));
    CHECK(app.acc.cell_voltage_valid[0][0] == true);
    CHECK(accumulator_cell_voltage_usable(&app.acc, 0u, 0u) == false);
    CHECK((app.acc.stale_voltage_mask[0] & 0x0001u) != 0u);

    app.acc.smb_ics[0].temp.raw[0] = raw_for_temp_c(25.0f);
    app.acc.smb.last_temp_updated_mask[0] = 0x00000001u;
    accumulator_update_temp_stats_at(&app.acc, before_wrap);
    CHECK(app.acc.temp_sensor_valid[0][0] == true);
    app.acc.smb.last_temp_updated_mask[0] = 0u;
    accumulator_update_temp_stats_at(&app.acc, after_wrap);
    CHECK(app.acc.temp_sensor_valid[0][0] == true);
    accumulator_update_temp_stats_at(
        &app.acc,
        (uint32_t)(before_wrap + ACCUMULATOR_TEMP_STALE_TIMEOUT_MS + 1u));
    CHECK(app.acc.temp_sensor_valid[0][0] == true);
    CHECK(accumulator_temp_sensor_usable(&app.acc, 0u, 0u) == false);
    CHECK((app.acc.stale_temp_mask[0] & 0x00000001u) != 0u);
}

static void test_charge_state_disable_matrix(void){
    static CAN_HandleTypeDef hcan;
    struct Case { bool hard_fault, voltage_fault, temp_fault, temp_charge_stop, bms_state, hw, ot, input, sense, timeout; uint8_t disable; } cases[] = {
        {0,0,0,0,1,0,0,0,0,0,0},
        {1,0,0,0,1,0,0,0,0,0,1},
        {0,1,0,0,1,0,0,0,0,0,1},
        {0,0,1,0,1,0,0,0,0,0,1},
        {0,0,0,1,1,0,0,0,0,0,1},
        {0,0,0,0,0,0,0,0,0,0,1},
        {0,0,0,0,1,1,0,0,0,0,1},
        {0,0,0,0,1,0,1,0,0,0,1},
        {0,0,0,0,1,0,0,1,0,0,1},
        {0,0,0,0,1,0,0,0,1,0,1},
        {0,0,0,0,1,0,0,0,0,1,1},
    };
    for(size_t i=0; i<sizeof(cases)/sizeof(cases[0]); i++){
        init_fake_app(); fill_nominal_pack(&app, 3.700f); charger_init(&app.board.charger, &app.board.canbus);
        app.board.canbus.hcan = &hcan; app.state = STATE_CHARGE; app.current_valid = true;
        app.hard_fault = cases[i].hard_fault; app.voltage_fault = cases[i].voltage_fault; app.temp_fault = cases[i].temp_fault; app.temp_charge_stop = cases[i].temp_charge_stop; app.bms_state = cases[i].bms_state;
        app.board.charger.hardware_fail = cases[i].hw; app.board.charger.overtemp_fail = cases[i].ot; app.board.charger.input_volt_fail = cases[i].input; app.board.charger.voltage_sense_fail = cases[i].sense;
        fake_tick = cases[i].timeout ? 6001u : 1000u;
        app.board.charger.last_rx_tick = 1000u;
        tx_count = 0; tx_free_level = 3; bms_pin_state = app.bms_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
        run_one_canbus_task_iteration(&app);
        CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT);
        CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].ide == CAN_ID_EXT && tx_log[HOST_CHARGER_FRAME_INDEX].extid == CCS_CANBUS_ID);
        CHECK(word_at(HOST_CHARGER_FRAME_INDEX,0) == (uint16_t)(CHARGE_MAX_VOLTAGE * 10.0f));
        CHECK(word_at(HOST_CHARGER_FRAME_INDEX,1) == (uint16_t)(CHARGE_MAX_CURRENT * 10.0f));
        CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == cases[i].disable);
        if(cases[i].disable){ CHECK(app.bms_state == false); }
        else { CHECK(app.bms_state == true && app.charger_fault == false); }
    }
}

static void test_charger_state_exit_shutdown_burst(void)
{
    static CAN_HandleTypeDef hcan;

    init_fake_app();
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.canbus.hcan = &hcan;
    app.state = STATE_CHARGE;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;

    taskENTER_CRITICAL();
    set_bms(false);
    CHECK(ams_state_transition_begin(&app,
                                     STATE_DISCARGE,
                                     AMS_STATE_TRANSITION_SERVICE_COMMAND,
                                     fake_tick) == AMS_STATE_TRANSITION_APPLIED);
    ams_state_transition_finish(&app);
    taskEXIT_CRITICAL();

    CHECK(app.board.charger.shutdown_pending == true);
    CHECK(app.board.charger.shutdown_frames_remaining == CHARGER_EXIT_DISABLE_FRAMES);
    CHECK(app.charger_fault == true);

    for(uint8_t frame = 0u; frame < CHARGER_EXIT_DISABLE_FRAMES; frame++)
    {
        tx_count = 0u;
        tx_free_level = 3u;
        run_one_canbus_task_iteration(&app);

        CHECK(tx_count == (HOST_NONCHARGE_CAN_FRAME_COUNT + 1u));
        CHECK(tx_log[0].ide == CAN_ID_EXT);
        CHECK(tx_log[0].extid == CCS_CANBUS_ID);
        CHECK(word_at(0u, 0u) == 0u);
        CHECK(word_at(0u, 1u) == 0u);
        CHECK(tx_log[0].data[4] == CHARGER_CMD_DISABLE);
        CHECK(app.board.charger.shutdown_tx_count == (uint32_t)frame + 1u);
        CHECK(app.board.charger.shutdown_frames_remaining ==
              (uint8_t)(CHARGER_EXIT_DISABLE_FRAMES - frame - 1u));

        if((frame + 1u) < CHARGER_EXIT_DISABLE_FRAMES)
        {
            CHECK(app.board.charger.shutdown_pending == true);
            CHECK(app.charger_fault == true);
        }
    }

    CHECK(app.board.charger.shutdown_pending == false);
    CHECK(app.board.charger.shutdown_frames_remaining == 0u);
    CHECK(app.board.charger.shutdown_tx_fail_count == 0u);
    CHECK(app.board.charger.last_shutdown_status == HAL_OK);
    CHECK(app.board.charger.tx_fail == false);
    CHECK(app.charger_fault == false);

    /* A failed first queue attempt consumes no shutdown frame and cannot be
     * hidden by the ordinary non-charge cleanup path. */
    init_fake_app();
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.canbus.hcan = &hcan;
    app.state = STATE_CHARGE;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    taskENTER_CRITICAL();
    set_bms(false);
    CHECK(ams_state_transition_begin(&app,
                                     STATE_DISCARGE,
                                     AMS_STATE_TRANSITION_SERVICE_COMMAND,
                                     fake_tick) == AMS_STATE_TRANSITION_APPLIED);
    ams_state_transition_finish(&app);
    taskEXIT_CRITICAL();

    fake_can_add_tx_status = HAL_ERROR;
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(&app);
    CHECK(app.board.charger.shutdown_pending == true);
    CHECK(app.board.charger.shutdown_frames_remaining == CHARGER_EXIT_DISABLE_FRAMES);
    CHECK(app.board.charger.shutdown_tx_count == 0u);
    CHECK(app.board.charger.shutdown_tx_fail_count == 1u);
    CHECK(app.board.charger.last_shutdown_status == HAL_ERROR);
    CHECK(app.board.charger.tx_fail == true);
    CHECK((app.board.charger.disable_reason_mask &
           (CHARGER_DISABLE_REASON_STATE_EXIT |
            CHARGER_DISABLE_REASON_TX_FAIL)) ==
          (CHARGER_DISABLE_REASON_STATE_EXIT |
           CHARGER_DISABLE_REASON_TX_FAIL));
    CHECK(app.charger_fault == true);
    CHECK(app.bms_state == false);

    fake_can_add_tx_status = HAL_OK;
    tx_count = 0u;
    tx_free_level = 3u;
    run_one_canbus_task_iteration(&app);
    CHECK(app.board.charger.shutdown_pending == true);
    CHECK(app.board.charger.shutdown_frames_remaining ==
          (CHARGER_EXIT_DISABLE_FRAMES - 1u));
    CHECK(app.board.charger.shutdown_tx_count == 1u);
    CHECK(app.board.charger.tx_fail == false);
    CHECK(app.charger_fault == true);
}

static void test_charger_command_priority_tx_failure_and_cli(void)
{
    static CAN_HandleTypeDef hcan;

    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.canbus.hcan = &hcan;
    app.state = STATE_CHARGE;
    app.current_valid = true;
    app.voltage_valid = true;
    app.temp_valid = true;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    fake_tick = 1000u;
    app.board.charger.last_rx_tick = fake_tick;
    tx_count = 0u;
    tx_free_level = 3u;

    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == HOST_CHARGE_CAN_FRAME_COUNT);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].ide == CAN_ID_EXT);
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].extid == CCS_CANBUS_ID);
    CHECK(word_at(HOST_CHARGER_FRAME_INDEX, 0u) == (uint16_t)(CHARGE_MAX_VOLTAGE * 10.0f));
    CHECK(word_at(HOST_CHARGER_FRAME_INDEX, 1u) == (uint16_t)(CHARGE_MAX_CURRENT * 10.0f));
    CHECK(tx_log[HOST_CHARGER_FRAME_INDEX].data[4] == CHARGER_CMD_ENABLE);
    CHECK(app.board.charger.tx_count == 1u);
    CHECK(app.board.charger.tx_fail == false);

    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
    charger_init(&app.board.charger, &app.board.canbus);
    app.board.canbus.hcan = &hcan;
    app.state = STATE_CHARGE;
    app.current_valid = true;
    app.voltage_valid = true;
    app.temp_valid = true;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    fake_tick = 2000u;
    app.board.charger.last_rx_tick = fake_tick;
    fake_can_add_tx_status = HAL_ERROR;
    tx_count = 0u;
    tx_free_level = 3u;

    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == 0u);
    CHECK(app.board.charger.tx_fail == true);
    CHECK(app.board.charger.tx_fail_count == 1u);
    CHECK(app.board.charger.last_tx_status == HAL_ERROR);
    CHECK((app.board.charger.disable_reason_mask & CHARGER_DISABLE_REASON_TX_FAIL) != 0u);
    CHECK(app.charger_fault == true);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
    CHECK(app.canbus_fault == true);

    fake_can_add_tx_status = HAL_OK;
    sil_prepare_cli_capture();
    get_charger(0, NULL);
    CHECK(strstr(cli_capture, "Charger target:") != NULL);
    CHECK(strstr(cli_capture, "tx_fail:1") != NULL);
    CHECK(strstr(cli_capture, "disable_mask:") != NULL);
    CHECK(strstr(cli_capture, "tx:0x1806E5F4") != NULL);
    CHECK(strstr(cli_capture, "BYTE5/data[4] enable:0 disable:1") != NULL);
}

static void test_telemetry_absent_segments_and_invalid_channels(void){
    static CAN_HandleTypeDef hcan;
    init_fake_app();
    app.board.canbus.hcan = &hcan;
    app.state = STATE_DISCARGE;
    app.air_state = false;
    app.current = 42.0f;
    app.max_temp = 25.0f; app.min_voltage = 3.4f; app.max_voltage = 4.0f;
    app.acc.smb.num_ics = 2;
    app.acc.smb.ics = app.acc.smb_ics;
    for(int ic=0; ic<NSMBS; ic++){
        for(int c=0; c<NCELLS; c++) app.acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(3.6f + 0.01f * (float)c);
        for(int s=0; s<NTEMPS; s++) app.acc.smb_ics[ic].temp.raw[s] = raw_for_temp_c(25.0f);
    }
    app.acc.smb_ics[0].cell.c_codes[1] = INT16_MIN;
    app.acc.smb_ics[1].temp.raw[3] = -1;
    host_mark_updated_cells(&app);
    accumulator_update_voltage_stats_at(&app.acc, fake_tick);
    host_mark_updated_temps(&app, (1UL << NTEMPS) - 1UL);
    accumulator_update_temp_stats_at(&app.acc, fake_tick);
    tx_count = 0; tx_free_level = 3;
    can_measurement_view_t view;
    can_measurement_view_build(&app, NULL, &view);
    CHECK(send_ecu_compact_telemetry(&app.board.canbus, &app, &view, 0u) == HAL_OK);
    CHECK(send_ecu_ams_status(&app.board.canbus, &app) == HAL_OK);
    CHECK(send_ecu_ams_voltages(&app.board.canbus, &app) == HAL_OK);
    CHECK(send_ecu_ams_temps(&app.board.canbus, &app) == HAL_OK);
    CHECK(send_ecu_ams_fans(&app.board.canbus, &app) == HAL_OK);
    CHECK(tx_count == (HOST_ECU_COMPACT_FRAME_COUNT + HOST_ECU_FRAME_COUNT));
    CHECK(word_at(HOST_LEGACY_ECU_FRAME_OFFSET + 3u,1) > 3500u && word_at(HOST_LEGACY_ECU_FRAME_OFFSET + 3u,2) == 0u && word_at(HOST_LEGACY_ECU_FRAME_OFFSET + 3u,3) > 3500u);
    CHECK(word_at(HOST_LEGACY_ECU_FRAME_OFFSET + 28u + 6u + 1u,1) == ECU_TEMP_INVALID_DECI_C); // segment 1, temp sensor 3 invalid => packet 35 word0
    // Segments 2..4 are absent because num_ics=2; their voltage and temp packets must be zero-filled.
    for(uint32_t frame=HOST_LEGACY_ECU_FRAME_OFFSET + 13u; frame<=HOST_LEGACY_ECU_FRAME_OFFSET + 27u; frame++) CHECK(word_at(frame,1)==0u && word_at(frame,2)==0u && word_at(frame,3)==0u);
    for(uint8_t seg=2u; seg<NSMBS; seg++){
        for(uint8_t packet=0u; packet<6u; packet++){
            uint32_t frame = HOST_LEGACY_ECU_FRAME_OFFSET + 28u + ((uint32_t)seg * 6u) + packet;
            uint8_t sensor = (uint8_t)(packet * 3u);
            CHECK(word_at(frame,1) == ECU_TEMP_INVALID_DECI_C);
            CHECK(word_at(frame,2) == ECU_TEMP_INVALID_DECI_C);
            CHECK(word_at(frame,3) == ((sensor + 2u < ECU_SEG_TEMPS) ? ECU_TEMP_INVALID_DECI_C : 0u));
        }
    }
}

static void test_periods_and_driver_edge_cases(void){
    static CAN_HandleTypeDef hcan;
	static SPI_HandleTypeDef hspi;
	static GPIO_TypeDef cs_a;
	static GPIO_TypeDef cs_b;
    init_fake_app(); fill_nominal_pack(&app, 3.7f); app.board.canbus.hcan=&hcan; app.state=STATE_DISCARGE; fake_tick=123; tx_count=0; tx_free_level=3;
    run_one_canbus_task_iteration(&app); CHECK(fake_tick == 123u + CAN_ECU_FAST_PERIOD_MS);

    init_fake_app(); app.bms_state=true; fake_tick=100; run_one_error_task_iteration(&app); CHECK(fake_tick == 100u + (1000u / ERR_FREQ));

    fan_t fan={0}; uint32_t ccr=1234; static TIM_HandleTypeDef htim;
    CHECK(fan_init(&fan, NULL, &htim, 1000u, &ccr, 1) == 0);
    CHECK(set_fan_percent(&fan, NAN) == 0); CHECK(ccr == 0u && fan.duty_cycle == 0.0f);
    CHECK(set_fan_percent(&fan, INFINITY) == 0); CHECK(ccr == 0u && fan.duty_cycle == 0.0f);
    CHECK(set_fan_percent(&fan, -INFINITY) == 0); CHECK(ccr == 0u && fan.duty_cycle == 0.0f);
	fake_tim_pwm_start_status = HAL_ERROR;
	ccr = 1234u;
	CHECK(fan_init(&fan, NULL, &htim, 1000u, &ccr, 1) != 0);
	CHECK(fan.initialized == false);
	CHECK(fan.init_status == HAL_ERROR);
	CHECK(ccr == 0u);
	CHECK(set_fan_percent(&fan, 50.0f) != 0);
	fake_tim_pwm_start_status = HAL_OK;

	accumulator_t timer_acc;
	fake_tim_base_start_status = HAL_ERROR;
	accumulator_init(&timer_acc, &hspi, &cs_a, &cs_b, 1u, 2u, &htim);
	CHECK(timer_acc.delay_timer_ready == false);
	CHECK(timer_acc.delay_timer_status == HAL_ERROR);
	CHECK(timer_acc.smb.htim == NULL);
	fake_tim_base_start_status = HAL_OK;
	fake_adbms_init_status = HAL_TIMEOUT;
	accumulator_init(&timer_acc, &hspi, &cs_a, &cs_b, 1u, 2u, &htim);
	CHECK(timer_acc.delay_timer_ready == true);
	CHECK(timer_acc.smb_ready == false);
	CHECK(timer_acc.smb_init_status == HAL_TIMEOUT);
	fake_adbms_init_status = HAL_OK;
	accumulator_init(&timer_acc, &hspi, &cs_a, &cs_b, 1u, 2u, &htim);
	CHECK(timer_acc.delay_timer_ready == true);
	CHECK(timer_acc.delay_timer_status == HAL_OK);
	CHECK(timer_acc.smb_ready == true);
	CHECK(timer_acc.smb_init_status == HAL_OK);
	CHECK(timer_acc.smb.htim == &htim);
	fake_adbms_config_mismatch_mask = 0x0001u;
	accumulator_init(&timer_acc, &hspi, &cs_a, &cs_b, 1u, 2u, &htim);
	CHECK(timer_acc.smb_ready == false);
	CHECK(timer_acc.smb_init_status == HAL_ERROR);
	fake_adbms_config_mismatch_mask = 0u;
	fake_adbms_diag_status = HAL_TIMEOUT;
	accumulator_init(&timer_acc, &hspi, &cs_a, &cs_b, 1u, 2u, &htim);
	CHECK(timer_acc.smb_ready == false);
	CHECK(timer_acc.smb_init_status == HAL_TIMEOUT);
	fake_adbms_diag_status = HAL_OK;

	canbus_device_t cb = {0};
	CHECK(canbus_device_init(NULL, &hcan) == HAL_ERROR);
	fake_can_recover_status = HAL_ERROR;
	CHECK(canbus_device_init(&cb, &hcan) == HAL_ERROR);
	CHECK(cb.started == false);
	CHECK(cb.notification_active == false);
	fake_can_recover_status = HAL_OK;
	fake_can_notification_status = HAL_ERROR;
	CHECK(canbus_device_init(&cb, &hcan) == HAL_ERROR);
	CHECK(cb.started == true);
	CHECK(cb.notification_active == false);
	fake_can_notification_status = HAL_OK;
	CHECK(canbus_device_init(&cb, &hcan) == HAL_OK);
	CHECK(cb.started == true);
	CHECK(cb.notification_active == true);

	CHECK(canbus_send(NULL, CAN_ID_STD, ECU_CANBUS_ID, (uint8_t[8]){0}) == HAL_ERROR);
	cb.hcan = NULL;
    CHECK(canbus_send(&cb, CAN_ID_STD, ECU_CANBUS_ID, (uint8_t[8]){0}) == HAL_ERROR);
    cb.hcan = &hcan; CHECK(canbus_send(&cb, CAN_ID_STD, ECU_CANBUS_ID, NULL) == HAL_ERROR);
}


#if AMS_HOST_ONLY_5SMB_NO_APM_TEST
static void test_five_smb_no_apm_topology_and_measurement_paths(void)
{
    accumulator_t acc;
    SPI_HandleTypeDef spi = {0};
    GPIO_TypeDef gpio_a = {0};
    GPIO_TypeDef gpio_b = {0};
    TIM_HandleTypeDef timer = {0};

    CHECK(AMS_ACCUMULATOR_5SMB_NO_APM == 1);
    CHECK(AMS_BUILD_PROFILE == AMS_PROFILE_BENCH);
    CHECK(AMS_HW_BRINGUP == 1);
    CHECK(AMS_HIL_REPLACE_ADBMS == 0);
    CHECK(AMS_ENABLE_HIL_CAN == 0);
    CHECK(AMS_ENABLE_APM_2950 == 0);
    CHECK(AMS_ADBMS_PHYSICAL_CHAIN_COUNT == 5u);
    CHECK(AMS_ADBMS_BALANCE_ACTIVATION_ENABLED == 0);
    CHECK(AMS_ADBMS_OPEN_WIRE_ENABLED == 0);
    CHECK(NSMBS == 5u);
    CHECK(NCELLS == 15u);
    CHECK(NTEMPS == 24u);

    fake_tim_base_start_status = HAL_OK;
    fake_adbms_init_status = HAL_OK;
    fake_adbms_diag_status = HAL_OK;
    fake_adbms_config_mismatch_mask = 0u;
    fake_apm_init_call_count = 0u;
    fake_apm_sample_call_count = 0u;
    fake_adbms_wrcfgb_call_count = 0u;
    fake_adbms_wrpwm_call_count = 0u;
    memset(&acc, 0xA5, sizeof(acc));

    accumulator_init(&acc,
                     &spi,
                     &gpio_a,
                     &gpio_b,
                     1u,
                     2u,
                     &timer);

    CHECK(acc.delay_timer_ready == true);
    CHECK(acc.smb_ready == true);
    CHECK(acc.smb.num_ics == 5);
    CHECK(acc.smb.physical_chain_count == 5u);
    CHECK(acc.smb.ics_capacity == 5u);
    CHECK(acc.smb.ics == acc.smb_ics);
    CHECK(acc.smb.string == STRING_A);
    CHECK(acc.smb.write_string == STRING_A);
    CHECK(acc.smb.monitored_cell_count == 15u);
    CHECK(accumulator_final_ring_topology_valid(&acc));
    CHECK(acc.apm_ready == false);
    CHECK(acc.apm_init_status == HAL_ERROR);
    CHECK(fake_apm_init_call_count == 0u);

    CHECK(accumulator_read_volt(&acc) == 0);
    CHECK(acc.apm_full_ring_awake_token == false);
    CHECK(accumulator_read_apm(&acc, 100u) == -1);
    CHECK(fake_apm_sample_call_count == 0u);
    CHECK(accumulator_read_temp(&acc) == 0);

    /* Activation is compile-time disabled, while an explicit all-off write
     * remains available to clear any stale discharge state safely. */
    uint32_t cfg_before = fake_adbms_wrcfgb_call_count;
    uint32_t pwm_before = fake_adbms_wrpwm_call_count;
    CHECK(accumulator_set_balance(&acc) == -1);
    CHECK(fake_adbms_wrcfgb_call_count == cfg_before);
    CHECK(fake_adbms_wrpwm_call_count == pwm_before);
    CHECK(accumulator_clear_balance(&acc) == 0);
    CHECK(fake_adbms_wrcfgb_call_count > cfg_before);
    CHECK(fake_adbms_wrpwm_call_count > pwm_before);
}

static void test_five_smb_no_apm_cli_lockouts(void)
{
    char *apm_sid[] = {"apm", "sid", NULL};
    char *probe_b[] = {"spi", "probeb", NULL};
    char *open_wire[] = {"spi", "owcheck", NULL};
    char *bms_release[] = {"bmsok", "release", NULL};
    char *balance_release[] = {"balance", "release", NULL};
    char *state_charge[] = {"state", "charge", NULL};
    char *current_zero[] = {"current", "zero", NULL};
    char *evidence[] = {"bringup", "evidence", NULL};

    init_fake_app();
    app.acc.apm_ready = false;
    app.acc.apm_init_status = HAL_ERROR;
    app.acc.apm.health.initialized = false;
    app.bms_output_inhibit = true;
    app.balance_inhibit = true;
    sil_prepare_cli_capture();

    fake_apm_probe_call_count = 0u;
    CHECK(get_apm_debug(2, apm_sid) == 0);
    CHECK(fake_apm_probe_call_count == 0u);
    CHECK(strstr(cli_capture, "physically absent") != NULL);

    cli_capture_clear();
    CHECK(get_spi_debug(2, probe_b) == 0);
    CHECK(fake_apm_probe_call_count == 0u);
    CHECK(strstr(cli_capture, "blocked") != NULL);

    cli_capture_clear();
    fake_adbms_open_wire_call_count = 0u;
    CHECK(get_spi_debug(2, open_wire) == 0);
    CHECK(fake_adbms_open_wire_call_count == 0u);
    CHECK(strstr(cli_capture, "blocked") != NULL);

    cli_capture_clear();
    CHECK(bmsok_control(2, bms_release) == 0);
    CHECK(app.bms_output_inhibit == true);
    CHECK(app.bms_state == false);
    CHECK(strstr(cli_capture, "compile-time output-inhibited") != NULL);

    cli_capture_clear();
    CHECK(balance_control(2, balance_release) == 0);
    CHECK(app.balance_inhibit == true);
    CHECK(strstr(cli_capture, "clear/off writes only") != NULL);

    cli_capture_clear();
    CHECK(set_state(2, state_charge) == 0);
    CHECK(app.state == STATE_START);
    CHECK(strstr(cli_capture, "START/monitor") != NULL);

    cli_capture_clear();
    CHECK(get_current(2, current_zero) == 0);
    CHECK(strstr(cli_capture, "mutation blocked") != NULL);

    app.temp_valid = false;
    app.temp_read_fault = true;
    run_one_fan_task_iteration(&app);
    CHECK(app.fan_state == false);
    CHECK(app.fan_command_percent == 0.0f);

    cli_capture_clear();
    CHECK(get_bringup(2, evidence) == 0);
    CHECK(strstr(cli_capture, "confirm masks 0x001F") != NULL);
    CHECK(strstr(cli_capture, "apm sample") == NULL);
}
#endif

#if AMS_HOST_PRODUCTION_GATE_TEST
static void test_production_safety_gates(void)
{
    static CAN_HandleTypeDef hcan;

    CHECK(AMS_ENABLE_HIL_CAN == 0);
    CHECK(AMS_ENABLE_SERVICE_CLI == 0);
    CHECK(AMS_ENABLE_AIR_AUX_FEEDBACK == 0);
    CHECK(AMS_AIR_AUX_BOARD_ADAPTER_READY == 0);
    CHECK(AMS_AIR_MONITOR_PERIOD_MS == 0u);
    CHECK(AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS == 0u);

    /* Production CAN builds must record the frame for diagnostics without
     * allowing HIL IDs to overwrite authoritative accumulator measurements. */
    init_fake_app();
    app.acc.smb.num_ics = NSMBS;
    app.acc.smb.ics = app.acc.smb_ics;
    app.acc.smb_ics[0].cell.c_codes[0] = 12345;
    app.acc.smb_ics[0].temp.raw[0] = 2345;
    app.hil.meas.fresh = 0u;
    app.hil.truth.fresh = 0u;
    app.hil.summary.fresh = 0u;

    host_send_hil_cell_triplet(0u, 0u, 4100u, 4090u, 4080u);
    host_send_hil_temp_triplet(0u, 0u, 550, 560, 570);
    CHECK(app.acc.smb_ics[0].cell.c_codes[0] == 12345);
    CHECK(app.acc.smb_ics[0].temp.raw[0] == 2345);

    memset(&fake_rx_hdr, 0, sizeof(fake_rx_hdr));
    memset(fake_rx_data, 0, sizeof(fake_rx_data));
    fake_rx_hdr.IDE = CAN_ID_STD;
    fake_rx_hdr.StdId = AMS_HIL_CAN_ID_MEAS;
    fake_rx_hdr.DLC = 7u;
    fake_rx_data[0] = 0x0Bu;
    fake_rx_data[1] = 0xB8u;
    app.board.canbus.hcan = &hcan;
    (void)host_receive_can_frame(&hcan);
    CHECK(app.hil.meas.fresh == 0u);

    /* Service actions remain visible for diagnosis but are immutable in the
     * production profile.  Safe inhibit/status commands remain available. */
    sil_prepare_cli_capture();
    app.bms_output_inhibit = true;
    char *bms_release[] = {"bmsok", "release", NULL};
    CHECK(bmsok_control(2, bms_release) == 0);
    CHECK(app.bms_output_inhibit == true);
    CHECK(strstr(cli_capture, "refused") != NULL);

    sil_prepare_cli_capture();
    app.balance_inhibit = true;
    char *balance_release[] = {"balance", "release", NULL};
    CHECK(balance_control(2, balance_release) == 0);
    CHECK(app.balance_inhibit == true);
    CHECK(strstr(cli_capture, "refused") != NULL);

    sil_prepare_cli_capture();
    app.state = STATE_START;
    app.board.charger.last_rx_tick = 777u;
    app.board.charger.communication_fail = false;
    char *state_charge[] = {"state", "charge", NULL};
    CHECK(set_state(2, state_charge) == 0);
    CHECK(app.state == STATE_START);
    CHECK(app.board.charger.last_rx_tick == 777u);
    CHECK(app.board.charger.communication_fail == false);
    CHECK(strstr(cli_capture, "refused") != NULL);

    ams_fault_log_clear();
    ams_fault_log_event(AMS_FAULT_LOG_BMS_OK_DROPPED, 0u, 1u, 2u);
    uint32_t log_count_before = ams_fault_log_get()->count;
    sil_prepare_cli_capture();
    char *fault_clear[] = {"fault", "log", "clear", NULL};
    CHECK(get_faults(3, fault_clear) == 0);
    CHECK(ams_fault_log_get()->count == log_count_before);
    CHECK(strstr(cli_capture, "refused") != NULL);

    sil_prepare_cli_capture();
    uint32_t spi_tx_before = app.acc.smb.spi_debug.tx_count;
    char *spi_scope[] = {"spi", "scope", "b", "read", "1", NULL};
    CHECK(get_spi_debug(5, spi_scope) == 0);
    CHECK(app.acc.smb.spi_debug.tx_count == spi_tx_before);
    CHECK(strstr(cli_capture, "refused") != NULL);

    sil_prepare_cli_capture();
    char *apm_probe[] = {"apm", "probe", NULL};
    CHECK(get_apm_debug(2, apm_probe) == 0);
    CHECK(strstr(cli_capture, "refused") != NULL);

    /* The IMD driver and application state are fail-closed until a validated
     * PWM/status result is produced. */
    imd_t imd;
    memset(&imd, 0xA5, sizeof(imd));
    imd_init(&imd, 1000000u, NULL, NULL, TIM_CHANNEL_1, TIM_CHANNEL_2, NULL, 0u);
    CHECK(imd.ret != 0);
    CHECK(imd.OK_HS == false);
    CHECK(imd.status == IMD_UNKNOWN);

    init_fake_app();
    sil_make_measurement_gates_ready(&app);
    sil_mark_all_heartbeats_alive(&app);
    app.imd_valid = false;
    app.imd_ok = false;
    app.imd_fault = true;
    app.imd_status = IMD_UNKNOWN;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    error_task_update(&app, fake_tick);
    CHECK(app.hard_fault == true);
    CHECK(app.bms_supervisor_ready == false);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
}
#endif

#if AMS_HOST_AIR_FEEDBACK_GATE_TEST
static void test_air_feedback_future_gate_is_fail_closed(void)
{
    init_fake_app();
    sil_make_measurement_gates_ready(&app);
    sil_mark_all_heartbeats_alive(&app);
    ams_air_monitor_init(&app.air_monitor, true);
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;

    /* Enabling the future gate without a published feedback snapshot cannot
     * accidentally inherit the old AIR_CONTROL_MCU signal as "healthy". */
    error_task_update(&app, fake_tick);
    CHECK(app.air_state == true);
    CHECK(app.air_monitor.feedback_valid == false);
    CHECK(app.air_monitor.reason == AMS_AIR_FAULT_WAITING_FOR_INPUTS);
    CHECK(app.hard_fault == true);
    CHECK(app.bms_supervisor_ready == false);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    /* Synthetic future steady-state snapshot: AIR- and AIR+ closed, precharge
     * open. This only proves the software gate contract; no pin/timing values
     * are claimed by this host test. */
    app.air_monitor.configuration_valid = true;
    app.air_monitor.command_valid = true;
    app.air_monitor.feedback_valid = true;
    app.air_monitor.steady_state_valid = true;
    app.air_monitor.boot_open_verified = true;
    app.air_monitor.transition_authorized = true;
    app.air_monitor.permit = true;
    app.air_monitor.fault = false;
    app.air_monitor.fault_latched = false;
    app.air_monitor.phase = AMS_AIR_PHASE_RUN;
    app.air_monitor.pos_aux = AMS_AIR_CONTACT_CLOSED;
    app.air_monitor.neg_aux = AMS_AIR_CONTACT_CLOSED;
    app.air_monitor.precharge_aux = AMS_AIR_CONTACT_OPEN;
    app.air_monitor.reason = AMS_AIR_FAULT_NONE;
    app.air_monitor.last_update_tick = fake_tick;
    error_task_update(&app, fake_tick);
    CHECK(app.state == STATE_DISCARGE);
    CHECK(app.bms_supervisor_ready == false);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);

    /* START -> DISCARGE deliberately cannot assert BMS_OK using a current
     * result evaluated against the old startup/precharge thresholds.  Model
     * the next current-task publication before asking the supervisor again. */
    app.current_fault_mode = CURRENT_FAULT_MODE_DRIVE;
    error_task_update(&app, fake_tick);
    CHECK(app.hard_fault == false);
    CHECK(app.bms_supervisor_ready == true);
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);

    /* Even an internally inconsistent/corrupted snapshot cannot turn the
     * terminal SHUTDOWN phase or a nonzero mask into a healthy supervisor
     * decision. */
    app.air_monitor.phase = AMS_AIR_PHASE_SHUTDOWN;
    error_task_update(&app, fake_tick);
    CHECK(app.hard_fault == false);
    CHECK(app.bms_supervisor_ready == false);
    CHECK(app.bms_state == false);
    app.air_monitor.phase = AMS_AIR_PHASE_RUN;
    app.air_monitor.active_fault_mask = AMS_AIR_FAULT_BIT_COMMAND;
    error_task_update(&app, fake_tick);
    CHECK(app.hard_fault == true);
    CHECK(app.bms_supervisor_ready == false);
    CHECK(app.bms_state == false);
    app.air_monitor.active_fault_mask = 0u;
    error_task_update(&app, fake_tick);
    CHECK(app.hard_fault == false);
    CHECK(app.bms_supervisor_ready == true);
    CHECK(app.bms_state == true);

    /* The supervisor independently ages the publication itself. A frozen AIR
     * task cannot leave its last healthy snapshot authoritative forever. */
    fake_tick += AMS_AIR_MONITOR_PUBLICATION_TIMEOUT_MS + 1u;
    error_task_update(&app, fake_tick);
    CHECK(app.hard_fault == true);
    CHECK(app.bms_supervisor_ready == false);
    CHECK(app.bms_state == false);
    app.air_monitor.last_update_tick = fake_tick;
    error_task_update(&app, fake_tick);
    CHECK(app.hard_fault == false);
    CHECK(app.bms_supervisor_ready == true);
    CHECK(app.bms_state == true);

    /* A validated closing transition may retain permission while the contact
     * moves inside its deadline; it is not yet a fault. */
    app.air_monitor.steady_state_valid = false;
    app.air_monitor.transition_pending = true;
    app.air_monitor.reason = AMS_AIR_FAULT_TRANSITION_PENDING;
    error_task_update(&app, fake_tick);
    CHECK(app.hard_fault == false);
    CHECK(app.bms_supervisor_ready == true);
    CHECK(app.bms_state == true);

    /* Opening transitions deliberately do not retain that permission. */
    app.air_monitor.permit = false;
    error_task_update(&app, fake_tick);
    CHECK(app.hard_fault == false);
    CHECK(app.bms_supervisor_ready == false);
    CHECK(app.bms_state == false);

    app.air_monitor.fault = true;
    app.air_monitor.fault_latched = true;
    app.air_monitor.reason = AMS_AIR_FAULT_POS_WELDED;
    error_task_update(&app, fake_tick);
    CHECK(app.hard_fault == true);
    CHECK(app.bms_supervisor_ready == false);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
}
#endif

int main(void){
#if AMS_HOST_ONLY_5SMB_NO_APM_TEST
    test_five_smb_no_apm_topology_and_measurement_paths();
    puts("PASS five-SMB/no-APM topology and voltage/temperature paths");
    test_five_smb_no_apm_cli_lockouts();
    puts("PASS five-SMB/no-APM CLI and actuation lockouts");
    test_current_sensor_measurement_model();
    puts("PASS DHAB dual-range measurement model in fixture build");
    test_current_task_measurement_state();
    puts("PASS DHAB current-task publication in fixture build");
    puts("ALL FIVE-SMB/NO-APM FIXTURE TESTS PASSED");
    return 0;
#elif AMS_HOST_PRODUCTION_GATE_TEST
    test_production_safety_gates();
    puts("PASS production HIL/service/IMD/AIR/supervisor gates");
    puts("ALL PRODUCTION SAFETY GATE TESTS PASSED");
    return 0;
#elif AMS_HOST_AIR_FEEDBACK_GATE_TEST
    test_air_feedback_future_gate_is_fail_closed();
    puts("PASS future AIR auxiliary-feedback fail-closed gate");
    return 0;
#elif AMS_HOST_ONLY_HIL_ADBMS_TEST
    test_hil_adbms_image_replaces_raw_reads(); puts("PASS HIL ADBMS image replacement");
    test_accumulator_tick_wrap_freshness(); puts("PASS accumulator tick-wrap freshness");
    puts("ALL HIL ADBMS REPLACEMENT TESTS PASSED");
    return 0;
#else
    test_accumulator_stats_and_balance(); puts("PASS accumulator stats/balance");
    test_adbms_voltage_scan_timing_contract(); puts("PASS ADBMS wake/reference/conversion timing contract");
    test_voltage_stats_boundaries_and_fuzz(); puts("PASS voltage boundary/fuzz stats");
    test_voltage_fault_policy_and_strict_scan_freshness(); puts("PASS voltage fault strict scan freshness policy");
    test_system_sil_boot_ready_and_bms_conjunction(); puts("PASS system SIL boot/readiness/BMS conjunction");
    test_system_sil_single_pec_miss_drops_bms_then_recovers(); puts("PASS system SIL single PEC miss fail-closed/recovery");
    test_system_sil_persistent_voltage_stale_drops_bms_ok(); puts("PASS system SIL persistent voltage stale fail-closed");
    test_system_sil_charge_stop_allows_balance_before_hard_ov(); puts("PASS system SIL charge-stop balancing vs hard OV");
    test_system_sil_balance_inhibit_ladder_bringup_lockout(); puts("PASS system SIL balance inhibit ladder bring-up lockout");
    test_system_sil_bench_cli_abuse_and_balance_idempotence(); puts("PASS system SIL bench CLI abuse/balance idempotence");
    test_system_sil_bench_state_transition_balance_cleanup(); puts("PASS system SIL bench state-transition balance cleanup");
    test_system_sil_state_transition_guard_and_audit(); puts("PASS system SIL state-transition guard/audit");
    test_system_sil_bench_bmsok_inhibit_survives_ready_tasks(); puts("PASS system SIL bench BMS_OK inhibit survives ready tasks");
    test_system_sil_bench_adbms_write_failures_and_recovery(); puts("PASS system SIL bench ADBMS write failures/recovery");
    test_system_sil_voltage_uv_ov_severe_diagnostics_and_latch(); puts("PASS system SIL voltage severe diagnostics/latch");
    test_system_sil_current_warning_fast_trip_and_latch_persistence(); puts("PASS system SIL current warning/fast-trip/latch");
    test_system_sil_current_stale_adc_pair_fails_safe(); puts("PASS system SIL current stale ADC pair fail-safe");
    test_system_sil_regen_and_charge_current_placeholders(); puts("PASS system SIL regen/charge current placeholder behavior");
    test_system_sil_2950_advisory_sampling_and_cli(); puts("PASS system SIL 2950 advisory sampling/CLI");
    test_system_sil_2950_final_ring_init_ownership(); puts("PASS system SIL 2950 final-ring init/reset ownership");
    test_system_sil_final_ring_topology_corruption_fails_closed(); puts("PASS system SIL final-ring topology corruption fail-closed");
    test_system_sil_2950_requires_successful_full_ring_scan(); puts("PASS system SIL 2950 full-ring scan coordination");
    test_system_sil_combined_fault_precedence_and_reset_path(); puts("PASS system SIL combined fault precedence/reset path");
    test_system_sil_harsh_timeline_no_false_enable(); puts("PASS system SIL harsh timeline/no false enable");
    test_system_sil_current_invalid_immediate_bms_drop_and_recovery(); puts("PASS system SIL current-invalid immediate BMS drop/recovery");
    test_system_sil_hard_fault_and_corrupt_smb_config_fail_closed(); puts("PASS system SIL hard-fault/corrupt SMB config fail-closed");
    test_system_sil_voltage_threshold_exact_edges(); puts("PASS system SIL exact voltage threshold edges");
    test_system_sil_charger_disable_from_dynamic_gates(); puts("PASS system SIL charger disable dynamic gates");
    test_system_sil_deterministic_fault_injection_invariants(); puts("PASS system SIL deterministic fault-injection invariants");
    test_system_sil_task_order_permutations_fail_closed(); puts("PASS system SIL task-order permutations fail closed");
    test_system_sil_recovery_and_latch_reset_paths(); puts("PASS system SIL warning recovery/latch reset paths");
    test_system_sil_current_boundary_timing_edges(); puts("PASS system SIL current debounce boundary timing");
    test_system_sil_cli_can_diagnostic_consistency(); puts("PASS system SIL CLI/CAN diagnostic consistency");
    test_current_service_calibration_boundary(); puts("PASS current service calibration ownership boundary");
    test_cli_numeric_and_telemetry_conversion_guards(); puts("PASS CLI numeric/telemetry conversion guards");
    test_adbms6830_diagnostic_commands_and_cli_health(); puts("PASS ADBMS6830 diagnostic commands/CLI health");
    test_adbms_periodic_diagnostics_and_safe_open_wire(); puts("PASS ADBMS6830 periodic diagnostics/safe open-wire");
    test_adbms_cli_scan_guard_and_cs_probe_commands(); puts("PASS ADBMS CLI scan guard/CS probe commands");
    test_software_heartbeat_monitor_faults_and_recovery(); puts("PASS software heartbeat monitor faults/recovery");
    test_supervisor_rejects_non_operating_and_corrupt_states(); puts("PASS supervisor rejects non-operating/corrupt states");
    test_system_sil_bringup_status_and_bmsok_inhibit(); puts("PASS system SIL bring-up status/BMS_OK inhibit");
    test_bringup_cli_board_ready_and_adbms_summaries(); puts("PASS bring-up CLI board/ready/ADBMS summaries");
    test_bringup_cli_apm_and_charger_phase_split(); puts("PASS bring-up CLI APM/charger phase split");
    test_system_sil_contradictory_dhab_vs_2950_observable_non_gating(); puts("PASS system SIL DHAB vs 2950 contradiction observable/non-gating");
    test_system_sil_startup_garbage_never_enables_bms(); puts("PASS system SIL startup garbage never enables BMS_OK");
    test_system_sil_long_run_seeded_fuzz_invariants(); puts("PASS system SIL seeded fuzz invariants");
    test_system_sil_concurrent_heartbeat_starvation_and_recovery(); puts("PASS system SIL concurrent heartbeat starvation/recovery");
    test_system_sil_concurrent_charger_tx_recovery_ordering(); puts("PASS system SIL concurrent charger TX recovery ordering");
    test_system_sil_concurrent_seeded_scheduler_abuse(); puts("PASS system SIL concurrent seeded scheduler abuse");
    test_temp_stats(); puts("PASS temp stats");
    test_temp_invalid_and_cold_valid_fault_behavior(); puts("PASS temp invalid/cold-valid fault behavior");
    test_system_sil_temperature_mux_cadence_no_false_stale(); puts("PASS system SIL temperature mux cadence no false stale");
    test_system_sil_temperature_invalid_update_overrides_history(); puts("PASS system SIL temperature invalid update overrides history");
    test_system_sil_temperature_startup_partial_and_stale_fail_closed(); puts("PASS system SIL temperature startup/stale fail-closed");
    test_system_sil_temperature_fan_ramp_warning_and_recovery(); puts("PASS system SIL temperature fan ramp/warning/recovery");
    test_system_sil_temperature_charge_stop_thresholds(); puts("PASS system SIL temperature charge-stop thresholds");
    test_system_sil_temperature_hard_latch_and_reset_path(); puts("PASS system SIL temperature hard/severe latch/reset");
    test_system_sil_temperature_cli_can_diagnostics(); puts("PASS system SIL temperature CLI/CAN diagnostics");
    test_can_telemetry_packets(); puts("PASS CAN telemetry packetization");
    test_can_telemetry_pacing_and_snapshot(); puts("PASS CAN telemetry pacing/snapshot contract");
    test_can_priority_metrics_and_deadlines(); puts("PASS CAN priority metrics/deadline accounting");
    test_logger_can_contract_packets(); puts("PASS logger CAN contract packetization");
    test_telemetry_absent_segments_and_invalid_channels(); puts("PASS telemetry absent segments/invalid channels");
    test_charger_rx_and_tx(); puts("PASS charger RX/TX parse");
    test_measurement_epoch_contract(); puts("PASS measurement epoch/current-window contract");
    test_estimator_ra8m1_architecture_parity(); puts("PASS estimator RA8M1 architecture parity");
    test_estimator_lut_and_config_matrix(); puts("PASS estimator LUT/config matrix");
    test_estimator_step_faults_and_scalability(); puts("PASS estimator step faults/scalability");
    test_estimator_epoch_sequence_and_timing(); puts("PASS estimator epoch sequence/timing");
    test_estimator_model_domain_flags(); puts("PASS estimator model-domain flags");
    test_hil_parser_edge_cases(); puts("PASS HIL parser edge cases");
    test_hil_parser_and_estimator_core(); puts("PASS HIL parser and estimator core");
    test_hil_adbms_image_replaces_raw_reads(); puts("PASS HIL ADBMS image replacement");
    test_accumulator_tick_wrap_freshness(); puts("PASS accumulator tick-wrap freshness");
    test_estimator_task_hil_and_hardware_paths(); puts("PASS estimator task HIL/hardware paths");
    test_estimator_rejects_invalid_hardware_inputs(); puts("PASS estimator rejects invalid hardware inputs");
    test_estimator_status_packet_edges(); puts("PASS estimator status packet edges");
    test_can_rx_filter_matrix(); puts("PASS CAN RX filter matrix");
    test_can_rx_isr_queue_ownership_and_overflow(); puts("PASS CAN RX ISR queue ownership/overflow");
    test_charge_state_disable_matrix(); puts("PASS charge-state disable matrix");
    test_charger_state_exit_shutdown_burst(); puts("PASS charger state-exit shutdown burst/retry");
    test_charger_command_priority_tx_failure_and_cli(); puts("PASS charger command priority/TX failure/CLI diagnostics");
    test_current_sensor_measurement_model(); puts("PASS current sensor measurement model");
    test_current_task_measurement_state(); puts("PASS current task measurement state");
    test_current_task_threshold_faults(); puts("PASS current task threshold faults");
    test_air_feedback_scaffold(); puts("PASS AIR feedback disabled/fail-closed scaffold");
    test_air_monitor_nominal_sequence_and_weld_clear(); puts("PASS AIR monitor nominal sequence/weld clear");
    test_air_monitor_shutdown_rearm_and_live_precharge_proof(); puts("PASS AIR monitor shutdown rearm/live precharge proof");
    test_air_monitor_transition_state_space_and_stale_recovery(); puts("PASS AIR monitor transition state-space/stale recovery");
    test_air_monitor_ready_snapshot_integrity(); puts("PASS AIR monitor ready snapshot integrity");
    test_air_monitor_faults_freshness_and_tick_wrap(); puts("PASS AIR monitor fault/freshness/tick-wrap policy");
    test_air_monitor_seeded_invariants(); puts("PASS AIR monitor seeded invariants");
    test_fan_current_and_null_guards(); puts("PASS fan/current/null guards");
    test_imd_capture_validation(); puts("PASS IMD capture validation");
    test_periods_and_driver_edge_cases(); puts("PASS periods and driver edge cases");
    test_task_iterations_with_injected_signals(); puts("PASS one-iteration task injection tests");
    test_safety_panic_reset_watchdog_and_log(); puts("PASS safety panic/reset/log path");
    test_retained_fault_log_integrity_recovery(); puts("PASS retained fault-log integrity/recovery");
    test_watchdog_feed_gate(); puts("PASS watchdog feed gate");
    test_watchdog_boot_arm_and_startup_grace(); puts("PASS watchdog boot arm/startup grace");
    test_watchdog_start_failure_is_fail_closed(); puts("PASS watchdog start-failure fail-closed gate");
    test_rtos_stack_heap_diag_and_faults(); puts("PASS RTOS stack/heap diagnostics");
    test_can_busoff_sets_fault_and_recovers(); puts("PASS CAN bus-off fault/recovery");
    test_fault_matrix_extra(); puts("PASS fault matrix extra");
    puts("ALL COMPREHENSIVE HOST INJECTION TESTS PASSED");
    return 0;
#endif
}
