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

GPIO_TypeDef dummy_gpio;
TIM_TypeDef tim3_inst, tim4_inst, tim5_inst;
app_data_t app;
static uint32_t fake_tick = 0;
static GPIO_PinState bms_pin_state = GPIO_PIN_RESET;
static uint32_t tx_count = 0;
static struct { uint32_t ide, stdid, extid, dlc; uint8_t data[8]; } tx_log[AMS_HOST_TX_LOG_CAPACITY];
static uint32_t tx_free_level = 3;
static HAL_StatusTypeDef fake_can_add_tx_status = HAL_OK;
static uint32_t fake_can_error = HAL_CAN_ERROR_NONE;
static HAL_StatusTypeDef fake_can_recover_status = HAL_OK;
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
static uint16_t fake_adbms_config_mismatch_mask = 0u;

static uint16_t adc_count_for_mcu_voltage(float v);
static uint16_t adc_count_for_sensor_voltage(float v);
static void fake_adc_set_two_read_sequence(uint16_t high_count, uint16_t low_count);
static void fake_adc_set_status_sequence(HAL_StatusTypeDef high_status, HAL_StatusTypeDef low_status);
static void fake_adbms_voltage_masks_full_update(void);
static void fake_adbms_voltage_masks_all_missing(bool pec_fail);
static void fake_adbms_voltage_masks_one_missing(uint8_t seg, uint8_t cell, bool pec_fail);
static void fake_adc_set_current_a(float current_a);
static void sil_publish_temp_state(app_data_t *d);
static void sil_expect_balancing_clear(const app_data_t *d);
static uint8_t sil_balance_pwm_duty(const app_data_t *d, uint8_t ic, uint8_t cell);
static void sil_prepare_cli_capture(void);
static void sil_run_can_charge_iteration(app_data_t *d, CAN_HandleTypeDef *hcan);

uint32_t osKernelGetTickCount(void){ return fake_tick; }
osStatus_t osDelay(uint32_t ticks){ fake_tick += ticks; return osOK; }
osStatus_t osDelayUntil(uint32_t ticks){ fake_tick = ticks; if(task_exit_after_delay_until){ task_exit_after_delay_until = 0; longjmp(task_exit_jmp, 1); } return osOK; }
BaseType_t xTaskCreate(TaskFunction_t fn, const char * const name, const configSTACK_DEPTH_TYPE stack, void * const arg, UBaseType_t prio, TaskHandle_t * const handle){ (void)fn;(void)name;(void)stack;(void)arg;(void)prio; if(handle) *handle=(TaskHandle_t)0x1; return pdPASS; }
void vTaskDelete(TaskHandle_t handle){ (void)handle; }

void vPortEnterCritical(void){}
void vPortExitCritical(void){}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan){ return hcan ? fake_can_recover_status : HAL_ERROR; }
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan){ return hcan ? fake_can_recover_status : HAL_ERROR; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan){ if(!hcan) return HAL_ERROR; if(fake_can_recover_status == HAL_OK) fake_can_error = HAL_CAN_ERROR_NONE; return fake_can_recover_status; }
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *hcan){ (void)hcan; return fake_can_error; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t notif){ (void)notif; return hcan ? HAL_OK : HAL_ERROR; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *hcan){ (void)hcan; return tx_free_level; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *hdr, const uint8_t *data, uint32_t *mailbox){
    if(!hcan || !hdr || !data || tx_count >= AMS_HOST_TX_LOG_CAPACITY) return HAL_ERROR;
    if(fake_can_add_tx_status != HAL_OK) return fake_can_add_tx_status;
    tx_log[tx_count].ide = hdr->IDE; tx_log[tx_count].stdid = hdr->StdId; tx_log[tx_count].extid = hdr->ExtId; tx_log[tx_count].dlc = hdr->DLC;
    memcpy(tx_log[tx_count].data, data, 8); if(mailbox) *mailbox=0; tx_count++; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo, CAN_RxHeaderTypeDef *hdr, uint8_t data[]){
    (void)fifo; if(!hcan || !hdr || !data) return HAL_ERROR; *hdr = fake_rx_hdr; memcpy(data, fake_rx_data, 8); return fake_rx_status;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel){ (void)channel; return htim ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim){ return htim ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t channel){ (void)channel; return htim ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t channel){ (void)channel; return htim ? HAL_OK : HAL_ERROR; }
uint32_t HAL_TIM_ReadCapturedValue(const TIM_HandleTypeDef *htim, uint32_t channel){ (void)htim; return channel == TIM_CHANNEL_1 ? 1000u : 500u; }
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
void adbms_spi_lock(void){}
void adbms_spi_unlock(void){}

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
    if((d == NULL) || (id >= AMS_HEARTBEAT_COUNT)) return;
    d->heartbeat.last_tick[id] = now;
    d->heartbeat.count[id]++;
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
static HAL_StatusTypeDef fake_adbms_wrcfgb_status = HAL_OK;
static HAL_StatusTypeDef fake_adbms_wrpwm_status = HAL_OK;
static int fake_adbms_wrpwm_fail_after_ok = -1;
void adBms6830_init(adbms6830_driver_t* dev, uint8_t num_ics, adbms6830_asic* ics, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port_a, GPIO_TypeDef* cs_port_b, uint16_t cs_pin_a, uint16_t cs_pin_b, TIM_HandleTypeDef *htim){ if(dev){ dev->num_ics=num_ics; dev->ics=ics; dev->hspi=hspi; dev->cs_port[0]=cs_port_a; dev->cs_port[1]=cs_port_b; dev->cs_pin[0]=cs_pin_a; dev->cs_pin[1]=cs_pin_b; dev->htim=htim; dev->string=STRING_B; memset(&dev->spi_debug, 0, sizeof(dev->spi_debug)); memset(&dev->health, 0, sizeof(dev->health)); dev->spi_debug.last_status=HAL_OK; dev->spi_debug.last_tx_status=HAL_OK; dev->spi_debug.last_rx_status=HAL_OK; dev->spi_debug.last_xfer_status=HAL_OK; dev->health.last_status=HAL_OK; for(uint8_t ic=0; ic<ADBMS6830_MAX_TRACKED_ICS; ic++){ dev->last_cell_updated_mask[ic]=0u; dev->last_cell_pec_mask[ic]=0u; dev->last_temp_updated_mask[ic]=0u; } } }
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
void adbms6830_reset_cfg(adbms6830_driver_t *dev){(void)dev;} void adbms6830_srst(adbms6830_driver_t *dev){(void)dev;} void adbms6830_wrcfga(adbms6830_driver_t *dev){(void)dev;} void adbms6830_wrcfgb(adbms6830_driver_t *dev){(void)adbms6830_wrcfgb_checked(dev);} HAL_StatusTypeDef adbms6830_wrcfgb_checked(adbms6830_driver_t *dev){(void)dev; return fake_adbms_wrcfgb_status;} HAL_StatusTypeDef adbms6830_wrpwma_checked(adbms6830_driver_t *dev){(void)dev; return fake_adbms_wrpwm_next_status();} HAL_StatusTypeDef adbms6830_wrpwmb_checked(adbms6830_driver_t *dev){(void)dev; return fake_adbms_wrpwm_next_status();} HAL_StatusTypeDef adbms6830_write_pwm_checked(adbms6830_driver_t *dev){(void)dev; return fake_adbms_wrpwm_next_status();} void adbms6830_rdcfga(adbms6830_driver_t *dev){(void)dev;} void adbms6830_rdcfgb(adbms6830_driver_t *dev){(void)dev;}
void adbms6830_adcv(adbms6830_driver_t *dev, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs){(void)dev;(void)rd;(void)cont;(void)dcp;(void)rstf;(void)owcs;} void adbms6830_wakeup(adbms6830_driver_t* dev){(void)dev;} void adbms6830_us_delay(adbms6830_driver_t* dev, uint16_t microseconds){(void)dev;(void)microseconds;} void adbms6830_start_adc_cell_voltage_measurement(adbms6830_driver_t *dev){(void)dev;} void adbms6830_parse_cell(adbms6830_driver_t *dev, uint8_t *data, GRP grp){(void)dev;(void)data;(void)grp;}
void adbms6830_wakeup_cold(adbms6830_driver_t* dev){ if(dev){ dev->spi_debug.last_op = ADBMS6830_SPI_OP_COLD_WAKE; } }
void adbms6830_read_cell_voltages(adbms6830_driver_t *dev){
    if(dev){
        for(uint8_t ic=0; ic<ADBMS6830_MAX_TRACKED_ICS; ic++){
            dev->last_cell_updated_mask[ic]=0u;
            dev->last_cell_pec_mask[ic]=0u;
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
    }
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
        case ADBMS6830_SPI_OP_CLEAR_FLAGS: return "clear_flags";
        case ADBMS6830_SPI_OP_CONFIG_CHECK: return "config_check";
        case ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST: return "cell_adc_diag";
        case ADBMS6830_SPI_OP_OPEN_WIRE_EVEN: return "open_wire_even";
        case ADBMS6830_SPI_OP_OPEN_WIRE_ODD: return "open_wire_odd";
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
    for(uint8_t ic = 0u; (dev->ics != NULL) && (ic < (uint8_t)dev->num_ics) && (ic < ADBMS6830_MAX_TRACKED_ICS); ic++){
        for(uint8_t b = 0u; b < RSID; b++){
            dev->diag[ic].sid[b] = (uint8_t)(0x10u + (ic * 0x10u) + b);
            dev->ics[ic].sid.sid[b] = dev->diag[ic].sid[b];
        }
        dev->diag[ic].sid_valid = true;
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
HAL_StatusTypeDef adbms6830_clear_all_flags(adbms6830_driver_t *dev){
    if(dev == NULL) return HAL_ERROR;
    dev->spi_debug.last_op = ADBMS6830_SPI_OP_CLEAR_FLAGS;
    dev->spi_debug.last_status = HAL_OK;
    dev->spi_debug.tx_count++;
    return HAL_OK;
}
const adbms6830_diag_health_t *adbms6830_diag_health_get(const adbms6830_driver_t *dev){ return dev ? &dev->health : NULL; }
void adbms6830_diag_health_clear(adbms6830_driver_t *dev){ if(dev){ memset(&dev->health, 0, sizeof(dev->health)); dev->health.last_status = HAL_OK; } }
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
HAL_StatusTypeDef adbms6830_run_open_wire_check(adbms6830_driver_t *dev, bool odd_channels){
    if(dev == NULL) return HAL_ERROR;
    dev->health.last_op = odd_channels ? ADBMS6830_SPI_OP_OPEN_WIRE_ODD : ADBMS6830_SPI_OP_OPEN_WIRE_EVEN;
    dev->health.last_status = fake_adbms_diag_status;
    dev->spi_debug.last_op = dev->health.last_op;
    dev->spi_debug.last_status = fake_adbms_diag_status;
    if(odd_channels) dev->health.open_wire_odd_count++; else dev->health.open_wire_even_count++;
    return fake_adbms_diag_status;
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
int adbms6830_read_temp_raw(adbms6830_driver_t *dev, uint8_t ic_idx, int16_t *out_raw){ (void)dev;(void)ic_idx; if(out_raw) *out_raw=0; return 0; }
float adbms6830_convert_temp(adbms6830_driver_t *dev, uint8_t ic_idx, uint8_t sensor_num, float vref){ (void)dev;(void)ic_idx;(void)sensor_num;(void)vref; return 25.0f; }
float voltage_to_temp(float raw){ float voltage = ((float)raw + 10000.0f) * 0.000150f; if(voltage <= 0.0f || voltage >= 5.0f) return NAN; float resistance = 10000.0f * (5.0f - voltage) / voltage; float x = logf(resistance / 10000.0f); return (1.0f / (3.354016435e-3f + 2.565235509e-4f * x)) - 273.15f; }
int mux_set_channel(adbms6830_driver_t *dev, uint8_t sensor_num){ (void)dev; return sensor_num < 24 ? 0 : -1; }

void adbms2950_gpo_set(adbms2950_driver_t *dev, GPO gp, CFGA_GPO state){(void)dev;(void)gp;(void)state;} void adbms2950_wakeup(adbms2950_driver_t *dev){(void)dev;} void adbms2950_wrcfga(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdcfga(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdvb(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdi(adbms2950_driver_t *dev){(void)dev;} void adbms2950_adv(adbms2950_driver_t *dev, adv_ *adv){(void)dev;(void)adv;} void adbms2950_plv(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdv1d(adbms2950_driver_t *dev){(void)dev;}
void adbms2950_spi_debug_enable(adbms2950_driver_t *dev, bool enable){ if(dev) dev->spi_debug.enabled = enable; }
void adbms2950_spi_debug_clear(adbms2950_driver_t *dev){ if(dev){ bool en = dev->spi_debug.enabled; memset(&dev->spi_debug, 0, sizeof(dev->spi_debug)); dev->spi_debug.enabled = en; } }
const adbms2950_spi_debug_t *adbms2950_spi_debug_get(const adbms2950_driver_t *dev){ return dev ? &dev->spi_debug : NULL; }
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
#include "Core/Src/tasks/estimator_task.c"

static uint16_t word_at(uint32_t frame, uint8_t word_index){ return ((uint16_t)tx_log[frame].data[word_index*2] << 8) | tx_log[frame].data[word_index*2+1]; }
static int16_t code_for_volts(float v){ return (int16_t)((v / 0.000150f) - 10000.0f); }
static int16_t raw_for_ntc_voltage(float v){ return (int16_t)((v / 0.000150f) - 10000.0f); }
static float ntc_voltage_for_temp_c(float temp_c){
    const float A = 3.354016435e-3f;
    const float B = 2.565235509e-4f;
    float t_k = temp_c + 273.15f;
    float r = 10000.0f * expf(((1.0f / t_k) - A) / B);
    return 50000.0f / (r + 10000.0f);
}
static int16_t raw_for_temp_c(float temp_c){ return raw_for_ntc_voltage(ntc_voltage_for_temp_c(temp_c)); }
#define CHECK(cond) do{ if(!(cond)){ fprintf(stderr,"FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); exit(1);} }while(0)
#define HOST_LOGGER_FRAME_COUNT 120u
#define HOST_ECU_FRAME_COUNT 62u
#define HOST_ECU_COMPACT_FRAME_COUNT 4u
#define HOST_LEGACY_ECU_FRAME_OFFSET HOST_ECU_COMPACT_FRAME_COUNT
#define HOST_NONCHARGE_CAN_FRAME_COUNT (HOST_ECU_COMPACT_FRAME_COUNT + HOST_ECU_FRAME_COUNT + HOST_LOGGER_FRAME_COUNT)
#define HOST_CHARGE_CAN_FRAME_COUNT (HOST_ECU_COMPACT_FRAME_COUNT + HOST_LOGGER_FRAME_COUNT + 1u)
#define HOST_CHARGER_FRAME_INDEX HOST_ECU_COMPACT_FRAME_COUNT

static void sil_mark_all_heartbeats_alive(app_data_t *d);

static void init_fake_app(void){ memset(&app,0,sizeof(app)); ams_safety_host_reset_state(); ams_rtos_host_reset_state(); ams_rtos_diag_init(&app); app.acc.smb.num_ics = NSMBS; app.acc.smb.ics = app.acc.smb_ics; current_fault_init(&app.current_fault_state); voltage_fault_init(&app.voltage_fault_state); temperature_fault_init(&app.temp_fault_state); ams_heartbeat_init(&app, fake_tick); app.current_meas_reason = CURRENT_SENSOR_REASON_ADC_READ; app.current_fault_reason = CURRENT_FAULT_REASON_SENSOR_NOT_READY; app.voltage_fault_reason = VOLTAGE_FAULT_REASON_NOT_READY; app.temp_fault = true; app.temp_read_fault = true; app.temp_fan_max = true; app.temp_fault_reason = TEMPERATURE_FAULT_REASON_NOT_READY; app.imd_valid = true; app.imd_ok = true; app.imd_fault = false; app.imd_status = IMD_NORMAL; app.balance_inhibit = (AMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT != 0); fake_adbms_voltage_masks_full_update(); fake_adc_read_index = 0u; fake_adbms_wrcfgb_status = HAL_OK; fake_adbms_wrpwm_status = HAL_OK; fake_adbms_wrpwm_fail_after_ok = -1; fake_adbms_diag_status = HAL_OK; fake_adbms_config_mismatch_mask = 0u; fake_can_add_tx_status = HAL_OK; fake_can_error = HAL_CAN_ERROR_NONE; fake_can_recover_status = HAL_OK; bms_pin_state = GPIO_PIN_RESET; }

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
    HAL_CAN_RxFifo0MsgPendingCallback(&hil_fake_hcan);
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
    HAL_CAN_RxFifo0MsgPendingCallback(&hil_fake_hcan);
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
    fake_adbms_wrpwm_status = HAL_ERROR;
    CHECK(accumulator_set_balance(&app.acc) == -1);
    fake_adbms_wrpwm_status = HAL_OK;
    fake_adbms_wrcfgb_status = HAL_ERROR;
    CHECK(accumulator_clear_balance(&app.acc) == -1);
    fake_adbms_wrcfgb_status = HAL_OK;
    CHECK(accumulator_clear_balance(&app.acc) == 0);
    sil_expect_balancing_clear(&app);

    app.acc.smb.num_ics = 99; // bad config should still stay bounded
    accumulator_update_voltage_stats(&app.acc);
    CHECK(app.acc.max_volt > 4.0f && app.acc.min_volt > 3.0f);
}

static void test_temp_stats(void){
    init_fake_app();
    for(int ic=0; ic<NSMBS; ic++) for(int s=0;s<NTEMPS;s++) app.acc.smb_ics[ic].temp.raw[s] = raw_for_ntc_voltage(2.5f);
    app.acc.smb_ics[1].temp.raw[2] = 0; // invalid skip
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
    app.acc.smb_ics[2].temp.raw[5] = 0;
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
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
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

static void sil_publish_temp_state(app_data_t *d)
{
    CHECK(d != NULL);
    temperature_fault_update(&d->temp_fault_state, &d->acc);
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

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state = STATE_DISCARGE; app.current_valid=true; app.bms_state=false; bms_pin_state=GPIO_PIN_RESET; fake_tick=0;
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

    init_fake_app(); for(int i=0;i<NFANS;i++){ static TIM_HandleTypeDef ht; static uint32_t ccrs[NFANS]; app.board.fans[i].CCR=&ccrs[i]; app.board.fans[i].max_timer_val=1000; app.board.fans[i].htim=&ht; }
    app.max_temp = 0.0f; app.temp_valid = false; app.temp_usable_sensor_count = 0u; run_one_fan_task_iteration(&app); CHECK(app.fan_state == true); for(int i=0;i<NFANS;i++) CHECK(app.board.fans[i].duty_cycle == 100.0f);
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

static void test_fan_current_and_null_guards(void){
    fan_t fan={0}; uint32_t ccr=999; static TIM_HandleTypeDef htim; CHECK(fan_init(NULL,NULL,NULL,0,NULL,1) != 0); CHECK(fan_init(&fan,NULL,&htim,1000,&ccr,1)==0); CHECK(ccr==0u); CHECK(set_fan_percent(&fan,120.0f)==0 && ccr==1000u && fabsf(fan.duty_cycle-100.0f)<0.01f); CHECK(set_fan_percent(&fan,-10.0f)==0 && ccr==0u && fabsf(fan.duty_cycle)<0.01f);
    CHECK(current_sensor_convert(NULL) == 0.0f);
    CHECK(accumulator_set_balance(NULL) == -1); CHECK(accumulator_clear_balance(NULL) == -1); accumulator_update_voltage_stats(NULL); accumulator_update_temp_stats(NULL);
    CHECK(imd_read(NULL) == 1); imd_init(NULL, 0, NULL, NULL, 0, 0, NULL, 0);
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
    d->heartbeat.boot_tick = 0u;
    sil_mark_all_heartbeats_alive(d);
    (void)ams_heartbeat_update(d, fake_tick);
}

static void test_watchdog_feed_gate(void)
{
    init_fake_app();
    fake_tick = AMS_HEARTBEAT_STARTUP_GRACE_MS + 100u;
    ams_safety_watchdog_enable_runtime(&app, true);
    ams_safety_watchdog_task_update(&app);

#if AMS_ENABLE_IWDG
    CHECK(app.watchdog_runtime_enabled == true);
    CHECK(app.watchdog_hw_started == false);
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
    CHECK(app.can_last_error_tick > first_recovery_tick);
    uint32_t failed_recovery_tick = app.can_last_error_tick;
    fake_tick += (AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS / 2u);
    canbus_poll_errors(&app.board.canbus, &app);
    CHECK(app.can_last_error_tick == failed_recovery_tick);

    fake_can_recover_status = HAL_OK;
    fake_tick = failed_recovery_tick + AMS_CAN_BUSOFF_RECOVERY_COOLDOWN_MS + 1u;
    canbus_poll_errors(&app.board.canbus, &app);
    CHECK(app.can_busoff_fault == false);
    CHECK(app.can_recover_pending == false);
    CHECK(app.can_recover_count == 1u);
    CHECK(app.canbus_fault == false);

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

    // 3) High temperature must hard-disable BMS and prevent balancing.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); sil_set_all_temps(&app, 66.0f, (1UL << NTEMPS) - 1UL); app.state=STATE_CHARGE; app.current_valid=true; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
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
    init_fake_app(); sil_make_measurement_gates_ready(&app); app.temp_usable_sensor_count = AMS_EXPECTED_TEMP_SENSOR_COUNT; app.max_temp = TEMP_FAN_MAX_C + 5.0f; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
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
    CHECK(app.acc.valid_voltage_count == 15u);
    CHECK(app.acc.min_volt > 3.49f && app.acc.max_volt < 3.52f);

    init_fake_app();
    app.acc.smb.num_ics = 255;
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
            if(code != 0 && code != INT16_MIN && v >= 0.5f && v <= 5.0f){
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
}


static void test_voltage_fault_policy_and_strict_scan_freshness(void){
    init_fake_app();
    fake_tick = 0u;
    fill_nominal_pack(&app, 3.700f);
    voltage_fault_update(&app.voltage_fault_state, &app.acc);
    CHECK(app.voltage_fault_state.voltage_valid == true);
    CHECK(app.voltage_fault_state.read_fault == false);
    CHECK(app.voltage_fault_state.reason == VOLTAGE_FAULT_REASON_NONE);

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
    sil_prepare_cli_capture();
    CHECK(balance_control(2, balance_clear_argv) == 0);
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
        CHECK(app.bms_state == true);
        CHECK(bms_pin_state == GPIO_PIN_SET);
    }
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
    CHECK(app.adbms_status_fault == true);
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
    CHECK(app.bms_state == true);
    CHECK(bms_pin_state == GPIO_PIN_SET);

    sil_prepare_ready_system(STATE_CHARGE, 0.0f, 3.700f);
    app.balance_inhibit = false;
    sil_set_cell_voltage(&app, 0u, 0u, 4.100f);
    sil_set_cell_voltage(&app, 0u, 1u, 4.180f);
    fake_adbms_voltage_masks_full_update();
    fake_adbms_wrpwm_fail_after_ok = 1;
    sil_run_voltage_sample(&app);
    CHECK(app.bms_state == true);
    CHECK(app.charge_voltage_stop == true);
    CHECK(sil_balance_pwm_duty(&app, 0u, 1u) == BALANCE_PWM_DUTY);

    app.state = STATE_DISCARGE;
    fake_adbms_wrpwm_fail_after_ok = -1;
    sil_run_voltage_sample(&app);
    sil_expect_balancing_clear(&app);
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

static void test_system_sil_2950_debug_non_safety_until_integrated(void)
{
    sil_prepare_ready_system(STATE_DISCARGE, 0.0f, 3.700f);

    app.acc.apm.spi_debug.enabled = true;
    app.acc.apm.spi_debug.error_count = 1234u;
    app.acc.apm.spi_debug.last_status = HAL_TIMEOUT;
    app.acc.apm.spi_debug.last_xfer_status = HAL_TIMEOUT;

    fake_adbms_voltage_masks_full_update();
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == true);
    CHECK(app.voltage_fault == false);
    CHECK(app.current_valid == true);
    CHECK(app.current_fault == false);
    CHECK(app.bms_state == true);
    CHECK(app.acc.apm.spi_debug.error_count == 1234u);
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
    CHECK(app.bms_state == true);

    fake_adbms_voltage_masks_one_missing(0u, 4u, true);
    sil_run_voltage_sample(&app);
    CHECK(app.voltage_valid == false);
    CHECK(app.voltage_read_fault == true);
    CHECK(app.voltage_warning == false);
    CHECK(app.voltage_fault == true);
    CHECK(app.voltage_fault_reason == VOLTAGE_FAULT_REASON_PEC_FAILURE);
    CHECK(app.bms_state == false);

    sil_run_current_sample(&app, 2.5f);
    sil_run_current_sample(&app, 2.5f);
    CHECK(app.current_fault == true);
    CHECK(app.current_fault_latched == true);
    CHECK(app.current_fault_latched_reason == CURRENT_FAULT_REASON_PRECHARGE_FAST_OVERCURRENT);
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

static void test_adbms6830_diagnostic_commands_and_cli_health(void)
{
    static SPI_HandleTypeDef hspi;
    char *cfgchk[] = {"spi", "cfgchk"};
    char *cellst[] = {"spi", "cellst"};
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
    app.acc.smb.num_ics = NSMBS;
    app.acc.smb.ics = app.acc.smb_ics;
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
    get_spi_debug(2, cfgchk);
    CHECK(strstr(cli_capture, "CFGA/CFGB readback check status: OK") != NULL);
    CHECK(strstr(cli_capture, "diag op:config_check") != NULL);
    CHECK(strstr(cli_capture, "cfgB:0x0002 cfg:0x0002") != NULL);
    CHECK(strstr(cli_capture, "IC1 health") != NULL);
    CHECK(app.acc.smb.health.config_readback_count == 1u);
    CHECK(app.acc.smb.health.config_mismatch_count[1] == 1u);

    sil_prepare_cli_capture();
    get_spi_debug(2, cellst);
    CHECK(strstr(cli_capture, "Cell ADC diagnostic hook status: OK") != NULL);
    CHECK(strstr(cli_capture, "diag op:cell_adc_diag") != NULL);
    CHECK(app.acc.smb.health.cell_adc_self_test_count == 1u);

    app.state = STATE_CHARGE;
    sil_prepare_cli_capture();
    get_spi_debug(2, oweven);
    CHECK(strstr(cli_capture, "Open-wire even-channel command status: OK") != NULL);
    CHECK(strstr(cli_capture, "diag op:open_wire_even") != NULL);
    CHECK(app.acc.smb.health.open_wire_even_count == 1u);

    sil_prepare_cli_capture();
    get_spi_debug(2, owodd);
    CHECK(strstr(cli_capture, "Open-wire odd-channel command status: OK") != NULL);
    CHECK(strstr(cli_capture, "diag op:open_wire_odd") != NULL);
    CHECK(app.acc.smb.health.open_wire_odd_count == 1u);

    sil_prepare_cli_capture();
    get_spi_debug(2, auxdiag);
    CHECK(strstr(cli_capture, "AUX/GPIO diagnostic hook status: OK") != NULL);
    CHECK(strstr(cli_capture, "diag op:aux_gpio_diag") != NULL);
    CHECK(app.acc.smb.health.aux_gpio_diag_count == 1u);

    sil_prepare_cli_capture();
    get_spi_debug(2, diagclear);
    CHECK(strstr(cli_capture, "diagnostic health counters cleared") != NULL);
    CHECK(app.acc.smb.health.config_readback_count == 0u);
    CHECK(app.acc.smb.health.sticky_pec_fail_mask == 0u);
    CHECK(app.acc.smb.health.last_status == HAL_OK);

    fake_adbms_diag_status = HAL_ERROR;
    sil_prepare_cli_capture();
    get_spi_debug(2, cellst);
    CHECK(strstr(cli_capture, "Cell ADC diagnostic hook status: ERROR") != NULL);
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

    for(uint8_t i = 0u; i < 9u; i++)
    {
        run_one_adbms_task_iteration(&app);
    }
    CHECK(app.adbms_status_diag_count == 0u);
    CHECK(app.adbms_config_diag_count == 0u);
    CHECK(app.adbms_open_wire_diag_count == 0u);
    CHECK(app.adbms_diag_fault == false);

    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_status_diag_count == 1u);
    CHECK(app.adbms_config_diag_count == 0u);
    CHECK(app.adbms_open_wire_diag_count == 0u);
    CHECK(app.adbms_diag_fault == false);

    for(uint8_t i = 0u; i < 49u; i++)
    {
        run_one_adbms_task_iteration(&app);
    }
    CHECK(app.adbms_scan_count == 59u);
    CHECK(app.adbms_config_diag_count == 0u);

    fake_adbms_config_mismatch_mask = 0x0004u;
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
    app.adbms_scan_count = ADBMS_OPEN_WIRE_DIAG_PERIOD_CYCLES - 1u;
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_open_wire_diag_count == 1u);
    CHECK(app.acc.smb.health.open_wire_even_count == 1u);
    CHECK(app.acc.smb.health.open_wire_odd_count == 0u);
    CHECK(app.adbms_open_wire_fault == false);
}

static void test_adbms_cli_scan_guard_and_cs_probe_commands(void)
{
    static SPI_HandleTypeDef hspi;
    static GPIO_TypeDef cs_a;
    static GPIO_TypeDef cs_b;
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
    CHECK(app.acc.smb.string == STRING_B);
    app.acc.smb.hspi = &hspi;
    app.acc.smb.num_ics = NSMBS;
    app.acc.smb.ics = app.acc.smb_ics;
    app.acc.smb.string = STRING_B;

    app.adbms_scan_active = true;
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, probe) == 0);
    CHECK(strstr(cli_capture, "spi probe refused") != NULL);
    CHECK(app.acc.smb.spi_debug.rx_count == 0u);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(5, scope) == 0);
    CHECK(strstr(cli_capture, "spi scope refused") != NULL);
    CHECK(app.acc.smb.spi_debug.rx_count == 0u);

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
    CHECK(app.acc.smb.string == STRING_B);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, probeb) == 0);
    CHECK(strstr(cli_capture, "CS_B/stringB probe status: OK") != NULL);
    CHECK(app.acc.smb.spi_debug.last_string == STRING_B);
    CHECK(app.acc.smb.string == STRING_B);

    adbms6830_spi_debug_clear(&app.acc.smb);
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(5, scope) == 0);
    CHECK(strstr(cli_capture, "scope string:CS_B mode:read repeat:20 status:OK") != NULL);
    CHECK(strstr(cli_capture, "Probe MCU: SCK PG13") != NULL);
    CHECK(app.acc.smb.spi_debug.last_op == ADBMS6830_SPI_OP_SCOPE);
    CHECK(app.acc.smb.spi_debug.last_string == STRING_B);
    CHECK(app.acc.smb.spi_debug.rx_count == 20u);
    CHECK(app.acc.smb.string == STRING_B);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(3, preset_status) == 0);
    CHECK(strstr(cli_capture, "scope preset string:CS_B mode:read repeat:20") != NULL);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(3, preset_cmd) == 0);
    CHECK(strstr(cli_capture, "scope preset string:CS_B mode:cmd repeat:50") != NULL);

    adbms6830_spi_debug_clear(&app.acc.smb);
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, scope_default) == 0);
    CHECK(strstr(cli_capture, "scope string:CS_B mode:cmd repeat:50 status:OK") != NULL);
    CHECK(app.acc.smb.spi_debug.last_op == ADBMS6830_SPI_OP_SCOPE);
    CHECK(app.acc.smb.spi_debug.last_string == STRING_B);
    CHECK(app.acc.smb.spi_debug.tx_count == 50u);
    CHECK(app.acc.smb.spi_debug.rx_count == 0u);

    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, preset_toggle) == 0);
    CHECK(strstr(cli_capture, "scope preset string:CS_B mode:pattern repeat:20") != NULL);

    app.state = STATE_DISCARGE;
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, oweven) == 0);
    CHECK(strstr(cli_capture, "Open-wire refused") != NULL);
    CHECK(app.acc.smb.health.open_wire_even_count == 0u);

    app.state = STATE_CHARGE;
    sil_prepare_cli_capture();
    CHECK(get_spi_debug(2, oweven) == 0);
    CHECK(strstr(cli_capture, "Open-wire even-channel command status: OK") != NULL);
    CHECK(app.acc.smb.health.open_wire_even_count == 1u);

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

    init_fake_app();
    fake_tick = 5000u;
    ams_heartbeat_init(&app, fake_tick);
    sil_make_measurement_gates_ready(&app);
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
    fake_tick += AMS_HEARTBEAT_LOGGER_TIMEOUT_MS + 1u;
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_ADBMS, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CURRENT, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_TEMP, fake_tick);
    ams_heartbeat_kick(&app, AMS_HEARTBEAT_CAN, fake_tick);

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
    sil_prepare_cli_capture();
    CHECK(get_bringup(2, apm_argv) == 0);
    CHECK(strstr(cli_capture, "BRINGUP APM2950") != NULL);
    CHECK(strstr(cli_capture, "initialized:0") != NULL);
    CHECK(strstr(cli_capture, "DEBUG_ONLY_NON_GATING") != NULL);

    hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    app.acc.apm.hspi = &hspi;
    app.acc.apm.num_ics = 1u;
    app.acc.apm.spi_debug.rx_count = 1u;
    app.acc.apm.spi_debug.last_status = HAL_OK;
    memset(app.acc.apm.spi_debug.last_rx_preview, 0xFF, sizeof(app.acc.apm.spi_debug.last_rx_preview));

    sil_prepare_cli_capture();
    CHECK(get_bringup(2, apm_argv) == 0);
    CHECK(strstr(cli_capture, "initialized:1") != NULL);
    CHECK(strstr(cli_capture, "mode=PASS") != NULL);
    CHECK(strstr(cli_capture, "response=FAIL all_ff") != NULL);
    CHECK(strstr(cli_capture, "scaling=UNPROVEN") != NULL);

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
    for(int ic=0; ic<NSMBS; ic++) for(int s=0; s<NTEMPS; s++) app.acc.smb_ics[ic].temp.raw[s] = 0;
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
    app.acc.smb_ics[2].temp.raw[5] = 0;
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
    run_one_adbms_task_iteration(&app);
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
    run_one_adbms_task_iteration(&app);
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
    app.acc.smb_ics[2].temp.raw[5] = 0;
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

    fake_rx_status = HAL_ERROR;
    fake_rx_hdr.IDE = CAN_ID_EXT; fake_rx_hdr.ExtId = CHARGER_RX_ID; fake_rx_hdr.DLC = 8;
    memset(fake_rx_data, 0xAA, sizeof(fake_rx_data));
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(app.board.charger.rx_count == 0u);
    CHECK(app.board.charger.flags == 0u);

    fake_rx_status = HAL_OK;
    fake_rx_hdr.IDE = CAN_ID_STD; fake_rx_hdr.StdId = (uint32_t)CHARGER_RX_ID; fake_rx_hdr.DLC = 8;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(app.board.canbus.rx_packet.id == (uint32_t)CHARGER_RX_ID);
    CHECK(app.board.charger.rx_count == 0u);

    fake_rx_hdr.IDE = CAN_ID_EXT; fake_rx_hdr.ExtId = CHARGER_RX_ID ^ 0x10u; fake_rx_hdr.DLC = 8;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(app.board.canbus.rx_packet.id == (CHARGER_RX_ID ^ 0x10u));
    CHECK(app.board.charger.rx_count == 0u);

    fake_rx_hdr.IDE = CAN_ID_EXT; fake_rx_hdr.ExtId = CHARGER_RX_ID; fake_rx_hdr.DLC = 4;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(app.board.charger.rx_count == 0u);

    fake_rx_hdr.DLC = 8;
    fake_rx_data[0] = 0x0C; fake_rx_data[1] = 0x30; // 312.0 V
    fake_rx_data[2] = 0x00; fake_rx_data[3] = 0x0A; // 1.0 A
    fake_rx_data[4] = 0x0Fu;
    fake_tick = 7777u;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(app.board.charger.rx_count == 1u);
    CHECK(fabsf(app.board.charger.read_voltage - 312.0f) < 0.01f);
    CHECK(fabsf(app.board.charger.read_current - 1.0f) < 0.01f);
    CHECK(app.board.charger.hardware_fail && app.board.charger.overtemp_fail && app.board.charger.input_volt_fail && app.board.charger.voltage_sense_fail);
    CHECK(app.board.charger.communication_fail == false);
    CHECK(app.board.charger.last_rx_tick == 7777u);
}



static float estimator_expected_voltage(const ams_ekf_config_t *cfg, float soc, float current_A, float temp_C, float r0_ohm){
    float i_cell = current_A / cfg->parallel_cell_count;
    return (float)cfg->series_group_count * (ams_p42a_ocv_v(soc, temp_C) - (r0_ohm * i_cell));
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

    CHECK(ams_estimator_configure_segments(&est));
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
    CHECK(est.pack_r0_ohm > 0.010f && est.pack_r0_ohm < 0.020f);
    CHECK(fabsf(est.pack_v_pred_V - 277.5f) < 0.5f);
    CHECK(est.pack_innovation_V > 4.0f && est.pack_innovation_V < 5.0f);
}

static void test_hil_parser_edge_cases(void){
    static CAN_HandleTypeDef hcan;
    init_fake_app(); app.board.canbus.hcan = &hcan;
    memset(&fake_rx_hdr, 0, sizeof(fake_rx_hdr));
    memset(fake_rx_data, 0xFF, sizeof(fake_rx_data));

    fake_rx_hdr.IDE = CAN_ID_EXT; fake_rx_hdr.ExtId = AMS_HIL_CAN_ID_MEAS; fake_rx_hdr.DLC = 8;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(app.hil.meas.fresh == 0u);

    fake_rx_hdr.IDE = CAN_ID_STD; fake_rx_hdr.StdId = AMS_HIL_CAN_ID_MEAS; fake_rx_hdr.DLC = 6;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(app.hil.meas.fresh == 0u);

    fake_rx_hdr.DLC = 8;
    fake_rx_data[0]=0x7B; fake_rx_data[1]=0x00; /* 314.88 V at 10 mV/count */
    fake_rx_data[2]=0xFF; fake_rx_data[3]=0x38; /* -2.00 A at 10 mA/count */
    fake_rx_data[4]=0x09; fake_rx_data[5]=0xC4; /* 25.00 C */
    fake_rx_data[6]=3u; fake_tick=44u;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
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
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(app.hil.summary.fresh == 1u);
    CHECK(app.hil.summary.last_rx_tick == 55u);
    CHECK(fabsf(app.hil.summary.v_min_V - 3.600f) < 0.002f);
    CHECK(fabsf(app.hil.summary.v_max_V - 4.100f) < 0.002f);
    CHECK(fabsf(app.hil.summary.t_max_C - 26.0f) < 0.02f);
}

static void test_estimator_task_hil_and_hardware_paths(void){
    static CAN_HandleTypeDef hcan;
    init_fake_app(); fill_nominal_pack(&app, 3.90f); app.board.canbus.hcan = &hcan;
    app.current = 0.0f; fake_tick = 100u;
    run_one_estimator_task_iteration(&app);
    CHECK(app.estimator.input_source == AMS_ESTIMATOR_INPUT_HARDWARE);
    CHECK(app.estimator.inst[0].valid == 1u);
    CHECK(app.estimator_fault == false);
    CHECK(app.estimator.pack_soc >= 0.0f && app.estimator.pack_soc <= 1.0f);

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
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
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
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
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
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
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

    run_one_adbms_task_iteration(&app);

    CHECK(app.voltage_valid == true);
    CHECK(app.voltage_fault == false);
    CHECK(app.temp_valid == true);
    CHECK(app.temp_fault == false);
    CHECK(app.adbms_diag_fault == false);
    CHECK(app.acc.valid_voltage_count == (uint16_t)(NSMBS * NCELLS));
    CHECK(app.acc.valid_temp_count == (uint16_t)(NSMBS * NTEMPS));

    app.can_busoff_fault = true;
    app.can_recover_pending = true;
    app.bms_state = true;
    bms_pin_state = GPIO_PIN_SET;
    run_one_adbms_task_iteration(&app);
    CHECK(app.adbms_diag_fault == true);
    CHECK(app.adbms_last_diag_status == HAL_ERROR);
    CHECK(app.bms_state == false);
    CHECK(bms_pin_state == GPIO_PIN_RESET);
#endif
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
    app.acc.smb_ics[1].temp.raw[3] = 0;
    host_mark_updated_cells(&app);
    accumulator_update_voltage_stats_at(&app.acc, fake_tick);
    host_mark_updated_temps(&app, (1UL << NTEMPS) - 1UL);
    accumulator_update_temp_stats_at(&app.acc, fake_tick);
    tx_count = 0; tx_free_level = 3;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == HOST_NONCHARGE_CAN_FRAME_COUNT);
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
    init_fake_app(); fill_nominal_pack(&app, 3.7f); app.board.canbus.hcan=&hcan; app.state=STATE_DISCARGE; fake_tick=123; tx_count=0; tx_free_level=3;
    run_one_canbus_task_iteration(&app); CHECK(fake_tick == 123u + CAN_ECU_FAST_PERIOD_MS);

    init_fake_app(); app.bms_state=true; fake_tick=100; run_one_error_task_iteration(&app); CHECK(fake_tick == 100u + (1000u / ERR_FREQ));

    fan_t fan={0}; uint32_t ccr=1234; static TIM_HandleTypeDef htim;
    CHECK(fan_init(&fan, NULL, &htim, 1000u, &ccr, 1) == 0);
    CHECK(set_fan_percent(&fan, NAN) == 0); CHECK(ccr == 0u && fan.duty_cycle == 0.0f);
    CHECK(set_fan_percent(&fan, INFINITY) == 0); CHECK(ccr == 0u && fan.duty_cycle == 0.0f);
    CHECK(set_fan_percent(&fan, -INFINITY) == 0); CHECK(ccr == 0u && fan.duty_cycle == 0.0f);

    canbus_device_t cb={0}; CHECK(canbus_send(NULL, CAN_ID_STD, ECU_CANBUS_ID, (uint8_t[8]){0}) == HAL_ERROR);
    CHECK(canbus_send(&cb, CAN_ID_STD, ECU_CANBUS_ID, (uint8_t[8]){0}) == HAL_ERROR);
    cb.hcan = &hcan; CHECK(canbus_send(&cb, CAN_ID_STD, ECU_CANBUS_ID, NULL) == HAL_ERROR);
}


#if AMS_HOST_PRODUCTION_GATE_TEST
static void test_production_safety_gates(void)
{
    static CAN_HandleTypeDef hcan;

    CHECK(AMS_ENABLE_HIL_CAN == 0);
    CHECK(AMS_ENABLE_SERVICE_CLI == 0);

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
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
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

int main(void){
#if AMS_HOST_PRODUCTION_GATE_TEST
    test_production_safety_gates();
    puts("PASS production HIL/service/IMD/supervisor gates");
    puts("ALL PRODUCTION SAFETY GATE TESTS PASSED");
    return 0;
#elif AMS_HOST_ONLY_HIL_ADBMS_TEST
    test_hil_adbms_image_replaces_raw_reads(); puts("PASS HIL ADBMS image replacement");
    puts("ALL HIL ADBMS REPLACEMENT TESTS PASSED");
    return 0;
#else
    test_accumulator_stats_and_balance(); puts("PASS accumulator stats/balance");
    test_voltage_stats_boundaries_and_fuzz(); puts("PASS voltage boundary/fuzz stats");
    test_voltage_fault_policy_and_strict_scan_freshness(); puts("PASS voltage fault strict scan freshness policy");
    test_system_sil_boot_ready_and_bms_conjunction(); puts("PASS system SIL boot/readiness/BMS conjunction");
    test_system_sil_single_pec_miss_drops_bms_then_recovers(); puts("PASS system SIL single PEC miss fail-closed/recovery");
    test_system_sil_persistent_voltage_stale_drops_bms_ok(); puts("PASS system SIL persistent voltage stale fail-closed");
    test_system_sil_charge_stop_allows_balance_before_hard_ov(); puts("PASS system SIL charge-stop balancing vs hard OV");
    test_system_sil_balance_inhibit_ladder_bringup_lockout(); puts("PASS system SIL balance inhibit ladder bring-up lockout");
    test_system_sil_bench_cli_abuse_and_balance_idempotence(); puts("PASS system SIL bench CLI abuse/balance idempotence");
    test_system_sil_bench_state_transition_balance_cleanup(); puts("PASS system SIL bench state-transition balance cleanup");
    test_system_sil_bench_bmsok_inhibit_survives_ready_tasks(); puts("PASS system SIL bench BMS_OK inhibit survives ready tasks");
    test_system_sil_bench_adbms_write_failures_and_recovery(); puts("PASS system SIL bench ADBMS write failures/recovery");
    test_system_sil_voltage_uv_ov_severe_diagnostics_and_latch(); puts("PASS system SIL voltage severe diagnostics/latch");
    test_system_sil_current_warning_fast_trip_and_latch_persistence(); puts("PASS system SIL current warning/fast-trip/latch");
    test_system_sil_current_stale_adc_pair_fails_safe(); puts("PASS system SIL current stale ADC pair fail-safe");
    test_system_sil_regen_and_charge_current_placeholders(); puts("PASS system SIL regen/charge current placeholder behavior");
    test_system_sil_2950_debug_non_safety_until_integrated(); puts("PASS system SIL 2950 debug non-safety behavior");
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
    test_adbms6830_diagnostic_commands_and_cli_health(); puts("PASS ADBMS6830 diagnostic commands/CLI health");
    test_adbms_periodic_diagnostics_and_safe_open_wire(); puts("PASS ADBMS6830 periodic diagnostics/safe open-wire");
    test_adbms_cli_scan_guard_and_cs_probe_commands(); puts("PASS ADBMS CLI scan guard/CS probe commands");
    test_software_heartbeat_monitor_faults_and_recovery(); puts("PASS software heartbeat monitor faults/recovery");
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
    test_logger_can_contract_packets(); puts("PASS logger CAN contract packetization");
    test_telemetry_absent_segments_and_invalid_channels(); puts("PASS telemetry absent segments/invalid channels");
    test_charger_rx_and_tx(); puts("PASS charger RX/TX parse");
    test_estimator_ra8m1_architecture_parity(); puts("PASS estimator RA8M1 architecture parity");
    test_estimator_lut_and_config_matrix(); puts("PASS estimator LUT/config matrix");
    test_estimator_step_faults_and_scalability(); puts("PASS estimator step faults/scalability");
    test_hil_parser_edge_cases(); puts("PASS HIL parser edge cases");
    test_hil_parser_and_estimator_core(); puts("PASS HIL parser and estimator core");
    test_hil_adbms_image_replaces_raw_reads(); puts("PASS HIL ADBMS image replacement");
    test_estimator_task_hil_and_hardware_paths(); puts("PASS estimator task HIL/hardware paths");
    test_estimator_status_packet_edges(); puts("PASS estimator status packet edges");
    test_can_rx_filter_matrix(); puts("PASS CAN RX filter matrix");
    test_charge_state_disable_matrix(); puts("PASS charge-state disable matrix");
    test_charger_command_priority_tx_failure_and_cli(); puts("PASS charger command priority/TX failure/CLI diagnostics");
    test_current_sensor_measurement_model(); puts("PASS current sensor measurement model");
    test_current_task_measurement_state(); puts("PASS current task measurement state");
    test_current_task_threshold_faults(); puts("PASS current task threshold faults");
    test_fan_current_and_null_guards(); puts("PASS fan/current/null guards");
    test_periods_and_driver_edge_cases(); puts("PASS periods and driver edge cases");
    test_task_iterations_with_injected_signals(); puts("PASS one-iteration task injection tests");
    test_safety_panic_reset_watchdog_and_log(); puts("PASS safety panic/reset/log path");
    test_watchdog_feed_gate(); puts("PASS watchdog feed gate");
    test_rtos_stack_heap_diag_and_faults(); puts("PASS RTOS stack/heap diagnostics");
    test_can_busoff_sets_fault_and_recovers(); puts("PASS CAN bus-off fault/recovery");
    test_fault_matrix_extra(); puts("PASS fault matrix extra");
    puts("ALL COMPREHENSIVE HOST INJECTION TESTS PASSED");
    return 0;
#endif
}
