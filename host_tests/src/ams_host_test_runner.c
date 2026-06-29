/*
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
static CAN_RxHeaderTypeDef fake_rx_hdr;
static uint8_t fake_rx_data[8];
static HAL_StatusTypeDef fake_rx_status = HAL_OK;
static jmp_buf task_exit_jmp;
static int task_exit_after_delay_until = 0;
static int fake_mux_write_enable = 1;

uint32_t osKernelGetTickCount(void){ return fake_tick; }
osStatus_t osDelay(uint32_t ticks){ fake_tick += ticks; return osOK; }
osStatus_t osDelayUntil(uint32_t ticks){ fake_tick = ticks; if(task_exit_after_delay_until){ task_exit_after_delay_until = 0; longjmp(task_exit_jmp, 1); } return osOK; }
BaseType_t xTaskCreate(TaskFunction_t fn, const char * const name, const configSTACK_DEPTH_TYPE stack, void * const arg, UBaseType_t prio, TaskHandle_t * const handle){ (void)fn;(void)name;(void)stack;(void)arg;(void)prio; if(handle) *handle=(TaskHandle_t)0x1; return pdPASS; }
void vTaskDelete(TaskHandle_t handle){ (void)handle; }

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan){ return hcan ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t notif){ (void)notif; return hcan ? HAL_OK : HAL_ERROR; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *hcan){ (void)hcan; return tx_free_level; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *hdr, const uint8_t *data, uint32_t *mailbox){
    if(!hcan || !hdr || !data || tx_count >= AMS_HOST_TX_LOG_CAPACITY) return HAL_ERROR;
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
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout){ (void)huart;(void)pData;(void)Size;(void)Timeout; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size){ (void)huart;(void)pData;(void)Size; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size){ (void)huart;(void)pData;(void)Size; return HAL_OK; }

void set_bms(bool state){ app.bms_state = state; HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }

// External driver stubs used by accumulator/CLI paths
void adBms6830_init(adbms6830_driver_t* dev, uint8_t num_ics, adbms6830_asic* ics, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port_a, GPIO_TypeDef* cs_port_b, uint16_t cs_pin_a, uint16_t cs_pin_b, TIM_HandleTypeDef *htim){ if(dev){ dev->num_ics=num_ics; dev->ics=ics; dev->hspi=hspi; dev->cs_port[0]=cs_port_a; dev->cs_port[1]=cs_port_b; dev->cs_pin[0]=cs_pin_a; dev->cs_pin[1]=cs_pin_b; dev->htim=htim; } }
void adbms6830_reset_cfg(adbms6830_driver_t *dev){(void)dev;} void adbms6830_srst(adbms6830_driver_t *dev){(void)dev;} void adbms6830_wrcfga(adbms6830_driver_t *dev){(void)dev;} void adbms6830_wrcfgb(adbms6830_driver_t *dev){(void)dev;} void adbms6830_rdcfga(adbms6830_driver_t *dev){(void)dev;} void adbms6830_rdcfgb(adbms6830_driver_t *dev){(void)dev;}
void adbms6830_adcv(adbms6830_driver_t *dev, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs){(void)dev;(void)rd;(void)cont;(void)dcp;(void)rstf;(void)owcs;} void adbms6830_wakeup(adbms6830_driver_t* dev){(void)dev;} void adbms6830_us_delay(adbms6830_driver_t* dev, uint16_t microseconds){(void)dev;(void)microseconds;} void adbms6830_start_adc_cell_voltage_measurement(adbms6830_driver_t *dev){(void)dev;} void adbms6830_parse_cell(adbms6830_driver_t *dev, uint8_t *data, GRP grp){(void)dev;(void)data;(void)grp;} void adbms6830_read_cell_voltages(adbms6830_driver_t *dev){(void)dev;}
int mux_read_gpio_voltage(adbms6830_driver_t *dev, uint8_t sensor_num){
    if(fake_mux_write_enable && dev && dev->ics && dev->num_ics > 0 && sensor_num < 24){
        dev->ics[0].temp.raw[sensor_num] = (int16_t)((2.5f/0.000150f)-10000.0f);
    }
    return (sensor_num < 24) ? 0 : -1;
}
int adbms6830_read_temp_raw(adbms6830_driver_t *dev, uint8_t ic_idx, int16_t *out_raw){ (void)dev;(void)ic_idx; if(out_raw) *out_raw=0; return 0; }
float adbms6830_convert_temp(adbms6830_driver_t *dev, uint8_t ic_idx, uint8_t sensor_num, float vref){ (void)dev;(void)ic_idx;(void)sensor_num;(void)vref; return 25.0f; }
float voltage_to_temp(float raw){ float voltage = ((float)raw + 10000.0f) * 0.000150f; if(voltage <= 0.0f || voltage >= 5.0f) return NAN; float resistance = 10000.0f * (5.0f - voltage) / voltage; float x = logf(resistance / 10000.0f); return (1.0f / (3.354016435e-3f + 2.565235509e-4f * x)) - 273.15f; }
int mux_set_channel(adbms6830_driver_t *dev, uint8_t sensor_num){ (void)dev; return sensor_num < 24 ? 0 : -1; }

void adbms2950_gpo_set(adbms2950_driver_t *dev, GPO gp, CFGA_GPO state){(void)dev;(void)gp;(void)state;} void adbms2950_wakeup(adbms2950_driver_t *dev){(void)dev;} void adbms2950_wrcfga(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdcfga(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdvb(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdi(adbms2950_driver_t *dev){(void)dev;} void adbms2950_adv(adbms2950_driver_t *dev, adv_ *adv){(void)dev;(void)adv;} void adbms2950_plv(adbms2950_driver_t *dev){(void)dev;} void adbms2950_rdv1d(adbms2950_driver_t *dev){(void)dev;}
HAL_StatusTypeDef stm32f767z_adc_switch_channel(ADC_HandleTypeDef *hadc, uint32_t channel){ (void)channel; return hadc ? HAL_OK : HAL_ERROR; }
uint16_t stm32f767z_adc_read(ADC_HandleTypeDef *hadc){ (void)hadc; return 2048; }

// Include actual implementation files so static helpers in canbus_task are testable.
#include "Core/Src/ext_drivers/charger.c"
#include "Core/Src/ext_drivers/fans.c"
#include "Core/Src/ext_drivers/current_sensor.c"
#include "Core/Src/ext_drivers/imd.c"
#include "Core/Src/ext_drivers/accumulator.c"
#include "Core/Src/ext_drivers/canbus.c"
#include "Core/Src/tasks/canbus_task.c"
#include "Core/Src/tasks/adbms_task.c"
#include "Core/Src/tasks/error_task.c"
#include "Core/Src/tasks/fan_task.c"
#include "Core/Src/tasks/current_task.c"

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

static void init_fake_app(void){ memset(&app,0,sizeof(app)); app.acc.smb.num_ics = NSMBS; app.acc.smb.ics = app.acc.smb_ics; }

static void test_accumulator_stats_and_balance(void){
    init_fake_app();
    for(int ic=0; ic<NSMBS; ic++) for(int c=0;c<NCELLS;c++) app.acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(3.700f);
    app.acc.smb_ics[2].cell.c_codes[7] = code_for_volts(4.100f);
    app.acc.smb_ics[4].cell.c_codes[14] = code_for_volts(3.200f);
    app.acc.smb_ics[0].cell.c_codes[0] = 0; // invalid should be skipped
    accumulator_update_voltage_stats(&app.acc);
    CHECK(fabsf(app.acc.max_volt - 4.100f) < 0.002f);
    CHECK(fabsf(app.acc.min_volt - 3.200f) < 0.002f);
    CHECK(app.acc.total_volt > 270.0f && app.acc.total_volt < 280.0f);
    CHECK(accumulator_set_balance(&app.acc) == 0);
    CHECK((app.acc.smb_ics[2].tx_cfgb.dcc & (1u << 7)) != 0u);
    CHECK((app.acc.smb_ics[4].tx_cfgb.dcc & (1u << 14)) == 0u);
    CHECK(accumulator_clear_balance(&app.acc) == 0);
    for(int ic=0; ic<NSMBS; ic++) CHECK(app.acc.smb_ics[ic].tx_cfgb.dcc == 0u);

    app.acc.smb.num_ics = 99; // bad config should still stay bounded
    accumulator_update_voltage_stats(&app.acc);
    CHECK(app.acc.max_volt > 4.0f && app.acc.min_volt > 3.0f);
}

static void test_temp_stats(void){
    init_fake_app();
    for(int ic=0; ic<NSMBS; ic++) for(int s=0;s<NTEMPS;s++) app.acc.smb_ics[ic].temp.raw[s] = raw_for_ntc_voltage(2.5f);
    app.acc.smb_ics[1].temp.raw[2] = 0; // invalid skip
    app.acc.smb_ics[3].temp.raw[5] = INT16_MIN; // invalid skip
    accumulator_update_temp_stats(&app.acc);
    CHECK(app.acc.max_temp > 20.0f && app.acc.max_temp < 30.0f);
    CHECK(app.acc.avg_temp > 20.0f && app.acc.avg_temp < 30.0f);
}

static void test_can_telemetry_packets(void){
    init_fake_app(); static CAN_HandleTypeDef hcan; app.board.canbus.hcan = &hcan;
    app.state = STATE_DISCARGE; app.air_state = true; app.current = -12.3f; app.imd_ok=true; app.imd_status=IMD_NORMAL; app.board.imd.duty=42.5f; app.max_temp=37.2f; app.min_voltage=3.201f; app.max_voltage=4.099f;
    for(int i=0;i<NFANS;i++) app.board.fans[i].duty_cycle = (float)(i*10);
    for(int ic=0; ic<NSMBS; ic++) for(int c=0;c<NCELLS;c++) app.acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(3.0f + 0.001f*(float)(ic*NCELLS+c));
    for(int ic=0; ic<NSMBS; ic++) for(int s=0;s<NTEMPS;s++) app.acc.smb_ics[ic].temp.raw[s] = raw_for_ntc_voltage(2.5f);
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

    app.state=STATE_CHARGE; app.hard_fault=false; app.voltage_fault=false; app.temp_fault=false; app.bms_state=true; app.board.charger.hardware_fail=false; app.board.charger.overtemp_fail=false; app.board.charger.input_volt_fail=false; app.board.charger.voltage_sense_fail=false; app.board.charger.communication_fail=false; app.board.charger.last_rx_tick=fake_tick;
    tx_count=0; tx_free_level=3; CHECK(canbus_send(&app.board.canbus, CAN_ID_EXT, CCS_CANBUS_ID, (uint8_t[8]){0x0C,0x30,0,0x0A,0,0,0,0}) == HAL_OK);
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

static void fill_nominal_pack(app_data_t *d, float base_v){
    d->acc.smb.num_ics = NSMBS; d->acc.smb.ics = d->acc.smb_ics;
    for(int ic=0; ic<NSMBS; ic++) for(int c=0;c<NCELLS;c++) d->acc.smb_ics[ic].cell.c_codes[c] = code_for_volts(base_v);
    for(int ic=0; ic<NSMBS; ic++) for(int s=0;s<NTEMPS;s++) d->acc.smb_ics[ic].temp.raw[s] = raw_for_ntc_voltage(2.5f);
}

static void test_task_iterations_with_injected_signals(void){
    static CAN_HandleTypeDef hcan;
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan = &hcan; app.state = STATE_DISCARGE;
    tx_count=0; tx_free_level=3; fake_tick=0;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == 62u); CHECK(app.canbus_fault == false); CHECK(app.board.charger.target_voltage == 0.0f);

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan = &hcan; app.state = STATE_CHARGE; app.bms_state=true; app.board.charger.last_rx_tick=1000; fake_tick=1000; tx_count=0; tx_free_level=3;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == 1u); CHECK(tx_log[0].ide == CAN_ID_EXT); CHECK(tx_log[0].extid == CCS_CANBUS_ID); CHECK(tx_log[0].data[4] == 0u); CHECK(app.canbus_fault == false); CHECK(app.charger_fault == false);

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan = &hcan; app.state = STATE_CHARGE; app.bms_state=true; app.board.charger.last_rx_tick=1; fake_tick=6005; tx_count=0; tx_free_level=3; bms_pin_state = GPIO_PIN_SET;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == 1u); CHECK(tx_log[0].data[4] == 1u); CHECK(app.board.charger.communication_fail == true); CHECK(app.charger_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET);

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state = STATE_CHARGE; app.bms_state=true; app.acc.smb_ics[0].cell.c_codes[0] = code_for_volts(4.000f); fake_tick=0;
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == false); CHECK(app.temp_fault == false); CHECK(app.max_voltage > 3.99f); CHECK(app.min_voltage > 3.69f); CHECK(app.acc.smb_ics[0].tx_cfgb.dcc != 0u);

    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state = STATE_CHARGE; app.bms_state=true; app.acc.smb_ics[2].cell.c_codes[3] = code_for_volts(2.400f); fake_tick=0; bms_pin_state=GPIO_PIN_SET;
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET); for(int ic=0; ic<NSMBS; ic++) CHECK(app.acc.smb_ics[ic].tx_cfgb.dcc == 0u);

    init_fake_app(); app.fuse_fault = true; app.temp_fault=false; app.voltage_fault=false; app.charger_fault=false; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
    run_one_error_task_iteration(&app);
    CHECK(app.hard_fault == true); CHECK(app.bms_state == false);

    init_fake_app(); for(int i=0;i<NFANS;i++){ static TIM_HandleTypeDef ht; static uint32_t ccrs[NFANS]; app.board.fans[i].CCR=&ccrs[i]; app.board.fans[i].max_timer_val=1000; app.board.fans[i].htim=&ht; }
    app.max_temp = TEMP_THRESH_H + 1.0f; run_one_fan_task_iteration(&app); CHECK(app.fan_state == true); for(int i=0;i<NFANS;i++) CHECK(app.board.fans[i].duty_cycle == 100.0f);
    app.max_temp = TEMP_THRESH_L - 1.0f; run_one_fan_task_iteration(&app); CHECK(app.fan_state == false); for(int i=0;i<NFANS;i++) CHECK(app.board.fans[i].duty_cycle == 0.0f);

    static ADC_HandleTypeDef adc1, adc2; init_fake_app(); app.board.current_sensor.hadc_high=&adc1; app.board.current_sensor.hadc_low=&adc2; run_one_current_task_iteration(&app); CHECK(app.current_fault == false);
    init_fake_app(); app.board.current_sensor.hadc_high=NULL; app.board.current_sensor.hadc_low=&adc2; run_one_current_task_iteration(&app); CHECK(app.current_fault == true);
}

static void test_fan_current_and_null_guards(void){
    fan_t fan={0}; uint32_t ccr=999; static TIM_HandleTypeDef htim; CHECK(fan_init(NULL,NULL,NULL,0,NULL,1) != 0); CHECK(fan_init(&fan,NULL,&htim,1000,&ccr,1)==0); CHECK(ccr==0u); CHECK(set_fan_percent(&fan,120.0f)==0 && ccr==1000u && fabsf(fan.duty_cycle-100.0f)<0.01f); CHECK(set_fan_percent(&fan,-10.0f)==0 && ccr==0u && fabsf(fan.duty_cycle)<0.01f);
    current_sensor_t cs={0}; cs.count_low=3102; cs.count_high=3102; float current=current_sensor_convert(&cs); CHECK(current > -1.0f && current < 1.0f); cs.count_low=4095; cs.count_high=3000; current=current_sensor_convert(&cs); CHECK(fabsf(current - cs.current_high) < 0.001f);
    CHECK(current_sensor_convert(NULL) == 0.0f);
    CHECK(accumulator_set_balance(NULL) == -1); CHECK(accumulator_clear_balance(NULL) == -1); accumulator_update_voltage_stats(NULL); accumulator_update_temp_stats(NULL);
    CHECK(imd_read(NULL) == 1); imd_init(NULL, 0, NULL, NULL, 0, 0, NULL, 0);
}


static void test_fault_matrix_extra(void){
    static CAN_HandleTypeDef hcan;
    static ADC_HandleTypeDef adc1;

    // 1) Overvoltage must hard-disable BMS and clear balancing.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state=STATE_CHARGE; app.bms_state=true; bms_pin_state=GPIO_PIN_SET; app.acc.smb_ics[1].cell.c_codes[4]=code_for_volts(4.250f);
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET);
    for(int ic=0; ic<NSMBS; ic++) CHECK(app.acc.smb_ics[ic].tx_cfgb.dcc == 0u);

    // 2) All invalid cell readings must fail safe. A pack with no valid cell data cannot be considered safe.
    init_fake_app(); app.acc.smb.num_ics=NSMBS; app.acc.smb.ics=app.acc.smb_ics; app.state=STATE_CHARGE; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
    for(int ic=0; ic<NSMBS; ic++) for(int c=0;c<NCELLS;c++) app.acc.smb_ics[ic].cell.c_codes[c] = 0;
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET);

    // 3) High temperature must hard-disable BMS and prevent balancing.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.state=STATE_CHARGE; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
    for(int ic=0; ic<NSMBS; ic++) for(int s=0;s<NTEMPS;s++) app.acc.smb_ics[ic].temp.raw[s] = raw_for_ntc_voltage(4.5f); // very hot NTC divider result
    run_one_adbms_task_iteration(&app);
    CHECK(app.temp_fault == true); CHECK(app.bms_state == false); CHECK(bms_pin_state == GPIO_PIN_RESET);
    for(int ic=0; ic<NSMBS; ic++) CHECK(app.acc.smb_ics[ic].tx_cfgb.dcc == 0u);

    // 4) Charger-reported faults must command disable in charge state.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan=&hcan; app.state=STATE_CHARGE; app.bms_state=true; app.board.charger.last_rx_tick=1000; fake_tick=1000;
    app.board.charger.hardware_fail=true; app.board.charger.overtemp_fail=false; app.board.charger.input_volt_fail=false; app.board.charger.voltage_sense_fail=false; tx_count=0; tx_free_level=3; bms_pin_state=GPIO_PIN_SET;
    run_one_canbus_task_iteration(&app);
    CHECK(app.charger_fault == true); CHECK(app.bms_state == false); CHECK(tx_count == 1u); CHECK(tx_log[0].data[4] == 1u);

    // 5) Pre-existing hard fault must command charger disable even without charger self-fault.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan=&hcan; app.state=STATE_CHARGE; app.bms_state=true; app.hard_fault=true; app.board.charger.last_rx_tick=1000; fake_tick=1000; tx_count=0; tx_free_level=3;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == 1u); CHECK(tx_log[0].data[4] == 1u); CHECK(app.bms_state == false);

    // 6) CAN transmit failure must become a soft CAN fault, not silently pass.
    init_fake_app(); fill_nominal_pack(&app, 3.700f); app.board.canbus.hcan=&hcan; app.state=STATE_DISCARGE; tx_count=0; tx_free_level=0; fake_tick=0;
    run_one_canbus_task_iteration(&app);
    CHECK(app.canbus_fault == true);

    // 7) Fan driver failure must set fan soft fault, then error task should mark soft fault only.
    init_fake_app(); app.max_temp = TEMP_THRESH_H + 5.0f; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
    // Leave fan CCR pointers NULL, so set_fan_percent() fails.
    run_one_fan_task_iteration(&app);
    CHECK(app.fan_fault == true);
    run_one_error_task_iteration(&app);
    CHECK(app.soft_fault == true); CHECK(app.hard_fault == false); CHECK(app.bms_state == true);

    // 8) Current ADC missing must set current fault and become a soft fault only.
    init_fake_app(); app.board.current_sensor.hadc_high=&adc1; app.board.current_sensor.hadc_low=NULL; app.bms_state=true; bms_pin_state=GPIO_PIN_SET;
    run_one_current_task_iteration(&app);
    CHECK(app.current_fault == true);
    run_one_error_task_iteration(&app);
    CHECK(app.soft_fault == true); CHECK(app.hard_fault == false); CHECK(app.bms_state == true);

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
    accumulator_update_voltage_stats(&app.acc);
    CHECK(app.acc.valid_voltage_count == expected_valid);
    CHECK(fabsf(app.acc.min_volt - expected_min) < 0.002f);
    CHECK(fabsf(app.acc.max_volt - expected_max) < 0.002f);
    CHECK(fabsf(app.acc.total_volt - expected_total) < 0.050f);
}

static void test_temp_invalid_and_cold_valid_fault_behavior(void){
    fake_mux_write_enable = 0;

    init_fake_app();
    fill_nominal_pack(&app, 3.700f);
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
    int16_t raw0c = raw_for_temp_c(0.0f);
    for(int ic=0; ic<NSMBS; ic++) for(int s=0; s<NTEMPS; s++) app.acc.smb_ics[ic].temp.raw[s] = raw0c;
    app.state = STATE_DISCARGE; app.bms_state = true; bms_pin_state = GPIO_PIN_SET; fake_tick = 0;
    run_one_adbms_task_iteration(&app);
    CHECK(app.voltage_fault == false);
    CHECK(app.temp_fault == false);
    CHECK(app.acc.valid_temp_count == (uint16_t)(NSMBS * NTEMPS));
    CHECK(app.max_temp > -2.0f && app.max_temp < 2.0f);
    CHECK(app.bms_state == true);

    fake_mux_write_enable = 1;
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

static void test_charge_state_disable_matrix(void){
    static CAN_HandleTypeDef hcan;
    struct Case { bool hard_fault, voltage_fault, temp_fault, bms_state, hw, ot, input, sense, timeout; uint8_t disable; } cases[] = {
        {0,0,0,1,0,0,0,0,0,0},
        {1,0,0,1,0,0,0,0,0,1},
        {0,1,0,1,0,0,0,0,0,1},
        {0,0,1,1,0,0,0,0,0,1},
        {0,0,0,0,0,0,0,0,0,1},
        {0,0,0,1,1,0,0,0,0,1},
        {0,0,0,1,0,1,0,0,0,1},
        {0,0,0,1,0,0,1,0,0,1},
        {0,0,0,1,0,0,0,1,0,1},
        {0,0,0,1,0,0,0,0,1,1},
    };
    for(size_t i=0; i<sizeof(cases)/sizeof(cases[0]); i++){
        init_fake_app(); fill_nominal_pack(&app, 3.700f); charger_init(&app.board.charger, &app.board.canbus);
        app.board.canbus.hcan = &hcan; app.state = STATE_CHARGE;
        app.hard_fault = cases[i].hard_fault; app.voltage_fault = cases[i].voltage_fault; app.temp_fault = cases[i].temp_fault; app.bms_state = cases[i].bms_state;
        app.board.charger.hardware_fail = cases[i].hw; app.board.charger.overtemp_fail = cases[i].ot; app.board.charger.input_volt_fail = cases[i].input; app.board.charger.voltage_sense_fail = cases[i].sense;
        fake_tick = cases[i].timeout ? 6001u : 1000u;
        app.board.charger.last_rx_tick = 1000u;
        tx_count = 0; tx_free_level = 3; bms_pin_state = app.bms_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
        run_one_canbus_task_iteration(&app);
        CHECK(tx_count == 1u);
        CHECK(tx_log[0].ide == CAN_ID_EXT && tx_log[0].extid == CCS_CANBUS_ID);
        CHECK(word_at(0,0) == (uint16_t)(CHARGE_MAX_VOLTAGE * 10.0f));
        CHECK(word_at(0,1) == (uint16_t)(CHARGE_MAX_CURRENT * 10.0f));
        CHECK(tx_log[0].data[4] == cases[i].disable);
        if(cases[i].disable){ CHECK(app.bms_state == false); }
        else { CHECK(app.bms_state == true && app.charger_fault == false); }
    }
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
    tx_count = 0; tx_free_level = 3;
    run_one_canbus_task_iteration(&app);
    CHECK(tx_count == 62u);
    CHECK(word_at(3,1) > 3500u && word_at(3,2) == 0u && word_at(3,3) > 3500u);
    CHECK(word_at(28 + 6 + 1,1) == 0u); // segment 1, temp sensor 3 invalid => packet 35 word0
    // Segments 2..4 are absent because num_ics=2; their voltage and temp packets must be zero-filled.
    for(uint32_t frame=13u; frame<=27u; frame++) CHECK(word_at(frame,1)==0u && word_at(frame,2)==0u && word_at(frame,3)==0u);
    for(uint32_t frame=40u; frame<=57u; frame++) CHECK(word_at(frame,1)==0u && word_at(frame,2)==0u && word_at(frame,3)==0u);
}

static void test_periods_and_driver_edge_cases(void){
    static CAN_HandleTypeDef hcan;
    init_fake_app(); fill_nominal_pack(&app, 3.7f); app.board.canbus.hcan=&hcan; app.state=STATE_DISCARGE; fake_tick=123; tx_count=0; tx_free_level=3;
    run_one_canbus_task_iteration(&app); CHECK(fake_tick == 123u + (1000u / CAN_FREQ));

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

int main(void){
    test_accumulator_stats_and_balance(); puts("PASS accumulator stats/balance");
    test_voltage_stats_boundaries_and_fuzz(); puts("PASS voltage boundary/fuzz stats");
    test_temp_stats(); puts("PASS temp stats");
    test_temp_invalid_and_cold_valid_fault_behavior(); puts("PASS temp invalid/cold-valid fault behavior");
    test_can_telemetry_packets(); puts("PASS CAN telemetry packetization");
    test_telemetry_absent_segments_and_invalid_channels(); puts("PASS telemetry absent segments/invalid channels");
    test_charger_rx_and_tx(); puts("PASS charger RX/TX parse");
    test_can_rx_filter_matrix(); puts("PASS CAN RX filter matrix");
    test_charge_state_disable_matrix(); puts("PASS charge-state disable matrix");
    test_fan_current_and_null_guards(); puts("PASS fan/current/null guards");
    test_periods_and_driver_edge_cases(); puts("PASS periods and driver edge cases");
    test_task_iterations_with_injected_signals(); puts("PASS one-iteration task injection tests");
    test_fault_matrix_extra(); puts("PASS fault matrix extra");
    puts("ALL COMPREHENSIVE HOST INJECTION TESTS PASSED");
    return 0;
}
