/*
 * plant_shared.h
 *
 * Shared data between plant_task and CAN tasks.
 * Protected by plant_mutex — always take before read or write.
 *
 * Plant convention:
 *   - 75s6p Molicel P42A accumulator model
 *   - positive I_pack = discharge
 *   - model step period = 100 ms
 *
 * CAN base frames:
 *   0x200 measurement  V_pack, I_pack, T_surf, counter
 *   0x201 truth/debug  SoC_true, T_core, counter, plant_step[23:0]
 *   0x202 AMS summary  V_min, V_max, T_max, T_avg
 */

#ifndef MAIN_PLANT_SHARED_H_
#define MAIN_PLANT_SHARED_H_

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define PLANT_NUM_GROUPS       75U
#define PLANT_NUM_SEGMENTS     5U
#define PLANT_NUM_THERMISTORS  120U

typedef struct {
    float    V_pack;    /* V  — 75s pack terminal voltage             */
    float    I_pack;    /* A  — pack current, positive = discharge    */
    float    T_surf;    /* degC — representative surface temperature  */
    float    T_core;    /* degC — representative core temperature     */
    float    SoC_true;  /* 0–1 — true representative-cell SoC         */

    float    V_group[PLANT_NUM_GROUPS];          /* V — 75 synthetic series group voltages */
    float    V_segment[PLANT_NUM_SEGMENTS];      /* V — 5 segment voltages, 15 groups each */
    float    T_sensor[PLANT_NUM_THERMISTORS];    /* degC — synthetic AMS thermistor map    */
    float    SoC_group[PLANT_NUM_GROUPS];        /* 0–1 — synthetic group SoC spread       */

    float    V_min;     /* V — min group voltage                      */
    float    V_max;     /* V — max group voltage                      */
    float    T_max;     /* degC — max thermistor temperature          */
    float    T_avg;     /* degC — average thermistor temperature      */

    uint8_t  counter;   /* wrapping frame counter                     */
    uint32_t step;      /* plant model step counter, 10 Hz            */
} plant_shared_t;

extern SemaphoreHandle_t plant_mutex;
extern plant_shared_t    plant_shared;

#endif /* MAIN_PLANT_SHARED_H_ */
