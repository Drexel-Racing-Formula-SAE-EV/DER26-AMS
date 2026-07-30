/*
 * plant_shared.h
 *
 * Shared data between plant_task and CAN tasks.
 * Protected by plant_mutex — always take before read or write.
 *
 * Plant convention:
 *   - topology and array dimensions come from plant_model_manifest.h
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

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "plant_model_manifest.h"

typedef struct {
    bool     valid;     /* true only after one complete model step          */
    float    V_pack;    /* V  — pack terminal voltage                 */
    float    I_pack;    /* A  — pack current, positive = discharge    */
    float    T_surf;    /* degC — representative surface temperature  */
    float    T_core;    /* degC — representative core temperature     */
    float    SoC_true;  /* 0–1 — true representative-cell SoC         */

    float    V_group[PLANT_NUM_GROUPS];          /* V — series-group voltages               */
    float    V_segment[PLANT_NUM_SEGMENTS];      /* V — mapped segment voltages             */
    float    T_sensor[PLANT_NUM_THERMISTORS];    /* degC — mapped AMS thermistor image      */
    float    SoC_group[PLANT_NUM_GROUPS];        /* 0–1 — group SoC image                   */

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
