/*
 * charger.h
 *
 *  Created on: Jun 9, 2026
 *      Author: logan
 */

#ifndef INC_EXT_DRIVERS_CHARGER_H_
#define INC_EXT_DRIVERS_CHARGER_H_

#ifndef __CHARGER_H_
#define __CHARGER_H_

#include <stdbool.h>
#include <stdint.h>
#include "ext_drivers/canbus.h"

#define CCS_CANBUS_ID      0x1806E5F4
#define CHARGER_RX_ID      0x18FF50E5
#define CHARGE_MAX_VOLTAGE 312.0f
#define CHARGE_MAX_CURRENT 1.0f

typedef struct {
    canbus_device_t *canbus;
    float target_voltage;
    float target_current;
    float read_voltage;
    float read_current;
    uint8_t flags;
    bool hardware_fail;
    bool overtemp_fail;
    bool input_volt_fail;
    bool voltage_sense_fail;
    bool communication_fail;
    uint32_t last_rx_tick;
    uint32_t tx_count;
    uint32_t rx_count;
} charger_t;

void charger_init(charger_t *dev, canbus_device_t *canbus);

#endif

#endif /* INC_EXT_DRIVERS_CHARGER_H_ */
