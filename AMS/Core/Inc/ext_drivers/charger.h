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
#define CHARGE_MAX_CURRENT 10.0f
#define CHARGER_COMMAND_PERIOD_MS 1000u
#define CHARGER_RX_TIMEOUT_MS     5000u
#define CHARGER_CMD_ENABLE        0u
#define CHARGER_CMD_DISABLE       1u

typedef enum {
    CHARGER_DISABLE_REASON_NONE = 0u,
    CHARGER_DISABLE_REASON_HW_FAULT = (1u << 0),
    CHARGER_DISABLE_REASON_HARD_FAULT = (1u << 1),
    CHARGER_DISABLE_REASON_VOLTAGE_FAULT = (1u << 2),
    CHARGER_DISABLE_REASON_VOLTAGE_CHARGE_STOP = (1u << 3),
    CHARGER_DISABLE_REASON_VOLTAGE_INVALID = (1u << 4),
    CHARGER_DISABLE_REASON_TEMP_CHARGE_STOP = (1u << 5),
    CHARGER_DISABLE_REASON_TEMP_FAULT = (1u << 6),
    CHARGER_DISABLE_REASON_CURRENT_FAULT = (1u << 7),
    CHARGER_DISABLE_REASON_CURRENT_INVALID = (1u << 8),
    CHARGER_DISABLE_REASON_BMS_NOT_OK = (1u << 9),
    CHARGER_DISABLE_REASON_TX_FAIL = (1u << 10)
} charger_disable_reason_t;

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
    bool tx_fail;
    HAL_StatusTypeDef last_tx_status;
    uint16_t disable_reason_mask;
    uint32_t last_rx_tick;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t tx_fail_count;
} charger_t;

void charger_init(charger_t *dev, canbus_device_t *canbus);

#endif

#endif /* INC_EXT_DRIVERS_CHARGER_H_ */
