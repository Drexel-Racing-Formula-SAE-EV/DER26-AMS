/*
 * charger.h
 *
 *  Created on: Jun 9, 2026
 *      Author: logan
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef INC_EXT_DRIVERS_CHARGER_H_
#define INC_EXT_DRIVERS_CHARGER_H_

#ifndef __CHARGER_H_
#define __CHARGER_H_

#include <stdbool.h>
#include <stdint.h>
#include "ext_drivers/canbus.h"

#define CCS_CANBUS_ID      0x1806E5F4
#define CHARGER_RX_ID      0x18FF50E5u
#define CHARGE_MAX_VOLTAGE 312.0f
#define CHARGE_MAX_CURRENT 10.0f
#define CHARGER_COMMAND_PERIOD_MS 1000u
#define CHARGER_RX_TIMEOUT_MS     5000u
#define CHARGER_CMD_ENABLE        0u
#define CHARGER_CMD_DISABLE       1u
/* A successful HAL queue operation is not an end-to-end charger
 * acknowledgement.  Repeat a zero-demand disable command on mode exit to
 * reduce the chance that one lost/arbitrated frame leaves the charger active. */
#define CHARGER_EXIT_DISABLE_FRAMES 3u

#if (CHARGER_EXIT_DISABLE_FRAMES < 1u) || (CHARGER_EXIT_DISABLE_FRAMES > 255u)
#error "CHARGER_EXIT_DISABLE_FRAMES must fit the nonzero uint8_t retry field"
#endif

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
    CHARGER_DISABLE_REASON_TX_FAIL = (1u << 10),
    CHARGER_DISABLE_REASON_STATE_EXIT = (1u << 11)
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
    bool shutdown_pending;
    uint8_t shutdown_frames_remaining;
    HAL_StatusTypeDef last_tx_status;
    HAL_StatusTypeDef last_shutdown_status;
    uint16_t disable_reason_mask;
    uint32_t last_rx_tick;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t tx_fail_count;
    uint32_t shutdown_request_count;
    uint32_t shutdown_tx_count;
    uint32_t shutdown_tx_fail_count;
} charger_t;

void charger_init(charger_t *dev, canbus_device_t *canbus);

#endif

#endif /* INC_EXT_DRIVERS_CHARGER_H_ */
