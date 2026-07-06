/**
 * @file canbus.h
 * @author Cole Bardin (cab572@drexel.edu)
 * @author Mahad Faisal (major firmware updates, 2026)
 * @brief
 * @version 0.1
 * @date 2023-04-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CANBUS_H_
#define __CANBUS_H_

#include <stdbool.h>
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"

#define DATALEN 8

typedef struct {
    uint32_t id;
    uint32_t ide;
    uint32_t dlc;
    uint8_t data[DATALEN];
} canbus_packet_t;

typedef struct {
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    canbus_packet_t rx_packet;
} canbus_device_t;

typedef struct {
    bool enabled;
    uint32_t total_tx;
    uint32_t std_tx;
    uint32_t ext_tx;
    uint32_t ecu_tx;
    uint32_t estimator_tx;
    uint32_t logger_tx;
    uint32_t charger_tx;
    uint32_t other_tx;
    uint32_t last_id;
    uint32_t last_ide;
    uint8_t last_payload[DATALEN];
    uint32_t logger_id_count[24];
} canbus_capture_status_t;

void canbus_device_init(canbus_device_t *dev, CAN_HandleTypeDef *hcan);
bool canbus_capture_enabled(void);
void canbus_capture_clear(void);
canbus_capture_status_t canbus_capture_get_status(void);

#endif
