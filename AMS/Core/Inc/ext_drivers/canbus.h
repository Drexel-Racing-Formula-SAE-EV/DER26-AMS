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

/*
 * One complete HIL accumulator image is 65 CAN frames (25 cell frames and
 * 40 temperature frames).  Keep enough ISR-to-task buffering for that burst,
 * plus margin for charger and diagnostic traffic.  The single-producer,
 * single-consumer ring intentionally leaves one entry empty.
 */
#define CANBUS_RX_QUEUE_DEPTH 96u

#if CANBUS_RX_QUEUE_DEPTH < 2u
#error "CANBUS_RX_QUEUE_DEPTH must be at least 2"
#endif

typedef struct app_data_t app_data_t;

typedef struct {
    uint32_t id;
    uint32_t ide;
    uint32_t rtr;
    uint32_t dlc;
    uint32_t tick;
    uint8_t data[DATALEN];
} canbus_packet_t;

typedef struct {
    uint32_t id;
    uint32_t tick;
    uint8_t ide;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[DATALEN];
} canbus_rx_frame_t;

typedef struct {
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    canbus_packet_t rx_packet;
    canbus_rx_frame_t rx_queue[CANBUS_RX_QUEUE_DEPTH];
    volatile uint16_t rx_queue_head;
    volatile uint16_t rx_queue_tail;
    volatile uint16_t rx_queue_high_water;
    volatile uint32_t rx_isr_count;
    volatile uint32_t rx_processed_count;
    volatile uint32_t rx_queue_drop_count;
    volatile uint32_t rx_hal_error_count;
    uint32_t rx_queue_drop_reported;
    uint32_t rx_hal_error_reported;
	HAL_StatusTypeDef init_status;
	HAL_StatusTypeDef start_status;
	HAL_StatusTypeDef notification_status;
	bool started;
	bool notification_active;
} canbus_device_t;

HAL_StatusTypeDef canbus_device_init(canbus_device_t *dev, CAN_HandleTypeDef *hcan);
uint16_t canbus_rx_queue_count(const canbus_device_t *dev);
uint32_t canbus_process_rx_queue(canbus_device_t *dev, app_data_t *data, uint32_t max_frames);
void canbus_poll_errors(canbus_device_t *dev, app_data_t *data);
HAL_StatusTypeDef canbus_recover(canbus_device_t *dev);
const char *canbus_error_str(uint32_t err);

#endif
