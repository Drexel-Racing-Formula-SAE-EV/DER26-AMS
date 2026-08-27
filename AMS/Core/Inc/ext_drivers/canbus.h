/**
 * @file canbus.h
 * @brief DER26 AMS bxCAN transport, RX buffering, and asynchronous TX service.
 */
#ifndef __CANBUS_H_
#define __CANBUS_H_

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include "ext_drivers/can_tx_scheduler.h"

#define DATALEN 8

/* CAN1 owns filter banks 0..13. CAN2, if ever enabled, starts at bank 14. */
#define CANBUS_SLAVE_FILTER_START 14u
#define CANBUS_RX_QUEUE_DEPTH 96u

#if CANBUS_RX_QUEUE_DEPTH < 2u
#error "CANBUS_RX_QUEUE_DEPTH must be at least 2"
#endif

/* DER26-CAN-V4 standard-data diagnostic feedback from ECU to AMS. It is
 * observability only and may never be required to assert BMS_OK. */
#define AMS_ECU_DIAG_FEEDBACK_CAN_ID 0x6F0u
#define AMS_ECU_DIAG_FEEDBACK_DLC    8u
#define AMS_ECU_DIAG_FEEDBACK_VERSION 1u

/* Encoded-frame build buffer. The CAN task is the only producer. */
#define CANBUS_TX_BUILD_MAX_FRAMES AMS_CAN_TX_DETAIL_MAX_FRAMES

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

typedef enum {
    CANBUS_TX_BUILD_NONE = 0,
    CANBUS_TX_BUILD_CRITICAL,
    CANBUS_TX_BUILD_PROTECTED,
    CANBUS_TX_BUILD_DETAIL
} canbus_tx_build_kind_t;

typedef enum {
    CANBUS_TX_TAG_NONE = 0,
    CANBUS_TX_TAG_CHARGER_NORMAL,
    CANBUS_TX_TAG_CHARGER_SHUTDOWN
} canbus_tx_source_tag_t;

typedef enum {
    CANBUS_TX_MB_FREE = 0,
    /* RESERVED is a deliberately brief software-owned transition used after
     * HAL has selected a mailbox but before the scheduler token is committed
     * as LOADED. TX-mailbox interrupts are masked during this transition, so
     * callbacks cannot observe half-written metadata. */
    CANBUS_TX_MB_RESERVED,
    CANBUS_TX_MB_LOADED,
    CANBUS_TX_MB_ABORT_REQUESTED
} canbus_tx_mailbox_state_t;

typedef struct {
    canbus_tx_mailbox_state_t state;
    ams_can_tx_token_t token;
    uint32_t can_id;
    uint32_t loaded_tick;
    ams_can_tx_class_t tx_class;
    uint16_t source_tag;
    uint32_t controller_epoch;
} canbus_tx_mailbox_meta_t;

typedef struct {
    bool active;
    canbus_tx_build_kind_t kind;
    uint32_t generation;
    uint32_t publish_tick;
    uint16_t required_count;
    uint16_t source_tag;
    uint16_t frame_count;
    ams_can_tx_frame_t frames[CANBUS_TX_BUILD_MAX_FRAMES];
} canbus_tx_builder_t;

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
    volatile uint32_t rx_filtered_count;
    volatile uint32_t rx_remote_reject_count;
    volatile uint32_t rx_malformed_count;
    volatile uint32_t rx_fifo_overrun_count;
    volatile uint32_t rx_queue_drop_count;
    volatile uint32_t rx_hal_error_count;
    volatile uint32_t rx_feedback_count;
    uint32_t rx_queue_drop_reported;
    uint32_t rx_hal_error_reported;
    uint32_t rx_fifo_overrun_reported;

    uint8_t ecu_feedback_version;
    uint8_t ecu_feedback_protected_sequence;
    uint8_t ecu_feedback_snapshot_sequence;
    uint8_t ecu_feedback_flags;
    uint16_t ecu_feedback_rx_diag;
    uint16_t ecu_feedback_uptime_counter;
    uint32_t ecu_feedback_last_tick;

    ams_can_tx_scheduler_t tx_scheduler;
    canbus_tx_builder_t tx_builder;
    canbus_tx_mailbox_meta_t tx_mailbox_meta[3];
    volatile bool tx_pump_busy;
    volatile bool tx_kick_pending;
    volatile bool tx_suspended;
    volatile bool tx_latched_inhibit;
    volatile bool error_isr_pending;
    volatile uint32_t error_isr_code;
    uint32_t tx_generation_counter;
    uint32_t tx_hal_load_error_count;
    uint32_t tx_hal_load_error_reported;
    uint32_t tx_hal_load_error_critical_count;
    uint32_t tx_hal_load_error_critical_reported;
    uint32_t tx_hal_load_error_protected_count;
    uint32_t tx_hal_load_error_protected_reported;
    uint32_t tx_hal_load_error_detail_count;
    uint32_t tx_hal_load_error_detail_reported;
    uint32_t tx_charger_normal_load_error_count;
    uint32_t tx_charger_normal_load_error_reported;
    uint32_t tx_charger_shutdown_load_error_count;
    uint32_t tx_charger_shutdown_load_error_reported;
    uint16_t tx_last_hal_load_error_source_tag;
    ams_can_tx_class_t tx_last_hal_load_error_class;
    uint32_t tx_complete_count;
    uint32_t tx_abort_request_count;
    uint32_t tx_abort_complete_count;
    uint32_t tx_abort_race_complete_count;
    uint32_t tx_abort_request_fail_count;
    uint32_t tx_unexpected_callback_count;
    uint32_t tx_pump_kick_count;
    uint32_t tx_pump_deferred_kick_count;
    uint32_t tx_irq_mask_error_count;
    uint32_t tx_irq_mask_error_reported;

    /* Publication/build failures are separate from bus congestion. A full
     * detail build buffer is an explicit source-side capacity failure and
     * must never look like normal snapshot supersession. */
    uint32_t tx_build_overflow_count;
    uint32_t tx_build_detail_overflow_count;
    uint32_t tx_build_class_reject_count;
    uint32_t tx_build_commit_reject_count;

    uint32_t tx_recovery_epoch_count;
    uint32_t busoff_window_start_tick;
    uint8_t busoff_window_count;

    HAL_StatusTypeDef init_status;
    HAL_StatusTypeDef start_status;
    HAL_StatusTypeDef notification_status;
    bool started;
    bool notification_active;
} canbus_device_t;

HAL_StatusTypeDef canbus_device_init(canbus_device_t *dev, CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef canbus_configure_rx_filters(CAN_HandleTypeDef *hcan);
uint16_t canbus_rx_queue_count(const canbus_device_t *dev);
uint32_t canbus_process_rx_queue(canbus_device_t *dev, app_data_t *data, uint32_t max_frames);
void canbus_poll_errors(canbus_device_t *dev, app_data_t *data);
HAL_StatusTypeDef canbus_recover(canbus_device_t *dev);
const char *canbus_error_str(uint32_t err);

/* Task-context asynchronous TX publication API. No function below waits for
 * wire serialization. canbus_tx_build_append() only encodes into fixed RAM;
 * commit publishes ACTIVE/PENDING scheduler state and primes bxCAN. */
HAL_StatusTypeDef canbus_tx_build_begin(canbus_device_t *dev,
                                        canbus_tx_build_kind_t kind,
                                        uint32_t generation,
                                        uint32_t publish_tick,
                                        uint16_t source_tag);
HAL_StatusTypeDef canbus_tx_build_append(canbus_device_t *dev,
                                         uint32_t ide,
                                         uint32_t id,
                                         const uint8_t payload[8]);
HAL_StatusTypeDef canbus_tx_build_commit(canbus_device_t *dev,
                                         uint16_t required_count);
void canbus_tx_build_cancel(canbus_device_t *dev);
void canbus_tx_kick(canbus_device_t *dev);
void canbus_tx_abort_protected_generation(canbus_device_t *dev,
                                          uint32_t generation);
void canbus_tx_note_busoff(canbus_device_t *dev);
void canbus_tx_note_recovered(canbus_device_t *dev);
uint32_t canbus_tx_next_generation(canbus_device_t *dev);

#endif /* __CANBUS_H_ */
