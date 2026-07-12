/*
 * Percepio TraceRecorder for Tracealyzer v4.12.0
 * Copyright 2025 Percepio AB
 * www.percepio.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Configuration parameters for the kernel port.
 * More settings can be found in trcKernelPortStreamingConfig.h and
 * trcKernelPortSnapshotConfig.h.
 */

#ifndef TRC_KERNEL_PORT_CONFIG_H
#define TRC_KERNEL_PORT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def TRC_CFG_FREERTOS_VERSION
 * @

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def TRC_CFG_Fbrief FreeRTOS kernel version used by this project.
 *
 * DER26-AMS uses FreeRTOS V10.2.0.
 */
#define TRC_CFG_FREERTOS_VERSION TRC_FREERTOS_VERSION_10_2_0

/**
 * @def TRC_CFG_INCLUDE_EVENT_GROUP_EVENTS
 * @brief Include FreeRTOS event-group events in the trace.
 *
 * Leave disabled unless the application uses event groups and
 * event_groups.c is included in the build.
 */
#define TRC_CFG_INCLUDE_EVENT_GROUP_EVENTS 0

/**
 * @def TRC_CFG_INCLUDE_TIMER_EVENTS
 * @brief Include FreeRTOS software-timer events.
 *
 * The project includes timers.c, but leave this disabled during the first
 * low-bandwidth SWO bring-up. It can be enabled after streaming is stable.
 */
#define TRC_CFG_INCLUDE_TIMER_EVENTS 0

/**
 * @def TRC_CFG_INCLUDE_PEND_FUNC_CALL_EVENTS
 * @brief Include xTimerPendFunctionCall events.
 */
#define TRC_CFG_INCLUDE_PEND_FUNC_CALL_EVENTS 0

/**
 * @def TRC_CFG_INCLUDE_STREAM_BUFFER_EVENTS
 * @brief Include stream-buffer and message-buffer events.
 *
 * Keep disabled unless the application actually uses these objects.
 */
#define TRC_CFG_INCLUDE_STREAM_BUFFER_EVENTS 0

/**
 * @def TRC_CFG_ACKNOWLEDGE_QUEUE_SET_SEND
 *
 * Only relevant to FreeRTOS V10.3.0 and V10.3.1. DER26-AMS uses V10.2.0,
 * so this remains disabled.
 */
#define TRC_CFG_ACKNOWLEDGE_QUEUE_SET_SEND 0

/**
 * @def TRC_CFG_KERNEL_PORT_TASK_MONITOR_TLS_INDEX
 * @brief Thread-local-storage slot used by TraceRecorder's task monitor.
 *
 * Slot 0 is acceptable provided no application code uses that TLS slot.
 */
#define TRC_CFG_KERNEL_PORT_TASK_MONITOR_TLS_INDEX 0

#ifdef __cplusplus
}
#endif

#endif /* TRC_KERNEL_PORT_CONFIG_H */
