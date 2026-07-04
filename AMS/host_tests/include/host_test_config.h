/*
 * host_test_config.h
 * Author: Mahad Faisal (2026)
 */

#ifndef AMS_HOST_TEST_CONFIG_H
#define AMS_HOST_TEST_CONFIG_H

/*
 * DER26 AMS host test harness config.
 *
 * Override these in the compiler command line if the team changes timing or
 * protocol assumptions and wants the host harness to document that change.
 */
#ifndef AMS_HOST_TX_LOG_CAPACITY
#define AMS_HOST_TX_LOG_CAPACITY 256u
#endif

#ifndef AMS_HOST_CAN_MAILBOX_TIMEOUT_TICKS
#define AMS_HOST_CAN_MAILBOX_TIMEOUT_TICKS 10u
#endif

#endif
