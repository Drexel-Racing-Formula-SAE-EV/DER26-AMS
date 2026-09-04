#ifndef INC_EXT_DRIVERS_CAN_TX_SCHEDULER_H_
#define INC_EXT_DRIVERS_CAN_TX_SCHEDULER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AMS_CAN_TX_CRITICAL_MAX_FRAMES   4u
#define AMS_CAN_TX_PROTECTED_MAX_FRAMES 12u
#define AMS_CAN_TX_DETAIL_MAX_FRAMES   192u
#define AMS_CAN_TX_PROTECTED_PERIOD_MS 100u

#if AMS_CAN_TX_PROTECTED_MAX_FRAMES < 8u
#error "Protected TX storage must hold the complete required 0x680-0x687 set"
#endif

typedef enum {
    AMS_CAN_TX_CLASS_CRITICAL = 0,
    AMS_CAN_TX_CLASS_PROTECTED_REQUIRED,
    AMS_CAN_TX_CLASS_PROTECTED_ADVISORY,
    AMS_CAN_TX_CLASS_DETAIL
} ams_can_tx_class_t;

typedef struct {
    uint32_t id;
    uint32_t ide;
    uint8_t dlc;
    uint8_t data[8];
    ams_can_tx_class_t tx_class;
    uint16_t source_tag;
    uint32_t request_id; /* Charger shutdown identity; independent of TX order. */
} ams_can_tx_frame_t;

typedef enum {
    AMS_CAN_TX_FRAME_PENDING = 0,
    AMS_CAN_TX_FRAME_RESERVED,
    AMS_CAN_TX_FRAME_LOADED,
    AMS_CAN_TX_FRAME_ABORT_REQUESTED,
    AMS_CAN_TX_FRAME_COMPLETE,
    AMS_CAN_TX_FRAME_DISCARDED
} ams_can_tx_frame_state_t;

typedef enum {
    AMS_CAN_TX_STREAM_CRITICAL = 0,
    AMS_CAN_TX_STREAM_PROTECTED,
    AMS_CAN_TX_STREAM_DETAIL
} ams_can_tx_stream_t;

typedef struct {
    ams_can_tx_stream_t stream;
    uint32_t generation;
    uint16_t frame_index;
    uint32_t controller_epoch;
    bool valid;
} ams_can_tx_token_t;

typedef struct {
    bool valid;
    bool stale;
    uint32_t generation;
    uint32_t publish_tick;
    uint16_t frame_count;
    ams_can_tx_frame_t frames[AMS_CAN_TX_CRITICAL_MAX_FRAMES];
    ams_can_tx_frame_state_t state[AMS_CAN_TX_CRITICAL_MAX_FRAMES];
} ams_can_tx_critical_generation_t;

typedef struct {
    bool valid;
    bool stale;
    uint32_t generation;
    uint32_t publish_tick;
    uint16_t frame_count;
    uint16_t required_count;
    bool required_complete_recorded;
    ams_can_tx_frame_t frames[AMS_CAN_TX_PROTECTED_MAX_FRAMES];
    ams_can_tx_frame_state_t state[AMS_CAN_TX_PROTECTED_MAX_FRAMES];
} ams_can_tx_protected_generation_t;

typedef struct {
    bool valid;
    bool is_tuning;
    uint32_t generation;
    uint32_t publish_tick;
    uint16_t frame_count;
    ams_can_tx_frame_t frames[AMS_CAN_TX_DETAIL_MAX_FRAMES];
    ams_can_tx_frame_state_t state[AMS_CAN_TX_DETAIL_MAX_FRAMES];
} ams_can_tx_detail_generation_t;

typedef struct {
    uint32_t controller_epoch;

    ams_can_tx_critical_generation_t critical_active;
    ams_can_tx_critical_generation_t critical_pending;
    ams_can_tx_protected_generation_t protected_active;
    ams_can_tx_protected_generation_t protected_pending;
    ams_can_tx_detail_generation_t detail_active;
    ams_can_tx_detail_generation_t detail_pending;

    uint32_t critical_generated;
    uint32_t critical_superseded;
    uint32_t protected_generated;
    uint32_t protected_superseded;
    uint32_t protected_deadline_miss;
    uint32_t protected_required_complete_count;
    uint32_t protected_required_latency_last_ms;
    uint32_t protected_required_latency_max_ms;
    uint32_t protected_required_latency_over_50ms;
    uint32_t detail_generated;
    uint32_t detail_completed;
    uint32_t detail_superseded;
    uint32_t detail_tuning_shed;
    uint32_t detail_discarded_on_recovery;
    uint32_t reserve_failures;
    uint32_t unexpected_completions;
} ams_can_tx_scheduler_t;

void ams_can_tx_scheduler_init(ams_can_tx_scheduler_t *sched);
bool ams_can_generation_is_newer(uint32_t a, uint32_t b);

bool ams_can_tx_publish_critical(ams_can_tx_scheduler_t *sched,
                                 uint32_t generation,
                                 uint32_t publish_tick,
                                 const ams_can_tx_frame_t *frames,
                                 uint16_t frame_count);

/* Returns true on accepted publication. stale_generation_out identifies an
 * unfinished ACTIVE generation at the 100 ms boundary whose mailboxes need
 * aborting. Only unfinished required frames count as a deadline miss; leftover
 * advisory frames are obsolete without implying failed authority delivery. */
bool ams_can_tx_publish_protected(ams_can_tx_scheduler_t *sched,
                                  uint32_t generation,
                                  uint32_t publish_tick,
                                  const ams_can_tx_frame_t *frames,
                                  uint16_t frame_count,
                                  uint16_t required_count,
                                  uint32_t *stale_generation_out);

bool ams_can_tx_publish_detail(ams_can_tx_scheduler_t *sched,
                               uint32_t generation,
                               uint32_t publish_tick,
                               const ams_can_tx_frame_t *frames,
                               uint16_t frame_count);

/* Same storage/priority as detail. A pending base snapshot wins over tuning. */
bool ams_can_tx_publish_tuning(ams_can_tx_scheduler_t *sched,
                               uint32_t generation,
                               uint32_t publish_tick,
                               const ams_can_tx_frame_t *frames,
                               uint16_t frame_count);

bool ams_can_tx_reserve_next(ams_can_tx_scheduler_t *sched,
                             ams_can_tx_token_t *token,
                             ams_can_tx_frame_t *frame);
void ams_can_tx_load_failed(ams_can_tx_scheduler_t *sched,
                            const ams_can_tx_token_t *token);
void ams_can_tx_mark_loaded(ams_can_tx_scheduler_t *sched,
                            const ams_can_tx_token_t *token);
void ams_can_tx_mark_abort_requested(ams_can_tx_scheduler_t *sched,
                                     const ams_can_tx_token_t *token);
void ams_can_tx_mark_abort_failed(ams_can_tx_scheduler_t *sched,
                                  const ams_can_tx_token_t *token);
void ams_can_tx_mark_complete(ams_can_tx_scheduler_t *sched,
                              const ams_can_tx_token_t *token,
                              bool completed_on_wire,
                              uint32_t completion_tick);

bool ams_can_tx_generation_has_loaded(const ams_can_tx_scheduler_t *sched,
                                      ams_can_tx_stream_t stream,
                                      uint32_t generation);
bool ams_can_tx_token_requires_abort(const ams_can_tx_scheduler_t *sched,
                                     const ams_can_tx_token_t *token);

/* Call only after old hardware requests and callbacks have settled. Preserve only
 * the newest critical/protected generation and make every preserved frame
 * pending again. Detail is deliberately discarded because an incomplete
 * historical snapshot is not useful after a controller epoch change. */
void ams_can_tx_controller_epoch_reset(ams_can_tx_scheduler_t *sched);

uint16_t ams_can_tx_pending_count(const ams_can_tx_scheduler_t *sched,
                                  ams_can_tx_class_t tx_class);

#endif /* INC_EXT_DRIVERS_CAN_TX_SCHEDULER_H_ */
