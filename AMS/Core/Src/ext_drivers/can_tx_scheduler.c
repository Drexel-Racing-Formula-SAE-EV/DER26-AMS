#include "ext_drivers/can_tx_scheduler.h"

#include <string.h>
#include <limits.h>

static void sat_inc(uint32_t *value)
{
    if((value != NULL) && (*value != UINT32_MAX))
    {
        (*value)++;
    }
}

bool ams_can_generation_is_newer(uint32_t a, uint32_t b)
{
    uint32_t delta = a - b;
    return (delta != 0u) && (delta < 0x80000000u);
}

void ams_can_tx_scheduler_init(ams_can_tx_scheduler_t *sched)
{
    if(sched == NULL)
    {
        return;
    }
    memset(sched, 0, sizeof(*sched));
    sched->controller_epoch = 1u;
}

static bool states_done(const ams_can_tx_frame_state_t *state, uint16_t count)
{
    if(state == NULL)
    {
        return true;
    }
    for(uint16_t i = 0u; i < count; i++)
    {
        if((state[i] != AMS_CAN_TX_FRAME_COMPLETE) &&
           (state[i] != AMS_CAN_TX_FRAME_DISCARDED))
        {
            return false;
        }
    }
    return true;
}


static bool protected_required_done(const ams_can_tx_protected_generation_t *gen)
{
    if((gen == NULL) || !gen->valid || (gen->required_count == 0u) ||
       (gen->required_count > gen->frame_count))
    {
        return false;
    }

    /* Required authority is considered delivered only if every required
     * frame actually completed on the wire. DISCARDED is terminal for
     * bookkeeping, but it is not a successful protected delivery. */
    uint16_t required = 0u;
    for(uint16_t i = 0u; i < gen->frame_count; i++)
    {
        if(gen->frames[i].tx_class != AMS_CAN_TX_CLASS_PROTECTED_REQUIRED)
        {
            continue;
        }
        required++;
        if(gen->state[i] != AMS_CAN_TX_FRAME_COMPLETE)
        {
            return false;
        }
    }
    return required == gen->required_count;
}

static void reset_states(ams_can_tx_frame_state_t *state, uint16_t count)
{
    for(uint16_t i = 0u; i < count; i++)
    {
        state[i] = AMS_CAN_TX_FRAME_PENDING;
    }
}

static void discard_not_loaded(ams_can_tx_frame_state_t *state, uint16_t count)
{
    for(uint16_t i = 0u; i < count; i++)
    {
        /* RESERVED means a task/ISR pump has already selected the frame and
         * may be inside HAL_CAN_AddTxMessage() outside the metadata critical
         * section. It must not be discarded behind that pump. Once the HAL
         * load finishes, the stale-generation check requests an abort. */
        if(state[i] == AMS_CAN_TX_FRAME_PENDING)
        {
            state[i] = AMS_CAN_TX_FRAME_DISCARDED;
        }
    }
}

static void promote_critical(ams_can_tx_scheduler_t *sched)
{
    if((sched == NULL) || !sched->critical_active.valid ||
       !states_done(sched->critical_active.state,
                    sched->critical_active.frame_count))
    {
        return;
    }

    memset(&sched->critical_active, 0, sizeof(sched->critical_active));
    if(sched->critical_pending.valid)
    {
        sched->critical_active = sched->critical_pending;
        memset(&sched->critical_pending, 0, sizeof(sched->critical_pending));
    }
}

static void promote_protected(ams_can_tx_scheduler_t *sched)
{
    if((sched == NULL) || !sched->protected_active.valid ||
       !states_done(sched->protected_active.state,
                    sched->protected_active.frame_count))
    {
        return;
    }

    memset(&sched->protected_active, 0, sizeof(sched->protected_active));
    if(sched->protected_pending.valid)
    {
        sched->protected_active = sched->protected_pending;
        memset(&sched->protected_pending, 0, sizeof(sched->protected_pending));
    }
}

static void promote_detail(ams_can_tx_scheduler_t *sched)
{
    if((sched == NULL) || !sched->detail_active.valid ||
       !states_done(sched->detail_active.state, sched->detail_active.frame_count))
    {
        return;
    }

    sat_inc(&sched->detail_completed);
    memset(&sched->detail_active, 0, sizeof(sched->detail_active));
    if(sched->detail_pending.valid)
    {
        sched->detail_active = sched->detail_pending;
        memset(&sched->detail_pending, 0, sizeof(sched->detail_pending));
    }
}

static void promote_all(ams_can_tx_scheduler_t *sched)
{
    promote_critical(sched);
    promote_protected(sched);
    promote_detail(sched);
}

static bool generation_has_hardware_or_reserved(
    const ams_can_tx_frame_state_t *state, uint16_t count)
{
    for(uint16_t i = 0u; i < count; i++)
    {
        if((state[i] == AMS_CAN_TX_FRAME_RESERVED) ||
           (state[i] == AMS_CAN_TX_FRAME_LOADED) ||
           (state[i] == AMS_CAN_TX_FRAME_ABORT_REQUESTED))
        {
            return true;
        }
    }
    return false;
}

bool ams_can_tx_publish_critical(ams_can_tx_scheduler_t *sched,
                                 uint32_t generation,
                                 uint32_t publish_tick,
                                 const ams_can_tx_frame_t *frames,
                                 uint16_t frame_count)
{
    ams_can_tx_critical_generation_t next;

    if((sched == NULL) || (frames == NULL) || (frame_count == 0u) ||
       (frame_count > AMS_CAN_TX_CRITICAL_MAX_FRAMES))
    {
        return false;
    }

    memset(&next, 0, sizeof(next));
    next.valid = true;
    next.generation = generation;
    next.publish_tick = publish_tick;
    next.frame_count = frame_count;
    memcpy(next.frames, frames, (size_t)frame_count * sizeof(frames[0]));
    reset_states(next.state, frame_count);
    sat_inc(&sched->critical_generated);

    promote_critical(sched);
    if(!sched->critical_active.valid)
    {
        sched->critical_active = next;
    }
    else if(!generation_has_hardware_or_reserved(
                sched->critical_active.state,
                sched->critical_active.frame_count))
    {
        /* Critical commands are latest-value too. If the previous command was
         * never accepted into a bxCAN mailbox (for example HAL load failure),
         * do not faithfully transmit it later after safety state changed. */
        sat_inc(&sched->critical_superseded);
        sched->critical_active = next;
        memset(&sched->critical_pending, 0, sizeof(sched->critical_pending));
    }
    else
    {
        /* Critical commands are latest-value, including commands already in
         * hardware. The transport aborts obsolete requests; a completion that
         * wins that race is still accounted to its original request identity. */
        sched->critical_active.stale = true;
        discard_not_loaded(sched->critical_active.state,
                           sched->critical_active.frame_count);
        if(sched->critical_pending.valid)
        {
            sat_inc(&sched->critical_superseded);
        }
        sched->critical_pending = next;
    }
    return true;
}

bool ams_can_tx_publish_protected(ams_can_tx_scheduler_t *sched,
                                  uint32_t generation,
                                  uint32_t publish_tick,
                                  const ams_can_tx_frame_t *frames,
                                  uint16_t frame_count,
                                  uint16_t required_count,
                                  uint32_t *stale_generation_out)
{
    ams_can_tx_protected_generation_t next;
    bool stale_boundary = false;

    if(stale_generation_out != NULL)
    {
        *stale_generation_out = 0u;
    }
    if((sched == NULL) || (frames == NULL) || (frame_count == 0u) ||
       (frame_count > AMS_CAN_TX_PROTECTED_MAX_FRAMES) ||
       (required_count == 0u) || (required_count > frame_count))
    {
        return false;
    }

    uint16_t actual_required = 0u;
    for(uint16_t i = 0u; i < frame_count; i++)
    {
        if(frames[i].tx_class == AMS_CAN_TX_CLASS_PROTECTED_REQUIRED)
        {
            actual_required++;
        }
        else if(frames[i].tx_class != AMS_CAN_TX_CLASS_PROTECTED_ADVISORY)
        {
            return false;
        }
    }
    if(actual_required != required_count)
    {
        return false;
    }

    memset(&next, 0, sizeof(next));
    next.valid = true;
    next.generation = generation;
    next.publish_tick = publish_tick;
    next.frame_count = frame_count;
    next.required_count = required_count;
    memcpy(next.frames, frames, (size_t)frame_count * sizeof(frames[0]));
    reset_states(next.state, frame_count);
    sat_inc(&sched->protected_generated);

    promote_protected(sched);
    if(!sched->protected_active.valid)
    {
        sched->protected_active = next;
        return true;
    }

    /* ACTIVE required traffic is expected to finish in a few milliseconds.
     * At the next 100 ms publication boundary there are two distinct cases:
     *
     *  1) required frames are still incomplete: this is a real protected
     *     deadline miss. Discard software-pending old frames and request
     *     aborts for old hardware-resident frames.
     *  2) required frames completed but advisory traffic remains: advisory
     *     data is simply obsolete. Supersede it without calling that a safety
     *     deadline miss, then move on to the newest generation.
     *
     * Healthy ACTIVE generations are never routinely aborted merely because
     * a newer generation was published. */
    if((uint32_t)(publish_tick - sched->protected_active.publish_tick) >=
       AMS_CAN_TX_PROTECTED_PERIOD_MS)
    {
        stale_boundary = true;
        sched->protected_active.stale = true;
        if(!protected_required_done(&sched->protected_active))
        {
            sat_inc(&sched->protected_deadline_miss);
        }
        discard_not_loaded(sched->protected_active.state,
                           sched->protected_active.frame_count);
        if(stale_generation_out != NULL)
        {
            *stale_generation_out = sched->protected_active.generation;
        }
    }

    if(sched->protected_pending.valid)
    {
        sat_inc(&sched->protected_superseded);
    }
    sched->protected_pending = next;

    /* If the stale ACTIVE had no hardware-resident frames, discarding its
     * software-pending frames completed it immediately and the latest PENDING
     * generation can become ACTIVE now. */
    promote_protected(sched);
    (void)stale_boundary;
    return true;
}

static bool publish_detail(ams_can_tx_scheduler_t *sched,
                               uint32_t generation,
                               uint32_t publish_tick,
                               const ams_can_tx_frame_t *frames,
                               uint16_t frame_count,
                               bool is_tuning)
{
    ams_can_tx_detail_generation_t *next;

    if((sched == NULL) || (frames == NULL) || (frame_count == 0u) ||
       (frame_count > AMS_CAN_TX_DETAIL_MAX_FRAMES))
    {
        return false;
    }

    sat_inc(&sched->detail_generated);

    promote_detail(sched);
    if(!sched->detail_active.valid)
    {
        next = &sched->detail_active;
    }
    else
    {
        if(sched->detail_pending.valid)
        {
            if(is_tuning && !sched->detail_pending.is_tuning)
            {
                /* Intentional telemetry shedding, not a transport fault. */
                sat_inc(&sched->detail_tuning_shed);
                return true;
            }
            sat_inc(&sched->detail_superseded);
        }
        next = &sched->detail_pending;
    }
    /* Caller serializes publication against ISR promotion. Write directly to
     * the owned slot: a full-generation local exceeds the CAN task stack. */
    next->valid = true;
    next->is_tuning = is_tuning;
    next->generation = generation;
    next->publish_tick = publish_tick;
    next->frame_count = frame_count;
    memcpy(next->frames, frames, (size_t)frame_count * sizeof(frames[0]));
    reset_states(next->state, frame_count);
    return true;
}

bool ams_can_tx_publish_detail(ams_can_tx_scheduler_t *sched,
                               uint32_t generation, uint32_t publish_tick,
                               const ams_can_tx_frame_t *frames,
                               uint16_t frame_count)
{
    return publish_detail(sched, generation, publish_tick, frames, frame_count, false);
}

bool ams_can_tx_publish_tuning(ams_can_tx_scheduler_t *sched,
                               uint32_t generation, uint32_t publish_tick,
                               const ams_can_tx_frame_t *frames,
                               uint16_t frame_count)
{
    return publish_detail(sched, generation, publish_tick, frames, frame_count, true);
}

static bool reserve_from_generation(ams_can_tx_frame_t *frames,
                                    ams_can_tx_frame_state_t *state,
                                    uint16_t first,
                                    uint16_t last,
                                    ams_can_tx_stream_t stream,
                                    uint32_t generation,
                                    uint32_t epoch,
                                    ams_can_tx_token_t *token,
                                    ams_can_tx_frame_t *frame)
{
    for(uint16_t i = first; i < last; i++)
    {
        if(state[i] == AMS_CAN_TX_FRAME_PENDING)
        {
            state[i] = AMS_CAN_TX_FRAME_RESERVED;
            token->stream = stream;
            token->generation = generation;
            token->frame_index = i;
            token->controller_epoch = epoch;
            token->valid = true;
            *frame = frames[i];
            return true;
        }
    }
    return false;
}


static bool reserve_class_from_generation(ams_can_tx_frame_t *frames,
                                          ams_can_tx_frame_state_t *state,
                                          uint16_t count,
                                          ams_can_tx_class_t wanted_class,
                                          ams_can_tx_stream_t stream,
                                          uint32_t generation,
                                          uint32_t epoch,
                                          ams_can_tx_token_t *token,
                                          ams_can_tx_frame_t *frame)
{
    for(uint16_t i = 0u; i < count; i++)
    {
        if((frames[i].tx_class == wanted_class) &&
           (state[i] == AMS_CAN_TX_FRAME_PENDING))
        {
            state[i] = AMS_CAN_TX_FRAME_RESERVED;
            token->stream = stream;
            token->generation = generation;
            token->frame_index = i;
            token->controller_epoch = epoch;
            token->valid = true;
            *frame = frames[i];
            return true;
        }
    }
    return false;
}
bool ams_can_tx_reserve_next(ams_can_tx_scheduler_t *sched,
                             ams_can_tx_token_t *token,
                             ams_can_tx_frame_t *frame)
{
    if((sched == NULL) || (token == NULL) || (frame == NULL))
    {
        return false;
    }

    promote_all(sched);
    memset(token, 0, sizeof(*token));

    if(sched->critical_active.valid &&
       reserve_from_generation(sched->critical_active.frames,
                               sched->critical_active.state,
                               0u, sched->critical_active.frame_count,
                               AMS_CAN_TX_STREAM_CRITICAL,
                               sched->critical_active.generation,
                               sched->controller_epoch, token, frame))
    {
        return true;
    }

    if(sched->protected_active.valid)
    {
        if(reserve_class_from_generation(
                sched->protected_active.frames,
                sched->protected_active.state,
                sched->protected_active.frame_count,
                AMS_CAN_TX_CLASS_PROTECTED_REQUIRED,
                AMS_CAN_TX_STREAM_PROTECTED,
                sched->protected_active.generation,
                sched->controller_epoch, token, frame))
        {
            return true;
        }
        if(reserve_class_from_generation(
                sched->protected_active.frames,
                sched->protected_active.state,
                sched->protected_active.frame_count,
                AMS_CAN_TX_CLASS_PROTECTED_ADVISORY,
                AMS_CAN_TX_STREAM_PROTECTED,
                sched->protected_active.generation,
                sched->controller_epoch, token, frame))
        {
            return true;
        }
    }

    if(sched->detail_active.valid &&
       reserve_from_generation(sched->detail_active.frames,
                               sched->detail_active.state,
                               0u, sched->detail_active.frame_count,
                               AMS_CAN_TX_STREAM_DETAIL,
                               sched->detail_active.generation,
                               sched->controller_epoch, token, frame))
    {
        return true;
    }

    return false;
}

static ams_can_tx_frame_state_t *find_state(ams_can_tx_scheduler_t *sched,
                                            const ams_can_tx_token_t *token)
{
    if((sched == NULL) || (token == NULL) || !token->valid ||
       (token->controller_epoch != sched->controller_epoch))
    {
        return NULL;
    }

    switch(token->stream)
    {
    case AMS_CAN_TX_STREAM_CRITICAL:
        if(sched->critical_active.valid &&
           (sched->critical_active.generation == token->generation) &&
           (token->frame_index < sched->critical_active.frame_count))
        {
            return &sched->critical_active.state[token->frame_index];
        }
        break;
    case AMS_CAN_TX_STREAM_PROTECTED:
        if(sched->protected_active.valid &&
           (sched->protected_active.generation == token->generation) &&
           (token->frame_index < sched->protected_active.frame_count))
        {
            return &sched->protected_active.state[token->frame_index];
        }
        break;
    case AMS_CAN_TX_STREAM_DETAIL:
        if(sched->detail_active.valid &&
           (sched->detail_active.generation == token->generation) &&
           (token->frame_index < sched->detail_active.frame_count))
        {
            return &sched->detail_active.state[token->frame_index];
        }
        break;
    default:
        break;
    }
    return NULL;
}

void ams_can_tx_load_failed(ams_can_tx_scheduler_t *sched,
                            const ams_can_tx_token_t *token)
{
    ams_can_tx_frame_state_t *state = find_state(sched, token);
    if(state == NULL)
    {
        sat_inc((sched != NULL) ? &sched->reserve_failures : NULL);
        return;
    }
    if(*state == AMS_CAN_TX_FRAME_RESERVED)
    {
        *state = ams_can_tx_token_requires_abort(sched, token) ?
            AMS_CAN_TX_FRAME_DISCARDED : AMS_CAN_TX_FRAME_PENDING;
        promote_all(sched);
    }
}

void ams_can_tx_mark_loaded(ams_can_tx_scheduler_t *sched,
                            const ams_can_tx_token_t *token)
{
    ams_can_tx_frame_state_t *state = find_state(sched, token);
    if((state != NULL) && (*state == AMS_CAN_TX_FRAME_RESERVED))
    {
        *state = AMS_CAN_TX_FRAME_LOADED;
    }
}

void ams_can_tx_mark_abort_requested(ams_can_tx_scheduler_t *sched,
                                     const ams_can_tx_token_t *token)
{
    ams_can_tx_frame_state_t *state = find_state(sched, token);
    if((state != NULL) && (*state == AMS_CAN_TX_FRAME_LOADED))
    {
        *state = AMS_CAN_TX_FRAME_ABORT_REQUESTED;
    }
}

void ams_can_tx_mark_abort_failed(ams_can_tx_scheduler_t *sched,
                                  const ams_can_tx_token_t *token)
{
    ams_can_tx_frame_state_t *state = find_state(sched, token);
    if((state != NULL) && (*state == AMS_CAN_TX_FRAME_ABORT_REQUESTED))
    {
        *state = AMS_CAN_TX_FRAME_LOADED;
    }
}

void ams_can_tx_mark_complete(ams_can_tx_scheduler_t *sched,
                              const ams_can_tx_token_t *token,
                              bool completed_on_wire,
                              uint32_t completion_tick)
{
    ams_can_tx_frame_state_t *state = find_state(sched, token);

    if(state == NULL)
    {
        if(sched != NULL)
        {
            sat_inc(&sched->unexpected_completions);
        }
        return;
    }

    if((*state == AMS_CAN_TX_FRAME_LOADED) ||
       (*state == AMS_CAN_TX_FRAME_ABORT_REQUESTED))
    {
        /* An abort callback means the frame did not complete on the wire. It
         * still settles this generation's bookkeeping, but retain that fact
         * as DISCARDED rather than falsely calling it COMPLETE. If completion
         * won the abort race, completed_on_wire is true and the state is
         * COMPLETE. */
        *state = completed_on_wire ? AMS_CAN_TX_FRAME_COMPLETE :
                                     AMS_CAN_TX_FRAME_DISCARDED;

        if((token->stream == AMS_CAN_TX_STREAM_PROTECTED) &&
           sched->protected_active.valid &&
           (sched->protected_active.generation == token->generation) &&
           !sched->protected_active.required_complete_recorded &&
           protected_required_done(&sched->protected_active))
        {
            uint32_t latency = completion_tick -
                               sched->protected_active.publish_tick;
            sched->protected_active.required_complete_recorded = true;
            sat_inc(&sched->protected_required_complete_count);
            sched->protected_required_latency_last_ms = latency;
            if(latency > sched->protected_required_latency_max_ms)
            {
                sched->protected_required_latency_max_ms = latency;
            }
            if(latency > 50u)
            {
                sat_inc(&sched->protected_required_latency_over_50ms);
            }
        }

        promote_all(sched);
    }
    else
    {
        sat_inc(&sched->unexpected_completions);
    }
}

bool ams_can_tx_token_requires_abort(const ams_can_tx_scheduler_t *sched,
                                     const ams_can_tx_token_t *token)
{
    if((sched == NULL) || (token == NULL) || !token->valid ||
       (token->controller_epoch != sched->controller_epoch))
    {
        return false;
    }

    if(token->stream == AMS_CAN_TX_STREAM_CRITICAL)
    {
        return sched->critical_active.valid && sched->critical_active.stale &&
            (sched->critical_active.generation == token->generation) &&
            (token->frame_index < sched->critical_active.frame_count);
    }
    return (token->stream == AMS_CAN_TX_STREAM_PROTECTED) &&
        sched->protected_active.valid && sched->protected_active.stale &&
        (sched->protected_active.generation == token->generation) &&
        (token->frame_index < sched->protected_active.frame_count);
}

bool ams_can_tx_generation_has_loaded(const ams_can_tx_scheduler_t *sched,
                                      ams_can_tx_stream_t stream,
                                      uint32_t generation)
{
    if(sched == NULL)
    {
        return false;
    }

    const ams_can_tx_frame_state_t *state = NULL;
    uint16_t count = 0u;
    switch(stream)
    {
    case AMS_CAN_TX_STREAM_CRITICAL:
        if(sched->critical_active.valid &&
           (sched->critical_active.generation == generation))
        {
            state = sched->critical_active.state;
            count = sched->critical_active.frame_count;
        }
        break;
    case AMS_CAN_TX_STREAM_PROTECTED:
        if(sched->protected_active.valid &&
           (sched->protected_active.generation == generation))
        {
            state = sched->protected_active.state;
            count = sched->protected_active.frame_count;
        }
        break;
    case AMS_CAN_TX_STREAM_DETAIL:
        if(sched->detail_active.valid &&
           (sched->detail_active.generation == generation))
        {
            state = sched->detail_active.state;
            count = sched->detail_active.frame_count;
        }
        break;
    default:
        break;
    }

    if(state == NULL)
    {
        return false;
    }
    for(uint16_t i = 0u; i < count; i++)
    {
        if((state[i] == AMS_CAN_TX_FRAME_LOADED) ||
           (state[i] == AMS_CAN_TX_FRAME_ABORT_REQUESTED))
        {
            return true;
        }
    }
    return false;
}

static void protected_reset_to_pending(ams_can_tx_protected_generation_t *gen)
{
    if((gen == NULL) || !gen->valid)
    {
        return;
    }
    gen->stale = false;
    gen->required_complete_recorded = false;
    reset_states(gen->state, gen->frame_count);
}

static void critical_reset_to_pending(ams_can_tx_critical_generation_t *gen)
{
    if((gen == NULL) || !gen->valid)
    {
        return;
    }
    gen->stale = false;
    reset_states(gen->state, gen->frame_count);
}

void ams_can_tx_controller_epoch_reset(ams_can_tx_scheduler_t *sched)
{
    if(sched == NULL)
    {
        return;
    }

    sched->controller_epoch++;
    if(sched->controller_epoch == 0u)
    {
        sched->controller_epoch = 1u;
    }

    /* PENDING was published after ACTIVE. Tokens are identities, not clocks. */
    if(sched->critical_pending.valid)
    {
        sched->critical_active = sched->critical_pending;
    }
    memset(&sched->critical_pending, 0, sizeof(sched->critical_pending));
    critical_reset_to_pending(&sched->critical_active);

    if(sched->protected_pending.valid)
    {
        sched->protected_active = sched->protected_pending;
    }
    memset(&sched->protected_pending, 0, sizeof(sched->protected_pending));
    protected_reset_to_pending(&sched->protected_active);

    if(sched->detail_active.valid || sched->detail_pending.valid)
    {
        sat_inc(&sched->detail_discarded_on_recovery);
    }
    memset(&sched->detail_active, 0, sizeof(sched->detail_active));
    memset(&sched->detail_pending, 0, sizeof(sched->detail_pending));
}

static uint16_t count_class_in_generation(const ams_can_tx_frame_t *frames,
                                          const ams_can_tx_frame_state_t *state,
                                          uint16_t count,
                                          ams_can_tx_class_t tx_class)
{
    uint16_t result = 0u;
    for(uint16_t i = 0u; i < count; i++)
    {
        if((frames[i].tx_class == tx_class) &&
           ((state[i] == AMS_CAN_TX_FRAME_PENDING) ||
            (state[i] == AMS_CAN_TX_FRAME_RESERVED)))
        {
            result++;
        }
    }
    return result;
}

uint16_t ams_can_tx_pending_count(const ams_can_tx_scheduler_t *sched,
                                  ams_can_tx_class_t tx_class)
{
    uint16_t count = 0u;
    if(sched == NULL)
    {
        return 0u;
    }

    if(sched->critical_active.valid)
    {
        count = (uint16_t)(count + count_class_in_generation(
            sched->critical_active.frames, sched->critical_active.state,
            sched->critical_active.frame_count, tx_class));
    }
    if(sched->protected_active.valid)
    {
        count = (uint16_t)(count + count_class_in_generation(
            sched->protected_active.frames, sched->protected_active.state,
            sched->protected_active.frame_count, tx_class));
    }
    if(sched->detail_active.valid)
    {
        count = (uint16_t)(count + count_class_in_generation(
            sched->detail_active.frames, sched->detail_active.state,
            sched->detail_active.frame_count, tx_class));
    }
    return count;
}
