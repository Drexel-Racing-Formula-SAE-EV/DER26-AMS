#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ext_drivers/can_tx_scheduler.h"

#define CHECK(x) do { if(!(x)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); \
} } while(0)

static ams_can_tx_frame_t frame(uint32_t id, ams_can_tx_class_t cls)
{
    ams_can_tx_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    f.ide = 0u;
    f.dlc = 8u;
    f.tx_class = cls;
    f.data[0] = (uint8_t)id;
    return f;
}

static void complete_all(ams_can_tx_scheduler_t *s)
{
    ams_can_tx_token_t tok;
    ams_can_tx_frame_t f;
    unsigned guard = 0u;
    while(ams_can_tx_reserve_next(s, &tok, &f))
    {
        ams_can_tx_mark_loaded(s, &tok);
        ams_can_tx_mark_complete(s, &tok, true, 0u);
        CHECK(++guard < 1000u);
    }
}

static void test_generation_wrap(void)
{
    CHECK(ams_can_generation_is_newer(2u, 1u));
    CHECK(!ams_can_generation_is_newer(1u, 1u));
    CHECK(ams_can_generation_is_newer(1u, UINT32_MAX));
    CHECK(ams_can_generation_is_newer(0u, UINT32_MAX));
    CHECK(!ams_can_generation_is_newer(0x80000000u, 0u));
    CHECK(!ams_can_generation_is_newer(0u, 0x80000000u));
}

static void test_priority_order(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);

    ams_can_tx_frame_t crit[1] = {frame(0x100u, AMS_CAN_TX_CLASS_CRITICAL)};
    ams_can_tx_frame_t prot[3] = {
        frame(0x689u, AMS_CAN_TX_CLASS_PROTECTED_ADVISORY),
        frame(0x680u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED),
        frame(0x681u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED),
    };
    ams_can_tx_frame_t det[1] = {frame(0x690u, AMS_CAN_TX_CLASS_DETAIL)};
    CHECK(ams_can_tx_publish_detail(&s, 10u, 0u, det, 1u));
    CHECK(ams_can_tx_publish_protected(&s, 11u, 0u, prot, 3u, 2u, NULL));
    CHECK(ams_can_tx_publish_critical(&s, 12u, 0u, crit, 1u));

    ams_can_tx_token_t tok;
    ams_can_tx_frame_t out;
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out)); CHECK(out.tx_class == AMS_CAN_TX_CLASS_CRITICAL);
    ams_can_tx_mark_loaded(&s, &tok); ams_can_tx_mark_complete(&s, &tok, true, 0u);
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out)); CHECK(out.tx_class == AMS_CAN_TX_CLASS_PROTECTED_REQUIRED);
    ams_can_tx_mark_loaded(&s, &tok); ams_can_tx_mark_complete(&s, &tok, true, 0u);
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out)); CHECK(out.tx_class == AMS_CAN_TX_CLASS_PROTECTED_REQUIRED);
    ams_can_tx_mark_loaded(&s, &tok); ams_can_tx_mark_complete(&s, &tok, true, 0u);
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out)); CHECK(out.tx_class == AMS_CAN_TX_CLASS_PROTECTED_ADVISORY);
    ams_can_tx_mark_loaded(&s, &tok); ams_can_tx_mark_complete(&s, &tok, true, 0u);
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out)); CHECK(out.tx_class == AMS_CAN_TX_CLASS_DETAIL);
}

static void test_load_failure_restores_pending(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t p[1] = {frame(0x680u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED)};
    CHECK(ams_can_tx_publish_protected(&s, 1u, 0u, p, 1u, 1u, NULL));
    ams_can_tx_token_t a, b;
    ams_can_tx_frame_t out;
    CHECK(ams_can_tx_reserve_next(&s, &a, &out));
    ams_can_tx_load_failed(&s, &a);
    CHECK(ams_can_tx_reserve_next(&s, &b, &out));
    CHECK(b.generation == a.generation && b.frame_index == a.frame_index);
}

static void test_protected_normal_pending_no_abort(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t p[2] = {
        frame(0x680u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED),
        frame(0x681u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED)
    };
    uint32_t stale = 0u;
    CHECK(ams_can_tx_publish_protected(&s, 1u, 0u, p, 2u, 2u, &stale));
    ams_can_tx_token_t tok;
    ams_can_tx_frame_t out;
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out));
    ams_can_tx_mark_loaded(&s, &tok);
    CHECK(ams_can_tx_publish_protected(&s, 2u, 99u, p, 2u, 2u, &stale));
    CHECK(stale == 0u);
    CHECK(s.protected_deadline_miss == 0u);
    CHECK(!ams_can_tx_token_requires_abort(&s, &tok));
    ams_can_tx_mark_complete(&s, &tok, true, 0u);
    complete_all(&s);
}

static void test_protected_boundary_abort_is_pathological(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t p[3] = {
        frame(0x680u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED),
        frame(0x681u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED),
        frame(0x689u, AMS_CAN_TX_CLASS_PROTECTED_ADVISORY)
    };
    uint32_t stale = 0u;
    CHECK(ams_can_tx_publish_protected(&s, 10u, 0u, p, 3u, 2u, &stale));
    ams_can_tx_token_t tok;
    ams_can_tx_frame_t out;
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out));
    ams_can_tx_mark_loaded(&s, &tok);
    CHECK(ams_can_tx_publish_protected(&s, 11u, 100u, p, 3u, 2u, &stale));
    CHECK(stale == 10u);
    CHECK(s.protected_deadline_miss == 1u);
    CHECK(ams_can_tx_token_requires_abort(&s, &tok));
    ams_can_tx_mark_abort_requested(&s, &tok);
    ams_can_tx_mark_complete(&s, &tok, false, 0u);
    CHECK(s.protected_active.valid);
    CHECK(s.protected_active.generation == 11u);
}

static void test_advisory_supersession_not_deadline_miss(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t p[2] = {
        frame(0x680u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED),
        frame(0x689u, AMS_CAN_TX_CLASS_PROTECTED_ADVISORY)
    };
    uint32_t stale = 0u;
    CHECK(ams_can_tx_publish_protected(&s, 1u, 0u, p, 2u, 1u, &stale));
    ams_can_tx_token_t req, adv;
    ams_can_tx_frame_t out;
    CHECK(ams_can_tx_reserve_next(&s, &req, &out));
    ams_can_tx_mark_loaded(&s, &req); ams_can_tx_mark_complete(&s, &req, true, 0u);
    CHECK(ams_can_tx_reserve_next(&s, &adv, &out));
    ams_can_tx_mark_loaded(&s, &adv);
    CHECK(ams_can_tx_publish_protected(&s, 2u, 100u, p, 2u, 1u, &stale));
    CHECK(stale == 1u);
    CHECK(s.protected_deadline_miss == 0u);
    CHECK(ams_can_tx_token_requires_abort(&s, &adv));
    ams_can_tx_mark_abort_requested(&s, &adv);
    ams_can_tx_mark_complete(&s, &adv, false, 0u);
    CHECK(s.protected_active.generation == 2u);
}


static void test_protected_required_latency_metrics(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t p[2] = {
        frame(0x680u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED),
        frame(0x681u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED)
    };
    ams_can_tx_token_t tok;
    ams_can_tx_frame_t out;

    CHECK(ams_can_tx_publish_protected(&s, 1u, 100u, p, 2u, 2u, NULL));
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out));
    ams_can_tx_mark_loaded(&s, &tok);
    ams_can_tx_mark_complete(&s, &tok, true, 105u);
    CHECK(s.protected_required_complete_count == 0u);
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out));
    ams_can_tx_mark_loaded(&s, &tok);
    ams_can_tx_mark_complete(&s, &tok, true, 120u);
    CHECK(s.protected_required_complete_count == 1u);
    CHECK(s.protected_required_latency_last_ms == 20u);
    CHECK(s.protected_required_latency_max_ms == 20u);
    CHECK(s.protected_required_latency_over_50ms == 0u);

    CHECK(ams_can_tx_publish_protected(&s, 2u, 200u, p, 2u, 2u, NULL));
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out));
    ams_can_tx_mark_loaded(&s, &tok);
    ams_can_tx_mark_complete(&s, &tok, true, 205u);
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out));
    ams_can_tx_mark_loaded(&s, &tok);
    ams_can_tx_mark_complete(&s, &tok, true, 260u);
    CHECK(s.protected_required_complete_count == 2u);
    CHECK(s.protected_required_latency_last_ms == 60u);
    CHECK(s.protected_required_latency_max_ms == 60u);
    CHECK(s.protected_required_latency_over_50ms == 1u);
}

static void test_abort_completion_races(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t p[1] = {frame(0x680u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED)};
    ams_can_tx_token_t tok;
    ams_can_tx_frame_t out;
    CHECK(ams_can_tx_publish_protected(&s, 1u, 0u, p, 1u, 1u, NULL));
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out));
    ams_can_tx_mark_loaded(&s, &tok);
    ams_can_tx_mark_abort_requested(&s, &tok);
    ams_can_tx_mark_complete(&s, &tok, true, 0u); /* completion won race */
    CHECK(!s.protected_active.valid);

    CHECK(ams_can_tx_publish_protected(&s, 2u, 0u, p, 1u, 1u, NULL));
    CHECK(ams_can_tx_reserve_next(&s, &tok, &out));
    ams_can_tx_mark_loaded(&s, &tok);
    ams_can_tx_mark_abort_requested(&s, &tok);
    ams_can_tx_mark_complete(&s, &tok, false, 0u); /* abort won */
    CHECK(!s.protected_active.valid);
}

static void test_detail_active_runs_to_completion(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t d[3] = {
        frame(0x690u, AMS_CAN_TX_CLASS_DETAIL),
        frame(0x691u, AMS_CAN_TX_CLASS_DETAIL),
        frame(0x692u, AMS_CAN_TX_CLASS_DETAIL)
    };
    CHECK(ams_can_tx_publish_detail(&s, 100u, 0u, d, 3u));
    ams_can_tx_token_t first;
    ams_can_tx_frame_t out;
    CHECK(ams_can_tx_reserve_next(&s, &first, &out));
    ams_can_tx_mark_loaded(&s, &first);
    CHECK(ams_can_tx_publish_detail(&s, 101u, 10u, d, 3u));
    CHECK(ams_can_tx_publish_detail(&s, 102u, 20u, d, 3u));
    CHECK(s.detail_active.generation == 100u);
    CHECK(s.detail_pending.generation == 102u);
    CHECK(s.detail_superseded == 1u);
    ams_can_tx_mark_complete(&s, &first, true, 0u);
    complete_all(&s);
    CHECK(s.detail_completed == 2u); /* 100 and newest 102; 101 superseded */
}

static void test_epoch_reset_preserves_newest_authority_discards_detail(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t p[1] = {frame(0x680u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED)};
    ams_can_tx_frame_t d[1] = {frame(0x690u, AMS_CAN_TX_CLASS_DETAIL)};
    CHECK(ams_can_tx_publish_protected(&s, 1u, 0u, p, 1u, 1u, NULL));
    ams_can_tx_token_t oldtok;
    ams_can_tx_frame_t out;
    CHECK(ams_can_tx_reserve_next(&s, &oldtok, &out));
    ams_can_tx_mark_loaded(&s, &oldtok);
    CHECK(ams_can_tx_publish_protected(&s, 2u, 20u, p, 1u, 1u, NULL));
    CHECK(ams_can_tx_publish_detail(&s, 3u, 20u, d, 1u));
    uint32_t old_epoch = s.controller_epoch;
    ams_can_tx_controller_epoch_reset(&s);
    CHECK(s.controller_epoch != old_epoch);
    CHECK(s.protected_active.valid && s.protected_active.generation == 2u);
    CHECK(!s.protected_pending.valid);
    CHECK(!s.detail_active.valid && !s.detail_pending.valid);
    CHECK(s.detail_discarded_on_recovery == 1u);
    uint32_t unexpected = s.unexpected_completions;
    ams_can_tx_mark_complete(&s, &oldtok, true, 0u); /* stale callback token */
    CHECK(s.unexpected_completions == unexpected + 1u);
}

static void test_capacity_rejection(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t d[AMS_CAN_TX_DETAIL_MAX_FRAMES + 1u];
    memset(d, 0, sizeof(d));
    CHECK(!ams_can_tx_publish_detail(&s, 1u, 0u, d,
                                     AMS_CAN_TX_DETAIL_MAX_FRAMES + 1u));
}

static uint32_t lcg(uint32_t *x)
{
    *x = (*x * 1664525u) + 1013904223u;
    return *x;
}

static void test_randomized_transition_stress(void)
{
    ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t p[3] = {
        frame(0x680u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED),
        frame(0x681u, AMS_CAN_TX_CLASS_PROTECTED_REQUIRED),
        frame(0x689u, AMS_CAN_TX_CLASS_PROTECTED_ADVISORY)
    };
    ams_can_tx_frame_t d[4] = {
        frame(0x690u, AMS_CAN_TX_CLASS_DETAIL), frame(0x691u, AMS_CAN_TX_CLASS_DETAIL),
        frame(0x692u, AMS_CAN_TX_CLASS_DETAIL), frame(0x693u, AMS_CAN_TX_CLASS_DETAIL)
    };
    uint32_t rnd = 0x26C0FFEEu;
    uint32_t gen = 1u;
    uint32_t tick = 0u;
    ams_can_tx_token_t outstanding[3];
    bool used[3] = {false,false,false};

    for(unsigned step = 0u; step < 100000u; step++)
    {
        uint32_t r = lcg(&rnd);
        tick += (r & 3u);
        switch((r >> 4u) % 7u)
        {
        case 0:
            (void)ams_can_tx_publish_protected(&s, ++gen, tick, p, 3u, 2u, NULL);
            break;
        case 1:
            (void)ams_can_tx_publish_detail(&s, ++gen, tick, d, 4u);
            break;
        case 2:
            for(unsigned i=0;i<3u;i++) if(!used[i]) {
                ams_can_tx_frame_t out;
                if(ams_can_tx_reserve_next(&s,&outstanding[i],&out)) {
                    ams_can_tx_mark_loaded(&s,&outstanding[i]); used[i]=true;
                }
                break;
            }
            break;
        case 3:
            for(unsigned i=0;i<3u;i++) if(used[i]) {
                ams_can_tx_mark_complete(&s,&outstanding[i], true, 0u); used[i]=false; break;
            }
            break;
        case 4:
            for(unsigned i=0;i<3u;i++) if(used[i] && ams_can_tx_token_requires_abort(&s,&outstanding[i])) {
                ams_can_tx_mark_abort_requested(&s,&outstanding[i]);
                ams_can_tx_mark_complete(&s,&outstanding[i], false, 0u); used[i]=false; break;
            }
            break;
        case 5:
            if((r & 0x3FFu)==0u) {
                ams_can_tx_controller_epoch_reset(&s);
                memset(used,0,sizeof(used));
            }
            break;
        default:
            break;
        }
        CHECK(s.critical_active.frame_count <= AMS_CAN_TX_CRITICAL_MAX_FRAMES);
        CHECK(s.protected_active.frame_count <= AMS_CAN_TX_PROTECTED_MAX_FRAMES);
        CHECK(s.detail_active.frame_count <= AMS_CAN_TX_DETAIL_MAX_FRAMES);
    }
}

int main(void)
{
    test_generation_wrap();
    test_priority_order();
    test_load_failure_restores_pending();
    test_protected_normal_pending_no_abort();
    test_protected_boundary_abort_is_pathological();
    test_advisory_supersession_not_deadline_miss();
    test_protected_required_latency_metrics();
    test_abort_completion_races();
    test_detail_active_runs_to_completion();
    test_epoch_reset_preserves_newest_authority_discards_detail();
    test_capacity_rejection();
    test_randomized_transition_stress();
    puts("PASS DER26 CAN V4 TX scheduler unit/stress tests");
    return 0;
}
