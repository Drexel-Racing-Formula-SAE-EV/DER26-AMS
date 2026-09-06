/* Focused September 2026 CAN fixes. Register-backed mock, real transport and
 * scheduler. The mock holds completions until IRQ dispatch and uses the same
 * cached-TSR callback order as the bundled HAL. No battery/MiL scenarios. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app.h"
#include "ext_drivers/ams_safety.h"

#define CHECK(x) do { if(!(x)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); \
} } while(0)

app_data_t app;
static CAN_TypeDef regs;
static CAN_HandleTypeDef hcan;
static uint32_t tick, abort_mask, hal_error;
static void (*after_first_callback)(void);
static void (*on_state_read)(void);
static void (*critical_exit_hook)(void);
static uint32_t injected_busoff_tick;
static const uint32_t empty_mask = CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2;

uint32_t osKernelGetTickCount(void) { return tick; }
void vPortEnterCritical(void) {}
void vPortExitCritical(void)
{
    if(critical_exit_hook != NULL)
    {
        void (*hook)(void) = critical_exit_hook;
        critical_exit_hook = NULL;
        hook();
    }
}
void set_bms(bool state) { CHECK(!state); }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ (void)port; (void)pin; CHECK(state == GPIO_PIN_RESET); }
void ams_fault_log_event(ams_fault_log_event_t event, uint16_t detail,
                         uint32_t a, uint32_t b)
{ (void)event; (void)detail; (void)a; (void)b; }

HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *can)
{
    if(on_state_read != NULL)
    {
        void (*hook)(void) = on_state_read;
        on_state_read = NULL;
        hook();
    }
    return can->State;
}
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *can)
{ (void)can; return hal_error; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *can)
{ (void)can; hal_error = 0u; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *can)
{ return can ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can)
{ if(can == NULL) return HAL_ERROR; can->State = HAL_CAN_STATE_LISTENING; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t bits)
{
    can->Instance->IER |= bits;
    /* Hardware clears TME as soon as TXRQ is written. Model that before
     * callbacks can see the register-backed requests. */
    for(unsigned i = 0u; i < 3u; i++)
        if((can->Instance->sTxMailBox[i].TIR & CAN_TI0R_TXRQ) != 0u)
            can->Instance->TSR &= ~(CAN_TSR_TME0 << i);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *can, uint32_t bits)
{ can->Instance->IER &= ~bits; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can, uint32_t bits)
{ (void)can; abort_mask |= bits; return HAL_OK; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *can)
{
    uint32_t count = 0u;
    for(unsigned i = 0u; i < 3u; i++)
        if((can->Instance->TSR & (CAN_TSR_TME0 << i)) != 0u) count++;
    return count;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can,
                                      const CAN_TxHeaderTypeDef *hdr,
                                      const uint8_t data[], uint32_t *mb)
{ (void)can; (void)hdr; (void)data; (void)mb; CHECK(false); return HAL_ERROR; }

static void finish(unsigned i, uint32_t result)
{
    regs.sTxMailBox[i].TIR &= ~CAN_TI0R_TXRQ;
    regs.TSR |= (CAN_TSR_TME0 << i) | ((CAN_TSR_RQCP0 | result) << (8u * i));
}

void HAL_CAN_IRQHandler(CAN_HandleTypeDef *can)
{
    uint32_t snapshot = can->Instance->TSR;
    if((can->Instance->IER & CAN_IT_TX_MAILBOX_EMPTY) == 0u) return;
    void (*complete[3])(CAN_HandleTypeDef *) = {
        HAL_CAN_TxMailbox0CompleteCallback, HAL_CAN_TxMailbox1CompleteCallback,
        HAL_CAN_TxMailbox2CompleteCallback};
    void (*aborted[3])(CAN_HandleTypeDef *) = {
        HAL_CAN_TxMailbox0AbortCallback, HAL_CAN_TxMailbox1AbortCallback,
        HAL_CAN_TxMailbox2AbortCallback};
    for(unsigned i = 0u; i < 3u; i++)
    {
        uint32_t result = snapshot >> (8u * i);
        if((result & CAN_TSR_RQCP0) == 0u) continue;
        /* Emulate the peripheral's W1C behavior, which plain RAM lacks. */
        can->Instance->TSR &= ~(0xFu << (8u * i));
        if((result & CAN_TSR_TXOK0) != 0u) complete[i](can);
        else if((result & (CAN_TSR_TERR0 | CAN_TSR_ALST0)) != 0u)
        {
            hal_error |= HAL_CAN_ERROR_TX_TERR0 << i;
            HAL_CAN_ErrorCallback(can);
        }
        else aborted[i](can);
        if((i == 0u) && (after_first_callback != NULL)) after_first_callback();
    }
}

/* Include to exercise the private fixed-mailbox loader as well as public API.
 * Unused RX/HAL functions are removed by --gc-sections. */
#include "Core/Src/ext_drivers/canbus.c"

static canbus_device_t *reset(void)
{
    memset(&app, 0, sizeof(app));
    memset(&regs, 0, sizeof(regs));
    memset(&hcan, 0, sizeof(hcan));
    regs.TSR = empty_mask;
    regs.IER = CAN_IT_TX_MAILBOX_EMPTY;
    hcan.Instance = &regs;
    hcan.State = HAL_CAN_STATE_LISTENING;
    tick = abort_mask = hal_error = injected_busoff_tick = 0u;
    after_first_callback = on_state_read = critical_exit_hook = NULL;
    canbus_device_t *dev = &app.board.canbus;
    dev->hcan = &hcan;
    ams_can_tx_scheduler_init(&dev->tx_scheduler);
    return dev;
}

static void publish(canbus_tx_build_kind_t kind, unsigned count, uint32_t request)
{
    canbus_device_t *dev = &app.board.canbus;
    uint32_t gen = canbus_tx_next_generation(dev);
    uint16_t tag = (kind == CANBUS_TX_BUILD_CRITICAL) ?
        CANBUS_TX_TAG_CHARGER_SHUTDOWN : CANBUS_TX_TAG_NONE;
    CHECK(canbus_tx_build_begin(dev, kind, gen, tick, tag) == HAL_OK);
    dev->tx_builder.request_id = request;
    for(unsigned i = 0u; i < count; i++)
    {
        uint8_t bytes[8] = {(uint8_t)gen, (uint8_t)i, 2u, 3u, 4u, 5u, 6u, 7u};
        uint32_t id = (kind == CANBUS_TX_BUILD_CRITICAL) ? CCS_CANBUS_ID :
            ((kind == CANBUS_TX_BUILD_PROTECTED) ? 0x680u : 0x690u) + i;
        CHECK(canbus_tx_build_append(dev,
            kind == CANBUS_TX_BUILD_CRITICAL ? CAN_ID_EXT : CAN_ID_STD,
            id, bytes) == HAL_OK);
    }
    CHECK(canbus_tx_build_commit(dev,
        kind == CANBUS_TX_BUILD_PROTECTED ? count : 0u) == HAL_OK);
}

static void verify_no_early_refill(void)
{
    canbus_device_t *dev = &app.board.canbus;
    CHECK(dev->tx_mailbox_meta[0].state == CANBUS_TX_MB_FREE);
    CHECK(dev->tx_mailbox_meta[1].token.frame_index == 1u);
    CHECK(dev->tx_scheduler.detail_active.state[1] == AMS_CAN_TX_FRAME_LOADED);
    CHECK(dev->tx_scheduler.detail_active.state[3] == AMS_CAN_TX_FRAME_PENDING);
}

static void test_coalesced_completions(void)
{
    canbus_device_t *dev = reset();
    publish(CANBUS_TX_BUILD_DETAIL, 6u, 0u);
    finish(0u, CAN_TSR_TXOK0);
    finish(1u, CAN_TSR_TXOK0);
    /* Task pumping before the pending IRQ must not reuse either mailbox. */
    canbus_tx_kick(dev);
    CHECK(dev->tx_mailbox_meta[0].token.frame_index == 0u);
    CHECK(dev->tx_mailbox_meta[1].token.frame_index == 1u);
    CHECK(dev->tx_hal_load_error_count == 0u);
    after_first_callback = verify_no_early_refill;
    canbus_irq_handler(&hcan);
    CHECK(dev->tx_complete_count == 2u);
    CHECK(dev->tx_mailbox_meta[0].token.frame_index == 3u);
    CHECK(dev->tx_mailbox_meta[1].token.frame_index == 4u);
    CHECK(dev->tx_scheduler.detail_active.state[0] == AMS_CAN_TX_FRAME_COMPLETE);
    CHECK(dev->tx_scheduler.detail_active.state[1] == AMS_CAN_TX_FRAME_COMPLETE);
    CHECK(dev->tx_scheduler.detail_active.state[3] == AMS_CAN_TX_FRAME_LOADED);
    CHECK(dev->tx_mailbox_meta[2].token.frame_index == 2u);
    CHECK(dev->tx_scheduler.detail_active.state[2] == AMS_CAN_TX_FRAME_LOADED);
    CHECK(dev->tx_unexpected_callback_count == 0u);
}

static void finish_during_load(void) { finish(0u, CAN_TSR_TXOK0); }

static void test_completion_during_task_load(void)
{
    canbus_device_t *dev = reset();
    regs.TSR = CAN_TSR_TME0;
    publish(CANBUS_TX_BUILD_DETAIL, 4u, 0u);
    regs.TSR = CAN_TSR_TME1 | CAN_TSR_TME2;
    on_state_read = finish_during_load;
    canbus_tx_kick(dev);
    CHECK(dev->tx_mailbox_meta[0].token.frame_index == 0u);
    CHECK(dev->tx_mailbox_meta[1].token.frame_index == 1u);
    CHECK((regs.TSR & CAN_TSR_RQCP0) != 0u);
    CHECK(regs.sTxMailBox[1].TIR == ((0x691u << CAN_TI0R_STID_Pos) | CAN_TI0R_TXRQ));
    CHECK(regs.sTxMailBox[1].TDLR == 0x03020101u);
    CHECK(regs.sTxMailBox[1].TDHR == 0x07060504u);
    canbus_irq_handler(&hcan);
    CHECK(dev->tx_complete_count == 1u);
    CHECK(dev->tx_mailbox_meta[0].token.frame_index == 3u);
}

static void test_terminal_error_settles_owner(void)
{
    canbus_device_t *dev = reset();
    publish(CANBUS_TX_BUILD_DETAIL, 4u, 0u);
    finish(0u, CAN_TSR_TERR0);
    canbus_irq_handler(&hcan);
    CHECK(dev->tx_scheduler.detail_active.state[0] == AMS_CAN_TX_FRAME_DISCARDED);
    CHECK(dev->tx_mailbox_meta[0].token.frame_index == 3u);
    CHECK(dev->tx_complete_count == 0u);
}

static void test_rx_irq_respects_masked_tx_transaction(void)
{
    canbus_device_t *dev = reset();
    ams_can_tx_frame_t f = {0};
    f.id = 0x690u; f.dlc = 8u; f.tx_class = AMS_CAN_TX_CLASS_DETAIL;
    CHECK(ams_can_tx_publish_detail(&dev->tx_scheduler, 1u, tick, &f, 1u));
    regs.IER = 0u; /* Task is between masking TME and issuing an abort/load. */
    canbus_irq_handler(&hcan);
    CHECK(regs.IER == 0u);
    CHECK(dev->tx_mailbox_meta[0].state == CANBUS_TX_MB_FREE);
    CHECK(dev->tx_scheduler.detail_active.state[0] == AMS_CAN_TX_FRAME_PENDING);
}

static void test_busoff_settlement_and_freshness(void)
{
    canbus_device_t *dev = reset();
    publish(CANBUS_TX_BUILD_CRITICAL, 1u, 41u);
    publish(CANBUS_TX_BUILD_PROTECTED, 8u, 0u);
    regs.ESR = CAN_ESR_BOFF;
    regs.MSR = CAN_MSR_ERRI;
    hal_error = HAL_CAN_ERROR_BOF;
    canbus_sce_irq_note(&hcan);
    regs.MSR &= ~CAN_MSR_ERRI;
    HAL_CAN_ErrorCallback(&hcan);
    CHECK(dev->tx_suspended);
    CHECK(abort_mask == 7u);
    canbus_poll_errors(dev, &app);
    CHECK(app.can_busoff_count == 1u);
    CHECK(dev->tx_scheduler.controller_epoch == 1u);
    CHECK(HAL_CAN_GetState(&hcan) == HAL_CAN_STATE_LISTENING);
    for(unsigned i = 0u; i < 3u; i++) canbus_poll_errors(dev, &app);
    CHECK(app.can_recover_count == 0u);
    CHECK(app.can_busoff_count == 1u);
    regs.ESR = 0u;
    canbus_poll_errors(dev, &app); /* BOFF clear alone is insufficient. */
    CHECK(app.can_recover_count == 0u);
    for(unsigned i = 0u; i < 3u; i++) finish(i, 0u);
    canbus_poll_errors(dev, &app); /* Unretired RQCP must also block reset. */
    CHECK(dev->tx_scheduler.controller_epoch == 1u);
    canbus_irq_handler(&hcan);
    canbus_poll_errors(dev, &app);
    CHECK(app.can_recover_count == 1u);
    CHECK(dev->tx_scheduler.controller_epoch == 2u);
    CHECK(dev->tx_suspended && dev->tx_refresh_pending);
    CHECK(!dev->tx_scheduler.critical_active.valid);
    CHECK(!dev->tx_scheduler.protected_active.valid);
    canbus_tx_kick(dev);
    CHECK(dev->tx_mailbox_meta[0].state == CANBUS_TX_MB_FREE);
    publish(CANBUS_TX_BUILD_CRITICAL, 1u, 42u);
    publish(CANBUS_TX_BUILD_PROTECTED, 8u, 0u);
    CHECK(dev->tx_mailbox_meta[0].state == CANBUS_TX_MB_FREE);
    canbus_tx_resume_after_refresh(dev);
    CHECK(!dev->tx_suspended);
    CHECK(dev->tx_mailbox_meta[0].request_id == 42u);
    CHECK(regs.sTxMailBox[0].TIR ==
        ((CCS_CANBUS_ID << CAN_TI0R_EXID_Pos) | CAN_ID_EXT | CAN_TI0R_TXRQ));
}

static void test_busoff_latch_counts_once(void)
{
    canbus_device_t *dev = reset();
    for(unsigned n = 1u; n <= 3u; n++)
    {
        regs.ESR = CAN_ESR_BOFF;
        regs.MSR = CAN_MSR_ERRI;
        hal_error = HAL_CAN_ERROR_BOF;
        canbus_sce_irq_note(&hcan);
        regs.MSR &= ~CAN_MSR_ERRI;
        canbus_poll_errors(dev, &app);
        CHECK(app.can_busoff_count == n);
        regs.ESR = 0u;
        canbus_poll_errors(dev, &app);
        CHECK(app.can_recover_count == n);
        tick += 100u;
    }
    CHECK(dev->tx_latched_inhibit && app.can_busoff_fault);
    for(unsigned i = 0u; i < 5u; i++) canbus_poll_errors(dev, &app);
    CHECK(app.can_recover_count == 3u);
    CHECK(dev->busoff_window_count == 3u);
    canbus_tx_resume_after_refresh(dev);
    CHECK(dev->tx_suspended);
}

static void inject_physical_equivalent_busoff_without_sticky_hw(void)
{
    canbus_device_t *dev = &app.board.canbus;
    canbus_record_busoff_event(dev, &app, injected_busoff_tick,
                               (uint8_t)app.state);
}

static void test_clustered_busoff_events_are_preserved(void)
{
    canbus_device_t *dev = reset();
    app.state = STATE_DISCARGE;
    dev->started = true;
    dev->notification_active = true;
    tick = 1000u;

    canbus_record_busoff_event(dev, &app, 1000u, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app, 1002u, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app, 1004u, (uint8_t)app.state);

    CHECK(dev->busoff_event_sequence == 3u);
    CHECK(dev->busoff_window_count == 3u);
    CHECK(dev->tx_latched_inhibit);
    CHECK(app.can_busoff_hard_fault_latched);
    CHECK(app.can_busoff_policy_latch_reason ==
          AMS_CAN_POLICY_LATCH_REPEATED_BUSOFF);
    CHECK(app.can_busoff_recovery_start_tick == 1000u);

    canbus_poll_errors(dev, &app);
    CHECK(app.can_busoff_count == 3u);
    CHECK(dev->busoff_event_consumed_sequence == 3u);
}

static void test_busoff_during_pending_recovery_is_preserved(void)
{
    canbus_device_t *dev = reset();
    app.state = STATE_DISCARGE;
    dev->started = true;
    dev->notification_active = true;

    canbus_record_busoff_event(dev, &app, 2000u, (uint8_t)app.state);
    canbus_poll_errors(dev, &app);
    CHECK(app.can_busoff_count == 1u);
    CHECK(app.can_recover_pending);
    CHECK(app.can_busoff_recovery_start_tick == 2000u);

    canbus_record_busoff_event(dev, &app, 2010u, (uint8_t)app.state);
    canbus_poll_errors(dev, &app);
    CHECK(app.can_busoff_count == 2u);
    CHECK(app.can_recover_pending);
    CHECK(app.can_busoff_recovery_start_tick == 2000u);
    CHECK(dev->busoff_event_consumed_sequence == 2u);
}

static void test_busoff_sliding_window_boundary_and_wrap(void)
{
    canbus_device_t *dev = reset();
    app.state = STATE_DISCARGE;
    canbus_record_busoff_event(dev, &app, 100u, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app, 5100u, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app,
        100u + AMS_CAN_BUSOFF_REPEAT_WINDOW_MS, (uint8_t)app.state);
    CHECK(dev->busoff_window_count == 3u);
    CHECK(dev->tx_latched_inhibit);

    dev = reset();
    app.state = STATE_DISCARGE;
    canbus_record_busoff_event(dev, &app, 100u, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app, 5100u, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app,
        101u + AMS_CAN_BUSOFF_REPEAT_WINDOW_MS, (uint8_t)app.state);
    CHECK(dev->busoff_window_count == 2u);
    CHECK(!dev->tx_latched_inhibit);

    dev = reset();
    app.state = STATE_DISCARGE;
    const uint32_t first = UINT32_MAX - 4999u;
    canbus_record_busoff_event(dev, &app, first, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app, first + 5000u,
                               (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app,
        first + AMS_CAN_BUSOFF_REPEAT_WINDOW_MS, (uint8_t)app.state);
    CHECK(dev->busoff_window_count == 3u);
    CHECK(dev->tx_latched_inhibit);
}

static void test_sticky_hal_bof_is_not_double_counted(void)
{
    canbus_device_t *dev = reset();
    app.state = STATE_DISCARGE;
    dev->started = true;
    dev->notification_active = true;
    tick = 3000u;
    regs.MSR = CAN_MSR_ERRI;
    regs.ESR = CAN_ESR_BOFF;
    canbus_sce_irq_note(&hcan);
    CHECK(dev->busoff_event_sequence == 1u);

    hal_error = HAL_CAN_ERROR_BOF;
    HAL_CAN_ErrorCallback(&hcan);
    HAL_CAN_ErrorCallback(&hcan);
    CHECK(dev->busoff_event_sequence == 1u);

    canbus_poll_errors(dev, &app);
    CHECK(app.can_busoff_count == 1u);
}

static void test_recovery_commit_rejects_new_busoff_sequence(void)
{
    canbus_device_t *dev = reset();
    app.state = STATE_DISCARGE;
    dev->started = true;
    dev->notification_active = true;
    tick = 4000u;

    canbus_record_busoff_event(dev, &app, tick, (uint8_t)app.state);
    canbus_poll_errors(dev, &app);
    CHECK(app.can_busoff_count == 1u);
    CHECK(app.can_recover_pending);

    /* Model ABOM having electrically cleared BOFF before task recovery. A new
     * SCE event arrives at the state-read boundary and also clears quickly, so
     * event sequence rather than a sticky hardware bit must protect commit. */
    regs.ESR = 0u;
    hal_error = HAL_CAN_ERROR_NONE;
    injected_busoff_tick = 4010u;
    on_state_read = inject_physical_equivalent_busoff_without_sticky_hw;
    canbus_poll_errors(dev, &app);

    CHECK(dev->busoff_event_sequence == 2u);
    CHECK(app.can_recover_count == 0u);
    CHECK(app.can_recover_pending);
    CHECK(app.can_busoff_fault);
    CHECK(app.canbus_fault);
    CHECK(dev->tx_recovery_pending);
}

static void test_service_recovery_cannot_clear_new_busoff(void)
{
    canbus_device_t *dev = reset();
    app.state = STATE_DISCARGE;
    dev->started = true;
    dev->notification_active = true;
    tick = 5000u;

    canbus_record_busoff_event(dev, &app, 5000u, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app, 5001u, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app, 5002u, (uint8_t)app.state);
    CHECK(dev->tx_latched_inhibit);
    CHECK(dev->busoff_window_count == 3u);

    regs.ESR = 0u;
    hal_error = HAL_CAN_ERROR_NONE;
    injected_busoff_tick = 5010u;
    on_state_read = inject_physical_equivalent_busoff_without_sticky_hw;
    CHECK(canbus_recover(dev, &app) == HAL_BUSY);

    CHECK(dev->busoff_event_sequence == 4u);
    CHECK(dev->tx_latched_inhibit);
    CHECK(dev->busoff_window_count == 3u);
    CHECK(dev->tx_recovery_pending);
}

static void test_post_commit_busoff_survives_service_recovery(void)
{
    canbus_device_t *dev = reset();
    app.state = STATE_DISCARGE;
    dev->started = true;
    dev->notification_active = true;
    tick = 6000u;

    /* Build the service-resettable repeated-bus-off state. */
    canbus_record_busoff_event(dev, &app, 6000u, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app, 6001u, (uint8_t)app.state);
    canbus_record_busoff_event(dev, &app, 6002u, (uint8_t)app.state);
    CHECK(dev->tx_latched_inhibit);
    CHECK(app.can_busoff_hard_fault_latched);

    /* Electrical BOFF has cleared and administrative recovery commits. */
    regs.ESR = 0u;
    hal_error = HAL_CAN_ERROR_NONE;
    CHECK(canbus_recover(dev, &app) == HAL_OK);
    CHECK(!dev->tx_latched_inhibit);
    CHECK(!app.can_busoff_fault);
    CHECK(!app.can_recover_pending);
    CHECK(!app.can_busoff_hard_fault_latched);
    CHECK(app.can_authority_refresh_pending);
    CHECK(!app.can_authority_ready);

    /* This models the old race window: a physical BOFF arrives immediately
     * after successful transport settlement. Because application cleanup now
     * committed atomically inside canbus_recover(), there is no later CLI-side
     * clear that can erase the newly asserted event. */
    canbus_record_busoff_event(dev, &app, 6010u, (uint8_t)app.state);
    CHECK(dev->busoff_event_sequence == 4u);
    CHECK(app.can_busoff_fault);
    CHECK(app.canbus_fault);
    CHECK(app.can_busoff_recovery_active);
    CHECK(app.can_busoff_recovery_start_tick == 6010u);
    CHECK(app.can_busoff_recovery_state == (uint8_t)STATE_DISCARGE);
    CHECK(!app.can_authority_ready);
    CHECK(dev->tx_suspended);
}

static void test_shutdown_identity_and_supersession(void)
{
    canbus_device_t *dev = reset();
    charger_t *ccs = &app.board.charger;
    ccs->shutdown_request_count = 42u;
    ccs->shutdown_pending = true;
    ccs->shutdown_frames_remaining = 3u;
    dev->tx_generation_counter = 100u;
    publish(CANBUS_TX_BUILD_CRITICAL, 1u, 41u);
    publish(CANBUS_TX_BUILD_CRITICAL, 1u, 42u);
    CHECK((abort_mask & CAN_TX_MAILBOX0) != 0u);
    CHECK(dev->tx_mailbox_meta[0].state == CANBUS_TX_MB_ABORT_REQUESTED);
    finish(0u, CAN_TSR_TXOK0); /* Old completion wins its abort race. */
    canbus_irq_handler(&hcan);
    CHECK(ccs->shutdown_frames_remaining == 3u);
    CHECK(dev->tx_mailbox_meta[0].token.generation == 102u);
    CHECK(dev->tx_mailbox_meta[0].request_id == 42u);
    finish(0u, CAN_TSR_TXOK0);
    canbus_irq_handler(&hcan);
    CHECK(ccs->shutdown_frames_remaining == 2u);
}

static ams_can_tx_frame_t make_frame(uint32_t id, ams_can_tx_class_t cls)
{
    ams_can_tx_frame_t f = {0};
    f.id = id; f.dlc = 8u; f.tx_class = cls;
    return f;
}

static void test_interleaved_required_frames(void)
{
    static ams_can_tx_scheduler_t s;
    ams_can_tx_frame_t frames[11];
    const uint16_t ids[11] = {0x680,0x681,0x682,0x683,0x68B,
                             0x684,0x685,0x686,0x687,0x689,0x68A};
    for(unsigned i = 0u; i < 11u; i++)
        frames[i] = make_frame(ids[i], ids[i] <= 0x687 ?
            AMS_CAN_TX_CLASS_PROTECTED_REQUIRED : AMS_CAN_TX_CLASS_PROTECTED_ADVISORY);
    for(unsigned exclude_last = 0u; exclude_last < 2u; exclude_last++)
    {
        ams_can_tx_scheduler_init(&s);
        CHECK(!ams_can_tx_publish_protected(&s, 1u, 0u, frames, 11u, 7u, NULL));
        CHECK(ams_can_tx_publish_protected(&s, 1u, 0u, frames, 11u, 8u, NULL));
        ams_can_tx_token_t tok;
        ams_can_tx_frame_t f;
        while(ams_can_tx_reserve_next(&s, &tok, &f))
        {
            ams_can_tx_mark_loaded(&s, &tok);
            bool done = exclude_last ? (f.id != 0x687u) :
                (f.tx_class == AMS_CAN_TX_CLASS_PROTECTED_REQUIRED);
            if(done) ams_can_tx_mark_complete(&s, &tok, true, 20u);
        }
        CHECK(s.protected_required_complete_count == (exclude_last ? 0u : 1u));
        CHECK(ams_can_tx_publish_protected(&s, 2u, 100u, frames, 11u, 8u, NULL));
        CHECK(s.protected_deadline_miss == exclude_last);
    }
}

static void test_epoch_uses_publication_order(void)
{
    static ams_can_tx_scheduler_t s;
    ams_can_tx_scheduler_init(&s);
    ams_can_tx_frame_t normal = make_frame(1u, AMS_CAN_TX_CLASS_CRITICAL);
    ams_can_tx_frame_t shutdown = make_frame(2u, AMS_CAN_TX_CLASS_CRITICAL);
    ams_can_tx_token_t tok;
    ams_can_tx_frame_t f;
    CHECK(ams_can_tx_publish_critical(&s, 100u, 0u, &normal, 1u));
    CHECK(ams_can_tx_reserve_next(&s, &tok, &f));
    ams_can_tx_mark_loaded(&s, &tok);
    CHECK(ams_can_tx_publish_critical(&s, 1u, 1u, &shutdown, 1u));
    CHECK(ams_can_tx_token_requires_abort(&s, &tok));
    ams_can_tx_controller_epoch_reset(&s);
    CHECK(s.critical_active.frames[0].id == 2u);

    /* A newer disable must also win if the obsolete RESERVED load fails. */
    ams_can_tx_scheduler_init(&s);
    CHECK(ams_can_tx_publish_critical(&s, 100u, 0u, &normal, 1u));
    CHECK(ams_can_tx_reserve_next(&s, &tok, &f));
    CHECK(ams_can_tx_publish_critical(&s, 101u, 1u, &shutdown, 1u));
    ams_can_tx_load_failed(&s, &tok);
    CHECK(s.critical_active.frames[0].id == 2u);
}

static void test_base_survives_tuning_and_full_capacity(void)
{
    static ams_can_tx_scheduler_t s;
    static ams_can_tx_frame_t frames[AMS_CAN_TX_DETAIL_MAX_FRAMES];
    ams_can_tx_scheduler_init(&s);
    for(unsigned i = 0u; i < AMS_CAN_TX_DETAIL_MAX_FRAMES; i++)
        frames[i] = make_frame(0x690u + i, AMS_CAN_TX_CLASS_DETAIL);
    CHECK(ams_can_tx_publish_tuning(&s, 1u, 0u, frames, 1u));
    CHECK(ams_can_tx_publish_detail(&s, 2u, 1u, frames, AMS_CAN_TX_DETAIL_MAX_FRAMES));
    CHECK(ams_can_tx_publish_tuning(&s, 3u, 2u, frames, 1u));
    CHECK(s.detail_pending.generation == 2u);
    CHECK(s.detail_tuning_shed == 1u);
    CHECK(s.detail_pending.frames[191].id == frames[191].id);
    ams_can_tx_token_t tok;
    ams_can_tx_frame_t f;
    unsigned count = 0u;
    while(ams_can_tx_reserve_next(&s, &tok, &f))
    {
        ams_can_tx_mark_loaded(&s, &tok);
        ams_can_tx_mark_complete(&s, &tok, true, 3u);
        CHECK(++count <= 193u);
    }
    CHECK(count == 193u);
}


static void inject_sce_busoff_after_poll_snapshot(void)
{
    regs.MSR |= CAN_MSR_ERRI;
    regs.ESR |= CAN_ESR_BOFF;
    canbus_sce_irq_note(&hcan);
}

static void test_sce_after_poll_snapshot_is_not_synthesized_twice(void)
{
    canbus_device_t *dev = reset();
    app.state = STATE_DISCARGE;
    dev->started = true;
    dev->notification_active = true;
    tick = 7000u;

    /* Model a genuine SCE interrupt immediately after the CAN task's initial
     * protected snapshot. Physical target identity must come only from SCE;
     * the task's later hardware-BOFF observation must not invent event #2. */
    critical_exit_hook = inject_sce_busoff_after_poll_snapshot;
    canbus_poll_errors(dev, &app);

    CHECK(dev->busoff_event_sequence == 1u);
    CHECK(dev->busoff_event_consumed_sequence == 0u);
    CHECK(app.can_busoff_count == 0u);
    CHECK(dev->busoff_window_count == 1u);
    CHECK(!dev->tx_latched_inhibit);

    /* The next 10 Hz poll consumes exactly that preserved SCE event. */
    canbus_poll_errors(dev, &app);
    CHECK(dev->busoff_event_sequence == 1u);
    CHECK(dev->busoff_event_consumed_sequence == 1u);
    CHECK(app.can_busoff_count == 1u);
    CHECK(dev->busoff_window_count == 1u);
    CHECK(!dev->tx_latched_inhibit);
}


static void increment_hal_load_error_after_poll_snapshot(void)
{
    canbus_device_t *dev = &app.board.canbus;
    if(dev->tx_hal_load_error_count != UINT32_MAX)
    {
        dev->tx_hal_load_error_count++;
    }
}

static void test_isr_diagnostic_watermark_preserves_post_snapshot_increment(void)
{
    canbus_device_t *dev = reset();
    dev->started = true;
    dev->notification_active = true;
    tick = 8000u;

    dev->tx_hal_load_error_count = 1u;
    critical_exit_hook = increment_hal_load_error_after_poll_snapshot;
    canbus_poll_errors(dev, &app);

    CHECK(dev->tx_hal_load_error_count == 2u);
    CHECK(dev->tx_hal_load_error_reported == 1u);
    CHECK(app.can_error_count == 1u);

    canbus_poll_errors(dev, &app);
    CHECK(dev->tx_hal_load_error_reported == 2u);
    CHECK(app.can_error_count == 2u);
}

int main(void)
{
    test_interleaved_required_frames();
    test_epoch_uses_publication_order();
    test_base_survives_tuning_and_full_capacity();
    test_coalesced_completions();
    test_completion_during_task_load();
    test_terminal_error_settles_owner();
    test_rx_irq_respects_masked_tx_transaction();
    test_busoff_settlement_and_freshness();
    test_busoff_latch_counts_once();
    test_clustered_busoff_events_are_preserved();
    test_busoff_during_pending_recovery_is_preserved();
    test_busoff_sliding_window_boundary_and_wrap();
    test_sticky_hal_bof_is_not_double_counted();
    test_sce_after_poll_snapshot_is_not_synthesized_twice();
    test_isr_diagnostic_watermark_preserves_post_snapshot_increment();
    test_recovery_commit_rejects_new_busoff_sequence();
    test_service_recovery_cannot_clear_new_busoff();
    test_post_commit_busoff_survives_service_recovery();
    test_shutdown_identity_and_supersession();
    puts("PASS 19 focused CAN scheduler/transport regressions");
    return 0;
}
