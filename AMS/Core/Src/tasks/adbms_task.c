/*
 * adbms_task.h
 *
 *  Created on: June 3, 2025
 *      Author: Cole Bardin
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#include <tasks/adbms_task.h>
#include <string.h>

#define ADBMS_STATUS_DIAG_PERIOD_MS            1000u
#define ADBMS_CONFIG_DIAG_PERIOD_MS            6000u
#define ADBMS_OPEN_WIRE_DIAG_PERIOD_MS        30000u
/* Authority ages are deliberately wider than the nominal diagnostic period,
 * but finite: a surviving old register snapshot must not remain authoritative
 * forever if the periodic diagnostic path stops updating. */
#define ADBMS_STATUS_AUTH_MAX_AGE_MS            2500u
#define ADBMS_CONFIG_AUTH_MAX_AGE_MS           13000u
#define ADBMS_IDENTITY_AUTH_MAX_AGE_MS           2500u
#define ADBMS_DIAG_CYCLES(period_ms) \
    ((((period_ms) * ADBMS_FREQ) + 999u) / 1000u)
#define ADBMS_STATUS_DIAG_PERIOD_CYCLES \
    ADBMS_DIAG_CYCLES(ADBMS_STATUS_DIAG_PERIOD_MS)
#define ADBMS_CONFIG_DIAG_PERIOD_CYCLES \
    ADBMS_DIAG_CYCLES(ADBMS_CONFIG_DIAG_PERIOD_MS)
#define ADBMS_OPEN_WIRE_DIAG_PERIOD_CYCLES \
    ADBMS_DIAG_CYCLES(ADBMS_OPEN_WIRE_DIAG_PERIOD_MS)

#define ADBMS_LIFECYCLE_COMMON_BLOCKERS ( \
    AMS_ADBMS_FAULT_COMMUNICATION | \
    AMS_ADBMS_FAULT_PEC | \
    AMS_ADBMS_FAULT_COMMAND_COUNTER | \
    AMS_ADBMS_FAULT_CONFIG_READBACK | \
    AMS_ADBMS_FAULT_REFERENCE | \
    AMS_ADBMS_FAULT_CELL_DATA_STALE | \
    AMS_ADBMS_FAULT_OPEN_WIRE | \
    AMS_ADBMS_FAULT_BALANCE_WRITE | \
    AMS_ADBMS_FAULT_STATUS | \
    AMS_ADBMS_FAULT_IDENTITY | \
    AMS_ADBMS_FAULT_C_DATA_INVALID | \
    AMS_ADBMS_FAULT_TOPOLOGY | \
    AMS_ADBMS_FAULT_DEVICE_RESET)

#if AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_REDUNDANT_CS
#define ADBMS_LIFECYCLE_BLOCKERS \
    (ADBMS_LIFECYCLE_COMMON_BLOCKERS | AMS_ADBMS_FAULT_S_REDUNDANCY)
#else
#define ADBMS_LIFECYCLE_BLOCKERS ADBMS_LIFECYCLE_COMMON_BLOCKERS
#endif

void adbms_task_fn(void *argument);

static StaticTask_t adbms_task_tcb;
static StackType_t adbms_task_stack[AMS_STACK_ADBMS_WORDS];
static TaskHandle_t adbms_task_handle = NULL;


/* ADBMS transport profiling uses the Cortex-M7 cycle counter on target.  Host
 * builds fall back to the RTOS tick because they validate state-machine logic,
 * not physical isoSPI timing. */
static bool adbms_task_dwt_ready = false;
#if !(defined(AMS_HOST_TEST) && AMS_HOST_TEST)
static uint32_t adbms_task_dwt_last_cycles = 0u;
static uint64_t adbms_task_dwt_epoch_cycles = 0u;
#endif
static uint64_t adbms_task_cooperative_yield_total_us = 0u;

static bool adbms_tick_due(uint32_t now, uint32_t due)
{
    /* Portable half-range modular ordering for the 32-bit RTOS tick. */
    return ((uint32_t)(now - due) < 0x80000000u);
}

static void adbms_task_timing_init(void)
{
#if defined(AMS_HOST_TEST) && AMS_HOST_TEST
    adbms_task_dwt_ready = false;
#else
#if defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    adbms_task_dwt_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u);
    adbms_task_dwt_last_cycles = 0u;
    adbms_task_dwt_epoch_cycles = 0u;
#else
    adbms_task_dwt_ready = false;
#endif
#endif
}

static uint32_t adbms_task_time_us(void *ctx)
{
    (void)ctx;
#if !(defined(AMS_HOST_TEST) && AMS_HOST_TEST)
    if(adbms_task_dwt_ready && (SystemCoreClock >= 1000000u))
    {
        uint32_t cycles_per_us = SystemCoreClock / 1000000u;
        if(cycles_per_us != 0u)
        {
            uint32_t now_cycles = DWT->CYCCNT;
            /* CYCCNT wraps in about 20 s at 216 MHz. Extend it before
             * converting to microseconds; dividing the raw 32-bit counter
             * first would create a false multi-billion-us session gap at each
             * cycle-counter wrap. The ADBMS task calls this at least every
             * scan, so at most one CYCCNT wrap can occur between samples. */
            if(now_cycles < adbms_task_dwt_last_cycles)
            {
                adbms_task_dwt_epoch_cycles += (UINT64_C(1) << 32u);
            }
            adbms_task_dwt_last_cycles = now_cycles;
            return (uint32_t)((adbms_task_dwt_epoch_cycles + now_cycles) /
                              cycles_per_us);
        }
    }
#endif
    return osKernelGetTickCount() * 1000u;
}

/* Long conversion/settling intervals must not busy-spin at ADBMS task priority.
 * Yield in one-tick chunks so a fail-low safety decision can interrupt the
 * wait within about one RTOS tick.  Sub-millisecond residue remains a bounded
 * timer-backed wait and is intentionally not a scheduler suspension. */
static HAL_StatusTypeDef adbms_task_cooperative_wait(void *ctx, uint32_t wait_us)
{
    app_data_t *data = (app_data_t *)ctx;
    uint32_t remaining = wait_us;

    if(data == NULL)
    {
        return HAL_ERROR;
    }

    while(remaining >= 1000u)
    {
        if(data->adbms_urgent_mute_requested)
        {
            return HAL_BUSY;
        }
        osDelay(1u);
        adbms_task_cooperative_yield_total_us += 1000u;
        remaining -= 1000u;
    }

    if(data->adbms_urgent_mute_requested)
    {
        return HAL_BUSY;
    }
    if(remaining != 0u)
    {
        return adbms6830_us_delay(&data->acc.smb, (uint16_t)remaining);
    }
    return HAL_OK;
}


static bool adbms_task_service_urgent_mute(app_data_t *data,
                                           accumulator_balance_inhibit_reason_t reason)
{
    bool requested;
    int result;

    if(data == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    requested = data->adbms_urgent_mute_requested;
    if(requested)
    {
        /* Consume this generation before the physical attempt.  A new fail-low
         * request arriving during the transaction will set it again. */
        data->adbms_urgent_mute_requested = false;
    }
    taskEXIT_CRITICAL();

    if(!requested)
    {
        return true;
    }

    result = accumulator_emergency_balance_inhibit(&data->acc, reason);

    taskENTER_CRITICAL();
    if(data->adbms_urgent_mute_service_count != UINT32_MAX)
    {
        data->adbms_urgent_mute_service_count++;
    }
    data->adbms_mute_asserted = data->acc.last_balance_mute_ok;
    data->adbms_balance_durable_zero_verified =
        data->acc.last_balance_durable_zero_verified;
    data->adbms_balance_inhibit_reason = reason;
    data->adbms_balance_active = false;
    if(result != 0)
    {
        if(data->adbms_urgent_mute_fail_count != UINT32_MAX)
        {
            data->adbms_urgent_mute_fail_count++;
        }
        data->adbms_balance_write_fault = true;
    }
    taskEXIT_CRITICAL();

    return (result == 0);
}

static void adbms_task_publish_voltage_state(app_data_t *data,
                                             const voltage_fault_state_t *fault,
                                             const accumulator_t *acc);

static uint16_t adbms_task_expected_ic_mask(const adbms6830_driver_t *smb)
{
    if((smb == NULL) || (smb->num_ics <= 0) ||
       (smb->num_ics > (int)ADBMS6830_MAX_TRACKED_ICS))
    {
        return 0u;
    }

    return (uint16_t)((1u << (uint8_t)smb->num_ics) - 1u);
}

static uint16_t adbms_task_monitored_cell_mask(const adbms6830_driver_t *smb)
{
    if((smb == NULL) || (smb->monitored_cell_count == 0u) ||
       (smb->monitored_cell_count > 16u))
    {
        return 0u;
    }

    return (smb->monitored_cell_count == 16u) ? UINT16_MAX :
           (uint16_t)((1u << smb->monitored_cell_count) - 1u);
}

static bool adbms_task_all_s_samples_valid(const adbms6830_driver_t *smb)
{
    uint16_t cell_mask = adbms_task_monitored_cell_mask(smb);

    if((smb == NULL) || (cell_mask == 0u) ||
       (smb->num_ics <= 0) ||
       (smb->num_ics > (int)ADBMS6830_MAX_TRACKED_ICS))
    {
        return false;
    }

    for(uint8_t ic = 0u; ic < (uint8_t)smb->num_ics; ic++)
    {
        if(((smb->last_scell_updated_mask[ic] & cell_mask) != cell_mask) ||
           ((smb->last_scell_pec_mask[ic] & cell_mask) != 0u))
        {
            return false;
        }
    }

    return true;
}

static uint16_t adbms_task_compute_c_authority(const app_data_t *data)
{
    const adbms6830_driver_t *smb;
    const adbms6830_diag_health_t *health;
    uint16_t mask = 0u;
    uint16_t expected_ic_mask;
    uint16_t expected_cell_count;
    uint32_t now;

    if(data == NULL)
    {
        return 0u;
    }

    smb = &data->acc.smb;
    health = adbms6830_diag_health_get(smb);
    now = osKernelGetTickCount();
    expected_ic_mask = adbms_task_expected_ic_mask(smb);
    expected_cell_count = (uint16_t)((uint16_t)smb->monitored_cell_count *
                                     (uint16_t)((smb->num_ics > 0) ?
                                         smb->num_ics : 0));

#if AMS_HIL_REPLACE_ADBMS
    /* The HIL image has no physical ADBMS PEC, command counter, configuration,
     * reference or SID registers. A fresh, accepted CAN image satisfies those
     * not-applicable transport-contract checks; the same C-code/range/slew/
     * freshness and topology gates remain real. */
    if(data->adbms_last_voltage_scan_ok &&
       !data->can_busoff_fault && !data->can_recover_pending)
    {
        mask |= AMS_ADBMS_C_AUTH_TRANSPORT |
                AMS_ADBMS_C_AUTH_PEC |
                AMS_ADBMS_C_AUTH_COUNTER |
                AMS_ADBMS_C_AUTH_CONFIG |
                AMS_ADBMS_C_AUTH_REFERENCE |
                AMS_ADBMS_C_AUTH_IDENTITY;
    }
    if(data->acc.voltage_full_updated &&
       (data->voltage_updated_cell_count == expected_cell_count))
    {
        mask |= AMS_ADBMS_C_AUTH_CODES;
    }
    if((data->voltage_usable_cell_count == expected_cell_count) &&
       !data->voltage_read_fault)
    {
        mask |= AMS_ADBMS_C_AUTH_RANGE;
    }
    if(data->voltage_jump_cell_count == 0u)
    {
        mask |= AMS_ADBMS_C_AUTH_SLEW;
    }
    if((data->voltage_stale_cell_count == 0u) &&
       data->acc.voltage_full_usable)
    {
        mask |= AMS_ADBMS_C_AUTH_FRESH;
    }
    if(accumulator_final_ring_topology_valid(&data->acc))
    {
        mask |= AMS_ADBMS_C_AUTH_TOPOLOGY;
    }
    return mask;
#endif

    if(data->adbms_voltage_scan_attempted && data->adbms_last_voltage_scan_ok)
    {
        mask |= AMS_ADBMS_C_AUTH_TRANSPORT;
    }
    if(data->adbms_last_voltage_scan_ok &&
       (data->voltage_pec_fail_cell_count == 0u))
    {
        mask |= AMS_ADBMS_C_AUTH_PEC;
    }
    if(data->adbms_last_voltage_scan_ok &&
       (health != NULL) &&
       (health->last_cmd_counter_mismatch_mask == 0u))
    {
        mask |= AMS_ADBMS_C_AUTH_COUNTER;
    }
    if(data->acc.voltage_full_updated &&
       (data->voltage_updated_cell_count == expected_cell_count))
    {
        mask |= AMS_ADBMS_C_AUTH_CODES;
    }
    if((data->voltage_usable_cell_count == expected_cell_count) &&
       !data->voltage_read_fault)
    {
        mask |= AMS_ADBMS_C_AUTH_RANGE;
    }
    if(data->voltage_jump_cell_count == 0u)
    {
        mask |= AMS_ADBMS_C_AUTH_SLEW;
    }
    if((data->voltage_stale_cell_count == 0u) &&
       data->acc.voltage_full_usable)
    {
        mask |= AMS_ADBMS_C_AUTH_FRESH;
    }
    if(!data->adbms_config_fault &&
       (data->adbms_config_last_valid_tick != 0u) &&
       ((now - data->adbms_config_last_valid_tick) <=
        ADBMS_CONFIG_AUTH_MAX_AGE_MS) &&
       (data->adbms_config_expected_fingerprint != 0u) &&
       (data->adbms_config_expected_fingerprint ==
        data->adbms_config_readback_fingerprint))
    {
        mask |= AMS_ADBMS_C_AUTH_CONFIG;
    }
    if((health != NULL) &&
       (data->adbms_status_last_valid_tick != 0u) &&
       ((now - data->adbms_status_last_valid_tick) <=
        ADBMS_STATUS_AUTH_MAX_AGE_MS) &&
       ((health->reference_invalid_ic_mask |
         health->reference_fault_ic_mask) == 0u))
    {
        mask |= AMS_ADBMS_C_AUTH_REFERENCE;
    }
    if((health != NULL) && (expected_ic_mask != 0u) &&
       (data->adbms_identity_last_valid_tick != 0u) &&
       ((now - data->adbms_identity_last_valid_tick) <=
        ADBMS_IDENTITY_AUTH_MAX_AGE_MS) &&
       ((health->sid_valid_ic_mask & expected_ic_mask) == expected_ic_mask) &&
       ((health->sid_identity_mismatch_ic_mask & expected_ic_mask) == 0u))
    {
        mask |= AMS_ADBMS_C_AUTH_IDENTITY;
    }
    if(accumulator_final_ring_topology_valid(&data->acc))
    {
        mask |= AMS_ADBMS_C_AUTH_TOPOLOGY;
    }

    return mask;
}

static uint32_t adbms_task_device_reset_total(const adbms6830_driver_t *smb,
                                              uint16_t *mask_out)
{
    uint32_t total = 0u;
    uint16_t mask = 0u;
    const adbms6830_diag_health_t *health = adbms6830_diag_health_get(smb);

    if(health != NULL)
    {
        mask = health->sticky_unexpected_counter_reset_mask;
        for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
        {
            uint32_t count = health->unexpected_counter_reset_count[ic];
            if(UINT32_MAX - total < count)
            {
                total = UINT32_MAX;
                break;
            }
            total += count;
        }
    }

    if(mask_out != NULL)
    {
        *mask_out = mask;
    }
    return total;
}

static bool adbms_task_s_redundancy_blocks(const app_data_t *data)
{
#if AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_REDUNDANT_CS
    const adbms6830_diag_health_t *health;

    if(data == NULL)
    {
        return true;
    }

    health = adbms6830_diag_health_get(&data->acc.smb);
    return (health == NULL) || (health->cs_fault_ic_mask != 0u);
#else
    (void)data;
    return false;
#endif
}

static bool adbms_task_diag_fault_from_components(const app_data_t *data)
{
    if(data == NULL)
    {
        return true;
    }

#if AMS_HIL_REPLACE_ADBMS
    return data->can_busoff_fault || data->can_recover_pending;
#else
    return data->adbms_config_fault ||
           data->adbms_status_fault ||
           data->adbms_open_wire_fault ||
           data->adbms_open_wire_restore_fault ||
           data->adbms_balance_write_fault ||
           adbms_task_s_redundancy_blocks(data);
#endif
}

static bool adbms_open_wire_auto_allowed(const app_data_t *data)
{
#if AMS_ENABLE_AUTO_C_OPEN_WIRE && \
    !(AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && AMS_S_PATH_ECO_VALIDATED)
    float abs_current;

    if(data == NULL)
    {
        return false;
    }
    abs_current = (data->current < 0.0f) ? -data->current : data->current;
    return ((data->state == STATE_START) ||
            (data->state == STATE_CHARGE)) &&
           data->voltage_valid &&
           !data->voltage_read_fault &&
           data->current_valid &&
           (abs_current <= 2.0f) &&
           !data->adbms_balance_active &&
           !accumulator_balance_shadow_active(&data->acc) &&
           !data->hard_fault &&
           !data->voltage_fault &&
           !data->current_fault;
#else
    (void)data;
    return false;
#endif
}

static void adbms_task_run_periodic_diagnostics(app_data_t *data)
{
    accumulator_t *acc;
    adbms6830_driver_t *smb;
    HAL_StatusTypeDef status;

    if(data == NULL)
    {
        return;
    }

    acc = &data->acc;
    smb = &acc->smb;

    if((data->adbms_scan_count % ADBMS_STATUS_DIAG_PERIOD_CYCLES) == 0u)
    {
        bool status_fault;
        bool transport_ok;
        bool non_cs_ok;
        const adbms6830_diag_health_t *health;
        uint16_t expected_mask;
        uint32_t now;

        status = adbms6830_refresh_diagnostics(smb);
        health = adbms6830_diag_health_get(smb);
        expected_mask = adbms_task_expected_ic_mask(smb);
        now = osKernelGetTickCount();
        transport_ok = adbms6830_diagnostic_transport_ok(smb);
        non_cs_ok = adbms6830_non_cs_diagnostics_ok(smb);
        if(transport_ok && (smb->health.silicon_health_sweep_count != UINT32_MAX))
        {
            smb->health.silicon_health_sweep_count++;
        }
        /* A successful one-off Status transaction cannot re-authorize a chain
         * whose startup/transport baseline was never established.  Preserve
         * the separate safety_ready semantics for C-only degraded operation,
         * but make loss of read-only transport readiness an explicit status
         * fault. */
        status_fault = !acc->delay_timer_ready ||
                       !acc->smb_transport_ready ||
                       !transport_ok ||
                       !non_cs_ok;
        taskENTER_CRITICAL();
        /* adbms6830_refresh_diagnostics() intentionally returns HAL_ERROR for
         * a policy fault such as CSxFLT.  The application status-transport
         * result must not conflate that with a failed Status A-E transaction. */
        data->adbms_last_diag_status = transport_ok ? HAL_OK : status;
		if(data->adbms_status_diag_count != UINT32_MAX)
		{
			data->adbms_status_diag_count++;
		}
		data->adbms_status_fault = status_fault;
        if(transport_ok)
        {
            data->adbms_status_last_valid_tick = now;
        }
        if(transport_ok && (health != NULL) &&
           (expected_mask != 0u) &&
           ((health->sid_valid_ic_mask & expected_mask) == expected_mask) &&
           ((health->sid_identity_mismatch_ic_mask & expected_mask) == 0u))
        {
            data->adbms_identity_last_valid_tick = now;
        }
        taskEXIT_CRITICAL();
    }

    if((data->adbms_scan_count % ADBMS_CONFIG_DIAG_PERIOD_CYCLES) == 0u)
    {
        const adbms6830_diag_health_t *health;
        uint32_t expected_fingerprint;
        uint32_t readback_fingerprint;
        uint32_t now;

        status = adbms6830_verify_config_readback(smb);
        health = adbms6830_diag_health_get(smb);
        expected_fingerprint = adbms6830_config_expected_fingerprint(smb);
        readback_fingerprint = adbms6830_config_readback_fingerprint(smb);
        now = osKernelGetTickCount();
        taskENTER_CRITICAL();
        data->adbms_last_diag_status = status;
        if(data->adbms_config_diag_count != UINT32_MAX)
        {
            data->adbms_config_diag_count++;
        }
        data->adbms_config_fault = ((status != HAL_OK) ||
                                    ((health != NULL) &&
                                     (health->config_mismatch_mask != 0u)));
        data->adbms_config_expected_fingerprint = expected_fingerprint;
        data->adbms_config_readback_fingerprint = readback_fingerprint;
        if(!data->adbms_config_fault && (expected_fingerprint != 0u) &&
           (expected_fingerprint == readback_fingerprint))
        {
            data->adbms_config_last_valid_tick = now;
        }
        taskEXIT_CRITICAL();
    }

    if(((data->adbms_scan_count % ADBMS_OPEN_WIRE_DIAG_PERIOD_CYCLES) == 0u) &&
       adbms_open_wire_auto_allowed(data))
    {
        adbms6830_open_wire_result_t ow = {0};
        voltage_fault_state_t post_ow_voltage_fault;
        uint32_t restore_tick;
        int ow_result;

        /* While the Rev5 S2N-S15N routing defect exists, use the C path. The
         * accumulator wrapper treats baseline/even/odd as one diagnostic and
         * always performs a normal checked conversion afterward. No open-wire
         * image is ever published as a cell-voltage image. */
        ow_result = accumulator_run_c_open_wire_diagnostic(acc, &ow);
        status = (ow_result == 0) ? HAL_OK : HAL_ERROR;
        restore_tick = osKernelGetTickCount();
        post_ow_voltage_fault = data->voltage_fault_state;
        voltage_fault_update(&post_ow_voltage_fault, acc);
        adbms_task_publish_voltage_state(data,
                                         &post_ow_voltage_fault,
                                         acc);
        taskENTER_CRITICAL();
        data->adbms_last_diag_status = status;
        data->adbms_open_wire_last_path = ow.path;
        data->adbms_open_wire_restore_fault = !ow.restored_normal_c_image;
        data->adbms_last_voltage_scan_ok = ow.restored_normal_c_image;
        if(ow.restored_normal_c_image && acc->voltage_full_updated)
        {
            data->adbms_c_last_valid_tick = restore_tick;
        }
        if(data->adbms_open_wire_diag_count != UINT32_MAX)
        {
            data->adbms_open_wire_diag_count++;
        }
        for(uint8_t ic = 0u; ic < (uint8_t)smb->num_ics; ic++)
        {
            data->adbms_sense_path_open_mask[ic] = ow.cell_fault_mask[ic];
            data->adbms_sense_path_open_sticky_mask[ic] |=
                ow.cell_fault_mask[ic];
        }
        if(status != HAL_OK)
        {
            /* This means electrical sense-path open/incomplete/restore failure.
             * Possible physical causes include the 1 A fuse, harness, cell tab,
             * connector, PCB trace, filter resistor or solder joint. */
            data->adbms_open_wire_fault = true;
        }
        taskEXIT_CRITICAL();
    }


#if AMS_ENABLE_ADBMS_AUX2_REDUNDANCY
    if(acc->smb_transport_ready && accumulator_final_ring_topology_valid(acc))
    {
        const uint32_t aux2_period_ms =
            1000u / AMS_ADBMS_AUX2_POSITIONS_PER_SECOND;
        uint32_t now = osKernelGetTickCount();
        if(adbms_tick_due(now, data->adbms_aux2_next_due_tick))
        {
            uint8_t sensor = data->adbms_aux2_next_sensor;
            HAL_StatusTypeDef aux2_status =
                adbms6830_run_aux2_redundancy(smb, sensor);
            /* Advance the absolute due time instead of setting last=now.
             * At the 10 Hz vehicle scan and a 250 ms target this produces a
             * 300/200 ms cadence whose average is the requested 4 positions/s
             * instead of quantizing permanently to 3.33 positions/s. Bench
             * 1 Hz naturally executes at most one diagnostic per scan. */
            data->adbms_aux2_next_due_tick += aux2_period_ms;
            data->adbms_aux2_next_sensor =
                (uint8_t)((sensor + 1u) % ADBMS6830_TEMP_SENSOR_COUNT);
            if(data->adbms_aux2_diag_count != UINT32_MAX)
            {
                data->adbms_aux2_diag_count++;
            }
            if((aux2_status != HAL_OK) &&
               (data->adbms_aux2_diag_fail_count != UINT32_MAX))
            {
                /* Diagnostic-only until the AUX/AUX2 mismatch threshold is
                 * characterized on the complete mux/NTC network. Primary AUX
                 * temperature safety authority is not silently replaced. */
                data->adbms_aux2_diag_fail_count++;
            }
        }
    }
#endif

#if AMS_ENABLE_ADBMS_THERM_OPEN_WIRE_DIAG
    if(acc->smb_transport_ready && accumulator_final_ring_topology_valid(acc))
    {
        uint32_t now = osKernelGetTickCount();
        if((data->adbms_therm_ow_last_tick == 0u) ||
           ((uint32_t)(now - data->adbms_therm_ow_last_tick) >=
            AMS_ADBMS_THERM_OPEN_WIRE_PERIOD_MS))
        {
            uint8_t sensor = data->adbms_therm_ow_next_sensor;
            HAL_StatusTypeDef therm_ow_status =
                adbms6830_run_thermistor_open_wire(smb, sensor);
            data->adbms_therm_ow_last_tick = now;
            data->adbms_therm_ow_next_sensor =
                (uint8_t)((sensor + 1u) % ADBMS6830_TEMP_SENSOR_COUNT);
            if(data->adbms_therm_ow_diag_count != UINT32_MAX)
            {
                data->adbms_therm_ow_diag_count++;
            }
            if((therm_ow_status != HAL_OK) &&
               (data->adbms_therm_ow_diag_fail_count != UINT32_MAX))
            {
                /* Also observational until the stimulated mux/NTC response is
                 * hardware characterized. Never promote the OW-stimulated AUX
                 * sample into the normal temperature image. */
                data->adbms_therm_ow_diag_fail_count++;
            }
        }
    }
#endif

#if AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && AMS_S_PATH_ECO_VALIDATED
    if(acc->smb_transport_ready && accumulator_final_ring_topology_valid(acc))
    {
        uint32_t now = osKernelGetTickCount();
        if((data->adbms_s_diag_last_tick == 0u) ||
           ((uint32_t)(now - data->adbms_s_diag_last_tick) >=
            AMS_ADBMS_S_DIAGNOSTIC_PERIOD_MS))
        {
            HAL_StatusTypeDef s_status = adbms6830_run_s_periodic_diagnostic(smb);
            data->adbms_s_diag_last_tick = now;
            if(s_status != HAL_OK)
            {
                /* Once the S hardware ECO is explicitly validated and this
                 * feature is enabled, failed redundancy/open-wire coverage is
                 * a real measurement-path diagnostic fault. */
                data->adbms_open_wire_fault = true;
                data->adbms_last_diag_status = s_status;
            }
        }
    }
#endif

    taskENTER_CRITICAL();
    data->adbms_diag_fault = adbms_task_diag_fault_from_components(data);
    taskEXIT_CRITICAL();
}

static uint32_t adbms_task_pack_cell_extremes(const app_data_t *data)
{
    if(data == NULL)
    {
        return 0u;
    }

    return (((uint32_t)data->max_voltage_seg & 0xFFu) << 24) |
           (((uint32_t)data->max_voltage_cell & 0xFFu) << 16) |
           (((uint32_t)data->min_voltage_seg & 0xFFu) << 8) |
           ((uint32_t)data->min_voltage_cell & 0xFFu);
}

static uint32_t adbms_task_pack_temp_extremes(const app_data_t *data)
{
    if(data == NULL)
    {
        return 0u;
    }

    return (((uint32_t)data->max_temp_seg & 0xFFu) << 24) |
           (((uint32_t)data->max_temp_sensor & 0xFFu) << 16) |
           (((uint32_t)data->min_temp_seg & 0xFFu) << 8) |
           ((uint32_t)data->min_temp_sensor & 0xFFu);
}

static uint16_t adbms_task_diag_reason_bits(const app_data_t *data)
{
    uint16_t reason = 0u;

    if(data == NULL)
    {
        return reason;
    }

    if(data->adbms_config_fault)
    {
        reason |= 0x0001u;
    }
    if(data->adbms_status_fault)
    {
        reason |= 0x0002u;
    }
    if(data->adbms_open_wire_fault)
    {
        reason |= 0x0004u;
    }
    if(data->adbms_balance_write_fault)
    {
        reason |= 0x0008u;
    }

    return reason;
}

static uint16_t adbms_task_fault_active_mask(const app_data_t *data)
{
    uint16_t mask = 0u;
    const adbms6830_diag_health_t *health;
    const adbms6830_driver_t *smb;
    uint16_t expected_ic_mask = 0u;
    uint16_t expected_cell_count;
    uint32_t now;

    if(data == NULL)
    {
        return AMS_ADBMS_FAULT_TOPOLOGY;
    }

    smb = &data->acc.smb;
    health = adbms6830_diag_health_get(smb);
    now = osKernelGetTickCount();
    if((smb->num_ics > 0) &&
       (smb->num_ics <= (int)ADBMS6830_MAX_TRACKED_ICS))
    {
        expected_ic_mask = (uint16_t)((1u << (uint8_t)smb->num_ics) - 1u);
    }
    expected_cell_count = (uint16_t)((uint16_t)smb->monitored_cell_count *
                                     (uint16_t)((smb->num_ics > 0) ? smb->num_ics : 0));

#if AMS_HIL_REPLACE_ADBMS
    /* In CAN-fed HIL there is no physical ADBMS chain to classify.  Preserve
     * the same mask vocabulary, but derive it only from injected-image
     * freshness and the CAN transport. */
    if(data->can_busoff_fault || data->can_recover_pending ||
       !data->adbms_last_voltage_scan_ok)
    {
        mask |= AMS_ADBMS_FAULT_COMMUNICATION;
    }
    if(!data->acc.voltage_full_updated ||
       (data->voltage_updated_cell_count == 0u))
    {
        mask |= AMS_ADBMS_FAULT_C_DATA_INVALID;
    }
    if(data->voltage_stale_cell_count != 0u)
    {
        mask |= AMS_ADBMS_FAULT_CELL_DATA_STALE;
    }
    if(!data->temp_valid || data->temp_read_fault)
    {
        mask |= AMS_ADBMS_FAULT_TEMP_UNAVAILABLE;
    }
    return mask;
#endif

    if(!accumulator_final_ring_topology_valid(&data->acc) ||
       (expected_ic_mask == 0u))
    {
        mask |= AMS_ADBMS_FAULT_TOPOLOGY;
    }
    if(data->adbms_voltage_scan_attempted &&
       !data->adbms_last_voltage_scan_ok)
    {
        mask |= AMS_ADBMS_FAULT_COMMUNICATION;
    }
    if(data->adbms_config_fault ||
       ((data->adbms_config_last_valid_tick != 0u) &&
        ((now - data->adbms_config_last_valid_tick) >
         ADBMS_CONFIG_AUTH_MAX_AGE_MS)))
    {
        mask |= AMS_ADBMS_FAULT_CONFIG_READBACK;
    }
    if((data->adbms_status_last_valid_tick != 0u) &&
       ((now - data->adbms_status_last_valid_tick) >
        ADBMS_STATUS_AUTH_MAX_AGE_MS))
    {
        mask |= AMS_ADBMS_FAULT_STATUS;
    }
    if((data->adbms_identity_last_valid_tick != 0u) &&
       ((now - data->adbms_identity_last_valid_tick) >
        ADBMS_IDENTITY_AUTH_MAX_AGE_MS))
    {
        mask |= AMS_ADBMS_FAULT_IDENTITY;
    }
    if(data->adbms_open_wire_fault)
    {
        mask |= AMS_ADBMS_FAULT_OPEN_WIRE;
    }
    if(data->adbms_balance_write_fault)
    {
        mask |= AMS_ADBMS_FAULT_BALANCE_WRITE;
    }
    if(!data->temp_valid || data->temp_read_fault)
    {
        mask |= AMS_ADBMS_FAULT_TEMP_UNAVAILABLE;
    }
    if(data->voltage_stale_cell_count != 0u)
    {
        mask |= AMS_ADBMS_FAULT_CELL_DATA_STALE;
    }
    if(!data->acc.voltage_full_updated ||
       (data->voltage_updated_cell_count < expected_cell_count))
    {
        mask |= AMS_ADBMS_FAULT_C_DATA_INVALID;
    }

    if(health != NULL)
    {
        if(health->last_pec_fail_mask != 0u)
        {
            mask |= AMS_ADBMS_FAULT_PEC;
        }
        if(health->last_cmd_counter_mismatch_mask != 0u)
        {
            mask |= AMS_ADBMS_FAULT_COMMAND_COUNTER;
        }
        if(health->unexpected_counter_reset_mask != 0u)
        {
            mask |= AMS_ADBMS_FAULT_DEVICE_RESET;
        }
        if((health->config_mismatch_mask |
            health->config_write_guard_fault_mask) != 0u)
        {
            mask |= AMS_ADBMS_FAULT_CONFIG_READBACK;
        }
        if((health->reference_invalid_ic_mask |
            health->reference_fault_ic_mask) != 0u)
        {
            mask |= AMS_ADBMS_FAULT_REFERENCE;
        }
        if(health->cs_fault_ic_mask != 0u)
        {
            mask |= AMS_ADBMS_FAULT_S_REDUNDANCY;
        }
        if((health->status_invalid_ic_mask |
            health->supply_flag_fault_ic_mask |
            health->memory_fault_ic_mask |
            health->digital_fault_ic_mask |
            health->oscillator_counter_fault_ic_mask) != 0u)
        {
            mask |= AMS_ADBMS_FAULT_STATUS;
        }
        if(((health->sid_valid_ic_mask & expected_ic_mask) != expected_ic_mask) ||
           ((health->sid_identity_mismatch_ic_mask & expected_ic_mask) != 0u))
        {
            mask |= AMS_ADBMS_FAULT_IDENTITY;
        }
    }

#if AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP
    mask |= AMS_ADBMS_FAULT_VOLTAGE_DEGRADED;
#endif
#if AMS_ENABLE_ADBMS_FAULT_INJECTION
    mask |= data->adbms_fault_injection_mask;
#endif
    return mask;
}

static void adbms_task_update_fault_classification(app_data_t *data)
{
    uint16_t active;
    uint16_t previous_active;
    uint16_t new_bits;
    uint16_t cleared_bits;
    uint16_t latched_after;
    uint16_t c_authority;
    uint16_t reset_mask = 0u;
    uint32_t driver_reset_total;
    uint32_t driver_reset_seen;
    uint32_t reset_delta;
    uint32_t previous_reset_total;
    uint32_t updated_reset_total;
    uint32_t now;
    bool log_fault_change = false;
    bool log_recovery = false;
    bool log_device_reset = false;
    ams_adbms_state_t next_state;
    ams_adbms_state_reason_t state_reason;

    if(data == NULL)
    {
        return;
    }

    now = osKernelGetTickCount();
    taskENTER_CRITICAL();
    previous_reset_total = data->adbms_device_reset_count;
    driver_reset_seen = data->adbms_device_reset_driver_count_seen;
    taskEXIT_CRITICAL();

    active = adbms_task_fault_active_mask(data);
    c_authority = adbms_task_compute_c_authority(data);
    driver_reset_total = adbms_task_device_reset_total(&data->acc.smb,
                                                       &reset_mask);
    /* The service-level diagnostic counter can be cleared independently of
     * the application.  Accumulate deltas so the application-level count is
     * monotonic across a diagnostic clear and still recognizes the first new
     * reset after that clear. */
    reset_delta = (driver_reset_total >= driver_reset_seen) ?
                  (driver_reset_total - driver_reset_seen) :
                  driver_reset_total;
    updated_reset_total = previous_reset_total;
    if(UINT32_MAX - updated_reset_total < reset_delta)
    {
        updated_reset_total = UINT32_MAX;
    }
    else
    {
        updated_reset_total += reset_delta;
    }
    if(reset_delta != 0u)
    {
        /* A later checked read can legitimately clear the driver's current
         * per-transaction reset mask before classification runs. Preserve the
         * newly observed reset for this scan using the accumulated counter. */
        active |= AMS_ADBMS_FAULT_DEVICE_RESET;
    }

    taskENTER_CRITICAL();
    previous_active = data->adbms_fault_active_mask;
    new_bits = (uint16_t)(active & (uint16_t)~previous_active);
    cleared_bits = (uint16_t)(previous_active & (uint16_t)~active);

    data->adbms_fault_active_mask = active;
    data->adbms_fault_latched_mask |= active;
    latched_after = data->adbms_fault_latched_mask;
    data->adbms_c_authority_mask = c_authority;
    data->adbms_c_authority_valid =
        ((c_authority & AMS_ADBMS_C_AUTH_REQUIRED_MASK) ==
         AMS_ADBMS_C_AUTH_REQUIRED_MASK);

    if((new_bits | cleared_bits) != 0u)
    {
        if(data->adbms_fault_transition_count != UINT32_MAX)
        {
            data->adbms_fault_transition_count++;
        }
        log_fault_change = true;
    }

    if((new_bits & (uint16_t)~AMS_ADBMS_FAULT_VOLTAGE_DEGRADED) != 0u)
    {
        uint16_t first_candidates =
            (uint16_t)(new_bits & (uint16_t)~AMS_ADBMS_FAULT_VOLTAGE_DEGRADED);
        if(data->adbms_first_fault_mask == 0u)
        {
            data->adbms_first_fault_mask = first_candidates;
            data->adbms_first_fault_tick = now;
        }
        data->adbms_last_fault_tick = now;
    }

    if(((previous_active & ADBMS_LIFECYCLE_BLOCKERS) != 0u) &&
       ((active & ADBMS_LIFECYCLE_BLOCKERS) == 0u))
    {
        data->adbms_last_recovery_tick = now;
        log_recovery = true;
    }

    data->adbms_device_reset_count = updated_reset_total;
    data->adbms_device_reset_driver_count_seen = driver_reset_total;
    if(reset_delta != 0u)
    {
        data->adbms_last_device_reset_mask = reset_mask;
    }
    log_device_reset = (reset_delta != 0u);
    if(log_device_reset)
    {
        data->adbms_diag_fault = true;
        data->adbms_last_diag_status = HAL_ERROR;
    }

#if AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP
    data->adbms_voltage_redundancy_degraded = true;
#else
    data->adbms_voltage_redundancy_degraded = false;
#endif

#if AMS_ENABLE_ADBMS_FAULT_INJECTION
    if(data->adbms_fault_injection_mask != 0u)
    {
        data->adbms_diag_fault = true;
        data->adbms_last_diag_status = HAL_ERROR;
    }
#endif

    if((active & ADBMS_LIFECYCLE_BLOCKERS) != 0u)
    {
        next_state = AMS_ADBMS_STATE_FAULTED;
        state_reason = AMS_ADBMS_STATE_REASON_FAULT_ACTIVE;
    }
    else if(!data->acc.smb_ready)
    {
        next_state = AMS_ADBMS_STATE_WAKING;
        state_reason = AMS_ADBMS_STATE_REASON_SCAN_BEGIN;
    }
    else if(data->adbms_c_authority_valid)
    {
#if AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP
        next_state = AMS_ADBMS_STATE_READY_C_ONLY_DEGRADED;
#else
        next_state = (data->voltage_valid && !data->voltage_read_fault) ?
                     AMS_ADBMS_STATE_READY_REDUNDANT :
                     AMS_ADBMS_STATE_MEASURING;
#endif
        state_reason = AMS_ADBMS_STATE_REASON_MEASUREMENT_OK;
    }
    else
    {
        next_state = AMS_ADBMS_STATE_MEASURING;
        state_reason = AMS_ADBMS_STATE_REASON_SCAN_BEGIN;
    }
    taskEXIT_CRITICAL();

    if(log_fault_change)
    {
        ams_fault_log_event(AMS_FAULT_LOG_ADBMS_FAULT_CHANGE,
                            new_bits,
                            active,
                            (((uint32_t)cleared_bits) << 16u) | latched_after);
    }
    if(log_recovery)
    {
        ams_fault_log_event(AMS_FAULT_LOG_ADBMS_FAULT_CHANGE,
                            0u,
                            active,
                            cleared_bits);
    }
    if(log_device_reset)
    {
        ams_fault_log_event(AMS_FAULT_LOG_ADBMS_DEVICE_RESET,
                            reset_mask,
                            updated_reset_total,
                            previous_reset_total);
    }

    ams_adbms_transition_state(data, next_state, state_reason, now);
}

bool adbms_record_balance_write_result(app_data_t *data, int result)
{
    bool was_faulted;
    uint16_t reason_bits = 0u;
    uint32_t fail_count = 0u;

    if(data == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    was_faulted = data->adbms_balance_write_fault;
    if(result == 0)
    {
        data->adbms_balance_write_fault = false;
        data->adbms_diag_fault = adbms_task_diag_fault_from_components(data);
        taskEXIT_CRITICAL();
        return true;
    }

    data->adbms_balance_write_fault = true;
    if(data->adbms_balance_write_fail_count != UINT32_MAX)
    {
        data->adbms_balance_write_fail_count++;
    }
    data->adbms_last_diag_status = HAL_ERROR;
    data->adbms_diag_fault = true;
    reason_bits = adbms_task_diag_reason_bits(data);
    fail_count = data->adbms_balance_write_fail_count;
    taskEXIT_CRITICAL();
    if(!was_faulted)
    {
        ams_fault_log_event(AMS_FAULT_LOG_ADBMS_DIAG_FAIL,
                            reason_bits,
                            (uint32_t)HAL_ERROR,
                            fail_count);
    }
    set_bms(false);
    return false;
}

static void adbms_task_publish_voltage_state(app_data_t *data,
                                             const voltage_fault_state_t *fault,
                                             const accumulator_t *acc)
{
    if((data == NULL) || (fault == NULL) || (acc == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    data->voltage_fault_state = *fault;
    data->max_voltage = acc->max_volt;
    data->min_voltage = acc->min_volt;
    data->total_voltage = acc->total_volt;
    data->voltage_valid = fault->voltage_valid;
    data->voltage_read_fault = fault->read_fault;
    data->voltage_read_fault_pending = fault->read_fault_pending;
    data->voltage_read_fault_streak = fault->read_fault_streak;
    data->voltage_warning = fault->warning;
    data->charge_voltage_stop = fault->charge_stop;
    data->overvoltage_fault = fault->overvoltage_fault;
    data->undervoltage_fault = fault->undervoltage_fault;
    data->voltage_fault_latched = fault->latched;
    data->voltage_fault_reason = fault->reason;
    data->voltage_fault_latched_reason = fault->latched_reason;
    data->voltage_usable_cell_count = fault->usable_cell_count;
    data->voltage_updated_cell_count = fault->updated_cell_count;
    data->voltage_stale_cell_count = fault->stale_cell_count;
    data->voltage_pec_fail_cell_count = fault->pec_fail_cell_count;
    data->voltage_jump_cell_count = acc->voltage_jump_cell_count;
    data->voltage_stuck_cell_count = acc->voltage_stuck_cell_count;
    data->voltage_max_delta_mv = acc->voltage_max_delta_mv;
    data->voltage_max_delta_seg = acc->voltage_max_delta_seg;
    data->voltage_max_delta_cell = acc->voltage_max_delta_cell;
    data->max_voltage_seg = fault->max_cell_segment;
    data->max_voltage_cell = fault->max_cell_index;
    data->min_voltage_seg = fault->min_cell_segment;
    data->min_voltage_cell = fault->min_cell_index;

    data->voltage_fault = ((fault->read_fault &&
                            !fault->read_fault_pending) ||
                           fault->overvoltage_fault ||
                           fault->undervoltage_fault ||
                           fault->latched);
    taskEXIT_CRITICAL();
}

static void adbms_task_publish_temperature_state(app_data_t *data,
                                                 const temperature_fault_state_t *fault,
                                                 const accumulator_t *acc)
{
    if((data == NULL) || (fault == NULL) || (acc == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    data->temp_fault_state = *fault;
    data->max_temp = acc->max_temp;
    data->avg_temp = acc->avg_temp;
    data->temp_valid = fault->temp_valid;
    data->temp_read_fault = fault->read_fault;
    data->temp_warning = fault->warning;
    data->temp_fan_max = fault->fan_max;
    data->temp_charge_stop = fault->charge_stop;
    data->temp_overtemp_pending = fault->pending;
    data->overtemp_fault = fault->overtemp_fault;
    data->severe_overtemp_fault = fault->severe_overtemp_fault;
    data->temp_fault_latched = fault->latched;
    data->temp_fault_reason = fault->reason;
    data->temp_fault_pending_reason = fault->pending_reason;
    data->temp_fault_latched_reason = fault->latched_reason;
    data->temp_fault_pending_ms = fault->pending_ms;
    data->temp_usable_sensor_count = fault->usable_sensor_count;
    data->temp_updated_sensor_count = fault->updated_sensor_count;
    data->temp_stale_sensor_count = fault->stale_sensor_count;
    data->temp_invalid_sensor_count = fault->invalid_sensor_count;
    data->temp_open_sensor_count = fault->open_sensor_count;
    data->temp_short_sensor_count = fault->short_sensor_count;
    data->temp_jump_sensor_count = fault->jump_sensor_count;
    data->temp_rate_rise_sensor_count = fault->rate_rise_sensor_count;
    data->temp_filtered_max = (float)fault->filtered_max_temp_deci_c / 10.0f;
    data->temp_filtered_avg = (float)fault->filtered_avg_temp_deci_c / 10.0f;
    data->temp_max_rate_c_per_s = (float)fault->max_rate_deci_c_per_s / 10.0f;
    data->temp_max_rate_seg = fault->max_rate_segment;
    data->temp_max_rate_sensor = fault->max_rate_sensor;
    data->max_temp_seg = fault->max_temp_segment;
    data->max_temp_sensor = fault->max_temp_sensor;
    data->min_temp_seg = fault->min_temp_segment;
    data->min_temp_sensor = fault->min_temp_sensor;

    data->temp_fault = (fault->read_fault ||
                        fault->overtemp_fault ||
                        fault->latched);
    taskEXIT_CRITICAL();
}

TaskHandle_t adbms_task_start(app_data_t *data)
{
    if(data == NULL)
    {
        return NULL;
    }

    if(adbms_task_handle == NULL)
    {
        adbms_task_handle = xTaskCreateStatic(adbms_task_fn,
                                              "adbms task",
                                              AMS_STACK_ADBMS_WORDS,
                                              (void *)data,
                                              ADBMS_PRIO,
                                              adbms_task_stack,
                                              &adbms_task_tcb);
    }

    return adbms_task_handle;
}

void adbms_task_fn(void *argument)
{
	app_data_t *data = (app_data_t *) argument;
    if(data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

	accumulator_t *acc = &data->acc;
	uint32_t entry;

    adbms_task_timing_init();
    adbms6830_bind_runtime_hooks(&acc->smb,
                                 adbms_task_cooperative_wait,
                                 adbms_task_time_us,
                                 data);

	for(;;)
	{
	    entry = osKernelGetTickCount();
        uint32_t scan_start_us = adbms_task_time_us(data);
        uint64_t yield_start_us = adbms_task_cooperative_yield_total_us;
        adbms6830_session_begin_scan(&acc->smb);
	    uint16_t balance_mask_at_acquisition[NSMBS] = {0u};
	    uint16_t balance_shadow_plan[NSMBS] = {0u};
	    bool balance_was_active = false;
	    bool balance_recovery_verified = true;
	    uint32_t balance_off_ms = 0u;
	    uint32_t balance_clear_tick = 0u;
	    bool balance_clear_timed = false;
	    ams_current_window_t current_window = {0};
	    uint32_t voltage_complete_tick;
	    uint32_t balance_apply_tick = 0u;
	    /* Own the complete accumulator-image epoch, including the initial
	     * balance-shadow capture. Individual physical accumulator/driver
	     * helpers lock recursively. In CAN-fed HIL builds, the same lock
	     * excludes injected cell/temperature writes until the immutable
	     * snapshot has been published. */
	    adbms_spi_lock();
	    for(uint8_t seg = 0u; seg < NSMBS; seg++)
	    {
	        balance_mask_at_acquisition[seg] =
	            accumulator_balance_shadow_mask(acc, seg);
	        balance_was_active = balance_was_active ||
	                             (balance_mask_at_acquisition[seg] != 0u);
	    }
#if !AMS_HIL_REPLACE_ADBMS
	    bool balance_recovery_required =
	        balance_was_active || data->adbms_balance_write_fault;
#endif

#if !AMS_HIL_REPLACE_ADBMS
        /* A fail-low supervisor decision never blocks in the high-priority
         * task.  Service its queued ASIC-native balance kill here while the
         * ADBMS task owns the recursive transport lock. */
        (void)adbms_task_service_urgent_mute(
            data, ACCUMULATOR_BALANCE_INHIBIT_SHUTDOWN);
#endif
	    data->adbms_scan_active = true;
        /* Do not oscillate READY/FAULTED -> MEASURING -> READY/FAULTED on
         * every periodic scan. Lifecycle events describe meaningful external
         * state changes, not the normal 10 Hz sampling phase. */
        ams_adbms_state_t lifecycle_at_entry = data->adbms_lifecycle_state;
        if((lifecycle_at_entry == AMS_ADBMS_STATE_OFFLINE) ||
           (lifecycle_at_entry == AMS_ADBMS_STATE_WAKING) ||
           (lifecycle_at_entry == AMS_ADBMS_STATE_IDENTIFIED) ||
           (lifecycle_at_entry == AMS_ADBMS_STATE_CONFIGURING) ||
           (lifecycle_at_entry == AMS_ADBMS_STATE_RECOVERING))
        {
            ams_adbms_transition_state(
                data,
                acc->smb_ready ? AMS_ADBMS_STATE_MEASURING :
                                 AMS_ADBMS_STATE_WAKING,
                AMS_ADBMS_STATE_REASON_SCAN_BEGIN,
                entry);
        }
	    /* This counter is also the modulo schedule for periodic diagnostics.
	     * Unsigned wrap is intentional so those diagnostics cannot stop after
	     * the counter reaches its maximum value. */
	    data->adbms_scan_count++;

        bool voltage_was_latched = data->voltage_fault_latched;
        voltage_fault_reason_t voltage_prev_latched_reason = data->voltage_fault_latched_reason;
        bool temp_was_latched = data->temp_fault_latched;
        temperature_fault_reason_t temp_prev_latched_reason = data->temp_fault_latched_reason;
#if !AMS_HIL_REPLACE_ADBMS
        bool adbms_diag_was_faulted = data->adbms_diag_fault;
#endif

	    /* Turn off balancing before reading so cell voltages recover from load.
	     * Do not impose this delay on every non-balancing scan: at 10 Hz the
	     * old unconditional 100 ms wait consumed the complete task period before
	     * conversion, temperature, diagnostics, and SPI work even started. */
#if !AMS_HIL_REPLACE_ADBMS
	    if(balance_recovery_required)
	    {
	        balance_recovery_verified = adbms_record_balance_write_result(
	            data, accumulator_clear_balance(acc));
        /* accumulator_clear_balance() completed the MUTE -> durable zero
         * transaction. Copy that verified hardware state immediately. A
         * successful clear also clears adbms_balance_write_fault, so waiting
         * for the later balance scheduler branch can otherwise leave this
         * application-level qualification bit stale-low indefinitely. */
        data->adbms_mute_asserted = acc->last_balance_mute_ok;
        data->adbms_balance_durable_zero_verified =
            acc->last_balance_durable_zero_verified;
        data->adbms_balance_inhibit_reason =
            (accumulator_balance_inhibit_reason_t)acc->last_balance_inhibit_reason;
	        if(!balance_recovery_verified)
	        {
            adbms_diag_was_faulted = true;
	        }
	        else
	        {
	            balance_clear_tick = osKernelGetTickCount();
	            balance_clear_timed = true;
	            if(data->adbms_balance_active)
	            {
	                data->adbms_last_balance_on_ms =
	                    (uint32_t)(balance_clear_tick -
	                               data->adbms_balance_apply_tick);
	            }
	        }
	        if(data->adbms_balance_recovery_count != UINT32_MAX)
	        {
	            data->adbms_balance_recovery_count++;
	        }
	        osDelay(AMS_ADBMS_BALANCE_RECOVERY_MS);
	    }
#endif

#if AMS_HIL_REPLACE_ADBMS
        accumulator_hil_refresh_update_masks(acc,
                                             osKernelGetTickCount(),
                                             AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS);
        data->adbms_voltage_scan_attempted = true;
        data->adbms_last_voltage_scan_ok = acc->voltage_full_updated;
#else
#if AMS_APM_STANDALONE_EVAL_BENCH
        /* This release is a dedicated one-device ADBMS2950B bench image.  Do
         * not create a synthetic SMB scan failure every cycle; periodically
         * sample the APM directly while keeping all cell-voltage authority
         * invalid and BMS_OK inhibited by the bench profile. */
        taskENTER_CRITICAL();
        data->adbms_voltage_scan_attempted = false;
        data->adbms_last_voltage_scan_ok = false;
        taskEXIT_CRITICAL();
        (void)accumulator_read_apm(acc, osKernelGetTickCount());
#else
        bool voltage_scan_attempted = acc->smb_transport_ready &&
                                      accumulator_final_ring_topology_valid(acc);
        int voltage_scan_result = accumulator_read_volt(acc);
        taskENTER_CRITICAL();
        data->adbms_voltage_scan_attempted = voltage_scan_attempted;
        data->adbms_last_voltage_scan_ok =
            voltage_scan_attempted && (voltage_scan_result == 0);
        if(voltage_scan_attempted && (voltage_scan_result != 0) &&
           (data->adbms_comm_failure_count != UINT32_MAX))
        {
            data->adbms_comm_failure_count++;
        }
        taskEXIT_CRITICAL();
        if(data->adbms_urgent_mute_requested)
        {
            (void)adbms_task_service_urgent_mute(
                data, ACCUMULATOR_BALANCE_INHIBIT_SHUTDOWN);
        }
		if(voltage_scan_result == 0)
		{
			/* The compatible ADCV command also starts the ADBMS2950 conversion.
			 * Read the one-device String-B APM subset only after the successful
			 * SMB transaction has proven the complete ring was awake.  This makes
			 * the three shared SNAP/UNSNAP counter increments deterministic.
			 * Results remain diagnostic only and never enter BMS_OK. */
			(void)accumulator_read_apm(acc, osKernelGetTickCount());
		}
#endif
#endif
        accumulator_update_voltage_stats_at(acc, osKernelGetTickCount());
	    voltage_complete_tick = osKernelGetTickCount();
	    if(balance_clear_timed)
	    {
	        balance_off_ms =
	            (uint32_t)(voltage_complete_tick - balance_clear_tick);
	    }
	    data->adbms_last_balance_off_ms = balance_off_ms;
	    ams_current_window_lock();
	    (void)ams_current_window_rotate(&data->current_window,
	                                    voltage_complete_tick,
	                                    &current_window);
	    ams_current_window_unlock();

        voltage_fault_state_t next_voltage_fault = data->voltage_fault_state;
        voltage_fault_update(&next_voltage_fault, acc);
        adbms_task_publish_voltage_state(data, &next_voltage_fault, acc);

        taskENTER_CRITICAL();
        if(data->adbms_last_voltage_scan_ok && acc->voltage_full_updated)
        {
            data->adbms_c_last_valid_tick = voltage_complete_tick;
        }
        if(adbms_task_all_s_samples_valid(&acc->smb))
        {
            data->adbms_s_last_valid_tick = voltage_complete_tick;
        }
        taskEXIT_CRITICAL();

        if(data->voltage_fault_latched &&
           (!voltage_was_latched ||
            (voltage_prev_latched_reason != data->voltage_fault_latched_reason)))
        {
            ams_fault_log_event(AMS_FAULT_LOG_VOLTAGE_LATCH,
                                (uint16_t)data->voltage_fault_latched_reason,
                                (((uint32_t)data->acc.max_voltage_mv) << 16) |
                                    (uint32_t)data->acc.min_voltage_mv,
                                adbms_task_pack_cell_extremes(data));
        }

        if(data->voltage_fault)
        {
            set_bms(0);
        }
        ams_heartbeat_kick(data, AMS_HEARTBEAT_ADBMS, osKernelGetTickCount());

        /* Parallel-connection degradation is advisory only. It looks for a
         * repeated outlier in apparent group resistance across short current
         * steps; it cannot identify an individual fusible wire bond. */
        ams_parallel_connection_observer_step(
            &data->parallel_connection_observer,
            acc,
            data->current,
            data->current_valid,
            balance_was_active || data->adbms_balance_active,
            voltage_complete_tick);

        /* Main 80 A fuse/HV-path plausibility requires future AIR auxiliary
         * feedback and independent load-side voltage. Current hardware leaves
         * these inputs invalid, so this remains explicitly unavailable and
         * cannot assert fuse_fault. */
        {
            ams_air_monitor_t air_snapshot;
            ams_air_monitor_inputs_t air_inputs_snapshot;
            taskENTER_CRITICAL();
            air_snapshot = data->air_monitor;
            air_inputs_snapshot = data->air_monitor_inputs;
            taskEXIT_CRITICAL();
            ams_main_fuse_monitor_step(&data->main_fuse_monitor,
                                       &air_snapshot,
                                       &air_inputs_snapshot,
                                       data->current,
                                       data->current_valid,
                                       voltage_complete_tick);
#if AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED
            if(data->main_fuse_monitor.authority_valid &&
               data->main_fuse_monitor.confirmed_open)
            {
                data->fuse_fault = true;
            }
#endif
        }

#if !AMS_HIL_REPLACE_ADBMS
        if(data->adbms_urgent_mute_requested)
        {
            (void)adbms_task_service_urgent_mute(
                data, ACCUMULATOR_BALANCE_INHIBIT_VOLTAGE);
        }
#endif
#if AMS_HIL_REPLACE_ADBMS
        accumulator_hil_refresh_update_masks(acc,
                                             osKernelGetTickCount(),
                                             AMS_HIL_ADBMS_IMAGE_TIMEOUT_MS);
#elif AMS_ENABLE_AUTO_TEMP_MUX_SCAN
        (void)accumulator_read_temp(acc);
#else
        /* Bench temperature-bus bring-up is explicit-only.  Do not repeatedly
         * drive GPIO4/SDA and GPIO5/SCL from the periodic task while the
         * pull-up network is under hardware review. */
        for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
        {
            acc->smb.last_temp_updated_mask[ic] = 0u;
        }
#endif
        accumulator_update_temp_stats_at(acc, osKernelGetTickCount());
        temperature_fault_state_t next_temp_fault = data->temp_fault_state;
        temperature_fault_update_with_period(&next_temp_fault,
                                             acc,
                                             AMS_ADBMS_TASK_PERIOD_MS);
        adbms_task_publish_temperature_state(data, &next_temp_fault, acc);

        if(acc->updated_temp_count != 0u)
        {
            taskENTER_CRITICAL();
            data->adbms_temp_last_valid_tick = osKernelGetTickCount();
            taskEXIT_CRITICAL();
        }

        if(data->temp_fault_latched &&
           (!temp_was_latched ||
            (temp_prev_latched_reason != data->temp_fault_latched_reason)))
        {
            ams_fault_log_event(AMS_FAULT_LOG_TEMP_LATCH,
                                (uint16_t)data->temp_fault_latched_reason,
                                (((uint32_t)(uint16_t)data->acc.max_temp_deci_c) << 16) |
                                    (uint32_t)(uint16_t)data->acc.min_temp_deci_c,
                                adbms_task_pack_temp_extremes(data));
        }

	    if(data->temp_fault)
	    {
	        set_bms(0);
	    }
	    ams_heartbeat_kick(data, AMS_HEARTBEAT_TEMP, osKernelGetTickCount());

#if AMS_HIL_REPLACE_ADBMS
	    taskENTER_CRITICAL();
	    data->adbms_config_fault = false;
	    data->adbms_status_fault = false;
	    data->adbms_open_wire_fault = false;
	    data->adbms_open_wire_restore_fault = false;
	    memset(data->adbms_sense_path_open_mask, 0,
	           sizeof(data->adbms_sense_path_open_mask));
	    data->adbms_balance_write_fault = false;
        /* In ADBMS-image HIL mode, CAN is the measurement transport. Do not
         * clear a CAN bus-off/recovery-pending condition inside the ADBMS task
         * just because the last injected image has not aged out yet. */
	    data->adbms_diag_fault = (data->can_busoff_fault || data->can_recover_pending);
	    data->adbms_last_diag_status = data->adbms_diag_fault ? HAL_ERROR : HAL_OK;
	    taskEXIT_CRITICAL();
#else
	    adbms_task_run_periodic_diagnostics(data);
        if(data->adbms_diag_fault && !adbms_diag_was_faulted)
        {
            ams_fault_log_event(AMS_FAULT_LOG_ADBMS_DIAG_FAIL,
                                adbms_task_diag_reason_bits(data),
                                (uint32_t)data->adbms_last_diag_status,
                                data->adbms_scan_count);
        }
#endif

        adbms_task_update_fault_classification(data);

	    (void)accumulator_plan_balance(acc, balance_shadow_plan);
	    taskENTER_CRITICAL();
	    memcpy(data->adbms_balance_shadow_plan,
	           balance_shadow_plan,
	           sizeof(data->adbms_balance_shadow_plan));
	    data->adbms_balance_shadow_plan_tick = osKernelGetTickCount();
	    taskEXIT_CRITICAL();

	    /* The active measurement transport does not change the fail-closed
	     * action.  In physical-SPI builds this covers ADBMS diagnostics; in
	     * CAN-fed HIL builds it covers a bus-off/recovery-pending condition.
	     * Keep this outside the compile-time split so neither profile can retain
	     * a previously asserted BMS_OK while its transport is unhealthy. */
	    if(data->adbms_diag_fault)
	    {
	        set_bms(0);
	    }

	    /* Do not assert BMS_OK from a measurement task.  The error/safety
	     * supervisor owns the complete readiness decision and is the only task
	     * permitted to drive the output high. */

#if !AMS_HIL_REPLACE_ADBMS
        if(data->adbms_urgent_mute_requested)
        {
            accumulator_balance_inhibit_reason_t reason =
                data->temp_fault ? ACCUMULATOR_BALANCE_INHIBIT_TEMP :
                (data->adbms_diag_fault ? ACCUMULATOR_BALANCE_INHIBIT_ADBMS_HEALTH :
                                          ACCUMULATOR_BALANCE_INHIBIT_SHUTDOWN);
            (void)adbms_task_service_urgent_mute(data, reason);
        }
#endif

        /* Voltage charge-stop can still balance; hard faults/temp stop cannot. */
#if !AMS_HIL_REPLACE_ADBMS
	        bool balance_applied_active = false;
	        if(AMS_PROFILE_BALANCE_RUNTIME_AUTHORITY_ALLOWED &&
               (data->state == STATE_CHARGE) &&
	           !data->balance_inhibit &&
	           data->voltage_valid &&
	           !data->voltage_read_fault &&
	           !data->voltage_fault &&
           data->temp_valid &&
           !data->temp_charge_stop &&
           !data->temp_fault &&
           !data->hard_fault &&
           !data->current_fault &&
           data->current_valid &&
           data->bms_state)
        {
            bool balance_write_ok = adbms_record_balance_write_result(
                data, accumulator_set_balance(acc));
            bool balance_shadow_active = accumulator_balance_shadow_active(acc);
            balance_applied_active = balance_write_ok &&
                                     acc->last_balance_unmute_ok &&
                                     balance_shadow_active;
            if(balance_applied_active)
            {
                data->adbms_mute_asserted = false;
                data->adbms_balance_durable_zero_verified = false;
                data->adbms_balance_inhibit_reason =
                    ACCUMULATOR_BALANCE_INHIBIT_NONE;
            }
            else
            {
                /* A valid zero-cell balance plan remains MUTE + durable zero.
                 * Synchronize the verified state instead of treating every
                 * successful accumulator_set_balance() call as an UNMUTE. */
                data->adbms_mute_asserted = acc->last_balance_mute_ok;
                data->adbms_balance_durable_zero_verified =
                    acc->last_balance_durable_zero_verified;
                data->adbms_balance_inhibit_reason =
                    (accumulator_balance_inhibit_reason_t)
                        acc->last_balance_inhibit_reason;
            }
        }
        else if(data->adbms_balance_active ||
                accumulator_balance_shadow_active(acc) ||
                data->adbms_balance_write_fault)
        {
	        (void)adbms_record_balance_write_result(data,
                                                   accumulator_clear_balance(acc));
            data->adbms_mute_asserted = acc->last_balance_mute_ok;
            data->adbms_balance_durable_zero_verified =
                acc->last_balance_durable_zero_verified;
            data->adbms_balance_inhibit_reason =
                (accumulator_balance_inhibit_reason_t)acc->last_balance_inhibit_reason;
	    }
	        data->adbms_balance_active = balance_applied_active;
	        if(balance_applied_active)
	        {
	            balance_apply_tick = osKernelGetTickCount();
	            data->adbms_balance_apply_tick = balance_apply_tick;
	        }
#else
	        const bool balance_applied_active = false;
	        data->adbms_balance_active = false;
#endif

	        uint32_t measurement_flags = 0u;
	        if(data->voltage_valid && !data->voltage_read_fault)
	        {
	            measurement_flags |= AMS_MEAS_VALID_VOLTAGE;
	        }
	        if(data->temp_valid && !data->temp_read_fault)
	        {
	            measurement_flags |= AMS_MEAS_VALID_TEMPERATURE;
	        }
	        if(current_window.valid)
	        {
	            measurement_flags |= AMS_MEAS_VALID_CURRENT;
	        }
	        if(balance_recovery_verified)
	        {
	            measurement_flags |= AMS_MEAS_BALANCE_RECOVERED;
	        }
	        if(balance_was_active)
	        {
	            measurement_flags |= AMS_MEAS_BALANCE_WAS_ACTIVE;
	        }

	        uint32_t publication_tick = osKernelGetTickCount();
	        ams_measurement_snapshot_t *measurement =
	            ams_measurement_store_begin_write(&data->measurement_store);
	        ams_measurement_snapshot_prepare(measurement,
	                                         acc,
	                                         &current_window,
	                                         entry,
	                                         voltage_complete_tick,
	                                         publication_tick,
	                                         balance_mask_at_acquisition,
	                                         balance_off_ms,
	                                         measurement_flags);
	        (void)ams_measurement_store_publish(&data->measurement_store,
	                                            measurement);

	        data->adbms_scan_active = false;
        adbms6830_session_end_scan(&acc->smb);
	        adbms_spi_unlock();
	        uint32_t scan_end = osKernelGetTickCount();
        uint32_t scan_end_us = adbms_task_time_us(data);
        uint64_t yielded_us64 =
            adbms_task_cooperative_yield_total_us - yield_start_us;
        uint32_t yielded_us = (yielded_us64 > UINT32_MAX) ?
                              UINT32_MAX : (uint32_t)yielded_us64;
        uint32_t wall_us = (uint32_t)(scan_end_us - scan_start_us);
        uint32_t cpu_us = (wall_us > yielded_us) ?
                          (wall_us - yielded_us) : 0u;
        data->adbms_last_scan_yield_us = yielded_us;
        data->adbms_last_scan_cpu_us = cpu_us;
        if(yielded_us > data->adbms_max_scan_yield_us)
        {
            data->adbms_max_scan_yield_us = yielded_us;
        }
        if(cpu_us > data->adbms_max_scan_cpu_us)
        {
            data->adbms_max_scan_cpu_us = cpu_us;
        }

	        uint32_t deadline = entry + AMS_ADBMS_TASK_PERIOD_MS;
	        if(balance_applied_active)
	        {
	            /* Base the minimum on-time on the verified write/readback time,
	             * not the scan entry. This prevents an overrun from setting PWM
	             * and immediately clearing it in the next iteration. */
	            uint32_t balance_deadline =
	                balance_apply_tick + AMS_ADBMS_BALANCE_MIN_ON_MS;
	            if((int32_t)(balance_deadline - deadline) > 0)
	            {
	                deadline = balance_deadline;
	            }
	        }

	        uint32_t duration_ms = (uint32_t)(scan_end - entry);
	        data->adbms_last_scan_duration_ms = duration_ms;
	        if(duration_ms > data->adbms_max_scan_duration_ms)
	        {
	            data->adbms_max_scan_duration_ms = duration_ms;
	        }
	        data->adbms_last_schedule_interval_ms = (uint32_t)(deadline - entry);
	        if((int32_t)(scan_end - deadline) > 0)
	        {
	            if(data->adbms_scan_deadline_miss_count != UINT32_MAX)
	            {
	                data->adbms_scan_deadline_miss_count++;
	            }
	        }
	        osDelayUntil(deadline);
	}
}
