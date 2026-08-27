/*
 * accumulator.c
 *
 *  Created on: Feb 1, 2024
 *      Author: cole
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#include "ext_drivers/accumulator.h"
#include "ams_build_profile.h"
#include "ext_drivers/thermistor_model.h"
#include <math.h>
#include <string.h>

static uint8_t sensor_num = 0;

uint8_t accumulator_configured_smb_count(const accumulator_t *dev)
{
    if((dev == NULL) ||
       (dev->smb.ics == NULL) ||
       (dev->smb.num_ics <= 0) ||
       (dev->smb.num_ics > NSMBS) ||
       (dev->smb.num_ics > (int)ADBMS6830_MAX_TRACKED_ICS) ||
       (dev->smb.ics_capacity < (uint8_t)dev->smb.num_ics))
    {
        return 0u;
    }

    return (uint8_t)dev->smb.num_ics;
}

bool accumulator_final_ring_topology_valid(const accumulator_t *dev)
{
    if((dev == NULL) ||
       (dev->smb.num_ics != NSMBS) ||
       (dev->smb.physical_chain_count != (uint8_t)ACCUMULATOR_PHYSICAL_CHAIN_COUNT) ||
       (dev->smb.ics_capacity != NSMBS) ||
       (dev->smb.ics != dev->smb_ics) ||
       (dev->smb.string != ACCUMULATOR_SMB_STRING) ||
       (dev->smb.write_string != ACCUMULATOR_SMB_STRING) ||
       (dev->smb.hspi == NULL) ||
       (dev->smb.htim == NULL) ||
       (dev->smb.cs_port[STRING_A] == NULL) ||
       (dev->smb.cs_port[STRING_B] == NULL) ||
       (dev->smb.cs_pin[STRING_A] == 0u) ||
       (dev->smb.cs_pin[STRING_B] == 0u))
    {
        return false;
    }

#if AMS_ENABLE_APM_2950 && !AMS_HIL_REPLACE_ADBMS
    if((dev->apm.num_ics != NAPMS) ||
       (dev->apm.ics_capacity != NAPMS) ||
       (dev->apm.ics != dev->apm_ics) ||
       (dev->apm.string != STRING_B) ||
       (dev->apm.write_string != STRING_B) ||
       (dev->apm.hspi != dev->smb.hspi) ||
       (dev->apm.htim != dev->smb.htim) ||
       (dev->apm.cs_port[STRING_A] != dev->smb.cs_port[STRING_A]) ||
       (dev->apm.cs_port[STRING_B] != dev->smb.cs_port[STRING_B]) ||
       (dev->apm.cs_pin[STRING_A] != dev->smb.cs_pin[STRING_A]) ||
       (dev->apm.cs_pin[STRING_B] != dev->smb.cs_pin[STRING_B]))
    {
        return false;
    }
#endif

    return true;
}

int smb_read_voltage(adbms6830_driver_t* dev);
static int smb_read_voltage_checked(adbms6830_driver_t *dev,
                                    bool *compatible_adi1_sent);
int smb_read_temp(adbms6830_driver_t* dev);

static void accumulator_invalidate_apm_sample(accumulator_t *dev)
{
	if(dev == NULL)
	{
		return;
	}

	/* Preserve the last numeric sample and timestamp for post-fault analysis,
	 * but never leave them advertised as belonging to the current scan. */
	dev->apm.health.sample_valid = false;
	dev->apm.health.current_valid = false;
	dev->apm.health.pack_voltage_valid = false;
	dev->apm.health.counter_advanced = false;
}

static void accumulator_invalidate_cell_voltage_authority(accumulator_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

    /* A C-path open-wire conversion overwrites the authoritative C result
     * registers.  If the mandatory restoring conversion fails, no previously
     * cached sample may remain advertised as current merely until its normal
     * freshness timeout expires.  Preserve numeric values/timestamps for
     * service forensics, but invalidate every publication and driver mask
     * immediately. */
    memset(dev->cell_voltage_valid, 0, sizeof(dev->cell_voltage_valid));
    memset(dev->updated_voltage_mask, 0, sizeof(dev->updated_voltage_mask));
    memset(dev->usable_voltage_mask, 0, sizeof(dev->usable_voltage_mask));
    memset(dev->pec_fail_voltage_mask, 0, sizeof(dev->pec_fail_voltage_mask));
    memset(dev->stale_voltage_mask, 0, sizeof(dev->stale_voltage_mask));
    for(uint8_t seg = 0u;
        seg < (uint8_t)(((dev->smb.num_ics > 0) &&
                         (dev->smb.num_ics <= NSMBS)) ?
                            dev->smb.num_ics : 0);
        seg++)
    {
        dev->stale_voltage_mask[seg] =
            (NCELLS >= 16u) ? UINT16_MAX :
            (uint16_t)((1u << NCELLS) - 1u);
    }
    memset(dev->smb.last_cell_updated_mask, 0,
           sizeof(dev->smb.last_cell_updated_mask));
    memset(dev->smb.last_cell_pec_mask, 0,
           sizeof(dev->smb.last_cell_pec_mask));
    dev->updated_voltage_count = 0u;
    dev->usable_voltage_count = 0u;
    dev->stale_voltage_count = (uint16_t)(NCELLS *
        (uint16_t)(((dev->smb.num_ics > 0) &&
                    (dev->smb.num_ics <= NSMBS)) ?
                       dev->smb.num_ics : 0));
    dev->pec_fail_cell_count = 0u;
    dev->voltage_full_updated = false;
    dev->voltage_full_usable = false;
}

static void accumulator_clear_balance_shadow(adbms6830_asic *ic)
{
    if(ic == NULL)
    {
        return;
    }

    /* Balancing policy owns only DCC/PWM.  The ADBMS6830 discharge timer is a
     * separate feature and must never be armed implicitly by a clear/apply
     * helper. */
    ic->tx_cfgb.dcc = 0u;
    memset(ic->PwmA.pwma, 0, sizeof(ic->PwmA.pwma));
    memset(ic->PwmB.pwmb, 0, sizeof(ic->PwmB.pwmb));
}

static void accumulator_set_balance_pwm_cell(adbms6830_asic *ic, uint8_t cell, uint8_t duty)
{
    if((ic == NULL) || (cell >= CELL))
    {
        return;
    }

    if(cell < PWMA)
    {
        ic->PwmA.pwma[cell] = (uint8_t)(duty & 0x0Fu);
    }
    else
    {
        uint8_t pwmb_index = (uint8_t)(cell - PWMA);
        if(pwmb_index < PWMB)
        {
            ic->PwmB.pwmb[pwmb_index] = (uint8_t)(duty & 0x0Fu);
        }
    }
}

bool accumulator_balance_shadow_active(const accumulator_t *dev)
{
    if(dev == NULL)
    {
        return false;
    }

    uint8_t ic_count = accumulator_configured_smb_count(dev);
    const adbms6830_asic *smb_ics = (dev->smb.ics != NULL) ?
                                      dev->smb.ics : dev->smb_ics;

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        if(smb_ics[ic].tx_cfgb.dcc != 0u)
        {
            return true;
        }
        for(uint8_t cell = 0u; cell < PWMA; cell++)
        {
            if(smb_ics[ic].PwmA.pwma[cell] != 0u)
            {
                return true;
            }
        }
        for(uint8_t cell = 0u; cell < PWMB; cell++)
        {
            if(smb_ics[ic].PwmB.pwmb[cell] != 0u)
            {
                return true;
            }
        }
    }

    return false;
}

uint16_t accumulator_balance_shadow_mask(const accumulator_t *dev, uint8_t seg)
{
    if(dev == NULL)
    {
        return 0u;
    }

    uint8_t ic_count = accumulator_configured_smb_count(dev);
    if(seg >= ic_count)
    {
        return 0u;
    }

    const adbms6830_asic *smb_ics = (dev->smb.ics != NULL) ?
                                      dev->smb.ics : dev->smb_ics;
    uint16_t mask = (uint16_t)(smb_ics[seg].tx_cfgb.dcc & 0x7FFFu);
    for(uint8_t cell = 0u; cell < NCELLS; cell++)
    {
        uint8_t duty = (cell < PWMA) ? smb_ics[seg].PwmA.pwma[cell] :
                       smb_ics[seg].PwmB.pwmb[cell - PWMA];
        if(duty != 0u)
        {
            mask |= (uint16_t)(1u << cell);
        }
    }
    return mask;
}

/* Caller owns the ADBMS SPI lock.  This is intentionally best-effort: the
 * original write/readback error remains the reported result, but every failed
 * balance transaction still gets an immediate second attempt to command all
 * discharge paths off. */
static void accumulator_best_effort_clear_balance_locked(adbms6830_driver_t *smb,
                                                          adbms6830_asic *smb_ics,
                                                          uint8_t ic_count)
{
    if((smb == NULL) || (smb_ics == NULL))
    {
        return;
    }

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        accumulator_clear_balance_shadow(&smb_ics[ic]);
    }

    adbms6830_disable_discharge_timer_shadow(smb);
    (void)adbms6830_wrcfgb_checked_reason(
        smb, ADBMS6830_CFGB_WRITE_BALANCE_RECOVERY);
    (void)adbms6830_write_pwm_checked(smb);
}

void accumulator_init(accumulator_t *dev,
				      SPI_HandleTypeDef *hspi,
					  GPIO_TypeDef *cs_port_a,
					  GPIO_TypeDef *cs_port_b,
					  uint16_t cs_pin_a,
					  uint16_t cs_pin_b,
					  TIM_HandleTypeDef* htim)
{
	TIM_HandleTypeDef *ready_timer = NULL;

	if(dev == NULL)
	{
		return;
    }

	dev->total_volt = 0;
	dev->max_temp = 0.0f;
	dev->avg_temp = 0.0f;
	dev->max_volt = 0.0f;
	dev->min_volt = 0.0f;
	dev->valid_voltage_count = 0u;
	dev->valid_temp_count = 0u;
	dev->updated_temp_count = 0u;
	dev->usable_temp_count = 0u;
	dev->stale_temp_count = 0u;
	dev->invalid_temp_count = 0u;
	dev->temp_open_count = 0u;
	dev->temp_short_count = 0u;
	dev->temp_jump_count = 0u;
	dev->temp_rate_rise_count = 0u;
	dev->max_temp_deci_c = 0;
	dev->min_temp_deci_c = 0;
	dev->filtered_max_temp_deci_c = 0;
	dev->filtered_min_temp_deci_c = 0;
	dev->filtered_avg_temp_deci_c = 0;
	dev->temp_max_rate_deci_c_per_s = 0;
	dev->max_temp_seg = 0u;
	dev->max_temp_sensor = 0u;
	dev->min_temp_seg = 0u;
	dev->min_temp_sensor = 0u;
	dev->temp_max_rate_seg = 0u;
	dev->temp_max_rate_sensor = 0u;
	dev->temp_full_updated = false;
	dev->temp_full_usable = false;
	dev->temp_startup_scan_complete = false;
	memset(dev->temp_deci_c, 0, sizeof(dev->temp_deci_c));
	memset(dev->temp_raw_code, 0, sizeof(dev->temp_raw_code));
	memset(dev->temp_filtered_deci_c, 0, sizeof(dev->temp_filtered_deci_c));
	memset(dev->temp_sensor_valid, 0, sizeof(dev->temp_sensor_valid));
	memset(dev->temp_filter_valid_mask, 0, sizeof(dev->temp_filter_valid_mask));
	memset(dev->temp_last_update_ms, 0, sizeof(dev->temp_last_update_ms));
	memset(dev->temp_consecutive_misses, 0, sizeof(dev->temp_consecutive_misses));
	memset(dev->updated_temp_mask, 0, sizeof(dev->updated_temp_mask));
	memset(dev->usable_temp_mask, 0, sizeof(dev->usable_temp_mask));
	memset(dev->stale_temp_mask, 0, sizeof(dev->stale_temp_mask));
	memset(dev->invalid_temp_mask, 0, sizeof(dev->invalid_temp_mask));
	memset(dev->temp_open_mask, 0, sizeof(dev->temp_open_mask));
	memset(dev->temp_short_mask, 0, sizeof(dev->temp_short_mask));
	memset(dev->temp_jump_mask, 0, sizeof(dev->temp_jump_mask));
	memset(dev->temp_rate_rise_mask, 0, sizeof(dev->temp_rate_rise_mask));
	dev->updated_voltage_count = 0u;
	dev->usable_voltage_count = 0u;
	dev->stale_voltage_count = 0u;
	dev->pec_fail_cell_count = 0u;
	dev->voltage_jump_cell_count = 0u;
	dev->voltage_stuck_cell_count = 0u;
	dev->voltage_max_delta_mv = 0u;
	dev->max_voltage_mv = 0u;
	dev->min_voltage_mv = 0u;
	dev->max_voltage_seg = 0u;
	dev->max_voltage_cell = 0u;
	dev->min_voltage_seg = 0u;
	dev->min_voltage_cell = 0u;
	dev->voltage_max_delta_seg = 0u;
	dev->voltage_max_delta_cell = 0u;
	dev->voltage_full_updated = false;
	dev->voltage_full_usable = false;
	dev->voltage_startup_scan_complete = false;
	dev->delay_timer_ready = false;
	dev->delay_timer_status = HAL_ERROR;
	dev->smb_transport_ready = false;
	dev->smb_ready = false;
	dev->smb_init_status = HAL_ERROR;
	dev->apm_ready = false;
	dev->apm_init_status = HAL_ERROR;
	dev->apm_full_ring_awake_token = false;
    dev->last_balance_mute_ok = false;
    dev->last_balance_durable_zero_verified = false;
    dev->last_balance_unmute_ok = false;
    dev->last_balance_inhibit_reason = (uint8_t)ACCUMULATOR_BALANCE_INHIBIT_NONE;
	memset(dev->cell_voltage_mv, 0, sizeof(dev->cell_voltage_mv));
    memset(dev->cell_voltage_avg8_mv, 0, sizeof(dev->cell_voltage_avg8_mv));
    memset(dev->cell_voltage_iir_mv, 0, sizeof(dev->cell_voltage_iir_mv));
    memset(dev->avg8_usable_voltage_mask, 0, sizeof(dev->avg8_usable_voltage_mask));
    memset(dev->iir_usable_voltage_mask, 0, sizeof(dev->iir_usable_voltage_mask));
	memset(dev->cell_voltage_valid, 0, sizeof(dev->cell_voltage_valid));
	memset(dev->cell_voltage_last_update_ms, 0, sizeof(dev->cell_voltage_last_update_ms));
	memset(dev->cell_voltage_consecutive_misses, 0, sizeof(dev->cell_voltage_consecutive_misses));
	memset(dev->cell_voltage_same_count, 0, sizeof(dev->cell_voltage_same_count));
	memset(dev->hil_cell_last_update_ms, 0, sizeof(dev->hil_cell_last_update_ms));
	memset(dev->hil_temp_last_update_ms, 0, sizeof(dev->hil_temp_last_update_ms));
	memset(dev->hil_cell_seen_mask, 0, sizeof(dev->hil_cell_seen_mask));
	memset(dev->hil_temp_seen_mask, 0, sizeof(dev->hil_temp_seen_mask));
	memset(dev->updated_voltage_mask, 0, sizeof(dev->updated_voltage_mask));
	memset(dev->usable_voltage_mask, 0, sizeof(dev->usable_voltage_mask));
	memset(dev->pec_fail_voltage_mask, 0, sizeof(dev->pec_fail_voltage_mask));
	memset(dev->stale_voltage_mask, 0, sizeof(dev->stale_voltage_mask));
	memset(dev->voltage_jump_mask, 0, sizeof(dev->voltage_jump_mask));
	memset(dev->voltage_stuck_mask, 0, sizeof(dev->voltage_stuck_mask));

	if(htim != NULL)
    {
		dev->delay_timer_status = HAL_TIM_Base_Start(htim);
		if(dev->delay_timer_status == HAL_OK)
		{
			dev->delay_timer_ready = true;
			ready_timer = htim;
		}
    }

#if AMS_APM_STANDALONE_EVAL_BENCH
	/* Standalone APM evaluation image: no SMB is connected to the isoSPI
	 * chain.  Keep all SMB readiness false instead of spending startup time on
	 * a device that is deliberately absent. */
	memset(&dev->smb, 0, sizeof(dev->smb));
	memset(dev->smb_ics, 0, sizeof(dev->smb_ics));
	dev->smb_init_status = HAL_ERROR;
	dev->smb_transport_ready = false;
	dev->smb_ready = false;
#else
	/* Temporary low-voltage chain: one SMB reached through the ADBMS6822
	 * evaluation board on String B / CS_B.  No APM is present. */
	memset(dev->smb_ics, 0, sizeof(dev->smb_ics));
	dev->smb_init_status = adBms6830_init(&dev->smb,
	                                      NSMBS,
	                                      (uint8_t)ACCUMULATOR_PHYSICAL_CHAIN_COUNT,
	                                      dev->smb_ics,
	                                      NSMBS,
	                                      hspi,
	                                      cs_port_a,
	                                      cs_port_b,
	                                      cs_pin_a,
	                                      cs_pin_b,
                                              ACCUMULATOR_SMB_STRING,
	                                      ready_timer);
	if((dev->smb_init_status == HAL_OK) &&
	   !adbms6830_set_monitored_cell_count(&dev->smb, NCELLS))
	{
		dev->smb_init_status = HAL_ERROR;
	}
	if(dev->smb_init_status == HAL_OK)
	{
		const adbms6830_diag_health_t *health;
		HAL_StatusTypeDef baseline_status;

		/* A successful write only proves that the MCU completed the SPI
		 * transfer. Read both configuration groups back, clear power-on flags,
		 * then require a fresh clean Status A-E image before declaring the
		 * monitor chain ready for safety use. */
		adbms_spi_lock();
		dev->smb_init_status = adbms6830_verify_config_readback(&dev->smb);
		health = adbms6830_diag_health_get(&dev->smb);
		if((dev->smb_init_status == HAL_OK) &&
		   ((health == NULL) || (health->config_mismatch_mask != 0u)))
		{
			dev->smb_init_status = HAL_ERROR;
		}
		if(dev->smb_init_status == HAL_OK)
		{
			baseline_status = adbms6830_establish_diagnostic_baseline(&dev->smb);
#if AMS_ENABLE_ADBMS_STARTUP_POST
            if(baseline_status == HAL_OK)
            {
                baseline_status = adbms6830_run_startup_post(&dev->smb);
            }
#endif
			health = adbms6830_diag_health_get(&dev->smb);

			/* Keep the legacy safety-ready result strict, but preserve a separate
			 * read-only transport capability when the only blocking class is the
			 * reported C-vs-S comparison.  Identity/configuration integrity and all
			 * non-CS status/reference classes must still be clean. */
			dev->smb_transport_ready = dev->delay_timer_ready &&
#if AMS_ENABLE_ADBMS_STARTUP_POST
                dev->smb.post.passed &&
#endif
				adbms6830_diagnostic_transport_ok(&dev->smb) &&
				adbms6830_non_cs_diagnostics_ok(&dev->smb) &&
				(health != NULL) &&
				(health->config_mismatch_mask == 0u);
			dev->smb_init_status = baseline_status;
		}
		adbms_spi_unlock();
	}
	dev->smb_ready = (dev->smb_init_status == HAL_OK);
	if(dev->smb_ready)
	{
		dev->smb_transport_ready = true;
	}


#endif

	/* APM initialization supports both the final mixed ring and this dedicated
	 * single-device evaluation image.  The standalone path may reset the chain
	 * because no SMB is present; the mixed-ring path must not reset the SMBs
	 * whose configuration was already verified above. */
	memset(&dev->apm, 0, sizeof(dev->apm));
	memset(dev->apm_ics, 0, sizeof(dev->apm_ics));
#if AMS_ENABLE_APM_2950 && !AMS_HIL_REPLACE_ADBMS
#if AMS_APM_STANDALONE_EVAL_BENCH
	dev->apm_init_status = adbms2950_init_mixed_chain(&dev->apm,
	                                                   NAPMS,
	                                                   dev->apm_ics,
	                                                   NAPMS,
	                                                   hspi,
	                                                   cs_port_a,
	                                                   cs_port_b,
	                                                   cs_pin_a,
	                                                   cs_pin_b,
	                                                   ready_timer,
	                                                   STRING_B,
	                                                   true,
	                                                   false);
	if(dev->apm_init_status == HAL_OK)
	{
		/* The ADI evaluation board uses a 50 uohm shunt. */
		dev->apm_init_status = adbms2950_set_calibration_profile(
			&dev->apm, ADBMS2950_CAL_PROFILE_EVAL_BASIC);
	}
#else
	if(dev->smb_ready)
	{
		dev->apm_init_status = adbms2950_init_mixed_chain(&dev->apm,
		                                                   NAPMS,
		                                                   dev->apm_ics,
		                                                   NAPMS,
		                                                   hspi,
		                                                   cs_port_a,
		                                                   cs_port_b,
		                                                   cs_pin_a,
		                                                   cs_pin_b,
		                                                   ready_timer,
		                                                   STRING_B,
		                                                   false,
		                                                   AMS_APM_ENABLE_HV_DIVIDERS != 0);
		/* The mixed commands can affect the shared ADBMS6830 command counters.
		 * Seed each SMB prediction from its next valid packet. */
		adbms6830_resync_command_counter_tracking(&dev->smb);
	}
#endif
#endif

	dev->apm_ready = (dev->apm_init_status == HAL_OK);

    /* Startup is not balance-safe merely because the configuration baseline
     * read clean.  Perform this after mixed-ring binding so the final topology
     * invariant is valid even when an ADBMS2950 is physically present. */
    if(dev->smb_transport_ready && accumulator_final_ring_topology_valid(dev))
    {
        if(accumulator_emergency_balance_inhibit(
               dev, ACCUMULATOR_BALANCE_INHIBIT_CONFIG) != 0)
        {
            dev->smb_ready = false;
            dev->smb_init_status = HAL_ERROR;
        }
    }
}

int accumulator_read_volt(accumulator_t *dev)
{
	bool compatible_adi1_sent = false;
	int status;

	if((dev == NULL) || !dev->smb_transport_ready ||
	   !accumulator_final_ring_topology_valid(dev))
	{
		if(dev != NULL)
		{
			dev->apm_full_ring_awake_token = false;
		}
		return -1;
	}
	dev->apm_full_ring_awake_token = false;

	/* Serialize the complete wake/convert/read/parse sequence.  The ADBMS
	 * driver uses shared scratch buffers, so locking only the final HAL
	 * transfer is not sufficient. */
	adbms_spi_lock();
	status = smb_read_voltage_checked(&dev->smb, &compatible_adi1_sent);
	if(compatible_adi1_sent && dev->apm_ready)
	{
		/* The ADBMS6830 ADCV encoding is a compatible ADI1 command on the
		 * ADBMS2950.  It resets I1CNT/I1PHA even if a later cell-voltage read
		 * fails, so start the new freshness epoch immediately after the command
		 * was accepted rather than conditioning it on the complete scan. */
		adbms2950_note_compatible_adi1(&dev->apm);
	}
	else if((status != 0) && dev->apm_ready)
	{
		/* No compatible ADI1 reached the APM, but the coordinated scan still
		 * failed. Do not carry a prior-cycle advisory sample forward as valid. */
		accumulator_invalidate_apm_sample(dev);
	}
	dev->apm_full_ring_awake_token = (status == 0) && dev->apm_ready;
	adbms_spi_unlock();

	return status;
}

int accumulator_read_apm(accumulator_t *dev, uint32_t now_ms)
{
	HAL_StatusTypeDef status;

#if AMS_APM_STANDALONE_EVAL_BENCH
	/* Dedicated one-device evaluation chain.  No SMB wake token or external
	 * command-counter bookkeeping exists in this topology; serialize the
	 * complete coherent sample against CLI traffic and read the APM directly. */
	if((dev == NULL) || !dev->apm_ready ||
	   !dev->apm.health.initialized ||
	   (dev->apm.hspi == NULL) ||
	   (dev->apm.htim == NULL) ||
	   (dev->apm.string != STRING_B) ||
	   (dev->apm.write_string != STRING_B))
	{
		if(dev != NULL)
		{
			accumulator_invalidate_apm_sample(dev);
		}
		return -1;
	}

	adbms_spi_lock();
	status = adbms2950_read_primary_sample(&dev->apm, now_ms);
	adbms_spi_unlock();
#else
	if((dev == NULL) || !dev->smb_ready || !dev->apm_ready ||
	   !accumulator_final_ring_topology_valid(dev))
	{
		return -1;
	}

	adbms_spi_lock();
	if(!dev->apm_full_ring_awake_token)
	{
		accumulator_invalidate_apm_sample(dev);
		adbms_spi_unlock();
		return -1;
	}
	/* Consume before issuing any command: retries must first re-prove that the
	 * complete physical ring is awake. */
	dev->apm_full_ring_awake_token = false;
	status = adbms2950_read_primary_sample(&dev->apm, now_ms);
	if(status == HAL_OK)
	{
		/* This production path follows a successful SMB scan, so the complete
		 * ring is awake. UNSNAP + SNAP + UNSNAP are compatible broadcast
		 * commands and advance all five SMB command-counter predictions. */
		adbms6830_note_external_counter_increments(
			&dev->smb,
			ADBMS2950_SHARED_COUNTER_INCREMENTS_PER_SAMPLE);
	}
	else
	{
		adbms6830_resync_command_counter_tracking(&dev->smb);
	}
	adbms_spi_unlock();
#endif
	return (status == HAL_OK) ? 0 : -1;
}

int smb_read_voltage(adbms6830_driver_t* dev)
{
	return smb_read_voltage_checked(dev, NULL);
}

static int smb_read_voltage_checked(adbms6830_driver_t *dev,
                                    bool *compatible_adi1_sent)
{
#if AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && AMS_S_PATH_ECO_VALIDATED
    bool continuous_c_was_running = false;
#endif
    if(compatible_adi1_sent != NULL)
    {
        *compatible_adi1_sent = false;
    }

    if(dev == NULL)
    {
        return -1;
    }

    memset(dev->last_cell_updated_mask, 0, sizeof(dev->last_cell_updated_mask));
    memset(dev->last_cell_pec_mask, 0, sizeof(dev->last_cell_pec_mask));

    if((dev->ics == NULL) || (dev->num_ics <= 0) ||
       (dev->num_ics > (int)dev->ics_capacity) ||
       (dev->num_ics > (int)ADBMS6830_MAX_TRACKED_ICS))
    {
        return -1;
    }

#if AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && AMS_S_PATH_ECO_VALIDATED
    continuous_c_was_running = dev->continuous_c_running;
    if(!continuous_c_was_running)
#endif
    {
		if(adbms6830_wakeup_checked(dev) != HAL_OK)
		{
			return -1;
		}

		/* REFON is asserted in the verified production configuration. Together
		 * with the checked wake inside the ADCV wrapper, this interval leaves
		 * margin for a cold/reference transition before the first production
		 * ADCV. The post-ECO continuous-C path pays it only when C has to be
		 * (re)established, not on every 10 Hz readout. */
		if(adbms6830_wait_cooperative(
		       dev, ADBMS6830_REFERENCE_PRECONVERSION_WAIT_US) != HAL_OK)
		{
			return -1;
		}
    }

	// 2. START ADC CONVERSION
	if(adbms6830_start_adc_cell_voltage_measurement(dev) != HAL_OK)
	{
		return -1;
	}
	if(compatible_adi1_sent != NULL)
	{
#if AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && AMS_S_PATH_ECO_VALIDATED
		/* Only the first/recovery ADCV is physically emitted once continuous C
		 * is established. This also stops resetting an already-continuous
		 * ADBMS2950 current-conversion epoch on every cell read. */
		*compatible_adi1_sent = !continuous_c_was_running;
#else
		*compatible_adi1_sent = true;
#endif
	}

#if AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && AMS_S_PATH_ECO_VALIDATED
	if(!continuous_c_was_running)
	{
		/* C provides 1 ms results and AVG8 needs one complete 8 ms window. */
		if(adbms6830_wait_cooperative(dev, 9000u) != HAL_OK)
		{
			return -1;
		}
	}
#else
	/* Current Rev5 uses the redundant C/S command. Depending on C-ADC
	 * synchronization the documented result time is 8 ms to 16 ms. */
	if(adbms6830_wait_cooperative(dev,
	                               ADBMS6830_REDUNDANT_CONVERSION_WAIT_US) != HAL_OK)
	{
		return -1;
	}
#endif

	// 4. SNAP, READ, AND PARSE
	if(adbms6830_read_cell_voltage_products(
           dev,
           (AMS_ENABLE_ADBMS_AVG8_VOLTAGE != 0),
           (AMS_ENABLE_ADBMS_FILTERED_VOLTAGE != 0)) != HAL_OK)
	{
		return -1;
	}
//	adbms6830_wakeup(dev);
	return 0;
}

int accumulator_run_c_open_wire_diagnostic(
    accumulator_t *dev,
    adbms6830_open_wire_result_t *result)
{
    adbms6830_open_wire_result_t local = {0};
    const adbms6830_diag_health_t *health;
    bool compatible_adi1_sent = false;
    int restore_result;

    local.path = ADBMS6830_OPEN_WIRE_PATH_C;
    local.diagnostic_status = HAL_ERROR;
    local.restore_status = HAL_ERROR;

    if((dev == NULL) || !dev->smb_transport_ready ||
       !accumulator_final_ring_topology_valid(dev) ||
       accumulator_balance_shadow_active(dev))
    {
        if(result != NULL)
        {
            *result = local;
        }
        return -1;
    }

    adbms_spi_lock();
    dev->apm_full_ring_awake_token = false;
    local.diagnostic_status = adbms6830_run_open_wire_diagnostic_path(
        &dev->smb,
        ADBMS6830_OPEN_WIRE_PATH_C);

    health = adbms6830_diag_health_get(&dev->smb);
    if(health != NULL)
    {
        local.incomplete_ic_mask = health->open_wire_incomplete_ic_mask;
        local.fault_ic_mask = health->open_wire_fault_ic_mask;
        for(uint8_t ic = 0u; ic < (uint8_t)dev->smb.num_ics; ic++)
        {
            local.cell_fault_mask[ic] = health->open_wire_cell_fault_mask[ic];
        }
    }
    local.complete = (local.incomplete_ic_mask == 0u);

    /* The diagnostic C conversions are never allowed to become the published
     * voltage image. Restore a normal checked conversion even after a detected
     * open or a failed diagnostic transaction. */
    restore_result = smb_read_voltage_checked(&dev->smb,
                                               &compatible_adi1_sent);
    local.restore_status = (restore_result == 0) ? HAL_OK : HAL_ERROR;
    local.restored_normal_c_image = (restore_result == 0);
    dev->smb.health.open_wire_last_restore_status = local.restore_status;
    if(dev->smb.health.open_wire_restore_count != UINT32_MAX)
    {
        dev->smb.health.open_wire_restore_count++;
    }
    if(restore_result != 0)
    {
        if(dev->smb.health.open_wire_restore_fail_count != UINT32_MAX)
        {
            dev->smb.health.open_wire_restore_fail_count++;
        }
        accumulator_invalidate_cell_voltage_authority(dev);
    }

    if(compatible_adi1_sent && dev->apm_ready)
    {
        adbms2950_note_compatible_adi1(&dev->apm);
    }
    else if((restore_result != 0) && dev->apm_ready)
    {
        accumulator_invalidate_apm_sample(dev);
    }
    dev->apm_full_ring_awake_token = (restore_result == 0) && dev->apm_ready;
    adbms_spi_unlock();

    accumulator_update_voltage_stats_at(dev, HAL_GetTick());
    if(result != NULL)
    {
        *result = local;
    }

    return ((local.diagnostic_status == HAL_OK) &&
            local.complete &&
            (local.fault_ic_mask == 0u) &&
            local.restored_normal_c_image) ? 0 : -1;
}

#define ACCUMULATOR_TEMP_MUX_SETTLE_US 3000u
#define ACCUMULATOR_TEMP_AUX_GUARD_US  1000u
#if AMS_TEMP_PULLUPS_TARGET_VALIDATED
#define ACCUMULATOR_TEMP_TRANSACTION_ATTEMPTS 2u
#else
/* Rev5 100-ohm pull-ups are not electrically validated. Explicit bench
 * commands make one bounded attempt only; automatic scanning is build-gated. */
#define ACCUMULATOR_TEMP_TRANSACTION_ATTEMPTS 1u
#endif

static int accumulator_temp_select_checked(adbms6830_driver_t *dev,
                                           uint8_t sensor)
{
    for(uint8_t attempt = 0u;
        attempt < ACCUMULATOR_TEMP_TRANSACTION_ATTEMPTS;
        attempt++)
    {
        if(mux_set_channel(dev, sensor) == 0)
        {
            return (adbms6830_wait_cooperative(dev,
                                       ACCUMULATOR_TEMP_MUX_SETTLE_US) == HAL_OK)
                       ? 0 : -1;
        }
        if((attempt + 1u) < ACCUMULATOR_TEMP_TRANSACTION_ATTEMPTS)
        {
            if((adbms6830_wakeup_checked(dev) != HAL_OK) ||
               (adbms6830_wait_cooperative(dev,
                                   ACCUMULATOR_TEMP_MUX_SETTLE_US) != HAL_OK))
            {
                return -1;
            }
        }
    }
    return -1;
}

static int accumulator_temp_capture_checked(adbms6830_driver_t *dev,
                                            uint8_t sensor)
{
    for(uint8_t attempt = 0u;
        attempt < ACCUMULATOR_TEMP_TRANSACTION_ATTEMPTS;
        attempt++)
    {
        if(mux_read_gpio_voltage(dev, sensor) == 0)
        {
            return (adbms6830_wait_cooperative(dev,
                                       ACCUMULATOR_TEMP_AUX_GUARD_US) == HAL_OK)
                       ? 0 : -1;
        }
        if((attempt + 1u) < ACCUMULATOR_TEMP_TRANSACTION_ATTEMPTS)
        {
            if((adbms6830_wakeup_checked(dev) != HAL_OK) ||
               (adbms6830_wait_cooperative(dev,
                                   ACCUMULATOR_TEMP_MUX_SETTLE_US) != HAL_OK))
            {
                return -1;
            }
        }
    }
    return -1;
}

int smb_read_temp(adbms6830_driver_t* dev)
{
    uint8_t selected[ADBMS6830_MUX_COUNT];

    if(dev == NULL)
    {
        return -1;
    }

    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        dev->last_temp_updated_mask[ic] = 0u;
    }

    if((dev->ics == NULL) || (dev->num_ics <= 0) ||
       (dev->num_ics > (int)dev->ics_capacity) ||
       (dev->num_ics > (int)ADBMS6830_MAX_TRACKED_ICS))
    {
        return -1;
    }

    if(adbms6830_wakeup_checked(dev) != HAL_OK)
    {
        return -1;
    }

    /* All three muxes select the same local channel first, settle, and are
     * then captured one at a time. This avoids publishing a sample whose mux
     * address/ACK did not belong to that exact sensor. */
    sensor_num = (sensor_num % (NTEMPS / ADBMS6830_MUX_COUNT)) + 1u;
    selected[0] = (uint8_t)(sensor_num - 1u);
    selected[1] = (uint8_t)(sensor_num + 7u);
    selected[2] = (uint8_t)(sensor_num + 15u);

    for(uint8_t mux = 0u; mux < ADBMS6830_MUX_COUNT; mux++)
    {
        if(accumulator_temp_select_checked(dev, selected[mux]) != 0)
        {
            return -1;
        }
    }

    for(uint8_t mux = 0u; mux < ADBMS6830_MUX_COUNT; mux++)
    {
        if(accumulator_temp_capture_checked(dev, selected[mux]) != 0)
        {
            return -1;
        }
    }

    return 0;
}

int accumulator_read_temp(accumulator_t *dev)
{
	if((dev == NULL) || !dev->smb_transport_ready ||
	   !accumulator_final_ring_topology_valid(dev))
	{
		return -1;
	}

	adbms_spi_lock();
	int status = smb_read_temp(&dev->smb);
	adbms_spi_unlock();

	return status;
}

int accumulator_stat_temp(accumulator_t *dev)
{
    if(dev == NULL)
    {
        return -1;
    }

    accumulator_update_temp_stats(dev);
    return 0;
}

int accumulator_set_temp_ch(accumulator_t *dev, uint8_t channel)
{
    if(dev == NULL)
    {
        return -1;
    }

    return accumulator_set_mux_ch(dev, channel, 0u);
}

int accumulator_set_mux_ch(accumulator_t *dev, uint8_t channel, uint8_t addr7)
{
	int status;

	(void)addr7;

	if((dev == NULL) || !dev->smb_transport_ready ||
	   !accumulator_final_ring_topology_valid(dev) ||
	   (channel >= NTEMPS))
	{
		return -1;
	}

	adbms_spi_lock();
	status = mux_set_channel(&dev->smb, channel);
	adbms_spi_unlock();
	return status;
}

float convert_adc_to_volt(int value)
{
	/* Convert before adding the signed offset so even an adversarial full-range
	 * int input cannot overflow in integer arithmetic. */
	return ((float)value + 10000.0f) * 0.000150f;
}

static uint16_t accumulator_code_to_mv(int16_t code)
{
    /* ADBMS6830 cell codes use V = (code + 10000) * 150 uV.
     * 0x0000 is therefore a valid 1.500 V measurement and must reach the
     * severe-undervoltage policy.  Only 0x8000 is the reset/clear sentinel. */
    if(code == INT16_MIN)
    {
        return 0u;
    }

    float volts = convert_adc_to_volt(code);

    if(!isfinite(volts) || (volts < 0.0f))
    {
        return 0u;
    }

    if(volts >= 65.535f)
    {
        return UINT16_MAX;
    }

    return (uint16_t)((volts * 1000.0f) + 0.5f);
}

static int16_t accumulator_mv_to_code(uint16_t mv)
{
    float volts = (float)mv / 1000.0f;
    float code = (volts / 0.000150f) - 10000.0f;

    if(!isfinite(code))
    {
        return 0;
    }
    if(code >= (float)INT16_MAX)
    {
        return INT16_MAX;
    }
    if(code <= (float)INT16_MIN)
    {
        return INT16_MIN;
    }

    return (int16_t)lroundf(code);
}

static int16_t accumulator_temp_deci_c_to_raw(int16_t deci_c)
{
    int16_t raw = 0;
    float temperature_c = (float)deci_c / 10.0f;

    if(!thermistor_adbms_raw_from_temperature_c(temperature_c,
                                                 THERMISTOR_NOMINAL_VREG_V,
                                                 &raw))
    {
        return THERMISTOR_ADBMS_RESET_CODE;
    }

    return raw;
}

int accumulator_hil_ingest_cell_triplet(accumulator_t *dev,
                                        uint8_t seg,
                                        uint8_t first_cell,
                                        const uint16_t cell_mv[3],
                                        uint32_t now_ms)
{
    if((dev == NULL) || (cell_mv == NULL) || (seg >= NSMBS) || (first_cell >= NCELLS))
    {
        return -1;
    }

    adbms6830_asic *smb_ics = (dev->smb.ics != NULL) ? dev->smb.ics : dev->smb_ics;

    for(uint8_t n = 0u; n < 3u; n++)
    {
        uint8_t cell = (uint8_t)(first_cell + n);
        if(cell >= NCELLS)
        {
            break;
        }

        uint16_t bit = (uint16_t)(1u << cell);
        smb_ics[seg].cell.c_codes[cell] = accumulator_mv_to_code(cell_mv[n]);
        dev->hil_cell_last_update_ms[seg][cell] = now_ms;
        dev->hil_cell_seen_mask[seg] |= bit;
        dev->smb.last_cell_updated_mask[seg] |= bit;
        dev->smb.last_cell_pec_mask[seg] &= (uint16_t)~bit;
    }

    return 0;
}

int accumulator_hil_ingest_temp_triplet(accumulator_t *dev,
                                        uint8_t seg,
                                        uint8_t first_sensor,
                                        const int16_t temp_deci_c[3],
                                        uint32_t now_ms)
{
    if((dev == NULL) || (temp_deci_c == NULL) || (seg >= NSMBS) || (first_sensor >= NTEMPS))
    {
        return -1;
    }

    adbms6830_asic *smb_ics = (dev->smb.ics != NULL) ? dev->smb.ics : dev->smb_ics;

    for(uint8_t n = 0u; n < 3u; n++)
    {
        uint8_t sensor = (uint8_t)(first_sensor + n);
        if(sensor >= NTEMPS)
        {
            break;
        }

        uint32_t bit = (uint32_t)(1UL << sensor);
        smb_ics[seg].temp.raw[sensor] = accumulator_temp_deci_c_to_raw(temp_deci_c[n]);
        dev->hil_temp_last_update_ms[seg][sensor] = now_ms;
        dev->hil_temp_seen_mask[seg] |= bit;
        dev->smb.last_temp_updated_mask[seg] |= bit;
    }

    return 0;
}

void accumulator_hil_refresh_update_masks(accumulator_t *dev,
                                          uint32_t now_ms,
                                          uint32_t timeout_ms)
{
    if(dev == NULL)
    {
        return;
    }

    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        uint16_t cell_mask = 0u;
        uint32_t temp_mask = 0u;

        for(uint8_t cell = 0u; cell < NCELLS; cell++)
        {
            uint16_t bit = (uint16_t)(1u << cell);
            if((dev->hil_cell_seen_mask[seg] & bit) != 0u)
            {
                uint32_t age_ms = (uint32_t)(now_ms - dev->hil_cell_last_update_ms[seg][cell]);
                if(age_ms <= timeout_ms)
                {
                    cell_mask |= bit;
                }
            }
        }

        for(uint8_t sensor = 0u; sensor < NTEMPS; sensor++)
        {
            uint32_t bit = (uint32_t)(1UL << sensor);
            if((dev->hil_temp_seen_mask[seg] & bit) != 0u)
            {
                uint32_t age_ms = (uint32_t)(now_ms - dev->hil_temp_last_update_ms[seg][sensor]);
                if(age_ms <= timeout_ms)
                {
                    temp_mask |= bit;
                }
            }
        }

        dev->smb.last_cell_updated_mask[seg] = cell_mask;
        dev->smb.last_cell_pec_mask[seg] = 0u;
        dev->smb.last_temp_updated_mask[seg] = temp_mask;
    }
}

static uint16_t accumulator_expected_cell_count(const accumulator_t *dev)
{
    (void)dev;

    /* Safety policy is tied to the real accumulator topology: 5 SMBs x 15 cells.
     * Do not reduce the required cell count if smb.num_ics is corrupted or
     * accidentally configured low; that could otherwise allow BMS_OK with only
     * a partial pack represented in firmware.
     */
    return (uint16_t)(NSMBS * NCELLS);
}

static uint16_t accumulator_count_bits(uint16_t mask)
{
    uint16_t count = 0u;

    while(mask != 0u)
    {
        count += (uint16_t)(mask & 1u);
        mask >>= 1u;
    }

    return count;
}

static uint16_t accumulator_count_bits32(uint32_t mask)
{
    uint16_t count = 0u;

    while(mask != 0u)
    {
        count += (uint16_t)(mask & 1u);
        mask >>= 1u;
    }

    return count;
}

bool accumulator_cell_voltage_usable(const accumulator_t *dev, uint8_t seg, uint8_t cell)
{
    if((dev == NULL) || (seg >= NSMBS) || (cell >= NCELLS))
    {
        return false;
    }

    return ((dev->usable_voltage_mask[seg] & (uint16_t)(1u << cell)) != 0u);
}

uint16_t accumulator_cell_voltage_mv(const accumulator_t *dev, uint8_t seg, uint8_t cell)
{
    if((dev == NULL) || (seg >= NSMBS) || (cell >= NCELLS))
    {
        return 0u;
    }

    return accumulator_cell_voltage_usable(dev, seg, cell) ? dev->cell_voltage_mv[seg][cell] : 0u;
}

void accumulator_update_voltage_stats(accumulator_t *dev)
{
    accumulator_update_voltage_stats_at(dev, 0u);
}

void accumulator_update_voltage_stats_at(accumulator_t *dev, uint32_t now_ms)
{
    if(dev == NULL)
    {
        return;
    }

    uint32_t now = now_ms;
    uint16_t expected_count = accumulator_expected_cell_count(dev);
    uint16_t updated_count = 0u;
    uint16_t usable_count = 0u;
    uint16_t stale_count = 0u;
    uint16_t pec_fail_count = 0u;
    uint16_t jump_count = 0u;
    uint16_t stuck_count = 0u;
    uint16_t max_delta_mv = 0u;
    uint16_t max_mv = 0u;
    uint16_t min_mv = UINT16_MAX;
    float total = 0.0f;

    uint8_t max_seg = 0u;
    uint8_t max_cell = 0u;
    uint8_t min_seg = 0u;
    uint8_t min_cell = 0u;
    uint8_t max_delta_seg = 0u;
    uint8_t max_delta_cell = 0u;

    uint8_t ic_count = accumulator_configured_smb_count(dev);
    adbms6830_asic *smb_ics = (dev->smb.ics != NULL) ? dev->smb.ics : dev->smb_ics;

    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        dev->updated_voltage_mask[ic] = 0u;
        dev->usable_voltage_mask[ic] = 0u;
        dev->pec_fail_voltage_mask[ic] = 0u;
        dev->stale_voltage_mask[ic] = 0u;
        dev->voltage_jump_mask[ic] = 0u;
        dev->voltage_stuck_mask[ic] = 0u;
        dev->avg8_usable_voltage_mask[ic] = 0u;
        dev->iir_usable_voltage_mask[ic] = 0u;
    }

    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        uint16_t read_updated_mask = 0u;
        uint16_t read_pec_mask = 0u;

        if(ic < ADBMS6830_MAX_TRACKED_ICS)
        {
            read_updated_mask = dev->smb.last_cell_updated_mask[ic];
            read_pec_mask = dev->smb.last_cell_pec_mask[ic];
        }

        dev->pec_fail_voltage_mask[ic] = (uint16_t)(read_pec_mask & ((1u << NCELLS) - 1u));
        pec_fail_count += accumulator_count_bits(dev->pec_fail_voltage_mask[ic]);

        for(uint8_t cell = 0u; cell < NCELLS; cell++)
        {
            uint16_t bit = (uint16_t)(1u << cell);
            bool updated_this_scan = ((read_updated_mask & bit) != 0u);
            bool usable = false;

            if(updated_this_scan)
            {
                int16_t code = smb_ics[ic].cell.c_codes[cell];
                uint16_t mv = accumulator_code_to_mv(code);

                if((mv >= ACCUMULATOR_CELL_VALID_MIN_MV) &&
                   (mv <= ACCUMULATOR_CELL_VALID_MAX_MV))
                {
                    bool had_previous = dev->cell_voltage_valid[ic][cell];
                    uint16_t previous_mv = dev->cell_voltage_mv[ic][cell];

                    if(had_previous)
                    {
                        uint16_t delta_mv = (mv >= previous_mv) ?
                                            (uint16_t)(mv - previous_mv) :
                                            (uint16_t)(previous_mv - mv);
                        if(delta_mv > max_delta_mv)
                        {
                            max_delta_mv = delta_mv;
                            max_delta_seg = ic;
                            max_delta_cell = cell;
                        }
                        if(delta_mv >= ACCUMULATOR_CELL_IMPLAUSIBLE_JUMP_MV)
                        {
                            dev->voltage_jump_mask[ic] |= bit;
                            jump_count++;
                        }

                        if(mv == previous_mv)
                        {
                            if(dev->cell_voltage_same_count[ic][cell] < UINT8_MAX)
                            {
                                dev->cell_voltage_same_count[ic][cell]++;
                            }
                            if(dev->cell_voltage_same_count[ic][cell] >= ACCUMULATOR_CELL_STUCK_SAME_COUNT)
                            {
                                dev->voltage_stuck_mask[ic] |= bit;
                                stuck_count++;
                            }
                        }
                        else
                        {
                            dev->cell_voltage_same_count[ic][cell] = 0u;
                        }
                    }
                    else
                    {
                        dev->cell_voltage_same_count[ic][cell] = 0u;
                    }

                    dev->cell_voltage_mv[ic][cell] = mv;
                    dev->cell_voltage_valid[ic][cell] = true;
                    dev->cell_voltage_last_update_ms[ic][cell] = now;
                    dev->cell_voltage_consecutive_misses[ic][cell] = 0u;
                    dev->updated_voltage_mask[ic] |= bit;
                    updated_count++;
                }
                else
                {
                    if(dev->cell_voltage_consecutive_misses[ic][cell] < UINT8_MAX)
                    {
                        dev->cell_voltage_consecutive_misses[ic][cell]++;
                    }
                }
            }
            else
            {
                if(dev->cell_voltage_consecutive_misses[ic][cell] < UINT8_MAX)
                {
                    dev->cell_voltage_consecutive_misses[ic][cell]++;
                }
            }

            if(dev->cell_voltage_valid[ic][cell])
            {
                uint32_t age_ms = (uint32_t)(now - dev->cell_voltage_last_update_ms[ic][cell]);

                usable = (age_ms <= ACCUMULATOR_CELL_STALE_TIMEOUT_MS) &&
                         (dev->cell_voltage_consecutive_misses[ic][cell] <= ACCUMULATOR_CELL_MAX_CONSEC_MISSES);

                if(usable)
                {
                    uint16_t mv = dev->cell_voltage_mv[ic][cell];
                    dev->usable_voltage_mask[ic] |= bit;
                    usable_count++;
                    total += ((float)mv / 1000.0f);

                    if(mv > max_mv)
                    {
                        max_mv = mv;
                        max_seg = ic;
                        max_cell = cell;
                    }
                    if(mv < min_mv)
                    {
                        min_mv = mv;
                        min_seg = ic;
                        min_cell = cell;
                    }
                }
            }

            if(!usable)
            {
                dev->stale_voltage_mask[ic] |= bit;
                stale_count++;
            }
        }
    }

    /* Normalize the two non-authoritative voltage products separately.  They
     * never change raw safety usability: AVG8 and IIR are estimator/diagnostic
     * products and may disappear independently without masking a raw fault. */
    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        uint16_t avg_updated = dev->smb.last_acell_updated_mask[ic];
        uint16_t avg_pec = dev->smb.last_acell_pec_mask[ic];
        uint16_t iir_updated = dev->smb.last_fcell_updated_mask[ic];
        uint16_t iir_pec = dev->smb.last_fcell_pec_mask[ic];

        for(uint8_t cell = 0u; cell < NCELLS; cell++)
        {
            uint16_t bit = (uint16_t)(1u << cell);
            if(((avg_updated & bit) != 0u) && ((avg_pec & bit) == 0u))
            {
                uint16_t mv = accumulator_code_to_mv(smb_ics[ic].acell.ac_codes[cell]);
                if((mv >= ACCUMULATOR_CELL_VALID_MIN_MV) &&
                   (mv <= ACCUMULATOR_CELL_VALID_MAX_MV))
                {
                    dev->cell_voltage_avg8_mv[ic][cell] = mv;
                    dev->avg8_usable_voltage_mask[ic] |= bit;
                }
            }
            if(dev->smb.filtered_voltage_ready &&
               ((iir_updated & bit) != 0u) && ((iir_pec & bit) == 0u))
            {
                uint16_t mv = accumulator_code_to_mv(smb_ics[ic].fcell.fc_codes[cell]);
                if((mv >= ACCUMULATOR_CELL_VALID_MIN_MV) &&
                   (mv <= ACCUMULATOR_CELL_VALID_MAX_MV))
                {
                    dev->cell_voltage_iir_mv[ic][cell] = mv;
                    dev->iir_usable_voltage_mask[ic] |= bit;
                }
            }
        }
    }

    dev->updated_voltage_count = updated_count;
    dev->usable_voltage_count = usable_count;
    dev->stale_voltage_count = stale_count;
    dev->pec_fail_cell_count = pec_fail_count;
    dev->voltage_jump_cell_count = jump_count;
    dev->voltage_stuck_cell_count = stuck_count;
    dev->voltage_max_delta_mv = max_delta_mv;
    dev->voltage_max_delta_seg = max_delta_seg;
    dev->voltage_max_delta_cell = max_delta_cell;
    dev->valid_voltage_count = usable_count;
    dev->voltage_full_updated = (expected_count > 0u) && (updated_count == expected_count);
    dev->voltage_full_usable = (expected_count > 0u) && (usable_count == expected_count);

    if(dev->voltage_full_updated)
    {
        dev->voltage_startup_scan_complete = true;
    }

    if(usable_count > 0u)
    {
        dev->max_voltage_mv = max_mv;
        dev->min_voltage_mv = min_mv;
        dev->max_voltage_seg = max_seg;
        dev->max_voltage_cell = max_cell;
        dev->min_voltage_seg = min_seg;
        dev->min_voltage_cell = min_cell;
        dev->max_volt = (float)max_mv / 1000.0f;
        dev->min_volt = (float)min_mv / 1000.0f;
        dev->total_volt = total;
    }
    else
    {
        dev->max_voltage_mv = 0u;
        dev->min_voltage_mv = 0u;
        dev->max_voltage_seg = 0u;
        dev->max_voltage_cell = 0u;
        dev->min_voltage_seg = 0u;
        dev->min_voltage_cell = 0u;
        dev->max_volt = 0.0f;
        dev->min_volt = 0.0f;
        dev->total_volt = 0.0f;
    }
}


void accumulator_update_temp_stats(accumulator_t *dev)
{
    accumulator_update_temp_stats_at(dev, 0u);
}

static bool accumulator_temp_code_to_deci_c(int16_t raw,
                                                int16_t *deci_c,
                                                thermistor_status_t *status_out)
{
    thermistor_result_t result = thermistor_from_adbms_raw(
        raw, THERMISTOR_NOMINAL_VREG_V);

    if(status_out != NULL)
    {
        *status_out = result.status;
    }

    if(!result.valid ||
       !isfinite(result.temperature_c) ||
       (result.temperature_c < ((float)ACCUMULATOR_TEMP_VALID_MIN_DECI_C / 10.0f)) ||
       (result.temperature_c > ((float)ACCUMULATOR_TEMP_VALID_MAX_DECI_C / 10.0f)))
    {
        return false;
    }

    if(deci_c != NULL)
    {
        *deci_c = (int16_t)lroundf(result.temperature_c * 10.0f);
    }
    return true;
}

static int16_t accumulator_iir_deci_c(int16_t old_deci_c, int16_t new_deci_c)
{
    int32_t num = ((int32_t)old_deci_c *
                   (int32_t)(ACCUMULATOR_TEMP_FILTER_ALPHA_DEN - ACCUMULATOR_TEMP_FILTER_ALPHA_NUM)) +
                  ((int32_t)new_deci_c * (int32_t)ACCUMULATOR_TEMP_FILTER_ALPHA_NUM);

    if(num >= 0)
    {
        num += (ACCUMULATOR_TEMP_FILTER_ALPHA_DEN / 2);
    }
    else
    {
        num -= (ACCUMULATOR_TEMP_FILTER_ALPHA_DEN / 2);
    }

    return (int16_t)(num / ACCUMULATOR_TEMP_FILTER_ALPHA_DEN);
}

bool accumulator_temp_sensor_usable(const accumulator_t *dev, uint8_t seg, uint8_t sensor)
{
    if((dev == NULL) || (seg >= NSMBS) || (sensor >= NTEMPS))
    {
        return false;
    }

    return ((dev->usable_temp_mask[seg] & (uint32_t)(1UL << sensor)) != 0u);
}

int16_t accumulator_temp_deci_c(const accumulator_t *dev, uint8_t seg, uint8_t sensor)
{
    if((dev == NULL) || (seg >= NSMBS) || (sensor >= NTEMPS))
    {
        return 0;
    }

    return accumulator_temp_sensor_usable(dev, seg, sensor) ? dev->temp_deci_c[seg][sensor] : 0;
}

void accumulator_update_temp_stats_at(accumulator_t *dev, uint32_t now_ms)
{
    if(dev == NULL)
    {
        return;
    }

    uint32_t now = now_ms;
    uint16_t expected_count = (uint16_t)(NSMBS * NTEMPS);
    uint16_t updated_count = 0u;
    uint16_t usable_count = 0u;
    uint16_t stale_count = 0u;
    uint16_t invalid_count = 0u;
    uint16_t open_count = 0u;
    uint16_t short_count = 0u;
    uint16_t jump_count = 0u;
    uint16_t rate_rise_count = 0u;
    int16_t max_deci_c = INT16_MIN;
    int16_t min_deci_c = INT16_MAX;
    int16_t filtered_max_deci_c = INT16_MIN;
    int16_t filtered_min_deci_c = INT16_MAX;
    int16_t max_rate_deci_c_per_s = 0;
    int32_t sum_deci_c = 0;
    int32_t filtered_sum_deci_c = 0;
    uint16_t filtered_count = 0u;

    uint8_t ic_count = accumulator_configured_smb_count(dev);
    adbms6830_asic *smb_ics = (dev->smb.ics != NULL) ? dev->smb.ics : dev->smb_ics;

    uint8_t max_seg = 0u;
    uint8_t max_sensor = 0u;
    uint8_t min_seg = 0u;
    uint8_t min_sensor = 0u;
    uint8_t max_rate_seg = 0u;
    uint8_t max_rate_sensor = 0u;

    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        dev->updated_temp_mask[ic] = 0u;
        dev->usable_temp_mask[ic] = 0u;
        dev->stale_temp_mask[ic] = 0u;
        dev->invalid_temp_mask[ic] = 0u;
        dev->temp_open_mask[ic] = 0u;
        dev->temp_short_mask[ic] = 0u;
        dev->temp_jump_mask[ic] = 0u;
        dev->temp_rate_rise_mask[ic] = 0u;
    }

    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        uint32_t read_updated_mask = 0u;
        if((ic < ic_count) && (ic < ADBMS6830_MAX_TRACKED_ICS))
        {
            read_updated_mask = dev->smb.last_temp_updated_mask[ic] & ((1UL << NTEMPS) - 1UL);
        }

        for(uint8_t sensor = 0u; sensor < NTEMPS; sensor++)
        {
            uint32_t bit = (uint32_t)(1UL << sensor);
            bool updated_this_scan = ((read_updated_mask & bit) != 0u);
            bool usable = false;

            if(updated_this_scan && (ic < ic_count))
            {
                int16_t deci_c = 0;
                int16_t raw = smb_ics[ic].temp.raw[sensor];
                thermistor_status_t thermistor_status = THERMISTOR_STATUS_NUMERIC_FAULT;
                bool had_previous = dev->temp_sensor_valid[ic][sensor];
                int16_t previous_deci_c = dev->temp_deci_c[ic][sensor];
                uint32_t previous_tick = dev->temp_last_update_ms[ic][sensor];

                dev->temp_raw_code[ic][sensor] = raw;

                if(accumulator_temp_code_to_deci_c(raw, &deci_c, &thermistor_status))
                {
                    if(had_previous)
                    {
                        uint16_t delta_deci_c = (deci_c >= previous_deci_c) ?
                                                (uint16_t)(deci_c - previous_deci_c) :
                                                (uint16_t)(previous_deci_c - deci_c);
                        uint32_t elapsed_ms = (uint32_t)(now - previous_tick);

                        if(delta_deci_c >= ACCUMULATOR_TEMP_IMPLAUSIBLE_JUMP_DECI_C)
                        {
                            dev->temp_jump_mask[ic] |= bit;
                            jump_count++;
                        }

                        if(elapsed_ms > 0u)
                        {
                            uint32_t rate = ((uint32_t)delta_deci_c * 1000u) / elapsed_ms;
                            if(rate > (uint32_t)INT16_MAX)
                            {
                                rate = (uint32_t)INT16_MAX;
                            }
                            if((int16_t)rate > max_rate_deci_c_per_s)
                            {
                                max_rate_deci_c_per_s = (int16_t)rate;
                                max_rate_seg = ic;
                                max_rate_sensor = sensor;
                            }
                            if(rate >= ACCUMULATOR_TEMP_RATE_WARN_DECI_C_PER_S)
                            {
                                dev->temp_rate_rise_mask[ic] |= bit;
                                rate_rise_count++;
                            }
                        }
                    }

                    if((dev->temp_filter_valid_mask[ic] & bit) == 0u)
                    {
                        dev->temp_filtered_deci_c[ic][sensor] = deci_c;
                        dev->temp_filter_valid_mask[ic] |= bit;
                    }
                    else
                    {
                        dev->temp_filtered_deci_c[ic][sensor] =
                            accumulator_iir_deci_c(dev->temp_filtered_deci_c[ic][sensor], deci_c);
                    }

                    dev->temp_deci_c[ic][sensor] = deci_c;
                    dev->temp_sensor_valid[ic][sensor] = true;
                    dev->temp_last_update_ms[ic][sensor] = now;
                    dev->temp_consecutive_misses[ic][sensor] = 0u;
                    dev->updated_temp_mask[ic] |= bit;
                    updated_count++;
                }
                else
                {
                    dev->temp_sensor_valid[ic][sensor] = false;
                    dev->temp_filter_valid_mask[ic] &= ~bit;
                    dev->invalid_temp_mask[ic] |= bit;
                    invalid_count++;
                    if(thermistor_status == THERMISTOR_STATUS_OPEN_CIRCUIT)
                    {
                        dev->temp_open_mask[ic] |= bit;
                        open_count++;
                    }
                    else if(thermistor_status == THERMISTOR_STATUS_SHORT_CIRCUIT)
                    {
                        dev->temp_short_mask[ic] |= bit;
                        short_count++;
                    }
                    if(dev->temp_consecutive_misses[ic][sensor] < UINT8_MAX)
                    {
                        dev->temp_consecutive_misses[ic][sensor]++;
                    }
                }
            }
            else
            {
                if(dev->temp_consecutive_misses[ic][sensor] < UINT8_MAX)
                {
                    dev->temp_consecutive_misses[ic][sensor]++;
                }
            }

            if(dev->temp_sensor_valid[ic][sensor])
            {
                uint32_t age_ms = (uint32_t)(now - dev->temp_last_update_ms[ic][sensor]);

                usable = (age_ms <= ACCUMULATOR_TEMP_STALE_TIMEOUT_MS) &&
                         (dev->temp_consecutive_misses[ic][sensor] <= ACCUMULATOR_TEMP_MAX_CONSEC_MISSES);

                if(usable)
                {
                    int16_t deci_c = dev->temp_deci_c[ic][sensor];
                    dev->usable_temp_mask[ic] |= bit;
                    usable_count++;
                    sum_deci_c += deci_c;

                    if(deci_c > max_deci_c)
                    {
                        max_deci_c = deci_c;
                        max_seg = ic;
                        max_sensor = sensor;
                    }
                    if(deci_c < min_deci_c)
                    {
                        min_deci_c = deci_c;
                        min_seg = ic;
                        min_sensor = sensor;
                    }

                    if((dev->temp_filter_valid_mask[ic] & bit) != 0u)
                    {
                        int16_t filt = dev->temp_filtered_deci_c[ic][sensor];
                        filtered_sum_deci_c += filt;
                        filtered_count++;
                        if(filt > filtered_max_deci_c)
                        {
                            filtered_max_deci_c = filt;
                        }
                        if(filt < filtered_min_deci_c)
                        {
                            filtered_min_deci_c = filt;
                        }
                    }
                }
            }

            if(!usable)
            {
                dev->stale_temp_mask[ic] |= bit;
                stale_count++;
            }
        }
    }

    dev->updated_temp_count = updated_count;
    dev->usable_temp_count = usable_count;
    dev->stale_temp_count = stale_count;
    dev->invalid_temp_count = invalid_count;
    dev->temp_open_count = open_count;
    dev->temp_short_count = short_count;
    dev->temp_jump_count = jump_count;
    dev->temp_rate_rise_count = rate_rise_count;
    dev->temp_max_rate_deci_c_per_s = max_rate_deci_c_per_s;
    dev->temp_max_rate_seg = max_rate_seg;
    dev->temp_max_rate_sensor = max_rate_sensor;
    dev->valid_temp_count = usable_count;
    dev->temp_full_updated = (expected_count > 0u) && (updated_count == expected_count);
    dev->temp_full_usable = (expected_count > 0u) && (usable_count == expected_count);

    if(dev->temp_full_usable)
    {
        dev->temp_startup_scan_complete = true;
    }

    if(usable_count > 0u)
    {
        dev->max_temp_deci_c = max_deci_c;
        dev->min_temp_deci_c = min_deci_c;
        dev->max_temp_seg = max_seg;
        dev->max_temp_sensor = max_sensor;
        dev->min_temp_seg = min_seg;
        dev->min_temp_sensor = min_sensor;
        dev->max_temp = (float)max_deci_c / 10.0f;
        dev->avg_temp = ((float)sum_deci_c / 10.0f) / (float)usable_count;
    }
    else
    {
        dev->max_temp_deci_c = 0;
        dev->min_temp_deci_c = 0;
        dev->max_temp_seg = 0u;
        dev->max_temp_sensor = 0u;
        dev->min_temp_seg = 0u;
        dev->min_temp_sensor = 0u;
        dev->max_temp = 0.0f;
        dev->avg_temp = 0.0f;
    }

    if(filtered_count > 0u)
    {
        dev->filtered_max_temp_deci_c = filtered_max_deci_c;
        dev->filtered_min_temp_deci_c = filtered_min_deci_c;
        dev->filtered_avg_temp_deci_c = (int16_t)(filtered_sum_deci_c / (int32_t)filtered_count);
    }
    else
    {
        dev->filtered_max_temp_deci_c = 0;
        dev->filtered_min_temp_deci_c = 0;
        dev->filtered_avg_temp_deci_c = 0;
    }
}

bool accumulator_plan_balance(const accumulator_t *dev,
                              uint16_t planned_mask[NSMBS])
{
    if(planned_mask == NULL)
    {
        return false;
    }

    memset(planned_mask, 0, sizeof(uint16_t) * NSMBS);

    if((dev == NULL) || !dev->smb_ready ||
       !accumulator_final_ring_topology_valid(dev) ||
       !dev->voltage_full_usable || (dev->min_voltage_mv == 0u))
    {
        return false;
    }

    uint8_t ic_count = accumulator_configured_smb_count(dev);

    for(uint8_t ic = 0; ic < ic_count; ic++)
    {
        uint16_t cohort_min_mv = UINT16_MAX;
        uint8_t balance_count = 0u;

        for(uint8_t cell = 0; cell < NCELLS; cell++)
        {
            if(!accumulator_cell_voltage_usable(dev, ic, cell))
            {
                continue;
            }

            uint16_t cell_mv = dev->cell_voltage_mv[ic][cell];
            if((cell_mv >= BALANCE_START_MV) && (cell_mv < cohort_min_mv))
            {
                cohort_min_mv = cell_mv;
            }
        }

        if(cohort_min_mv == UINT16_MAX)
        {
            continue;
        }

        for(uint8_t cell = 0; cell < NCELLS; cell++)
        {
            if(!accumulator_cell_voltage_usable(dev, ic, cell))
            {
                continue;
            }

            uint16_t cell_mv = dev->cell_voltage_mv[ic][cell];
            if((cell_mv >= BALANCE_START_MV) &&
			   (cell_mv > cohort_min_mv) &&
			   ((uint16_t)(cell_mv - cohort_min_mv) > BALANCE_ON_DELTA_MV) &&
               (balance_count < BALANCE_MAX_CELLS_PER_SEG))
            {
                planned_mask[ic] |= (uint16_t)(1u << cell);
                balance_count++;
            }
        }
    }

    return true;
}

int accumulator_set_balance(accumulator_t *dev)
{
    uint16_t planned_mask[NSMBS] = {0u};

    if((dev == NULL) || !dev->smb_transport_ready ||
       !accumulator_final_ring_topology_valid(dev) ||
       !accumulator_plan_balance(dev, planned_mask))
    {
        return -1;
    }

    bool any_balance_requested = false;
    for(uint8_t ic = 0u; ic < NSMBS; ic++)
    {
        if(planned_mask[ic] != 0u)
        {
            any_balance_requested = true;
            break;
        }
    }

    /* A zero-cell plan is not a reason to expose the Sx switches. Keep the
     * verified durable-zero state if it is already established; otherwise
     * establish it once with MUTE + DCC/PWM zero/readback. This also prevents
     * a charge scan with no balancing demand from clearing the BMS_OK
     * durable-zero qualification merely because UNMUTE succeeded. */
    if(!any_balance_requested)
    {
        if(dev->last_balance_mute_ok &&
           dev->last_balance_durable_zero_verified)
        {
            dev->last_balance_unmute_ok = false;
            dev->last_balance_inhibit_reason =
                (uint8_t)ACCUMULATOR_BALANCE_INHIBIT_NONE;
            return 0;
        }
        return accumulator_emergency_balance_inhibit(
            dev, ACCUMULATOR_BALANCE_INHIBIT_NONE);
    }

    adbms6830_driver_t *smb = &dev->smb;
    adbms6830_asic *smb_ics = (smb->ics != NULL) ? smb->ics : dev->smb_ics;
    uint8_t ic_count = accumulator_configured_smb_count(dev);
    HAL_StatusTypeDef mute_status = HAL_ERROR;
    HAL_StatusTypeDef cfg_status = HAL_ERROR;
    HAL_StatusTypeDef pwm_status = HAL_ERROR;
    HAL_StatusTypeDef verify_status = HAL_ERROR;
    HAL_StatusTypeDef unmute_status = HAL_ERROR;
    int result = -1;

    dev->last_balance_mute_ok = false;
    dev->last_balance_durable_zero_verified = false;
    dev->last_balance_unmute_ok = false;
    dev->last_balance_inhibit_reason = (uint8_t)ACCUMULATOR_BALANCE_INHIBIT_NONE;

    adbms_spi_lock();

    /* Program a new balance plan only while discharge is muted.  This avoids
     * exposing a partially written CFGB/PWM image on the physical switches. */
    mute_status = adbms6830_mute_checked(smb);
    dev->last_balance_mute_ok = (mute_status == HAL_OK);
    if(mute_status == HAL_OK)
    {
        /* Applying balance owns only DCC/PWM.  It must not silently rewrite
         * the separate discharge-timer policy. */
        for(uint8_t ic = 0u; ic < ic_count; ic++)
        {
            accumulator_clear_balance_shadow(&smb_ics[ic]);
            for(uint8_t cell = 0u; cell < NCELLS; cell++)
            {
                if((planned_mask[ic] & (uint16_t)(1u << cell)) != 0u)
                {
                    accumulator_set_balance_pwm_cell(&smb_ics[ic],
                                                      cell,
                                                      BALANCE_PWM_DUTY);
                }
            }
        }

        cfg_status = adbms6830_wrcfgb_checked_reason(
            smb, ADBMS6830_CFGB_WRITE_BALANCE_APPLY);
        if(cfg_status == HAL_OK)
        {
            pwm_status = adbms6830_write_pwm_checked(smb);
        }
        if((cfg_status == HAL_OK) && (pwm_status == HAL_OK))
        {
            verify_status = adbms6830_verify_balance_readback(smb);
        }

        if((cfg_status == HAL_OK) && (pwm_status == HAL_OK) &&
           (verify_status == HAL_OK))
        {
            /* Only a completely written/read-back plan is eligible to become
             * physically active.  UNMUTE is deliberately the final bus action. */
            unmute_status = adbms6830_unmute_checked(smb);
            dev->last_balance_unmute_ok = (unmute_status == HAL_OK);
            if(unmute_status == HAL_OK)
            {
                result = 0;
            }
        }
    }

    if(result != 0)
    {
        /* Keep or re-establish the fast inhibit, then force the durable state
         * back to DCC/PWM zero.  Do not leave a latent nonzero plan underneath
         * MUTE after any failed apply/unmute transaction. */
        (void)adbms6830_mute_checked(smb);
        accumulator_best_effort_clear_balance_locked(smb, smb_ics, ic_count);
        dev->last_balance_inhibit_reason =
            (uint8_t)ACCUMULATOR_BALANCE_INHIBIT_WRITE_FAILURE;
    }

    adbms_spi_unlock();
    return result;
}

int accumulator_emergency_balance_inhibit(
    accumulator_t *dev,
    accumulator_balance_inhibit_reason_t reason)
{
    if((dev == NULL) || !dev->smb_transport_ready ||
       !accumulator_final_ring_topology_valid(dev))
    {
        return -1;
    }

    adbms6830_driver_t *smb = &dev->smb;
    adbms6830_asic *smb_ics = (smb->ics != NULL) ? smb->ics : dev->smb_ics;
    uint8_t ic_count = accumulator_configured_smb_count(dev);
    HAL_StatusTypeDef mute_status;
    HAL_StatusTypeDef cfg_status;
    HAL_StatusTypeDef pwm_status;
    HAL_StatusTypeDef verify_status = HAL_ERROR;

    dev->last_balance_mute_ok = false;
    dev->last_balance_durable_zero_verified = false;
    dev->last_balance_unmute_ok = false;
    dev->last_balance_inhibit_reason = (uint8_t)reason;

    adbms_spi_lock();

    /* Stage 1: fastest ASIC-native discharge inhibit. */
    mute_status = adbms6830_mute_checked(smb);
    dev->last_balance_mute_ok = (mute_status == HAL_OK);

    /* Stage 2: make the safe state durable even if the ADBMS watchdog later
     * clears MUTE.  Clear every static/PWM request and disable the discharge
     * timer, then require physical-register readback. */
    adbms6830_disable_discharge_timer_shadow(smb);
    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        accumulator_clear_balance_shadow(&smb_ics[ic]);
    }

    cfg_status = adbms6830_wrcfgb_checked_reason(
        smb, ADBMS6830_CFGB_WRITE_BALANCE_CLEAR);
    pwm_status = adbms6830_write_pwm_checked(smb);
    if((cfg_status == HAL_OK) && (pwm_status == HAL_OK))
    {
        verify_status = adbms6830_verify_balance_readback(smb);
    }

    dev->last_balance_durable_zero_verified =
        (cfg_status == HAL_OK) && (pwm_status == HAL_OK) &&
        (verify_status == HAL_OK);

    if(!dev->last_balance_durable_zero_verified)
    {
        accumulator_best_effort_clear_balance_locked(smb, smb_ics, ic_count);
    }

    adbms_spi_unlock();

    /* MUTE failure is still a failed emergency transition even when the slower
     * durable zero ultimately succeeds.  Callers retain both result bits. */
    return (dev->last_balance_mute_ok &&
            dev->last_balance_durable_zero_verified) ? 0 : -1;
}

int accumulator_balance_requalify_and_unmute(accumulator_t *dev)
{
    if((dev == NULL) || !dev->smb_transport_ready ||
       !accumulator_final_ring_topology_valid(dev))
    {
        return -1;
    }

    adbms6830_driver_t *smb = &dev->smb;
    int result = -1;

    dev->last_balance_unmute_ok = false;
    adbms_spi_lock();

    /* The caller owns policy qualification; this function owns the final
     * hardware proof.  Never UNMUTE until the exact programmed DCC/PWM image
     * has been read back successfully. */
    if(adbms6830_verify_balance_readback(smb) == HAL_OK)
    {
        if(adbms6830_unmute_checked(smb) == HAL_OK)
        {
            dev->last_balance_unmute_ok = true;
            result = 0;
        }
    }

    adbms_spi_unlock();
    return result;
}

int accumulator_clear_balance(accumulator_t *dev)
{
    return accumulator_emergency_balance_inhibit(
        dev, ACCUMULATOR_BALANCE_INHIBIT_USER);
}

