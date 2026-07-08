/*
 * accumulator.c
 *
 *  Created on: Feb 1, 2024
 *      Author: cole
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#include "ext_drivers/accumulator.h"
#include <math.h>
#include <string.h>

static uint8_t sensor_num = 0;

uint8_t accumulator_configured_smb_count(const accumulator_t *dev)
{
    if((dev == NULL) || (dev->smb.num_ics <= 0))
    {
        return 0u;
    }

    return (dev->smb.num_ics > NSMBS) ? (uint8_t)NSMBS : (uint8_t)dev->smb.num_ics;
}

void smb_read_voltage(adbms6830_driver_t* dev);
void smb_read_temp(adbms6830_driver_t* dev);
void apm_read_vbadc_viadc(adbms2950_driver_t* apm);
void apm_read_temps(adbms2950_driver_t* apm);

static void accumulator_clear_balance_shadow(adbms6830_asic *ic)
{
    if(ic == NULL)
    {
        return;
    }

    ic->tx_cfgb.dtmen = 1u;
    ic->tx_cfgb.dtrng = RANG_0_TO_63_MIN;
    ic->tx_cfgb.dcto = TIME_1MIN_OR_0_26HR;
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

void accumulator_init(accumulator_t *dev,
				      SPI_HandleTypeDef *hspi,
					  GPIO_TypeDef *cs_port_a,
					  GPIO_TypeDef *cs_port_b,
					  uint16_t cs_pin_a,
					  uint16_t cs_pin_b,
					  TIM_HandleTypeDef* htim)
{
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
	memset(dev->cell_voltage_mv, 0, sizeof(dev->cell_voltage_mv));
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
        (void)HAL_TIM_Base_Start(htim);
    }

	// Init pack monitor, just on port A. Disabled by default until ADBMS2950
	// NDA documentation and board bring-up are complete.
#if AMS_ENABLE_APM_2950_DEBUG
	adbms2950_init(&dev->apm, NAPMS, dev->apm_ics, hspi, cs_port_a, cs_port_a, cs_pin_a, cs_pin_a, htim);
#else
	memset(&dev->apm, 0, sizeof(dev->apm));
	memset(dev->apm_ics, 0, sizeof(dev->apm_ics));
#endif

	adBms6830_init(&dev->smb, NSMBS, dev->smb_ics, hspi, cs_port_a, cs_port_b, cs_pin_a, cs_pin_b, htim);
}

int accumulator_read_volt(accumulator_t *dev)
{
	int ret = 0;

	if(dev == NULL)
	{
		return -1;
	}

	smb_read_voltage(&dev->smb);
//	apm_read_vbadc_viadc(&dev->apm);
//	adbms6830_us_delay(&dev->smb, 5000);



    return ret;
}

void smb_read_voltage(adbms6830_driver_t* dev)
{
    if(dev == NULL)
    {
        return;
    }

	adbms6830_wakeup(dev);
//	adbms6830_wrcfga(dev);
//	adbms6830_wrcfgb(dev);

	// Wait ~3ms for the precision voltage reference to warm up and settle
	adbms6830_us_delay(dev, 3000);

	// 2. START ADC CONVERSION
	adbms6830_start_adc_cell_voltage_measurement(dev);

	// 3. WAIT FOR THE FIRST CONVERSION CYCLE TO FINISH
	adbms6830_us_delay(dev, 5000);

	// 4. SNAP, READ, AND PARSE
	adbms6830_read_cell_voltages(dev);
//	adbms6830_us_delay(dev, 2000);
//	adbms6830_wakeup(dev);

}

void smb_read_temp(adbms6830_driver_t* dev)
{
    if(dev == NULL)
    {
        return;
    }

    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        dev->last_temp_updated_mask[ic] = 0u;
    }

//	adbms6830_wakeup(dev);
//	adbms6830_wrcfga(dev);
//	adbms6830_wrcfgb(dev);

//	sensor_num = ((sensor_num) % (NTEMPS / 3)) + 1u;
//	sensor_num = ((sensor_num) % (NTEMPS)) + 1u;
//	adbms6830_us_delay(dev, 3000);
//	mux_read_gpio_voltage(dev, sensor_num - 1u);
//	adbms6830_us_delay(dev, 3000);
//	adbms6830_wakeup(dev);
//	adbms6830_us_delay(dev, 3000);

    adbms6830_wakeup(dev);
//    sensor_num = ((sensor_num) % (NTEMPS)) + 1u;
    sensor_num = (sensor_num % (NTEMPS / 3)) + 1u;

    mux_set_channel(dev, sensor_num - 1u);
    adbms6830_us_delay(dev, 2000u);
    mux_set_channel(dev, sensor_num + 7u);
    adbms6830_us_delay(dev, 2000u);
    mux_set_channel(dev, sensor_num + 15u);
    adbms6830_us_delay(dev, 2000u);

    adbms6830_wakeup(dev);
    mux_read_gpio_voltage(dev, sensor_num - 1u);
    adbms6830_us_delay(dev, 2000u);
    mux_read_gpio_voltage(dev, sensor_num + 7u);
    adbms6830_us_delay(dev, 2000u);
    mux_read_gpio_voltage(dev, sensor_num + 15u);
    adbms6830_us_delay(dev, 2000u);

//	adbms6830_us_delay(dev, 3000);
//	mux_read_gpio_voltage(dev, sensor_num + 7u);
//	adbms6830_us_delay(dev, 3000);
//	mux_read_gpio_voltage(dev, sensor_num + 15u);

}

void apm_read_vbadc_viadc(adbms2950_driver_t* apm)
{
    if(apm == NULL)
    {
        return;
    }

	// Set GPO to enabled to read VBAT voltage
	adbms2950_gpo_set(apm, HVEN1, GPO_SET);
	adbms2950_gpo_set(apm, HVEN2, GPO_SET);
	adbms2950_wakeup(apm);
	adbms2950_wrcfga(apm);
	adbms2950_rdcfga(apm);

	// Read VBxADC results (ADCs are in continuous mode)
	adbms2950_wakeup(apm);
	adbms2950_rdvb(apm);
	apm->vbat_adc[0] = (int16_t)(apm->ics[0].vbat.vbat1) * VBAT1_SCALE;
	apm->vbat_adc[1] = (int16_t)(apm->ics[0].vbat.vbat2) * VBAT2_SCALE;
	apm->vbat[0] = apm->vbat_adc[0] * VBAT_DIV_SCALE;
	apm->vbat[1] = apm->vbat_adc[1] * VBAT_DIV_SCALE;

	// Read VxADC results (ADCs are in continuous mode)
	adbms2950_wakeup(apm);
	adbms2950_rdi(apm);
	// TODO: These values seem off. Verify in DS and hardware
	apm->vi_adc[0] = (int32_t)(apm->ics[0].i.i1) * VI1_SCALE;
	apm->vi_adc[1] = (int32_t)(apm->ics[0].i.i2) * VI2_SCALE;
	apm->current[0] = apm->vi_adc[0] * CURRENT_R_SCALE;
	apm->current[1] = apm->vi_adc[1] * CURRENT_R_SCALE;
}

int accumulator_read_temp(accumulator_t *dev)
{
	int error = 0;

	if(dev == NULL)
	{
		return -1;
	}

//	apm_read_temps(&dev->apm);

	smb_read_temp(&dev->smb);

	return error;
}

void apm_read_temps(adbms2950_driver_t* apm)
{
    if(apm == NULL)
    {
        return;
    }

	adv_ adv;
	adv.ow = OW_OFF; // Open wire detection disabled
	adv.ch = SM_V7_V9; // Single measurement, V7 and V9
	// Start aux ADC
	adbms2950_wakeup(apm);
	adbms2950_adv(apm, &adv);

	// Poll aux ADC
	adbms2950_wakeup(apm);
	adbms2950_plv(apm);

	// Read aux ADC
	adbms2950_wakeup(apm);
	adbms2950_rdv1d(apm);

	apm->vtemp_adc[0] = (int16_t)(apm->ics[0].vr.v_codes[9]) * VxA_SCALE; // V7A
	apm->vtemp_adc[1] = (int16_t)(apm->ics[0].vr.v_codes[11]) * VxB_SCALE; // V9B
	// TODO: calibrate NTCs on APM and set 'dev->apm.temps[]' values. this just copies the voltage for now
	apm->temps[0] = apm->vtemp_adc[0];
	apm->temps[1] = apm->vtemp_adc[1];
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
    (void)addr7;

    if((dev == NULL) || (channel >= NTEMPS))
    {
        return -1;
    }

    return mux_set_channel(&dev->smb, channel);
}

float NXFT15XV103FEAB050_convert(float ratio)
{
	// TODO: Verify
    if(!isfinite(ratio))
    {
        return 0.0f;
    }
	double a = 104.517;
	double b = 0.221876;
	return a * pow(b, ratio);
}

float convert_adc_to_volt(int value)
{
	return (value + 10000) * .000150;
}

static uint16_t accumulator_code_to_mv(int16_t code)
{
    if((code == 0) || (code == INT16_MIN))
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
    float temp_c = (float)deci_c / 10.0f;
    float temp_k = temp_c + 273.15f;

    if((temp_k <= 0.0f) || !isfinite(temp_k))
    {
        return 0;
    }

    const float a = 3.354016435e-3f;
    const float b = 2.565235509e-4f;
    float r = 10000.0f * expf(((1.0f / temp_k) - a) / b);

    if((r <= 0.0f) || !isfinite(r))
    {
        return 0;
    }

    float voltage = 5.0f * 10000.0f / (r + 10000.0f);
    float raw = (voltage / 0.000150f) - 10000.0f;

    if(!isfinite(raw))
    {
        return 0;
    }
    if(raw >= (float)INT16_MAX)
    {
        return INT16_MAX;
    }
    if(raw <= (float)INT16_MIN)
    {
        return INT16_MIN;
    }

    return (int16_t)lroundf(raw);
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
                uint32_t age_ms = (now_ms >= dev->hil_cell_last_update_ms[seg][cell]) ?
                                  (now_ms - dev->hil_cell_last_update_ms[seg][cell]) : 0u;
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
                uint32_t age_ms = (now_ms >= dev->hil_temp_last_update_ms[seg][sensor]) ?
                                  (now_ms - dev->hil_temp_last_update_ms[seg][sensor]) : 0u;
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
                uint32_t age_ms = (now >= dev->cell_voltage_last_update_ms[ic][cell]) ?
                                  (now - dev->cell_voltage_last_update_ms[ic][cell]) : 0u;

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

static bool accumulator_temp_raw_to_mv(int16_t raw, uint16_t *mv_out)
{
    if((raw == -1) || (raw == INT16_MIN))
    {
        return false;
    }

    float mv = ((float)raw + 10000.0f) * 0.15f;
    if(!isfinite(mv) || (mv < 0.0f) || (mv > 5000.0f))
    {
        return false;
    }

    if(mv_out != NULL)
    {
        *mv_out = (uint16_t)(mv + 0.5f);
    }
    return true;
}

static bool accumulator_temp_code_to_deci_c(int16_t raw, int16_t *deci_c)
{
    if((raw == -1) || (raw == INT16_MIN) || (raw == 0))
    {
        return false;
    }

    float temp = voltage_to_temp(raw);
    if(!isfinite(temp) ||
       (temp < ((float)ACCUMULATOR_TEMP_VALID_MIN_DECI_C / 10.0f)) ||
       (temp > ((float)ACCUMULATOR_TEMP_VALID_MAX_DECI_C / 10.0f)))
    {
        return false;
    }

    if(deci_c != NULL)
    {
        *deci_c = (int16_t)lroundf(temp * 10.0f);
    }
    return true;
}

static bool accumulator_temp_raw_looks_open(int16_t raw)
{
    uint16_t mv = 0u;
    return accumulator_temp_raw_to_mv(raw, &mv) && (mv <= ACCUMULATOR_TEMP_OPEN_LOW_MV);
}

static bool accumulator_temp_raw_looks_short(int16_t raw)
{
    uint16_t mv = 0u;
    return accumulator_temp_raw_to_mv(raw, &mv) && (mv >= ACCUMULATOR_TEMP_SHORT_HIGH_MV);
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
                bool had_previous = dev->temp_sensor_valid[ic][sensor];
                int16_t previous_deci_c = dev->temp_deci_c[ic][sensor];
                uint32_t previous_tick = dev->temp_last_update_ms[ic][sensor];

                dev->temp_raw_code[ic][sensor] = raw;

                if(accumulator_temp_code_to_deci_c(raw, &deci_c))
                {
                    if(had_previous)
                    {
                        uint16_t delta_deci_c = (deci_c >= previous_deci_c) ?
                                                (uint16_t)(deci_c - previous_deci_c) :
                                                (uint16_t)(previous_deci_c - deci_c);
                        uint32_t elapsed_ms = (now >= previous_tick) ? (now - previous_tick) : 0u;

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
                    if(accumulator_temp_raw_looks_open(raw))
                    {
                        dev->temp_open_mask[ic] |= bit;
                        open_count++;
                    }
                    else if(accumulator_temp_raw_looks_short(raw))
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
                uint32_t age_ms = (now >= dev->temp_last_update_ms[ic][sensor]) ?
                                  (now - dev->temp_last_update_ms[ic][sensor]) : 0u;

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

int accumulator_set_balance(accumulator_t *dev)
{
    if((dev == NULL) || !dev->voltage_full_usable || (dev->min_voltage_mv == 0u))
    {
        return -1;
    }

    adbms6830_driver_t *smb = &dev->smb;
    adbms6830_asic *smb_ics = (smb->ics != NULL) ? smb->ics : dev->smb_ics;
    uint8_t ic_count = accumulator_configured_smb_count(dev);

    for(uint8_t ic = 0; ic < ic_count; ic++)
    {
        uint16_t cohort_min_mv = UINT16_MAX;
        uint8_t balance_count = 0u;
        accumulator_clear_balance_shadow(&smb_ics[ic]);

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
               (cell_mv > (uint16_t)(cohort_min_mv + BALANCE_ON_DELTA_MV)) &&
               (balance_count < BALANCE_MAX_CELLS_PER_SEG))
            {
                accumulator_set_balance_pwm_cell(&smb_ics[ic], cell, BALANCE_PWM_DUTY);
                balance_count++;
            }
        }
    }

    adbms6830_wakeup(smb);
    if(adbms6830_wrcfgb_checked(smb) != HAL_OK)
    {
        return -1;
    }
    return (adbms6830_write_pwm_checked(smb) == HAL_OK) ? 0 : -1;
}

int accumulator_clear_balance(accumulator_t *dev)
{
    if(dev == NULL)
    {
        return -1;
    }

    adbms6830_driver_t *smb = &dev->smb;
    adbms6830_asic *smb_ics = (smb->ics != NULL) ? smb->ics : dev->smb_ics;
    uint8_t ic_count = accumulator_configured_smb_count(dev);

    for(uint8_t ic = 0; ic < ic_count; ic++)
    {
        accumulator_clear_balance_shadow(&smb_ics[ic]);
    }
    adbms6830_wakeup(smb);
    HAL_StatusTypeDef cfg_status = adbms6830_wrcfgb_checked(smb);
    HAL_StatusTypeDef pwm_status = adbms6830_write_pwm_checked(smb);
    return ((cfg_status == HAL_OK) && (pwm_status == HAL_OK)) ? 0 : -1;
}
