/*
 * accumulator.c
 *
 *  Created on: Feb 1, 2024
 *      Author: cole
 */

#include "ext_drivers/accumulator.h"
#include <math.h>

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

	if(htim != NULL)
    {
        (void)HAL_TIM_Base_Start(htim);
    }

	// Init pack monitor, just on port A
	// adbms2950_init(&dev->apm, NAPMS, dev->apm_ics, hspi, cs_port_a, cs_port_a, cs_pin_a, cs_pin_a, htim);

	adBms6830_init(&dev->smb, NSMBS, dev->smb_ics, hspi, cs_port_b, cs_port_b, cs_pin_b, cs_pin_b, htim);
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

void accumulator_update_voltage_stats(accumulator_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

    int16_t max_code = INT16_MIN;
    int16_t min_code = INT16_MAX;
    float total = 0.0f;
    uint16_t valid_count = 0u;

    uint8_t ic_count = accumulator_configured_smb_count(dev);

    for (uint8_t ic = 0; ic < ic_count; ic++)
    {
        for (uint8_t cell = 0u; cell < NCELLS; cell++)
        {
            int16_t code = dev->smb.ics[ic].cell.c_codes[cell];

            /* Skip obvious uninitialized / invalid readings. */
            if ((code == 0) || (code == INT16_MIN))
            {
                continue;
            }

            float v = convert_adc_to_volt(code);
            if ((v < 0.5f) || (v > 5.0f))
            {
                continue;
            }

            if (code > max_code) max_code = code;
            if (code < min_code) min_code = code;
            total += v;
            valid_count++;
        }
    }

    dev->valid_voltage_count = valid_count;

    if (valid_count > 0u)
    {
        dev->max_volt = convert_adc_to_volt(max_code);
        dev->min_volt = convert_adc_to_volt(min_code);
        dev->total_volt = total;
    }
    else
    {
        dev->max_volt = 0.0f;
        dev->min_volt = 0.0f;
        dev->total_volt = 0.0f;
    }
}


void accumulator_update_temp_stats(accumulator_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

	float   max_temp = -273.15f;
	float   sum_temp = 0.0f;
	uint8_t count    = 0u;

	uint8_t ic_count = accumulator_configured_smb_count(dev);

	for (uint8_t ic = 0; ic < ic_count; ic++)
	{
		for (uint8_t sensor = 0u; sensor < NTEMPS; sensor++)
		{
			int16_t raw = dev->smb.ics[ic].temp.raw[sensor];

			if (raw == -1 || raw == INT16_MIN || raw == 0) continue;

			float temp    = voltage_to_temp(raw);

			if (temp < -40.0f || temp > 150.0f) continue;

			if (temp > max_temp) max_temp = temp;
			sum_temp += temp;
			count++;
		}
	}

	dev->valid_temp_count = count;
	dev->max_temp = (count > 0u) ? max_temp  : 0.0f;
	dev->avg_temp = (count > 0u) ? (sum_temp / (float)count) : 0.0f;
}

int accumulator_set_balance(accumulator_t *dev)
{
    if((dev == NULL) || (dev->min_volt <= 0.0f) || (dev->valid_voltage_count == 0u))
    {
        return -1;
    }

    adbms6830_driver_t *smb = &dev->smb;
    float min_v = dev->min_volt;

    uint8_t ic_count = accumulator_configured_smb_count(dev);

    for(uint8_t ic = 0; ic < ic_count; ic++)
    {
        uint16_t dcc_mask = 0;
        for(uint8_t cell = 0; cell < NCELLS; cell++)
        {
            int16_t code = smb->ics[ic].cell.c_codes[cell];
            if((code == 0) || (code == INT16_MIN))
            {
                continue;
            }

            float v = convert_adc_to_volt(code);
            if((v < 0.5f) || (v > 5.0f))
            {
                continue;
            }

            if((v - min_v) > BALANCE_THRESH)
            {
                dcc_mask |= (uint16_t)(1u << cell);
            }
        }
        smb->ics[ic].tx_cfgb.dcc = dcc_mask;
    }

    adbms6830_wakeup(smb);
    adbms6830_wrcfgb(smb);
    return 0;
}

int accumulator_clear_balance(accumulator_t *dev)
{
    if(dev == NULL)
    {
        return -1;
    }

    adbms6830_driver_t *smb = &dev->smb;
    uint8_t ic_count = accumulator_configured_smb_count(dev);

    for(uint8_t ic = 0; ic < ic_count; ic++)
    {
        smb->ics[ic].tx_cfgb.dcc = 0;
    }
    adbms6830_wakeup(smb);
    adbms6830_wrcfgb(smb);
    return 0;
}
