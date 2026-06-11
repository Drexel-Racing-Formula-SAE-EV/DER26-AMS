/*
 * accumulator.c
 *
 *  Created on: Feb 1, 2024
 *      Author: cole
 */

#include "ext_drivers/accumulator.h"
#include <math.h>

static uint8_t sensor_num = 0;

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
	dev->total_volt = 0;
	dev->max_temp = 0.0f;
	dev->avg_temp = 0.0f;
	dev->max_volt = 0.0f;
	dev->min_volt = 0.0f;

	HAL_TIM_Base_Start(htim);

	// Init pack monitor, just on port A
	// adbms2950_init(&dev->apm, NAPMS, dev->apm_ics, hspi, cs_port_a, cs_port_a, cs_pin_a, cs_pin_a, htim);

	adBms6830_init(&dev->smb, NSMBS, dev->smb_ics, hspi, cs_port_a, cs_port_a, cs_pin_a, cs_pin_a, htim);
}

int accumulator_read_volt(accumulator_t *dev)
{
	int ret = 0;

	smb_read_voltage(&dev->smb);
//	apm_read_vbadc_viadc(&dev->apm);
//	adbms6830_us_delay(&dev->smb, 5000);



    return ret;
}

void smb_read_voltage(adbms6830_driver_t* dev)
{
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

//	apm_read_temps(&dev->apm);

	smb_read_temp(&dev->smb);

	return error;
}

void apm_read_temps(adbms2950_driver_t* apm)
{
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

	return 0;
}

int accumulator_set_temp_ch(accumulator_t *dev, uint8_t channel)
{
	int error = 0;
	error |= accumulator_set_mux_ch(dev, channel, MUX_ADDR7_00);
	error |= accumulator_set_mux_ch(dev, channel, MUX_ADDR7_01);
    return error;
}

int accumulator_set_mux_ch(accumulator_t *dev, uint8_t channel, uint8_t addr7)
{
	int error = 0;
    return error;
}

float NXFT15XV103FEAB050_convert(float ratio)
{
	// TODO: Verify
	double a = 104.517;
	double b = 0.221876;
	return a * pow(b, ratio);
}

float convert_adc_to_volt(int value)
{
	return (value + 10000) * .000150;
}

void adbms6830_update_cell_voltage_limits(accumulator_t *dev)
{
    int16_t max_code = INT16_MIN;
    int16_t min_code = INT16_MAX;

    for (uint8_t ic = 0; ic < dev->smb.num_ics; ic++)
    {
        for (uint8_t cell = 0u; cell < 15u; cell++)
        {
            int16_t v = dev->smb.ics[ic].cell.c_codes[cell];
            if (v > max_code) max_code = v;
            if (v < min_code) min_code = v;
        }
    }

    dev->max_volt = convert_adc_to_volt(max_code);
    dev->min_volt = convert_adc_to_volt(min_code);
}


void accumulator_update_temp_stats(accumulator_t *dev)
{
	float   max_temp = -273.15f;
	float   sum_temp = 0.0f;
	uint8_t count    = 0u;

	for (uint8_t ic = 0; ic < dev->smb.num_ics; ic++)
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

	dev->max_temp = (count > 0u) ? max_temp  : 0.0f;
	dev->avg_temp = (count > 0u) ? (sum_temp / (float)count) : 0.0f;
}

int accumulator_set_balance(accumulator_t *dev)
{
    adbms6830_driver_t *smb = &dev->smb;
    float min_v = dev->min_volt;

    for(uint8_t ic = 0; ic < NSMBS; ic++)
    {
        uint16_t dcc_mask = 0;
        for(uint8_t cell = 0; cell < NCELLS; cell++)
        {
            float v = convert_adc_to_volt(smb->ics[ic].cell.c_codes[cell]);
            if((v - min_v) > BALANCE_THRESH)
            {
                dcc_mask |= (1 << cell);
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
    adbms6830_driver_t *smb = &dev->smb;
    for(uint8_t ic = 0; ic < NSMBS; ic++)
    {
        smb->ics[ic].tx_cfgb.dcc = 0;
    }
    adbms6830_wakeup(smb);
    adbms6830_wrcfgb(smb);
    return 0;
}
