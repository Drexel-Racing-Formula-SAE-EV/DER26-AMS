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
	adbms6830_wrcfga(dev);
	adbms6830_wrcfgb(dev);

	// Wait ~3ms for the precision voltage reference to warm up and settle
	adbms6830_us_delay(dev, 3000);

	// 2. START ADC CONVERSION
	adbms6830_start_adc_cell_voltage_measurement(dev);

	// 3. WAIT FOR THE FIRST CONVERSION CYCLE TO FINISH
	adbms6830_us_delay(dev, 5000);

	// 4. SNAP, READ, AND PARSE
	adbms6830_read_cell_voltages(dev);
	adbms6830_wakeup(dev);

}

void smb_read_temp(adbms6830_driver_t* dev)
{
//    adbms6830_wakeup(dev);
//    adbms6830_wrcfga(dev);
//    adbms6830_wrcfgb(dev);
	adbms6830_wakeup(dev);
	adbms6830_wrcfga(dev);
	adbms6830_wrcfgb(dev);
//    for (uint8_t sensor = 0; sensor < 24u; sensor++)
//    {
      sensor_num = ((sensor_num) % (NTEMPS / 3)) + 1u;
      mux_read_gpio_voltage(dev, sensor_num - 1u);
      adbms6830_us_delay(dev, 3000);
      mux_read_gpio_voltage(dev, sensor_num + 7u);
      adbms6830_us_delay(dev, 3000);
      mux_read_gpio_voltage(dev, sensor_num + 15u);
//	adbms6830_us_delay(dev, 50000u);
//	adbms6830_us_delay(dev, 50000u);
//	adbms6830_us_delay(dev, 50000u);
//    }
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
