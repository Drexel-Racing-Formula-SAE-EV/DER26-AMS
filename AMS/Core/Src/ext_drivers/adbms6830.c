/*
 * adbms6830.c
 *
 *  Created on: May 13, 2025
 *      Author: realb
 */

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "ext_drivers/adbms6830_functions.h"

unsigned char shared_buf[BUFSZ] = {0};
uint8_t write_buf[BUFSZ] = {0};

#define RDCVALL_CMD         {0x00, 0x0C}

#define RDCVALL_DATA_SZ     32   /* 16 cells x 2 bytes */

#define RDCVALL_RX_DATA     34   /* 32 data bytes + 2 PEC/counter bytes */

///* =========================================================================
// * ICOMM / FCOMM nibble values (ADBMS6830 datasheet, COMM register)
// * ========================================================================= */
//#define ICOMM_START     0x6u  /* Generate I2C START then clock byte  */
//#define ICOMM_BLANK     0x0u  /* No START, continue clocking         */
//#define ICOMM_STOP      0x1u  /* Generate I2C STOP after byte        */
//#define FCOMM_ACK       0x0u  /* Master ACK                          */
//#define FCOMM_NACK_STOP 0x9u  /* Master NACK + STOP                  */
//#define I2C_WRITE       0x00u
//#define I2C_READ        0x01u
//
///* ADG728 I2C addresses (A1,A0 pins) */
#define ADG728_U2_ADDR  0x4Cu  /* TEMP0-7  -> GPIO1 */
#define ADG728_U3_ADDR  0x4Du  /* TEMP8-15 -> GPIO2 */
#define ADG728_U4_ADDR  0x4Eu  /* TEMP16-23-> GPIO3 */
//
///* ADAX channel bytes for GPIO1/2/3 (CH enum value packed into cmd[1]) */
#define ADAX_GPIO1  0x11u
#define ADAX_GPIO2  0x12u
#define ADAX_GPIO3  0x13u

#define SENSORS_PER_MUX   8u
#define ADAX_CMD_BYTE0    0x04u   /* ADAX base */
#define ADAX_CMD_BYTE1    0x60u   /* CH=000 = all GPIOs, continuous off */

/* WRCOMM / STCOMM copies — same bytes as in adg728_i2c.c but local
 * to avoid pulling in the adBms_Application layer headers here.       */
#define ICOMM_START_  0x6u
#define ICOMM_BLANK_  0x0u
#define ICOMM_STOP_   0x1u
#define FCOMM_ACK_    0x0u
#define FCOMM_NACK_STOP_ 0x9u



/* configuration registers commands */
static uint8_t WRCFGA[2]        = { 0x00, 0x01 };
static uint8_t WRCFGB[2]        = { 0x00, 0x24 };
static uint8_t RDCFGA[2]        = { 0x00, 0x02 };
static uint8_t RDCFGB[2]        = { 0x00, 0x26 };

/* Read cell voltage result registers commands */
static uint8_t RDCVA[2]         = { 0x00, 0x04 };
static uint8_t RDCVB[2]         = { 0x00, 0x06 };
static uint8_t RDCVC[2]         = { 0x00, 0x08 };
static uint8_t RDCVD[2]         = { 0x00, 0x0A };
static uint8_t RDCVE[2]         = { 0x00, 0x09 };
static uint8_t RDCVF[2]         = { 0x00, 0x0B };
static uint8_t RDCVALL[2]       = { 0x00, 0x0C };

/* Read average cell voltage result registers commands commands */
static uint8_t RDACA[2]         = { 0x00, 0x44 };
static uint8_t RDACB[2]         = { 0x00, 0x46 };
static uint8_t RDACC[2]         = { 0x00, 0x48 };
static uint8_t RDACD[2]         = { 0x00, 0x4A };
static uint8_t RDACE[2]         = { 0x00, 0x49 };
static uint8_t RDACF[2]         = { 0x00, 0x4B };
static uint8_t RDACALL[2]       = { 0x00, 0x4C };

/* Read s voltage result registers commands */
static uint8_t RDSVA[2]         = { 0x00, 0x03 };
static uint8_t RDSVB[2]         = { 0x00, 0x05 };
static uint8_t RDSVC[2]         = { 0x00, 0x07 };
static uint8_t RDSVD[2]         = { 0x00, 0x0D };
static uint8_t RDSVE[2]         = { 0x00, 0x0E };
static uint8_t RDSVF[2]         = { 0x00, 0x0F };
static uint8_t RDSALL[2]        = { 0x00, 0x10 };

/* Read c and s results */
static uint8_t RDCSALL[2]       = { 0x00, 0x11 };
static uint8_t RDACSALL[2]      = { 0x00, 0x51 };

/* Read all AUX and all Status Registers */
static uint8_t RDASALL[2]       = { 0x00, 0x35 };

/* Read filtered cell voltage result registers*/
static uint8_t RDFCA[2]         = { 0x00, 0x12 };
static uint8_t RDFCB[2]         = { 0x00, 0x13 };
static uint8_t RDFCC[2]         = { 0x00, 0x14 };
static uint8_t RDFCD[2]         = { 0x00, 0x15 };
static uint8_t RDFCE[2]         = { 0x00, 0x16 };
static uint8_t RDFCF[2]         = { 0x00, 0x17 };
static uint8_t RDFCALL[2]       = { 0x00, 0x18 };

/* Read aux results */
static uint8_t RDAUXA[2]        = { 0x00, 0x19 };
static uint8_t RDAUXB[2]        = { 0x00, 0x1A };
static uint8_t RDAUXC[2]        = { 0x00, 0x1B };
static uint8_t RDAUXD[2]        = { 0x00, 0x1F };

/* Read redundant aux results */
static uint8_t RDRAXA[2]        = { 0x00, 0x1C };
static uint8_t RDRAXB[2]        = { 0x00, 0x1D };
static uint8_t RDRAXC[2]        = { 0x00, 0x1E };
static uint8_t RDRAXD[2]        = { 0x00, 0x25 };

/* Read status registers */
static uint8_t RDSTATA[2]       = { 0x00, 0x30 };
static uint8_t RDSTATB[2]       = { 0x00, 0x31 };
static uint8_t RDSTATC[2]       = { 0x00, 0x32 };
static uint8_t RDSTATCERR[2]    = { 0x00, 0x72 };              /* ERR */
static uint8_t RDSTATD[2]       = { 0x00, 0x33 };
static uint8_t RDSTATE[2]       = { 0x00, 0x34 };

/* Pwm registers commands */
static uint8_t WRPWM1[2]        = { 0x00, 0x20 };
static uint8_t RDPWM1[2]        = { 0x00, 0x22 };

static uint8_t WRPWM2[2]        = { 0x00, 0x21 };
static uint8_t RDPWM2[2]        = { 0x00, 0x23 };

/* Clear commands */
static uint8_t CLRCELL[2]       = { 0x07, 0x11 };
static uint8_t CLRAUX [2]       = { 0x07, 0x12 };
static uint8_t CLRSPIN[2]       = { 0x07, 0x16 };
static uint8_t CLRFLAG[2]       = { 0x07, 0x17 };
static uint8_t CLRFC[2]         = { 0x07, 0x14 };
static uint8_t CLOVUV[2]        = { 0x07, 0x15 };

/* Poll adc command */
static uint8_t PLADC[2]         = { 0x07, 0x18 };
static uint8_t PLAUT[2]         = { 0x07, 0x19 };
static uint8_t PLCADC[2]        = { 0x07, 0x1C };
static uint8_t PLSADC[2]        = { 0x07, 0x1D };
static uint8_t PLAUX1[2]        = { 0x07, 0x1E };
static uint8_t PLAUX2[2]        = { 0x07, 0x1F };

/* Diagn command */
static uint8_t DIAG[2]         = {0x07 , 0x15};

/* GPIOs Comm commands */
static uint8_t WRCOMM[2]        = { 0x07, 0x21 };
static uint8_t RDCOMM[2]        = { 0x07, 0x22 };
static uint8_t STCOMM[13]       = { 0x07, 0x23, 0xB9, 0xE4 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};

/* Mute and Unmute commands */
static uint8_t MUTE[2]          = { 0x00, 0x28 };
static uint8_t UNMUTE[2]        = { 0x00, 0x29 };

static uint8_t RSTCC[2]         = { 0x00, 0x2E };
static uint8_t SNAP[2]          = { 0x00, 0x2D };
static uint8_t UNSNAP[2]        = { 0x00, 0x2F };
static uint8_t SRST[2]          = { 0x00, 0x27 };

/* Read SID command */
static uint8_t RDSID[2]         = { 0x00, 0x2C };

static uint16_t cmd_cntr = 0;
static uint16_t rx_pec_error = 0;

/* Mux address and ADAX channel lookup by mux index (0=U2, 1=U3, 2=U4) */
static const uint8_t MUX_ADDRS[3]    = { ADG728_U2_ADDR, ADG728_U3_ADDR, ADG728_U4_ADDR };
static const uint8_t ADAX_CH[3]      = { ADAX_GPIO1,     ADAX_GPIO2,     ADAX_GPIO3     };

/* RDAUXA a_codes[] index for GPIO1/2/3 */
static const uint8_t GPIO_AUX_IDX[3] = { 0u, 1u, 2u };

// SPI communication
void adbms6830_set_cs(adbms6830_driver_t* dev, uint8_t state);
void adbms6830_spi_write(adbms6830_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs);
void adbms6830_spi_write_read(adbms6830_driver_t *dev, uint8_t* tx_Data, uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len, uint8_t use_cs);

// Tx/Rx Utility
void adbms6830_cmd(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ]);
void adbms6830_wr48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data);
void adbms6830_rd48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data);
// Parsing Rx Data
void adbms6830_parse_cfga(adbms6830_driver_t* dev, uint8_t *data);
void adbms6830_parse_cfgb(adbms6830_driver_t* dev, uint8_t *data);
// Packing Tx Data
void adbms6830_pack_cfga(adbms6830_driver_t *dev);
void adbms6830_pack_cfgb(adbms6830_driver_t *dev);
void adbms6830_pack_comm(adbms6830_driver_t* dev);
void adbms6830_pack_clr_flag_data(adbms6830_driver_t* dev);


void adBms6830_init(adbms6830_driver_t* dev,
					   uint8_t num_ics,
					   adbms6830_asic* ics,
					   SPI_HandleTypeDef* hspi,
					   GPIO_TypeDef* cs_port_a,
					   GPIO_TypeDef* cs_port_b,
					   uint16_t cs_pin_a,
					   uint16_t cs_pin_b,
					   TIM_HandleTypeDef *htim)
{
	dev->num_ics = num_ics;
	dev->ics = ics;
	dev->hspi = hspi;
	dev->cs_port[0] = cs_port_a;
	dev->cs_port[1] = cs_port_b;
	dev->cs_pin[0] = cs_pin_a;
	dev->cs_pin[1] = cs_pin_b;
	dev->htim = htim;

	// Set CS pins high
	dev->string = STRING_B;
	adbms6830_set_cs(dev, 1);
	dev->string = STRING_A;
	adbms6830_set_cs(dev, 1);

	adbms6830_srst(dev);
	adbms6830_us_delay(dev, 300);

	adbms6830_reset_cfg(dev);

	adbms6830_wakeup(dev);
	adbms6830_wrcfga(dev);
	adbms6830_wrcfgb(dev);

	// Testing Code

//	float temp_volts[24];
//	float temp_volt;
//
//	for (uint8_t ic = 0; ic < dev->num_ics; ic++)
//	{
////	    adbms6830_read_all_temps(dev, ic, 24);
//	    adbms6830_read_temp_raw(dev, 0, &dev->ics[0].temp.raw[0]);
//
//	    temp_volts[0] = adbms6830_convert_temp(dev, ic, 0, 5.0);
////	    for (uint8_t sensor = 0; sensor < 24; sensor++)
////	    {
////	        temp_volts[sensor] = adbms6830_convert_temp(dev, ic, sensor, 5.0);
////	    }
//	}

//	for (uint8_t sensor = 0; sensor < 24u; sensor++)
//	{
//	    mux_read_gpio_voltage(dev, sensor);
//
//	    for (uint8_t ic = 0; ic < dev->num_ics; ic++)
//	    {
//		float voltage = adbms6830_convert_temp(dev, ic, sensor, 5.0f);
//
//	    }
//
//	    adbms6830_us_delay(dev, 500u);
//	}

//	adbms6830_gpio_i2c_write(dev, ADG728_U2_ADDR, 0x01u);
//	adbms6830_gpio_i2c_write(dev, ADG728_U3_ADDR, 0x00u);
//	adbms6830_gpio_i2c_write(dev, ADG728_U4_ADDR, 0x00u);


//	adbms6830_gpio_i2c_write(dev, ADG728_U2_ADDR, 0x00u);
//	adbms6830_us_delay(dev, 10000);

	adbms6830_wakeup(dev);
}

void adbms6830_reset_cfg(adbms6830_driver_t *dev)
{
	uint16_t vov_value;
	uint16_t vuv_value;
	uint8_t rbits = 12;

	/* ADC Command Configuration */
	dev->adc_config.REDUNDANT_MEASUREMENT = RD_OFF;
	dev->adc_config.AUX_CH_TO_CONVERT = AUX_ALL;
	dev->adc_config.CONTINUOUS_MEASUREMENT = SINGLE;
	dev->adc_config.CELL_OPEN_WIRE_DETECTION = OW_OFF_ALL_CH;
	dev->adc_config.AUX_OPEN_WIRE_DETECTION = AUX_OW_OFF;
	dev->adc_config.OPEN_WIRE_CURRENT_SOURCE = PUP_DOWN;
	dev->adc_config.DISCHARGE_PERMITTED = DCP_OFF;
	dev->adc_config.RESET_FILTER = RSTF_OFF;
	dev->adc_config.INJECT_ERR_SPI_READ = WITHOUT_ERR;

	/* Set Over & Under Voltage conditions */
	dev->thresholds.OV_THRESHOLD = 4.2;
	dev->thresholds.UV_THRESHOLD = 3.0;
	dev->thresholds.OWC_Threshold = 2000;
	dev->thresholds.OWA_Threshold = 50000;

	/* Loop measurement setup */
	dev->loop_manager.LOOP_MEASUREMENT_COUNT = 1;
	dev->loop_manager.MEASUREMENT_LOOP_TIME = 10;
	dev->loop_manager.loop_count = 0;
	dev->loop_manager.pladc_count = 0;
	dev->loop_manager.MEASURE_CELL = ENABLED;
	dev->loop_manager.MEASURE_AVG_CELL = ENABLED;
	dev->loop_manager.MEASURE_F_CELL = ENABLED;
	dev->loop_manager.MEASURE_S_VOLTAGE = ENABLED;
	dev->loop_manager.MEASURE_AUX = DISABLED;
	dev->loop_manager.MEASURE_RAUX = DISABLED;
	dev->loop_manager.MEASURE_STAT = DISABLED;

	for(uint8_t i = 0; i < dev->num_ics; i++)
	{
		float over_voltage = dev->thresholds.OV_THRESHOLD;
		float under_voltage = dev->thresholds.UV_THRESHOLD;
		/* Setup cell_asic */
		/* Init config A */
		dev->ics[i].tx_cfga.refon = PWR_UP;
		dev->ics[i].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */

		/* Init config B */
		over_voltage = (over_voltage - 1.5);
		over_voltage = over_voltage / (16 * 0.000150);
		vov_value = (uint16_t )(over_voltage + 2 * (1 << (rbits - 1)));
		vov_value &= 0xFFF;
		dev->ics[i].tx_cfgb.vov = vov_value;

		under_voltage = (under_voltage - 1.5);
		under_voltage = under_voltage / (16 * 0.000150);
		vuv_value = (uint16_t )(under_voltage + 2 * (1 << (rbits - 1)));
		vuv_value &= 0xFFF;
		dev->ics[i].tx_cfgb.vuv = vuv_value;
	}
}

void adbms6830_wrcfga(adbms6830_driver_t *dev)
{
	adbms6830_asic *ic = dev->ics;
	adbms6830_pack_cfga(dev);
    for (uint8_t cic = 0; cic < dev->num_ics; cic++)
    {
      for (uint8_t data = 0; data < TX_DATA; data++)
      {
        shared_buf[(cic * TX_DATA) + data] = ic[cic].configa.tx_data[data];
      }
    }
	adbms6830_wr48(dev, WRCFGA, shared_buf);
}

void adbms6830_wrcfgb(adbms6830_driver_t *dev)
{
	adbms6830_asic *ic = dev->ics;
	adbms6830_pack_cfgb(dev);
    for (uint8_t cic = 0; cic < dev->num_ics; cic++)
    {
      for (uint8_t data = 0; data < TX_DATA; data++)
      {
        shared_buf[(cic * TX_DATA) + data] = ic[cic].configb.tx_data[data];
      }
    }
	adbms6830_wr48(dev, WRCFGB, shared_buf);
}

void adbms6830_rdcfga(adbms6830_driver_t *dev)
{
	adbms6830_rd48(dev, RDCFGA, shared_buf);
	adbms6830_parse_cfga(dev, shared_buf);
}

void adbms6830_rdcfgb(adbms6830_driver_t *dev)
{
	adbms6830_rd48(dev, RDCFGB, shared_buf);
	adbms6830_parse_cfgb(dev, shared_buf);
}

void adbms6830_pack_cfga(adbms6830_driver_t *dev)
{
	adbms6830_asic *ics = dev->ics;
	for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	{
		ics[curr_ic].configa.tx_data[0] = (((ics[curr_ic].tx_cfga.refon & 0x01) << 7) | (ics[curr_ic].tx_cfga.cth & 0x07));
		ics[curr_ic].configa.tx_data[1] = (ics[curr_ic].tx_cfga.flag_d & 0xFF);
		ics[curr_ic].configa.tx_data[2] = (((ics[curr_ic].tx_cfga.soakon & 0x01) << 7) | ((ics[curr_ic].tx_cfga.owrng & 0x01) << 6) | ((ics[curr_ic].tx_cfga.owa & 0x07) << 3));
		ics[curr_ic].configa.tx_data[3] = ((ics[curr_ic].tx_cfga.gpo & 0x00FF));
		ics[curr_ic].configa.tx_data[4] = ((ics[curr_ic].tx_cfga.gpo & 0x0300)>>8);
		ics[curr_ic].configa.tx_data[5] = (((ics[curr_ic].tx_cfga.snap & 0x01) << 5) | ((ics[curr_ic].tx_cfga.mute_st & 0x01) << 4) | ((ics[curr_ic].tx_cfga.comm_bk & 0x01) << 3) | (ics[curr_ic].tx_cfga.fc & 0x07));
	}
}

void adbms6830_pack_cfgb(adbms6830_driver_t *dev)
{
	adbms6830_asic *ics = dev->ics;
	for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	{
		ics[curr_ic].configb.tx_data[0] = ((ics[curr_ic].tx_cfgb.vuv ));
		ics[curr_ic].configb.tx_data[1] = (((ics[curr_ic].tx_cfgb.vov & 0x000F) << 4) | ((ics[curr_ic].tx_cfgb.vuv ) >> 8));
		ics[curr_ic].configb.tx_data[2] = ((ics[curr_ic].tx_cfgb.vov >>4)&0x0FF);
		ics[curr_ic].configb.tx_data[3] = (((ics[curr_ic].tx_cfgb.dtmen & 0x01) << 7) | ((ics[curr_ic].tx_cfgb.dtrng & 0x01) << 6) | ((ics[curr_ic].tx_cfgb.dcto & 0x3F) << 0));
		ics[curr_ic].configb.tx_data[4] = ((ics[curr_ic].tx_cfgb.dcc & 0xFF));
		ics[curr_ic].configb.tx_data[5] = ((ics[curr_ic].tx_cfgb.dcc >>8 ));
	}
}

void adbms6830_parse_cfga(adbms6830_driver_t* dev, uint8_t *data)
{
	adbms6830_asic *ic = dev->ics;
	  uint8_t address = 0;
	  for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	  {
	    memcpy(&ic[curr_ic].configa.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
	    address = ((curr_ic+1) * (RX_DATA));

	    ic[curr_ic].rx_cfga.cth = (ic[curr_ic].configa.rx_data[0] & 0x07);
	    ic[curr_ic].rx_cfga.refon   = (ic[curr_ic].configa.rx_data[0] & 0x80) >> 7;

	    ic[curr_ic].rx_cfga.flag_d  = (ic[curr_ic].configa.rx_data[1] & 0xFF);

	    ic[curr_ic].rx_cfga.soakon   = (ic[curr_ic].configa.rx_data[2] & 0x80) >> 7;
	    ic[curr_ic].rx_cfga.owrng    = (((ic[curr_ic].configa.rx_data[2] & 0x40) >> 6));
	    ic[curr_ic].rx_cfga.owa    = ( (ic[curr_ic].configa.rx_data[2] & 0x38) >> 3);

	    ic[curr_ic].rx_cfga.gpo        = ( (ic[curr_ic].configa.rx_data[3] & 0xFF)| ((ic[curr_ic].configa.rx_data[4] & 0x03) << 8) );

	    ic[curr_ic].rx_cfga.snap   = ((ic[curr_ic].configa.rx_data[5] & 0x20) >> 5);
	    ic[curr_ic].rx_cfga.mute_st   = ((ic[curr_ic].configa.rx_data[5] & 0x10) >> 4);
	    ic[curr_ic].rx_cfga.comm_bk   = ((ic[curr_ic].configa.rx_data[5] & 0x08) >> 3);
	    ic[curr_ic].rx_cfga.fc   = ((ic[curr_ic].configa.rx_data[5] & 0x07) >> 0);
	  }
}

void adbms6830_parse_cfgb(adbms6830_driver_t* dev, uint8_t *data)
{
	adbms6830_asic *ic = dev->ics;
	  uint8_t address = 0;
	  for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	  {
	    memcpy(&ic[curr_ic].configb.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
	    address = ((curr_ic+1) * (RX_DATA));

	    ic[curr_ic].rx_cfgb.vuv = ((ic[curr_ic].configb.rx_data[0])  | ((ic[curr_ic].configb.rx_data[1] & 0x0F) << 8));
	    ic[curr_ic].rx_cfgb.vov  = (ic[curr_ic].configb.rx_data[2]<<4)+((ic[curr_ic].configb.rx_data[1] &0xF0)>>4)  ;
	    ic[curr_ic].rx_cfgb.dtmen = (((ic[curr_ic].configb.rx_data[3] & 0x80) >> 7));
	    ic[curr_ic].rx_cfgb.dtrng= ((ic[curr_ic].configb.rx_data[3] & 0x40) >> 6);
	    ic[curr_ic].rx_cfgb.dcto   = ((ic[curr_ic].configb.rx_data[3] & 0x3F));
	    ic[curr_ic].rx_cfgb.dcc = ((ic[curr_ic].configb.rx_data[4]) | ((ic[curr_ic].configb.rx_data[5] & 0xFF) << 8));
	  }
}

void adbms6830_srst(adbms6830_driver_t *dev)
{
	adbms6830_cmd(dev, SRST);
}

void adbms6830_wakeup(adbms6830_driver_t* dev)
{
	for(uint8_t i = 0; i < dev->num_ics; i++)
	{
		adbms6830_set_cs(dev, 0);
		adbms6830_us_delay(dev, WAKEUP_US_DELAY);
		adbms6830_set_cs(dev, 1);
		adbms6830_us_delay(dev, WAKEUP_BW_DELAY);
	}
}

void adbms6830_pack_comm(adbms6830_driver_t* dev)
{
	adbms6830_asic *ics = dev->ics;
	for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	{
		ics[curr_ic].com.tx_data[0] = ((ics[curr_ic].comm.icomm[0] & 0x0F)  << 4  | (ics[curr_ic].comm.fcomm[0]   & 0x0F));
		ics[curr_ic].com.tx_data[1] = ((ics[curr_ic].comm.data[0] ));
		ics[curr_ic].com.tx_data[2] = ((ics[curr_ic].comm.icomm[1] & 0x0F)  << 4 ) | (ics[curr_ic].comm.fcomm[1]   & 0x0F);
		ics[curr_ic].com.tx_data[3] = ((ics[curr_ic].comm.data[1]));
		ics[curr_ic].com.tx_data[4] = ((ics[curr_ic].comm.icomm[2] & 0x0F)  << 4  | (ics[curr_ic].comm.fcomm[2]   & 0x0F));
		ics[curr_ic].com.tx_data[5] = ((ics[curr_ic].comm.data[2]));
	}
}

void adbms6830_pack_clr_flag_data(adbms6830_driver_t* dev)
{
	adbms6830_asic *ics = dev->ics;
	for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	{
		ics[curr_ic].clrflag.tx_data[0] = (ics[curr_ic].clflag.cl_csflt & 0x00FF);
		ics[curr_ic].clrflag.tx_data[1] = ((ics[curr_ic].clflag.cl_csflt & 0xFF00) >> 8);
		ics[curr_ic].clrflag.tx_data[2] = 0x00;
		ics[curr_ic].clrflag.tx_data[3] = 0x00;
		ics[curr_ic].clrflag.tx_data[4] = ((ics[curr_ic].clflag.cl_vaov << 7) | (ics[curr_ic].clflag.cl_vauv << 6) | (ics[curr_ic].clflag.cl_vdov << 5) | (ics[curr_ic].clflag.cl_vduv << 4)
			  																  |(ics[curr_ic].clflag.cl_ced << 3)   | (ics[curr_ic].clflag.cl_cmed << 2) | (ics[curr_ic].clflag.cl_sed << 1) | (ics[curr_ic].clflag.cl_smed));
		ics[curr_ic].clrflag.tx_data[5] = ((ics[curr_ic].clflag.cl_vde << 7)  | (ics[curr_ic].clflag.cl_vdel << 6) | (ics[curr_ic].clflag.cl_spiflt << 4) |(ics[curr_ic].clflag.cl_sleep << 3)
																			  | (ics[curr_ic].clflag.cl_thsd << 2) | (ics[curr_ic].clflag.cl_tmode << 1) | (ics[curr_ic].clflag.cl_oscchk));
	}
}

void adbms6830_spi_write(adbms6830_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs)
{
	if((dev == NULL) || (dev->hspi == NULL) || (data == NULL))
	{
		return;
	}

	if(use_cs) adbms6830_set_cs(dev, 0);
	HAL_SPI_Transmit(dev->hspi, data, len, SPI_TIMEOUT);
	if(use_cs) adbms6830_set_cs(dev, 1);
}


void adbms6830_cmd(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ])
{
	uint16_t pec15;

	write_buf[0] = cmd[0];
	write_buf[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	write_buf[2] = (uint8_t)(pec15 >> 8);
	write_buf[3] = (uint8_t)(pec15);
	adbms6830_spi_write(dev, write_buf, CMDSZ + PEC15SZ, 1);
}

// Tx/Rx Utility
void adbms6830_wr48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data)
{
    uint16_t pec15;
    uint16_t data_pec;
    // tx_sz = 4 bytes cmd/pec + (6 bytes data + 2 bytes pec) * num_ics
    uint16_t tx_sz = CMDSZ + PEC15SZ + ((TX_DATA + DPECSZ) * dev->num_ics);
    uint16_t cmd_index = 0;
    uint8_t src_addr = 0;
    uint8_t temp[TX_DATA];

    /* 1. Wakeup the isoSPI Daisy Chain */
    adbms6830_wakeup(dev);

    /* 2. Prepare Command + PEC15 */
    write_buf[0] = cmd[0];
    write_buf[1] = cmd[1];
    pec15 = Pec15_Calc(CMDSZ, cmd);
    write_buf[2] = (uint8_t)(pec15 >> 8);
    write_buf[3] = (uint8_t)pec15;
    cmd_index = 4;

    /* 3. Pack Data + PEC10 in reverse order for daisy chain */
    // The first configuration written is received by the last IC in the daisy chain
    for (uint8_t current_ic = dev->num_ics; current_ic > 0; current_ic--)
    {
        src_addr = ((current_ic - 1) * TX_DATA);

        // Copy the 6 bytes of data for this specific IC
        for (uint8_t current_byte = 0; current_byte < TX_DATA; current_byte++)
        {
            write_buf[cmd_index] = tx_data[src_addr + current_byte];
            temp[current_byte] = tx_data[src_addr + current_byte];
            cmd_index++;
        }

        // Calculate the PEC10 for these 6 bytes
        // Note: ADI's pec10_calc takes (bIsRxCmd, length, data). Write commands are false.
        data_pec = pec10_calc(0, TX_DATA, temp);

        // Append the 2 PEC bytes
        write_buf[cmd_index] = (uint8_t)(data_pec >> 8);
        cmd_index++;
        write_buf[cmd_index] = (uint8_t)data_pec;
        cmd_index++;
    }

    /* 4. Transmit over SPI */
    adbms6830_spi_write(dev, write_buf, tx_sz, 1);
}


void adbms6830_rd48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data)
{
    uint16_t pec15, received_pec, calculated_pec;
    uint16_t rx_sz = RX_DATA * dev->num_ics;
    uint8_t wrcmd[CMDSZ + PEC15SZ] = {0};

    /* 1. Prepare Command + PEC15 */
    wrcmd[0] = cmd[0];
    wrcmd[1] = cmd[1];
    pec15 = Pec15_Calc(CMDSZ, cmd);
    wrcmd[2] = (uint8_t)(pec15 >> 8);
    wrcmd[3] = (uint8_t)pec15;

    /* 2. Wakeup the isoSPI Daisy Chain
     * Crucial to ensure the ICs are ready to receive the SPI clock/data
     */
    adbms6830_wakeup(dev);

    /* 3. Send Command and Read Data
     * Using your existing custom SPI wrapper
     */
    adbms6830_spi_write_read(dev, wrcmd, CMDSZ + PEC15SZ, rx_data, rx_sz, 1);

    /* 4. Parse, Check PEC10, and Extract Command Counter */
    for (uint8_t current_ic = 0; current_ic < dev->num_ics; current_ic++)
    {
        // Calculate base index for this IC's data chunk
        uint16_t ic_base_idx = current_ic * RX_DATA;

        // Pointer to this specific IC's data for cleaner access
        uint8_t* ic_data = &rx_data[ic_base_idx];

        // Extract Command Counter (top 6 bits of the 7th byte)
        // dev->ics[current_ic].cmd_cntr = (ic_data[RX_DATA - 2] >> 2);
        cmd_cntr = (ic_data[RX_DATA - 2] >> 2);

        // Extract Received PEC10 (bottom 2 bits of 7th byte + all 8 bits of 8th byte)
        received_pec = (uint16_t)(((ic_data[RX_DATA - 2] & 0x03) << 8) | ic_data[RX_DATA - 1]);

        // Calculate Expected PEC10
        // The ADI pec10_calc expects `true` for Rx commands to include the command counter byte in the calculation
        calculated_pec = pec10_calc(1, RX_DATA - 2, ic_data);

        // Flag the error in the device struct for the application layer to handle
        if (received_pec != calculated_pec)
        {
            // dev->ics[current_ic].rx_pec_error = 1; // PEC Mismatch - Data is corrupt
        	rx_pec_error = 1;
        }
        else
        {
            // dev->ics[current_ic].rx_pec_error = 0; // Data is valid
        	rx_pec_error = 0;
        }
    }
}

// SPI communication
void adbms6830_set_cs(adbms6830_driver_t* dev, uint8_t state)
{
	if((dev == NULL) ||
	   (dev->string > STRING_B) ||
	   (dev->cs_port[dev->string] == NULL))
	{
		return;
	}

	HAL_GPIO_WritePin(dev->cs_port[dev->string], dev->cs_pin[dev->string], state);
}


void adbms6830_spi_write_read(adbms6830_driver_t *dev, uint8_t* tx_Data, uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len, uint8_t use_cs)
{
	if((dev == NULL) || (dev->hspi == NULL) || (tx_Data == NULL) || (rx_data == NULL))
	{
		return;
	}

	if(use_cs) adbms6830_set_cs(dev, 0);
	(void)HAL_SPI_Transmit(dev->hspi, tx_Data, tx_len, 100);
	(void)HAL_SPI_Receive(dev->hspi, rx_data, rx_len, 100);
	if(use_cs) adbms6830_set_cs(dev, 1);
}

void adbms6830_us_delay(adbms6830_driver_t* dev, uint16_t microseconds)
{
	if((dev == NULL) || (dev->htim == NULL))
	{
		return;
	}

	__HAL_TIM_SET_COUNTER(dev->htim, 0);
	while (__HAL_TIM_GET_COUNTER(dev->htim) < microseconds);
	return;
}

void adbms6830_adcv(adbms6830_driver_t *dev, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs)
{
    uint8_t cmd[2];
    cmd[0] = 0x02u | (uint8_t)rd;
    cmd[1] = ((uint8_t)cont << 7u) | ((uint8_t)dcp << 4u)
           | ((uint8_t)rstf << 2u) | ((uint8_t)owcs & 0x03u) | 0x60u;
    adbms6830_wakeup(dev);
    adbms6830_cmd(dev, cmd);
}

void adbms6830_start_adc_cell_voltage_measurement(adbms6830_driver_t *dev)
{
    adbms6830_adcv(dev, RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
}

//void adbms6830_parse_cell(adbms6830_driver_t *dev, uint8_t *data, GRP grp)
//{
//    for (uint8_t curr_ic = 0; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
//    {
//        uint8_t *d = &data[curr_ic * RX_DATA];
//
//        switch (grp)
//        {
//        case A:
//            dev->ics[curr_ic].cell.c_codes[0]  = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8u));
//            dev->ics[curr_ic].cell.c_codes[1]  = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8u));
//            dev->ics[curr_ic].cell.c_codes[2]  = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8u));
//            break;
//        case B:
//            dev->ics[curr_ic].cell.c_codes[3]  = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8u));
//            dev->ics[curr_ic].cell.c_codes[4]  = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8u));
//            dev->ics[curr_ic].cell.c_codes[5]  = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8u));
//            break;
//        case C:
//            dev->ics[curr_ic].cell.c_codes[6]  = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8u));
//            dev->ics[curr_ic].cell.c_codes[7]  = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8u));
//            dev->ics[curr_ic].cell.c_codes[8]  = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8u));
//            break;
//        case D:
//            dev->ics[curr_ic].cell.c_codes[9]  = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8u));
//            dev->ics[curr_ic].cell.c_codes[10] = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8u));
//            dev->ics[curr_ic].cell.c_codes[11] = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8u));
//            break;
//        case E:
//            dev->ics[curr_ic].cell.c_codes[12] = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8u));
//            dev->ics[curr_ic].cell.c_codes[13] = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8u));
//            dev->ics[curr_ic].cell.c_codes[14] = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8u));
//            break;
//        case F:
//            dev->ics[curr_ic].cell.c_codes[15] = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8u));
//            break;
//        default:
//            break;
//        }
//
//        uint16_t received_pec = (uint16_t)(((d[RX_DATA - 2u] & 0x03u) << 8u)
//                                           | d[RX_DATA - 1u]);
//        uint16_t calc_pec     = pec10_calc(1, RX_DATA - 2, d);
//
//        dev->ics[curr_ic].cccrc.cmd_cntr = d[RX_DATA - 2u] >> 2u;
//        dev->ics[curr_ic].cccrc.cell_pec = (received_pec != calc_pec) ? 1u : 0u;
//    }
//}

void adbms6830_parse_cell(adbms6830_driver_t *dev, uint8_t *data, GRP grp)
{
    #define IS_VALID_CODE(lo, hi) \
        (!((d[lo] == 0xFFu) && (d[hi] == 0xFFu)) && \
         !((d[lo] == 0x00u) && (d[hi] == 0x80u)))

    #define STORE_CODE(idx, byte_lo, byte_hi)                                       \
    do {                                                                            \
        if (IS_VALID_CODE(byte_lo, byte_hi)) {                                      \
            dev->ics[curr_ic].cell.c_codes[idx] =                                   \
                (int16_t)((uint16_t)d[byte_lo] | ((uint16_t)d[byte_hi] << 8u));    \
        }                                                                           \
    } while(0)

    for (uint8_t curr_ic = 0; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
    {
        uint8_t *d = &data[curr_ic * RX_DATA];

        uint16_t received_pec = (uint16_t)(((d[RX_DATA - 2u] & 0x03u) << 8u)
                                           | d[RX_DATA - 1u]);
        uint16_t calc_pec     = pec10_calc(1, RX_DATA - 2, d);
        dev->ics[curr_ic].cccrc.cmd_cntr = d[RX_DATA - 2u] >> 2u;
        dev->ics[curr_ic].cccrc.cell_pec = (received_pec != calc_pec) ? 1u : 0u;

        if (dev->ics[curr_ic].cccrc.cell_pec) continue;

        switch (grp)
        {
        case A:
            STORE_CODE(0,  0, 1);
            STORE_CODE(1,  2, 3);
            STORE_CODE(2,  4, 5);
            break;
        case B:
            STORE_CODE(3,  0, 1);
            STORE_CODE(4,  2, 3);
            STORE_CODE(5,  4, 5);
            break;
        case C:
            STORE_CODE(6,  0, 1);
            STORE_CODE(7,  2, 3);
            STORE_CODE(8,  4, 5);
            break;
        case D:
            STORE_CODE(9,  0, 1);
            STORE_CODE(10, 2, 3);
            STORE_CODE(11, 4, 5);
            break;
        case E:
            STORE_CODE(12, 0, 1);
            STORE_CODE(13, 2, 3);
            STORE_CODE(14, 4, 5);
            break;
        case F:
            STORE_CODE(15, 0, 1);
            break;
        default:
            break;
        }
    }

    #undef STORE_CODE
    #undef IS_VALID_CODE
}

void adbms6830_read_cell_voltages(adbms6830_driver_t *dev)
{
    uint8_t snap_cmd[2]   = { 0x00u, 0x2Du };
    uint8_t unsnap_cmd[2] = { 0x00u, 0x2Fu };

    adbms6830_wakeup(dev);
    adbms6830_cmd(dev, snap_cmd);
    adbms6830_us_delay(dev, 10u);

    adbms6830_rd48(dev, RDCVA, shared_buf); adbms6830_parse_cell(dev, shared_buf, A);
    adbms6830_rd48(dev, RDCVB, shared_buf); adbms6830_parse_cell(dev, shared_buf, B);
    adbms6830_rd48(dev, RDCVC, shared_buf); adbms6830_parse_cell(dev, shared_buf, C);
    adbms6830_rd48(dev, RDCVD, shared_buf); adbms6830_parse_cell(dev, shared_buf, D);
    adbms6830_rd48(dev, RDCVE, shared_buf); adbms6830_parse_cell(dev, shared_buf, E);
    adbms6830_rd48(dev, RDCVF, shared_buf); adbms6830_parse_cell(dev, shared_buf, F);

    adbms6830_wakeup(dev);
    adbms6830_cmd(dev, unsnap_cmd);
}


/* ---------------------------------------------------------------------------
 * GPIO AUX channel mapping
 *
 * The three ADG728 mux outputs connect to:
 *   U2 → GPIO1  (RDAUXA byte 0/1, a_codes[0])
 *   U3 → GPIO2  (RDAUXA byte 2/3, a_codes[1])
 *   U4 → GPIO3  (RDAUXA byte 4/5, a_codes[2])
 *
 * sensor_num  0–7  : U2, switch S1–S8, read from GPIO1 → a_codes[0]
 * sensor_num  8–15 : U3, switch S1–S8, read from GPIO2 → a_codes[1]
 * sensor_num 16–23 : U4, switch S1–S8, read from GPIO3 → a_codes[2]
 * ------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * adbms6830_gpio_i2c_write
 *
 * Issue a one-byte I2C write to a slave address via the ADBMS6830 COMM
 * register on every IC in the daisy-chain.
 * ------------------------------------------------------------------------- */
static void adbms6830_gpio_i2c_write(adbms6830_driver_t *dev,
                                      uint8_t slave_addr,
                                      uint8_t data_byte)
{
    /* Pack COMM register: 3 slots                                          *
     * Slot 0: START + address byte (write)                                 *
     * Slot 1: data byte                                                    *
     * Slot 2: STOP (data don't-care = 0xFF)                                */
    for (int curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
    {
        dev->ics[curr_ic].comm.icomm[0] = ICOMM_START_;
        dev->ics[curr_ic].comm.fcomm[0] = FCOMM_ACK_;
        dev->ics[curr_ic].comm.data[0]  = (uint8_t)((slave_addr << 1u) | 0x00u);

        dev->ics[curr_ic].comm.icomm[1] = ICOMM_BLANK_;
        dev->ics[curr_ic].comm.fcomm[1] = FCOMM_ACK_;
        dev->ics[curr_ic].comm.data[1]  = data_byte;

        dev->ics[curr_ic].comm.icomm[2] = ICOMM_STOP_;
        dev->ics[curr_ic].comm.fcomm[2] = FCOMM_NACK_STOP_;
        dev->ics[curr_ic].comm.data[2]  = 0xFFu;
    }

    /* Serialise comm struct → tx_data bytes */
    adbms6830_pack_comm(dev);

    /* Flatten into shared_buf for wr48 */
    for (int curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
    {
        for (uint8_t b = 0; b < TX_DATA; b++)
        {
            shared_buf[(curr_ic * TX_DATA) + b] = dev->ics[curr_ic].com.tx_data[b];
        }
    }

//    adbms6830_wakeup(dev);

    /* Write COMM register, then clock it out via STCOMM */
//    taskENTER_CRITICAL();
    adbms6830_wr48(dev, WRCOMM, shared_buf);

    /* STCOMM: pulse CS low and clock 72 bits to push the I2C transaction  */
    adbms6830_set_cs(dev, 0);
    HAL_SPI_Transmit(dev->hspi, STCOMM, sizeof(STCOMM), SPI_TIMEOUT);
    adbms6830_set_cs(dev, 1);
//    taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------
 * adbms6830_parse_aux_gpio
 *
 * Extract GPIO1/2/3 raw codes from an RDAUXA response buffer and store them
 * into ax_.a_codes[0..2] for each IC.
 * ------------------------------------------------------------------------- */
//static void adbms6830_parse_aux_gpio(adbms6830_driver_t *dev, uint8_t *data)
//{
//    for (uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
//    {
//        uint8_t *d = &data[curr_ic * RX_DATA];
//
//        /* GPIO1 = AUX channel 0, GPIO2 = channel 1, GPIO3 = channel 2     */
//        dev->ics[curr_ic].aux.a_codes[0] = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8u));
//        dev->ics[curr_ic].aux.a_codes[1] = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8u));
//        dev->ics[curr_ic].aux.a_codes[2] = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8u));
//
//        /* PEC + command counter */
//        uint16_t received_pec = (uint16_t)(((d[RX_DATA - 2u] & 0x03u) << 8u) | d[RX_DATA - 1u]);
//        uint16_t calc_pec     = pec10_calc(1, RX_DATA - 2, d);
//        dev->ics[curr_ic].cccrc.cmd_cntr = d[RX_DATA - 2u] >> 2u;
//        dev->ics[curr_ic].cccrc.aux_pec  = (received_pec != calc_pec) ? 1u : 0u;
//    }
//}

static void adbms6830_parse_aux_gpio(adbms6830_driver_t *dev, uint8_t *data)
{
    #define IS_VALID_CODE(lo, hi) \
        (!((d[lo] == 0xFFu) && (d[hi] == 0xFFu)) && \
         !((d[lo] == 0x00u) && (d[hi] == 0x80u)))

    #define STORE_AUX(idx, byte_lo, byte_hi)                                        \
    do {                                                                            \
        if (IS_VALID_CODE(byte_lo, byte_hi)) {                                      \
            dev->ics[curr_ic].aux.a_codes[idx] =                                    \
                (int16_t)((uint16_t)d[byte_lo] | ((uint16_t)d[byte_hi] << 8u));    \
        }                                                                           \
    } while(0)

    for (uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
    {
        uint8_t *d = &data[curr_ic * RX_DATA];

        uint16_t received_pec = (uint16_t)(((d[RX_DATA - 2u] & 0x03u) << 8u) | d[RX_DATA - 1u]);
        uint16_t calc_pec     = pec10_calc(1, RX_DATA - 2, d);
        dev->ics[curr_ic].cccrc.cmd_cntr = d[RX_DATA - 2u] >> 2u;
        dev->ics[curr_ic].cccrc.aux_pec  = (received_pec != calc_pec) ? 1u : 0u;

        if (dev->ics[curr_ic].cccrc.aux_pec) continue;

        STORE_AUX(0, 0, 1);
        STORE_AUX(1, 2, 3);
        STORE_AUX(2, 4, 5);
    }

    #undef STORE_AUX
    #undef IS_VALID_CODE
}

/* ---------------------------------------------------------------------------
 * adbms6830_read_temp_raw
 *
 * Select temperature sensor <sensor_num> (0–23) via its ADG728 mux, trigger
 * a single AUX conversion, read back the GPIO voltage, and store the raw
 * 16-bit ADC code into dev->ics[ic_idx].temp.raw[sensor_num].
 *
 * The ADG728 mapping is:
 *   sensor  0–7  → U2 (0x4C), output on GPIO1 → a_codes[0]
 *   sensor  8–15 → U3 (0x4D), output on GPIO2 → a_codes[1]
 *   sensor 16–23 → U4 (0x4E), output on GPIO3 → a_codes[2]
 *
 * @param ic_idx     Index of the target IC in dev->ics[]
 * @param sensor_num Temperature sensor index (0–23)
 * @param out_raw    Pointer to receive the raw ADC code (may be NULL)
 * @return  0 on success, -1 if arguments are out of range
 * ------------------------------------------------------------------------- */
int adbms6830_read_temp_raw(adbms6830_driver_t *dev,
                             uint8_t ic_idx,
                             int16_t *out_raw)
{
    /* Validate — caller must pass the sensor_num separately; we derive the
     * mux address and GPIO channel from it.  The sensor_num is tracked
     * outside this function (e.g. a loop over 0–23).  Here we just perform
     * the measurement for whichever sensor is currently selected.
     * This signature matches the existing call-site in adBms6830_init:
     *   adbms6830_read_temp_raw(dev, 0, &dev->ics[0].temp.raw[0]);
     * so ic_idx is the IC and out_raw points into temp.raw[sensor_num].    */

    if ((dev == NULL) || (dev->ics == NULL) || (dev->num_ics <= 0) ||
        (ic_idx >= (uint8_t)dev->num_ics) || (out_raw == NULL))
    {
        return -1;
    }

    /* 1. Trigger a single AUX conversion on all GPIO channels              */
    uint8_t adax_cmd[2] = { ADAX_CMD_BYTE0, ADAX_CMD_BYTE1 };
    adbms6830_wakeup(dev);


    adbms6830_cmd(dev, adax_cmd);

    /* Wait for conversion to complete (~1 ms for 7kHz mode)                */
    adbms6830_us_delay(dev, 1200u);

    /* 2. Read RDAUXA — contains GPIO1, GPIO2, GPIO3                        */
    adbms6830_rd48(dev, RDAUXA, shared_buf);
    adbms6830_parse_aux_gpio(dev, shared_buf);

    /* 3. The caller supplies out_raw pointing at temp.raw[sensor_num].
     * Store the GPIO voltage that corresponds to the active mux output.
     * Since only one ADG728 switch is closed at a time, the relevant
     * a_codes slot is determined by which mux device is selected.
     * We read all three here and let the caller pick the right slot via
     * out_raw; however, to keep things self-contained we also write the
     * value through out_raw using the same gpio_ch logic.                  */

    /* Derive gpio_ch from the position of out_raw within temp.raw[]        */
    ptrdiff_t sensor_num = out_raw - dev->ics[ic_idx].temp.raw;
    if (sensor_num < 0 || sensor_num >= 24)
    {
        /* Fall back: store GPIO1 reading */
        *out_raw = dev->ics[ic_idx].aux.a_codes[0];
        return 0;
    }

    uint8_t gpio_ch = (uint8_t)(sensor_num / SENSORS_PER_MUX); /* 0, 1, or 2 */
    *out_raw = dev->ics[ic_idx].aux.a_codes[gpio_ch];

    return 0;
}

/* ---------------------------------------------------------------------------
 * mux_read_gpio_voltage
 *
 * Full cycle for a single temperature sensor:
 *   1. Select <sensor_num> on the appropriate ADG728 via I2C COMM.
 *   2. Trigger an AUX ADC conversion.
 *   3. Read back the GPIO voltage (RDAUXA).
 *   4. Store the raw code in dev->ics[ic_idx].temp.raw[sensor_num].
 *
 * All ICs in the daisy-chain receive the same mux command (broadcast), and
 * the raw result is stored per-IC.
 *
 * @param ic_idx     IC to store the result for (0 … num_ics-1)
 * @param sensor_num Sensor index 0–23
 * @return  0 on success, -1 if arguments are out of range
 * ------------------------------------------------------------------------- */

//int mux_read_gpio_voltage(adbms6830_driver_t *dev, uint8_t sensor_num)
//{
//    if (sensor_num >= 24u)
//    {
//        return -1;
//    }
//
////    uint8_t mux_idx    = sensor_num / SENSORS_PER_MUX;
////    uint8_t sw_pos     = sensor_num % SENSORS_PER_MUX;
////    uint8_t sw_mask    = (uint8_t)(1u << sw_pos);
////    uint8_t slave_addr = MUX_ADDRS[mux_idx];
////    uint8_t gpio_ch    = GPIO_AUX_IDX[mux_idx];
////
////    uint8_t sensor_num_n = (sensor_num + 1) % 24;
////    uint8_t mux_idx_n    = sensor_num_n / SENSORS_PER_MUX;
////	uint8_t sw_pos_n     = sensor_num_n % SENSORS_PER_MUX;
////	uint8_t sw_mask_n    = (uint8_t)(1u << sw_pos);
////	uint8_t slave_addr_n = MUX_ADDRS[mux_idx_n];
////
////    uint8_t adax_cmd[2] = { ADAX_CMD_BYTE0, ADAX_CH[mux_idx] };
//////    adbms6830_wakeup(dev);
////
////    adbms6830_cmd(dev, adax_cmd);
////    adbms6830_us_delay(dev, 4000u);
////
////    adbms6830_rd48(dev, RDAUXA, shared_buf);
////    adbms6830_parse_aux_gpio(dev, shared_buf);
////
////    /* Store result for every IC in the chain */
////    for (uint8_t ic = 0; ic < dev->num_ics; ic++)
////    {
////        dev->ics[ic].temp.raw[sensor_num] = dev->ics[ic].aux.a_codes[gpio_ch];
////    }
////
////    adbms6830_gpio_i2c_write(dev, slave_addr_n, sw_mask_n);
////	adbms6830_us_delay(dev, 2000u);
////
////
////    return 0;
//
//    /* Current sensor MUX config */
//    uint8_t mux_idx    = sensor_num / SENSORS_PER_MUX;
//    uint8_t sw_pos     = sensor_num % SENSORS_PER_MUX;
//    uint8_t sw_mask    = (uint8_t)(1u << sw_pos);
//    uint8_t slave_addr = MUX_ADDRS[mux_idx];
//    uint8_t gpio_ch    = GPIO_AUX_IDX[mux_idx];
//
//    /* Next sensor MUX config — written after the read to pre-stage for timing */
//    uint8_t sensor_num_n = (sensor_num + 1u) % 24u;
//    uint8_t mux_idx_n    = sensor_num_n / SENSORS_PER_MUX;
//    uint8_t sw_pos_n     = sensor_num_n % SENSORS_PER_MUX;
//    uint8_t sw_mask_n    = (uint8_t)(1u << sw_pos_n);   // BUG FIX: was sw_pos, not sw_pos_n
//    uint8_t slave_addr_n = MUX_ADDRS[mux_idx_n];
//
//    /* Trigger ADC conversion on the current GPIO channel */
//    uint8_t adax_cmd[2] = { ADAX_CMD_BYTE0, ADAX_CH[mux_idx] };
//    adbms6830_wakeup(dev);
//	adbms6830_cmd(dev, adax_cmd);
//    adbms6830_us_delay(dev, 4000u);
//
//    /* Read back and parse the auxiliary GPIO result */
//    adbms6830_rd48(dev, RDAUXA, shared_buf);
//    adbms6830_parse_aux_gpio(dev, shared_buf);
//
//    /* Store raw ADC result for every IC in the chain */
//    for (uint8_t ic = 0; ic < dev->num_ics; ic++)
//    {
//        dev->ics[ic].temp.raw[sensor_num] = dev->ics[ic].aux.a_codes[gpio_ch];
//    }
//
//    /* Pre-stage the next MUX channel over I2C so it's settled before the next read */
//    adbms6830_gpio_i2c_write(dev, slave_addr_n, sw_mask_n);
//
//    return 0;
//}

int mux_set_channel(adbms6830_driver_t *dev, uint8_t sensor_num)
{
    if ((dev == NULL) || (dev->ics == NULL) || (dev->num_ics <= 0) || (sensor_num >= 24u))
    {
        return -1;
    }

    uint8_t mux_idx    = sensor_num / SENSORS_PER_MUX;
    uint8_t sw_pos     = sensor_num % SENSORS_PER_MUX;
    uint8_t sw_mask    = (uint8_t)(1u << sw_pos);
    uint8_t slave_addr = MUX_ADDRS[mux_idx];

    adbms6830_gpio_i2c_write(dev, slave_addr, sw_mask);

    return 0;
}


int mux_read_gpio_voltage(adbms6830_driver_t *dev, uint8_t sensor_num)
{
    if ((dev == NULL) || (dev->ics == NULL) || (dev->num_ics <= 0) || (sensor_num >= 24u))
    {
        return -1;
    }

    uint8_t mux_idx = sensor_num / SENSORS_PER_MUX;
    uint8_t gpio_ch = GPIO_AUX_IDX[mux_idx];

    /* Trigger ADC conversion on the current GPIO channel */
    uint8_t adax_cmd[2] = { ADAX_CMD_BYTE0, ADAX_CH[mux_idx] };
    adbms6830_cmd(dev, adax_cmd);
    adbms6830_us_delay(dev, 4000u);

    /* Read back and parse the auxiliary GPIO result */
    adbms6830_rd48(dev, RDAUXA, shared_buf);
    adbms6830_parse_aux_gpio(dev, shared_buf);

    /* Store raw ADC result for every IC in the chain */
    uint8_t ic_count = (dev->num_ics > 255) ? 255u : (uint8_t)dev->num_ics;
    for (uint8_t ic = 0; ic < ic_count; ic++)
    {
        dev->ics[ic].temp.raw[sensor_num] = dev->ics[ic].aux.a_codes[gpio_ch];
    }

    return 0;
}


/* ---------------------------------------------------------------------------
 * adbms6830_convert_temp
 *
 * Convert a raw GPIO ADC code to a voltage in volts.
 *
 * The ADBMS6830 AUX ADC uses the same 16-bit LSB resolution as cell voltage:
 *   V = raw_code * 150e-6  (150 µV per LSB)
 *
 * To convert voltage to temperature, apply your NTC/PTC transfer function
 * outside this function using the returned voltage and the known pull-up/
 * pull-down resistor values.
 *
 * @param dev        Driver handle
 * @param ic_idx     IC index
 * @param sensor_num Sensor index 0–23 (selects temp.raw[sensor_num])
 * @param vref       Reference / supply voltage used for the divider (V)
 *                   (not used in the raw-to-voltage step, kept for caller
 *                    convenience so signature matches the existing call-site)
 * @return Measured voltage at the GPIO pin in volts, or -1.0f on error.
 * ------------------------------------------------------------------------- */
float adbms6830_convert_temp(adbms6830_driver_t *dev,
                              uint8_t ic_idx,
                              uint8_t sensor_num,
                              float vref)
{
    (void)vref;   /* reserved for NTC conversion in the application layer */

    if ((dev == NULL) || (dev->ics == NULL) || (dev->num_ics <= 0) ||
        (ic_idx >= (uint8_t)dev->num_ics) || (sensor_num >= 24u))
    {
        return -1.0f;
    }

    int16_t raw = dev->ics[ic_idx].temp.raw[sensor_num];

    /* 150 µV per LSB (same scale as cell voltage registers) */
    return (float)raw * 150.0e-6f;


}

float voltage_to_temp(float v)
{
    float V = (10000.0f + v) * 0.00015f;

    if((V <= 0.0f) || (V >= 5.0f))
    {
        return -273.15f;
    }

    float R = 10000.0f * (5.0f - V) / V;
    if(R <= 0.0f)
    {
        return -273.15f;
    }

    float x = logf(R / 10000.0f);
    float T = 1.0f / (3.354016435e-3f + 2.565235509e-4f*x) - 273.15f;
    return T;
}
