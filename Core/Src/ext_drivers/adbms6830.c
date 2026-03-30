/*
 * adbms6830.c
 *
 *  Created on: May 13, 2025
 *      Author: realb
 */

#include <string.h>

#include "ext_drivers/adbms6830_functions.h"

unsigned char shared_buf[BUFSZ] = {0};
uint8_t write_buf[BUFSZ] = {0};

#define RDCVALL_CMD         {0x00, 0x0C}

#define RDCVALL_DATA_SZ     32   /* 16 cells x 2 bytes */

#define RDCVALL_RX_DATA     34   /* 32 data bytes + 2 PEC/counter bytes */

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
	// DELAY

	adbms6830_reset_cfg(dev);

//    float c1_volt; //voltage in Volts
//    float c2_volt; //voltage in Volts
//    float c3_volt; //voltage in Volts
//    start:
	debug_loop:
	// 1. WAKEUP AND WRITE CONFIGURATION
	// You MUST do this so dev->ics[i].tx_cfga.refon = PWR_UP is sent to the IC.
	adbms6830_wakeup(dev);
	adbms6830_wrcfga(dev);
	adbms6830_wrcfgb(dev);

	// Wait ~3ms for the precision voltage reference to warm up and settle
	adbms6830_us_delay(dev, 3000);

	// 2. START ADC CONVERSION (Only Once!)
	adbms6830_wakeup(dev);
	uint8_t cmd[2];
	cmd[0] = 0x02 + 1; // RD_ON = 1
	cmd[1] = (1<<7)+(0<<4)+(0<<2)+(0 & 0x03) + 0x60; // CONTINUOUS mode
	adbms6830_cmd(dev, cmd);

	// 3. WAIT FOR THE FIRST CONVERSION CYCLE TO FINISH
	// Wait at least 3 to 5 milliseconds to be safe.
	adbms6830_us_delay(dev, 5000);

	// 4. SNAP AND READ
	// Wakeup again just in case the isoSPI bus idled out during the 5ms delay
	adbms6830_wakeup(dev);

	// Send SNAP
	cmd[0] = 0x00;
	cmd[1] = 0x2D;
	adbms6830_cmd(dev, cmd);
	adbms6830_us_delay(dev, 10); // 10us is fine for SNAP

//	// Read Register A
//	adbms6830_rd48(dev, RDCVA, shared_buf);
//	for (uint8_t ic = 0; ic < dev->num_ics; ic++)
//	{
//	    uint8_t *d = &shared_buf[ic * RX_DATA];
//
//	    uint16_t c1 = (d[1] << 8) | d[0];
//	    uint16_t c2 = (d[3] << 8) | d[2];
//	    uint16_t c3 = (d[5] << 8) | d[4];
//
//	    // Example: store or print
//	    dev->ics[ic].cell.c_codes[0] = c1;
//	    dev->ics[ic].cell.c_codes[1] = c2;
//	    dev->ics[ic].cell.c_codes[2] = c3;
//
//	    // float c1_volt; //voltage in Volts
//	    c1_volt = ((c1 + 10000) * 0.000150);
//	    // float c2_volt; //voltage in Volts
//	    c2_volt = ((c2 + 10000) * 0.000150);
//	    // float c3_volt; //voltage in Volts
//	    c3_volt = ((c3 + 10000) * 0.000150);
//	}
//	goto start;


	// Array of commands for Registers A through F
//	uint8_t* cell_cmds[6] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};
//
//	// Registers A-E hold 3 cells each. Register F holds 1 cell (Cell 16).
//	uint8_t cells_in_reg[6] = {3, 3, 3, 3, 3, 1};
//
//	// Array to hold the converted float voltages locally (optional)
	float cell_volts[16];
//
//	for (uint8_t reg = 0; reg < 6; reg++)
//	{
//	    // 1. Read the current register (A, B, C, D, E, or F)
//	    adbms6830_rd48(dev, cell_cmds[reg], shared_buf);
//
//	    // 2. Parse the data for each IC in the daisy chain
//	    for (uint8_t ic = 0; ic < dev->num_ics; ic++)
//	    {
//	        uint8_t *d = &shared_buf[ic * RX_DATA];
//
//	        // Calculate the starting index for this register (0, 3, 6, 9, 12, or 15)
//	        uint8_t base_idx = reg * 3;
//
//	        for (uint8_t i = 0; i < cells_in_reg[reg]; i++)
//	        {
//	            // Parse the 16-bit raw code (Little Endian: Low byte first, then High byte)
//	            uint16_t raw_code = (d[(i * 2) + 1] << 8) | d[i * 2];
//
//	            // Calculate the absolute cell index (0 to 15)
//	            uint8_t cell_idx = base_idx + i;
//
//	            // Store the raw code in your struct
//	            dev->ics[ic].cell.c_codes[cell_idx] = raw_code;
//
//	            // Calculate the voltage in Volts
//	            cell_volts[cell_idx] = ((raw_code + 10000) * 0.000150f);
//
//	            /* * Tip: If you need to access these float voltages elsewhere in your program,
//	             * you should add a `float volts[16];` array to your `adbms6830_asic` struct
//	             * and save it there:
//	             * dev->ics[ic].cell.volts[cell_idx] = cell_volts[cell_idx];
//	             */
//	        }
//	    }
//	}

	adbms6830_rd_cvall(dev, shared_buf);

	// Parse the data for each IC in the daisy chain
	for (uint8_t ic = 0; ic < dev->num_ics; ic++)
	{
	    uint8_t *d = &shared_buf[ic * RDCVALL_RX_DATA];

	    for (uint8_t cell_idx = 0; cell_idx < 16; cell_idx++)
	    {
	        // Parse the 16-bit raw code (Little Endian: Low byte first, then High byte)
	        uint16_t raw_code = (d[(cell_idx * 2) + 1] << 8) | d[cell_idx * 2];

	        // Store the raw code in your struct
	        dev->ics[ic].cell.c_codes[cell_idx] = raw_code;

	        // Calculate the voltage in Volts
	        cell_volts[cell_idx] = ((raw_code + 10000) * 0.000150f);
	    }
	}

	adbms6830_wakeup(dev);
//	goto debug_loop; // TODO: Remove if not doing debug

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
	HAL_GPIO_WritePin(dev->cs_port[dev->string], dev->cs_pin[dev->string], state);
}


void adbms6830_spi_write_read(adbms6830_driver_t *dev, uint8_t* tx_Data, uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len, uint8_t use_cs)
{
	HAL_StatusTypeDef ret = 0;
	if(use_cs) adbms6830_set_cs(dev, 0);
	ret |= HAL_SPI_Transmit(dev->hspi, tx_Data, tx_len, 100);
	ret |= HAL_SPI_Receive(dev->hspi, rx_data, rx_len, 100);
	if(use_cs) adbms6830_set_cs(dev, 1);
}

void adbms6830_us_delay(adbms6830_driver_t* dev, uint16_t microseconds)
{
	__HAL_TIM_SET_COUNTER(dev->htim, 0);
	while (__HAL_TIM_GET_COUNTER(dev->htim) < microseconds);
	return;
}

/* RDCVALL: Read All Cell Voltage Registers (Groups A-F)

* CMD: CC[10:0] = 00000001100 -> 0x00, 0x0C

* Returns 32 data bytes per IC (C1-C16, 2 bytes each)

* + 2 bytes command counter/PEC10 = 34 bytes per IC total

*

* WARNING (per datasheet): RDCVALL is intended for single IC

* applications only. In a daisy chain, a single PEC covers all

* ICs' data — you cannot isolate which IC has corrupt data on

* a PEC failure.

*/



/* WARNING: On a PEC failure for a given IC, you cannot determine
 * which register group (A-F) contains the corrupt data — the
 * entire 32-byte payload for that IC must be discarded and re-read.
 * Use individual RDCVx commands if per-group error isolation is needed.
 * For coherent data across multiple ICs, use SNAP/UNSNAP before reading.
 */
void adbms6830_rd_cvall(adbms6830_driver_t* dev, uint8_t* rx_data)

{

    uint16_t pec15, received_pec, calculated_pec;

    uint16_t rx_sz = RDCVALL_RX_DATA * dev->num_ics;

    uint8_t cmd[CMDSZ] = RDCVALL_CMD;

    uint8_t wrcmd[CMDSZ + PEC15SZ] = {0};

    /* 1. Prepare Command + PEC15 */

    wrcmd[0] = cmd[0];

    wrcmd[1] = cmd[1];

    pec15 = Pec15_Calc(CMDSZ, cmd);

    wrcmd[2] = (uint8_t)(pec15 >> 8);

    wrcmd[3] = (uint8_t)pec15;

    /* 2. Wakeup the isoSPI Daisy Chain */

    adbms6830_wakeup(dev);

    /* 3. Send Command and Read Data */

    adbms6830_spi_write_read(dev, wrcmd, CMDSZ + PEC15SZ, rx_data, rx_sz, 1);

    /* 4. Parse, Check PEC10, and Extract Command Counter

     * Note: in a daisy chain, a PEC mismatch here cannot tell

     * you which IC's data is corrupt — only that something in

     * the full response is bad.

     */

    for (uint8_t current_ic = 0; current_ic < dev->num_ics; current_ic++)

    {

        uint16_t ic_base_idx = current_ic * RDCVALL_RX_DATA;

        uint8_t* ic_data = &rx_data[ic_base_idx];

        cmd_cntr = (ic_data[RDCVALL_RX_DATA - 2] >> 2);

        received_pec = (uint16_t)(((ic_data[RDCVALL_RX_DATA - 2] & 0x03) << 8)

                                 | ic_data[RDCVALL_RX_DATA - 1]);

        calculated_pec = pec10_calc(1, RDCVALL_RX_DATA - 2, ic_data);

        rx_pec_error = (received_pec != calculated_pec) ? 1 : 0;

    }

}

