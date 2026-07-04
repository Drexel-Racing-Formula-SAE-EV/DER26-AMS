/*
 * adbms2950.c
 *
 *  Created on: May 13, 2025
 *      Author: cole
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#include "ext_drivers/adbms2950.h"
#include "ext_drivers/adbms_shared.h"
#include <stddef.h>
#include <string.h>

uint8_t buf[BUFSZ] = {0};
uint8_t wrbuf[BUFSZ] = {0};
static uint8_t adbms2950_spi_txrx_tx_buf[BUFSZ] = {0};
static uint8_t adbms2950_spi_txrx_rx_buf[BUFSZ] = {0};

#define ADBMS2950_SPI_DUMMY_BYTE 0xFFu

/*!< configuration registers commands */
uint8_t WRCFGA[2]        = { 0x00, 0x01 };
uint8_t WRCFGB[2]        = { 0x00, 0x24 };
uint8_t RDCFGA[2]        = { 0x00, 0x02 };
uint8_t RDCFGB[2]        = { 0x00, 0x26 };

/* Read VBxADC and IxADC result registers commands */
uint8_t RDI[2]           = { 0x00, 0x04 };
uint8_t RDVB[2]          = { 0x00, 0x06 };
uint8_t RDIVB1[2]        = { 0x00, 0x08 };
uint8_t RDIACC[2]        = { 0x00, 0x44 };
uint8_t RDVBACC[2]       = { 0x00, 0x46 };
uint8_t RDIVB1ACC[2]      = { 0x00, 0x48 };

/* Read OCxADC result registers commands */
uint8_t RDOC[2]         = { 0x00, 0x0B };

/* Read VxADC result registers commands */
uint8_t RDV1A[2]         = { 0x00, 0x0A };
uint8_t RDV1B[2]         = { 0x00, 0x09 };
uint8_t RDV1C[2]         = { 0x00, 0x03 };
uint8_t RDV1D[2]         = { 0x00, 0x1B };
uint8_t RDV2A[2]         = { 0x00, 0x07 };//RDRVA
uint8_t RDV2B[2]         = { 0x00, 0x0D };
uint8_t RDV2C[2]         = { 0x00, 0x05 };
uint8_t RDV2D[2]         = { 0x00, 0x1F };
uint8_t RDV2E[2]         = { 0x00, 0x25 };

/* Read Status register */
uint8_t RDSTAT[2]       = { 0x00, 0x34 };

/* Read Flag register */
uint8_t RDFLAG[2]       = { 0x00, 0x32 };
uint8_t RDFLAGERR[2]    = { 0x00, 0x72 };   /* ERR */

/* Read AUX ADC result registers */
uint8_t RDXA[2]       = { 0x00, 0x30 };
uint8_t RDXB[2]       = { 0x00, 0x31 };
uint8_t RDXC[2]       = { 0x00, 0x33 };

/* Read all commands */
//------Read All IxADC and VBxADC results+Status+Flag-------
uint8_t RDALLI[2]        = { 0x00, 0x0C };

//------Read All IxACC and VBxACC results+Status+Flag-------
uint8_t RDALLA[2]        = { 0x00, 0x4C };

//------Read All configuration registers+Status+Flag-------
uint8_t RDALLC[2]        = { 0x00, 0x10 };

//------Read All Voltages-------
uint8_t RDALLV[2]        = { 0x00, 0x35 };

//------Read All Redundant Voltages-------
uint8_t RDALLR[2]        = { 0x00, 0x11 };

//------Read All Aux Voltages-------
uint8_t RDALLX[2]        = { 0x00, 0x51 };

/* Pwm registers commands */
uint8_t WRPWMA[2]         = { 0x00, 0x20 };
uint8_t RDPWMA[2]         = { 0x00, 0x22 };
uint8_t WRPWMB[2]         = { 0x00, 0x21 };
uint8_t RDPWMB[2]         = { 0x00, 0x23 };

/* Clear commands */
//uint8_t CLRAB[2]         = { 0x07, 0x11 };
uint8_t CLRI[2]         = { 0x07, 0x11 };
uint8_t CLRA[2]         = { 0x07, 0x14 };
uint8_t CLRO[2]         = { 0x07, 0x13 };
uint8_t CLRC[2]          = { 0x07, 0x16 };//Ask about CLRC//Sayani
uint8_t CLRVX [2]       = { 0x07, 0x12 };
//uint8_t CLRSTAT [2]      = { 0x07, 0x13 };
uint8_t CLRFLAG[2]       = { 0x07, 0x17 };

/*!< Poll adc command */
uint8_t PLADC[2]         = { 0x07, 0x18 };
uint8_t PLI1[2]       = { 0x07, 0x1C };
uint8_t PLI2[2]       = { 0x07, 0x1D };
uint8_t PLV[2]        = { 0x07, 0x1E };
uint8_t PLX[2]         = { 0x07, 0x1F };

/*!< GPIOs Comm commands */
uint8_t WRCOMM[2]        = { 0x07, 0x21 };
uint8_t RDCOMM[2]        = { 0x07, 0x22 };
/*!< command + dummy data for 72 clock cycles */
uint8_t STCOMM[13]       = { 0x07, 0x23, 0xB9, 0xE4 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};

/*!< Control Commands */
uint8_t RDSID[2]         = { 0x00, 0x2C };
uint8_t RSTCC[2]         = { 0x00, 0x2E };
uint8_t SNAP[2]          = { 0x00, 0x2D };
uint8_t UNSNAP[2]        = { 0x00, 0x2F };
uint8_t SRST[2]          = { 0x00, 0x27 };
uint8_t sADI1[2]         = { 0x02, 0x60 };
uint8_t sADI2[2]         = { 0x01, 0x68 };
uint8_t sADV[2]          = { 0x04, 0x30 };
uint8_t sADX[2]          = { 0x05, 0x30 };

//Command + pec
uint8_t RSTATD[4]        = {0x00, 0x33, 0x4D, 0x4A};  //Command +Pec
//uint8_t RSTATC[4]        = {0x00, 0x32, 0xc6, 0x78}; // Tiger CC is in Status C
uint8_t RFLAG[4]        = {0x00, 0x32, 0xc6, 0x78}; // Tiger CC is in Flag//Check PEC code//Sayani//Put breakpoint at Pec15_Calc for RDFLAG to check
uint8_t sRDI[4]          = {0x00, 0x04, 0x07, 0xC2};
uint8_t sCLRAB[4]        = {0x07,0x11,0xC9,0xC0};
//uint8_t sRSTATA [4]      = { 0x00, 0x30, 0x5B, 0x2E };
uint8_t sRDVA [4]        = { 0x00, 0x0A ,  0xC3 , 0x04};
uint8_t sRDVB [4]        = { 0x00, 0x09 , 0xD5 , 0x60};
uint8_t sRDVC [4]        = { 0x00, 0x03 , 0xA0, 0x38};
uint8_t sRDVD [4]        = { 0x00, 0x05 , 0x8C, 0xF0};
uint8_t sRDIAV[4]        = { 0x00, 0x44 , 0xE0, 0x48};
uint8_t sRDVBAT[4]       = { 0x00, 0x06 , 0x9A, 0x94};
/* Testmode and debugging commands */
uint8_t TM_48[2]       = { 0x00, 0x0E };        // LION: RDSVE

// Tx/Rx Utility
void adbms2950_cmd(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ]);
void adbms2950_wr48(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data);
void adbms2950_rd48(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data);
static HAL_StatusTypeDef adbms2950_cmd_checked(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ]);
static HAL_StatusTypeDef adbms2950_wr48_checked(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data);
static HAL_StatusTypeDef adbms2950_rd48_checked(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data);

// SPI communication
void adbms2950_set_cs(adbms2950_driver_t* dev, uint8_t state);
HAL_StatusTypeDef adbms2950_spi_write(adbms2950_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs);
HAL_StatusTypeDef adbms2950_spi_write_read(adbms2950_driver_t *dev, uint8_t* tx_Data, uint16_t tx_len, uint8_t* rx_data, uint16_t rx_len, uint8_t use_cs);
static void adbms2950_spi_debug_note_tx(adbms2950_driver_t *dev, adbms2950_spi_op_t op, const uint8_t *cmd, const uint8_t *tx, uint16_t tx_len, uint16_t rx_len);
static void adbms2950_spi_debug_note_rx(adbms2950_driver_t *dev, const uint8_t *rx, uint16_t rx_len, HAL_StatusTypeDef status);

// Data parsing
void adbms2950_parse_cfga(adbms2950_driver_t* dev, uint8_t* data);
void adbms2950_parse_cfgb(adbms2950_driver_t* dev, uint8_t* data);
void adbms2950_parse_rdvb(adbms2950_driver_t* dev, uint8_t* vbat_data);
void adbms2950_parse_rdi(adbms2950_driver_t* dev, uint8_t* i_data);
void adbms2950_parse_rdv1d(adbms2950_driver_t* dev, uint8_t* v_data);

// Data packing
void adbms2950_pack_cfga(adbms2950_driver_t* dev);
void adbms2950_pack_cfgb(adbms2950_driver_t* dev);

// Data validation
uint16_t Pec15_Calc(uint8_t len, uint8_t *data);
uint16_t pec10_calc(uint8_t rx_cmd, int len, uint8_t *data);
uint16_t pec10_calc_modular(uint8_t * data, uint8_t PEC_Format);

uint16_t pec10_calc_int(uint16_t remainder, uint8_t bit);

static uint16_t adbms2950_min_u16(uint16_t a, uint16_t b)
{
	return (a < b) ? a : b;
}

void adbms2950_spi_debug_enable(adbms2950_driver_t *dev, bool enable)
{
	if(dev == NULL)
	{
		return;
	}

	dev->spi_debug.enabled = enable;
}

void adbms2950_spi_debug_clear(adbms2950_driver_t *dev)
{
	bool was_enabled;

	if(dev == NULL)
	{
		return;
	}

	was_enabled = dev->spi_debug.enabled;
	memset(&dev->spi_debug, 0, sizeof(dev->spi_debug));
	dev->spi_debug.enabled = was_enabled;
	dev->spi_debug.last_status = HAL_OK;
	dev->spi_debug.last_tx_status = HAL_OK;
	dev->spi_debug.last_rx_status = HAL_OK;
	dev->spi_debug.last_xfer_status = HAL_OK;
}

const adbms2950_spi_debug_t *adbms2950_spi_debug_get(const adbms2950_driver_t *dev)
{
	return (dev == NULL) ? NULL : &dev->spi_debug;
}

const char *adbms2950_spi_op_str(adbms2950_spi_op_t op)
{
	switch(op)
	{
	case ADBMS2950_SPI_OP_NONE:  return "none";
	case ADBMS2950_SPI_OP_CMD:   return "cmd";
	case ADBMS2950_SPI_OP_WR48:  return "wr48";
	case ADBMS2950_SPI_OP_RD48:  return "rd48";
	case ADBMS2950_SPI_OP_PROBE: return "probe";
	default:                     return "unknown";
	}
}

static void adbms2950_spi_debug_note_tx(adbms2950_driver_t *dev,
										adbms2950_spi_op_t op,
										const uint8_t *cmd,
										const uint8_t *tx,
										uint16_t tx_len,
										uint16_t rx_len)
{
	uint16_t preview_len;

	if((dev == NULL) || (!dev->spi_debug.enabled))
	{
		return;
	}

	dev->spi_debug.last_op = op;
	dev->spi_debug.last_string = dev->string;
	dev->spi_debug.last_tx_len = tx_len;
	dev->spi_debug.last_rx_len = rx_len;
	dev->spi_debug.last_total_len = (uint16_t)(tx_len + rx_len);
	dev->spi_debug.last_read_pec_pass_mask = 0u;
	dev->spi_debug.last_read_pec_fail_mask = 0u;
	memset(dev->spi_debug.last_cmd_counter, 0, sizeof(dev->spi_debug.last_cmd_counter));
	memset(dev->spi_debug.last_tx_preview, 0, sizeof(dev->spi_debug.last_tx_preview));
	memset(dev->spi_debug.last_rx_preview, 0, sizeof(dev->spi_debug.last_rx_preview));

	if(cmd != NULL)
	{
		dev->spi_debug.last_cmd[0] = cmd[0];
		dev->spi_debug.last_cmd[1] = cmd[1];
	}
	else
	{
		dev->spi_debug.last_cmd[0] = 0u;
		dev->spi_debug.last_cmd[1] = 0u;
	}

	if((tx != NULL) && (tx_len > 0u))
	{
		preview_len = adbms2950_min_u16(tx_len, ADBMS2950_SPI_DEBUG_PREVIEW_BYTES);
		memcpy(dev->spi_debug.last_tx_preview, tx, preview_len);
	}
}

static void adbms2950_spi_debug_note_rx(adbms2950_driver_t *dev,
										const uint8_t *rx,
										uint16_t rx_len,
										HAL_StatusTypeDef status)
{
	uint16_t preview_len;

	if((dev == NULL) || (!dev->spi_debug.enabled))
	{
		return;
	}

	if((rx != NULL) && (rx_len > 0u))
	{
		preview_len = adbms2950_min_u16(rx_len, ADBMS2950_SPI_DEBUG_PREVIEW_BYTES);
		memcpy(dev->spi_debug.last_rx_preview, rx, preview_len);
	}

	dev->spi_debug.last_status = status;
	dev->spi_debug.last_xfer_status = status;
}

HAL_StatusTypeDef adbms2950_spi_probe_rdcfga(adbms2950_driver_t *dev)
{
	HAL_StatusTypeDef status;

	if(dev == NULL)
	{
		return HAL_ERROR;
	}

	adbms2950_spi_debug_enable(dev, true);
	adbms2950_wakeup(dev);

	if(dev->spi_debug.enabled)
	{
		dev->spi_debug.last_op = ADBMS2950_SPI_OP_PROBE;
	}

	status = adbms2950_rd48_checked(dev, RDCFGA, buf);
	if(status == HAL_OK)
	{
		adbms2950_parse_cfga(dev, buf);
	}

	return status;
}

void adbms2950_init(adbms2950_driver_t *dev,
					uint8_t num_asics,
					adbms2950_asic *ics,
					SPI_HandleTypeDef *hspi,
					GPIO_TypeDef *CSA_Port,
					GPIO_TypeDef *CSB_Port,
					uint16_t CSA_Pin,
					uint16_t CSB_Pin,
					TIM_HandleTypeDef *htim)
{
	dev->num_ics = num_asics;
	dev->ics = ics;
	dev->hspi = hspi;
	dev->cs_port[0] = CSA_Port;
	dev->cs_port[1] = CSB_Port;
	dev->cs_pin[0] = CSA_Pin;
	dev->cs_pin[1] = CSB_Pin;
	dev->htim = htim;

	memset(&dev->spi_debug, 0, sizeof(dev->spi_debug));
	dev->spi_debug.enabled = true;
	dev->spi_debug.last_status = HAL_OK;
	dev->spi_debug.last_tx_status = HAL_OK;
	dev->spi_debug.last_rx_status = HAL_OK;
	dev->spi_debug.last_xfer_status = HAL_OK;

	// Set CS pins high
	dev->string = STRING_B;
	adbms2950_set_cs(dev, 1);
	dev->string = STRING_A;
	adbms2950_set_cs(dev, 1);

	adbms2950_wakeup(dev);
	adbms2950_srst(dev);
	// 8ms delay. DS and vendor code recommend
	adbms2950_us_delay(dev, 8000);

	adbms2950_reset_cfg_regs(dev);
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		// GPO1 as PUSH-PULL, set to LOW
		dev->ics[cic].tx_cfga.gpo1od = PUSH_PULL;
		dev->ics[cic].tx_cfga.gpo1c = PULLED_DOWN;

		// GPO2 as PUSH-PULL, set to LOW
		dev->ics[cic].tx_cfga.gpo2od = PUSH_PULL;
		dev->ics[cic].tx_cfga.gpo2c = PULLED_DOWN;
	}

	adbms2950_wakeup(dev);
	adbms2950_wrcfga(dev);
	adbms2950_wrcfgb(dev);
	adbms2950_rdcfga(dev);
	adbms2950_rdcfgb(dev);

	// Using Redundant, Continuous measurement
	// See Table 42 page 34
	adi1_ adi1;
	adi1.rd = RD_ON; // Redundant measurement on, starts VB2ADC and I2ADC as well
	adi1.opt = OPT12_C; // Continuous measurement
	adbms2950_wakeup(dev);
	adbms2950_adi1(dev, &adi1);

	// Don't need this call since RD_ON starts VB2ADC and I2ADC above
	/*adi2_ adi2;
	adi2.opt = OPT12_C;
	adbms2950_wakeup(dev);
	adbms2950_adi2(dev, &adi2);
	*/

	// Add delay for device to start
	adbms2950_us_delay(dev, 8000);
}


void adbms2950_cmd(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ])
{
	(void)adbms2950_cmd_checked(dev, cmd);
}

static HAL_StatusTypeDef adbms2950_cmd_checked(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ])
{
	uint16_t pec15;

	if((dev == NULL) || (cmd == NULL))
	{
		return HAL_ERROR;
	}

	wrbuf[0] = cmd[0];
	wrbuf[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	wrbuf[2] = (uint8_t)(pec15 >> 8);
	wrbuf[3] = (uint8_t)pec15;

	if(dev->spi_debug.enabled)
	{
		dev->spi_debug.last_op = ADBMS2950_SPI_OP_CMD;
	}

	return adbms2950_spi_write(dev, wrbuf, CMDSZ + PEC15SZ, 1);
}

void adbms2950_wr48(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data)
{
	(void)adbms2950_wr48_checked(dev, cmd, tx_data);
}

static HAL_StatusTypeDef adbms2950_wr48_checked(adbms2950_driver_t* dev,
										uint8_t cmd[CMDSZ],
										uint8_t* tx_data)
{
	uint16_t pec15;
	uint16_t pec10;
	uint16_t tx_sz;
	uint16_t cmd_index;
	uint8_t src_addr = 0u;
	uint8_t temp[TX_DATA];

	if((dev == NULL) || (cmd == NULL) || (tx_data == NULL) || (dev->num_ics == 0u))
	{
		return HAL_ERROR;
	}

	tx_sz = CMDSZ + PEC15SZ + ((TX_DATA + DPECSZ) * (uint16_t)dev->num_ics);
	if(tx_sz > BUFSZ)
	{
		return HAL_ERROR;
	}

	adbms2950_wakeup(dev);

	wrbuf[0] = cmd[0];
	wrbuf[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	wrbuf[2] = (uint8_t)(pec15 >> 8);
	wrbuf[3] = (uint8_t)pec15;
	cmd_index = 4u;

    for (uint8_t current_ic = dev->num_ics; current_ic > 0u; current_ic--)
    {
      src_addr = (uint8_t)((current_ic - 1u) * TX_DATA);
      /*!< The first configuration written is received by the last IC in the daisy chain */
      for (uint8_t current_byte = 0u; current_byte < TX_DATA; current_byte++)
      {
        wrbuf[cmd_index] = tx_data[src_addr + current_byte];
        cmd_index = cmd_index + 1u;
      }
      /*!< Copy each ic correspond data + pec value for calculate data pec */
      memcpy(temp, &tx_data[src_addr], TX_DATA); /*!< dst, src, size */
      /*!< calculating the PEC for each Ics configuration register data */
      pec10 = (uint16_t)pec10_calc_modular(temp, PEC10_WRITE);
      wrbuf[cmd_index] = (uint8_t)(pec10 >> 8);
      cmd_index = cmd_index + 1u;
      wrbuf[cmd_index] = (uint8_t)pec10;
      cmd_index = cmd_index + 1u;
    }

	if(dev->spi_debug.enabled)
	{
		dev->spi_debug.last_op = ADBMS2950_SPI_OP_WR48;
	}

	return adbms2950_spi_write(dev, wrbuf, tx_sz, 1);
}

void adbms2950_rd48(adbms2950_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data)
{
	(void)adbms2950_rd48_checked(dev, cmd, rx_data);
}

static HAL_StatusTypeDef adbms2950_rd48_checked(adbms2950_driver_t* dev,
										uint8_t cmd[CMDSZ],
										uint8_t* rx_data)
{
	uint16_t pec15;
	uint16_t rx_sz;
	uint8_t wrcmd[CMDSZ + PEC15SZ] = {0};
	uint8_t src_addr = 0u;
	uint16_t received_pec;
	uint16_t calculated_pec;
	uint8_t temp[RX_DATA];
	HAL_StatusTypeDef status;

	if((dev == NULL) || (cmd == NULL) || (rx_data == NULL) || (dev->num_ics == 0u))
	{
		return HAL_ERROR;
	}

	rx_sz = RX_DATA * (uint16_t)dev->num_ics;
	if(rx_sz > BUFSZ)
	{
		return HAL_ERROR;
	}

	wrcmd[0] = cmd[0];
	wrcmd[1] = cmd[1];
	pec15 = Pec15_Calc(CMDSZ, cmd);
	wrcmd[2] = (uint8_t)(pec15 >> 8);
	wrcmd[3] = (uint8_t)pec15;

	adbms2950_wakeup(dev);

	if(dev->spi_debug.enabled)
	{
		dev->spi_debug.last_op = ADBMS2950_SPI_OP_RD48;
	}

	status = adbms2950_spi_write_read(dev, wrcmd, CMDSZ + PEC15SZ, rx_data, rx_sz, 1);
	if(status != HAL_OK)
	{
		return status;
	}

    for (uint8_t current_ic = 0u; current_ic < dev->num_ics; current_ic++)
    {
      uint8_t cmd_counter;
      received_pec = (uint16_t)(((rx_data[(current_ic * RX_DATA) + (RX_DATA - 2u)] & 0x03u) << 8u) |
                                rx_data[(current_ic * RX_DATA) + (RX_DATA - 1u)]);
      memcpy(temp, &rx_data[src_addr], RX_DATA);
      src_addr = (uint8_t)((current_ic + 1u) * RX_DATA);
      calculated_pec = (uint16_t)pec10_calc(1, (RX_DATA - DPECSZ), temp);
      cmd_counter = (uint8_t)(rx_data[(current_ic * RX_DATA) + (RX_DATA - 2u)] >> 2u);

      dev->ics[current_ic].rx_cmd_cntr = cmd_counter;
      dev->ics[current_ic].rx_pec_error = (received_pec != calculated_pec);

      if(dev->spi_debug.enabled && (current_ic < ADBMS2950_MAX_TRACKED_ICS))
      {
        dev->spi_debug.last_cmd_counter[current_ic] = cmd_counter;
        if(dev->ics[current_ic].rx_pec_error)
        {
          dev->spi_debug.last_read_pec_fail_mask |= (uint16_t)(1u << current_ic);
          dev->spi_debug.error_count++;
        }
        else
        {
          dev->spi_debug.last_read_pec_pass_mask |= (uint16_t)(1u << current_ic);
        }
      }
    }

	return HAL_OK;
}

void adbms2950_reset_cfg_regs(adbms2950_driver_t* dev)
{
	// Set device registers to default
	for(uint8_t i = 0; i < dev->num_ics; i++)
	{
		//CFGA
		dev->ics[i].tx_cfga.gpo1c = PULLED_UP_TRISTATED;
		dev->ics[i].tx_cfga.gpo2c = PULLED_UP_TRISTATED;
		dev->ics[i].tx_cfga.gpo3c = PULLED_UP_TRISTATED;
		dev->ics[i].tx_cfga.gpo4c = PULLED_UP_TRISTATED;
		dev->ics[i].tx_cfga.gpo5c = PULLED_UP_TRISTATED;
		dev->ics[i].tx_cfga.gpo6c = PULLED_UP_TRISTATED;

		dev->ics[i].tx_cfga.gpo1od = OPEN_DRAIN;
		dev->ics[i].tx_cfga.gpo2od = OPEN_DRAIN;
		dev->ics[i].tx_cfga.gpo3od = OPEN_DRAIN;
		dev->ics[i].tx_cfga.gpo4od = OPEN_DRAIN;
		dev->ics[i].tx_cfga.gpo5od = OPEN_DRAIN;
		dev->ics[i].tx_cfga.gpo6od = OPEN_DRAIN;

		dev->ics[i].tx_cfga.vs1  = VSM_SGND;
		dev->ics[i].tx_cfga.vs2  = VSM_SGND;
		dev->ics[i].tx_cfga.vs3  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs4  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs5  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs6  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs7  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs8  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs9  = VSMV_SGND;
		dev->ics[i].tx_cfga.vs10 = VSMV_SGND;

		dev->ics[i].tx_cfga.injosc = INJOSC0_NORMAL;
		dev->ics[i].tx_cfga.injmon = INJMON0_NORMAL;
		dev->ics[i].tx_cfga.injts  = NO_THSD;
		dev->ics[i].tx_cfga.injecc = NO_ECC;
		dev->ics[i].tx_cfga.injtm  = NO_TMODE;

		dev->ics[i].tx_cfga.soak    = SOAK_DISABLE;
		dev->ics[i].tx_cfga.ocen    = OC_DISABLE;
		dev->ics[i].tx_cfga.gpio1fe = FAULT_STATUS_DISABLE;
		dev->ics[i].tx_cfga.spi3w   = FOUR_WIRE;

		dev->ics[i].tx_cfga.acci    = ACCI_8;
		dev->ics[i].tx_cfga.commbk  = COMMBK_OFF;
		dev->ics[i].tx_cfga.vb1mux  = SINGLE_ENDED_SGND;
		dev->ics[i].tx_cfga.vb2mux  = SINGLE_ENDED_SGND;

		//CFGB
		dev->ics[i].tx_cfgb.gpio1c = PULL_DOWN_OFF;
		dev->ics[i].tx_cfgb.gpio2c = PULL_DOWN_OFF;
		dev->ics[i].tx_cfgb.gpio3c = PULL_DOWN_OFF;
		dev->ics[i].tx_cfgb.gpio4c = PULL_DOWN_OFF;

		dev->ics[i].tx_cfgb.oc1th = 0x0;
		dev->ics[i].tx_cfgb.oc2th = 0x0;
		dev->ics[i].tx_cfgb.oc3th = 0x0;

		dev->ics[i].tx_cfgb.oc1ten = NORMAL_INPUT;
		dev->ics[i].tx_cfgb.oc2ten = NORMAL_INPUT;
		dev->ics[i].tx_cfgb.oc3ten = NORMAL_INPUT;

		dev->ics[i].tx_cfgb.ocdgt  = OCDGT0_1oo1;
		dev->ics[i].tx_cfgb.ocdp   = OCDP0_NORMAL;
		dev->ics[i].tx_cfgb.reften = NORMAL_INPUT;
		dev->ics[i].tx_cfgb.octsel = OCTSEL0_OCxADC_P140_REFADC_M20;

		dev->ics[i].tx_cfgb.ocod   = PUSH_PULL;
		dev->ics[i].tx_cfgb.oc1gc  = GAIN_1;
		dev->ics[i].tx_cfgb.oc2gc  = GAIN_1;
		dev->ics[i].tx_cfgb.oc3gc  = GAIN_1;
		dev->ics[i].tx_cfgb.ocmode = OCMODE0_DISABLED;
		dev->ics[i].tx_cfgb.ocax   = OCABX_ACTIVE_HIGH;
		dev->ics[i].tx_cfgb.ocbx   = OCABX_ACTIVE_HIGH;

		dev->ics[i].tx_cfgb.diagsel   = DIAGSEL0_IAB_VBAT;
		dev->ics[i].tx_cfgb.gpio2eoc  = EOC_DISABLED;
	}
}

void adbms2950_srst(adbms2950_driver_t* dev)
{
	adbms2950_cmd(dev, SRST);
}

void adbms2950_wrcfga(adbms2950_driver_t* dev)
{
	uint8_t address;

	adbms2950_pack_cfga(dev);
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		address = cic * TX_DATA;
		for(uint8_t byte = 0; byte < TX_DATA; byte++)
		{
			buf[address + byte] = dev->ics[cic].configa.tx_data[byte];
		}
	}
	adbms2950_wr48(dev, WRCFGA, buf);
}

void adbms2950_wrcfgb(adbms2950_driver_t* dev)
{
	uint8_t address;

	adbms2950_pack_cfgb(dev);
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		address = cic * TX_DATA;
		for(uint8_t byte = 0; byte < TX_DATA; byte++)
		{
			buf[address + byte] = dev->ics[cic].configb.tx_data[byte];
		}
	}
	adbms2950_wr48(dev, WRCFGB, buf);
}

void adbms2950_rdcfga(adbms2950_driver_t* dev)
{
	adbms2950_rd48(dev, RDCFGA, buf);
	adbms2950_parse_cfga(dev, buf);
}

void adbms2950_rdcfgb(adbms2950_driver_t* dev)
{
	adbms2950_rd48(dev, RDCFGB, buf);
	adbms2950_parse_cfgb(dev, buf);
}

void adbms2950_parse_cfga(adbms2950_driver_t* dev, uint8_t *data)
{
  uint8_t address = 0;
  adbms2950_asic* ic = dev->ics;
  for(uint8_t cic = 0; cic < dev->num_ics; cic++)
  {
	  address = cic * RX_DATA;
    memcpy(ic[cic].configa.rx_data, &data[address], RX_DATA);

    ic[cic].rx_cfga.vs1             = (ic[cic].configa.rx_data[0] & 0x03);
    ic[cic].rx_cfga.vs2             = (ic[cic].configa.rx_data[0] & 0x0C) >> 2;
    ic[cic].rx_cfga.vs3             = (ic[cic].configa.rx_data[0] & 0x10) >> 4;
    ic[cic].rx_cfga.vs4             = (ic[cic].configa.rx_data[0] & 0x20) >> 5;
    ic[cic].rx_cfga.vs5             = (ic[cic].configa.rx_data[0] & 0x40) >> 6;
    ic[cic].rx_cfga.ocen            = (ic[cic].configa.rx_data[0] & 0x80) >> 7;

    ic[cic].rx_cfga.injosc          = (ic[cic].configa.rx_data[1] & 0x03);
    ic[cic].rx_cfga.injmon          = (ic[cic].configa.rx_data[1] & 0x0C) >> 2;
    ic[cic].rx_cfga.injts           = (ic[cic].configa.rx_data[1] & 0x10) >> 4;
    ic[cic].rx_cfga.injecc          = (ic[cic].configa.rx_data[1] & 0x40) >> 6;
    ic[cic].rx_cfga.injtm           = (ic[cic].configa.rx_data[1] & 0x80) >> 7;

    ic[cic].rx_cfga.vs6             = (ic[cic].configa.rx_data[2] & 0x01);
    ic[cic].rx_cfga.vs7             = (ic[cic].configa.rx_data[2] & 0x02) >> 1;
    ic[cic].rx_cfga.vs8             = (ic[cic].configa.rx_data[2] & 0x04) >> 2;
    ic[cic].rx_cfga.vs9             = (ic[cic].configa.rx_data[2] & 0x08) >> 3;
    ic[cic].rx_cfga.vs10            = (ic[cic].configa.rx_data[2] & 0x10) >> 4;
    ic[cic].rx_cfga.soak            = (ic[cic].configa.rx_data[2] & 0xE0) >> 5;

    ic[cic].rx_cfga.gpo1c           = (ic[cic].configa.rx_data[3] & 0x01);
    ic[cic].rx_cfga.gpo2c           = (ic[cic].configa.rx_data[3] & 0x02) >> 1;
    ic[cic].rx_cfga.gpo3c           = (ic[cic].configa.rx_data[3] & 0x04) >> 2;
    ic[cic].rx_cfga.gpo4c           = (ic[cic].configa.rx_data[3] & 0x08) >> 3;
    ic[cic].rx_cfga.gpo5c           = (ic[cic].configa.rx_data[3] & 0x10) >> 4;
    ic[cic].rx_cfga.gpo6c           = (ic[cic].configa.rx_data[3] & 0x60) >> 5;

    ic[cic].rx_cfga.gpo1od          = (ic[cic].configa.rx_data[4] & 0x01);
    ic[cic].rx_cfga.gpo2od          = (ic[cic].configa.rx_data[4] & 0x02) >> 1;
    ic[cic].rx_cfga.gpo3od          = (ic[cic].configa.rx_data[4] & 0x04) >> 2;
    ic[cic].rx_cfga.gpo4od          = (ic[cic].configa.rx_data[4] & 0x08) >> 3;
    ic[cic].rx_cfga.gpo5od          = (ic[cic].configa.rx_data[4] & 0x10) >> 4;
    ic[cic].rx_cfga.gpo6od          = (ic[cic].configa.rx_data[4] & 0x20) >> 5;
    ic[cic].rx_cfga.gpio1fe         = (ic[cic].configa.rx_data[4] & 0x40) >> 6;
    ic[cic].rx_cfga.spi3w           = (ic[cic].configa.rx_data[4] & 0x80) >> 7;

    ic[cic].rx_cfga.acci            = (ic[cic].configa.rx_data[5] & 0x07);
    ic[cic].rx_cfga.commbk          = (ic[cic].configa.rx_data[5] & 0x08) >> 3;
    ic[cic].rx_cfga.refup           = (ic[cic].configa.rx_data[5] & 0x10) >> 4;
    ic[cic].rx_cfga.snapst          = (ic[cic].configa.rx_data[5] & 0x20) >> 5;
    ic[cic].rx_cfga.vb1mux          = (ic[cic].configa.rx_data[5] & 0x40) >> 6;
    ic[cic].rx_cfga.vb2mux          = (ic[cic].configa.rx_data[5] & 0x80) >> 7;
  }
}

void adbms2950_pack_cfga(adbms2950_driver_t* dev)
{
	adbms2950_asic *ic = dev->ics;
  for(uint8_t cic = 0; cic < dev->num_ics; cic++)
  {
    ic[cic].configa.tx_data[0] = (((ic[cic].tx_cfga.ocen & 0x01) << 7) | ((ic[cic].tx_cfga.vs5 & 0x01) << 6) | ((ic[cic].tx_cfga.vs4 & 0x01) << 5)
                                      | ((ic[cic].tx_cfga.vs3 & 0x01) << 4) | ((ic[cic].tx_cfga.vs2 & 0x03) << 2) | (ic[cic].tx_cfga.vs1 & 0x03));
    ic[cic].configa.tx_data[1] = (((ic[cic].tx_cfga.injtm & 0x01) << 7) | ((ic[cic].tx_cfga.injecc & 0x01) << 6) | ((ic[cic].tx_cfga.injts & 0x01) << 4)
                                      | ((ic[cic].tx_cfga.injmon & 0x03) << 2) | (ic[cic].tx_cfga.injosc & 0x03));
    ic[cic].configa.tx_data[2] = (((ic[cic].tx_cfga.soak & 0x07) << 5) | ((ic[cic].tx_cfga.vs10 & 0x01) << 4) | ((ic[cic].tx_cfga.vs9 & 0x01) << 3)
                                      | ((ic[cic].tx_cfga.vs8 & 0x01) << 2) | ((ic[cic].tx_cfga.vs7 & 0x01) << 1) | (ic[cic].tx_cfga.vs6 & 0x01));
    ic[cic].configa.tx_data[3] = (((ic[cic].tx_cfga.gpo6c & 0x03) << 5) | ((ic[cic].tx_cfga.gpo5c & 0x01) << 4)  | ((ic[cic].tx_cfga.gpo4c & 0x01) << 3)
                                      | ((ic[cic].tx_cfga.gpo3c & 0x01) << 2) | ((ic[cic].tx_cfga.gpo2c & 0x01) << 1)| (ic[cic].tx_cfga.gpo1c & 0x01));   // GPO1 is at position 0
    ic[cic].configa.tx_data[4] = (((ic[cic].tx_cfga.spi3w & 0x01)  << 7)| ((ic[cic].tx_cfga.gpio1fe & 0x01) << 6) | ((ic[cic].tx_cfga.gpo6od & 0x01) << 5)
                                      | ((ic[cic].tx_cfga.gpo5od & 0x01) << 4) |((ic[cic].tx_cfga.gpo4od & 0x01) << 3) | ((ic[cic].tx_cfga.gpo3od & 0x01) << 2)
                                        | ((ic[cic].tx_cfga.gpo2od & 0x01) << 1) | (ic[cic].tx_cfga.gpo1od & 0x01));
    ic[cic].configa.tx_data[5] = (((ic[cic].tx_cfga.vb2mux & 0x01)  << 7)| ((ic[cic].tx_cfga.vb1mux & 0x01) << 6) | ((ic[cic].tx_cfga.snapst & 0x01) << 5)
                                      | ((ic[cic].tx_cfga.refup & 0x01) << 4) |((ic[cic].tx_cfga.commbk & 0x01) << 3) | (ic[cic].tx_cfga.acci & 0x07));
  }
}

void adbms2950_parse_cfgb(adbms2950_driver_t* dev, uint8_t *data)
{
	uint8_t address = 0;
	adbms2950_asic *ic = dev->ics;
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		memcpy(ic[cic].configb.rx_data, &data[address], RX_DATA); /* dst , src , size */
		address = ((cic+1) * (RX_DATA));

		ic[cic].rx_cfgb.oc1th           = (ic[cic].configb.rx_data[0] & 0x7F);
		ic[cic].rx_cfgb.oc1ten          = (ic[cic].configb.rx_data[0] & 0x80) >> 7;

		ic[cic].rx_cfgb.oc2th           = (ic[cic].configb.rx_data[1] & 0x7F);
		ic[cic].rx_cfgb.oc2ten          = (ic[cic].configb.rx_data[1] & 0x80) >> 7;

		ic[cic].rx_cfgb.oc3th           = (ic[cic].configb.rx_data[2] & 0x7F);
		ic[cic].rx_cfgb.oc3ten          = (ic[cic].configb.rx_data[2] & 0x80) >> 7;

		ic[cic].rx_cfgb.ocdgt           = (ic[cic].configb.rx_data[3] & 0x03);
		ic[cic].rx_cfgb.ocdp            = (ic[cic].configb.rx_data[3] & 0x08) >> 3;
		ic[cic].rx_cfgb.reften          = (ic[cic].configb.rx_data[3] & 0x20) >> 5;
		ic[cic].rx_cfgb.octsel          = (ic[cic].configb.rx_data[3] & 0xC0) >> 6;

		ic[cic].rx_cfgb.ocod            = (ic[cic].configb.rx_data[4] & 0x01);
		ic[cic].rx_cfgb.oc1gc           = (ic[cic].configb.rx_data[4] & 0x02) >> 1;
		ic[cic].rx_cfgb.oc2gc           = (ic[cic].configb.rx_data[4] & 0x04) >> 2;
		ic[cic].rx_cfgb.oc3gc           = (ic[cic].configb.rx_data[4] & 0x08) >> 3;
		ic[cic].rx_cfgb.ocmode          = (ic[cic].configb.rx_data[4] & 0x30) >> 4;
		ic[cic].rx_cfgb.ocax            = (ic[cic].configb.rx_data[4] & 0x40) >> 6;
		ic[cic].rx_cfgb.ocbx            = (ic[cic].configb.rx_data[4] & 0x80) >> 7;

		ic[cic].rx_cfgb.diagsel         = (ic[cic].configb.rx_data[5] & 0x07);
		ic[cic].rx_cfgb.gpio2eoc        = (ic[cic].configb.rx_data[5] & 0x08) >> 3;
		ic[cic].rx_cfgb.gpio1c          = (ic[cic].configb.rx_data[5] & 0x10) >> 4;
		ic[cic].rx_cfgb.gpio2c          = (ic[cic].configb.rx_data[5] & 0x20) >> 5;
		ic[cic].rx_cfgb.gpio3c          = (ic[cic].configb.rx_data[5] & 0x40) >> 6;
		ic[cic].rx_cfgb.gpio4c          = (ic[cic].configb.rx_data[5] & 0x80) >> 7;
	}
}

void adbms2950_pack_cfgb(adbms2950_driver_t* dev)
{
  adbms2950_asic *ic = dev->ics;
  for(uint8_t cic = 0; cic < dev->num_ics; cic++)
  {
    ic[cic].configb.tx_data[0] = (((ic[cic].tx_cfgb.oc1ten & 0x01) << 7) | (ic[cic].tx_cfgb.oc1th & 0x7F));
    ic[cic].configb.tx_data[1] = (((ic[cic].tx_cfgb.oc2ten & 0x01) << 7) | (ic[cic].tx_cfgb.oc2th & 0x7F));
    ic[cic].configb.tx_data[2] = (((ic[cic].tx_cfgb.oc3ten & 0x01) << 7) | (ic[cic].tx_cfgb.oc3th & 0x7F));
    ic[cic].configb.tx_data[3] = 0x00;
    ic[cic].configb.tx_data[4] = 0x00;
    ic[cic].configb.tx_data[5] = 0x00;
    ic[cic].configb.tx_data[3] = (((ic[cic].tx_cfgb.octsel & 0x03) << 6) | ((ic[cic].tx_cfgb.reften & 0x01) << 5)  | ((ic[cic].tx_cfgb.ocdp & 0x01) << 3)
                                      | (ic[cic].tx_cfgb.ocdgt & 0x03));   // GPO1 is at position 0
    ic[cic].configb.tx_data[4] = (((ic[cic].tx_cfgb.ocbx & 0x01)  << 7)| ((ic[cic].tx_cfgb.ocax & 0x01) << 6) | ((ic[cic].tx_cfgb.ocmode & 0x03) << 4)
                                      |((ic[cic].tx_cfgb.oc3gc & 0x01) << 3) | ((ic[cic].tx_cfgb.oc2gc & 0x01) << 2) | ((ic[cic].tx_cfgb.oc1gc & 0x01) << 1)
                                        | (ic[cic].tx_cfgb.ocod & 0x01));
    ic[cic].configb.tx_data[5] = (((ic[cic].tx_cfgb.gpio4c & 0x01)  << 7)| ((ic[cic].tx_cfgb.gpio3c & 0x01) << 6) | ((ic[cic].tx_cfgb.gpio2c & 0x01) << 5)
                                      | ((ic[cic].tx_cfgb.gpio1c & 0x01) << 4) |((ic[cic].tx_cfgb.gpio2eoc & 0x01) << 3) | (ic[cic].tx_cfgb.diagsel & 0x07));
  }
}

void adbms2950_adi1(adbms2950_driver_t* dev, adi1_* arg)
{
	uint8_t cmd[CMDSZ];
	uint8_t rd = arg->rd & 0x01;
	uint8_t opt = arg->opt & 0x0F;

	cmd[0] = sADI1[0] | rd;
	cmd[1] = sADI1[1] | ((opt & 0x08) << 4) | ((opt & 0x04) << 2) | (opt & 0x03);

	adbms2950_cmd(dev, cmd);
}

void adbms2950_adi2(adbms2950_driver_t* dev, adi2_* arg)
{
	uint8_t cmd[CMDSZ];
	uint8_t opt = arg->opt & 0x0F;

	cmd[0] = sADI2[0];
	cmd[1] = sADI2[1] | ((opt & 0x08) << 4) | ((opt & 0x04) << 2) | (opt & 0x03);

	adbms2950_cmd(dev, cmd);
}

void adbms2950_adv(adbms2950_driver_t* dev, adv_* arg)
{
	uint8_t cmd[CMDSZ];
	uint8_t OW = arg->ow & 0x03;
	uint8_t VCH = arg->ch & 0x0F;

	cmd[0] = sADV[0];
	cmd[1] = sADV[1] | (OW << 6) | VCH;

	adbms2950_cmd(dev, cmd);
}

void adbms2950_plv(adbms2950_driver_t* dev)
{
	adbms2950_cmd(dev, PLV);
}

void adbms2950_rdvb(adbms2950_driver_t* dev)
{
	adbms2950_rd48(dev, RDVB, buf);
	adbms2950_parse_rdvb(dev, buf);
}

void adbms2950_parse_rdvb(adbms2950_driver_t* dev, uint8_t* vbat_data)
{
	  uint8_t address = 0;
	  for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	  {
		  address = cic * RX_DATA;
	    memcpy(&dev->ics[cic].reg.rx_data[0], &vbat_data[address], RX_DATA);
	    dev->ics[cic].vbat.vbat1 = dev->ics[cic].reg.rx_data[2] + (dev->ics[cic].reg.rx_data[3] << 8);
	    dev->ics[cic].vbat.vbat2 = dev->ics[cic].reg.rx_data[4] + (dev->ics[cic].reg.rx_data[5] << 8);
	  }
}

void adbms2950_rdi(adbms2950_driver_t* dev)
{
	adbms2950_rd48(dev, RDI, buf);
	adbms2950_parse_rdi(dev, buf);
}

void adbms2950_parse_rdi(adbms2950_driver_t* dev, uint8_t* i_data)
{
	  uint8_t address = 0;
	  for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	  {
		  address = cic * RX_DATA;
	    memcpy(&dev->ics[cic].reg.rx_data[0], &i_data[address], RX_DATA);
	    dev->ics[cic].i.i1 = (uint32_t)0 + dev->ics[cic].reg.rx_data[0] + (dev->ics[cic].reg.rx_data[1] << 8) + (dev->ics[cic].reg.rx_data[2] << 16);
	    dev->ics[cic].i.i2 = (uint32_t)0 + dev->ics[cic].reg.rx_data[3] + (dev->ics[cic].reg.rx_data[4] << 8) + (dev->ics[cic].reg.rx_data[5] << 16);
	    // Sign extend signed 24 bit value to int32_t
	    if(dev->ics[cic].i.i1 & 0x800000) dev->ics[cic].i.i1 |= 0xFF000000;
	    if(dev->ics[cic].i.i2 & 0x800000) dev->ics[cic].i.i2 |= 0xFF000000;
	  }
}

void adbms2950_rdv1d(adbms2950_driver_t* dev)
{
	adbms2950_rd48(dev, RDV1D, buf);
	adbms2950_parse_rdv1d(dev, buf);
}

void adbms2950_parse_rdv1d(adbms2950_driver_t* dev, uint8_t* v_data)
{
	uint8_t address;
	uint8_t temp[RX_DATA];
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		address = cic * RX_DATA;
		memcpy(temp, &v_data[address], RX_DATA);
		dev->ics[cic].vr.v_codes[9] =  (temp[0] + (temp[1] << 8)); // V7A
		dev->ics[cic].vr.v_codes[10] =  (temp[2] + (temp[3] << 8)); // V8A
		dev->ics[cic].vr.v_codes[11] =  (temp[4] + (temp[5] << 8)); // V9B
	}
}

void adbms2950_gpo_set(adbms2950_driver_t* dev, GPO gpo, CFGA_GPO state)
{
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		switch(gpo)
		{
			case GPO1:
				dev->ics[cic].tx_cfga.gpo1c = state;
				break;
			case GPO2:
				dev->ics[cic].tx_cfga.gpo2c = state;
				break;
			case GPO3:
				dev->ics[cic].tx_cfga.gpo3c = state;
				break;
			case GPO4:
				dev->ics[cic].tx_cfga.gpo4c = state;
				break;
			case GPO5:
				dev->ics[cic].tx_cfga.gpo5c = state;
				break;
			case GPO6:
				dev->ics[cic].tx_cfga.gpo6c = state;
				break;
			default:
				return;
		}
	}
}

void adbms2950_wakeup(adbms2950_driver_t *dev)
{
	for(uint8_t i = 0; i < dev->num_ics; i++)
	{
		adbms2950_set_cs(dev, 0);
		adbms2950_us_delay(dev, WAKEUP_US_DELAY);
		adbms2950_set_cs(dev, 1);
		adbms2950_us_delay(dev, WAKEUP_BW_DELAY);
	}
}

void adbms2950_set_cs(adbms2950_driver_t* dev, uint8_t state)
{
	if((dev == NULL) ||
	   (dev->string > STRING_B) ||
	   (dev->cs_port[dev->string] == NULL))
	{
		return;
	}

	HAL_GPIO_WritePin(dev->cs_port[dev->string], dev->cs_pin[dev->string], state);
}

void adbms2950_us_delay(adbms2950_driver_t* dev, uint16_t microseconds)
{
	if((dev == NULL) || (dev->htim == NULL))
	{
		return;
	}

	__HAL_TIM_SET_COUNTER(dev->htim, 0);
	while (__HAL_TIM_GET_COUNTER(dev->htim) < microseconds);
	return;
}


HAL_StatusTypeDef adbms2950_spi_write(adbms2950_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs)
{
	HAL_StatusTypeDef status;

	if((dev == NULL) || (dev->hspi == NULL) || (data == NULL) || (len == 0u))
	{
		return HAL_ERROR;
	}

	adbms2950_spi_debug_note_tx(dev, dev->spi_debug.last_op, data, data, len, 0u);

	adbms_spi_lock();
	if(use_cs)
	{
		adbms2950_set_cs(dev, 0);
	}

	status = HAL_SPI_Transmit(dev->hspi, data, len, SPI_TIMEOUT);

	if(use_cs)
	{
		adbms2950_set_cs(dev, 1);
	}
	adbms_spi_unlock();

	if(dev->spi_debug.enabled)
	{
		dev->spi_debug.tx_count++;
		dev->spi_debug.last_tx_status = status;
		dev->spi_debug.last_status = status;
		if(status != HAL_OK)
		{
			dev->spi_debug.error_count++;
		}
	}

	return status;
}

HAL_StatusTypeDef adbms2950_spi_write_read(adbms2950_driver_t *dev,
										  uint8_t* tx_Data,
										  uint16_t tx_len,
										  uint8_t* rx_data,
										  uint16_t rx_len,
										  uint8_t use_cs)
{
	HAL_StatusTypeDef status;
	uint16_t total_len;

	if((dev == NULL) || (dev->hspi == NULL) || (tx_Data == NULL) ||
	   (rx_data == NULL) || (tx_len == 0u) || (rx_len == 0u))
	{
		return HAL_ERROR;
	}

	total_len = (uint16_t)(tx_len + rx_len);
	if(total_len > BUFSZ)
	{
		return HAL_ERROR;
	}

	memset(adbms2950_spi_txrx_tx_buf, ADBMS2950_SPI_DUMMY_BYTE, total_len);
	memset(adbms2950_spi_txrx_rx_buf, 0, total_len);
	memcpy(adbms2950_spi_txrx_tx_buf, tx_Data, tx_len);

	adbms2950_spi_debug_note_tx(dev, dev->spi_debug.last_op, tx_Data, adbms2950_spi_txrx_tx_buf, tx_len, rx_len);

	adbms_spi_lock();
	if(use_cs)
	{
		adbms2950_set_cs(dev, 0);
	}

	status = HAL_SPI_TransmitReceive(dev->hspi,
								  adbms2950_spi_txrx_tx_buf,
								  adbms2950_spi_txrx_rx_buf,
								  total_len,
								  SPI_TIMEOUT);

	if(use_cs)
	{
		adbms2950_set_cs(dev, 1);
	}
	adbms_spi_unlock();

	if(status == HAL_OK)
	{
		memcpy(rx_data, &adbms2950_spi_txrx_rx_buf[tx_len], rx_len);
	}
	else
	{
		memset(rx_data, 0, rx_len);
	}

	if(dev->spi_debug.enabled)
	{
		dev->spi_debug.tx_count++;
		dev->spi_debug.rx_count++;
		dev->spi_debug.last_tx_status = status;
		dev->spi_debug.last_rx_status = status;
		dev->spi_debug.last_xfer_status = status;
		dev->spi_debug.last_status = status;
		if(status != HAL_OK)
		{
			dev->spi_debug.error_count++;
		}
	}

	adbms2950_spi_debug_note_rx(dev, rx_data, rx_len, status);

	return status;
}
