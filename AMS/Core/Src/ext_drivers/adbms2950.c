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

static uint8_t buf[BUFSZ] = {0};
static uint8_t wrbuf[BUFSZ] = {0};
static uint8_t adbms2950_spi_txrx_tx_buf[BUFSZ] = {0};
static uint8_t adbms2950_spi_txrx_rx_buf[BUFSZ] = {0};

#define ADBMS2950_SPI_DUMMY_BYTE 0xFFu
#define ADBMS2950_DELAY_SPINS_PER_US 256u
#define ADBMS2950_DELAY_BASE_SPINS 1024u
#define ADBMS2950_I1_CAL_MASK 0x40u

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
static HAL_StatusTypeDef adbms2950_wrcfga_checked(adbms2950_driver_t *dev);
static HAL_StatusTypeDef adbms2950_wrcfgb_checked(adbms2950_driver_t *dev);
static void adbms2950_force_dividers_off_best_effort(adbms2950_driver_t *dev);
static bool adbms2950_topology_valid(const adbms2950_driver_t *dev);
static void adbms2950_sat_inc_u32(uint32_t *value);
static int32_t adbms2950_sign_extend_24(uint32_t raw);

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

static bool adbms2950_topology_valid(const adbms2950_driver_t *dev)
{
	return (dev != NULL) &&
	       (dev->num_ics > 0u) &&
	       (dev->num_ics <= ADBMS2950_MAX_TRACKED_ICS) &&
	       (dev->ics != NULL) &&
	       (dev->ics_capacity >= dev->num_ics) &&
	       (dev->hspi != NULL) &&
	       (dev->htim != NULL) &&
	       ((int)dev->string >= (int)STRING_A) &&
	       (dev->string <= STRING_B) &&
	       (dev->cs_port[dev->string] != NULL) &&
	       (dev->cs_pin[dev->string] != 0u);
}

static void adbms2950_sat_inc_u32(uint32_t *value)
{
	if((value != NULL) && (*value != UINT32_MAX))
	{
		(*value)++;
	}
}

static int32_t adbms2950_sign_extend_24(uint32_t raw)
{
	raw &= 0x00FFFFFFu;
	if((raw & 0x00800000u) != 0u)
	{
		raw |= 0xFF000000u;
	}
	return (int32_t)raw;
}

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

const adbms2950_health_t *adbms2950_health_get(const adbms2950_driver_t *dev)
{
	return (dev == NULL) ? NULL : &dev->health;
}

void adbms2950_health_clear_counters(adbms2950_driver_t *dev)
{
	if(dev == NULL)
	{
		return;
	}

	dev->health.sample_count = 0u;
	dev->health.sample_error_count = 0u;
	dev->health.pec_error_count = 0u;
	dev->health.counter_mismatch_count = 0u;
	dev->health.counter_stall_count = 0u;
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
	dev->spi_debug.last_total_len =
		(rx_len > (uint16_t)(UINT16_MAX - tx_len)) ?
		UINT16_MAX : (uint16_t)(tx_len + rx_len);
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
	status = adbms2950_rd48_checked(dev, RDCFGA, buf);
	if(status == HAL_OK)
	{
		adbms2950_parse_cfga(dev, buf);
	}
	if(dev->spi_debug.enabled)
	{
		dev->spi_debug.last_op = ADBMS2950_SPI_OP_PROBE;
	}

	return status;
}

HAL_StatusTypeDef adbms2950_spi_probe_sid(adbms2950_driver_t *dev)
{
	HAL_StatusTypeDef status;

	if(dev == NULL)
	{
		return HAL_ERROR;
	}

	adbms2950_spi_debug_enable(dev, true);
	status = adbms2950_read_sid(dev);
	if(dev->spi_debug.enabled)
	{
		dev->spi_debug.last_op = ADBMS2950_SPI_OP_PROBE;
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
	(void)adbms2950_init_mixed_chain(dev,
	                                 num_asics,
	                                 ics,
	                                 num_asics,
	                                 hspi,
	                                 CSA_Port,
	                                 CSB_Port,
	                                 CSA_Pin,
	                                 CSB_Pin,
	                                 htim,
	                                 STRING_A,
	                                 true,
	                                 false);
}

HAL_StatusTypeDef adbms2950_init_mixed_chain(adbms2950_driver_t *dev,
											 uint8_t num_asics,
											 adbms2950_asic *ics,
											 uint8_t ics_capacity,
											 SPI_HandleTypeDef *hspi,
											 GPIO_TypeDef *CSA_Port,
											 GPIO_TypeDef *CSB_Port,
											 uint16_t CSA_Pin,
											 uint16_t CSB_Pin,
											 TIM_HandleTypeDef *htim,
											 adbms_string primary_string,
											 bool issue_chain_reset,
											 bool enable_hv_dividers)
{
	HAL_StatusTypeDef status;

	if(dev == NULL)
	{
		return HAL_ERROR;
	}

	memset(dev, 0, sizeof(*dev));
	dev->num_ics = num_asics;
	dev->ics_capacity = ics_capacity;
	dev->ics = ics;
	dev->hspi = hspi;
	dev->cs_port[STRING_A] = CSA_Port;
	dev->cs_port[STRING_B] = CSB_Port;
	dev->cs_pin[STRING_A] = CSA_Pin;
	dev->cs_pin[STRING_B] = CSB_Pin;
	dev->htim = htim;
	dev->string = primary_string;
	dev->delay_last_status = (htim != NULL) ? HAL_OK : HAL_ERROR;
	dev->spi_debug.enabled = true;
	dev->spi_debug.last_status = HAL_ERROR;
	dev->spi_debug.last_tx_status = HAL_ERROR;
	dev->spi_debug.last_rx_status = HAL_ERROR;
	dev->spi_debug.last_xfer_status = HAL_ERROR;
	dev->health.last_status = HAL_ERROR;
	dev->health.hv_dividers_enabled = false;

	if((num_asics == 0u) ||
	   (num_asics > ADBMS2950_MAX_TRACKED_ICS) ||
	   (ics == NULL) ||
	   (ics_capacity < num_asics) ||
	   (hspi == NULL) ||
	   (htim == NULL) ||
	   (CSA_Port == NULL) ||
	   (CSB_Port == NULL) ||
	   (CSA_Pin == 0u) ||
	   (CSB_Pin == 0u) ||
	   ((int)primary_string < (int)STRING_A) ||
	   (primary_string > STRING_B))
	{
		return HAL_ERROR;
	}

	memset(ics, 0, sizeof(*ics) * num_asics);
	dev->string = STRING_A;
	adbms2950_set_cs(dev, 1u);
	dev->string = STRING_B;
	adbms2950_set_cs(dev, 1u);
	dev->string = primary_string;

	if(issue_chain_reset)
	{
		status = adbms2950_wakeup_checked(dev);
		if(status == HAL_OK)
		{
			status = adbms2950_cmd_checked(dev, SRST);
		}
		if(status == HAL_OK)
		{
			status = adbms2950_us_delay(dev, 8000u);
		}
		if(status != HAL_OK)
		{
			dev->health.last_status = status;
			return status;
		}
	}

	/* RDCFGA cannot distinguish a 2950 from a compatible cell monitor.  SID
	 * derivative bits must identify an ADBMS2950B before any APM-specific
	 * configuration is written into the mixed chain. */
	status = adbms2950_read_sid(dev);
	if(status != HAL_OK)
	{
		dev->health.last_status = status;
		return status;
	}

	adbms2950_reset_cfg_regs(dev);
	for(uint8_t cic = 0u; cic < dev->num_ics; cic++)
	{
		dev->ics[cic].tx_cfga.gpo1od = PUSH_PULL;
		dev->ics[cic].tx_cfga.gpo2od = PUSH_PULL;
		dev->ics[cic].tx_cfga.gpo1c = enable_hv_dividers ? GPO_SET : GPO_CLR;
		dev->ics[cic].tx_cfga.gpo2c = enable_hv_dividers ? GPO_SET : GPO_CLR;
		dev->ics[cic].tx_cfga.commbk = COMMBK_OFF;
	}

	status = adbms2950_wrcfga_checked(dev);
	if(status == HAL_OK)
	{
		status = adbms2950_wrcfgb_checked(dev);
	}
	if(status == HAL_OK)
	{
		status = adbms2950_verify_config_readback(dev);
	}
	if(status != HAL_OK)
	{
		/* The final-board divider enables are deliberately fail-low.  A failed
		 * write or mismatched readback leaves the remote register state unknown,
		 * so make one bounded best-effort attempt to drive both enables low.  The
		 * original failure remains authoritative even if this cleanup succeeds. */
		adbms2950_force_dividers_off_best_effort(dev);
	}

	dev->health.config_valid = (status == HAL_OK);
	dev->health.initialized = (status == HAL_OK);
	dev->health.hv_dividers_enabled = (status == HAL_OK) && enable_hv_dividers;
	dev->health.last_status = status;
	return status;
}

static void adbms2950_force_dividers_off_best_effort(adbms2950_driver_t *dev)
{
	if(!adbms2950_topology_valid(dev))
	{
		return;
	}

	for(uint8_t ic = 0u; ic < dev->num_ics; ic++)
	{
		dev->ics[ic].tx_cfga.gpo1od = PUSH_PULL;
		dev->ics[ic].tx_cfga.gpo2od = PUSH_PULL;
		dev->ics[ic].tx_cfga.gpo1c = GPO_CLR;
		dev->ics[ic].tx_cfga.gpo2c = GPO_CLR;
	}
	dev->health.hv_dividers_enabled = false;
	(void)adbms2950_wrcfga_checked(dev);
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
	HAL_StatusTypeDef status;

	if(!adbms2950_topology_valid(dev) || (cmd == NULL) || (tx_data == NULL))
	{
		return HAL_ERROR;
	}

	tx_sz = CMDSZ + PEC15SZ + ((TX_DATA + DPECSZ) * (uint16_t)dev->num_ics);
	if(tx_sz > BUFSZ)
	{
		return HAL_ERROR;
	}

	status = adbms2950_wakeup_checked(dev);
	if(status != HAL_OK)
	{
		return status;
	}

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
	bool integrity_ok = true;

	if(!adbms2950_topology_valid(dev) || (cmd == NULL) || (rx_data == NULL))
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

	status = adbms2950_wakeup_checked(dev);
	if(status != HAL_OK)
	{
		return status;
	}

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
	  if(dev->ics[current_ic].rx_pec_error)
	  {
		integrity_ok = false;
		adbms2950_sat_inc_u32(&dev->health.pec_error_count);
	  }

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

	if(!integrity_ok && dev->spi_debug.enabled)
	{
		/* Preserve last_xfer_status as the underlying HAL transport result while
		 * exposing the failed end-to-end transaction through last_status. */
		dev->spi_debug.last_status = HAL_ERROR;
	}
	return integrity_ok ? HAL_OK : HAL_ERROR;
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
	(void)adbms2950_wrcfga_checked(dev);
}

static HAL_StatusTypeDef adbms2950_wrcfga_checked(adbms2950_driver_t *dev)
{
	uint8_t address;

	if(!adbms2950_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	adbms2950_pack_cfga(dev);
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		address = cic * TX_DATA;
		for(uint8_t byte = 0; byte < TX_DATA; byte++)
		{
			buf[address + byte] = dev->ics[cic].configa.tx_data[byte];
		}
	}
	return adbms2950_wr48_checked(dev, WRCFGA, buf);
}

void adbms2950_wrcfgb(adbms2950_driver_t* dev)
{
	(void)adbms2950_wrcfgb_checked(dev);
}

static HAL_StatusTypeDef adbms2950_wrcfgb_checked(adbms2950_driver_t *dev)
{
	uint8_t address;

	if(!adbms2950_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	adbms2950_pack_cfgb(dev);
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		address = cic * TX_DATA;
		for(uint8_t byte = 0; byte < TX_DATA; byte++)
		{
			buf[address + byte] = dev->ics[cic].configb.tx_data[byte];
		}
	}
	return adbms2950_wr48_checked(dev, WRCFGB, buf);
}

void adbms2950_rdcfga(adbms2950_driver_t* dev)
{
	if(adbms2950_rd48_checked(dev, RDCFGA, buf) == HAL_OK)
	{
		adbms2950_parse_cfga(dev, buf);
	}
}

void adbms2950_rdcfgb(adbms2950_driver_t* dev)
{
	if(adbms2950_rd48_checked(dev, RDCFGB, buf) == HAL_OK)
	{
		adbms2950_parse_cfgb(dev, buf);
	}
}

HAL_StatusTypeDef adbms2950_verify_config_readback(adbms2950_driver_t *dev)
{
	HAL_StatusTypeDef status;

	if(!adbms2950_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	adbms2950_pack_cfga(dev);
	status = adbms2950_rd48_checked(dev, RDCFGA, buf);
	if(status != HAL_OK)
	{
		dev->health.config_valid = false;
		return status;
	}
	for(uint8_t ic = 0u; ic < dev->num_ics; ic++)
	{
		if(memcmp(&buf[(uint16_t)ic * RX_DATA],
		          dev->ics[ic].configa.tx_data,
		          TX_DATA) != 0)
		{
			dev->health.config_valid = false;
			return HAL_ERROR;
		}
	}
	adbms2950_parse_cfga(dev, buf);

	adbms2950_pack_cfgb(dev);
	status = adbms2950_rd48_checked(dev, RDCFGB, buf);
	if(status != HAL_OK)
	{
		dev->health.config_valid = false;
		return status;
	}
	for(uint8_t ic = 0u; ic < dev->num_ics; ic++)
	{
		if(memcmp(&buf[(uint16_t)ic * RX_DATA],
		          dev->ics[ic].configb.tx_data,
		          TX_DATA) != 0)
		{
			dev->health.config_valid = false;
			return HAL_ERROR;
		}
	}
	adbms2950_parse_cfgb(dev, buf);
	dev->health.config_valid = true;
	return HAL_OK;
}

HAL_StatusTypeDef adbms2950_read_sid(adbms2950_driver_t *dev)
{
	HAL_StatusTypeDef status;

	if(!adbms2950_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	status = adbms2950_rd48_checked(dev, RDSID, buf);
	if(status != HAL_OK)
	{
		dev->health.sid_valid = false;
		dev->health.last_status = status;
		return status;
	}

	for(uint8_t ic = 0u; ic < dev->num_ics; ic++)
	{
		uint8_t *packet = &buf[(uint16_t)ic * RX_DATA];
		uint8_t device_id = (uint8_t)((packet[5] >> 1u) & 0x3Fu);

		memcpy(dev->ics[ic].sid.sid, packet, RSID);
		if(device_id != ADBMS2950B_DEVICE_ID)
		{
			dev->health.sid_valid = false;
			dev->health.device_id = device_id;
			dev->health.last_status = HAL_ERROR;
			return HAL_ERROR;
		}
	}

	memcpy(dev->health.sid, dev->ics[0].sid.sid, RSID);
	dev->health.device_id = ADBMS2950B_DEVICE_ID;
	dev->health.sid_valid = true;
	dev->health.last_status = HAL_OK;
	return HAL_OK;
}

HAL_StatusTypeDef adbms2950_read_status(adbms2950_driver_t *dev)
{
	HAL_StatusTypeDef status;

	if(!adbms2950_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	status = adbms2950_rd48_checked(dev, RDSTAT, buf);
	if(status != HAL_OK)
	{
		dev->health.i1_calibrated = false;
		dev->health.last_status = status;
		return status;
	}

	dev->health.i1_calibrated = ((buf[1] & ADBMS2950_I1_CAL_MASK) != 0u);
	dev->health.revision = (uint8_t)((buf[5] >> 4u) & 0x0Fu);
	dev->health.last_status = HAL_OK;
	return HAL_OK;
}

HAL_StatusTypeDef adbms2950_read_primary_sample(adbms2950_driver_t *dev,
												 uint32_t now_ms)
{
	HAL_StatusTypeDef status;
	uint32_t raw_i1;
	uint16_t raw_vb1;
	int32_t signed_i1;
	int16_t signed_vb1;
	uint8_t status_counter;
	uint8_t counter;

	if(!adbms2950_topology_valid(dev) || !dev->health.initialized)
	{
		return HAL_ERROR;
	}

	status = adbms2950_read_status(dev);
	if((status != HAL_OK) || !dev->health.i1_calibrated)
	{
		dev->health.sample_valid = false;
		dev->health.current_valid = false;
		dev->health.pack_voltage_valid = false;
		dev->health.last_status = (status == HAL_OK) ? HAL_ERROR : status;
		adbms2950_sat_inc_u32(&dev->health.sample_error_count);
		return dev->health.last_status;
	}
	status_counter = dev->ics[0].rx_cmd_cntr;

	status = adbms2950_rd48_checked(dev, RDIVB1, buf);
	if(status != HAL_OK)
	{
		dev->health.sample_valid = false;
		dev->health.current_valid = false;
		dev->health.pack_voltage_valid = false;
		dev->health.last_status = status;
		adbms2950_sat_inc_u32(&dev->health.sample_error_count);
		return status;
	}

	/* Read commands do not advance the device command counter.  RDSTAT and
	 * RDIVB1 are consecutive reads under the same driver lock, so a mismatch
	 * proves the two packets are not one coherent sample. */
	counter = dev->ics[0].rx_cmd_cntr;
	if((counter == 0u) || (counter != status_counter))
	{
		dev->health.sample_valid = false;
		dev->health.current_valid = false;
		dev->health.pack_voltage_valid = false;
		dev->health.counter_advanced = false;
		dev->health.last_status = HAL_ERROR;
		adbms2950_sat_inc_u32(&dev->health.counter_mismatch_count);
		adbms2950_sat_inc_u32(&dev->health.sample_error_count);
		return HAL_ERROR;
	}

	raw_i1 = (uint32_t)buf[0] |
	         ((uint32_t)buf[1] << 8u) |
	         ((uint32_t)buf[2] << 16u);
	raw_vb1 = (uint16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8u));
	if((raw_i1 == ADBMS2950_I1_RESET_CODE) ||
	   (raw_i1 == ADBMS2950_I1_CLEAR_CODE) ||
	   (raw_vb1 == ADBMS2950_VB1_RESET_CODE) ||
	   (raw_vb1 == ADBMS2950_VB1_CLEAR_CODE))
	{
		dev->health.sample_valid = false;
		dev->health.current_valid = false;
		dev->health.pack_voltage_valid = false;
		dev->health.last_status = HAL_ERROR;
		adbms2950_sat_inc_u32(&dev->health.sample_error_count);
		return HAL_ERROR;
	}

	signed_i1 = adbms2950_sign_extend_24(raw_i1);
	signed_vb1 = (int16_t)raw_vb1;
	if(dev->health.counter_seen)
	{
		dev->health.counter_advanced = (counter != 0u) &&
		                               (counter != dev->health.last_cmd_counter);
		if(!dev->health.counter_advanced)
		{
			adbms2950_sat_inc_u32(&dev->health.counter_stall_count);
		}
	}
	else
	{
		dev->health.counter_seen = true;
		dev->health.counter_advanced = (counter != 0u);
	}
	dev->health.last_cmd_counter = counter;

	dev->ics[0].ivbat.i1 = (uint32_t)signed_i1;
	dev->ics[0].ivbat.vbat1 = raw_vb1;
	dev->vi_adc[0] = (float)signed_i1 * VI1_SCALE;
	dev->current[0] = dev->vi_adc[0] * CURRENT_R_SCALE;
	dev->vbat_adc[0] = (float)signed_vb1 * VBAT1_SCALE;
	dev->vbat[0] = dev->vbat_adc[0] * VBAT_DIV_SCALE;

	dev->health.i1_raw = signed_i1;
	dev->health.vb1_raw = signed_vb1;
	dev->health.current_a = dev->current[0];
	dev->health.pack_voltage_v = dev->vbat[0];
	dev->health.current_valid = true;
	dev->health.pack_voltage_valid = dev->health.hv_dividers_enabled;
	dev->health.sample_valid = true;
	dev->health.last_update_ms = now_ms;
	dev->health.last_status = HAL_OK;
	adbms2950_sat_inc_u32(&dev->health.sample_count);
	return HAL_OK;
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
	if(adbms2950_rd48_checked(dev, RDVB, buf) == HAL_OK)
	{
		adbms2950_parse_rdvb(dev, buf);
	}
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
	if(adbms2950_rd48_checked(dev, RDI, buf) == HAL_OK)
	{
		adbms2950_parse_rdi(dev, buf);
	}
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
	if(adbms2950_rd48_checked(dev, RDV1D, buf) == HAL_OK)
	{
		adbms2950_parse_rdv1d(dev, buf);
	}
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
	(void)adbms2950_wakeup_checked(dev);
}

HAL_StatusTypeDef adbms2950_wakeup_checked(adbms2950_driver_t *dev)
{
	HAL_StatusTypeDef status;

	if(!adbms2950_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	for(uint8_t i = 0; i < dev->num_ics; i++)
	{
		adbms2950_set_cs(dev, 0);
		status = adbms2950_us_delay(dev, WAKEUP_US_DELAY);
		if(status != HAL_OK)
		{
			adbms2950_set_cs(dev, 1);
			return status;
		}
		adbms2950_set_cs(dev, 1);
		status = adbms2950_us_delay(dev, WAKEUP_BW_DELAY);
		if(status != HAL_OK)
		{
			return status;
		}
	}
	return HAL_OK;
}

void adbms2950_set_cs(adbms2950_driver_t* dev, uint8_t state)
{
	if((dev == NULL) ||
	   ((int)dev->string < (int)STRING_A) ||
	   (dev->string > STRING_B) ||
	   (dev->cs_port[dev->string] == NULL) ||
	   (dev->cs_pin[dev->string] == 0u))
	{
		return;
	}

	HAL_GPIO_WritePin(dev->cs_port[dev->string], dev->cs_pin[dev->string], state);
}

HAL_StatusTypeDef adbms2950_us_delay(adbms2950_driver_t* dev, uint16_t microseconds)
{
	uint32_t max_spins;
	uint32_t spins = 0u;

	if((dev == NULL) || (dev->htim == NULL) || (dev->htim->Instance == NULL))
	{
		if(dev != NULL)
		{
			dev->delay_last_status = HAL_ERROR;
		}
		return HAL_ERROR;
	}

	__HAL_TIM_SET_COUNTER(dev->htim, 0);
	max_spins = ADBMS2950_DELAY_BASE_SPINS +
	            ((uint32_t)microseconds * ADBMS2950_DELAY_SPINS_PER_US);
	while(__HAL_TIM_GET_COUNTER(dev->htim) < microseconds)
	{
#if AMS_HOST_TEST
		/* The host HAL has no running timer peripheral.  Advance the fake CNT
		 * explicitly so production timeout logic is still executed in tests. */
		dev->htim->Instance->CNT++;
#endif
		spins++;
		if(spins >= max_spins)
		{
			dev->delay_last_status = HAL_TIMEOUT;
			adbms2950_sat_inc_u32(&dev->delay_timeout_count);
			return HAL_TIMEOUT;
		}
	}
	dev->delay_last_status = HAL_OK;
	return HAL_OK;
}


HAL_StatusTypeDef adbms2950_spi_write(adbms2950_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs)
{
	HAL_StatusTypeDef status;

	if((dev == NULL) || (dev->hspi == NULL) || (data == NULL) || (len == 0u) ||
	   (use_cs &&
	    (((int)dev->string < (int)STRING_A) ||
	     (dev->string > STRING_B) ||
	     (dev->cs_port[dev->string] == NULL) ||
	     (dev->cs_pin[dev->string] == 0u))))
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
	   (rx_data == NULL) || (tx_len == 0u) || (rx_len == 0u) ||
	   (use_cs &&
	    (((int)dev->string < (int)STRING_A) ||
	     (dev->string > STRING_B) ||
	     (dev->cs_port[dev->string] == NULL) ||
	     (dev->cs_pin[dev->string] == 0u))))
	{
		return HAL_ERROR;
	}

	if((tx_len > BUFSZ) || (rx_len > BUFSZ) ||
	   (((uint32_t)tx_len + (uint32_t)rx_len) > BUFSZ))
	{
		return HAL_ERROR;
	}
	total_len = (uint16_t)(tx_len + rx_len);

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
