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
#include <math.h>

static uint8_t buf[BUFSZ] = {0};
static uint8_t wrbuf[BUFSZ] = {0};
static uint8_t adbms2950_spi_txrx_tx_buf[BUFSZ] = {0};
static uint8_t adbms2950_spi_txrx_rx_buf[BUFSZ] = {0};

#define ADBMS2950_SPI_DUMMY_BYTE 0xFFu
#define ADBMS2950_DELAY_SPINS_PER_US 256u
#define ADBMS2950_DELAY_BASE_SPINS 1024u
#define ADBMS2950_I1_CAL_MASK 0x40u
#define ADBMS2950_I2_CAL_MASK 0x80u
#define ADBMS2950_CS_SETUP_HOLD_US 2u
#define ADBMS2950_REFUP_RETRY_DELAY_US 5000u
#define ADBMS2950_REFUP_MAX_ATTEMPTS 3u
#define ADBMS2950_REDUNDANT_INIT_WAIT_US 160000u
#define ADBMS2950_REDUNDANT_SETTLE_US 2000u
#define ADBMS2950_COMM_BYTES 3u
#define ADBMS2950_ICOM_START 0x6u
#define ADBMS2950_ICOM_BLANK 0x0u
#define ADBMS2950_ICOM_STOP 0x1u
#define ADBMS2950_FCOM_RELEASE_ACK 0x8u
#define ADBMS2950_FCOM_NACK_STOP 0x9u
#define ADBMS2950_FCOM_READBACK_ACK 0x7u
#define ADBMS2950_FCOM_READBACK_NACK 0xFu
#define ADBMS2950_I1_INITIALIZE_OPT 0x00u
#define ADBMS2950_I1_CONTINUOUS_MIXED_OPT 0x0Cu
#define ADBMS2950_I1_INIT_WAIT_US 160000u
#define ADBMS2950_I1_INIT_MIN_COUNT 136u
#define ADBMS2950_I1_CONTINUOUS_VERIFY_WAIT_US 2000u
#define ADBMS2950_LONG_DELAY_CHUNK_US 60000u

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

/* RDALL* commands are intentionally unavailable in this mixed isoSPI driver.
 * The ADBMS2950B does not pass their complete response through a daisy chain;
 * use the individual 48-bit read commands above. */

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
uint8_t STCOMM_CMD[2]     = { 0x07, 0x23 };

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

/* Writable-bit masks from the ADBMS2950B CFGA/CFGB register maps.  CFGA
 * SNAPST/REFUP are live read-only state, and all reserved bits are ignored on
 * comparison. */
static const uint8_t adbms2950_cfga_compare_mask[TX_DATA] =
    {0xFFu, 0xDFu, 0xFFu, 0x7Fu, 0xFFu, 0xCFu};
static const uint8_t adbms2950_cfgb_compare_mask[TX_DATA] =
    {0x7Fu, 0x7Fu, 0x7Fu, 0x0Bu, 0xFFu, 0xFFu};

/* Command PEC is always generated at runtime.  Precomputed command+PEC arrays
 * were removed so stale constants cannot silently diverge from the command map. */
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
static HAL_StatusTypeDef adbms2950_delay_long_checked(adbms2950_driver_t *dev,
                                                       uint32_t microseconds);
static HAL_StatusTypeDef adbms2950_start_i1_continuous_checked(adbms2950_driver_t *dev);
static HAL_StatusTypeDef adbms2950_read_i1_counter_checked(adbms2950_driver_t *dev,
                                                           uint16_t *i1cntpha,
                                                           uint8_t *cmd_counter);
static void adbms2950_invalidate_sample(adbms2950_driver_t *dev,
                                        HAL_StatusTypeDef status,
                                        bool count_error);
static bool adbms2950_calibration_valid(const adbms2950_calibration_t *calibration);
static void adbms2950_ensure_calibration(adbms2950_driver_t *dev);
static float adbms2950_scale_current(const adbms2950_driver_t *dev,
                                     float shunt_voltage_v);
static float adbms2950_scale_vbat1(const adbms2950_driver_t *dev,
                                   int16_t raw);
static float adbms2950_scale_vbat2(const adbms2950_driver_t *dev,
                                   int16_t raw);
static bool adbms2950_masked_equal(const uint8_t *actual,
                                   const uint8_t *expected,
                                   const uint8_t *mask,
                                   uint8_t length);
static void adbms2950_pack_comm_payload(adbms2950_driver_t *dev,
                                        uint8_t *packed);
static void adbms2950_parse_comm_payload(adbms2950_driver_t *dev,
                                         const uint8_t *packed);
static void adbms2950_build_adi1_command(uint8_t rd, uint8_t opt,
                                         uint8_t cmd[CMDSZ]);
static void adbms2950_set_failure(adbms2950_driver_t *dev,
                                  adbms2950_stage_t stage,
                                  adbms2950_reason_t reason,
                                  HAL_StatusTypeDef status);
static adbms2950_reason_t adbms2950_transaction_reason(
    const adbms2950_driver_t *dev);
static uint8_t adbms2950_counter_next(uint8_t current);
static void adbms2950_note_counter_reset(adbms2950_driver_t *dev);
static void adbms2950_note_counter_increment(adbms2950_driver_t *dev);
static void adbms2950_note_observed_counter(adbms2950_driver_t *dev,
                                             uint8_t current_ic,
                                             uint8_t observed_counter,
                                             bool pec_ok);
static bool adbms2950_cmd_resets_counter(const uint8_t cmd[CMDSZ]);

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

static void adbms2950_set_failure(adbms2950_driver_t *dev,
                                  adbms2950_stage_t stage,
                                  adbms2950_reason_t reason,
                                  HAL_StatusTypeDef status)
{
	if(dev == NULL)
	{
		return;
	}
	dev->health.last_stage = stage;
	dev->health.last_reason = reason;
	dev->health.last_status = status;
}

static adbms2950_reason_t adbms2950_transaction_reason(
    const adbms2950_driver_t *dev)
{
	if(dev == NULL)
	{
		return ADBMS2950_REASON_ARGUMENT;
	}
	if(dev->spi_debug.last_read_pec_fail_mask != 0u)
	{
		return ADBMS2950_REASON_PEC;
	}
	if(dev->spi_debug.cmd_counter_mismatch_mask != 0u)
	{
		return ADBMS2950_REASON_COMMAND_COUNTER;
	}
	return ADBMS2950_REASON_TRANSPORT;
}

static uint8_t adbms2950_counter_next(uint8_t current)
{
	current &= 0x3Fu;
	return ((current == 0u) || (current >= 63u)) ? 1u : (uint8_t)(current + 1u);
}

static void adbms2950_note_counter_reset(adbms2950_driver_t *dev)
{
	if(dev == NULL)
	{
		return;
	}
	dev->spi_debug.cmd_counter_expected_mask = 0u;
	dev->spi_debug.cmd_counter_mismatch_mask = 0u;
	for(uint8_t ic = 0u; ic < ADBMS2950_MAX_TRACKED_ICS; ic++)
	{
		dev->spi_debug.expected_cmd_counter[ic] = 0u;
		if(ic < dev->num_ics)
		{
			dev->spi_debug.cmd_counter_expected_mask |= (uint16_t)(1u << ic);
		}
	}
}

void adbms2950_resync_command_counter_tracking(adbms2950_driver_t *dev)
{
	if(dev == NULL)
	{
		return;
	}
	dev->spi_debug.cmd_counter_expected_mask = 0u;
	dev->spi_debug.cmd_counter_mismatch_mask = 0u;
	memset(dev->spi_debug.expected_cmd_counter, 0,
	       sizeof(dev->spi_debug.expected_cmd_counter));
}

static void adbms2950_note_counter_increment(adbms2950_driver_t *dev)
{
	if(dev == NULL)
	{
		return;
	}
	for(uint8_t ic = 0u; (ic < dev->num_ics) &&
	                      (ic < ADBMS2950_MAX_TRACKED_ICS); ic++)
	{
		uint16_t bit = (uint16_t)(1u << ic);
		if((dev->spi_debug.cmd_counter_expected_mask & bit) != 0u)
		{
			dev->spi_debug.expected_cmd_counter[ic] =
				adbms2950_counter_next(dev->spi_debug.expected_cmd_counter[ic]);
		}
	}
}

static void adbms2950_note_observed_counter(adbms2950_driver_t *dev,
                                             uint8_t current_ic,
                                             uint8_t observed_counter,
                                             bool pec_ok)
{
	uint16_t bit;

	if((dev == NULL) || !pec_ok ||
	   (current_ic >= ADBMS2950_MAX_TRACKED_ICS))
	{
		return;
	}
	bit = (uint16_t)(1u << current_ic);
	observed_counter &= 0x3Fu;
	dev->spi_debug.cmd_counter_seen_mask |= bit;
	if((dev->spi_debug.cmd_counter_expected_mask & bit) == 0u)
	{
		dev->spi_debug.expected_cmd_counter[current_ic] = observed_counter;
		dev->spi_debug.cmd_counter_expected_mask |= bit;
		return;
	}
	if((dev->spi_debug.expected_cmd_counter[current_ic] != 0u) &&
	   (observed_counter == 0u))
	{
		dev->spi_debug.unexpected_counter_reset_mask |= bit;
		dev->spi_debug.sticky_unexpected_counter_reset_mask |= bit;
		adbms2950_sat_inc_u32(&dev->health.unexpected_counter_reset_count);
	}
	if(dev->spi_debug.expected_cmd_counter[current_ic] != observed_counter)
	{
		dev->spi_debug.cmd_counter_mismatch_mask |= bit;
		dev->spi_debug.sticky_cmd_counter_mismatch_mask |= bit;
		adbms2950_sat_inc_u32(&dev->spi_debug.cmd_counter_error_count);
		adbms2950_sat_inc_u32(&dev->health.counter_mismatch_count);
		dev->spi_debug.expected_cmd_counter[current_ic] = observed_counter;
	}
	else
	{
		dev->spi_debug.cmd_counter_mismatch_mask &= (uint16_t)~bit;
		dev->spi_debug.unexpected_counter_reset_mask &= (uint16_t)~bit;
	}
}

static bool adbms2950_cmd_resets_counter(const uint8_t cmd[CMDSZ])
{
	return (cmd != NULL) &&
	       ((((cmd[0] == SRST[0]) && (cmd[1] == SRST[1]))) ||
	        (((cmd[0] == RSTCC[0]) && (cmd[1] == RSTCC[1]))));
}

static bool adbms2950_calibration_valid(const adbms2950_calibration_t *calibration)
{
	return (calibration != NULL) &&
	       ((calibration->profile == ADBMS2950_CAL_PROFILE_DER_APM) ||
	        (calibration->profile == ADBMS2950_CAL_PROFILE_EVAL_BASIC) ||
	        (calibration->profile == ADBMS2950_CAL_PROFILE_CUSTOM)) &&
	       isfinite(calibration->shunt_resistance_ohm) &&
	       isfinite(calibration->current_gain) &&
	       isfinite(calibration->current_offset_uv) &&
	       isfinite(calibration->vb1_divider_ratio) &&
	       isfinite(calibration->vb2_divider_ratio) &&
	       (calibration->shunt_resistance_ohm > 0.0f) &&
	       (calibration->shunt_resistance_ohm < 0.01f) &&
	       (calibration->current_gain > 0.0f) &&
	       (calibration->current_gain < 100.0f) &&
	       ((calibration->current_polarity == 1) ||
	        (calibration->current_polarity == -1)) &&
	       (calibration->vb1_divider_ratio > 0.0f) &&
	       (calibration->vb2_divider_ratio > 0.0f);
}

HAL_StatusTypeDef adbms2950_set_calibration(
    adbms2950_driver_t *dev, const adbms2950_calibration_t *calibration)
{
	if((dev == NULL) || !adbms2950_calibration_valid(calibration))
	{
		return HAL_ERROR;
	}
	dev->calibration = *calibration;
	dev->health.sample_valid = false;
	dev->health.current_valid = false;
	dev->health.pack_voltage_valid = false;
	dev->redundant_sample.valid = false;
	return HAL_OK;
}

HAL_StatusTypeDef adbms2950_set_calibration_profile(
    adbms2950_driver_t *dev, adbms2950_calibration_profile_t profile)
{
	adbms2950_calibration_t calibration;

	if(dev == NULL)
	{
		return HAL_ERROR;
	}
	memset(&calibration, 0, sizeof(calibration));
	calibration.profile = profile;
	calibration.current_gain = ADBMS2950_DEFAULT_CURRENT_GAIN;
	calibration.current_offset_uv = ADBMS2950_DEFAULT_CURRENT_OFFSET_UV;
	calibration.current_polarity = ADBMS2950_DEFAULT_CURRENT_POLARITY;

	switch(profile)
	{
	case ADBMS2950_CAL_PROFILE_DER_APM:
		calibration.shunt_resistance_ohm = ADBMS2950_DER_SHUNT_RESISTANCE_OHM;
		calibration.vb1_divider_ratio = ADBMS2950_DER_VB1_DIVIDER_RATIO;
		calibration.vb2_divider_ratio = ADBMS2950_DER_VB2_DIVIDER_RATIO;
		break;
	case ADBMS2950_CAL_PROFILE_EVAL_BASIC:
		calibration.shunt_resistance_ohm = ADBMS2950_EVAL_SHUNT_RESISTANCE_OHM;
		calibration.vb1_divider_ratio = ADBMS2950_EVAL_VB1_DIVIDER_RATIO;
		calibration.vb2_divider_ratio = ADBMS2950_EVAL_VB2_DIVIDER_RATIO;
		break;
	default:
		return HAL_ERROR;
	}
	return adbms2950_set_calibration(dev, &calibration);
}

const adbms2950_calibration_t *adbms2950_calibration_get(
    const adbms2950_driver_t *dev)
{
	return (dev == NULL) ? NULL : &dev->calibration;
}

const char *adbms2950_calibration_profile_str(
    adbms2950_calibration_profile_t profile)
{
	switch(profile)
	{
	case ADBMS2950_CAL_PROFILE_DER_APM: return "DER_APM_100uR";
	case ADBMS2950_CAL_PROFILE_EVAL_BASIC: return "EVAL_BASIC_50uR";
	case ADBMS2950_CAL_PROFILE_CUSTOM: return "CUSTOM";
	default: return "INVALID";
	}
}

const char *adbms2950_stage_str(adbms2950_stage_t stage)
{
	switch(stage)
	{
	case ADBMS2950_STAGE_IDLE: return "IDLE";
	case ADBMS2950_STAGE_VALIDATE: return "VALIDATE";
	case ADBMS2950_STAGE_RESET: return "RESET";
	case ADBMS2950_STAGE_SID: return "SID";
	case ADBMS2950_STAGE_REFUP: return "REFUP";
	case ADBMS2950_STAGE_CFGA_WRITE: return "CFGA_WRITE";
	case ADBMS2950_STAGE_CFGB_WRITE: return "CFGB_WRITE";
	case ADBMS2950_STAGE_CONFIG_READBACK: return "CONFIG_READBACK";
	case ADBMS2950_STAGE_I1_INITIALIZE: return "I1_INITIALIZE";
	case ADBMS2950_STAGE_I1_CONTINUOUS: return "I1_CONTINUOUS";
	case ADBMS2950_STAGE_STATUS: return "STATUS";
	case ADBMS2950_STAGE_FLAG: return "FLAG";
	case ADBMS2950_STAGE_CORE_SNAPSHOT: return "CORE_SNAPSHOT";
	case ADBMS2950_STAGE_SNAPSHOT_RECOVER: return "SNAPSHOT_RECOVER";
	case ADBMS2950_STAGE_SNAPSHOT: return "SNAPSHOT";
	case ADBMS2950_STAGE_SAMPLE: return "SAMPLE";
	case ADBMS2950_STAGE_UNSNAP: return "UNSNAP";
	case ADBMS2950_STAGE_REDUNDANT_START: return "REDUNDANT_START";
	case ADBMS2950_STAGE_REDUNDANT_SAMPLE: return "REDUNDANT_SAMPLE";
	case ADBMS2950_STAGE_RESTORE: return "RESTORE";
	case ADBMS2950_STAGE_EEPROM: return "EEPROM";
	case ADBMS2950_STAGE_RECOVERY: return "RECOVERY";
	default: return "UNKNOWN";
	}
}

const char *adbms2950_reason_str(adbms2950_reason_t reason)
{
	switch(reason)
	{
	case ADBMS2950_REASON_NONE: return "NONE";
	case ADBMS2950_REASON_ARGUMENT: return "ARGUMENT";
	case ADBMS2950_REASON_TOPOLOGY: return "TOPOLOGY";
	case ADBMS2950_REASON_TRANSPORT: return "TRANSPORT";
	case ADBMS2950_REASON_PEC: return "PEC";
	case ADBMS2950_REASON_IDENTITY: return "IDENTITY";
	case ADBMS2950_REASON_REFUP_TIMEOUT: return "REFUP_TIMEOUT";
	case ADBMS2950_REASON_CONFIG_MISMATCH: return "CONFIG_MISMATCH";
	case ADBMS2950_REASON_NOT_INITIALIZED: return "NOT_INITIALIZED";
	case ADBMS2950_REASON_CALIBRATION: return "CALIBRATION";
	case ADBMS2950_REASON_COMMAND_COUNTER: return "COMMAND_COUNTER";
	case ADBMS2950_REASON_CONVERSION_STALL: return "CONVERSION_STALL";
	case ADBMS2950_REASON_SENTINEL: return "SENTINEL";
	case ADBMS2950_REASON_PERIPHERAL_NACK: return "PERIPHERAL_NACK";
	case ADBMS2950_REASON_RESTORE: return "RESTORE";
	default: return "UNKNOWN";
	}
}

static void adbms2950_ensure_calibration(adbms2950_driver_t *dev)
{
	if((dev != NULL) && !adbms2950_calibration_valid(&dev->calibration))
	{
		(void)adbms2950_set_calibration_profile(
			dev, ADBMS2950_CAL_PROFILE_DER_APM);
	}
}

static float adbms2950_scale_current(const adbms2950_driver_t *dev,
                                     float shunt_voltage_v)
{
	float corrected_v;
	if((dev == NULL) || !adbms2950_calibration_valid(&dev->calibration))
	{
		return 0.0f;
	}
	corrected_v = shunt_voltage_v - (dev->calibration.current_offset_uv * 1.0e-6f);
	return ((corrected_v / dev->calibration.shunt_resistance_ohm) *
	        dev->calibration.current_gain *
	        (float)dev->calibration.current_polarity);
}

static float adbms2950_scale_vbat1(const adbms2950_driver_t *dev, int16_t raw)
{
	return (dev == NULL) ? 0.0f :
	       ((float)raw * VBAT1_SCALE * dev->calibration.vb1_divider_ratio);
}

static float adbms2950_scale_vbat2(const adbms2950_driver_t *dev, int16_t raw)
{
	return (dev == NULL) ? 0.0f :
	       ((float)raw * VBAT2_SCALE * dev->calibration.vb2_divider_ratio);
}

static bool adbms2950_masked_equal(const uint8_t *actual,
                                   const uint8_t *expected,
                                   const uint8_t *mask,
                                   uint8_t length)
{
	if((actual == NULL) || (expected == NULL) || (mask == NULL))
	{
		return false;
	}
	for(uint8_t i = 0u; i < length; i++)
	{
		if((actual[i] & mask[i]) != (expected[i] & mask[i]))
		{
			return false;
		}
	}
	return true;
}

static void adbms2950_build_adi1_command(uint8_t rd, uint8_t opt,
                                         uint8_t cmd[CMDSZ])
{
	cmd[0] = (uint8_t)(sADI1[0] | (rd & 0x01u));
	opt &= 0x0Fu;
	cmd[1] = (uint8_t)(sADI1[1] |
	                   ((opt & 0x08u) << 4u) |
	                   ((opt & 0x04u) << 2u) |
	                   (opt & 0x03u));
}

static void adbms2950_pack_comm_payload(adbms2950_driver_t *dev,
                                        uint8_t *packed)
{
	if((dev == NULL) || (packed == NULL))
	{
		return;
	}
	for(uint8_t ic = 0u; ic < dev->num_ics; ic++)
	{
		uint16_t base = (uint16_t)ic * TX_DATA;
		for(uint8_t slot = 0u; slot < ADBMS2950_COMM_BYTES; slot++)
		{
			dev->ics[ic].com.tx_data[(uint8_t)(slot * 2u)] =
				(uint8_t)(((dev->ics[ic].tx_comm.icomm[slot] & 0x0Fu) << 4u) |
				          (dev->ics[ic].tx_comm.fcomm[slot] & 0x0Fu));
			dev->ics[ic].com.tx_data[(uint8_t)(slot * 2u + 1u)] =
				dev->ics[ic].tx_comm.data[slot];
		}
		memcpy(&packed[base], dev->ics[ic].com.tx_data, TX_DATA);
	}
}

static void adbms2950_parse_comm_payload(adbms2950_driver_t *dev,
                                         const uint8_t *packed)
{
	if((dev == NULL) || (packed == NULL))
	{
		return;
	}
	for(uint8_t ic = 0u; ic < dev->num_ics; ic++)
	{
		uint16_t base = (uint16_t)ic * RX_DATA;
		memcpy(dev->ics[ic].com.rx_data, &packed[base], TX_DATA);
		for(uint8_t slot = 0u; slot < ADBMS2950_COMM_BYTES; slot++)
		{
			uint8_t control = dev->ics[ic].com.rx_data[(uint8_t)(slot * 2u)];
			dev->ics[ic].rx_comm.icomm[slot] = (uint8_t)(control >> 4u);
			dev->ics[ic].rx_comm.fcomm[slot] = (uint8_t)(control & 0x0Fu);
			dev->ics[ic].rx_comm.data[slot] =
				dev->ics[ic].com.rx_data[(uint8_t)(slot * 2u + 1u)];
		}
	}
}

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
	       ((int)dev->write_string >= (int)STRING_A) &&
	       (dev->write_string <= STRING_B) &&
	       (dev->cs_port[dev->write_string] != NULL) &&
	       (dev->cs_pin[dev->write_string] != 0u) &&
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

static HAL_StatusTypeDef adbms2950_delay_long_checked(adbms2950_driver_t *dev,
                                                       uint32_t microseconds)
{
	while(microseconds > 0u)
	{
		uint16_t chunk = (microseconds > ADBMS2950_LONG_DELAY_CHUNK_US) ?
		                 (uint16_t)ADBMS2950_LONG_DELAY_CHUNK_US :
		                 (uint16_t)microseconds;
		HAL_StatusTypeDef status = adbms2950_us_delay(dev, chunk);
		if(status != HAL_OK)
		{
			return status;
		}
		microseconds -= chunk;
	}
	return HAL_OK;
}

static HAL_StatusTypeDef adbms2950_read_i1_counter_checked(adbms2950_driver_t *dev,
                                                            uint16_t *i1cntpha,
                                                            uint8_t *cmd_counter)
{
	HAL_StatusTypeDef status;
	uint16_t count;
	uint8_t phase;

	if((i1cntpha == NULL) || (cmd_counter == NULL))
	{
		return HAL_ERROR;
	}

	status = adbms2950_read_flag(dev);
	if(status != HAL_OK)
	{
		return status;
	}

	/* adbms2950_read_flag() is the single FLAG decoder.  Reuse its decoded
	 * values so the freshness path and the CLI cannot silently diverge. */
	count = dev->health.i1_conversion_count;
	phase = dev->health.i1_conversion_phase;
	*i1cntpha = (uint16_t)((count << 2u) | phase);
	*cmd_counter = dev->ics[0].rx_cmd_cntr;
	dev->health.i1_conversion_count = count;
	dev->health.i1_conversion_phase = phase;
	return HAL_OK;
}

static HAL_StatusTypeDef adbms2950_start_i1_continuous_checked(adbms2950_driver_t *dev)
{
	uint8_t initialize_cmd[CMDSZ];
	uint8_t continuous_cmd[CMDSZ];
	uint16_t initialization_counter;
	uint16_t continuous_counter;
	uint8_t status_cmd_counter;
	uint8_t flag_cmd_counter;
	HAL_StatusTypeDef status;

	/* The first ADI1 after power-up/reset must use OPT=0000 while the IxADC
	 * performs its 136-cycle initialization.  Only after I1CAL and I1CNT prove
	 * completion do we select OPT=1100 continuous operation.  OPT=1100 is
	 * intentionally invalid on an ADBMS6830B, so it does not disturb the cell
	 * monitors' existing C/S conversion mode. */
	adbms2950_build_adi1_command(0u, ADBMS2950_I1_INITIALIZE_OPT, initialize_cmd);
	adbms2950_build_adi1_command(0u, ADBMS2950_I1_CONTINUOUS_MIXED_OPT, continuous_cmd);

	dev->health.i1_continuous_ready = false;
	dev->health.counter_seen = false;
	dev->health.counter_advanced = false;
	status = adbms2950_wakeup_checked(dev);
	if(status == HAL_OK)
	{
		status = adbms2950_cmd_checked(dev, initialize_cmd);
	}
	if(status == HAL_OK)
	{
		/* tREFUP + tIxADC_STARTUP + tIxADC_INIT is at most about 155 ms.
		 * The 160 ms bounded wait leaves margin before checking I1CAL and
		 * the documented minimum conversion count of 136. */
		status = adbms2950_delay_long_checked(dev, ADBMS2950_I1_INIT_WAIT_US);
	}
	if(status == HAL_OK)
	{
		status = adbms2950_read_status(dev);
		status_cmd_counter = dev->ics[0].rx_cmd_cntr;
	}
	else
	{
		status_cmd_counter = 0u;
	}
	if(status == HAL_OK)
	{
		status = adbms2950_read_i1_counter_checked(dev,
		                                          &initialization_counter,
		                                          &flag_cmd_counter);
	}
	else
	{
		initialization_counter = 0u;
		flag_cmd_counter = 0u;
	}
	if((status == HAL_OK) &&
	   ((!dev->health.i1_calibrated) ||
	    (dev->health.i1_conversion_count < ADBMS2950_I1_INIT_MIN_COUNT) ||
	    (status_cmd_counter != flag_cmd_counter)))
	{
		status = HAL_ERROR;
	}

	if(status == HAL_OK)
	{
		status = adbms2950_cmd_checked(dev, continuous_cmd);
	}
	if(status == HAL_OK)
	{
		/* One conversion is produced every 1 ms.  Require a nonzero counter
		 * after two periods so a transport-successful command that did not
		 * actually start continuous conversion cannot pass initialization. */
		status = adbms2950_delay_long_checked(
			dev, ADBMS2950_I1_CONTINUOUS_VERIFY_WAIT_US);
	}
	if(status == HAL_OK)
	{
		status = adbms2950_read_status(dev);
		status_cmd_counter = dev->ics[0].rx_cmd_cntr;
	}
	if(status == HAL_OK)
	{
		status = adbms2950_read_i1_counter_checked(dev,
		                                          &continuous_counter,
		                                          &flag_cmd_counter);
	}
	else
	{
		continuous_counter = 0u;
		flag_cmd_counter = 0u;
	}
	if((status == HAL_OK) &&
	   ((!dev->health.i1_calibrated) ||
	    (dev->health.i1_conversion_count == 0u) ||
	    (status_cmd_counter != flag_cmd_counter)))
	{
		status = HAL_ERROR;
	}

	if(status == HAL_OK)
	{
		dev->health.i1_continuous_ready = true;
		dev->health.counter_seen = true;
		dev->health.counter_advanced = true;
		dev->health.last_i1cntpha = continuous_counter;
		dev->health.last_i1_conversion_count = dev->health.i1_conversion_count;
		dev->health.last_cmd_counter = flag_cmd_counter;
	}
	return status;
}

static void adbms2950_invalidate_sample(adbms2950_driver_t *dev,
                                         HAL_StatusTypeDef status,
                                         bool count_error)
{
	dev->health.sample_valid = false;
	dev->health.current_valid = false;
	dev->health.pack_voltage_valid = false;
	dev->health.last_status = status;
	if(count_error)
	{
		adbms2950_sat_inc_u32(&dev->health.sample_error_count);
	}
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

const adbms2950_redundant_sample_t *adbms2950_redundant_sample_get(
    const adbms2950_driver_t *dev)
{
	return (dev == NULL) ? NULL : &dev->redundant_sample;
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
	dev->health.unexpected_counter_reset_count = 0u;
	dev->health.refup_failure_count = 0u;
	dev->health.recovery_count = 0u;
	dev->health.recovery_failure_count = 0u;
	dev->spi_debug.cmd_counter_error_count = 0u;
	dev->spi_debug.sticky_cmd_counter_mismatch_mask = 0u;
	dev->spi_debug.sticky_unexpected_counter_reset_mask = 0u;
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

HAL_StatusTypeDef adbms2950_stcomm_checked(adbms2950_driver_t *dev,
                                           uint8_t comm_byte_count)
{
	uint8_t frame[CMDSZ + PEC15SZ + (ADBMS2950_COMM_BYTES * 3u)];
	uint16_t pec15;
	uint16_t frame_length;
	HAL_StatusTypeDef status;

	if(!adbms2950_topology_valid(dev) ||
	   (comm_byte_count == 0u) ||
	   (comm_byte_count > ADBMS2950_COMM_BYTES))
	{
		return HAL_ERROR;
	}

	frame[0] = STCOMM_CMD[0];
	frame[1] = STCOMM_CMD[1];
	pec15 = Pec15_Calc(CMDSZ, STCOMM_CMD);
	frame[2] = (uint8_t)(pec15 >> 8u);
	frame[3] = (uint8_t)pec15;
	frame_length = (uint16_t)(CMDSZ + PEC15SZ + (3u * comm_byte_count));
	memset(&frame[CMDSZ + PEC15SZ], ADBMS2950_SPI_DUMMY_BYTE,
	       (size_t)(3u * comm_byte_count));

	status = adbms2950_wakeup_checked(dev);
	if(status != HAL_OK)
	{
		return status;
	}
	if(dev->spi_debug.enabled)
	{
		dev->spi_debug.last_op = ADBMS2950_SPI_OP_CMD;
	}
	status = adbms2950_spi_write(dev, frame, frame_length, 1u);
	if(status == HAL_OK)
	{
		adbms2950_note_counter_increment(dev);
	}
	else
	{
		adbms2950_resync_command_counter_tracking(dev);
	}
	return status;
}

HAL_StatusTypeDef adbms2950_i2c_write_probe(adbms2950_driver_t *dev,
                                            uint8_t address_7bit,
                                            uint8_t data_byte,
                                            adbms2950_i2c_probe_result_t *result)
{
	uint8_t payload[BUFSZ];
	HAL_StatusTypeDef status;

	if(result != NULL)
	{
		memset(result, 0, sizeof(*result));
		result->transport_status = HAL_ERROR;
		result->address_7bit = address_7bit;
		result->write_data = data_byte;
	}
	if(!adbms2950_topology_valid(dev) || (result == NULL) ||
	   (address_7bit > 0x7Fu))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_EEPROM,
		                      ADBMS2950_REASON_ARGUMENT, HAL_ERROR);
		return HAL_ERROR;
	}
	dev->health.last_stage = ADBMS2950_STAGE_EEPROM;

	for(uint8_t ic = 0u; ic < dev->num_ics; ic++)
	{
		dev->ics[ic].tx_comm.icomm[0] = ADBMS2950_ICOM_START;
		dev->ics[ic].tx_comm.fcomm[0] = ADBMS2950_FCOM_RELEASE_ACK;
		dev->ics[ic].tx_comm.data[0] = (uint8_t)(address_7bit << 1u);
		dev->ics[ic].tx_comm.icomm[1] = ADBMS2950_ICOM_BLANK;
		dev->ics[ic].tx_comm.fcomm[1] = ADBMS2950_FCOM_RELEASE_ACK;
		dev->ics[ic].tx_comm.data[1] = data_byte;
		dev->ics[ic].tx_comm.icomm[2] = ADBMS2950_ICOM_STOP;
		dev->ics[ic].tx_comm.fcomm[2] = ADBMS2950_FCOM_NACK_STOP;
		dev->ics[ic].tx_comm.data[2] = ADBMS2950_SPI_DUMMY_BYTE;
	}
	memset(payload, 0, sizeof(payload));
	adbms2950_pack_comm_payload(dev, payload);
	status = adbms2950_wr48_checked(dev, WRCOMM, payload);
	if(status == HAL_OK)
	{
		status = adbms2950_rd48_checked(dev, RDCOMM, payload);
	}
	if(status == HAL_OK)
	{
		memcpy(result->pre_rdcomm, payload, TX_DATA);
		status = adbms2950_stcomm_checked(dev, ADBMS2950_COMM_BYTES);
	}
	if(status == HAL_OK)
	{
		status = adbms2950_rd48_checked(dev, RDCOMM, payload);
	}
	if(status == HAL_OK)
	{
		memcpy(result->post_rdcomm, payload, TX_DATA);
		adbms2950_parse_comm_payload(dev, payload);
		result->address_ack =
			(dev->ics[0].rx_comm.fcomm[0] == ADBMS2950_FCOM_READBACK_ACK);
		result->data_ack =
			(dev->ics[0].rx_comm.fcomm[1] == ADBMS2950_FCOM_READBACK_ACK);
	}
	result->transport_status = status;
	if(status != HAL_OK)
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_EEPROM,
		                      adbms2950_transaction_reason(dev), status);
		return status;
	}
	if(!result->address_ack || !result->data_ack)
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_EEPROM,
		                      ADBMS2950_REASON_PERIPHERAL_NACK, HAL_ERROR);
		return HAL_ERROR;
	}
	dev->health.last_reason = ADBMS2950_REASON_NONE;
	dev->health.last_status = HAL_OK;
	return HAL_OK;
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
	(void)adbms2950_set_calibration_profile(dev, ADBMS2950_CAL_PROFILE_DER_APM);
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
	dev->write_string = primary_string;
	dev->delay_last_status = (htim != NULL) ? HAL_OK : HAL_ERROR;
	dev->spi_debug.enabled = true;
	dev->spi_debug.last_status = HAL_ERROR;
	dev->spi_debug.last_tx_status = HAL_ERROR;
	dev->spi_debug.last_rx_status = HAL_ERROR;
	dev->spi_debug.last_xfer_status = HAL_ERROR;
	dev->health.last_status = HAL_ERROR;
	dev->health.last_stage = ADBMS2950_STAGE_VALIDATE;
	dev->health.last_reason = ADBMS2950_REASON_NONE;
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
		adbms2950_set_failure(dev, ADBMS2950_STAGE_VALIDATE,
		                      ADBMS2950_REASON_ARGUMENT, HAL_ERROR);
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
		dev->health.last_stage = ADBMS2950_STAGE_RESET;
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
			adbms2950_set_failure(dev, ADBMS2950_STAGE_RESET,
			                      ADBMS2950_REASON_TRANSPORT, status);
			return status;
		}
	}

	/* RDCFGA cannot distinguish a 2950 from a compatible cell monitor.  SID
	 * derivative bits must identify an ADBMS2950B before any APM-specific
	 * configuration is written into the mixed chain. */
	dev->health.last_stage = ADBMS2950_STAGE_SID;
	status = adbms2950_read_sid(dev);
	if(status != HAL_OK)
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_SID,
		                      ADBMS2950_REASON_IDENTITY, status);
		return status;
	}

	/* The 2950 acknowledges ADC commands while still in STANDBY, but does not
	 * actually start conversions.  Verify REFUP explicitly before the first
	 * ADI1 so a transport-successful but electrically unready device cannot
	 * pass initialization. */
	status = adbms2950_verify_refup(dev);
	if(status != HAL_OK)
	{
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

	dev->health.last_stage = ADBMS2950_STAGE_CFGA_WRITE;
	status = adbms2950_wrcfga_checked(dev);
	if(status == HAL_OK)
	{
		dev->health.last_stage = ADBMS2950_STAGE_CFGB_WRITE;
		status = adbms2950_wrcfgb_checked(dev);
	}
	if(status == HAL_OK)
	{
		dev->health.last_stage = ADBMS2950_STAGE_CONFIG_READBACK;
		status = adbms2950_verify_config_readback(dev);
	}
	if(status == HAL_OK)
	{
		dev->health.last_stage = ADBMS2950_STAGE_I1_INITIALIZE;
		/* Establish I1ADC accuracy before the 10 Hz cell-monitor loop begins.
		 * Otherwise each compatible ADCV/ADI1 can arrive before the 136-cycle
		 * initialization window completes and I1CAL may never become valid. */
		status = adbms2950_start_i1_continuous_checked(dev);
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
	if(status == HAL_OK)
	{
		dev->health.last_stage = ADBMS2950_STAGE_I1_CONTINUOUS;
		dev->health.last_reason = ADBMS2950_REASON_NONE;
	}
	else if(dev->health.last_reason == ADBMS2950_REASON_NONE)
	{
		adbms2950_set_failure(dev, dev->health.last_stage,
		                      ADBMS2950_REASON_TRANSPORT, status);
	}
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

	{
		HAL_StatusTypeDef status = adbms2950_spi_write(dev, wrbuf, CMDSZ + PEC15SZ, 1);
		if(status == HAL_OK)
		{
			if(adbms2950_cmd_resets_counter(cmd))
			{
				adbms2950_note_counter_reset(dev);
			}
			else
			{
				adbms2950_note_counter_increment(dev);
			}
		}
		else
		{
			adbms2950_resync_command_counter_tracking(dev);
		}
		return status;
	}
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

	if(!adbms2950_topology_valid(dev) ||
	   (dev->string != dev->write_string) ||
	   (cmd == NULL) ||
	   (tx_data == NULL))
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

	status = adbms2950_spi_write(dev, wrbuf, tx_sz, 1);
	if(status == HAL_OK)
	{
		adbms2950_note_counter_increment(dev);
	}
	else
	{
		adbms2950_resync_command_counter_tracking(dev);
	}
	return status;
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
	  adbms2950_note_observed_counter(dev, current_ic, cmd_counter,
	                                  !dev->ics[current_ic].rx_pec_error);
	  if((dev->spi_debug.cmd_counter_mismatch_mask &
	      (uint16_t)(1u << current_ic)) != 0u)
	  {
		integrity_ok = false;
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
		adbms2950_set_failure(dev, ADBMS2950_STAGE_CONFIG_READBACK,
		                      ADBMS2950_REASON_TOPOLOGY, HAL_ERROR);
		return HAL_ERROR;
	}
	dev->health.last_stage = ADBMS2950_STAGE_CONFIG_READBACK;

	adbms2950_pack_cfga(dev);
	status = adbms2950_rd48_checked(dev, RDCFGA, buf);
	if(status != HAL_OK)
	{
		dev->health.config_valid = false;
		adbms2950_set_failure(dev, ADBMS2950_STAGE_CONFIG_READBACK,
		                      adbms2950_transaction_reason(dev), status);
		return status;
	}
	dev->health.configa_mismatch_ic_mask = 0u;
	for(uint8_t ic = 0u; ic < dev->num_ics; ic++)
	{
		if(!adbms2950_masked_equal(&buf[(uint16_t)ic * RX_DATA],
		                            dev->ics[ic].configa.tx_data,
		                            adbms2950_cfga_compare_mask,
		                            TX_DATA))
		{
			dev->health.configa_mismatch_ic_mask |= (uint16_t)(1u << ic);
		}
	}
	if(dev->health.configa_mismatch_ic_mask != 0u)
	{
		dev->health.config_valid = false;
		adbms2950_set_failure(dev, ADBMS2950_STAGE_CONFIG_READBACK,
		                      ADBMS2950_REASON_CONFIG_MISMATCH, HAL_ERROR);
		return HAL_ERROR;
	}
	adbms2950_parse_cfga(dev, buf);

	adbms2950_pack_cfgb(dev);
	status = adbms2950_rd48_checked(dev, RDCFGB, buf);
	if(status != HAL_OK)
	{
		dev->health.config_valid = false;
		adbms2950_set_failure(dev, ADBMS2950_STAGE_CONFIG_READBACK,
		                      adbms2950_transaction_reason(dev), status);
		return status;
	}
	dev->health.configb_mismatch_ic_mask = 0u;
	for(uint8_t ic = 0u; ic < dev->num_ics; ic++)
	{
		if(!adbms2950_masked_equal(&buf[(uint16_t)ic * RX_DATA],
		                            dev->ics[ic].configb.tx_data,
		                            adbms2950_cfgb_compare_mask,
		                            TX_DATA))
		{
			dev->health.configb_mismatch_ic_mask |= (uint16_t)(1u << ic);
		}
	}
	if(dev->health.configb_mismatch_ic_mask != 0u)
	{
		dev->health.config_valid = false;
		adbms2950_set_failure(dev, ADBMS2950_STAGE_CONFIG_READBACK,
		                      ADBMS2950_REASON_CONFIG_MISMATCH, HAL_ERROR);
		return HAL_ERROR;
	}
	adbms2950_parse_cfgb(dev, buf);
	dev->health.config_valid = true;
	dev->health.last_reason = ADBMS2950_REASON_NONE;
	dev->health.last_status = HAL_OK;
	return HAL_OK;
}

HAL_StatusTypeDef adbms2950_read_sid(adbms2950_driver_t *dev)
{
	HAL_StatusTypeDef status;

	if(!adbms2950_topology_valid(dev))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_SID,
		                      ADBMS2950_REASON_TOPOLOGY, HAL_ERROR);
		return HAL_ERROR;
	}
	dev->health.last_stage = ADBMS2950_STAGE_SID;

	status = adbms2950_rd48_checked(dev, RDSID, buf);
	if(status != HAL_OK)
	{
		dev->health.sid_valid = false;
		adbms2950_set_failure(dev, ADBMS2950_STAGE_SID,
		                      adbms2950_transaction_reason(dev), status);
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
			adbms2950_set_failure(dev, ADBMS2950_STAGE_SID,
			                      ADBMS2950_REASON_IDENTITY, HAL_ERROR);
			return HAL_ERROR;
		}
	}

	memcpy(dev->health.sid, dev->ics[0].sid.sid, RSID);
	dev->health.device_id = ADBMS2950B_DEVICE_ID;
	dev->health.sid_valid = true;
	dev->health.last_reason = ADBMS2950_REASON_NONE;
	dev->health.last_status = HAL_OK;
	return HAL_OK;
}

HAL_StatusTypeDef adbms2950_read_status(adbms2950_driver_t *dev)
{
	HAL_StatusTypeDef status;

	if(!adbms2950_topology_valid(dev))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_STATUS,
		                      ADBMS2950_REASON_TOPOLOGY, HAL_ERROR);
		return HAL_ERROR;
	}

	dev->health.last_stage = ADBMS2950_STAGE_STATUS;
	status = adbms2950_rd48_checked(dev, RDSTAT, buf);
	if(status != HAL_OK)
	{
		dev->health.i1_calibrated = false;
		dev->health.i2_calibrated = false;
		dev->health.status_valid = false;
		adbms2950_set_failure(dev, ADBMS2950_STAGE_STATUS,
		                      adbms2950_transaction_reason(dev),
		                      status);
		return status;
	}

	memcpy(dev->health.raw_status, buf, TX_DATA);
	dev->health.i1_calibrated = ((buf[1] & ADBMS2950_I1_CAL_MASK) != 0u);
	dev->health.i2_calibrated = ((buf[1] & ADBMS2950_I2_CAL_MASK) != 0u);
	dev->health.derivative = (uint8_t)(buf[1] & 0x03u);
	dev->health.revision = (uint8_t)((buf[5] >> 4u) & 0x0Fu);
	dev->health.status_valid = (dev->health.derivative == 0u);
	if(!dev->health.status_valid)
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_STATUS,
		                      ADBMS2950_REASON_IDENTITY, HAL_ERROR);
		return HAL_ERROR;
	}
	dev->health.last_reason = ADBMS2950_REASON_NONE;
	dev->health.last_status = HAL_OK;
	return HAL_OK;
}

HAL_StatusTypeDef adbms2950_read_flag(adbms2950_driver_t *dev)
{
	HAL_StatusTypeDef status;
	uint32_t faults = 0u;

	if(!adbms2950_topology_valid(dev))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_FLAG,
		                      ADBMS2950_REASON_TOPOLOGY, HAL_ERROR);
		return HAL_ERROR;
	}
	dev->health.last_stage = ADBMS2950_STAGE_FLAG;
	status = adbms2950_rd48_checked(dev, RDFLAG, buf);
	if(status != HAL_OK)
	{
		dev->health.flag_valid = false;
		adbms2950_set_failure(dev, ADBMS2950_STAGE_FLAG,
		                      adbms2950_transaction_reason(dev),
		                      status);
		return status;
	}
	memcpy(dev->health.raw_flag, buf, TX_DATA);
	dev->health.i2_conversion_count = (uint8_t)((buf[2] >> 5u) & 0x07u);
	dev->health.i1_conversion_count =
		(uint16_t)(((uint16_t)(buf[2] & 0x1Fu) << 6u) |
		           ((uint16_t)(buf[3] >> 2u) & 0x3Fu));
	dev->health.i1_conversion_phase = (uint8_t)(buf[3] & 0x03u);
	if((buf[0] & 0x40u) != 0u) faults |= (1u << 0);  /* VDRUV */
	if((buf[0] & 0x20u) != 0u) faults |= (1u << 1);  /* OCMM */
	if((buf[0] & 0x10u) != 0u) faults |= (1u << 2);  /* OC3L */
	if((buf[0] & 0x08u) != 0u) faults |= (1u << 3);  /* OCAGD */
	if((buf[0] & 0x04u) != 0u) faults |= (1u << 4);  /* OCAL */
	if((buf[0] & 0x02u) != 0u) faults |= (1u << 5);  /* OC1L */
	if((buf[1] & 0x40u) != 0u) faults |= (1u << 6);  /* VDDUV */
	if((buf[1] & 0x20u) != 0u) faults |= (1u << 7);  /* NOCLK */
	if((buf[1] & 0x10u) != 0u) faults |= (1u << 8);  /* REFFLT */
	if((buf[1] & 0x08u) != 0u) faults |= (1u << 9);  /* OCBGD */
	if((buf[1] & 0x04u) != 0u) faults |= (1u << 10); /* OCBL */
	if((buf[1] & 0x02u) != 0u) faults |= (1u << 11); /* OC2L */
	if((buf[4] & 0x80u) != 0u) faults |= (1u << 12); /* VREGOV */
	if((buf[4] & 0x40u) != 0u) faults |= (1u << 13); /* VREGUV */
	if((buf[4] & 0x20u) != 0u) faults |= (1u << 14); /* VDIGOV */
	if((buf[4] & 0x10u) != 0u) faults |= (1u << 15); /* VDIGUV */
	if((buf[4] & 0x08u) != 0u) faults |= (1u << 16); /* SED1 */
	if((buf[4] & 0x04u) != 0u) faults |= (1u << 17); /* MED1 */
	if((buf[4] & 0x02u) != 0u) faults |= (1u << 18); /* SED2 */
	if((buf[4] & 0x01u) != 0u) faults |= (1u << 19); /* MED2 */
	if((buf[5] & 0x80u) != 0u) faults |= (1u << 20); /* VDEL */
	if((buf[5] & 0x40u) != 0u) faults |= (1u << 21); /* VDE */
	if((buf[5] & 0x10u) != 0u) faults |= (1u << 22); /* SPIFLT */
	if((buf[5] & 0x08u) != 0u) faults |= (1u << 23); /* RESET */
	if((buf[5] & 0x04u) != 0u) faults |= (1u << 24); /* THSD */
	if((buf[5] & 0x02u) != 0u) faults |= (1u << 25); /* TMODE */
	if((buf[5] & 0x01u) != 0u) faults |= (1u << 26); /* OSCFLT */
	dev->health.fault_mask = faults;
	dev->health.flag_valid = true;
	dev->health.last_reason = ADBMS2950_REASON_NONE;
	dev->health.last_status = HAL_OK;
	return HAL_OK;
}

HAL_StatusTypeDef adbms2950_verify_refup(adbms2950_driver_t *dev)
{
	HAL_StatusTypeDef status = HAL_ERROR;

	if(!adbms2950_topology_valid(dev))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_REFUP,
		                      ADBMS2950_REASON_TOPOLOGY, HAL_ERROR);
		return HAL_ERROR;
	}
	dev->health.refup_valid = false;
	dev->health.refup = false;
	for(uint8_t attempt = 0u; attempt < ADBMS2950_REFUP_MAX_ATTEMPTS; attempt++)
	{
		dev->health.last_stage = ADBMS2950_STAGE_REFUP;
		status = adbms2950_rd48_checked(dev, RDCFGA, buf);
		if(status == HAL_OK)
		{
			bool all_refup = true;
			adbms2950_parse_cfga(dev, buf);
			for(uint8_t ic = 0u; ic < dev->num_ics; ic++)
			{
				if(dev->ics[ic].rx_cfga.refup == 0u)
				{
					all_refup = false;
				}
			}
			dev->health.refup_valid = true;
			dev->health.refup = all_refup;
			if(all_refup)
			{
				dev->health.last_reason = ADBMS2950_REASON_NONE;
				dev->health.last_status = HAL_OK;
				return HAL_OK;
			}
		}
		if(attempt + 1u < ADBMS2950_REFUP_MAX_ATTEMPTS)
		{
			HAL_StatusTypeDef delay_status =
				adbms2950_us_delay(dev, ADBMS2950_REFUP_RETRY_DELAY_US);
			if(delay_status != HAL_OK)
			{
				status = delay_status;
				break;
			}
		}
	}
	adbms2950_sat_inc_u32(&dev->health.refup_failure_count);
	adbms2950_set_failure(dev, ADBMS2950_STAGE_REFUP,
	                      (status == HAL_OK) ? ADBMS2950_REASON_REFUP_TIMEOUT :
	                      ADBMS2950_REASON_TRANSPORT,
	                      (status == HAL_OK) ? HAL_ERROR : status);
	return (status == HAL_OK) ? HAL_ERROR : status;
}

HAL_StatusTypeDef adbms2950_read_core_snapshot(adbms2950_driver_t *dev,
                                                adbms2950_core_snapshot_t *snapshot)
{
	HAL_StatusTypeDef status;
	uint8_t counters[5] = {0};
	bool counter_epoch_mismatch = false;

	if((dev == NULL) || (snapshot == NULL) || !adbms2950_topology_valid(dev))
	{
		return HAL_ERROR;
	}
	memset(snapshot, 0, sizeof(*snapshot));
	snapshot->last_status = HAL_ERROR;
	dev->health.last_stage = ADBMS2950_STAGE_CORE_SNAPSHOT;
	dev->health.last_reason = ADBMS2950_REASON_NONE;
	status = adbms2950_rd48_checked(dev, RDCFGA, buf);
	if(status == HAL_OK) { memcpy(snapshot->cfga, buf, TX_DATA); counters[0] = dev->ics[0].rx_cmd_cntr; snapshot->cfga_ccnt = counters[0]; }
	if(status == HAL_OK) { status = adbms2950_rd48_checked(dev, RDCFGB, buf); }
	if(status == HAL_OK) { memcpy(snapshot->cfgb, buf, TX_DATA); counters[1] = dev->ics[0].rx_cmd_cntr; snapshot->cfgb_ccnt = counters[1]; }
	if(status == HAL_OK) { status = adbms2950_read_status(dev); }
	if(status == HAL_OK) { memcpy(snapshot->status, dev->health.raw_status, TX_DATA); counters[2] = dev->ics[0].rx_cmd_cntr; snapshot->status_ccnt = counters[2]; }
	if(status == HAL_OK) { status = adbms2950_read_flag(dev); }
	if(status == HAL_OK) { memcpy(snapshot->flag, dev->health.raw_flag, TX_DATA); counters[3] = dev->ics[0].rx_cmd_cntr; snapshot->flag_ccnt = counters[3]; }
	if(status == HAL_OK) { status = adbms2950_read_sid(dev); }
	if(status == HAL_OK) { memcpy(snapshot->sid, dev->health.sid, TX_DATA); counters[4] = dev->ics[0].rx_cmd_cntr; snapshot->sid_ccnt = counters[4]; }
	if(status == HAL_OK)
	{
		for(uint8_t i = 1u; i < 5u; i++)
		{
			if(counters[i] != counters[0])
			{
				status = HAL_ERROR;
				counter_epoch_mismatch = true;
				adbms2950_sat_inc_u32(&dev->health.counter_mismatch_count);
				break;
			}
		}
	}
	snapshot->valid = (status == HAL_OK);
	snapshot->last_status = status;
	dev->core_snapshot = *snapshot;
	if(status != HAL_OK)
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_CORE_SNAPSHOT,
		                      counter_epoch_mismatch ?
		                      ADBMS2950_REASON_COMMAND_COUNTER :
		                      adbms2950_transaction_reason(dev),
		                      status);
	}
	else
	{
		dev->health.last_stage = ADBMS2950_STAGE_CORE_SNAPSHOT;
		dev->health.last_reason = ADBMS2950_REASON_NONE;
		dev->health.last_status = HAL_OK;
	}
	return status;
}

HAL_StatusTypeDef adbms2950_recover(adbms2950_driver_t *dev,
                                    bool allow_chain_reset)
{
	HAL_StatusTypeDef status;
	adbms2950_calibration_t saved_cal;
	uint8_t num_ics;
	uint8_t capacity;
	adbms2950_asic *ics;
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *csa_port;
	GPIO_TypeDef *csb_port;
	uint16_t csa_pin;
	uint16_t csb_pin;
	TIM_HandleTypeDef *htim;
	adbms_string string;

	if(!adbms2950_topology_valid(dev))
	{
		return HAL_ERROR;
	}
	dev->health.last_stage = ADBMS2950_STAGE_RECOVERY;
	saved_cal = dev->calibration;
	num_ics = dev->num_ics; capacity = dev->ics_capacity; ics = dev->ics;
	hspi = dev->hspi; csa_port = dev->cs_port[STRING_A]; csb_port = dev->cs_port[STRING_B];
	csa_pin = dev->cs_pin[STRING_A]; csb_pin = dev->cs_pin[STRING_B]; htim = dev->htim;
	string = dev->string;
	if(allow_chain_reset)
	{
		status = adbms2950_init_mixed_chain(dev, num_ics, ics, capacity,
		                                     hspi, csa_port, csb_port,
		                                     csa_pin, csb_pin, htim, string,
		                                     true, false);
		if((status == HAL_OK) && adbms2950_calibration_valid(&saved_cal))
		{
			status = adbms2950_set_calibration(dev, &saved_cal);
		}
	}
	else
	{
		adbms2950_force_dividers_off_best_effort(dev);
		status = adbms2950_read_sid(dev);
		if(status == HAL_OK) status = adbms2950_verify_refup(dev);
		if(status == HAL_OK) status = adbms2950_verify_config_readback(dev);
		if(status == HAL_OK) status = adbms2950_start_i1_continuous_checked(dev);
		dev->health.initialized = (status == HAL_OK);
	}
	if(status == HAL_OK)
	{
		adbms2950_sat_inc_u32(&dev->health.recovery_count);
		dev->health.last_stage = ADBMS2950_STAGE_RECOVERY;
		dev->health.last_reason = ADBMS2950_REASON_NONE;
		dev->health.last_status = HAL_OK;
	}
	else
	{
		adbms2950_sat_inc_u32(&dev->health.recovery_failure_count);
		adbms2950_set_failure(dev, ADBMS2950_STAGE_RECOVERY,
		                      ADBMS2950_REASON_RESTORE, status);
	}
	return status;
}

HAL_StatusTypeDef adbms2950_read_primary_sample(adbms2950_driver_t *dev,
												 uint32_t now_ms)
{
	HAL_StatusTypeDef status;
	HAL_StatusTypeDef unsnap_status;
	uint32_t raw_i1;
	uint16_t raw_vb1;
	int32_t signed_i1;
	int16_t signed_vb1;
	uint8_t status_counter;
	uint8_t sample_counter;
	uint8_t flag_counter;
	uint16_t i1cntpha;
	bool snapshot_active = false;

	if(dev == NULL)
	{
		return HAL_ERROR;
	}
	adbms2950_ensure_calibration(dev);
	if(!adbms2950_topology_valid(dev))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_VALIDATE,
		                      ADBMS2950_REASON_TOPOLOGY, HAL_ERROR);
		adbms2950_invalidate_sample(dev, HAL_ERROR, true);
		return HAL_ERROR;
	}
	if(!dev->health.initialized || !dev->health.i1_continuous_ready)
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_SAMPLE,
		                      ADBMS2950_REASON_NOT_INITIALIZED, HAL_ERROR);
		adbms2950_invalidate_sample(dev, HAL_ERROR, true);
		return HAL_ERROR;
	}
	if(!adbms2950_calibration_valid(&dev->calibration))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_SAMPLE,
		                      ADBMS2950_REASON_CALIBRATION, HAL_ERROR);
		adbms2950_invalidate_sample(dev, HAL_ERROR, true);
		return HAL_ERROR;
	}

	/* Recover from a prior interrupted snapshot, then freeze I1CNTPHA,
	 * I1/VB1 and status into one coherent read epoch. */
	dev->health.last_stage = ADBMS2950_STAGE_SNAPSHOT_RECOVER;
	dev->health.last_reason = ADBMS2950_REASON_NONE;
	status = adbms2950_wakeup_checked(dev);
	if(status == HAL_OK)
	{
		status = adbms2950_cmd_checked(dev, UNSNAP);
	}
	if(status == HAL_OK)
	{
		dev->health.last_stage = ADBMS2950_STAGE_SNAPSHOT;
		status = adbms2950_cmd_checked(dev, SNAP);
		snapshot_active = (status == HAL_OK);
	}
	if(status == HAL_OK)
	{
		status = adbms2950_read_status(dev);
	}
	if((status == HAL_OK) && !dev->health.i1_calibrated)
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_STATUS,
		                      ADBMS2950_REASON_CALIBRATION, HAL_ERROR);
		status = HAL_ERROR;
	}
	status_counter = dev->ics[0].rx_cmd_cntr;

	if(status == HAL_OK)
	{
		dev->health.last_stage = ADBMS2950_STAGE_SAMPLE;
		status = adbms2950_rd48_checked(dev, RDIVB1, buf);
	}
	if(status == HAL_OK)
	{
		sample_counter = dev->ics[0].rx_cmd_cntr;
		raw_i1 = (uint32_t)buf[0] |
		         ((uint32_t)buf[1] << 8u) |
		         ((uint32_t)buf[2] << 16u);
		raw_vb1 = (uint16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8u));
	}
	else
	{
		sample_counter = 0u;
		raw_i1 = 0u;
		raw_vb1 = 0u;
	}

	if(status == HAL_OK)
	{
		status = adbms2950_read_i1_counter_checked(dev, &i1cntpha, &flag_counter);
	}
	else
	{
		i1cntpha = 0u;
		flag_counter = 0u;
	}

	if(snapshot_active)
	{
		dev->health.last_stage = ADBMS2950_STAGE_UNSNAP;
		unsnap_status = adbms2950_cmd_checked(dev, UNSNAP);
		if((status == HAL_OK) && (unsnap_status != HAL_OK))
		{
			status = unsnap_status;
		}
	}

	/* Read commands do not advance CCNT.  All three packets must therefore
	 * report the same command-counter epoch established by SNAP.  CCNT may
	 * legitimately roll through zero, so equality—not a nonzero test—is the
	 * integrity condition. */
	if((status == HAL_OK) &&
	   ((status_counter != sample_counter) ||
	    (status_counter != flag_counter)))
	{
		adbms2950_sat_inc_u32(&dev->health.counter_mismatch_count);
		adbms2950_set_failure(dev, ADBMS2950_STAGE_SAMPLE,
		                      ADBMS2950_REASON_COMMAND_COUNTER, HAL_ERROR);
		status = HAL_ERROR;
	}

	if((status == HAL_OK) &&
	   ((raw_i1 == ADBMS2950_I1_RESET_CODE) ||
	    (raw_i1 == ADBMS2950_I1_CLEAR_CODE) ||
	    (raw_vb1 == ADBMS2950_VB1_RESET_CODE) ||
	    (raw_vb1 == ADBMS2950_VB1_CLEAR_CODE)))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_SAMPLE,
		                      ADBMS2950_REASON_SENTINEL, HAL_ERROR);
		status = HAL_ERROR;
	}

	if(status != HAL_OK)
	{
		if(dev->health.last_reason == ADBMS2950_REASON_NONE)
		{
			adbms2950_set_failure(dev, dev->health.last_stage,
			                      adbms2950_transaction_reason(dev), status);
		}
		dev->health.counter_advanced = false;
		adbms2950_invalidate_sample(dev, status, true);
		return status;
	}

	if(dev->health.counter_seen)
	{
		dev->health.counter_advanced =
			(dev->health.i1_conversion_count != dev->health.last_i1_conversion_count);
	}
	else
	{
		/* A successful compatible ADI1 resets I1CNTPHA.  A nonzero value
		 * proves at least one new conversion phase completed in that epoch. */
		dev->health.counter_advanced = (dev->health.i1_conversion_count != 0u);
	}
	if(!dev->health.counter_advanced)
	{
		adbms2950_sat_inc_u32(&dev->health.counter_stall_count);
		adbms2950_set_failure(dev, ADBMS2950_STAGE_SAMPLE,
		                      ADBMS2950_REASON_CONVERSION_STALL, HAL_ERROR);
		adbms2950_invalidate_sample(dev, HAL_ERROR, true);
		return HAL_ERROR;
	}

	signed_i1 = adbms2950_sign_extend_24(raw_i1);
	signed_vb1 = (int16_t)raw_vb1;
	dev->health.counter_seen = true;
	dev->health.last_i1cntpha = i1cntpha;
	dev->health.last_i1_conversion_count = dev->health.i1_conversion_count;
	dev->health.last_cmd_counter = flag_counter;

	dev->ics[0].ivbat.i1 = (uint32_t)signed_i1;
	dev->ics[0].ivbat.vbat1 = raw_vb1;
	dev->vi_adc[0] = (float)signed_i1 * VI1_SCALE;
	dev->current[0] = adbms2950_scale_current(dev, dev->vi_adc[0]);
	dev->vbat_adc[0] = (float)signed_vb1 * VBAT1_SCALE;
	dev->vbat[0] = adbms2950_scale_vbat1(dev, signed_vb1);

	dev->health.i1_raw = signed_i1;
	dev->health.vb1_raw = signed_vb1;
	dev->health.current_a = dev->current[0];
	dev->health.pack_voltage_v = dev->vbat[0];
	dev->health.current_valid = true;
	dev->health.pack_voltage_valid = dev->health.hv_dividers_enabled;
	dev->health.sample_valid = true;
	dev->health.last_update_ms = now_ms;
	dev->health.last_stage = ADBMS2950_STAGE_SAMPLE;
	dev->health.last_reason = ADBMS2950_REASON_NONE;
	dev->health.last_status = HAL_OK;
	adbms2950_sat_inc_u32(&dev->health.sample_count);
	return HAL_OK;
}


HAL_StatusTypeDef adbms2950_read_redundant_sample(adbms2950_driver_t *dev,
                                                  uint32_t now_ms)
{
	adi1_ redundant_cmd;
	adi1_ restore_cmd;
	HAL_StatusTypeDef status;
	HAL_StatusTypeDef restore_status = HAL_OK;
	HAL_StatusTypeDef unsnap_status;
	uint8_t status_counter = 0u;
	uint8_t i_counter = 0u;
	uint8_t vb_counter = 0u;
	uint8_t flag_counter = 0u;
	uint16_t i1cntpha = 0u;
	bool snapshot_active = false;
	bool redundant_started = false;
	int32_t i1_raw = 0;
	int32_t i2_raw = 0;
	int16_t vb1_raw = 0;
	int16_t vb2_raw = 0;

	if(dev == NULL)
	{
		return HAL_ERROR;
	}
	adbms2950_ensure_calibration(dev);
	memset(&dev->redundant_sample, 0, sizeof(dev->redundant_sample));
	dev->redundant_sample.last_status = HAL_ERROR;
	if(!adbms2950_topology_valid(dev))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_VALIDATE,
		                      ADBMS2950_REASON_TOPOLOGY, HAL_ERROR);
		return HAL_ERROR;
	}
	if(!dev->health.initialized || !dev->health.i1_continuous_ready)
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_REDUNDANT_START,
		                      ADBMS2950_REASON_NOT_INITIALIZED, HAL_ERROR);
		return HAL_ERROR;
	}
	if(!adbms2950_calibration_valid(&dev->calibration))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_REDUNDANT_START,
		                      ADBMS2950_REASON_CALIBRATION, HAL_ERROR);
		return HAL_ERROR;
	}

	/* RD=1 starts I1/VB1 and I2/VB2 synchronously.  OPT=1100 is selected
	 * because it is intentionally invalid for the ADBMS6830B cell monitors in
	 * the mixed ring, preventing a diagnostic from changing their ADC mode. */
	memset(&redundant_cmd, 0, sizeof(redundant_cmd));
	redundant_cmd.rd = 1u;
	redundant_cmd.opt = ADBMS2950_I1_CONTINUOUS_MIXED_OPT;
	dev->health.last_stage = ADBMS2950_STAGE_REDUNDANT_START;
	dev->health.last_reason = ADBMS2950_REASON_NONE;
	status = adbms2950_wakeup_checked(dev);
	if(status == HAL_OK)
	{
		status = adbms2950_adi1_checked(dev, &redundant_cmd);
		redundant_started = (status == HAL_OK);
	}
	if(status == HAL_OK)
	{
		status = adbms2950_delay_long_checked(dev,
			dev->health.i2_calibrated ? ADBMS2950_REDUNDANT_SETTLE_US :
			ADBMS2950_REDUNDANT_INIT_WAIT_US);
	}
	if(status == HAL_OK)
	{
		status = adbms2950_read_status(dev);
	}
	if((status == HAL_OK) &&
	   (!dev->health.i1_calibrated || !dev->health.i2_calibrated))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_STATUS,
		                      ADBMS2950_REASON_CALIBRATION, HAL_ERROR);
		status = HAL_ERROR;
	}
	dev->health.i2_continuous_ready = (status == HAL_OK);

	if(status == HAL_OK)
	{
		dev->health.last_stage = ADBMS2950_STAGE_SNAPSHOT_RECOVER;
		status = adbms2950_cmd_checked(dev, UNSNAP);
	}
	if(status == HAL_OK)
	{
		dev->health.last_stage = ADBMS2950_STAGE_SNAPSHOT;
		status = adbms2950_cmd_checked(dev, SNAP);
		snapshot_active = (status == HAL_OK);
	}
	if(status == HAL_OK)
	{
		status = adbms2950_read_status(dev);
		status_counter = dev->ics[0].rx_cmd_cntr;
	}
	if(status == HAL_OK)
	{
		dev->health.last_stage = ADBMS2950_STAGE_REDUNDANT_SAMPLE;
		status = adbms2950_rd48_checked(dev, RDI, buf);
		if(status == HAL_OK)
		{
			i_counter = dev->ics[0].rx_cmd_cntr;
			i1_raw = adbms2950_sign_extend_24((uint32_t)buf[0] |
			          ((uint32_t)buf[1] << 8u) | ((uint32_t)buf[2] << 16u));
			i2_raw = adbms2950_sign_extend_24((uint32_t)buf[3] |
			          ((uint32_t)buf[4] << 8u) | ((uint32_t)buf[5] << 16u));
		}
	}
	if(status == HAL_OK)
	{
		status = adbms2950_rd48_checked(dev, RDVB, buf);
		if(status == HAL_OK)
		{
			vb_counter = dev->ics[0].rx_cmd_cntr;
			vb1_raw = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8u));
			vb2_raw = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8u));
		}
	}
	if(status == HAL_OK)
	{
		status = adbms2950_read_i1_counter_checked(dev, &i1cntpha, &flag_counter);
	}
	if(snapshot_active)
	{
		dev->health.last_stage = ADBMS2950_STAGE_UNSNAP;
		unsnap_status = adbms2950_cmd_checked(dev, UNSNAP);
		if((status == HAL_OK) && (unsnap_status != HAL_OK))
		{
			status = unsnap_status;
		}
	}
	if((status == HAL_OK) &&
	   ((status_counter != i_counter) || (status_counter != vb_counter) ||
	    (status_counter != flag_counter)))
	{
		adbms2950_sat_inc_u32(&dev->health.counter_mismatch_count);
		adbms2950_set_failure(dev, ADBMS2950_STAGE_REDUNDANT_SAMPLE,
		                      ADBMS2950_REASON_COMMAND_COUNTER, HAL_ERROR);
		status = HAL_ERROR;
	}
	if((status == HAL_OK) && (dev->health.i1_conversion_count == 0u))
	{
		adbms2950_sat_inc_u32(&dev->health.counter_stall_count);
		adbms2950_set_failure(dev, ADBMS2950_STAGE_REDUNDANT_SAMPLE,
		                      ADBMS2950_REASON_CONVERSION_STALL, HAL_ERROR);
		status = HAL_ERROR;
	}
	if((status == HAL_OK) &&
	   ((((uint32_t)i1_raw & 0x00FFFFFFu) == ADBMS2950_I1_RESET_CODE) ||
	    (((uint32_t)i1_raw & 0x00FFFFFFu) == ADBMS2950_I1_CLEAR_CODE) ||
	    (((uint32_t)i2_raw & 0x00FFFFFFu) == ADBMS2950_I1_RESET_CODE) ||
	    (((uint32_t)i2_raw & 0x00FFFFFFu) == ADBMS2950_I1_CLEAR_CODE) ||
	    ((uint16_t)vb1_raw == ADBMS2950_VB1_RESET_CODE) ||
	    ((uint16_t)vb1_raw == ADBMS2950_VB1_CLEAR_CODE) ||
	    ((uint16_t)vb2_raw == ADBMS2950_VB1_RESET_CODE) ||
	    ((uint16_t)vb2_raw == ADBMS2950_VB1_CLEAR_CODE)))
	{
		adbms2950_set_failure(dev, ADBMS2950_STAGE_REDUNDANT_SAMPLE,
		                      ADBMS2950_REASON_SENTINEL, HAL_ERROR);
		status = HAL_ERROR;
	}

	if(status == HAL_OK)
	{
		dev->redundant_sample.i1_raw = i1_raw;
		dev->redundant_sample.i2_raw = i2_raw;
		dev->redundant_sample.vb1_raw = vb1_raw;
		dev->redundant_sample.vb2_raw = vb2_raw;
		dev->redundant_sample.current1_a =
			adbms2950_scale_current(dev, (float)i1_raw * VI1_SCALE);
		dev->redundant_sample.current2_a =
			adbms2950_scale_current(dev, (float)i2_raw * VI2_SCALE);
		dev->redundant_sample.current_disagreement_a =
			fabsf(dev->redundant_sample.current1_a -
			      dev->redundant_sample.current2_a);
		dev->redundant_sample.pack_voltage1_v = adbms2950_scale_vbat1(dev, vb1_raw);
		dev->redundant_sample.pack_voltage2_v = adbms2950_scale_vbat2(dev, vb2_raw);
		dev->redundant_sample.pack_voltage_disagreement_v =
			fabsf(dev->redundant_sample.pack_voltage1_v -
			      dev->redundant_sample.pack_voltage2_v);
		dev->redundant_sample.last_update_ms = now_ms;
		dev->redundant_sample.valid = true;
	}

	/* Restore the normal I1/VB1-only continuous mode even after a diagnostic
	 * failure.  This invalidates the primary sample epoch so the next scheduler
	 * read must prove a fresh I1CNT. */
	if(redundant_started)
	{
		dev->health.last_stage = ADBMS2950_STAGE_RESTORE;
		memset(&restore_cmd, 0, sizeof(restore_cmd));
		restore_cmd.rd = 0u;
		restore_cmd.opt = ADBMS2950_I1_CONTINUOUS_MIXED_OPT;
		restore_status = adbms2950_adi1_checked(dev, &restore_cmd);
		adbms2950_note_compatible_adi1(dev);
		dev->health.i2_continuous_ready = false;
		if(restore_status != HAL_OK)
		{
			dev->health.i1_continuous_ready = false;
			dev->redundant_sample.valid = false;
			if(status == HAL_OK)
			{
				status = restore_status;
			}
			adbms2950_set_failure(dev, ADBMS2950_STAGE_RESTORE,
			                      ADBMS2950_REASON_RESTORE, restore_status);
		}
	}
	if(status == HAL_OK)
	{
		dev->health.last_stage = ADBMS2950_STAGE_REDUNDANT_SAMPLE;
		dev->health.last_reason = ADBMS2950_REASON_NONE;
		dev->health.last_status = HAL_OK;
	}
	else if(dev->health.last_reason == ADBMS2950_REASON_NONE)
	{
		adbms2950_set_failure(dev, dev->health.last_stage,
		                      adbms2950_transaction_reason(dev), status);
	}
	dev->redundant_sample.last_status = status;
	return status;
}

void adbms2950_note_compatible_adi1(adbms2950_driver_t *dev)
{
	if(dev == NULL)
	{
		return;
	}

	dev->health.counter_seen = false;
	dev->health.counter_advanced = false;
	dev->health.last_i1_conversion_count = 0u;
	/* The compatible command starts a new conversion epoch. Keep the previous
	 * values and timestamp for diagnostics, but withdraw their validity until a
	 * coherent SNAP/RDSTAT/RDIVB1/RDFLAG transaction succeeds. */
	dev->health.sample_valid = false;
	dev->health.current_valid = false;
	dev->health.pack_voltage_valid = false;
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
    /* SNAPST and REFUP are read-only live status bits and must be written 0. */
    ic[cic].configa.tx_data[5] = (((ic[cic].tx_cfga.vb2mux & 0x01) << 7) |
                                      ((ic[cic].tx_cfga.vb1mux & 0x01) << 6) |
                                      ((ic[cic].tx_cfga.commbk & 0x01) << 3) |
                                      (ic[cic].tx_cfga.acci & 0x07));
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
    ic[cic].configb.tx_data[0] = (ic[cic].tx_cfgb.oc1th & 0x7Fu);
    ic[cic].configb.tx_data[1] = (ic[cic].tx_cfgb.oc2th & 0x7Fu);
    ic[cic].configb.tx_data[2] = (ic[cic].tx_cfgb.oc3th & 0x7Fu);
    ic[cic].configb.tx_data[3] = 0x00;
    ic[cic].configb.tx_data[4] = 0x00;
    ic[cic].configb.tx_data[5] = 0x00;
    ic[cic].configb.tx_data[3] = (uint8_t)(((ic[cic].tx_cfgb.ocdp & 0x01u) << 3u) |
                                              (ic[cic].tx_cfgb.ocdgt & 0x03u));
    ic[cic].configb.tx_data[4] = (((ic[cic].tx_cfgb.ocbx & 0x01)  << 7)| ((ic[cic].tx_cfgb.ocax & 0x01) << 6) | ((ic[cic].tx_cfgb.ocmode & 0x03) << 4)
                                      |((ic[cic].tx_cfgb.oc3gc & 0x01) << 3) | ((ic[cic].tx_cfgb.oc2gc & 0x01) << 2) | ((ic[cic].tx_cfgb.oc1gc & 0x01) << 1)
                                        | (ic[cic].tx_cfgb.ocod & 0x01));
    ic[cic].configb.tx_data[5] = (((ic[cic].tx_cfgb.gpio4c & 0x01)  << 7)| ((ic[cic].tx_cfgb.gpio3c & 0x01) << 6) | ((ic[cic].tx_cfgb.gpio2c & 0x01) << 5)
                                      | ((ic[cic].tx_cfgb.gpio1c & 0x01) << 4) |((ic[cic].tx_cfgb.gpio2eoc & 0x01) << 3) | (ic[cic].tx_cfgb.diagsel & 0x07));
  }
}

HAL_StatusTypeDef adbms2950_cmd_status(adbms2950_driver_t *dev,
                                       const uint8_t cmd[CMDSZ])
{
	uint8_t local_cmd[CMDSZ];
	if(cmd == NULL)
	{
		return HAL_ERROR;
	}
	local_cmd[0] = cmd[0];
	local_cmd[1] = cmd[1];
	return adbms2950_cmd_checked(dev, local_cmd);
}

HAL_StatusTypeDef adbms2950_adi1_checked(adbms2950_driver_t *dev,
                                         const adi1_ *arg)
{
	uint8_t cmd[CMDSZ];
	if(arg == NULL)
	{
		return HAL_ERROR;
	}
	adbms2950_build_adi1_command(arg->rd, arg->opt, cmd);
	return adbms2950_cmd_checked(dev, cmd);
}

void adbms2950_adi1(adbms2950_driver_t* dev, adi1_* arg)
{
	(void)adbms2950_adi1_checked(dev, arg);
}

HAL_StatusTypeDef adbms2950_adi2_checked(adbms2950_driver_t *dev,
                                         const adi2_ *arg)
{
	uint8_t cmd[CMDSZ];
	uint8_t opt;
	if(arg == NULL)
	{
		return HAL_ERROR;
	}
	opt = (uint8_t)(arg->opt & 0x0Fu);
	cmd[0] = sADI2[0];
	cmd[1] = (uint8_t)(sADI2[1] | ((opt & 0x08u) << 4u) |
	                   ((opt & 0x04u) << 2u) | (opt & 0x03u));
	return adbms2950_cmd_checked(dev, cmd);
}

void adbms2950_adi2(adbms2950_driver_t* dev, adi2_* arg)
{
	(void)adbms2950_adi2_checked(dev, arg);
}

HAL_StatusTypeDef adbms2950_adv_checked(adbms2950_driver_t *dev,
                                        const adv_ *arg)
{
	uint8_t cmd[CMDSZ];
	if(arg == NULL)
	{
		return HAL_ERROR;
	}
	cmd[0] = sADV[0];
	cmd[1] = (uint8_t)(sADV[1] | ((arg->ow & 0x03u) << 6u) |
	                   (arg->ch & 0x0Fu));
	return adbms2950_cmd_checked(dev, cmd);
}

void adbms2950_adv(adbms2950_driver_t* dev, adv_* arg)
{
	(void)adbms2950_adv_checked(dev, arg);
}

HAL_StatusTypeDef adbms2950_plv_checked(adbms2950_driver_t *dev)
{
	return adbms2950_cmd_checked(dev, PLV);
}

void adbms2950_plv(adbms2950_driver_t* dev)
{
	(void)adbms2950_plv_checked(dev);
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
	status = HAL_OK;
	if(use_cs)
	{
		adbms2950_set_cs(dev, 0);
		status = adbms2950_us_delay(dev, ADBMS2950_CS_SETUP_HOLD_US);
	}

	if(status == HAL_OK)
	{
		status = HAL_SPI_Transmit(dev->hspi, data, len, SPI_TIMEOUT);
	}

	if(use_cs)
	{
		HAL_StatusTypeDef hold_status =
			adbms2950_us_delay(dev, ADBMS2950_CS_SETUP_HOLD_US);
		adbms2950_set_cs(dev, 1);
		if((status == HAL_OK) && (hold_status != HAL_OK))
		{
			status = hold_status;
		}
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
	status = HAL_OK;
	if(use_cs)
	{
		adbms2950_set_cs(dev, 0);
		status = adbms2950_us_delay(dev, ADBMS2950_CS_SETUP_HOLD_US);
	}

	if(status == HAL_OK)
	{
		status = HAL_SPI_TransmitReceive(dev->hspi,
									  adbms2950_spi_txrx_tx_buf,
									  adbms2950_spi_txrx_rx_buf,
									  total_len,
									  SPI_TIMEOUT);
	}

	if(use_cs)
	{
		HAL_StatusTypeDef hold_status =
			adbms2950_us_delay(dev, ADBMS2950_CS_SETUP_HOLD_US);
		adbms2950_set_cs(dev, 1);
		if((status == HAL_OK) && (hold_status != HAL_OK))
		{
			status = hold_status;
		}
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
