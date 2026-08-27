/*
 * adbms6830.c
 *
 *  Created on: May 13, 2025
 *      Author: realb
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "ext_drivers/adbms6830_functions.h"
#include "ext_drivers/thermistor_model.h"
#include "ams_build_profile.h"

static unsigned char shared_buf[BUFSZ] = {0};
static uint8_t write_buf[BUFSZ] = {0};
static uint8_t spi_txrx_tx_buf[BUFSZ] = {0};
static uint8_t spi_txrx_rx_buf[BUFSZ] = {0};

#define ADBMS6830_SPI_DUMMY_BYTE 0xFFu

#define ADBMS6830_DELAY_SPINS_PER_US 256u
#define ADBMS6830_DELAY_BASE_SPINS 1024u
#define ADBMS6830_CONVERSION_POLL_TIMEOUT_US 30000u
#define ADBMS6830_POLL_SPIN_MARGIN 4096u

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
#define ADAX_CMD_BYTE1    0x10u   /* CH=00000 = all AUX/GPIO channels, OW/PUP off */

/* WRCOMM / STCOMM copies — same bytes as in adg728_i2c.c but local
 * to avoid pulling in the adBms_Application layer headers here.       */
#define ICOMM_START_  0x6u
#define ICOMM_BLANK_  0x0u
#define ICOMM_STOP_   0x1u
/* For an I2C write, the ADBMS6830 must release SDA on the ninth clock so
 * the slave can pull it low for ACK.  Table 33 calls this write code
 * "Master no acknowledge" (0x8).  Using 0x0 would make the ADBMS6830 drive
 * the ACK bit low itself, preventing meaningful slave-ACK detection. */
#define FCOMM_RELEASE_FOR_SLAVE_ACK_ 0x8u
#define FCOMM_NACK_STOP_             0x9u



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

/* Read average cell voltage result registers commands commands */
static uint8_t RDACA[2]         = { 0x00, 0x44 };
static uint8_t RDACB[2]         = { 0x00, 0x46 };
static uint8_t RDACC[2]         = { 0x00, 0x48 };
static uint8_t RDACD[2]         = { 0x00, 0x4A };
static uint8_t RDACE[2]         = { 0x00, 0x49 };
static uint8_t RDACF[2]         = { 0x00, 0x4B };

/* Read s voltage result registers commands */
static uint8_t RDSVA[2]         = { 0x00, 0x03 };
static uint8_t RDSVB[2]         = { 0x00, 0x05 };
static uint8_t RDSVC[2]         = { 0x00, 0x07 };
static uint8_t RDSVD[2]         = { 0x00, 0x0D };
static uint8_t RDSVE[2]         = { 0x00, 0x0E };
static uint8_t RDSVF[2]         = { 0x00, 0x0F };

/* Read filtered cell voltage result registers*/
static uint8_t RDFCA[2]         = { 0x00, 0x12 };
static uint8_t RDFCB[2]         = { 0x00, 0x13 };
static uint8_t RDFCC[2]         = { 0x00, 0x14 };
static uint8_t RDFCD[2]         = { 0x00, 0x15 };
static uint8_t RDFCE[2]         = { 0x00, 0x16 };
static uint8_t RDFCF[2]         = { 0x00, 0x17 };

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


/* Mux address and ADAX channel lookup by mux index (0=U2, 1=U3, 2=U4) */
static const uint8_t MUX_ADDRS[3]    = { ADG728_U2_ADDR, ADG728_U3_ADDR, ADG728_U4_ADDR };
static const uint8_t ADAX_CH[3]      = { ADAX_GPIO1,     ADAX_GPIO2,     ADAX_GPIO3     };

/* RDAUXA a_codes[] index for GPIO1/2/3 */
static const uint8_t GPIO_AUX_IDX[3] = { 0u, 1u, 2u };

static bool adbms6830_string_valid(adbms_string string)
{
    return ((int)string >= (int)STRING_A) && (string <= STRING_B);
}

static bool adbms6830_active_cs_valid(const adbms6830_driver_t *dev)
{
    return (dev != NULL) &&
           adbms6830_string_valid(dev->string) &&
           (dev->cs_port[dev->string] != NULL) &&
           (dev->cs_pin[dev->string] != 0u);
}

static bool adbms6830_topology_valid(const adbms6830_driver_t *dev)
{
    return (dev != NULL) &&
           (dev->ics != NULL) &&
           (dev->hspi != NULL) &&
           (dev->num_ics > 0) &&
           (dev->num_ics <= (int)ADBMS6830_MAX_TRACKED_ICS) &&
           (dev->physical_chain_count >= (uint8_t)dev->num_ics) &&
           (dev->physical_chain_count <= ADBMS6830_MAX_PHYSICAL_DEVICES) &&
           (dev->ics_capacity > 0u) &&
           (dev->num_ics <= (int)dev->ics_capacity) &&
           adbms6830_string_valid(dev->write_string) &&
           (dev->cs_port[dev->write_string] != NULL) &&
           (dev->cs_pin[dev->write_string] != 0u) &&
           adbms6830_active_cs_valid(dev);
}

static uint16_t adbms6830_expected_ic_mask(const adbms6830_driver_t *dev)
{
    if(!adbms6830_topology_valid(dev))
    {
        return 0u;
    }

    if(dev->num_ics == (int)ADBMS6830_MAX_TRACKED_ICS)
    {
        return UINT16_MAX;
    }

    return (uint16_t)((1UL << (uint8_t)dev->num_ics) - 1UL);
}

static bool adbms6830_aux_code_valid(const uint8_t *packet, uint8_t gpio_ch)
{
    uint8_t byte_lo;
    uint8_t byte_hi;

    if((packet == NULL) || (gpio_ch >= ADBMS6830_MUX_COUNT))
    {
        return false;
    }

    byte_lo = (uint8_t)(gpio_ch * 2u);
    byte_hi = (uint8_t)(byte_lo + 1u);
    return !(((packet[byte_lo] == 0xFFu) && (packet[byte_hi] == 0xFFu)) ||
             ((packet[byte_lo] == 0x00u) && (packet[byte_hi] == 0x80u)));
}

static bool adbms6830_status_code_valid(int16_t code)
{
    return (code != INT16_MIN) && (code != INT16_MAX) && (code != (int16_t)-1);
}

static bool adbms6830_status_voltage_mv(int16_t code, int16_t *millivolts)
{
    int32_t mv;

    if((millivolts == NULL) || !adbms6830_status_code_valid(code))
    {
        return false;
    }

    /* Status voltage code: V = 1.5 V + code * 150 uV. */
    mv = 1500 + (((int32_t)code * 150) / 1000);
    if((mv < INT16_MIN) || (mv > INT16_MAX))
    {
        return false;
    }

    *millivolts = (int16_t)mv;
    return true;
}

static bool adbms6830_status_die_temp_deci_c(int16_t code, int16_t *deci_c)
{
    int32_t temperature;

    if((deci_c == NULL) || !adbms6830_status_code_valid(code))
    {
        return false;
    }

    /* Datasheet ITMP transfer function expressed directly in 0.1 deg C. */
    temperature = (((int32_t)code + 10000) / 5) - 2730;
    if((temperature < INT16_MIN) || (temperature > INT16_MAX))
    {
        return false;
    }

    *deci_c = (int16_t)temperature;
    return true;
}

static uint16_t adbms6830_monitored_cell_mask(const adbms6830_driver_t *dev)
{
    if((dev == NULL) || (dev->monitored_cell_count == 0u) ||
       (dev->monitored_cell_count > CELL))
    {
        return 0u;
    }

    return (dev->monitored_cell_count == 16u) ? UINT16_MAX :
           (uint16_t)((1UL << dev->monitored_cell_count) - 1UL);
}

// SPI communication
void adbms6830_set_cs(adbms6830_driver_t* dev, uint8_t state);
HAL_StatusTypeDef adbms6830_spi_write(adbms6830_driver_t* dev, uint8_t* data, uint16_t len, uint8_t use_cs);
HAL_StatusTypeDef adbms6830_spi_write_read(adbms6830_driver_t *dev, uint8_t* tx_Data, uint8_t tx_len, uint8_t* rx_data, uint16_t rx_len, uint8_t use_cs);

// Tx/Rx Utility
void adbms6830_cmd(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ]);
void adbms6830_wr48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data);
void adbms6830_rd48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data);
static HAL_StatusTypeDef adbms6830_cmd_checked(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ]);
static HAL_StatusTypeDef adbms6830_adcv_checked(adbms6830_driver_t *dev, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs);
static HAL_StatusTypeDef adbms6830_wr48_checked(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data);
static HAL_StatusTypeDef adbms6830_rd48_checked(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data);
static uint32_t adbms6830_runtime_time_us(const adbms6830_driver_t *dev);
static void adbms6830_session_note_activity(adbms6830_driver_t *dev);
static void adbms6830_session_open(adbms6830_driver_t *dev);
static void adbms6830_session_close(adbms6830_driver_t *dev);
static HAL_StatusTypeDef adbms6830_session_require_awake(adbms6830_driver_t *dev);
static HAL_StatusTypeDef adbms6830_poll_conversion_checked(
    adbms6830_driver_t *dev,
    const uint8_t poll_cmd[CMDSZ],
    uint32_t timeout_us,
    uint32_t *elapsed_us,
    uint32_t *clock_bytes,
    bool *observed_busy,
    bool *complete);
static void adbms6830_spi_debug_note_tx(adbms6830_driver_t *dev, adbms6830_spi_op_t op, const uint8_t *cmd, const uint8_t *tx, uint16_t tx_len, uint16_t rx_len);
static void adbms6830_spi_debug_note_rx(adbms6830_driver_t *dev, const uint8_t *rx, uint16_t rx_len, HAL_StatusTypeDef status);
static void adbms6830_note_counter_reset(adbms6830_driver_t *dev);
static void adbms6830_note_counter_increment(adbms6830_driver_t *dev);
static void adbms6830_note_observed_counter(adbms6830_driver_t *dev, uint8_t current_ic, uint8_t observed_counter, bool pec_ok);
static void adbms6830_note_pec_result(adbms6830_driver_t *dev, uint8_t current_ic, bool pec_ok);
static bool adbms6830_cmd_increments_counter(const uint8_t cmd[CMDSZ]);
static bool adbms6830_cmd_resets_counter(const uint8_t cmd[CMDSZ]);
static void adbms6830_parse_sid(adbms6830_driver_t *dev, const uint8_t *data);
static void adbms6830_parse_stata(adbms6830_driver_t *dev, const uint8_t *data);
static void adbms6830_parse_statb(adbms6830_driver_t *dev, const uint8_t *data);
static void adbms6830_parse_statc(adbms6830_driver_t *dev, const uint8_t *data);
static void adbms6830_parse_statd(adbms6830_driver_t *dev, const uint8_t *data);
static void adbms6830_parse_state(adbms6830_driver_t *dev, const uint8_t *data);
static void adbms6830_refresh_status_health(adbms6830_driver_t *dev);
static HAL_StatusTypeDef adbms6830_clear_all_ovuv_flags(adbms6830_driver_t *dev);
static void adbms6830_parse_aux_gpio(adbms6830_driver_t *dev, uint8_t *data);
// Parsing Rx Data
void adbms6830_parse_cfga(adbms6830_driver_t* dev, uint8_t *data);
void adbms6830_parse_cfgb(adbms6830_driver_t* dev, uint8_t *data);
// Packing Tx Data
void adbms6830_pack_cfga(adbms6830_driver_t *dev);
void adbms6830_pack_cfgb(adbms6830_driver_t *dev);
static void adbms6830_pack_pwma(adbms6830_driver_t *dev);
static void adbms6830_pack_pwmb(adbms6830_driver_t *dev);
void adbms6830_pack_comm(adbms6830_driver_t* dev);
void adbms6830_pack_clr_flag_data(adbms6830_driver_t* dev);



static uint32_t adbms6830_runtime_time_us(const adbms6830_driver_t *dev)
{
    if((dev != NULL) && (dev->time_us_fn != NULL))
    {
        return dev->time_us_fn(dev->runtime_hook_ctx);
    }
    return 0u;
}

static void adbms6830_session_note_activity(adbms6830_driver_t *dev)
{
    uint32_t now;
    uint32_t gap;

    if((dev == NULL) || !dev->session.active)
    {
        return;
    }

    now = adbms6830_runtime_time_us(dev);
    if(now == 0u)
    {
        return;
    }

    if(dev->session.last_activity_us != 0u)
    {
        gap = now - dev->session.last_activity_us;
        dev->session.last_gap_us = gap;
        if(gap > dev->session.max_gap_us)
        {
            dev->session.max_gap_us = gap;
        }
    }
    dev->session.last_activity_us = now;
}

static void adbms6830_session_open(adbms6830_driver_t *dev)
{
    uint32_t now;

    if(dev == NULL)
    {
        return;
    }

    now = adbms6830_runtime_time_us(dev);
    dev->session.active = true;
    dev->session.coherent_snapshot_active = false;
    dev->session.session_id++;
    if(dev->session.session_id == 0u)
    {
        dev->session.session_id = 1u;
    }
    if(dev->session.session_count != UINT32_MAX)
    {
        dev->session.session_count++;
    }
    dev->session.start_us = now;
    dev->session.last_activity_us = now;
    dev->session.last_gap_us = 0u;
}

static void adbms6830_session_close(adbms6830_driver_t *dev)
{
    uint32_t now;
    uint32_t duration;

    if((dev == NULL) || !dev->session.active)
    {
        return;
    }

    now = adbms6830_runtime_time_us(dev);
    if((now != 0u) && (dev->session.start_us != 0u))
    {
        duration = now - dev->session.start_us;
        dev->session.last_duration_us = duration;
        if(duration > dev->session.max_duration_us)
        {
            dev->session.max_duration_us = duration;
        }
    }
    dev->session.active = false;
    dev->session.coherent_snapshot_active = false;
}

static HAL_StatusTypeDef adbms6830_session_require_awake(adbms6830_driver_t *dev)
{
#if AMS_ENABLE_ADBMS_AWAKE_SESSION
    uint32_t now;
    uint32_t gap;

    if((dev == NULL) || !dev->session.active)
    {
        return HAL_ERROR;
    }

    /* Bench/service-only one-shot timing injection. Normal mode lets the
     * guard detect the deliberately over-idle gap and restart a coherent
     * epoch. Raw mode bypasses the guard exactly once to exercise the actual
     * PEC/read-fault path after an over-idle serial interval. */
    if(dev->session.inject_gap_us_once != 0u)
    {
        uint32_t injected = dev->session.inject_gap_us_once;
        bool bypass = dev->session.inject_bypass_guard_once;
        dev->session.inject_gap_us_once = 0u;
        dev->session.inject_bypass_guard_once = false;
        dev->session.last_injected_gap_us = injected;
        if(dev->session.injected_gap_count != UINT32_MAX)
        {
            dev->session.injected_gap_count++;
        }
        if(adbms6830_wait_cooperative(dev, injected) != HAL_OK)
        {
            return HAL_BUSY;
        }
        if(bypass)
        {
            return HAL_OK;
        }
    }

    now = adbms6830_runtime_time_us(dev);
    if((now == 0u) || (dev->session.last_activity_us == 0u))
    {
        return HAL_OK;
    }

    gap = now - dev->session.last_activity_us;
    dev->session.last_gap_us = gap;
    if(gap > dev->session.max_gap_us)
    {
        dev->session.max_gap_us = gap;
    }

    if(gap < AMS_ADBMS_SESSION_GUARD_US)
    {
        /* This guard is deliberately advisory, not a scheduler lock. A higher
         * priority task or ISR can still preempt between this check and the
         * following SPI transaction. The target-qualified guard therefore
         * includes measured preemption margin; any residual check/act race is
         * detected by PEC/counter integrity and causes the coherent epoch to be
         * discarded/retried, with persistent read faults qualified over scans.
         * Do not close this race by masking safety interrupts. */
        return HAL_OK;
    }

    if(dev->session.guard_expiry_count != UINT32_MAX)
    {
        dev->session.guard_expiry_count++;
    }

    /* Never splice two wake epochs into one coherent SNAP image.  The caller
     * must discard and restart the full snapshot. */
    if(dev->session.coherent_snapshot_active)
    {
        return HAL_BUSY;
    }

    if(adbms6830_wakeup_checked(dev) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if(dev->session.guard_rewake_count != UINT32_MAX)
    {
        dev->session.guard_rewake_count++;
    }
    dev->session.last_activity_us = adbms6830_runtime_time_us(dev);
#else
    (void)dev;
#endif
    return HAL_OK;
}

static HAL_StatusTypeDef adbms6830_read_session_begin(
    adbms6830_driver_t *dev, bool *owned)
{
    HAL_StatusTypeDef status;

    if((dev == NULL) || (owned == NULL))
    {
        return HAL_ERROR;
    }
    *owned = false;
#if AMS_ENABLE_ADBMS_AWAKE_SESSION
    if(dev->session.active)
    {
        return adbms6830_session_require_awake(dev);
    }

    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        adbms6830_session_open(dev);
        *owned = true;
    }
    return status;
#else
    (void)status;
    /* Legacy/diagnostic build: rd48_checked retains its per-read wake. */
    return HAL_OK;
#endif
}

static void adbms6830_read_session_end(adbms6830_driver_t *dev, bool owned)
{
#if AMS_ENABLE_ADBMS_AWAKE_SESSION
    if(owned)
    {
        adbms6830_session_close(dev);
    }
#else
    (void)dev;
    (void)owned;
#endif
}

static uint16_t adbms6830_min_u16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

static void adbms6830_increment_u32_sat(uint32_t *value)
{
    if((value != NULL) && (*value != UINT32_MAX))
    {
        (*value)++;
    }
}

static bool adbms6830_last_read_integrity_ok(const adbms6830_driver_t *dev)
{
    uint16_t expected_mask = adbms6830_expected_ic_mask(dev);

    return (expected_mask != 0u) &&
           (dev->health.last_pec_pass_mask == expected_mask) &&
           (dev->health.last_pec_fail_mask == 0u) &&
           (dev->health.last_cmd_counter_mismatch_mask == 0u);
}

static void adbms6830_diag_note_status(adbms6830_driver_t *dev,
                                       adbms6830_spi_op_t op,
                                       HAL_StatusTypeDef status)
{
    if(dev == NULL)
    {
        return;
    }

    dev->health.last_op = op;
    dev->health.last_status = status;
}

static void adbms6830_note_pec_result(adbms6830_driver_t *dev, uint8_t current_ic, bool pec_ok)
{
    uint16_t bit;

    if((dev == NULL) || (current_ic >= ADBMS6830_MAX_TRACKED_ICS))
    {
        return;
    }

    bit = (uint16_t)(1u << current_ic);
    if(pec_ok)
    {
        dev->health.last_pec_pass_mask |= bit;
        adbms6830_increment_u32_sat(&dev->health.pec_pass_count[current_ic]);
    }
    else
    {
        dev->health.last_pec_fail_mask |= bit;
        dev->health.sticky_pec_fail_mask |= bit;
        adbms6830_increment_u32_sat(&dev->health.pec_fail_count[current_ic]);
    }
}

static uint8_t adbms6830_counter_next(uint8_t current)
{
    current &= 0x3Fu;
    if(current == 0u)
    {
        return 1u;
    }
    if(current >= 63u)
    {
        return 1u;
    }
    return (uint8_t)(current + 1u);
}

static void adbms6830_note_counter_reset(adbms6830_driver_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

    dev->spi_debug.cmd_counter_expected_mask = 0u;
    dev->spi_debug.cmd_counter_mismatch_mask = 0u;
    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        dev->spi_debug.expected_cmd_counter[ic] = 0u;
        if(ic < (uint8_t)dev->num_ics)
        {
            dev->spi_debug.cmd_counter_expected_mask |= (uint16_t)(1u << ic);
        }
    }
}

static void adbms6830_note_counter_increment(adbms6830_driver_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

    for(uint8_t ic = 0u; (ic < (uint8_t)dev->num_ics) && (ic < ADBMS6830_MAX_TRACKED_ICS); ic++)
    {
        if((dev->spi_debug.cmd_counter_expected_mask & (uint16_t)(1u << ic)) != 0u)
        {
            dev->spi_debug.expected_cmd_counter[ic] =
                adbms6830_counter_next(dev->spi_debug.expected_cmd_counter[ic]);
        }
    }
}

void adbms6830_note_external_counter_increments(adbms6830_driver_t *dev,
                                                 uint8_t increment_count)
{
    if(!adbms6830_topology_valid(dev))
    {
        return;
    }

    for(uint8_t i = 0u; i < increment_count; i++)
    {
        adbms6830_note_counter_increment(dev);
    }
}

void adbms6830_resync_command_counter_tracking(adbms6830_driver_t *dev)
{
    if(!adbms6830_topology_valid(dev))
    {
        return;
    }

    /* A timeout cannot prove whether a correctly PEC-protected command
     * reached zero, some, or all remote devices. Let the next valid packet
     * establish each IC's observed counter independently. */
    dev->spi_debug.cmd_counter_expected_mask = 0u;
    dev->spi_debug.cmd_counter_mismatch_mask = 0u;
    dev->health.last_cmd_counter_mismatch_mask = 0u;
    memset(dev->spi_debug.expected_cmd_counter,
           0,
           sizeof(dev->spi_debug.expected_cmd_counter));
}

static void adbms6830_note_observed_counter(adbms6830_driver_t *dev,
                                            uint8_t current_ic,
                                            uint8_t observed_counter,
                                            bool pec_ok)
{
    uint16_t bit;

    if((dev == NULL) || (current_ic >= ADBMS6830_MAX_TRACKED_ICS) || !pec_ok)
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
        dev->health.unexpected_counter_reset_mask |= bit;
        dev->health.sticky_unexpected_counter_reset_mask |= bit;
        adbms6830_increment_u32_sat(
            &dev->health.unexpected_counter_reset_count[current_ic]);
    }

    if(dev->spi_debug.expected_cmd_counter[current_ic] != observed_counter)
    {
        dev->spi_debug.cmd_counter_mismatch_mask |= bit;
        adbms6830_increment_u32_sat(&dev->spi_debug.cmd_counter_error_count);
        adbms6830_increment_u32_sat(&dev->spi_debug.error_count);
        dev->health.last_cmd_counter_mismatch_mask |= bit;
        dev->health.sticky_cmd_counter_mismatch_mask |= bit;
        adbms6830_increment_u32_sat(
            &dev->health.cmd_counter_mismatch_count[current_ic]);
        dev->spi_debug.expected_cmd_counter[current_ic] = observed_counter;
    }
    else
    {
        dev->spi_debug.cmd_counter_mismatch_mask &= (uint16_t)~bit;
    }
}

static bool adbms6830_cmd_resets_counter(const uint8_t cmd[CMDSZ])
{
    if(cmd == NULL)
    {
        return false;
    }

    return ((cmd[0] == SRST[0]) && (cmd[1] == SRST[1])) ||
           ((cmd[0] == RSTCC[0]) && (cmd[1] == RSTCC[1]));
}

static bool adbms6830_cmd_increments_counter(const uint8_t cmd[CMDSZ])
{
    if(cmd == NULL)
    {
        return false;
    }

    if(adbms6830_cmd_resets_counter(cmd))
    {
        return false;
    }

    if(((cmd[0] == SNAP[0]) && (cmd[1] == SNAP[1])) ||
       ((cmd[0] == UNSNAP[0]) && (cmd[1] == UNSNAP[1])) ||
       ((cmd[0] == MUTE[0]) && (cmd[1] == MUTE[1])) ||
       ((cmd[0] == UNMUTE[0]) && (cmd[1] == UNMUTE[1])))
    {
        return true;
    }

    /* ADC/aux commands are encoded families with option bits in cmd[1]. */
    if((cmd[0] == 0x01u) || (cmd[0] == 0x02u) || (cmd[0] == 0x03u) || (cmd[0] == 0x04u) ||
       (cmd[0] == 0x05u) || (cmd[0] == 0x06u))
    {
        return true;
    }

    if((cmd[0] == CLRCELL[0]) && ((cmd[1] == CLRCELL[1]) ||
                                  (cmd[1] == CLRAUX[1]) ||
                                  (cmd[1] == CLRSPIN[1]) ||
                                  (cmd[1] == CLRFLAG[1]) ||
                                  (cmd[1] == CLRFC[1]) ||
                                  (cmd[1] == CLOVUV[1]) ||
                                  (cmd[1] == PLADC[1]) ||
                                  (cmd[1] == PLCADC[1]) ||
                                  (cmd[1] == PLSADC[1]) ||
                                  (cmd[1] == PLAUX1[1]) ||
                                  (cmd[1] == PLAUX2[1])))
    {
        return true;
    }

    return false;
}

static bool adbms6830_wr48_increments_counter(const uint8_t cmd[CMDSZ])
{
    if(cmd == NULL)
    {
        return false;
    }

    return ((cmd[0] == WRCFGA[0]) && (cmd[1] == WRCFGA[1])) ||
           ((cmd[0] == WRCFGB[0]) && (cmd[1] == WRCFGB[1])) ||
           ((cmd[0] == WRPWM1[0]) && (cmd[1] == WRPWM1[1])) ||
           ((cmd[0] == WRPWM2[0]) && (cmd[1] == WRPWM2[1])) ||
           ((cmd[0] == WRCOMM[0]) && (cmd[1] == WRCOMM[1])) ||
           ((cmd[0] == CLRFLAG[0]) && (cmd[1] == CLRFLAG[1])) ||
           ((cmd[0] == CLOVUV[0]) && (cmd[1] == CLOVUV[1]));
}

static bool adbms6830_read_packet_pec_ok(const uint8_t *data)
{
    uint16_t received_pec;
    uint16_t calculated_pec;

    if(data == NULL)
    {
        return false;
    }

    received_pec = (uint16_t)(((data[RX_DATA - 2u] & 0x03u) << 8u) |
                              data[RX_DATA - 1u]);
    calculated_pec = pec10_calc(1, RX_DATA - 2u, (uint8_t *)data);
    return received_pec == calculated_pec;
}

static bool adbms6830_read_packet_counter_ok(const adbms6830_driver_t *dev,
                                             uint8_t current_ic)
{
    if((dev == NULL) || (current_ic >= ADBMS6830_MAX_TRACKED_ICS))
    {
        return false;
    }

    return (dev->health.last_cmd_counter_mismatch_mask &
            (uint16_t)(1u << current_ic)) == 0u;
}

static void adbms6830_parse_sid(adbms6830_driver_t *dev, const uint8_t *data)
{
    if((dev == NULL) || (dev->ics == NULL) || (data == NULL))
    {
        return;
    }

    for(uint8_t curr_ic = 0u; (curr_ic < (uint8_t)dev->num_ics) && (curr_ic < ADBMS6830_MAX_TRACKED_ICS); curr_ic++)
    {
        const uint8_t *d = &data[(uint16_t)curr_ic * RX_DATA];
        bool pec_ok = adbms6830_read_packet_pec_ok(d);
        bool transport_valid = pec_ok && adbms6830_read_packet_counter_ok(dev, curr_ic);
        uint8_t device_id = transport_valid ?
            (uint8_t)((d[1] >> 1u) & 0x3Fu) : UINT8_MAX;
        bool identity_valid = transport_valid &&
                              (device_id == ADBMS6830B_DEVICE_ID);
        uint16_t ic_bit = (uint16_t)(1u << curr_ic);

        dev->ics[curr_ic].cccrc.sid_pec = pec_ok ? 0u : 1u;
        dev->diag[curr_ic].device_id = device_id;
        dev->diag[curr_ic].sid_valid = identity_valid;
        if(transport_valid)
        {
            memcpy(dev->ics[curr_ic].sid.sid, d, RSID);
            memcpy(dev->diag[curr_ic].sid, d, RSID);
        }
        if(identity_valid)
        {
            dev->health.sid_valid_ic_mask |= ic_bit;
        }
        else if(transport_valid)
        {
            dev->health.sid_identity_mismatch_ic_mask |= ic_bit;
            dev->health.sticky_sid_identity_mismatch_ic_mask |= ic_bit;
        }
    }
}

static void adbms6830_update_reference_validity(adbms6830_ic_diag_t *diag)
{
    bool valid;

    if(diag == NULL)
    {
        return;
    }

    valid = diag->stata_valid && diag->statb_valid;
    valid = valid && adbms6830_status_voltage_mv(diag->vref2_raw, &diag->vref2_mv);
    valid = valid && adbms6830_status_die_temp_deci_c(diag->itmp_raw,
                                                       &diag->die_temp_deci_c);
    valid = valid && adbms6830_status_voltage_mv(diag->vd_raw, &diag->vd_mv);
    valid = valid && adbms6830_status_voltage_mv(diag->va_raw, &diag->va_mv);
    valid = valid && adbms6830_status_voltage_mv(diag->vres_raw, &diag->vres_mv);
    diag->reference_values_valid = valid;
}

static void adbms6830_parse_stata(adbms6830_driver_t *dev, const uint8_t *data)
{
    if((dev == NULL) || (dev->ics == NULL) || (data == NULL))
    {
        return;
    }

    for(uint8_t curr_ic = 0u;
        (curr_ic < (uint8_t)dev->num_ics) && (curr_ic < ADBMS6830_MAX_TRACKED_ICS);
        curr_ic++)
    {
        const uint8_t *d = &data[(uint16_t)curr_ic * RX_DATA];
        bool pec_ok = adbms6830_read_packet_pec_ok(d);
        bool packet_valid = pec_ok && adbms6830_read_packet_counter_ok(dev, curr_ic);
        adbms6830_ic_diag_t *diag = &dev->diag[curr_ic];

        dev->ics[curr_ic].cccrc.stat_pec = pec_ok ? 0u : 1u;
        diag->stata_valid = packet_valid;
        diag->reference_values_valid = false;
        if(packet_valid)
        {
            diag->vref2_raw = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8u));
            diag->itmp_raw = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8u));
            dev->ics[curr_ic].stata.vref2 = (uint16_t)diag->vref2_raw;
            dev->ics[curr_ic].stata.itmp = (uint16_t)diag->itmp_raw;
            dev->ics[curr_ic].stata.vref3 =
                (uint16_t)d[4] | ((uint16_t)d[5] << 8u);
        }
        adbms6830_update_reference_validity(diag);
    }
}

static void adbms6830_parse_statb(adbms6830_driver_t *dev, const uint8_t *data)
{
    if((dev == NULL) || (dev->ics == NULL) || (data == NULL))
    {
        return;
    }

    for(uint8_t curr_ic = 0u;
        (curr_ic < (uint8_t)dev->num_ics) && (curr_ic < ADBMS6830_MAX_TRACKED_ICS);
        curr_ic++)
    {
        const uint8_t *d = &data[(uint16_t)curr_ic * RX_DATA];
        bool pec_ok = adbms6830_read_packet_pec_ok(d);
        bool packet_valid = pec_ok && adbms6830_read_packet_counter_ok(dev, curr_ic);
        adbms6830_ic_diag_t *diag = &dev->diag[curr_ic];

        dev->ics[curr_ic].cccrc.stat_pec = pec_ok ? 0u : 1u;
        diag->statb_valid = packet_valid;
        diag->reference_values_valid = false;
        if(packet_valid)
        {
            diag->vd_raw = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8u));
            diag->va_raw = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8u));
            diag->vres_raw = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8u));
            dev->ics[curr_ic].statb.vd = (uint16_t)diag->vd_raw;
            dev->ics[curr_ic].statb.va = (uint16_t)diag->va_raw;
            dev->ics[curr_ic].statb.vr4k = (uint16_t)diag->vres_raw;
        }
        adbms6830_update_reference_validity(diag);
    }
}

static void adbms6830_parse_statc(adbms6830_driver_t *dev, const uint8_t *data)
{
    if((dev == NULL) || (dev->ics == NULL) || (data == NULL))
    {
        return;
    }

    for(uint8_t curr_ic = 0u; (curr_ic < (uint8_t)dev->num_ics) && (curr_ic < ADBMS6830_MAX_TRACKED_ICS); curr_ic++)
    {
        const uint8_t *d = &data[(uint16_t)curr_ic * RX_DATA];
        bool pec_ok = adbms6830_read_packet_pec_ok(d);
        bool packet_valid = pec_ok && adbms6830_read_packet_counter_ok(dev, curr_ic);
        adbms6830_ic_diag_t *diag = &dev->diag[curr_ic];

        dev->ics[curr_ic].cccrc.stat_pec = pec_ok ? 0u : 1u;
        diag->statc_valid = packet_valid;
        if(!packet_valid)
        {
            continue;
        }

        diag->cs_flt_mask = (uint16_t)d[0] | ((uint16_t)d[1] << 8u);
        diag->cadc_counter = (uint16_t)(((uint16_t)(d[2] & 0x1Fu) << 6u) |
                                        ((uint16_t)(d[3] & 0xFCu) >> 2u));
        diag->cadc_subcounter = (uint8_t)(d[3] & 0x03u);
        diag->va_ov = (uint8_t)((d[4] >> 7u) & 0x01u);
        diag->va_uv = (uint8_t)((d[4] >> 6u) & 0x01u);
        diag->vd_ov = (uint8_t)((d[4] >> 5u) & 0x01u);
        diag->vd_uv = (uint8_t)((d[4] >> 4u) & 0x01u);
        diag->ced = (uint8_t)((d[4] >> 3u) & 0x01u);
        diag->cmed = (uint8_t)((d[4] >> 2u) & 0x01u);
        diag->sed = (uint8_t)((d[4] >> 1u) & 0x01u);
        diag->smed = (uint8_t)(d[4] & 0x01u);
        diag->vdel = (uint8_t)((d[5] >> 7u) & 0x01u);
        diag->vde = (uint8_t)((d[5] >> 6u) & 0x01u);
        diag->comp = (uint8_t)((d[5] >> 5u) & 0x01u);
        diag->spiflt = (uint8_t)((d[5] >> 4u) & 0x01u);
        diag->sleep = (uint8_t)((d[5] >> 3u) & 0x01u);
        diag->thsd = (uint8_t)((d[5] >> 2u) & 0x01u);
        diag->tmodchk = (uint8_t)((d[5] >> 1u) & 0x01u);
        diag->oscchk = (uint8_t)(d[5] & 0x01u);

        dev->ics[curr_ic].statc.cs_flt = diag->cs_flt_mask;
        dev->ics[curr_ic].statc.va_ov = diag->va_ov;
        dev->ics[curr_ic].statc.va_uv = diag->va_uv;
        dev->ics[curr_ic].statc.vd_ov = diag->vd_ov;
        dev->ics[curr_ic].statc.vd_uv = diag->vd_uv;
        dev->ics[curr_ic].statc.otp1_ed = diag->ced;
        dev->ics[curr_ic].statc.otp1_med = diag->cmed;
        dev->ics[curr_ic].statc.otp2_ed = diag->sed;
        dev->ics[curr_ic].statc.otp2_med = diag->smed;
        dev->ics[curr_ic].statc.vde = diag->vde;
        dev->ics[curr_ic].statc.vdel = diag->vdel;
        dev->ics[curr_ic].statc.comp = diag->comp;
        dev->ics[curr_ic].statc.spiflt = diag->spiflt;
        dev->ics[curr_ic].statc.sleep = diag->sleep;
        dev->ics[curr_ic].statc.thsd = diag->thsd;
        dev->ics[curr_ic].statc.tmodchk = diag->tmodchk;
        dev->ics[curr_ic].statc.oscchk = diag->oscchk;
    }
}

static void adbms6830_parse_statd(adbms6830_driver_t *dev, const uint8_t *data)
{
    if((dev == NULL) || (dev->ics == NULL) || (data == NULL))
    {
        return;
    }

    for(uint8_t curr_ic = 0u; (curr_ic < (uint8_t)dev->num_ics) && (curr_ic < ADBMS6830_MAX_TRACKED_ICS); curr_ic++)
    {
        const uint8_t *d = &data[(uint16_t)curr_ic * RX_DATA];
        bool pec_ok = adbms6830_read_packet_pec_ok(d);
        bool packet_valid = pec_ok && adbms6830_read_packet_counter_ok(dev, curr_ic);
        adbms6830_ic_diag_t *diag = &dev->diag[curr_ic];

        dev->ics[curr_ic].cccrc.stat_pec = pec_ok ? 0u : 1u;
        diag->statd_valid = packet_valid;
        if(!packet_valid)
        {
            continue;
        }

        diag->cell_ov_mask = 0u;
        diag->cell_uv_mask = 0u;
        for(uint8_t cell = 0u; cell < CELL; cell++)
        {
            uint8_t byte = d[cell / 4u];
            uint8_t shift = (uint8_t)((cell % 4u) * 2u);
            uint8_t uv = (uint8_t)((byte >> shift) & 0x01u);
            uint8_t ov = (uint8_t)((byte >> (shift + 1u)) & 0x01u);
            dev->ics[curr_ic].statd.c_uv[cell] = uv;
            dev->ics[curr_ic].statd.c_ov[cell] = ov;
            if(uv)
            {
                diag->cell_uv_mask |= (uint16_t)(1u << cell);
            }
            if(ov)
            {
                diag->cell_ov_mask |= (uint16_t)(1u << cell);
            }
        }

        diag->osc_counter = d[5];
        dev->ics[curr_ic].statd.oc_cntr = d[5];
    }
}

static void adbms6830_parse_state(adbms6830_driver_t *dev, const uint8_t *data)
{
    if((dev == NULL) || (dev->ics == NULL) || (data == NULL))
    {
        return;
    }

    for(uint8_t curr_ic = 0u; (curr_ic < (uint8_t)dev->num_ics) && (curr_ic < ADBMS6830_MAX_TRACKED_ICS); curr_ic++)
    {
        const uint8_t *d = &data[(uint16_t)curr_ic * RX_DATA];
        bool pec_ok = adbms6830_read_packet_pec_ok(d);
        bool packet_valid = pec_ok && adbms6830_read_packet_counter_ok(dev, curr_ic);
        adbms6830_ic_diag_t *diag = &dev->diag[curr_ic];

        dev->ics[curr_ic].cccrc.stat_pec = pec_ok ? 0u : 1u;
        diag->state_valid = packet_valid;
        if(!packet_valid)
        {
            continue;
        }

        diag->gpi_mask = (uint16_t)d[4] | ((uint16_t)(d[5] & 0x03u) << 8u);
        diag->revision = (uint8_t)((d[5] >> 4u) & 0x0Fu);
        dev->ics[curr_ic].state.gpi = diag->gpi_mask;
        dev->ics[curr_ic].state.rev = diag->revision;
    }
}

static void adbms6830_refresh_status_health(adbms6830_driver_t *dev)
{
    uint16_t expected_mask;
    uint16_t cell_mask;

    if(!adbms6830_topology_valid(dev))
    {
        return;
    }

    expected_mask = adbms6830_expected_ic_mask(dev);
    cell_mask = adbms6830_monitored_cell_mask(dev);
    dev->health.status_invalid_ic_mask = 0u;
    dev->health.status_fault_ic_mask = 0u;
    dev->health.reference_invalid_ic_mask = 0u;
    dev->health.reference_fault_ic_mask = 0u;
    dev->health.cs_fault_ic_mask = 0u;
    dev->health.supply_flag_fault_ic_mask = 0u;
    dev->health.memory_fault_ic_mask = 0u;
    dev->health.digital_fault_ic_mask = 0u;
    dev->health.oscillator_counter_fault_ic_mask = 0u;
    dev->health.cell_ovuv_fault_ic_mask = 0u;

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        const adbms6830_ic_diag_t *diag = &dev->diag[ic];
        uint16_t ic_bit = (uint16_t)(1u << ic);
        int32_t vres_delta_mv;

        if(!diag->statc_valid || !diag->statd_valid || !diag->state_valid ||
           (cell_mask == 0u))
        {
            dev->health.status_invalid_ic_mask |= ic_bit;
        }
        else
        {
            if((diag->cs_flt_mask & cell_mask) != 0u)
            {
                dev->health.cs_fault_ic_mask |= ic_bit;
            }
            if((diag->va_ov != 0u) || (diag->va_uv != 0u) ||
               (diag->vd_ov != 0u) || (diag->vd_uv != 0u))
            {
                dev->health.supply_flag_fault_ic_mask |= ic_bit;
            }
            /* Treat both correctable and multiple-bit OTP errors as safety
             * faults.  A corrected read is useful diagnostically but is not a
             * clean basis for asserting BMS_OK. */
            if((diag->ced != 0u) || (diag->cmed != 0u) ||
               (diag->sed != 0u) || (diag->smed != 0u))
            {
                dev->health.memory_fault_ic_mask |= ic_bit;
            }
            /* Rev. 0 Table 91 and ADI's Release 1.0.3 parser disagree on the
             * VDE/VDEL names for bits 6 and 7.  Both raw bits are gated here,
             * so that naming ambiguity cannot create a fail-open path. */
            /* COMP is an activity indication, not a fault: it is expected to
             * be one while the configured C-ADC/S-ADC comparison is running.
             * The actual comparison failures are reported in CSxFLT. */
            if((diag->vde != 0u) || (diag->vdel != 0u) ||
               (diag->spiflt != 0u) ||
               (diag->sleep != 0u) || (diag->thsd != 0u) ||
               (diag->tmodchk != 0u) || (diag->oscchk != 0u))
            {
                dev->health.digital_fault_ic_mask |= ic_bit;
            }
            /* OSCCHK is the monitor's own pass/fail bit. Independently check
             * the reported count against the documented passing range so a
             * corrupt or inconsistent flag image cannot mask the failure. */
            if((diag->osc_counter < ADBMS6830_OSC_COUNTER_MIN) ||
               (diag->osc_counter > ADBMS6830_OSC_COUNTER_MAX))
            {
                dev->health.oscillator_counter_fault_ic_mask |= ic_bit;
            }
            if(((diag->cell_ov_mask | diag->cell_uv_mask) & cell_mask) != 0u)
            {
                dev->health.cell_ovuv_fault_ic_mask |= ic_bit;
            }
        }

        if(!diag->stata_valid || !diag->statb_valid ||
           !diag->reference_values_valid)
        {
            dev->health.reference_invalid_ic_mask |= ic_bit;
        }
        else
        {
            vres_delta_mv = (int32_t)diag->vres_mv - (int32_t)diag->vref2_mv;
            if(vres_delta_mv < 0)
            {
                vres_delta_mv = -vres_delta_mv;
            }
            if((diag->vref2_mv < ADBMS6830_VREF2_MIN_MV) ||
               (diag->vref2_mv > ADBMS6830_VREF2_MAX_MV) ||
               (diag->vd_mv < ADBMS6830_VD_MIN_MV) ||
               (diag->vd_mv > ADBMS6830_VD_MAX_MV) ||
               (diag->va_mv < ADBMS6830_VA_MIN_MV) ||
               (diag->va_mv > ADBMS6830_VA_MAX_MV) ||
               (vres_delta_mv > ADBMS6830_VRES_MAX_DELTA_MV) ||
               (diag->die_temp_deci_c < ADBMS6830_DIE_TEMP_MIN_DECI_C) ||
               (diag->die_temp_deci_c > ADBMS6830_DIE_TEMP_MAX_DECI_C))
            {
                dev->health.reference_fault_ic_mask |= ic_bit;
            }
        }
    }

    dev->health.status_invalid_ic_mask &= expected_mask;
    dev->health.reference_invalid_ic_mask &= expected_mask;
    dev->health.reference_fault_ic_mask &= expected_mask;
    dev->health.status_fault_ic_mask =
        (uint16_t)((dev->health.cs_fault_ic_mask |
                    dev->health.supply_flag_fault_ic_mask |
                    dev->health.memory_fault_ic_mask |
                    dev->health.digital_fault_ic_mask |
                    dev->health.oscillator_counter_fault_ic_mask) & expected_mask);
    /* Cell OV/UV is kept in its dedicated mask and evaluated by the voltage
     * policy.  It is a legitimate operating protection condition, not a
     * transport/silicon diagnostic failure and must not pollute the generic
     * sticky status-fault history. */
    dev->health.sticky_status_fault_ic_mask |=
        (uint16_t)(dev->health.status_invalid_ic_mask |
                   dev->health.status_fault_ic_mask);
    dev->health.sticky_reference_fault_ic_mask |=
        (uint16_t)(dev->health.reference_invalid_ic_mask |
                   dev->health.reference_fault_ic_mask);
}

bool adbms6830_diagnostic_transport_ok(const adbms6830_driver_t *dev)
{
    const adbms6830_diag_health_t *health;
    uint16_t expected_mask;

    if(!adbms6830_topology_valid(dev))
    {
        return false;
    }

    health = &dev->health;
    expected_mask = adbms6830_expected_ic_mask(dev);

    return (expected_mask != 0u) &&
           ((health->sid_valid_ic_mask & expected_mask) == expected_mask) &&
           ((health->sid_identity_mismatch_ic_mask & expected_mask) == 0u) &&
           ((health->status_invalid_ic_mask & expected_mask) == 0u) &&
           ((health->reference_invalid_ic_mask & expected_mask) == 0u);
}

bool adbms6830_non_cs_diagnostics_ok(const adbms6830_driver_t *dev)
{
    const adbms6830_diag_health_t *health;
    uint16_t expected_mask;
    uint16_t non_cs_fault_mask;

    if(!adbms6830_diagnostic_transport_ok(dev))
    {
        return false;
    }

    health = &dev->health;
    expected_mask = adbms6830_expected_ic_mask(dev);
    non_cs_fault_mask =
        (uint16_t)((health->supply_flag_fault_ic_mask |
                    health->memory_fault_ic_mask |
                    health->digital_fault_ic_mask |
                    health->oscillator_counter_fault_ic_mask |
                    health->reference_fault_ic_mask) & expected_mask);

    return non_cs_fault_mask == 0u;
}

static bool adbms6830_current_diagnostic_image_ok(const adbms6830_driver_t *dev)
{
    const adbms6830_diag_health_t *health;
    uint16_t expected_mask;

    if(!adbms6830_non_cs_diagnostics_ok(dev))
    {
        return false;
    }

    health = &dev->health;
    expected_mask = adbms6830_expected_ic_mask(dev);

#if AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP
    /* The known SMB routing defect leaves S2N-S15N without their required
     * lower-cell references. In the explicit MVP build only, preserve and
     * report CSxFLT but do not let that one fault class prevent C-ADC bring-up.
     * Every other status/reference/identity fault remains blocking. */
    (void)health;
    (void)expected_mask;
    return true;
#else
    return (health->cs_fault_ic_mask & expected_mask) == 0u;
#endif
}

void adbms6830_spi_debug_enable(adbms6830_driver_t *dev, bool enable)
{
    if(dev == NULL)
    {
        return;
    }

    dev->spi_debug.enabled = enable;
}

void adbms6830_spi_debug_clear(adbms6830_driver_t *dev)
{
    bool was_enabled;

    if(dev == NULL)
    {
        return;
    }

    was_enabled = dev->spi_debug.enabled;
    memset(&dev->spi_debug, 0, sizeof(dev->spi_debug));
    /* WRCFGB history is a separate safety audit trail.  A routine SPI-counter
     * clear must not erase the evidence needed to identify a delayed shadow
     * mutation; the bounded ring is reset only by driver initialization. */
    dev->spi_debug.enabled = was_enabled;
    dev->spi_debug.last_status = HAL_OK;
    dev->spi_debug.last_tx_status = HAL_OK;
    dev->spi_debug.last_rx_status = HAL_OK;
    dev->spi_debug.last_xfer_status = HAL_OK;
}

const adbms6830_spi_debug_t *adbms6830_spi_debug_get(const adbms6830_driver_t *dev)
{
    return (dev == NULL) ? NULL : &dev->spi_debug;
}

const char *adbms6830_spi_op_str(adbms6830_spi_op_t op)
{
    switch(op)
    {
    case ADBMS6830_SPI_OP_NONE:   return "none";
    case ADBMS6830_SPI_OP_CMD:    return "cmd";
    case ADBMS6830_SPI_OP_WR48:   return "wr48";
    case ADBMS6830_SPI_OP_RD48:   return "rd48";
    case ADBMS6830_SPI_OP_STCOMM: return "stcomm";
    case ADBMS6830_SPI_OP_PROBE:  return "probe";
    case ADBMS6830_SPI_OP_WAKE:   return "wake";
    case ADBMS6830_SPI_OP_COLD_WAKE: return "cold_wake";
    case ADBMS6830_SPI_OP_READ_SID: return "read_sid";
    case ADBMS6830_SPI_OP_READ_STATUS: return "read_status";
    case ADBMS6830_SPI_OP_DIAGNOSTIC_REFRESH: return "diag_refresh";
    case ADBMS6830_SPI_OP_STARTUP_BASELINE: return "startup_baseline";
    case ADBMS6830_SPI_OP_CLEAR_FLAGS: return "clear_flags";
    case ADBMS6830_SPI_OP_CONFIG_CHECK: return "config_check";
    case ADBMS6830_SPI_OP_BALANCE_CHECK: return "balance_check";
    case ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST: return "cell_adc_diag";
    case ADBMS6830_SPI_OP_CS_COMPARE: return "cs_compare";
    case ADBMS6830_SPI_OP_S_ADC_DUMP: return "s_adc_dump";
    case ADBMS6830_SPI_OP_C_ADC_DUMP: return "c_adc_dump";
    case ADBMS6830_SPI_OP_CONVERSION_TIMING: return "conv_timing";
    case ADBMS6830_SPI_OP_CONFIG_STRESS: return "config_stress";
    case ADBMS6830_SPI_OP_RECOVERY: return "recovery";
    case ADBMS6830_SPI_OP_RAW_DUMP: return "raw_dump";
    case ADBMS6830_SPI_OP_OPEN_WIRE_BASELINE: return "open_wire_baseline";
    case ADBMS6830_SPI_OP_OPEN_WIRE_EVEN: return "open_wire_even";
    case ADBMS6830_SPI_OP_OPEN_WIRE_ODD: return "open_wire_odd";
    case ADBMS6830_SPI_OP_OPEN_WIRE_FULL: return "open_wire_full";
    case ADBMS6830_SPI_OP_AUX_GPIO_DIAG: return "aux_gpio_diag";
    case ADBMS6830_SPI_OP_SCOPE: return "scope";
    default:                      return "unknown";
    }
}

static void adbms6830_spi_debug_note_tx(adbms6830_driver_t *dev,
                                        adbms6830_spi_op_t op,
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
        preview_len = adbms6830_min_u16(tx_len, ADBMS6830_SPI_DEBUG_PREVIEW_BYTES);
        memcpy(dev->spi_debug.last_tx_preview, tx, preview_len);
    }
}

static void adbms6830_spi_debug_note_rx(adbms6830_driver_t *dev,
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
        preview_len = adbms6830_min_u16(rx_len, ADBMS6830_SPI_DEBUG_PREVIEW_BYTES);
        memcpy(dev->spi_debug.last_rx_preview, rx, preview_len);
    }

    dev->spi_debug.last_status = status;
    dev->spi_debug.last_xfer_status = status;
}

HAL_StatusTypeDef adbms6830_spi_probe_rdcfga(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef status;

    if(dev == NULL)
    {
        return HAL_ERROR;
    }

    adbms6830_spi_debug_enable(dev, true);

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_PROBE;
    }

    status = adbms6830_rd48_checked(dev, RDCFGA, shared_buf);
    if((status == HAL_OK) && adbms6830_last_read_integrity_ok(dev))
    {
        adbms6830_parse_cfga(dev, shared_buf);
    }
    else if(status == HAL_OK)
    {
        status = HAL_ERROR;
    }

    return status;
}

HAL_StatusTypeDef adbms6830_spi_probe_rdcfga_on_string(adbms6830_driver_t *dev, adbms_string string)
{
    HAL_StatusTypeDef status;
    adbms_string previous;

    if((dev == NULL) || !adbms6830_string_valid(string))
    {
        return HAL_ERROR;
    }

    previous = dev->string;
    dev->string = string;
    status = adbms6830_spi_probe_rdcfga(dev);
    dev->string = previous;

    return status;
}

HAL_StatusTypeDef adbms6830_scope_activity(adbms6830_driver_t *dev,
                                           adbms_string string,
                                           adbms6830_scope_mode_t mode,
                                           uint16_t repeat_count)
{
    static uint8_t scope_pattern[] =
    {
        0xAAu, 0x55u, 0xFFu, 0x00u, 0x69u, 0x96u, 0x12u, 0x34u
    };
    HAL_StatusTypeDef status = HAL_OK;
    adbms_string previous;
    uint16_t repeat;

    if((dev == NULL) || !adbms6830_string_valid(string) || (repeat_count == 0u))
    {
        return HAL_ERROR;
    }

    repeat = (repeat_count > 100u) ? 100u : repeat_count;
    previous = dev->string;
    dev->string = string;
    adbms6830_spi_debug_enable(dev, true);

    for(uint16_t i = 0u; i < repeat; i++)
    {
        if(dev->spi_debug.enabled)
        {
            dev->spi_debug.last_op = ADBMS6830_SPI_OP_SCOPE;
        }

        switch(mode)
        {
        case ADBMS6830_SCOPE_WAKE:
            status = adbms6830_wakeup_checked(dev);
            break;
        case ADBMS6830_SCOPE_CMD:
            status = adbms6830_wakeup_checked(dev);
            if(status == HAL_OK)
            {
                status = adbms6830_cmd_checked(dev, RDCFGA);
            }
            if(dev->spi_debug.enabled)
            {
                dev->spi_debug.last_op = ADBMS6830_SPI_OP_SCOPE;
            }
            break;
        case ADBMS6830_SCOPE_READ:
            status = adbms6830_rd48_checked(dev, RDCFGA, shared_buf);
            if(dev->spi_debug.enabled)
            {
                dev->spi_debug.last_op = ADBMS6830_SPI_OP_SCOPE;
            }
            break;
        case ADBMS6830_SCOPE_PATTERN:
            status = adbms6830_spi_write(dev,
                                         scope_pattern,
                                         (uint16_t)sizeof(scope_pattern),
                                         1u);
            break;
        default:
            status = HAL_ERROR;
            break;
        }

        if(status != HAL_OK)
        {
            break;
        }

        status = adbms6830_wait_cooperative(dev, 2000u);
        if(status != HAL_OK)
        {
            break;
        }
    }

    dev->string = previous;
    return status;
}

HAL_StatusTypeDef adbms6830_read_sid(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef status;
    uint16_t expected_mask;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    expected_mask = adbms6830_expected_ic_mask(dev);
    dev->health.sid_valid_ic_mask = 0u;
    dev->health.sid_identity_mismatch_ic_mask = 0u;
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        dev->diag[ic].sid_valid = false;
        dev->diag[ic].device_id = UINT8_MAX;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_READ_SID;
    }

    status = adbms6830_rd48_checked(dev, RDSID, shared_buf);
    if(status == HAL_OK)
    {
        adbms6830_parse_sid(dev, shared_buf);
        if(!adbms6830_last_read_integrity_ok(dev) ||
           ((dev->health.sid_valid_ic_mask & expected_mask) != expected_mask) ||
           ((dev->health.sid_identity_mismatch_ic_mask & expected_mask) != 0u))
        {
            status = HAL_ERROR;
        }
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_READ_SID;
    }

    return status;
}

HAL_StatusTypeDef adbms6830_read_status(adbms6830_driver_t *dev, bool inject_spiflt)
{
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    bool session_owned = false;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    status = adbms6830_read_session_begin(dev, &session_owned);
    if(status != HAL_OK)
    {
        return status;
    }

    for(uint8_t ic = 0u;
        (ic < (uint8_t)dev->num_ics) && (ic < ADBMS6830_MAX_TRACKED_ICS);
        ic++)
    {
        dev->diag[ic].statc_valid = false;
        dev->diag[ic].statd_valid = false;
        dev->diag[ic].state_valid = false;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_READ_STATUS;
    }

    status = adbms6830_rd48_checked(dev, inject_spiflt ? RDSTATCERR : RDSTATC, shared_buf);
    if(status == HAL_OK)
    {
        adbms6830_parse_statc(dev, shared_buf);
        if(!adbms6830_last_read_integrity_ok(dev))
        {
            first_error = HAL_ERROR;
        }
    }
    else
    {
        first_error = status;
    }

    status = adbms6830_rd48_checked(dev, RDSTATD, shared_buf);
    if(status == HAL_OK)
    {
        adbms6830_parse_statd(dev, shared_buf);
        if(!adbms6830_last_read_integrity_ok(dev) && (first_error == HAL_OK))
        {
            first_error = HAL_ERROR;
        }
    }
    else if(first_error == HAL_OK)
    {
        first_error = status;
    }

    status = adbms6830_rd48_checked(dev, RDSTATE, shared_buf);
    if(status == HAL_OK)
    {
        adbms6830_parse_state(dev, shared_buf);
        if(!adbms6830_last_read_integrity_ok(dev) && (first_error == HAL_OK))
        {
            first_error = HAL_ERROR;
        }
    }
    else if(first_error == HAL_OK)
    {
        first_error = status;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_READ_STATUS;
    }

    adbms6830_refresh_status_health(dev);
    adbms6830_read_session_end(dev, session_owned);

    return first_error;
}

HAL_StatusTypeDef adbms6830_refresh_diagnostics(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    uint8_t adax_cmd[2] = { ADAX_CMD_BYTE0, ADAX_CMD_BYTE1 };

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    adbms6830_increment_u32_sat(&dev->health.diagnostic_refresh_count);
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        dev->diag[ic].stata_valid = false;
        dev->diag[ic].statb_valid = false;
        dev->diag[ic].statc_valid = false;
        dev->diag[ic].statd_valid = false;
        dev->diag[ic].state_valid = false;
        dev->diag[ic].reference_values_valid = false;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_DIAGNOSTIC_REFRESH;
    }

    /* ADAX with AUX_ALL updates VREF2, ITMP, VD, VA and VRES before Status
     * A/B are evaluated.  Reading stale reset values is never accepted. */
    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adax_cmd);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_wait_cooperative(dev, ADBMS6830_AUX_CONVERSION_WAIT_US);
    }
    if(status != HAL_OK)
    {
        first_error = status;
    }

    if(first_error == HAL_OK)
    {
        bool read_session_owned = false;
        status = adbms6830_read_session_begin(dev, &read_session_owned);
        if(status != HAL_OK)
        {
            first_error = status;
        }

        if(first_error == HAL_OK)
        {
        status = adbms6830_rd48_checked(dev, RDSTATA, shared_buf);
        if(status == HAL_OK)
        {
            adbms6830_parse_stata(dev, shared_buf);
            if(!adbms6830_last_read_integrity_ok(dev))
            {
                first_error = HAL_ERROR;
            }
        }
        else
        {
            first_error = status;
        }

        status = adbms6830_rd48_checked(dev, RDSTATB, shared_buf);
        if(status == HAL_OK)
        {
            adbms6830_parse_statb(dev, shared_buf);
            if(!adbms6830_last_read_integrity_ok(dev) && (first_error == HAL_OK))
            {
                first_error = HAL_ERROR;
            }
        }
        else if(first_error == HAL_OK)
        {
            first_error = status;
        }

        status = adbms6830_read_status(dev, false);
        if((status != HAL_OK) && (first_error == HAL_OK))
        {
            first_error = status;
        }
        }
        adbms6830_read_session_end(dev, read_session_owned);
    }

    adbms6830_refresh_status_health(dev);
    if((first_error == HAL_OK) && !adbms6830_current_diagnostic_image_ok(dev))
    {
        first_error = HAL_ERROR;
    }

    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_DIAGNOSTIC_REFRESH, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_DIAGNOSTIC_REFRESH;
        dev->spi_debug.last_status = first_error;
    }
    return first_error;
}

HAL_StatusTypeDef adbms6830_establish_diagnostic_baseline(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef status;

    if(!adbms6830_topology_valid(dev) || (adbms6830_monitored_cell_mask(dev) == 0u))
    {
        return HAL_ERROR;
    }

    adbms6830_increment_u32_sat(&dev->health.startup_baseline_count);
    dev->health.startup_baseline_passed = false;

    /* Status C/D flags power up asserted.  Clear both the general latched
     * flags and cell OV/UV latches once, then run real conversions and require
     * the newly produced diagnostic image to be clean. */
    status = adbms6830_clear_all_flags(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_clear_all_ovuv_flags(dev);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_adcv_checked(dev,
                                        RD_ON,
                                        SINGLE,
                                        DCP_OFF,
                                        RSTF_ON,
                                        OW_OFF_ALL_CH);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_wait_cooperative(dev,
                                    ADBMS6830_REDUNDANT_CONVERSION_WAIT_US);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_refresh_diagnostics(dev);
    }

    if((status == HAL_OK) && adbms6830_current_diagnostic_image_ok(dev))
    {
        dev->health.startup_baseline_passed = true;
    }
    else if(status == HAL_OK)
    {
        status = HAL_ERROR;
    }

    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_STARTUP_BASELINE, status);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_STARTUP_BASELINE;
        dev->spi_debug.last_status = status;
    }
    return status;
}

bool adbms6830_safety_diagnostics_ok(const adbms6830_driver_t *dev)
{
    return adbms6830_current_diagnostic_image_ok(dev) &&
           dev->health.startup_baseline_passed;
}

static HAL_StatusTypeDef adbms6830_clear_all_ovuv_flags(adbms6830_driver_t *dev)
{
    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    /* CLOVUV is a WR48 command, not a command-only transaction.  Bytes 0..3
     * select the sixteen UV/OV pairs to clear; bytes 4..5 are reserved and
     * are deliberately written as zero. */
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        uint16_t offset = (uint16_t)ic * TX_DATA;
        shared_buf[offset + 0u] = 0xFFu;
        shared_buf[offset + 1u] = 0xFFu;
        shared_buf[offset + 2u] = 0xFFu;
        shared_buf[offset + 3u] = 0xFFu;
        shared_buf[offset + 4u] = 0x00u;
        shared_buf[offset + 5u] = 0x00u;
    }

    return adbms6830_wr48_checked(dev, CLOVUV, shared_buf);
}

HAL_StatusTypeDef adbms6830_clear_all_flags(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef status;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    for(uint8_t curr_ic = 0u; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
    {
        dev->ics[curr_ic].clflag.cl_csflt = 0xFFFFu;
        dev->ics[curr_ic].clflag.cl_smed = 1u;
        dev->ics[curr_ic].clflag.cl_sed = 1u;
        dev->ics[curr_ic].clflag.cl_cmed = 1u;
        dev->ics[curr_ic].clflag.cl_ced = 1u;
        dev->ics[curr_ic].clflag.cl_vduv = 1u;
        dev->ics[curr_ic].clflag.cl_vdov = 1u;
        dev->ics[curr_ic].clflag.cl_vauv = 1u;
        dev->ics[curr_ic].clflag.cl_vaov = 1u;
        dev->ics[curr_ic].clflag.cl_oscchk = 1u;
        dev->ics[curr_ic].clflag.cl_tmode = 1u;
        dev->ics[curr_ic].clflag.cl_thsd = 1u;
        dev->ics[curr_ic].clflag.cl_sleep = 1u;
        dev->ics[curr_ic].clflag.cl_spiflt = 1u;
        dev->ics[curr_ic].clflag.cl_vdel = 1u;
        dev->ics[curr_ic].clflag.cl_vde = 1u;
    }

    adbms6830_pack_clr_flag_data(dev);
    for(uint8_t curr_ic = 0u; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
    {
        for(uint8_t data = 0u; data < TX_DATA; data++)
        {
            shared_buf[((uint16_t)curr_ic * TX_DATA) + data] =
                dev->ics[curr_ic].clrflag.tx_data[data];
        }
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CLEAR_FLAGS;
    }

    status = adbms6830_wr48_checked(dev, CLRFLAG, shared_buf);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CLEAR_FLAGS;
    }
    return status;
}

const adbms6830_diag_health_t *adbms6830_diag_health_get(const adbms6830_driver_t *dev)
{
    return (dev == NULL) ? NULL : &dev->health;
}

void adbms6830_diag_health_clear(adbms6830_driver_t *dev)
{
    bool startup_baseline_passed;
    uint16_t sid_valid_ic_mask;
    uint16_t sid_identity_mismatch_ic_mask;

    if(dev == NULL)
    {
        return;
    }

    startup_baseline_passed = dev->health.startup_baseline_passed;
    sid_valid_ic_mask = dev->health.sid_valid_ic_mask;
    sid_identity_mismatch_ic_mask = dev->health.sid_identity_mismatch_ic_mask;
    memset(&dev->health, 0, sizeof(dev->health));
    dev->health.last_status = HAL_OK;
    dev->health.startup_baseline_passed = startup_baseline_passed;
    dev->health.sid_valid_ic_mask = sid_valid_ic_mask;
    dev->health.sid_identity_mismatch_ic_mask = sid_identity_mismatch_ic_mask;
    adbms6830_refresh_status_health(dev);
}

HAL_StatusTypeDef adbms6830_verify_config_readback(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    bool cfga_verified = false;
    bool cfgb_verified = false;
    bool session_owned = false;
    uint16_t expected_mask;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    expected_mask = adbms6830_expected_ic_mask(dev);
    adbms6830_increment_u32_sat(&dev->health.config_readback_count);
    dev->health.configa_mismatch_mask = 0u;
    dev->health.configb_mismatch_mask = 0u;
    dev->health.config_mismatch_mask = 0u;

    adbms6830_pack_cfga(dev);
    adbms6830_pack_cfgb(dev);

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CONFIG_CHECK;
    }

    status = adbms6830_read_session_begin(dev, &session_owned);
    if(status != HAL_OK)
    {
        dev->health.configa_mismatch_mask = expected_mask;
        dev->health.configb_mismatch_mask = expected_mask;
        dev->health.config_mismatch_mask = expected_mask;
        adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_CONFIG_CHECK, status);
        return status;
    }

    status = adbms6830_rd48_checked(dev, RDCFGA, shared_buf);
    if((status == HAL_OK) && adbms6830_last_read_integrity_ok(dev))
    {
        adbms6830_parse_cfga(dev, shared_buf);
        cfga_verified = true;
    }
    else
    {
        first_error = (status == HAL_OK) ? HAL_ERROR : status;
        dev->health.configa_mismatch_mask = expected_mask;
    }

    status = adbms6830_rd48_checked(dev, RDCFGB, shared_buf);
    if((status == HAL_OK) && adbms6830_last_read_integrity_ok(dev))
    {
        adbms6830_parse_cfgb(dev, shared_buf);
        cfgb_verified = true;
    }
    else if(first_error == HAL_OK)
    {
        first_error = (status == HAL_OK) ? HAL_ERROR : status;
    }
    if(!cfgb_verified)
    {
        dev->health.configb_mismatch_mask = expected_mask;
    }

    for(uint8_t ic = 0u; (ic < (uint8_t)dev->num_ics) && (ic < ADBMS6830_MAX_TRACKED_ICS); ic++)
    {
        uint16_t bit = (uint16_t)(1u << ic);
        bool mismatch = false;
        bool actual_cfg_mismatch = false;

        if(!cfga_verified ||
           (memcmp(dev->ics[ic].configa.tx_data,
                   dev->ics[ic].configa.rx_data,
                   TX_DATA) != 0))
        {
            dev->health.configa_mismatch_mask |= bit;
            mismatch = true;
            actual_cfg_mismatch = true;
        }
        if(!cfgb_verified ||
           (memcmp(dev->ics[ic].configb.tx_data,
                   dev->ics[ic].configb.rx_data,
                   TX_DATA) != 0))
        {
            dev->health.configb_mismatch_mask |= bit;
            mismatch = true;
            actual_cfg_mismatch = true;
        }
        else
        {
            /* A rejected unauthorized timer payload remains safety-visible
             * until a later transport-clean read proves the device still
             * contains the reviewed safe image. */
            dev->health.config_write_guard_fault_mask &= (uint16_t)~bit;
        }
        if((dev->health.config_write_guard_fault_mask & bit) != 0u)
        {
            dev->health.configb_mismatch_mask |= bit;
            mismatch = true;
        }
        if(mismatch)
        {
            dev->health.config_mismatch_mask |= bit;
            if(actual_cfg_mismatch)
            {
                adbms6830_increment_u32_sat(&dev->health.config_mismatch_count[ic]);
            }
        }
    }

    if((first_error == HAL_OK) && (dev->health.config_mismatch_mask != 0u))
    {
        first_error = HAL_ERROR;
    }

    adbms6830_read_session_end(dev, session_owned);
    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_CONFIG_CHECK, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CONFIG_CHECK;
        dev->spi_debug.last_status = first_error;
    }

    return first_error;
}

HAL_StatusTypeDef adbms6830_verify_balance_readback(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    uint16_t expected_mask;
    bool session_owned = false;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    expected_mask = adbms6830_expected_ic_mask(dev);
    adbms6830_increment_u32_sat(&dev->health.balance_readback_count);
    dev->health.balance_cfgb_mismatch_mask = 0u;
    dev->health.balance_pwma_mismatch_mask = 0u;
    dev->health.balance_pwmb_mismatch_mask = 0u;
    dev->health.balance_mismatch_mask = 0u;

    adbms6830_pack_cfgb(dev);
    adbms6830_pack_pwma(dev);
    adbms6830_pack_pwmb(dev);

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_BALANCE_CHECK;
    }

    /* RDCFGB/RDPWMA/RDPWMB are one logical verification epoch.  Establish
     * chain readiness once and keep these back-to-back reads in the same
     * short awake session rather than paying a full six-device wake before
     * every register group.  No long conversion wait occurs in this epoch. */
    status = adbms6830_read_session_begin(dev, &session_owned);
    if(status != HAL_OK)
    {
        dev->health.balance_cfgb_mismatch_mask = expected_mask;
        dev->health.balance_pwma_mismatch_mask = expected_mask;
        dev->health.balance_pwmb_mismatch_mask = expected_mask;
        dev->health.balance_mismatch_mask = expected_mask;
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            adbms6830_increment_u32_sat(&dev->health.balance_mismatch_count[ic]);
        }
        first_error = status;
        adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_BALANCE_CHECK, first_error);
        if(dev->spi_debug.enabled)
        {
            dev->spi_debug.last_status = first_error;
        }
        return first_error;
    }

    status = adbms6830_rd48_checked(dev, RDCFGB, shared_buf);
    if((status == HAL_OK) && adbms6830_last_read_integrity_ok(dev))
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            if(memcmp(dev->ics[ic].configb.tx_data,
                      &shared_buf[(uint16_t)ic * RX_DATA],
                      TX_DATA) != 0)
            {
                dev->health.balance_cfgb_mismatch_mask |= (uint16_t)(1u << ic);
            }
        }
    }
    else
    {
        dev->health.balance_cfgb_mismatch_mask = expected_mask;
        first_error = (status == HAL_OK) ? HAL_ERROR : status;
    }

    status = adbms6830_rd48_checked(dev, RDPWM1, shared_buf);
    if((status == HAL_OK) && adbms6830_last_read_integrity_ok(dev))
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            if(memcmp(dev->ics[ic].pwma.tx_data,
                      &shared_buf[(uint16_t)ic * RX_DATA],
                      TX_DATA) != 0)
            {
                dev->health.balance_pwma_mismatch_mask |= (uint16_t)(1u << ic);
            }
        }
    }
    else
    {
        dev->health.balance_pwma_mismatch_mask = expected_mask;
        if(first_error == HAL_OK)
        {
            first_error = (status == HAL_OK) ? HAL_ERROR : status;
        }
    }

    status = adbms6830_rd48_checked(dev, RDPWM2, shared_buf);
    if((status == HAL_OK) && adbms6830_last_read_integrity_ok(dev))
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            if(memcmp(dev->ics[ic].pwmb.tx_data,
                      &shared_buf[(uint16_t)ic * RX_DATA],
                      TX_DATA) != 0)
            {
                dev->health.balance_pwmb_mismatch_mask |= (uint16_t)(1u << ic);
            }
        }
    }
    else
    {
        dev->health.balance_pwmb_mismatch_mask = expected_mask;
        if(first_error == HAL_OK)
        {
            first_error = (status == HAL_OK) ? HAL_ERROR : status;
        }
    }

    dev->health.balance_mismatch_mask =
        (uint16_t)(dev->health.balance_cfgb_mismatch_mask |
                   dev->health.balance_pwma_mismatch_mask |
                   dev->health.balance_pwmb_mismatch_mask);

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        if((dev->health.balance_mismatch_mask & (uint16_t)(1u << ic)) != 0u)
        {
            adbms6830_increment_u32_sat(&dev->health.balance_mismatch_count[ic]);
        }
    }

    if((first_error == HAL_OK) && (dev->health.balance_mismatch_mask != 0u))
    {
        first_error = HAL_ERROR;
    }

    adbms6830_read_session_end(dev, session_owned);
    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_BALANCE_CHECK, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_BALANCE_CHECK;
        dev->spi_debug.last_status = first_error;
    }

    return first_error;
}

HAL_StatusTypeDef adbms6830_run_cell_adc_self_test(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef status;
    uint16_t monitored_mask;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    monitored_mask = adbms6830_monitored_cell_mask(dev);
    if(monitored_mask == 0u)
    {
        return HAL_ERROR;
    }

    adbms6830_increment_u32_sat(&dev->health.cell_adc_self_test_count);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST;
    }

    /* This legacy-named API is a conversion-path diagnostic, not a silicon
     * latent-fault self-test.  Start a redundant single-shot C/S conversion,
     * wait for the documented worst-case completion time, then require every
     * configured cell register to arrive with valid PEC/CCNT and a clean,
     * freshly converted Status A-E image.  PLCADC is intentionally not used:
     * a command-only transmit cannot observe or interpret its poll response.
     */
    status = adbms6830_adcv_checked(dev,
                                    RD_ON,
                                    SINGLE,
                                    DCP_OFF,
                                    RSTF_ON,
                                    OW_OFF_ALL_CH);
    if(status == HAL_OK)
    {
        status = adbms6830_wait_cooperative(dev,
                                    ADBMS6830_REDUNDANT_CONVERSION_WAIT_US);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_read_cell_voltages(dev);
    }
    if(status == HAL_OK)
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            if((dev->last_cell_updated_mask[ic] & monitored_mask) != monitored_mask)
            {
                status = HAL_ERROR;
                break;
            }
        }
    }
    if(status == HAL_OK)
    {
        status = adbms6830_refresh_diagnostics(dev);
    }

    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST, status);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST;
        dev->spi_debug.last_status = status;
    }
    return status;
}

static uint8_t adbms6830_cell_group_first_index(GRP group)
{
    switch(group)
    {
    case A: return 0u;
    case B: return 3u;
    case C: return 6u;
    case D: return 9u;
    case E: return 12u;
    case F: return 15u;
    default: return UINT8_MAX;
    }
}

static bool adbms6830_open_wire_code_to_mv(int16_t code, uint16_t *millivolts)
{
    int32_t mv;

    if((millivolts == NULL) || (code == INT16_MIN) ||
       (code == INT16_MAX) || (code == (int16_t)-1))
    {
        return false;
    }

    /* ADBMS6830 cell and S-channel result: V = (code + 10000) * 150 uV. */
    mv = (((int32_t)code + 10000) * 150) / 1000;
    if((mv < 0) || (mv > 5500))
    {
        return false;
    }

    *millivolts = (uint16_t)mv;
    return true;
}

static void adbms6830_open_wire_refresh_health(adbms6830_driver_t *dev)
{
    uint16_t expected_mask;
    uint16_t baseline_valid_mask = 0u;
    uint16_t even_valid_mask = 0u;
    uint16_t odd_valid_mask = 0u;
    uint16_t fault_ic_mask = 0u;

    if(!adbms6830_topology_valid(dev))
    {
        return;
    }

    expected_mask = adbms6830_expected_ic_mask(dev);
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        adbms6830_ic_diag_t *diag = &dev->diag[ic];
        uint16_t ic_bit = (uint16_t)(1u << ic);

        diag->open_wire_fault_mask =
            (uint16_t)(diag->open_wire_even_fault_mask |
                       diag->open_wire_odd_fault_mask);
        dev->health.open_wire_cell_fault_mask[ic] = diag->open_wire_fault_mask;
        if(diag->open_wire_baseline_valid)
        {
            baseline_valid_mask |= ic_bit;
        }
        if(diag->open_wire_even_valid)
        {
            even_valid_mask |= ic_bit;
        }
        if(diag->open_wire_odd_valid)
        {
            odd_valid_mask |= ic_bit;
        }
        if(diag->open_wire_fault_mask != 0u)
        {
            fault_ic_mask |= ic_bit;
        }
    }

    dev->health.open_wire_baseline_valid_ic_mask = baseline_valid_mask;
    dev->health.open_wire_even_valid_ic_mask = even_valid_mask;
    dev->health.open_wire_odd_valid_ic_mask = odd_valid_mask;
    dev->health.open_wire_incomplete_ic_mask =
        (uint16_t)(expected_mask &
                   (uint16_t)~(baseline_valid_mask & even_valid_mask & odd_valid_mask));
    dev->health.open_wire_fault_ic_mask = fault_ic_mask;
    dev->health.sticky_open_wire_fault_ic_mask |= fault_ic_mask;
}

bool adbms6830_set_monitored_cell_count(adbms6830_driver_t *dev, uint8_t cell_count)
{
    if(!adbms6830_topology_valid(dev) || (cell_count == 0u) || (cell_count > CELL))
    {
        return false;
    }

    dev->monitored_cell_count = cell_count;
    return true;
}

static HAL_StatusTypeDef adbms6830_run_open_wire_phase(
    adbms6830_driver_t *dev,
    adbms6830_open_wire_path_t path,
    OW_C_S owcs)
{
    static uint8_t *const c_read_commands[] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};
    static uint8_t *const s_read_commands[] = {RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF};
    static const GRP groups[] = {A, B, C, D, E, F};
    bool phase_valid[ADBMS6830_MAX_TRACKED_ICS] = {false};
    uint16_t phase_fault_mask[ADBMS6830_MAX_TRACKED_ICS] = {0u};
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    uint8_t adsv_ow_cmd[2];
    bool baseline_phase = (owcs == OW_OFF_ALL_CH);
    bool odd_channels = (owcs == OW_ON_ODD_CH);
    adbms6830_spi_op_t op = baseline_phase ? ADBMS6830_SPI_OP_OPEN_WIRE_BASELINE :
                              (odd_channels ? ADBMS6830_SPI_OP_OPEN_WIRE_ODD :
                                              ADBMS6830_SPI_OP_OPEN_WIRE_EVEN);
    uint16_t threshold_mv;
    uint16_t attenuation_min_permille;
    uint16_t attenuation_max_permille;
    uint8_t group_count;
    uint8_t *const *read_commands;
    bool read_session_owned = false;

    if(!adbms6830_topology_valid(dev) ||
       ((path != ADBMS6830_OPEN_WIRE_PATH_C) &&
        (path != ADBMS6830_OPEN_WIRE_PATH_S)) ||
       (!baseline_phase && (owcs != OW_ON_EVEN_CH) && (owcs != OW_ON_ODD_CH)) ||
       (dev->monitored_cell_count == 0u) ||
       (dev->monitored_cell_count > CELL))
    {
        return HAL_ERROR;
    }

    threshold_mv = ((dev->thresholds.OWC_Threshold > 0) &&
                    (dev->thresholds.OWC_Threshold <= 5500))
                       ? (uint16_t)dev->thresholds.OWC_Threshold
                       : ADBMS6830_OPEN_WIRE_THRESHOLD_MV;
    group_count = (uint8_t)((dev->monitored_cell_count + 2u) / 3u);
    read_commands = (path == ADBMS6830_OPEN_WIRE_PATH_C) ?
                    c_read_commands : s_read_commands;
    attenuation_min_permille =
        (path == ADBMS6830_OPEN_WIRE_PATH_C) ?
        ADBMS6830_C_OPEN_WIRE_MIN_ATTENUATION_PERMILLE :
        ADBMS6830_S_OPEN_WIRE_MIN_ATTENUATION_PERMILLE;
    attenuation_max_permille =
        (path == ADBMS6830_OPEN_WIRE_PATH_C) ?
        ADBMS6830_C_OPEN_WIRE_MAX_ATTENUATION_PERMILLE :
        ADBMS6830_S_OPEN_WIRE_MAX_ATTENUATION_PERMILLE;

    dev->health.open_wire_last_path = path;
    if(baseline_phase)
    {
        adbms6830_increment_u32_sat(&dev->health.open_wire_baseline_count);
    }
    else if(odd_channels)
    {
        adbms6830_increment_u32_sat(&dev->health.open_wire_odd_count);
    }
    else
    {
        adbms6830_increment_u32_sat(&dev->health.open_wire_even_count);
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        phase_valid[ic] = true;
        dev->diag[ic].open_wire_path = path;
        if(baseline_phase)
        {
            dev->diag[ic].open_wire_baseline_valid = false;
            dev->diag[ic].open_wire_even_valid = false;
            dev->diag[ic].open_wire_odd_valid = false;
            dev->diag[ic].open_wire_even_fault_mask = 0u;
            dev->diag[ic].open_wire_odd_fault_mask = 0u;
            dev->diag[ic].open_wire_even_attenuation_fault_mask = 0u;
            dev->diag[ic].open_wire_odd_attenuation_fault_mask = 0u;
            memset(dev->diag[ic].open_wire_baseline_mv,
                   0,
                   sizeof(dev->diag[ic].open_wire_baseline_mv));
            memset(dev->diag[ic].open_wire_even_mv,
                   0,
                   sizeof(dev->diag[ic].open_wire_even_mv));
            memset(dev->diag[ic].open_wire_odd_mv,
                   0,
                   sizeof(dev->diag[ic].open_wire_odd_mv));
        }
        else if(odd_channels)
        {
            dev->diag[ic].open_wire_odd_valid = false;
            dev->diag[ic].open_wire_odd_fault_mask = 0u;
            dev->diag[ic].open_wire_odd_attenuation_fault_mask = 0u;
            memset(dev->diag[ic].open_wire_odd_mv,
                   0,
                   sizeof(dev->diag[ic].open_wire_odd_mv));
        }
        else
        {
            dev->diag[ic].open_wire_even_valid = false;
            dev->diag[ic].open_wire_even_fault_mask = 0u;
            dev->diag[ic].open_wire_even_attenuation_fault_mask = 0u;
            memset(dev->diag[ic].open_wire_even_mv,
                   0,
                   sizeof(dev->diag[ic].open_wire_even_mv));
        }
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = op;
    }

    if(path == ADBMS6830_OPEN_WIRE_PATH_C)
    {
        /* C-path open-wire is legitimate when the redundant S input network is
         * unavailable, but it overwrites the C result registers. The
         * accumulator-level API must perform and validate a normal restoring
         * C conversion before any voltage image is republished. */
        status = adbms6830_adcv_checked(dev,
                                        RD_OFF,
                                        SINGLE,
                                        DCP_OFF,
                                        RSTF_OFF,
                                        owcs);
    }
    else
    {
        /* S-path open-wire preserves the continuously authoritative C image and
         * is preferred after the SMB S2N-S15N hardware has been corrected. */
        adsv_ow_cmd[0] = 0x01u;
        adsv_ow_cmd[1] = ((uint8_t)SINGLE << 7u) |
                         ((uint8_t)DCP_OFF << 4u) |
                         ((uint8_t)owcs & 0x03u) | 0x68u;
        status = adbms6830_wakeup_checked(dev);
        if(status == HAL_OK)
        {
            status = adbms6830_cmd_checked(dev, adsv_ow_cmd);
        }
    }

    if(status == HAL_OK)
    {
        status = adbms6830_wait_cooperative(dev, ADBMS6830_OPEN_WIRE_CONVERSION_WAIT_US);
    }
    if(status != HAL_OK)
    {
        first_error = status;
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            phase_valid[ic] = false;
        }
    }
    else
    {
        /* The millisecond conversion wait deliberately ends any prior serial
         * readiness assumption.  Re-establish chain readiness once, then batch
         * the result groups in one short read session. */
        status = adbms6830_read_session_begin(dev, &read_session_owned);
        if(status != HAL_OK)
        {
            first_error = status;
            for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
            {
                phase_valid[ic] = false;
            }
        }

        for(uint8_t group_index = 0u;
            (status == HAL_OK) && (group_index < group_count);
            group_index++)
        {
            uint8_t first_cell = adbms6830_cell_group_first_index(groups[group_index]);

            status = adbms6830_rd48_checked(dev,
                                             read_commands[group_index],
                                             shared_buf);
            if(status != HAL_OK)
            {
                if(first_error == HAL_OK)
                {
                    first_error = status;
                }
                for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
                {
                    phase_valid[ic] = false;
                }
                continue;
            }

            for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
            {
                const uint8_t *packet = &shared_buf[(uint16_t)ic * RX_DATA];
                bool packet_valid = adbms6830_read_packet_pec_ok(packet) &&
                                    adbms6830_read_packet_counter_ok(dev, ic);

                if(!packet_valid)
                {
                    phase_valid[ic] = false;
                    if(first_error == HAL_OK)
                    {
                        first_error = HAL_ERROR;
                    }
                    continue;
                }

                for(uint8_t slot = 0u; slot < 3u; slot++)
                {
                    uint8_t cell = (uint8_t)(first_cell + slot);
                    int16_t code;
                    uint16_t mv;
                    bool selected_parity;

                    if(cell >= dev->monitored_cell_count)
                    {
                        continue;
                    }

                    code = (int16_t)((uint16_t)packet[slot * 2u] |
                                     ((uint16_t)packet[(slot * 2u) + 1u] << 8u));

                    if(!adbms6830_open_wire_code_to_mv(code, &mv))
                    {
                        phase_valid[ic] = false;
                        phase_fault_mask[ic] |= (uint16_t)(1u << cell);
                        if(first_error == HAL_OK)
                        {
                            first_error = HAL_ERROR;
                        }
                        continue;
                    }

                    if(baseline_phase)
                    {
                        dev->diag[ic].open_wire_baseline_mv[cell] = mv;
                        if(mv < threshold_mv)
                        {
                            phase_valid[ic] = false;
                            if(first_error == HAL_OK)
                            {
                                first_error = HAL_ERROR;
                            }
                        }
                        continue;
                    }

                    if(odd_channels)
                    {
                        dev->ics[ic].owcell.cell_ow_odd[cell] = (int)code;
                        dev->diag[ic].open_wire_odd_mv[cell] = mv;
                    }
                    else
                    {
                        dev->ics[ic].owcell.cell_ow_even[cell] = (int)code;
                        dev->diag[ic].open_wire_even_mv[cell] = mv;
                    }

                    /* ADI channel numbering is one-based: odd channels have
                     * even zero-based indices in the firmware arrays. */
                    selected_parity = odd_channels ? ((cell & 1u) == 0u)
                                                   : ((cell & 1u) != 0u);
                    if(!selected_parity)
                    {
                        continue;
                    }

                    if(!dev->diag[ic].open_wire_baseline_valid ||
                       (dev->diag[ic].open_wire_baseline_mv[cell] == 0u))
                    {
                        phase_valid[ic] = false;
                        phase_fault_mask[ic] |= (uint16_t)(1u << cell);
                        if(first_error == HAL_OK)
                        {
                            first_error = HAL_ERROR;
                        }
                    }
                    else
                    {
                        uint16_t baseline_mv =
                            dev->diag[ic].open_wire_baseline_mv[cell];
                        uint32_t attenuation_permille;
                        bool attenuation_fault = false;

                        if(mv > baseline_mv)
                        {
                            attenuation_fault = true;
                        }
                        else
                        {
                            attenuation_permille =
                                (((uint32_t)(baseline_mv - mv) * 1000u) +
                                 ((uint32_t)baseline_mv / 2u)) /
                                (uint32_t)baseline_mv;
                            attenuation_fault =
                                (attenuation_permille < attenuation_min_permille) ||
                                (attenuation_permille > attenuation_max_permille);
                        }

                        if(mv < threshold_mv)
                        {
                            phase_fault_mask[ic] |= (uint16_t)(1u << cell);
                        }
                        if(attenuation_fault)
                        {
                            phase_fault_mask[ic] |= (uint16_t)(1u << cell);
                            if(odd_channels)
                            {
                                dev->diag[ic].open_wire_odd_attenuation_fault_mask |=
                                    (uint16_t)(1u << cell);
                            }
                            else
                            {
                                dev->diag[ic].open_wire_even_attenuation_fault_mask |=
                                    (uint16_t)(1u << cell);
                            }
                        }
                    }
                }
            }
        }
        adbms6830_read_session_end(dev, read_session_owned);
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        if(baseline_phase)
        {
            dev->diag[ic].open_wire_baseline_valid = phase_valid[ic];
        }
        else if(odd_channels)
        {
            dev->diag[ic].open_wire_odd_valid = phase_valid[ic];
            dev->diag[ic].open_wire_odd_fault_mask = phase_fault_mask[ic];
        }
        else
        {
            dev->diag[ic].open_wire_even_valid = phase_valid[ic];
            dev->diag[ic].open_wire_even_fault_mask = phase_fault_mask[ic];
        }

        if((!phase_valid[ic] || (phase_fault_mask[ic] != 0u)) &&
           (first_error == HAL_OK))
        {
            first_error = HAL_ERROR;
        }
    }

    adbms6830_open_wire_refresh_health(dev);
    adbms6830_diag_note_status(dev, op, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = op;
        dev->spi_debug.last_status = first_error;
    }
    return first_error;
}

HAL_StatusTypeDef adbms6830_run_open_wire_check_path(
    adbms6830_driver_t *dev,
    adbms6830_open_wire_path_t path,
    bool odd_channels)
{
    HAL_StatusTypeDef status;

    status = adbms6830_run_open_wire_phase(dev, path, OW_OFF_ALL_CH);
    if(status != HAL_OK)
    {
        return status;
    }

    return adbms6830_run_open_wire_phase(dev,
                                         path,
                                         odd_channels ? OW_ON_ODD_CH :
                                                        OW_ON_EVEN_CH);
}

HAL_StatusTypeDef adbms6830_run_open_wire_diagnostic_path(
    adbms6830_driver_t *dev,
    adbms6830_open_wire_path_t path)
{
    HAL_StatusTypeDef baseline_status;
    HAL_StatusTypeDef even_status;
    HAL_StatusTypeDef odd_status;
    HAL_StatusTypeDef result;

    if(!adbms6830_topology_valid(dev) ||
       ((path != ADBMS6830_OPEN_WIRE_PATH_C) &&
        (path != ADBMS6830_OPEN_WIRE_PATH_S)))
    {
        return HAL_ERROR;
    }

    adbms6830_increment_u32_sat(&dev->health.open_wire_full_count);
    if(path == ADBMS6830_OPEN_WIRE_PATH_C)
    {
        adbms6830_increment_u32_sat(&dev->health.open_wire_c_full_count);
    }
    else
    {
        adbms6830_increment_u32_sat(&dev->health.open_wire_s_full_count);
    }

    baseline_status = adbms6830_run_open_wire_phase(dev,
                                                     path,
                                                     OW_OFF_ALL_CH);
    even_status = (baseline_status == HAL_OK) ?
                  adbms6830_run_open_wire_phase(dev,
                                                path,
                                                OW_ON_EVEN_CH) :
                  baseline_status;
    odd_status = ((baseline_status == HAL_OK) && (even_status == HAL_OK)) ?
                 adbms6830_run_open_wire_phase(dev,
                                               path,
                                               OW_ON_ODD_CH) :
                 ((baseline_status != HAL_OK) ? baseline_status : even_status);
    adbms6830_open_wire_refresh_health(dev);

    result = ((baseline_status == HAL_OK) &&
              (even_status == HAL_OK) &&
              (odd_status == HAL_OK) &&
              (dev->health.open_wire_incomplete_ic_mask == 0u) &&
              (dev->health.open_wire_fault_ic_mask == 0u)) ? HAL_OK : HAL_ERROR;
    if(baseline_status != HAL_OK)
    {
        result = baseline_status;
    }
    else if(even_status != HAL_OK)
    {
        result = even_status;
    }
    else if(odd_status != HAL_OK)
    {
        result = odd_status;
    }
    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_OPEN_WIRE_FULL, result);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_OPEN_WIRE_FULL;
        dev->spi_debug.last_status = result;
    }
    return result;
}

HAL_StatusTypeDef adbms6830_run_open_wire_check(adbms6830_driver_t *dev,
                                                 bool odd_channels)
{
    return adbms6830_run_open_wire_check_path(dev,
                                               ADBMS6830_OPEN_WIRE_PATH_S,
                                               odd_channels);
}

HAL_StatusTypeDef adbms6830_run_open_wire_diagnostic(adbms6830_driver_t *dev)
{
    return adbms6830_run_open_wire_diagnostic_path(
        dev,
        ADBMS6830_OPEN_WIRE_PATH_S);
}

HAL_StatusTypeDef adbms6830_run_aux_gpio_diagnostic(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    uint8_t adax_cmd[2] = { ADAX_CMD_BYTE0, ADAX_CMD_BYTE1 };
    bool read_session_owned = false;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    adbms6830_increment_u32_sat(&dev->health.aux_gpio_diag_count);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_AUX_GPIO_DIAG;
    }

    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adax_cmd);
    }
    if(status == HAL_OK)
    {
        /* AUX_ALL is an 18-channel sequential conversion.  Reading RDAUXA
         * before the full sequence finishes can republish an older sample. */
        status = adbms6830_wait_cooperative(dev, ADBMS6830_AUX_CONVERSION_WAIT_US);
    }
    if(status != HAL_OK)
    {
        first_error = status;
    }

    if(first_error == HAL_OK)
    {
        /* The 20 ms AUX conversion wait intentionally allows isoSPI to idle.
         * Wake once afterward and batch RDAUXA plus Status C/D/E in the same
         * short read session. */
        status = adbms6830_read_session_begin(dev, &read_session_owned);
        if(status != HAL_OK)
        {
            first_error = status;
        }
        else
        {
            status = adbms6830_rd48_checked(dev, RDAUXA, shared_buf);
            if(status == HAL_OK)
            {
                adbms6830_parse_aux_gpio(dev, shared_buf);
                if(!adbms6830_last_read_integrity_ok(dev))
                {
                    first_error = HAL_ERROR;
                }
            }
            else
            {
                first_error = status;
            }

            status = adbms6830_read_status(dev, false);
            if((status != HAL_OK) && (first_error == HAL_OK))
            {
                first_error = status;
            }
        }
        adbms6830_read_session_end(dev, read_session_owned);
    }
    else
    {
        /* Preserve the legacy diagnostic refresh after a failed conversion. */
        status = adbms6830_read_status(dev, false);
        if((status != HAL_OK) && (first_error == HAL_OK))
        {
            first_error = status;
        }
    }

    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_AUX_GPIO_DIAG, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_AUX_GPIO_DIAG;
        dev->spi_debug.last_status = first_error;
    }
    return first_error;
}

HAL_StatusTypeDef adBms6830_init(adbms6830_driver_t* dev,
						   uint8_t num_ics,
						   uint8_t physical_chain_count,
						   adbms6830_asic* ics,
					   uint8_t ics_capacity,
					   SPI_HandleTypeDef* hspi,
					   GPIO_TypeDef* cs_port_a,
					   GPIO_TypeDef* cs_port_b,
					   uint16_t cs_pin_a,
					   uint16_t cs_pin_b,
                           adbms_string selected_string,
					   TIM_HandleTypeDef *htim)
{
	HAL_StatusTypeDef status;

	if(dev == NULL)
	{
		return HAL_ERROR;
	}

	memset(dev, 0, sizeof(*dev));
	dev->ics = ics;
	dev->ics_capacity = ics_capacity;
	dev->hspi = hspi;
	dev->cs_port[0] = cs_port_a;
	dev->cs_port[1] = cs_port_b;
	dev->cs_pin[0] = cs_pin_a;
	dev->cs_pin[1] = cs_pin_b;
	dev->htim = htim;
	dev->delay_last_status = (htim != NULL) ? HAL_OK : HAL_ERROR;
	dev->spi_debug.enabled = true;
	dev->spi_debug.last_status = HAL_ERROR;
	dev->spi_debug.last_tx_status = HAL_ERROR;
	dev->spi_debug.last_rx_status = HAL_ERROR;
	dev->spi_debug.last_xfer_status = HAL_ERROR;
	dev->health.last_status = HAL_ERROR;

	if((num_ics == 0u) ||
	   (num_ics > ADBMS6830_MAX_TRACKED_ICS) ||
	   (physical_chain_count < num_ics) ||
	   (physical_chain_count > ADBMS6830_MAX_PHYSICAL_DEVICES) ||
	   (ics == NULL) ||
	   (ics_capacity < num_ics) ||
	   (hspi == NULL) ||
	   (cs_port_a == NULL) ||
	   (cs_port_b == NULL) ||
	   (cs_pin_a == 0u) ||
	   (cs_pin_b == 0u) ||
       ((selected_string != STRING_A) && (selected_string != STRING_B)))
	{
		return HAL_ERROR;
	}

	memset(ics, 0, sizeof(*ics) * num_ics);
	dev->num_ics = num_ics;
	dev->physical_chain_count = physical_chain_count;

	memset(&dev->spi_debug, 0, sizeof(dev->spi_debug));
	dev->spi_debug.enabled = true;
	dev->spi_debug.last_status = HAL_OK;
	dev->spi_debug.last_tx_status = HAL_OK;
	dev->spi_debug.last_rx_status = HAL_OK;
	dev->spi_debug.last_xfer_status = HAL_OK;
	memset(&dev->health, 0, sizeof(dev->health));
	dev->health.last_status = HAL_OK;
	adbms6830_note_counter_reset(dev);

	for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
	{
		dev->last_cell_updated_mask[ic] = 0u;
		dev->last_cell_pec_mask[ic] = 0u;
		dev->last_scell_updated_mask[ic] = 0u;
		dev->last_scell_pec_mask[ic] = 0u;
		dev->last_temp_updated_mask[ic] = 0u;
		for(uint8_t mux = 0u; mux < ADBMS6830_MUX_COUNT; mux++)
		{
			dev->mux_selected_channel[ic][mux] = UINT8_MAX;
		}
	}
	memset(dev->mux_selection_valid_mask, 0, sizeof(dev->mux_selection_valid_mask));
	memset(dev->diag, 0, sizeof(dev->diag));

	/* Deassert both host chip-selects before selecting the commanded access
	 * direction.  The one-SMB EVAL path enters from String B, while the final
	 * five-SMB + ADBMS2950 ring addresses the SMB subset from String A and the
	 * APM from String B.  Do not bury that board-topology choice in the driver. */
	dev->string = STRING_B;
	adbms6830_set_cs(dev, 1);
	dev->string = STRING_A;
	adbms6830_set_cs(dev, 1);
	dev->string = selected_string;
	dev->write_string = selected_string;

	status = adbms6830_wakeup_checked(dev);
	if(status == HAL_OK)
	{
		status = adbms6830_cmd_checked(dev, SRST);
	}
	if(status != HAL_OK)
	{
		adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_CMD, status);
		return status;
	}
	status = adbms6830_us_delay(dev, 300u);
	if(status != HAL_OK)
	{
		adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_CMD, status);
		return status;
	}

	/* Identify the complete leading String-A subset before writing any
	 * configuration.  The ADBMS2950 also returns a valid RDSID packet, so PEC
	 * alone cannot detect a swapped physical order. */
	status = adbms6830_read_sid(dev);
	if(status != HAL_OK)
	{
		adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_READ_SID, status);
		return status;
	}

	adbms6830_reset_cfg(dev);

	status = adbms6830_wrcfga_checked(dev);
	if(status != HAL_OK)
	{
		adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_WR48, status);
		return status;
	}
    status = adbms6830_wrcfgb_checked_reason(
        dev, ADBMS6830_CFGB_WRITE_INITIALIZATION);
	if(status != HAL_OK)
	{
		adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_WR48, status);
		return status;
	}

	return HAL_OK;
}

void adbms6830_reset_cfg(adbms6830_driver_t *dev)
{
	uint16_t vov_value;
	uint16_t vuv_value;
	uint8_t rbits = 12;

	if(!adbms6830_topology_valid(dev))
	{
		return;
	}

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

	for(uint8_t i = 0u; i < (uint8_t)dev->num_ics; i++)
	{
		float over_voltage = dev->thresholds.OV_THRESHOLD;
		float under_voltage = dev->thresholds.UV_THRESHOLD;
		/* Setup cell_asic */
		/* Init config A */
		dev->ics[i].tx_cfga.refon = PWR_UP;
		dev->ics[i].tx_cfga.cth   = 1u;      /* 001: 8.1 mV datasheet default */
        dev->ics[i].tx_cfga.flag_d = 0u;
        dev->ics[i].tx_cfga.fc = (uint8_t)(AMS_ADBMS_IIR_FC & 0x07u);
		dev->ics[i].tx_cfga.gpo   = 0x3FFu;

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
		dev->ics[i].tx_cfgb.dtmen = 0u;
		dev->ics[i].tx_cfgb.dtrng = 0u;
		dev->ics[i].tx_cfgb.dcto  = 0u;
		dev->ics[i].tx_cfgb.dcc   = 0u;
		memset(dev->ics[i].PwmA.pwma, 0, sizeof(dev->ics[i].PwmA.pwma));
		memset(dev->ics[i].PwmB.pwmb, 0, sizeof(dev->ics[i].PwmB.pwmb));
		memset(dev->ics[i].pwma.tx_data, 0, sizeof(dev->ics[i].pwma.tx_data));
		memset(dev->ics[i].pwmb.tx_data, 0, sizeof(dev->ics[i].pwmb.tx_data));
	}
    dev->filtered_voltage_ready = false;
    dev->filtered_successful_epoch_count = 0u;
    dev->filter_ready_after_ms = 60u; /* 21 Hz: ~52 ms to 0.1%; keep explicit margin. */
}

void adbms6830_wrcfga(adbms6830_driver_t *dev)
{
	(void)adbms6830_wrcfga_checked(dev);
}

HAL_StatusTypeDef adbms6830_wrcfga_checked(adbms6830_driver_t *dev)
{
	if(!adbms6830_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	adbms6830_asic *ic = dev->ics;
	adbms6830_pack_cfga(dev);
    for (uint8_t cic = 0; cic < dev->num_ics; cic++)
    {
      for (uint8_t data = 0; data < TX_DATA; data++)
      {
        shared_buf[(cic * TX_DATA) + data] = ic[cic].configa.tx_data[data];
      }
    }
	return adbms6830_wr48_checked(dev, WRCFGA, shared_buf);
}

const char *adbms6830_cfgb_write_reason_str(adbms6830_cfgb_write_reason_t reason)
{
    switch(reason)
    {
    case ADBMS6830_CFGB_WRITE_UNSPECIFIED: return "unspecified";
    case ADBMS6830_CFGB_WRITE_INITIALIZATION: return "initialization";
    case ADBMS6830_CFGB_WRITE_BALANCE_APPLY: return "balance_apply";
    case ADBMS6830_CFGB_WRITE_BALANCE_CLEAR: return "balance_clear";
    case ADBMS6830_CFGB_WRITE_BALANCE_RECOVERY: return "balance_recovery";
    case ADBMS6830_CFGB_WRITE_CONFIG_STRESS: return "config_stress";
    case ADBMS6830_CFGB_WRITE_DISCHARGE_TIMER_CONFIG: return "timer_config";
    default: return "unknown";
    }
}

static bool adbms6830_cfgb_timer_write_allowed(
    adbms6830_cfgb_write_reason_t reason)
{
#if AMS_ENABLE_ADBMS_DISCHARGE_TIMER
    return reason == ADBMS6830_CFGB_WRITE_DISCHARGE_TIMER_CONFIG;
#else
    (void)reason;
    return false;
#endif
}

static bool adbms6830_cfgb_balance_shadow_active(const adbms6830_asic *ic)
{
    if(ic == NULL)
    {
        return false;
    }

    if(ic->tx_cfgb.dcc != 0u)
    {
        return true;
    }
    for(uint8_t cell = 0u; cell < PWMA; cell++)
    {
        if(ic->PwmA.pwma[cell] != 0u)
        {
            return true;
        }
    }
    for(uint8_t cell = 0u; cell < PWMB; cell++)
    {
        if(ic->PwmB.pwmb[cell] != 0u)
        {
            return true;
        }
    }
    return false;
}

static adbms6830_cfgb_write_event_t *adbms6830_cfgb_history_begin(
    adbms6830_driver_t *dev,
    adbms6830_cfgb_write_reason_t reason,
    const uint8_t *payload,
    uint16_t timer_nonzero_mask,
    uint16_t balance_shadow_mask,
    uint16_t rejected_mask)
{
    adbms6830_cfgb_write_event_t *event;
    uint8_t index;
    uint8_t ic_count;

    if((dev == NULL) || (payload == NULL))
    {
        return NULL;
    }

    index = dev->cfgb_write_history_index;
    if(index >= ADBMS6830_CFGB_WRITE_HISTORY_DEPTH)
    {
        index = 0u;
    }
    event = &dev->cfgb_write_history[index];
    memset(event, 0, sizeof(*event));

    adbms6830_increment_u32_sat(&dev->cfgb_write_total_count);
    event->sequence = dev->cfgb_write_total_count;
#if AMS_HOST_TEST
    event->tick_ms = dev->cfgb_write_total_count;
#else
    event->tick_ms = HAL_GetTick();
#endif
    event->reason = reason;
    /* HAL_BUSY means "attempt recorded, transfer result not committed yet".
     * The result is updated before the owner lock is released. */
    event->status = HAL_BUSY;
    event->string = dev->string;
    event->timer_nonzero_mask = timer_nonzero_mask;
    event->balance_shadow_mask = balance_shadow_mask;
    event->rejected_mask = rejected_mask;

    ic_count = (dev->num_ics > (int)ADBMS6830_MAX_TRACKED_ICS) ?
               ADBMS6830_MAX_TRACKED_ICS : (uint8_t)dev->num_ics;
    event->ic_count = ic_count;
    for(uint8_t ic = 0u; ic < ic_count; ic++)
    {
        memcpy(event->payload[ic], &payload[(uint16_t)ic * TX_DATA], TX_DATA);
    }

    dev->cfgb_write_history_index =
        (uint8_t)((index + 1u) % ADBMS6830_CFGB_WRITE_HISTORY_DEPTH);
    if(dev->cfgb_write_history_count < ADBMS6830_CFGB_WRITE_HISTORY_DEPTH)
    {
        dev->cfgb_write_history_count++;
    }
    return event;
}

void adbms6830_disable_discharge_timer_shadow(adbms6830_driver_t *dev)
{
    if(!adbms6830_topology_valid(dev))
    {
        return;
    }

    adbms_spi_lock();
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        dev->ics[ic].tx_cfgb.dtmen = 0u;
        dev->ics[ic].tx_cfgb.dtrng = 0u;
        dev->ics[ic].tx_cfgb.dcto = 0u;
    }
    adbms6830_pack_cfgb(dev);
    adbms_spi_unlock();
}

void adbms6830_wrcfgb(adbms6830_driver_t *dev)
{
	(void)adbms6830_wrcfgb_checked(dev);
}

HAL_StatusTypeDef adbms6830_wrcfgb_checked(adbms6830_driver_t *dev)
{
	return adbms6830_wrcfgb_checked_reason(
		dev, ADBMS6830_CFGB_WRITE_UNSPECIFIED);
}

HAL_StatusTypeDef adbms6830_wrcfgb_checked_reason(
    adbms6830_driver_t *dev,
    adbms6830_cfgb_write_reason_t reason)
{
    uint8_t payload[ADBMS6830_MAX_TRACKED_ICS * TX_DATA] = {0u};
    uint16_t timer_nonzero_mask = 0u;
    uint16_t balance_shadow_mask = 0u;
    uint16_t rejected_mask = 0u;
    adbms6830_cfgb_write_event_t *history_event;
    HAL_StatusTypeDef status;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    /* The owner mutex is recursive.  Taking it here guarantees that the exact
     * payload validated below is the payload clocked onto the bus, even when a
     * caller already owns the larger logical transaction. */
    adbms_spi_lock();

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        const cfb6830_ *cfg = &dev->ics[ic].tx_cfgb;
        uint8_t *dst = &payload[(uint16_t)ic * TX_DATA];
        uint16_t bit = (uint16_t)(1u << ic);

        dst[0] = (uint8_t)cfg->vuv;
        dst[1] = (uint8_t)(((cfg->vov & 0x000Fu) << 4u) |
                           (cfg->vuv >> 8u));
        dst[2] = (uint8_t)((cfg->vov >> 4u) & 0x00FFu);
        dst[3] = (uint8_t)(((cfg->dtmen & 0x01u) << 7u) |
                           ((cfg->dtrng & 0x01u) << 6u) |
                           (cfg->dcto & 0x3Fu));
        dst[4] = (uint8_t)(cfg->dcc & 0x00FFu);
        dst[5] = (uint8_t)(cfg->dcc >> 8u);
        memcpy(dev->ics[ic].configb.tx_data, dst, TX_DATA);

        if(dst[3] != 0u)
        {
            timer_nonzero_mask |= bit;
        }
        if(adbms6830_cfgb_balance_shadow_active(&dev->ics[ic]))
        {
            balance_shadow_mask |= bit;
        }
    }

    if((timer_nonzero_mask != 0u) &&
       !adbms6830_cfgb_timer_write_allowed(reason))
    {
        rejected_mask = timer_nonzero_mask;
        dev->health.config_write_guard_fault_mask |= rejected_mask;
        dev->health.sticky_config_write_guard_fault_mask |= rejected_mask;
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            uint16_t bit = (uint16_t)(1u << ic);
            if((rejected_mask & bit) != 0u)
            {
                adbms6830_increment_u32_sat(
                    &dev->health.config_write_guard_reject_count[ic]);
                /* Preserve the rejected bytes in history, then restore the
                 * software shadow to the reviewed timer-disabled policy. */
                dev->ics[ic].tx_cfgb.dtmen = 0u;
                dev->ics[ic].tx_cfgb.dtrng = 0u;
                dev->ics[ic].tx_cfgb.dcto = 0u;
            }
        }
        adbms6830_pack_cfgb(dev);
        status = HAL_ERROR;
        history_event = adbms6830_cfgb_history_begin(
            dev, reason, payload, timer_nonzero_mask,
            balance_shadow_mask, rejected_mask);
        if(history_event != NULL)
        {
            history_event->status = status;
        }
        adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_WR48, status);
        if(dev->spi_debug.enabled)
        {
            dev->spi_debug.last_op = ADBMS6830_SPI_OP_WR48;
            dev->spi_debug.last_status = status;
            adbms6830_increment_u32_sat(&dev->spi_debug.error_count);
        }
        adbms_spi_unlock();
        return status;
    }

    /* Commit the exact payload to history before the physical transfer. */
    history_event = adbms6830_cfgb_history_begin(
        dev, reason, payload, timer_nonzero_mask,
        balance_shadow_mask, rejected_mask);
    status = adbms6830_wr48_checked(dev, WRCFGB, payload);
    if(history_event != NULL)
    {
        history_event->status = status;
    }
    adbms_spi_unlock();
    return status;
}

static void adbms6830_pack_pwma(adbms6830_driver_t *dev)
{
	adbms6830_asic *ics = dev->ics;
	for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	{
		memset(ics[curr_ic].pwma.tx_data, 0, TX_DATA);
		ics[curr_ic].pwma.tx_data[0] = (uint8_t)(((ics[curr_ic].PwmA.pwma[1] & 0x0Fu) << 4) |
		                                         (ics[curr_ic].PwmA.pwma[0] & 0x0Fu));
		ics[curr_ic].pwma.tx_data[1] = (uint8_t)(((ics[curr_ic].PwmA.pwma[3] & 0x0Fu) << 4) |
		                                         (ics[curr_ic].PwmA.pwma[2] & 0x0Fu));
		ics[curr_ic].pwma.tx_data[2] = (uint8_t)(((ics[curr_ic].PwmA.pwma[5] & 0x0Fu) << 4) |
		                                         (ics[curr_ic].PwmA.pwma[4] & 0x0Fu));
		ics[curr_ic].pwma.tx_data[3] = (uint8_t)(((ics[curr_ic].PwmA.pwma[7] & 0x0Fu) << 4) |
		                                         (ics[curr_ic].PwmA.pwma[6] & 0x0Fu));
		ics[curr_ic].pwma.tx_data[4] = (uint8_t)(((ics[curr_ic].PwmA.pwma[9] & 0x0Fu) << 4) |
		                                         (ics[curr_ic].PwmA.pwma[8] & 0x0Fu));
		ics[curr_ic].pwma.tx_data[5] = (uint8_t)(((ics[curr_ic].PwmA.pwma[11] & 0x0Fu) << 4) |
		                                         (ics[curr_ic].PwmA.pwma[10] & 0x0Fu));
	}
}

static void adbms6830_pack_pwmb(adbms6830_driver_t *dev)
{
	adbms6830_asic *ics = dev->ics;
	for(uint8_t curr_ic = 0; curr_ic < dev->num_ics; curr_ic++)
	{
		memset(ics[curr_ic].pwmb.tx_data, 0, TX_DATA);
		ics[curr_ic].pwmb.tx_data[0] = (uint8_t)(((ics[curr_ic].PwmB.pwmb[1] & 0x0Fu) << 4) |
		                                         (ics[curr_ic].PwmB.pwmb[0] & 0x0Fu));
		ics[curr_ic].pwmb.tx_data[1] = (uint8_t)(((ics[curr_ic].PwmB.pwmb[3] & 0x0Fu) << 4) |
		                                         (ics[curr_ic].PwmB.pwmb[2] & 0x0Fu));
	}
}

HAL_StatusTypeDef adbms6830_wrpwma_checked(adbms6830_driver_t *dev)
{
	if(!adbms6830_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	adbms6830_pack_pwma(dev);
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		for(uint8_t data = 0; data < TX_DATA; data++)
		{
			shared_buf[(cic * TX_DATA) + data] = dev->ics[cic].pwma.tx_data[data];
		}
	}
	return adbms6830_wr48_checked(dev, WRPWM1, shared_buf);
}

HAL_StatusTypeDef adbms6830_wrpwmb_checked(adbms6830_driver_t *dev)
{
	if(!adbms6830_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	adbms6830_pack_pwmb(dev);
	for(uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		for(uint8_t data = 0; data < TX_DATA; data++)
		{
			shared_buf[(cic * TX_DATA) + data] = dev->ics[cic].pwmb.tx_data[data];
		}
	}
	return adbms6830_wr48_checked(dev, WRPWM2, shared_buf);
}

HAL_StatusTypeDef adbms6830_write_pwm_checked(adbms6830_driver_t *dev)
{
	HAL_StatusTypeDef status = adbms6830_wrpwma_checked(dev);
	if(status != HAL_OK)
	{
		return status;
	}
	return adbms6830_wrpwmb_checked(dev);
}

void adbms6830_rdcfga(adbms6830_driver_t *dev)
{
	if((adbms6830_rd48_checked(dev, RDCFGA, shared_buf) == HAL_OK) &&
	   adbms6830_last_read_integrity_ok(dev))
	{
		adbms6830_parse_cfga(dev, shared_buf);
	}
}

void adbms6830_rdcfgb(adbms6830_driver_t *dev)
{
	if((adbms6830_rd48_checked(dev, RDCFGB, shared_buf) == HAL_OK) &&
	   adbms6830_last_read_integrity_ok(dev))
	{
		adbms6830_parse_cfgb(dev, shared_buf);
	}
}

void adbms6830_pack_cfga(adbms6830_driver_t *dev)
{
	if(!adbms6830_topology_valid(dev))
	{
		return;
	}

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
	if(!adbms6830_topology_valid(dev))
	{
		return;
	}

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
	if(!adbms6830_topology_valid(dev) || (data == NULL))
	{
		return;
	}

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
	if(!adbms6830_topology_valid(dev) || (data == NULL))
	{
		return;
	}

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
	if(dev != NULL)
	{
		dev->continuous_c_running = false;
	}
	adbms6830_cmd(dev, SRST);
}

static HAL_StatusTypeDef adbms6830_verify_mute_state(adbms6830_driver_t *dev,
                                                      bool expected_muted)
{
    HAL_StatusTypeDef status;
    uint16_t expected_mask;
    uint16_t matched_mask = 0u;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    expected_mask = adbms6830_expected_ic_mask(dev);
    status = adbms6830_rd48_checked(dev, RDCFGA, shared_buf);
    if(status != HAL_OK)
    {
        return status;
    }
    if(!adbms6830_last_read_integrity_ok(dev))
    {
        return HAL_ERROR;
    }
    adbms6830_parse_cfga(dev, shared_buf);
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        bool actual = (dev->ics[ic].rx_cfga.mute_st != 0u);
        if(actual == expected_muted)
        {
            matched_mask |= (uint16_t)(1u << ic);
        }
    }
    return (matched_mask == expected_mask) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef adbms6830_mute_checked(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef status;
    bool session_owned = false;
    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }
    status = adbms6830_read_session_begin(dev, &session_owned);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, MUTE);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_verify_mute_state(dev, true);
        if(status != HAL_OK)
        {
            adbms6830_increment_u32_sat(&dev->health.mute_verify_fail_count);
        }
    }
    if(status == HAL_OK)
    {
        adbms6830_increment_u32_sat(&dev->health.mute_count);
    }
    else
    {
        adbms6830_increment_u32_sat(&dev->health.mute_fail_count);
    }
    adbms6830_read_session_end(dev, session_owned);
    return status;
}

HAL_StatusTypeDef adbms6830_unmute_checked(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef status;
    bool session_owned = false;
    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }
    status = adbms6830_read_session_begin(dev, &session_owned);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, UNMUTE);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_verify_mute_state(dev, false);
        if(status != HAL_OK)
        {
            adbms6830_increment_u32_sat(&dev->health.unmute_verify_fail_count);
        }
    }
    if(status == HAL_OK)
    {
        adbms6830_increment_u32_sat(&dev->health.unmute_count);
    }
    else
    {
        adbms6830_increment_u32_sat(&dev->health.unmute_fail_count);
    }
    adbms6830_read_session_end(dev, session_owned);
    return status;
}

static bool adbms6830_post_stage_matches(const adbms6830_driver_t *dev,
                                         adbms6830_post_stage_t stage,
                                         uint16_t *failed_mask,
                                         uint16_t *unexpected_mask)
{
    uint16_t failed = 0u;
    uint16_t unexpected = 0u;

    if(!adbms6830_topology_valid(dev))
    {
        return false;
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        const adbms6830_ic_diag_t *d = &dev->diag[ic];
        uint16_t bit = (uint16_t)(1u << ic);
        bool hit = false;
        bool extra = false;

        if(!d->statc_valid)
        {
            failed |= bit;
            continue;
        }
        switch(stage)
        {
        case ADBMS6830_POST_OSC_FAST:
        case ADBMS6830_POST_OSC_SLOW:
            hit = d->oscchk != 0u;
            extra = (d->thsd || d->tmodchk || d->cmed || d->smed);
            break;
        case ADBMS6830_POST_SUPPLY_UV:
            hit = (d->va_uv != 0u) || (d->vd_uv != 0u);
            extra = (d->thsd || d->tmodchk || d->cmed || d->smed);
            break;
        case ADBMS6830_POST_SUPPLY_OV:
            hit = (d->va_ov != 0u) || (d->vd_ov != 0u) ||
                  (d->vde != 0u) || (d->vdel != 0u);
            extra = (d->thsd || d->tmodchk || d->cmed || d->smed);
            break;
        case ADBMS6830_POST_THSD_FLAG:
            hit = d->thsd != 0u;
            extra = d->tmodchk || d->cmed || d->smed;
            break;
        case ADBMS6830_POST_NVM_ED:
            hit = (d->ced != 0u) && (d->sed != 0u);
            extra = d->cmed || d->smed || d->tmodchk;
            break;
        case ADBMS6830_POST_NVM_MED:
            hit = (d->cmed != 0u) && (d->smed != 0u);
            extra = d->tmodchk;
            break;
        case ADBMS6830_POST_TMOD:
            hit = d->tmodchk != 0u;
            break;
        case ADBMS6830_POST_SPIFLT:
            hit = d->spiflt != 0u;
            extra = d->thsd || d->tmodchk || d->cmed || d->smed;
            break;
        default:
            hit = true;
            break;
        }
        if(!hit) failed |= bit;
        if(extra) unexpected |= bit;
    }
    if(failed_mask != NULL) *failed_mask = failed;
    if(unexpected_mask != NULL) *unexpected_mask = unexpected;
    return (failed == 0u) && (unexpected == 0u);
}

static HAL_StatusTypeDef adbms6830_run_startup_post_once(
    adbms6830_driver_t *dev,
    const cfa6830_ production_cfga[ADBMS6830_MAX_TRACKED_ICS])
{
    static const struct
    {
        adbms6830_post_stage_t stage;
        uint8_t flag_d;
    } tests[] =
    {
        {ADBMS6830_POST_OSC_FAST,  0x01u},
        {ADBMS6830_POST_OSC_SLOW,  0x02u},
        {ADBMS6830_POST_SUPPLY_UV, 0x04u},
        {ADBMS6830_POST_SUPPLY_OV, 0x0Cu},
        {ADBMS6830_POST_THSD_FLAG, 0x10u},
        {ADBMS6830_POST_NVM_ED,    0x20u},
        {ADBMS6830_POST_NVM_MED,   0x40u},
        {ADBMS6830_POST_TMOD,      0x80u}
    };
    HAL_StatusTypeDef status = HAL_OK;
    uint16_t failed = 0u;
    uint16_t unexpected = 0u;

    dev->post.stage = ADBMS6830_POST_BASELINE;
    dev->post.last_status = HAL_ERROR;
    dev->post.failed_ic_mask = 0u;
    dev->post.unexpected_ic_mask = 0u;
    dev->post.expected_flag_d = 0u;

    /* Start every attempt from the exact production CFGA image, not merely
     * FLAG_D=0. This prevents a failed attempt from carrying modified test
     * bits or filter settings into the retry. */
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        dev->ics[ic].tx_cfga = production_cfga[ic];
    }
    status = adbms6830_wrcfga_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_clear_all_flags(dev);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_establish_diagnostic_baseline(dev);
    }

    for(uint8_t i = 0u;
        (status == HAL_OK) && (i < (uint8_t)(sizeof(tests) / sizeof(tests[0])));
        i++)
    {
        dev->post.stage = tests[i].stage;
        dev->post.expected_flag_d = tests[i].flag_d;
        failed = 0u;
        unexpected = 0u;
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            dev->ics[ic].tx_cfga = production_cfga[ic];
            dev->ics[ic].tx_cfga.flag_d = tests[i].flag_d;
        }
        status = adbms6830_wrcfga_checked(dev);
        if(status == HAL_OK)
        {
            status = adbms6830_read_status(dev, false);
        }
        if((status != HAL_OK) ||
           !adbms6830_post_stage_matches(dev, tests[i].stage,
                                         &failed, &unexpected))
        {
            dev->post.failed_ic_mask |= failed;
            dev->post.unexpected_ic_mask |= unexpected;
            status = HAL_ERROR;
            break;
        }

        /* Clear each deliberately injected status before moving to the next
         * stimulus. FLAG_D proves the report/clear path only; it does not
         * emulate the actual physical diagnostic mechanism. */
        status = adbms6830_clear_all_flags(dev);
    }

    /* SPIFLT has a dedicated read-path injection rather than a FLAG_D bit.
     * RDSTATC with ERR=1 must report SPIFLT, proving the redundant SPI-slave
     * comparison/reporting path can assert. As with FLAG_D, this validates the
     * reporting diagnostic, not a physical external-bus corruption event. */
    if(status == HAL_OK)
    {
        dev->post.stage = ADBMS6830_POST_SPIFLT;
        dev->post.expected_flag_d = 0u;
        failed = 0u;
        unexpected = 0u;
        status = adbms6830_read_status(dev, true);
        if((status != HAL_OK) ||
           !adbms6830_post_stage_matches(dev, ADBMS6830_POST_SPIFLT,
                                         &failed, &unexpected))
        {
            dev->post.failed_ic_mask |= failed;
            dev->post.unexpected_ic_mask |= unexpected;
            status = HAL_ERROR;
        }
        if(status == HAL_OK)
        {
            status = adbms6830_clear_all_flags(dev);
        }
    }

    /* Restoration is mandatory even when an injected stage failed. */
    dev->post.stage = ADBMS6830_POST_RESTORE_CONFIG;
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        dev->ics[ic].tx_cfga = production_cfga[ic];
    }
    {
        HAL_StatusTypeDef restore_status = adbms6830_wrcfga_checked(dev);
        if(restore_status == HAL_OK)
        {
            restore_status = adbms6830_clear_all_flags(dev);
        }
        if((status == HAL_OK) && (restore_status != HAL_OK))
        {
            status = restore_status;
        }
    }

    dev->post.stage = ADBMS6830_POST_FINAL_BASELINE;
    if(status == HAL_OK)
    {
        status = adbms6830_establish_diagnostic_baseline(dev);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_verify_config_readback(dev);
    }
    return status;
}

HAL_StatusTypeDef adbms6830_run_startup_post(adbms6830_driver_t *dev)
{
#if AMS_ENABLE_ADBMS_STARTUP_POST
    HAL_StatusTypeDef status = HAL_ERROR;
    cfa6830_ production_cfga[ADBMS6830_MAX_TRACKED_ICS];
    uint32_t prior_run_count;
    uint32_t prior_fail_count;
    uint16_t prior_sticky_status;
    uint16_t prior_sticky_reference;
    uint16_t prior_sticky_open_wire;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    prior_run_count = dev->post.run_count;
    prior_fail_count = dev->post.fail_count;
    prior_sticky_status = dev->health.sticky_status_fault_ic_mask;
    prior_sticky_reference = dev->health.sticky_reference_fault_ic_mask;
    prior_sticky_open_wire = dev->health.sticky_open_wire_fault_ic_mask;
    memset(production_cfga, 0, sizeof(production_cfga));
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        production_cfga[ic] = dev->ics[ic].tx_cfga;
        production_cfga[ic].flag_d = 0u;
    }

    memset(&dev->post, 0, sizeof(dev->post));
    dev->post.run_count = prior_run_count;
    dev->post.fail_count = prior_fail_count;
    adbms6830_increment_u32_sat(&dev->post.run_count);

    /* One bounded retry is permitted. Every attempt starts from and restores
     * the exact production configuration; there is no unbounded startup loop. */
    for(uint8_t attempt = 1u; attempt <= 2u; attempt++)
    {
        dev->post.attempts = attempt;
        status = adbms6830_run_startup_post_once(dev, production_cfga);
        if(status == HAL_OK)
        {
            break;
        }
    }

    /* A passing POST intentionally exercised Status-C fault bits. Do not let
     * those injected observations poison runtime sticky fault history. Restore
     * the sticky state that existed before POST. Transport/counter statistics
     * remain untouched and therefore still expose any real communication
     * errors encountered while running the test. */
    if(status == HAL_OK)
    {
        dev->health.sticky_status_fault_ic_mask = prior_sticky_status;
        dev->health.sticky_reference_fault_ic_mask = prior_sticky_reference;
        dev->health.sticky_open_wire_fault_ic_mask = prior_sticky_open_wire;
    }
    else
    {
        dev->health.sticky_status_fault_ic_mask |= prior_sticky_status;
        dev->health.sticky_reference_fault_ic_mask |= prior_sticky_reference;
        dev->health.sticky_open_wire_fault_ic_mask |= prior_sticky_open_wire;
    }

    dev->post.last_status = status;
    dev->post.passed = (status == HAL_OK);
    dev->post.stage = dev->post.passed ? ADBMS6830_POST_PASS : ADBMS6830_POST_FAIL;
    if(!dev->post.passed)
    {
        adbms6830_increment_u32_sat(&dev->post.fail_count);
    }
    return status;
#else
    (void)dev;
    return HAL_OK;
#endif
}

HAL_StatusTypeDef adbms6830_wakeup_checked(adbms6830_driver_t* dev)
{
	HAL_StatusTypeDef status;

	if((dev == NULL) || !adbms6830_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	for(uint8_t i = 0u; i < dev->physical_chain_count; i++)
	{
		adbms6830_set_cs(dev, 0);
		status = adbms6830_us_delay(dev, WAKEUP_US_DELAY);
		adbms6830_set_cs(dev, 1);
		if(status != HAL_OK)
		{
			return status;
		}
		status = adbms6830_us_delay(dev, WAKEUP_BW_DELAY);
		if(status != HAL_OK)
		{
			return status;
		}
	}

    if(dev->session.full_wake_count != UINT32_MAX)
    {
        dev->session.full_wake_count++;
    }
    if(dev->session.active)
    {
        dev->session.last_activity_us = adbms6830_runtime_time_us(dev);
    }
	return HAL_OK;
}

void adbms6830_wakeup(adbms6830_driver_t* dev)
{
	(void)adbms6830_wakeup_checked(dev);
}

void adbms6830_wakeup_cold(adbms6830_driver_t* dev)
{
    if(!adbms6830_topology_valid(dev))
    {
        return;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_COLD_WAKE;
    }

    for(uint8_t pass = 0u; pass < 2u; pass++)
    {
		for(uint8_t i = 0u; i < dev->physical_chain_count; i++)
        {
            adbms6830_set_cs(dev, 0);
            if(adbms6830_us_delay(dev, WAKEUP_US_DELAY) != HAL_OK)
            {
                adbms6830_set_cs(dev, 1);
                return;
            }
            adbms6830_set_cs(dev, 1);
            if(adbms6830_us_delay(dev, WAKEUP_BW_DELAY) != HAL_OK)
            {
                return;
            }
        }
    }

    /* Leave time for references/regulators after a true sleep/cold state. */
    (void)adbms6830_wait_cooperative(dev, 5000u);
}

void adbms6830_pack_comm(adbms6830_driver_t* dev)
{
	if(!adbms6830_topology_valid(dev))
	{
		return;
	}

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
	if(!adbms6830_topology_valid(dev))
	{
		return;
	}

	adbms6830_asic *ics = dev->ics;
	/* The Rev. 0 status table and ADI Release 1.0.3 reference parser swap the
	 * VDE/VDEL names for bits 6 and 7.  The production clear path deliberately
	 * sets both bits; do not introduce a selective clear until that naming is
	 * resolved against the exact ordered silicon documentation. */
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

HAL_StatusTypeDef adbms6830_spi_write(adbms6830_driver_t* dev,
                                         uint8_t* data,
                                         uint16_t len,
                                         uint8_t use_cs)
{
    HAL_StatusTypeDef status;

    if((dev == NULL) || (dev->hspi == NULL) || (data == NULL) || (len == 0u) ||
       (use_cs && !adbms6830_active_cs_valid(dev)))
    {
        return HAL_ERROR;
    }

    adbms6830_spi_debug_note_tx(dev, dev->spi_debug.last_op, data, data, len, 0u);

    adbms_spi_lock();
    if(use_cs)
    {
        adbms6830_set_cs(dev, 0);
    }

    status = HAL_SPI_Transmit(dev->hspi, data, len, SPI_TIMEOUT);

    if(use_cs)
    {
        adbms6830_set_cs(dev, 1);
    }
    adbms_spi_unlock();

    if(status == HAL_OK)
    {
        adbms6830_session_note_activity(dev);
    }

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


void adbms6830_cmd(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ])
{
    (void)adbms6830_cmd_checked(dev, cmd);
}

static HAL_StatusTypeDef adbms6830_cmd_checked(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ])
{
    uint16_t pec15;
    HAL_StatusTypeDef status;

    if((dev == NULL) || (cmd == NULL) || !adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    write_buf[0] = cmd[0];
    write_buf[1] = cmd[1];
    pec15 = Pec15_Calc(CMDSZ, cmd);
    write_buf[2] = (uint8_t)(pec15 >> 8);
    write_buf[3] = (uint8_t)(pec15);

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CMD;
    }

    status = adbms6830_spi_write(dev, write_buf, CMDSZ + PEC15SZ, 1);
    if(status == HAL_OK)
    {
        if(adbms6830_cmd_resets_counter(cmd))
        {
            adbms6830_note_counter_reset(dev);
        }
        else if(adbms6830_cmd_increments_counter(cmd))
        {
            adbms6830_note_counter_increment(dev);
        }
    }

    return status;
}

// Tx/Rx Utility
void adbms6830_wr48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* tx_data)
{
    (void)adbms6830_wr48_checked(dev, cmd, tx_data);
}

static HAL_StatusTypeDef adbms6830_wr48_checked(adbms6830_driver_t* dev,
                                                uint8_t cmd[CMDSZ],
                                                uint8_t* tx_data)
{
    uint16_t pec15;
    uint16_t data_pec;
    uint16_t tx_sz;
    uint16_t cmd_index = 0;
    uint8_t src_addr = 0;
    uint8_t temp[TX_DATA];
    HAL_StatusTypeDef status;

    if(!adbms6830_topology_valid(dev) ||
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

    /* 1. Wakeup the isoSPI Daisy Chain */
    status = adbms6830_wakeup_checked(dev);
    if(status != HAL_OK)
    {
        return status;
    }

    /* 2. Prepare Command + PEC15 */
    write_buf[0] = cmd[0];
    write_buf[1] = cmd[1];
    pec15 = Pec15_Calc(CMDSZ, cmd);
    write_buf[2] = (uint8_t)(pec15 >> 8);
    write_buf[3] = (uint8_t)pec15;
    cmd_index = 4;

    /* 3. Pack Data + PEC10 in reverse order for daisy chain */
    for (uint8_t current_ic = (uint8_t)dev->num_ics; current_ic > 0u; current_ic--)
    {
        src_addr = (uint8_t)((current_ic - 1u) * TX_DATA);

        for (uint8_t current_byte = 0u; current_byte < TX_DATA; current_byte++)
        {
            write_buf[cmd_index] = tx_data[src_addr + current_byte];
            temp[current_byte] = tx_data[src_addr + current_byte];
            cmd_index++;
        }

        data_pec = pec10_calc(0, TX_DATA, temp);
        write_buf[cmd_index] = (uint8_t)(data_pec >> 8);
        cmd_index++;
        write_buf[cmd_index] = (uint8_t)data_pec;
        cmd_index++;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_WR48;
    }

    status = adbms6830_spi_write(dev, write_buf, tx_sz, 1);
    if((status == HAL_OK) && adbms6830_wr48_increments_counter(cmd))
    {
        adbms6830_note_counter_increment(dev);
    }

    return status;
}


void adbms6830_rd48(adbms6830_driver_t* dev, uint8_t cmd[CMDSZ], uint8_t* rx_data)
{
    (void)adbms6830_rd48_checked(dev, cmd, rx_data);
}

static HAL_StatusTypeDef adbms6830_rd48_checked(adbms6830_driver_t* dev,
                                                uint8_t cmd[CMDSZ],
                                                uint8_t* rx_data)
{
    uint16_t pec15;
    uint16_t received_pec;
    uint16_t calculated_pec;
    uint16_t rx_sz;
    uint8_t wrcmd[CMDSZ + PEC15SZ] = {0};
    HAL_StatusTypeDef status;

    if(!adbms6830_topology_valid(dev) || (cmd == NULL) || (rx_data == NULL))
    {
        return HAL_ERROR;
    }

    rx_sz = RX_DATA * (uint16_t)dev->num_ics;
    if(rx_sz > BUFSZ)
    {
        return HAL_ERROR;
    }

    /* 1. Prepare Command + PEC15 */
    wrcmd[0] = cmd[0];
    wrcmd[1] = cmd[1];
    pec15 = Pec15_Calc(CMDSZ, cmd);
    wrcmd[2] = (uint8_t)(pec15 >> 8);
    wrcmd[3] = (uint8_t)pec15;

    dev->health.last_pec_pass_mask = 0u;
    dev->health.last_pec_fail_mask = 0u;
    dev->health.last_cmd_counter_mismatch_mask = 0u;
    dev->health.unexpected_counter_reset_mask = 0u;

    /* 2. Standalone reads retain the conservative checked wake.  A bounded
     * awake session may skip it only while the session-age guard is valid. */
#if AMS_ENABLE_ADBMS_AWAKE_SESSION
    status = dev->session.active ?
             adbms6830_session_require_awake(dev) :
             adbms6830_wakeup_checked(dev);
#else
    status = adbms6830_wakeup_checked(dev);
#endif
    if(status != HAL_OK)
    {
        return status;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_RD48;
    }

    /* 3. Send Command and Read Data with one CS-low full-duplex transfer. */
    status = adbms6830_spi_write_read(dev, wrcmd, CMDSZ + PEC15SZ, rx_data, rx_sz, 1);
    if(status != HAL_OK)
    {
        return status;
    }

    /* 4. Parse, Check PEC10, and Extract Command Counter */
    for (uint8_t current_ic = 0u; current_ic < (uint8_t)dev->num_ics; current_ic++)
    {
        uint16_t ic_base_idx = (uint16_t)current_ic * RX_DATA;
        uint8_t* ic_data = &rx_data[ic_base_idx];

        uint8_t cmd_counter = (uint8_t)(ic_data[RX_DATA - 2u] >> 2u);
        received_pec = (uint16_t)(((ic_data[RX_DATA - 2u] & 0x03u) << 8u) | ic_data[RX_DATA - 1u]);
        calculated_pec = pec10_calc(1, RX_DATA - 2u, ic_data);

        if(dev->spi_debug.enabled && (current_ic < ADBMS6830_MAX_TRACKED_ICS))
        {
            dev->spi_debug.last_cmd_counter[current_ic] = cmd_counter;
        }

        if (received_pec != calculated_pec)
        {
            adbms6830_note_pec_result(dev, current_ic, false);
            if(dev->spi_debug.enabled)
            {
                dev->spi_debug.last_read_pec_fail_mask |= (uint16_t)(1u << current_ic);
                dev->spi_debug.error_count++;
            }
        }
        else
        {
            adbms6830_note_pec_result(dev, current_ic, true);
            if(dev->spi_debug.enabled)
            {
                dev->spi_debug.last_read_pec_pass_mask |= (uint16_t)(1u << current_ic);
            }
        }

        /* Command-counter validation is a transport-integrity check, not a
         * diagnostic-logging feature.  Keep tracking it even when verbose SPI
         * capture is disabled from the service CLI. */
        adbms6830_note_observed_counter(dev,
                                        current_ic,
                                        cmd_counter,
                                        received_pec == calculated_pec);
    }

    return HAL_OK;
}

// SPI communication
void adbms6830_set_cs(adbms6830_driver_t* dev, uint8_t state)
{
    if(!adbms6830_active_cs_valid(dev))
    {
        return;
    }

    HAL_GPIO_WritePin(dev->cs_port[dev->string], dev->cs_pin[dev->string], state);
}


HAL_StatusTypeDef adbms6830_spi_write_read(adbms6830_driver_t *dev,
                                           uint8_t* tx_Data,
                                           uint8_t tx_len,
                                           uint8_t* rx_data,
                                           uint16_t rx_len,
                                           uint8_t use_cs)
{
    HAL_StatusTypeDef status;
    uint16_t total_len;

    if((dev == NULL) || (dev->hspi == NULL) || (tx_Data == NULL) ||
       (rx_data == NULL) || (tx_len == 0u) || (rx_len == 0u) ||
       (use_cs && !adbms6830_active_cs_valid(dev)))
    {
        return HAL_ERROR;
    }

    if((rx_len > BUFSZ) ||
       (((uint32_t)tx_len + (uint32_t)rx_len) > BUFSZ))
    {
        return HAL_ERROR;
    }
    total_len = (uint16_t)((uint16_t)tx_len + rx_len);

    memset(spi_txrx_tx_buf, ADBMS6830_SPI_DUMMY_BYTE, total_len);
    memset(spi_txrx_rx_buf, 0, total_len);
    memcpy(spi_txrx_tx_buf, tx_Data, tx_len);

    adbms6830_spi_debug_note_tx(dev, dev->spi_debug.last_op, tx_Data, spi_txrx_tx_buf, tx_len, rx_len);

    adbms_spi_lock();
    if(use_cs)
    {
        adbms6830_set_cs(dev, 0);
    }

    status = HAL_SPI_TransmitReceive(dev->hspi,
                                     spi_txrx_tx_buf,
                                     spi_txrx_rx_buf,
                                     total_len,
                                     SPI_TIMEOUT);

    if(use_cs)
    {
        adbms6830_set_cs(dev, 1);
    }
    adbms_spi_unlock();

    if(status == HAL_OK)
    {
        memcpy(rx_data, &spi_txrx_rx_buf[tx_len], rx_len);
        adbms6830_session_note_activity(dev);
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

    adbms6830_spi_debug_note_rx(dev, rx_data, rx_len, status);

    return status;
}

void adbms6830_bind_runtime_hooks(adbms6830_driver_t *dev,
                                  adbms6830_cooperative_wait_fn_t wait_fn,
                                  adbms6830_time_us_fn_t time_fn,
                                  void *ctx)
{
    if(dev == NULL)
    {
        return;
    }
    dev->cooperative_wait_fn = wait_fn;
    dev->time_us_fn = time_fn;
    dev->runtime_hook_ctx = ctx;
}

const adbms6830_session_health_t *adbms6830_session_health_get(const adbms6830_driver_t *dev)
{
    return (dev != NULL) ? &dev->session : NULL;
}

HAL_StatusTypeDef adbms6830_session_inject_gap_once(adbms6830_driver_t *dev,
                                                       uint32_t gap_us,
                                                       bool bypass_guard)
{
#if AMS_ENABLE_SERVICE_CLI
    if((dev == NULL) || (gap_us == 0u) || (gap_us > 100000u))
    {
        return HAL_ERROR;
    }
    dev->session.inject_gap_us_once = gap_us;
    dev->session.inject_bypass_guard_once = bypass_guard;
    return HAL_OK;
#else
    (void)dev;
    (void)gap_us;
    (void)bypass_guard;
    return HAL_ERROR;
#endif
}

void adbms6830_session_begin_scan(adbms6830_driver_t *dev)
{
    if(dev != NULL)
    {
        dev->session.wake_count_scan_start = dev->session.full_wake_count;
    }
}

void adbms6830_session_end_scan(adbms6830_driver_t *dev)
{
    if(dev != NULL)
    {
        dev->session.wake_count_last_scan =
            dev->session.full_wake_count - dev->session.wake_count_scan_start;
    }
}

HAL_StatusTypeDef adbms6830_wait_cooperative(adbms6830_driver_t *dev, uint32_t microseconds)
{
    if(dev == NULL)
    {
        return HAL_ERROR;
    }
    if(microseconds == 0u)
    {
        return HAL_OK;
    }

    if((microseconds >= AMS_ADBMS_COOPERATIVE_WAIT_MIN_US) &&
       (dev->cooperative_wait_fn != NULL))
    {
        HAL_StatusTypeDef status;
        if(dev->session.long_wait_count != UINT32_MAX)
        {
            dev->session.long_wait_count++;
        }
        if(UINT64_MAX - dev->session.long_wait_requested_us >= microseconds)
        {
            dev->session.long_wait_requested_us += microseconds;
        }
        status = dev->cooperative_wait_fn(dev->runtime_hook_ctx, microseconds);
        if((status == HAL_BUSY) &&
           (dev->session.long_wait_interrupted_count != UINT32_MAX))
        {
            dev->session.long_wait_interrupted_count++;
        }
        return status;
    }

    if(microseconds > UINT16_MAX)
    {
        return HAL_ERROR;
    }
    return adbms6830_us_delay(dev, (uint16_t)microseconds);
}

HAL_StatusTypeDef adbms6830_us_delay(adbms6830_driver_t* dev, uint16_t microseconds)
{
	uint32_t spins = 0u;
	uint32_t spin_budget;

	if(dev == NULL)
	{
		return HAL_ERROR;
	}
	if((dev->htim == NULL) || (dev->htim->Instance == NULL))
	{
		dev->delay_last_status = HAL_ERROR;
		return HAL_ERROR;
	}
	if(microseconds == 0u)
	{
		dev->delay_last_status = HAL_OK;
		return HAL_OK;
	}

	spin_budget = ADBMS6830_DELAY_BASE_SPINS +
	              ((uint32_t)microseconds * ADBMS6830_DELAY_SPINS_PER_US);
	__HAL_TIM_SET_COUNTER(dev->htim, 0);
	while(__HAL_TIM_GET_COUNTER(dev->htim) < microseconds)
	{
		if(spins++ >= spin_budget)
		{
			dev->delay_last_status = HAL_TIMEOUT;
			if(dev->delay_timeout_count != UINT32_MAX)
			{
				dev->delay_timeout_count++;
			}
			return HAL_TIMEOUT;
		}
	}
	dev->delay_last_status = HAL_OK;
	return HAL_OK;
}

void adbms6830_adcv(adbms6830_driver_t *dev, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs)
{
    (void)adbms6830_adcv_checked(dev, rd, cont, dcp, rstf, owcs);
}

static HAL_StatusTypeDef adbms6830_adcv_checked(adbms6830_driver_t *dev,
                                                RD rd,
                                                CONT cont,
                                                DCP dcp,
                                                RSTF rstf,
                                                OW_C_S owcs)
{
    uint8_t cmd[2];
    HAL_StatusTypeDef status;
    cmd[0] = 0x02u | (uint8_t)rd;
    cmd[1] = ((uint8_t)cont << 7u) | ((uint8_t)dcp << 4u)
           | ((uint8_t)rstf << 2u) | ((uint8_t)owcs & 0x03u) | 0x60u;
    /* Every ADCV restarts the C converters and resets CCTS. A diagnostic ADCV
     * therefore invalidates the post-ECO "continuous C already established"
     * state until the production command is explicitly issued again. */
    if(dev != NULL)
    {
        dev->continuous_c_running = false;
    }
    status = adbms6830_wakeup_checked(dev);
    return (status == HAL_OK) ? adbms6830_cmd_checked(dev, cmd) : status;
}

/* Send one polling command and keep CS asserted while clocking status bits.
 * In a daisy chain, the status is valid after at least 2*N clocks and remains
 * low while any device is busy. The final received bit is sampled after every
 * full byte, which is safely beyond the required propagation clocks. */
static HAL_StatusTypeDef adbms6830_poll_conversion_checked(
    adbms6830_driver_t *dev,
    const uint8_t poll_cmd[CMDSZ],
    uint32_t timeout_us,
    uint32_t *elapsed_us,
    uint32_t *clock_bytes,
    bool *observed_busy,
    bool *complete)
{
    uint8_t cmd_frame[CMDSZ + PEC15SZ];
    uint8_t tx = 0xFFu;
    uint8_t rx = 0u;
    uint16_t pec15;
    uint32_t bytes = 0u;
    uint32_t minimum_bits;
    uint32_t spins = 0u;
    uint32_t spin_budget;
    HAL_StatusTypeDef status;

    if(elapsed_us != NULL)
    {
        *elapsed_us = 0u;
    }
    if(clock_bytes != NULL)
    {
        *clock_bytes = 0u;
    }
    if(complete != NULL)
    {
        *complete = false;
    }
    if(observed_busy != NULL)
    {
        *observed_busy = false;
    }

    if(!adbms6830_topology_valid(dev) || (poll_cmd == NULL) ||
       (dev->htim == NULL) || (dev->htim->Instance == NULL) ||
       (timeout_us == 0u))
    {
        return HAL_ERROR;
    }

    /* The caller has already woken the chain and started the conversion.
     * Do not insert another wake delay here: it would distort the measured
     * conversion time and could allow a short conversion to complete before
     * polling begins. */
    cmd_frame[0] = poll_cmd[0];
    cmd_frame[1] = poll_cmd[1];
    pec15 = Pec15_Calc(CMDSZ, (uint8_t *)poll_cmd);
    cmd_frame[2] = (uint8_t)(pec15 >> 8u);
    cmd_frame[3] = (uint8_t)pec15;

    minimum_bits = 2u * (uint32_t)dev->physical_chain_count;
    spin_budget = ADBMS6830_POLL_SPIN_MARGIN +
                  (timeout_us * ADBMS6830_DELAY_SPINS_PER_US);

    adbms_spi_lock();
    adbms6830_set_cs(dev, 0u);
    status = HAL_SPI_Transmit(dev->hspi, cmd_frame, sizeof(cmd_frame), SPI_TIMEOUT);
    if(status == HAL_OK)
    {
        adbms6830_note_counter_increment(dev);
        __HAL_TIM_SET_COUNTER(dev->htim, 0u);

        while(__HAL_TIM_GET_COUNTER(dev->htim) < timeout_us)
        {
            status = HAL_SPI_TransmitReceive(dev->hspi, &tx, &rx, 1u, SPI_TIMEOUT);
            if(status != HAL_OK)
            {
                break;
            }

            bytes++;
            if((bytes * 8u) >= minimum_bits)
            {
                bool ready = (rx & 0x01u) != 0u;

                if(!ready)
                {
                    if(observed_busy != NULL)
                    {
                        *observed_busy = true;
                    }
                }
                else if((observed_busy != NULL) && *observed_busy)
                {
                    if(complete != NULL)
                    {
                        *complete = true;
                    }
                    break;
                }
            }

            if(spins++ >= spin_budget)
            {
                status = HAL_TIMEOUT;
                break;
            }
        }

        if((status == HAL_OK) && (complete != NULL) && !(*complete))
        {
            status = HAL_TIMEOUT;
        }
    }
    adbms6830_set_cs(dev, 1u);
    adbms_spi_unlock();

    if(elapsed_us != NULL)
    {
        *elapsed_us = __HAL_TIM_GET_COUNTER(dev->htim);
    }
    if(clock_bytes != NULL)
    {
        *clock_bytes = bytes;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.tx_count++;
        dev->spi_debug.rx_count += bytes;
        dev->spi_debug.last_tx_status = status;
        dev->spi_debug.last_rx_status = status;
        dev->spi_debug.last_xfer_status = status;
        dev->spi_debug.last_status = status;
        dev->spi_debug.last_cmd[0] = poll_cmd[0];
        dev->spi_debug.last_cmd[1] = poll_cmd[1];
        if(status != HAL_OK)
        {
            dev->spi_debug.error_count++;
        }
    }

    return status;
}

HAL_StatusTypeDef adbms6830_start_adc_cell_voltage_measurement(adbms6830_driver_t *dev)
{
#if AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && AMS_S_PATH_ECO_VALIDATED
    /* Post-ECO production architecture: keep the primary C converters
     * continuous and uninterrupted so raw C remains authoritative and the
     * AVG8/IIR products retain continuous history. The independent S path is
     * brought up only by adbms6830_run_s_periodic_diagnostic(). */
    HAL_StatusTypeDef status;
    if(dev == NULL)
    {
        return HAL_ERROR;
    }
    if(dev->continuous_c_running)
    {
        return HAL_OK;
    }
    status = adbms6830_adcv_checked(dev,
                                    RD_OFF,
                                    CONTINUOUS,
                                    DCP_OFF,
                                    RSTF_OFF,
                                    OW_OFF_ALL_CH);
    if(status == HAL_OK)
    {
        dev->continuous_c_running = true;
    }
    return status;
#else
    /* Current Rev5 hardware intentionally keeps S observable because the
     * S2N-S15N routing defect is still under qualification. Published voltage
     * remains C-authoritative; there is no automatic S fallback. */
    return adbms6830_adcv_checked(dev,
                                 RD_ON,
                                 CONTINUOUS,
                                 DCP_OFF,
                                 RSTF_OFF,
                                 OW_OFF_ALL_CH);
#endif
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

static uint16_t adbms6830_cell_group_mask(GRP grp)
{
    switch(grp)
    {
        case A: return 0x0007u;       /* cells 0..2 */
        case B: return 0x0038u;       /* cells 3..5 */
        case C: return 0x01C0u;       /* cells 6..8 */
        case D: return 0x0E00u;       /* cells 9..11 */
        case E: return 0x7000u;       /* cells 12..14 */
        case F: return 0x8000u;       /* cell 15, unused on DER26 SMB */
        default: return 0u;
    }
}

void adbms6830_parse_cell(adbms6830_driver_t *dev, uint8_t *data, GRP grp)
{
    if(!adbms6830_topology_valid(dev) || (data == NULL))
    {
        return;
    }

    #define IS_VALID_CODE(lo, hi) \
        (!((d[lo] == 0xFFu) && (d[hi] == 0xFFu)) && \
         !((d[lo] == 0x00u) && (d[hi] == 0x80u)))

    #define STORE_CODE(idx, byte_lo, byte_hi)                                       \
    do {                                                                            \
        if (IS_VALID_CODE(byte_lo, byte_hi)) {                                      \
            dev->ics[curr_ic].cell.c_codes[idx] =                                   \
                (int16_t)((uint16_t)d[byte_lo] | ((uint16_t)d[byte_hi] << 8u));    \
            if(curr_ic < ADBMS6830_MAX_TRACKED_ICS) {                              \
                dev->last_cell_updated_mask[curr_ic] |= (uint16_t)(1u << (idx));   \
            }                                                                       \
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

        if (dev->ics[curr_ic].cccrc.cell_pec)
        {
            if(curr_ic < ADBMS6830_MAX_TRACKED_ICS)
            {
                dev->last_cell_pec_mask[curr_ic] |= adbms6830_cell_group_mask(grp);
            }
            continue;
        }

        /* A valid PEC proves the bytes were received intact, but a command
         * counter mismatch means they do not belong to the transaction that
         * requested them.  Do not refresh any cells from that IC/group. */
        if(!adbms6830_read_packet_counter_ok(dev, curr_ic))
        {
            continue;
        }

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

static void adbms6830_mask_open_wire_cells(adbms6830_driver_t *dev)
{
    if(!adbms6830_topology_valid(dev))
    {
        return;
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        /* The most recent complete even/odd diagnostic owns this channel mask.
         * Suppress apparently plausible normal C-ADC data from a lead already
         * diagnosed open. A later complete pass may clear the active mask, but
         * the application safety fault remains latched until reset. */
        dev->last_cell_updated_mask[ic] &=
            (uint16_t)~dev->diag[ic].open_wire_fault_mask;
    }
}

static void adbms6830_parse_cell_product(adbms6830_driver_t *dev,
                                         const uint8_t *data,
                                         GRP grp,
                                         bool filtered)
{
    uint8_t first_cell;
    uint16_t group_mask;

    if(!adbms6830_topology_valid(dev) || (data == NULL))
    {
        return;
    }

    first_cell = adbms6830_cell_group_first_index(grp);
    group_mask = adbms6830_cell_group_mask(grp);
    if((first_cell == UINT8_MAX) || (group_mask == 0u))
    {
        return;
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        const uint8_t *packet = &data[(uint16_t)ic * RX_DATA];
        bool pec_ok = adbms6830_read_packet_pec_ok(packet);
        bool counter_ok = adbms6830_read_packet_counter_ok(dev, ic);
        uint16_t *updated = filtered ? &dev->last_fcell_updated_mask[ic] :
                                      &dev->last_acell_updated_mask[ic];
        uint16_t *pec_mask = filtered ? &dev->last_fcell_pec_mask[ic] :
                                       &dev->last_acell_pec_mask[ic];

        if(!pec_ok)
        {
            *pec_mask |= group_mask;
            continue;
        }
        if(!counter_ok)
        {
            continue;
        }

        for(uint8_t slot = 0u; slot < 3u; slot++)
        {
            uint8_t cell = (uint8_t)(first_cell + slot);
            uint8_t lo = (uint8_t)(slot * 2u);
            uint8_t hi = (uint8_t)(lo + 1u);
            int16_t code;

            if(cell >= dev->monitored_cell_count)
            {
                continue;
            }
            if(((packet[lo] == 0xFFu) && (packet[hi] == 0xFFu)) ||
               ((packet[lo] == 0x00u) && (packet[hi] == 0x80u)))
            {
                continue;
            }

            code = (int16_t)((uint16_t)packet[lo] |
                             ((uint16_t)packet[hi] << 8u));
            if(filtered)
            {
                dev->ics[ic].fcell.fc_codes[cell] = code;
            }
            else
            {
                dev->ics[ic].acell.ac_codes[cell] = code;
            }
            *updated |= (uint16_t)(1u << cell);
        }
    }
}

static HAL_StatusTypeDef adbms6830_capture_coherent_cadc_counter(
    adbms6830_driver_t *dev,
    const uint8_t *data)
{
    uint16_t expected_mask;
    uint16_t valid_mask = 0u;
    uint16_t fault_mask = 0u;

    if(!adbms6830_topology_valid(dev) || (data == NULL))
    {
        return HAL_ERROR;
    }

    expected_mask = adbms6830_expected_ic_mask(dev);
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        const uint8_t *packet = &data[(uint16_t)ic * RX_DATA];
        uint16_t bit = (uint16_t)(1u << ic);
        uint16_t ct;
        uint16_t ccts;

        if(!adbms6830_read_packet_pec_ok(packet) ||
           !adbms6830_read_packet_counter_ok(dev, ic))
        {
            fault_mask |= bit;
            continue;
        }

        ct = (uint16_t)(((uint16_t)(packet[2] & 0x1Fu) << 6u) |
                        ((uint16_t)(packet[3] & 0xFCu) >> 2u));
        ccts = (uint16_t)((ct << 2u) | (uint16_t)(packet[3] & 0x03u));
        ccts &= 0x1FFFu;
        dev->health.cadc_ccts_last[ic] = ccts;
        valid_mask |= bit;

#if AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && AMS_S_PATH_ECO_VALIDATED
        /* In the post-ECO architecture C conversion is intentionally left
         * running between scans.  Equal coherent counters on consecutive
         * epochs are therefore a real stale-conversion indication.  Modular
         * wrap (including a value of zero) is valid as long as it advanced. */
        if((dev->health.cadc_ccts_initialized_mask & bit) != 0u)
        {
            uint16_t previous = dev->health.cadc_ccts_previous[ic];
            uint16_t delta = (uint16_t)((ccts - previous) & 0x1FFFu);
            if(delta == 0u)
            {
                fault_mask |= bit;
            }
        }
#else
        /* Current Rev5 restarts ADCV for each scan. After the bounded
         * conversion wait the coherent counter must have left reset zero.
         * This catches a plausible-but-stale C register image without adding
         * a guessed minimum conversion-count threshold. */
        if(ccts == 0u)
        {
            fault_mask |= bit;
        }
#endif

        dev->health.cadc_ccts_previous[ic] = ccts;
        dev->health.cadc_ccts_initialized_mask |= bit;
    }

    dev->health.cadc_ccts_valid_ic_mask = valid_mask;
    dev->health.cadc_ccts_fault_ic_mask = fault_mask;
    dev->health.sticky_cadc_ccts_fault_ic_mask |= fault_mask;
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        if((fault_mask & (uint16_t)(1u << ic)) != 0u)
        {
            adbms6830_increment_u32_sat(&dev->health.cadc_ccts_fault_count[ic]);
        }
    }

    return ((valid_mask == expected_mask) && (fault_mask == 0u)) ?
           HAL_OK : HAL_ERROR;
}

static void adbms6830_invalidate_avg8_product(adbms6830_driver_t *dev)
{
    if(dev == NULL)
    {
        return;
    }
    memset(dev->last_acell_updated_mask, 0, sizeof(dev->last_acell_updated_mask));
    memset(dev->last_acell_pec_mask, 0, sizeof(dev->last_acell_pec_mask));
}

static void adbms6830_invalidate_filtered_product(adbms6830_driver_t *dev)
{
    if(dev == NULL)
    {
        return;
    }
    memset(dev->last_fcell_updated_mask, 0, sizeof(dev->last_fcell_updated_mask));
    memset(dev->last_fcell_pec_mask, 0, sizeof(dev->last_fcell_pec_mask));
    dev->filtered_voltage_ready = false;
    dev->filtered_successful_epoch_count = 0u;
}

static HAL_StatusTypeDef adbms6830_read_voltage_epoch(adbms6830_driver_t *dev,
                                                       bool read_avg8,
                                                       bool read_filtered)
{
    static uint8_t *const raw_commands[] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};
    static uint8_t *const avg_commands[] = {RDACA, RDACB, RDACC, RDACD, RDACE, RDACF};
    static uint8_t *const filt_commands[] = {RDFCA, RDFCB, RDFCC, RDFCD, RDFCE, RDFCF};
    static const GRP groups[] = {A, B, C, D, E, F};
    HAL_StatusTypeDef critical_error = HAL_OK;
    HAL_StatusTypeDef status = HAL_OK;
    bool snapped = false;

    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        dev->last_cell_updated_mask[ic] = 0u;
        dev->last_cell_pec_mask[ic] = 0u;
        dev->last_acell_updated_mask[ic] = 0u;
        dev->last_acell_pec_mask[ic] = 0u;
        dev->last_fcell_updated_mask[ic] = 0u;
        dev->last_fcell_pec_mask[ic] = 0u;
        dev->last_temp_updated_mask[ic] = 0u;
        dev->diag[ic].statd_valid = false;
    }
    dev->health.cadc_ccts_valid_ic_mask = 0u;
    dev->health.cadc_ccts_fault_ic_mask = 0u;

    status = adbms6830_wakeup_checked(dev);
    if(status != HAL_OK)
    {
        return status;
    }

#if AMS_ENABLE_ADBMS_AWAKE_SESSION
    adbms6830_session_open(dev);
#endif
    status = adbms6830_cmd_checked(dev, SNAP);
    if(status == HAL_OK)
    {
        dev->session.coherent_snapshot_active = true;
        snapped = true;
        status = adbms6830_us_delay(dev, 10u);
    }
    if(status != HAL_OK)
    {
        critical_error = status;
    }

    /* Raw C voltage is the safety-authoritative product. Any raw group
     * integrity failure invalidates this coherent epoch. */
    for(uint8_t group = 0u;
        (critical_error == HAL_OK) &&
        (group < (uint8_t)(sizeof(groups) / sizeof(groups[0])));
        group++)
    {
        status = adbms6830_rd48_checked(dev, raw_commands[group], shared_buf);
        if(status == HAL_OK)
        {
            adbms6830_parse_cell(dev, shared_buf, groups[group]);
            if(!adbms6830_last_read_integrity_ok(dev))
            {
                status = HAL_ERROR;
            }
        }
        if(status != HAL_OK)
        {
            critical_error = status;
        }
    }

    /* Status C is captured in the same SNAP epoch as raw C. CCTS is an
     * independent freshness proof for the C converter. Unlike AVG8/IIR, a
     * failed/stalled coherent conversion counter makes the raw image unsafe. */
    if(critical_error == HAL_OK)
    {
        status = adbms6830_rd48_checked(dev, RDSTATC, shared_buf);
        if(status == HAL_OK)
        {
            adbms6830_increment_u32_sat(&dev->health.coherent_statc_read_count);
            adbms6830_parse_statc(dev, shared_buf);
            if(!adbms6830_last_read_integrity_ok(dev) ||
               (adbms6830_capture_coherent_cadc_counter(dev, shared_buf) != HAL_OK))
            {
                status = HAL_ERROR;
            }
        }
        if(status != HAL_OK)
        {
            adbms6830_increment_u32_sat(&dev->health.coherent_statc_read_fail_count);
            critical_error = status;
        }
    }

    /* Status D is also frozen by SNAP, so keep the ASIC CxOV/CxUV image
     * aligned with the raw software-threshold image. A transport error here
     * is diagnostic degradation rather than permission to discard otherwise
     * valid raw C measurements. A guard expiry is different: the coherent
     * snapshot session itself has become uncertain and must restart. */
    if(critical_error == HAL_OK)
    {
        status = adbms6830_rd48_checked(dev, RDSTATD, shared_buf);
        if(status == HAL_OK)
        {
            adbms6830_increment_u32_sat(&dev->health.coherent_statd_read_count);
            adbms6830_parse_statd(dev, shared_buf);
            if(!adbms6830_last_read_integrity_ok(dev))
            {
                status = HAL_ERROR;
            }
        }
        if(status != HAL_OK)
        {
            adbms6830_increment_u32_sat(&dev->health.coherent_statd_read_fail_count);
            for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
            {
                dev->diag[ic].statd_valid = false;
            }
            if(status == HAL_BUSY)
            {
                critical_error = status;
            }
        }
    }

    /* AVG8 and filtered values are estimator/diagnostic products only. A PEC
     * or register failure in either product withdraws that product but never
     * revokes a clean raw safety image. A session guard expiry still forces a
     * coherent-epoch restart because no product may be spliced across wakes. */
    if(read_avg8 && (critical_error == HAL_OK))
    {
        HAL_StatusTypeDef product_status = HAL_OK;
        for(uint8_t group = 0u;
            group < (uint8_t)(sizeof(groups) / sizeof(groups[0]));
            group++)
        {
            status = adbms6830_rd48_checked(dev, avg_commands[group], shared_buf);
            if(status == HAL_OK)
            {
                adbms6830_parse_cell_product(dev, shared_buf, groups[group], false);
                if(!adbms6830_last_read_integrity_ok(dev))
                {
                    status = HAL_ERROR;
                }
            }
            if(status != HAL_OK)
            {
                product_status = status;
                break;
            }
        }
        if(product_status == HAL_OK)
        {
            adbms6830_increment_u32_sat(&dev->health.avg8_read_count);
        }
        else
        {
            adbms6830_increment_u32_sat(&dev->health.avg8_read_fail_count);
            adbms6830_invalidate_avg8_product(dev);
            if(product_status == HAL_BUSY)
            {
                critical_error = product_status;
            }
        }
    }

    if(read_filtered && (critical_error == HAL_OK))
    {
        HAL_StatusTypeDef product_status = HAL_OK;
        for(uint8_t group = 0u;
            group < (uint8_t)(sizeof(groups) / sizeof(groups[0]));
            group++)
        {
            status = adbms6830_rd48_checked(dev, filt_commands[group], shared_buf);
            if(status == HAL_OK)
            {
                adbms6830_parse_cell_product(dev, shared_buf, groups[group], true);
                if(!adbms6830_last_read_integrity_ok(dev))
                {
                    status = HAL_ERROR;
                }
            }
            if(status != HAL_OK)
            {
                product_status = status;
                break;
            }
        }
        if(product_status == HAL_OK)
        {
            adbms6830_increment_u32_sat(&dev->health.filtered_read_count);
            if(dev->filtered_successful_epoch_count < UINT8_MAX)
            {
                dev->filtered_successful_epoch_count++;
            }
            /* Require two complete coherent filtered epochs after reset. At
             * the 10 Hz vehicle cadence this is >=100 ms, conservatively past
             * the 21 Hz filter's ~52 ms 0.1% settling time. */
            if((AMS_ADBMS_IIR_FC != 0u) &&
               (dev->filtered_successful_epoch_count >= 2u))
            {
                dev->filtered_voltage_ready = true;
            }
        }
        else
        {
            adbms6830_increment_u32_sat(&dev->health.filtered_read_fail_count);
            adbms6830_invalidate_filtered_product(dev);
            if(product_status == HAL_BUSY)
            {
                critical_error = product_status;
            }
        }
    }

    if(snapped)
    {
        /* If a session-age guard expired during a coherent snapshot, do not
         * silently re-wake and splice epochs. A best-effort wake+UNSNAP is
         * cleanup only; the caller discards/restarts the whole image. */
        dev->session.coherent_snapshot_active = false;
        if(critical_error == HAL_BUSY)
        {
            (void)adbms6830_wakeup_checked(dev);
        }
        status = adbms6830_cmd_checked(dev, UNSNAP);
        if((status != HAL_OK) && (critical_error == HAL_OK))
        {
            critical_error = status;
        }
    }

#if AMS_ENABLE_ADBMS_AWAKE_SESSION
    adbms6830_session_close(dev);
#endif
    return critical_error;
}

HAL_StatusTypeDef adbms6830_read_cell_voltage_products(adbms6830_driver_t *dev,
                                                       bool read_avg8,
                                                       bool read_filtered)
{
    HAL_StatusTypeDef status;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    for(uint8_t attempt = 0u; attempt < 2u; attempt++)
    {
        status = adbms6830_read_voltage_epoch(dev, read_avg8, read_filtered);
        if(status == HAL_OK)
        {
            adbms6830_mask_open_wire_cells(dev);
            return HAL_OK;
        }

        /* Retry exactly once for any coherent-epoch transport/integrity
         * failure. HAL_BUSY is the explicit session-age restart request; a
         * PEC/SPI/UNSNAP error can also be a transient consequence of the
         * unavoidable guard check/act preemption race. The second failure is
         * returned to the existing per-scan read-fault qualification. */
        if(attempt == 0u)
        {
            if(dev->session.coherent_restart_count != UINT32_MAX)
            {
                dev->session.coherent_restart_count++;
            }
            continue;
        }
        break;
    }

    if(dev->session.coherent_restart_fail_count != UINT32_MAX)
    {
        dev->session.coherent_restart_fail_count++;
    }
    adbms6830_mask_open_wire_cells(dev);
    return status;
}

HAL_StatusTypeDef adbms6830_read_cell_voltages(adbms6830_driver_t *dev)
{
    return adbms6830_read_cell_voltage_products(dev, false, false);
}


static void adbms6830_parse_scell(adbms6830_driver_t *dev,
                                  const uint8_t *data,
                                  GRP grp)
{
    uint8_t first_cell;
    uint16_t group_mask;

    if(!adbms6830_topology_valid(dev) || (data == NULL))
    {
        return;
    }

    first_cell = adbms6830_cell_group_first_index(grp);
    group_mask = adbms6830_cell_group_mask(grp);
    if((first_cell == UINT8_MAX) || (group_mask == 0u))
    {
        return;
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        const uint8_t *packet = &data[(uint16_t)ic * RX_DATA];
        bool pec_ok = adbms6830_read_packet_pec_ok(packet);
        bool counter_ok = adbms6830_read_packet_counter_ok(dev, ic);

        dev->ics[ic].cccrc.scell_pec = pec_ok ? 0u : 1u;
        if(!pec_ok)
        {
            dev->last_scell_pec_mask[ic] |= group_mask;
            continue;
        }
        if(!counter_ok)
        {
            continue;
        }

        for(uint8_t slot = 0u; slot < 3u; slot++)
        {
            uint8_t cell = (uint8_t)(first_cell + slot);
            uint8_t lo = (uint8_t)(slot * 2u);
            uint8_t hi = (uint8_t)(lo + 1u);
            bool code_valid;

            if(cell >= dev->monitored_cell_count)
            {
                continue;
            }

            code_valid = !(((packet[lo] == 0xFFu) && (packet[hi] == 0xFFu)) ||
                           ((packet[lo] == 0x00u) && (packet[hi] == 0x80u)));
            if(!code_valid)
            {
                continue;
            }

            dev->ics[ic].scell.sc_codes[cell] =
                (int16_t)((uint16_t)packet[lo] |
                          ((uint16_t)packet[hi] << 8u));
            dev->last_scell_updated_mask[ic] |= (uint16_t)(1u << cell);
        }
    }
}

HAL_StatusTypeDef adbms6830_capture_cs_comparison(adbms6830_driver_t *dev)
{
    static uint8_t *const c_commands[] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};
    static uint8_t *const s_commands[] = {RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF};
    static const GRP groups[] = {A, B, C, D, E, F};
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    uint16_t monitored_mask;
    uint8_t group_count;
    bool snapped = false;
    bool read_session_owned = false;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    monitored_mask = adbms6830_monitored_cell_mask(dev);
    if(monitored_mask == 0u)
    {
        return HAL_ERROR;
    }
    group_count = (uint8_t)((dev->monitored_cell_count + 2u) / 3u);
    if(group_count > (uint8_t)(sizeof(groups) / sizeof(groups[0])))
    {
        return HAL_ERROR;
    }

    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        dev->last_cell_updated_mask[ic] = 0u;
        dev->last_cell_pec_mask[ic] = 0u;
        dev->last_scell_updated_mask[ic] = 0u;
        dev->last_scell_pec_mask[ic] = 0u;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CS_COMPARE;
    }

    /* Recreate the exact startup comparison that is currently asserting
     * CSxFLT: clear the old latch, request a single redundant C/S conversion
     * with filter reset, and wait the documented worst-case completion time. */
    status = adbms6830_clear_all_flags(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_adcv_checked(dev,
                                        RD_ON,
                                        SINGLE,
                                        DCP_OFF,
                                        RSTF_ON,
                                        OW_OFF_ALL_CH);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_wait_cooperative(dev,
                                    ADBMS6830_REDUNDANT_CONVERSION_WAIT_US);
    }
    if(status != HAL_OK)
    {
        first_error = status;
    }

    /* Hold one coherent conversion image while both register banks and the
     * associated Status-C comparison latch are read. */
    if(first_error == HAL_OK)
    {
        status = adbms6830_read_session_begin(dev, &read_session_owned);
        if(status == HAL_OK)
        {
            status = adbms6830_cmd_checked(dev, SNAP);
        }
        if(status == HAL_OK)
        {
            dev->session.coherent_snapshot_active = true;
            status = adbms6830_us_delay(dev, 10u);
        }
        if(status == HAL_OK)
        {
            snapped = true;
        }
        else
        {
            first_error = status;
        }
    }

    if(snapped)
    {
        for(uint8_t group = 0u; group < group_count; group++)
        {
            status = adbms6830_rd48_checked(dev, c_commands[group], shared_buf);
            if(status == HAL_OK)
            {
                adbms6830_parse_cell(dev, shared_buf, groups[group]);
                if(!adbms6830_last_read_integrity_ok(dev))
                {
                    status = HAL_ERROR;
                }
            }
            if((status != HAL_OK) && (first_error == HAL_OK))
            {
                first_error = status;
            }
        }

        for(uint8_t group = 0u; group < group_count; group++)
        {
            status = adbms6830_rd48_checked(dev, s_commands[group], shared_buf);
            if(status == HAL_OK)
            {
                adbms6830_parse_scell(dev, shared_buf, groups[group]);
                if(!adbms6830_last_read_integrity_ok(dev))
                {
                    status = HAL_ERROR;
                }
            }
            if((status != HAL_OK) && (first_error == HAL_OK))
            {
                first_error = status;
            }
        }

        /* Read Status C even when one register group failed so the command
         * still returns the freshest CSxFLT image alongside partial data. */
        status = adbms6830_rd48_checked(dev, RDSTATC, shared_buf);
        if(status == HAL_OK)
        {
            adbms6830_parse_statc(dev, shared_buf);
            if(!adbms6830_last_read_integrity_ok(dev))
            {
                status = HAL_ERROR;
            }
        }
        if((status != HAL_OK) && (first_error == HAL_OK))
        {
            first_error = status;
        }
    }

    /* Always release SNAP after it was accepted. If the guard expired during
     * the diagnostic epoch, coherence is already lost, so a re-wake here is
     * cleanup only and never licenses the partial image. */
    if(snapped)
    {
        dev->session.coherent_snapshot_active = false;
        status = adbms6830_session_require_awake(dev);
        if(status == HAL_OK)
        {
            status = adbms6830_cmd_checked(dev, UNSNAP);
        }
        if((status != HAL_OK) && (first_error == HAL_OK))
        {
            first_error = status;
        }
    }
    adbms6830_read_session_end(dev, read_session_owned);

    if(first_error == HAL_OK)
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            if(((dev->last_cell_updated_mask[ic] & monitored_mask) != monitored_mask) ||
               ((dev->last_scell_updated_mask[ic] & monitored_mask) != monitored_mask) ||
               !dev->diag[ic].statc_valid)
            {
                first_error = HAL_ERROR;
                break;
            }
        }
    }

    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_CS_COMPARE, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CS_COMPARE;
        dev->spi_debug.last_status = first_error;
    }
    return first_error;
}

HAL_StatusTypeDef adbms6830_capture_s_adc(adbms6830_driver_t *dev)
{
    static uint8_t *const s_commands[ADBMS6830_SADC_GROUP_COUNT] =
    {
        RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF
    };
    static const GRP groups[ADBMS6830_SADC_GROUP_COUNT] =
    {
        A, B, C, D, E, F
    };
    uint8_t adsv_cmd[2];
    adbms6830_sadc_debug_t *dbg;
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    uint16_t monitored_mask;
    uint8_t group_count;
    bool read_session_owned = false;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    monitored_mask = adbms6830_monitored_cell_mask(dev);
    if(monitored_mask == 0u)
    {
        return HAL_ERROR;
    }

    group_count = (uint8_t)((dev->monitored_cell_count + 2u) / 3u);
    if((group_count == 0u) || (group_count > ADBMS6830_SADC_GROUP_COUNT))
    {
        return HAL_ERROR;
    }

    dbg = &dev->sadc_debug;
    memset(dbg, 0, sizeof(*dbg));
    dbg->valid = true;
    dbg->group_count = group_count;
    dbg->expected_ic_mask = adbms6830_expected_ic_mask(dev);
    dbg->wake_status = HAL_ERROR;
    dbg->command_status = HAL_ERROR;
    dbg->delay_status = HAL_ERROR;
    dbg->overall_status = HAL_ERROR;
    for(uint8_t group = 0u; group < ADBMS6830_SADC_GROUP_COUNT; group++)
    {
        dbg->group_read_status[group] = HAL_ERROR;
    }

    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        dev->last_scell_updated_mask[ic] = 0u;
        dev->last_scell_pec_mask[ic] = 0u;
    }

    /* Standalone ADSV, single conversion, discharge disabled, open-wire
     * switches disabled. This is the same proven command encoding used by the
     * open-wire baseline path, without enabling any diagnostic switch. */
    adsv_cmd[0] = 0x01u;
    adsv_cmd[1] = ((uint8_t)SINGLE << 7u) |
                  ((uint8_t)DCP_OFF << 4u) |
                  ((uint8_t)OW_OFF_ALL_CH & 0x03u) |
                  0x68u;

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_S_ADC_DUMP;
    }

    dbg->wake_status = adbms6830_wakeup_checked(dev);
    if(dbg->wake_status == HAL_OK)
    {
        dbg->command_status = adbms6830_cmd_checked(dev, adsv_cmd);
    }
    if(dbg->command_status == HAL_OK)
    {
        dbg->delay_status = adbms6830_wait_cooperative(
            dev,
            ADBMS6830_OPEN_WIRE_CONVERSION_WAIT_US);
    }

    if(dbg->wake_status != HAL_OK)
    {
        first_error = dbg->wake_status;
    }
    else if(dbg->command_status != HAL_OK)
    {
        first_error = dbg->command_status;
    }
    else if(dbg->delay_status != HAL_OK)
    {
        first_error = dbg->delay_status;
    }

    if(first_error == HAL_OK)
    {
        status = adbms6830_read_session_begin(dev, &read_session_owned);
        if(status != HAL_OK)
        {
            first_error = status;
        }
    }

    if(first_error == HAL_OK)
    {
        for(uint8_t group = 0u; group < group_count; group++)
        {
            status = adbms6830_rd48_checked(dev, s_commands[group], shared_buf);
            dbg->group_read_status[group] = status;

            if(status != HAL_OK)
            {
                if(first_error == HAL_OK)
                {
                    first_error = status;
                }
                continue;
            }

            for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
            {
                memcpy(dbg->packet[ic][group],
                       &shared_buf[(uint16_t)ic * RX_DATA],
                       RX_DATA);
            }

            dbg->pec_pass_mask[group] = dev->health.last_pec_pass_mask;
            dbg->pec_fail_mask[group] = dev->health.last_pec_fail_mask;
            dbg->counter_mismatch_mask[group] =
                dev->health.last_cmd_counter_mismatch_mask;
            dbg->transport_valid_mask[group] =
                (uint16_t)(dbg->pec_pass_mask[group] &
                           (uint16_t)~dbg->counter_mismatch_mask[group] &
                           dbg->expected_ic_mask);

            adbms6830_parse_scell(dev, shared_buf, groups[group]);
            if(!adbms6830_last_read_integrity_ok(dev) && (first_error == HAL_OK))
            {
                first_error = HAL_ERROR;
            }
        }
    }

    adbms6830_read_session_end(dev, read_session_owned);

    if(first_error == HAL_OK)
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            if((dev->last_scell_updated_mask[ic] & monitored_mask) != monitored_mask)
            {
                first_error = HAL_ERROR;
                break;
            }
        }
    }

    dbg->overall_status = first_error;
    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_S_ADC_DUMP, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_S_ADC_DUMP;
        dev->spi_debug.last_status = first_error;
    }

    return first_error;
}


HAL_StatusTypeDef adbms6830_capture_c_adc(adbms6830_driver_t *dev)
{
    static uint8_t *const c_commands[ADBMS6830_CADC_GROUP_COUNT] =
    {
        RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF
    };
    static const GRP groups[ADBMS6830_CADC_GROUP_COUNT] =
    {
        A, B, C, D, E, F
    };
    uint8_t adcv_cmd[2];
    adbms6830_cadc_debug_t *dbg;
    HAL_StatusTypeDef first_error = HAL_OK;
    bool observed_busy = false;
    HAL_StatusTypeDef status;
    uint16_t monitored_mask;
    uint8_t group_count;
    bool read_session_owned = false;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    monitored_mask = adbms6830_monitored_cell_mask(dev);
    if(monitored_mask == 0u)
    {
        return HAL_ERROR;
    }

    group_count = (uint8_t)((dev->monitored_cell_count + 2u) / 3u);
    if((group_count == 0u) || (group_count > ADBMS6830_CADC_GROUP_COUNT))
    {
        return HAL_ERROR;
    }

    dbg = &dev->cadc_debug;
    memset(dbg, 0, sizeof(*dbg));
    dbg->valid = true;
    dbg->group_count = group_count;
    dbg->expected_ic_mask = adbms6830_expected_ic_mask(dev);
    dbg->wake_status = HAL_ERROR;
    dbg->command_status = HAL_ERROR;
    dbg->poll_status = HAL_ERROR;
    dbg->overall_status = HAL_ERROR;
    for(uint8_t group = 0u; group < ADBMS6830_CADC_GROUP_COUNT; group++)
    {
        dbg->group_read_status[group] = HAL_ERROR;
    }

    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        dev->last_cell_updated_mask[ic] = 0u;
        dev->last_cell_pec_mask[ic] = 0u;
    }

    /* C-ADC only: RD=0, single-shot, discharge disabled, filter not reset,
     * open-wire switches disabled. */
    adcv_cmd[0] = 0x02u | (uint8_t)RD_OFF;
    adcv_cmd[1] = ((uint8_t)SINGLE << 7u) |
                  ((uint8_t)DCP_OFF << 4u) |
                  ((uint8_t)RSTF_OFF << 2u) |
                  ((uint8_t)OW_OFF_ALL_CH & 0x03u) |
                  0x60u;

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_C_ADC_DUMP;
    }

    dbg->wake_status = adbms6830_wakeup_checked(dev);
    if(dbg->wake_status == HAL_OK)
    {
        dbg->command_status = adbms6830_cmd_checked(dev, adcv_cmd);
    }
    if(dbg->command_status == HAL_OK)
    {
        dbg->poll_status = adbms6830_poll_conversion_checked(
            dev,
            PLCADC,
            ADBMS6830_CONVERSION_POLL_TIMEOUT_US,
            &dbg->conversion_time_us,
            &dbg->poll_clock_bytes,
            &observed_busy,
            &dbg->poll_complete);
    }

    if(dbg->wake_status != HAL_OK)
    {
        first_error = dbg->wake_status;
    }
    else if(dbg->command_status != HAL_OK)
    {
        first_error = dbg->command_status;
    }
    else if(dbg->poll_status != HAL_OK)
    {
        first_error = dbg->poll_status;
    }

    if(first_error == HAL_OK)
    {
        status = adbms6830_read_session_begin(dev, &read_session_owned);
        if(status != HAL_OK)
        {
            first_error = status;
        }
    }

    if(first_error == HAL_OK)
    {
        for(uint8_t group = 0u; group < group_count; group++)
        {
            status = adbms6830_rd48_checked(dev, c_commands[group], shared_buf);
            dbg->group_read_status[group] = status;

            if(status != HAL_OK)
            {
                if(first_error == HAL_OK)
                {
                    first_error = status;
                }
                continue;
            }

            for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
            {
                memcpy(dbg->packet[ic][group],
                       &shared_buf[(uint16_t)ic * RX_DATA],
                       RX_DATA);
            }

            dbg->pec_pass_mask[group] = dev->health.last_pec_pass_mask;
            dbg->pec_fail_mask[group] = dev->health.last_pec_fail_mask;
            dbg->counter_mismatch_mask[group] =
                dev->health.last_cmd_counter_mismatch_mask;
            dbg->transport_valid_mask[group] =
                (uint16_t)(dbg->pec_pass_mask[group] &
                           (uint16_t)~dbg->counter_mismatch_mask[group] &
                           dbg->expected_ic_mask);

            adbms6830_parse_cell(dev, shared_buf, groups[group]);
            if(!adbms6830_last_read_integrity_ok(dev) && (first_error == HAL_OK))
            {
                first_error = HAL_ERROR;
            }
        }
    }

    adbms6830_read_session_end(dev, read_session_owned);

    if(first_error == HAL_OK)
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            if((dev->last_cell_updated_mask[ic] & monitored_mask) != monitored_mask)
            {
                first_error = HAL_ERROR;
                break;
            }
        }
    }

    dbg->overall_status = first_error;
    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_C_ADC_DUMP, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_C_ADC_DUMP;
        dev->spi_debug.last_status = first_error;
    }
    return first_error;
}

const char *adbms6830_timing_kind_name(adbms6830_timing_kind_t kind)
{
    switch(kind)
    {
    case ADBMS6830_TIMING_C_ADC:   return "C-ADC";
    case ADBMS6830_TIMING_S_ADC:   return "S-ADC";
    case ADBMS6830_TIMING_AUX_ADC: return "AUX";
    default:                       return "unknown";
    }
}

HAL_StatusTypeDef adbms6830_profile_conversion_timing(
    adbms6830_driver_t *dev,
    adbms6830_timing_kind_t kind,
    adbms6830_timing_result_t *result)
{
    uint8_t command[2] = {0u, 0u};
    const uint8_t *poll_command = NULL;
    HAL_StatusTypeDef first_error = HAL_OK;

    if(!adbms6830_topology_valid(dev) || (result == NULL))
    {
        return HAL_ERROR;
    }

    memset(result, 0, sizeof(*result));
    result->kind = kind;
    result->wake_status = HAL_ERROR;
    result->command_status = HAL_ERROR;
    result->poll_status = HAL_ERROR;
    result->overall_status = HAL_ERROR;

    switch(kind)
    {
    case ADBMS6830_TIMING_C_ADC:
        command[0] = 0x02u | (uint8_t)RD_OFF;
        command[1] = ((uint8_t)SINGLE << 7u) |
                     ((uint8_t)DCP_OFF << 4u) |
                     ((uint8_t)RSTF_OFF << 2u) |
                     ((uint8_t)OW_OFF_ALL_CH & 0x03u) |
                     0x60u;
        poll_command = PLCADC;
        break;

    case ADBMS6830_TIMING_S_ADC:
        command[0] = 0x01u;
        command[1] = ((uint8_t)SINGLE << 7u) |
                     ((uint8_t)DCP_OFF << 4u) |
                     ((uint8_t)OW_OFF_ALL_CH & 0x03u) |
                     0x68u;
        poll_command = PLSADC;
        break;

    case ADBMS6830_TIMING_AUX_ADC:
        command[0] = ADAX_CMD_BYTE0;
        command[1] = ADAX_CMD_BYTE1;
        poll_command = PLAUX1;
        break;

    default:
        return HAL_ERROR;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CONVERSION_TIMING;
    }

    result->wake_status = adbms6830_wakeup_checked(dev);
    if(result->wake_status == HAL_OK)
    {
        result->command_status = adbms6830_cmd_checked(dev, command);
    }
    if(result->command_status == HAL_OK)
    {
        result->poll_status = adbms6830_poll_conversion_checked(
            dev,
            poll_command,
            ADBMS6830_CONVERSION_POLL_TIMEOUT_US,
            &result->elapsed_us,
            &result->poll_clock_bytes,
            &result->observed_busy,
            &result->complete);
    }

    if(result->wake_status != HAL_OK)
    {
        first_error = result->wake_status;
    }
    else if(result->command_status != HAL_OK)
    {
        first_error = result->command_status;
    }
    else if(result->poll_status != HAL_OK)
    {
        first_error = result->poll_status;
    }

    result->overall_status = first_error;
    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_CONVERSION_TIMING, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CONVERSION_TIMING;
        dev->spi_debug.last_status = first_error;
    }
    return first_error;
}

HAL_StatusTypeDef adbms6830_config_write_readback_cycle(
    adbms6830_driver_t *dev,
    adbms6830_config_cycle_result_t *result)
{
    HAL_StatusTypeDef first_error = HAL_OK;
    const adbms6830_diag_health_t *health;

    if(!adbms6830_topology_valid(dev) || (result == NULL))
    {
        return HAL_ERROR;
    }

    memset(result, 0, sizeof(*result));
    result->write_cfga_status = HAL_ERROR;
    result->write_cfgb_status = HAL_ERROR;
    result->readback_status = HAL_ERROR;
    result->overall_status = HAL_ERROR;

    /* Never turn a configuration stress test into a balancing command. */
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        bool discharge_nonzero = (dev->ics[ic].tx_cfgb.dcc != 0u);
        for(uint8_t cell = 0u; cell < PWMA; cell++)
        {
            discharge_nonzero = discharge_nonzero ||
                                (dev->ics[ic].PwmA.pwma[cell] != 0u);
        }
        for(uint8_t cell = 0u; cell < PWMB; cell++)
        {
            discharge_nonzero = discharge_nonzero ||
                                (dev->ics[ic].PwmB.pwmb[cell] != 0u);
        }
        if(discharge_nonzero)
        {
            result->discharge_nonzero_mask |= (uint16_t)(1u << ic);
        }
    }

    if(result->discharge_nonzero_mask != 0u)
    {
        return HAL_ERROR;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CONFIG_STRESS;
    }

    result->write_cfga_status = adbms6830_wrcfga_checked(dev);
    if(result->write_cfga_status != HAL_OK)
    {
        first_error = result->write_cfga_status;
    }

    result->write_cfgb_status = adbms6830_wrcfgb_checked_reason(
        dev, ADBMS6830_CFGB_WRITE_CONFIG_STRESS);
    if((result->write_cfgb_status != HAL_OK) && (first_error == HAL_OK))
    {
        first_error = result->write_cfgb_status;
    }

    result->readback_status = adbms6830_verify_config_readback(dev);
    if((result->readback_status != HAL_OK) && (first_error == HAL_OK))
    {
        first_error = result->readback_status;
    }

    health = adbms6830_diag_health_get(dev);
    if(health != NULL)
    {
        result->cfga_mismatch_mask = health->configa_mismatch_mask;
        result->cfgb_mismatch_mask = health->configb_mismatch_mask;
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        if(dev->ics[ic].rx_cfgb.dcc != 0u)
        {
            result->discharge_nonzero_mask |= (uint16_t)(1u << ic);
        }
    }
    if((result->discharge_nonzero_mask != 0u) && (first_error == HAL_OK))
    {
        first_error = HAL_ERROR;
    }

    result->overall_status = first_error;
    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_CONFIG_STRESS, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CONFIG_STRESS;
        dev->spi_debug.last_status = first_error;
    }
    return first_error;
}

HAL_StatusTypeDef adbms6830_recovery_check(adbms6830_driver_t *dev)
{
    adbms6830_recovery_debug_t *dbg;
    adbms6830_config_cycle_result_t config_result;
    const adbms6830_diag_health_t *health;
    HAL_StatusTypeDef first_error = HAL_OK;
    uint16_t monitored_mask;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    dbg = &dev->recovery_debug;
    memset(dbg, 0, sizeof(*dbg));
    dbg->wake_status = HAL_ERROR;
    dbg->sid_status = HAL_ERROR;
    dbg->write_cfga_status = HAL_ERROR;
    dbg->write_cfgb_status = HAL_ERROR;
    dbg->config_status = HAL_ERROR;
    dbg->diagnostic_status = HAL_ERROR;
    dbg->cadc_status = HAL_ERROR;
    dbg->overall_status = HAL_ERROR;

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_RECOVERY;
    }

    dbg->wake_status = adbms6830_wakeup_checked(dev);
    if(dbg->wake_status != HAL_OK)
    {
        first_error = dbg->wake_status;
    }

    dbg->sid_status = adbms6830_read_sid(dev);
    if((dbg->sid_status != HAL_OK) && (first_error == HAL_OK))
    {
        first_error = dbg->sid_status;
    }

    memset(&config_result, 0, sizeof(config_result));
    dbg->config_status = adbms6830_config_write_readback_cycle(dev, &config_result);
    dbg->write_cfga_status = config_result.write_cfga_status;
    dbg->write_cfgb_status = config_result.write_cfgb_status;
    if((dbg->config_status != HAL_OK) && (first_error == HAL_OK))
    {
        first_error = dbg->config_status;
    }

    dbg->diagnostic_status = adbms6830_refresh_diagnostics(dev);
    if((dbg->diagnostic_status != HAL_OK) && (first_error == HAL_OK))
    {
        first_error = dbg->diagnostic_status;
    }

    dbg->cadc_status = adbms6830_capture_c_adc(dev);
    if((dbg->cadc_status != HAL_OK) && (first_error == HAL_OK))
    {
        first_error = dbg->cadc_status;
    }

    health = adbms6830_diag_health_get(dev);
    if(health != NULL)
    {
        dbg->sid_valid_mask = health->sid_valid_ic_mask;
        dbg->config_mismatch_mask = health->config_mismatch_mask;
        dbg->reference_fault_mask =
            (uint16_t)(health->reference_invalid_ic_mask |
                       health->reference_fault_ic_mask);
        dbg->status_fault_mask =
            (uint16_t)(health->status_invalid_ic_mask |
                       health->status_fault_ic_mask);
    }

    monitored_mask = adbms6830_monitored_cell_mask(dev);
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        if((dev->last_cell_updated_mask[ic] & monitored_mask) == monitored_mask)
        {
            dbg->cadc_valid_ic_mask |= (uint16_t)(1u << ic);
        }
    }

    dbg->overall_status = first_error;
    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_RECOVERY, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_RECOVERY;
        dev->spi_debug.last_status = first_error;
    }
    return first_error;
}

static uint32_t adbms6830_fnv1a_byte(uint32_t hash, uint8_t value)
{
    hash ^= value;
    hash *= 16777619u;
    return hash;
}

static uint32_t adbms6830_config_fingerprint(const adbms6830_driver_t *dev,
                                             bool readback)
{
    uint32_t hash = 2166136261u;

    if(!adbms6830_topology_valid(dev))
    {
        return 0u;
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        const uint8_t *cfga = readback ? dev->ics[ic].configa.rx_data :
                                         dev->ics[ic].configa.tx_data;
        const uint8_t *cfgb = readback ? dev->ics[ic].configb.rx_data :
                                         dev->ics[ic].configb.tx_data;

        hash = adbms6830_fnv1a_byte(hash, ic);
        for(uint8_t byte = 0u; byte < TX_DATA; byte++)
        {
            hash = adbms6830_fnv1a_byte(hash, cfga[byte]);
        }
        for(uint8_t byte = 0u; byte < TX_DATA; byte++)
        {
            hash = adbms6830_fnv1a_byte(hash, cfgb[byte]);
        }
    }

    return hash;
}

uint32_t adbms6830_config_expected_fingerprint(const adbms6830_driver_t *dev)
{
    return adbms6830_config_fingerprint(dev, false);
}

uint32_t adbms6830_config_readback_fingerprint(const adbms6830_driver_t *dev)
{
    return adbms6830_config_fingerprint(dev, true);
}

static uint8_t *adbms6830_raw_command(adbms6830_raw_register_t reg)
{
    switch(reg)
    {
    case ADBMS6830_RAW_CFGA:  return RDCFGA;
    case ADBMS6830_RAW_CFGB:  return RDCFGB;
    case ADBMS6830_RAW_CVA:   return RDCVA;
    case ADBMS6830_RAW_CVB:   return RDCVB;
    case ADBMS6830_RAW_CVC:   return RDCVC;
    case ADBMS6830_RAW_CVD:   return RDCVD;
    case ADBMS6830_RAW_CVE:   return RDCVE;
    case ADBMS6830_RAW_CVF:   return RDCVF;
    case ADBMS6830_RAW_SVA:   return RDSVA;
    case ADBMS6830_RAW_SVB:   return RDSVB;
    case ADBMS6830_RAW_SVC:   return RDSVC;
    case ADBMS6830_RAW_SVD:   return RDSVD;
    case ADBMS6830_RAW_SVE:   return RDSVE;
    case ADBMS6830_RAW_SVF:   return RDSVF;
    case ADBMS6830_RAW_AUXA:  return RDAUXA;
    case ADBMS6830_RAW_AUXB:  return RDAUXB;
    case ADBMS6830_RAW_AUXC:  return RDAUXC;
    case ADBMS6830_RAW_AUXD:  return RDAUXD;
    case ADBMS6830_RAW_STATA: return RDSTATA;
    case ADBMS6830_RAW_STATB: return RDSTATB;
    case ADBMS6830_RAW_STATC: return RDSTATC;
    case ADBMS6830_RAW_STATD: return RDSTATD;
    case ADBMS6830_RAW_STATE: return RDSTATE;
    case ADBMS6830_RAW_COMM:  return RDCOMM;
    case ADBMS6830_RAW_PWMA:  return RDPWM1;
    case ADBMS6830_RAW_PWMB:  return RDPWM2;
    default:                  return NULL;
    }
}

const char *adbms6830_raw_register_name(adbms6830_raw_register_t reg)
{
    switch(reg)
    {
    case ADBMS6830_RAW_CFGA:  return "RDCFGA";
    case ADBMS6830_RAW_CFGB:  return "RDCFGB";
    case ADBMS6830_RAW_CVA:   return "RDCVA";
    case ADBMS6830_RAW_CVB:   return "RDCVB";
    case ADBMS6830_RAW_CVC:   return "RDCVC";
    case ADBMS6830_RAW_CVD:   return "RDCVD";
    case ADBMS6830_RAW_CVE:   return "RDCVE";
    case ADBMS6830_RAW_CVF:   return "RDCVF";
    case ADBMS6830_RAW_SVA:   return "RDSVA";
    case ADBMS6830_RAW_SVB:   return "RDSVB";
    case ADBMS6830_RAW_SVC:   return "RDSVC";
    case ADBMS6830_RAW_SVD:   return "RDSVD";
    case ADBMS6830_RAW_SVE:   return "RDSVE";
    case ADBMS6830_RAW_SVF:   return "RDSVF";
    case ADBMS6830_RAW_AUXA:  return "RDAUXA";
    case ADBMS6830_RAW_AUXB:  return "RDAUXB";
    case ADBMS6830_RAW_AUXC:  return "RDAUXC";
    case ADBMS6830_RAW_AUXD:  return "RDAUXD";
    case ADBMS6830_RAW_STATA: return "RDSTATA";
    case ADBMS6830_RAW_STATB: return "RDSTATB";
    case ADBMS6830_RAW_STATC: return "RDSTATC";
    case ADBMS6830_RAW_STATD: return "RDSTATD";
    case ADBMS6830_RAW_STATE: return "RDSTATE";
    case ADBMS6830_RAW_COMM:  return "RDCOMM";
    case ADBMS6830_RAW_PWMA:  return "RDPWMA";
    case ADBMS6830_RAW_PWMB:  return "RDPWMB";
    default:                  return "UNKNOWN";
    }
}

HAL_StatusTypeDef adbms6830_read_raw_register(adbms6830_driver_t *dev,
                                              adbms6830_raw_register_t reg,
                                              adbms6830_raw_read_t *result)
{
    uint8_t *command;
    HAL_StatusTypeDef status;

    if(!adbms6830_topology_valid(dev) || (result == NULL))
    {
        return HAL_ERROR;
    }

    command = adbms6830_raw_command(reg);
    if(command == NULL)
    {
        return HAL_ERROR;
    }

    memset(result, 0, sizeof(*result));
    result->reg = reg;
    result->status = HAL_ERROR;

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_RAW_DUMP;
    }

    status = adbms6830_rd48_checked(dev, command, shared_buf);
    result->status = status;
    if(status == HAL_OK)
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            memcpy(result->packet[ic],
                   &shared_buf[(uint16_t)ic * RX_DATA],
                   RX_DATA);
        }
        result->pec_pass_mask = dev->health.last_pec_pass_mask;
        result->pec_fail_mask = dev->health.last_pec_fail_mask;
        result->counter_mismatch_mask =
            dev->health.last_cmd_counter_mismatch_mask;
    }

    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_RAW_DUMP, status);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_RAW_DUMP;
        dev->spi_debug.last_status = status;
    }
    return status;
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

static void adbms6830_temp_debug_prepare(adbms6830_driver_t *dev,
                                         uint8_t sensor_num,
                                         bool force_aux_capture)
{
    adbms6830_temp_debug_t *dbg;
    adbms6830_temp_route_t route;

    if((dev == NULL) ||
       !adbms6830_temp_sensor_route(sensor_num, &route))
    {
        return;
    }

    dbg = &dev->temp_debug;
    memset(dbg, 0, sizeof(*dbg));

    dbg->valid = true;
    dbg->forced_aux_capture = force_aux_capture;
    dbg->sensor_num = route.sensor_num;
    dbg->mux_idx = route.mux_idx;
    dbg->switch_index = route.switch_index;
    dbg->switch_mask = route.switch_mask;
    dbg->mux_address = route.mux_address;
    dbg->gpio_channel = route.gpio_channel;
    dbg->expected_ic_mask = adbms6830_expected_ic_mask(dev);

    dbg->select_status = HAL_ERROR;
    dbg->wrc_status = HAL_ERROR;
    dbg->pre_rdcomm_status = HAL_ERROR;
    dbg->stcomm_status = HAL_ERROR;
    dbg->rdcomm_status = HAL_ERROR;
    dbg->wake_status = HAL_ERROR;
    dbg->adax_status = HAL_ERROR;
    dbg->rdaux_status = HAL_ERROR;
    dbg->overall_status = HAL_ERROR;
}


static void adbms6830_temp_debug_prepare_probe(adbms6830_driver_t *dev,
                                                uint8_t slave_addr,
                                                uint8_t data_byte)
{
    adbms6830_temp_debug_t *dbg;

    if(dev == NULL)
    {
        return;
    }

    dbg = &dev->temp_debug;
    memset(dbg, 0, sizeof(*dbg));

    dbg->valid = true;
    dbg->sensor_num = UINT8_MAX;
    dbg->mux_idx = UINT8_MAX;
    dbg->mux_address = slave_addr;
    dbg->switch_index = UINT8_MAX;
    dbg->switch_mask = data_byte;
    dbg->gpio_channel = UINT8_MAX;
    dbg->expected_ic_mask = adbms6830_expected_ic_mask(dev);

    dbg->select_status = HAL_ERROR;
    dbg->wrc_status = HAL_ERROR;
    dbg->pre_rdcomm_status = HAL_ERROR;
    dbg->stcomm_status = HAL_ERROR;
    dbg->rdcomm_status = HAL_ERROR;
    dbg->wake_status = HAL_ERROR;
    dbg->adax_status = HAL_ERROR;
    dbg->rdaux_status = HAL_ERROR;
    dbg->overall_status = HAL_ERROR;
}

static void adbms6830_invalidate_mux_selections(adbms6830_driver_t *dev)
{
    if(dev == NULL)
    {
        return;
    }

    for(uint8_t mux = 0u; mux < ADBMS6830_MUX_COUNT; mux++)
    {
        dev->mux_selection_valid_mask[mux] = 0u;
    }

    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        for(uint8_t mux = 0u; mux < ADBMS6830_MUX_COUNT; mux++)
        {
            dev->mux_selected_channel[ic][mux] = UINT8_MAX;
        }
    }
}

/* ---------------------------------------------------------------------------
 * adbms6830_gpio_i2c_write
 *
 * Issue a one-byte I2C write to a slave address via the ADBMS6830 COMM
 * register on every IC in the daisy-chain.
 * ------------------------------------------------------------------------- */
static HAL_StatusTypeDef adbms6830_gpio_i2c_write(adbms6830_driver_t *dev,
                                                   uint8_t slave_addr,
                                                   uint8_t data_byte,
                                                   uint16_t *ack_mask)
{
    HAL_StatusTypeDef status;
    uint16_t acknowledged = 0u;
    uint16_t expected_mask;

    if(ack_mask != NULL)
    {
        *ack_mask = 0u;
    }
    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    expected_mask = adbms6830_expected_ic_mask(dev);

    /* Pack COMM register: 3 slots.  FCOMM=0x8 releases SDA on each
     * ninth clock so the ADG728 can generate the address/data ACK.          *
     * Slot 0: START + address byte (write)                                 *
     * Slot 1: data byte                                                    *
     * Slot 2: STOP (data don't-care = 0xFF)                                */
    for(uint8_t curr_ic = 0u; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
    {
        dev->ics[curr_ic].comm.icomm[0] = ICOMM_START_;
        dev->ics[curr_ic].comm.fcomm[0] = FCOMM_RELEASE_FOR_SLAVE_ACK_;
        dev->ics[curr_ic].comm.data[0]  = (uint8_t)(slave_addr << 1u);

        dev->ics[curr_ic].comm.icomm[1] = ICOMM_BLANK_;
        dev->ics[curr_ic].comm.fcomm[1] = FCOMM_RELEASE_FOR_SLAVE_ACK_;
        dev->ics[curr_ic].comm.data[1]  = data_byte;

        dev->ics[curr_ic].comm.icomm[2] = ICOMM_STOP_;
        dev->ics[curr_ic].comm.fcomm[2] = FCOMM_NACK_STOP_;
        dev->ics[curr_ic].comm.data[2]  = 0xFFu;
    }

    adbms6830_pack_comm(dev);

    for(uint8_t curr_ic = 0u; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
    {
        for(uint8_t b = 0u; b < TX_DATA; b++)
        {
            uint8_t value = dev->ics[curr_ic].com.tx_data[b];
            shared_buf[((uint16_t)curr_ic * TX_DATA) + b] = value;
            dev->temp_debug.wrcomm_payload[curr_ic][b] = value;
        }
    }

    /* Write the COMM register, then read it back before STCOMM.  This is a
     * fail-closed service diagnostic: malformed data, PEC failure, counter
     * mismatch, or byte mismatch prevents any GPIO/I2C waveform generation. */
    status = adbms6830_wr48_checked(dev, WRCOMM, shared_buf);
    dev->temp_debug.wrc_status = status;
    if(status != HAL_OK)
    {
        dev->temp_debug.overall_status = status;
        return status;
    }

#if AMS_HW_BRINGUP
    status = adbms6830_rd48_checked(dev, RDCOMM, shared_buf);
    dev->temp_debug.pre_rdcomm_status = status;
    dev->temp_debug.pre_comm_pec_pass_mask = dev->health.last_pec_pass_mask;
    dev->temp_debug.pre_comm_pec_fail_mask = dev->health.last_pec_fail_mask;
    dev->temp_debug.pre_comm_counter_mismatch_mask =
        dev->health.last_cmd_counter_mismatch_mask;

    for(uint8_t curr_ic = 0u; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
    {
        uint16_t bit = (uint16_t)(1u << curr_ic);
        const uint8_t *packet = &shared_buf[(uint16_t)curr_ic * RX_DATA];
        bool transport_valid =
            ((dev->temp_debug.pre_comm_pec_pass_mask & bit) != 0u) &&
            ((dev->temp_debug.pre_comm_counter_mismatch_mask & bit) == 0u);

        memcpy(dev->temp_debug.pre_rdcomm_packet[curr_ic], packet, RX_DATA);
        if(transport_valid &&
           (memcmp(packet,
                   dev->temp_debug.wrcomm_payload[curr_ic],
                   TX_DATA) == 0))
        {
            dev->temp_debug.pre_comm_match_mask |= bit;
        }
    }

    if((status != HAL_OK) ||
       (dev->temp_debug.pre_comm_match_mask != expected_mask))
    {
        dev->temp_debug.overall_status =
            (status != HAL_OK) ? status : HAL_ERROR;
        return dev->temp_debug.overall_status;
    }
#else
    /* Preserve the established production timing. The extra WRCOMM readback
     * is a controlled-bench diagnostic only. */
    dev->temp_debug.pre_rdcomm_status = HAL_OK;
    dev->temp_debug.pre_comm_match_mask = expected_mask;
#endif

    /* Exactly one explicit STCOMM transaction is issued for a tempsns CLI
     * request. Automatic mux scans are separately disabled in the bench
     * profile, so this path cannot retry in the background. */
    dev->temp_debug.stcomm_attempted = true;
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_STCOMM;
    }
    status = adbms6830_spi_write(dev, STCOMM, sizeof(STCOMM), 1u);
    dev->temp_debug.stcomm_status = status;
    if(status != HAL_OK)
    {
        dev->temp_debug.overall_status = status;
        return status;
    }
    adbms6830_note_counter_increment(dev);

    /* Read the post-transaction COMM image.  Keep PEC and command-counter
     * failures separate so a valid all-zero packet is not misreported as a
     * generic transport failure. */
    status = adbms6830_rd48_checked(dev, RDCOMM, shared_buf);
    dev->temp_debug.rdcomm_status = status;
    dev->temp_debug.comm_pec_pass_mask = dev->health.last_pec_pass_mask;
    dev->temp_debug.comm_pec_fail_mask = dev->health.last_pec_fail_mask;
    dev->temp_debug.comm_counter_mismatch_mask =
        dev->health.last_cmd_counter_mismatch_mask;
    if(status != HAL_OK)
    {
        dev->temp_debug.overall_status = status;
        return status;
    }

    for(uint8_t curr_ic = 0u; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
    {
        uint16_t bit = (uint16_t)(1u << curr_ic);
        const uint8_t *packet = &shared_buf[(uint16_t)curr_ic * RX_DATA];
        bool transport_valid =
            ((dev->temp_debug.comm_pec_pass_mask & bit) != 0u) &&
            ((dev->temp_debug.comm_counter_mismatch_mask & bit) == 0u);
        bool address_ack =
            adbms6830_comm_address_acknowledged(packet);
        bool data_ack =
            adbms6830_comm_data_acknowledged(packet);

        memcpy(dev->temp_debug.rdcomm_packet[curr_ic], packet, RX_DATA);
        if(transport_valid)
        {
            dev->temp_debug.comm_transport_valid_mask |= bit;
        }
        if(address_ack)
        {
            dev->temp_debug.address_ack_mask |= bit;
        }
        if(data_ack)
        {
            dev->temp_debug.data_ack_mask |= bit;
        }

        if(transport_valid &&
           adbms6830_comm_write_acknowledged(packet))
        {
            acknowledged |= bit;
        }
    }

    dev->temp_debug.acknowledged_mask = acknowledged;
    if(ack_mask != NULL)
    {
        *ack_mask = acknowledged;
    }

    status = (acknowledged == expected_mask) ? HAL_OK : HAL_ERROR;
    dev->temp_debug.overall_status = status;
    return status;
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
    if(!adbms6830_topology_valid(dev) || (data == NULL))
    {
        return;
    }

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

        if(dev->ics[curr_ic].cccrc.aux_pec ||
           !adbms6830_read_packet_counter_ok(dev, curr_ic))
        {
            continue;
        }

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
                            uint8_t sensor_num,
                            int16_t *out_raw)
{
    uint32_t sensor_bit;

    if(!adbms6830_topology_valid(dev) ||
       (ic_idx >= (uint8_t)dev->num_ics) ||
       (sensor_num >= ADBMS6830_TEMP_SENSOR_COUNT) ||
       (out_raw == NULL))
    {
        return -1;
    }

    if(mux_set_channel(dev, sensor_num) != 0)
    {
        return -1;
    }
    if(adbms6830_wait_cooperative(dev, 3000u) != HAL_OK)
    {
        return -1;
    }

    if(mux_read_gpio_voltage(dev, sensor_num) != 0)
    {
        return -1;
    }

    sensor_bit = (uint32_t)(1UL << sensor_num);
    if((dev->last_temp_updated_mask[ic_idx] & sensor_bit) == 0u)
    {
        return -1;
    }

    *out_raw = dev->ics[ic_idx].temp.raw[sensor_num];
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
////    adbms6830_wait_cooperative(dev, 4000u);
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
////	adbms6830_wait_cooperative(dev, 3000u);
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
//    adbms6830_wait_cooperative(dev, 4000u);
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
    HAL_StatusTypeDef status;
    uint16_t ack_mask = 0u;

    if(!adbms6830_topology_valid(dev) ||
       (sensor_num >= ADBMS6830_TEMP_SENSOR_COUNT))
    {
        return -1;
    }

    adbms6830_temp_debug_prepare(dev, sensor_num, false);

    uint8_t mux_idx    = sensor_num / SENSORS_PER_MUX;
    uint8_t sw_pos     = sensor_num % SENSORS_PER_MUX;
    uint8_t sw_mask    = (uint8_t)(1u << sw_pos);
    uint8_t slave_addr = MUX_ADDRS[mux_idx];

    dev->mux_selection_valid_mask[mux_idx] = 0u;
    status = adbms6830_gpio_i2c_write(dev, slave_addr, sw_mask, &ack_mask);

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        uint16_t bit = (uint16_t)(1u << ic);
        if((ack_mask & bit) != 0u)
        {
            dev->mux_selected_channel[ic][mux_idx] = sw_pos;
            dev->mux_selection_valid_mask[mux_idx] |= bit;
        }
        else
        {
            dev->mux_selected_channel[ic][mux_idx] = UINT8_MAX;
        }
    }

    dev->temp_debug.select_status = status;
    dev->temp_debug.overall_status = status;
    return (status == HAL_OK) ? 0 : -1;
}


static HAL_StatusTypeDef adbms6830_capture_aux_gpio_for_sensor(
    adbms6830_driver_t *dev,
    uint8_t sensor_num,
    bool require_acknowledged_selection,
    bool publish_sample)
{
    HAL_StatusTypeDef status;
    uint16_t selected_mask = 0u;
    uint16_t updated_mask = 0u;
    uint32_t sensor_bit;

    if(!adbms6830_topology_valid(dev) ||
       (sensor_num >= ADBMS6830_TEMP_SENSOR_COUNT))
    {
        return HAL_ERROR;
    }

    uint8_t mux_idx = sensor_num / SENSORS_PER_MUX;
    uint8_t sw_pos = sensor_num % SENSORS_PER_MUX;
    uint8_t gpio_ch = GPIO_AUX_IDX[mux_idx];
    sensor_bit = (uint32_t)(1UL << sensor_num);

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        uint16_t bit = (uint16_t)(1u << ic);
        if(publish_sample)
        {
            dev->last_temp_updated_mask[ic] &= ~sensor_bit;
        }
        if(((dev->mux_selection_valid_mask[mux_idx] & bit) != 0u) &&
           (dev->mux_selected_channel[ic][mux_idx] == sw_pos))
        {
            selected_mask |= bit;
        }
    }

    dev->temp_debug.selected_mask = selected_mask;

    if(require_acknowledged_selection && (selected_mask == 0u))
    {
        dev->temp_debug.overall_status = HAL_ERROR;
        return HAL_ERROR;
    }

    /* Trigger ADC conversion on the current GPIO channel */
    uint8_t adax_cmd[2] = { ADAX_CMD_BYTE0, ADAX_CH[mux_idx] };
    status = adbms6830_wakeup_checked(dev);
    dev->temp_debug.wake_status = status;
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adax_cmd);
    }
    dev->temp_debug.adax_status = status;
    if(status != HAL_OK)
    {
        dev->temp_debug.overall_status = status;
        return status;
    }
    if(adbms6830_wait_cooperative(dev, 4000u) != HAL_OK)
    {
        dev->temp_debug.overall_status = HAL_TIMEOUT;
        return HAL_TIMEOUT;
    }

    /* Read back and parse the auxiliary GPIO result */
    status = adbms6830_rd48_checked(dev, RDAUXA, shared_buf);
    dev->temp_debug.rdaux_status = status;
    dev->temp_debug.aux_pec_pass_mask = dev->health.last_pec_pass_mask;
    dev->temp_debug.aux_pec_fail_mask = dev->health.last_pec_fail_mask;
    dev->temp_debug.aux_counter_mismatch_mask =
        dev->health.last_cmd_counter_mismatch_mask;
    if(status != HAL_OK)
    {
        dev->temp_debug.overall_status = status;
        return status;
    }
    adbms6830_parse_aux_gpio(dev, shared_buf);

    /* Publish only readings whose mux selection, transport, command counter,
     * PEC, and AUX code are all valid for this exact sensor. */
    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        uint16_t bit = (uint16_t)(1u << ic);
        const uint8_t *packet = &shared_buf[(uint16_t)ic * RX_DATA];
        bool transport_valid = ((dev->health.last_pec_pass_mask & bit) != 0u) &&
                               ((dev->health.last_cmd_counter_mismatch_mask & bit) == 0u);
        bool code_valid = adbms6830_aux_code_valid(packet, gpio_ch);

        memcpy(dev->temp_debug.rdaux_packet[ic], packet, RX_DATA);
        if(transport_valid)
        {
            dev->temp_debug.aux_transport_valid_mask |= bit;
        }
        if(code_valid)
        {
            dev->temp_debug.aux_code_valid_mask |= bit;
        }

        if(publish_sample &&
           ((selected_mask & bit) != 0u) &&
           transport_valid &&
           code_valid)
        {
            uint8_t byte_lo = (uint8_t)(gpio_ch * 2u);
            int16_t raw = (int16_t)((uint16_t)packet[byte_lo] |
                                    ((uint16_t)packet[byte_lo + 1u] << 8u));
            dev->ics[ic].temp.raw[sensor_num] = raw;
            dev->last_temp_updated_mask[ic] |= sensor_bit;
            updated_mask |= bit;
        }
    }

    dev->temp_debug.updated_mask = updated_mask;

    if(publish_sample)
    {
        status = (updated_mask == adbms6830_expected_ic_mask(dev)) ? HAL_OK : HAL_ERROR;
    }
    else
    {
        uint16_t valid_raw_mask = (uint16_t)(dev->temp_debug.aux_transport_valid_mask &
                                             dev->temp_debug.aux_code_valid_mask);
        status = (valid_raw_mask == adbms6830_expected_ic_mask(dev)) ? HAL_OK : HAL_ERROR;
    }

    dev->temp_debug.overall_status = status;
    return status;
}


static int16_t adbms6830_code_delta_mv(int16_t a, int16_t b)
{
    int32_t delta = (int32_t)a - (int32_t)b;
    uint32_t magnitude;
    uint32_t millivolts;

    if(delta < 0)
    {
        delta = -delta;
    }
    magnitude = (uint32_t)delta;
    /* Both AUX and AUX2 result registers are normalized to the primary AUX
     * weight of 150 uV/bit, so compare the two paths in the physical mV
     * domain rather than comparing unequal native converter resolutions. */
    millivolts = ((magnitude * 150u) + 500u) / 1000u;
    return (millivolts > (uint32_t)INT16_MAX) ? INT16_MAX : (int16_t)millivolts;
}

static HAL_StatusTypeDef adbms6830_read_selected_auxa_raw(
    adbms6830_driver_t *dev,
    uint8_t gpio_ch,
    int16_t raw_out[ADBMS6830_MAX_TRACKED_ICS],
    uint16_t *valid_mask_out)
{
    HAL_StatusTypeDef status;
    uint16_t expected_mask;
    uint16_t valid_mask = 0u;

    if(!adbms6830_topology_valid(dev) ||
       (gpio_ch >= 3u) || (raw_out == NULL) || (valid_mask_out == NULL))
    {
        return HAL_ERROR;
    }

    expected_mask = adbms6830_expected_ic_mask(dev);
    memset(raw_out, 0, sizeof(int16_t) * ADBMS6830_MAX_TRACKED_ICS);
    *valid_mask_out = 0u;

    status = adbms6830_rd48_checked(dev, RDAUXA, shared_buf);
    if(status != HAL_OK)
    {
        return status;
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        uint16_t bit = (uint16_t)(1u << ic);
        const uint8_t *packet = &shared_buf[(uint16_t)ic * RX_DATA];
        bool transport_valid = ((dev->health.last_pec_pass_mask & bit) != 0u) &&
                               ((dev->health.last_cmd_counter_mismatch_mask & bit) == 0u);
        bool code_valid = adbms6830_aux_code_valid(packet, gpio_ch);
        uint8_t byte_lo = (uint8_t)(gpio_ch * 2u);

        if(transport_valid && code_valid)
        {
            raw_out[ic] = (int16_t)((uint16_t)packet[byte_lo] |
                                    ((uint16_t)packet[byte_lo + 1u] << 8u));
            valid_mask |= bit;
        }
    }

    *valid_mask_out = valid_mask;
    return (valid_mask == expected_mask) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef adbms6830_capture_aux2_gpio_for_sensor(
    adbms6830_driver_t *dev,
    uint8_t sensor_num,
    int16_t raw_out[ADBMS6830_MAX_TRACKED_ICS],
    uint16_t *valid_mask_out)
{
    HAL_StatusTypeDef status;
    uint8_t mux_idx;
    uint8_t sw_pos;
    uint8_t gpio_ch;
    uint8_t adax2_cmd[2];
    uint16_t selected_mask = 0u;
    uint16_t valid_mask = 0u;
    uint16_t expected_mask;

    if(!adbms6830_topology_valid(dev) ||
       (sensor_num >= ADBMS6830_TEMP_SENSOR_COUNT) ||
       (raw_out == NULL) || (valid_mask_out == NULL))
    {
        return HAL_ERROR;
    }

    mux_idx = sensor_num / SENSORS_PER_MUX;
    sw_pos = sensor_num % SENSORS_PER_MUX;
    gpio_ch = GPIO_AUX_IDX[mux_idx];
    expected_mask = adbms6830_expected_ic_mask(dev);

    memset(raw_out, 0, sizeof(int16_t) * ADBMS6830_MAX_TRACKED_ICS);
    *valid_mask_out = 0u;

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        uint16_t bit = (uint16_t)(1u << ic);
        if(((dev->mux_selection_valid_mask[mux_idx] & bit) != 0u) &&
           (dev->mux_selected_channel[ic][mux_idx] == sw_pos))
        {
            selected_mask |= bit;
        }
    }
    if(selected_mask != expected_mask)
    {
        return HAL_ERROR;
    }

    /* ADAX2 command is 0x400 + CH[3:0]. ADAX_CH carries the matching ADAX
     * GPIO selector in its low nibble, so strip ADAX's mode bits here. */
    adax2_cmd[0] = 0x04u;
    adax2_cmd[1] = (uint8_t)(ADAX_CH[mux_idx] & 0x0Fu);

    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adax2_cmd);
    }
    if(status == HAL_OK)
    {
        /* AUX2 updates at nominal 125 Hz. Allow one complete 8 ms period plus
         * margin; cooperative waiting prevents the priority-11 ADBMS task from
         * starving CAN/estimator/fan/IMD while the silicon converts. */
        status = adbms6830_wait_cooperative(dev, 9000u);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_rd48_checked(dev, RDRAXA, shared_buf);
    }
    if(status != HAL_OK)
    {
        return status;
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        uint16_t bit = (uint16_t)(1u << ic);
        const uint8_t *packet = &shared_buf[(uint16_t)ic * RX_DATA];
        bool transport_valid = ((dev->health.last_pec_pass_mask & bit) != 0u) &&
                               ((dev->health.last_cmd_counter_mismatch_mask & bit) == 0u);
        bool code_valid = adbms6830_aux_code_valid(packet, gpio_ch);
        uint8_t byte_lo = (uint8_t)(gpio_ch * 2u);

        if(((selected_mask & bit) != 0u) && transport_valid && code_valid)
        {
            int16_t raw = (int16_t)((uint16_t)packet[byte_lo] |
                                    ((uint16_t)packet[byte_lo + 1u] << 8u));
            dev->ics[ic].raux.ra_codes[gpio_ch] = raw;
            raw_out[ic] = raw;
            valid_mask |= bit;
        }
    }

    *valid_mask_out = valid_mask;
    return (valid_mask == expected_mask) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef adbms6830_run_aux2_redundancy(adbms6830_driver_t *dev,
                                                 uint8_t sensor_num)
{
#if AMS_ENABLE_ADBMS_AUX2_REDUNDANCY
    HAL_StatusTypeDef status;
    int16_t primary_raw[ADBMS6830_MAX_TRACKED_ICS] = {0};
    int16_t redundant_raw[ADBMS6830_MAX_TRACKED_ICS] = {0};
    uint16_t aux2_valid_mask = 0u;
    uint16_t expected_mask;
    uint8_t mux_idx;
    uint8_t gpio_ch;

    if(!adbms6830_topology_valid(dev) ||
       (sensor_num >= ADBMS6830_TEMP_SENSOR_COUNT))
    {
        return HAL_ERROR;
    }

    dev->aux2_health.sensor = sensor_num;
    dev->aux2_health.valid_mask = 0u;
    dev->aux2_health.disagree_mask = 0u;
    memset(dev->aux2_health.aux_raw, 0, sizeof(dev->aux2_health.aux_raw));
    memset(dev->aux2_health.aux2_raw, 0, sizeof(dev->aux2_health.aux2_raw));
    memset(dev->aux2_health.delta_mv, 0, sizeof(dev->aux2_health.delta_mv));
    adbms6830_increment_u32_sat(&dev->aux2_health.count);

    expected_mask = adbms6830_expected_ic_mask(dev);
    mux_idx = sensor_num / SENSORS_PER_MUX;
    gpio_ch = GPIO_AUX_IDX[mux_idx];

    status = (mux_set_channel(dev, sensor_num) == 0) ? HAL_OK : HAL_ERROR;
    if(status == HAL_OK)
    {
        status = adbms6830_wait_cooperative(dev, 3000u);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_capture_aux_gpio_for_sensor(dev,
                                                       sensor_num,
                                                       true,
                                                       false);
    }
    if(status == HAL_OK)
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            primary_raw[ic] = dev->ics[ic].aux.a_codes[gpio_ch];
        }
        status = adbms6830_capture_aux2_gpio_for_sensor(dev,
                                                        sensor_num,
                                                        redundant_raw,
                                                        &aux2_valid_mask);
    }

    if(status == HAL_OK)
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            uint16_t bit = (uint16_t)(1u << ic);
            int16_t delta_mv = adbms6830_code_delta_mv(primary_raw[ic],
                                                       redundant_raw[ic]);
            dev->aux2_health.aux_raw[ic] = primary_raw[ic];
            dev->aux2_health.aux2_raw[ic] = redundant_raw[ic];
            dev->aux2_health.delta_mv[ic] = delta_mv;
            if((aux2_valid_mask & bit) != 0u)
            {
                dev->aux2_health.valid_mask |= bit;
                if((uint16_t)delta_mv > AMS_ADBMS_AUX2_COMPARE_THRESHOLD_MV)
                {
                    dev->aux2_health.disagree_mask |= bit;
                }
            }
        }
        status = ((dev->aux2_health.valid_mask == expected_mask) &&
                  (dev->aux2_health.disagree_mask == 0u)) ? HAL_OK : HAL_ERROR;
    }

    if(status != HAL_OK)
    {
        adbms6830_increment_u32_sat(&dev->aux2_health.fail_count);
    }
    return status;
#else
    (void)dev;
    (void)sensor_num;
    return HAL_OK;
#endif
}

HAL_StatusTypeDef adbms6830_run_thermistor_open_wire(adbms6830_driver_t *dev,
                                                      uint8_t sensor_num)
{
#if AMS_ENABLE_ADBMS_THERM_OPEN_WIRE_DIAG
    HAL_StatusTypeDef status = HAL_OK;
    HAL_StatusTypeDef restore_status = HAL_OK;
    HAL_StatusTypeDef recovery_status = HAL_ERROR;
    cfa6830_ production_cfga[ADBMS6830_MAX_TRACKED_ICS];
    int16_t baseline[ADBMS6830_MAX_TRACKED_ICS] = {0};
    int16_t pulldown[ADBMS6830_MAX_TRACKED_ICS] = {0};
    int16_t pullup[ADBMS6830_MAX_TRACKED_ICS] = {0};
    int16_t recovery[ADBMS6830_MAX_TRACKED_ICS] = {0};
    uint16_t baseline_valid_mask = 0u;
    uint16_t pulldown_valid_mask = 0u;
    uint16_t pullup_valid_mask = 0u;
    uint16_t recovery_valid_mask = 0u;
    uint16_t expected_mask;
    uint8_t mux_idx;
    uint8_t gpio_ch;
    uint8_t adax_ow_down_cmd[2];
    uint8_t adax_ow_up_cmd[2];
    bool diagnostic_cfga_written = false;

    if(!adbms6830_topology_valid(dev) ||
       (sensor_num >= ADBMS6830_TEMP_SENSOR_COUNT))
    {
        return HAL_ERROR;
    }

    dev->therm_ow_health.sensor = sensor_num;
    dev->therm_ow_health.valid_mask = 0u;
    dev->therm_ow_health.suspect_mask = 0u;
    memset(dev->therm_ow_health.baseline_raw, 0,
           sizeof(dev->therm_ow_health.baseline_raw));
    memset(dev->therm_ow_health.pulldown_raw, 0,
           sizeof(dev->therm_ow_health.pulldown_raw));
    memset(dev->therm_ow_health.pullup_raw, 0,
           sizeof(dev->therm_ow_health.pullup_raw));
    memset(dev->therm_ow_health.recovery_raw, 0,
           sizeof(dev->therm_ow_health.recovery_raw));
    memset(dev->therm_ow_health.pulldown_delta_mv, 0,
           sizeof(dev->therm_ow_health.pulldown_delta_mv));
    memset(dev->therm_ow_health.pullup_delta_mv, 0,
           sizeof(dev->therm_ow_health.pullup_delta_mv));
    memset(dev->therm_ow_health.recovery_delta_mv, 0,
           sizeof(dev->therm_ow_health.recovery_delta_mv));
    adbms6830_increment_u32_sat(&dev->therm_ow_health.count);

    expected_mask = adbms6830_expected_ic_mask(dev);
    mux_idx = sensor_num / SENSORS_PER_MUX;
    gpio_ch = GPIO_AUX_IDX[mux_idx];

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        production_cfga[ic] = dev->ics[ic].tx_cfga;
    }

    status = (mux_set_channel(dev, sensor_num) == 0) ? HAL_OK : HAL_ERROR;
    if(status == HAL_OK)
    {
        status = adbms6830_wait_cooperative(dev, 3000u);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_capture_aux_gpio_for_sensor(dev,
                                                       sensor_num,
                                                       true,
                                                       false);
    }
    if(status == HAL_OK)
    {
        baseline_valid_mask = expected_mask;
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            baseline[ic] = dev->ics[ic].aux.a_codes[gpio_ch];
        }

        /* The current source must be present before the ADC samples if this is
         * to diagnose high-resistance/open wiring. Waiting after an immediate
         * conversion is not a soak. Temporarily enable the silicon's maximum
         * short-range ~4.1 ms AUX soak, then restore the exact production CFGA
         * before normal temperature publication resumes. */
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            dev->ics[ic].tx_cfga = production_cfga[ic];
            dev->ics[ic].tx_cfga.soakon = 1u;
            dev->ics[ic].tx_cfga.owrng = 0u;
            dev->ics[ic].tx_cfga.owa = 7u;
        }
        status = adbms6830_wrcfga_checked(dev);
        diagnostic_cfga_written = (status == HAL_OK);
    }

    /* ADAX: command bit OW=1. PUP=0 uses the pull-down source; PUP=1 uses
     * pull-up. ADAX_CH supplies the selected GPIO/mux output. */
    adax_ow_down_cmd[0] = (uint8_t)(ADAX_CMD_BYTE0 | 0x01u);
    adax_ow_down_cmd[1] = ADAX_CH[mux_idx];
    adax_ow_up_cmd[0] = (uint8_t)(ADAX_CMD_BYTE0 | 0x01u);
    adax_ow_up_cmd[1] = (uint8_t)(ADAX_CH[mux_idx] | 0x80u);

    if(status == HAL_OK)
    {
        status = adbms6830_wakeup_checked(dev);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adax_ow_down_cmd);
    }
    if(status == HAL_OK)
    {
        /* 4.1 ms programmed soak + ~1 ms AUX conversion + margin. */
        status = adbms6830_wait_cooperative(dev, 6000u);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_read_selected_auxa_raw(dev, gpio_ch,
                                                  pulldown,
                                                  &pulldown_valid_mask);
    }

    if(status == HAL_OK)
    {
        status = adbms6830_wakeup_checked(dev);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adax_ow_up_cmd);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_wait_cooperative(dev, 6000u);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_read_selected_auxa_raw(dev, gpio_ch,
                                                  pullup,
                                                  &pullup_valid_mask);
    }

    /* Restore the exact production CFGA even when the stimulus failed. A
     * failed restore is not merely an observational thermistor failure: it is
     * a configuration-integrity fault and is surfaced to the periodic config
     * checker as well as this diagnostic's own counter. */
    if(diagnostic_cfga_written)
    {
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            dev->ics[ic].tx_cfga = production_cfga[ic];
        }
        restore_status = adbms6830_wrcfga_checked(dev);
        if(restore_status == HAL_OK)
        {
            restore_status = adbms6830_verify_config_readback(dev);
        }
        if(restore_status != HAL_OK)
        {
            adbms6830_increment_u32_sat(
                &dev->therm_ow_health.config_restore_fail_count);
        }
    }

    /* Always acquire one ordinary unstimulated sample after CFGA restoration.
     * The diagnostic sample is never copied to the normal temperature image. */
    recovery_status = (restore_status == HAL_OK) ?
        adbms6830_capture_aux_gpio_for_sensor(dev, sensor_num, true, false) :
        restore_status;
    if(recovery_status == HAL_OK)
    {
        recovery_valid_mask = expected_mask;
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            recovery[ic] = dev->ics[ic].aux.a_codes[gpio_ch];
        }
    }

    if((status == HAL_OK) && (restore_status == HAL_OK) &&
       (recovery_status == HAL_OK))
    {
        uint16_t combined_valid = (uint16_t)(baseline_valid_mask &
                                             pulldown_valid_mask &
                                             pullup_valid_mask &
                                             recovery_valid_mask);
        dev->therm_ow_health.valid_mask = combined_valid;
        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            uint16_t bit = (uint16_t)(1u << ic);
            int16_t down_delta = adbms6830_code_delta_mv(pulldown[ic], baseline[ic]);
            int16_t up_delta = adbms6830_code_delta_mv(pullup[ic], baseline[ic]);
            int16_t recovery_delta = adbms6830_code_delta_mv(recovery[ic], baseline[ic]);

            dev->therm_ow_health.baseline_raw[ic] = baseline[ic];
            dev->therm_ow_health.pulldown_raw[ic] = pulldown[ic];
            dev->therm_ow_health.pullup_raw[ic] = pullup[ic];
            dev->therm_ow_health.recovery_raw[ic] = recovery[ic];
            dev->therm_ow_health.pulldown_delta_mv[ic] = down_delta;
            dev->therm_ow_health.pullup_delta_mv[ic] = up_delta;
            dev->therm_ow_health.recovery_delta_mv[ic] = recovery_delta;

            if(((combined_valid & bit) == 0u) ||
               ((uint16_t)down_delta < AMS_ADBMS_THERM_OW_MIN_DELTA_MV) ||
               ((uint16_t)up_delta < AMS_ADBMS_THERM_OW_MIN_DELTA_MV) ||
               ((uint16_t)recovery_delta > AMS_ADBMS_THERM_OW_RECOVERY_TOL_MV))
            {
                dev->therm_ow_health.suspect_mask |= bit;
            }
        }
        status = ((combined_valid == expected_mask) &&
                  (dev->therm_ow_health.suspect_mask == 0u)) ? HAL_OK : HAL_ERROR;
    }
    else if(restore_status != HAL_OK)
    {
        status = restore_status;
    }
    else if(recovery_status != HAL_OK)
    {
        status = recovery_status;
    }

    if(status != HAL_OK)
    {
        adbms6830_increment_u32_sat(&dev->therm_ow_health.fail_count);
    }
    return status;
#else
    (void)dev;
    (void)sensor_num;
    return HAL_OK;
#endif
}


HAL_StatusTypeDef adbms6830_run_s_periodic_diagnostic(adbms6830_driver_t *dev)
{
#if AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && AMS_S_PATH_ECO_VALIDATED
    HAL_StatusTypeDef status;
    HAL_StatusTypeDef ow_status;
    uint8_t adsv_sync_cmd[2] = { 0x01u, 0xE8u }; /* CONT=1,DCP=0,OW=0 */
    uint8_t adsv_stop_cmd[2] = { 0x01u, 0x78u }; /* CONT=0,DCP=1,OW=0 */

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    adbms6830_increment_u32_sat(&dev->health.s_periodic_diag_count);

    /* Synchronize S to the already-running C average and let the silicon run
     * its C/S comparison. Discharge is inhibited during this diagnostic. */
    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adsv_sync_cmd);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_wait_cooperative(dev, 16000u);
    }
    if(status == HAL_OK)
    {
        /* Refresh Status C so CSxFLT generated by the synchronized comparison
         * is captured before the S open-wire phases alter S measurements. */
        status = adbms6830_read_status(dev, false);
    }

    /* Perform baseline/even/odd S-path open-wire. This path never overwrites
     * the continuously authoritative C result registers. */
    ow_status = (status == HAL_OK) ?
                adbms6830_run_open_wire_diagnostic_path(dev,
                    ADBMS6830_OPEN_WIRE_PATH_S) : status;
    if((status == HAL_OK) && (ow_status != HAL_OK))
    {
        status = ow_status;
    }

    /* Explicitly return the S path to a final single conversion with discharge
     * permitted, after which S stops while C continues. */
    {
        HAL_StatusTypeDef stop_status = adbms6830_wakeup_checked(dev);
        if(stop_status == HAL_OK)
        {
            stop_status = adbms6830_cmd_checked(dev, adsv_stop_cmd);
        }
        if(stop_status == HAL_OK)
        {
            stop_status = adbms6830_wait_cooperative(dev,
                ADBMS6830_OPEN_WIRE_CONVERSION_WAIT_US);
        }
        if((status == HAL_OK) && (stop_status != HAL_OK))
        {
            status = stop_status;
        }
    }

    if(status != HAL_OK)
    {
        adbms6830_increment_u32_sat(&dev->health.s_periodic_diag_fail_count);
    }
    return status;
#else
    (void)dev;
    return HAL_OK;
#endif
}


int mux_read_gpio_voltage(adbms6830_driver_t *dev, uint8_t sensor_num)
{
    HAL_StatusTypeDef status = adbms6830_capture_aux_gpio_for_sensor(
        dev, sensor_num, true, true);
    return (status == HAL_OK) ? 0 : -1;
}


HAL_StatusTypeDef adbms6830_temp_debug_capture(adbms6830_driver_t *dev,
                                                uint8_t sensor_num,
                                                bool force_aux_capture)
{
    HAL_StatusTypeDef select_status;
    HAL_StatusTypeDef aux_status = HAL_ERROR;

    if(!adbms6830_topology_valid(dev) ||
       (sensor_num >= ADBMS6830_TEMP_SENSOR_COUNT))
    {
        return HAL_ERROR;
    }

    select_status = (mux_set_channel(dev, sensor_num) == 0) ? HAL_OK : HAL_ERROR;
    dev->temp_debug.forced_aux_capture = force_aux_capture;
    dev->temp_debug.select_status = select_status;

    if(adbms6830_wait_cooperative(dev, 3000u) != HAL_OK)
    {
        dev->temp_debug.overall_status = HAL_TIMEOUT;
        return HAL_TIMEOUT;
    }

    if(select_status == HAL_OK)
    {
        aux_status = adbms6830_capture_aux_gpio_for_sensor(
            dev, sensor_num, true, true);
    }
    else if(force_aux_capture)
    {
        /* A raw AUX capture can reveal whether the mux transaction actually
         * changed the analog node even when RDCOMM reported NACK. Never
         * publish this sample because the channel identity is unverified. */
        aux_status = adbms6830_capture_aux_gpio_for_sensor(
            dev, sensor_num, false, false);
    }

    dev->temp_debug.overall_status =
        ((select_status == HAL_OK) && (aux_status == HAL_OK)) ? HAL_OK : HAL_ERROR;
    return dev->temp_debug.overall_status;
}


HAL_StatusTypeDef adbms6830_temp_bus_idle_capture(adbms6830_driver_t *dev)
{
    adbms6830_temp_bus_debug_t *dbg;
    HAL_StatusTypeDef status;
    uint8_t adax_cmd[2] = { ADAX_CMD_BYTE0, ADAX_CMD_BYTE1 };

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    dbg = &dev->temp_bus_debug;
    memset(dbg, 0, sizeof(*dbg));
    dbg->valid = true;
    dbg->expected_ic_mask = adbms6830_expected_ic_mask(dev);
    dbg->wake_status = HAL_ERROR;
    dbg->adax_status = HAL_ERROR;
    dbg->rdauxb_status = HAL_ERROR;
    dbg->overall_status = HAL_ERROR;

    /* This command only starts the high-impedance AUX ADC.  It does not write
     * COMM and it does not issue STCOMM, so GPIO4/SDA and GPIO5/SCL are never
     * actively pulled low by this diagnostic. */
    status = adbms6830_wakeup_checked(dev);
    dbg->wake_status = status;
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adax_cmd);
    }
    dbg->adax_status = status;
    if(status != HAL_OK)
    {
        dbg->overall_status = status;
        return status;
    }

    status = adbms6830_wait_cooperative(dev, ADBMS6830_AUX_CONVERSION_WAIT_US);
    if(status != HAL_OK)
    {
        dbg->overall_status = status;
        return status;
    }

    status = adbms6830_rd48_checked(dev, RDAUXB, shared_buf);
    dbg->rdauxb_status = status;
    dbg->pec_pass_mask = dev->health.last_pec_pass_mask;
    dbg->pec_fail_mask = dev->health.last_pec_fail_mask;
    dbg->counter_mismatch_mask = dev->health.last_cmd_counter_mismatch_mask;
    if(status != HAL_OK)
    {
        dbg->overall_status = status;
        return status;
    }

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        uint16_t bit = (uint16_t)(1u << ic);
        const uint8_t *packet = &shared_buf[(uint16_t)ic * RX_DATA];
        bool transport_valid = ((dbg->pec_pass_mask & bit) != 0u) &&
                               ((dbg->counter_mismatch_mask & bit) == 0u);
        bool gpio4_valid = !(((packet[0] == 0xFFu) && (packet[1] == 0xFFu)) ||
                             ((packet[0] == 0x00u) && (packet[1] == 0x80u)));
        bool gpio5_valid = !(((packet[2] == 0xFFu) && (packet[3] == 0xFFu)) ||
                             ((packet[2] == 0x00u) && (packet[3] == 0x80u)));

        memcpy(dbg->rdauxb_packet[ic], packet, RX_DATA);
        dbg->gpio4_raw[ic] = (int16_t)((uint16_t)packet[0] |
                                       ((uint16_t)packet[1] << 8u));
        dbg->gpio5_raw[ic] = (int16_t)((uint16_t)packet[2] |
                                       ((uint16_t)packet[3] << 8u));

        if(transport_valid)
        {
            dbg->transport_valid_mask |= bit;
        }
        if(gpio4_valid)
        {
            dbg->gpio4_code_valid_mask |= bit;
        }
        if(gpio5_valid)
        {
            dbg->gpio5_code_valid_mask |= bit;
        }
    }

    dbg->overall_status =
        ((dbg->transport_valid_mask == dbg->expected_ic_mask) &&
         (dbg->gpio4_code_valid_mask == dbg->expected_ic_mask) &&
         (dbg->gpio5_code_valid_mask == dbg->expected_ic_mask)) ?
        HAL_OK : HAL_ERROR;
    return dbg->overall_status;
}



HAL_StatusTypeDef adbms6830_temp_bus_scan_capture(adbms6830_driver_t *dev)
{
    adbms6830_temp_bus_scan_t *scan;
    bool all_transport_ok = true;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    scan = &dev->temp_bus_scan;
    memset(scan, 0, sizeof(*scan));
    scan->valid = true;
    scan->first_address = ADBMS6830_TEMP_BUS_SCAN_FIRST_ADDR;
    scan->address_count = ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT;
    scan->data_byte = ADBMS6830_TEMP_BUS_SCAN_DATA_BYTE;
    scan->expected_ic_mask = adbms6830_expected_ic_mask(dev);
    scan->overall_status = HAL_ERROR;

    /* The scan intentionally writes 0x00, which opens every ADG728 switch.
     * Invalidate all cached channel identities before and after the scan so a
     * later AUX conversion can never be mislabeled as a previously selected
     * thermistor. */
    adbms6830_invalidate_mux_selections(dev);

    for(uint8_t index = 0u;
        index < ADBMS6830_TEMP_BUS_SCAN_ADDRESS_COUNT;
        index++)
    {
        uint8_t address =
            (uint8_t)(ADBMS6830_TEMP_BUS_SCAN_FIRST_ADDR + index);
        uint16_t ack_mask = 0u;
        HAL_StatusTypeDef probe_status;
        HAL_StatusTypeDef transport_status;
        const adbms6830_temp_debug_t *dbg;

        adbms6830_temp_debug_prepare_probe(
            dev, address, ADBMS6830_TEMP_BUS_SCAN_DATA_BYTE);
        probe_status = adbms6830_gpio_i2c_write(
            dev,
            address,
            ADBMS6830_TEMP_BUS_SCAN_DATA_BYTE,
            &ack_mask);
        dbg = &dev->temp_debug;

        scan->probe_status[index] = probe_status;
        scan->wrc_status[index] = dbg->wrc_status;
        scan->pre_rdcomm_status[index] = dbg->pre_rdcomm_status;
        scan->stcomm_status[index] = dbg->stcomm_status;
        scan->rdcomm_status[index] = dbg->rdcomm_status;
        scan->pre_comm_match_mask[index] = dbg->pre_comm_match_mask;
        scan->comm_pec_pass_mask[index] = dbg->comm_pec_pass_mask;
        scan->comm_pec_fail_mask[index] = dbg->comm_pec_fail_mask;
        scan->comm_counter_mismatch_mask[index] =
            dbg->comm_counter_mismatch_mask;
        scan->comm_transport_valid_mask[index] =
            dbg->comm_transport_valid_mask;
        scan->address_ack_mask[index] = dbg->address_ack_mask;
        scan->data_ack_mask[index] = dbg->data_ack_mask;
        scan->acknowledged_mask[index] = ack_mask;

        for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
        {
            memcpy(scan->rdcomm_packet[index][ic],
                   dbg->rdcomm_packet[ic],
                   RX_DATA);
        }

        /* ACK is a discovery result, not a transport result. A healthy scan
         * can complete successfully while finding zero slaves. */
        if((dbg->wrc_status == HAL_OK) &&
           (dbg->pre_rdcomm_status == HAL_OK) &&
           (dbg->stcomm_status == HAL_OK) &&
           (dbg->rdcomm_status == HAL_OK) &&
           (dbg->pre_comm_match_mask == scan->expected_ic_mask) &&
           (dbg->comm_transport_valid_mask == scan->expected_ic_mask))
        {
            transport_status = HAL_OK;
        }
        else if((dbg->wrc_status == HAL_TIMEOUT) ||
                (dbg->pre_rdcomm_status == HAL_TIMEOUT) ||
                (dbg->stcomm_status == HAL_TIMEOUT) ||
                (dbg->rdcomm_status == HAL_TIMEOUT))
        {
            transport_status = HAL_TIMEOUT;
        }
        else if((dbg->wrc_status == HAL_BUSY) ||
                (dbg->pre_rdcomm_status == HAL_BUSY) ||
                (dbg->stcomm_status == HAL_BUSY) ||
                (dbg->rdcomm_status == HAL_BUSY))
        {
            transport_status = HAL_BUSY;
        }
        else
        {
            transport_status = HAL_ERROR;
        }

        scan->transport_status[index] = transport_status;
        if(transport_status != HAL_OK)
        {
            all_transport_ok = false;
        }

        if(dbg->address_ack_mask != 0u)
        {
            scan->any_ack_address_bitmap |= (uint8_t)(1u << index);
        }
        if(ack_mask == scan->expected_ic_mask)
        {
            scan->full_ack_address_bitmap |= (uint8_t)(1u << index);
        }
    }

    adbms6830_invalidate_mux_selections(dev);
    scan->overall_status = all_transport_ok ? HAL_OK : HAL_ERROR;
    return scan->overall_status;
}


/* ---------------------------------------------------------------------------
 * adbms6830_convert_temp
 *
 * Convert a raw GPIO ADC code to a voltage in volts.
 *
 * The ADBMS6830 AUX result registers use the signed offset representation:
 *   V = (raw_code + 10000) * 150e-6  (150 µV per LSB)
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

    if(raw == INT16_MIN)
    {
        return -1.0f;
    }

    return ((float)raw + 10000.0f) * 150.0e-6f;


}

float voltage_to_temp(float v)
{
    /* Legacy public wrapper retained for existing callers. The input is the
     * signed DER26 ADBMS AUX code, despite the historic function name. */
    if(!isfinite(v) || (v < (float)INT16_MIN) || (v > (float)INT16_MAX))
    {
        return NAN;
    }

    thermistor_result_t result = thermistor_from_adbms_raw(
        (int16_t)lroundf(v), THERMISTOR_NOMINAL_VREG_V);
    return result.valid ? result.temperature_c : NAN;
}
