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

static unsigned char shared_buf[BUFSZ] = {0};
static uint8_t write_buf[BUFSZ] = {0};
static uint8_t spi_txrx_tx_buf[BUFSZ] = {0};
static uint8_t spi_txrx_rx_buf[BUFSZ] = {0};

#define ADBMS6830_SPI_DUMMY_BYTE 0xFFu

#define RDCVALL_CMD         {0x00, 0x0C}

#define RDCVALL_DATA_SZ     32   /* 16 cells x 2 bytes */

#define RDCVALL_RX_DATA     34   /* 32 data bytes + 2 PEC/counter bytes */
#define ADBMS6830_DELAY_SPINS_PER_US 256u
#define ADBMS6830_DELAY_BASE_SPINS 1024u

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
           (dev->ics_capacity > 0u) &&
           (dev->num_ics <= (int)dev->ics_capacity) &&
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
static void adbms6830_spi_debug_note_tx(adbms6830_driver_t *dev, adbms6830_spi_op_t op, const uint8_t *cmd, const uint8_t *tx, uint16_t tx_len, uint16_t rx_len);
static void adbms6830_spi_debug_note_rx(adbms6830_driver_t *dev, const uint8_t *rx, uint16_t rx_len, HAL_StatusTypeDef status);
static void adbms6830_note_counter_reset(adbms6830_driver_t *dev);
static void adbms6830_note_counter_increment(adbms6830_driver_t *dev);
static void adbms6830_note_observed_counter(adbms6830_driver_t *dev, uint8_t current_ic, uint8_t observed_counter, bool pec_ok);
static void adbms6830_note_pec_result(adbms6830_driver_t *dev, uint8_t current_ic, bool pec_ok);
static bool adbms6830_cmd_increments_counter(const uint8_t cmd[CMDSZ]);
static bool adbms6830_cmd_resets_counter(const uint8_t cmd[CMDSZ]);
static void adbms6830_parse_sid(adbms6830_driver_t *dev, const uint8_t *data);
static void adbms6830_parse_statc(adbms6830_driver_t *dev, const uint8_t *data);
static void adbms6830_parse_statd(adbms6830_driver_t *dev, const uint8_t *data);
static void adbms6830_parse_state(adbms6830_driver_t *dev, const uint8_t *data);
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
        dev->health.pec_pass_count[current_ic]++;
    }
    else
    {
        dev->health.last_pec_fail_mask |= bit;
        dev->health.sticky_pec_fail_mask |= bit;
        dev->health.pec_fail_count[current_ic]++;
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

    if(dev->spi_debug.expected_cmd_counter[current_ic] != observed_counter)
    {
        dev->spi_debug.cmd_counter_mismatch_mask |= bit;
        dev->spi_debug.cmd_counter_error_count++;
        dev->spi_debug.error_count++;
        dev->health.last_cmd_counter_mismatch_mask |= bit;
        dev->health.sticky_cmd_counter_mismatch_mask |= bit;
        dev->health.cmd_counter_mismatch_count[current_ic]++;
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
    if((cmd[0] == 0x02u) || (cmd[0] == 0x03u) || (cmd[0] == 0x04u) ||
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
        bool packet_valid = pec_ok && adbms6830_read_packet_counter_ok(dev, curr_ic);

        dev->ics[curr_ic].cccrc.sid_pec = pec_ok ? 0u : 1u;
        dev->diag[curr_ic].sid_valid = packet_valid;
        if(packet_valid)
        {
            memcpy(dev->ics[curr_ic].sid.sid, d, RSID);
            memcpy(dev->diag[curr_ic].sid, d, RSID);
        }
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
    case ADBMS6830_SPI_OP_CLEAR_FLAGS: return "clear_flags";
    case ADBMS6830_SPI_OP_CONFIG_CHECK: return "config_check";
    case ADBMS6830_SPI_OP_BALANCE_CHECK: return "balance_check";
    case ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST: return "cell_adc_diag";
    case ADBMS6830_SPI_OP_OPEN_WIRE_EVEN: return "open_wire_even";
    case ADBMS6830_SPI_OP_OPEN_WIRE_ODD: return "open_wire_odd";
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

        status = adbms6830_us_delay(dev, 2000u);
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

    if(dev == NULL)
    {
        return HAL_ERROR;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_READ_SID;
    }

    status = adbms6830_rd48_checked(dev, RDSID, shared_buf);
    if(status == HAL_OK)
    {
        adbms6830_parse_sid(dev, shared_buf);
        if(!adbms6830_last_read_integrity_ok(dev))
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

    if(dev == NULL)
    {
        return HAL_ERROR;
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

    return first_error;
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
    if(dev == NULL)
    {
        return;
    }

    memset(&dev->health, 0, sizeof(dev->health));
    dev->health.last_status = HAL_OK;
}

HAL_StatusTypeDef adbms6830_verify_config_readback(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    bool cfga_verified = false;
    bool cfgb_verified = false;
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

        if(!cfga_verified ||
           (memcmp(dev->ics[ic].configa.tx_data,
                   dev->ics[ic].configa.rx_data,
                   TX_DATA) != 0))
        {
            dev->health.configa_mismatch_mask |= bit;
            mismatch = true;
        }
        if(!cfgb_verified ||
           (memcmp(dev->ics[ic].configb.tx_data,
                   dev->ics[ic].configb.rx_data,
                   TX_DATA) != 0))
        {
            dev->health.configb_mismatch_mask |= bit;
            mismatch = true;
        }
        if(mismatch)
        {
            dev->health.config_mismatch_mask |= bit;
            adbms6830_increment_u32_sat(&dev->health.config_mismatch_count[ic]);
        }
    }

    if((first_error == HAL_OK) && (dev->health.config_mismatch_mask != 0u))
    {
        first_error = HAL_ERROR;
    }

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
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    uint8_t adcv_diag_cmd[2];

    if(dev == NULL)
    {
        return HAL_ERROR;
    }

    dev->health.cell_adc_self_test_count++;
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST;
    }

    /* Exercise the cell ADC path with filter reset, then poll and read status.
     * This is a bring-up diagnostic hook; final pass/fail interpretation must
     * be confirmed against the ADBMS6830 datasheet and bench measurements.
     */
    adcv_diag_cmd[0] = 0x02u | (uint8_t)RD_ON;
    adcv_diag_cmd[1] = ((uint8_t)SINGLE << 7u) | ((uint8_t)DCP_OFF << 4u)
                     | ((uint8_t)RSTF_ON << 2u) | ((uint8_t)OW_OFF_ALL_CH & 0x03u)
                     | 0x60u;
    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adcv_diag_cmd);
    }
    if(status != HAL_OK)
    {
        first_error = status;
    }

    status = adbms6830_cmd_checked(dev, PLCADC);
    if((status != HAL_OK) && (first_error == HAL_OK))
    {
        first_error = status;
    }

    status = adbms6830_read_status(dev, false);
    if((status != HAL_OK) && (first_error == HAL_OK))
    {
        first_error = status;
    }

    adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST, first_error);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_CELL_ADC_SELF_TEST;
        dev->spi_debug.last_status = first_error;
    }
    return first_error;
}

HAL_StatusTypeDef adbms6830_run_open_wire_check(adbms6830_driver_t *dev, bool odd_channels)
{
    HAL_StatusTypeDef status;
    uint8_t adcv_ow_cmd[2];
    OW_C_S owcs = odd_channels ? OW_ON_ODD_CH : OW_ON_EVEN_CH;
    adbms6830_spi_op_t op = odd_channels ? ADBMS6830_SPI_OP_OPEN_WIRE_ODD :
                                            ADBMS6830_SPI_OP_OPEN_WIRE_EVEN;

    if(dev == NULL)
    {
        return HAL_ERROR;
    }

    if(odd_channels)
    {
        dev->health.open_wire_odd_count++;
    }
    else
    {
        dev->health.open_wire_even_count++;
    }

    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = op;
    }

    adcv_ow_cmd[0] = 0x02u | (uint8_t)RD_ON;
    adcv_ow_cmd[1] = ((uint8_t)SINGLE << 7u) | ((uint8_t)DCP_OFF << 4u)
                   | ((uint8_t)RSTF_ON << 2u) | ((uint8_t)owcs & 0x03u) | 0x60u;
    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adcv_ow_cmd);
    }

    adbms6830_diag_note_status(dev, op, status);
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = op;
        dev->spi_debug.last_status = status;
    }
    return status;
}

HAL_StatusTypeDef adbms6830_run_aux_gpio_diagnostic(adbms6830_driver_t *dev)
{
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;
    uint8_t adax_cmd[2] = { ADAX_CMD_BYTE0, ADAX_CMD_BYTE1 };

    if(dev == NULL)
    {
        return HAL_ERROR;
    }

    dev->health.aux_gpio_diag_count++;
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_AUX_GPIO_DIAG;
    }

    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adax_cmd);
    }
    if(status != HAL_OK)
    {
        first_error = status;
    }

    status = adbms6830_rd48_checked(dev, RDAUXA, shared_buf);
    if(status == HAL_OK)
    {
        adbms6830_parse_aux_gpio(dev, shared_buf);
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
					   adbms6830_asic* ics,
					   uint8_t ics_capacity,
					   SPI_HandleTypeDef* hspi,
					   GPIO_TypeDef* cs_port_a,
					   GPIO_TypeDef* cs_port_b,
					   uint16_t cs_pin_a,
					   uint16_t cs_pin_b,
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
	   (ics == NULL) ||
	   (ics_capacity < num_ics) ||
	   (hspi == NULL) ||
	   (cs_port_a == NULL) ||
	   (cs_port_b == NULL) ||
	   (cs_pin_a == 0u) ||
	   (cs_pin_b == 0u))
	{
		return HAL_ERROR;
	}

	memset(ics, 0, sizeof(*ics) * num_ics);
	dev->num_ics = num_ics;

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
		dev->last_temp_updated_mask[ic] = 0u;
		for(uint8_t mux = 0u; mux < ADBMS6830_MUX_COUNT; mux++)
		{
			dev->mux_selected_channel[ic][mux] = UINT8_MAX;
		}
	}
	memset(dev->mux_selection_valid_mask, 0, sizeof(dev->mux_selection_valid_mask));
	memset(dev->diag, 0, sizeof(dev->diag));

	// Set CS pins high
	dev->string = STRING_B;
	adbms6830_set_cs(dev, 1);
	dev->string = STRING_A;
	adbms6830_set_cs(dev, 1);
	/* Final mixed ring order is String A -> five SMB monitors -> APM ->
	 * String B.  Keeping the SMBs as the leading String-A subset allows their
	 * 5-packet reads and writes to stop before the ADBMS2950. */
	dev->string = STRING_A;

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

	adbms6830_reset_cfg(dev);

	status = adbms6830_wrcfga_checked(dev);
	if(status != HAL_OK)
	{
		adbms6830_diag_note_status(dev, ADBMS6830_SPI_OP_WR48, status);
		return status;
	}
	status = adbms6830_wrcfgb_checked(dev);
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
		dev->ics[i].tx_cfgb.dtmen = 1u;
		dev->ics[i].tx_cfgb.dtrng = RANG_0_TO_63_MIN;
		dev->ics[i].tx_cfgb.dcto = TIME_1MIN_OR_0_26HR;
		dev->ics[i].tx_cfgb.dcc = 0u;
		memset(dev->ics[i].PwmA.pwma, 0, sizeof(dev->ics[i].PwmA.pwma));
		memset(dev->ics[i].PwmB.pwmb, 0, sizeof(dev->ics[i].PwmB.pwmb));
		memset(dev->ics[i].pwma.tx_data, 0, sizeof(dev->ics[i].pwma.tx_data));
		memset(dev->ics[i].pwmb.tx_data, 0, sizeof(dev->ics[i].pwmb.tx_data));
	}
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

void adbms6830_wrcfgb(adbms6830_driver_t *dev)
{
	(void)adbms6830_wrcfgb_checked(dev);
}

HAL_StatusTypeDef adbms6830_wrcfgb_checked(adbms6830_driver_t *dev)
{
	if(!adbms6830_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	adbms6830_asic *ic = dev->ics;
	adbms6830_pack_cfgb(dev);
	for (uint8_t cic = 0; cic < dev->num_ics; cic++)
	{
		for (uint8_t data = 0; data < TX_DATA; data++)
		{
			shared_buf[(cic * TX_DATA) + data] = ic[cic].configb.tx_data[data];
		}
	}
	return adbms6830_wr48_checked(dev, WRCFGB, shared_buf);
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
	adbms6830_cmd(dev, SRST);
}

HAL_StatusTypeDef adbms6830_wakeup_checked(adbms6830_driver_t* dev)
{
	HAL_StatusTypeDef status;

	if(!adbms6830_topology_valid(dev))
	{
		return HAL_ERROR;
	}

	for(uint8_t i = 0; i < dev->num_ics; i++)
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
        for(uint8_t i = 0u; i < (uint8_t)dev->num_ics; i++)
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
    (void)adbms6830_us_delay(dev, 5000u);
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

    if(!adbms6830_topology_valid(dev) || (cmd == NULL))
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

    if(!adbms6830_topology_valid(dev) || (cmd == NULL) || (tx_data == NULL))
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

    /* 2. Wakeup the isoSPI Daisy Chain. A stalled delay timer is a transport
     * failure; do not clock a transaction with unverified wake timing. */
    status = adbms6830_wakeup_checked(dev);
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

        cmd_cntr = (ic_data[RX_DATA - 2u] >> 2u);
        received_pec = (uint16_t)(((ic_data[RX_DATA - 2u] & 0x03u) << 8u) | ic_data[RX_DATA - 1u]);
        calculated_pec = pec10_calc(1, RX_DATA - 2u, ic_data);

        if(dev->spi_debug.enabled && (current_ic < ADBMS6830_MAX_TRACKED_ICS))
        {
            dev->spi_debug.last_cmd_counter[current_ic] = (uint8_t)cmd_cntr;
        }

        if (received_pec != calculated_pec)
        {
            rx_pec_error = 1u;
            adbms6830_note_pec_result(dev, current_ic, false);
            if(dev->spi_debug.enabled)
            {
                dev->spi_debug.last_read_pec_fail_mask |= (uint16_t)(1u << current_ic);
                dev->spi_debug.error_count++;
            }
        }
        else
        {
            rx_pec_error = 0u;
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
                                        (uint8_t)cmd_cntr,
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
    status = adbms6830_wakeup_checked(dev);
    return (status == HAL_OK) ? adbms6830_cmd_checked(dev, cmd) : status;
}

HAL_StatusTypeDef adbms6830_start_adc_cell_voltage_measurement(adbms6830_driver_t *dev)
{
    return adbms6830_adcv_checked(dev,
                                 RD_ON,
                                 CONTINUOUS,
                                 DCP_OFF,
                                 RSTF_OFF,
                                 OW_OFF_ALL_CH);
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

HAL_StatusTypeDef adbms6830_read_cell_voltages(adbms6830_driver_t *dev)
{
    uint8_t snap_cmd[2]   = { 0x00u, 0x2Du };
    uint8_t unsnap_cmd[2] = { 0x00u, 0x2Fu };
    static uint8_t *const commands[] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};
    static const GRP groups[] = {A, B, C, D, E, F};
    HAL_StatusTypeDef first_error = HAL_OK;
    HAL_StatusTypeDef status;

    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    for(uint8_t ic = 0u; ic < ADBMS6830_MAX_TRACKED_ICS; ic++)
    {
        dev->last_cell_updated_mask[ic] = 0u;
        dev->last_cell_pec_mask[ic] = 0u;
        dev->last_temp_updated_mask[ic] = 0u;
    }

    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, snap_cmd);
    }
    if(status == HAL_OK)
    {
        status = adbms6830_us_delay(dev, 10u);
    }

    if(status == HAL_OK)
    {
        for(uint8_t group = 0u; group < (uint8_t)(sizeof(groups) / sizeof(groups[0])); group++)
        {
            status = adbms6830_rd48_checked(dev, commands[group], shared_buf);
            if(status == HAL_OK)
            {
                adbms6830_parse_cell(dev, shared_buf, groups[group]);
                if(!adbms6830_last_read_integrity_ok(dev) && (first_error == HAL_OK))
                {
                    first_error = HAL_ERROR;
                }
            }
            else if(first_error == HAL_OK)
            {
                first_error = status;
            }
        }
    }
    else
    {
        first_error = status;
    }

    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, unsnap_cmd);
    }
    if((status != HAL_OK) && (first_error == HAL_OK))
    {
        first_error = status;
    }
    return first_error;
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
static HAL_StatusTypeDef adbms6830_gpio_i2c_write(adbms6830_driver_t *dev,
                                                   uint8_t slave_addr,
                                                   uint8_t data_byte,
                                                   uint16_t *ack_mask)
{
    HAL_StatusTypeDef status;
    uint16_t acknowledged = 0u;

    if(ack_mask != NULL)
    {
        *ack_mask = 0u;
    }
    if(!adbms6830_topology_valid(dev))
    {
        return HAL_ERROR;
    }

    /* Pack COMM register: 3 slots                                          *
     * Slot 0: START + address byte (write)                                 *
     * Slot 1: data byte                                                    *
     * Slot 2: STOP (data don't-care = 0xFF)                                */
    for (uint8_t curr_ic = 0u; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
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
    for (uint8_t curr_ic = 0u; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
    {
        for (uint8_t b = 0; b < TX_DATA; b++)
        {
            shared_buf[((uint16_t)curr_ic * TX_DATA) + b] = dev->ics[curr_ic].com.tx_data[b];
        }
    }

//    adbms6830_wakeup(dev);

    /* Write COMM register, then clock it out via STCOMM */
//    taskENTER_CRITICAL();
    status = adbms6830_wr48_checked(dev, WRCOMM, shared_buf);
    if(status != HAL_OK)
    {
        return status;
    }

    /* STCOMM: pulse CS low and clock 72 bits to push the I2C transaction. */
    if(dev->spi_debug.enabled)
    {
        dev->spi_debug.last_op = ADBMS6830_SPI_OP_STCOMM;
    }
    status = adbms6830_spi_write(dev, STCOMM, sizeof(STCOMM), 1u);
    if(status != HAL_OK)
    {
        return status;
    }
    adbms6830_note_counter_increment(dev);

    /* RDCOMM reports the I2C slave ACK/NACK result for every IC.  Require
     * both the address and data bytes to have been acknowledged. */
    status = adbms6830_rd48_checked(dev, RDCOMM, shared_buf);
    if(status != HAL_OK)
    {
        return status;
    }

    for(uint8_t curr_ic = 0u; curr_ic < (uint8_t)dev->num_ics; curr_ic++)
    {
        uint16_t bit = (uint16_t)(1u << curr_ic);
        const uint8_t *packet = &shared_buf[(uint16_t)curr_ic * RX_DATA];
        bool transport_valid = ((dev->health.last_pec_pass_mask & bit) != 0u) &&
                               ((dev->health.last_cmd_counter_mismatch_mask & bit) == 0u);
        bool address_ack = ((packet[0] >> 4u) == ICOMM_START_) &&
                           ((packet[0] & 0x0Fu) == 0x07u);
        bool data_ack = ((packet[2] >> 4u) == ICOMM_BLANK_) &&
                        ((packet[2] & 0x0Fu) == 0x07u);

        if(transport_valid && address_ack && data_ack)
        {
            acknowledged |= bit;
        }
    }

    if(ack_mask != NULL)
    {
        *ack_mask = acknowledged;
    }

    return (acknowledged == adbms6830_expected_ic_mask(dev)) ? HAL_OK : HAL_ERROR;
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
    if(adbms6830_us_delay(dev, 2000u) != HAL_OK)
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
    HAL_StatusTypeDef status;
    uint16_t ack_mask = 0u;

    if(!adbms6830_topology_valid(dev) ||
       (sensor_num >= ADBMS6830_TEMP_SENSOR_COUNT))
    {
        return -1;
    }

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

    return (status == HAL_OK) ? 0 : -1;
}


int mux_read_gpio_voltage(adbms6830_driver_t *dev, uint8_t sensor_num)
{
    HAL_StatusTypeDef status;
    uint16_t selected_mask = 0u;
    uint16_t updated_mask = 0u;
    uint32_t sensor_bit;

    if(!adbms6830_topology_valid(dev) ||
       (sensor_num >= ADBMS6830_TEMP_SENSOR_COUNT))
    {
        return -1;
    }

    uint8_t mux_idx = sensor_num / SENSORS_PER_MUX;
    uint8_t sw_pos = sensor_num % SENSORS_PER_MUX;
    uint8_t gpio_ch = GPIO_AUX_IDX[mux_idx];
    sensor_bit = (uint32_t)(1UL << sensor_num);

    for(uint8_t ic = 0u; ic < (uint8_t)dev->num_ics; ic++)
    {
        uint16_t bit = (uint16_t)(1u << ic);
        dev->last_temp_updated_mask[ic] &= ~sensor_bit;
        if(((dev->mux_selection_valid_mask[mux_idx] & bit) != 0u) &&
           (dev->mux_selected_channel[ic][mux_idx] == sw_pos))
        {
            selected_mask |= bit;
        }
    }

    if(selected_mask == 0u)
    {
        return -1;
    }

    /* Trigger ADC conversion on the current GPIO channel */
    uint8_t adax_cmd[2] = { ADAX_CMD_BYTE0, ADAX_CH[mux_idx] };
    status = adbms6830_wakeup_checked(dev);
    if(status == HAL_OK)
    {
        status = adbms6830_cmd_checked(dev, adax_cmd);
    }
    if(status != HAL_OK)
    {
        return -1;
    }
    if(adbms6830_us_delay(dev, 4000u) != HAL_OK)
    {
        return -1;
    }

    /* Read back and parse the auxiliary GPIO result */
    status = adbms6830_rd48_checked(dev, RDAUXA, shared_buf);
    if(status != HAL_OK)
    {
        return -1;
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

        if(((selected_mask & bit) != 0u) && transport_valid &&
           adbms6830_aux_code_valid(packet, gpio_ch))
        {
            uint8_t byte_lo = (uint8_t)(gpio_ch * 2u);
            int16_t raw = (int16_t)((uint16_t)packet[byte_lo] |
                                    ((uint16_t)packet[byte_lo + 1u] << 8u));
            dev->ics[ic].temp.raw[sensor_num] = raw;
            dev->last_temp_updated_mask[ic] |= sensor_bit;
            updated_mask |= bit;
        }
    }

    return (updated_mask == adbms6830_expected_ic_mask(dev)) ? 0 : -1;
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
