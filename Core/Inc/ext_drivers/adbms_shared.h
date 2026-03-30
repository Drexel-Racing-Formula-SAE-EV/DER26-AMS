/*
 * adbms_shared.h
 *
 *  Created on: Jun 9, 2025
 *      Author: cole
 */

#ifndef INC_EXT_DRIVERS_ADBMS_SHARED_H_
#define INC_EXT_DRIVERS_ADBMS_SHARED_H_

#include <stdint.h>

#define VR_SIZE 12              /*!< Bms ic number of Voltage Resister  */
#define RVR_SIZE 13             /* Bms ic number of Redundant Voltage Resister  */
#define COMM 3                  /*!< communication comm reg byte size   */
#define RSID 6                  /*!< Bms ic number of SID byte          */
#define CMDSZ 2
#define PEC15SZ 2
#define TX_DATA 6               /*!< Bms tx data byte                   */
#define RX_DATA 8               /*!< Bms rx data byte                   */
#define DPECSZ 2
#define RDALLI_SIZE    22       /* RDALLI data byte size              */
#define RDALLA_SIZE    22       /* RDALLA data byte size              */
#define RDALLC_SIZE    22       /* RDALLC data byte size              */
#define RDALLV_SIZE    22       /* RDALLV data byte size              */
#define RDALLR_SIZE    22       /* RDALLR data byte size              */
#define RDALLX_SIZE    22       /* RDALLX data byte size              */

#define ALLVR_SIZE     22       /* ALL Voltage Reg. byte size         */
#define ALLREDVR_SIZE  22       /* ALL Redundant Voltage Reg. byte size*/

#define WAKEUP_US_DELAY 250
#define WAKEUP_BW_DELAY 100
#define SPI_TIMEOUT 500
#define BUFSZ 512

#define CELL 16                 /* Bms ic number of cell              */
#define AUX  12                 /* Bms ic number of Aux               */
#define RAUX 10                 /* Bms ic number of RAux              */
#define PWMA 12                 /* Bms ic number of PWMA              */
#define PWMB 4                  /* Bms ic number of PWMB              */
#define COMM 3                  /* GPIO communication comm reg        */
#define RSID 6                  /* Bms ic number of SID byte          */
#define CMDSZ 2
#define PEC15SZ 2
#define DPECSZ 2
#define TX_DATA 6               /* Bms tx data byte                   */
#define RX_DATA 8               /* Bms rx data byte                   */
#define RDCVALL_SIZE    34      /* RDCVALL data byte size             */
#define RDSALL_SIZE     34      /* RDSALL data byte size              */
#define RDACALL_SIZE    34      /* RDACALL data byte size             */
#define RDFCALL_SIZE    34      /* RDFCALL data byte size             */
#define RDCSALL_SIZE    66      /* RDCSALL data byte size             */
#define RDASALL_SIZE    70      /* RDASALL data byte size             */
#define RDACSALL_SIZE   66      /* RDACSALL data byte size */

uint16_t Pec15_Calc(uint8_t len, uint8_t *data);
uint16_t pec10_calc(uint8_t rx_cmd, int len, uint8_t *data);
uint16_t pec10_calc_int(uint16_t remainder, uint8_t bit);
uint16_t pec10_calc_modular(uint8_t * data, uint8_t PEC_Format);

/*!
*  \enum Single & Round-Robin Measurement.
* VCH: Single & Round-Robin Measurement Cmd bytes.
*/
typedef enum
{
  SM_V1 = 0,
  SM_V2,
  SM_V3,
  SM_V4,
  SM_V5,
  SM_V6,
  SM_V7_V9,
  SM_V8_V10,
  SM_VREF2,
  RR_VCH0_VCH8,
  RR_VCH1_VCH3_VCH5,
  RR_VCH0_VCH5,
  RR_VCH0_VCH3,
  RR_VCH0_VCH2_VCH4,
  RR_VCH4_VCH7,
  RR_VCH5_VCH7
}VCH;

/*!
*  \enum OPT
* OPT: Continuous or single measurement and with or without diagnostic in ADI1, ADI2
*/
typedef enum
{
  OPT0_SS = 0,
  OPT1_INVALID,
  OPT2_INVALID,
  OPT3_INVALID,
  OPT4_SS,
  OPT5_SS_Diagn,
  OPT6_SS_Diagn,
  OPT7_SS_Diagn,
  OPT8_C,
  OPT9_C_Diagn,
  OPT10_C_Diagn,
  OPT11_C_Diagn,
  OPT12_C,
  OPT13_SS_Diagn,
  OPT14_SS_Diagn,
  OPT15_SS_Diagn
}OPT;

/*!
*  \enum OPEN WIRE
* OW: OPEN WIRE in ADV.
*/
typedef enum { OW_OFF = 0X0, OW_SRC_P_SNK_N, OW_SRC_N_SNK_P, OW_OFF_} OW;

/*!
*  \enum DIAGSEL
* DIAGSEL: Continuous or single measurement and with or without diagnostic in CFGB
*/
typedef enum
{
  DIAGSEL0_IAB_VBAT = 0,
  DIAGSEL1_OW_IxA_IAB_OW_VBATP_VBAT,
  DIAGSEL2_OW_IxB_IAB_OW_VBATM_VBAT,
  DIAGSEL3_OW_SxA_IAS_VBAT,
  DIAGSEL4_IAS_SGND,
  DIAGSEL5_118_118,
  DIAGSEL6_M125_2375,
  DIAGSEL7_OW_IxB_IAS_VBAT
}DIAGSEL;

/*!
*  \enum ERR
* ERR: Inject error is spi read out.
*/
/* Inject error is spi read out */
typedef enum  { WITHOUT_ERR = 0x0, WITH_ERR = 0x1 } ERR;

/*!
*  \enum RD
* RD: Read Device.
*/
typedef enum { RD_OFF = 0X0, RD_ON = 0X1} RD;


/**************************************** Mem bits *************************************************/
/*!< Configuration Register A */
/*!
*  \enum OCEN
* OCEN: OC ADC Enable.
*/
/* OC ADC Enable */
typedef enum  { OC_DISABLE = 0x0, OC_ENABLE = 0x1 } OCEN;

/*!
*  \enum SOAK
* SOAK: Enables soak on V- ADCs in CFGA
*/
typedef enum
{
  SOAK_DISABLE = 0,
  SOAK_100us,
  SOAK_500us,
  SOAK_1ms,
  SOAK_2ms,
  SOAK_10ms,
  SOAK_20ms,
  SOAK_150ms
}SOAK;

/*!
*  \enum INJOSC
* INJOSC:  Inject Oscillator Monitor Check in CFGA
*/
typedef enum
{
  INJOSC0_NORMAL = 0,
  INJOSC1_OSC_FAST,
  INJOSC2_OSC_SLOW_NOCLK_HIGH,
  INJOSC3_NOCLK_LOW
}INJOSC;

/*!
*  \enum INJMON
* INJMON:  Inject Supply Monitor Check in CFGA
*/
typedef enum
{
  INJMON0_NORMAL = 0,
  INJMON1_DEGLITCHER_MISMATCH,
  INJMON2_UV,
  INJMON3_OV_DE
}INJMON;

/*!
*  \enum INJTS
* INJTS:  Inject Thermal Shutdown Monitor Check in CFGA
*/
typedef enum  { NO_THSD = 0x0, FORCE_THSD = 0x1 } INJTS;

/*!
*  \enum INJECC
* INJECC:  Inject ECC (Fuse single ED, Multiple ED) Check in CFGA
*/
typedef enum  { NO_ECC = 0x0, FORCE_ECC = 0x1 } INJECC;

/*!
*  \enum INJTM
* INJTM:  Inject TMODE Check in CFGA
*/
typedef enum  { NO_TMODE = 0x0, FORCE_TMODE = 0x1 } INJTM;

/*!
*  \enum GPOxC
* GPOxC:  Selecting between Pull-down and Pull-up/Tristated Mode in CFGA
*/
typedef enum  { PULLED_DOWN = 0x0, PULLED_UP_TRISTATED = 0x1 } GPOxC;

/*!
*  \enum GPOxOD_OPOD
* GPOxOD_OPOD:  Selecting between Push-Pull and Open-Drain Mode in CFGA and CFGB
*/
typedef enum  { PUSH_PULL = 0x0, OPEN_DRAIN = 0x1 } GPOxOD_OPOD;

/*!
*  \enum GPIO1FE
* GPIO1FE:  Selecting between GPIO1C/SPIM and Fault-Pin Mode in CFGA
*/
typedef enum  { FAULT_STATUS_DISABLE = 0x0, FAULT_STATUS_ENABLE = 0x1 } GPIO1FE;

/*!
*  \enum SPI3W
* SPI3W:  Selecting between 4-wire and 3-wire Mode in CFGA
*/
typedef enum  { FOUR_WIRE = 0x0, THREE_WIRE = 0x1 } SPI3W;

/*!
*  \enum ACCI
* ACCI:  Accumulation Modes for Battery Current and Battery Voltage Accumulation Registers in CFGA
*/
typedef enum
{
  ACCI_4 = 0,
  ACCI_8,
  ACCI_12,
  ACCI_16,
  ACCI_20,
  ACCI_24,
  ACCI_28,
  ACCI_32
}ACCI;

/*!
*  \enum CL FLAG
* CL FLAG: Fault clear bit set or clear enum
*/
typedef enum  { CL_FLAG_CLR = 0x0, CL_FLAG_SET = 0x1 } CL_FLAG;

/*!
*  \enum COMM_BK
* COMM_BK: Communication Break.
*/
typedef enum  { COMMBK_OFF = 0x0, COMMBK_ON = 0x1 } COMMBK;

/*!
*  \enum SNAPSHOT
* SNAPSHOT: Snapshot.
*/
typedef enum  { SNAP_OFF = 0x0, SNAP_ON = 0x1 } SNAPSHOT;

/*!
*  \enum VBxMUX
* VBxMUX: VBxADC Negative Input Channel Selection.
*/
typedef enum  { SINGLE_ENDED_SGND = 0x0, DIFFERENTIAL = 0x1 } VBxMUX;

/* Configuration Register B */
/*!
*  \enum OCxTEN_REFTEN
* OCxTEN_REFTEN: OCxADC and REFADC Measurement Mode Selection between input signal and reference input
*/
typedef enum  { NORMAL_INPUT = 0x0, REFERENCE_INPUT = 0x1 } OCxTEN_REFTEN;

/*!
*  \enum OCDGT
* OCDGT: OC Deglitch Mode Selection.
*/
typedef enum
{
  OCDGT0_1oo1 = 0,
  OCDGT1_2oo3,
  OCDGT2_4oo8,
  OCDGT3_7oo8
} OCDGT;

/*!
*  \enum OCDP
* OCDP: Diagnostic Mode Selection between normal mode and fast mode for latent fault check
*/
typedef enum  { OCDP0_NORMAL = 0x0, OCDP1_FAST = 0x1 } OCDP;

/*!
*  \enum OCTSEL
* OCTSEL: Reference Input Mode Selection for OCxADC and REFADC.
*/
typedef enum
{
  OCTSEL0_OCxADC_P140_REFADC_M20 = 0,
  OCTSEL1_OCxADC_P293_REFADC_P20,
  OCTSEL2_OCxADC_M140_REFADC_M20,
  OCTSEL3_OCxADC_M293_REFADC_P20
} OCTSEL;

/*!
*  \enum OCxGC
* OCxGC:  Selecting between Gain 1 and Gain 2 in CFGB
*/
typedef enum  { GAIN_1 = 0x0, GAIN_2 = 0x1 } OCxGC;

/*!
*  \enum OCMODE
* OCMODE: OC ADC Mode.
*/
typedef enum
{
  OCMODE0_DISABLED = 0,
  OCMODE1_PWM1,
  OCMODE2_PWM2,
  OCMODE3_STATIC
} OCMODE;

/*!
*  \enum OCABX
* OCABX:  Polarity Selection for OC Outputs in CFGB
*/
typedef enum  { OCABX_ACTIVE_HIGH = 0x0, OCABX_ACTIVE_LOW = 0x1 } OCABX;

/*!
*  \enum GPIO2EOC
* GPIO2EOC:  Selection of OC1 EOC feature for GPIO2 in CFGB
*/
typedef enum  { EOC_DISABLED = 0x0, EOC_ENABLED = 0x1 } GPIO2EOC;

/*!
*  \enum GPIOxC
* GPIOxC:  Selection between Pull-down On and Off for GPIO in CFGB
*/
typedef enum  { PULL_DOWN_ON = 0x0, PULL_DOWN_OFF = 0x1 } GPIOxC;

/*!
*  \enum GPIO
* GPIO: GPIO Pins.
*/
typedef enum
{
  GPIO1 = 0,
  GPIO2,
  GPIO3,
  GPIO4,
} GPIO;

/*!
*  \enum GPIO
* GPIO: GPIO Pin Control.
*/
typedef enum  { GPIO_CLR = 0x0, GPIO_SET = 0x1 } CFGA_GPIO;

/*!
*  \enum GPO
* GPO: GPO Pins.
*/
/* GPO Pins */
typedef enum
{
  GPO1 = 0,
  GPO2,
  GPO3,
  GPO4,
  GPO5,
  GPO6,
  GPO7,
  GPO8,
  GPO9,
  GPO10,
} GPO;

/*!
*  \enum GPO
* GPIO: GPO Pin Control.
*/
/* GPO Pin Control */
typedef enum  { GPO_CLR = 0x0, GPO_SET = 0x1 } CFGA_GPO;

/*!
*  \enum Reference Voltage for VSx (x=3 to 10) measurement
* VSx: Reference Voltage for VSx measurement.
*/
typedef enum  { VSMV_SGND = 0x0, VSMV_VREF1P25 = 0x1 } VSB;

/*!
*  \enum Reference Voltage for VSx[1:0] Measurement
* VS[1:0]: Reference Voltage for VSx[1:0] Measurement.
*/
typedef enum
{
  VSM_SGND = 0,
  VSM_VREF1P25,
  VSM_V3,
  VSM_V4
} VS;

/* General Enums */
/*!
*  \enum Group
* GRP: Group/Groups of register to which data corresponds
*/
//typedef enum { ALL_GRP = 0x0, A, B, C, D, E, F, CERR, CD, NONE} GRP;
typedef enum { ALL_GRP = 0x0, A, B, C, D, E, F, FLAG_NOERR, FLAG_ERR, FLAGD, NONE} GRP;

/*!
*  \enum DIAGNOSTIC_TYPE
* DIAGNOSTIC_TYPE: type of diagnostic check
*/
typedef enum { OSC_MISMATCH = 0x0, SUPPLY_ERROR, THSD_DGN, FUSE_ED, FUSE_MED, TMODCHK_DGN} DIAGNOSTIC_TYPE;
/*!
*  \enum LOOP_MEASURMENT
* LOOP_MEASURMENT: enabled / disabled
*/
typedef enum { DISABLED = 0X0, ENABLED = 0X1} LOOP_MEASURMENT;
/*!
*  \enum RESULT
* RESULT: pass or fail
*/
typedef enum { FAIL = 0x0, PASS } RESULT ;
/*!
*  \enum COMMANDS
* COMMANDS: type of ADC convesion command
*/
//typedef enum {ADI1=0x0, ADI2=0x1, ADV=0x2, ADAUX=0x3} COMMANDS;
typedef enum {ADI1=0x0, ADI2=0x1, ADV=0x2} COMMANDS;

/*!
*  \enum COMM_MODE
* COMM_MODE: master spi/i2c mode selector
*/
typedef enum
{
  COMM_SPI3=0,
  COMM_SPI4,
  COMM_I2C
} COMM_MODE;

/*!
*  \enum DIAGN_FLAG_POS
* DIAGN_FLAG_POS: enum for diagnostic flags positions
*/
typedef enum
{
  I1D               = 0x80000000U,
  V1D               = 0x40000000U,
  VDRUV             = 0x20000000U,
  OCMM              = 0x10000000U,
  OCAGD_CLRM        = 0x04000000U,

  I2D               = 0x00800000U,
  V2D               = 0x00400000U,
  VDDUV             = 0x00200000U,
  NOCLK             = 0x00100000U,
  REFFLT            = 0x00080000U,
  OCBGD             = 0x00040000U,

  VREGOV            = 0x00008000U,
  VREGUV            = 0x00004000U,
  VDIGOV            = 0x00002000U,
  VDIGUV            = 0x00001000U,
  SED1              = 0x00000800U,
  MED1              = 0x00000400U,
  SED2              = 0x00000200U,
  MED2              = 0x00000100U,

  VDEL              = 0x00000080U,
  VDE               = 0x00000040U,
  SPIFLT            = 0x00000010U,
  RESET_DIAGN_FLAG  = 0x00000008U,
  THSD              = 0x00000004U,
  TMODE             = 0x00000002U,
  OSCFLT            = 0x00000001U
} DIAGN_FLAG_POS;
/*!
*  \enum OSC_CNTR_MODE
* OSC_CNTR_MODE: mode of oscillator
*/
typedef enum{ NORMAL=0, FAST=1, SLOW=2 }OSC_CNTR_MODE;

/**
  * @brief Status structures definition
  */
typedef enum
{
  UART_OK       = 0x00U,
  UART_ERROR    = 0x01U,
  UART_BUSY     = 0x02U,
  UART_TIMEOUT  = 0x03U
} UART_STATUS;
/*!
*  \enum STAT_CHK
* STAT_CHK: Determines the type of check have to be done on status registers
*/
typedef enum {DEFAULT, ALL_SET, ALL_CLEAR, DIAGN, ALL_RESET, OCxR_RESET_OCMAXMIN_CLEAR, VDD_LV, GPIO1_CLEAR, GPIO1_SET} STAT_CHK ;

/*!
*  \enum CMD_CNTR_CHK
* CMD_CNTR_CHK: Command counter set/reset values
*/
typedef enum {CMD_CNTR_SET=1, CMD_CNTR_RESET = 0  }CMD_CNTR_CHK;
/*!
*  \enum EXTREMES
* EXTREMES: MIN or MAX
*/
typedef enum { MIN = 0x0, MAX } EXTREMES ;

/* PEC_Format*/
/*!
    \enum                PEC_Format
    \brief               Enum PEC Format.
*/
typedef enum
{
  PEC10_WRITE,
  PEC10_READ,
  PEC10_READ256,
  PEC10_READ512,
  PEC10_WRITE2,
  PEC10_READ2,
}PEC_Format;

/*!< ADBMS2950 Configuration Register Group A structure */
typedef struct
{
  uint8_t       ocen      :1;
  uint8_t       vs5       :1;
  uint8_t       vs4       :1;
  uint8_t       vs3       :1;
  uint8_t       vs2       :2;
  uint8_t       vs1       :2;

  uint8_t       injtm     :1;
  uint8_t       injecc    :1;
  uint8_t       injts     :1;
  uint8_t       injmon    :2;
  uint8_t       injosc    :2;

  uint8_t       soak      :3;
  uint8_t       vs10      :1;
  uint8_t 	vs9       :1;
  uint8_t 	vs8       :1;
  uint8_t       vs7       :1;
  uint8_t       vs6       :1;

  uint8_t       gpo6c     :2;
  uint8_t       gpo5c     :1;
  uint8_t       gpo4c     :1;
  uint8_t       gpo3c     :1;
  uint8_t       gpo2c     :1;
  uint8_t       gpo1c     :1;

  uint8_t       spi3w     :1;
  uint8_t       gpio1fe   :1;
  uint8_t       gpo6od    :1;
  uint8_t       gpo5od    :1;
  uint8_t       gpo4od    :1;
  uint8_t       gpo3od    :1;
  uint8_t       gpo2od    :1;
  uint8_t       gpo1od    :1;

  uint8_t       vb2mux    :1;
  uint8_t       vb1mux    :1;
  uint8_t       snapst    :1;
  uint8_t       refup     :1;
  uint8_t       commbk    :1;
  uint8_t       acci      :3;
}cfa_;

/*!< ADBMS2950 Configuration Register Group B structure */
typedef struct
{
  uint8_t 	oc1ten    :1;
  uint8_t 	oc1th     :7;

  uint8_t 	oc2ten    :1;
  uint8_t 	oc2th     :7;

  uint8_t 	oc3ten    :1;
  uint8_t 	oc3th     :7;

  uint8_t 	octsel    :2;
  uint8_t 	reften    :1;
  uint8_t 	ocdp      :1;
  uint8_t 	ocdgt     :2;

  uint8_t 	ocbx      :1;
  uint8_t 	ocax      :1;
  uint8_t 	ocmode    :2;
  uint8_t 	oc3gc     :1;
  uint8_t 	oc2gc     :1;
  uint8_t 	oc1gc     :1;
  uint8_t 	ocod      :1;

  uint8_t       gpio4c    :1;
  uint8_t       gpio3c    :1;
  uint8_t       gpio2c    :1;
  uint8_t       gpio1c    :1;
  uint8_t       gpio2eoc  :1;
  uint8_t       diagsel   :3;
}cfb_;

/* ADBMS2950 Flag Register Data structure*/
typedef struct
{
  uint8_t  i1d          :1;
  uint8_t  v1d          :1;
  uint8_t  vdruv        :1;
  uint8_t  ocmm         :1;
  uint8_t  oc3l         :1;
  uint8_t  ocagd_clrm   :1;
  uint8_t  ocal         :1;
  uint8_t  oc1l         :1;

  uint8_t  i2d          :1;
  uint8_t  v2d          :1;
  uint8_t  vdduv        :1;
  uint8_t  noclk        :1;
  uint8_t  refflt       :1;
  uint8_t  ocbgd        :1;
  uint8_t  ocbl         :1;
  uint8_t  oc2l         :1;

  uint8_t  i2cnt        :3;
  uint16_t  i1cnt       :11;
  uint8_t  i1pha        :2;

  uint8_t  vregov       :1;
  uint8_t  vreguv       :1;
  uint8_t  vdigov       :1;
  uint8_t  vdiguv       :1;
  uint8_t  sed1         :1;
  uint8_t  med1         :1;
  uint8_t  sed2         :1;
  uint8_t  med2         :1;

  uint8_t  vdel         :1;
  uint8_t  vde          :1;
  uint8_t  spiflt       :1;
  uint8_t  reset        :1;
  uint8_t  thsd         :1;
  uint8_t  tmode        :1;
  uint8_t  oscflt       :1;
} flag_;

/*!< ADBMS2950 Current Register Data structure */
typedef struct
{
  uint32_t i1;
  uint32_t i2;
} crg_;

/*!< ADBMS2950 Battery Voltage Register Data structure */
typedef struct
{
  uint16_t vbat1;
  uint16_t vbat2;
} vbat_;

/*!< ADBMS2950 Current and Battery Voltage Register Data structure */
typedef struct
{
  uint32_t i1;
  uint16_t vbat1;
} i_vbat_;

/*!< ADBMS2950 Overcurrent ADC Register Data structure */
typedef struct
{
  uint8_t oc1r;
  uint8_t oc2r;
  uint8_t oc3r;
  uint8_t refr;
  uint8_t oc3max;
  uint8_t oc3min;
} oc_;

/*!< ADBMS2950 Accumulated Current Register Data structure */
typedef struct
{
  uint32_t i1acc;
  uint32_t i2acc;
} iacc_;

/*!< ADBMS2950 Accumulated Battery Voltage Register Data structure */
typedef struct
{
  uint32_t vb1acc;
  uint32_t vb2acc;
} vbacc_;

/*!< ADBMS2950 Accumulated Battery Current and Voltage Register Data structure */
typedef struct
{
  uint32_t i1acc;
  uint32_t vb1acc;
} i_vbacc_;

/*!< ADBMS2950 Voltage Register Data structure */
typedef struct
{
  uint16_t v_codes[VR_SIZE];
} vr_;

/* ADBMS2950 Redundant Voltage Register Data structure */
typedef struct
{
  uint16_t redv_codes[RVR_SIZE];
} rvr_;

/* ADBMS2950 Aux ADC A Register Data structure */
typedef struct
{
  uint16_t vref1p25;
  uint16_t tmp1;
  uint16_t vreg;
} auxa_;

/* ADBMS2950 Aux ADC B Register Data structure */
typedef struct
{
  uint16_t vdd;
  uint16_t vdig;
  uint16_t epad;
}auxb_;

/* ADBMS2950 Aux ADC C Register Data structure */
typedef struct
{
  uint16_t vdiv;
  uint16_t tmp2;
  uint8_t osccnt;
} auxc_;

/* ADBMS2950 RDALLI Data structure */
typedef struct
{
  // IxADC
  uint32_t i1;
  uint32_t i2;

  // VBxDC
  uint16_t vb1;
  uint16_t vb2;

  // OCxR
  uint8_t oc1r;
  uint8_t oc2r;
  uint8_t oc3r;

  // STATUS
  uint8_t stat3;

  // FLAG
  uint8_t flag0;
  uint8_t flag1;
  uint8_t flag2;
  uint8_t flag3;
  uint8_t flag4;
  uint8_t flag5;
} rdalli_;

/* ADBMS2950 RDALLA Data structure */
typedef struct
{
  // IxADC
  uint32_t i1acc;
  uint32_t i2acc;

  // VBxDC
  uint32_t vb1acc;
  uint32_t vb2acc;

  // STATUS
  uint8_t stat3;
  uint8_t stat4;

  // FLAG
  uint8_t flag0;
  uint8_t flag1;
  uint8_t flag2;
  uint8_t flag3;
  uint8_t flag4;
  uint8_t flag5;
} rdalla_;

/* ADBMS2950 RDALLC Data structure */
typedef struct
{
  // CFGA
  uint8_t cfga0;
  uint8_t cfga1;
  uint8_t cfga2;
  uint8_t cfga3;
  uint8_t cfga4;
  uint8_t cfga5;

  // CFGB
  uint8_t cfgb0;
  uint8_t cfgb1;
  uint8_t cfgb2;
  uint8_t cfgb3;
  uint8_t cfgb4;
  uint8_t cfgb5;

  // STATUS
  uint8_t stat3;
  uint8_t stat4;

  // FLAG
  uint8_t flag0;
  uint8_t flag1;
  uint8_t flag2;
  uint8_t flag3;
  uint8_t flag4;
  uint8_t flag5;
} rdallc_;

/* ADBMS2950 RDALLV Data structure */
typedef struct
{
  uint16_t v1;
  uint16_t v2;
  uint16_t v3;
  uint16_t v4;
  uint16_t v5;
  uint16_t v6;
  uint16_t v7;
  uint16_t v8;
  uint16_t v9;
  uint16_t v10;
} rdallv_;

/* ADBMS2950 RDALLR Data structure */
typedef struct
{
  uint16_t v1;
  uint16_t v2;
  uint16_t v3;
  uint16_t v4;
  uint16_t v5;
  uint16_t v6;
  uint16_t v7;
  uint16_t v8;
  uint16_t v9;
  uint16_t v10;
} rdallr_;

/* ADBMS2950 RDALLX Data structure */
typedef struct
{
  uint16_t vref2A;
  uint16_t vref2B;
  uint16_t vref1p25;
  uint16_t tmp1;
  uint16_t vreg;
  uint16_t vdd;
  uint16_t vdig;
  uint16_t epad;
  uint16_t vdiv;
  uint16_t tmp2;
} rdallx_;

/* ADBMS2950 Status Register Data structure */
typedef struct
{
  uint8_t ocap   :1;
  uint8_t ocbp   :1;
  uint8_t der    :2;
  uint8_t i1cal  :1;
  uint8_t i2cal  :1;

  uint8_t gpo1l  :1;
  uint8_t gpo2l  :1;
  uint8_t gpo3l  :1;
  uint8_t gpo4l  :1;
  uint8_t gpo5l  :1;
  uint8_t gpo6l  :1;

  uint8_t gpo1h  :1;
  uint8_t gpo2h  :1;
  uint8_t gpo3h  :1;
  uint8_t gpo4h  :1;
  uint8_t gpo5h  :1;
  uint8_t gpo6h  :1;

  uint8_t gpio1l  :1;
  uint8_t gpio2l  :1;
  uint8_t gpio3l  :1;
  uint8_t gpio4l  :1;

  uint8_t rev   :4;
} state_;


/* ADBMS2950 tm48 Data structure */
typedef struct
{
  uint16_t redv_codes[RVR_SIZE];
} tm48_;


/* ADBMS2950 COMM register Data structure*/
typedef struct
{
  uint8_t fcomm[COMM];
  uint8_t icomm[COMM];
  uint8_t data[COMM];
} com_;

/*!< ADBMS2950 SID Register Structure */
typedef struct
{
  uint8_t sid[RSID];
} sid_;

/*!< Transmit byte and recived byte data structure */
typedef struct
{
  uint8_t tx_data[TX_DATA];
  uint8_t rx_data[RX_DATA];
} ic_register_;

/*!< Command counter and pec error data Structure */
typedef struct
{
  uint8_t cmd_cntr;
  uint8_t cal_cmd_cntr;
  uint8_t cc_error;
  uint8_t cfgr_pec;
  uint8_t cr_pec;
  uint8_t vbat_pec;
  uint8_t ivbat_pec;
  uint8_t oc_pec;
  uint8_t avgcr_pec;
  uint8_t avgvbat_pec;
  uint8_t avgivbat_pec;
  uint8_t aux_pec;
  uint8_t flag_pec;
  uint8_t vr_pec;
  uint8_t rvr_pec;
  uint8_t comm_pec;
  uint8_t stat_pec;
  uint8_t sid_pec;
  uint8_t tm48_pec;
  uint8_t rdalli_pec;
  uint8_t rdallv_pec;
  uint8_t rdallr_pec;
  uint8_t rdallc_pec;
  uint8_t rdalla_pec;
  uint8_t rdallx_pec;
} cmdcnt_pec_;

/*!< Diagnostic test result data structure */
typedef struct
{
  uint8_t osc_mismatch;
  uint8_t supply_error;
  uint8_t supply_ovuv;
  uint8_t thsd;
  uint8_t fuse_ed;
  uint8_t fuse_med;
  uint8_t tmodchk;
} diag_test_;

/* hold loop manager variables */
typedef struct
{
  uint8_t loop_measurment_count;      /* Loop measurment count (default count)*/
  uint8_t loop_measurment_time;        /* milliseconds(mS)*/
  uint8_t loop_count;
} loop_manager_2950_t;

typedef struct{
  uint32_t pladc_count;
  const uint32_t ITER_LOOP_MEASUREMENT_COUNT;      /* Loop measurment count */
  const uint16_t MEASUREMENT_LOOP_TIME;     /* milliseconds(mS)*/
} pladc_manager_t;

typedef struct
{
  VCH   VOLTAGE_MEASUREMENT;
  RD    REDUNDANT_MEASUREMENT;
  //ACH   AUX_CH_TO_CONVERT       = ALL;
  //CONT  CONTINUOUS_MEASUREMENT  = SINGLE;
  OW    OW_WIRE_DETECTION;
  ERR   INJECT_ERR_SPI_READ;
} adc_configuration_t;

typedef enum
{
	STRING_A = 0,
	STRING_B
} adbms_string;

/* ADI1 command parameters structure */
typedef struct
{
  uint8_t rd;
  uint8_t opt; //4 bits
}adi1_;

/* ADI2 command parameters structure */
typedef struct
{
  uint8_t opt;//4 bits
}adi2_;

/* ADV command parameters structure */
typedef struct
{
  uint8_t  ow;
  uint8_t ch;
}adv_;

typedef struct
{
  uint8_t cmd[2];
} cmd_description;

/*!
    \struct     adi1Struct
    \brief      Structure defining ADI1 command
*/
typedef struct
{
  uint8_t             RD;
  uint8_t             CONT;
}adi1Struct;

/*!
\struct     adi2Struct
\brief      Structure defining ADI2 command
*/
typedef struct
{
  uint8_t             CONT;
}adi2Struct;

/////////////////////////// 6830

/*!
*  \enum GPIO CHANNEL
* CH: GPIO Channels.
*/
/* Channel selection */
typedef enum
{
  AUX_ALL = 0,
  GPIO1_CH,
  GPIO2_CH,
  GPIO3_CH,
  GPIO4_CH,
  GPIO5,
  GPIO6,
  GPIO7,
  GPIO8,
  GPIO9,
  GPIO10,
  VREF2,
  LD03V,
  LD05V,
  TEMP,
  V_POSTIVE_2_NAGATIVE,
  V_NAGATIVE,
  VR4K,
  VREF3
}CH;

/*!
*  \enum CONT
* CONT: Continuous or single measurement.
*/
/* Continuous or single measurement */
typedef enum { SINGLE = 0X0, CONTINUOUS = 0X1} CONT;

/*!
*  \enum OW_C_S
* OW_C_S: Open wire c/s.
*/
/* Open wire c/s adcs */
typedef enum { OW_OFF_ALL_CH = 0X0, OW_ON_EVEN_CH, OW_ON_ODD_CH, OW_ON_ALL_CH} OW_C_S;

/*!
*  \enum OW_AUX
* OW_AUX: Open wire Aux.
*/
/* Open wire AUX */
typedef enum { AUX_OW_OFF = 0X0, AUX_OW_ON = 0X1} OW_AUX;

/*!
*  \enum PUP
* PUP: Pull Down current during aux conversion.
*/
/* Pull Down current during aux conversion (if OW = 1) */
typedef enum { PUP_DOWN = 0X0, PUP_UP = 0X1 } PUP;

/*!
*  \enum DCP
* DCP: Discharge permitted.
*/
/* Discharge permitted */
typedef enum { DCP_OFF = 0X0, DCP_ON = 0X1} DCP;

/*!
*  \enum RSTF
* RSTF: Reset Filter.
*/
/* Reset filter */
typedef enum  { RSTF_OFF = 0x0, RSTF_ON = 0x1 } RSTF;

/**************************************** Mem bits *************************************************/
/* Configuration Register A */

/*!
*  \enum REFON
* REFON: Refernece remains power up/down.
*/
/* Refernece remains power up/down */
typedef enum  { PWR_DOWN = 0x0, PWR_UP = 0x1 } REFON;

/*!
*  \enum CTH
* CTH: Comparison voltages threshold C vs S.
*/
/* Comparison voltages threshold C vs S*/
typedef enum
{
  CVT_5_1mV = 0,        /* 5.1mV                */
  CVT_8_1mV,            /* 8.1mV (Default)      */
  CVT_10_05mV,          /* 10.05mV              */
  CVT_15mV,             /* 15mV                 */
  CVT_22_5mV,           /* 22.5mV               */
  CVT_45mV,             /* 45mV                 */
  CVT_75mV,             /* 75mV                 */
  CVT_135mV,            /* 135mV                */
}CTH;

/*!
*  \enum FLAG_D
* FLAG_D: Fault flags.
*/
/* Fault flags */
typedef enum
{
  FLAG_D0 = 0,          /* Force oscillator counter fast */
  FLAG_D1,              /* Force oscillator counter slow */
  FLAG_D2,              /* Force Supply Error detection  */
  FLAG_D3,              /* FLAG_D[3]: 1--> Select Supply OV and delta detection, 0 --> Selects UV */
  FLAG_D4,              /* Set THSD */
  FLAG_D5,              /* Force Fuse ED */
  FLAG_D6,              /* Force Fuse MED */
  FLAG_D7,              /* Force TMODCHK  */
} FLAG_D;

typedef enum  { FLAG_CLR = 0x0, FLAG_SET = 0x1 } CFGA_FLAG;

/*!
*  \enum SOAKON
* SOAKON: Enables or disable soak time for all commands.
*/
/* Enables or disable soak time for all commands */
typedef enum  { SOAKON_CLR = 0x0, SOAKON_SET = 0x1 } SOAKON;


/* Open wire sokon time owa */
typedef enum  {OWA0 = 0x0, OWA1, OWA2, OWA3, OWA4, OWA5, OWA6, OWA7} OWA;

/*!
*  \enum OWRNG
* OWRNG: Set soak time range Long/Short.
*/
/* Set soak time range Long/Short */
typedef enum  { SHORT = 0x0, LONG = 0x1 } OWRNG;


/*!
*  \enum OW_TIME
* OW_TIME:Open Wire Soak times
*          For Aux commands. If OWRNG=0, Soak time = 2^(6 +OWA[2:0]) Clocks (32 us 4.1 ms)
*          For Aux commands. If OWRNG=1, Soak time = 2^(13+OWA[2:0]) Clocks (41 ms 524 ms)
*/
typedef enum  { TIME_32US_TO_4_1MS = 0x0, TIME_41MS_TO_524MS = 0x1 } OW_TIME;

/*!
*  \enum IIR_FPA
* IIR_FPA: IIR Filter Parameter.
*/
/* IIR Filter Parameter */
typedef enum
{
  IIR_FPA_OFF = 0,              /* Filter Disabled          */
  IIR_FPA2,                     /* 110   Hz -3dB Frequency  */
  IIR_FPA4,                     /* 45    Hz -3dB Frequency  */
  IIR_FPA8,                     /* 21    Hz -3dB Frequency  */
  IIR_FPA16,                    /* 10    Hz -3dB Frequency  */
  IIR_FPA32,                    /* 5     Hz -3dB Frequency  */
  IIR_FPA128,                   /* 1.25  Hz -3dB Frequency  */
  IIR_FPA256,                   /* 0.625 Hz -3dB Frequency  */
}IIR_FPA;

/*!
*  \enum COMM_BK
* COMM_BK: Communication Break.
*/
/* Communication Break */
typedef enum  { COMM_BK_OFF = 0x0, COMM_BK_ON = 0x1 } COMM_BK;


/* Configuration Register B */

/*!
*  \enum DTMEN
* DTMEN: Enable Dis-charge Timer Monitor.
*/
/* Enable Dis-charge Timer Monitor */
typedef enum  { DTMEN_OFF = 0x0, DTMEN_ON = 0x1 } DTMEN;

/*!
*  \enum DTRNG
* DTRNG: Discharge Timer Range Setting.
*/
/* Discharge Timer Range Setting */
typedef enum  { RANG_0_TO_63_MIN = 0x0, RANG_0_TO_16_8_HR = 0x1 } DTRNG;

/*!
*  \enum DCTO
* DCTO: DCTO timeout values.
*/
typedef enum
{
  DCTO_TIMEOUT = 0,
  TIME_1MIN_OR_0_26HR,
  TIME_2MIN_OR_0_53HR,
  TIME_3MIN_OR_0_8HR,
  TIME_4MIN_OR_1_06HR,
  TIME_5MIN_OR_1_33HR,
  TIME_6MIN_OR_1_6HR,
  /* If required more time out value add here */
} DCTO;

/*!
*  \enum PWM
* PWM: PWM Duty cycle.
*/
typedef enum
{
  PWM_0_0_PCT = 0,      /* 0.0%  (default) */
  PWM_6_6_PCT,          /* 6.6%            */
  PWM_13_2_PCT,         /* 13.2%           */
  PWM_19_8_PCT,         /* 19.8%           */
  PWM_26_4_PCT,         /* 26.4%           */
  PWM_33_0_PCT,         /* 33.0%           */
  PWM_39_6_PCT,         /* 39.6%           */
  PWM_46_2_PCT,         /* 46.2%           */
  PWM_52_8_PCT,         /* 52.8%           */
  PWM_59_4_PCT,         /* 59.4%           */
  PWM_66_0_PCT,         /* 66.0%           */
  PWM_72_6_PCT,         /* 72.6%           */
  PWM_79_2_PCT,         /* 79.2%           */
  PWM_85_8_PCT,         /* 85.8%           */
  PWM_92_4_PCT,         /* 92.4%           */
  PWM_100_0_PCT,        /* ~100.0%         */
} PWM_DUTY;

/*!
*  \enum DCC
* DCC: DCC bits.
*/
/* DCC bits */
typedef enum
{
  DCC1 = 0,
  DCC2,
  DCC3,
  DCC4,
  DCC5,
  DCC6,
  DCC7,
  DCC8,
  DCC9,
  DCC10,
  DCC11,
  DCC12,
  DCC13,
  DCC14,
  DCC15,
  DCC16,
} DCC;

/*!
*  \enum DCC_BIT
* DCC_BIT: Discharge cell set and claer.
*/
/* Discharge cell set and claer  */
typedef enum  { DCC_BIT_CLR = 0x0, DCC_BIT_SET = 0x1 } DCC_BIT;

/* For ADBMS6830 config register structure */
typedef struct
{
  uint8_t       refon;
  uint8_t       cth;
  uint8_t       flag_d;
  uint8_t       soakon;
  uint8_t       owrng;
  uint8_t       owa;
  uint16_t      gpo;
  uint8_t       snap;
  uint8_t       mute_st;
  uint8_t       comm_bk;
  uint8_t       fc;
}cfa6830_;

/* For ADBMS6830 config register structure */
typedef struct
{
  uint16_t  vuv;
  uint16_t  vov;
  uint8_t   dtmen;
  uint8_t   dtrng;
  uint8_t   dcto;
  uint16_t  dcc;
}cfb6830_;

/* Cell Voltage Data structure */
typedef struct
{
  int16_t c_codes[CELL]; /* Cell Voltage Codes */
} cv_;

typedef struct
{
  int16_t ac_codes[CELL]; /* Average Cell Voltage Codes */
} acv_;

/* S Voltage Data structure */
typedef struct
{
  int16_t sc_codes[CELL]; /* S Voltage Codes */
} scv_;

/* Filtered Cell Voltage Data structure */
typedef struct
{
  int16_t fc_codes[CELL]; /* filtered Cell Voltage Codes */
} fcv_;

/* Aux Voltage Data Structure*/
typedef struct
{
  int16_t a_codes[AUX]; /* Aux Voltage Codes */
} ax_;

/* Redundant Aux Voltage Data Structure*/
typedef struct
{
  int16_t ra_codes[RAUX]; /* Aux Voltage Codes */
} rax_;

/* Status A register Data structure*/
typedef struct
{
  uint16_t  vref2;
  uint16_t  itmp;
  uint16_t  vref3;
} sta_;

/* Status B register Data structure*/
typedef struct
{
  uint16_t vd;
  uint16_t va;
  uint16_t vr4k;
} stb_;

/* Status C register Data structure*/
typedef struct
{
  uint16_t      cs_flt;
  uint8_t       va_ov;
  uint8_t       va_uv;
  uint8_t       vd_ov;
  uint8_t       vd_uv;
  uint8_t       otp1_ed;
  uint8_t       otp1_med;
  uint8_t       otp2_ed;
  uint8_t       otp2_med;
  uint8_t       vde;
  uint8_t       vdel;
  uint8_t       comp;
  uint8_t       spiflt;
  uint8_t       sleep;
  uint8_t       thsd;
  uint8_t       tmodchk;
  uint8_t       oscchk;
} stc_;

/* ClrFlag register Data structure*/
typedef struct
{
  uint16_t      cl_csflt;
  uint8_t       cl_smed;
  uint8_t       cl_sed;
  uint8_t       cl_cmed;
  uint8_t       cl_ced;
  uint8_t       cl_vduv;
  uint8_t       cl_vdov;
  uint8_t       cl_vauv;
  uint8_t       cl_vaov;
  uint8_t       cl_oscchk;
  uint8_t       cl_tmode;
  uint8_t       cl_thsd;
  uint8_t       cl_sleep;
  uint8_t       cl_spiflt;
  uint8_t       cl_vdel;
  uint8_t       cl_vde;
} clrflag_;

/* Status D register Data structure*/
typedef struct
{
  uint8_t c_ov[CELL];
  uint8_t c_uv[CELL];
  uint8_t ct;
  uint8_t cts;
  uint8_t oc_cntr;
} std_;

/* Status E register Data structure*/
typedef struct
{
  uint16_t gpi;
  uint8_t rev;
} ste_;

/* Pwm register Data structure*/
typedef struct
{
  uint8_t pwma[PWMA];
} pwma_;

/*PWMB Register Structure */
typedef struct
{
  uint8_t pwmb[PWMB];
} pwmb_;

/* Command counter and pec error data Structure */
typedef struct
{
  uint8_t cmd_cntr;
  uint8_t cfgr_pec;
  uint8_t cell_pec;
  uint8_t acell_pec;
  uint8_t scell_pec;
  uint8_t fcell_pec;
  uint8_t aux_pec;
  uint8_t raux_pec;
  uint8_t stat_pec;
  uint8_t comm_pec;
  uint8_t pwm_pec;
  uint8_t sid_pec;
} cmdcnt_pec_6830_;

/* Diagnostic test result data structure */
typedef struct
{
  uint8_t osc_mismatch;
  uint8_t supply_error;
  uint8_t supply_ovuv;
  uint8_t thsd;
  uint8_t fuse_ed;
  uint8_t fuse_med;
  uint8_t tmodchk;
  uint8_t cell_ow[CELL];
  uint8_t cellred_ow[CELL];
  uint8_t aux_ow[(AUX-2)];
} diag_test_6830_;

/* Aux open wire data structure */
typedef struct
{
  int cell_ow_even[CELL];
  int cell_ow_odd[CELL];
} cell_ow_;

/* Aux open wire data structure */
typedef struct
{
  int aux_pup_up[(AUX-2)];
  int aux_pup_down[(AUX-2)];
} aux_ow_;

/* adc command config - holds the command configuration for the adbms */
typedef struct
{
  RD REDUNDANT_MEASUREMENT;
  CH AUX_CH_TO_CONVERT;
  CONT CONTINUOUS_MEASUREMENT;
  OW_C_S CELL_OPEN_WIRE_DETECTION;
  OW_AUX AUX_OPEN_WIRE_DETECTION;
  PUP OPEN_WIRE_CURRENT_SOURCE;
  DCP DISCHARGE_PERMITTED;
  RSTF RESET_FILTER;
  ERR  INJECT_ERR_SPI_READ;
} adc_command_config_t;

typedef struct
{
  float OV_THRESHOLD; /* Volt */
  float UV_THRESHOLD; /* Volt */
  int OWC_Threshold; /* Cell Open wire threshold(mili volt) */
  int OWA_Threshold; /* Aux Open wire threshold(mili volt) */
} threshold_config_t;

/* loop_manager_t - holds loop measurement variables
* LOOP_MEASUREMENT variables are either ENABLED or DISABLED (all caps)
*/
typedef struct
{
  uint32_t LOOP_MEASUREMENT_COUNT;
  uint16_t MEASUREMENT_LOOP_TIME;
  uint32_t loop_count;
  uint32_t pladc_count;
  LOOP_MEASURMENT MEASURE_CELL;
  LOOP_MEASURMENT MEASURE_AVG_CELL;
  LOOP_MEASURMENT MEASURE_F_CELL;
  LOOP_MEASURMENT MEASURE_S_VOLTAGE;
  LOOP_MEASURMENT MEASURE_AUX;
  LOOP_MEASURMENT MEASURE_RAUX;
  LOOP_MEASURMENT MEASURE_STAT;
} loop_manager_6830_t;

#endif /* INC_EXT_DRIVERS_ADBMS_SHARED_H_ */
