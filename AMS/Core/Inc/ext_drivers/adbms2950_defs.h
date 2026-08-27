/*
 * adbms2950_defs.h
 *
 *  Created on: May 13, 2025
 *      Author: cole
 *      Modified by: Mahad Faisal (major firmware updates, 2026)
 */

#ifndef INC_EXT_DRIVERS_ADBMS2950_DEFS_H_
#define INC_EXT_DRIVERS_ADBMS2950_DEFS_H_

#include <stdint.h>

#define NVBATS 2 // Number of VBat ADC values
#define NVIS 2 // Number of Current ADC values
#define NAPMTEMPS 2 // Number of temp sensors on the APM

#define VBAT1_SCALE 0.0001f // 100uV resolution
#define VBAT2_SCALE -0.000085f // 85uV resolution but gain is inverted (negative)
#define VI1_SCALE 0.000001f // 1uV resolution
#define VI2_SCALE -0.000001f // 1uV resolution but gain is inverted (negative)
#define VxA_SCALE 0.0001f // 100uV resolution
#define VxB_SCALE -0.000085 // 85uV resolution but gain is inverted (negative)

/* Board-specific analog scaling.  The DER APM and the ADI evaluation board
 * do not use the same shunt or VBAT divider.  Keep the legacy names as DER
 * aliases so older code remains source-compatible, but production conversion
 * uses adbms2950_calibration_t from adbms2950.h. */
#define ADBMS2950_DER_SHUNT_RESISTANCE_OHM 0.000100f
#define ADBMS2950_EVAL_SHUNT_RESISTANCE_OHM 0.000050f
#define ADBMS2950_DER_VB1_DIVIDER_RATIO (3622.0f / 22.0f)
#define ADBMS2950_DER_VB2_DIVIDER_RATIO ADBMS2950_DER_VB1_DIVIDER_RATIO
#define ADBMS2950_EVAL_VB1_DIVIDER_RATIO (3609.1f / 9.1f)
#define ADBMS2950_EVAL_VB2_DIVIDER_RATIO ADBMS2950_EVAL_VB1_DIVIDER_RATIO
#define ADBMS2950_DEFAULT_CURRENT_GAIN 1.0f
#define ADBMS2950_DEFAULT_CURRENT_OFFSET_UV 0.0f
#define ADBMS2950_DEFAULT_CURRENT_POLARITY 1

#define VBAT_DIV_SCALE ADBMS2950_DER_VB1_DIVIDER_RATIO
#define CURRENT_R_SCALE (1.0f / ADBMS2950_DER_SHUNT_RESISTANCE_OHM)

#endif /* INC_EXT_DRIVERS_ADBMS2950_DEFS_H_ */
