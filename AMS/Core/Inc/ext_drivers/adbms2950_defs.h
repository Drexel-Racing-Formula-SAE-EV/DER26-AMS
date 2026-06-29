/*
 * adbms2950_defs.h
 *
 *  Created on: May 13, 2025
 *      Author: cole
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

#define VBAT_DIV_SCALE (3622.0f / 22.0f) // 3.622MR / 22kR
#define CURRENT_R_SCALE 10000.0f // 1 / 100uR

#endif /* INC_EXT_DRIVERS_ADBMS2950_DEFS_H_ */
