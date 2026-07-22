/*
 * ams_estimator_lut.h
 * Author: Mahad Faisal (2026)
 *
 * P42A ECM lookup tables used by the AMS physics-only DAEKF estimator.
 * Values are copied from the validated 75s6p P42A HIL plant / RA estimator
 * source. All values are per representative cell unless otherwise noted.
 */

#ifndef INC_ESTIMATOR_AMS_ESTIMATOR_LUT_H_
#define INC_ESTIMATOR_AMS_ESTIMATOR_LUT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

float ams_p42a_ocv_v(float soc, float temp_c);
float ams_p42a_r0_ohm(float soc, float temp_c);
float ams_p42a_inv_c1(float soc, float temp_c);
float ams_p42a_neg_inv_tau1(float soc, float temp_c);
float ams_p42a_inv_r1_from_luts(float inv_c1, float neg_inv_tau1);

#ifdef __cplusplus
}
#endif

#endif /* INC_ESTIMATOR_AMS_ESTIMATOR_LUT_H_ */
