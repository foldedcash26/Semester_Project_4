/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sensorFusionEntryPoint.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 16-May-2026 16:25:25
 */

#ifndef SENSORFUSIONENTRYPOINT_H
#define SENSORFUSIONENTRYPOINT_H

/* Include Files */
#include "rtwtypes.h"
#include "sensorFusionEntryPoint_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void sensorFusionEntryPoint(float ax, float ay, float az, float gx,
                                   float gy, float gz, float mx, float my,
                                   float mz, float Ts, struct0_T *state,
                                   float *pitch, float *roll, float *yaw);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for sensorFusionEntryPoint.h
 *
 * [EOF]
 */
