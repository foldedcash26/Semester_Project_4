/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sensorFusionEntryPoint.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 16-May-2026 16:25:25
 */

/* Include Files */
#include "sensorFusionEntryPoint.h"
#include "atan2d.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "sensorFusionEntryPoint_types.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : float ax
 *                float ay
 *                float az
 *                float gx
 *                float gy
 *                float gz
 *                float mx
 *                float my
 *                float mz
 *                float Ts
 *                struct0_T *state
 *                float *pitch
 *                float *roll
 *                float *yaw
 * Return Type  : void
 */
void sensorFusionEntryPoint(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz, float Ts,
                            struct0_T *state, float *pitch, float *roll,
                            float *yaw)
{
  float diff;
  float f;
  float f1;
  (void)mz;
  /*  ============================================================ */
  /*   9-DOF Complementary Filter */
  /*   MATLAB Coder Compatible */
  /*  */
  /*   Inputs: */
  /*     accel in g */
  /*     gyro in deg/s */
  /*     mag raw calibrated values */
  /*  */
  /*   state contains: */
  /*     pitch */
  /*     roll */
  /*     yaw */
  /*     magMidX */
  /*     magMidY */
  /*  */
  /*  ============================================================ */
  /*  ---- Tunable parameters ---- */
  /*  ------------------------------------------------------------ */
  /*  ACCEL ANGLES */
  /*  ------------------------------------------------------------ */
  /*  COMPLEMENTARY FILTER */
  /*  ------------------------------------------------------------ */
  diff = az * az;
  f = 0.9F * (state->pitch + gx * Ts) +
      0.100000024F * b_atan2d(-ax, sqrtf(ay * ay + diff));
  state->pitch = f;
  f1 = 0.9F * (state->roll + gy * Ts) +
       0.100000024F * b_atan2d(ay, sqrtf(ax * ax + diff));
  state->roll = f1;
  /*  ------------------------------------------------------------ */
  /*  MAGNETOMETER HEADING */
  /*  ------------------------------------------------------------ */
  /*  Hard-iron corrected values */
  /*  ------------------------------------------------------------ */
  /*  YAW COMPLEMENTARY FILTER */
  /*  ------------------------------------------------------------ */
  /*diff = b_mod(b_atan2d(-(mx - state->magMidX), my - state->magMidY) + 4.33F) - HARD + SOFT IRON CORRECTION */
float corr_mx = (mx - state->magMidX) * state->magScaleX;
float corr_my = (my - state->magMidY) * state->magScaleY;

/* gyro prediction */
float yaw_gyro = state->yaw + gz * Ts;

/* mag heading */
float yaw_mag = b_mod(b_atan2d(-corr_mx, corr_my) + 4.33F);



/* shortest-angle difference */
float err = yaw_mag - yaw_gyro;

if (err > 180.0F) {
    err -= 360.0F;
}
if (err < -180.0F) {
    err += 360.0F;
}

/* complementary fusion */
state->yaw = yaw_gyro + 0.01F * err;

/* wrap final angle */
if (state->yaw >= 360.0F) {
    state->yaw -= 360.0F;
}
if (state->yaw < 0.0F) {
    state->yaw += 360.0F;
}

*yaw = state->yaw;
*pitch = state->pitch;
*roll  = state->roll;
}

/*
 * File trailer for sensorFusionEntryPoint.c
 *
 * [EOF]
 */

