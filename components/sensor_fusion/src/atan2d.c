/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: atan2d.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 16-May-2026 16:25:25
 */

/* Include Files */
#include "atan2d.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : float y
 *                float x
 * Return Type  : float
 */
float b_atan2d(float y, float x)
{
  float r;
  if (rtIsNaNF(y) || rtIsNaNF(x)) {
    r = rtNaNF;
  } else if (rtIsInfF(y) && rtIsInfF(x)) {
    int i;
    int i1;
    if (y > 0.0F) {
      i = 1;
    } else {
      i = -1;
    }
    if (x > 0.0F) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    r = atan2f((float)i, (float)i1);
  } else if (x == 0.0F) {
    if (y > 0.0F) {
      r = RT_PIF / 2.0F;
    } else if (y < 0.0F) {
      r = -(RT_PIF / 2.0F);
    } else {
      r = 0.0F;
    }
  } else {
    r = atan2f(y, x);
  }
  r *= 57.2957802F;
  return r;
}

/*
 * File trailer for atan2d.c
 *
 * [EOF]
 */
