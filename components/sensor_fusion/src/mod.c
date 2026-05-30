/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mod.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 16-May-2026 16:25:25
 */

/* Include Files */
#include "mod.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : float x
 * Return Type  : float
 */
float b_mod(float x)
{
  float r;
  if (rtIsNaNF(x) || rtIsInfF(x)) {
    r = rtNaNF;
  } else {
    r = fmodf(x, 360.0F);
    if (r == 0.0F) {
      r = 0.0F;
    } else if (r < 0.0F) {
      r += 360.0F;
    }
  }
  return r;
}

/*
 * File trailer for mod.c
 *
 * [EOF]
 */
