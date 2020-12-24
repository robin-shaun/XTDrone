//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: rt_nonfinite.cpp
//
// Code generated for Simulink model 'obstacleStopper'.
//
// Model version                  : 1.86
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri May 24 15:32:42 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//

//
//  Abstract:
//       Function to initialize non-finites,
//       (Inf, NaN and -Inf).
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include "rtGetInf.h"
#define NumBitsPerChar                 8U

extern "C" {
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}
  extern "C"
{
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  // Test if value is infinite
  boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  boolean_T rtIsNaN(real_T value)
  {
    boolean_T result = (boolean_T) 0;
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    if (bitsPerReal == 32U) {
      result = rtIsNaNF((real32_T)value);
    } else {
      uint16_T one = 1U;
      enum {
        LittleEndian,
        BigEndian
      } machByteOrder = (*((uint8_T *) &one) == 1U) ? LittleEndian : BigEndian;
      switch (machByteOrder) {
       case LittleEndian:
        {
          union {
            LittleEndianIEEEDouble bitVal;
            real_T fltVal;
          } tmpVal;

          tmpVal.fltVal = value;
          result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                               0x7FF00000 &&
                               ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                                (tmpVal.bitVal.words.wordL != 0) ));
          break;
        }

       case BigEndian:
        {
          union {
            BigEndianIEEEDouble bitVal;
            real_T fltVal;
          } tmpVal;

          tmpVal.fltVal = value;
          result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                               0x7FF00000 &&
                               ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                                (tmpVal.bitVal.words.wordL != 0) ));
          break;
        }
      }
    }

    return result;
  }

  // Test if single-precision value is not a number
  boolean_T rtIsNaNF(real32_T value)
  {
    IEEESingle tmp;
    tmp.wordL.wordLreal = value;
    return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                       (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
