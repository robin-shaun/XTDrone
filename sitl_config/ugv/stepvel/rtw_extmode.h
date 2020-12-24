/* Copyright 1990-2013 The MathWorks, Inc. */

/*
 * File: rtw_extmode.h     
 *
 * Abstract:
 *   Type definitions for Simulink External Mode support.
 */

#ifndef RTW_EXTMODE_H__
#define RTW_EXTMODE_H__

#ifndef _RTWEXTMODEINFO
# define _RTWEXTMODEINFO
  typedef struct _RTWExtModeInfo_tag RTWExtModeInfo;
#endif

/* =============================================================================
 * External mode object
 * =============================================================================
 */
struct _RTWExtModeInfo_tag {

    void *subSysActiveVectorAddr;  /* Array of addresses pointing to
                                    * the active vector slots for sub-systems.
                                    * Sub-systems store information about their
                                    * state in their extmode active vector.
                                    */
    uint32_T *checksumsPtr;        /* Pointer to the model's checksums array
                                    */
    const void **mdlMappingInfoPtr;/* Pointer to the model's mapping info
                                    * pointer
                                    */

#if !defined(ENABLE_SLEXEC_SSBRIDGE)
    void       *tPtr;              /* Copy of model's time pointer */
#else
    void *simStruct; /* simulink execution (raccel/rsim) needs simstruct */
#endif

    int32_T tFinalTicks;           /* Used with integer only code, holds the
                                    * number of base rate ticks representing
                                    * the final time (final time in seconds
                                    * divided by base rate step size).
                                    */

};

/* gnat 3.12a2 doesn't like use "/" as line continuation here */
#define rteiSetSubSystemActiveVectorAddresses(E,addr) ((E)->subSysActiveVectorAddr = ((void *)addr))
#define rteiGetSubSystemActiveVectorAddresses(E)    ((E)->subSysActiveVectorAddr)
#define rteiGetAddrOfSubSystemActiveVector(E,idx)   ((int8_T*)((int8_T**)((E)->subSysActiveVectorAddr))[idx])

#define rteiSetModelMappingInfoPtr(E,mip) ((E)->mdlMappingInfoPtr = (mip))
#define rteiGetModelMappingInfo(E) (*((E)->mdlMappingInfoPtr))

#define rteiSetChecksumsPtr(E,cp) ((E)->checksumsPtr = (cp))
#define rteiGetChecksum0(E) (E)->checksumsPtr[0]
#define rteiGetChecksum1(E) (E)->checksumsPtr[1]
#define rteiGetChecksum2(E) (E)->checksumsPtr[2]
#define rteiGetChecksum3(E) (E)->checksumsPtr[3]

#define rteiGetTFinalTicks(E) ((int32_T)((E)->tFinalTicks))
#define rteiGetPtrTFinalTicks(E) ((int32_T *)(&((E)->tFinalTicks)))

#if defined(ENABLE_SLEXEC_SSBRIDGE)
  #include "slexec_simbridge.h"
#else
  #define rteiSetTPtr(E,p) ((E)->tPtr = (p))
  #define rteiGetT(E)      ((time_T *)(E)->tPtr)[0]
#endif

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
# if defined(__LCC__)
#   define UNUSED_PARAMETER(x)  /* do nothing */
# else
/*
 * This is the semi-ANSI standard way of indicating that a
 * unused function parameter is required.
 */
#   define UNUSED_PARAMETER(x) (void) (x)
# endif
#endif

#endif /* __RTW_EXTMODE_H__ */

