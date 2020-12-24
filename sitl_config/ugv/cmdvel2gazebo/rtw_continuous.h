/* Copyright 1990-2014 The MathWorks, Inc. */

/*
 * File: rtw_continuous.h     
 *
 * Abstract:
 *   Type definitions for continuous-time support.
 *
 */

#ifndef RTW_CONTINUOUS_H__
#define RTW_CONTINUOUS_H__

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

/* For models registering MassMatrix */
typedef enum {
    SS_MATRIX_NONE,
    SS_MATRIX_CONSTANT,
    SS_MATRIX_TIMEDEP,
    SS_MATRIX_STATEDEP
} ssMatrixType;
    
typedef enum {
    SOLVER_MODE_AUTO,          /* only occurs in
                                  mdlInitializeSizes/mdlInitializeSampleTimes */
    SOLVER_MODE_SINGLETASKING,
    SOLVER_MODE_MULTITASKING
} SolverMode;

typedef enum {
    MINOR_TIME_STEP,
    MAJOR_TIME_STEP
} SimTimeStep;

/* =============================================================================
 * Model methods object
 * =============================================================================
 */
typedef void (*rtMdlInitializeSizesFcn)(void *rtModel);
typedef void (*rtMdlInitializeSampleTimesFcn)(void *rtModel);
typedef void (*rtMdlStartFcn)(void *rtModel);
typedef void (*rtMdlOutputsFcn)(void *rtModel, int_T tid);
typedef void (*rtMdlUpdateFcn)(void *rtModel, int_T tid);
typedef void (*rtMdlDerivativesFcn)(void *rtModel);
typedef void (*rtMdlProjectionFcn)(void *rtModel);
typedef void (*rtMdlMassMatrixFcn)(void *rtModel);
typedef void (*rtMdlForcingFunctionFcn)(void *rtModel);
typedef void (*rtMdlTerminateFcn)(void *rtModel);
#ifdef RT_MALLOC
typedef real_T (*rtMdlDiscreteEventsFcn)(void  *pModel,
                                         int_T  rtmNumSampTimes, 
                                         void  *rtmTimingData, 
                                         int_T  *rtmSampleHitPtr, 
                                         int_T  *rtmPerTaskSampleHits);
#endif

typedef struct _RTWRTModelMethodsInfo_tag {
    void                          *rtModelPtr;
    rtMdlInitializeSizesFcn       rtmInitSizesFcn;
    rtMdlInitializeSampleTimesFcn rtmInitSampTimesFcn;
    rtMdlStartFcn                 rtmStartFcn;
    rtMdlOutputsFcn               rtmOutputsFcn;
    rtMdlUpdateFcn                rtmUpdateFcn;
    rtMdlDerivativesFcn           rtmDervisFcn;
    rtMdlProjectionFcn            rtmProjectionFcn;
    rtMdlMassMatrixFcn            rtmMassMatrixFcn;
    rtMdlForcingFunctionFcn       rtmForcingFunctionFcn;
    rtMdlTerminateFcn             rtmTerminateFcn;
#ifdef  RT_MALLOC
    rtMdlDiscreteEventsFcn        rtmDiscreteEventsFcn;
#endif
} RTWRTModelMethodsInfo;

#define rtmiSetRTModelPtr(M,rtmp) ((M).rtModelPtr = (rtmp))
#define rtmiGetRTModelPtr(M)      (M).rtModelPtr

#define rtmiSetInitSizesFcn(M,fp) \
  ((M).rtmInitSizesFcn = ((rtMdlInitializeSizesFcn)(fp)))
#define rtmiSetInitSampTimesFcn(M,fp) \
  ((M).rtmInitSampTimesFcn = ((rtMdlInitializeSampleTimesFcn)(fp)))
#define rtmiSetStartFcn(M,fp) \
  ((M).rtmStartFcn = ((rtMdlStartFcn)(fp)))
#define rtmiSetOutputsFcn(M,fp) \
  ((M).rtmOutputsFcn = ((rtMdlOutputsFcn)(fp)))
#define rtmiSetUpdateFcn(M,fp) \
  ((M).rtmUpdateFcn = ((rtMdlUpdateFcn)(fp)))
#define rtmiSetDervisFcn(M,fp) \
  ((M).rtmDervisFcn = ((rtMdlDerivativesFcn)(fp)))
#define rtmiSetProjectionFcn(M,fp) \
  ((M).rtmProjectionFcn = ((rtMdlProjectionFcn)(fp)))
#define rtmiSetMassMatrixFcn(M,fp) \
  ((M).rtmMassMatrixFcn = ((rtMdlMassMatrixFcn)(fp)))
#define rtmiSetForcingFunctionFcn(M,fp) \
  ((M).rtmForcingFunctionFcn = ((rtMdlForcingFunctionFcn)(fp)))
#define rtmiSetTerminateFcn(M,fp) \
  ((M).rtmTerminateFcn = ((rtMdlTerminateFcn)(fp)))
#ifdef  RT_MALLOC
#define rtmiSetDiscreteEventsFcn(M,fp) \
  ((M).rtmDiscreteEventsFcn = ((rtMdlDiscreteEventsFcn)(fp)))
#endif

#define rtmiInitializeSizes(M)                  \
    ((*(M).rtmInitSizesFcn)((M).rtModelPtr))
#define rtmiInitializeSampleTimes(M)                    \
    ((*(M).rtmInitSampTimesFcn)((M).rtModelPtr))
#define rtmiStart(M) \
    ((*(M).rtmStartFcn)((M).rtModelPtr))
#define rtmiOutputs(M, tid) \
    ((*(M).rtmOutputsFcn)((M).rtModelPtr,tid))
#define rtmiUpdate(M, tid) \
    ((*(M).rtmUpdateFcn)((M).rtModelPtr,tid))
#define rtmiDerivatives(M) \
    ((*(M).rtmDervisFcn)((M).rtModelPtr))
#define rtmiProjection(M) \
    ((*(M).rtmProjectionFcn)((M).rtModelPtr))
#define rtmiMassMatrix(M) \
    ((*(M).rtmMassMatrixFcn)((M).rtModelPtr))
#define rtmiForcingFunction(M) \
    ((*(M).rtmForcingFunctionFcn)((M).rtModelPtr))
#define rtmiTerminate(M) \
    ((*(M).rtmTerminateFcn)((M).rtModelPtr))
#ifdef  RT_MALLOC
#define rtmiDiscreteEvents(M,x1,x2,x3,x4)                               \
    ((*(M).rtmDiscreteEventsFcn)((M).rtModelPtr,(x1),(x2),(x3),(x4)))
#endif
#endif /* __RTW_CONTINUOUS_H__ */
