/* Copyright 1990-2016 The MathWorks, Inc. */

/*
 * File: rtw_solver.h     
 *
 * Abstract:
 *   Type definitions for continuous-time solver support.
 *
 */

#ifndef RTW_SOLVER_H__
#define RTW_SOLVER_H__

#include "rtw_continuous.h"

/* =============================================================================
 * Solver object
 * =============================================================================
 */
#ifndef NO_FLOATS /* ERT integer-only */
/*
 * Enum for solver tolerance
 */
typedef enum {
    SL_SOLVER_TOLERANCE_AUTO  = 0,  /* Set Automatically by Solver */
    SL_SOLVER_TOLERANCE_LOCAL = 1,  /* Set Locally, e.g., by Blocks */
    SL_SOLVER_TOLERANCE_GLOBAL = 2, /* Set Globally, e.g., by Block Diagram */
    SL_SOLVER_TOLERANCE_UNDEFINED = 255 /* Signal uninitialized */
} SL_SolverToleranceControlFlag_T;


/*
 * Enum for jacobian method control
 */
typedef enum {
    SL_JM_BD_AUTO = 0,
    SL_JM_BD_SPARSE_PERTURBATION,
    SL_JM_BD_FULL_PERTURBATION,
    SL_JM_BD_SPARSE_ANALYTICAL,
    SL_JM_BD_FULL_ANALYTICAL
} slJmBdControl;


typedef struct _ssSolverInfo_tag {
    void        *rtModelPtr;

    SimTimeStep *simTimeStepPtr;
    void        *solverData;
    const char_T  *solverName;
    boolean_T   isVariableStepSolver;
    boolean_T   solverNeedsReset;
    SolverMode  solverMode;

    time_T      solverStopTime;
    time_T      *stepSizePtr;
    time_T      minStepSize;
    time_T      maxStepSize;
    time_T      fixedStepSize;
      
    int_T       solverShapePreserveControl;
    int_T       solverMaxConsecutiveMinStep;
    int_T       maxNumMinSteps;
    int_T       solverMaxOrder;
    real_T      solverConsecutiveZCsStepRelTol;
    int_T       solverMaxConsecutiveZCs;
   
    int_T       solverExtrapolationOrder;
    int_T       solverNumberNewtonIterations;

    int_T       solverRefineFactor;
    real_T      solverRelTol;
    real_T      unused_real_T_1;

    real_T      **dXPtr;
    time_T      **tPtr;

    int_T       *numContStatesPtr;
    real_T      **contStatesPtr;

    int_T*      numPeriodicContStatesPtr;
    int_T**     periodicContStateIndicesPtr;
    real_T**    periodicContStateRangesPtr;

    real_T*     zcSignalVector;
    uint8_T*    zcEventsVector;
    uint8_T*    zcSignalAttrib;
    int_T       zcSignalVectorLength;
    uint8_T*    reserved;

    boolean_T   foundContZcEvents;
    boolean_T   isAtLeftPostOfContZcEvent;
    boolean_T   isAtRightPostOfContZcEvent;
    boolean_T   adaptiveZcDetection;

    int_T       numZcSignals;

    boolean_T   stateProjection;
    boolean_T   robustResetMethod;  /* user's preference */
    boolean_T   updateJacobianAtReset; /* S-Fcn request (sticky) */
    boolean_T   consistencyChecking;

    ssMatrixType  massMatrixType;
    int_T         massMatrixNzMax;
    int_T*        massMatrixIr; 
    int_T*        massMatrixJc; 
    real_T*       massMatrixPr; 

    const char_T **errStatusPtr;

    RTWRTModelMethodsInfo *modelMethodsPtr;
    real_T      zcThreshold;
    int_T       zeroCrossAlgorithm;
    int_T       consecutiveZCsError;
    boolean_T   blkStateChange;
    boolean_T   isComputingJacobian;
    slJmBdControl solverJacobianMethodControl;
    int_T       ignoredZcDiagnostic;
    int_T       maskedZcDiagnostic;
    boolean_T   isOutputMethodComputed;
} ssSolverInfo;

/* Support old name RTWSolverInfo */
typedef ssSolverInfo RTWSolverInfo;

#define rtsiSetRTModelPtr(S,rtmp) ((S)->rtModelPtr = (rtmp))
#define rtsiGetRTModelPtr(S)      (S)->rtModelPtr

#define rtsiSetSimTimeStepPtr(S,stp) ((S)->simTimeStepPtr = (stp))
#define rtsiGetSimTimeStepPtr(S) ((S)->simTimeStepPtr)
#define rtsiGetSimTimeStep(S)        *((S)->simTimeStepPtr)
#define rtsiSetSimTimeStep(S,st)     (*((S)->simTimeStepPtr) = (st))

#define rtsiSetSolverData(S,sd) ((S)->solverData = (sd))
#define rtsiGetSolverData(S)    (S)->solverData

#define rtsiSetSolverName(S,sn) ((S)->solverName = (sn))
#define rtsiGetSolverName(S)    (S)->solverName

#define rtsiSetVariableStepSolver(S,vs) ((S)->isVariableStepSolver = (vs))
#define rtsiIsVariableStepSolver(S)     (S)->isVariableStepSolver

#define rtsiSetSolverNeedsReset(S,sn) ((S)->solverNeedsReset = (sn))
#define rtsiGetSolverNeedsReset(S)    (S)->solverNeedsReset

#define rtsiSetBlkStateChange(S,sn) ((S)->blkStateChange = (sn))
#define rtsiGetBlkStateChange(S)    (S)->blkStateChange

#define rtsiSetSolverMode(S,sm) ((S)->solverMode = (sm))
#define rtsiGetSolverMode(S)    (S)->solverMode

#define rtsiSetSolverStopTime(S,st) ((S)->solverStopTime = (st))
#define rtsiGetSolverStopTime(S)    (S)->solverStopTime

#define rtsiSetStepSizePtr(S,ssp) ((S)->stepSizePtr = (ssp))
#define rtsiSetStepSize(S,ss)     (*((S)->stepSizePtr) = (ss))
#define rtsiGetStepSize(S)        *((S)->stepSizePtr)

#define rtsiSetMinStepSize(S,ss) (((S)->minStepSize = (ss)))
#define rtsiGetMinStepSize(S)    (S)->minStepSize

#define rtsiSetMaxStepSize(S,ss) ((S)->maxStepSize = (ss))
#define rtsiGetMaxStepSize(S)    (S)->maxStepSize

#define rtsiSetFixedStepSize(S,ss) ((S)->fixedStepSize = (ss))
#define rtsiGetFixedStepSize(S)    (S)->fixedStepSize

#define rtsiSetMaxNumMinSteps(S,mns) ((S)->maxNumMinSteps = (mns))
#define rtsiGetMaxNumMinSteps(S)     (S)->maxNumMinSteps

#define rtsiSetSolverMaxOrder(S,smo) ((S)->solverMaxOrder = (smo))
#define rtsiGetSolverMaxOrder(S)     (S)->solverMaxOrder

#define rtsiSetSolverJacobianMethodControl(S,smcm)   (ssGetSolverInfo(S)->solverJacobianMethodControl = (smcm))
#define rtsiGetSolverJacobianMethodControl(S)        ssGetSolverInfo(S)->solverJacobianMethodControl

#define rtsiSetSolverShapePreserveControl(S,smcm)   (ssGetSolverInfo(S)->solverShapePreserveControl = (smcm))
#define rtsiGetSolverShapePreserveControl(S)        ssGetSolverInfo(S)->solverShapePreserveControl

#define rtsiSetSolverConsecutiveZCsStepRelTol(S,scr) (ssGetSolverInfo(S)->solverConsecutiveZCsStepRelTol = (scr))
#define rtsiGetSolverConsecutiveZCsStepRelTol(S)     ssGetSolverInfo(S)->solverConsecutiveZCsStepRelTol  

#define rtsiSetSolverMaxConsecutiveZCs(S,smcz)       (ssGetSolverInfo(S)->solverMaxConsecutiveZCs = (smcz))
#define rtsiGetSolverMaxConsecutiveZCs(S)            ssGetSolverInfo(S)->solverMaxConsecutiveZCs

#define rtsiSetSolverMaxConsecutiveMinStep(S,smcm)   (ssGetSolverInfo(S)->solverMaxConsecutiveMinStep = (smcm))
#define rtsiGetSolverMaxConsecutiveMinStep(S)        ssGetSolverInfo(S)->solverMaxConsecutiveMinStep

#define rtsiSetSolverExtrapolationOrder(S,seo) ((S)->solverExtrapolationOrder = (seo))
#define rtsiGetSolverExtrapolationOrder(S)      (S)->solverExtrapolationOrder

#define rtsiSetSolverNumberNewtonIterations(S,nni) ((S)->solverNumberNewtonIterations = (nni))
#define rtsiGetSolverNumberNewtonIterations(S)      (S)->solverNumberNewtonIterations

#define rtsiSetSolverRefineFactor(S,smo) ((S)->solverRefineFactor = (smo))
#define rtsiGetSolverRefineFactor(S)     (S)->solverRefineFactor

#define rtsiSetSolverRelTol(S,smo) ((S)->solverRelTol = (smo))
#define rtsiGetSolverRelTol(S)     (S)->solverRelTol

#define rtsiSetSolverMassMatrixType(S,type)  ((S)->massMatrixType = (type))
#define rtsiGetSolverMassMatrixType(S)  (S)->massMatrixType

#define rtsiSetSolverMassMatrixNzMax(S,nzMax)  ((S)->massMatrixNzMax = (nzMax))
#define rtsiGetSolverMassMatrixNzMax(S)  (S)->massMatrixNzMax

#define rtsiSetSolverMassMatrixIr(S,ir)  ((S)->massMatrixIr = (ir))
#define rtsiGetSolverMassMatrixIr(S)  (S)->massMatrixIr

#define rtsiSetSolverMassMatrixJc(S,jc)  ((S)->massMatrixJc = (jc))
#define rtsiGetSolverMassMatrixJc(S)  (S)->massMatrixJc

#define rtsiSetSolverMassMatrixPr(S,pr)  ((S)->massMatrixPr = (pr))
#define rtsiGetSolverMassMatrixPr(S)  (S)->massMatrixPr

#define rtsiSetdXPtr(S,dxp) ((S)->dXPtr = (dxp))
#define rtsiSetdX(S,dx)     (*((S)->dXPtr) = (dx))
#define rtsiGetdX(S)        *((S)->dXPtr)

#define rtsiSetTPtr(S,tp) ((S)->tPtr = (tp))
#define rtsiSetT(S,t)     ((*((S)->tPtr))[0] = (t))
#define rtsiGetT(S)       (*((S)->tPtr))[0]

#define rtsiSetContStatesPtr(S,cp) ((S)->contStatesPtr = (cp))
#define rtsiGetContStates(S)       *((S)->contStatesPtr)

#define rtsiSetNumContStatesPtr(S,cp) ((S)->numContStatesPtr = (cp))
#define rtsiGetNumContStates(S)       *((S)->numContStatesPtr)

#define rtsiSetNumPeriodicContStatesPtr(S,cp) ((S)->numPeriodicContStatesPtr = (cp))
#define rtsiGetNumPeriodicContStates(S)       *((S)->numPeriodicContStatesPtr)

#define rtsiSetPeriodicContStateIndicesPtr(S,cp) ((S)->periodicContStateIndicesPtr = (cp))
#define rtsiGetPeriodicContStateIndices(S)       *((S)->periodicContStateIndicesPtr)

#define rtsiSetPeriodicContStateRangesPtr(S,cp) ((S)->periodicContStateRangesPtr = (cp))
#define rtsiGetPeriodicContStateRanges(S)       *((S)->periodicContStateRangesPtr)

#define rtsiSetErrorStatusPtr(S,esp) ((S)->errStatusPtr = (esp))
#define rtsiSetErrorStatus(S,es) (*((S)->errStatusPtr) = (es))
#define rtsiGetErrorStatus(S)    *((S)->errStatusPtr)

#define rtsiSetModelMethodsPtr(S,mmp) ((S)->modelMethodsPtr = (mmp))
#define rtsiGetModelMethodsPtr(S)     (S)->modelMethodsPtr

#define rtsiSetSolverComputingJacobian(S,val) ((S)->isComputingJacobian = (val))
#define rtsiIsSolverComputingJacobian(S)    (S)->isComputingJacobian

#define rtsiSetSolverOutputComputed(S,val) ((S)->isOutputMethodComputed = (val))
#define rtsiIsSolverOutputComputed(S) (S)->isOutputMethodComputed

#endif /* !NO_FLOATS */

#endif /* RTW_SOLVER_H__ */
