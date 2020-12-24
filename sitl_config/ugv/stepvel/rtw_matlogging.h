/* Copyright 1990-2015 The MathWorks, Inc. */

/*
 * File: rtw_matlogging.h     
 *
 * Abstract:
 *   Type definitions for MAT-file logging support.
 */

#ifndef RTW_MATLOGGING_H__
#define RTW_MATLOGGING_H__

#include "sl_types_def.h" /* DTypeId */

/*
 * The RTWLogSignalInfo and RTWLogInfo structures are for use by
 * the Real-Time Workshop and should not be used by S-functions.
 */
typedef const int8_T * const * LogSignalPtrsType;

#ifndef NO_FLOATS /* ERT integer-only */

typedef struct RTWLogDataTypeConvert_tag {

    int conversionNeeded;
    BuiltInDTypeId dataTypeIdLoggingTo;
    DTypeId        dataTypeIdOriginal;
    int            bitsPerChunk;
    int            numOfChunk;
    unsigned int   isSigned;
    real_T         fracSlope;
    int            fixedExp;
    real_T         bias;

} RTWLogDataTypeConvert;

typedef void (*RTWPreprocessingFcnPtr)(void*, const void*);

typedef struct RTWLogSignalInfo_tag {
    int_T           numSignals;
    int_T          *numCols;
    int_T          *numDims;
    int_T          *dims;
    boolean_T      *isVarDims;       /* Dimension Mode: true -- VarDims / false -- fixed dims */
    void           **currSigDims;    /* current signal dimensions */
    int_T          *currSigDimsSize; /* Size of currSigDims in bytes */
    BuiltInDTypeId *dataTypes;
    int_T          *complexSignals;
    int_T          *frameData;
    RTWPreprocessingFcnPtr* preprocessingPtrs;

    union {
        const char_T** cptr;
        char_T**       ptr;
    } labels;

    char_T         *titles;
    int_T          *titleLengths;
    int_T          *plotStyles;

    union {
        const char_T** cptr;
        char_T**       ptr;
    } blockNames;

    union {
        const char_T** cptr;
        char_T**       ptr;
    } stateNames;

    boolean_T      *crossMdlRef;

    RTWLogDataTypeConvert *dataTypeConvert;

} RTWLogSignalInfo;

/* =============================================================================
 * Logging object
 * =============================================================================
 */
typedef struct _RTWLogInfo_tag {
  void              *logInfo;      /* Pointer to a book keeping structure    *
                                    * used in rtwlog.c                       */

  union {
      LogSignalPtrsType cptr; /* Pointers to the memory location    */
      int8_T**    ptr;        /* of the data to be logged into the  * 
                               * states structure. Not used if      * 
                               * logging data in matrix format.     */
      
  } logXSignalPtrs;

  union {
      LogSignalPtrsType cptr; /* Pointers to the memory location    */
      int8_T**    ptr;        /* of the data to be logged into the  *
                               * outputs structure. Not used if     *
                               * logging data in matrix format.     */
  } logYSignalPtrs;

  int_T         logFormat;          /* matrix=0, struct=1, or strut_wo_time=2 */

  int_T         logMaxRows;         /* Max number of rows (0 for no limit)    */
  int_T         logDecimation;      /* Data logging interval                  */

  const char_T  *logVarNameModifier;/* pre(post)fix string modifier for the   *
                                     * log variable names                     */

  const char_T  *logT;              /* Name of variable to log time           */
  const char_T  *logX;              /* Name of variable to log states         */
  const char_T  *logXFinal;         /* Name of variable to log final state    */
  const char_T  *logY;              /* Name of variable(s) to log outputs     */
  const char_T  *logSL;          /* Name of variable(s) to log signal logging */

  union { /* Info about the states             */
      const RTWLogSignalInfo *cptr;
      RTWLogSignalInfo       *ptr;
  } logXSignalInfo;

  union {/* Info about the outputs            */
      const RTWLogSignalInfo *cptr;
      RTWLogSignalInfo       *ptr;
  }logYSignalInfo;

  void (*mdlLogData)(void *rtli, void *tPtr);
  void (*mdlLogDataIfInInterval)(void *rtli, void *tPtr, boolean_T isInInterval);

  const void * mmi;    /* Add the ModelMapping Info to the LogInfo 
                        * so we can get at it for state logging */
  void * loggingInterval;

} RTWLogInfo;

#endif

/* Macros associated with RTWLogInfo */
#define rtliGetLogInfo(rtli)     ((LogInfo*)(rtli)->logInfo)
#define rtliSetLogInfo(rtli,ptr) ((rtli)->logInfo = ((void *)ptr))

#define rtliGetLogFormat(rtli)   (rtli)->logFormat
#define rtliSetLogFormat(rtli,f) ((rtli)->logFormat = (f))

#define rtliGetLogVarNameModifier(rtli)      (rtli)->logVarNameModifier
#define rtliSetLogVarNameModifier(rtli,name) ((rtli)->logVarNameModifier=(name))

#define rtliGetLogMaxRows(rtli)     (rtli)->logMaxRows
#define rtliSetLogMaxRows(rtli,num) ((rtli)->logMaxRows = (num))

#define rtliGetLogDecimation(rtli)   (rtli)->logDecimation
#define rtliSetLogDecimation(rtli,l) ((rtli)->logDecimation = (l))

#define rtliGetLogT(rtli)     (rtli)->logT
#define rtliSetLogT(rtli,lt)  ((rtli)->logT = (lt))

#define rtliGetLogX(rtli)     (rtli)->logX
#define rtliSetLogX(rtli,lx)  ((rtli)->logX = (lx))

#define rtliGetLogY(rtli)     (rtli)->logY
#define rtliSetLogY(rtli,ly)  ((rtli)->logY = (ly))

#define rtliGetLogXFinal(rtli)     (rtli)->logXFinal
#define rtliSetLogXFinal(rtli,lxf) ((rtli)->logXFinal = (lxf))

#define rtliGetLogXSignalInfo(rtli)     (rtli)->logXSignalInfo.cptr
#define rtliSetLogXSignalInfo(rtli,lxi) ((rtli)->logXSignalInfo.cptr = (lxi))
#define _rtliGetLogXSignalInfo(rtli)    (rtli)->logXSignalInfo.ptr

#define rtliGetLogYSignalInfo(rtli)     (rtli)->logYSignalInfo.cptr
#define rtliSetLogYSignalInfo(rtli,lyi) ((rtli)->logYSignalInfo.cptr = (lyi))
#define _rtliGetLogYSignalInfo(rtli)    (rtli)->logYSignalInfo.ptr

#define rtliGetLogXSignalPtrs(rtli)     (rtli)->logXSignalPtrs.cptr
#define rtliSetLogXSignalPtrs(rtli,lxp) ((rtli)->logXSignalPtrs.cptr = (lxp))
#define _rtliGetLogXSignalPtrs(rtli)    (rtli)->logXSignalPtrs.ptr

#define rtliGetLogYSignalPtrs(rtli)     (rtli)->logYSignalPtrs.cptr
#define rtliSetLogYSignalPtrs(rtli,lyp) ((rtli)->logYSignalPtrs.cptr = (lyp))
#define _rtliGetLogYSignalPtrs(rtli)    (rtli)->logYSignalPtrs.ptr

#define rtliGetMMI(rtli)     (rtli)->mmi
#define rtliSetMMI(rtli,mmiIn) ((rtli)->mmi = ((void *)mmiIn))

#define rtliGetLoggingInterval(rtli)     (rtli)->loggingInterval

/* ========================================================================== */

#endif /* __RTW_MATLOGGING_H__ */

