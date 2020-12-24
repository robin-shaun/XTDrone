/* Copyright 2015-2016 The MathWorks, Inc. */

#ifndef SLTESTRESULT_H
#define SLTESTRESULT_H

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef DEFINED_TYPEDEF_FOR_slTestResult_
#define DEFINED_TYPEDEF_FOR_slTestResult_
#ifndef _DEFINED_TYPEDEF_FOR_slTestResult_
#define _DEFINED_TYPEDEF_FOR_slTestResult_
#ifndef enum_slTestResult
#define enum_slTestResult

typedef enum {
   slTestResult_Untested = -1,
   slTestResult_Pass,
   slTestResult_Fail
} slTestResult;

#endif
#endif
#endif
#endif

