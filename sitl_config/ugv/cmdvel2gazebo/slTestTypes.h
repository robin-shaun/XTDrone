/* Copyright 2015-2016 The MathWorks, Inc. */

#ifndef SLTESTTYPES_H
#define SLTESTTYPES_H

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

typedef struct slTestBlkInfo {
    char* blkPath;
    int blkId;
    void* targetSpecificInfo;
    char* mdlRefFullPath;
} slTestBlkInfo_t;

#endif
