/* Copyright 2013-2017 The MathWorks, Inc. */
#ifndef _LINUXINITIALIZE_H_
#define _LINUXINITIALIZE_H_
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#define CHECK_STATUS(status, expStatus, fcn) if (status != expStatus) {fprintf(stderr, "Call to %s returned error status (%d).\n", fcn, status); perror(fcn); fflush(stderr); exit(EXIT_FAILURE);}
#define CHECK_STATUS_NOT(status, errStatus, fcn) if (status == errStatus) {fprintf(stderr, "Call to %s returned error status (%d).\n", fcn, status); perror(fcn); fflush(stderr); exit(EXIT_FAILURE);}

int mw_CreateArmedTimer(double periodInSeconds);
int mw_CreateUnarmedTimer(double periodInSeconds, int idx);
void mw_ArmTimer(int idx);
void mw_WaitForTimerEvent(int fd);
void mw_WaitForTimerEventCatchup(int fd);
void mw_CreateTask(void (*taskHandler)(void), const char* taskName, int priority, int policy, int coreSelection, int coreNum);
void myWaitForThisEvent(int sigNo);
void myAddBlockForThisEvent(int sigNo);
void myAddHandlerForThisEvent(int sigNo, int sigToBlock[], int numSigToBlock, void (*sigHandler)(int));
void myRestoreDefaultHandlerForThisEvent(int sigNo);
void myRTOSInit(double baseRatePeriod, int numSubrates);
#if (MW_NUMBER_TIMER_DRIVEN_TASKS > 0) 
extern timerTaskSem;
extern void mw_init_timerTaskSem(int idx);
#endif

#define UNUSED(x) x = x

#endif