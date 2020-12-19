/* Copyright 2010-2013 The MathWorks, Inc. */

/*
 * File: sl_sample_time_defs.h
 *
 * Abstract:
 *   This file contains sample time related definitions (typedefs, macros, etc.)
 *   prototypes that are used both by User S-Functions and inside Simulink.
 */

#ifndef __SL_SAMPLE_TIME_DEFS_H__
#define __SL_SAMPLE_TIME_DEFS_H__

/*
 * INHERITED_SAMPLE_TIME      - Specify for blocks that inherit their sample
 *                              time from the block that feeds their input.
 *
 * CONTINUOUS_SAMPLE_TIME     - A continuous sample time indicates that the
 *                              block executes every simulation step.
 *
 * VARIABLE_SAMPLE_TIME       - Specifies that this sample time is discrete
 *                              with a varying period.
 *
 * FIXED_IN_MINOR_STEP_OFFSET - This can be specified for the offset of either
 *                              the inherited or continuous sample time
 *                              indicating that the output does not change
 *                              in minor steps.
 */
#define INHERITED_SAMPLE_TIME      ((real_T)-1.0)
#define CONTINUOUS_SAMPLE_TIME     ((real_T)0.0)
#define VARIABLE_SAMPLE_TIME       ((real_T)-2.0)
#define FIXED_IN_MINOR_STEP_OFFSET ((real_T)1.0)


typedef enum{
    SS_TIMESOURCE_BASERATE,   /* async task does not manage time,
                    reads absolute time from base rate */
    SS_TIMESOURCE_SELF,        /* async task manage it own independent
                    timer */
    SS_TIMESOURCE_CALLER,       /* async task read time from its upstream
                    task */
    SS_TIMESOURCE_SELF_INTERNAL, /* If an async task periodically executes,
                                  * the task may choose to maintain internal timer
                                  * by incrementing at each execution */
    SS_INVALID_TIMESOURCE
} slTimeSource;

/* Value/representation of an unknown tid */
#define UNKNOWN_TID   (-5)

#endif
