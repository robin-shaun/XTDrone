/* Copyright 2004-2016 The MathWorks, Inc. */

/*
 * File: sysran_types.h
 *
 * Abstract:
 *   This files defines the enum and the macros to do with the system ran
 *   breadcrumbs
 * 
 *   This file is used both by Simulink and some of the generated code
 */

#ifndef SIMSTRUCT_SYSRAN_TYPES_H
#define SIMSTRUCT_SYSRAN_TYPES_H

/* Subsystem run state -- used by Model Reference simulation target */
typedef enum {
    SUBSYS_RAN_BC_DISABLE,
    SUBSYS_RAN_BC_ENABLE,
    SUBSYS_RAN_BC_DISABLE_TO_ENABLE,
    SUBSYS_RAN_BC_ENABLE_TO_DISABLE,
    SUBSYS_RAN_BC_ONE_SHOT
} SubSystemRanBCTransition;

/* This is the data type of the dwork entry for the system ran breadcrumb */
typedef int8_T sysRanDType;

/* 
 * System ran breadcrumb macros
 *
 * Clearing macro.  2 callers
 *    i) top of the sim-loop
 *   ii) top of each Outputs function for External mode
 *  
 *
 * Here is the basic idea:
 *   If a system is DISABLED, it stays DISABLED.
 *   For any other state, assume ENABLE_TO_DISABLE,
 *     If it runs then the OutputFcn will move it to ENABLE
 *     otherwise it will come back as  ENABLE_TO_DISABLE(hence it never ran)
 *
 * DISABLE            -> DISABLE  (latched)
 * ENABLE             -> ENABLE_TO_DISABLE (assume)
 * ENABLE_TO_DISABLE  -> DISABLE (new latch)
 * DISABLE_TO_ENABLE  -> ENABLE  (new enable)
 * ONE_SHOT           -> DISABLE (by definition)
 *
 * Note: These macros are called both by Simulink and the generated code
 */ 

/* sr stands for SystemRan, BC stands for BreadCrumb */
#define srClearBC(state)                                                   \
{                                                                          \
    SubSystemRanBCTransition prevState = (SubSystemRanBCTransition) state; \
    SubSystemRanBCTransition newState  = SUBSYS_RAN_BC_DISABLE;            \
                                                                           \
    switch(prevState) {                                                    \
      case SUBSYS_RAN_BC_DISABLE:                                          \
        newState = SUBSYS_RAN_BC_DISABLE;                                  \
        break;                                                             \
      case SUBSYS_RAN_BC_ENABLE:                                           \
        newState = SUBSYS_RAN_BC_ENABLE_TO_DISABLE;                        \
        break;                                                             \
      case SUBSYS_RAN_BC_DISABLE_TO_ENABLE:                                \
        newState = SUBSYS_RAN_BC_ENABLE_TO_DISABLE;                        \
        break;                                                             \
      case SUBSYS_RAN_BC_ENABLE_TO_DISABLE:                                \
        newState = SUBSYS_RAN_BC_DISABLE;                                  \
        break;                                                             \
      case SUBSYS_RAN_BC_ONE_SHOT:                                         \
        newState = SUBSYS_RAN_BC_DISABLE;                                  \
        break;                                                             \
    }                                                                      \
    state = newState;                                                      \
}

/* 
 * Update macro
 *
 * Called by the OutputFcn of all conditionally exec'd subsystems
 * from subsystm.cpp and commonbodlib.tlc
 *
 * DISABLE           -> DISABLE_ENABLE (new enable)
 * ENABLE_TO_DISABLE -> ENABLE (must have run last time step)
 * All other state   -> should really be an assert
 *
 */
/* sr stands for SystemRan, BC stands for BreadCrumb */
#define srUpdateBC(state)                                                  \
{                                                                          \
    SubSystemRanBCTransition prevState = (SubSystemRanBCTransition) state; \
    SubSystemRanBCTransition newState  = prevState;                        \
                                                                           \
    switch(prevState) {                                                    \
      case SUBSYS_RAN_BC_DISABLE:                                          \
        newState = SUBSYS_RAN_BC_DISABLE_TO_ENABLE;                        \
        break;                                                             \
      case SUBSYS_RAN_BC_ENABLE_TO_DISABLE:                                \
        newState = SUBSYS_RAN_BC_ENABLE;                                   \
        break;                                                             \
      case SUBSYS_RAN_BC_ENABLE:                                           \
      case SUBSYS_RAN_BC_DISABLE_TO_ENABLE:                                \
      case SUBSYS_RAN_BC_ONE_SHOT:                                         \
        break;                                                             \
    }                                                                      \
    state = newState;                                                      \
}

#endif /* SIMSTRUCT_SYSRAN_TYPES_H */

/* EOF sysran_types.h */

/* LocalWords:  sr exec'd subsystm commonbodlib
 */
