//----------------------------------------------------------------------
// PlatformCtrl.h:
//   Declaration for OS and platform functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __PLATFORM_CTRL_H__
#define __PLATFORM_CTRL_H__

#include <stdio.h>
#include "CommonDefines.h"

// output message level control
#define OUTPUT_LEVEL_NONE -1
#define OUTPUT_LEVEL_INFO 0
#define OUTPUT_LEVEL_WARNING 1
#define OUTPUT_LEVEL_ERROR 2
#define OUTPUT_LEVEL_CRITICAL 3
#define OUTPUT_LEVEL_OFF 4
// debug output mask (will output corresponding level and above)
#define OUTPUT_MASK_ACQUISITION     OUTPUT_LEVEL_INFO
#define OUTPUT_MASK_COH_PROC        OUTPUT_LEVEL_WARNING
#define OUTPUT_MASK_TRACKING_LOOP   OUTPUT_LEVEL_WARNING
#define OUTPUT_MASK_TRACKING_SWITCH OUTPUT_LEVEL_WARNING
#define OUTPUT_MASK_DATA_DECODE     OUTPUT_LEVEL_WARNING
#define OUTPUT_MASK_MEASUREMENT     OUTPUT_LEVEL_WARNING
#define OUTPUT_MASK_OUTPUT          OUTPUT_LEVEL_WARNING
#define OUTPUT_MASK_PVT             OUTPUT_LEVEL_INFO

#define OUTPUT_CONTROL(type, level) (OUTPUT_MASK_##type <= OUTPUT_LEVEL_##level)

// debug output control
extern FILE *fp_debug;
#define DEBUG_OUTPUT(enable, ...) if(enable&&fp_debug) fprintf(fp_debug, __VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ThreadFunction)(void *Param);

// thread control
void CreateThread(ThreadFunction Thread, int Priority, void *Param);
// IPC functions
void ENTER_CRITICAL();
void EXIT_CRITICAL();
U32 EventCreate();
void EventSet(U32 Event);
void EventWait(U32 Event);

int __builtin_popcount(unsigned int data);
int __builtin_clz(unsigned int data);

// saved parameter read/write
int LoadParameters(int Offset, void *Buffer, int Size);
void SaveParameters(int Offset, void *Buffer, int Size);

#ifdef __cplusplus
}
#endif

#endif	// __PLATFORM_CTRL_H__
