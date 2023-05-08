//----------------------------------------------------------------------
// PlatformCtrl.h:
//   Declaration for OS and platform functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __PLATFORM_CTRL_H__
#define __PLATFORM_CTRL_H__

#include "CommonDefines.h"

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
