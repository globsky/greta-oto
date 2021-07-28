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

// interrupt control
void EnableInt();
void DisableInt();
// thread control
void CreateThread(ThreadFunction Thread);
// IPC functions
#define ENTER_CRITICAL DisableInt
#define EXIT_CRITICAL EnableInt

#ifdef __cplusplus
}
#endif

#endif	// __PLATFORM_CTRL_H__
