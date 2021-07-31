//----------------------------------------------------------------------
// FirmwarePortal.h:
//   Firmware portal functions and support functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __FIRMWARE_PORTAL_H__
#define __FIRMWARE_PORTAL_H__

#include "CommonDefines.h"
#include "TaskQueue.h"

extern TASK_QUEUE RequestTask;
extern TASK_QUEUE BasebandTask;
extern TASK_QUEUE PostMeasTask;

void FirmwareInitialize();

#endif // __FIRMWARE_PORTAL_H__
