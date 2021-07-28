//----------------------------------------------------------------------
// HWCtrl.h:
//   Declaration for hardware access
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __HARDWARE_CTRL_H__
#define __HARDWARE_CTRL_H__

#include "CommonDefines.h"

#ifdef __cplusplus
extern "C" {
#endif

// declare a pointer point to a ISR, only for simulation envirenoment
typedef void (*InterruptFunction)();
// map interrupt service function
void AttachBasebandISR(InterruptFunction ISR);
// host read/write
U32 GetRegValue(int Address);
void SetRegValue(int Address, U32 Value);
// baseband memory load/save functions
void LoadMemory(U32 *DestAddr, U32 *BasebandAddr, int Size);
void SaveMemory(U32 *BasebandAddr, U32 *SrcAddr, int Size);
// input file for PC simulation
void SetInputFile(char *FileName);
// saved parameter read/write
void LoadParameters(void *Buffer);
void SaveParameters(void *Buffer);
// RF control
void EnableRF();

#ifdef __cplusplus
}
#endif

#endif	// __HARDWARE_CTRL_H__
