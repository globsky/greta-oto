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

//typedef enum {ParamConfig, ParamReceiverInfo, ParamIonoUtc, ParamGpsEph, ParamBdsEph, ParamGalEph, ParamGpsAlm, ParamBdsAlm, ParamGalAlm } ParameterType;

// declare a pointer to a ISR, only for simulation envirenoment
typedef void (*InterruptFunction)();
// declare a pointer to output debug information, only for simulation envirenoment
typedef void (*DebugFunction)(void *DebugParam, int DebugValue);

// map interrupt service function
void AttachBasebandISR(InterruptFunction ISR);
// map interrupt service function
void AttachDebugFunc(DebugFunction Function);
// host read/write
U32 GetRegValue(int Address);
void SetRegValue(int Address, U32 Value);
// baseband memory load/save functions
void LoadMemory(U32 *DestAddr, U32 *BasebandAddr, int Size);
void SaveMemory(U32 *BasebandAddr, U32 *SrcAddr, int Size);
// input file for PC simulation
void SetInputFile(char *FileName);
// saved parameter read/write
int LoadParameters(int Offset, void *Buffer, int Size);
void SaveParameters(int Offset, void *Buffer, int Size);
// RF control
void EnableRF();

#ifdef __cplusplus
}
#endif

#endif	// __HARDWARE_CTRL_H__
