//----------------------------------------------------------------------
// HWCtrl_Dummy.cpp:
//   Dummy functions for definitions in HWCtrl.h, for post process without baseband hardware
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "CommonDefines.h"
#include "AEManager.h"
#include "TEManager.h"

typedef void (*InterruptFunction)();
typedef void (*DebugFunction)(void *DebugParam, int DebugValue);

// functions for GNSS hardware access
void AttachBasebandISR(InterruptFunction ISR) {}
void AttachDebugFunc(DebugFunction Function) {}
U32 GetRegValue(int Address) { return 0; }
void SetRegValue(int Address, U32 Value) {}
U32 GetRequestCount() { return 0; }
void SetRequestCount(U32 Count) {}
void LoadMemory(U32 *DestAddr, U32 *BasebandAddr, int Size) {}
void SaveMemory(U32 *BasebandAddr, U32 *SrcAddr, int Size) {}
void SetInputFile(char *FileName) {}
void EnableRF() {}

// other baseband functions
void AEInitialize(void) {}
PACQ_CONFIG GetFreeAcqTask(void) { return NULL; }
int AddAcqTask(PACQ_CONFIG pAcqConfig) { return 0; }
void StartAcquisition(void *Param) {}
int AcqBufferReachTh(void) { return 0; }
int AdjustMeasInterval(void* Param) { return 0; }
void AcqIntService() {}
void TEInitialize() {}
