//----------------------------------------------------------------------
// PvtEntry.h:
//   Definition of PVT functions called by top module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __PVT_ENTRY_H__
#define __PVT_ENTRY_H__

#include "DataTypes.h"

// basic PVT entry functions
void MsrProcInit();
void PvtProcInit(StartType Start, PSYSTEM_TIME CurTime, LLH *CurPosition);
void PvtProc(int CurMsInterval, int ClockAdjust);
void GpsDecodeInit();
void BdsDecodeInit();
int BdsDecodeTask(void *Param);
void BdsFrameDecode(int LogicChannel, unsigned short *FrameBuffer, int ResiduleBits);
PRECEIVER_INFO GetReceiverInfo();
double GetClockError(int FirstPriorityFreq);
int GetSatelliteInView(SAT_PREDICT_PARAM SatList[32]);

#endif //__PVT_ENTRY_H__
