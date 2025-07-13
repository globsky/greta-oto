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
void MsrProc(PBB_MEAS_PARAM MeasParam);
void PvtProcInit(PRECEIVER_INFO pReceiverInfo);
void PvtProc(int CurMsInterval);
void GpsDecodeInit();
void BdsDecodeInit();
int BdsDecodeTask(void *Param);
void BdsFrameDecode(int LogicChannel, unsigned short *FrameBuffer, int ResiduleBits);
PRECEIVER_INFO GetReceiverInfo();
int GetSatelliteInView(SAT_PREDICT_PARAM SatList[32]);

#endif //__PVT_ENTRY_H__
