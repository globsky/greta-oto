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
void MsrProc(PBB_MEASUREMENT Measurements, unsigned int ActiveMask, int CurMsInterval, int DefaultMsInterval);
void PvtProcInit(PRECEIVER_INFO pReceiverInfo);
void PvtProc(int CurMsInterval);
PRECEIVER_INFO GetReceiverInfo();

#endif //__PVT_ENTRY_H__
