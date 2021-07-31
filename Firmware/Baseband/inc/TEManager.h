//----------------------------------------------------------------------
// TEManager.h:
//   Tracking engine management functions and definitions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __TE_MANAGER_H__
#define __TE_MANAGER_H__

#include "CommonDefines.h"
#include "ChannelManager.h"

extern int MeasurementInterval;
extern U32 ChannelOccupation;
extern BB_MEASUREMENT BasebandMeasurement[32];

void UpdateChannels();
PCHANNEL_STATE GetAvailableChannel();
void CohSumInterruptProc();
void MeasurementProc();

#endif // __TE_MANAGER_H__
