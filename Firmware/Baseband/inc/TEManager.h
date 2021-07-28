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

extern U32 ChannelOccupation;

void UpdateChannels();
PCHANNEL_STATE GetAvailableChannel();
void CohSumInterruptProc();

#endif // __TE_MANAGER_H__
