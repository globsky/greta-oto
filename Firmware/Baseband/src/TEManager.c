//----------------------------------------------------------------------
// TEManager.c:
//   Tracking engine management functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "RegAddress.h"
#include "BBDefines.h"
#include "HWCtrl.h"
#include "ChannelManager.h"
#include "TEManager.h"

U32 ChannelOccupation;

//*************** Update all channel state in hardware synchronized from cache ****************
// Parameters:
//   none
// Return value:
//   none
void UpdateChannels()
{
	int i;
	U32 ChannelMask;

	for (i = 0, ChannelMask = 1; i < 32; i ++, ChannelMask <<= 1)
	{
		if (ChannelOccupation & ChannelMask)
			SyncCacheWrite(ChannelStateArray + i);
	}
}

//*************** Get one available channel (not occupied channel) ****************
// Parameters:
//   none
// Return value:
//   pointer to corresponding channel state structure, or null pointer if no available channel
PCHANNEL_STATE GetAvailableChannel()
{
	int i;

	for (i = 0; i < 32; i ++)
	{
		if ((ChannelOccupation & (1 << i)) == 0)
		{
			ChannelOccupation |= (1 << i);
			return ChannelStateArray + i;
		}
	}
	return (PCHANNEL_STATE)0;
}

//*************** Process coherent sum interrupt ****************
// Parameters:
//   none
// Return value:
//   none
void CohSumInterruptProc()
{
	int i;
	U32 ChannelMask;
	U32 CohDataReady = GetRegValue(ADDR_TE_COH_DATA_READY);

	for (i = 0, ChannelMask = 1; i < 32; i ++, ChannelMask <<= 1)
	{
		if (CohDataReady & ChannelMask)
			ProcessCohSum(i);
	}
	UpdateChannels();
}
