//----------------------------------------------------------------------
// ComposeOutput.c:
//   Compose output string and send to output port (UART etc.)
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include "ChannelManager.h"
#include "TEManager.h"
#include "PlatformCtrl.h"
#include "BBDefines.h"

//*************** Task to output baseband measurements ****************
// Parameters:
//   Param: Pointer to measurement parameter structure
// Return value:
//   0
int MeasPrintTask(void *Param)
{
	int i;
	PBB_MEAS_PARAM MeasParam = (PBB_MEAS_PARAM)Param;
	PBB_MEASUREMENT Msr = BasebandMeasurement;
	U32 ChannelMask;

	DEBUG_OUTPUT(OUTPUT_CONTROL(OUTPUT, INFO), "$PMSRP,%d,%d,%d,%d\n", __builtin_popcount(MeasParam->MeasMask), MeasParam->TickCount, MeasParam->Interval, MeasParam->ClockAdjust);
	for (i = 0, ChannelMask = 1; i < TOTAL_CHANNEL_NUMBER; i ++, ChannelMask <<= 1)
	{
		if ((MeasParam->MeasMask & ChannelMask) == 0)
			continue;
		DEBUG_OUTPUT(OUTPUT_CONTROL(OUTPUT, INFO), "$PBMSR,%2d,%2d,%2d,%10u,%10u,%10u,%5d,%10u,%5d,%9d,%8x,%4d,%8u\n",
			Msr[i].ChannelState->LogicChannel, Msr[i].ChannelState->Svid, Msr[i].ChannelState->FreqID,
			Msr[i].CarrierFreq, Msr[i].CarrierPhase, Msr[i].CarrierCount, Msr[i].CodeCount, Msr[i].CodePhase, 2046,
			Msr[i].WeekMsCount, Msr[i].ChannelState->State, Msr[i].ChannelState->CN0, Msr[i].ChannelState->TrackingTime);
	}
	DEBUG_OUTPUT(OUTPUT_CONTROL(OUTPUT, INFO), "$PMSRE,%c,%d,%d\n", "UECKA"[MeasParam->TimeQuality], MeasParam->GpsMsCount, MeasParam->BdsMsCount);
	return 0;
}

int BasebandDataOutput(void* Param)
{
	PDATA_FOR_DECODE DataForDecode = (PDATA_FOR_DECODE)Param;

	DEBUG_OUTPUT(OUTPUT_CONTROL(OUTPUT, INFO), "$PDATA,%2d,%2d,%2d,%5d,%10d,%08x\n",
		DataForDecode->ChannelState->LogicChannel, DataForDecode->ChannelState->Svid, DataForDecode->ChannelState->FreqID,
		DataForDecode->SymbolIndex, DataForDecode->TickCount, DataForDecode->DataStream);
	return 0;
}
