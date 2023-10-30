//----------------------------------------------------------------------
// ComposeOutput.c:
//   Compose output string and send to output port (UART etc.)
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include "PlatformCtrl.h"
#include "BBDefines.h"

//*************** Task to output baseband measurements ****************
// Parameters:
//   Param: Pointer to measurement parameter structure
// Return value:
//   0
int MeasPrintTask(void *Param)
{
	int i, j;
	PBB_MEAS_PARAM MeasParam = (PBB_MEAS_PARAM)Param;
	PBB_MEASUREMENT Msr = MeasParam->Measurements;
	int WordNumber;
	U32 ChannelMask;

	for (i = 0, ChannelMask = 1; i < TOTAL_CHANNEL_NUMBER; i ++, ChannelMask <<= 1)
	{
		if ((MeasParam->MeasMask & ChannelMask) == 0)
			continue;
//		if (Msr[i].Svid != 19)
//			continue;
		if ((Msr[i].State & DATA_STREAM_MASK) == DATA_STREAM_1BIT)
			WordNumber = (Msr[i].DataNumber + 31) / 32;
		else if ((Msr[i].State & DATA_STREAM_MASK) == DATA_STREAM_4BIT)
			WordNumber = (Msr[i].DataNumber + 7) / 8;
		else if ((Msr[i].State & DATA_STREAM_MASK) == DATA_STREAM_8BIT)
			WordNumber = (Msr[i].DataNumber + 3) / 4;
		else
			WordNumber = 0;
		DEBUG_OUTPUT(OUTPUT_CONTROL(OUTPUT, INFO), "$PBMSR,%2d,%2d,%2d,%10u,%10u,%10u,%5d,%10u,%10u,%5d,%8x,%3d,%4d,%8u\n",
			Msr[i].LogicChannel, Msr[i].Svid, Msr[i].FreqID, Msr[i].CarrierFreq, Msr[i].CarrierNCO, Msr[i].CarrierCount,
			Msr[i].CodeFreq, Msr[i].CodeCount, Msr[i].CodeNCO, 2046, Msr[i].State, Msr[i].LockIndicator, Msr[i].CN0, Msr[i].TrackingTime);
		if (WordNumber > 0)
		{
			DEBUG_OUTPUT(OUTPUT_CONTROL(OUTPUT, INFO), "$PDATA,%d,%d", Msr[i].DataNumber, Msr[i].FrameIndex);
			for (j = 0; j < WordNumber; j ++)
				DEBUG_OUTPUT(OUTPUT_CONTROL(OUTPUT, INFO), ",%08x", Msr[i].DataStreamAddr[j]);
			DEBUG_OUTPUT(OUTPUT_CONTROL(OUTPUT, INFO), "\n");
		}
	}
	DEBUG_OUTPUT(OUTPUT_CONTROL(OUTPUT, INFO), "$PMSRP,%3d,%d\n", MeasParam->MeasInterval, MeasParam->RunTimeAcc);
	return 0;
}
