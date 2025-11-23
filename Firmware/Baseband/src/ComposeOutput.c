//----------------------------------------------------------------------
// ComposeOutput.c:
//   Compose output string and send to output port (UART etc.)
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "ChannelManager.h"
#include "TEManager.h"
#include "PlatformCtrl.h"
#include "BBDefines.h"

int OutputBasebandMeasPort = DEFAULT_BB_MEAS_PORT;
int OutputBasebandDataPort = DEFAULT_BB_DATA_PORT;

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
	char OutputBuffer[256];

	if (!PortOpened(OutputBasebandMeasPort))
		return 0;
	sprintf(OutputBuffer, "$PMSRP,%d,%d,%d,%d\r\n", __builtin_popcount(MeasParam->MeasMask), MeasParam->TickCount, MeasParam->Interval, MeasParam->ClockAdjust);
	WriteStreamPort(OutputBasebandMeasPort, OutputBuffer, strlen(OutputBuffer));
	for (i = 0, ChannelMask = 1; i < TOTAL_CHANNEL_NUMBER; i ++, ChannelMask <<= 1)
	{
		if ((MeasParam->MeasMask & ChannelMask) == 0)
			continue;
		sprintf(OutputBuffer, "$PBMSR,%2d,%2d,%2d,%10u,%10u,%10u,%5d,%10u,%5d,%9d,%8x,%4d,%8u\r\n",
			Msr[i].ChannelState->LogicChannel, Msr[i].ChannelState->Svid, Msr[i].ChannelState->FreqID,
			Msr[i].CarrierFreq, Msr[i].CarrierPhase, Msr[i].CarrierCount, Msr[i].CodeCount, Msr[i].CodePhase, 2046,
			Msr[i].WeekMsCount, Msr[i].ChannelState->State, Msr[i].ChannelState->CN0, Msr[i].ChannelState->TrackingTime);
		WriteStreamPort(OutputBasebandMeasPort, OutputBuffer, strlen(OutputBuffer));
	}
	sprintf(OutputBuffer, "$PMSRE,%c,%d,%d\r\n", "UECKA"[MeasParam->TimeQuality], MeasParam->GpsMsCount, MeasParam->BdsMsCount);
	WriteStreamPort(OutputBasebandMeasPort, OutputBuffer, strlen(OutputBuffer));
	return 0;
}

int BasebandDataOutput(void* Param)
{
	PDATA_FOR_DECODE DataForDecode = (PDATA_FOR_DECODE)Param;
	char OutputBuffer[64];

	if (!PortOpened(OutputBasebandDataPort))
		return 0;
	sprintf(OutputBuffer, "$PDATA,%2d,%2d,%2d,%5d,%10d,%08x\r\n",
		DataForDecode->ChannelState->LogicChannel, DataForDecode->ChannelState->Svid, DataForDecode->ChannelState->FreqID,
		DataForDecode->SymbolIndex, DataForDecode->TickCount, DataForDecode->DataStream);
	WriteStreamPort(OutputBasebandDataPort, OutputBuffer, strlen(OutputBuffer));
	return 0;
}
