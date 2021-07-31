//----------------------------------------------------------------------
// TEManager.c:
//   Tracking engine management functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------
#include <stdio.h>
#include "RegAddress.h"
#include "BBDefines.h"
#include "HWCtrl.h"
#include "FirmwarePortal.h"
#include "TaskQueue.h"
#include "ChannelManager.h"
#include "TEManager.h"

int MeasurementInterval;
U32 ChannelOccupation;
BB_MEASUREMENT BasebandMeasurement[32];
U32 DataStreamBuffer[100/4*32];		// 100 8bit symbols x 32 channels
BB_MEAS_PARAM MeasurementParam;

int MeasProcTask(void *Param);

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

//*************** Process measurement interrupt ****************
// Parameters:
//   none
// Return value:
//   none
void MeasurementProc()
{
	int i, ch_num = 0;
	U32 ChannelMask;
	U32 *BufferPointer = DataStreamBuffer;
	PBB_MEASUREMENT Msr;
	int WordNumber;

	for (i = 0, ChannelMask = 1; i < 32; i ++, ChannelMask <<= 1)
	{
		if (ChannelOccupation & ChannelMask)
		{
			Msr = &BasebandMeasurement[ch_num++];
			BufferPointer += (WordNumber = ComposeMeasurement(i, Msr, BufferPointer));
		}
	}

	// assign measurement parameter structure and add process task to PostMeasTask queue
	MeasurementParam.MeasNumber = ch_num;
	MeasurementParam.MeasInterval = MeasurementInterval;
	MeasurementParam.Measurements = BasebandMeasurement;
	AddTaskToQueue(&PostMeasTask, MeasProcTask, &MeasurementParam, sizeof(BB_MEAS_PARAM));
}

//*************** Task to process baseband measurements ****************
//* This task is added to and called within PostMeasTask queue
// Parameters:
//   Param: Pointer to measurement parameter structure
// Return value:
//   0
int MeasProcTask(void *Param)
{
	int i, j;
	PBB_MEAS_PARAM MeasParam = (PBB_MEAS_PARAM)Param;
	PBB_MEASUREMENT Msr = MeasParam->Measurements;
	int WordNumber;

	for (i = 0; i < MeasParam->MeasNumber; i ++)
	{
		WordNumber = (Msr[i].DataNumber + 31) / 32;
		printf("$PBMSR,%2d,%2d,%2d,%10u,%10u,%10u,%5d,%10u,%10u,%5d,%8x,%3d,%4d,%8u\n",
			Msr[i].LogicChannel, Msr[i].Svid, Msr[i].FreqID, Msr[i].CarrierFreq, Msr[i].CarrierNCO, Msr[i].CarrierCount,
			Msr[i].CodeCount, Msr[i].CodeFreq, Msr[i].CodeNCO, 2046, Msr[i].State, Msr[i].LockIndicator, Msr[i].CN0, Msr[i].TrackingTime);
		if (WordNumber > 0)
		{
			printf("$PDATA,%d", Msr[i].DataNumber);
			for (j = 0; j < WordNumber; j ++)
				printf(",%08x", Msr[i].DataStreamAddr[j]);
			printf("\n");
		}
	}
	printf("$PMSRP,%3d\n", MeasParam->MeasInterval);

	return 0;
}
