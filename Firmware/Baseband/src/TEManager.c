//----------------------------------------------------------------------
// TEManager.c:
//   Tracking engine management functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------
#include <string.h>
#include "RegAddress.h"
#include "BBDefines.h"
#include "HWCtrl.h"
#include "FirmwarePortal.h"
#include "TaskManager.h"
#include "ChannelManager.h"
#include "TEManager.h"
#include "TimeManager.h"
#include "PvtEntry.h"
#include "ComposeOutput.h"

int MeasurementInterval;
unsigned int MeasIntCounter;
unsigned int BasebandTickCount;
U32 ChannelOccupation;
BB_MEASUREMENT BasebandMeasurement[TOTAL_CHANNEL_NUMBER];
BB_MEAS_PARAM MeasurementParam;

int MeasProcTask(void *Param);

//*************** Initialize TE manager ****************
//* this function is called at initialization stage
// Parameters:
//   none
// Return value:
//   none
void TEInitialize()
{
	int i;

	MeasIntCounter = BasebandTickCount = 0;
	ChannelOccupation = 0;
	memset(ChannelStateArray, 0, sizeof(ChannelStateArray));
	for (i = 0; i < TOTAL_CHANNEL_NUMBER; i ++)
	{
		ChannelStateArray[i].LogicChannel = i;
		ChannelStateArray[i].StateBufferHW = (PSTATE_BUFFER)(ADDR_BASE_TE_BUFFER + i * 128);	// each channel occupy 128 bytes start from ADDR_BASE_TE_BUFFER
	}
}

//*************** Get channel enable mask ****************
// Parameters:
//   none
// Return value:
//   channel enable mask
U32 GetChannelEnable()
{
	return ChannelOccupation;
}

//*************** Update all channel state in hardware synchronized from cache ****************
// Parameters:
//   none
// Return value:
//   none
void UpdateChannels()
{
	int i;
	U32 ChannelMask;

	for (i = 0, ChannelMask = 1; i < TOTAL_CHANNEL_NUMBER; i ++, ChannelMask <<= 1)
	{
		if (ChannelOccupation & ChannelMask)
		{
			if ((ChannelStateArray[i].State & STAGE_MASK) == STAGE_RELEASE)
				ReleaseChannel(i);
			else
				SyncCacheWrite(ChannelStateArray + i);
		}
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

	for (i = 0; i < TOTAL_CHANNEL_NUMBER; i ++)
	{
		if ((ChannelOccupation & (1 << i)) == 0)
		{
			ChannelOccupation |= (1 << i);
			return ChannelStateArray + i;
		}
	}
	return (PCHANNEL_STATE)0;
}

//*************** Release a channel and stop track ****************
// Parameters:
//   ChannelID: channel ID (index) to release
// Return value:
//   none
void ReleaseChannel(int ChannelID)
{
	ChannelOccupation &= ~(1 << ChannelID);
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
	U32 OverwriteProtectChannel = GetRegValue(ADDR_TE_OVERWRITE_PROTECT_CHANNEL);

	for (i = 0, ChannelMask = 1; i < TOTAL_CHANNEL_NUMBER; i ++, ChannelMask <<= 1)
	{
		if (CohDataReady & ChannelMask)
			ProcessCohSum(i, OverwriteProtectChannel & ChannelMask);
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
	PBB_MEASUREMENT Msr;

	for (i = 0, ChannelMask = 1; i < TOTAL_CHANNEL_NUMBER; i ++, ChannelMask <<= 1)
	{
		if (ChannelOccupation & ChannelMask)
		{
			Msr = &BasebandMeasurement[i];
			ComposeMeasurement(i, Msr);
		}
	}

	// assign measurement parameter structure and add process task to PostMeasTask queue
	MeasurementParam.MeasMask = ChannelOccupation;
	MeasurementParam.Interval = MeasurementInterval;
/*	if (ClockAdjustment != 0)	// there is receiver clock adjustment
	{
		if (MeasInterval != NominalMeasInterval)	// To3 epoch with interval adjustment
		{
			MeasurementData.ClockAdjust = ClockAdjustment;
			ClockAdjustment = 0;	// clear to 0 to be sure adjustment to be used once
		}
		else if (NextInterval == NominalMeasInterval)	// To2 epoch with clock adjustment only
		{
			MeasurementData.ClockAdjust = ClockAdjustment;
			ClockAdjustment = 0;	// clear to 0 to be sure adjustment to be used once
		}
		else	// To2 epoch with interval adjustment
			MeasurementData.ClockAdjust = 0;
	}
	else*/
		MeasurementParam.ClockAdjust = 0;
	MeasurementParam.TickCount = BasebandTickCount;
	AddToTask(TASK_POSTMEAS, MeasProcTask, &MeasurementParam, sizeof(BB_MEAS_PARAM));
//	printf("MSR %d\n", MeasurementParam.RunTimeAcc);
}

//*************** Task to process baseband measurements ****************
//* This task is added to and called within PostMeasTask queue
// Parameters:
//   Param: Pointer to measurement parameter structure
// Return value:
//   0
int MeasProcTask(void *Param)
{
	int OutputBasebandMeas = 1;
	PBB_MEAS_PARAM MeasParam = (PBB_MEAS_PARAM)Param;
	PBB_MEASUREMENT Msr = BasebandMeasurement;

	// update receiver time to current epoch
	UpdateReceiverTime(MeasParam->TickCount, MeasParam->Interval - MeasParam->ClockAdjust);
	// calculate raw measurement
	MsrProc(MeasParam);
	// do PVT
	PvtProc(MeasParam->Interval);
	if (OutputBasebandMeas)
	{
		MeasParam->TimeQuality = GnssTime.TimeQuality;
		MeasParam->GpsMsCount = GnssTime.GpsMsCount;
		MeasParam->BdsMsCount = GnssTime.BdsMsCount;
		AddToTask(TASK_INOUT, MeasPrintTask, Param, sizeof(BB_MEAS_PARAM));
	}

	return 0;
}
