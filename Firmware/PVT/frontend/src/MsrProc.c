//----------------------------------------------------------------------
// MsrProc.c:
//   Raw measurement process functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "CommonDefines.h"
#include "PlatformCtrl.h"
#include "ChannelManager.h"
#include "TEManager.h"
#include "TaskManager.h"
#include "TimeManager.h"
#include "PvtConst.h"
#include "DataTypes.h"
#include "GlobalVar.h"
#include "SupportPackage.h"
#include "GpsFrame.h"
#include "BdsFrame.h"
#include "GalFrame.h"
#include "PvtEntry.h"
#include "ComposeOutput.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

U32 EphAlmMutex;
static int AdjustIntervalDelay;

static void MsrProc(PBB_MEAS_PARAM MeasParam);
static void CalculateRawMsr(PCHANNEL_STATUS pChannelStatus, PBB_MEASUREMENT pMsr, int MsInterval, int TimeAdjust);
static int AlignObsTime(PRECEIVER_INFO ReceiverInfo);

static void InitFrame(int ch_num)
{
	unsigned char FreqID = g_ChannelStatus[ch_num].FreqID;

	g_ChannelStatus[ch_num].LockTime = 0;
	g_ChannelStatus[ch_num].CarrierCountAcc = 0;
	g_ChannelStatus[ch_num].CarrierCountOld = 0;
	g_ChannelStatus[ch_num].FrameInfo.FrameFlag = 0;
	g_ChannelStatus[ch_num].FrameInfo.SymbolNumber = 0;
	g_ChannelStatus[ch_num].FrameInfo.FrameStatus = -1;
	g_ChannelStatus[ch_num].FrameInfo.TimeTag = -1;
}

static void InitChannelStatus(int ch_num)
{
	g_ChannelStatus[ch_num].ChannelFlag = g_ChannelStatus[ch_num].ChannelErrorFlag = 0;
	InitFrame(ch_num);
}

//*************** Raw measurement process initialization ****************
//* This task is called once at startup to do initialization
// Parameters:
//   none
// Return value:
//   none
void MsrProcInit()
{
	// clear channel status buffer
	memset(g_ChannelStatus, 0, sizeof(g_ChannelStatus));
	TimeInitialize();
	AdjustIntervalDelay = 0;
	EphAlmMutex = MutexCreate();
}

//*************** Task to do symbol decode on navigation data ****************
//* This is a baseband task, has higher priority than measurement calculation
// Parameters:
//   Param: pointer to structure of decoded symbols
// Return value:
//   none
int DoDataDecode(void* Param)
{
	PDATA_FOR_DECODE DataForDecode = (PDATA_FOR_DECODE)Param;
	PCHANNEL_STATE ChannelState = DataForDecode->ChannelState;
	PFRAME_INFO pFrameInfo = &(g_ChannelStatus[ChannelState->LogicChannel].FrameInfo);
	int WeekMs = -1, WeekNumber = -1, CurTimeMs;

	DEBUG_OUTPUT(OUTPUT_CONTROL(DATA_DECODE, INFO), "Ch%02d %c%02d decode data %08x start index%4d at time%8d\n", ChannelState->LogicChannel, "GECG"[ChannelState->FreqID], ChannelState->Svid, DataForDecode->DataStream, DataForDecode->SymbolIndex, DataForDecode->TickCount);
//	for (i = 31; i >= 0; i--)
//		DEBUG_OUTPUT(OUTPUT_CONTROL(DATA_DECODE, INFO), "%1d", (DataForDecode->DataStream & (1 << i)) ? 1 : 0);
//	DEBUG_OUTPUT(OUTPUT_CONTROL(DATA_DECODE, INFO), "\n");
//	if (OutputBasebandDataPort >= 0)
		AddToTask(TASK_INOUT, BasebandDataOutput, DataForDecode, sizeof(DATA_FOR_DECODE));

	switch (ChannelState->FreqID)
	{
		case FREQ_L1CA:
			WeekMs = GpsNavDataProc(pFrameInfo, DataForDecode);
			break;
		case FREQ_B1C:
			WeekMs = BdsNavDataProc(pFrameInfo, DataForDecode);
			WeekNumber = pFrameInfo->TimeTag;
			break;
		case FREQ_E1:
			WeekMs = GalNavDataProc(pFrameInfo, DataForDecode);
//			WeekNumber = pFrameInfo->TimeTag;
			break;
		default:
			WeekMs = -1;
	}
	if (WeekMs >= 0)
	{
		if (DataForDecode->ChannelState->SyncTickCount == 0)
			DataForDecode->ChannelState->SyncTickCount = DataForDecode->TickCount + (MS_IN_WEEK - WeekMs);	// tick count corresponding to week boundary
		if (!ReceiverWeekMsValid())
		{
//			if (FREQ_ID_IS_B1C(ChannelState->FreqID) && BDS_GEO_SVID(Svid))
//				CurTimeMs = WeekMs + 130;
//			else
				CurTimeMs = WeekMs + 80;
			SetReceiverTime(ChannelState->FreqID, WeekNumber, CurTimeMs, DataForDecode->TickCount);
		}
	}

	return 0;
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

#if !defined USE_PRE_ASSIGNED_TIME	// realtime process (baseband observation post-process will skip receiver time calculation and uses input data)
	PRECEIVER_INFO ReceiverInfo;

	if ((ReceiverInfo = GetReceiverInfo()) != NULL && ReceiverInfo->PosQuality > CoarsePos)
	{
		if (AdjustIntervalDelay)
			AdjustIntervalDelay --;
		else
			AlignObsTime(ReceiverInfo);
	}
	// update receiver time to current epoch
	UpdateReceiverTime(MeasParam->TickCount, MeasParam->Interval + MeasParam->ClockAdjust);
#endif
	// calculate raw measurement
	MsrProc(MeasParam);
	if (OutputBasebandMeas)
	{
		MeasParam->TimeQuality = GnssTime.TimeQuality;
		MeasParam->GpsMsCount = GnssTime.GpsMsCount;
		MeasParam->BdsMsCount = GnssTime.BdsMsCount;
		AddToTask(TASK_INOUT, MeasPrintTask, Param, sizeof(BB_MEAS_PARAM));
	}
	// do PVT
	PvtProc(MeasParam->Interval, MeasParam->ClockAdjust);

	return 0;
}

//*************** Frame sync and raw measurement calculation ****************
// Parameters:
//   Measurements: baseband measurement array arranged by channel
//   ActiveMask: channel active (baseband measurement valid) indicator
//   CurMsInterval: actual time interval between current epoch and previous epoch
//   DefaultMsInterval: nominal time interval between two epochs
// Return value:
//   none
void MsrProc(PBB_MEAS_PARAM MeasParam)
{
	int ch_num, svid, FreqID, SatID, meas_num;
	PBB_MEASUREMENT Measurements = BasebandMeasurement;
	SYSTEM_TIME ReceiverTime;
	unsigned int ActiveMask = MeasParam->MeasMask;

	// loop to extrace BB measurements
	for (ch_num = 0; ch_num < TOTAL_CHANNEL_NUMBER; ch_num ++)
	{
		// if corresponding channel is not activated
		if ((ActiveMask & (1 << ch_num)) == 0)
		{
			g_ChannelStatus[ch_num].ChannelErrorFlag = 0;
			g_ChannelStatus[ch_num].svid = 0;
			g_ChannelStatus[ch_num].ChannelFlag &= ~CHANNEL_ACTIVE;
			continue;
		}

		FreqID = Measurements[ch_num].ChannelState->FreqID;
		svid = Measurements[ch_num].ChannelState->Svid;
		SatID = GET_SAT_ID(FreqID, svid);
		// if SatID changed, initialize channel
		if (g_ChannelStatus[ch_num].SatID != SatID)
		{
			InitChannelStatus(ch_num);
			g_ChannelStatus[ch_num].FreqID = FreqID;
			g_ChannelStatus[ch_num].svid = svid;
			g_ChannelStatus[ch_num].SatID = SatID;
		}
		g_ChannelStatus[ch_num].Channel = ch_num;
		g_ChannelStatus[ch_num].cn0 = (unsigned short)Measurements[ch_num].ChannelState->CN0;
		g_ChannelStatus[ch_num].LockTime = Measurements[ch_num].ChannelState->TrackingTime;
		g_ChannelStatus[ch_num].state = Measurements[ch_num].ChannelState->State;
		g_ChannelStatus[ch_num].ChannelFlag |= CHANNEL_ACTIVE;
	}

	// loop to do measurement calculation if receiver time determined
	meas_num = 0;
	if (GnssTime.TimeQuality >= ExtSetTime)
//	if (ReceiverWeekMsValid())
	{
		for (ch_num = 0; ch_num < TOTAL_CHANNEL_NUMBER; ch_num ++)
		{
			// if corresponding channel is not activated
			if ((ActiveMask & (1 << ch_num)) != 0)
				CalculateRawMsr(&g_ChannelStatus[ch_num], &Measurements[ch_num], MeasParam->Interval, MeasParam->ClockAdjust);
			if (g_ChannelStatus[ch_num].ChannelFlag & MEASUREMENT_VALID)
				meas_num ++;
		}
	}
	if (1 && meas_num > 0)
	{
		GpsTimeToUtc(GnssTime.GpsWeekNumber, GnssTime.GpsMsCount, &ReceiverTime, (PUTC_PARAM)0);
		DEBUG_OUTPUT(OUTPUT_CONTROL(MEASUREMENT, INFO), "> %04d %02d %02d %02d %02d %02d.%03d0000 0 %d 0.0000000\n",
			ReceiverTime.Year, ReceiverTime.Month, ReceiverTime.Day, ReceiverTime.Hour, ReceiverTime.Minute, ReceiverTime.Second, ReceiverTime.Millisecond, meas_num);
		for (ch_num = 0; ch_num < TOTAL_CHANNEL_NUMBER; ch_num ++)
		{
			if (g_ChannelStatus[ch_num].ChannelFlag & MEASUREMENT_VALID)
				DEBUG_OUTPUT(OUTPUT_CONTROL(MEASUREMENT, INFO), "%c%02d %13.3f 8 %13.3f 8 %13.3f          %6.3f\n", FREQ_ID_IS_B1C(g_ChannelStatus[ch_num].FreqID) ? 'C' : FREQ_ID_IS_E1(g_ChannelStatus[ch_num].FreqID) ? 'E' : 'G',
					g_ChannelStatus[ch_num].svid, g_ChannelStatus[ch_num].PseudoRangeOrigin, g_ChannelStatus[ch_num].CarrierPhase, g_ChannelStatus[ch_num].DopplerHz, g_ChannelStatus[ch_num].cn0 / 100.);
		}
	}

	// TODO: check for cross correlation by comparing data message
}

//*************** Calculate raw measurement from baseband measurement ****************
// Parameters:
//   pChannelStatus: pointer to channel status holding raw measurement
//   pMsr: pointer to baseband measurement structure
//   CurMsInterval: actual time interval between current epoch and previous epoch
//   DefaultMsInterval: nominal time interval between two epochs
// Return value:
//   none
void CalculateRawMsr(PCHANNEL_STATUS pChannelStatus, PBB_MEASUREMENT pMsr, int MsInterval, int TimeAdjust)
{
	int Count;
	int IFFreq = IF_FREQ, sv_index = pChannelStatus->svid - 1;
	double WaveLength = GPS_L1_WAVELENGTH;
	PFRAME_INFO pFrameInfo = &(pChannelStatus->FrameInfo);

	// clear all valid flags related to raw measurement
	pChannelStatus->ChannelFlag &= ~MEASUREMENT_FLAGS;

	// do not calculate raw measurement if satellite is not in tracking state or CN0 too low
	if (((pChannelStatus->state & STAGE_MASK) < STAGE_TRACK) || (pChannelStatus->cn0 <= 500))
		return;
	// do not calculate raw measurement if there is no valid week ms count
	if (pMsr->WeekMsCount < 0)
		return;

	// integer part of code count and fractional part of code NCO
	pChannelStatus->TransmitTime = (double)pMsr->CodeCount + ScaleDoubleU(pMsr->CodePhase, 32);
	// divide correlator interval to get fractional part of transmit time in unit of millisecond
	pChannelStatus->TransmitTime /= 2046.;
	pChannelStatus->TransmitTimeMs = pMsr->WeekMsCount;

	// Doppler is actual carrier frequency minus nominal number
	if (pChannelStatus->FreqID != FREQ_L1CA && !(pChannelStatus->state & STATE_ENABLE_BOC))
		IFFreq += 1023000;
	pChannelStatus->DopplerHz = (double)pMsr->CarrierFreq * ScaleDoubleU(SAMPLE_FREQ, 32) - IFFreq;
	pChannelStatus->Doppler = pChannelStatus->DopplerHz * WaveLength;

	// pseudorange = (Tr-Tt) * LIGHT_SPEED
	Count = FREQ_ID_IS_B1C(pChannelStatus->FreqID) ? GnssTime.BdsMsCount : GnssTime.GpsMsCount;
	Count -= pChannelStatus->TransmitTimeMs;
	if (Count < -1000)
		Count += MS_IN_WEEK;
	if (Count > 1000 || Count < -1000)	// pseudorange too large
		return;

	pChannelStatus->PseudoRangeOrigin = (double)Count;
	pChannelStatus->PseudoRangeOrigin -= pChannelStatus->TransmitTime;
	pChannelStatus->PseudoRangeOrigin *= LIGHT_SPEED_MS;

	// Calculate carrier phase:
	// Step 1: calculate integer part of cycles increased
	Count = (int)(pMsr->CarrierCount - pChannelStatus->CarrierCountOld);
	// Step 2: remove nominal number and do accumulation
	if (pChannelStatus->CarrierCountAcc <= 0 || pChannelStatus->LockTime == 0)	// initialize and reinitialize
		pChannelStatus->CarrierCountAcc = (int)(pChannelStatus->PseudoRangeOrigin / WaveLength);	// initialize to pseudorange
	else
	{
		pChannelStatus->CarrierCountAcc -= (Count - IFFreq * MsInterval / 1000);
		if (TimeAdjust)	// time adjust in unit of ms
			pChannelStatus->CarrierCountAcc -= 1575420 * TimeAdjust;	// 1ms corresponds to 1575420 cycles
	}
	// Step 3: add the fractional part of carrier phase
	pChannelStatus->CarrierPhase = (double)pChannelStatus->CarrierCountAcc - ScaleDoubleU(pMsr->CarrierPhase, 32);
	// Step 4: determine whether to add 0.5 cycle to compensate negative data stream
	if (pChannelStatus->FreqID == FREQ_L1CA)	// only L1C/A has half cycle
	{
		if (pFrameInfo->FrameFlag & POLARITY_VALID)
		{
			if ((pFrameInfo->FrameFlag & NEGATIVE_STREAM))
				pChannelStatus->CarrierPhase += 0.5;
		}
		else
			pChannelStatus->ChannelFlag |= HALF_CYCLE;
	}
	pChannelStatus->ChannelFlag |= ADR_VALID;
	pChannelStatus->CarrierCountOld = pMsr->CarrierCount;

	pChannelStatus->ChannelFlag |= MEASUREMENT_VALID;
}

int AlignObsTime(PRECEIVER_INFO ReceiverInfo)
{
	double ClockError;
	int ObsInterval;
	int ObsTimeAlign = 0;
	int LocalTimeAdjust = 0;
	int IntervalAdjustment[2];
	int MsCount = ReceiverInfo->ReceiverTime->GpsMsCount;

	if (MsCount < 0)	// time not valid, do not adjust obs time
		return 0;
	ClockError = GetClockError(FREQ_L1CA) * 1000;	// convert to ms
	if (fabs(ClockError) > 1.1)	// adjust clock error first, do not adjust measurement interval
	{
		LocalTimeAdjust = (int)(fabs(ClockError / 2) + 0.5); // LocalTimeAdjust = +-2n for +-2n-1<ClockError<+-2n+1
		LocalTimeAdjust = (ClockError < 0) ? (-LocalTimeAdjust * 2) : (LocalTimeAdjust * 2);
	}
	ObsInterval = NominalMeasInterval;
	ObsTimeAlign = (MsCount + ObsInterval + LocalTimeAdjust) % ObsInterval;
	if (ObsTimeAlign != 0 || LocalTimeAdjust != 0)	// observation time not aligned to nominal interval edge
	{
		// next interval range from 0.5*ObsInterval~1.5*ObsInterval
		if (ObsTimeAlign < ObsInterval / 2)
			ObsTimeAlign = ObsInterval - ObsTimeAlign;
		else
			ObsTimeAlign = ObsInterval * 2 - ObsTimeAlign;
		IntervalAdjustment[0] = ObsTimeAlign;
		IntervalAdjustment[1] = LocalTimeAdjust;
		AdjustIntervalDelay = 3;	// prevent this function called at next MeasInt (at least two interval period delay for Meas ISR to complete adjust)
		AddToTask(TASK_REQUEST, AdjustMeasInterval, IntervalAdjustment, sizeof(int) * 2);
	}

	return LocalTimeAdjust;
}
