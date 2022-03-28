//----------------------------------------------------------------------
// MsrProc.c:
//   Raw measurement process functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "CommonDefines.h"
#include "PvtConst.h"
#include "DataTypes.h"
#include "GlobalVar.h"
#include "SupportPackage.h"
#include "GpsFrame.h"
#include "BdsFrame.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

static unsigned int FrameStatusBuffer[sizeof(GPS_FRAME_INFO)*32/4];

static void ProcessReceiverTime(int CurMsInterval, int DefaultMsInterval);
static void CalculateRawMsr(PCHANNEL_STATUS pChannelStatus, PBB_MEASUREMENT pMsr, int CurMsInterval, int DefaultMsInterval);

#define INIT_GPS_FRAME(gps_frame) \
do \
{ \
	(gps_frame)->NavBitNumber = 0; \
	(gps_frame)->FrameStatus = -1; \
	(gps_frame)->tow = -1; \
} while (0)

static void InitFrame(int ch_num)
{
	unsigned char FreqID = g_ChannelStatus[ch_num].FreqID;

	g_ChannelStatus[ch_num].LockTime = 0;
	g_ChannelStatus[ch_num].CarrierCountAcc = 0;
	g_ChannelStatus[ch_num].CarrierCountOld = 0;
	((PGPS_FRAME_INFO)(g_ChannelStatus[ch_num].FrameInfo))->FrameFlag = 0;
	INIT_GPS_FRAME((PGPS_FRAME_INFO)(g_ChannelStatus[ch_num].FrameInfo));
}

static void InitChannel(int ch_num)
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
	int i;
	unsigned int *pBuffer;

	// clear channel status buffer
	memset(g_ChannelStatus, 0, sizeof(g_ChannelStatus));
	// clear frame status buffer
	memset(FrameStatusBuffer, 0, sizeof(FrameStatusBuffer));

	// allocate frame buffer for each channel, different system may use different frame structure
	pBuffer = FrameStatusBuffer;
	for (i = 0; i < TOTAL_CHANNEL_NUMBER; i ++)
	{
		g_ChannelStatus[i].FrameInfo = (void *)pBuffer;
		pBuffer += sizeof(GPS_FRAME_INFO) / 4;
	}
}

//*************** Frame sync and raw measurement calculation ****************
// Parameters:
//   Measurements: baseband measurement array arranged by channel
//   ActiveMask: channel active (baseband measurement valid) indicator
//   CurMsInterval: actual time interval between current epoch and previous epoch
//   DefaultMsInterval: nominal time interval between two epochs
// Return value:
//   none
void MsrProc(PBB_MEASUREMENT Measurements, unsigned int ActiveMask, int CurMsInterval, int DefaultMsInterval)
{
	int ch_num, svid, FreqID, SatID, meas_num;
	SYSTEM_TIME ReceiverTime;

	// loop to extrace BB measurements and do frame sync
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

		FreqID = Measurements[ch_num].FreqID;
		svid = Measurements[ch_num].Svid;
		SatID = GET_SAT_ID(FreqID, svid);
		// if SatID changed, initialize channel
		if (g_ChannelStatus[ch_num].SatID != SatID)
		{
			InitChannel(ch_num);
			g_ChannelStatus[ch_num].FreqID = FreqID;
			g_ChannelStatus[ch_num].svid = svid;
			g_ChannelStatus[ch_num].SatID = SatID;
		}
		g_ChannelStatus[ch_num].Channel = ch_num;
		g_ChannelStatus[ch_num].cn0 = (unsigned short)Measurements[ch_num].CN0;
		g_ChannelStatus[ch_num].LockTime = Measurements[ch_num].TrackingTime;
		g_ChannelStatus[ch_num].state = Measurements[ch_num].State;
		g_ChannelStatus[ch_num].ChannelFlag |= CHANNEL_ACTIVE;

		// if data count less than expected in time interval, there is signal loss, init frame
		if (Measurements[ch_num].DataNumber < CurMsInterval / ((FREQ_ID_IS_L1CA(FreqID) ? 20 : (FREQ_ID_IS_E1(FreqID) ? 4 : 10))) - 1)
			InitFrame(ch_num);

		// if bit sync get, do frame process
		if ((Measurements[ch_num].State & STAGE_MASK) >= STAGE_TRACK && Measurements[ch_num].CN0 > 0)
		{
			if (FreqID == FREQ_L1CA)
				GpsFrameSync(&g_ChannelStatus[ch_num], Measurements[ch_num].DataNumber, Measurements[ch_num].DataStreamAddr[0], Measurements[ch_num].DataStreamAddr[1], -1);
			else if (FreqID == FREQ_B1C)
				BdsFrameProc(&g_ChannelStatus[ch_num]);
		}
		else
		{
			InitFrame(ch_num);
		}
	}

	// TODO: loop to do fast frame sync

	// determine or predict receiver time
	ProcessReceiverTime(CurMsInterval, DefaultMsInterval);

	// loop to do measurement calculation if receiver time determined
	meas_num = 0;
	if (g_ReceiverInfo.GpsTimeQuality >= CoarseTime)
	{
		for (ch_num = 0; ch_num < TOTAL_CHANNEL_NUMBER; ch_num ++)
		{
			// if corresponding channel is not activated
			if ((ActiveMask & (1 << ch_num)) != 0)
				CalculateRawMsr(&g_ChannelStatus[ch_num], &Measurements[ch_num], CurMsInterval, DefaultMsInterval);
			if (g_ChannelStatus[ch_num].ChannelFlag & MEASUREMENT_VALID)
				meas_num ++;
		}
	}
	if (0 && meas_num > 0)
	{
		GpsTimeToUtc(g_ReceiverInfo.WeekNumber, g_ReceiverInfo.GpsMsCount, &ReceiverTime, (PUTC_PARAM)0);
		printf("> %04d %02d %02d %02d %02d %02d.%03d0000 0 %d 0.0000000\n",
			ReceiverTime.Year, ReceiverTime.Month, ReceiverTime.Day, ReceiverTime.Hour, ReceiverTime.Minute, ReceiverTime.Second, ReceiverTime.Millisecond, meas_num);
		for (ch_num = 0; ch_num < TOTAL_CHANNEL_NUMBER; ch_num ++)
		{
			if (g_ChannelStatus[ch_num].ChannelFlag & MEASUREMENT_VALID)
				printf("%c%02d %13.3f 8 %13.3f 8 %13.3f          %6.3f\n", FREQ_ID_IS_B1C(g_ChannelStatus[ch_num].FreqID) ? 'C' : FREQ_ID_IS_E1(g_ChannelStatus[ch_num].FreqID) ? 'E' : 'G',
					g_ChannelStatus[ch_num].svid, g_ChannelStatus[ch_num].PseudoRangeOrigin, g_ChannelStatus[ch_num].CarrierPhase, g_ChannelStatus[ch_num].DopplerHz, g_ChannelStatus[ch_num].cn0 / 100.);
		}
	}

	// TODO: check for cross correlation by comparing data message
}

//*************** Estimate or calculate receiver time ****************
// Parameters:
//   CurMsInterval: actual time interval between current epoch and previous epoch
//   DefaultMsInterval: nominal time interval between two epochs
// Return value:
//   none
void ProcessReceiverTime(int CurMsInterval, int DefaultMsInterval)
{
	int i;
	PGPS_FRAME_INFO pGpsFrameInfo;
	PBDS_FRAME_INFO pBdsFrameInfo;
	double ClkDrifting;
	int WeekMsCount = -1;

	// predict receiver time
	g_ReceiverInfo.GpsMsCount += (g_ReceiverInfo.GpsMsCount >= 0) ? DefaultMsInterval : 0;
	if (g_ReceiverInfo.GpsMsCount >= 604800000 && g_ReceiverInfo.WeekNumber >= 0)
	{
		g_ReceiverInfo.GpsMsCount -= 604800000;
		g_ReceiverInfo.WeekNumber ++;
	}

	// predict clock error, g_ReceiverInfo.ClkDrifting in unit of m/s, g_ReceiverInfo.XXXClkError in unit of second
	ClkDrifting = g_ReceiverInfo.ClkDrifting * CurMsInterval / 1000. / LIGHT_SPEED;
	g_ReceiverInfo.GpsClkError += ClkDrifting;
	g_ReceiverInfo.GalileoClkError += ClkDrifting;
	g_ReceiverInfo.BdsClkError += ClkDrifting;

	// receiver time already set
	if (g_ReceiverInfo.GpsTimeQuality >= CoarseTime)
		return;

	for (i = 0; i < TOTAL_CHANNEL_NUMBER; i ++)
	{
		if (!(g_ChannelStatus[i].ChannelFlag & CHANNEL_ACTIVE))
			continue;

		switch (g_ChannelStatus[i].FreqID)
		{
		case FREQ_L1CA:
			pGpsFrameInfo = (PGPS_FRAME_INFO)(g_ChannelStatus[i].FrameInfo);
			// for GPS, receiver time approximate to transmit time + 80ms
			// transmit time approximate to start of current subframe (tow*6000) + received bits * 20ms
			// received bit is bit number in buffer (NavBitNumber) - 2 (D29 and D30 in previous subframe)
			// so the equation is tow*6000+(NavBitNumber-2)*20+80=tow*6000+(NavBitNumber+2)*20
			if (pGpsFrameInfo->FrameStatus >= 30 && pGpsFrameInfo->tow >= 0)
			{
				WeekMsCount = pGpsFrameInfo->tow * 6000 + (pGpsFrameInfo->NavBitNumber + 2) * 20;
			}
			break;
		case FREQ_B1C:
			pBdsFrameInfo = (PBDS_FRAME_INFO)(g_ChannelStatus[i].FrameInfo);
			// for BDS, receiver time approximate to transmit time + 80ms (MEO) or 140ms (GEO/IGSO)
			// transmit time approximate to start of current frame (tow*1000) + received bits * 10ms
			if (pBdsFrameInfo->tow >= 0)
			{
				WeekMsCount = pBdsFrameInfo->tow * 1000 + pBdsFrameInfo->NavBitNumber * 10 + 14000;	// 14s leap second
				WeekMsCount += ((pBdsFrameInfo->FrameFlag & 0xc) == 0xc) ? 80 : 140;
			}
			break;
		default:	// TODO: other satellite system
			break;
		}

		// if current time in week get then break (only find the first channel got frame sync)
		if (WeekMsCount >= 0)
			break;
	}

	if (WeekMsCount >= 0)
	{
		// align ms count to be multiple of 100ms
		WeekMsCount = (WeekMsCount + 50) / 100 * 100;
		g_ReceiverInfo.GpsMsCount = WeekMsCount;
		g_ReceiverInfo.GpsTimeQuality = CoarseTime;
	}
}

//*************** Calculate raw measurement from baseband measurement ****************
// Parameters:
//   pChannelStatus: pointer to channel status holding raw measurement
//   pMsr: pointer to baseband measurement structure
//   CurMsInterval: actual time interval between current epoch and previous epoch
//   DefaultMsInterval: nominal time interval between two epochs
// Return value:
//   none
void CalculateRawMsr(PCHANNEL_STATUS pChannelStatus, PBB_MEASUREMENT pMsr, int CurMsInterval, int DefaultMsInterval)
{
	int Count;
	int IFFreq = IF_FREQ, sv_index = pChannelStatus->svid - 1;
	double WaveLength = GPS_L1_WAVELENGTH, PsrDiff;
	PGPS_FRAME_INFO pGpsFrameInfo = (PGPS_FRAME_INFO)(pChannelStatus->FrameInfo);
	PBDS_FRAME_INFO pBdsFrameInfo = (PBDS_FRAME_INFO)(pChannelStatus->FrameInfo);

	// clear all valid flags related to raw measurement
	pChannelStatus->ChannelFlag &= ~MEASUREMENT_FLAGS;

	// do not calculate raw measurement if satellite is not in tracking state or CN0 too low
	if (((pChannelStatus->state & STAGE_MASK) < STAGE_TRACK) || (pChannelStatus->cn0 <= 500))
		return;

	// integer part of code count and fractional part of code NCO
	pChannelStatus->TransmitTime = (double)pMsr->CodeCount + ScaleDoubleU(pMsr->CodeNCO, 32);
	// divide correlator interval to get fractional part of transmit time in unit of millisecond
	pChannelStatus->TransmitTime /= 2046.;

	// determine integer part of transmit time
	switch (pChannelStatus->FreqID)
	{
	case FREQ_L1CA:
		if (pGpsFrameInfo->FrameStatus >= 30 && pGpsFrameInfo->tow >= 0)	// transmit time get from frame sync
		{
			// transmit time is current tow*6000ms plus bit_count*20ms
			pChannelStatus->TransmitTimeMs = pGpsFrameInfo->tow * 6000 + (pGpsFrameInfo->NavBitNumber - 2) * 20;
		}
		else if (g_ReceiverInfo.PosQuality >= KalmanPos && pChannelStatus->LockTime > 0 && (g_GpsSatelliteInfo[sv_index].SatInfoFlag & SAT_INFO_PSR_VALID))	// recover transmit time from valid receiver position
		{
			PsrDiff = g_GpsSatelliteInfo[sv_index].PsrPredict / LIGHT_SPEED_MS;
			PsrDiff += pChannelStatus->TransmitTime;
			pChannelStatus->TransmitTimeMs = ((g_ReceiverInfo.GpsMsCount - (int)PsrDiff + 10) / 20) * 20;
		}
		else
			return;
		break;
	case FREQ_B1C:
		if (pBdsFrameInfo->tow >= 0)	// transmit time get
		{
			// transmit time is current tow*1000ms plus bit_count*10ms
			pChannelStatus->TransmitTimeMs = pBdsFrameInfo->tow * 1000 + pBdsFrameInfo->NavBitNumber * 10;
		}
		else
			return;
		break;
	default:	// TODO: other satellite system
		return;
	}

	// Doppler is actual carrier frequency minus nominal number
	if (pChannelStatus->FreqID != FREQ_L1CA && !(pChannelStatus->state & STATE_ENABLE_BOC))
		IFFreq += 1023000;
	pChannelStatus->DopplerHz = (double)pMsr->CarrierFreq * ScaleDoubleU(SAMPLE_FREQ, 32) - IFFreq;
	pChannelStatus->Doppler = pChannelStatus->DopplerHz * WaveLength;

	// pseudorange = (Tr-Tt) * LIGHT_SPEED
	Count = g_ReceiverInfo.GpsMsCount;
	if (pChannelStatus->FreqID == FREQ_B1C)
		Count -= 14000;
	Count -= pChannelStatus->TransmitTimeMs;
	if (Count < -1000)
		Count += 604800000;
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
		pChannelStatus->CarrierCountAcc -= (Count - IFFreq * CurMsInterval / 1000);
	// Step 3: add the fractional part of carrier phase
	pChannelStatus->CarrierPhase = (double)pChannelStatus->CarrierCountAcc - ScaleDoubleU(pMsr->CarrierNCO, 32);
	// Step 4: determine whether to add 0.5 cycle to compensate negative data stream
	if (pChannelStatus->FreqID == FREQ_L1CA)	// only L1C/A has half cycle
	{
		if (pGpsFrameInfo->FrameFlag & POLARITY_VALID)
		{
			if ((pGpsFrameInfo->FrameFlag & NEGATIVE_STREAM))
				pChannelStatus->CarrierPhase += 0.5;
		}
		else
			pChannelStatus->ChannelFlag |= HALF_CYCLE;
	}
	pChannelStatus->ChannelFlag |= ADR_VALID;
	pChannelStatus->CarrierCountOld = pMsr->CarrierCount;

	pChannelStatus->ChannelFlag |= MEASUREMENT_VALID;
}
