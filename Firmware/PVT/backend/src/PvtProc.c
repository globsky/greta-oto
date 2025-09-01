//----------------------------------------------------------------------
// PvtProc.c:
//   PVT process functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "CommonDefines.h"
#include "PlatformCtrl.h"
#include "TimeManager.h"
#include "PvtConst.h"
#include "DataTypes.h"
#include "GlobalVar.h"
#include "SupportPackage.h"
#include "SatManage.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

static int PvtFix(int MsInterval, int ClockAdjust);
static PositionType GetPosMethod(PCHANNEL_STATUS ObservationList[], int *Count, PositionType PrevPosType);

// in order to adapt to multiple system, state placement in core data is as following:
// 3 dimention velocity and clock drifting (mostly comes from XTAL so same to all systems) followed by
// 3 dimention position and clock error of GPS, BDS and Galileo
#define STATE_VX (g_PvtCoreData.StateVector[0])
#define STATE_VY (g_PvtCoreData.StateVector[1])
#define STATE_VZ (g_PvtCoreData.StateVector[2])
#define STATE_TDOT (g_PvtCoreData.StateVector[3])
#define STATE_X (g_PvtCoreData.StateVector[4])
#define STATE_Y (g_PvtCoreData.StateVector[5])
#define STATE_Z (g_PvtCoreData.StateVector[6])
#define STATE_DT_GPS (g_PvtCoreData.StateVector[7])
#define STATE_DT_BDS (g_PvtCoreData.StateVector[8])
#define STATE_DT_GAL (g_PvtCoreData.StateVector[9])

//*************** PVT process initialization ****************
//* This task is called once at startup to do initialization
// Parameters:
//   pReceiverInfo: pointer to receiver infomation to give initial position
// Return value:
//   none
void PvtProcInit(PRECEIVER_INFO pReceiverInfo)
{
	// clear Ionosphere and UTC parameters
	memset(&g_GpsIonoParam, 0, sizeof(GPS_IONO_PARAM));
	memset(&g_BdsIonoParam, 0, sizeof(BDS_IONO_PARAM));
	memset(&g_GpsUtcParam, 0, sizeof(UTC_PARAM));
	memset(&g_BdsUtcParam, 0, sizeof(UTC_PARAM));
	// clear satellite info structure
	memset(g_GpsSatelliteInfo, 0, sizeof(g_GpsSatelliteInfo));
	memset(g_GalileoSatelliteInfo, 0, sizeof(g_GalileoSatelliteInfo));
	memset(g_BdsSatelliteInfo, 0, sizeof(g_BdsSatelliteInfo));
	// clear PVT receiver info and core data structure
	memset(&g_ReceiverInfo, 0, sizeof(RECEIVER_INFO));
	memset(&g_PvtCoreData, 0, sizeof(g_PvtCoreData));

	g_ReceiverInfo.ReceiverTime = &GnssTime;
	g_PvtConfig.PvtConfigFlags |= ENABLE_KALMAN_FILTER ? PVT_CONFIG_USE_KF : 0;

	if (!pReceiverInfo)	// if initialize structure is NULL
		return;
	else
	{
		memcpy(&g_ReceiverInfo, pReceiverInfo, sizeof(RECEIVER_INFO));
		if (g_ReceiverInfo.PosQuality > ExtSetPos)
			g_ReceiverInfo.PosQuality = ExtSetPos;
	}

	// initialize state with input parameter
	g_PvtCoreData.StateVector[0] = g_ReceiverInfo.PosVel.vx;
	g_PvtCoreData.StateVector[1] = g_ReceiverInfo.PosVel.vy;
	g_PvtCoreData.StateVector[2] = g_ReceiverInfo.PosVel.vz;
	g_PvtCoreData.StateVector[4] = g_ReceiverInfo.PosVel.x;
	g_PvtCoreData.StateVector[5] = g_ReceiverInfo.PosVel.y;
	g_PvtCoreData.StateVector[6] = g_ReceiverInfo.PosVel.z;

	if (g_ReceiverInfo.PosQuality > UnknownPos)
	{
		memcpy(&(g_ReceiverInfo.PosVel), &(pReceiverInfo->PosVel), sizeof(KINEMATIC_INFO));
		LlhToEcef(&g_ReceiverInfo.PosLLH, &g_ReceiverInfo.PosVel);
		CalcConvMatrix(&g_ReceiverInfo.PosVel, &g_ReceiverInfo.ConvertMatrix);
		g_ReceiverInfo.PosQuality = ExtSetPos;
	}
}

//*************** Do position fix and update corresponding variables ****************
// Parameters:
//   CurMsInterval: actual time interval between current epoch and previous epoch
//   ClockAdjust: clock adjustment (in millisecond) to local receiver time
// Return value:
//   none
void PvtProc(int CurMsInterval, int ClockAdjust)
{
	int PosFixResult;
	SYSTEM_TIME UtcTime;

	PosFixResult = PvtFix(CurMsInterval, ClockAdjust);
	// TODO: update satellite in view list, adjust observation time etc.
	if (1 && PosFixResult >= 0)
	{
		GpsTimeToUtc(g_ReceiverInfo.ReceiverTime->GpsWeekNumber, g_ReceiverInfo.ReceiverTime->GpsMsCount, &UtcTime, (PUTC_PARAM)0);
		DEBUG_OUTPUT(OUTPUT_CONTROL(PVT, INFO), "%04d/%02d/%02d %02d:%02d:%02d.%03d %14.9f %14.9f %10.4f   5   9\n",
			UtcTime.Year, UtcTime.Month, UtcTime.Day, UtcTime.Hour, UtcTime.Minute, UtcTime.Second, UtcTime.Millisecond,
			g_ReceiverInfo.PosLLH.lat * 180 / PI, g_ReceiverInfo.PosLLH.lon * 180 / PI, g_ReceiverInfo.PosLLH.hae);
	}
}

//*************** Return pointer to receiver information ****************
// Parameters:
//   none
// Return value:
//   pointer to receiver information
PRECEIVER_INFO GetReceiverInfo()
{
	return &g_ReceiverInfo;
}

//*************** get clock error ****************
// return the clock error of corresponding system
// if not available, other system will be used at priority of GPS > BDS > Galileo
// Parameters:
//   FirstPriorityFreq defined as FREQ_XXX
// Return value:
//   clock error of corresponding system
//   or 0.0 if clock error of either system is not available
double GetClockError(int FirstPriorityFreq)
{
	PRECEIVER_TIME GnssTime = g_ReceiverInfo.ReceiverTime;

	// check first priority system
	if ((FREQ_ID_IS_L1CA(FirstPriorityFreq) || FREQ_ID_IS_L1C(FirstPriorityFreq)) && (GnssTime->TimeFlag & GPS_CLK_ERR_VALID))
		return GnssTime->GpsClkError;
	else if (FREQ_ID_IS_B1C(FirstPriorityFreq) && (GnssTime->TimeFlag & BDS_CLK_ERR_VALID))
		return GnssTime->BdsClkError;
	else if (FREQ_ID_IS_E1(FirstPriorityFreq) && (GnssTime->TimeFlag & GAL_CLK_ERR_VALID))
		return GnssTime->GalClkError;
	else if (GnssTime->TimeFlag & GPS_CLK_ERR_VALID)
		return GnssTime->GpsClkError;
	else if (GnssTime->TimeFlag & BDS_CLK_ERR_VALID)
		return GnssTime->BdsClkError;
	else if (GnssTime->TimeFlag & GAL_CLK_ERR_VALID)
		return GnssTime->GalClkError;
	else
		return 0.0;
}

//*************** Position fix core function ****************
// Parameters:
//   MsInterval: actual time interval between current epoch and previous epoch
// Return value:
//   none
int PvtFix(int MsInterval, int ClockAdjust)
{
	int i, PosResult;
	int SatCount = 0;
	PCHANNEL_STATUS ObservationList[DIMENSION_MAX_X];
	double DeltaT;
	const double Q[3] = { 25.0, 25.0, 0.25 };	// Qh and Qv are 5^2, Qf is 0.5^2;
	int PosUseSatCount[PVT_MAX_SYSTEM_ID];

	// use position in g_ReceiverInfo as initial position/velocity
	if (g_ReceiverInfo.PosQuality < PosTypeLSQ)
	{
		STATE_X = g_ReceiverInfo.PosVel.x;
		STATE_Y = g_ReceiverInfo.PosVel.y;
		STATE_Z = g_ReceiverInfo.PosVel.z;
		STATE_VX = g_ReceiverInfo.PosVel.vx;
		STATE_VY = g_ReceiverInfo.PosVel.vy;
		STATE_VZ = g_ReceiverInfo.PosVel.vz;
		STATE_DT_GPS = STATE_DT_BDS = STATE_DT_GAL = 0.0;
	}

	CalcConvMatrix(&g_ReceiverInfo.PosVel, &g_ReceiverInfo.ConvertMatrix);
	g_ReceiverInfo.PosFlag &= ~(PVT_USE_GPS | PVT_USE_BDS | PVT_USE_GAL);

	// first fill measurement list if satellite has valid raw measurement and ephemeris
	// scan 3 times to put the list in order of GPS, BDS and Galileo respectively
	for (i = 0; i < TOTAL_CHANNEL_NUMBER && SatCount < DIMENSION_MAX_X; i ++)
	{
		if ((g_ChannelStatus[i].FreqID != FREQ_L1CA) && (g_ChannelStatus[i].FreqID != FREQ_L1C))
			continue;
		if ((g_ChannelStatus[i].ChannelFlag & MEASUREMENT_VALID) && g_GpsEphemeris[g_ChannelStatus[i].svid-1].flag)
			ObservationList[SatCount++] = &g_ChannelStatus[i];
	}
	for (i = 0; i < TOTAL_CHANNEL_NUMBER && SatCount < DIMENSION_MAX_X; i ++)
	{
		if (g_ChannelStatus[i].FreqID != FREQ_B1C)
			continue;
		if ((g_ChannelStatus[i].ChannelFlag & MEASUREMENT_VALID) && g_BdsEphemeris[g_ChannelStatus[i].svid-1].flag)
			ObservationList[SatCount++] = &g_ChannelStatus[i];
	}
	for (i = 0; i < TOTAL_CHANNEL_NUMBER && SatCount < DIMENSION_MAX_X; i ++)
	{
		if (g_ChannelStatus[i].FreqID != FREQ_E1)
			continue;
		if ((g_ChannelStatus[i].ChannelFlag & MEASUREMENT_VALID) && g_GalileoEphemeris[g_ChannelStatus[i].svid-1].flag)
			ObservationList[SatCount++] = &g_ChannelStatus[i];
	}

	// calculate satellite information (pos. vel. el/az etc.)
	CalcSatelliteInfo(ObservationList, SatCount);

	// filter raw measurements
	SatCount = FilterObservation(ObservationList, SatCount);

	// apply correction (ionosphere/troposphere delay etc.)
	ApplyCorrection(ObservationList, SatCount);

	g_ReceiverInfo.PrevPosType = g_ReceiverInfo.CurrentPosType;
	g_ReceiverInfo.CurrentPosType = GetPosMethod(ObservationList, &SatCount, g_ReceiverInfo.PrevPosType);

	// state prediction using constant velocity model
	DeltaT = MsInterval / 1000.0;
	STATE_X += STATE_VX * DeltaT;
	STATE_Y += STATE_VY * DeltaT;
	STATE_Z += STATE_VZ * DeltaT;
	STATE_DT_GPS += STATE_TDOT * DeltaT;
	STATE_DT_BDS += STATE_TDOT * DeltaT;
	STATE_DT_GAL += STATE_TDOT * DeltaT;
	if (ClockAdjust != 0)	// adjustment to local receiver time
	{
		STATE_DT_GPS -= ClockAdjust * LIGHT_SPEED_MS;
		STATE_DT_BDS -= ClockAdjust * LIGHT_SPEED_MS;
		STATE_DT_GAL -= ClockAdjust * LIGHT_SPEED_MS;
	}

	// doing PVT
	if (g_ReceiverInfo.CurrentPosType == PosTypeNone)	// cannot do PVT
		return -1;
	if (g_ReceiverInfo.CurrentPosType == PosTypeKFPos)	// KF PVT
	{
		KFPrediction(g_PvtCoreData.PMatrix, DeltaT);		// KF prediction APA'
		KFAddQMatrix(g_PvtCoreData.PMatrix, Q, &(g_ReceiverInfo.ConvertMatrix), DeltaT);	// KF prediction APA'+Q
		if ((PosResult = KFPosition(ObservationList, SatCount, PosUseSatCount)) <= 0)	// KF update
		{
			return -1;
		}
		g_ReceiverInfo.PosQuality = AccuratePos;
		if (PosResult & PVT_USE_GPS)
			g_ReceiverInfo.ReceiverTime->TimeFlag |= GPS_CLK_ERR_VALID;
		if (PosResult & PVT_USE_BDS)
			g_ReceiverInfo.ReceiverTime->TimeFlag |= BDS_CLK_ERR_VALID;
		if (PosResult & PVT_USE_GAL)
			g_ReceiverInfo.ReceiverTime->TimeFlag |= GAL_CLK_ERR_VALID;
		g_ReceiverInfo.PosFlag |= PosResult;
	}
	else if (g_ReceiverInfo.CurrentPosType == PosTypeLSQ)	// normal LSQ PVT
	{
		if ((PosResult = PvtLsq(ObservationList, SatCount, (g_ReceiverInfo.PosQuality > ExtSetPos) ? 3 : 7)) <= 0)
		{
			return -1;
		}
		g_ReceiverInfo.PosQuality = AccuratePos;
		if (PosResult & PVT_USE_GPS)
			g_ReceiverInfo.ReceiverTime->TimeFlag |= GPS_CLK_ERR_VALID;
		if (PosResult & PVT_USE_BDS)
			g_ReceiverInfo.ReceiverTime->TimeFlag |= BDS_CLK_ERR_VALID;
		if (PosResult & PVT_USE_GAL)
			g_ReceiverInfo.ReceiverTime->TimeFlag |= GAL_CLK_ERR_VALID;
		g_ReceiverInfo.PosFlag |= PosResult;
	}
	else if (g_ReceiverInfo.CurrentPosType == PosTypeFlexTime)	// unknown transmit time PVT
	{
/*		if ((MsAdjust = PvtFlexibleTime(ObservationList, SatCount)) == 0x80000000)
		{
			return -1;
		}
		g_ReceiverInfo.GpsMsCount += MsAdjust;
		g_ReceiverInfo.PosQuality = FlexTimePos;
		g_ReceiverInfo.GpsTimeQuality = FlexTime;*/
	}

	// for LSQ, determine whether can transfer to KF
	if (g_ReceiverInfo.CurrentPosType == PosTypeLSQ && (g_PvtConfig.PvtConfigFlags & PVT_CONFIG_USE_KF) != 0 && g_ReceiverInfo.PosQuality == AccuratePos)
	{
		g_ReceiverInfo.CurrentPosType = PosTypeToKF;
		InitPMatrix(g_PvtCoreData.PMatrix, g_PvtCoreData.PosInvMatrix, PosResult);
	}

	// copy back result to receiver info structure
	g_ReceiverInfo.PosVel.x = STATE_X;
	g_ReceiverInfo.PosVel.y = STATE_Y;
	g_ReceiverInfo.PosVel.z = STATE_Z;
	g_ReceiverInfo.ReceiverTime->GpsClkError = STATE_DT_GPS / LIGHT_SPEED;
	g_ReceiverInfo.ReceiverTime->BdsClkError = STATE_DT_BDS / LIGHT_SPEED;
	g_ReceiverInfo.ReceiverTime->GalClkError = STATE_DT_GAL / LIGHT_SPEED;
	g_ReceiverInfo.PosVel.vx = STATE_VX;
	g_ReceiverInfo.PosVel.vy = STATE_VY;
	g_ReceiverInfo.PosVel.vz = STATE_VZ;
	g_ReceiverInfo.ReceiverTime->ClkDrifting = STATE_TDOT / LIGHT_SPEED;

	// convert ECEF coordinate to LLH coordinate
	EcefToLlh(&(g_ReceiverInfo.PosVel), &(g_ReceiverInfo.PosLLH));
	return 0;
}

//*************** Determine method to do position fix ****************
//* the determination uses the following order:
//* first to check whether can do Kalman filter
//* second to determine whether can do normal 3D or 2D LSQ
//* last to check whether 5 satellite positioning is possible
// Parameters:
//   ObservationList: pointer array of observations
//   Count: number of observations
//   PrevPosType: position type in previous epoch
// Return value:
//   position type in current epoch
PositionType GetPosMethod(PCHANNEL_STATUS ObservationList[], int *Count, PositionType PrevPosType)
{
	int i, SatCount = *Count, SystemMask = 0, RedundantSat;

	// determine positioning method
	if (SatCount < 1)
		return PosTypeNone;

	// first to check whether can do KF
	if ((g_PvtConfig.PvtConfigFlags & PVT_CONFIG_USE_KF) && PrevPosType >= PosTypeToKF && SatCount > 0)
		return PosTypeKFPos;

	// whether can do normal LSQ or 2D LSQ
	RedundantSat = SatCount;
	for (i = 0; i < SatCount; i ++)
	{
		if ((g_ChannelStatus[i].FreqID == FREQ_L1CA) || (g_ChannelStatus[i].FreqID == FREQ_L1C))
			SystemMask |= PVT_USE_GPS;
		else if (g_ChannelStatus[i].FreqID == FREQ_B1C)
			SystemMask |= PVT_USE_BDS;
		else if (g_ChannelStatus[i].FreqID == FREQ_E1)
			SystemMask |= PVT_USE_GAL;
	}
	if (SystemMask & PVT_USE_GPS) RedundantSat --;
	if (SystemMask & PVT_USE_BDS) RedundantSat --;
	if (SystemMask & PVT_USE_GAL) RedundantSat --;

	if (ObservationList[0]->ChannelFlag & TRANSTIME_ESTIMATE)	// transmit time is estimated, need one extra observation
		return (RedundantSat >= 4) ? PosTypeFlexTime : PosTypeNone;
	else if (RedundantSat >= 2)
		return (RedundantSat == 2) ? PosType2D : PosTypeLSQ;
	else
		return PosTypeNone;
}
