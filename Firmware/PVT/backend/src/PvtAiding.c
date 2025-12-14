//----------------------------------------------------------------------
// PvtAiding.c:
//   PVT aiding functions (satellite prediction etc.)
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "CommonDefines.h"
#include "PvtConst.h"
#include "DataTypes.h"
#include "GlobalVar.h"
#include "SupportPackage.h"

#include <math.h>

static PSAT_PREDICT_PARAM CalcPredictParam(int PvtValid, int System, int svid);

//*************** Initialize satellite predict parameter array ****************
// This function is called at startup, if receiver time is availabe
// it is used as signal transmit time, so Doppler is valid but CodePhase is not valid
// predict quality is FINE/CORSE/UNKNOWN
// Parameters:
//   none
// Return value:
//   none
void InitPredictParam()
{
	int i;

	for (i = 0; i < TOTAL_GPS_SAT_NUMBER; i ++)
		CalcPredictParam(0, SYSTEM_GPS, i + i);
	for (i = 0; i < TOTAL_BDS_SAT_NUMBER; i ++)
		CalcPredictParam(0, SYSTEM_BDS, i + i);
	for (i = 0; i < TOTAL_GAL_SAT_NUMBER; i ++)
		CalcPredictParam(0, SYSTEM_GAL, i + i);
}

//*************** Calculate satellite predict parameters based on time/pos/ephemeris/almanac ****************
// Parameters:
//   PvtValid: whether PVT result valid (at least has coarse position and time)
//   System: SYSTEM_GPS/SYSTEM_BDS/SYSTEM_GAL
//   svid: SVID of target satellite (caller to gurantee svid in correct range)
// Return value:
//   pointer to SAT_PREDICT_PARAM of associated to input parameter
PSAT_PREDICT_PARAM CalcPredictParam(int PvtValid, int System, int svid)
{
	PSAT_PREDICT_PARAM SatParam = GET_SYSTEM_ARRAY(System, g_GpsSatParam, g_BdsSatParam, g_GalileoSatParam) + (svid - 1);
	PGNSS_EPHEMERIS Ephemeris = GET_SYSTEM_ARRAY(System, g_GpsEphemeris, g_BdsEphemeris, g_GalileoEphemeris) + (svid - 1);
	PMIDI_ALMANAC Almanac = GET_SYSTEM_ARRAY(System, g_GpsAlmanac, g_BdsAlmanac, g_GalileoAlmanac) + (svid - 1);
	PSATELLITE_INFO SatInfo = GET_SYSTEM_ARRAY(System, g_GpsSatelliteInfo, g_BdsSatelliteInfo, g_GalileoSatelliteInfo) + (svid - 1);
	int ReceiverTime = (System == SYSTEM_BDS) ? g_ReceiverInfo.ReceiverTime->BdsMsCount : g_ReceiverInfo.ReceiverTime->GpsMsCount;
	int WeekNumber = GET_SYSTEM_ARRAY(System, g_ReceiverInfo.ReceiverTime->GpsWeekNumber, g_ReceiverInfo.ReceiverTime->BdsWeekNumber, g_ReceiverInfo.ReceiverTime->GpsWeekNumber - 1024);
	double Distance, TravelTime, TransmitTime = ReceiverTime / 1000.0;
	double ElevationMask = g_PvtConfig.ElevationMask * PI / 180;

	SatParam->Flag &= ~PREDICT_FLAG_MASK;
	if (PvtValid && Ephemeris->flag)
	{
		if (SatInfo->Time != ReceiverTime || !(SatInfo->SatInfoFlag & SAT_INFO_POSVEL_VALID))	// not calculated in most recent PVT
		{
			SatPosSpeedEph(TransmitTime, Ephemeris, &(SatInfo->PosVel));	// calculate satellite position at receiver time
			TravelTime = GeometryDistance(&(g_ReceiverInfo.PosVel), &(SatInfo->PosVel)) / LIGHT_SPEED;
			SatInfo->PosVel.x -= TravelTime * SatInfo->PosVel.vx; SatInfo->PosVel.y -= TravelTime * SatInfo->PosVel.vy; SatInfo->PosVel.z -= TravelTime * SatInfo->PosVel.vz;	// update satellite position by minus travel time
		}
		Distance = GeometryDistance(&(g_ReceiverInfo.PosVel), &(SatInfo->PosVel));
		TravelTime = Distance / LIGHT_SPEED - ClockCorrection(Ephemeris, TransmitTime);	// recalculate travel time with clock correction
		TravelTime -= WGS_F_GTR * Ephemeris->ecc * Ephemeris->sqrtA * sin(Ephemeris->Ek);		// relativity correction
		// ionosphere and troposphere correction ignored because it is relative small for prediction
		SatParam->Flag |= PREDICT_FLAG_ACCURATE;
	}
	else if (g_ReceiverInfo.ReceiverTime->TimeQuality == UnknownTime || g_ReceiverInfo.PosQuality == UnknownPos)
	{
		SatParam->Flag = PREDICT_FLAG_UNKNOWN;
		return SatParam;
	}
	else if (Ephemeris->flag && SatPosSpeedEph(TransmitTime, Ephemeris, &(SatInfo->PosVel)))
		SatParam->Flag |= PREDICT_FLAG_FINE;
	else if (Almanac->flag)
	{
		SatPosSpeedAlm(WeekNumber, (int)TransmitTime, Almanac, &(SatInfo->PosVel));
		SatParam->Flag |= PREDICT_FLAG_COARSE;
	}

	if ((SatParam->Flag & PREDICT_FLAG_MASK) != PREDICT_FLAG_UNKNOWN)
	{
		if ((SatParam->Flag & PREDICT_FLAG_MASK) == PREDICT_FLAG_ACCURATE)
		{
			TransmitTime -= TravelTime;
			if (TransmitTime < 0)
				TransmitTime += 604800000.;
			SatParam->WeekMsCounter = (int)TransmitTime;
			SatParam->CodePhase = (int)((TransmitTime - SatParam->WeekMsCounter) * PREDICT_CODE_UNIT);
		}
		else
			SatParam->CodePhase = 0;
		SatParam->Doppler = -(S16)(SatRelativeSpeed(&(g_ReceiverInfo.PosVel), &(SatInfo->PosVel)) / GPS_L1_WAVELENGTH);
		SatParam->TickCount = g_ReceiverInfo.ReceiverTime->TickCount;
		SatElAz(&(g_ReceiverInfo.PosVel), SatInfo);
		if (SatInfo->el > ElevationMask)
			SatParam->Flag |= PREDICT_STATE_VISIBAL;
		else
			SatParam->Flag &= ~PREDICT_STATE_VISIBAL;
	}

	return SatParam;
}

//*************** Get satellite in view with maximum 32 satellites ****************
// Parameters:
//   SatList: pointer array of predict parameters for valid satellites
//   SignalSvid: array of signal/svid combination for valid satellites
// Return value:
//   number of satellites in view
int GetSatelliteInView(PSAT_PREDICT_PARAM SatList[], U8 SignalSvid[])
{
	int i, sat_num = 0;
	PSAT_PREDICT_PARAM SatParam;

	if (g_ReceiverInfo.PosQuality == UnknownPos)
		return 0;
	if (g_ReceiverInfo.ReceiverTime->TimeQuality == UnknownTime)
		return 0;
	for (i = 1; i <= TOTAL_GPS_SAT_NUMBER && sat_num < 32; i ++)
	{
		SatParam = CalcPredictParam(0, SYSTEM_GPS, i);
		if (SatParam->Flag & PREDICT_STATE_VISIBAL)
		{
			SignalSvid[sat_num] = SIGNAL_SVID(SIGNAL_L1CA, i);
			SatList[sat_num++] = SatParam;
		}
	}
	for (i = 1; i <= TOTAL_GAL_SAT_NUMBER && sat_num < 32; i ++)
	{
		SatParam = CalcPredictParam(0, SYSTEM_GAL, i);
		if (SatParam->Flag & PREDICT_STATE_VISIBAL)
		{
			SignalSvid[sat_num] = SIGNAL_SVID(SIGNAL_E1, i);
			SatList[sat_num++] = SatParam;
		}
	}
	for (i = 1; i <= TOTAL_BDS_SAT_NUMBER && sat_num < 32; i ++)
	{
		SatParam = CalcPredictParam(0, SYSTEM_BDS, i);
		if (SatParam->Flag & PREDICT_STATE_VISIBAL)
		{
			SignalSvid[sat_num] = SIGNAL_SVID(SIGNAL_B1C, i);
			SatList[sat_num++] = SatParam;
		}
	}

	return sat_num;
}
#if 0
int PredictSatelliteParam(double Time, PGNSS_EPHEMERIS Ephemeris, PKINEMATIC_INFO ReceiverPos, PSAT_PREDICT_PARAM SatParam)
{
	SATELLITE_INFO SatInfo;

	Time -= ClockCorrection(Ephemeris, Time);
	SatPosSpeedEph(Time, Ephemeris, &(SatInfo.PosVel));
	// apply relativistic correction to clock
//	Trel = WGS_F_GTR * Ephemeris[sv_index].ecc * Ephemeris[sv_index].sqrtA * sin(Ephemeris[sv_index].Ek);
//	DeltaT += Trel;
	SatParam->PredictPsr = GeometryDistance(ReceiverPos, &(SatInfo.PosVel));
	SatParam->Doppler = -SatRelativeSpeed(ReceiverPos, &(SatInfo.PosVel)) / GPS_L1_WAVELENGTH;
	SatElAz(ReceiverPos, &SatInfo);
	if (SatInfo.el > DEG2RAD(5))
		return 1;

	return 0;
}
#endif
