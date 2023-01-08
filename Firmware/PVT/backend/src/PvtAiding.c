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

static int PredictSatelliteParam(double Time, PGNSS_EPHEMERIS Ephemeris, PKINEMATIC_INFO pReceiver, PSAT_PREDICT_PARAM SatParam);

// get satellite in view with maximum 32 satellites
int GetSatelliteInView(SAT_PREDICT_PARAM SatList[32])
{
	int i, sat_num = 0;
	PGNSS_EPHEMERIS Ephemeris;
	PSATELLITE_INFO SatelliteInfo;
	double Time;

	if (g_ReceiverInfo.PosQuality == UnknownPos)
		return 0;
	if ((g_ReceiverInfo.GpsTimeQuality == UnknownTime) && (g_ReceiverInfo.BdsTimeQuality == UnknownTime) && (g_ReceiverInfo.GalileoTimeQuality == UnknownTime))
		return 0;
	Ephemeris = g_GpsEphemeris;
	SatelliteInfo = g_GpsSatelliteInfo;
	Time = g_ReceiverInfo.GpsMsCount * 0.001;
	for (i = 0; i < TOTAL_GPS_SAT_NUMBER; i ++)
	{
		if (sat_num >= 32)
			break;
		if (!(Ephemeris[i].flag & 1))
			continue;
		if (PredictSatelliteParam(Time, &Ephemeris[i], &(g_ReceiverInfo.PosVel), &SatList[sat_num]))
		{
			SatList[sat_num].FreqID = FREQ_L1CA;
			SatList[sat_num].Svid = (U8)(i + 1);
			sat_num ++;
		}
	}
	Ephemeris = g_BdsEphemeris;
	SatelliteInfo = g_BdsSatelliteInfo;
	Time = (g_ReceiverInfo.GpsMsCount - 14000) * 0.001;
	for (i = 0; i < TOTAL_BDS_SAT_NUMBER; i ++)
	{
		if (sat_num >= 32)
			break;
		if (!(Ephemeris[i].flag & 1))
			continue;
		if (PredictSatelliteParam(Time, &Ephemeris[i], &(g_ReceiverInfo.PosVel), &SatList[sat_num]))
		{
			SatList[sat_num].FreqID = FREQ_B1C;
			SatList[sat_num].Svid = (U8)(i + 1);
			sat_num ++;
		}
	}
	Ephemeris = g_GalileoEphemeris;
	SatelliteInfo = g_GalileoSatelliteInfo;
	Time = g_ReceiverInfo.GpsMsCount * 0.001;
	for (i = 0; i < TOTAL_GAL_SAT_NUMBER; i ++)
	{
		if (sat_num >= 32)
			break;
		if (!(Ephemeris[i].flag & 1))
			continue;
		if (PredictSatelliteParam(Time, &Ephemeris[i], &(g_ReceiverInfo.PosVel), &SatList[sat_num]))
		{
			SatList[sat_num].FreqID = FREQ_E1;
			SatList[sat_num].Svid = (U8)(i + 1);
			sat_num ++;
		}
	}

	return sat_num;
}

int PredictSatelliteParam(double Time, PGNSS_EPHEMERIS Ephemeris, PKINEMATIC_INFO ReceiverPos, PSAT_PREDICT_PARAM SatParam)
{
	SATELLITE_INFO SatInfo;

	Time -= ClockCorrection(Ephemeris, Time);
	SatPosSpeedEph(Time, Ephemeris, &(SatInfo.PosVel));
	// apply relativistic correction to clock
//	Trel = WGS_F_GTR * Ephemeris[sv_index].ecc * Ephemeris[sv_index].sqrtA * sin(Ephemeris[sv_index].Ek);
//	DeltaT += Trel;
	SatParam->PredictPsr = GeometryDistance(ReceiverPos, &(SatInfo.PosVel));
	SatParam->Doppler = SatRelativeSpeed(ReceiverPos, &(SatInfo.PosVel)) / GPS_L1_WAVELENGTH;
	SatElAz(ReceiverPos, &SatInfo);
	if (SatInfo.el > DEG2RAD(5))
		return 1;

	return 0;
}
