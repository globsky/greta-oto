//----------------------------------------------------------------------
// SatManage.c:
//   satellite management related functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "CommonDefines.h"
#include "DataTypes.h"
#include "TimeManager.h"
#include "GlobalVar.h"
#include "SupportPackage.h"
#include <string.h>
#include <math.h>

static double GpsIonoDelay(PGPS_IONO_PARAM pIonoParam, LLH *ReceiverPos, int WeekMsCount, PSATELLITE_INFO pSatInfo);
static double TropoDelay(double Elevation, PRECEIVER_INFO pReceiverInfo);
static double GetTropoParam(int ParamIndex, int LatDegree, double SeasonVar);

//*************** Calculate satellite information of given satellite list ****************
// Parameters:
//   ObservationList: raw measurement pointer array
//   ObsCount: number of observations
// Return value:
//   none
void CalcSatelliteInfo(PCHANNEL_STATUS ObservationList[], int ObsCount)
{
	int i;
	int sv_index;
	int EphOK = 1;
	double Time, Trel;
	PGNSS_EPHEMERIS Ephemeris = g_GpsEphemeris;
	PSATELLITE_INFO SatelliteInfo = g_GpsSatelliteInfo;

	// calculate satellite position and velocity
	for (i = 0; i < ObsCount; i ++)
	{
		switch (ObservationList[i]->FreqID)
		{
		case FREQ_L1CA:
		case FREQ_L1C:
			Ephemeris = g_GpsEphemeris;
			SatelliteInfo = g_GpsSatelliteInfo;
			break;
		case FREQ_B1C:
			Ephemeris = g_BdsEphemeris;
			SatelliteInfo = g_BdsSatelliteInfo;
			break;
		case FREQ_E1:
			Ephemeris = g_GalileoEphemeris;
			SatelliteInfo = g_GalileoSatelliteInfo;
			break;
		default:
			// will not go here
			break;
		}

		sv_index = ObservationList[i]->svid - 1;
		// calculate signal transmit time and correct satellite clock error
		Time = (ObservationList[i]->TransmitTimeMs + ObservationList[i]->TransmitTime) * 0.001;
		ObservationList[i]->DeltaT = ClockCorrection(&(Ephemeris[sv_index]), Time);
		Time -= ObservationList[i]->DeltaT;
		// use transmit time to calculate satellite position and velocity
		EphOK = SatPosSpeedEph(Time, &(Ephemeris[sv_index]), &(SatelliteInfo[sv_index].PosVel));
		// apply relativistic correction to clock
		Trel = WGS_F_GTR * Ephemeris[sv_index].ecc * Ephemeris[sv_index].sqrtA * sin(Ephemeris[sv_index].Ek);
		ObservationList[i]->DeltaT += Trel;
		// compensate satellite transmit time calculation with Trel difference (before calling SatPosSpeedEph(), Ek is not calculated)
		// generally this is not necessory because the compensation is very small
//		SatelliteInfo[sv_index].PosVel.x -= Trel * SatelliteInfo[sv_index].PosVel.vx;
//		SatelliteInfo[sv_index].PosVel.y -= Trel * SatelliteInfo[sv_index].PosVel.vy;
//		SatelliteInfo[sv_index].PosVel.z -= Trel * SatelliteInfo[sv_index].PosVel.vz;
		// this is the time tag that used to calculate el/az and set flag
		SatelliteInfo[sv_index].Time = ObservationList[i]->TransmitTimeMs;
		SatelliteInfo[sv_index].SatInfoFlag = SAT_INFO_POSVEL_VALID | SAT_INFO_BY_EPH | (EphOK ? 0 : SAT_INFO_EPH_EXPIRE);
		if (g_ReceiverInfo.PosQuality >= ExtSetPos)
			SatElAz(&(g_ReceiverInfo.PosVel), &(SatelliteInfo[sv_index]));
	}
}

//*************** Filter raw measurements ****************
// Parameters:
//   ObservationList: raw measurement pointer array
//   ObsCount: number of observations
// Return value:
//   none
int FilterObservation(PCHANNEL_STATUS ObservationList[], int ObsCount)
{
	int i, sv_index, PrevFreqID = -1, index = 0;
	unsigned long long GpsInUseMask = g_PvtConfig.GpsSatMaskOut, BdsInUseMask = g_PvtConfig.BdsSatMaskOut, GalileoInUseMask = g_PvtConfig.GalileoSatMaskOut;
	unsigned long long *pInUseMask = &GpsInUseMask;
	PSATELLITE_INFO SatelliteInfo = g_GpsSatelliteInfo;
	double ElevationMask = g_PvtConfig.ElevationMask * PI / 180;

	// first round filtering, remove duplicate satellite, ephemeris expire satellite and do elevation mask
	for (i = 0; i < ObsCount; i ++)
	{
		// observations are arranged to put same system together and with order GPS, BDS, Galileo
		if (ObservationList[i]->FreqID == FREQ_B1C && PrevFreqID != FREQ_B1C)
		{
			PrevFreqID = FREQ_B1C;
			pInUseMask = &BdsInUseMask;
			SatelliteInfo = g_BdsSatelliteInfo;
		}
		else if (ObservationList[i]->FreqID == FREQ_E1 && PrevFreqID != FREQ_E1)
		{
			PrevFreqID = FREQ_E1;
			pInUseMask = &GalileoInUseMask;
			SatelliteInfo = g_GalileoSatelliteInfo;
		}
		sv_index = ObservationList[i]->svid - 1;

		// mask out satellite in mask or already put in list
		if (*pInUseMask & (1LL << sv_index))
			continue;
		
		// sat eph expire
		if (SatelliteInfo[sv_index].SatInfoFlag & SAT_INFO_EPH_EXPIRE)
			continue;

		// elevation too low
		if ((SatelliteInfo[sv_index].SatInfoFlag & SAT_INFO_ELAZ_VALID) && SatelliteInfo[sv_index].el < ElevationMask)
			continue;
		
		// cn0 too small
		if(ObservationList[i]->cn0 < 1000)
			continue;

		ObservationList[index++] = ObservationList[i];
		*pInUseMask |= (1LL << sv_index);
	}

	return index;
}

//*************** Apply correction to raw measurements ****************
// Parameters:
//   ObservationList: raw measurement pointer array
//   ObsCount: number of observations
// Return value:
//   none
void ApplyCorrection(PCHANNEL_STATUS ObservationList[], int ObsCount)
{
	int i, sv_index;
	PSATELLITE_INFO SatelliteInfo = g_GpsSatelliteInfo;

	// calculate Tclk + Trel - Tgd - Ttrop  - Tiono (earth rotate correction applied in GeometryDistanceXYZ())
	// ObservationList[i]->DeltaT has already assigned with clock error and relativistic correction in CalcSatelliteInfo()
	for (i = 0; i < ObsCount; i ++)
	{
		sv_index = ObservationList[i]->svid - 1;

		// group delay
		if (ObservationList[i]->FreqID == FREQ_L1CA || ObservationList[i]->FreqID == FREQ_L1C)
			ObservationList[i]->DeltaT -= g_GpsEphemeris[sv_index].tgd;
		else if (ObservationList[i]->FreqID == FREQ_B1C)
		{
			ObservationList[i]->DeltaT -= g_BdsEphemeris[sv_index].tgd;
			SatelliteInfo = g_BdsSatelliteInfo;
		}
		else if (ObservationList[i]->FreqID == FREQ_E1)
		{
			ObservationList[i]->DeltaT -= g_GalileoEphemeris[sv_index].tgd;
			SatelliteInfo = g_GalileoSatelliteInfo;
		}
		// ionosphere delay
		if (g_ReceiverInfo.PosQuality != UnknownPos && (SatelliteInfo[sv_index].SatInfoFlag & SAT_INFO_ELAZ_VALID)) 	// user position and satellite el/az valid
		{
			// ionosphere correction
			if (g_GpsIonoParam.flag)	// first try GPS ionosphere parameter
				ObservationList[i]->DeltaT -= GpsIonoDelay(&g_GpsIonoParam, &(g_ReceiverInfo.PosLLH), g_ReceiverInfo.ReceiverTime->GpsMsCount, &SatelliteInfo[sv_index]);
//			else if (g_BdsIonoParam.flag)	// then try BD2 ionosphere parameter
//				ObservationList[i]->DeltaT -= BdsIonoDelay(&g_BdsIonoParam, &(g_ReceiverInfo.PosLLH), g_ReceiverInfo.GpsMsCount, &SatelliteInfo[sv_index]);
			// troposphere correction
			ObservationList[i]->DeltaT -= TropoDelay(SatelliteInfo[sv_index].el, &g_ReceiverInfo);
		}
		// calculate corrected PSR
		ObservationList[i]->PseudoRange = ObservationList[i]->PseudoRangeOrigin + ObservationList[i]->DeltaT * LIGHT_SPEED;
		// correct Doppler with satellite clock drifting
		ObservationList[i]->Doppler -= g_GpsEphemeris[sv_index].af1 * LIGHT_SPEED;
	}
}

//*************** Calculate ionosphere delay ****************
// Parameters:
//   pIonoParam: pointer to ionosphere parameter structure
//   ReceiverPos: pointer to receiver position (lat/lon/altitude)
//   WeekMsCount: millisecond count within week
//   pSatInfo: pointer to satellite information structure
// Return value:
//   ionosphere delay in seconds
double GpsIonoDelay(PGPS_IONO_PARAM pIonoParam, LLH *ReceiverPos, int WeekMsCount, PSATELLITE_INFO pSatInfo)
{
	double El = pSatInfo->el / PI;
	double Lat = ReceiverPos->lat / PI;
	double Lon = ReceiverPos->lon / PI;
	double phi, F, PER, x, AMP, x1;
	double ReturnValue = 0.;
	int T;

	if (pIonoParam->flag == 0)
		return 0.;
	phi = 0.0137f / (El + 0.11f) - 0.022f;
	Lat += phi * cos(pSatInfo->az);
	if (Lat > 0.416f)
		Lat = 0.416f;
	else if (Lat < -0.416f)
		Lat = -0.416f;

	Lon += phi * sin(pSatInfo->az) / cos(Lat * PI);
	Lat += 0.064f * cos(Lon - 1.617f) * PI;
	F = 1.0f + 16.0f * CUBE(0.53f - El);
	PER = pIonoParam->b0 + (pIonoParam->b1 + (pIonoParam->b2 + pIonoParam->b3 * Lat) * Lat) * Lat;
	if (PER < 72000.0)
		PER = 72000.0f;

	T = (int)(43200000 * Lon) + WeekMsCount;	// T with unit of millisecond
	while (T >= 86400000)
		T -= 86400000;
	while (T < 0)
		T += 86400000;
	x = 0.002 * PI * (T - 50400000) / PER;

	if (x >= 1.57 || x <= -1.57)
	{
		ReturnValue =  F * 5e-9;
	}
	else
	{
		AMP = pIonoParam->a0 + (pIonoParam->a1 + (pIonoParam->a2 + pIonoParam->a3 * Lat) * Lat) * Lat;
	
		if (AMP < 0.0)
			ReturnValue = F * 5e-9;
		else
		{
			x *= x;
			x1 = 1.0 - x / 2.0;
			x *= x;
			x1 += x / 24.0;
			ReturnValue = F * (5e-9 + AMP * x1);
		}
	}

	return ReturnValue;
}

/*********************************************
* This is a simplified Hopfield's model
* with the standard atmosphere values
* Input elevation with rad
* Output delay in second
**********************************************/
static const double TropoTableAverage[5][5] = {
	{1013.25,	299.65,	26.31,	0.00630,	2.77},
	{1017.25,	294.15,	21.79,	0.00605,	3.15},	
	{1015.75,	283.15,	11.66,	0.00558,	2.57},
	{1011.75,	272.15,	 6.78,	0.00539,	1.81},
	{1013.00,	263.65,	 4.11,	0.00453,	1.55}};

static const double TropoTableAverageDelta[4][5] = {
	{(1017.25 - 1013.25) / 15.0, (294.15 - 299.65) / 15.0, (21.79 - 26.31) / 15.0, (0.00605 - 0.00630) / 15.0, (3.15 - 2.77) / 15.0},
	{(1015.75 - 1017.25) / 15.0, (283.15 - 294.15) / 15.0, (11.66 - 21.79) / 15.0, (0.00558 - 0.00605) / 15.0, (2.57 - 3.15) / 15.0},	
	{(1011.75 - 1015.75) / 15.0, (272.15 - 283.15) / 15.0, ( 6.78 - 11.66) / 15.0, (0.00539 - 0.00558) / 15.0, (1.81 - 2.57) / 15.0},
	{(1013.00 - 1011.75) / 15.0, (263.65 - 272.15) / 15.0, ( 4.11 -  6.78) / 15.0, (0.00453 - 0.00539) / 15.0, (1.55 - 1.81) / 15.0}};

static const double TropoTableSeason[5][5] = {
	{0,	    0,	  0,	0.00e-3, 0   },
	{-3.75,	7.0,  8.85,	0.25e-3, 0.33},	
	{-2.25,	11.0, 7.24,	0.32e-3, 0.46},
	{-1.75,	15.0, 5.36,	0.81e-3, 0.74},
	{-0.50,	14.5, 3.39,	0.62e-3, 0.30}};

static const double TropoTableSeasonDelta[4][5] = {
	{(-3.75 - 0.00) / 15.0, ( 7.0 -  0.0) / 15.0, (8.85 - 0.00) / 15.0,	(0.25e-3 - 0.00e-3) / 15.0, (0.33 - 0.00) / 15.0},
	{(-2.25 + 3.75) / 15.0, (11.0 -  7.0) / 15.0, (7.24 - 8.85) / 15.0,	(0.32e-3 - 0.25e-3) / 15.0, (0.46 - 0.33) / 15.0},	
	{(-1.75 + 2.25) / 15.0, (15.0 - 11.0) / 15.0, (5.36 - 7.24) / 15.0,	(0.81e-3 - 0.32e-3) / 15.0, (0.74 - 0.46) / 15.0},
	{(-0.50 + 1.75) / 15.0, (14.5 - 15.0) / 15.0, (3.39 - 5.36) / 15.0,	(0.62e-3 - 0.81e-3) / 15.0, (0.30 - 0.74) / 15.0}};

#define SEASON_VAR_SCALE 0.1204169668727093917549 // 2*pi/(365.25/7), used to multiply with week to compute cos(2*pi*(D-Dmin)/365.25)
#define DAY_MIN_NORTH 4.0	// 28 days equals to 4 weeks
#define DAY_MIN_SOUTH 30.142857142857142857143	// 211 days

//*************** Calculate troposphere delay ****************
// Parameters:
//   Elevation: satellite elevation angle in radian
//   pReceiverInfo: pointer to receiver information
// Return value:
//   troposphere delay in seconds
double TropoDelay(double Elevation, PRECEIVER_INFO pReceiverInfo)
{
	double temp = Elevation * Elevation;
	double SeasonVar;
	double p, T, e, beta, lambda;
	double ThermalParam, beta1, hyd, wet;
	const double k1 = 77.604;// k/mbar
	const double k2 = 382000;// k^2/mbar
	const double Rd = 287.054;// J/kg/K
	const double gm = 9.784;// m/s^2
	const double g = 9.80665;// m/s^2
	int LatDegree;

	// first try to find the current day of year on GPS time
	if (ReceiverWeekMsValid() && ReceiverWeekNumberValid())
		// based on GPS week 1669 start from 2012/01/01
		SeasonVar = pReceiverInfo->ReceiverTime->GpsWeekNumber - 1669 + (pReceiverInfo->ReceiverTime->GpsMsCount) / 604800000.0;
	// do not have date info, use simple equation
	else
		return (7.712e-9 / sin(sqrt(temp + 1.904e-3)) + 2.802e-10 / sin(sqrt(temp + 6.854e-4)));

	SeasonVar -= (pReceiverInfo->PosLLH.lat >= 0 ? DAY_MIN_NORTH : DAY_MIN_SOUTH);
	SeasonVar = cos(SEASON_VAR_SCALE * SeasonVar);

	LatDegree = ((int)(pReceiverInfo->PosLLH.lat * 360 / PI) + 1) / 2;
	if (LatDegree < 0)
		LatDegree = -LatDegree;

	p      = GetTropoParam(0, LatDegree, SeasonVar);
	T      = GetTropoParam(1, LatDegree, SeasonVar);
	e      = GetTropoParam(2, LatDegree, SeasonVar);
	beta   = GetTropoParam(3, LatDegree, SeasonVar);
	lambda = GetTropoParam(4, LatDegree, SeasonVar);

	ThermalParam = 1. - beta * pReceiverInfo->PosLLH.hae / T;
	if (ThermalParam <= 0)
		return 0.;
	beta1 = g / Rd / beta;
	lambda += 1.;

	hyd = 1e-6 * k1 * Rd / gm * p;
	wet = 1e-6 * k2 * Rd * e / (gm * lambda - beta * Rd) / T;
	hyd *= pow(ThermalParam, beta1);
	wet *= pow(ThermalParam, beta1 * lambda - 1);
	temp = sin(Elevation);
	temp *= temp;
	temp = 1.001 / sqrt(0.002001 + temp);
	return (hyd + wet) * temp / LIGHT_SPEED;
}

//*************** Calculate Hopfield's model parameters ****************
// Parameters:
//   ParamIndex: index of parameter
//   LatDegree: receiver latitude in degree
//   SeasonVar: seasonal variable
// Return value:
//   Hopfield's model parameter
double GetTropoParam(int ParamIndex, int LatDegree, double SeasonVar)
{
	int LatIndex;
	double e0, e_dot;

	if (LatDegree < 15)
	{
		e0 = TropoTableAverage[0][ParamIndex];
		e_dot = TropoTableSeason[0][ParamIndex];
	}
	else if (LatDegree > 75)
	{
		e0 = TropoTableAverage[4][ParamIndex];
		e_dot = TropoTableSeason[4][ParamIndex];
	}
	else
	{
		LatIndex = LatDegree / 15 - 1;
		LatDegree -= (LatIndex + 1) * 15;
		e0 = TropoTableAverage[LatIndex][ParamIndex] + TropoTableAverageDelta[LatIndex][ParamIndex] * LatDegree;
		e_dot = TropoTableSeason[LatIndex][ParamIndex] + TropoTableSeasonDelta[LatIndex][ParamIndex] * LatDegree;
	}

	return e0 - e_dot * SeasonVar;
}
