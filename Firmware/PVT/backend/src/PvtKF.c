//----------------------------------------------------------------------
// PvtKF.c:
//   PVT Kalman filter process functions
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
#include <string.h>

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

static void CalcQMatrix(double Qh, double Qv, PCONVERT_MATRIX pConvertMatrix, double QMatrix[]);
static double ObservationVariance(PCHANNEL_STATUS pChannelStatus, PSATELLITE_INFO pSatInfo, BOOL bVel);
static BOOL PsrObservationCheck(PCHANNEL_STATUS pChannelStatus, double DeltaPsr);
static BOOL DopplerObservationCheck(PCHANNEL_STATUS pChannelStatus, double DeltaDoppler);
static void SequencialUpdate(double UpdateVector[], double H[3], double *P, double Innovation, double r, int SystemIndex);

/* In Kalman filter, the P matrix is a symmetric matrix and is stored with following order
 p00
 p10 p11
 p20 p21 p22
 p30 p31 p32 p33
 p40 p41 p42 p43 p44
 p50 p51 p52 p53 p54 p55
 p60 p61 p62 p63 p64 p65 p66
 p70 p71 p72 p73 p74 p75 p76 p77
 p80 p81 p82 p83 p84 p85 p86 p87 p88
 p90 p91 p92 p93 p94 p95 p96 p97 p98 p99
 \----v----/  |  \----v----/  |   |   |
  VX/VY/VZ  TDOT    X/Y/Z    DTG DTC DTE
The P matrix elements have the same order as state vector, only lower triangle elements is stored
*/

//*************** Initialize P matrix for Kalman filter ****************
// assumptions here are:
//   if using weighted LSQ:
//     PMatrixInit is the Inv(HtWH) calculated in velocity calculation, in which W is the weight of PSR variance
//     Doppler observation STD is 1/20 of PSR observation, so velocity variance is (1/20)^2 of position variance
//     first 10 PMatrixInit/400 elements will be used to initialize P00~P33
//     first 6 PMatrixInit elements will be used to initialize P44~P66
//   if using unweighted LSQ:
//     diagonal elements of P00~P33 will be initialized to 0.25 (0.5m/s)^2
//     diagonal elements of P44~P66 will be initialized to 100 (10m)^2
//   P77/P88/P99 will be initialized to 100 if corresponding system used in positioning, otherwise 1e17 (approximate to (c*1s)^2)
//   all other elements be initialized to zero
// Parameters:
//   PMatrix: pointer to P matrix to be intialized
//   PMatrixInit: pointer to array used to initialize P matrix
//   PosFlag: flags of LSQ position result indicate which system used in LSQ PVT
// Return value:
//   none
void InitPMatrix(double *PMatrix, const double *PMatrixInit, unsigned int PosFlag)
{
	int i, j;
	double *pdest;
	const double *psrc;

	memset(PMatrix, 0, sizeof(double) * P_MATRIX_SIZE);
	// first 10 elements copy from velocity (PMatrixInit) calculated in LSQ velocity calculation
	// because in LSQ velocity uses the same weight as position, so need to apply 20^2 factor
	pdest = PMatrix;
	psrc = PMatrixInit;
	if (PosFlag & PVT_CONFIG_WEIGHTED_LSQ)
	{
		for (i = 0; i < 10; i ++)
			(*pdest ++) = (*psrc ++) / 400.;
	}
	else
	{
		PMatrix[0] = PMatrix[2] = PMatrix[5] = PMatrix[9] = 0.25;
		pdest = PMatrix + 10;
	}

	// start copy position matrix
	psrc = PMatrixInit;
	for (i = 0; i < 3; i ++)
	{
		// skip 4 elements
		pdest += 4;
		for (j = 0; j <= i; j ++)
			(*pdest ++) = (PosFlag & PVT_CONFIG_WEIGHTED_LSQ) ? (*psrc ++) : ((i == j) ? 1e2 : 0.0);
	}
	// start copy dt elements
	for (i = 0; i < PVT_MAX_SYSTEM_ID; i ++)
	{
		// skip to diagnal elements
		pdest += (i + 7);
		// if no corresponding dt, assign 1e17 (approximate to (c*1s)^2) to diagnal element
		// otherwise assign (10m)^2
		// cross-correlation elements set to 0
		if ((PosFlag & (1 << i)) == 0)
			*pdest ++ = 1e17;
		else
			*pdest ++ = 1e2;
	}
}

//*************** Do Kalman filter prediction ****************
// because state prediction has already been calculated, so only P matrix prediction is done here
// given state vector as [VX VY VZ TDOT X Y Z DT_GPS DT_BDS DT_GAL], the one-step transition matrix Phi(A) is
// [  1,  0,  0,  0, 0, 0, 0, 0, 0, 0]
// [  0,  1,  0,  0, 0, 0, 0, 0, 0, 0]
// [  0,  0,  1,  0, 0, 0, 0, 0, 0, 0]
// [  0,  0,  0,  1, 0, 0, 0, 0, 0, 0]
// [ dT,  0,  0,  0, 1, 0, 0, 0, 0, 0]
// [  0, dT,  0,  0, 0, 1, 0, 0, 0, 0]
// [  0,  0, dT,  0, 0, 0, 1, 0, 0, 0]
// [  0,  0,  0, dT, 0, 0, 0, 1, 0, 0]
// [  0,  0,  0, dT, 0, 0, 0, 0, 1, 0]
// [  0,  0,  0, dT, 0, 0, 0, 0, 0, 1]
// so A*P*A' will be P plus following matrix
// [      0,      0,      0,      0,                     dT*p00,                     dT*p10,                     dT*p20,                     dT*p30,                     dT*p30,                     dT*p30]
// [      0,      0,      0,      0,                     dT*p10,                     dT*p11,                     dT*p21,                     dT*p31,                     dT*p31,                     dT*p31]
// [      0,      0,      0,      0,                     dT*p20,                     dT*p21,                     dT*p22,                     dT*p32,                     dT*p32,                     dT*p32]
// [      0,      0,      0,      0,                     dT*p30,                     dT*p31,                     dT*p32,                     dT*p33,                     dT*p33,                     dT*p33]
// [ dT*p00, dT*p10, dT*p20, dT*p30, dT*p40 + dT*(p40 + dT*p00), dT*p50 + dT*(p41 + dT*p10), dT*p60 + dT*(p42 + dT*p20), dT*p70 + dT*(p43 + dT*p30), dT*p80 + dT*(p43 + dT*p30), dT*p90 + dT*(p43 + dT*p30)]
// [ dT*p10, dT*p11, dT*p21, dT*p31, dT*p41 + dT*(p50 + dT*p10), dT*p51 + dT*(p51 + dT*p11), dT*p61 + dT*(p52 + dT*p21), dT*p71 + dT*(p53 + dT*p31), dT*p81 + dT*(p53 + dT*p31), dT*p91 + dT*(p53 + dT*p31)]
// [ dT*p20, dT*p21, dT*p22, dT*p32, dT*p42 + dT*(p60 + dT*p20), dT*p52 + dT*(p61 + dT*p21), dT*p62 + dT*(p62 + dT*p22), dT*p72 + dT*(p63 + dT*p32), dT*p82 + dT*(p63 + dT*p32), dT*p92 + dT*(p63 + dT*p32)]
// [ dT*p30, dT*p31, dT*p32, dT*p33, dT*p43 + dT*(p70 + dT*p30), dT*p53 + dT*(p71 + dT*p31), dT*p63 + dT*(p72 + dT*p32), dT*p73 + dT*(p73 + dT*p33), dT*p83 + dT*(p73 + dT*p33), dT*p93 + dT*(p73 + dT*p33)]
// [ dT*p30, dT*p31, dT*p32, dT*p33, dT*p43 + dT*(p80 + dT*p30), dT*p53 + dT*(p81 + dT*p31), dT*p63 + dT*(p82 + dT*p32), dT*p73 + dT*(p83 + dT*p33), dT*p83 + dT*(p83 + dT*p33), dT*p93 + dT*(p83 + dT*p33)]
// [ dT*p30, dT*p31, dT*p32, dT*p33, dT*p43 + dT*(p90 + dT*p30), dT*p53 + dT*(p91 + dT*p31), dT*p63 + dT*(p92 + dT*p32), dT*p73 + dT*(p93 + dT*p33), dT*p83 + dT*(p93 + dT*p33), dT*p93 + dT*(p93 + dT*p33)]
// Parameters:
//   PMatrix: pointer to P matrix to be intialized
//   DeltaT: time interval
// Return value:
//   none
void KFPrediction(double *PMatrix, double DeltaT)
{
	const int line_start[] = {0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55};
	int i, j;
	double *p, *p1, *p2, *p3;
	double DeltaT2 = DeltaT * DeltaT;
	double DeltaT3 = DeltaT2 * PMatrix[9]; //DeltaT^2 * p33
	double TempArray[P_MATRIX_SIZE - 10];	// to store result except first 10 elements

	// first copy P matrix to TempArray (do not copy first 10 elements because they are unchanged)
	memcpy(TempArray, PMatrix + 10, sizeof(TempArray));

	p = TempArray;	// p point to element to be changed in P matrix
	// then the next 3 lines have the following value added
	// dT*p00, dT*p10, dT*p20, dT*p30, dT*(p40 + p40) + dT*dT*p00
	// dT*p10, dT*p11, dT*p21, dT*p31, dT*(p41 + p50£©+ dT*dT*p10, dT*(p51 + p51) + dT*dT*p11,
	// dT*p20, dT*p21, dT*p22, dT*p32, dT*(p42 + p60) + dT*dT*p20, dT*(p52 + p61) + dT*dT*p21, dT*(p62 + p62) + dT*dT*p22,
	p3 = PMatrix;	// p3 point to DeltaT^2 factor element
	for (i = 0; i < 3; i ++)
	{
		p1 = PMatrix + line_start[i]; // p1 point to first DeltaT factor elements
		for (j = 1; j <= 4; j ++)
		{
			(*p ++) += (*p1) * DeltaT;
			p1 += j;
			if (i == 2 && j == 2)
				p1 --;
		}
		p2 = PMatrix + line_start[i+4];	// p2 point to second DeltaT factor elements
		for (j = 0; j <= i; j ++)
		{
			*p += ((*p1) + (*p2 ++)) * DeltaT;
			p1 += (j + 5);
			(*p ++) += (*p3 ++) * DeltaT2;
		}
	}
	// then the next 3 lines have the following value added
	for (i = 0; i < PVT_MAX_SYSTEM_ID; i ++)
	{
		// first 3 columns
		// dT*p30, dT*p31, dT*p32, dT*p33
		// dT*p30, dT*p31, dT*p32, dT*p33
		// dT*p30, dT*p31, dT*p32, dT*p33
		p1 = PMatrix + 6;	// p1 point to p30
		for (j = 0; j < 4; j ++)
		{
			(*p ++) += (*p1 ++) * DeltaT;
		}
		// next 3 columns
		// dT*(p43 + p70) + dT*dT*p30, dT*(p53 + p71) + dT*dT*p31, dT*(p63 + p72) + dT*dT*p32
		// dT*(p43 + p80) + dT*dT*p30, dT*(p53 + p81) + dT*dT*p31, dT*(p63 + p82) + dT*dT*p32
		// dT*(p43 + p90) + dT*dT*p30, dT*(p53 + p91) + dT*dT*p31, dT*(p63 + p92) + dT*dT*p32
		p1 += 3;	// point to p43
		p2 = PMatrix + line_start[i+7];	// p2 point to second DeltaT factor elements
		p3 = PMatrix + 6;	// p3 point to p30
		for (j = 0; j < 3; j ++)
		{
			*p += ((*p1) + (*p2 ++)) * DeltaT;
			p1 += (j + 5);
			(*p ++) += (*p3 ++) * DeltaT2;
		}
		// last triangle
		// dT*(p73 + p73) + dT*dT*p33
		// dT*(p73 + p83) + dT*dT*p33, dT*(p83 + p83) + dT*dT*p33
		// dT*(p73 + p93) + dT*dT*p33, dT*(p83 + p93) + dT*dT*p33, dT*(p93 + p93) + dT*dT*p33
		for (j = 0; j <= i; j ++)
		{
			*p += ((*p1) + (*p2)) * DeltaT;
			p1 += (j + 8);
			(*p ++) += DeltaT3;
		}
	}

	// finally copy TempArray back to P matrix
	memcpy(PMatrix + 10, TempArray, sizeof(TempArray));
}

//*************** Add Q matrix to P matrix ****************
// the following is the Q matrix calculation and adding to P matrix
//         xdot   ydot   zdot    tdot       x      y      z      dt1   dt2   dt3
// xdot / Qxx*T1               |       |                      |                   \
// ydot | Qxy*T1 Qyy*T1        |       |                      |                   |
// zdot | Qxz*T1 Qyz*T1 Qzz*T1 |       |                      |                   |
//      |-------------------------------------------------------------------------|
// tdot |           0          | Qf*T1 |                      |                   |
//      |-------------------------------------------------------------------------|
//   x  | Qxx*T2 Qxy*T2 Qxz*T2 |       | Qxx*T3               |                   |
//   y  | Qxy*T2 Qyy*T2 Qyz*T2 |   0   | Qxy*T3 Qyy*T3        |                   |
//   z  | Qxz*T2 Qyz*T2 Qzz*T2 |       | Qxz*T3 Qyz*T3 Qzz*T3 |                   |
//      |-------------------------------------------------------------------------|
//  dt1 |                      | Qf*T2 |                      | Qf*T3             |
//  dt2 |           0          | Qf*T2 |           0          | Qf*T3 Qf*T3       |
//  dt3 \                      | Qf*T2 |                      | Qf*T3 Qf*T3 Qf*T3 /
// 
// Qxx, Qxy, Qyy, Qxz, Qyz, Qzz derived from Qh and Qv, Qf is noise on clock drifting
// T1=DeltaT, T2=(DeltaT^2)/2, T3=(DeltaT^3)/3
// Parameters:
//   PMatrix: pointer to P matrix
//   QConfig: array of Qh, Qv and Qf
//   pConvertMatrix: pointer to ECEF to ENU conversion matrix
//   DeltaT: time interval
// Return value:
//   none
void KFAddQMatrix(double *PMatrix, const double *QConfig, PCONVERT_MATRIX pConvertMatrix, double DeltaT)
{
	int i, j;
	double Qh = QConfig[0], Qv = QConfig[1], Qf = QConfig[2], Qxyz[9];
	double *p = PMatrix, *q;
	double T2 = DeltaT *  DeltaT / 2.;
	double T3 = DeltaT *  DeltaT * DeltaT / 3.;

	// calculate Qxyz from Qh, Qv andn Qf
	CalcQMatrix(Qh, Qv, pConvertMatrix, Qxyz);
	
	// xdot ydot zdot vs. xdot ydot zdot part
	q = Qxyz;
	for(i = 0; i < 3; i++)
	{
		for (j = 0; j <= i; j ++)
			(*p ++) += (*q ++) * DeltaT;
		q += (2 - i);
	}
	// skip xdot ydot zdot vs. tdot
	p += 3;

	// tdot vs tdot
	(*p ++) += Qf * DeltaT;

	// x y z vs. xdot ydot zdot and x y z vs. x y z
	q = Qxyz;
	for (i = 0; i < 3; i ++)
	{
		// x y z vs. xdot ydot zdot
		for (j = 0; j < 3; j ++)
			(*p ++) += (*q ++) * T2;
		// skip x y z vs. tdot
		p ++;
		// x y z vs. x y z
		q -= 3;
		for (j = 0; j <= i; j ++)
			(*p ++) += (*q ++) * T3;
		q += (2 - i);
	}

	// dt1 dt2 dt3 vs. tdot and dt1 dt2 dt3 vs. dt1 dt2 dt3
	for (i = 0; i < PVT_MAX_SYSTEM_ID; i ++)
	{
		// skip dt1 dt2 dt3 vs. xdot ydot zdot
		p += 3;
		// dt1 dt2 dt3 vs. tdot
		(*p ++) += Qf * T2;
		// skip dt1 dt2 dt3 vs. x y z
		p += 3;
		// dt1 dt2 dt3 vs. dt1 dt2 dt3
		for (j = 0; j <= i; j ++)
			(*p ++) += Qf * T3;
	}
}

//*************** Calculate Q matrix in XYZ coordinates from Q matrix in ENU coordinates ****************
// In ENU coordinates, Q matrix is diagonal matrix with elements Qh, Qh and Qv
// Output a 3x3 symmetrical matrix
// the result Q matrix is
// [ Qh + dQ*cos(lat)^2*cos(lon)^2,             dQ*cos(lon)*sin(lon)*cos(lat)^2,      dQ*cos(lat)*cos(lon)*sin(lat)]
// [      dQ*cos(lon)*sin(lon)*cos(lat)^2, Qh + dQ*cos(lat)^2*sin(lon)^2,             dQ*cos(lat)*sin(lat)*sin(lon)]
// [      dQ*cos(lat)*cos(lon)*sin(lat),        dQ*cos(lat)*sin(lat)*sin(lon),   Qh + dQ*sin(lat)^2                ]
// equals
// [ Qh + dQ*Cx2u^2,         dQ*Cx2u*Cy2u,       dQ*Cx2u*Cz2u]
// [      dQ*Cx2u*Cy2u, Qh + dQ*Cy2u^2,          dQ*Cy2u*Cz2u]
// [      dQ*Cx2u*Cz2u,      dQ*Cy2u*Cz2u], Qh + dQ*Cz2u^2   ]
// Parameters:
//   Qh: horizontal Q (variance on horizontal acceleration)
//   Qv: vertical Q (variance on vertical acceleration)
//   pConvertMatrix: pointer to ECEF to ENU conversion matrix
//   QMatrix: pointer to a 3x3 Qxyz matrix
// Return value:
//   none
void CalcQMatrix(double Qh, double Qv, PCONVERT_MATRIX pConvertMatrix, double QMatrix[])
{
	double dQ = Qv - Qh;
	double Cx2u = pConvertMatrix->x2u;
	double Cy2u = pConvertMatrix->y2u;
	double Cz2u = pConvertMatrix->z2u;

	QMatrix[0] = Qh + dQ * Cx2u * Cx2u;
	QMatrix[1] =      dQ * Cx2u * Cy2u;
	QMatrix[2] =      dQ * Cx2u * Cz2u;
	QMatrix[3] =      QMatrix[1];
	QMatrix[4] = Qh + dQ * Cy2u * Cy2u;
	QMatrix[5] =      dQ * Cy2u * Cz2u;
	QMatrix[6] =      QMatrix[2];
	QMatrix[7] =      QMatrix[5];
	QMatrix[8] = Qh + dQ * Cz2u * Cz2u;;
}

//*************** Do Kalman filter positioning ****************
// Parameters:
//   ObservationList: raw measurement pointer array
//   ObsCount: number of observations
//   PosUseSatCount: number of observation used in positioning
// Return value:
//   bit mask indicate participated system
int KFPosition(PCHANNEL_STATUS ObservationList[], int ObsCount, int PosUseSatCount[PVT_MAX_SYSTEM_ID])
{
	int i, j;
	int SystemIndex = 0;
	int sv_index;
	const int DtIndex[3] = { 35, 44, 54 };
	double UpdateVector[STATE_VECTOR_SIZE];
	double DeltaPsr, DeltaDoppler;
	double H[3];
	double GeoDistance;
	double *dT = &STATE_DT_GPS;
	int PrevFreqID = -1;
	PSATELLITE_INFO SatelliteInfo = g_GpsSatelliteInfo;
	int UseSystemMask = 0;

	for (i = 0; i < PVT_MAX_SYSTEM_ID; i ++)
		PosUseSatCount[i] = g_PvtCoreData.h.length[i] = 0;

	for (i = 0; i < ObsCount; i ++)
	{
		// observations are arranged to put same system together and with order GPS, BDS, Galileo
		if (ObservationList[i]->FreqID == FREQ_B1C && PrevFreqID != FREQ_B1C)
		{
			PrevFreqID = FREQ_B1C;
			SatelliteInfo = g_BdsSatelliteInfo;
			SystemIndex = 1;
		}
		else if (ObservationList[i]->FreqID == FREQ_E1 && PrevFreqID != FREQ_E1)
		{
			PrevFreqID = FREQ_E1;
			SatelliteInfo = g_GalileoSatelliteInfo;
			SystemIndex = 2;
		}
		sv_index = ObservationList[i]->svid - 1;

		// calculate PSR and Doppler residual
		GeoDistance = GeometryDistanceXYZ(&(STATE_X), SatelliteInfo[sv_index].PosVel.PosVel);
		DeltaPsr = GeoDistance - ObservationList[i]->PseudoRange - dT[SystemIndex];
		DeltaDoppler = SatRelativeSpeedXYZ(&STATE_VX, SatelliteInfo[sv_index].PosVel.PosVel) + ObservationList[i]->Doppler - STATE_TDOT;

		// calculate square root of variance of PSR and Doppler
		ObservationList[i]->PsrVariance = ObservationVariance(ObservationList[i], &SatelliteInfo[sv_index], 0);
		ObservationList[i]->DopplerVariance = ObservationVariance(ObservationList[i], &SatelliteInfo[sv_index], 1);

		SatelliteInfo[sv_index].VectorX = g_PvtCoreData.h.data[0][i] = H[0] = (SatelliteInfo[sv_index].PosVel.x - STATE_X) / GeoDistance;
		SatelliteInfo[sv_index].VectorY = g_PvtCoreData.h.data[1][i] = H[1] = (SatelliteInfo[sv_index].PosVel.y - STATE_Y) / GeoDistance;
		SatelliteInfo[sv_index].VectorZ = g_PvtCoreData.h.data[2][i] = H[2] = (SatelliteInfo[sv_index].PosVel.z - STATE_Z) / GeoDistance;
		SatelliteInfo[sv_index].SatInfoFlag |= SAT_INFO_LOS_VALID | SAT_INFO_LOS_MATCH;
		g_PvtCoreData.h.weight[i] = 1.0;// weight reserved for future weighted LSQ expansion

		if (fabs(g_PvtCoreData.PMatrix[DtIndex[SystemIndex]]) > 1e10 || PsrObservationCheck(ObservationList[i], DeltaPsr))
		{
			UseSystemMask |= (1 << SystemIndex);
			SequencialUpdate(UpdateVector, H, g_PvtCoreData.PMatrix, DeltaPsr, ObservationList[i]->PsrVariance, SystemIndex + 1);
			for (j = 0; j < STATE_VECTOR_SIZE; j ++)
				g_PvtCoreData.StateVector[j] += UpdateVector[j];

			g_PvtCoreData.h.length[0] ++;
		}
		if (DopplerObservationCheck(ObservationList[i], DeltaDoppler))
		{
			SequencialUpdate(UpdateVector, H, g_PvtCoreData.PMatrix, DeltaDoppler, ObservationList[i]->DopplerVariance, 0);
			for (j = 0; j < STATE_VECTOR_SIZE; j ++)
				g_PvtCoreData.StateVector[j] += UpdateVector[j];
		}
	}

	return UseSystemMask;
}

//*************** return the observation variance for PSR and Doppler ****************
// for PSR, the unit is m^2, for Doppler, the unit is (m/s)^2
// Parameters:
//   pChannelStatus: pointer to channel status structure
//   pSatInfo: pointer to satellite information structure
//   bVel: true to get Doppler variance, false to get PSR variance
// Return value:
//   variance of PSR or Doppler observation
double ObservationVariance(PCHANNEL_STATUS pChannelStatus, PSATELLITE_INFO pSatInfo, BOOL bVel)
{
	double Var, Elevation;

	// linearize variance calculation depend on CN0
	// 5m for 48dBHz ~ 45m for 8dBHz
	if(pChannelStatus->cn0 > 4800)
		Var = 5.0;
	else if(pChannelStatus->cn0 > 800)
		Var = 5.0 + (4800 - pChannelStatus->cn0) / 100.0;
	else
		Var = 50.;

	// LockTime too short need to increase variance
	if(pChannelStatus->LockTime < 2500)
		Var /= ((pChannelStatus->LockTime + 500) / 3000.);

	// elevation adjustment
	if (pSatInfo->SatInfoFlag & SAT_INFO_ELAZ_VALID)
		Elevation = pSatInfo->el;
	else
		Elevation = 0.0872664626;	// 5 degree
	Var += 0.5 / sin(Elevation);

	if(bVel)
		Var /= 20.;

	return Var*Var;
}

//*************** check validation of PSR observation ****************
// if PSR residual is less than 5 times of STD, the observation is valid
// if PSR residual is less than 10 times of STD, the observation is valid but increase variance (decrease weight)
// otherwise, the observation is invalid
// Parameters:
//   pChannelStatus: pointer to channel status structure
//   DeltaPsr: observation residual
// Return value:
//   whether observation is valid
BOOL PsrObservationCheck(PCHANNEL_STATUS pChannelStatus, double DeltaPsr)
{
	double SqrPsr = DeltaPsr * DeltaPsr;

	if (SqrPsr < 25.0 * pChannelStatus->PsrVariance)
		return 1;
	else if (SqrPsr < 100.0 * pChannelStatus->PsrVariance)
	{
		pChannelStatus->PsrVariance = SqrPsr;
		return 1;
	}
	else
		return 0;
}

//*************** check validation of Doppler observation ****************
// if Doppler residual is less than 5 times of STD, the observation is valid
// if Doppler residual is less than 10 times of STD, the observation is valid but increase variance (decrease weight)
// otherwise, the observation is invalid
// Parameters:
//   pChannelStatus: pointer to channel status structure
//   DeltaPsr: observation residual
// Return value:
//   whether observation is valid
BOOL DopplerObservationCheck(PCHANNEL_STATUS pChannelStatus, double DeltaDoppler)
{
	double SqrDoppler = DeltaDoppler * DeltaDoppler;

	if (SqrDoppler < 25.0 * pChannelStatus->DopplerVariance)
		return 1;
	else if (SqrDoppler < 100.0 * pChannelStatus->DopplerVariance)
	{
		pChannelStatus->DopplerVariance = SqrDoppler;
		return 1;
	}
	else
		return 0;
}

//*************** sequencial update of one PSR or Doppler observation in Kalman filter ****************
// The H matrix for update of each system has the following values:
//   for GPS PSR: H = [0 0 0 0 rx ry rz 1 0 0]
//   for BDS PSR: H = [0 0 0 0 rx ry rz 0 1 0]
//   for GAL PSR: H = [0 0 0 0 rx ry rz 0 0 1]
//   for Doppler: H = [rx ry rz 1 0 0 0 0 0 0]
// The sequencial update will be performed with following steps:
//   1. calculate vector PHt = P*H'
//     for Doppler, PHt is:
//       p30 + p00*rx + p10*ry + p20*rz
//       p31 + p10*rx + p11*ry + p21*rz
//       p32 + p20*rx + p21*ry + p22*rz
//       p33 + p30*rx + p31*ry + p32*rz
//       p43 + p40*rx + p41*ry + p42*rz
//       p53 + p50*rx + p51*ry + p52*rz
//       p63 + p60*rx + p61*ry + p62*rz
//       p73 + p70*rx + p71*ry + p72*rz
//       p83 + p80*rx + p81*ry + p82*rz
//       p93 + p90*rx + p91*ry + p92*rz
//     for GPS/BDS/Galileo, PHt is:	
//       p70/p80/p90 + p40*rx + p50*ry + p60*rz
//       p71/p81/p91 + p41*rx + p51*ry + p61*rz
//       p72/p82/p92 + p42*rx + p52*ry + p62*rz
//       p73/p83/p93 + p43*rx + p53*ry + p63*rz
//       p74/p84/p94 + p44*rx + p54*ry + p64*rz
//       p75/p85/p95 + p54*rx + p55*ry + p65*rz
//       p76/p86/p96 + p64*rx + p65*ry + p66*rz
//       p77/p87/p97 + p74*rx + p75*ry + p76*rz
//       p87/p88/p98 + p84*rx + p85*ry + p86*rz
//       p97/p98/p99 + p94*rx + p95*ry + p96*rz
//   2. calculate Inv(HPH'+R) = 1 / (H*PHt + r), which is a scalar value
//   3. calculate gain vector K = P*H'*Inv(HPH'+R)
//   4. calculate vector update value DeltaX = K * Innovation
//   5. update P matrix as P = P - K*(H*P) = P - K*(P'*H')' = P - K(PHt)' (P is a symmetrix matrix, so P'*H'=P*H'=PHt)
// Parameters:
//   UpdateVector: update values to state vector
//   H: first three elements (rx/ry/rz) of H matrix
//   P: pointer to P matrix
//   Innovation: innovation (residual) of observation
//   r: variance of observation
//   SystemIndex: 1~3 for GPS/BDS/Galileo PSR update respectively, 0 for Doppler update
// Return value:
//   whether observation is valid
void SequencialUpdate(double UpdateVector[], double H[3], double *P, double Innovation, double r, int SystemIndex)
{
	int i, j, len;
	const int start_pos1[4] = { 6, 28, 36, 45};
	const int start_pos2[2][3] = {{0, 1, 3}, {10, 15, 21}};
	double PHt[STATE_VECTOR_SIZE], K[STATE_VECTOR_SIZE], *pdest, *psrc, temp;

	// step 1, calculate PHt
	pdest = PHt;
	psrc = P + start_pos1[SystemIndex];	// point to first column
	len = (SystemIndex == 0) ? 4 : (7 + SystemIndex);
	for (i = 0; i < len; i ++)
		(*pdest ++) = (*psrc ++);
	psrc += (i - 1);
	for (; i < STATE_VECTOR_SIZE; i ++)
	{
		(*pdest ++) = (*psrc);
		psrc += (i + 1);
	}
	
	for (j = 0; j < 3; j ++)
	{
		pdest = PHt;
		temp = H[j];
		psrc = P + ((SystemIndex == 0) ? start_pos2[0][j] : start_pos2[1][j]);	// point to column2/3/4
		len = (SystemIndex == 0) ? (j + 1) : (j + 5);
		for (i = 0; i < len; i ++)
			(*pdest ++) += (*psrc ++) * temp;
		psrc += (i - 1);
		for (; i < STATE_VECTOR_SIZE; i ++)
		{
			(*pdest ++) += (*psrc) * temp;
			psrc += (i + 1);
		}
	}
	
	// step 2, calculate Inv(HPH'+r)
	psrc = (SystemIndex == 0) ? &PHt[0] : &PHt[4];
	temp = H[0] * psrc[0] + H[1] * psrc[1] + H[2] * psrc[2];
	temp += (SystemIndex == 0) ? PHt[3] : PHt[6 + SystemIndex];
	temp += r;
	temp = 1.0 / temp;

	// step 3, calculate K and update vector K*Innovation
	for (i = 0; i < STATE_VECTOR_SIZE; i ++)
	{
		K[i] = PHt[i] * temp;
		UpdateVector[i] = K[i] * Innovation;
	}

	// step 4, update P matrix
	pdest = P;
	for(i = 0; i < STATE_VECTOR_SIZE; i ++)
	{
		temp = K[i];
		psrc = PHt;
		for(j = 0; j <= i; j ++)
		{
			// P[i][j] -= K[i] * PHt[j]
			(*pdest ++) -= (*psrc ++) * temp;
		}
	}
}
