//----------------------------------------------------------------------
// PvtLsq.c:
//   PVT least square process functions
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

static void LSQResolve(double *DeltaPos, PHMATRIX H, double *DeltaPsr, double *InvMatrix, int dim);

//*************** Do LSQ position/velocity calculation ****************
// Parameters:
//   ObservationList: raw measurement pointer array
//   ObsCount: number of observations
//   LoopCount: maximum iteration number
// Return value:
//   -1 for not enough observations
//	 0 for not converge within given iterations
//   >0 for success with bit mask indicate participated system
int PvtLsq(PCHANNEL_STATUS ObservationList[], int ObsCount, int LoopCount)
{
	int i, iteration;
	int sv_index;
	int SystemNumber = 0;
	int SystemIndex[PVT_MAX_SYSTEM_ID];		// clock error index
	int PrevFreqID;
	PSATELLITE_INFO SatelliteInfo;
	double SolutionDelta[PVT_MAX_SYSTEM_ID+3];	// maximum 3 position + clock error
	double DeltaMsr[DIMENSION_MAX_X];
	double GeoDistance;
	double Residual;
	double dT;
	int UseSystemMask = 0;

	if (ObsCount < 3)
		return -1;

	// LSQ iteration
	for (iteration = 0; iteration < LoopCount; iteration ++)
	{
		PrevFreqID = -1;
		SatelliteInfo = g_GpsSatelliteInfo;
		dT = STATE_DT_GPS;
		UseSystemMask = 0;
		if ((ObservationList[0]->FreqID == FREQ_L1CA) || (ObservationList[0]->FreqID == FREQ_L1C))	// whether has GPS observation
		{
			SystemNumber = 1;
			SystemIndex[0] = 0;
			UseSystemMask = PVT_USE_GPS;
		}
		else
			SystemNumber = 0;

		for (i = 0; i < PVT_MAX_SYSTEM_ID; i ++)
			g_PvtCoreData.h.length[i] = 0;

		for (i = 0; i < ObsCount; i ++)
		{
			// observations are arranged to put same system together and with order GPS, BDS, Galileo
			if (ObservationList[i]->FreqID == FREQ_B1C && PrevFreqID != FREQ_B1C)
			{
				PrevFreqID = FREQ_B1C;
				SatelliteInfo = g_BdsSatelliteInfo;
				dT = STATE_DT_BDS;
				SystemIndex[SystemNumber] = 1;
				SystemNumber ++;
				UseSystemMask |= PVT_USE_BDS;
			}
			else if (ObservationList[i]->FreqID == FREQ_E1 && PrevFreqID != FREQ_E1)
			{
				PrevFreqID = FREQ_E1;
				SatelliteInfo = g_GalileoSatelliteInfo;
				dT = STATE_DT_GAL;
				SystemIndex[SystemNumber] = 2;
				SystemNumber ++;
				UseSystemMask |= PVT_USE_GAL;
			}
			sv_index = ObservationList[i]->svid - 1;

			GeoDistance = GeometryDistanceXYZ(&(STATE_X), SatelliteInfo[sv_index].PosVel.PosVel);
			DeltaMsr[i] = GeoDistance - ObservationList[i]->PseudoRange - dT;
							
			SatelliteInfo[sv_index].VectorX = g_PvtCoreData.h.data[0][i] = (SatelliteInfo[sv_index].PosVel.x - STATE_X) / GeoDistance;
			SatelliteInfo[sv_index].VectorY = g_PvtCoreData.h.data[1][i] = (SatelliteInfo[sv_index].PosVel.y - STATE_Y) / GeoDistance;
			SatelliteInfo[sv_index].VectorZ = g_PvtCoreData.h.data[2][i] = (SatelliteInfo[sv_index].PosVel.z - STATE_Z) / GeoDistance;
			SatelliteInfo[sv_index].SatInfoFlag |= SAT_INFO_LOS_VALID | SAT_INFO_LOS_MATCH;
			g_PvtCoreData.h.weight[i] = 1.0;// weight reserved for future weighted LSQ expansion
			g_PvtCoreData.h.length[SystemNumber-1] ++;
		}

		LSQResolve(SolutionDelta, &(g_PvtCoreData.h), DeltaMsr, g_PvtCoreData.PosInvMatrix, SystemNumber);

		// apply correction
		STATE_X += SolutionDelta[0];
		STATE_Y += SolutionDelta[1];
		STATE_Z += SolutionDelta[2];

		for (i = 0; i < SystemNumber; i ++)
			g_PvtCoreData.StateVector[SystemIndex[i]+7] += SolutionDelta[3+i];
	
		Residual = fabs(SolutionDelta[0]) + fabs(SolutionDelta[1]) + fabs(SolutionDelta[2]);
		if (Residual < 1e-3)
			break;
	}

	// calculate receiver velocity
	for (i = 1; i < SystemNumber; i ++)
		g_PvtCoreData.h.length[0] += g_PvtCoreData.h.length[i];

	PrevFreqID = FREQ_L1CA;
	SatelliteInfo = g_GpsSatelliteInfo;
	for (i = 0; i < ObsCount; i ++)
	{
		// observations are arranged to put same system together and with order GPS, BDS, Galileo
		if (ObservationList[i]->FreqID == FREQ_B1C && PrevFreqID != FREQ_B1C)
		{
			PrevFreqID = FREQ_B1C;
			SatelliteInfo = g_BdsSatelliteInfo;
		}
		else if (ObservationList[i]->FreqID == FREQ_E1 && PrevFreqID != FREQ_E1)
		{
			PrevFreqID = FREQ_E1;
			SatelliteInfo = g_GalileoSatelliteInfo;
		}
		sv_index = ObservationList[i]->svid - 1;
		DeltaMsr[i] = SatRelativeSpeedXYZ(&STATE_VX, SatelliteInfo[sv_index].PosVel.PosVel) + ObservationList[i]->Doppler;// - STATE_TDOT;
							
		// for velocity, H matrix has already initialized in position calculation, do not need to calculate again
//		g_PvtCoreData.h.weight[i] = 1.0;	// weight can be assigned different value for velocity calculation
	}
	LSQResolve(SolutionDelta, &(g_PvtCoreData.h), DeltaMsr, g_PvtCoreData.PosInvMatrix, 1);

	// assign result
	STATE_VX = SolutionDelta[0];
	STATE_VY = SolutionDelta[1];
	STATE_VZ = SolutionDelta[2];
	STATE_TDOT = SolutionDelta[3];

	return (iteration == LoopCount) ? 0 : UseSystemMask;
}

/*******************************************
* LSQ resolve of DeltaPsr=H*DeltaPos
*********************************************/
//*************** LSQ resolve of DeltaPsr=H*DeltaPos ****************
//* The result DeltaPos=Inv(HtWH)*HtW*DeltaPsr
// Parameters:
//   DeltaPos: result
//   H: H matrix
//   DeltaPsr: measurement difference
//   InvMatrix: place to hold Inv(HtWH)
//   dim: number of system participated
// Return value:
//   none
void LSQResolve(double *DeltaPos, PHMATRIX H, double *DeltaPsr, double *InvMatrix, int dim)
{
	double Delta[6];
	double TempVector[21];

	// compose delta by calculate Ht*DeltaPsr
	ComposeDelta(Delta, H, DeltaPsr, dim);

	// calculate Inv(HtH)
	GetHtH(H, (double *)0, InvMatrix, dim);	// HtH
	SymMatrixInv(InvMatrix, TempVector, dim+3);		// Inv(HtH)

	// calculate Inv(HtH)*Delta
	SymMatrixMultiply(DeltaPos, InvMatrix, Delta, dim+3);
}
