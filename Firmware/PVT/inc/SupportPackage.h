//----------------------------------------------------------------------
// SupportPackage.h:
//   This header file gathers functions and definitions used across PVT
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __SUPPORT_PACKAGE_H__
#define __SUPPORT_PACKAGE_H__

#include "DataTypes.h"

// ONES(n) is macro to get n continuous 1s
#define ONES(n) ((1 << n) - 1)
// GET_UBITS gets number of len bits starting from bit pos of data (bit pos is LSB)
#define GET_UBITS(data, pos, len) ((unsigned int)((data) >> (pos)) & ONES(len))
// GET_BITS is the same as GET_UBITS but sign extended (assuming data is a signed int)
#define GET_BITS(data, pos, len) (((((signed int)data) << (32 - pos - len)) & (ONES(len) << (32 - len))) >> (32 - len))

#define COMPOSE64(pData64, DataHi, DataLo) \
do \
{ \
	*(unsigned int *)(pData64) = DataLo; \
	*((unsigned int *)(pData64) + 1) = DataHi; \
} while (0)

#define CUBE(x) ((x) * (x) * (x))

// basic functions
double ScaleDouble(int value, int scale);
double ScaleDoubleU(unsigned int value, int scale);
double ScaleDoubleLong(long long value, int scale);
double ScaleDoubleULong(unsigned long long value, int scale);
BOOL GpsParityCheck(unsigned int word);

// conversion functions
void EcefToLlh(const KINEMATIC_INFO *ecef_pos, LLH *llh_pos);
void LlhToEcef (const LLH *llh_pos, KINEMATIC_INFO *ecef_pos);
void GpsTimeToUtc(int GpsWeek, int WeekMsCount, PSYSTEM_TIME pUtcTime, PUTC_PARAM pUtcParam);
void UtcToGpsTime(PSYSTEM_TIME pUtcTime, int *pGpsWeek, int *pWeekMsCount, PUTC_PARAM pUtcParam);
void GlonassTimeToUtc(int LeapYears, int DayNumber, int DayMsCount, PSYSTEM_TIME pUtcTime);
void UtcToGlonassTime(PSYSTEM_TIME pUtcTime, int *LeapYears, int *DayNumber, int *DayMsCount);
void CalcConvMatrix(KINEMATIC_INFO *pReceiverPos, PCONVERT_MATRIX pConvertMatrix);

// satellite coordinate related functions
double ClockCorrection(PGNSS_EPHEMERIS pEph, double TransmitTime);
int SatPosSpeedEph(double TransmitTime, PGNSS_EPHEMERIS pEph, PKINEMATIC_INFO pPosVel);
void SatPosSpeedAlm(int WeekNumber, int TransmitTime, PMIDI_ALMANAC pAlm, PKINEMATIC_INFO pPosVel);
double GeometryDistanceXYZ(const double *ReceiverPos, const double *SatellitePos);
double GeometryDistance(const PKINEMATIC_INFO pReceiver, const PKINEMATIC_INFO pSatellite);
double SatRelativeSpeed(PKINEMATIC_INFO pReceiver, PSATELLITE_INFO pSatellite);
double SatRelativeSpeedXYZ(double *ReceiverState, double *SatPosVel);
void SatElAz(PKINEMATIC_INFO pReceiver, PSATELLITE_INFO pSatellite);

// matrix related functions
void ComposeDelta(double *Delta, PHMATRIX H, double *MsrDelta, int dim);
void GetHtH(PHMATRIX DesignMatrix, double *InvP, double *HtH, int dim);
void SymMatrixInv(double *SymMat, double *WorkSpace, int dim);
void SymMatrixMultiply(double *DeltaPos, double *Inv, double *Delta, int dim);

// position fix functions
int PvtLsq(PCHANNEL_STATUS ObservationList[], int ObsCount, int LoopCount);

#endif //__SUPPORT_PKG_H__
