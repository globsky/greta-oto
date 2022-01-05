//----------------------------------------------------------------------
// GnssTop.cpp:
//   GNSS baseband top module simulator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __SIM_TOP_H__
#define __SIM_TOP_H__

#include "BasicTypes.h"
#include "Trajectory.h"
#include "GnssTime.h"
#include "NavData.h"
#include "LNavBit.h"
#include "BCNavBit.h"
#include "XmlInterpreter.h"
#include "SatelliteParam.h"
#include "TeFifoSim.h"
#include "AcqEngine.h"
#include "TrackingEngine.h"

typedef void (*InterruptFunction)();
typedef int S32;
typedef unsigned int U32;

#define TOTAL_GPS_SAT 32
#define TOTAL_BDS_SAT 63
#define TOTAL_GAL_SAT 50

class CGnssTop
{
public:
	CGnssTop();
	~CGnssTop();
	void Reset(U32 ResetMask);
	void Clear(U32 ClearMask);
	void SetRegValue(int Address, U32 Value);
	U32 GetRegValue(int Address);

	// global registers
	U32 TrackingEngineEnable;		// 1bit
	U32 MeasurementNumber;			// 10bit
	U32 MeasurementCount;			// 10bit
	U32 ReqCount;					// 10bit
	U32 InterruptFlag;				// 4bit, bit8~11
	U32 IntMask;					// 4bit, bit8~11

	// variables for SignalSim
	GNSS_TIME CurTime;
	UTC_TIME UtcTime;
	CTrajectory Trajectory;
	CNavData NavData;
	KINEMATIC_INFO CurPos;
	LOCAL_SPEED StartVel;
	OUTPUT_PARAM OutputParam;
	CPowerControl PowerControl;
	LNavBit GpsBits;
	BCNavBit BdsBits;
	NavBit *NavBitArray[4];

	PGPS_EPHEMERIS GpsEph[TOTAL_GPS_SAT], GpsEphVisible[TOTAL_GPS_SAT];
	PGPS_EPHEMERIS BdsEph[TOTAL_BDS_SAT], BdsEphVisible[TOTAL_BDS_SAT];
	PGPS_EPHEMERIS GalEph[TOTAL_GAL_SAT], GalEphVisible[TOTAL_GAL_SAT];
	SATELLITE_PARAM GpsSatParam[TOTAL_GPS_SAT], BdsSatParam[TOTAL_BDS_SAT], GalSatParam[TOTAL_GAL_SAT];	// satellite parameter array at CurTime
	PSATELLITE_PARAM SatParamList[TOTAL_GPS_SAT+TOTAL_BDS_SAT+TOTAL_GAL_SAT];
	int GpsSatNumber, BdsSatNumber, GalSatNumber;	// number of visible GPS satellite
	int TotalSatNumber;	// total number of visible GPS satellite

	CTrackingEngine TrackingEngine;
	CAcqEngine AcqEngine;
	CTeFifoSim TeFifo;

	int Process(int ReadBlockSize);
	void SetInputFile(char *FileName);
	int StepToNextTime(int TimeInterval);

	InterruptFunction InterruptService;
};

#endif //__SIM_TOP_H__
