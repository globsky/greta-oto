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
#include "XmlInterpreter.h"
#include "SatelliteParam.h"
#include "TeFifoSim.h"
#include "AcqEngine.h"
#include "TrackingEngine.h"

typedef void (*InterruptFunction)();
typedef int S32;
typedef unsigned int U32;

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
	LNavBit GpsBits;

	PGPS_EPHEMERIS GpsEph[32], GpsEphVisible[32];
	SATELLITE_PARAM GpsSatParam[32];	// GPS satellite parameter array at CurTime
	int GpsSatNumber;	// number of visible GPS satellite

	CTrackingEngine TrackingEngine;
	CAcqEngine AcqEngine;
	CTeFifoSim TeFifo;

	int Process(int ReadBlockSize);
	void SetInputFile(char *FileName);
	int StepToNextTime(int TimeInterval);

	InterruptFunction InterruptService;
};

#endif //__SIM_TOP_H__
