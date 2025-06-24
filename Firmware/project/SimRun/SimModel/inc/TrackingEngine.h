//----------------------------------------------------------------------
// TrackingEngine.h:
//   Tracking engine simulator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __TRACKING_ENGINE_SIM_H__
#define __TRACKING_ENGINE_SIM_H__

#include "CommonDefines.h"
#include "SignalSim.h"
#include "TrackingChannel.h"

#define LOGICAL_CHANNEL_NUMBER 32
#define TE_BUFFER_SIZE (LOGICAL_CHANNEL_NUMBER * 128)
#define COR_NUMBER 8
#define NOISE_AMP 625.

class CTrackingEngine
{
public:
	CTrackingEngine();
	~CTrackingEngine();
	void Reset();
	void SetRegValue(int Address, U32 Value);
	U32 GetRegValue(int Address);

	U32 ChannelEnable;				// 32bit
	U32 CohDataReady;				// 32bit
	U32 OverwriteProtectChannel;	// 32bit
	U32 OverwriteProtectAddr;		// 12bit
	U32 OverwriteProtectValue;		// 32bit
	U32 PrnPolyLength[4];			// 32bit;

	void SetTEBuffer(unsigned int Address, U32 Value);
	U32 GetTEBuffer(unsigned int Address);
	int ProcessData(int BlockSize, GNSS_TIME CurTime, PSATELLITE_PARAM SatParam[], int SatNumber);

	SATELLITE_PARAM* FindSatParam(int ChannelId, PSATELLITE_PARAM SatParam[], int SatNumber);

	unsigned int TEBuffer[TE_BUFFER_SIZE/4];
	CTrackingChannel LogicChannel[LOGICAL_CHANNEL_NUMBER];
	int SmoothScale;
	double NoiseFloor;
};

#endif //__TRACKING_ENGINE_SIM_H__
