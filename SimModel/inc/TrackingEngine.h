//----------------------------------------------------------------------
// TrackingEngine.cpp:
//   Tracking engine simulator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __TRACKING_ENGINE_SIM_H__
#define __TRACKING_ENGINE_SIM_H__

#include "CommonDefines.h"
#include "LNavBit.h"
#include "SatelliteParam.h"
#include "GaussNoise.h"

#define LOGICAL_CHANNEL_NUMBER 32
#define TE_BUFFER_SIZE (LOGICAL_CHANNEL_NUMBER * 128)
#define COR_NUMBER 8
#define NOISE_AMP 625.

enum SignalSystem { SignalL1CA = 0, SignalE1 = 1, SignalB1C = 2, SignalL1C = 3 };

struct ChannelConfig
{
	// config parameters
	double CarrierFreq;
	double CodeFreq;
	unsigned int CarrierFreqInt;
	unsigned int CodeFreqInt;
	int PreShiftBits;
	int PostShiftBits;
	int EnableBOC;
	int DataInQBranch;
	int EnableSecondPrn;
	int DecodeBit;
	int NarrowFactor;
	int BitLength;
	int CoherentNumber;
	unsigned int NHCode;
	int NHLength;
	int DumpLength;
	// translate from PRN config
	SignalSystem SystemSel;	// 0: GPS L1C/A, 1: Galileo E1, 2: BDS B1C, 3: GPS L1C
	int Svid;
	int PrnCount;
	// state parameters
	unsigned int CarrierPhase;
	unsigned int CarrierCount;
	unsigned int CodePhase;
	int JumpCount;
	int DumpCount;
	int CoherentDone;
	int OverwriteProtect;
	int CurrentCor;
	int Dumping;
	int CodeSubPhase;
	int BitCount;
	int CoherentCount;
	int NHCount;
	unsigned int DecodeData;
	// store Gauss noise
	complex_number GaussNoise[8];
	// variable to get modulation bit
	int CurrentFrame;	// frame number of data stream filling in Bits
	int CurrentBitIndex;	// bit index used for current ms correlation result
	int CurrentDataBit;		// modulation bit for current outputing correlation result (if not all correlator finished)
	int CurrentPilotBit;	// modulation bit for current outputing correlation result (if not all correlator finished)
	int CurrentNHCode;		// latest NH bit for pilot channel
	int DataBits[1800];
	int PilotBits[1800];
};

struct CarrierParameter
{
	double DopplerPrev2, DopplerPrev, DopplerCur;
	double LocalFreqPrev2, LocalFreqPrev;
	double DeltaPhiPrev2, DeltaPhiPrev, DeltaPhiCur;
};

class CTrackingEngine
{
public:
	CTrackingEngine();
	~CTrackingEngine();
	void Reset();
	void SetRegValue(int Address, U32 Value);
	U32 GetRegValue(int Address);
	void SetNavBit(NavBit *pNavBit[]) { NavData = pNavBit; }

	U32 ChannelEnable;				// 32bit
	U32 CohDataReady;				// 32bit
	U32 OverwriteProtectChannel;	// 32bit
	U32 OverwriteProtectAddr;		// 12bit
	U32 OverwriteProtectValue;		// 32bit
	U32 PrnPolyLength[4];			// 32bit;

	void SetTEBuffer(unsigned int Address, U32 Value);
	U32 GetTEBuffer(unsigned int Address);
	int ProcessData(int BlockSize, GNSS_TIME CurTime, PSATELLITE_PARAM SatParam[], int SatNumber);

	static const double Bpsk4PeakValues[160];
	static const double Boc4PeakValues[160];
	static const double Boc2PeakValues[160];

	int FindSvid(unsigned int ConfigArray[], int ArraySize, U32 PrnConfig);
	SATELLITE_PARAM* FindSatParam(int ChannelId, PSATELLITE_PARAM SatParam[], int SatNumber);
	void GetCorrelationResult(ChannelConfig *ChannelParam, CarrierParameter *CarrierParam, GNSS_TIME CurTime, SATELLITE_PARAM *pSatParam, int DumpDataI[], int DumpDataQ[], int CorIndex[], int CorPos[], int NHCode[], int DataLength);
	int CalculateCounter(int BlockSize, ChannelConfig *ChannelParam, int CorIndex[], int CorPos[], int NHCode[], int &DataLength);
	void DecodeDataAcc(ChannelConfig *ChannelParam, int DataValue);
	void InitChannel(int ChannelId, GNSS_TIME CurTime, PSATELLITE_PARAM SatParam[], int SatNumber);
	double NarrowCompensation(int CorIndex, int NarrowFactor);
	double PhaseAverage(double Phase1, double Ratio1, double Phase2, double Ratio2);

	unsigned int TEBuffer[TE_BUFFER_SIZE/4];
	NavBit **NavData;
	ChannelConfig ChannelParam[LOGICAL_CHANNEL_NUMBER];
	CarrierParameter CarrierParam[LOGICAL_CHANNEL_NUMBER];
	double CovarMatrix2[SUM_N(COR_NUMBER)], CovarMatrix4[SUM_N(COR_NUMBER)], CovarMatrix8[SUM_N(COR_NUMBER)];
	int SmoothScale;
	double NoiseFloor;
};

#endif //__TRACKING_ENGINE_SIM_H__
