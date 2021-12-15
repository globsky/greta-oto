//----------------------------------------------------------------------
// AcqEngine.h:
//   Acquisition engine simulator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __ACQ_ENGINE_SIM_H__
#define __ACQ_ENGINE_SIM_H__

#include "CommonDefines.h"
#include "BasicTypes.h"
#include "LNavBit.h"
#include "XmlInterpreter.h"
#include "SatelliteParam.h"
#include "ComplexNumber.h"
#include "PeakSorter.h"

#define MF_DEPTH 682
#define MAX_CHANNEL 32
#define CHANNEL_CONFIG_LEN 8
#define DFT_NUMBER 8
#define NOISE_AMP_SQRT2 120.	// noise amplitude of 1ms correlation

struct AeBufferSatParam
{
	int svid;	// svid
	int BitLength;	// number of ms within a bit
	int MsCount;	// ms count of next code round within a bit
	double Time2CodeEnd;	// number of 1/2 chip to end of code round
	double Doppler;
	double Amplitude;
	int BitArray[32];	// modulation bits from next code round
};

class CAcqEngine
{
public:
	GNSS_TIME AcqTime;

	// global registers
	int ChannelNumber;		// 6bit
	int BufferThreshold;	// 7bit

	// channel search configuration registers
	int EarlyTerminate;		// 1bit
	int PeakRatioTh;		// 3bit
	int StrideNumber;		// 6bit
	int CoherentNumber;		// 6bit
	int NonCoherentNumber;	// 7bit
	int Svid;				// 6bit
	int PrnSelect;			// 2bit
	int CodeSpan;			// 5bit
	int ReadAddress;		// 5bit
	double CenterFreq;
	double DftTwiddlePhase;
	double StrideInterval;


	// rate adaptor registers, only for register read/write
	unsigned int RateCarrierFreq;	// 32bit
	unsigned int CodeRateAdjustRatio;	// 24bit
	unsigned int Threshold;	// 8bit

	CAcqEngine();
	~CAcqEngine();

	// interface functions for acquisition
	void Reset();
	void SetRegValue(int Address, U32 Value);
	U32 GetRegValue(int Address);
	void DoAcquisition();
	void SetBufferParam(PSATELLITE_PARAM SatelliteParam[], int SatVisible, GNSS_TIME Time, NavBit *NavData);

	static const double Bpsk2PeakValues[160];
	static const double Boc2PeakValues[160];

	// internal variables and functions
	double CarrierFreq;
	int Success;				// 1bit
	int CodeRoundCount;			// 5bit, counter for CodeSpan
	int StrideCount;	// 6bit unsigned
	int StrideOffset;			// 6bit signed
	unsigned int NoiseFloor;	// 18bit
	int CurMsCount, CurBitIndex;
	AeBufferSatParam SatParam[32];
	int SatNumber;	// number of valid satellites in SatParam
	complex_number CohResult[MF_DEPTH];
	complex_number DftResult[MF_DEPTH][DFT_NUMBER];
	double NoncohResult[MF_DEPTH][DFT_NUMBER];

	void NonCoherentAcc(int NoncohCount);
	void DoNonCoherentSum(AeBufferSatParam *pSatParam);
	void InsertPeak(double Amp, int PartialCorPos, int PartialFreq);
	int PeakFound();
	void SearchOneChannel(AeBufferSatParam *pSatParam);

	// internal RAM
	unsigned int ChannelConfig[MAX_CHANNEL][CHANNEL_CONFIG_LEN];

	CPeakSorter PeakSorter;
};

#endif //__ACQ_ENGINE_SIM_H__
