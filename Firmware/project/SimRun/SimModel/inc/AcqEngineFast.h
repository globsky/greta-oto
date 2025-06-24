//----------------------------------------------------------------------
// AcqEngineFast.h:
//   Acquisition engine simulator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __ACQ_ENGINE_SIM_H__
#define __ACQ_ENGINE_SIM_H__

#include "CommonDefines.h"
#include "SignalSim.h"

#define MF_DEPTH 682
#define MAX_CHANNEL 32
#define CHANNEL_CONFIG_LEN 8
#define DFT_NUMBER 8
#define SIGMA0 156
#define LAMBDA_PARAM1 0.055422
#define LAMBDA_PARAM2 0.051488
#define LAMBDA_SLOPE 2.266

struct AeBufferSatParam
{
	int PrnSelect;
	int svid;	// svid
	int BitLength;	// number of ms within a bit
	int MsCount;	// ms count of buffer start within a bit
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
	void SetBufferParam(PSATELLITE_PARAM SatelliteParam[], int SatVisible, GNSS_TIME Time, NavBit *NavData[]);
	void AssignChannelParam(PSATELLITE_PARAM SatelliteParam, GNSS_TIME Time, NavBit *NavData, int PrnSelect, AeBufferSatParam *SatParam);

	static const double Bpsk2PeakValues[160];
	static const double Boc2PeakValues[160];
	static const double BasecPdfSegment[257];	// segment edge evenly divide CDF into 256 part
	static const double BasicPdfValues[257];	// corresponding possibility value at segment edge
	static const double BaseModeValues[20];		// mode value with different Noncoh
	static const double BaseVariance[20];		// variance with different Noncoh
	static const double ModeSlope[20];			// slope of mode square with different Noncoh
	static const double VarianceSlope[20];		// slope of variance with different Noncoh

	// internal variables and functions
	double CarrierFreq;
	int Success;				// 1bit
	int CurMsCount, CurBitIndex;
	int PhaseCount;
	AeBufferSatParam SatParam[32];
	int SatNumber;	// number of valid satellites in SatParam

	void SearchOneChannel(AeBufferSatParam *pSatParam, int Channel);
	void GetNoisePeaks(double NoisePeaks[3]);
	double GetSignalPeak(AeBufferSatParam *pSatParam, int &FreqBin, int &Cor);

	// internal RAM
	unsigned int ChannelConfig[MAX_CHANNEL][CHANNEL_CONFIG_LEN];
};

#endif //__ACQ_ENGINE_SIM_H__
