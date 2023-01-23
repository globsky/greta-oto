//----------------------------------------------------------------------
// TrackingChannel.h:
//   Logic tracking channel simulator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __TRACKING_CHANNEL_SIM_H__
#define __TRACKING_CHANNEL_SIM_H__

#include "CommonDefines.h"
#include "LNavBit.h"
#include "SatelliteParam.h"
#include "GaussNoise.h"

#define COR_NUMBER 8
#define NOISE_AMP 625.

enum SignalSystem { SignalL1CA = 0, SignalE1 = 1, SignalB1C = 2, SignalL1C = 3 };

struct CarrierState
{
	double GetFreqDiff(double SourceDoppler, double LocalDoppler, double Alpha);
	double GetPhaseDiff(double DeltaPhase, double Alpha);
	double PhaseAverage(double Phase1, double Ratio1, double Phase2, double Ratio2);

	double DopplerPrev2, DopplerPrev, DopplerCur;
	double LocalFreqPrev2, LocalFreqPrev;
	double DeltaPhiPrev2, DeltaPhiPrev, DeltaPhiCur;
};

class CTrackingChannel
{
public:
	CTrackingChannel();
	~CTrackingChannel();
	void Initial(GNSS_TIME CurTime, PSATELLITE_PARAM pSatParam, NavBit *pNavData);
	void SetChannelStates(unsigned int AddressOffset, U32 Value);
	U32 GetChannelStates(unsigned int AddressOffset);
	int FindSvid(unsigned int ConfigArray[], int ArraySize, U32 PrnConfig);
	void GetCorrelationResult(GNSS_TIME CurTime, SATELLITE_PARAM *pSatParam, int DumpDataI[], int DumpDataQ[], int CorIndex[], int CorPos[], int NHCode[], int DataLength);
	int CalculateCounter(int BlockSize, int CorIndex[], int CorPos[], int NHCode[], int &DataLength);
	void DecodeDataAcc(int DataValue);
	double NarrowCompensation(int CorIndex, int NarrowFactor);
	complex_number DataChannelSignal(SignalSystem SystemSel, double PeakAmp, double SignalAmp);
	complex_number PilotChannelSignal(SignalSystem SystemSel, double PeakAmp, double SignalAmp);

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
	NavBit *NavData;
	int CurrentFrame;	// frame number of data stream filling in Bits
	int CurrentBitIndex;	// bit index used for current ms correlation result
	int CurrentDataBit;		// modulation bit for current outputing correlation result (if not all correlator finished)
	int CurrentPilotBit;	// modulation bit for current outputing correlation result (if not all correlator finished)
	int CurrentNHCode;		// latest NH bit for pilot channel
	int DataBits[1800];
	int PilotBits[1800];
	// for carrier difference calculation
	CarrierState CarrierParam;

	// co-variance matrix to generate relative Gauss noise
	static double CovarMatrix[4][SUM_N(COR_NUMBER)];

	// correlation peak shape values
	static const double Bpsk4PeakValues[160];
	static const double Boc4PeakValues[160];
	static const double Boc2PeakValues[160];
};

#endif //__TRACKING_CHANNEL_SIM_H__
