//----------------------------------------------------------------------
// TrackingChannel.cpp:
//   Logic tracking channel simulator class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <memory.h>
#include <math.h>
#include "ConstVal.h"
#include "RegAddress.h"
#include "InitSet.h"
#include "ComplexNumber.h"
#include "GaussNoise.h"
#include "TrackingChannel.h"

const double CTrackingChannel::Bpsk4PeakValues[160] = {
  0.937500,  0.937256,  0.936523,  0.935303,  0.933594,  0.931396,  0.928711,  0.925537,  0.921875,  0.917725,
  0.913086,  0.907959,  0.902344,  0.896240,  0.889648,  0.882568,  0.875000,  0.867188,  0.859375,  0.851563,
  0.843750,  0.835938,  0.828125,  0.820313,  0.812500,  0.804688,  0.796875,  0.789062,  0.781250,  0.773438,
  0.765625,  0.757813,  0.750000,  0.742188,  0.734375,  0.726563,  0.718750,  0.710938,  0.703125,  0.695312,
  0.687500,  0.679688,  0.671875,  0.664063,  0.656250,  0.648438,  0.640625,  0.632813,  0.625000,  0.617188,
  0.609375,  0.601563,  0.593750,  0.585937,  0.578125,  0.570313,  0.562500,  0.554688,  0.546875,  0.539063,
  0.531250,  0.523438,  0.515625,  0.507813,  0.500000,  0.492187,  0.484375,  0.476563,  0.468750,  0.460937,
  0.453125,  0.445313,  0.437500,  0.429687,  0.421875,  0.414062,  0.406250,  0.398438,  0.390625,  0.382813,
  0.375000,  0.367187,  0.359375,  0.351563,  0.343750,  0.335937,  0.328125,  0.320312,  0.312500,  0.304687,
  0.296875,  0.289062,  0.281250,  0.273438,  0.265625,  0.257813,  0.250000,  0.242187,  0.234375,  0.226562,
  0.218750,  0.210937,  0.203125,  0.195312,  0.187500,  0.179687,  0.171875,  0.164062,  0.156250,  0.148437,
  0.140625,  0.132812,  0.125000,  0.117310,  0.109863,  0.102661,  0.095703,  0.088989,  0.082520,  0.076294,
  0.070312,  0.064575,  0.059082,  0.053833,  0.048828,  0.044067,  0.039551,  0.035278,  0.031250,  0.027466,
  0.023926,  0.020630,  0.017578,  0.014771,  0.012207,  0.009888,  0.007812,  0.005981,  0.004395,  0.003052,
  0.001953,  0.001099,  0.000488,  0.000122, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000,
 -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,
};

const double CTrackingChannel::Boc4PeakValues[160] = {
  0.812500,  0.811768,  0.809570,  0.805908,  0.800781,  0.794189,  0.786133,  0.776611,  0.765625,  0.753174,
  0.739258,  0.723877,  0.707031,  0.688721,  0.668945,  0.647705,  0.625000,  0.601563,  0.578125,  0.554688,
  0.531250,  0.507813,  0.484375,  0.460938,  0.437500,  0.414063,  0.390625,  0.367188,  0.343750,  0.320313,
  0.296875,  0.273438,  0.250000,  0.226562,  0.203125,  0.179688,  0.156250,  0.132812,  0.109375,  0.085938,
  0.062500,  0.039062,  0.015625, -0.007813, -0.031250, -0.054688, -0.078125, -0.101563, -0.125000, -0.147949,
 -0.169922, -0.190918, -0.210937, -0.229980, -0.248047, -0.265137, -0.281250, -0.296387, -0.310547, -0.323730,
 -0.335938, -0.347168, -0.357422, -0.366699, -0.375000, -0.382324, -0.388672, -0.394043, -0.398438, -0.401855,
 -0.404297, -0.405762, -0.406250, -0.405762, -0.404297, -0.401855, -0.398438, -0.394043, -0.388672, -0.382324,
 -0.375000, -0.367188, -0.359375, -0.351563, -0.343750, -0.335938, -0.328125, -0.320313, -0.312500, -0.304688,
 -0.296875, -0.289063, -0.281250, -0.273438, -0.265625, -0.257813, -0.250000, -0.242187, -0.234375, -0.226563,
 -0.218750, -0.210938, -0.203125, -0.195313, -0.187500, -0.179688, -0.171875, -0.164063, -0.156250, -0.148438,
 -0.140625, -0.132813, -0.125000, -0.117310, -0.109863, -0.102661, -0.095703, -0.088989, -0.082520, -0.076294,
 -0.070313, -0.064575, -0.059082, -0.053833, -0.048828, -0.044067, -0.039551, -0.035278, -0.031250, -0.027466,
 -0.023926, -0.020630, -0.017578, -0.014771, -0.012207, -0.009888, -0.007812, -0.005981, -0.004395, -0.003052,
 -0.001953, -0.001099, -0.000488, -0.000122, -0.000000, -0.000000, -0.000000,  0.000000, -0.000000, -0.000000,
  0.000000,  0.000000,  0.000000, -0.000000,  0.000000,  0.000000, -0.000000, -0.000000,  0.000000,  0.000000,
};

const double CTrackingChannel::Boc2PeakValues[160] = {
  0.588997,  0.588857,  0.588438,  0.587739,  0.586761,  0.585506,  0.583974,  0.582168,  0.580088,  0.577737,
  0.575118,  0.572233,  0.569085,  0.565677,  0.562013,  0.558096,  0.553932,  0.549524,  0.544877,  0.539997,
  0.534888,  0.529558,  0.524011,  0.518256,  0.512298,  0.506145,  0.499806,  0.493288,  0.486601,  0.479753,
  0.472756,  0.465618,  0.458351,  0.450968,  0.443480,  0.435900,  0.428242,  0.420522,  0.412754,  0.404955,
  0.397143,  0.389336,  0.381555,  0.373819,  0.366151,  0.358574,  0.351114,  0.343796,  0.336647,  0.329914,
  0.323820,  0.318358,  0.313518,  0.309282,  0.305625,  0.302520,  0.299932,  0.297823,  0.296150,  0.294869,
  0.293931,  0.293288,  0.292890,  0.292687,  0.292632,  0.292676,  0.292772,  0.292878,  0.292951,  0.292952,
  0.292844,  0.292592,  0.292164,  0.291533,  0.290670,  0.289553,  0.288160,  0.286472,  0.284473,  0.282149,
  0.279490,  0.276574,  0.273493,  0.270247,  0.266839,  0.263271,  0.259546,  0.255664,  0.251630,  0.247445,
  0.243112,  0.238633,  0.234012,  0.229250,  0.224352,  0.219320,  0.214157,  0.208867,  0.203452,  0.197916,
  0.192263,  0.186496,  0.180619,  0.174635,  0.168549,  0.162364,  0.156085,  0.149715,  0.143260,  0.136723,
  0.130108,  0.123422,  0.116669,  0.109960,  0.103407,  0.097015,  0.090789,  0.084734,  0.078856,  0.073158,
  0.067646,  0.062323,  0.057195,  0.052264,  0.047536,  0.043014,  0.038702,  0.034602,  0.030719,  0.027056,
  0.023614,  0.020398,  0.017410,  0.014652,  0.012126,  0.009835,  0.007779,  0.005962,  0.004384,  0.003047,
  0.001951,  0.001098,  0.000488,  0.000122,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,
  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,
};

CTrackingChannel::CTrackingChannel()
{
}

CTrackingChannel::~CTrackingChannel()
{
}

void CTrackingChannel::Initial(GNSS_TIME CurTime, PSATELLITE_PARAM pSatParam, NavBit *pNavData)
{
	GNSS_TIME TransmitTime;
	int FrameLength = (SystemSel == SignalL1CA) ? 6000 : ((SystemSel == SignalE1) ? 2000 : 18000);

	if (SystemSel == SignalB1C)
		CurTime.MilliSeconds -= 14000;	// compensate BDS leap second difference

	// initial structure to calculate FreqDiff and PhaseDiff
	CarrierParam.DopplerPrev2 = CarrierParam.DopplerPrev = CarrierParam.DopplerCur = GetDoppler(pSatParam, 0);
	CarrierParam.LocalFreqPrev2 = CarrierParam.LocalFreqPrev = CarrierFreq - (((SystemSel == SignalL1CA) || EnableBOC) ? IF_FREQ : (IF_FREQ + 1023000));
	// initial modulation bit and counter
	TransmitTime = GetTransmitTime(CurTime, GetTravelTime(pSatParam, 0));
	TransmitTime.MilliSeconds ++;	// correlation result will be calculated from next millisecond
	CurrentFrame = TransmitTime.MilliSeconds / FrameLength;	// frame number
	CurrentDataBit = CurrentPilotBit = 0;
	NavData = pNavData;
	NavData->GetFrameData(TransmitTime, Svid, 0, PilotBits);
	if (SystemSel == SignalL1CA)
		memcpy(DataBits, PilotBits, sizeof(int) * 300);	// GPS L1 uses same bit stream for data and pilot
	else
		NavData->GetFrameData(TransmitTime, Svid, 1, DataBits);
}

// on set value of channel state buffer, interprete channel configuration parameters
void CTrackingChannel::SetChannelStates(unsigned int AddressOffset, U32 Value)
{
	switch (AddressOffset)
	{
	case STATE_OFFSET_CARRIER_FREQ:
		CarrierFreq = Value / 4294967296. * SAMPLE_FREQ;
		CarrierFreqInt = Value;
		break;
	case STATE_OFFSET_CODE_FREQ:
		CodeFreq = Value / 4294967296. * SAMPLE_FREQ;
		CodeFreqInt = Value;
		break;
	case STATE_OFFSET_CORR_CONFIG:
		PreShiftBits = EXTRACT_UINT(Value, 0, 2);
		PostShiftBits = EXTRACT_UINT(Value, 2, 2);
		DataInQBranch = EXTRACT_UINT(Value, 5, 1);
		EnableSecondPrn = EXTRACT_UINT(Value, 6, 1);
		EnableBOC = EXTRACT_UINT(Value, 7, 1);
		DecodeBit = EXTRACT_UINT(Value, 8, 2);
		NarrowFactor = EXTRACT_UINT(Value, 10, 2);
		BitLength = EXTRACT_UINT(Value, 16, 5);
		CoherentNumber = EXTRACT_UINT(Value, 21, 6);
		break;
	case STATE_OFFSET_NH_CONFIG:
		NHCode = EXTRACT_UINT(Value, 0, 25);
		NHLength = EXTRACT_UINT(Value, 27, 5);
		break;
	case STATE_OFFSET_DUMP_LENGTH:
		DumpLength = EXTRACT_UINT(Value, 0, 16);
		break;
	case STATE_OFFSET_PRN_CONFIG:
		if ((Value & 0xf0000000) == 0)	// L1C/A
		{
			SystemSel = SignalL1CA;
			if ((Svid = FindSvid(CAPrnInit, 32, Value)) == 0)	// not found in L1C/A initial array
			{
				if ((Svid = FindSvid(WaasPrnInit, 32, Value)) != 0)	// this is SBAS, continue from 33
					Svid += 32;
			}
		}
		else if ((Value & 0xf0000000) == 0xc0000000)	// E1
		{
			SystemSel = SignalE1;
			Svid = ((Value >> 6) & 0x7f) - 49;
			if (Svid < 1 || Svid > 50)
				Svid = 0;	// only valid for E1C code range
		}
		else if ((Value & 0xf0000000) == 0x80000000)	// B1C
		{
			SystemSel = SignalB1C;
			Svid = FindSvid(B1CPilotInit, 63, Value);	// assume primary PRN code uses pilot code
		}
		else if ((Value & 0xf0000000) == 0xa0000000)	// L1C
		{
			SystemSel = SignalL1C;
			Svid = FindSvid(L1CPilotInit, 63, Value);	// assume primary PRN code uses pilot code
		}
		break;
	case STATE_OFFSET_PRN_COUNT:
		if (SystemSel == SignalL1CA)	// L1C/A
			PrnCount = (Value >> 14);
		else if (SystemSel == SignalE1)	// E1
			PrnCount = Value - (Value >> 10);
		else	// B1C or L1C
			PrnCount = Value & 0x3fff;
		break;
	case STATE_OFFSET_CARRIER_PHASE:
		CarrierPhase = Value;
		break;
	case STATE_OFFSET_CARRIER_COUNT:
		CarrierCount = Value;
		break;
	case STATE_OFFSET_CODE_PHASE:
		CodePhase = Value;
		break;
	case STATE_OFFSET_PRN_CODE:
		JumpCount = EXTRACT_INT(Value, 8, 8);
		DumpCount = EXTRACT_UINT(Value, 16, 16);
		break;
	case STATE_OFFSET_CORR_STATE:
		CurrentCor = EXTRACT_UINT(Value, 4, 3);
		Dumping = EXTRACT_UINT(Value, 7, 1);
		CodeSubPhase = EXTRACT_UINT(Value, 8, 1);
		BitCount = EXTRACT_UINT(Value, 16, 5);
		CoherentCount = EXTRACT_UINT(Value, 21, 6);
		NHCount = EXTRACT_UINT(Value, 27, 5);
	case STATE_OFFSET_DECODE_DATA:
		DecodeData = Value;
		break;
	}
}

U32 CTrackingChannel::GetChannelStates(unsigned int AddressOffset)
{
	switch (AddressOffset)
	{
	case STATE_OFFSET_PRN_COUNT:
		if (SystemSel == SignalL1CA)	// L1C/A
			return PrnCount << 14;
		else if (SystemSel == SignalE1)	// E1
			return ((PrnCount / 1023) << 10) | (PrnCount % 1023);
		else	// B1C or L1C
			return PrnCount;
		break;
	case STATE_OFFSET_CARRIER_PHASE:
		return CarrierPhase;
		break;
	case STATE_OFFSET_CARRIER_COUNT:
		return CarrierCount;
		break;
	case STATE_OFFSET_CODE_PHASE:
		return CodePhase;
		break;
	case STATE_OFFSET_PRN_CODE:
		return DumpCount << 16;
		break;
	case STATE_OFFSET_CORR_STATE:
		return (NHCount << 27) | (CoherentCount << 21) | (BitCount << 16) | (CodeSubPhase << 8) | (((CurrentCor == 0) ? 0 : 1) << 7) | (CurrentCor << 4) | (OverwriteProtect << 1) | CoherentDone;
	case STATE_OFFSET_DECODE_DATA:
		return DecodeData;
	default:
		return 0;
	}
}

int CTrackingChannel::FindSvid(unsigned int ConfigArray[], int ArraySize, U32 PrnConfig)
{
	int i;

	for (i = 0; i < ArraySize; i ++)
		if (PrnConfig == ConfigArray[i])
			return (i + 1);
	return 0;	// not found
}

void CTrackingChannel::GetCorrelationResult(GNSS_TIME CurTime, SATELLITE_PARAM *pSatParam, int DumpDataI[], int DumpDataQ[], int CorIndex[], int CorPos[], int NHCode[], int DataLength)
{
	int i;
	double Alpha;
	complex_number CorResult[16];
	complex_number Signal, Rotate;
	double CodeDiff[16];
	int CorCount = 0, StartIndex = 0;
	int NominalIF;
	double FreqDiff, PhaseDiff;
	double SourceCarrierPhase;
	double Amplitude, PeakPosition, NcoPhase, CorPosition, AmpRatio;
	GNSS_TIME TransmitTime;
	int CodeDiffIndex, CodeLength, FrameLength, BitLength;
	int FrameNumber, BitNumber, Milliseconds, DataBit, PilotBit;
	const double *PeakValues;

	// calculate half of 128x code length
	CodeLength = (SystemSel == SignalL1CA) ? 1023 : ((SystemSel == SignalE1) ? 4092 : 10230);
	CodeLength *= 64;
	FrameLength = (SystemSel == SignalL1CA) ? 6000 : ((SystemSel == SignalE1) ? 2000 : 18000);
	BitLength = (SystemSel == SignalL1CA) ? 20 : ((SystemSel == SignalE1) ? 4 : 10);

	if (SystemSel == SignalB1C)
		CurTime.MilliSeconds -= 14000;	// compensate BDS leap second difference

	// first generate relative noise
	while (CorCount < DataLength)
	{
		if ((CorIndex[CorCount] >> 2) == 0)	// new round of correlator 0~7, generate 8 Gauss Noise
			GenerateRelativeNoise(8, (2 << NarrowFactor), CovarMatrix[NarrowFactor], NOISE_AMP, GaussNoise);
		CorResult[CorCount ++] = GaussNoise[CorIndex[CorCount] >> 2];
	}
//	memset(CorResult, 0, sizeof(CorResult));
	if (pSatParam)
	{
		PeakValues = (SystemSel == SignalL1CA) ? Bpsk4PeakValues : (EnableBOC ? Boc4PeakValues : Boc2PeakValues);
		Alpha = ((double)CorPos[0] / DumpLength / 2);	// CorPos has unit of 1/2 chip
		Alpha -= (int)Alpha;	// modulo to 1ms
		Alpha = 1 - Alpha;

		// calculate frequency difference
		NominalIF = ((SystemSel == SignalL1CA) || EnableBOC) ? IF_FREQ : (IF_FREQ + 1023000);
		FreqDiff = CarrierParam.GetFreqDiff(GetDoppler(pSatParam, 0), CarrierFreq - NominalIF, Alpha) / 1000. * PI;

		// calculate carrier phase of source signal
		SourceCarrierPhase = GetCarrierPhase(pSatParam, 0);
		SourceCarrierPhase -= (int)SourceCarrierPhase;
		SourceCarrierPhase = 1 - SourceCarrierPhase;	// carrier is fractional part of negative of travel time, equvalent to 1 minus positive fractional part
		// calculate phase difference
		PhaseDiff = CarrierParam.GetPhaseDiff(SourceCarrierPhase - CarrierPhase / 4294967296., Alpha);
		Rotate = complex_number(cos(PhaseDiff * PI2), sin(PhaseDiff * PI2));
//		printf(" %.5f\n", PhaseDiff);

		// calculate signal amplitude
		Amplitude = 1.4142135 * pow(10, (pSatParam->CN0 - 3000) / 2000.) * NOISE_AMP;
		Amplitude *= (fabs(FreqDiff) > 1e-3) ? (sin(FreqDiff) / FreqDiff) : 1.0;
		TransmitTime = GetTransmitTime(CurTime, GetTravelTime(pSatParam, 0) + 0.001);	// add one extra millisecond to get previous finished millisecond
		FrameNumber = TransmitTime.MilliSeconds / FrameLength;
		BitNumber = (TransmitTime.MilliSeconds % FrameLength) / BitLength;
		Milliseconds = TransmitTime.MilliSeconds % BitLength;
		if (FrameNumber != CurrentFrame)
		{
			NavData->GetFrameData(TransmitTime, Svid, 0, PilotBits);
			if (SystemSel == SignalL1CA)
				memcpy(DataBits, PilotBits, sizeof(int) * 300);	// GPS L1 uses same bit stream for data and pilot
			else
				NavData->GetFrameData(TransmitTime, Svid, 1, DataBits);
			CurrentFrame = FrameNumber;
		}
		PeakPosition = (((SystemSel == SignalL1CA) ? 0 : Milliseconds) + TransmitTime.SubMilliSeconds) * 2046;
		NcoPhase = (double)CodePhase / 4294967296.;
		// calculate code difference for each correlator and add narrow correlator compensation (with unit of 1/64 correlator interval or 1/128 chip)
		for (i = 0; i < DataLength; i ++)
		{
			CorPosition = CorPos[i] + NcoPhase;
			if (NarrowFactor)
				CorPosition += NarrowCompensation(CorIndex[i] >> 2, NarrowFactor);
			CodeDiff[i] = fabs(CorPosition - PeakPosition) * 64;
		}
		// calculate 1ms correlation value
		DataBit = CurrentDataBit;	// if current correlator dump not finished, use CurrentDataBit
		PilotBit = CurrentPilotBit;	// if current correlator dump not finished, use CurrentPilotBit
		for (i = 0; i < DataLength; i ++)
		{
			if ((CorIndex[i] >> 2) == 0)	// new correlator dump uses new bit
			{
				CurrentDataBit  = DataBit  = DataBits[BitNumber];
				CurrentPilotBit = PilotBit = PilotBits[BitNumber];
				BitNumber += (Milliseconds + 1) / BitLength;	// if there will be next Cor0, check whether next milliseconds move to next bit
			}
			if (CodeDiff[i] > CodeLength)	// CodeDiff may be one whole code round difference
				CodeDiff[i] = fabs(CodeDiff[i] - (CodeLength * 2));
			CodeDiffIndex = (int)CodeDiff[i];	// integer part of code difference
			CodeDiff[i] -= CodeDiffIndex;	// factional part of code difference
			if (CodeDiffIndex < 159)
			{
				AmpRatio = PeakValues[CodeDiffIndex];
				AmpRatio += (PeakValues[CodeDiffIndex+1] - PeakValues[CodeDiffIndex]) * CodeDiff[i];
				if (((CorIndex[i] >> 2) == 0) && EnableSecondPrn)
					Signal = DataChannelSignal(SystemSel, DataBit ? -AmpRatio : AmpRatio, Amplitude);
				else
					Signal = PilotChannelSignal(SystemSel, (PilotBit ^ NHCode[i]) ? -AmpRatio : AmpRatio, Amplitude);
				CorResult[i] += Signal * Rotate;
//				if ((CorIndex[i] >> 2) == 4)
//					printf("Cor=%f %f %f\n", Amplitude * AmpRatio, CorResult[i].real, CorResult[i].imag);
			}
		}
	}
	for (i = 0; i < DataLength; i ++)
	{
		DumpDataI[i] = ((int)CorResult[i].real) >> (PreShiftBits + PostShiftBits);
		DumpDataQ[i] = ((int)CorResult[i].imag) >> (PreShiftBits + PostShiftBits);
	}
}

int CTrackingChannel::CalculateCounter(int BlockSize, int CorIndex[], int CorPos[], int NHBit[], int &DataLength)
{
	int i;
	U64 Count;
	int OverflowCount;
	int PrnCount2x;	// 2 times of PrnCount
	int DumpLength2x = DumpLength * 2;	// 2 times of DumpLength
	int CodeLength = (SystemSel == SignalL1CA) ? 1023 : ((SystemSel == SignalE1) ? 4092 : 10230);
	int DumpIncrease, CodeRound, DumpRemnant, CorPosition, CorPositionPrev;
	int NewCoh, DumpRound;
	int OverwriteProtect = 0;
	int FirstCor = -1;

	CoherentDone = 0;
	OverwriteProtect = 0;

	// carrier related counter
	Count = (((U64)CarrierCount) << 32) | CarrierPhase;
	Count += (U64)CarrierFreqInt * BlockSize;
	CarrierCount = (unsigned int)(Count >> 32);	// upper 32bit
	CarrierPhase = (unsigned int)(Count & 0xffffffff);	// lower 32bit
//	printf("Local=%d %f\n", CarrierCount, CarrierPhase / 4294967296.);

	// number of code NCO overflow
	Count = (U64)CodePhase + (U64)CodeFreqInt * BlockSize;
	OverflowCount = (int)(Count >> 32);
	OverflowCount += JumpCount;
	JumpCount = 0;	// reset JumpCount after each round

	// determine remnant correlator to dump before increase DumpCount
	DataLength = 0;
	DumpRemnant = DumpCount * 2 + CodeSubPhase;	// previous remnant 1/2 chip after dumping
	CorPosition = PrnCount * 2 + CodeSubPhase;	// code position with unit of 1/2 chip
	if (DumpRemnant < 7)	// need to complete remaining correlation result
	{
		CorPositionPrev = CorPosition - DumpLength2x;	// start code position of previous finished dump block
		if (CorPositionPrev < 0)
			CorPositionPrev += CodeLength * 2;
		FirstCor = DumpRemnant + 1;
		for (i = DumpRemnant + 1; i < 8; i ++)
		{
			CorPos[DataLength] = CorPosition - i;
			CorIndex[DataLength] = (i << 2) + (CoherentCount == 0 ? 1 : 0);
			NHBit[DataLength] = CurrentNHCode;
			DataLength ++;
		}
		CoherentDone |= (CoherentCount == CoherentNumber - 1) ? 1 : 0;
		CoherentCount ++;
		if (CoherentNumber)
			CoherentCount %= CoherentNumber;
	}

	// recalculate CodePhase, CodeSubPhase and PrnCount
	CodePhase = (unsigned int)(Count & 0xffffffff);	// lower 32bit
	PrnCount2x = PrnCount * 2 + CodeSubPhase;
	PrnCount2x += OverflowCount;
	DumpIncrease = PrnCount2x / 2 - PrnCount;
	CodeSubPhase = PrnCount2x & 1;
	PrnCount = PrnCount2x / 2;
	CodeRound = (PrnCount < CodeLength) ? 0 : (PrnCount - CodeLength) / DumpLength + 1; // 0: no next code round finish, 1: code finish on first dump, 2: code finish on second dump
	PrnCount %= CodeLength;

	// DumpCount and NHCount align with first correlator
	DumpCount += DumpIncrease;
	NewCoh = DumpCount / DumpLength;
	DumpCount %= DumpLength;

	// determine remnant correlator to dump after increase DumpCount
	DumpRemnant = DumpCount * 2 + CodeSubPhase;	// previous remnant 1/2 chip after dumping
	for (DumpRound = 1; DumpRound <= NewCoh; DumpRound ++)
	{
		for (i = 0; i < 8; i ++)
		{
			if (i > DumpRemnant && DumpRound == NewCoh)
				break;
			if (FirstCor == i && CoherentCount == 0)
				OverwriteProtect = 2;
			CorPos[DataLength] = ((i == 0) && EnableSecondPrn) ? CorPosition - 4 : CorPosition - i;		// if Cor0 uses secondary PRN, has extra 4 1/2 chip delay
			CorIndex[DataLength] = (i << 2) + OverwriteProtect;
			if (BitLength && EnableSecondPrn && (i == 0))	// do data decode
			{
				CorIndex[DataLength] |= (BitCount == 0 ? 1 : 0);
				if (++BitCount == BitLength)
					BitCount = 0;
			}
			else
				CorIndex[DataLength] |= (CoherentCount == 0 ? 1 : 0);
			// first correlator uses NH code at NHCount position, others uses the same as previous correlator
			NHBit[DataLength] = (i == 0) ? (NHLength ? ((NHCode & (1 << NHCount)) ? 1 : 0) : 0) : NHBit[DataLength-1];
			DataLength ++;
			CoherentDone |= (CoherentCount == CoherentNumber - 1) ? 1 : 0;
			if (NHLength && (i == 0) && DumpRound == CodeRound)	// NHCount increase with firse correlator
			{
				NHCount ++;
				NHCount %= NHLength;
			}
		}
		if (FirstCor < 0)
			FirstCor = 0;
		if (i == 8)	// last correlator dumped
		{
			CoherentCount ++;
			if (CoherentNumber)
				CoherentCount %= CoherentNumber;
		}
	}
	// save lastest used NH code
	CurrentNHCode = NHBit[DataLength-1];

//	if (DataLength > 8)
//		DataLength = DataLength;

	return CoherentDone;
}

void CTrackingChannel::DecodeDataAcc(int DataValue)
{
	int BitSelect;

	BitSelect = (BitLength >= 16) ? 2 : ((BitLength >= 8) ? 1 : 0);
	BitSelect = PreShiftBits + PostShiftBits - BitSelect;
	if (BitSelect < 0)
		BitSelect = 0;
	switch (DecodeBit)
	{
	case 0:	// 1bit
		DecodeData = (DecodeData << 1) | ((DataValue & 0x80000000) ? 1 : 0);
		break;
	case 1:	// 2bit
		DataValue >>= (13 - BitSelect);
		DataValue = (DataValue >> 1) + (DataValue & 1);	// round
		if (DataValue > 1)
			DataValue  = 1;
		else if (DataValue < -2)
			DataValue = -2;
		DecodeData = (DecodeData << 2) | (DataValue & 0x3);
		break;
	case 2:	// 4bit
		DataValue >>= (11 - BitSelect);
		DataValue = (DataValue >> 1) + (DataValue & 1);	// round
		if (DataValue > 7)
			DataValue  = 7;
		else if (DataValue < -8)
			DataValue = -8;
		DecodeData = (DecodeData << 4) | (DataValue & 0xf);
		break;
	case 3:	// 8bit
		DataValue >>= (8 - BitSelect);
		if (DataValue > 127)
			DataValue  = 127;
		else if (DataValue < -128)
			DataValue = -128;
		DecodeData = (DecodeData << 8) | (DataValue & 0xff);
		break;
	}
}

double CTrackingChannel::NarrowCompensation(int CorIndex, int NarrowFactor)
{
	if (CorIndex < 2 || CorIndex > 6 || NarrowFactor > 2)
		return 0.0;
	return (CorIndex - 4) * ((NarrowFactor == 1) ? 0.5 : 0.75);
}

complex_number CTrackingChannel::DataChannelSignal(SignalSystem SystemSel, double PeakAmp, double SignalAmp)
{
	if (SystemSel == SignalE1)	// E1B, -sqrt(1/2) in amplitude
		PeakAmp *= -0.7071067811865475244;
	else if (SystemSel >= SignalB1C)	// B1C/L1C, 1/2 in amplitude
		PeakAmp *= 0.5;
	if (SystemSel == SignalB1C)	// B1C lag PI/2 phase to pilot
		return complex_number(0, -SignalAmp * PeakAmp);
	else
		return complex_number(SignalAmp * PeakAmp, 0);
}

complex_number CTrackingChannel::PilotChannelSignal(SignalSystem SystemSel, double PeakAmp, double SignalAmp)
{
	if (SystemSel == SignalE1)	// E1C BOC(1,1), sqrt(5/11) in amplitude
		PeakAmp *= 0.6741998624632421;
	else if (SystemSel >= SignalB1C)	// B1C/L1C, sqrt(29/44) in amplitude
		PeakAmp *= 0.811844140885988713377;
	return complex_number(SignalAmp * PeakAmp, 0);
}

double CarrierState::GetFreqDiff(double SourceDoppler, double LocalDoppler, double Alpha)
{
	double Doppler1, Doppler2;

	DopplerPrev2 = DopplerPrev; DopplerPrev = DopplerCur; DopplerCur = SourceDoppler;
	LocalFreqPrev2 = LocalFreqPrev; LocalFreqPrev = LocalDoppler;

	Doppler1 = ((1 - Alpha) * DopplerPrev2 + (1 + Alpha) * DopplerPrev) / 2;
	Doppler2 = ((2 - Alpha) * DopplerPrev + Alpha * DopplerCur) / 2;
	return (1 - Alpha) * (Doppler1 - LocalFreqPrev2) + Alpha * (Doppler2 - LocalFreqPrev);
}

double CarrierState::GetPhaseDiff(double DeltaPhase, double Alpha)
{
	double DeltaPhi1, DeltaPhi2;

	DeltaPhiPrev2 = DeltaPhiPrev; DeltaPhiPrev = DeltaPhiCur; DeltaPhiCur = DeltaPhase;

	DeltaPhi1 = PhaseAverage(DeltaPhiPrev2, 1 - Alpha, DeltaPhiPrev, 1 + Alpha) / 2;
	DeltaPhi2 = PhaseAverage(DeltaPhiPrev, 2 - Alpha, DeltaPhiCur, Alpha) / 2;
	return PhaseAverage(DeltaPhi1, 1 - Alpha, DeltaPhi2, Alpha);
}

double CarrierState::PhaseAverage(double Phase1, double Ratio1, double Phase2, double Ratio2)
{
	// the average phase is Phase1*Ratio1 + Phase2*Ratio2
	// assume difference of Phase1 to Phase2 will not be larger than half cycle
	if (Phase2 < Phase1 - 0.5)
		Phase2 += 1.0;
	else if (Phase2 > Phase1 + 0.5)
		Phase2 -= 1.0;
	return Phase1 * Ratio1 + Phase2 * Ratio2;
}
