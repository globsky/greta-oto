//----------------------------------------------------------------------
// AcqEngine.cpp:
//   Acquisition engine simulator class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <string.h>
#include <intrin.h>

#include "ConstVal.h"
#include "RegAddress.h"
#include "AcqEngine.h"
#include "SatelliteParam.h"
#include "Coordinate.h"
#include "CommonDefines.h"
#include "ComplexNumber.h"
#include "GaussNoise.h"

const double CAcqEngine::Bpsk2PeakValues[160] = {
  0.875000,  0.874878,  0.874512,  0.873901,  0.873047,  0.871948,  0.870605,  0.869019,  0.867188,  0.865112,
  0.862793,  0.860229,  0.857422,  0.854370,  0.851074,  0.847534,  0.843750,  0.839722,  0.835449,  0.830933,
  0.826172,  0.821167,  0.815918,  0.810425,  0.804687,  0.798706,  0.792480,  0.786011,  0.779297,  0.772339,
  0.765137,  0.757690,  0.750000,  0.742188,  0.734375,  0.726563,  0.718750,  0.710938,  0.703125,  0.695313,
  0.687500,  0.679688,  0.671875,  0.664063,  0.656250,  0.648438,  0.640625,  0.632813,  0.625000,  0.617188,
  0.609375,  0.601563,  0.593750,  0.585938,  0.578125,  0.570313,  0.562500,  0.554688,  0.546875,  0.539063,
  0.531250,  0.523438,  0.515625,  0.507813,  0.500000,  0.492188,  0.484375,  0.476563,  0.468750,  0.460937,
  0.453125,  0.445312,  0.437500,  0.429688,  0.421875,  0.414063,  0.406250,  0.398438,  0.390625,  0.382813,
  0.375000,  0.367188,  0.359375,  0.351563,  0.343750,  0.335937,  0.328125,  0.320313,  0.312500,  0.304687,
  0.296875,  0.289063,  0.281250,  0.273438,  0.265625,  0.257813,  0.250000,  0.242249,  0.234619,  0.227112,
  0.219727,  0.212463,  0.205322,  0.198303,  0.191406,  0.184631,  0.177979,  0.171448,  0.165039,  0.158752,
  0.152588,  0.146545,  0.140625,  0.134827,  0.129150,  0.123596,  0.118164,  0.112854,  0.107666,  0.102600,
  0.097656,  0.092834,  0.088135,  0.083557,  0.079102,  0.074768,  0.070557,  0.066467,  0.062500,  0.058655,
  0.054932,  0.051331,  0.047852,  0.044495,  0.041260,  0.038147,  0.035156,  0.032288,  0.029541,  0.026917,
  0.024414,  0.022034,  0.019775,  0.017639,  0.015625,  0.013733,  0.011963,  0.010315,  0.008789,  0.007385,
  0.006104,  0.004944,  0.003906,  0.002991,  0.002197,  0.001526,  0.000977,  0.000549,  0.000244,  0.000061,
};

const double CAcqEngine::Boc2PeakValues[160] = {
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

CAcqEngine::CAcqEngine()
{
	Reset();
	memset(ChannelConfig, 0, sizeof(ChannelConfig));
}

CAcqEngine::~CAcqEngine()
{
}

void CAcqEngine::Reset()
{
	EarlyTerminate = 0;
	PeakRatioTh = 3;
}

void CAcqEngine::SetRegValue(int Address, U32 Value)
{
	Address &= 0xff;

	switch (Address)
	{
	case ADDR_OFFSET_AE_CONTROL:
		ChannelNumber = (int)EXTRACT_UINT(Value, 0, 6);
		if (Value & 0x100)
			DoAcquisition();
		break;
	case ADDR_OFFSET_AE_BUFFER_CONTROL:
		BufferThreshold = (int)EXTRACT_UINT(Value, 0, 7);
		break;
	case ADDR_OFFSET_AE_CARRIER_FREQ:
		RateCarrierFreq = Value;
		break;
	case ADDR_OFFSET_AE_CODE_RATIO:
		CodeRateAdjustRatio = EXTRACT_UINT(Value, 0, 24);
		break;
	case ADDR_OFFSET_AE_THRESHOLD:
		Threshold = EXTRACT_UINT(Value, 0, 8);
		break;
	default:
		break;
	}
}

U32 CAcqEngine::GetRegValue(int Address)
{
	Address &= 0xff;

	switch (Address)
	{
	case ADDR_OFFSET_AE_CONTROL:
		return 	ChannelNumber;
	case ADDR_OFFSET_AE_BUFFER_CONTROL:
		return BufferThreshold;
	case ADDR_OFFSET_AE_STATUS:
		return 	0;
	case ADDR_OFFSET_AE_CARRIER_FREQ:
		return 	RateCarrierFreq;
	case ADDR_OFFSET_AE_CODE_RATIO:
		return 	CodeRateAdjustRatio;
	case ADDR_OFFSET_AE_THRESHOLD:
		return 	Threshold;
	default:
		return 0;
	}
}

void CAcqEngine::NonCoherentAcc(int NoncohCount)
{
	int CorCount, FreqCount, MaxFreq;
	double Amplitude, AmplitudeAcc, MaxAmp;

	NoiseFloor = 0;
	// non-coherent acc
	for (CorCount = 0; CorCount < MF_DEPTH; CorCount ++)
	{
		MaxAmp = AmplitudeAcc = 0.;
		MaxFreq = 0;
		for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
		{
			AmplitudeAcc += (Amplitude = DftResult[CorCount][FreqCount].abs());
			NoncohResult[CorCount][FreqCount] += Amplitude;
			if (MaxAmp < NoncohResult[CorCount][FreqCount])
			{
				MaxAmp = NoncohResult[CorCount][FreqCount];
				MaxFreq = FreqCount;
			}
		}
		AmplitudeAcc /= 8;	// get average
		if (NoncohCount == (NonCoherentNumber - 1))		// last round
			NoiseFloor += (int)AmplitudeAcc;

		// insert the maximum amplitude value within frequency bins to peak sorter
		InsertPeak(MaxAmp, CorCount, MaxFreq);
	}
}

void CAcqEngine::DoNonCoherentSum(AeBufferSatParam *pSatParam)
{
	int i, j;
	int CohCount, NoncohCount, CorCount, FreqCount;
	complex_number PrevValue = GenerateNoise(NOISE_AMP_SQRT2), Value;
	double FreqDiff, FreqFade;
	int PhaseStart, PhaseEnd;
	int StartIndex, EndIndex, OffsetIndex;
	double CodeOffset;
	double PeakAmplitude[6], Amplitude;
	double DftTwiddleFactor, DftPhase;
	double DopplerPhase;
	int CurBit = 0;
	const double *PeakValues;

	PhaseStart = ((ReadAddress + CodeRoundCount) * MF_DEPTH) % PhaseCount;
	PhaseEnd = PhaseStart + MF_DEPTH;
	// calculate whether current search range cover code match position
	if (pSatParam && pSatParam->Time2CodeEnd >= (PhaseStart - 2) && pSatParam->Time2CodeEnd <= (PhaseEnd + 2))
	{
		PeakValues = (pSatParam->PrnSelect == 0) ? Bpsk2PeakValues : Boc2PeakValues;
		// amplitude fading due to frequency
		FreqDiff = fabs(PI * (CarrierFreq - pSatParam->Doppler) / 1000);
		FreqFade = (FreqDiff > 1e-3) ? (sin(FreqDiff) / FreqDiff) : 1.0;
		// calculate index of coherent result may have correlation peak
		CodeOffset = pSatParam->Time2CodeEnd - PhaseStart;
		// index range is +-3 code phases from current peak position
		StartIndex = (int)CodeOffset - 2;
		EndIndex = StartIndex + 5;
		if (StartIndex < 0) StartIndex = 0;
		if (EndIndex > MF_DEPTH) EndIndex = MF_DEPTH - 1;
		// determine whether farthest code phase within peak range
		OffsetIndex = (int)(fabs(CodeOffset - StartIndex) * 64 + 0.5);
		if (OffsetIndex > 160)
			StartIndex ++;
		OffsetIndex = (int)(fabs(CodeOffset - EndIndex) * 64 + 0.5);
		if (OffsetIndex > 160)
			EndIndex --;
		// assign peak amplitude
		for (j = StartIndex; j <= EndIndex; j ++)
		{
			OffsetIndex = (int)(fabs(CodeOffset - j) * 64 + 0.5);
			PeakAmplitude[j-StartIndex] = Bpsk2PeakValues[OffsetIndex] * pSatParam->Amplitude;
		}
	}
	else	// no correlation peak
	{
		StartIndex = 0;
		EndIndex = -1;
	}
//	printf("StartIndex=%d EndIndex=%d\n", StartIndex, EndIndex);

	// clear non-coherent result buffer
	memset(NoncohResult, 0, sizeof(NoncohResult));
	// do non-coherent and DFT accumulation loop
	for (NoncohCount = 0; NoncohCount < NonCoherentNumber; NoncohCount ++)
	{
		DopplerPhase = 0;		// reset Doppler phase for each non-coherent round
		DftTwiddleFactor = 0.;		// reset DFT twiddle factor for each non-coherent round
		for (CohCount = 0; CohCount < CoherentNumber; CohCount ++)
		{
			// generate Gauss noise and add adjacent two
			for (i = 0; i < MF_DEPTH; i ++)
			{
				Value = GenerateNoise(NOISE_AMP_SQRT2);
				CohResult[i] = PrevValue + Value;
				PrevValue = Value;
//				printf("%.5f %.5f\n", CohResult[i].real, CohResult[i].imag);
			}
//			memset(CohResult, 0, sizeof(CohResult));

			// determine modulation bit
			if (pSatParam)
				CurBit = pSatParam->BitArray[CurBitIndex];
			// add correlation peak result if any
			for (j = StartIndex; j <= EndIndex; j ++)
			{
				Amplitude = PeakAmplitude[j-StartIndex] * FreqFade;
				if (CurBit)
					Amplitude = -Amplitude;
				Value = complex_number(Amplitude, 0);
				Value *= complex_number(cos(DopplerPhase), sin(DopplerPhase));
//				printf("Coh=%f %f\n", Value.real, Value.imag);
				CohResult[j] += Value;
			}
			// add to DFT result
			for (CorCount = 0; CorCount < MF_DEPTH; CorCount ++)
			{
				if (CohCount == 0)	// for the first round of coherent sum, do not apply DFT twiddle factor
					for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
						DftResult[CorCount][FreqCount] = CohResult[CorCount];
				else	// for other round of coherent sum
				{
					for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
					{
						DftPhase = (FreqCount * 2 - 7) * DftTwiddleFactor;
						Value = complex_number(cos(DftPhase), sin(DftPhase));
						DftResult[CorCount][FreqCount] += CohResult[CorCount] * Value;
					}
				}
			}
//			for (FreqCount = 0; FreqCount < DFT_NUMBER && EndIndex >= 0; FreqCount ++)
//				printf("DFT=%f %f\n", DftResult[StartIndex+2][FreqCount].real, DftResult[StartIndex+2][FreqCount].imag);
			DopplerPhase += 2 * PI * (CarrierFreq - pSatParam->Doppler) / 1000;
			DftTwiddleFactor += DftTwiddlePhase;
//			printf("DopplerPhase=%.5f %DftPhase=%.5f\n", DopplerPhase/2/PI, DftTwiddleFactor/2/PI);
			// increase CurMsCount and determine bit position
			if (pSatParam && ++CurMsCount == pSatParam->BitLength)
			{
				CurMsCount = 0;
				CurBitIndex ++;
			}
		}
		NonCoherentAcc(NoncohCount);
		if (PeakFound() && EarlyTerminate)
			break;
	}
}

void CAcqEngine::InsertPeak(double Amp, int PartialCorPos, int PartialFreq)
{
	PeakData Peak;

	Peak.Amp = Amp;
	Peak.PhasePos = CodeRoundCount * MF_DEPTH + PartialCorPos;
	Peak.FreqPos = (StrideOffset << 3) + PartialFreq;
	PeakSorter.InsertValue(Peak);
}

int CAcqEngine::PeakFound()
{
	double PeakThreshold;

	PeakThreshold = PeakSorter.Peaks[2].Amp * (9 + PeakRatioTh) / 8;
	// early terminate acquisition if amplitude of maximum peak larger than threshold
	Success = (PeakSorter.Peaks[0].Amp >= PeakThreshold) ? 1 : 0;
	return Success;
}

void CAcqEngine::SearchOneChannel(AeBufferSatParam *pSatParam)
{
	Success = 0;

	PeakSorter.Clear();

	for (StrideCount = 1; StrideCount <= StrideNumber; StrideCount ++)
	{
		StrideOffset = (StrideCount >> 1);
		if (StrideCount & 1)
			StrideOffset = ~StrideOffset;
		StrideOffset += (StrideCount & 1);
		CarrierFreq = CenterFreq + StrideInterval * StrideOffset;
//		printf("Center Freq %f\n", CarrierFreq);

		for (CodeRoundCount = 0; CodeRoundCount < CodeSpan; CodeRoundCount ++)
		{
			// initialize CurMsCount and CurBitIndex for each non-coherent round
			if (pSatParam)
			{
				CurMsCount = pSatParam->MsCount + (CodeRoundCount + ReadAddress) / 3;
				CurBitIndex = CurMsCount / pSatParam->BitLength;
				CurMsCount %= pSatParam->BitLength;
			}
			else
			{
				CurMsCount = 0;
				CurBitIndex = 0;
			}

			DoNonCoherentSum(pSatParam);

			if (Success && EarlyTerminate)
				return;
		}
	}
}

void CAcqEngine::DoAcquisition()
{
	int i, j;
	int Freq;
	AeBufferSatParam *pSatParam;
	int GlobalExp;
	int Amp[3];

//	ChannelNumber = 1;
//	ChannelConfig[0][0] = 0x04010501;
//	ChannelConfig[0][1] = 0x07000201;

	for (i = 0; i < ChannelNumber; i ++)
	{
		// fill in config registers
		StrideNumber = EXTRACT_UINT(ChannelConfig[i][0], 0, 6);
		CoherentNumber = EXTRACT_UINT(ChannelConfig[i][0], 8, 6);
		NonCoherentNumber = EXTRACT_UINT(ChannelConfig[i][0], 16, 7);
		PeakRatioTh = EXTRACT_UINT(ChannelConfig[i][0], 24, 3);
		EarlyTerminate = EXTRACT_UINT(ChannelConfig[i][0], 27, 1);
		Freq = EXTRACT_INT(ChannelConfig[i][1], 0, 20);
		CenterFreq = Freq * 2.046e6 / 1048576.;
		Svid = EXTRACT_UINT(ChannelConfig[i][1], 24, 6);
		PrnSelect = EXTRACT_UINT(ChannelConfig[i][1], 30, 2);
		CodeSpan = EXTRACT_UINT(ChannelConfig[i][2], 0, 5);
		ReadAddress = EXTRACT_UINT(ChannelConfig[i][2], 8, 5);
		Freq = (int)EXTRACT_UINT(ChannelConfig[i][2], 20, 11);
		DftTwiddlePhase = Freq * PI / 8192;
		Freq = (int)EXTRACT_UINT(ChannelConfig[i][3], 0, 22);
		StrideInterval = Freq * 2.046e6 / 4294967296.;
		PhaseCount = 2046;	// default value for total phase count

		// find whether SV to be acquired exisst
		pSatParam = (AeBufferSatParam *)NULL;
		for (j = 0; j < SatNumber; j ++)
			if (Svid == SatParam[j].svid && PrnSelect == SatParam[j].PrnSelect)
			{
				switch (PrnSelect)
				{
				case FREQ_L1CA:	PhaseCount = 2046; break;
				case FREQ_E1:	PhaseCount = 2046 * 4; break;
				case FREQ_B1C:
				case FREQ_L1C:	PhaseCount = 2046 * 10; break;
				}
				pSatParam = &SatParam[j];
				break;
			}

		// Do searching
		SearchOneChannel(pSatParam);
		printf("Svid%2d Amp=%f Cor=%4d Freq=%f\n", Svid, PeakSorter.Peaks[0].Amp, PeakSorter.Peaks[0].PhasePos, (PeakSorter.Peaks[0].FreqPos - 3.5) * 62.5 + CenterFreq);

		// determine global exp
		GlobalExp = int(log10(PeakSorter.Peaks[0].Amp) / 0.3010 + 1) - 8;	// this is number of shift to have max amplitude fit in 8bit
		NoiseFloor >>= GlobalExp;
		Amp[0] = ((int)PeakSorter.Peaks[0].Amp) >> GlobalExp;
		Amp[1] = ((int)PeakSorter.Peaks[1].Amp) >> GlobalExp;
		Amp[2] = ((int)PeakSorter.Peaks[2].Amp) >> GlobalExp;

		// write back result
		ChannelConfig[i][4] = (Success << 31) | (GlobalExp << 24) | (NoiseFloor & 0x7ffff);
		ChannelConfig[i][5] = (Amp[0] << 24) | ((PeakSorter.Peaks[0].FreqPos & 0x1ff) << 15) | PeakSorter.Peaks[0].PhasePos;
		ChannelConfig[i][6] = (Amp[1] << 24) | ((PeakSorter.Peaks[1].FreqPos & 0x1ff) << 15) | PeakSorter.Peaks[1].PhasePos;
		ChannelConfig[i][7] = (Amp[2] << 24) | ((PeakSorter.Peaks[2].FreqPos & 0x1ff) << 15) | PeakSorter.Peaks[2].PhasePos;
//		printf("%08x %08x %08x %08x\n", ChannelConfig[i][4], ChannelConfig[i][5], ChannelConfig[i][6], ChannelConfig[i][7]);
	}
}

void CAcqEngine::SetBufferParam(PSATELLITE_PARAM SatelliteParam[], int SatVisible, GNSS_TIME Time, NavBit *NavData[])
{
	int i;

	SatNumber = 0;
	for (i = 0; i < SatVisible; i ++)
	{
		if (SatelliteParam[i]->system == GpsSystem)
		{
			AssignChannelParam(SatelliteParam[i], Time, NavData[0], 0, &SatParam[SatNumber++]);	// add L1C/A
//			AssignChannelParam(SatelliteParam[i], Time, NavData[3], 3, &SatParam[SatNumber++]);	// add L1C
		}
		else if (SatelliteParam[i]->system == BdsSystem)
			AssignChannelParam(SatelliteParam[i], Time, NavData[2], 2, &SatParam[SatNumber++]);	// add B1C
		else if (SatelliteParam[i]->system == GalileoSystem)
			AssignChannelParam(SatelliteParam[i], Time, NavData[1], 1, &SatParam[SatNumber++]);	// add E1C
	}
}

void CAcqEngine::AssignChannelParam(PSATELLITE_PARAM pSatelliteParam, GNSS_TIME Time, NavBit *NavData, int PrnSelect, AeBufferSatParam *pSatAcqParam)
{
	int i;
	int FrameLength, BitLength, FrameBits;
	int FrameNumber, BitNumber, MilliSeconds;
	int TotalBits, BitCount, Start, End;
	int Bits[1800];
	GNSS_TIME TransmitTime;
	double Time2CodeEnd;

	switch (PrnSelect)
	{
	case 0:	// GPS L1C/A
		FrameLength = 6000;
		BitLength = 20;
		FrameBits = 300;
		TotalBits = 8;	// maximum 8 bits within 128ms for GPS
		break;
	case 1:	// Galileo E1C
		FrameLength = 100;
		BitLength = 4;
		FrameBits = 25;
		TotalBits = 32;
		break;
	case 2:	// BDS B1C
		FrameLength = 18000;
		BitLength = 10;
		FrameBits = 1800;
		TotalBits = 14;
		Time.MilliSeconds -= 14000;	// compensate BDS leap second difference
		break;
	case 3:	// GPS L1C
		FrameLength = 18000;
		BitLength = 10;
		FrameBits = 1800;
		TotalBits = 14;
		break;
	}
	// calculate TransmitTimeMs, TransmitTime as Time - TravelTime
	TransmitTime = GetTransmitTime(Time, GetTravelTime(pSatelliteParam, 0));
	// calculate frame count and bit count
//	if (PrnSelect == 0)
		TransmitTime.MilliSeconds ++;	// time of NEXT code round
//	else
//		TransmitTime.MilliSeconds += BitLength;	// time of NEXT code round
	FrameNumber = TransmitTime.MilliSeconds / FrameLength;	// frame number
	MilliSeconds = TransmitTime.MilliSeconds - FrameNumber * FrameLength;	// remaining time in current frame in millisecond
	BitNumber = MilliSeconds / BitLength;	// bit number
	MilliSeconds -= BitNumber * BitLength;	// remaining time in current bit
	// assign parameters
	pSatAcqParam->PrnSelect = PrnSelect;
	pSatAcqParam->svid = pSatelliteParam->svid;
	pSatAcqParam->BitLength = BitLength;
	pSatAcqParam->MsCount = MilliSeconds;
	Time2CodeEnd = 1 - TransmitTime.SubMilliSeconds;
	if (PrnSelect != 0 && MilliSeconds != 0)
		Time2CodeEnd += (BitLength - MilliSeconds);
	pSatAcqParam->Time2CodeEnd = (Time2CodeEnd + 2.5 / SAMPLES_1MS) * 2046;	// convert to unit of 1/2 code chip with compensation of filter delay (2.5 samples)
	pSatAcqParam->Doppler = GetDoppler(pSatelliteParam, 0);
	pSatAcqParam->Amplitude = 2 * pow(10, (pSatelliteParam->CN0 - 3000) / 2000.) * NOISE_AMP_SQRT2;
	// generate bits
	BitCount = 0;
	while (BitCount < TotalBits)
	{
		Start = BitNumber;	// start from current bit
		End = FrameBits;	// to end of generated bit stream
		if ((End - Start) > (TotalBits - BitCount))	// if generated bit stream is longer, truncate the end
			End = Start + (TotalBits - BitCount);
		// get navigation bits and copy to BitArray
		NavData->GetFrameData(TransmitTime, pSatelliteParam->svid, 0, Bits);
		for (i = Start; i < End; i ++)
			pSatAcqParam->BitArray[BitCount++] = Bits[i];
		// move to next subframe
		BitNumber = 0;
		TransmitTime.MilliSeconds += FrameLength;
	}
}
