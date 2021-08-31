//----------------------------------------------------------------------
// TrackingEngine.cpp:
//   Tracking engine simulator class implementation
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
#include "TrackingEngine.h"

#define COH_OFFSET(ch_index, cor_index) ((ch_index << 5) + 24 + (cor_index >> 2))

const double CTrackingEngine::Bpsk4PeakValues[160] = {
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

const double CTrackingEngine::Boc4PeakValues[160] = {
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

CTrackingEngine::CTrackingEngine()
{
	Reset();
	memset(TEBuffer, 0, TE_BUFFER_SIZE);
	memset(ChannelParam, 0, sizeof(ChannelParam));
	// calculate relative matrix for noise generation
	CalculateCovar(COR_NUMBER, 2, CovarMatrix2);
	CalculateCovar(COR_NUMBER, 4, CovarMatrix4);
	CalculateCovar(COR_NUMBER, 8, CovarMatrix8);
}

CTrackingEngine::~CTrackingEngine()
{
}

void CTrackingEngine::Reset()
{
	ChannelEnable = CohDataReady = 0;
	OverwriteProtectAddr = 0;
	OverwriteProtectValue = 0;
}

void CTrackingEngine::SetRegValue(int Address, U32 Value)
{
	Address &= 0xff;
	switch (Address)
	{
	case ADDR_OFFSET_TE_CHANNEL_ENABLE:
		ChannelEnable = Value;
		break;
	case ADDR_OFFSET_TE_COH_DATA_READY:
		CohDataReady = Value;
		break;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_CHANNEL:
		OverwriteProtectChannel = Value;
		break;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_ADDR:
		OverwriteProtectAddr = Value;
		break;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_VALUE:
		OverwriteProtectValue = Value;
		break;
	case ADDR_OFFSET_TE_POLYNOMIAL:
		PrnPolyLength[0] = Value;
		break;
	case ADDR_OFFSET_TE_CODE_LENGTH:
		PrnPolyLength[1] = Value;
		break;
	case ADDR_OFFSET_TE_POLYNOMIAL2:
		PrnPolyLength[2] = Value;
		break;
	case ADDR_OFFSET_TE_CODE_LENGTH2:
		PrnPolyLength[3] = Value;
		break;
	default:
		break;
	}
}

U32 CTrackingEngine::GetRegValue(int Address)
{
	Address &= 0xff;
	switch (Address)
	{
	case ADDR_OFFSET_TE_CHANNEL_ENABLE:
		return ChannelEnable;
	case ADDR_OFFSET_TE_COH_DATA_READY:
		return CohDataReady;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_CHANNEL:
		return OverwriteProtectChannel;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_ADDR:
		return OverwriteProtectAddr;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_VALUE:
		return OverwriteProtectValue;
	case ADDR_OFFSET_TE_POLYNOMIAL:
		return PrnPolyLength[0];
	case ADDR_OFFSET_TE_CODE_LENGTH:
		return PrnPolyLength[1];
	case ADDR_OFFSET_TE_POLYNOMIAL2:
		return PrnPolyLength[2];
	case ADDR_OFFSET_TE_CODE_LENGTH2:
		return PrnPolyLength[3];
	default:
		return 0;
	}
}

// set value of TE config buffer
// interprete channel configuration parameters as well
void CTrackingEngine::SetTEBuffer(unsigned int Address, U32 Value)
{
	int ChannelNumber = (Address >> 5) & 0x1f;
	ChannelConfig *pChannelParam = &(ChannelParam[ChannelNumber]);

	TEBuffer[Address & 0xfff]= Value;
	Address &= 0x1f;	// offset within channel
	switch (Address)
	{
	case STATE_OFFSET_CARRIER_FREQ:
		pChannelParam->CarrierFreq = Value / 4294967296. * SAMPLE_FREQ;
		break;
	case STATE_OFFSET_CODE_FREQ:
		pChannelParam->CodeFreq = Value / 4294967296. * SAMPLE_FREQ;
		break;
	case STATE_OFFSET_CORR_CONFIG:
		pChannelParam->PreShiftBits = EXTRACT_UINT(Value, 0, 2);
		pChannelParam->EnableBOC = EXTRACT_UINT(Value, 2, 1);
		pChannelParam->DataInQBranch = EXTRACT_UINT(Value, 3, 1);
		pChannelParam->EnableSecondPrn = EXTRACT_UINT(Value, 4, 1);
		pChannelParam->NarrowFactor = EXTRACT_UINT(Value, 8, 2);
		pChannelParam->DumpLength = EXTRACT_UINT(Value, 16, 16);
		break;
	case STATE_OFFSET_NH_CONFIG:
		pChannelParam->NHCode = EXTRACT_UINT(Value, 0, 25);
		pChannelParam->NHLength = EXTRACT_UINT(Value, 27, 5);
		break;
	case STATE_OFFSET_COH_CONFIG:
		pChannelParam->NHCode2 = EXTRACT_UINT(Value, 0, 20);
		pChannelParam->MsDataNumber = EXTRACT_UINT(Value, 20, 5);
		pChannelParam->CoherentNumber = EXTRACT_UINT(Value, 25, 5);
		pChannelParam->PostShiftBits = EXTRACT_UINT(Value, 30, 2);
		break;
	case STATE_OFFSET_PRN_CONFIG:
		if ((Value & 0xf0000000) == 0)	// L1C/A
		{
			pChannelParam->SystemSel = 0;
			if ((pChannelParam->Svid = FindSvid(CAPrnInit, 32, Value)) == 0)	// not found in L1C/A initial array
			{
				if ((pChannelParam->Svid = FindSvid(CAPrnInit, 32, Value)) != 0)	// this is SBAS, continue from 33
					pChannelParam->Svid += 32;
			}
		}
		else if ((Value & 0xf0000000) == 0xc0000000)	// E1
		{
			pChannelParam->SystemSel = 1;
			pChannelParam->Svid = ((Value >> 6) & 0x7f) - 49;
			if (pChannelParam->Svid < 1 || pChannelParam->Svid > 50)
				pChannelParam->Svid = 0;	// only valid for E1C code range
		}
		else if ((Value & 0xf0000000) == 0x80000000)	// B1C
		{
			pChannelParam->SystemSel = 2;
			pChannelParam->Svid = FindSvid(B1CPilotInit, 63, Value);	// assume primary PRN code uses pilot code
		}
		else if ((Value & 0xf0000000) == 0xa0000000)	// L1C
		{
			pChannelParam->SystemSel = 3;
			pChannelParam->Svid = FindSvid(L1CPilotInit, 63, Value);	// assume primary PRN code uses pilot code
		}
		break;
	case STATE_OFFSET_PRN_COUNT:
		if (pChannelParam->SystemSel == 0)	// L1C/A
			pChannelParam->PrnCount = (Value >> 14);
		else if (pChannelParam->SystemSel == 1)	// E1
			pChannelParam->PrnCount = Value - (Value >> 10);
		else	// B1C or L1C
			pChannelParam->PrnCount = Value & 0x3fff;
		break;
	case STATE_OFFSET_CARRIER_PHASE:
		pChannelParam->CarrierPhase = Value;
		break;
	case STATE_OFFSET_CARRIER_COUNT:
		pChannelParam->CarrierCount = Value;
		break;
	case STATE_OFFSET_CODE_PHASE:
		pChannelParam->CodePhase = Value;
		break;
	case STATE_OFFSET_PRN_CODE:
		pChannelParam->JumpCount = EXTRACT_INT(Value, 8, 8);
		pChannelParam->DumpCount = EXTRACT_UINT(Value, 16, 16);
		break;
	case STATE_OFFSET_CORR_STATE:
		pChannelParam->CurrentCor = EXTRACT_UINT(Value, 4, 3);
		pChannelParam->Dumping = EXTRACT_UINT(Value, 7, 1);
		pChannelParam->CodeSubPhase = EXTRACT_UINT(Value, 8, 1);
		pChannelParam->MsDataCount = EXTRACT_UINT(Value, 16, 5);
		pChannelParam->CoherentCount = EXTRACT_UINT(Value, 21, 5);
		pChannelParam->NHCount = EXTRACT_UINT(Value, 27, 5);
		break;
	}
}

U32 CTrackingEngine::GetTEBuffer(unsigned int Address)
{
	unsigned int AddressOffset = Address & 0x1f;	// offset within channel
	int ChannelNumber = (Address >> 5) & 0x1f;
	ChannelConfig *pChannelParam = &(ChannelParam[ChannelNumber]);

	switch (AddressOffset)
	{
	case STATE_OFFSET_PRN_COUNT:
		if (pChannelParam->SystemSel == 0)	// L1C/A
			return pChannelParam->PrnCount << 14;
		else if (pChannelParam->SystemSel == 1)	// E1
			return ((pChannelParam->PrnCount / 1023) << 10) | (pChannelParam->PrnCount % 1023);
		else	// B1C or L1C
			return pChannelParam->PrnCount;
		break;
	case STATE_OFFSET_CARRIER_PHASE:
		return pChannelParam->CarrierPhase;
		break;
	case STATE_OFFSET_CARRIER_COUNT:
		return pChannelParam->CarrierCount;
		break;
	case STATE_OFFSET_CODE_PHASE:
		return pChannelParam->CodePhase;
		break;
	case STATE_OFFSET_PRN_CODE:
		return pChannelParam->DumpCount << 16;
		break;
	case STATE_OFFSET_CORR_STATE:
		return (pChannelParam->NHCount << 27) | (pChannelParam->CoherentCount << 21) | (pChannelParam->MsDataCount << 16) | (pChannelParam->CodeSubPhase << 8) | (((pChannelParam->CurrentCor == 0) ? 0 : 1) << 7) | (pChannelParam->CurrentCor << 4) | (pChannelParam->MsDataDone << 1) | pChannelParam->CoherentDone;
	default:
		return TEBuffer[Address & 0xfff];
	}
}

// process one system
// input parameter specify the system index
// return 1 if any correlator in any channel has data ready
int CTrackingEngine::ProcessData(GNSS_TIME CurTime, SATELLITE_PARAM GpsSatParam[], int GpsSatNumber)
{
	unsigned int EnableMask;
	int i, j;
	SATELLITE_PARAM *pSatParam;
	int DumpDataI[16], DumpDataQ[16];
	int CorIndex[16], CorPos[16];
	int DataLength;
	unsigned int CohData;
	S16 CohDataI, CohDataQ;

	// clear coherent data ready flag and overwrite protect flag
	CohDataReady = 0;
	OverwriteProtectChannel = 0;

	for (i = 0, EnableMask = 1; i < 32; i ++, EnableMask <<= 1)
	{
		if ((ChannelEnable & EnableMask) == 0)
			continue;
		// find whether there is visible satellite match current channel
		pSatParam = FindSatParam(i, GpsSatParam, GpsSatNumber);

		// recalculate corresponding counter of channel
		if (CalculateCounter(i, CorIndex, CorPos, DataLength))
			CohDataReady |= EnableMask;
		// calculate 1ms correlation result
		GetCorrelationResult(i, CurTime, pSatParam, DumpDataI, DumpDataQ, CorIndex, CorPos, DataLength);
		// do coherent sum
		for (j = 0; j < DataLength; j ++)
		{
			// if overwrite protect bit is set, set corresponding flag bit and set address and value. do NOT accumulate
			if (CorIndex[j] & 2)
			{
				OverwriteProtectChannel |= EnableMask;
				OverwriteProtectAddr = COH_OFFSET(i, CorIndex[j]) << 2;
				OverwriteProtectValue = ((DumpDataI[j] & 0xffff) << 16) | (DumpDataQ[j] & 0xffff);
				continue;
			}

			// if this is first epoch of coherent data accumulation, clear stored value
			if (CorIndex[j] & 1)
				CohData = 0;
			else
				CohData = TEBuffer[COH_OFFSET(i, CorIndex[j])];
			CohDataI = (S16)(CohData >> 16);
			CohDataQ = (S16)(CohData & 0xffff);
			CohDataI += (S16)DumpDataI[j];
			CohDataQ += (S16)DumpDataQ[j];
			CohData = ((unsigned int)CohDataI << 16) | ((unsigned int)CohDataQ & 0xffff);
			TEBuffer[COH_OFFSET(i, CorIndex[j])] = CohData;
		}
		if (DataLength)
			ChannelParam[i].CurrentCor = ((CorIndex[DataLength-1] >> 2) + 1) & 0x7;	// CurrentCor is next to the last output correlator
	}

	return (CohDataReady != 0);
}

int CTrackingEngine::FindSvid(unsigned int ConfigArray[], int ArraySize, U32 PrnConfig)
{
	int i;

	for (i = 0; i < ArraySize; i ++)
		if (PrnConfig == ConfigArray[i])
			return (i + 1);
	return 0;	// not found
}

SATELLITE_PARAM* CTrackingEngine::FindSatParam(int ChannelId, SATELLITE_PARAM GpsSatParam[], int GpsSatNumber)
{
	int i;
	int SystemSel = ChannelParam[ChannelId].SystemSel;
	int Svid = ChannelParam[ChannelId].Svid;

	switch (SystemSel)
	{
	case 0:
		for (i = 0; i < GpsSatNumber; i ++)
			if (Svid == GpsSatParam[i].svid)
				return &GpsSatParam[i];
		break;
	case 1:
	case 2:
	case 3:
	default:
		break;
	}
	return (SATELLITE_PARAM *)0;
}

void CTrackingEngine::GetCorrelationResult(int ChannelId, GNSS_TIME CurTime, SATELLITE_PARAM *pSatParam, int DumpDataI[], int DumpDataQ[], int CorIndex[], int CorPos[], int DataLength)
{
	int i;
	double Alpha;
	double *CovarMatrix;
	int MaxIndex;
	complex_number CorResult[16];
	double CodeDiff[16];
	int CorCount = 0, StartIndex = 0;
	double Doppler1, Doppler2;
	double FreqDiff, PhaseDiff;
	double CarrierPhase;
	double Amplitude, PeakPosition, NcoPhase, CorPosition, AmpRatio;
	GNSS_TIME TransmitTime;
	int CodeDiffIndex, CodeLength;
	int FrameNumber, BitNumber, Milliseconds, DataBit;

	// calculate half of 128x code length
	CodeLength = (ChannelParam[ChannelId].SystemSel == 0) ? 1023 : ((ChannelParam[ChannelId].SystemSel == 1) ? 4092 : 10230);
	CodeLength *= 64;

	// first generate relative noise
	if (ChannelParam[ChannelId].NarrowFactor == 1)
	{
		CovarMatrix = CovarMatrix4;
		MaxIndex = 4;
	}
	else if (ChannelParam[ChannelId].NarrowFactor == 2)
	{
		CovarMatrix = CovarMatrix8;
		MaxIndex = 8;
	}
	else
	{
		CovarMatrix = CovarMatrix2;
		MaxIndex = 2;
	}
	while (CorCount < DataLength)
	{
		if ((CorIndex[CorCount] >> 2) == 0)	// new round of correlator 0~7, generate 8 Gauss Noise
			GenerateRelativeNoise(8, MaxIndex, CovarMatrix, NOISE_AMP, ChannelParam[ChannelId].GaussNoise);
		CorResult[CorCount ++] = ChannelParam[ChannelId].GaussNoise[CorIndex[CorCount] >> 2];
	}
//	memset(CorResult, 0, sizeof(CorResult));
	if (pSatParam)
	{
		// calculate carrier phase of source signal
		CarrierPhase = pSatParam->TravelTime * 1575.42e6 - pSatParam->IonoDelay / GPS_L1_WAVELENGTH;
//		printf("Phase=%.5f", CarrierPhase);
		CarrierPhase -= (int)CarrierPhase;
		CarrierPhase = 1 - CarrierPhase;
//		printf(" %.5f\n", CarrierPhase);
//		printf("Diff=%.5f", CarrierPhase - ChannelParam[ChannelId].CarrierPhase / 4294967296.);
		// assign new value
		CarrierParam[ChannelId].DopplerPrev2 = CarrierParam[ChannelId].DopplerPrev;
		CarrierParam[ChannelId].DopplerPrev = CarrierParam[ChannelId].DopplerCur;
		CarrierParam[ChannelId].DopplerCur = -pSatParam->RelativeSpeed / GPS_L1_WAVELENGTH;
		CarrierParam[ChannelId].LocalFreqPrev2 = CarrierParam[ChannelId].LocalFreqPrev;
		CarrierParam[ChannelId].LocalFreqPrev = ChannelParam[ChannelId].CarrierFreq - IF_FREQ;
		CarrierParam[ChannelId].DeltaPhiPrev2 = CarrierParam[ChannelId].DeltaPhiPrev;
		CarrierParam[ChannelId].DeltaPhiPrev = CarrierParam[ChannelId].DeltaPhiCur;
		CarrierParam[ChannelId].DeltaPhiCur = (CarrierPhase - ChannelParam[ChannelId].CarrierPhase / 4294967296.);
		// first calculate average carrier frequency difference and phase difference
		Alpha = ((double)CorPos[0] / ChannelParam[ChannelId].DumpLength / 2);
		Doppler1 = ((1 - Alpha) * CarrierParam[ChannelId].DopplerPrev2 + (1 + Alpha) * CarrierParam[ChannelId].DopplerPrev) / 2;
		Doppler2 = ((2 - Alpha) * CarrierParam[ChannelId].DopplerPrev + Alpha * CarrierParam[ChannelId].DopplerCur) / 2;
		FreqDiff = ((1 - Alpha) * (Doppler1 - CarrierParam[ChannelId].LocalFreqPrev2) + Alpha * (Doppler2 - CarrierParam[ChannelId].LocalFreqPrev)) / 1000.;
		Doppler1 = PhaseAverage(CarrierParam[ChannelId].DeltaPhiPrev2, 1 - Alpha, CarrierParam[ChannelId].DeltaPhiPrev, 1 + Alpha) / 2;
		Doppler2 = PhaseAverage(CarrierParam[ChannelId].DeltaPhiPrev, 2 - Alpha, CarrierParam[ChannelId].DeltaPhiCur, Alpha) / 2;
		PhaseDiff = PhaseAverage(Doppler1, 1 - Alpha, Doppler2, Alpha);
//		printf(" %.5f\n", PhaseDiff);
		// calculate signal amplitude
		Amplitude = 1.4142135 * pow(10, (pSatParam->CN0 - 3000) / 2000.) * NOISE_AMP;
		Amplitude *= (FreqDiff > 1e-3) ? (sin(FreqDiff) / FreqDiff) : 1.0;
		TransmitTime = GetTransmitTime(CurTime, pSatParam->TravelTime + pSatParam->IonoDelay / LIGHT_SPEED + 0.001);	// add one extra millisecond to get previous finished millisecond
		FrameNumber = TransmitTime.MilliSeconds / 6000;
		BitNumber = (TransmitTime.MilliSeconds % 6000) / 20;
		Milliseconds = TransmitTime.MilliSeconds % 20;
		if (FrameNumber != ChannelParam[ChannelId].CurrentFrame)
		{
			NavData->GetFrameData(TransmitTime, ChannelParam[ChannelId].Svid, ChannelParam[ChannelId].Bits);
			ChannelParam[ChannelId].CurrentFrame = FrameNumber;
		}
		PeakPosition = TransmitTime.SubMilliSeconds * 2046;
		NcoPhase = (double)ChannelParam[ChannelId].CodePhase / 4294967296.;
		// calculate code difference for each correlator and add narrow correlator compensation
		for (i = 0; i < DataLength; i ++)
		{
			CorPosition = CorPos[i] + NcoPhase;
			if (ChannelParam[ChannelId].NarrowFactor)
				CorPosition += NarrowCompensation(CorIndex[i] >> 2, ChannelParam[ChannelId].NarrowFactor);
			CodeDiff[i] = fabs(CorPosition - PeakPosition) * 64;
		}
		// calculate 1ms correlation value
		DataBit = ChannelParam[ChannelId].CurrentBit;	// if current correlator dump not finished, use CurrentBit
		for (i = 0; i < DataLength; i ++)
		{
			if ((CorIndex[i] >> 2) == 0)	// new correlator dump uses new bit
			{
				ChannelParam[ChannelId].CurrentBit = DataBit = ChannelParam[ChannelId].Bits[BitNumber];
				BitNumber += (Milliseconds + 1) / 20;	// if there will be next Cor0, check whether next milliseconds move to next bit
			}
			if (CodeDiff[i] > CodeLength)	// CodeDiff may be one whole code round difference
				CodeDiff[i] = fabs(CodeDiff[i] - (CodeLength * 2));
			CodeDiffIndex = (int)CodeDiff[i];
			CodeDiff[i] -= CodeDiffIndex;
			if (CodeDiffIndex < 159)
			{
				AmpRatio = Bpsk4PeakValues[CodeDiffIndex];
				AmpRatio += (Bpsk4PeakValues[CodeDiffIndex+1] - Bpsk4PeakValues[CodeDiffIndex]) * CodeDiff[i];
				AmpRatio = DataBit ? -AmpRatio : AmpRatio;
				CorResult[i] += (complex_number(Amplitude * AmpRatio, 0) * complex_number(cos(PhaseDiff * PI2), sin(PhaseDiff * PI2)));
//				if ((CorIndex[i] >> 2) == 4)
//					printf("Cor=%f %f %f\n", Amplitude * AmpRatio, CorResult[i].real, CorResult[i].imag);
			}
		}
	}
	for (i = 0; i < DataLength; i ++)
	{
		DumpDataI[i] = ((int)CorResult[i].real) >> (ChannelParam[ChannelId].PreShiftBits + ChannelParam[ChannelId].PostShiftBits);
		DumpDataQ[i] = ((int)CorResult[i].imag) >> (ChannelParam[ChannelId].PreShiftBits + ChannelParam[ChannelId].PostShiftBits);
	}
}

int CTrackingEngine::CalculateCounter(int ChannelId, int CorIndex[], int CorPos[], int &DataLength)
{
	int i;
	U64 Count;
	int OverflowCount;
	int PrnCount2x;	// 2 times of PrnCount
	int DumpLength2x = ChannelParam[ChannelId].DumpLength * 2;	// 2 times of DumpLength
	int CodeLength = (ChannelParam[ChannelId].SystemSel == 0) ? 1023 : ((ChannelParam[ChannelId].SystemSel == 0) ? 4092 : 10230);
	int DumpIncrease, CodeRound, DumpRemnant;
	int NewCoh = 0;
	unsigned int *ChannelBuffer = TEBuffer + ChannelId * 32;
	int OverwriteProtect = 0;
	int FirstCor = -1;

	ChannelParam[ChannelId].CoherentDone = 0;
	ChannelParam[ChannelId].MsDataDone = 0;

	// carrier related counter
	Count = (((U64)ChannelParam[ChannelId].CarrierCount) << 32) | ChannelParam[ChannelId].CarrierPhase;
	Count += (U64)ChannelBuffer[STATE_OFFSET_CARRIER_FREQ] * SAMPLE_COUNT;
	ChannelParam[ChannelId].CarrierCount = (unsigned int)(Count >> 32);	// upper 32bit
	ChannelParam[ChannelId].CarrierPhase = (unsigned int)(Count & 0xffffffff);	// lower 32bit
//	printf("Local=%d %f\n", ChannelParam[ChannelId].CarrierCount, ChannelParam[ChannelId].CarrierPhase / 4294967296.);

	// number of code NCO overflow
	Count = (U64)ChannelParam[ChannelId].CodePhase + (U64)ChannelBuffer[STATE_OFFSET_CODE_FREQ] * SAMPLE_COUNT;
	OverflowCount = (int)(Count >> 32);
	OverflowCount += ChannelParam[ChannelId].JumpCount;
	ChannelParam[ChannelId].JumpCount = 0;	// reset JumpCount after each round

	// determine remnant correlator to dump before increase DumpCount
	DataLength = 0;
	DumpRemnant = ChannelParam[ChannelId].DumpCount * 2 + ChannelParam[ChannelId].CodeSubPhase;	// previous remnant 1/2 chip after dumping
	if (DumpRemnant < 7)	// need to complete remaining correlation result
	{
		FirstCor = DumpRemnant + 1;
		for (i = DumpRemnant + 1; i < 8; i ++)
		{
			CorPos[DataLength] = DumpRemnant - i;
			CorIndex[DataLength++] = (i << 2) + (ChannelParam[ChannelId].CoherentCount == 0 ? 1 : 0);
		}
		ChannelParam[ChannelId].CoherentDone |= (ChannelParam[ChannelId].CoherentCount == ChannelParam[ChannelId].CoherentNumber - 1) ? 1 : 0;
		ChannelParam[ChannelId].CoherentCount ++;
		if (ChannelParam[ChannelId].CoherentNumber)
			ChannelParam[ChannelId].CoherentCount %= ChannelParam[ChannelId].CoherentNumber;
	}

	// recalculate CodePhase, CodeSubPhase and PrnCount
	ChannelParam[ChannelId].CodePhase = (unsigned int)(Count & 0xffffffff);	// lower 32bit
	PrnCount2x = ChannelParam[ChannelId].PrnCount * 2 + ChannelParam[ChannelId].CodeSubPhase;
	PrnCount2x += OverflowCount;
	DumpIncrease = PrnCount2x / 2 - ChannelParam[ChannelId].PrnCount;
	ChannelParam[ChannelId].CodeSubPhase = PrnCount2x & 1;
	ChannelParam[ChannelId].PrnCount = PrnCount2x / 2;
	CodeRound = ChannelParam[ChannelId].PrnCount / CodeLength;
	ChannelParam[ChannelId].PrnCount %= CodeLength;

	// DumpCount and NHCount align with first correlator
	ChannelParam[ChannelId].DumpCount += DumpIncrease;
	NewCoh = ChannelParam[ChannelId].DumpCount / ChannelParam[ChannelId].DumpLength;
	ChannelParam[ChannelId].DumpCount %= ChannelParam[ChannelId].DumpLength;
	if (ChannelParam[ChannelId].NHLength)
	{
		ChannelParam[ChannelId].NHCount += CodeRound;
		ChannelParam[ChannelId].NHCount %= ChannelParam[ChannelId].NHLength;
	}

	// determine remnant correlator to dump after increase DumpCount
	DumpRemnant = ChannelParam[ChannelId].DumpCount * 2 + ChannelParam[ChannelId].CodeSubPhase;	// previous remnant 1/2 chip after dumping
	if (NewCoh == 2)
		NewCoh = NewCoh;
	while (NewCoh)
	{
		for (i = 0; i < 8; i ++)
		{
			if (i > DumpRemnant && NewCoh == 1)
				break;
			if (FirstCor == i && ChannelParam[ChannelId].CoherentCount == 0)
				OverwriteProtect = 2;
			CorPos[DataLength] = DumpRemnant - i;
			CorIndex[DataLength++] = (i << 2) + (ChannelParam[ChannelId].CoherentCount == 0 ? 1 : 0) + OverwriteProtect;
			ChannelParam[ChannelId].CoherentDone |= (ChannelParam[ChannelId].CoherentCount == ChannelParam[ChannelId].CoherentNumber - 1) ? 1 : 0;
		}
		if (FirstCor < 0)
			FirstCor = 0;
		NewCoh --;
		if (i == 8)	// last correlator dumped
		{
			ChannelParam[ChannelId].CoherentCount ++;
			if (ChannelParam[ChannelId].CoherentNumber)
				ChannelParam[ChannelId].CoherentCount %= ChannelParam[ChannelId].CoherentNumber;
		}
	}
	if (DataLength > 8)
		DataLength = DataLength;

	return ChannelParam[ChannelId].CoherentDone;
}

void CTrackingEngine::InitChannel(int ChannelId, GNSS_TIME CurTime, SATELLITE_PARAM GpsSatParam[], int GpsSatNumber)
{
	SATELLITE_PARAM *pSatParam;
	GNSS_TIME TransmitTime;

	pSatParam = FindSatParam(ChannelId, GpsSatParam, GpsSatNumber);
	if (!pSatParam)
		return;
	// initial structure to calculate FreqDiff and PhaseDiff
	CarrierParam[ChannelId].DopplerPrev2 = CarrierParam[ChannelId].DopplerPrev = CarrierParam[ChannelId].DopplerCur = -pSatParam->RelativeSpeed / GPS_L1_WAVELENGTH;
	CarrierParam[ChannelId].LocalFreqPrev2 = CarrierParam[ChannelId].LocalFreqPrev = ChannelParam[ChannelId].CarrierFreq - IF_FREQ;
	// initial modulation bit and counter
	TransmitTime = GetTransmitTime(CurTime, pSatParam->TravelTime + pSatParam->IonoDelay / LIGHT_SPEED);
	TransmitTime.MilliSeconds ++;	// correlation result will be calculated from next millisecond
	ChannelParam[ChannelId].CurrentFrame = TransmitTime.MilliSeconds / 6000;	// frame number
	ChannelParam[ChannelId].CurrentBit = 0;
	NavData->GetFrameData(TransmitTime, ChannelParam[ChannelId].Svid, ChannelParam[ChannelId].Bits);
}

double CTrackingEngine::NarrowCompensation(int CorIndex, int NarrowFactor)
{
	if (CorIndex < 2 || CorIndex > 6 || NarrowFactor > 2)
		return 0.0;
	return (CorIndex - 4) * ((NarrowFactor == 1) ? 0.5 : 0.75);
}

double CTrackingEngine::PhaseAverage(double Phase1, double Ratio1, double Phase2, double Ratio2)
{
	// the average phase is Phase1*Ratio1 + Phase2*Ratio2
	// assume difference of Phase1 to Phase2 will not be larger than half cycle
	if (Phase2 < Phase1 - 0.5)
		Phase2 += 1.0;
	else if (Phase2 > Phase1 + 0.5)
		Phase2 -= 1.0;
	return Phase1 * Ratio1 + Phase2 * Ratio2;
}
