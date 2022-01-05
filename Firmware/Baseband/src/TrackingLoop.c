//----------------------------------------------------------------------
// TrackingLoop.c:
//   Tracking loop related functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include "ChannelManager.h"
#include "BBCommonFunc.h"

static void FFT8(int InputReal[8], int InputImag[8], int OutputReal[8], int OutputImag[8]);
static int CordicAtan(int x, int y, int mode);
static int Rotate(int x, int y);
static void SearchPeakCoh(int NoncohBuffer[], PSEARCH_PEAK_RESULT SearchResult);
static void SearchPeakFft(int NoncohBuffer[], PSEARCH_PEAK_RESULT SearchResult);
static void GetCoefficients(int BnT16x, int Order, int Coef[3]);
static void AdjustLockIndicator(int *LockIndicator, int Adjustment);

// the table is calculated as Kn*2^33/fs, fs = 4113
static int FilterCoef1[10] = {	// first order coefficients
 81827, 160584, 236416, 309304, 379686, 447562, 512932, 576004, 637196, 696092,
};

static int FilterCoef20[10][2] = {	// second order coefficients for BnT < 0.1
{  54071,   709, }, { 105239,  2719, }, { 153671,  5869, }, { 199576, 10010, }, { 243100, 15018, },
{ 284452, 20780, }, { 323924, 27192, }, { 361517, 34188, }, { 397230, 41623, }, { 431481, 49539, },
};

static int FilterCoef30[10][3] = {	// third order coefficients for BnT < 0.1
{  53883,   601,    3, }, { 102712,  2310,   20, }, { 149786,  4977,   63, }, { 194271,  8479,  142, }, { 236416, 12708,  264, },
{ 276098, 17564,  433, }, { 313899, 22952,  655, }, { 349612, 28821,  931, }, { 383655, 35066, 1265, }, { 415817, 41665, 1655, },
};

static int FilterCoef21[6][2] = {	// second order coefficients for 0.1 < BnT < 0.35
{ 431481,  49539, }, { 582269,  93376, }, { 707370, 140764, }, { 817015, 188235, }, { 932299, 231404, }, {1021686, 277560, },
};

static int FilterCoef31[6][3] = {	// third order coefficients for 0.1 < BnT < 0.35
{ 415817,  41665,  1655, }, { 554910,  77984,  4459, }, { 664764, 116851,  8569, },
{ 753107, 155634, 13748, }, { 825160, 192934, 19738, }, { 885726, 228271, 26294, },
};

//#define POWER(x, y) AmplitudeJPL(x,y)
#define POWER(x, y) ((x)*(x) + (y)*(y))

//*************** Calculate discriminator result ****************
// Parameters:
//   ChannelState: pointer to channel state buffer
//   Method: indicator which discriminator to calculate
// Return value:
//   none
void CalcDiscriminator(PCHANNEL_STATE ChannelState, unsigned int Method)
{
	SEARCH_PEAK_RESULT SearchResult;
	int Denominator, Numerator;
	S16 CorResutReal, CorResultImag;
	int CohLength = ChannelState->CoherentNumber;
	int NoncohLength = CohLength * ChannelState->FftNumber * ChannelState->NonCohNumber;

	// for FLL and DLL, search for peak power
	if (Method & (TRACKING_UPDATE_FLL | TRACKING_UPDATE_DLL))
	{
		if (ChannelState->FftNumber == 1)
			SearchPeakCoh(ChannelState->NoncohBuffer, &SearchResult);
		else
			SearchPeakFft(ChannelState->NoncohBuffer, &SearchResult);
		ChannelState->PeakPower = SearchResult.PeakPower * SearchResult.PeakPower;
	}
	if ((Method & TRACKING_UPDATE_FLL) && ChannelState->fll_k1 > 0)
	{
		Denominator = 2 * SearchResult.PeakPower - SearchResult.LeftBinPower - SearchResult.RightBinPower;
		Numerator = SearchResult.LeftBinPower - SearchResult.RightBinPower;
		// atan((L-R)/(2P-R-L))
		ChannelState->FrequencyDiff = (CordicAtan(Denominator, Numerator, 0) >> 1);
		ChannelState->FrequencyDiff += (SearchResult.FreqBinDiff << 13);
		// lock indicator
		AdjustLockIndicator(&(ChannelState->FLD), ChannelState->FrequencyDiff >> 10);
//		printf("FLD=%3d\n", ChannelState->FLD);
		if (SearchResult.FreqBinDiff)
			ChannelState->LoseLockCounter += NoncohLength;
		else
			ChannelState->LoseLockCounter -= NoncohLength;
		ChannelState->State |= TRACKING_UPDATE_FLL;
	}
	if ((Method & TRACKING_UPDATE_DLL) && ChannelState->dll_k1 > 0)
	{
//		printf("EPL = %5d %5d %5d\n", SearchResult.EarlyPower, SearchResult.PeakPower, SearchResult.LatePower);
		Denominator = 2 * SearchResult.PeakPower - SearchResult.EarlyPower - SearchResult.LatePower;
		Numerator = SearchResult.EarlyPower - SearchResult.LatePower;
		// (E-L)/(2P-E-L))
		ChannelState->DelayDiff = Denominator ? -((Numerator << 13) / Denominator) : 0;
		ChannelState->DelayDiff += (SearchResult.CorDiff << 14);
		// lock indicator
		AdjustLockIndicator(&(ChannelState->DLD), ChannelState->DelayDiff >> 11);
//		printf("DLD=%3d\n", ChannelState->DLD);
		if (SearchResult.CorDiff)
			ChannelState->LoseLockCounter += NoncohLength;
		else
			ChannelState->LoseLockCounter -= NoncohLength;
		ChannelState->State |= TRACKING_UPDATE_DLL;
	}
	if ((Method & TRACKING_UPDATE_PLL) && ChannelState->pll_k1 > 0)
	{
		CorResutReal = (S16)(ChannelState->StateBufferCache.CoherentSum[4] >> 16);
		CorResultImag = (S16)(ChannelState->StateBufferCache.CoherentSum[4] & 0xffff);
		ChannelState->PhaseDiff = CordicAtan(CorResutReal, CorResultImag, 0);
		// lock indicator
		AdjustLockIndicator(&(ChannelState->PLD), ChannelState->PhaseDiff >> 9);
//		printf("PLD=%3d\n", ChannelState->PLD);
		if (ChannelState->PhaseDiff > 4096 || ChannelState->PhaseDiff < -4096)
			ChannelState->LoseLockCounter += CohLength;
		else
			ChannelState->LoseLockCounter -= CohLength;
		ChannelState->State |= TRACKING_UPDATE_PLL;
	}
	if (ChannelState->LoseLockCounter < 0)
		ChannelState->LoseLockCounter = 0;
//	if (ChannelState->Svid == 4)
//		printf("LostCounter=%d\n", ChannelState->LoseLockCounter);
}

//*************** Do 8 point FFT on coherent buffer and accumulate to noncoherent buffer ****************
// Parameters:
//   ChannelState: pointer to channel state buffer
// Return value:
//   none
void CohBufferFft(PCHANNEL_STATE ChannelState)
{
	int i, j;
	S32 CohResult;
	int CohReal[MAX_FFT_NUM], CohImag[MAX_FFT_NUM];
	int FftResultReal[MAX_BIN_NUM], FftResultImag[MAX_BIN_NUM];
//	int CarrierFreq;

	for (i = ChannelState->FftNumber; i < MAX_BIN_NUM; i ++)
		CohReal[i] = CohImag[i] = 0;	// fill rest of FFT input sample with 0
	// clear noncoherent acc result on first accumulation
	if (ChannelState->NonCohCount == 0)
		memset(ChannelState->NoncohBuffer, 0, sizeof(ChannelState->NoncohBuffer));
	// do FFT for all correlators
	for (i = 0; i < CORRELATOR_NUM; i ++)
	{
		for (j = 0; j < ChannelState->FftNumber; j ++)
		{
			CohResult = (S32)ChannelState->CohBuffer[j * CORRELATOR_NUM + i];
			CohReal[j] = (int)(CohResult >> 16);
			CohImag[j] = (int)((S16)CohResult);
		}
		FFT8(CohReal, CohImag, FftResultReal, FftResultImag);
/*		if ((ChannelState->State & STAGE_MASK) == 3)
		{
			printf("COH: ");
			for (j = 0; j < ChannelState->FftNumber; j ++)
			{
				printf("%5d %5d, ", CohReal[j], CohImag[j]);
			}
			printf("\n");
			printf("FFT: ");
			for (j = MAX_BIN_NUM/2; j < MAX_BIN_NUM; j ++)
				printf("%8d ", POWER(FftResultReal[j - MAX_BIN_NUM/2], FftResultImag[j - MAX_BIN_NUM/2]));
			for (j = 0; j < MAX_BIN_NUM/2; j ++)
				printf("%8d ", POWER(FftResultReal[j + MAX_BIN_NUM/2], FftResultImag[j + MAX_BIN_NUM/2]));
			printf("\n");
		}*/
		// accumulate power, move 0 frequency bin in middle
		for (j = 0; j < MAX_BIN_NUM/2; j ++)
			ChannelState->NoncohBuffer[i * MAX_BIN_NUM + j] += POWER(FftResultReal[j + MAX_BIN_NUM/2], FftResultImag[j + MAX_BIN_NUM/2]);
		for (; j < MAX_BIN_NUM; j ++)
			ChannelState->NoncohBuffer[i * MAX_BIN_NUM + j] += POWER(FftResultReal[j - MAX_BIN_NUM/2], FftResultImag[j - MAX_BIN_NUM/2]);
	}
	if (++ChannelState->NonCohCount == ChannelState->NonCohNumber)
	{
		ChannelState->NonCohCount = 0;
		CalcDiscriminator(ChannelState, TRACKING_UPDATE_FLL | TRACKING_UPDATE_DLL);
//		CarrierFreq = (int)(((S64)ChannelState->CarrierFreqBase * SAMPLE_FREQ) >> 32);
//		printf("SV%02d FD = %6d Doppler = %d DD = %6d\n", ChannelState->Svid, ChannelState->FrequencyDiff, CarrierFreq - IF_FREQ, ChannelState->DelayDiff);
	}
}

//*************** Do 8 point FFT on coherent buffer and put in noncoherent buffer ****************
// Parameters:
//   ChannelState: pointer to channel state buffer
// Return value:
//   none
void CohBufferAcc(PCHANNEL_STATE ChannelState)
{
	int i;
	S32 CohResult;
	int CohReal, CohImag;

	if (ChannelState->NonCohCount == 0)
		memset(ChannelState->NoncohBuffer, 0, sizeof(int) * 7);	// clear first 7 value for 7 correlators
	for (i = 0; i < CORRELATOR_NUM; i ++)
	{
		CohResult = (S32)ChannelState->CohBuffer[i];
		CohReal = (int)(CohResult >> 16);
		CohImag = (int)((S16)CohResult);
		ChannelState->NoncohBuffer[i] += POWER(CohReal, CohImag);
	}
	if (++ChannelState->NonCohCount == ChannelState->NonCohNumber)
	{
		ChannelState->NonCohCount = 0;
		CalcDiscriminator(ChannelState, TRACKING_UPDATE_DLL);
	}
}

#define BUTTERFLY(N, real, image, cos_value, sin_value) \
do { \
    int temp_r, temp_i; \
    temp_r = (int)((((*(real+N)) * cos_value) >> 16) + (((*(image+N)) * sin_value) >> 16)); \
    temp_i = (int)((((*(real+N)) * sin_value) >> 16) - (((*(image+N)) * cos_value) >> 16)); \
    *(real+N) = *(real) - temp_r; \
    *(image+N) = *(image) + temp_i; \
    *(real) += temp_r; \
    *(image) -= temp_i; \
} while(0);

//*************** 8 point FFT ****************
// Parameters:
//   InputReal: real part of time domain input
//   InputImag: imaginary part of time domain input
//   OutputReal: real part of frequency domain output
//   OutputImag: imaginary part of frequency domain output
// Return value:
//   none
static void FFT8(int InputReal[8], int InputImag[8], int OutputReal[8], int OutputImag[8])
{
    int i;

    // even position input do 4 point FFT
    OutputReal[0] = InputReal[0] + InputReal[4] + InputReal[2] + InputReal[6];
    OutputImag[0] = InputImag[0] + InputImag[4] + InputImag[2] + InputImag[6];
    OutputReal[1] = InputReal[0] - InputReal[4] - InputImag[2] + InputImag[6];
    OutputImag[1] = InputImag[0] - InputImag[4] + InputReal[2] - InputReal[6];
    OutputReal[2] = InputReal[0] + InputReal[4] - InputReal[2] - InputReal[6];
    OutputImag[2] = InputImag[0] + InputImag[4] - InputImag[2] - InputImag[6];
    OutputReal[3] = InputReal[0] - InputReal[4] + InputImag[2] - InputImag[6];
    OutputImag[3] = InputImag[0] - InputImag[4] - InputReal[2] + InputReal[6];
   // odd position input do 4 point FFT
    OutputReal[4] = InputReal[1] + InputReal[5] + InputReal[3] + InputReal[7];
    OutputImag[4] = InputImag[1] + InputImag[5] + InputImag[3] + InputImag[7];
    OutputReal[5] = InputReal[1] - InputReal[5] - InputImag[3] + InputImag[7];
    OutputImag[5] = InputImag[1] - InputImag[5] + InputReal[3] - InputReal[7];
    OutputReal[6] = InputReal[1] + InputReal[5] - InputReal[3] - InputReal[7];
    OutputImag[6] = InputImag[1] + InputImag[5] - InputImag[3] - InputImag[7];
    OutputReal[7] = InputReal[1] - InputReal[5] + InputImag[3] - InputImag[7];
    OutputImag[7] = InputImag[1] - InputImag[5] - InputReal[3] + InputReal[7];
	// butterfly calculation
	BUTTERFLY(4, OutputReal  , OutputImag  ,  65536,      0);
	BUTTERFLY(4, OutputReal+1, OutputImag+1,  46341, -46341);
	BUTTERFLY(4, OutputReal+2, OutputImag+2,      0, -65536);
	BUTTERFLY(4, OutputReal+3, OutputImag+3, -46341, -46341);
    // scale result to prevent overflow
    for (i = 0; i < 8; i ++)
    {
        OutputReal[i] >>= 3;
        OutputImag[i] >>= 3;
    }
}

//*************** Calculate 4 quadrant atan value ****************
//* The atan calculation use CORDIC algorithm
//* Result has gain of 65536/2PI radian
// Parameters:
//   x: real part of complex value
//   y: imaginary part of complex value
//   mode: 0 for 2 quadrant, 1 for 4 quadrant
// Return value:
//   4 quadrant atan value with range -32768~32767
int CordicAtan(int x, int y, int mode)
{
	short result;

	if (x == 0 && y == 0)
		return 0;

	// left shift x and y to 16bit
	while ((((x & 0xc000) == 0xc000) || ((x & 0xc000) == 0)) && (((y & 0xc000) == 0xc000) || ((y & 0xc000) == 0)))
	{
		x <<= 1;
		y <<= 1;
	}
	x >>= 2;
	y >>= 2;

	// 4 quadrant atan
	if (x >= 0)
		result = Rotate(x, y);
	else
		result = (mode ? 0x8000 : 0) - Rotate(-x, y);

	return (int)result;
}

#define FRACTION_BITS		14
static const int tan_table[15] = {
	0x2000, 0x12e4, 0x9fb, 0x511, 0x28b, 0x146, 0xa3, 0x51, 0x29, 0x14, 0xa, 0x5, 0x3, 0x1, 0x1
};

//*************** Iteration rotate to calculate atan value ****************
// Parameters:
//   x: real part of complex value
//   y: imaginary part of complex value
// Return value:
//   2 quadrant atan value with range -16384~16383
int Rotate(int x, int y)
{
	int i;
	int partial_result, temp_x;
	int result_acc;

	result_acc = 0;
	temp_x = x;
	for (i = 0; i < FRACTION_BITS; i ++)
	{
		partial_result = tan_table[i];
		temp_x = x;
		if (y >= 0)
		{
			x += (y >> i);
			y -= (temp_x >> i);
			result_acc += partial_result;
		}
		else
		{
			x -= (y >> i);
			y += (temp_x >> i);
			result_acc -= partial_result;
		}
	}

	return result_acc;
}

//*************** Search peak power position and surrouding power values ****************
//* When using coherent result (FftNumber == 1)
// Parameters:
//   NoncohBuffer: array of noncoherent sum result (with size NONCOH_BUF_LEN)
//   SearchResult: pointer to search result structure
// Return value:
//   none
void SearchPeakCoh(int NoncohBuffer[], PSEARCH_PEAK_RESULT SearchResult)
{
	int i;
	int MaxCorPos = 0;
	int MaxPower = 0;

	for (i = 0; i < CORRELATOR_NUM; i ++)
	{
		if (MaxPower < NoncohBuffer[i])
		{
			MaxPower = NoncohBuffer[i];
			MaxCorPos = i;
		}
	}
	SearchResult->CorDiff = MaxCorPos - 3;
	SearchResult->PeakPower = MaxPower;
	// if early/late at edge, use the power value at the other side
	SearchResult->EarlyPower = (MaxCorPos > 0) ? NoncohBuffer[MaxCorPos-1] : NoncohBuffer[MaxCorPos+1];
	SearchResult->LatePower = (MaxCorPos < 6) ? NoncohBuffer[MaxCorPos+1] : NoncohBuffer[MaxCorPos-1];
	// power to amplitude
	SearchResult->PeakPower = IntSqrt(SearchResult->PeakPower);
	SearchResult->EarlyPower = IntSqrt(SearchResult->EarlyPower);
	SearchResult->LatePower = IntSqrt(SearchResult->LatePower);
}

//*************** Search peak power position and surrouding power values ****************
//* When using FFT
// Parameters:
//   NoncohBuffer: array of noncoherent sum result (with size NONCOH_BUF_LEN)
//   SearchResult: pointer to search result structure
// Return value:
//   none
void SearchPeakFft(int NoncohBuffer[], PSEARCH_PEAK_RESULT SearchResult)
{
	int i, j;
	int MaxCorPos = 0, MaxBinPos = 0;
	int MaxPower = 0, *MaxPowerPos = NoncohBuffer;

	for (i = 0; i < CORRELATOR_NUM; i ++)
	{
		for (j = 0; j < MAX_BIN_NUM; j ++)
			if (MaxPower < *(NoncohBuffer ++))
			{
				MaxPowerPos = NoncohBuffer - 1;	//minus 1 because NoncohBuffer has already increased
				MaxPower = *MaxPowerPos;
				MaxCorPos = i;
				MaxBinPos = j;
			}
	}
	SearchResult->CorDiff = MaxCorPos - 3;	// after remove cor0, now peak position is 3
	SearchResult->FreqBinDiff = MAX_BIN_NUM / 2 - MaxBinPos;
	SearchResult->PeakPower = MaxPower;
	// if early/late/left/right at edge, use the power value at the other side
	SearchResult->EarlyPower = (MaxCorPos > 0) ? *(MaxPowerPos - MAX_BIN_NUM) : *(MaxPowerPos + MAX_BIN_NUM);
	SearchResult->LatePower = (MaxCorPos < 6) ? *(MaxPowerPos + MAX_BIN_NUM) : *(MaxPowerPos - MAX_BIN_NUM);
	SearchResult->LeftBinPower = (MaxBinPos > 0) ? *(MaxPowerPos - 1) : *(MaxPowerPos + 1);
	SearchResult->RightBinPower = (MaxBinPos < MAX_BIN_NUM) ? *(MaxPowerPos + 1) : *(MaxPowerPos - 1);
	// power to amplitude
	SearchResult->PeakPower = IntSqrt(SearchResult->PeakPower);
	SearchResult->EarlyPower = IntSqrt(SearchResult->EarlyPower);
	SearchResult->LatePower = IntSqrt(SearchResult->LatePower);
	SearchResult->LeftBinPower = IntSqrt(SearchResult->LeftBinPower);
	SearchResult->RightBinPower = IntSqrt(SearchResult->RightBinPower);
}

//*************** PLL/FLL/DLL loop filter ****************
//* update carrier frequency and code frequency acccording to dicriminator output
// Parameters:
//   ChannelState: Pointer to channel state structure
// Return value:
//   none
void DoTrackingLoop(PCHANNEL_STATE ChannelState)
{
	int k1, k2, k3;
	PSTATE_BUFFER StateBuffer = &(ChannelState->StateBufferCache);
	int CarrierFreq, CodeFreq;

	// return if no tracking loop update
	if ((ChannelState->State & TRACKING_UPDATE) == 0)
		return;
	// FLL update
	if (ChannelState->State & TRACKING_UPDATE_FLL)
	{
		k1 = ChannelState->fll_k1; k2 = ChannelState->fll_k2;
		ChannelState->FrequencyAcc += ChannelState->FrequencyDiff;
		ChannelState->CarrierFreqBase += ((k1 * ChannelState->FrequencyDiff + k2 * ChannelState->FrequencyAcc / 4) >> 13);
		CarrierFreq = ChannelState->CarrierFreqBase;
		if (ChannelState->FLD == 100 && ChannelState->LoseLockCounter == 0)
			ChannelState->CarrierFreqSave = ChannelState->CarrierFreqBase;
//		printf("SV%02d FLL Doppler = %5d %5d\n", ChannelState->Svid, (int)(((S64)ChannelState->CarrierFreqBase * SAMPLE_FREQ) >> 32) - IF_FREQ, (int)(((S64)CarrierFreq * SAMPLE_FREQ) >> 32) - IF_FREQ);
	}
	// DLL update
	if (ChannelState->State & TRACKING_UPDATE_DLL)
	{
		k1 = ChannelState->dll_k1; k2 = ChannelState->dll_k2;
		ChannelState->DelayAcc += ChannelState->DelayDiff;
		CodeFreq = ChannelState->CodeFreqBase - ((k1 * ChannelState->DelayDiff + k2 * ChannelState->DelayAcc) >> 15);
		STATE_BUF_SET_CODE_FREQ(StateBuffer, CodeFreq);
		if (ChannelState->DLD == 100 && ChannelState->LoseLockCounter == 0)
			ChannelState->CodeFreqSave = CodeFreq;
	}
	// PLL update
	if (ChannelState->State & TRACKING_UPDATE_PLL)
	{
		k1 = ChannelState->pll_k1; k2 = ChannelState->pll_k2; k3 = ChannelState->pll_k3;
		ChannelState->PhaseAcc += ChannelState->PhaseDiff;
		ChannelState->CarrierFreqBase += ((k2 * ChannelState->PhaseDiff + k3 * ChannelState->PhaseAcc) >> 15);
		CarrierFreq = ChannelState->CarrierFreqBase + ((k1 * ChannelState->PhaseDiff) >> 13);
		if (ChannelState->PLD == 100 && ChannelState->LoseLockCounter == 0)
		{
			ChannelState->CarrierFreqSave = CarrierFreq;//ChannelState->CarrierFreqBase;
		}
//		printf("SV%02d PLL Doppler = %5d %5d\n", ChannelState->Svid, (int)(((S64)ChannelState->CarrierFreqBase * SAMPLE_FREQ) >> 32) - IF_FREQ, (int)(((S64)CarrierFreq * SAMPLE_FREQ) >> 32) - IF_FREQ);
//		if (ChannelState->Svid == 4)
//			printf("SV%02d FREQ=%10d\n", ChannelState->Svid, CarrierFreq);
	}
	if (ChannelState->State & (TRACKING_UPDATE_PLL | TRACKING_UPDATE_FLL))
		STATE_BUF_SET_CARRIER_FREQ(StateBuffer, CarrierFreq);
	ChannelState->State |= STATE_CACHE_FREQ_DIRTY;
	ChannelState->State &= ~TRACKING_UPDATE;	// clear tracking update flags
}

//*************** Calculate loop filter coefficients ****************
// Parameters:
//   ChannelState: Pointer to channel state structure
//   CurTrackingConfig: Pointer to tracking config structure
// Return value:
//   none
void CalculateLoopCoefficients(PCHANNEL_STATE ChannelState, PTRACKING_CONFIG CurTrackingConfig)
{
	int Tc, T;	// coherent time and update interval
	int BnT, Order;
	int Coef[3];	// integer coefficients

	Tc = CurTrackingConfig->CoherentNumber;
	T = Tc * CurTrackingConfig->FftNumber * CurTrackingConfig->NonCohNumber;
	// determine PLL coefficients
	if ((CurTrackingConfig->BandWidthPLL16x & 0xffff) > 0)
	{
		Order = CurTrackingConfig->BandWidthPLL16x >> 16;
		BnT = ((CurTrackingConfig->BandWidthPLL16x & 0xffff) * Tc + 5) / 10;	// 16x of 0.01 BnT
		GetCoefficients(BnT, Order, Coef);
		ChannelState->pll_k1 = (Coef[0] + (Tc << 3)) / (Tc << 4);	// scale 2^29/fs/Tc = (2^33/fs)/(16Tc)
		ChannelState->pll_k2 = (Coef[1] + (Tc << 1)) / (Tc << 2);	// scale 2^31/fs/Tc = (2^33/fs)/(4Tc)
		ChannelState->pll_k3 = (Coef[2] + (Tc << 1)) / (Tc << 2);	// scale 2^31/fs/Tc = (2^33/fs)/(4Tc)
	}
	else
		ChannelState->pll_k1 = ChannelState->pll_k2 = ChannelState->pll_k3 = 0;
	// determine FLL coefficients
	if ((CurTrackingConfig->BandWidthFLL16x & 0xffff) > 0)
	{
		Order = CurTrackingConfig->BandWidthFLL16x >> 16;
		BnT = ((CurTrackingConfig->BandWidthFLL16x & 0xffff) * T + 5) / 10;	// 16x of 0.01 BnT
		GetCoefficients(BnT, Order, Coef);
		ChannelState->fll_k1 = (Coef[0] + (Tc << 3)) / (Tc << 4);	// scale 2^29/fs/Tc = (2^33/fs)/(16Tc)
		ChannelState->fll_k2 = (Coef[1] + (Tc << 1)) / (Tc << 2);	// scale 2^31/fs/Tc = (2^33/fs)/(4Tc)
	}
	else
		ChannelState->fll_k1 = ChannelState->fll_k2 = 0;
	// determine DLL coefficients
	if ((CurTrackingConfig->BandWidthDLL16x & 0xffff) > 0)
	{
		Order = CurTrackingConfig->BandWidthDLL16x >> 16;
		BnT = ((CurTrackingConfig->BandWidthDLL16x & 0xffff) * T + 5) / 10;	// 16x of 0.01 BnT
		GetCoefficients(BnT, Order, Coef);
		ChannelState->dll_k1 = (Coef[0] + (T >> 1)) / T;	// scale 2^33/fs/T = (2^33/fs)/(T)
		ChannelState->dll_k2 = (Coef[1] + (T >> 1)) / T;	// scale 2^33/fs/T = (2^33/fs)/(T)
	}
	else
		ChannelState->dll_k1 = ChannelState->dll_k2 = 0;
}

//*************** Interpolate loop filter coefficients ****************
// Parameters:
//   BnT16x: 16x of 0.01 BnT
//   Order: order of loop filter
//   Coef: interpolated value of coefficients from LUT
// Return value:
//   none
void GetCoefficients(int BnT16x, int Order, int Coef[3])
{
	int index, frac;
	int HighSegment = 0;
	int (*p2)[2];
	int (*p3)[3];

	if (BnT16x < 16)	// minimum 0.01
		index = frac = 0;
	else if (BnT16x <= 16 * 10)	// 0.01 to 0.1
	{
		index = (BnT16x >> 4) - 1;
		frac = BnT16x & 0xf;
	}
	else if (BnT16x <= 16 * 35)	// 0.1 to 0.35
	{
		index = (BnT16x / 80) - 2;
		frac = (BnT16x % 80) / 5;
		HighSegment = 1;
	}
	else	// clip to 0.35
	{
		index = 5;
		frac = 0;
	}

	switch (Order)
	{
	case 1:
		if (HighSegment)	// clip to 0.1
		{
			index = 9;
			frac = 0;
		}
		Coef[0] = FilterCoef1[index];
		if (frac)
			Coef[0] += ((FilterCoef1[index+1] - FilterCoef1[index]) * frac + 8) >> 4;
		Coef[1] = Coef[2] = 0;
		break;
	case 2:
		p2 = HighSegment ? FilterCoef21 : FilterCoef20;
		Coef[0] = p2[index][0];
		Coef[1] = p2[index][1];
		if (frac)
		{
			Coef[0] += ((p2[index+1][0] - p2[index][0]) * frac + 8) >> 4;
			Coef[1] += ((p2[index+1][1] - p2[index][1]) * frac + 8) >> 4;
		}
		Coef[2] = 0;
		break;
	case 3:
		p3 = HighSegment ? FilterCoef31 : FilterCoef30;
		Coef[0] = p3[index][0];
		Coef[1] = p3[index][1];
		Coef[2] = p3[index][2];
		if (frac)
		{
			Coef[0] += ((p3[index+1][0] - p3[index][0]) * frac + 8) >> 4;
			Coef[1] += ((p3[index+1][1] - p3[index][1]) * frac + 8) >> 4;
			Coef[2] += ((p3[index+1][2] - p3[index][2]) * frac + 8) >> 4;
		}
		break;
	}
}

void AdjustLockIndicator(int *LockIndicator, int Adjustment)
{
	Adjustment = ABS(Adjustment);

	if (Adjustment < 8)	// 000: 6, 001: 4, 01x: 2, 1xx: 1
		Adjustment = (Adjustment & 4 ) ? 1 : ((Adjustment & 2) ? 2 : (6 - Adjustment * 2));
	else
		Adjustment = -(Adjustment >> 3);
	
	*LockIndicator += Adjustment;
	if (*LockIndicator > 100)
		*LockIndicator = 100;
	else if (*LockIndicator < 0)
		*LockIndicator = 0;
}
