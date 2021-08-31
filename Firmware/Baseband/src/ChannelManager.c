//----------------------------------------------------------------------
// ChannelManager.c:
//   Channel management functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "RegAddress.h"
#include "BBDefines.h"
#include "HWCtrl.h"
#include "InitSet.h"
#include "FirmwarePortal.h"
#include "TaskQueue.h"
#include "ChannelManager.h"

CHANNEL_STATE ChannelStateArray[TOTAL_CHANNEL_NUMBER];

static void ProcessCohData(PCHANNEL_STATE ChannelState);
static void CohBufferFft(PCHANNEL_STATE ChannelState);
static void CohBufferAcc(PCHANNEL_STATE ChannelState);
static void FFT8(int InputReal[8], int InputImag[8], int OutputReal[8], int OutputImag[8]);
static int AmplitudeJPL(int Real, int Imag);
static int CordicAtan(int x, int y, int mode);
static int Rotate(int x, int y);
static void CalcDiscriminator(PCHANNEL_STATE ChannelState, unsigned int Method);
static void SearchPeakCoh(int NoncohBuffer[], PSEARCH_PEAK_RESULT SearchResult);
static void SearchPeakFft(int NoncohBuffer[], PSEARCH_PEAK_RESULT SearchResult);
static void DoTrackingLoop(PCHANNEL_STATE ChannelState);
static int StageDetermination(PCHANNEL_STATE ChannelState);
static void SwitchTrackingStage(PCHANNEL_STATE ChannelState, unsigned int TrackingStage);
static void CollectBitSyncData(PCHANNEL_STATE ChannelState);
static void DecodeDataStream(PCHANNEL_STATE ChannelState);
static int BitSyncTask(void *Param);

//*************** Initialize channel state structure ****************
// Parameters:
//   pChannel: pointer to channel state
// Return value:
//   none
void InitChannel(PCHANNEL_STATE pChannel)
{
	pChannel->CoherentNumber = 1;
	pChannel->FftNumber = 5;
	pChannel->NonCohNumber= 2;
	pChannel->FftCount = 0;
	pChannel->NonCohCount = 0;
}

//*************** Configure channel state buffer values ****************
// Parameters:
//   pChannel: pointer to channel state
//   Doppler: Doppler in Hz
//   CodePhase16x: 16 times of code phase
// Return value:
//   none
void ConfigChannel(PCHANNEL_STATE pChannel, int Doppler, int CodePhase16x)
{
	PSTATE_BUFFER pStateBuffer = &(pChannel->StateBufferCache);
	int StartPhase = CodePhase16x / 16;

	memset(pStateBuffer, 0, sizeof(STATE_BUFFER));

	// config DWORDs
	if (pChannel->FreqID == FREQ_L1CA)
		pChannel->CarrierFreqBase = CARRIER_FREQ(Doppler);
	else
		pChannel->CarrierFreqBase = CARRIER_FREQ_BOC(Doppler);
	pChannel->CodeFreqBase = CODE_FREQ(Doppler);
	STATE_BUF_SET_CARRIER_FREQ(pStateBuffer, pChannel->CarrierFreqBase);
	STATE_BUF_SET_CODE_FREQ(pStateBuffer, pChannel->CodeFreqBase);
	STATE_BUF_SET_CORR_CONFIG(pStateBuffer, 1023, 0, 0, 0, 0, 0);
	STATE_BUF_SET_NH_CONFIG(pStateBuffer, 0, 0);
	STATE_BUF_SET_COH_CONFIG(pStateBuffer, 1, 1, 0, 0);
	// PRN config
	if (pChannel->FreqID == FREQ_L1CA)
	{
		StartPhase %= 1023;	// remnant of code cycle
		STATE_BUF_SET_PRN_L1CA(pStateBuffer, pChannel->Svid, StartPhase);
		pChannel->DataStream.TotalAccTime = 20;	// 20ms for GPS L1CA
		pChannel->State |= DATA_STREAM_1BIT;
	}
	else if (pChannel->FreqID == FREQ_E1)
	{
		StartPhase %= 4092;	// remnant of code cycle
		STATE_BUF_SET_PRN_E1(pStateBuffer, pChannel->Svid, StartPhase);
		pChannel->DataStream.TotalAccTime = 4;	// 4ms for Galileo E1
		pChannel->State |= DATA_STREAM_4BIT;
	}
	else if (pChannel->FreqID == FREQ_B1C)
	{
		StartPhase %= 10230;	// remnant of code cycle
		STATE_BUF_SET_PRN_B1C(pStateBuffer, pChannel->Svid, StartPhase);
		pChannel->DataStream.TotalAccTime = 10;	// 10ms for BDS B1C
		pChannel->State |= DATA_STREAM_8BIT;
	}
	else if (pChannel->FreqID == FREQ_L1C)
	{
		StartPhase %= 10230;	// remnant of code cycle
		STATE_BUF_SET_PRN_L1C(pStateBuffer, pChannel->Svid, StartPhase);
		pChannel->DataStream.TotalAccTime = 10;	// 10ms for GPS L1C
		pChannel->State |= DATA_STREAM_8BIT;
	}
	// status fields
	STATE_BUF_SET_CODE_PHASE(pStateBuffer, (CodePhase16x << 29));
	STATE_BUF_SET_DUMP_COUNT(pStateBuffer, (StartPhase % 1023));
	STATE_BUF_SET_NH_COUNT(pStateBuffer, (StartPhase / 1023));
	STATE_BUF_SET_CODE_SUB_PHASE(pStateBuffer, (CodePhase16x >> 3));
	pChannel->State |= STATE_CACHE_DIRTY;	// set cache dirty
	pChannel->SkipCount = 1;	// skip the first coherent sum result (PRN not ready until code edge for L1CA)
	SwitchTrackingStage(pChannel, STAGE_PULL_IN);	// switch to pull-in stage
}

//*************** Synchronize state buffer cache value to HW ****************
//* according to different cache dirty field, different value will be written
// Parameters:
//   ChannelState: pointer to channel state structure
// Return value:
//   none
void SyncCacheWrite(PCHANNEL_STATE ChannelState)
{
	if ((ChannelState->State & STATE_CACHE_DIRTY) == 0)	// if no need to sync cache, return
		return;
	if ((ChannelState->State & STATE_CACHE_DIRTY) == STATE_CACHE_DIRTY)	// if entire cache need to be sync
		SaveMemory((U32 *)(ChannelState->StateBufferHW), (U32 *)(&ChannelState->StateBufferCache), sizeof(U32) * 16);
	else	// partial of cache to sync
	{
		if (ChannelState->State & STATE_CACHE_FREQ_DIRTY)	// update carrier and code frequency
		{
			SetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierFreq)), ChannelState->StateBufferCache.CarrierFreq);
			SetRegValue((U32)(&(ChannelState->StateBufferHW->CodeFreq)), ChannelState->StateBufferCache.CodeFreq);
		}
		if (ChannelState->State & STATE_CACHE_CONFIG_DIRTY)	// update CorrConfig, NHConfig and CohConfig
		{
			SetRegValue((U32)(&(ChannelState->StateBufferHW->CorrConfig)), ChannelState->StateBufferCache.CorrConfig);
			SetRegValue((U32)(&(ChannelState->StateBufferHW->NHConfig)),   ChannelState->StateBufferCache.NHConfig);
			SetRegValue((U32)(&(ChannelState->StateBufferHW->CohConfig)),  ChannelState->StateBufferCache.CohConfig);
		}
	}
	ChannelState->State &= ~STATE_CACHE_DIRTY;	// clear cache dirty flags
}

//*************** Synchronize state buffer cache value from HW ****************
// Parameters:
//   ChannelState: pointer to channel state structure
//   ReadContent: whether coherent data or status or both synchronize to cache
// Return value:
//   none
void SyncCacheRead(PCHANNEL_STATE ChannelState, int ReadContent)
{
	if (ReadContent & SYNC_CACHE_READ_DATA)
		LoadMemory(ChannelState->StateBufferCache.CoherentSum, (U32 *)(ChannelState->StateBufferHW) + 24, sizeof(U32) * 8);	// copy coherent result to cache
	if (ReadContent & SYNC_CACHE_READ_STATUS)
		LoadMemory(&(ChannelState->StateBufferCache.PrnCount), (U32 *)(ChannelState->StateBufferHW) + 7, sizeof(U32) * 6);	// copy channel status fields
}

//*************** Process coherent sum interrupt of a channel ****************
// Parameters:
//   ChannelID: physical channel ID (start from 0)
//   OverwriteProtect: whether this channel has overwrite protection
// Return value:
//   none
void ProcessCohSum(int ChannelID, unsigned int OverwriteProtect)
{
	PCHANNEL_STATE ChannelState = &ChannelStateArray[ChannelID];
	int CurrentCor, CohCount;
	int CompleteData;
//	int i;
	U32 *CohBuffer;
//	S16 CohResultI, CohResultQ;

	ChannelState->StateBufferCache.CorrState = GetRegValue((U32)(&(ChannelState->StateBufferHW->CorrState)));	// get CorrState to check CurrentCor
	CurrentCor = STATE_BUF_GET_CUR_CORR(&(ChannelState->StateBufferCache));
	CohCount = STATE_BUF_GET_COH_COUNT(&(ChannelState->StateBufferCache));
	CompleteData = (ChannelState->PendingCount == 0 && CurrentCor != 0) ? (CohCount == 0 && CurrentCor == 1) : 1;

	SyncCacheRead(ChannelState, SYNC_CACHE_READ_DATA);	// copy coherent result to cache
	CohBuffer = ChannelState->CohBuffer + ChannelState->FftCount * CORRELATOR_NUM;
	if (CompleteData)
	{
		memcpy(ChannelState->PendingCoh + ChannelState->PendingCount, ChannelState->StateBufferCache.CoherentSum + ChannelState->PendingCount, sizeof(U32) * (8 - ChannelState->PendingCount));	// concatinate data
		memcpy(CohBuffer, ChannelState->PendingCoh + 1, sizeof(U32) * CORRELATOR_NUM);	// copy coherent result (except Cor0) to coherent buffer
		ChannelState->PendingCount = 0;
	}
	if (CompleteData)	// for the case of all 8 coherent result get, do coherent sum result process
		ProcessCohData(ChannelState);
	if (CurrentCor && (CohCount == (ChannelState->CoherentNumber - 1)))
	{
		memcpy(ChannelState->PendingCoh, ChannelState->StateBufferCache.CoherentSum, sizeof(U32) * CurrentCor);
		ChannelState->PendingCount = CurrentCor;
	}
}

//*************** Process coherent data of a channel ****************
// Parameters:
//   ChannelID: physical channel ID (start from 0)
//   OverwriteProtect: whether this channel has overwrite protection
// Return value:
//   none
void ProcessCohData(PCHANNEL_STATE ChannelState)
{
	ChannelState->TrackingTime += ChannelState->CoherentNumber;	// accumulate tracking time
//	printf("track time %d\n", ChannelState->TrackingTime);
	if (ChannelState->SkipCount > 0)	// skip coherent result for following process
	{
		ChannelState->SkipCount --;
		return;
	}
	
	ChannelState->FftCount ++;
//	printf("SV%2d I/Q=%6d %6d\n", ChannelState->Svid, (S16)(ChannelState->StateBufferCache.CoherentSum[4] >> 16), (S16)(ChannelState->StateBufferCache.CoherentSum[4] & 0xffff));

	// perform PLL
	if ((ChannelState->State & STAGE_MASK) == STAGE_TRACK)	// tracking stage uses PLL (change to more flexible condition in the future)
		CalcDiscriminator(ChannelState, TRACKING_UPDATE_PLL);

	// do FFT and non-coherent accumulation
	if (ChannelState->FftCount == ChannelState->FftNumber)
	{
		ChannelState->FftCount = 0;
		if (ChannelState->FftNumber > 1)
			CohBufferFft(ChannelState);
		else
			CohBufferAcc(ChannelState);
	}

	// do tracking loop
	if (ChannelState->State & STATE_TRACKING_LOOP)
		DoTrackingLoop(ChannelState);

//	if ((ChannelState->State & STAGE_MASK) == STAGE_TRACK)
//		printf("T=%4d I/Q=%6d %6d\n", ChannelState->TrackingTime, (S16)(ChannelState->StateBufferCache.CoherentSum[4] >> 16), (S16)(ChannelState->StateBufferCache.CoherentSum[4] & 0xffff));
	
	// collect correlation result for bit sync if in bit sync stage
	if ((ChannelState->State & STAGE_MASK) == STAGE_BIT_SYNC)
		CollectBitSyncData(ChannelState);
	// do data decode at tracking stage
	else if (((ChannelState->State & STAGE_MASK) >= STAGE_TRACK) && ((ChannelState->State & DATA_STREAM_MASK) != 0))
		DecodeDataStream(ChannelState);

	// determine whether tracking stage switch is needed
	StageDetermination(ChannelState);

/*	printf("SV%2d", ChannelState->Svid);
	for (i = 0; i < 8; i ++)
	{
		CohResultI = (S16)(ChannelState->StateBufferCache.CoherentSum[i] >> 16);
		CohResultQ = (S16)(ChannelState->StateBufferCache.CoherentSum[i] & 0xffff);
		printf(" %5d %5d,", CohResultI, CohResultQ);
	}
	printf("\n");*/
}

//*************** Compose baseband measurement and data stream ****************
// Parameters:
//   ChannelID: physical channel ID (start from 0)
//   Measurement: pointer to baseband measurement structure
//   DataBuffer: buffer to store data stream
// Return value:
//   number of U32 data filled into DataBuffer
int ComposeMeasurement(int ChannelID, PBB_MEASUREMENT Measurement, U32 *DataBuffer)
{
	PCHANNEL_STATE ChannelState = &ChannelStateArray[ChannelID];
	PSTATE_BUFFER StateBuffer = &(ChannelState->StateBufferCache);
	int WordNumber;
	int CohCount, CurrentCor;

	LoadMemory(&(ChannelState->StateBufferCache.PrnCount), (U32 *)(ChannelState->StateBufferHW) + 7, sizeof(U32) * 6);	// copy channel status fields
	memcpy((void *)Measurement, (void *)ChannelState, sizeof(U32) * 3);	// copy first elements of structure

	Measurement->CodeFreq = STATE_BUF_GET_CODE_FREQ(StateBuffer);
	Measurement->CodeNCO = STATE_BUF_GET_CODE_PHASE(StateBuffer);
	if (ChannelState->FreqID == FREQ_L1CA)
		Measurement->CodeCount = StateBuffer->PrnCount >> 14;
	else if (ChannelState->FreqID == FREQ_E1)
		Measurement->CodeCount = StateBuffer->PrnCount - (StateBuffer->PrnCount >> 10);	// 4MSB*1023 + 10LSB = (4MSB*1024+10LSB) - 4MSB
	else// if (ChannelState->FreqID == FREQ_B1C) or (ChannelState->FreqID == FREQ_L1C)
		Measurement->CodeCount = StateBuffer->PrnCount >> 14;
	Measurement->CodeCount = (Measurement->CodeCount << 1) + STATE_BUF_GET_CODE_SUB_PHASE(StateBuffer) - 4;	// compensate 4 correlator interval to align to COR4
	CohCount = STATE_BUF_GET_COH_COUNT(StateBuffer);
	// check whether CurrentCor != 0, if so, CohCount has 1 lag to Cor0
	CurrentCor = STATE_BUF_GET_CUR_CORR(&(ChannelState->StateBufferCache));
	if (CurrentCor)
		CohCount ++;
	Measurement->CodeCount += (ChannelState->DataStream.CurrentAccTime + CohCount) * 2046;

	Measurement->CarrierFreq = STATE_BUF_GET_CARRIER_FREQ(StateBuffer);
	Measurement->CarrierNCO = STATE_BUF_GET_CARRIER_PHASE(StateBuffer);
	Measurement->CarrierCount = STATE_BUF_GET_CARRIER_COUNT(StateBuffer);
	// fill data stream here
	Measurement->DataNumber = ChannelState->DataStream.DataCount;
	Measurement->DataStreamAddr = DataBuffer;
	WordNumber = (Measurement->DataNumber + 31) / 32;
	memcpy(Measurement->DataStreamAddr, ChannelState->DataStream.DataBuffer, sizeof(U32) * Measurement->DataNumber);
	if ((Measurement->DataNumber & 0x1f) != 0)	// last DWORD not fill to the end
		DataBuffer[WordNumber-1] <<= (32 - (Measurement->DataNumber & 0x1f));	// shift to MSB
	ChannelState->DataStream.DataCount= 0;

	Measurement->CN0 = 4650;
	Measurement->LockIndicator = 100;

	return WordNumber;
}

//*************** Do 8 point FFT on coherent buffer and accumulate to noncoherent buffer ****************
// Parameters:
//   ChannelState: pointer to channel state buffer
// Return value:
//   none
static void CohBufferFft(PCHANNEL_STATE ChannelState)
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
		// accumulate power, move 0 frequency bin in middle
		for (j = 0; j < MAX_BIN_NUM/2; j ++)
			ChannelState->NoncohBuffer[i * MAX_BIN_NUM + j] += AmplitudeJPL(FftResultReal[j + MAX_BIN_NUM/2], FftResultImag[j + MAX_BIN_NUM/2]);
		for (; j < MAX_BIN_NUM; j ++)
			ChannelState->NoncohBuffer[i * MAX_BIN_NUM + j] += AmplitudeJPL(FftResultReal[j - MAX_BIN_NUM/2], FftResultImag[j - MAX_BIN_NUM/2]);
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
		ChannelState->NoncohBuffer[i] += AmplitudeJPL(CohReal, CohImag);
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

//*************** Calculate amplitude (absolute value) of a complex number ****************
//* The algorithm uses JPL method
// Parameters:
//   Real: real part of complex value
//   Imag: imaginary part of complex value
// Return value:
//   absolute value
int AmplitudeJPL(int Real, int Imag)
{
	int Amplitude;

	Real = (Real >= 0) ? Real : -Real;
	Imag = (Imag >= 0) ? Imag : -Imag;
	if (Real < Imag)
	{
		Real ^= Imag; Imag ^= Real; Real ^= Imag;	// swap
	}
	if (Real > Imag * 3)
		Amplitude = Real + (Imag >> 3);
	else
		Amplitude = Real - (Real >> 3) + (Imag >> 1);

	return Amplitude;
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

	// for FLL and DLL, search for peak power
	if (Method & (TRACKING_UPDATE_FLL | TRACKING_UPDATE_DLL))
	{
		if (ChannelState->FftNumber == 1)
			SearchPeakCoh(ChannelState->NoncohBuffer, &SearchResult);
		else
			SearchPeakFft(ChannelState->NoncohBuffer, &SearchResult);
	}
	if (Method & TRACKING_UPDATE_FLL)
	{
		Denominator = 2 * SearchResult.PeakPower - SearchResult.LeftBinPower - SearchResult.RightBinPower;
		Numerator = SearchResult.LeftBinPower - SearchResult.RightBinPower;
		// atan((L-R)/(2P-R-L))
		ChannelState->FrequencyDiff = (CordicAtan(Denominator, Numerator, 0) >> 1);
		ChannelState->FrequencyDiff += (SearchResult.FreqBinDiff << 13);
	}
	if (Method & TRACKING_UPDATE_DLL)
	{
//		printf("EPL = %5d %5d %5d\n", SearchResult.EarlyPower, SearchResult.PeakPower, SearchResult.LatePower);
		Denominator = 2 * SearchResult.PeakPower - SearchResult.EarlyPower - SearchResult.LatePower;
		Numerator = SearchResult.EarlyPower - SearchResult.LatePower;
		// (E-L)/(2P-E-L))
		ChannelState->DelayDiff = -((Numerator << 13) / Denominator);
		ChannelState->DelayDiff += (SearchResult.CorDiff << 14);
	}
	if (Method & TRACKING_UPDATE_PLL)
	{
		CorResutReal = (S16)(ChannelState->StateBufferCache.CoherentSum[4] >> 16);
		CorResultImag = (S16)(ChannelState->StateBufferCache.CoherentSum[4] & 0xffff);
		ChannelState->PhaseDiff = CordicAtan(CorResutReal, CorResultImag, 0);
	}
	ChannelState->State |= Method;
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
}

//*************** PLL/FLL/DLL loop filter ****************
//* update carrier frequency and code frequency acccording to dicriminator output
// Parameters:
//   ChannelState: Pointer to channel state structure
// Return value:
//   none
void DoTrackingLoop(PCHANNEL_STATE ChannelState)
{
	// FLL: Tc=1, T=1x5x2=10, Bn=5Hz, BnT=0.05
	// k1 = 0.1164, k2 = 0.007191
	int fll_k1 = 15194, fll_k2 = 3755;
	// DLL: Tc=1, T=1x5x2=10, Bn=5Hz, BnT=0.05
	// k1 = 0.1164, k2 = 0.007191
	int dll_k1 = 24310, dll_k2 = 1502;
	// PLL: Tc=5, T=5, Bn=20Hz, BnT=0.1
	// k1 = 0.2066, k2 = 0.02372
	int pll_k1 = 5394, pll_k2 = 2477, pll_k3 = 0;
	PSTATE_BUFFER StateBuffer = &(ChannelState->StateBufferCache);
	int CarrierFreq, CodeFreq;

	// return if no tracking loop update
	if ((ChannelState->State & TRACKING_UPDATE) == 0)
		return;
	// FLL update
	if (ChannelState->State & TRACKING_UPDATE_FLL)
	{
		ChannelState->FrequencyAcc += ChannelState->FrequencyDiff;
		ChannelState->CarrierFreqBase += ((fll_k1 * ChannelState->FrequencyDiff + fll_k2 * ChannelState->FrequencyAcc / 4) >> 13);
		CarrierFreq = ChannelState->CarrierFreqBase;
//		printf("SV%02d Doppler = %5d %5d\n", ChannelState->Svid, (int)(((S64)ChannelState->CarrierFreqBase * SAMPLE_FREQ) >> 32) - IF_FREQ, (int)(((S64)CarrierFreq * SAMPLE_FREQ) >> 32) - IF_FREQ);
	}
	// DLL update
	if (ChannelState->State & TRACKING_UPDATE_DLL)
	{
		ChannelState->DelayAcc += ChannelState->DelayDiff;
		CodeFreq = ChannelState->CodeFreqBase - ((dll_k1 * ChannelState->DelayDiff + dll_k2 * ChannelState->DelayAcc) >> 15);
		STATE_BUF_SET_CODE_FREQ(StateBuffer, CodeFreq);
	}
	// PLL update
	if (ChannelState->State & TRACKING_UPDATE_PLL)
	{
		ChannelState->PhaseAcc += ChannelState->PhaseDiff;
		ChannelState->CarrierFreqBase += ((pll_k2 * ChannelState->PhaseDiff + pll_k3 * ChannelState->PhaseAcc) >> 15);
		CarrierFreq = ChannelState->CarrierFreqBase + ((pll_k1 * ChannelState->PhaseDiff) >> 13);
//		printf("SV%02d Doppler = %5d %5d ", ChannelState->Svid, (int)(((S64)ChannelState->CarrierFreqBase * SAMPLE_FREQ) >> 32) - IF_FREQ, (int)(((S64)CarrierFreq * SAMPLE_FREQ) >> 32) - IF_FREQ);
	}
	if (ChannelState->State & (TRACKING_UPDATE_PLL | TRACKING_UPDATE_FLL))
		STATE_BUF_SET_CARRIER_FREQ(StateBuffer, CarrierFreq);
	ChannelState->State |= STATE_CACHE_FREQ_DIRTY;
	ChannelState->State &= ~TRACKING_UPDATE;	// clear tracking update flags
}

//*************** Determine whether tracking stage need to be changed ****************
//* update carrier frequency and code frequency acccording to dicriminator output
// Parameters:
//   ChannelState: Pointer to channel state structure
// Return value:
//   0 for no tracking stage change, 1 for tracking stage change
int StageDetermination(PCHANNEL_STATE ChannelState)
{
	int TrackingStage = ChannelState->State & STAGE_MASK;
	int Time;

	// determine stage switch cases before timeout
	if (TrackingStage == STAGE_BIT_SYNC && ChannelState->BitSyncResult)	// bit sync finished
	{
		if (ChannelState->BitSyncResult < 0)	// bit sync fail
			SwitchTrackingStage(ChannelState, STAGE_RELEASE);
		else if (ChannelState->BitSyncResult <= 20)	// bit sync success, adjust 
		{
			Time = (ChannelState->TrackingTime) % 20;	// remainder of TrackTime divided by 20 (0~19)
			Time = ChannelState->BitSyncResult + 20 - Time;	// gap to next bit edge
			ChannelState->SkipCount = Time % 20;	// modulo 20ms
			ChannelState->BitSyncResult = 21;	// ready to switch to tracking
			if (ChannelState->SkipCount > 0)	// if need to skip to align bit edge
			{
				ChannelState->SkipCount --;	// minus 1 because skip counting from NEXT coherent result
				return 0;
			}
		}
		if (ChannelState->BitSyncResult == 21 && ChannelState->SkipCount == 0)	// switch from bit sync to tracking
		{
			SwitchTrackingStage(ChannelState, STAGE_TRACK);
			return 1;
		}
	}
	if (ChannelState->TrackingTimeout < 0 || ChannelState->TrackingTime < ChannelState->TrackingTimeout)	// not timeout, do not switch stage
		return 0;
	if (TrackingStage == STAGE_PULL_IN)
	{
		SwitchTrackingStage(ChannelState, STAGE_BIT_SYNC);
	}
	return 1;
}

//*************** Switch tracking stage of a tracking channel ****************
// Parameters:
//   ChannelState: Pointer to channel state structure
//   TrackingStage: tracking stage to be switched
// Return value:
//   none
void SwitchTrackingStage(PCHANNEL_STATE ChannelState, unsigned int TrackingStage)
{
	ChannelState->TrackingTime = 0;		// reset tracking time
	ChannelState->State &= ~STAGE_MASK;
	ChannelState->State |= TrackingStage;
	if (TrackingStage == STAGE_PULL_IN)
		ChannelState->TrackingTimeout = 200;	// wait 200ms to have FLL/DLL converge and swith to bit sync
	else if (TrackingStage == STAGE_BIT_SYNC)
	{
		ChannelState->TrackingTimeout = 1500;	// timeout for bit sync not success
		ChannelState->BitSyncData.CorDataCount = 0;
		ChannelState->BitSyncData.ChannelState = ChannelState;
		ChannelState->BitSyncData.PrevCorData = 0;	// clear previous correlation result for first round
		memset(ChannelState->ToggleCount, 0, sizeof(ChannelState->ToggleCount));
		ChannelState->BitSyncResult = 0;
	}
	else if (TrackingStage == STAGE_TRACK)
	{
		// set coherent/FFT/non-coherent number
		ChannelState->CoherentNumber = 5;
		STATE_BUF_SET_COH_NUMBER(&(ChannelState->StateBufferCache), 5);
		STATE_BUF_SET_POST_SHIFT(&(ChannelState->StateBufferCache), 2);
		ChannelState->State |= STATE_CACHE_CONFIG_DIRTY;
		ChannelState->FftNumber = 1;
		ChannelState->NonCohNumber= 2;
		ChannelState->FftCount = 0;
		ChannelState->NonCohCount = 0;
		// reset data for data decode, switch to tracking stage at epoch of bit edge
		ChannelState->DataStream.PrevReal = ChannelState->DataStream.PrevImag = ChannelState->DataStream.PrevSymbol = 0;
		ChannelState->DataStream.CurReal = ChannelState->DataStream.CurImag = 0;
		ChannelState->DataStream.DataCount = ChannelState->DataStream.CurrentAccTime = 0;
	}
}

//*************** Put 1ms correlation result in buffer and send 20 results to bit sync task ****************
//* update carrier frequency and code frequency acccording to dicriminator output
// Parameters:
//   ChannelState: Pointer to channel state structure
// Return value:
//   none
void CollectBitSyncData(PCHANNEL_STATE ChannelState)
{
	PBIT_SYNC_DATA BitSyncData = &(ChannelState->BitSyncData);

	BitSyncData->CorData[BitSyncData->CorDataCount] = ChannelState->StateBufferCache.CoherentSum[4];	// copy peak correlator result
	if (++BitSyncData->CorDataCount == 20)	// 20 correlation result, send to bit sync task
	{
		BitSyncData->TimeTag = ChannelState->TrackingTime;
		AddTaskToQueue(&BasebandTask, BitSyncTask, BitSyncData, sizeof(BIT_SYNC_DATA));
		BitSyncData->CorDataCount = 0;
		BitSyncData->PrevCorData = BitSyncData->CorData[19];	// copy last one to PrevCorData for next 20 result
	}
}

//*************** Accumulate coherent data for data decode ****************
//* update carrier frequency and code frequency acccording to dicriminator output
// Parameters:
//   ChannelState: Pointer to channel state structure
// Return value:
//   none
void DecodeDataStream(PCHANNEL_STATE ChannelState)
{
	PDATA_STREAM DataStream = &(ChannelState->DataStream);
	int DataSymbol;
	int CurIndex;

	// accumulate time and coherent data
	DataStream->CurrentAccTime += ChannelState->CoherentNumber;
	DataStream->CurReal += (S16)(ChannelState->StateBufferCache.CoherentSum[4] >> 16);
	DataStream->CurImag += (S16)(ChannelState->StateBufferCache.CoherentSum[4] & 0xffff);

//	if (ChannelState->Svid == 30)
//		printf("DATA %d %d\n", DataStream->CurrentAccTime, DataStream->TotalAccTime);
	// if accumulate time reaches symbol length, decode data symbol
	if (DataStream->CurrentAccTime >= DataStream->TotalAccTime)
	{
		if (1)	// determine by data toggle
		{
			// determine symbol toggle
			DataSymbol = DataStream->CurReal * DataStream->PrevReal + DataStream->CurImag * DataStream->PrevImag;
			DataSymbol = ((DataSymbol < 0) ? 1 : 0) ^ (DataStream->PrevSymbol);
			// store to previous values for next symbol
			DataStream->PrevSymbol = DataSymbol;
			DataStream->PrevReal = DataStream->CurReal;
			DataStream->PrevImag = DataStream->CurImag;
		}
		else	// PLL lock, determine by sign
		{
			DataSymbol = ((DataStream->CurReal < 0) ? 1 : 0);
		}
		// put into data stream buffer
		CurIndex = DataStream->DataCount / 32;
		DataStream->DataBuffer[CurIndex] <<= 1;
		DataStream->DataBuffer[CurIndex] |= DataSymbol;
		DataStream->DataCount ++;
		DataStream->CurrentAccTime = 0;
		DataStream->CurReal = DataStream->CurImag = 0;
//		if (ChannelState->Svid == 3)
//			printf(" %d", DataStream->DataCount);
	}
//	if (ChannelState->Svid == 3)
//		printf("\n");
}

//*************** Task to do bit sync ****************
//* This task is added to and called within baseband task queue
// Parameters:
//   Param: Pointer to bit sync data structure
// Return value:
//   0
int BitSyncTask(void *Param)
{
	int i;
	PBIT_SYNC_DATA BitSyncData = (PBIT_SYNC_DATA)Param;
	S16 PrevReal = (S16)(BitSyncData->PrevCorData >> 16), PrevImag = (S16)(BitSyncData->PrevCorData & 0xffff);
	S16 CurrentReal, CurrentImag;
	int DotProduct;
	int ToggleCount, MaxCount = 0, TotalCount = 0;
	int MaxTogglePos = 0;

	// accumulate toggle at corresponding position
	for (i = 0; i < 20; i ++)
	{
		CurrentReal = (S16)(BitSyncData->CorData[i] >> 16);
		CurrentImag = (S16)(BitSyncData->CorData[i] & 0xffff);
		DotProduct = (int)PrevReal * CurrentReal + (int)PrevImag * CurrentImag;	// calculate I1*I2+Q1*Q2
		if (DotProduct < 0)
			BitSyncData->ChannelState->ToggleCount[i] ++;
		PrevReal = CurrentReal; PrevImag = CurrentImag;

		// find max toggle count and calculate total count
		ToggleCount = BitSyncData->ChannelState->ToggleCount[i];
		if (MaxCount < ToggleCount)
		{
			MaxCount = ToggleCount;
			MaxTogglePos = i;
		}
		TotalCount += ToggleCount;
	}

	// determine whether bit sync success
	if (MaxCount >= 5 && MaxCount >= TotalCount / 2)	// at least 5 toggles and max position toggle occupies at least 50%, SUCCESS
	{
		MaxTogglePos += BitSyncData->TimeTag;	// toggle position align to time tag
		MaxTogglePos %= 20;		// remnant of 20ms
		BitSyncData->ChannelState->BitSyncResult = MaxTogglePos ? MaxTogglePos : 20;	// set result, which means bit toggle when (TrackTime % 20 == BitSyncResult)
	}
	else if (TotalCount > 100)	// 100 toggles and still not success FAIL
		BitSyncData->ChannelState->BitSyncResult = -1;
	return 0;
}
