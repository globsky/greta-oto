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
#include "BBCommonFunc.h"

CHANNEL_STATE ChannelStateArray[TOTAL_CHANNEL_NUMBER];
extern PTRACKING_CONFIG TrackingConfig[][4];

void CalcDiscriminator(PCHANNEL_STATE ChannelState, unsigned int Method);
void CohBufferFft(PCHANNEL_STATE ChannelState);
void CohBufferAcc(PCHANNEL_STATE ChannelState);
void DoTrackingLoop(PCHANNEL_STATE ChannelState);
void SwitchTrackingStage(PCHANNEL_STATE ChannelState, unsigned int TrackingStage);
int StageDetermination(PCHANNEL_STATE ChannelState);

static void ProcessCohData(PCHANNEL_STATE ChannelState);
static void CollectBitSyncData(PCHANNEL_STATE ChannelState);
static void DecodeDataStream(PCHANNEL_STATE ChannelState);
static int BitSyncTask(void *Param);
static void CalcCN0(PCHANNEL_STATE ChannelState);

#define PRE_SHIFT_BITS 0

//*************** Initialize channel state structure ****************
// Parameters:
//   pChannel: pointer to channel state
// Return value:
//   none
void InitChannel(PCHANNEL_STATE pChannel)
{
	PSTATE_BUFFER pStateBuffer = &(pChannel->StateBufferCache);
	PTRACKING_CONFIG CurTrackingConfig = TrackingConfig[0][pChannel->FreqID];

	memset(pStateBuffer, 0, sizeof(STATE_BUFFER));

	STATE_BUF_SET_CORR_CONFIG(pStateBuffer, 1023, CurTrackingConfig->NarrowFactor, 0, 0, 0, PRE_SHIFT_BITS);
	STATE_BUF_SET_NH_CONFIG(pStateBuffer, 0, 0);
	STATE_BUF_SET_COH_CONFIG(pStateBuffer, CurTrackingConfig->PostShift, CurTrackingConfig->CoherentNumber, 0, 0);
	// PRN config
	if (pChannel->FreqID == FREQ_L1CA)
	{
		STATE_BUF_SET_PRN_CONFIG(pStateBuffer, PRN_CONFIG_L1CA(pChannel->Svid));
		pChannel->DataStream.TotalAccTime = 20;	// 20ms for GPS L1CA
		pChannel->State |= DATA_STREAM_1BIT;
	}
	else if (pChannel->FreqID == FREQ_E1)
	{
		STATE_BUF_SET_PRN_CONFIG(pStateBuffer, PRN_CONFIG_E1(pChannel->Svid));
		pChannel->DataStream.TotalAccTime = 4;	// 4ms for Galileo E1
		pChannel->State |= DATA_STREAM_4BIT;
	}
	else if (pChannel->FreqID == FREQ_B1C)
	{
		STATE_BUF_SET_PRN_CONFIG(pStateBuffer, PRN_CONFIG_B1C(pChannel->Svid));
		pChannel->DataStream.TotalAccTime = 10;	// 10ms for BDS B1C
		pChannel->State |= DATA_STREAM_8BIT;
	}
	else if (pChannel->FreqID == FREQ_L1C)
	{
		STATE_BUF_SET_PRN_CONFIG(pStateBuffer, PRN_CONFIG_L1C(pChannel->Svid));
		pChannel->DataStream.TotalAccTime = 10;	// 10ms for GPS L1C
		pChannel->State |= DATA_STREAM_8BIT;
	}
	pChannel->State |= STATE_CACHE_DIRTY;	// set cache dirty

	SwitchTrackingStage(pChannel, STAGE_PULL_IN);	// switch to pull-in stage
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

	// config DWORDs
	if (pChannel->FreqID == FREQ_L1CA)
		pChannel->CarrierFreqBase = CARRIER_FREQ(Doppler);
	else
		pChannel->CarrierFreqBase = CARRIER_FREQ_BOC(Doppler);
	pChannel->CodeFreqBase = CODE_FREQ(Doppler);
	STATE_BUF_SET_CARRIER_FREQ(pStateBuffer, pChannel->CarrierFreqBase);
	STATE_BUF_SET_CODE_FREQ(pStateBuffer, pChannel->CodeFreqBase);
	// PRN config
	if (pChannel->FreqID == FREQ_L1CA)
	{
		StartPhase %= 1023;	// remnant of code cycle
		STATE_BUF_SET_PRN_COUNT(pStateBuffer, PRN_COUNT_L1CA(StartPhase));
	}
	else if (pChannel->FreqID == FREQ_E1)
	{
		StartPhase %= 4092;	// remnant of code cycle
		STATE_BUF_SET_PRN_COUNT(pStateBuffer, PRN_COUNT_E1(StartPhase));
	}
	else if (pChannel->FreqID == FREQ_B1C)
	{
		StartPhase %= 10230;	// remnant of code cycle
		STATE_BUF_SET_PRN_COUNT(pStateBuffer, PRN_COUNT_B1C(StartPhase));
	}
	else if (pChannel->FreqID == FREQ_L1C)
	{
		StartPhase %= 10230;	// remnant of code cycle
		STATE_BUF_SET_PRN_COUNT(pStateBuffer, PRN_COUNT_L1C(StartPhase));
	}
	// status fields
	STATE_BUF_SET_CODE_PHASE(pStateBuffer, (CodePhase16x << 29));
	STATE_BUF_SET_DUMP_COUNT(pStateBuffer, (StartPhase % 1023));
	STATE_BUF_SET_NH_COUNT(pStateBuffer, (StartPhase / 1023));
	STATE_BUF_SET_CODE_SUB_PHASE(pStateBuffer, (CodePhase16x >> 3));
	pChannel->State |= (STATE_CACHE_FREQ_DIRTY | STATE_CACHE_CODE_DIRTY);	// set cache dirty
	pChannel->SkipCount = 1;	// skip the first coherent sum result (PRN not ready until code edge for L1CA)
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
//		printf("%6d %6d\n", (S16)(ChannelState->PendingCoh[0] >> 16), (S16)(ChannelState->PendingCoh[0] & 0xffff));
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
	if (((ChannelState->State & STAGE_MASK) >= STAGE_TRACK) && (ChannelState->pll_k1 > 0))	// tracking stage uses PLL (change to more flexible condition in the future)
		CalcDiscriminator(ChannelState, TRACKING_UPDATE_PLL);

	// do FFT and non-coherent accumulation
	if (ChannelState->FftCount == ChannelState->FftNumber)
	{
		ChannelState->FftCount = 0;
		if (ChannelState->FftNumber > 1)
			CohBufferFft(ChannelState);
		else
			CohBufferAcc(ChannelState);
		if (ChannelState->NonCohCount == 0)	// CohBufferFft() or CohBufferAcc() will set NonCohCount to 0 if it reaches NonCohNumber
			CalcCN0(ChannelState);
	}

	// do tracking loop
	if ((ChannelState->State & STAGE_MASK) >= STAGE_PULL_IN)
		DoTrackingLoop(ChannelState);

//	if ((ChannelState->State & STAGE_MASK) >= STAGE_TRACK)
//		printf("T=%4d I/Q=%6d %6d\n", ChannelState->TrackingTime, (S16)(ChannelState->StateBufferCache.CoherentSum[4] >> 16), (S16)(ChannelState->StateBufferCache.CoherentSum[4] & 0xffff));
	
	// collect correlation result for bit sync if in bit sync stage
	if ((ChannelState->State & STAGE_MASK) == STAGE_BIT_SYNC)
		CollectBitSyncData(ChannelState);
	// keep bit edge at tracking hold
	else if ((ChannelState->State & STAGE_MASK) == STAGE_HOLD3)
	{
		ChannelState->DataStream.CurrentAccTime += ChannelState->CoherentNumber;
		if (ChannelState->DataStream.CurrentAccTime >= ChannelState->DataStream.TotalAccTime)
			ChannelState->DataStream.CurrentAccTime = 0;
	}
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

	Measurement->CN0 = ChannelState->CN0;
	Measurement->LockIndicator = 100;

	return WordNumber;
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

void CalcCN0(PCHANNEL_STATE ChannelState)
{
	PTRACKING_CONFIG CurTrackingConfig = TrackingConfig[STAGE_CONFIG_INDEX(ChannelState->State & STAGE_MASK)][ChannelState->FreqID];
	int CohRatio = CurTrackingConfig->CoherentNumber * CurTrackingConfig->FftNumber;
	int NoncohRatio = CurTrackingConfig->NonCohNumber;
	int NoiseFloor = GetRegValue(ADDR_TE_NOISE_FLOOR);	// NF get from hardware
	int SignalPowerNorm = ChannelState->PeakPower;
	int Shift = CurTrackingConfig->PostShift * 2 + ((CurTrackingConfig->FftNumber > 1) ? 6 : 0);
	int FilterScale = (ChannelState->CN0 > 2500) ? 4 : 6;
	int CN0Gap, ResetPower;

	// calculate noise power 2(sigma^2) = 4 * (NF^2) / pi
	NoiseFloor = (NoiseFloor * NoiseFloor * 163) >> 7;
	// calculate adjusted noise power (2 * sigma^2 * Nc * Nn / 2^Shift)
	NoiseFloor = (NoiseFloor * CohRatio * NoncohRatio) >> Shift;
	// remove adjusted noise power from total power
	SignalPowerNorm -= NoiseFloor;
	if (SignalPowerNorm <= 0)
		SignalPowerNorm = 1;
	// calculate CN0 = 30.00 + 10log10(S/(N*Nc))
	NoiseFloor *= CohRatio;
	NoiseFloor = IntLog10(NoiseFloor) - 3000;
	ChannelState->FastCN0 = IntLog10(SignalPowerNorm) - NoiseFloor;
	if (ChannelState->FastCN0 < 500)	// clip lowest CN0 at 5dBHz
		ChannelState->FastCN0 = 500;

	// smooth signal power
	if (ChannelState->SmoothedPower == 0)
		ChannelState->SmoothedPower = SignalPowerNorm;
	else
		ChannelState->SmoothedPower = ChannelState->SmoothedPower + ((SignalPowerNorm - ChannelState->SmoothedPower) >> FilterScale);
	ChannelState->CN0 = IntLog10(ChannelState->SmoothedPower) - NoiseFloor;
	if (ChannelState->CN0 < 500)	// clip lowest CN0 at 5dBHz
		ChannelState->CN0 = 500;
//	printf("CN0=%4d fastCN0=%4d\n", ChannelState->CN0, ChannelState->FastCN0);

	// detect CN0 jump
	CN0Gap = ChannelState->CN0 - ChannelState->FastCN0;
	ResetPower = 0;
	if (ABS(CN0Gap) > 300 && ChannelState->CN0 > 2500 && ChannelState->FastCN0 > 2500)
		ResetPower = 1;
	if (ABS(CN0Gap) > 600 && ChannelState->CN0 > 1800 && ChannelState->FastCN0 > 1800)
		ResetPower = 1;
	if (ResetPower)
	{
		ChannelState->SmoothedPower = SignalPowerNorm;
		ChannelState->CN0 = ChannelState->FastCN0;
	}

	// count CN0 high and low time
	if (ChannelState->FastCN0 > 3200)
	{
		ChannelState->CN0HighCount += CohRatio * NoncohRatio;
		ChannelState->CNOLowCount = 0;
	}
	else if (ChannelState->FastCN0 < 2500)
	{
		ChannelState->CNOLowCount += CohRatio * NoncohRatio;
		ChannelState->CN0HighCount = 0;
	}
}
