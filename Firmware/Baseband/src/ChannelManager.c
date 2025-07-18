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
#include "ConstTable.h"
#include "BBDefines.h"
#include "HWCtrl.h"
#include "PlatformCtrl.h"
#include "InitSet.h"
#include "FirmwarePortal.h"
#include "TaskManager.h"
#include "ChannelManager.h"
#include "TeManager.h"
#include "BBCommonFunc.h"
#include "PvtEntry.h"

CHANNEL_STATE ChannelStateArray[TOTAL_CHANNEL_NUMBER];
extern PTRACKING_CONFIG TrackingConfig[][4];
extern int DoDataDecode(void* Param);

void CalcDiscriminator(PCHANNEL_STATE ChannelState, unsigned int Method);
void CohBufferFft(PCHANNEL_STATE ChannelState);
void CohBufferAcc(PCHANNEL_STATE ChannelState);
void DoTrackingLoop(PCHANNEL_STATE ChannelState);
void SwitchTrackingStage(PCHANNEL_STATE ChannelState, unsigned int TrackingStage);
int StageDetermination(PCHANNEL_STATE ChannelState);

static void ProcessCohData(PCHANNEL_STATE ChannelState);
static void CollectBitSyncData(PCHANNEL_STATE ChannelState);
static void DecodeDataStream(PCHANNEL_STATE ChannelState);
static void CalcCN0(PCHANNEL_STATE ChannelState);
static int GpsL1CABitSyncTask(void *Param);
static int GalE1BitSyncTask(void *Param);
static int DataSyncTask(void *Param);
static int SyncPilotData(unsigned int DataWord, const unsigned int SecondCode[57], int StartOffset);
static void SetWeekMsCount(PCHANNEL_STATE ChannelState);

static const unsigned int GalInvPos[25] = {
0x81f6b, 0x03ed6, 0x07dac, 0x0fb59, 0x1f6b2, 0x3ed64, 0x7dac9, 0xfb592, 0xf6b24, 0xed648, 
0xdac90, 0xb5920, 0x6b240, 0xd6481, 0xac903, 0x59207, 0xb240f, 0x6481f, 0xc903e, 0x9207d, 
0x240fb, 0x481f6, 0x903ed, 0x207da, 0x40fb5, 
};

void SetNHConfig(PCHANNEL_STATE ChannelState, int NHPos, const unsigned int *NHCode);

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

	STATE_BUF_SET_CORR_CONFIG(pStateBuffer, CurTrackingConfig->CoherentNumber, 0, CurTrackingConfig->NarrowFactor, 0, 0, 0, 0, CurTrackingConfig->PostShift, PRE_SHIFT_BITS);
	STATE_BUF_SET_NH_CONFIG(pStateBuffer, 0, 0);
	STATE_BUF_SET_DUMP_LENGTH(pStateBuffer, 1023);
	// PRN config
	if (FREQ_ID_IS_L1CA(pChannel->FreqID))
	{
		STATE_BUF_SET_PRN_CONFIG(pStateBuffer, PRN_CONFIG_L1CA(pChannel->Svid));
		pChannel->DataStream.TotalAccTime = 20;	// 20ms for GPS L1CA
		pChannel->State |= DATA_STREAM_1BIT;
	}
	else if (FREQ_ID_IS_E1(pChannel->FreqID))
	{
		STATE_BUF_SET_PRN_CONFIG(pStateBuffer, PRN_CONFIG_E1(pChannel->Svid));
		pChannel->DataStream.TotalAccTime = 4;	// 4ms for Galileo E1
		pChannel->State |= DATA_STREAM_4BIT;
	}
	else if (FREQ_ID_IS_B1C(pChannel->FreqID))
	{
		STATE_BUF_SET_PRN_CONFIG(pStateBuffer, PRN_CONFIG_B1C(pChannel->Svid));
		pChannel->DataStream.TotalAccTime = 10;	// 10ms for BDS B1C
		pChannel->State |= DATA_STREAM_8BIT;
	}
	else if (FREQ_ID_IS_L1C(pChannel->FreqID))
	{
		STATE_BUF_SET_PRN_CONFIG(pStateBuffer, PRN_CONFIG_L1C(pChannel->Svid));
		pChannel->DataStream.TotalAccTime = 10;	// 10ms for GPS L1C
		pChannel->State |= DATA_STREAM_8BIT;
	}
	pChannel->State |= STATE_CACHE_DIRTY;	// set cache dirty
	pChannel->WeekMsCounter = -1;

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
	if (FREQ_ID_IS_L1CA(pChannel->FreqID))
		pChannel->CarrierFreqBase = CARRIER_FREQ(Doppler);
	else
		pChannel->CarrierFreqBase = CARRIER_FREQ_BOC(Doppler);
	pChannel->CodeFreqBase = CODE_FREQ(Doppler);
	STATE_BUF_SET_CARRIER_FREQ(pStateBuffer, pChannel->CarrierFreqBase);
	STATE_BUF_SET_CODE_FREQ(pStateBuffer, pChannel->CodeFreqBase);
	// PRN config
	if (FREQ_ID_IS_L1CA(pChannel->FreqID))
	{
		StartPhase %= 1023;	// remnant of code cycle
		STATE_BUF_SET_PRN_COUNT(pStateBuffer, PRN_COUNT_L1CA(StartPhase));
		pChannel->SkipCount = 1;	// skip the first coherent sum result (PRN not ready until code edge for L1CA)
	}
	else if (FREQ_ID_IS_E1(pChannel->FreqID))
	{
		StartPhase %= 4092;	// remnant of code cycle
		STATE_BUF_SET_PRN_COUNT(pStateBuffer, PRN_COUNT_E1(StartPhase));
		pChannel->SkipCount = 4 - StartPhase / 1023;	// align to bit edge
		pChannel->TrackingTime = StartPhase / 1023;
	}
	else if (FREQ_ID_IS_B1C(pChannel->FreqID))
	{
		StartPhase %= 10230;	// remnant of code cycle
		STATE_BUF_SET_PRN_COUNT(pStateBuffer, PRN_COUNT_B1C(StartPhase));
		pChannel->SkipCount = 10 - StartPhase / 1023;	// align to bit edge
		pChannel->TrackingTime = StartPhase / 1023;
	}
	else if (FREQ_ID_IS_L1C(pChannel->FreqID))
	{
		StartPhase %= 10230;	// remnant of code cycle
		STATE_BUF_SET_PRN_COUNT(pStateBuffer, PRN_COUNT_L1C(StartPhase));
		pChannel->SkipCount = 10 - StartPhase / 1023;	// align to bit edge
		pChannel->TrackingTime = StartPhase / 1023;
	}
	// status fields
	STATE_BUF_SET_CODE_PHASE(pStateBuffer, (CodePhase16x << 29));
	STATE_BUF_SET_DUMP_COUNT(pStateBuffer, (StartPhase % 1023));
	STATE_BUF_SET_NH_COUNT(pStateBuffer, (StartPhase / 1023));
	STATE_BUF_SET_CODE_SUB_PHASE(pStateBuffer, (CodePhase16x >> 3));
	pChannel->State |= (STATE_CACHE_FREQ_DIRTY | STATE_CACHE_CODE_DIRTY);	// set cache dirty
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
		if (ChannelState->State & STATE_CACHE_CONFIG_DIRTY)	// update CorrConfig, NHConfig and DumpLength
		{
			SetRegValue((U32)(&(ChannelState->StateBufferHW->CorrConfig)), ChannelState->StateBufferCache.CorrConfig);
			SetRegValue((U32)(&(ChannelState->StateBufferHW->NHConfig)), ChannelState->StateBufferCache.NHConfig);
			SetRegValue((U32)(&(ChannelState->StateBufferHW->DumpLength)), ChannelState->StateBufferCache.DumpLength);
		}
		if (ChannelState->State & STATE_CACHE_CODE_DIRTY)	// update PrnCount, CodePhase, DumpCount and CorrState
		{
			SetRegValue((U32)(&(ChannelState->StateBufferHW->PrnCount)), ChannelState->StateBufferCache.PrnCount);
			SetRegValue((U32)(&(ChannelState->StateBufferHW->CodePhase)), ChannelState->StateBufferCache.CodePhase);
			SetRegValue((U32)(&(ChannelState->StateBufferHW->DumpCount)), ChannelState->StateBufferCache.DumpCount);
			SetRegValue((U32)(&(ChannelState->StateBufferHW->CorrState)), ChannelState->StateBufferCache.CorrState);
		}
		if (ChannelState->State & STATE_CACHE_STATE_DIRTY)	// update CorrState
			SetRegValue((U32)(&(ChannelState->StateBufferHW->CorrState)), ChannelState->StateBufferCache.CorrState);
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
	unsigned int StateValue;
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
		DEBUG_OUTPUT(OUTPUT_CONTROL(COH_PROC, OFF), "%6d %6d\n", (S16)(ChannelState->PendingCoh[0] >> 16), (S16)(ChannelState->PendingCoh[0] & 0xffff));
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

	// determine whether NH code segment need to update
	if (ChannelState->State & NH_SEGMENT_UPDATE)
	{
		StateValue = GetRegValue((U32)(&(ChannelState->StateBufferHW->CorrState)));
		if ((StateValue >> 27) >= 20)
			SetNHConfig(ChannelState, ChannelState->FrameCounter, B1CSecondCode[ChannelState->Svid-1]);
	}
}

//*************** Process coherent data of a channel ****************
// Parameters:
//   ChannelState: Pointer to channel state structure
// Return value:
//   none
void ProcessCohData(PCHANNEL_STATE ChannelState)
{
	ChannelState->TrackingTime += ChannelState->CoherentNumber;	// accumulate tracking time
	DEBUG_OUTPUT(OUTPUT_CONTROL(COH_PROC, OFF), "track time %d\n", ChannelState->TrackingTime);
	ChannelState->TickCount = BasebandTickCount;
	if (ChannelState->WeekMsCounter >= 0)	// frame sync completed
	{
		ChannelState->WeekMsCounter += ChannelState->CoherentNumber;
		if (ChannelState->WeekMsCounter >= MS_IN_WEEK)
			ChannelState->WeekMsCounter = 0;
	}
	else if (ChannelState->SyncTickCount > 0)
		ChannelState->WeekMsCounter = MS_IN_WEEK - (ChannelState->SyncTickCount - BasebandTickCount);

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

	if (ChannelState->SkipCount > 0)	// skip coherent result for following process
	{
		ChannelState->SkipCount --;
		return;
	}
	
//	if (ChannelState->Svid == 19)
	DEBUG_OUTPUT(OUTPUT_CONTROL(COH_PROC, INFO), "SV%2d Stage%d T=%4d I/Q[4]=%6d %6d I/Q[0]=%6d %6d\n", \
		ChannelState->Svid, (ChannelState->State & STAGE_MASK), ChannelState->TrackingTime, \
		(S16)(ChannelState->PendingCoh[4] >> 16), (S16)(ChannelState->PendingCoh[4] & 0xffff), \
		(S16)(ChannelState->PendingCoh[0] >> 16), (S16)(ChannelState->PendingCoh[0] & 0xffff));

	// perform PLL
	if (((ChannelState->State & STAGE_MASK) >= STAGE_TRACK) && (ChannelState->pll_k1 > 0))	// tracking stage uses PLL (change to more flexible condition in the future)
		CalcDiscriminator(ChannelState, TRACKING_UPDATE_PLL);

	// do FFT and non-coherent accumulation
	if (++ChannelState->FftCount == ChannelState->FftNumber)
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
	
	// determine whether tracking stage switch is needed
	StageDetermination(ChannelState);
}

//*************** Compose baseband measurement and data stream ****************
// Parameters:
//   ChannelID: physical channel ID (start from 0)
//   Measurement: pointer to baseband measurement structure
//   DataBuffer: buffer to store data stream
// Return value:
//   0
int ComposeMeasurement(int ChannelID, PBB_MEASUREMENT Measurement)
{
	PCHANNEL_STATE ChannelState = &ChannelStateArray[ChannelID];
	PSTATE_BUFFER StateBuffer = &(ChannelState->StateBufferCache);
	int CohCount, CurrentCor, CodeCount;
	int MsCount, DataAccTime;

	SyncCacheRead(ChannelState, SYNC_CACHE_READ_STATUS);
	Measurement->ChannelState = ChannelState;

	Measurement->CodePhase = STATE_BUF_GET_CODE_PHASE(StateBuffer);
	CodeCount = GET_PRN_COUNT(ChannelState->FreqID, StateBuffer->PrnCount);
	CodeCount %= 1023;	// within 1ms
	CodeCount = (CodeCount << 1) + STATE_BUF_GET_CODE_SUB_PHASE(StateBuffer) - 4;	// compensate 4 correlator interval to align to COR4
	CohCount = STATE_BUF_GET_COH_COUNT(StateBuffer);
	// check whether CurrentCor != 0, if so, CohCount has 1 lag to Cor0
	CurrentCor = STATE_BUF_GET_CUR_CORR(&(ChannelState->StateBufferCache));
	if (CurrentCor)
		CohCount ++;

	Measurement->CarrierFreq = STATE_BUF_GET_CARRIER_FREQ(StateBuffer);
	Measurement->CarrierPhase = STATE_BUF_GET_CARRIER_PHASE(StateBuffer);
	Measurement->CarrierCount = STATE_BUF_GET_CARRIER_COUNT(StateBuffer);

	MsCount = (ChannelState->WeekMsCounter < 0) ? -100 : ChannelState->WeekMsCounter;	// -100 make sure negative value after add CohCount
	// L1CA align to symbol boundary if tracking state go beyond BIT_SYNC and do data decode
	if (FREQ_ID_IS_L1CA(ChannelState->FreqID) && (GET_STAGE(ChannelState) >= STAGE_TRACK))
		DataAccTime = ChannelState->DataStream.CurrentAccTime;
	else
		DataAccTime = 0;
	Measurement->WeekMsCount = MsCount - DataAccTime; // week millisecond go back to symbol boundary
	CohCount += DataAccTime;	// adjustment to week millisecond apply to CorCount
	Measurement->CodeCount = CodeCount + CohCount * 2046;

	return 0;
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
	S16 PrevReal, PrevImag;
	S16 CurrentReal, CurrentImag;

	if (BitSyncData->PrevCorData == 0)	// not initialized
	{
		BitSyncData->PrevCorData = ChannelState->PendingCoh[4];
		return;
	}
	PrevReal = (S16)(BitSyncData->PrevCorData >> 16);
	PrevImag = (S16)(BitSyncData->PrevCorData & 0xffff);
	CurrentReal = (S16)(ChannelState->PendingCoh[4] >> 16);
	CurrentImag = (S16)(ChannelState->PendingCoh[4] & 0xffff);
	BitSyncData->PolarityToggle <<= 1;
	BitSyncData->PolarityToggle |= (((int)PrevReal * CurrentReal + (int)PrevImag * CurrentImag) < 0) ? 1 : 0;
	BitSyncData->PrevCorData = ChannelState->PendingCoh[4];
	if (++BitSyncData->CorDataCount == 20)	// 20 correlation result, send to bit sync task
	{
		BitSyncData->TimeTag = ChannelState->TrackingTime;
		AddToTask(TASK_BASEBAND, FREQ_ID_IS_L1CA(ChannelState->FreqID) ? GpsL1CABitSyncTask : GalE1BitSyncTask, BitSyncData, sizeof(BIT_SYNC_DATA));
		BitSyncData->CorDataCount = 0;
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
	PBIT_SYNC_DATA BitSyncData = &(ChannelState->BitSyncData);
	DATA_FOR_DECODE DataForDecode;
	unsigned int DataSymbol = 0;
	int i, SymbolCount;
	unsigned int Data;

	if (ChannelState->State & DATA_STREAM_PRN2)	// tracking pilot channel and decode data channel, uses hardware data decode
	{
		// determine whether at least one symbol decoded
		if (DataStream->TotalAccTime > ChannelState->CoherentNumber)	// symbol length longer than coherent length
		{
			DataStream->CurrentAccTime += ChannelState->CoherentNumber;
			if (DataStream->CurrentAccTime < DataStream->TotalAccTime)
				return;
			DataStream->CurrentAccTime = 0;
			SymbolCount = 1;
		}
		else
			SymbolCount = ChannelState->CoherentNumber / DataStream->TotalAccTime;	// coherent length must be multiple of symbol length

		// update FrameCounter
		if (ChannelState->FrameCounter >= 0)
			ChannelState->FrameCounter += SymbolCount;
		if (FREQ_ID_IS_L1CA(ChannelState->FreqID) && ChannelState->FrameCounter >= 1500)	// 1500bit for each round of subframe 1-5
			ChannelState->FrameCounter -= 1500;
		else if (FREQ_ID_IS_E1(ChannelState->FreqID) && ChannelState->FrameCounter >= 500)	// 500bit for each round of even/odd
			ChannelState->FrameCounter -= 500;
		else if (ChannelState->FrameCounter >= 1800)	// 1800bit for B1C or L1C
			ChannelState->FrameCounter -= 1800;

		Data = GetRegValue((U32)(&(ChannelState->StateBufferHW->DecodeData)));
		if (FREQ_ID_IS_B1C(ChannelState->FreqID) || FREQ_ID_IS_E1(ChannelState->FreqID))	// B1C/E1 has negative data
			Data ^= 0xffffffff;
		for (i = 0; i < SymbolCount; i ++)
		{
			switch (ChannelState->State & DATA_STREAM_MASK)
			{
			case DATA_STREAM_1BIT:
				DataSymbol = (Data >> (SymbolCount - i - 1)) & 1;
				DataStream->Symbols <<= 1;
				DataStream->BitCount ++;
				break;
			case DATA_STREAM_4BIT:
				DataSymbol = (Data >> ((SymbolCount - i - 1) * 4)) & 0xf;
				DataStream->Symbols <<= 4;
				DataStream->BitCount += 4;
				break;
			case DATA_STREAM_8BIT:
				DataSymbol = (Data >> ((SymbolCount - i - 1) * 8)) & 0xff;
				DataStream->Symbols <<= 8;
				DataStream->BitCount += 8;
				break;
			}
			DataStream->Symbols |= DataSymbol;
			if (DataStream->BitCount == 32)
			{
				DataForDecode.ChannelState = ChannelState;
				DataForDecode.DataStream = DataStream->Symbols;
				DataForDecode.StartIndex = DataStream->StartIndex;
				DataForDecode.TickCount = BasebandTickCount - DataStream->TotalAccTime * (SymbolCount - 1 - i);
				DataStream->BitCount = 0;
				AddToTask(TASK_BASEBAND, DoDataDecode, &DataForDecode, sizeof(DATA_FOR_DECODE));
				DataStream->StartIndex = ChannelState->FrameCounter - (SymbolCount - 1 - i);
			}
		}
	}
	else	// decode L1C/A or decode pilot channel NH code (1bit only)
	{
		DataStream->CurrentAccTime += ChannelState->CoherentNumber;
		DataStream->CurReal += (S16)(ChannelState->PendingCoh[4] >> 16);
		DataStream->CurImag += (S16)(ChannelState->PendingCoh[4] & 0xffff);

		if (DataStream->CurrentAccTime < DataStream->TotalAccTime)
			return;

		DEBUG_OUTPUT(OUTPUT_CONTROL(COH_PROC, OFF), "DATA %5d %5d at %d\n", DataStream->CurReal, DataStream->CurImag, DataStream->BitCount);

		if ((ChannelState->State & STAGE_MASK) == STAGE_TRACK0 || (ChannelState->State & STAGE_MASK) == STAGE_TRACK1)	// PLL lock, determine by sign
			DataSymbol = ((DataStream->CurReal < 0) ? 1 : 0);
		else	// determine by data toggle
		{
			// determine symbol toggle
			DataSymbol = DataStream->CurReal * DataStream->PrevReal + DataStream->CurImag * DataStream->PrevImag;
			DataSymbol = ((DataSymbol < 0) ? 1 : 0) ^ (DataStream->Symbols & 1);
			// store to previous values for next symbol
			DataStream->PrevReal = DataStream->CurReal;
			DataStream->PrevImag = DataStream->CurImag;
		}
		// put into data stream buffer
		DataStream->Symbols <<= 1;
		DataStream->Symbols |= DataSymbol;

		DataStream->BitCount ++;
		DataStream->CurrentAccTime = 0;
		DataStream->CurReal = DataStream->CurImag = 0;

		if (FREQ_ID_IS_L1CA(ChannelState->FreqID) && DataStream->BitCount == 32)	// 32bits for L1CA data decode
		{
			DataForDecode.ChannelState = ChannelState;
			DataForDecode.TickCount = BasebandTickCount;
			DataForDecode.StartIndex = -1;
			DataForDecode.DataStream = DataStream->Symbols;
			AddToTask(TASK_BASEBAND, DoDataDecode, &DataForDecode, sizeof(DATA_FOR_DECODE));
			DataStream->BitCount = 0;
		}
		if (!(FREQ_ID_IS_L1CA(ChannelState->FreqID)) && DataStream->BitCount == 24)	// 24bits for B1C/L1C secondary code sync
		{
			BitSyncData->PolarityToggle = DataStream->Symbols;
			BitSyncData->TimeTag = ChannelState->TrackingTime;
//			BitSyncData->PrevCorData = ChannelState->BitSyncResult;	// stage of frame sync
			AddToTask(TASK_BASEBAND, DataSyncTask, BitSyncData, sizeof(BIT_SYNC_DATA));
			DataStream->BitCount = 0;
		}
	}
}

//*************** Calculate smoothed CN0 and instant CN0 ****************
//* calculate CN0 using peak power and noise floor, also count CN0 high and low count
// Parameters:
//   ChannelState: Pointer to channel state structure
// Return value:
//   none
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
	NoiseFloor = (NoiseFloor * NoiseFloor * 163) >> 8;
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
	DEBUG_OUTPUT(OUTPUT_CONTROL(COH_PROC, OFF), "CN0=%4d fastCN0=%4d\n", ChannelState->CN0, ChannelState->FastCN0);

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

//*************** Set NH config ****************
//* set NH code and length according to NH position in secondary code
//* also set NH count field in state buffer
// Parameters:
//   ChannelState: Pointer to channel state structure
//   NHPos: current NH position within secondary code
//   NHCode: 1800bit NH code stream (LSB first in each DWORD)
// Return value:
//   none
void SetNHConfig(PCHANNEL_STATE ChannelState, int NHPos, const unsigned int *NHCode)
{
	unsigned int SegmentCode, StateValue;
	int Segment, NHCount;

	// calculate 20bit NH code from 1800bit secondary code stream
	NHCount = NHPos % 20;
	NHPos -= NHCount;
	Segment = NHPos / 32;
	NHPos &= 0x1f;
	SegmentCode = NHCode[Segment];
	SegmentCode >>= NHPos;
	if (NHPos > 8)	// 24bit code not within one DWORD
		SegmentCode |= (NHCode[Segment+1] << (32 - NHPos));
	SegmentCode &= 0xffffff;
	// write to state buffer
	STATE_BUF_SET_NH_CONFIG(&(ChannelState->StateBufferCache), 24, SegmentCode);
	SetRegValue((U32)(&(ChannelState->StateBufferHW->NHConfig)),  ChannelState->StateBufferCache.NHConfig);
	// set current NH count
	StateValue = GetRegValue((U32)(&(ChannelState->StateBufferHW->CorrState)));
	SET_FIELD(StateValue, 27, 5, NHCount);
	SetRegValue((U32)(&(ChannelState->StateBufferHW->CorrState)), StateValue);
}

//*************** Task to do GPS L1CA bit sync ****************
//* This task is added to and called within baseband task queue
// Parameters:
//   Param: Pointer to bit sync data structure
// Return value:
//   0
int GpsL1CABitSyncTask(void *Param)
{
	int i;
	PBIT_SYNC_DATA BitSyncData = (PBIT_SYNC_DATA)Param;
	int ToggleCount, MaxCount = 0, TotalCount = 0;
	int MaxTogglePos = 0;

	// accumulate toggle at corresponding position
	for (i = 0; i < 20; i ++)
	{
		if (BitSyncData->PolarityToggle & (1 << (19 - i)))
			BitSyncData->ChannelState->ToggleCount[i] ++;

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
		DEBUG_OUTPUT(OUTPUT_CONTROL(COH_PROC, OFF), "Bitsync found at %d with %d/%d\n", MaxTogglePos, MaxCount, TotalCount);
		MaxTogglePos += BitSyncData->TimeTag;	// toggle position align to time tag
		MaxTogglePos %= 20;		// remnant of 20ms
		BitSyncData->ChannelState->BitSyncResult = MaxTogglePos ? MaxTogglePos : 20;	// set result, which means bit toggle when (TrackTime % 20 == BitSyncResult)
	}
	else if (TotalCount > 100)	// 100 toggles and still not success FAIL
		BitSyncData->ChannelState->BitSyncResult = -1;

	return 0;
}

//*************** Task to do Galileo E1C secondary sync ****************
//* This task is added to and called within baseband task queue
// Parameters:
//   Param: Pointer to bit sync data structure
// Return value:
//   0
int GalE1BitSyncTask(void *Param)
{
	int i;
	PBIT_SYNC_DATA BitSyncData = (PBIT_SYNC_DATA)Param;
	int TotalCount = BitSyncData->ChannelState->ToggleCount[0];

	for (i = 0; i < 25; i ++)
		if ((BitSyncData->PolarityToggle & 0xfffff) == GalInvPos[i])
			break;
	if (i < 25)	// secondary code toggle pattern match
		BitSyncData->ChannelState->BitSyncResult = BitSyncData->TimeTag + (25 - i) * 4;	// set result, next secondary boundary when TrackTime == BitSyncResult)
	else if (++BitSyncData->ChannelState->ToggleCount[0] == 10)	// failed after 10 times check
		BitSyncData->ChannelState->BitSyncResult = -1;

	return 0;
}

//*************** Task to do pilot data sync ****************
//* This task is added to and called within baseband task queue
//* Set ChannelState->BitSyncResult with following value
//* 0: sync to secondary code fail
//* 0x800+index: secondary code start from index with positive sign
//* 0x1000+index: secondary code start from index with negative sign
//* index is the bit index within secondary code at TrackingTime==0
// Parameters:
//   Param: Pointer to bit sync data structure
// Return value:
//   0
int DataSyncTask(void *Param)
{
	int i;
	unsigned int DataWord = 0;

	PBIT_SYNC_DATA BitSyncData = (PBIT_SYNC_DATA)Param;
	int FreqID = (int)(BitSyncData->ChannelState->FreqID);
	int Svid = (int)(BitSyncData->ChannelState->Svid);

	if (FREQ_ID_IS_E1(FreqID))
		;
	else if (FREQ_ID_IS_B1C(FreqID))
	{
		// revert data order to LSB first
		for (i = 0; i < 24; i ++)
		{
			DataWord <<= 1;
			DataWord |= (BitSyncData->PolarityToggle & 1) ? 1 : 0;
			BitSyncData->PolarityToggle >>= 1;
		}
		BitSyncData->ChannelState->BitSyncResult = SyncPilotData(DataWord, B1CSecondCode[Svid-1], BitSyncData->TimeTag / 10 - 24);
		return 0;
	}
	else if (FREQ_ID_IS_L1C(FreqID))
		;
	return 0;
}

//*************** find pilot data sync match position ****************
// Parameters:
//   CorData: Pointer to pilot data, 8bit for each data from MSB
//   SecondCode: 1800 bit of pilot data
//   StartOffset: first bit from TrackingTime==0
// Return value:
//   0 for match position not found
//   0x800~0x800+1799 for match positive
//   0x1000~0x1000+1799 for match negative
int SyncPilotData(unsigned int DataWord, const unsigned int SecondCode[57], int StartOffset)
{
	int i;
	unsigned int CodeWord;
	unsigned int Match;

	CodeWord = SecondCode[0];
	for (i = 0; i < 1800; i ++)
	{
		Match = (DataWord ^(CodeWord & 0xffffff));
		if (Match == 0 || Match == 0xffffff)
			break;

		CodeWord >>= 1;
		if ((i & 0x7) == 7)	// i = 7, 15, 23, 31...
			CodeWord |= ((SecondCode[i/32+1] << ((i ^ 0x1f) & 0x18)) & 0xff000000);
	}

	if (i == 1800)
		return 0;
	i -= StartOffset;
	while (i < 0)
		i += 1800;
	return Match ? (0x1000 + i) : (0x800 + i);
}
