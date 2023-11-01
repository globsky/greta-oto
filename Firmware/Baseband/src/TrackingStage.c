//----------------------------------------------------------------------
// TrackingStage.c:
//   Functions to process tracking stages
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "ConstTable.h"
#include "HWCtrl.h"
#include "PlatformCtrl.h"
#include "ChannelManager.h"

#define C1 (1<<16)
#define C2 (2<<16)
#define C3 (3<<16)

// in order to have stage switch at correct data edge, positive Timeout value should be multiple of 20
static TRACKING_CONFIG TrackingConfigTable[] = {
// Coh FFT NonCoh Narrow Post  BnPLL   BnFLL   BnDLL  Timeout
 {  1,  5,   2,      0,    1,      0,  80|C2,  80|C2,   200,},	// 0 for pull-in
 {  1,  5,   2,      0,    1,      0,  80|C2,  80|C2,  1500,},	// 1 for bit_sync
 {  4,  5,   4,      0,    2,      0,      0,      0, 30000,},	// 2 for GPS L1 tracking hold
 {  5,  1,   4,      0,    2, 320|C2,   0|C2,  80|C2,  5000,},	// 3 for GPS L1 track 0
 { 20,  1,   4,      0,    3, 240|C2,   0|C2,  40|C2,    -1,},	// 4 for GPS L1 track 1
 {  4,  5,  16,      0,    3,   0|C2,  80|C2,  40|C2,    -1,},	// 5 for GPS L1 track 2
 {  5,  1,   4,      0,    2, 320|C2,   0|C2,  80|C2,    -1,},	// 6 for BDS B1C track 0
 { 10,  1,   4,      0,    3, 240|C2,   0|C2,  40|C2,    -1,},	// 7 for BDS B1C track 1
 {  1,  4,   2,      0,    1,      0,  80|C2,  80|C2,   200,},	// 8 for GAL E1 pull-in
 {  4,  1,   5,      0,    2, 320|C2,   0|C2,  80|C2,  1500,},	// 9 for GAL E1 bit_sync
 {  4,  1,   5,      0,    2, 320|C2,   0|C2,  80|C2,  5000,},	//10 for GAL E1 track 0
};

PTRACKING_CONFIG TrackingConfig[][4] = {	// pointer to TrackingConfigTable for different stage and different signal
	&TrackingConfigTable[2], &TrackingConfigTable[1], &TrackingConfigTable[2], &TrackingConfigTable[2],	// tracking hold
	&TrackingConfigTable[0], &TrackingConfigTable[8], &TrackingConfigTable[0], &TrackingConfigTable[0],	// pull-in
	&TrackingConfigTable[1], &TrackingConfigTable[9], &TrackingConfigTable[1], &TrackingConfigTable[1],	// bit_sync
	&TrackingConfigTable[3], &TrackingConfigTable[10],&TrackingConfigTable[6], &TrackingConfigTable[3],	// track 0
	&TrackingConfigTable[4], &TrackingConfigTable[4], &TrackingConfigTable[7], &TrackingConfigTable[4],	// track 1
	&TrackingConfigTable[5], &TrackingConfigTable[5], &TrackingConfigTable[5], &TrackingConfigTable[5],	// track 2
};

void CalculateLoopCoefficients(PCHANNEL_STATE ChannelState, PTRACKING_CONFIG CurTrackingConfig);
void SetNHConfig(PCHANNEL_STATE ChannelState, int NHPos, const unsigned int *NHCode);

//*************** Switch tracking stage of a tracking channel ****************
// Parameters:
//   ChannelState: Pointer to channel state structure
//   TrackingStage: tracking stage to be switched
// Return value:
//   none
void SwitchTrackingStage(PCHANNEL_STATE ChannelState, unsigned int TrackingStage)
{
	PTRACKING_CONFIG CurTrackingConfig = TrackingConfig[STAGE_CONFIG_INDEX(TrackingStage)][ChannelState->FreqID];
	unsigned int PrevStage = (ChannelState->State & STAGE_MASK);
	int CohCount;

	DEBUG_OUTPUT(OUTPUT_CONTROL(TRACKING_SWITCH, INFO), "SV%02d switch to %d\n", ChannelState->Svid, TrackingStage);
	ChannelState->TrackingTime = 0;		// reset tracking time
	ChannelState->State &= ~STAGE_MASK;
	ChannelState->State |= TrackingStage;

	ChannelState->FftCount = 0;
	ChannelState->NonCohCount = 0;
	ChannelState->TrackingTimeout = CurTrackingConfig->TrackingTimeout;
//	if (TrackingStage >= STAGE_PULL_IN)
		CalculateLoopCoefficients(ChannelState, CurTrackingConfig);

	if (TrackingStage == STAGE_HOLD3)
	{
		// restore carrier and code frequency to values before lose lock
		ChannelState->CarrierFreqBase = ChannelState->CarrierFreqSave;
		ChannelState->CodeFreqBase = ChannelState->CodeFreqSave;
		STATE_BUF_SET_CARRIER_FREQ(&(ChannelState->StateBufferCache), ChannelState->CarrierFreqSave);
		STATE_BUF_SET_CODE_FREQ(&(ChannelState->StateBufferCache), ChannelState->CodeFreqSave);
		ChannelState->State |= STATE_CACHE_FREQ_DIRTY;
		ChannelState->CodeSearchCount = 0;
	}
	else if (TrackingStage == STAGE_PULL_IN)
	{
	}
	else if (TrackingStage == STAGE_TRACK)
	{
		// reset data for data decode, switch to tracking stage at epoch of bit edge
		ChannelState->DataStream.PrevReal = ChannelState->DataStream.PrevImag = ChannelState->DataStream.PrevSymbol = 0;
		ChannelState->DataStream.CurReal = ChannelState->DataStream.CurImag = 0;
		ChannelState->DataStream.DataCount = ChannelState->DataStream.CurrentAccTime = 0;
		if (PrevStage == STAGE_BIT_SYNC)	// switch from bit sync, need to align to bit edge
		{
			CohCount = ChannelState->BitSyncResult % CurTrackingConfig->CoherentNumber;
			ChannelState->TrackingTime = ChannelState->BitSyncResult - CohCount;		// reset tracking time from previous bit edge
			STATE_BUF_SET_COH_COUNT(&(ChannelState->StateBufferCache), CohCount);
			ChannelState->State |= STATE_CACHE_STATE_DIRTY;
		}
	}
	// set coherent/FFT/non-coherent number
	STATE_BUF_SET_COH_NUMBER(&(ChannelState->StateBufferCache), CurTrackingConfig->CoherentNumber);
	STATE_BUF_SET_POST_SHIFT(&(ChannelState->StateBufferCache), CurTrackingConfig->PostShift);
	ChannelState->State |= STATE_CACHE_CONFIG_DIRTY;
	ChannelState->CoherentNumber = CurTrackingConfig->CoherentNumber;
	ChannelState->FftNumber = CurTrackingConfig->FftNumber;
	ChannelState->NonCohNumber = CurTrackingConfig->NonCohNumber;
	ChannelState->SmoothedPower = 0;
	ChannelState->PhaseAcc = ChannelState->FrequencyAcc = ChannelState->DelayAcc = 0;
}

//*************** Determine whether tracking stage need to be changed ****************
//* update carrier frequency and code frequency acccording to dicriminator output
// Parameters:
//   ChannelState: Pointer to channel state structure
// Return value:
//   0 for no tracking stage change, 1 for tracking stage change
int StageDetermination(PCHANNEL_STATE ChannelState)
{
	int CurStage = ChannelState->State & STAGE_MASK;
	int Time, Jump;
	unsigned int StateValue;
	int StageChange = 0;
	PSTATE_BUFFER StateBuffer = &(ChannelState->StateBufferCache);

	// B1C and L1C set pilot channel NH after data sync
	if (FREQ_ID_IS_B1C_L1C(ChannelState->FreqID) && (ChannelState->BitSyncResult & 0x1800) && (ChannelState->TrackingTime % 20) == 0)	// data sync finished and at 20ms boundary
	{
		// switch to decode data channel
		STATE_BUF_ENABLE_PRN2(StateBuffer);
		STATE_BUF_ENABLE_BOC(StateBuffer);	// enable BOC
		STATE_BUF_SET_NARROW_FACTOR(StateBuffer, 2);	// set correlator interval to 1/8 chip
		// enable HW data decode
		STATE_BUF_SET_BIT_LENGTH(StateBuffer, 10);
		STATE_BUF_SET_DECODE_BIT(StateBuffer, 3);
		STATE_BUF_DATA_IN_Q(StateBuffer);
		// remove 1.023MHz carrier offset
		ChannelState->StateBufferCache.CarrierFreq -= DIVIDE_ROUND(1023000LL << 32, SAMPLE_FREQ);
		ChannelState->CarrierFreqBase -= DIVIDE_ROUND(1023000LL << 32, SAMPLE_FREQ);
		ChannelState->CarrierFreqSave -= DIVIDE_ROUND(1023000LL << 32, SAMPLE_FREQ);
		ChannelState->State |= (DATA_STREAM_PRN2 | STATE_ENABLE_BOC | NH_SEGMENT_UPDATE | STATE_CACHE_FREQ_DIRTY);
		Time = ChannelState->BitSyncResult & 0x7ff;
		// if negative stream, rotate phase by PI
		if (ChannelState->BitSyncResult & 0x1000)
		{
			StateValue = GetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierPhase)));
			StateValue ^= 0x80000000;
			SetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierPhase)), StateValue);
		}
		// enable NH
		Time += ChannelState->TrackingTime / 10;
		Time %= 1800;	// determine bit position at current time
		ChannelState->FrameCounter = Time;
		SetNHConfig(ChannelState, Time, B1CSecondCode[ChannelState->Svid-1]);
		// switch to track 1 and set STATE_CACHE_CONFIG_DIRTY
		SwitchTrackingStage(ChannelState,  STAGE_TRACK + 1);
		ChannelState->BitSyncResult = 0;
		ChannelState->DataStream.DataCount = ChannelState->DataStream.CurrentAccTime = 0;	// reset data count for data stream decode
		ChannelState->DataStream.StartIndex = ChannelState->FrameCounter;
	}

	// lose lock, switch to hold
	if (CurStage >= STAGE_PULL_IN && ChannelState->LoseLockCounter > 100 && ChannelState->NonCohCount == 0)
	{
		SwitchTrackingStage(ChannelState, STAGE_HOLD3);
		return 1;
	}

	// determine stage switch cases before timeout
	switch (CurStage)
	{
	case STAGE_HOLD3:	// holding when signal lost
		if (ChannelState->FftCount == 0 && ChannelState->NonCohCount == 0)
		{
			if (ChannelState->FastCN0 > 2800)	// signal recovered
			{
				SwitchTrackingStage(ChannelState, STAGE_PULL_IN);
				ChannelState->LoseLockCounter = 0;
				StageChange = 1;
			}
			else	// scan code phases within code search range
			{
				if (++ChannelState->CodeSearchCount == 5)
					Jump = ChannelState->CodeSearchCount / 2;
				else
					Jump = ChannelState->CodeSearchCount;
				if ((ChannelState->CodeSearchCount & 1) == 0)
					Jump = -Jump;
				if (ChannelState->CodeSearchCount == 5)
					ChannelState->CodeSearchCount = 0;
				Jump *= 7;
				StateValue = GetRegValue((U32)(&(ChannelState->StateBufferHW->DumpCount)));
				StateValue |= (Jump & 0xff) << 8;
				SetRegValue((U32)(&(ChannelState->StateBufferHW->DumpCount)),  StateValue);
			}
		}
		break;
	case STAGE_BIT_SYNC:
		if (ChannelState->BitSyncResult)
		{
			if (ChannelState->BitSyncResult < 0)	// bit sync fail
				SwitchTrackingStage(ChannelState, STAGE_RELEASE);
			else
			{
				ChannelState->BitSyncResult = (ChannelState->TrackingTime- ChannelState->BitSyncResult) % 20;	// ms number passed bit edge
				SwitchTrackingStage(ChannelState, STAGE_TRACK);
			}
			StageChange = 1;
		}
		break;
	case STAGE_TRACK + 1:
		if (ChannelState->CNOLowCount > 500)	// CN0 low, switch to track 2
		{
			SwitchTrackingStage(ChannelState, STAGE_TRACK + 2);
			StageChange = 1;
		}
		break;
	case STAGE_TRACK + 2:
		if (ChannelState->CN0HighCount > 500)	// CN0 high, switch to track 0
		{
			SwitchTrackingStage(ChannelState, STAGE_TRACK);
			StageChange = 1;
		}
		break;
	}

	// timeout determineation
	if (StageChange)
		return 1;
	else if (ChannelState->TrackingTimeout < 0 || ChannelState->TrackingTime < ChannelState->TrackingTimeout)	// not timeout, do not switch stage
		return 0;

	// switch after timeout
	switch (CurStage)
	{
	case STAGE_HOLD3:
		SwitchTrackingStage(ChannelState, STAGE_RELEASE);
		break;
	case STAGE_PULL_IN:
		// reset data for bit/frame sync
		ChannelState->BitSyncData.CorDataCount = 0;
		ChannelState->BitSyncData.ChannelState = ChannelState;
		ChannelState->BitSyncData.PrevCorData = 0;	// clear previous correlation result for first round
		memset(ChannelState->ToggleCount, 0, sizeof(ChannelState->ToggleCount));
		ChannelState->BitSyncResult = 0;
		if (FREQ_ID_IS_L1CA(ChannelState->FreqID))	// L1C/A need to do bit sync
			SwitchTrackingStage(ChannelState, STAGE_BIT_SYNC);
		else	// other signal switch to track 0
			SwitchTrackingStage(ChannelState, STAGE_TRACK);
		break;
	case STAGE_TRACK:
		if (ChannelState->CN0 > 2500)	// strong signal switch to track 1
		{
			SwitchTrackingStage(ChannelState,  STAGE_TRACK + 1);
			// if previous decoded acc data sign and symbol not consistent, rotate phase by PI
			// in track 1 stage, will use acc data to determine symbol
			if (((ChannelState->DataStream.PrevReal >> 31) & 1) ^ ChannelState->DataStream.PrevSymbol)
			{
				StateValue = GetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierPhase)));
				StateValue ^= 0x80000000;
				SetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierPhase)),  StateValue);
			}
		}
		else	// weak signal switch to track 2
			SwitchTrackingStage(ChannelState,  STAGE_TRACK + 2);
		break;
	}

	return 1;
}
