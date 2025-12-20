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
 {  5,  1,   4,      0,    2, 320|C2,   0|C2,  80|C2,     0,},	// 6 for BDS B1C track 0
 { 10,  1,   4,      0,    3, 240|C2,   0|C2,  40|C2,    -1,},	// 7 for BDS B1C track 1
 {  1,  4,   2,      0,    1,      0,  80|C2,  80|C2,   200,},	// 8 for GAL E1 pull-in
 {  4,  1,   5,      0,    2, 320|C2,   0|C2,  80|C2,  1500,},	// 9 for GAL E1 bit_sync
 {  4,  1,   5,      0,    2, 320|C2,   0|C2,  80|C2,  5000,},	//10 for GAL E1 track 0
};

PTRACKING_CONFIG TrackingConfig[][4] = {	// pointer to TrackingConfigTable for different stage and different signal
//           L1CA                      E1                      B1C                      L1C
	&TrackingConfigTable[2], &TrackingConfigTable[2], &TrackingConfigTable[2], &TrackingConfigTable[2],	// tracking hold
	&TrackingConfigTable[0], &TrackingConfigTable[8], &TrackingConfigTable[0], &TrackingConfigTable[0],	// pull-in
	&TrackingConfigTable[1], &TrackingConfigTable[9], &TrackingConfigTable[1], &TrackingConfigTable[1],	// bit_sync
	&TrackingConfigTable[3], &TrackingConfigTable[10],&TrackingConfigTable[6], &TrackingConfigTable[6],	// track 0
	&TrackingConfigTable[4], &TrackingConfigTable[4], &TrackingConfigTable[7], &TrackingConfigTable[7],	// track 1
	&TrackingConfigTable[5], &TrackingConfigTable[5], &TrackingConfigTable[5], &TrackingConfigTable[5],	// track 2
};

void SetNHConfig(PCHANNEL_STATE ChannelState, int NHIndex, int NHPos, const unsigned int *NHCode);
void CalculateLoopCoefficients(PCHANNEL_STATE ChannelState, PTRACKING_CONFIG CurTrackingConfig);
static int StageSwitchCondition(PCHANNEL_STATE ChannelState);

//*************** Switch tracking stage of a tracking channel ****************
// Parameters:
//   ChannelState: Pointer to channel state structure
//   TrackingStage: tracking stage to be switched
// Return value:
//   none
void SwitchTrackingStage(PCHANNEL_STATE ChannelState, unsigned int TrackingStage)
{
	PTRACKING_CONFIG CurTrackingConfig = TrackingConfig[STAGE_CONFIG_INDEX(TrackingStage)][ChannelState->Signal];
	unsigned int PrevStage = (ChannelState->State & STAGE_MASK);
	int CohCount, Time = ChannelState->TrackingTime / 10;
	unsigned int StateValue;
	PSTATE_BUFFER StateBuffer = &(ChannelState->StateBufferCache);

	DEBUG_OUTPUT(OUTPUT_CONTROL(TRACKING_SWITCH, INFO), "%c%02d switch to %d at tick %d\n", "GECG"[ChannelState->Signal], ChannelState->Svid, TrackingStage, ChannelState->TickCount);
	SET_STAGE(ChannelState, TrackingStage);
	ChannelState->TrackingTime = 0;		// reset tracking time
	if (TrackingStage == STAGE_RELEASE)		// next acc interrupt will do release operation
		return;

	ChannelState->FftCount = 0;
	ChannelState->NonCohCount = 0;
	ChannelState->TrackingTimeout = CurTrackingConfig->TrackingTimeout;
//	if (TrackingStage >= STAGE_PULL_IN)
		CalculateLoopCoefficients(ChannelState, CurTrackingConfig);

	if (TrackingStage == STAGE_HOLD3)
	{
		// restore carrier and code frequency to values before lose lock
		ChannelState->CarrierFreqBase = ChannelState->CarrierFreqSave;
//		ChannelState->CodeFreqBase = ChannelState->CodeFreqSave;
		ChannelState->CodeFreqBase = CARR_TO_CODE_FREQ(ChannelState->CarrierFreqSave);
		STATE_BUF_SET_CARRIER_FREQ(&(ChannelState->StateBufferCache), ChannelState->CarrierFreqBase);
		STATE_BUF_SET_CODE_FREQ(&(ChannelState->StateBufferCache), ChannelState->CodeFreqBase);
		ChannelState->State |= STATE_CACHE_FREQ_DIRTY;
		ChannelState->CodeSearchCount = 0;
	}
	else if (TrackingStage == STAGE_PULL_IN)
	{
	}
	else if (TrackingStage == STAGE_TRACK0)
	{
		if (PrevStage == STAGE_BIT_SYNC)	// switch from bit sync, need to align to bit edge
		{
			// reset data for data decode, switch to tracking stage at epoch of bit edge
			ChannelState->DataStream.PrevReal = ChannelState->DataStream.PrevImag = 0;
			ChannelState->DataStream.CurReal = ChannelState->DataStream.CurImag = 0;
			ChannelState->DataStream.BitCount = ChannelState->DataStream.CurrentAccTime = 0;
			if (SIGNAL_IS_L1CA(ChannelState->Signal))	// for L1CA
			{
				CohCount = ChannelState->BitSyncResult % CurTrackingConfig->CoherentNumber;
				ChannelState->DataStream.CurrentAccTime = ChannelState->TrackingTime = ChannelState->BitSyncResult - CohCount;		// reset tracking time from previous bit edge
				STATE_BUF_SET_COH_COUNT(&(ChannelState->StateBufferCache), CohCount);
				ChannelState->State |= STATE_CACHE_STATE_DIRTY;
			}
			else	// for E1
			{
				// switch to decode data channel
				STATE_BUF_ENABLE_PRN2(StateBuffer);
				STATE_BUF_ENABLE_BOC(StateBuffer);	// enable BOC
				STATE_BUF_SET_NARROW_FACTOR(StateBuffer, 2);	// set correlator interval to 1/8 chip
				// enable HW data decode
				STATE_BUF_SET_BIT_LENGTH(StateBuffer, 4);
				STATE_BUF_SET_DECODE_BIT(StateBuffer, 2);
				STATE_BUF_DATA_IN_I(StateBuffer);
				// remove 1.023MHz carrier offset
				ChannelState->StateBufferCache.CarrierFreq -= DIVIDE_ROUND(1023000LL << 32, SAMPLE_FREQ);
				ChannelState->CarrierFreqBase = ChannelState->CarrierFreqSave = ChannelState->StateBufferCache.CarrierFreq;
				ChannelState->State |= (STATE_4QUAD_DISC | DATA_STREAM_PRN2 | STATE_ENABLE_BOC | STATE_CACHE_FREQ_DIRTY);
				// adjust carrier phase
				StateValue = GetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierPhase)));
				StateValue += 0x40000000;	// compensate pi/2 for sideband to BOC tracking and pi for negative stream
				SetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierPhase)), StateValue);
				// enable NH
				STATE_BUF_SET_NH_CONFIG(&(ChannelState->StateBufferCache), 25, 0x9b501c);
				// set current NH count and coherent count
				CohCount = ChannelState->BitSyncResult % CurTrackingConfig->CoherentNumber;
				ChannelState->TrackingTime = ChannelState->BitSyncResult % 20;		// reset tracking time from previous 20ms boundary
				STATE_BUF_SET_COH_COUNT(&(ChannelState->StateBufferCache), CohCount);
				STATE_BUF_SET_NH_COUNT(&(ChannelState->StateBufferCache), ChannelState->BitSyncResult / 4);
				ChannelState->State |= STATE_CACHE_STATE_DIRTY;
				ChannelState->DataStream.BitCount = ChannelState->DataStream.CurrentAccTime = 0;	// reset data count for data stream decode
			}
		}
		else if (PrevStage == STAGE_HOLD3)
		{
			if (SIGNAL_IS_B1C_L1C(ChannelState->Signal))	// B1C/L1C from HOLD3 to TRACK0, sync secondary code again
			{
				ChannelState->State &= ~(STATE_4QUAD_DISC | DATA_STREAM_PRN2 | NH_SEGMENT_UPDATE);
				STATE_BUF_DISABLE_PRN2(StateBuffer);
				STATE_BUF_SET_NH_CONFIG(StateBuffer, 0, 0);
				ChannelState->State |= STATE_CACHE_CONFIG_DIRTY;
			}
		}
	}
	else if (TrackingStage == STAGE_TRACK1)
	{
		if (SIGNAL_IS_B1C_L1C(ChannelState->Signal))
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
			if (!(ChannelState->State & STATE_ENABLE_BOC))
				ChannelState->StateBufferCache.CarrierFreq -= DIVIDE_ROUND(1023000LL << 32, SAMPLE_FREQ);
			ChannelState->CarrierFreqBase = ChannelState->CarrierFreqSave = ChannelState->StateBufferCache.CarrierFreq;
			ChannelState->State |= (STATE_4QUAD_DISC | DATA_STREAM_PRN2 | STATE_ENABLE_BOC | NH_SEGMENT_UPDATE | STATE_CACHE_FREQ_DIRTY);
			Time += ChannelState->BitSyncResult & 0x7ff;
			// adjust carrier phase
			StateValue = GetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierPhase)));
			StateValue += (ChannelState->BitSyncResult & 0x1000) ? 0xc0000000 : 0x40000000;	// compensate pi/2 for sideband to BOC tracking and pi for negative stream
			SetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierPhase)), StateValue);
			// enable NH
			Time %= 1800;	// determine bit position at current time
			ChannelState->NHIndex = Time / 20;
			SetNHConfig(ChannelState, ChannelState->NHIndex, Time - ChannelState->NHIndex * 20, B1CSecondCode[ChannelState->Svid-1]);
			ChannelState->BitSyncResult = 0;
			ChannelState->DataStream.BitCount = ChannelState->DataStream.CurrentAccTime = 0;	// reset data count for data stream decode
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
	int Jump;
	unsigned int StateValue;
	int StageChange = 0;
	PSTATE_BUFFER StateBuffer = &(ChannelState->StateBufferCache);

	// lose lock, switch to hold
	if (CurStage >= STAGE_PULL_IN && (ChannelState->CarrLoseLockCounter > 240 || ChannelState->CodeLoseLockCounter > 240) && ChannelState->NonCohCount == 0)
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
				// force adjust carrier frequency and align correlator peak
				ChannelState->StateBufferCache.CarrierFreq += ChannelState->FrequencyDiff;
				ChannelState->State |= STATE_CACHE_FREQ_DIRTY;
				Jump = (ChannelState->DelayDiff + 57344) / 16384 - 3;	// apply 3.5 CorInterval offset (16384) then round down to get nearest rounding
				if (Jump != 0)
				{
					StateValue = GetRegValue((U32)(&(ChannelState->StateBufferHW->DumpCount)));
					StateValue |= (Jump & 0xff) << 8;
					SetRegValue((U32)(&(ChannelState->StateBufferHW->DumpCount)),  StateValue);
				}
				SwitchTrackingStage(ChannelState, STAGE_TRACK0);
				ChannelState->CarrLoseLockCounter = ChannelState->CodeLoseLockCounter = 0;
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
				if (SIGNAL_IS_L1CA(ChannelState->Signal))	// for L1CA
					ChannelState->BitSyncResult = (ChannelState->TrackingTime- ChannelState->BitSyncResult) % 20;	// ms number passed bit edge
				else	// for E1, recalculate current millisecond count within 100ms NH cycle
					ChannelState->BitSyncResult = (1000 - (ChannelState->BitSyncResult - ChannelState->TrackingTime)) % 100;	// add 100 to make sure result if positive
				SwitchTrackingStage(ChannelState, STAGE_TRACK0);
			}
			StageChange = 1;
		}
		break;
	case STAGE_TRACK1:
		if (ChannelState->CNOLowCount > 500)	// CN0 low, switch to track 2
		{
			SwitchTrackingStage(ChannelState, STAGE_TRACK2);
			StageChange = 1;
		}
		break;
	case STAGE_TRACK2:
		if (ChannelState->CN0HighCount > 500)	// CN0 high, switch to track 0
		{
			SwitchTrackingStage(ChannelState, STAGE_TRACK0);
			StageChange = 1;
		}
		break;
	}

	// timeout determineation
	if (StageChange)
		return 1;
	else if (ChannelState->TrackingTimeout == 0 && StageSwitchCondition(ChannelState) == 0)
		return 0;
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
		if (SIGNAL_IS_B1C_L1C(ChannelState->Signal))	// B1C/L1C secondary code sync performed at TRACK0 stage
			SwitchTrackingStage(ChannelState, STAGE_TRACK0);
		else	// other signal switch to bit-sync
			SwitchTrackingStage(ChannelState, STAGE_BIT_SYNC);
		break;
	case STAGE_TRACK0:
		if (ChannelState->CN0 > 2500)	// strong signal switch to track 1
		{
			SwitchTrackingStage(ChannelState,  STAGE_TRACK1);
			// if previous decoded acc data sign and symbol not consistent, rotate phase by PI
			// in track 1 stage, will use acc data to determine symbol
			if (SIGNAL_IS_L1CA(ChannelState->Signal) && (((ChannelState->DataStream.PrevReal >> 31) & 1) ^ (ChannelState->DataStream.Symbols & 1)))
			{
				StateValue = GetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierPhase)));
				StateValue ^= 0x80000000;
				SetRegValue((U32)(&(ChannelState->StateBufferHW->CarrierPhase)),  StateValue);
			}
		}
		else	// weak signal switch to track 2
			SwitchTrackingStage(ChannelState,  STAGE_TRACK2);
		break;
	}

	return 1;
}

// determine whether stage switch condition satisfied
int StageSwitchCondition(PCHANNEL_STATE ChannelState)
{
	if ((ChannelState->State & STAGE_MASK) == STAGE_TRACK0)
	{
		switch (ChannelState->Signal)
		{
		case SIGNAL_L1CA:
			return (ChannelState->TrackingTime >= 500) ? 1 : 0;
		case SIGNAL_E1:
			return (ChannelState->TrackingTime >= 500) ? 1 : 0;
		case SIGNAL_B1C:
		case SIGNAL_L1C:
			return ((ChannelState->BitSyncResult & 0x1800) && (ChannelState->TrackingTime % 20) == 0) ? 1 : 0;
		}
	}
	return 0;
}
