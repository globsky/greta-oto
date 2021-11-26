//----------------------------------------------------------------------
// TrackingStage.c:
//   Functions to process tracking stages
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <string.h>
#include "ChannelManager.h"

#define C1 (1<<16)
#define C2 (2<<16)
#define C3 (3<<16)

static TRACKING_CONFIG TrackingConfigTable[] = {
// Coh FFT NonCoh Narrow Post  BnPLL   BnFLL   BnDLL  Timeout
 {  1,  5,   2,      0,    1,      0,  80|C2,  80|C2,   200,},	// 0 for pull-in
 {  4,  5,   4,      0,    2,      0,      0,      0, 30000,},	// 1 for GPS L1 tracking hold
 {  5,  1,   4,      0,    2, 320|C2,   0|C2,  80|C2,  5000,},	// 2 for GPS L1 track 0
 { 20,  1,   4,      0,    3, 240|C2,   0|C2,  40|C2,    -1,},	// 3 for GPS L1 track 1
 {  4,  5,  16,      0,    3,   0|C2,  80|C2,  40|C2,    -1,},	// 4 for GPS L1 track 2
};

PTRACKING_CONFIG TrackingConfig[][4] = {	// pointer to TrackingConfigTable for different stage and different signal
	&TrackingConfigTable[0], &TrackingConfigTable[0], &TrackingConfigTable[0], &TrackingConfigTable[0],
	&TrackingConfigTable[1], &TrackingConfigTable[1], &TrackingConfigTable[1], &TrackingConfigTable[1],
	&TrackingConfigTable[2], &TrackingConfigTable[2], &TrackingConfigTable[2], &TrackingConfigTable[2],
	&TrackingConfigTable[3], &TrackingConfigTable[3], &TrackingConfigTable[3], &TrackingConfigTable[3],
};

void CalculateLoopCoefficients(PCHANNEL_STATE ChannelState, PTRACKING_CONFIG CurTrackingConfig);

//*************** Switch tracking stage of a tracking channel ****************
// Parameters:
//   ChannelState: Pointer to channel state structure
//   TrackingStage: tracking stage to be switched
// Return value:
//   none
void SwitchTrackingStage(PCHANNEL_STATE ChannelState, unsigned int TrackingStage)
{
	PTRACKING_CONFIG CurTrackingConfig = TrackingConfig[STAGE_CONFIG_INDEX(TrackingStage)][ChannelState->FreqID];

	ChannelState->TrackingTime = 0;		// reset tracking time
	ChannelState->State &= ~STAGE_MASK;
	ChannelState->State |= TrackingStage;

	// switch to bit sync stage only change timeout and reset data for bit sync
	if (TrackingStage == STAGE_BIT_SYNC)
	{
		ChannelState->TrackingTimeout = 1500;	// timeout for bit sync not success
		// reset data for bit sync
		ChannelState->BitSyncData.CorDataCount = 0;
		ChannelState->BitSyncData.ChannelState = ChannelState;
		ChannelState->BitSyncData.PrevCorData = 0;	// clear previous correlation result for first round
		memset(ChannelState->ToggleCount, 0, sizeof(ChannelState->ToggleCount));
		ChannelState->BitSyncResult = 0;
		return;
	}

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

	// switch out from hold
	if (TrackingStage == STAGE_HOLD3 && (ChannelState->FastCN0 > 2800))
	{
		SwitchTrackingStage(ChannelState, STAGE_PULL_IN);
		ChannelState->LoseLockCounter = 0;
		return 1;
	}

	// lose lock, switch to hold
	if (TrackingStage >= STAGE_PULL_IN && ChannelState->LoseLockCounter > 10 && ChannelState->NonCohCount == 0)
	{
		SwitchTrackingStage(ChannelState, STAGE_HOLD3);
		return 1;
	}

	// CN0 change
	if (TrackingStage == STAGE_TRACK + 1 && ChannelState->CNOLowCount > 500)	// CN0 low, switch to track 2
	{
		SwitchTrackingStage(ChannelState, STAGE_TRACK + 2);
		return 1;
	}
	if (TrackingStage == STAGE_TRACK + 2 && ChannelState->CN0HighCount > 500)	// CN0 high, switch to track 0
	{
		SwitchTrackingStage(ChannelState, STAGE_TRACK);
		return 1;
	}

	if (ChannelState->TrackingTimeout < 0 || ChannelState->TrackingTime < ChannelState->TrackingTimeout)	// not timeout, do not switch stage
		return 0;
	if (TrackingStage == STAGE_PULL_IN)
	{
		SwitchTrackingStage(ChannelState, STAGE_BIT_SYNC);
	}
	else if (TrackingStage == STAGE_HOLD3)
	{
		SwitchTrackingStage(ChannelState, STAGE_RELEASE);
	}
	else if (TrackingStage >= STAGE_TRACK)
	{
		SwitchTrackingStage(ChannelState, (ChannelState->CN0 > 2500) ? STAGE_TRACK + 1 : STAGE_TRACK + 2);
	}
	return 1;
}
