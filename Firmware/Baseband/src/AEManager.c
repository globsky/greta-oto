//----------------------------------------------------------------------
// AEManager.c:
//   Acquisition engine management functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include "RegAddress.h"
#include "BBDefines.h"
#include "HWCtrl.h"
#include "PlatformCtrl.h"
#include "AEManager.h"
#include "TEManager.h"
#include "ChannelManager.h"
#include "TaskManager.h"

#define ACQ_TASK_NUMBER 4

static PACQ_CONFIG CurAcqTask;
static unsigned int AcqTaskPending;
static int CurSignalType;
static unsigned int AcqBufferTimeTag;
static ACQ_CONFIG AcqConfig[ACQ_TASK_NUMBER];

static void DoAcqTask();
static void FillAeBuffer(PACQ_CONFIG pAcqConfig);
static void DoVerification(void);

// different strategy to do acquisition
static const SEARCH_CONFIG SearchConfigArray[] = {
	// Coh  Noncoh Stride Interval
	{   4,     1,    19,    500, },	// 0 for BPSK (L1C/A) cold acquisiton
	{   4,     1,     3,    500, },	// 1 for BPSK (L1C/A) hot/warm acquisiton
	{   4,     2,    19,    500, },	// 2 for BOC signal cold acquisiton
	{   4,     2,     3,    500, },	// 3 for BOC signal hot/warm acquisiton
	{   4,     2,     1,    300, },	// 4 for verification
};

//*************** AE initialization ****************
// Parameters:
//   none
// Return value:
//   none
void AEInitialize(void)
{
	CurSignalType = -1;
	CurAcqTask= (PACQ_CONFIG)0;
	AcqTaskPending = 0;
	AcqBufferTimeTag = 0;
}

//*************** Get a new free acquire task ****************
// Parameters:
//   none
// Return value:
//   pointer to a ACQ_CONFIG structure if there is free acquire task
//   otherwise return NULL pointer
PACQ_CONFIG GetFreeAcqTask(void)
{
	int i;

	for (i = 0; i < ACQ_TASK_NUMBER; i ++)
		if ((AcqTaskPending & (1 << i)) == 0)
			return &AcqConfig[i];
	return (PACQ_CONFIG)0;
}

//*************** Add a new acquire task ****************
// After adding a new acquire task, the task will be executed
// immidiately if AE not busy, otherwise is is pending until current
// acquisition task is finished
// Parameters:
//   pAcqConfig: pointer to a ACQ_CONFIG structure
// Return value:
//   0
int AddAcqTask(PACQ_CONFIG pAcqConfig)
{
	int TaskIndex = pAcqConfig - AcqConfig;

	// set pending flag
	AcqTaskPending |= (1 << TaskIndex);
	// set search configuration according to search mode
	if ((pAcqConfig->SearchMode & SEARCH_MODE_TYPE_MASK) == SEARCH_MODE_TYPE_BPSK)
	{
		pAcqConfig->SearchConfig = ((pAcqConfig->SearchMode & SEARCH_MODE_FREQ_MASK) == SEARCH_MODE_FREQ_FULL) ? &SearchConfigArray[0] : &SearchConfigArray[1];
	}
	else
	{
		pAcqConfig->SearchConfig = ((pAcqConfig->SearchMode & SEARCH_MODE_FREQ_MASK) == SEARCH_MODE_FREQ_FULL) ? &SearchConfigArray[2] : &SearchConfigArray[3];
	}
	if (CurAcqTask == NULL)	// no AE task is undergoing
		DoAcqTask();
	return 0;
}

//*************** Perform a acquire task ****************
// Fill AE buffer first if data in AE buffer is not valid
// acquisition will be performed after AE buffer filled with
// sufficient data (in request task), otherwise acquisition
// will start immediately
// Parameters:
//   none
// Return value:
//   none
void DoAcqTask()
{
	int i;

	for (i = 0; i < ACQ_TASK_NUMBER; i ++)
		if ((AcqTaskPending & (1 << i)) != 0)
			break;
	if (i == ACQ_TASK_NUMBER)	// no task pending
		return;

	CurAcqTask = &AcqConfig[i];
	if (CurSignalType != (CurAcqTask->SearchMode & SEARCH_MODE_TYPE_MASK) || (BasebandTickCount - AcqBufferTimeTag) > 30000)	// AE buffer not filled desired signal or too old (>4s)
		FillAeBuffer(CurAcqTask);
	else
		StartAcquisition();
}

//*************** Perform a acquire task ****************
// Fill AE buffer, threshold set according to data used to
// do acquisition (coherent/non-coherent times, code search range)
// sufficient data (in request task), otherwise acquisition
// will start immediately
// carrier frequency will have 1.023MHz bias for BOC signal
// Parameters:
//   pAcqConfig: pointer to a ACQ_CONFIG structure
// Return value:
//   none
void FillAeBuffer(PACQ_CONFIG pAcqConfig)
{
	int i, PhaseRange = 0;
	int CorrelationRange = pAcqConfig->SearchConfig->CohNumber * pAcqConfig->SearchConfig->NoncohNumber;	// number of 1ms samples used for acquisition

	CurSignalType = pAcqConfig->SearchMode & SEARCH_MODE_TYPE_MASK;
	AcqBufferTimeTag = BasebandTickCount;

	// calculate lagest CodeSpan that will use extra signal
	for (i = 0; i < pAcqConfig->AcqChNumber; i ++)
		if (PhaseRange < (pAcqConfig->SatConfig[i].CodeSpan + 2) / 3)
			PhaseRange = (pAcqConfig->SatConfig[i].CodeSpan + 2) / 3;
	CorrelationRange += PhaseRange;

	SetRegValue(ADDR_AE_CARRIER_FREQ, CARRIER_FREQ(CurSignalType ? 1023000 : 0));
	SetRegValue(ADDR_AE_CODE_RATIO, (int)(2.046e6 / SAMPLE_FREQ * 16777216. + 0.5));
	SetRegValue(ADDR_AE_THRESHOLD, 37);
	SetRegValue(ADDR_AE_BUFFER_CONTROL, 0x300 + CorrelationRange);	// start fill AE buffer

	AddWaitRequest(WAIT_TASK_AE, CorrelationRange + 1);	// wait extra 1ms to make sure AE buffer fill complete
}

//*************** AE ISR ****************
// If current stage is last stage (verification)
// will add a request task to do A2T, otherwise
// will move to next acquisition stage
// Parameters:
//   none
// Return value:
//   none
void AeInterruptProc()
{
	PACQ_CONFIG pAcqConfig = CurAcqTask;
	int TaskIndex = pAcqConfig - AcqConfig;

	// call ProcessAcqResult on request interrupt, so it will sync with TE interrupt
	if ((pAcqConfig->SearchMode & SEARCH_MODE_STAGE_MASK) == SEARCH_MODE_STAGE_VERIFY)	// last search stage
	{
		SetRegValue(ADDR_TE_FIFO_CONFIG, 0);			// FIFO config, disable dummy write
		// call ProcessAcqResult on request interrupt, so it will sync with TE interrupt
		AddToTask(TASK_REQUEST, ProcessAcqResult, &pAcqConfig, sizeof(PACQ_CONFIG));
		AcqTaskPending &= ~(1 << TaskIndex);
		CurAcqTask = NULL;
	}
	else
		DoVerification();
}

//*************** Start acquisition with given configuration ****************
//* this function is a task function
// Parameters:
//   none
// Return value:
//   none
void StartAcquisition(void)
{
	int i;
	const SEARCH_CONFIG *SearchConfig = CurAcqTask->SearchConfig;
	int DftFreq = (SearchConfig->StrideInterval << 10) / 1000;
	unsigned int ConfigData[4];

	ConfigData[0] = 0x04000000 | (SearchConfig->NoncohNumber << 16) | (SearchConfig->CohNumber << 8) | SearchConfig->StrideNumber;	// threshold: 3'b100
	ConfigData[3]= AE_STRIDE_INTERVAL(SearchConfig->StrideInterval);
	for (i = 0; i < CurAcqTask->AcqChNumber; i ++)
	{
		ConfigData[1] = (CurAcqTask->SatConfig[i].SignalSvid << 24) | (AE_CENTER_FREQ(CurAcqTask->SatConfig[i].CenterFreq) & 0xfffff);
		ConfigData[2] = (DftFreq << 20) | CurAcqTask->SatConfig[i].CodeSpan;
		SetRegValue(ADDR_BASE_AE_BUFFER+i*32+ 0, ConfigData[0]);
		SetRegValue(ADDR_BASE_AE_BUFFER+i*32+ 4, ConfigData[1]);
		SetRegValue(ADDR_BASE_AE_BUFFER+i*32+ 8, ConfigData[2]);
		SetRegValue(ADDR_BASE_AE_BUFFER+i*32+12, ConfigData[3]);
	}
#if defined MODEL_RUN	// put in result instead, this is just for input IF file sim_signal_L1CA.bin
	SetRegValue(ADDR_BASE_AE_BUFFER+0x10, 0x840088da); SetRegValue(ADDR_BASE_AE_BUFFER+0x14, 0xfbe384ee); SetRegValue(ADDR_BASE_AE_BUFFER+0x18, 0x6a040538); SetRegValue(ADDR_BASE_AE_BUFFER+0x1c, 0x69138698);
	SetRegValue(ADDR_BASE_AE_BUFFER+0x30, 0x8500881b); SetRegValue(ADDR_BASE_AE_BUFFER+0x34, 0x8df28394); SetRegValue(ADDR_BASE_AE_BUFFER+0x38, 0x350285d2); SetRegValue(ADDR_BASE_AE_BUFFER+0x3c, 0x34f5032a);
	SetRegValue(ADDR_BASE_AE_BUFFER+0x50, 0x850086a8); SetRegValue(ADDR_BASE_AE_BUFFER+0x54, 0x8d0904ad); SetRegValue(ADDR_BASE_AE_BUFFER+0x58, 0x3602858e); SetRegValue(ADDR_BASE_AE_BUFFER+0x5c, 0x320b8130);
	SetRegValue(ADDR_BASE_AE_BUFFER+0x70, 0x85008677); SetRegValue(ADDR_BASE_AE_BUFFER+0x74, 0x9f1585dd); SetRegValue(ADDR_BASE_AE_BUFFER+0x78, 0x350c078e); SetRegValue(ADDR_BASE_AE_BUFFER+0x7c, 0x33f287e5);
	SetRegValue(ADDR_BASE_AE_BUFFER+0x90, 0x8500877c); SetRegValue(ADDR_BASE_AE_BUFFER+0x94, 0x87030585); SetRegValue(ADDR_BASE_AE_BUFFER+0x98, 0x350b040d); SetRegValue(ADDR_BASE_AE_BUFFER+0x9c, 0x351a00a5);
	SetRegValue(ADDR_BASE_AE_BUFFER+0xb0, 0x85008843); SetRegValue(ADDR_BASE_AE_BUFFER+0xb4, 0x8e1f83cc); SetRegValue(ADDR_BASE_AE_BUFFER+0xb8, 0x352783cc); SetRegValue(ADDR_BASE_AE_BUFFER+0xbc, 0x33dc0565);
	SetRegValue(ADDR_BASE_AE_BUFFER+0xd0, 0x850082cb); SetRegValue(ADDR_BASE_AE_BUFFER+0xd4, 0x96e7802b); SetRegValue(ADDR_BASE_AE_BUFFER+0xd8, 0x3ae0002b); SetRegValue(ADDR_BASE_AE_BUFFER+0xdc, 0x31dc03e3);
	SetRegValue(ADDR_BASE_AE_BUFFER+0xf0, 0x850087c6); SetRegValue(ADDR_BASE_AE_BUFFER+0xf4, 0x860b03db); SetRegValue(ADDR_BASE_AE_BUFFER+0xf8, 0x5f0c03db); SetRegValue(ADDR_BASE_AE_BUFFER+0xfc, 0x351b8355);
	SetRegValue(ADDR_BASE_AE_BUFFER+0x110, 0x850087b2); SetRegValue(ADDR_BASE_AE_BUFFER+0x114, 0x971486ae); SetRegValue(ADDR_BASE_AE_BUFFER+0x118, 0x481386ae); SetRegValue(ADDR_BASE_AE_BUFFER+0x11c, 0x36f403d5);
	SetRegValue(ADDR_AE_CONTROL, 0x100);
#else	// start AE
	SetRegValue(ADDR_AE_CONTROL, 0x100+CurAcqTask->AcqChNumber);
#endif
}

//*************** Do verification with given search result ****************
//* this function is an ISR function
// Parameters:
//   none
// Return value:
//   none
void DoVerification(void)
{
	int i, SatNumber = 0;
	int CodePhase, Doppler;
	unsigned int BufferData, Amp3;
	U32 AcqResult[4];

	DEBUG_OUTPUT(OUTPUT_CONTROL(ACQUISITION, INFO), "Acquire Result at search stage\n");
	for (i = 0; i < CurAcqTask->AcqChNumber; i ++)
	{
		LoadMemory(AcqResult, (U32 *)(ADDR_BASE_AE_BUFFER + i * 32 + 16), 16);
		Doppler = ((int)(AcqResult[1] << 8)) >> 23;
		Doppler = CurAcqTask->SatConfig[i].CenterFreq + (Doppler * 2 - 7) * CurAcqTask->SearchConfig->StrideInterval / 16;
		CodePhase = AcqResult[1] & 0x7fff;	// acquired code position, 2x chip scale
		DEBUG_OUTPUT(OUTPUT_CONTROL(ACQUISITION, INFO), "Ch%02d %08x %08x %08x %08x ", i, AcqResult[0], AcqResult[1], AcqResult[2], AcqResult[3]);
		DEBUG_OUTPUT(OUTPUT_CONTROL(ACQUISITION, INFO), "Svid%2d Amp=%3d Cor=%5d Freq=%d\n",  GET_SVID(CurAcqTask->SatConfig[i].SignalSvid), AcqResult[1] >> 24, CodePhase, Doppler);

		Amp3 = AcqResult[3] >> 24;	// get third peak amp
		Amp3 += Amp3 >> 1;	// *1.5
		BufferData = AcqResult[1];	// get first peak
		if ((BufferData >> 24) < Amp3)	// search fail
			continue;
		// assign new search
		CurAcqTask->SatConfig[SatNumber].SignalSvid = CurAcqTask->SatConfig[i].SignalSvid;
		CurAcqTask->SatConfig[SatNumber].CodeSpan = CurAcqTask->SatConfig[i].CodeSpan;
		CurAcqTask->SatConfig[SatNumber].CenterFreq = Doppler;
		SatNumber ++;
	}
	CurAcqTask->AcqChNumber = SatNumber;
	CurAcqTask->SearchMode &= ~SEARCH_MODE_STAGE_MASK; CurAcqTask->SearchMode |= SEARCH_MODE_STAGE_VERIFY;
	CurAcqTask->SearchConfig = &SearchConfigArray[4];
	StartAcquisition();
}

//*************** Start acquisition with given configuration ****************
//* this function is a task function
// Parameters:
//   none
// Return value:
//   1 if AE buffer fill reaches threshold, otherwise 0
int AcqBufferReachTh(void)
{
	return (GetRegValue(ADDR_AE_STATUS) & 0x20000) ? 1 : 0;
}

//*************** Add channel of acquired satellite to tracking engine ****************
//* this function is a task function
// Parameters:
//   Param: pointer to configuration structure
// Return value:
//   0
int ProcessAcqResult(void *Param)
{
	PACQ_CONFIG pAcqConfig = *((PACQ_CONFIG *)Param);
	int AddressGap, PhaseGap, TimeGap;
	U32 RegValue;
	int CodePhase, Doppler;
	int ReadRound, ReadAddress, WriteAddress;
	int LatchRound, LatchAddress;
	int i;
	U32 AcqResult[4];
	PCHANNEL_STATE NewChannel;

	CurAcqTask = (PACQ_CONFIG)0;

	// get AE latch address
	RegValue = GetRegValue(ADDR_TE_FIFO_LWADDR_AE);
	LatchRound = (RegValue >> 16);
	LatchAddress = ((RegValue >> 2) & 0x3fff) + LatchRound * 10240;
	// get write address and round
	RegValue = GetRegValue(ADDR_TE_FIFO_WRITE_ADDR);
	ReadRound = (RegValue >> 16);
	WriteAddress = (RegValue >> 2) & 0x3fff;
	// get read address and compare with write address to determine whether it has roll-over
	RegValue = GetRegValue(ADDR_TE_FIFO_READ_ADDR);
	if ((int)RegValue > WriteAddress)	// write address roll-over
		ReadRound --;
	ReadAddress = RegValue + ReadRound * 10240;
	AddressGap = ReadAddress - LatchAddress;
	if (AddressGap < 0)
		AddressGap += ((1 << 16) * 10240);	// 2^16 * 10240
	// get remnant of address gap
	TimeGap = AddressGap / SAMPLES_1MS;	// time elapsed in ms
	AddressGap %= (SAMPLES_1MS * 20);	// remnant of 20ms
	PhaseGap = (AddressGap * 1023 * 16) / SAMPLES_1MS;	// convert to code phase with 16x scale

	DEBUG_OUTPUT(OUTPUT_CONTROL(ACQUISITION, INFO), "Acquire Result at verify stage\n");
	for (i = 0; i < pAcqConfig->AcqChNumber; i ++)
	{
		LoadMemory(AcqResult, (U32 *)(ADDR_BASE_AE_BUFFER + i * 32 + 16), 16);
		Doppler = ((int)(AcqResult[1] << 8)) >> 23;
		Doppler = pAcqConfig->SatConfig[i].CenterFreq + (Doppler * 2 - 7) * pAcqConfig->SearchConfig->StrideInterval / 16;
		CodePhase = AcqResult[1] & 0x7fff;	// acquired code position, 2x chip scale
		DEBUG_OUTPUT(OUTPUT_CONTROL(ACQUISITION, INFO), "Ch%02d %08x %08x %08x %08x ", i, AcqResult[0], AcqResult[1], AcqResult[2], AcqResult[3]);
		DEBUG_OUTPUT(OUTPUT_CONTROL(ACQUISITION, INFO), "Svid%2d Amp=%3d Cor=%5d Freq=%d\n",  GET_SVID(pAcqConfig->SatConfig[i].SignalSvid), AcqResult[1] >> 24, CodePhase, Doppler);
		CodePhase = PhaseGap - (CodePhase - 5) * 8;	// additional 5 correlator interval (interval at 1/2 chip) to set peak at Cor4
		CodePhase += TimeGap * Doppler / 96250;	// 16 x Doppler x dt / 1540 (dt = TimeGap / 1000)
		if (CodePhase < 0)
			CodePhase += 20 * 1023 * 16;	// 20ms code phase round
//		if (GET_FREQ_ID(pAcqConfig->SatConfig[i].FreqSvid) != FREQ_E1)
//			continue;
//		if (GET_SVID(pAcqConfig->SatConfig[i].FreqSvid) != 19 && GET_SVID(pAcqConfig->SatConfig[i].FreqSvid) != 1)
//			continue;
		if ((NewChannel = GetAvailableChannel()) != NULL)
		{
			NewChannel->Signal = GET_SIGNAL(pAcqConfig->SatConfig[i].SignalSvid);
			NewChannel->Svid = GET_SVID(pAcqConfig->SatConfig[i].SignalSvid);
			InitChannel(NewChannel);
			ConfigChannel(NewChannel, Doppler, CodePhase);
		}
//		break;
	}
	UpdateChannels();
	SetRegValue(ADDR_TE_CHANNEL_ENABLE, GetChannelEnable());

	DoAcqTask();

	return 0;
}
