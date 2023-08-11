//----------------------------------------------------------------------
// AEManager.c:
//   Acquisition engine management functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "RegAddress.h"
#include "BBDefines.h"
#include "HWCtrl.h"
#include "AEManager.h"
#include "TEManager.h"
#include "ChannelManager.h"
#include "TaskManager.h"

#define ACQ_TASK_NUMBER 2

static PACQ_CONFIG CurAcqTask;
static unsigned int AcqTaskPending;
static int CurSignalType;
static unsigned int AcqBufferTimeTag;
static ACQ_CONFIG AcqConfig[ACQ_TASK_NUMBER];

static void DoAcqTask();
static void FillAeBuffer(PACQ_CONFIG pAcqConfig);

void AEInitialize(void)
{
	CurSignalType = -1;
	CurAcqTask= (PACQ_CONFIG)0;
	AcqTaskPending = 0;
	AcqBufferTimeTag = 0;
}

PACQ_CONFIG GetFreeAcqTask(void)
{
	int i;

	for (i = 0; i < ACQ_TASK_NUMBER; i ++)
		if ((AcqTaskPending & (1 << i)) == 0)
			return &AcqConfig[i];
	return (PACQ_CONFIG)0;
}

int AddAcqTask(PACQ_CONFIG pAcqConfig)
{
	int TaskIndex = pAcqConfig - AcqConfig;
	AcqTaskPending |= (1 << TaskIndex);
	if (CurAcqTask == NULL)	// no AE task is undergoing
		DoAcqTask();
	return 0;
}

void DoAcqTask()
{
	int i;

	for (i = 0; i < ACQ_TASK_NUMBER; i ++)
		if ((AcqTaskPending & (1 << i)) != 0)
			break;
	if (i == ACQ_TASK_NUMBER)	// no task pending
		return;

	CurAcqTask = &AcqConfig[i];
	if (CurSignalType != CurAcqTask->SignalType || (BasebandTickCount - AcqBufferTimeTag) > 10000)	// AE buffer not filled desired signal or too old
		FillAeBuffer(CurAcqTask);
	else
		StartAcquisition();
}

void FillAeBuffer(PACQ_CONFIG pAcqConfig)
{
	int i, PhaseRange = 0;
	int CorrelationRange = pAcqConfig->CohNumber * pAcqConfig->NoncohNumber;	// number of 1ms samples used for acquisition

	CurSignalType = pAcqConfig->SignalType;
	AcqBufferTimeTag = BasebandTickCount;

	// calculate lagest CodeSpan that will use extra signal
	for (i = 0; i < pAcqConfig->AcqChNumber; i ++)
		if (PhaseRange < (pAcqConfig->SatConfig[i].CodeSpan + 2) / 3)
			PhaseRange = (pAcqConfig->SatConfig[i].CodeSpan + 2) / 3;
	CorrelationRange += PhaseRange;

	SetRegValue(ADDR_AE_CARRIER_FREQ, CARRIER_FREQ(0));
	SetRegValue(ADDR_AE_CODE_RATIO, (int)(2.046e6 / SAMPLE_FREQ * 16777216. + 0.5));
	SetRegValue(ADDR_AE_THRESHOLD, 8);
	SetRegValue(ADDR_AE_BUFFER_CONTROL, 0x300 + CorrelationRange);	// start fill AE buffer

	AddWaitRequest(WAIT_TASK_AE, CorrelationRange + 1);	// wait extra 1ms to make sure AE buffer fill complete
}

void AeInterruptProc()
{
	PACQ_CONFIG pAcqConfig = CurAcqTask;
	int TaskIndex = pAcqConfig - AcqConfig;

	AcqTaskPending &= ~(1 << TaskIndex);
	SetRegValue(ADDR_TE_FIFO_CONFIG, 0);			// FIFO config, disable dummy write
	// call ProcessAcqResult on request interrupt, so it will sync with TE interrupt
	AddToTask(TASK_REQUEST, ProcessAcqResult, &pAcqConfig, sizeof(PACQ_CONFIG));
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
	int DftFreq = (CurAcqTask->StrideInterval << 10) / 1000;
	unsigned int ConfigData[4];

	ConfigData[0] = 0x04000000 | (CurAcqTask->NoncohNumber << 16) | (CurAcqTask->CohNumber << 8) | CurAcqTask->StrideNumber;	// threshold: 3'b100
	ConfigData[3]= AE_STRIDE_INTERVAL(CurAcqTask->StrideInterval);
	for (i = 0; i < CurAcqTask->AcqChNumber; i ++)
	{
		ConfigData[1] = (CurAcqTask->SatConfig[i].FreqSvid << 24) | (AE_CENTER_FREQ(CurAcqTask->SatConfig[i].CenterFreq) & 0xfffff);
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
	int AddressGap, PhaseGap;
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
	LatchRound = (RegValue >> 20);
	LatchAddress = ((RegValue >> 6) & 0x3fff) + LatchRound * 10240;
	// get write address and round
	RegValue = GetRegValue(ADDR_TE_FIFO_WRITE_ADDR);
	ReadRound = (RegValue >> 20);
	WriteAddress = (RegValue >> 6) & 0x3fff;
	// get read address and compare with write address to determine whether it has roll-over
	RegValue = GetRegValue(ADDR_TE_FIFO_READ_ADDR);
	if ((int)RegValue > WriteAddress)	// write address roll-over
		ReadRound --;
	ReadAddress = RegValue + ReadRound * 10240;
	AddressGap = ReadAddress - LatchAddress;
	if (AddressGap < 0)
		AddressGap += 41943040;	// 2^12 * 10240
	// get remnant of address gap
	AddressGap %= (SAMPLES_1MS * 20);	// remnant of 20ms
	PhaseGap = (AddressGap * 1023 * 16) / SAMPLES_1MS;	// convert to code phase with 16x scale

	for (i = 0; i < pAcqConfig->AcqChNumber; i ++)
	{
		LoadMemory(AcqResult, (U32 *)(ADDR_BASE_AE_BUFFER + i * 32 + 16), 16);
		CodePhase = AcqResult[1] & 0x3fff;	// acquired code position, 2x chip scale
		CodePhase = PhaseGap - (CodePhase - 5) * 8;	// additional 5 correlator interval (interval at 1/2 chip) to set peak at Cor4
		if (CodePhase < 0)
			CodePhase += 20 * 1023 * 16;	// 20ms code phase round
		Doppler = ((int)(AcqResult[1] << 8)) >> 23;
		Doppler = pAcqConfig->SatConfig[i].CenterFreq + (Doppler * 2 - 7) * pAcqConfig->StrideInterval / 16;
		if ((NewChannel = GetAvailableChannel()) != NULL)
		{
			NewChannel->FreqID = GET_FREQ_ID(pAcqConfig->SatConfig[i].FreqSvid);
			NewChannel->Svid = GET_SVID(pAcqConfig->SatConfig[i].FreqSvid);
			InitChannel(NewChannel);
			ConfigChannel(NewChannel, Doppler, CodePhase);
		}
	}
	UpdateChannels();
	SetRegValue(ADDR_TE_CHANNEL_ENABLE, GetChannelEnable());

	DoAcqTask();

	return 0;
}
