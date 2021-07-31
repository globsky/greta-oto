//----------------------------------------------------------------------
// FirmwarePortal.c:
//   Firmware portal functions and support functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <memory.h>
#include "CommonDefines.h"
#include "RegAddress.h"
#include "BBDefines.h"
#include "HWCtrl.h"
#include "TaskQueue.h"
#include "AEManager.h"
#include "TEManager.h"
#include "ChannelManager.h"

TASK_QUEUE RequestTask;
TASK_ITEM RequestItems[32];
U32 RequestBuffer[1024];
TASK_QUEUE BasebandTask;
TASK_ITEM BasebandItems[32];
U32 BasebandBuffer[1024];
TASK_QUEUE PostMeasTask;
TASK_ITEM PostMeasItems[8];
U32 PostMeasBuffer[1024];

//*************** Baseband interrupt service routine ****************
//* this function is a task function
// Parameters:
//   none
// Return value:
//   none
void InterruptService()
{
	unsigned int IntFlag = GetRegValue(ADDR_INTERRUPT_FLAG);
	
	if (IntFlag & (1 << 8))		// coherent sum interrupt
		CohSumInterruptProc();
	if (IntFlag & (1 << 9))		// measurement interrupt
		MeasurementProc();
	if (IntFlag & (1 << 10))	// request interrupt
		DoTaskQueue(&RequestTask);
	if (IntFlag & (1 << 11))	// AE interrupt
	{
		SetRegValue(ADDR_TE_FIFO_CONFIG, 0);			// FIFO config, disable dummy write
		// call ProcessAcqResult on request interrupt, so it will sync with TE interrupt
		AddTaskToQueue(&RequestTask, ProcessAcqResult, &AcqConfig, sizeof(AcqConfig));
		if (!GetRegValue(ADDR_REQUEST_COUNT))	// request count is zero, set a new request
			SetRegValue(ADDR_REQUEST_COUNT, 1);
	}
	// clear interrupt
	SetRegValue(ADDR_INTERRUPT_FLAG, IntFlag);
	// if it is TE interrupt, resume TE
	if (IntFlag & 0x700)
		SetRegValue(ADDR_TRACKING_START, 1);
}

//*************** firmware initialization ****************
//* this function is a task function
// Parameters:
//   none
// Return value:
//   none
void FirmwareInitialize()
{
	int i, sv_list[32] = { 3, 4, 7, 8, 9, 14, 16, 27, 30, 0};	// for debug use only

	MeasurementInterval = 100;

	AttachBasebandISR(InterruptService);

	// initial baseband hardware
	SetRegValue(ADDR_BB_ENABLE, 0x00000100);		// enable TE
	SetRegValue(ADDR_FIFO_CLEAR, 0x00000100);		// clear FIFO
	SetRegValue(ADDR_MEAS_NUMBER, MeasurementInterval);	// measurement number
	SetRegValue(ADDR_MEAS_COUNT, 0);				// measurement count
	SetRegValue(ADDR_REQUEST_COUNT, 8);				// request count
	SetRegValue(ADDR_INTERRUPT_MASK, 0x00000f00);	// enable all interrupts
	SetRegValue(ADDR_TE_FIFO_CONFIG, 1);			// FIFO config, enable dummy write
	SetRegValue(ADDR_TE_FIFO_BLOCK_SIZE, 4113);		// FIFO block size
	SetRegValue(ADDR_TE_CHANNEL_ENABLE, 0);			// disable all channels
	SetRegValue(ADDR_TE_POLYNOMIAL, 0x00e98204);	// set L1CA polynomial
	SetRegValue(ADDR_TE_CODE_LENGTH, 0x00ffc000);	// set L1CA code length
	SetRegValue(ADDR_AE_CARRIER_FREQ, CARRIER_FREQ(0));
	SetRegValue(ADDR_AE_CODE_RATIO, (int)(2.046e6 / SAMPLE_FREQ * 16777216. + 0.5));
	SetRegValue(ADDR_AE_THRESHOLD, 37);
	SetRegValue(ADDR_AE_BUFFER_CONTROL, 0x300 + 5);	// start fill AE buffer

	ChannelOccupation = 0;
	memset(ChannelStateArray, 0, sizeof(ChannelStateArray));
	for (i = 0; i < 32; i ++)
	{
		ChannelStateArray[i].LogicChannel = i;
		ChannelStateArray[i].StateBufferHW = (PSTATE_BUFFER)(ADDR_BASE_TE_BUFFER + i * 128);	// each channel occupy 128 bytes start from ADDR_BASE_TE_BUFFER
	}
	// initial request task queue
	InitTaskQueue(&RequestTask, RequestItems, 32, RequestBuffer, sizeof(RequestBuffer));
	InitTaskQueue(&BasebandTask, BasebandItems, 32, BasebandBuffer, sizeof(BasebandBuffer));
	InitTaskQueue(&PostMeasTask, PostMeasItems, 8, PostMeasBuffer, sizeof(PostMeasBuffer));

	// start acquisition
	for (i = 0; i < 32; i ++)
	{
		if (sv_list[i] == 0)
			break;
		AcqConfig.SatConfig[i].FreqSvid = (U8)sv_list[i];
		AcqConfig.SatConfig[i].CodeSpan = 3;
		AcqConfig.SatConfig[i].CenterFreq = 0;
	}
	AcqConfig.AcqChNumber = i;
	AcqConfig.CohNumber = 4;
	AcqConfig.NoncohNumber = 1;
	AcqConfig.StrideNumber = 19;
	AcqConfig.StrideInterval = 500;
	AddTaskToQueue(&RequestTask, AcquisitionTask, &AcqConfig, sizeof(AcqConfig));
}
