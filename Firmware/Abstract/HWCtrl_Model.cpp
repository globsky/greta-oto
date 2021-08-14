//----------------------------------------------------------------------
// HWCtrl_Model.cpp:
//   Implementation for hardware access using C model
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "GnssTop.h"
#include "HWCtrl.h"
extern "C" {
#include "TaskQueue.h"
#include "FirmwarePortal.h"
}

#define SAMPLE_FREQ 4113000
#define SAMPLE_COUNT (SAMPLE_FREQ / 1000)

static CGnssTop Baseband;

//*************** Attach ISR to baseband interrupt ****************
// Parameters:
//   ISR: baseband interrupt service routine
void AttachBasebandISR(InterruptFunction ISR)
{
	Baseband.InterruptService = ISR;
}

//*************** Host read from baseband ****************
// Parameters:
//   Address: address offset of baseband (DWORD aligned, only 16LSB effect)
// Return value:
//   data read from baseband
U32 GetRegValue(int Address)
{
	return Baseband.GetRegValue(Address);
}

//*************** Host write to baseband ****************
// Parameters:
//   Address: address offset of baseband (DWORD aligned, only 16LSB effect)
//   Value: data written to baseband
void SetRegValue(int Address, U32 Value)
{
	Baseband.SetRegValue(Address, Value);
}

//*************** Copy baseband memory out to system memory ****************
// Parameters:
//   DestAddr: address of system memory
//   BasebandAddr: address mapped to baseband memory (TE state buffer, AE config buffer)
//   Size: copy size in bytes
void LoadMemory(U32 *DestAddr, U32 *BasebandAddr, int Size)
{
	int i;
	for (i = 0; i < Size / 4; i ++)
		DestAddr[i] = GetRegValue((int)(BasebandAddr + i));
}

//*************** Copy baseband memory out to system memory ****************
// Parameters:
//   BasebandAddr: address mapped to baseband memory (TE state buffer, AE config buffer)
//   SrcAddr: address of system memory
//   Size: copy size in bytes
void SaveMemory(U32 *BasebandAddr, U32 *SrcAddr, int Size)
{
	int i;
	for (i = 0; i < Size / 4; i ++)
		SetRegValue((int)(BasebandAddr + i), SrcAddr[i]);
}

//*************** Set input file of the scenario ****************
//* in C model, this is a RF file
//* in SignalSim, this is a scenario config file
//* in real system, this function has no effect
// Parameters:
//   FileName: file name
void SetInputFile(char *FileName)
{
	Baseband.SetInputFile(FileName);
}

//*************** Load parameter (ephemeris/almanac, receiver position etc.) ****************
//* in PC platform, this is a file read
//* in real system, read from flash or host
// Parameters:
//   Buffer: address to store load parameters
void LoadParameters(void *Buffer)
{
}

//*************** Save parameter (ephemeris/almanac, receiver position etc.) ****************
//* in PC platform, this is a file write
//* in real system, write to flash or host
// Parameters:
//   Buffer: address to store load parameters
void SaveParameters(void *Buffer)
{
}

//*************** enable RF clock ****************
//* in PC platform, this will run baseband process until end of scenario
//* in real system, this will enable RF and its ADC clock
void EnableRF()
{
	static int ProcessCount = 0;

	while (Baseband.Process(SAMPLE_COUNT) >= 0)
	{
//		printf("ProcessCount=%d\n", ProcessCount);
		ProcessCount ++;
		if (ProcessCount == 2001)
			break;
		DoTaskQueue(&BasebandTask);
		DoTaskQueue(&PostMeasTask);
	}
}
