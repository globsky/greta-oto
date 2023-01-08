//----------------------------------------------------------------------
// HWCtrl_Model.cpp:
//   Implementation for hardware access using C model
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <memory.h>
#include "GnssTop.h"
#include "HWCtrl.h"
extern "C" {
#include "TaskQueue.h"
#include "FirmwarePortal.h"
}

#define BLOCK_SIZE (SAMPLE_FREQ / 1000)		// one data block has 1ms length

static CGnssTop Baseband;
static DebugFunction DebugFunc = 0;

SYSTEM_TIME InitTime;
LLH InitPosition;

//*************** Attach ISR to baseband interrupt ****************
// Parameters:
//   ISR: baseband interrupt service routine
void AttachBasebandISR(InterruptFunction ISR)
{
	Baseband.InterruptService = ISR;
}

//*************** Attach ISR to baseband interrupt ****************
// Parameters:
//   Function: debug function to output tracking status
void AttachDebugFunc(DebugFunction Function)
{
	DebugFunc = Function;
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
	InitTime.Year = Baseband.UtcTime.Year;
	InitTime.Month = Baseband.UtcTime.Month;
	InitTime.Day = Baseband.UtcTime.Day;
	InitTime.Hour = Baseband.UtcTime.Hour;
	InitTime.Minute = Baseband.UtcTime.Minute;
	InitTime.Second = (int)Baseband.UtcTime.Second;
	InitTime.Millisecond = (int)((Baseband.UtcTime.Second - InitTime.Second) * 1000);
	InitPosition.lon = Baseband.StartPos.lon;
	InitPosition.lat = Baseband.StartPos.lat;
	InitPosition.hae = Baseband.StartPos.alt;
}

//*************** Load parameter (ephemeris/almanac, receiver position etc.) ****************
//* in PC platform, this is a file read
//* in real system, read from flash or host
// Parameters:
//   Buffer: address to store load parameters
int LoadParameters(int Offset, void *Buffer, int Size)
{
	FILE *fp;
	int ReturnValue;

	if ((fp = fopen("ParamFile.bin", "rb")) == NULL)
	{
		memset(Buffer, 0, Size);
		return 0;
	}
	fseek(fp, Offset, SEEK_SET);
	ReturnValue = fread(Buffer, 1, Size, fp);
	fclose(fp);

	return ReturnValue;
}

//*************** Save parameter (ephemeris/almanac, receiver position etc.) ****************
//* in PC platform, this is a file write
//* in real system, write to flash or host
// Parameters:
//   Buffer: address to store load parameters
void SaveParameters(int Offset, void *Buffer, int Size)
{
	FILE *fp;

	if ((fp = fopen("ParamFile.bin", "rb+")) == NULL)
		return;
	fseek(fp, Offset, SEEK_SET);
	fwrite(Buffer, 1, Size, fp);
	fclose(fp);
}

//*************** enable RF clock ****************
//* in PC platform, this will run baseband process until end of scenario
//* in real system, this will enable RF and its ADC clock
void EnableRF()
{
	static int ProcessCount = 0;

	while (Baseband.Process(BLOCK_SIZE) >= 0)
	{
//		printf("ProcessCount=%d\n", ProcessCount);
		if (ProcessCount == 497400)
			ProcessCount = ProcessCount;
		DoTaskQueue(&BasebandTask);
		DoTaskQueue(&PostMeasTask);
		DoTaskQueue(&InputOutputTask);
		if (DebugFunc)
			DebugFunc((void *)(&Baseband), ProcessCount);
		ProcessCount ++;
		if (ProcessCount == 40000)
			break;
	}
}
