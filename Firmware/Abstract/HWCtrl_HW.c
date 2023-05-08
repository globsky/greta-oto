//----------------------------------------------------------------------
// HWCtrl_HW.cpp:
//   Implementation for hardware access on hardware platform
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <memory.h>
#include "HWCtrl.h"
#include "PlatformCtrl.h"

#define GNSS_BASE_ADDR 0xc8000000	// base address to map the GNSS on bus
#define GNSS_INT_NUMBER 4	// GNSS interupt number

//*************** Attach ISR to baseband interrupt ****************
// Parameters:
//   ISR: baseband interrupt service routine
void AttachBasebandISR(InterruptFunction ISR)
{
//	xPortInstallInterruptHandler(GNSS_INT_NUMBER, ISR, NULL);	// call corresponding OS function
}

void AttachDebugFunc(DebugFunction Function) {};
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
	return *(U32*)(GNSS_BASE_ADDR + Address);
}

//*************** Host write to baseband ****************
// Parameters:
//   Address: address offset of baseband (DWORD aligned, only 16LSB effect)
//   Value: data written to baseband
void SetRegValue(int Address, U32 Value)
{
	*(U32*)(GNSS_BASE_ADDR + Address) = Value;
}

//*************** Copy baseband memory out to system memory ****************
// Parameters:
//   DestAddr: address of system memory
//   BasebandAddr: address mapped to baseband memory (TE state buffer, AE config buffer)
//   Size: copy size in bytes
void LoadMemory(U32 *DestAddr, U32 *BasebandAddr, int Size)
{
	int i;
	for (i = 0; i < Size; i += 4)
		DestAddr[i] = *(U32*)(GNSS_BASE_ADDR + BasebandAddr + i);
}

//*************** Copy baseband memory out to system memory ****************
// Parameters:
//   BasebandAddr: address mapped to baseband memory (TE state buffer, AE config buffer)
//   SrcAddr: address of system memory
//   Size: copy size in bytes
void SaveMemory(U32 *BasebandAddr, U32 *SrcAddr, int Size)
{
	int i;
	for (i = 0; i < Size; i += 4)
		*(U32*)(GNSS_BASE_ADDR + BasebandAddr + i) = SrcAddr[i];
}

//*************** Set input file of the scenario ****************
//* in C model, this is a RF file
//* in SignalSim, this is a scenario config file
//* in real system, this function has no effect
// Parameters:
//   FileName: file name
void SetInputFile(char *FileName) {}

//*************** enable RF clock ****************
//* in PC platform, this will run baseband process until end of scenario
//* in real system, this will enable RF and its ADC clock
void EnableRF()
{
	// call corresponding device driver functions to enable RF and ADC clock
}
