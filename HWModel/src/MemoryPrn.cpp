//----------------------------------------------------------------------
// MemoryPrn.cpp:
//   Memory code PRN generator class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "CommonOps.h"
#include "MemoryPrn.h"

CMemoryPrn::CMemoryPrn(unsigned int *Address)
{
	CodeMemory = Address;
	Reset();
}

CMemoryPrn::~CMemoryPrn()
{
}

void CMemoryPrn::Reset()
{
	CurrentCount = 0;
}

// Status buffer contents:
// address 00: bit0~3: length, bit4~15: start address
// address 04: bit0~31: current code word
// address 08: bit0~13: current code count (5LSB as current bit in current code word
void CMemoryPrn::FillState(unsigned int *StateBuffer)
{
	StartIndex = EXTRACT_UINT(StateBuffer[0], 4, 12);
	Length = EXTRACT_UINT(StateBuffer[0], 0, 4);
	CurrentCode = StateBuffer[1];
	CurrentCount = EXTRACT_UINT(StateBuffer[2], 0, 14);
}

void CMemoryPrn::DumpState(unsigned int *StateBuffer)
{
	StateBuffer[1] = CurrentCode;
	StateBuffer[2] = CurrentCount & 0x3fff;
}

int CMemoryPrn::GetCode()
{
	return (CurrentCode & (0x80000000 >> (CurrentCount & 0x1f))) ? 1 : 0;
}

int CMemoryPrn::ShiftCode()
{
	int NewRound = 0;

	CurrentCount ++;
	if ((CurrentCount & 0x3ff) == 0x3ff)	// skip 1bit for every 1023bit
	{
		CurrentCount ++;
		if ((CurrentCount >> 10) == Length)	// round back
		{
			CurrentCount = 0;
			NewRound = 1;
		}
	}
	if ((CurrentCount & 0x1f) == 0)	// next word
		CurrentCode = CodeMemory[StartIndex * 32 + (CurrentCount >> 5)];

	return NewRound;
}

void CMemoryPrn::PhaseInit(unsigned int PrnConfig)
{
	StartIndex = EXTRACT_UINT(PrnConfig, 4, 12);
	Length = EXTRACT_UINT(PrnConfig, 0, 4);
	CurrentCount = 0;
	CurrentCode = CodeMemory[StartIndex * 32];
}
