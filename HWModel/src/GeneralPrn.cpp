//----------------------------------------------------------------------
// GeneralPrn.cpp:
//   General purpose PRN class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "CommonOps.h"
#include "GeneralPrn.h"

CGeneralPrn::CGeneralPrn(const unsigned int PolySettings[])
{
	PrnPolySettings = PolySettings;
	Reset();
}

CGeneralPrn::~CGeneralPrn()
{
}

void CGeneralPrn::Reset()
{
	G1CurState = G1InitState;
	G2CurState = G2InitState;
	G1CurCount = GlobalCount = 0;
}

// Status buffer contents:
// address 00: bit0~13: G1 initial state, bit14~27: G2 initial state
// address 04: bit0~13: G1 current state, bit14~27: G2 current state
// address 08: for parallel: bit0~13: G1 current count, bit14~31: total code count
// address 08: for serialize: bit0~31: total code count
// Global polynomial set registers
// address 00: bit0~13: G1 polynomial, bit14~27: G2 polynomial, bit31: serialize for set 0
// address 04: for parallel: bit0~13: G1 length, bit14~31: total code length for set 0
// address 04: for serialize: bit0~31: total code length for set 0
// address 08: bit0~13: G1 polynomial, bit14~27: G2 polynomial, bit31: serialize for set 1
// address 0c: for parallel: bit0~13: G1 length, bit14~31: total code length for set 1
// address 0c: for serialize: bit0~31: total code length for set 1
// ...
void CGeneralPrn::FillState(unsigned int *StateBuffer)
{
	G1InitState = EXTRACT_UINT(StateBuffer[0], 0, 14);
	G2InitState = EXTRACT_UINT(StateBuffer[0], 14, 14);
	G1CurState = EXTRACT_UINT(StateBuffer[1], 0, 14);
	G2CurState = EXTRACT_UINT(StateBuffer[1], 14, 14);
	G1GenPoly = EXTRACT_UINT(PrnPolySettings[0], 0, 14);
	G2GenPoly = EXTRACT_UINT(PrnPolySettings[0], 14, 14);
	ParallelSerial = EXTRACT_UINT(PrnPolySettings[0], 31, 1);
	G1CurCount = EXTRACT_UINT(StateBuffer[2], 0, 14);
	G1Length = EXTRACT_UINT(PrnPolySettings[1], 0, 14);
	if (ParallelSerial == 0)
	{
		GlobalCount = EXTRACT_UINT(StateBuffer[2], 14, 18);
		GlobalLength = EXTRACT_UINT(PrnPolySettings[1], 14, 18);
	}
	else
	{
		GlobalCount = StateBuffer[2];
		GlobalLength = PrnPolySettings[1];
	}
}

void CGeneralPrn::DumpState(unsigned int *StateBuffer)
{
	StateBuffer[1] = G1CurState | (G2CurState << 14);
	if (ParallelSerial == 0)
		StateBuffer[2] = (G1CurCount & 0x3fff) | ((GlobalCount & 0x3ffff) << 14);
	else
		StateBuffer[2] = GlobalCount;
}

int CGeneralPrn::GetCode()
{
	if (ParallelSerial)
		return (G2CurState & 0x2000) ? 1 : 0;
	else
		return ((G1CurState ^ G2CurState) & 0x2000) ? 1 : 0;
}

int CGeneralPrn::ShiftCode()
{
	unsigned int PolyResult1, PolyResult2;
	int NewRound = 0;

	// if overall count reach length, init both G1 and G2
	if (++GlobalCount == GlobalLength)
	{
		Reset();
		NewRound = 1;
	}
	else
	{
		// get G1 polynomial result
		PolyResult1 = G1CurState & G1GenPoly;
		// xor of all PolyResult
		PolyResult1 = (PolyResult1 & 0x5555) + ((PolyResult1 & 0xaaaa) >> 1);
		PolyResult1 = (PolyResult1 & 0x3333) + ((PolyResult1 & 0xcccc) >> 2);
		PolyResult1 = (PolyResult1 & 0x0f0f) + ((PolyResult1 & 0xf0f0) >> 4);
		PolyResult1 = (PolyResult1 & 0x000f) + ((PolyResult1 & 0x0f00) >> 8);
		// get G2 polynomial result
		PolyResult2 = G2CurState & G2GenPoly;
		// xor of all PolyResult
		PolyResult2 = (PolyResult2 & 0x5555) + ((PolyResult2 & 0xaaaa) >> 1);
		PolyResult2 = (PolyResult2 & 0x3333) + ((PolyResult2 & 0xcccc) >> 2);
		PolyResult2 = (PolyResult2 & 0x0f0f) + ((PolyResult2 & 0xf0f0) >> 4);
		PolyResult2 = (PolyResult2 & 0x000f) + ((PolyResult2 & 0x0f00) >> 8);

		// reset G1
		if (ParallelSerial == 0 && ++G1CurCount == G1Length)
		{
			G1CurState = G1InitState;
			G1CurCount = 0;
		}
		// shift G1
		else
		{
			// for serialize, G1 shift in bit is xor of two result and G2 shift in bit is MSB of G1
			if (ParallelSerial)
			{
				PolyResult1 ^= PolyResult2;
				PolyResult2 = (G1CurState & 0x2000) ? 1 : 0;
			}
			// shift G1CurState by 1bit and shift in PolyResult
			G1CurState <<= 1; G1CurState &= 0x3fff;
			if (PolyResult1 & 1)
				G1CurState |= 1;
		}

		// shift G2
		// shift G2CurState by 1bit and shift in PolyResult
		G2CurState <<= 1; G2CurState &= 0x3fff;
		if (PolyResult2 & 1)
			G2CurState |= 1;
	}

	return NewRound;
}

void CGeneralPrn::PhaseInit(unsigned int PrnConfig)
{
	G1GenPoly = EXTRACT_UINT(PrnPolySettings[0], 0, 14);
	G2GenPoly = EXTRACT_UINT(PrnPolySettings[0], 14, 14);

	G1CurState = G1InitState = EXTRACT_UINT(PrnConfig, 0, 14);
	G2CurState = G2InitState = EXTRACT_UINT(PrnConfig, 14, 14);
	ParallelSerial = EXTRACT_UINT(PrnConfig, 31, 1);
	GlobalCount = G1CurCount = G1Length = 0;
	if (ParallelSerial == 0)
		GlobalLength = EXTRACT_UINT(PrnPolySettings[1], 14, 18);
	else
		GlobalLength = PrnPolySettings[1];
}
