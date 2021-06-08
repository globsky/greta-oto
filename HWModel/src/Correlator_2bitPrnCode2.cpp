//----------------------------------------------------------------------
// Correlator.cpp:
//   Correlator class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <malloc.h>
#include <memory.h>
#include "CommonOps.h"
#include "Correlator.h"

#define DEBUG_PRINT(...) printf(__VA_ARGS__)

const complex_int CCorrelator::DownConvertTable[64] = {
complex_int( 12,  -1), complex_int( 12,  -2), complex_int( 12,  -3), complex_int( 11,  -4),
complex_int( 11,  -5), complex_int( 10,  -6), complex_int( 10,  -7), complex_int(  9,  -8),
complex_int(  8,  -9), complex_int(  7, -10), complex_int(  6, -10), complex_int(  5, -11),
complex_int(  4, -11), complex_int(  3, -12), complex_int(  2, -12), complex_int(  1, -12),
complex_int( -1, -12), complex_int( -2, -12), complex_int( -3, -12), complex_int( -4, -11),
complex_int( -5, -11), complex_int( -6, -10), complex_int( -7, -10), complex_int( -8,  -9),
complex_int( -9,  -8), complex_int(-10,  -7), complex_int(-10,  -6), complex_int(-11,  -5),
complex_int(-11,  -4), complex_int(-12,  -3), complex_int(-12,  -2), complex_int(-12,  -1),
complex_int(-12,   1), complex_int(-12,   2), complex_int(-12,   3), complex_int(-11,   4),
complex_int(-11,   5), complex_int(-10,   6), complex_int(-10,   7), complex_int( -9,   8),
complex_int( -8,   9), complex_int( -7,  10), complex_int( -6,  10), complex_int( -5,  11),
complex_int( -4,  11), complex_int( -3,  12), complex_int( -2,  12), complex_int( -1,  12),
complex_int(  1,  12), complex_int(  2,  12), complex_int(  3,  12), complex_int(  4,  11),
complex_int(  5,  11), complex_int(  6,  10), complex_int(  7,  10), complex_int(  8,   9),
complex_int(  9,   8), complex_int( 10,   7), complex_int( 10,   6), complex_int( 11,   5),
complex_int( 11,   4), complex_int( 12,   3), complex_int( 12,   2), complex_int( 12,   1),
};

CCorrelator::CCorrelator(unsigned int PolySettings[], unsigned int *MemCodeAddress)
{
	DumpDataOutput = NULL;
	PrnGen[0] = new CGeneralPrn(PolySettings);
	PrnGen[1] = new CGeneralPrn(PolySettings);
	PrnGen[2] = new CWeilPrn;
	PrnGen[3] = new CMemoryPrn(MemCodeAddress);
	PrnGen2[0] = new CGeneralPrn(PolySettings);
	PrnGen2[1] = new CGeneralPrn(PolySettings);
	PrnGen2[2] = new CWeilPrn;
	PrnGen2[3] = new CMemoryPrn(MemCodeAddress);
	Reset();
}

CCorrelator::~CCorrelator()
{
	delete PrnGen[0];
	delete PrnGen[1];
	delete PrnGen[2];
	delete PrnGen[3];
	delete PrnGen2[0];
	delete PrnGen2[1];
	delete PrnGen2[2];
	delete PrnGen2[3];
}

void CCorrelator::Reset()
{
	int i;

	for (i = 0; i < 4; i ++)
	{
		PrnGen[i]->Reset();
		PrnGen2[i]->Reset();
	}
}

// Status buffer contents:
void CCorrelator::FillState(unsigned int *StateBuffer)
{
	int i;
	unsigned int SecondPrnState[3];

	CarrierFreq = StateBuffer[0];
	CodeFreq = StateBuffer[1];
	PreShiftBits = EXTRACT_UINT(StateBuffer[2], 0, 2);
	EnableBOC = EXTRACT_UINT(StateBuffer[2], 2, 1);
	DataInQBranch = EXTRACT_UINT(StateBuffer[2], 3, 1);
	EnableSecondPrn = EXTRACT_UINT(StateBuffer[2], 4, 1);
	NarrowFactor = EXTRACT_UINT(StateBuffer[2], 8, 2);
	DumpLength = EXTRACT_UINT(StateBuffer[2], 16, 16);
	NHCode = EXTRACT_UINT(StateBuffer[3], 0, 25);
	NHLength = EXTRACT_UINT(StateBuffer[3], 27, 5);
	NHCode2 = EXTRACT_UINT(StateBuffer[4], 0, 20);
	MsDataNumber = EXTRACT_UINT(StateBuffer[4], 20, 5);
	CoherentNumber = EXTRACT_UINT(StateBuffer[4], 25, 5);
	PostShiftBits = EXTRACT_UINT(StateBuffer[4], 30, 2);

	CarrierPhase = StateBuffer[8];
	CarrierCount = StateBuffer[9];
	CodePhase = StateBuffer[10];
	PrnCode = EXTRACT_UINT(StateBuffer[11], 0, 8);
	JumpCount = EXTRACT_INT(StateBuffer[11], 8, 8);
	DumpCount = EXTRACT_UINT(StateBuffer[11], 16, 16);
	// CoherentDone and MsDataDone will be reset on every correlation process
	CurrentCor = EXTRACT_UINT(StateBuffer[12], 4, 3);
	Dumping = EXTRACT_UINT(StateBuffer[12], 7, 1);
	CodeSubPhase = EXTRACT_UINT(StateBuffer[12], 8, 1);
	PrnCode2 = EXTRACT_UINT(StateBuffer[12], 14, 2);
	MsDataCount = EXTRACT_UINT(StateBuffer[12], 16, 5);
	CoherentCount = EXTRACT_UINT(StateBuffer[12], 21, 5);
	NHCount = EXTRACT_UINT(StateBuffer[12], 27, 5);
	MsDataSum = EXTRACT_INT(StateBuffer[13], 0, 16);

	for (i = 0; i < 8; i ++)
	{
		AccDataI[i] = (S16)(StateBuffer[i+16] >> 16);
		AccDataQ[i] = (S16)StateBuffer[i+16];
	}
	PrnIndex = StateBuffer[5] >> 30;
	PrnGen[PrnIndex]->FillState(StateBuffer + 5);
	// bit0 of PrnCode is not a register but a wire from PrnGen output
	PrnCode &= ~1;
	PrnCode |= PrnGen[PrnIndex]->GetCode() ^ (EnableBOC & CodeSubPhase) ^ ((NHLength && (NHCode & (1 << NHCount))) ? 1 : 0);
	PrnCode2 <<= 1;
	if (EnableSecondPrn)
	{
		SecondPrnState[0] = StateBuffer[14];
		SecondPrnState[1] = StateBuffer[15];
		SecondPrnState[2] = StateBuffer[7];
		PrnIndex2 = SecondPrnState[0] >> 30;
		PrnGen2[PrnIndex2]->FillState(SecondPrnState);
		PrnCode2 |= PrnGen2[PrnIndex2]->GetCode();
	}
}

void CCorrelator::DumpState(unsigned int *StateBuffer)
{
	int i;
	unsigned int SecondPrnState[3];

	StateBuffer[8] = CarrierPhase;	
	StateBuffer[9] = CarrierCount;
	StateBuffer[10] = CodePhase;
	StateBuffer[11] = (PrnCode & 0xff) | ((JumpCount & 0xff) << 8) | (DumpCount << 16);
	StateBuffer[12] = CoherentDone | (MsDataDone << 1) | (CurrentCor << 4) | (Dumping << 7) | (CodeSubPhase << 8) | ((PrnCode2 & 0x6) << 13) | (MsDataCount << 16) | (CoherentCount << 21) | (NHCount << 27);
	StateBuffer[13] = MsDataSum & 0xffff;

	for (i = 0; i < 8; i ++)
		StateBuffer[i+16] = ((unsigned int)AccDataI[i] << 16) | ((unsigned int)AccDataQ[i] & 0xffff);

	PrnGen[PrnIndex]->DumpState(StateBuffer + 5);
	if (EnableSecondPrn)
	{
		PrnGen2[PrnIndex2]->DumpState(SecondPrnState);
		StateBuffer[15] = SecondPrnState[1];
	}
}

// Do down conversion on input data with length SampleNumber
void CCorrelator::DownConvert(int SampleNumber, complex_int InputData[], complex_int OutputData[])
{
	int i, PhaseIndex;
	complex_int MulResult;
	unsigned int CarrierPhaseNew;

	for (i = 0; i < SampleNumber; i ++)
	{
		CarrierPhaseNew = CarrierPhase + CarrierFreq;
		if (CarrierFreq & 0x80000000)
		{
			if (CarrierPhaseNew > CarrierPhase)	// negative freq increase carrier count on underflow
				CarrierCount --;
		}
		else
		{
			if (CarrierPhaseNew < CarrierPhase)	// positive freq increase carrier count on overflow
				CarrierCount ++;
		}
		PhaseIndex = CarrierPhase >> 26;
		MulResult = InputData[i] * DownConvertTable[PhaseIndex];
		CarrierPhase = CarrierPhaseNew;

		// do convergent right shift by ShiftBits
		switch (PreShiftBits)
		{
		case 0:
			MulResult.real = CONVERGENT_ROUND_SHIFT(MulResult.real, 3);
			MulResult.imag = CONVERGENT_ROUND_SHIFT(MulResult.imag, 3);
			break;
		case 1:
			MulResult.real = CONVERGENT_ROUND_SHIFT(MulResult.real, 4);
			MulResult.imag = CONVERGENT_ROUND_SHIFT(MulResult.imag, 4);
			break;
		case 2:
			MulResult.real = CONVERGENT_ROUND_SHIFT(MulResult.real, 5);
			MulResult.imag = CONVERGENT_ROUND_SHIFT(MulResult.imag, 5);
			break;
		default:
			MulResult.real >>= 3;
			MulResult.imag >>= 3;
			break;
		}

		// overflow protection to scale down to 6bit
		if (MulResult.real > 31)
			MulResult.real = 31;
		else if (MulResult.real < -31)
			MulResult.real = -31;
		if (MulResult.imag > 31)
			MulResult.imag = 31;
		else if (MulResult.imag < -31)
			MulResult.imag = -31;

		OutputData[i] = MulResult;
	}
}

// Do correlation on SampleDataI and SampleDataQ with length ClusterNumber
// Output correlation result in DumpDataI/DumpDataQ
// Bit2~4 of CorIndex indicate which correlator to accumulate
// bit0 of CorIndex is 1 means first coherent value, store instead of accumulate
// bit1 of CorIndex is 1 means overwrite protection, store to TE_OVERWRITE_PROTECT_VALUE instead store to coherent buffer
// DumpDataLength is the actual length of DumpDataI/DumpDataQ/CohAddr
// return 1 means any correlator reaches last coherent sum
int CCorrelator::Correlation(int SampleNumber, complex_int SampleData[], S16 DumpDataI[], S16 DumpDataQ[], int CorIndex[], int &DumpDataLength)
{
	int i = 0;
	unsigned int CodePhaseNew;
	int CodeLength = 0, NewCode = 0;
	
	// clear overwrite protect registers at the beginning of every round
	FirstCorIndexValid = 0;
	FirstCorIndex = 0;
	OverwriteProtect = 0;

	// clear coherent data FIFO at the beginning of every round
	DumpDataLength = 0;

	// clear CoherentDone and MsDataDone at the beginning of every round
	CoherentDone = MsDataDone = 0;

	// first check whether there is positive jump, force overflow
	while (JumpCount > 0)
	{
		CoherentDone |= ProcessOverflow(DumpDataI, DumpDataQ, CorIndex, DumpDataLength);
		JumpCount --;
	}
	// second check whether there is negative jump, skip sample
	for (i = 0; i < SampleNumber; i ++)
	{
		if (JumpCount >= 0)
			AccumulateSample((S16)SampleData[i].real, (S16)SampleData[i].imag, 8);
		CodePhaseNew = CodePhase + CodeFreq;
		if (CodePhaseNew < CodePhase)	// code overflow
		{
			if (JumpCount < 0)
				JumpCount ++;
			else
				CoherentDone |= ProcessOverflow(DumpDataI, DumpDataQ, CorIndex, DumpDataLength);
			DEBUG_PRINT(" 1");
		}
		else
			DEBUG_PRINT(" 0");
		DEBUG_PRINT(" %1d\n", Dumping);
		CodePhase = CodePhaseNew;
	}

	if (DumpDataOutput)
		DumpDataOutput(DumpDataI, DumpDataQ, CorIndex, DumpDataLength);

	return CoherentDone;
}

// Add/Sub sample to AccDataI and AccDataQ for each correlator
void CCorrelator::AccumulateSample(S16 SampleI, S16 SampleQ, int CorCount)
{
	int j, BitMask;
	unsigned int PrnValue, Code2;
	int Advance4, Lag4;
	int Advance8, Lag8;
	int AdvanceBit = (PrnCode & (1 << 3)) ? 1 : 0;
	int PromptBit = (PrnCode & (1 << 4)) ? 1 : 0;
	int LagBit = (PrnCode & (1 << 5)) ? 1 : 0;

	DEBUG_PRINT("%3d %3d", SampleI, SampleQ);
	if (NarrowFactor)
	{
		Advance4 = (CodePhase & 0x80000000) ? 1 : 0;
		Lag4 = (CodePhase & 0x80000000) ? 0 : 1;
		Advance8 = ((~CodePhase) & 0xc0000000) ? 0 : 1;
		Lag8 = (CodePhase & 0xc0000000) ? 0 : 1;
		if (NarrowFactor == 1)
		{
			PrnValue = PrnCode & 0x93;	// 8'b10010011, clear bit 2,3,5,6
			PrnValue |= (AdvanceBit << 2);	// set bit2
			PrnValue |= ((Advance4 ? AdvanceBit : PromptBit) << 3);	// set bit3
			PrnValue |= ((Lag4 ? LagBit : PromptBit) << 5);	// set bit5
			PrnValue |= (LagBit << 6);	// set bit6
		}
		else if (NarrowFactor == 2)
		{
			PrnValue = PrnCode & 0x93;	// 8'b10010011, clear bit 2,3,5,6
			PrnValue |= ((Advance4 ? AdvanceBit : PromptBit) << 2);	// set bit2
			PrnValue |= ((Advance8 ? AdvanceBit : PromptBit) << 3);	// set bit3
			PrnValue |= ((Lag8 ? LagBit : PromptBit) << 5);	// set bit5
			PrnValue |= ((Lag4 ? LagBit : PromptBit) << 6);	// set bit5
		}
		else
			PrnValue = PrnCode;
	}
	else
		PrnValue = PrnCode;
	if (EnableSecondPrn)	// use second PRN code at cor0
	{
		Code2 = ((PrnCode2 >> 2) & 0x1) ^ (EnableBOC & CodeSubPhase) ^ ((NHLength && (NHCode2 & (1 << NHCount))) ? 1 : 0);
		PrnValue = (PrnValue & ~0x1) | Code2;
	}
	for (j = 0, BitMask = 1; j < CorCount; j ++, BitMask <<= 1)
	{
		DEBUG_PRINT(" %1d", (PrnValue & BitMask) ? 1 : 0);
		if (PrnValue & BitMask)
		{
			AccDataI[j] -= SampleI;
			AccDataQ[j] -= SampleQ;
		}
		else
		{
			AccDataI[j] += SampleI;
			AccDataQ[j] += SampleQ;
		}
	}
//	printf("%1d %1x %04x %04x\n", PrnGen2[PrnIndex]->GetCode(), (PrnCode2 & 0x4) >> 2, AccDataI[0] & 0xffff, AccDataQ[0] & 0xffff);
}

// Processing when overflow is high
// If there is data dump, put data into DumpDataI/DumpDataQ/CohAddr and increase CurrentLength
// Change related register value accordingly
// return 1 means there is correlator reaches last coherent sum
int CCorrelator::ProcessOverflow(S16 DumpDataI[], S16 DumpDataQ[], int CorIndex[], int &CurrentLength)
{
	int DataReady = 0;

	// toggle CodeSubPhase
	CodeSubPhase = 1 - CodeSubPhase;
	if (CodeSubPhase == 0)
	{
		if (PrnGen[PrnIndex]->ShiftCode())
		{
			if (NHLength)
			{
				NHCount = (NHCount + 1) & 0x1f;
				if (NHCount == NHLength)
					NHCount = 0;
			}
		}
		if (EnableSecondPrn)
			PrnGen2[PrnIndex2]->ShiftCode();
		// increase DumpCount
		if (++DumpCount == DumpLength)
		{
			DumpCount = 0;
			Dumping = 1;
		}
		if (EnableSecondPrn)
		{
			PrnCode2 <<= 1;
			PrnCode2 |= PrnGen2[PrnIndex2]->GetCode();
		}
	}

	// shift PrnCode
	PrnCode <<= 1;
	PrnCode |= PrnGen[PrnIndex]->GetCode() ^ (EnableBOC & CodeSubPhase) ^ ((NHLength && (NHCode & (1 << NHCount))) ? 1 : 0);

	// Check whether there is data to dump when overflow is high
	if (Dumping)
	{
		DataReady = DumpData(DumpDataI, DumpDataQ, CorIndex, CurrentLength);
	}

	return DataReady;
}

// Put accmulated data into coherent data FIFO
// increase counter accordingly
// return 1 means there is correlator reaches last coherent sum
int CCorrelator::DumpData(S16 DumpDataI[], S16 DumpDataQ[], int CorIndex[], int &CurrentLength)
{
	unsigned int NextCoherentCount = (CoherentCount + 1) & 0x3f;
	int DataReady = 0;

	// fill in coherent data FIFO with accumulated value
	DumpDataI[CurrentLength] = (AccDataI[CurrentCor] >> PostShiftBits);
	DumpDataQ[CurrentLength] = (AccDataQ[CurrentCor] >> PostShiftBits);
	AccDataI[CurrentCor] = 0;
	AccDataQ[CurrentCor] = 0;
	
	if (MsDataNumber != 0 && CurrentCor == (EnableSecondPrn ? 0 : PEAK_COR_INDEX))
	{
		if (MsDataCount == 0)
			MsDataSum = 0;
		MsDataSum += DataInQBranch ? DumpDataQ[CurrentLength] : DumpDataI[CurrentLength];
		if (++MsDataCount == MsDataNumber)
		{
			MsDataCount = 0;
			MsDataDone = 1;
		}
	}

	// assign overwrite protect registers
	if (OverwriteProtect == 0)
	{
		if (FirstCorIndexValid == 0)
		{
			FirstCorIndexValid = 1;
			FirstCorIndex = CurrentCor;
		}
		else if (FirstCorIndex == CurrentCor && CoherentCount == 0)
		{
			OverwriteProtect = 1;
		}
	}

	// assign coherent address
	CorIndex[CurrentLength] = CurrentCor << 2;
	if (OverwriteProtect == 1)
		CorIndex[CurrentLength] |= 2;
	if (CoherentCount == 0)
		CorIndex[CurrentLength] |= 1;
	if (NextCoherentCount == CoherentNumber)
		DataReady = 1;
	CurrentLength ++;
	CurrentCor = (CurrentCor + 1) & 0x7;
	// if this is last correlator
	if (CurrentCor == 0)
	{
		Dumping = 0;
		// increase Coherent count
		CoherentCount = NextCoherentCount;
		if (CoherentCount == CoherentNumber)
			CoherentCount = 0;
	}

	return DataReady;
}
