//----------------------------------------------------------------------
// TeFifoMem.cpp:
//   TE FIFO (with embedded FIFO memory) class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <malloc.h>
#include "TeFifoMem.h"
#include "RegAddress.h"

CTeFifoMem::CTeFifoMem(int Index, int Size)
{
	FifoIndex = Index;
	FifoSize = Size;
	pBuffer = (complex_int *)malloc(FifoSize * sizeof(complex_int));
	Reset();
	TriggerCallback = NULL;
}

CTeFifoMem::~CTeFifoMem()
{
	free(pBuffer);
}

void CTeFifoMem::Reset()
{
	FifoEnable = 0;
	FifoWaitTrigger = 0;
	DummyWrite = 0;
	OverflowFlag = 0;
	TriggerSource = 0;
	FifoGuard = 0;
	ReadAddress = 0;
	WriteAddress = 0;
	WriteAddressRound = 0;
	CurReadAddress = 0;
	BlockSize = 0;
	BlockSizeAdjust = 0;
	WriteAddressLatchCPU = 0;
	WriteAddressLatchEM = 0;
	WriteAddressLatchPPS = 0;
	WriteAddressLatchAE = 0;
	WriteAddressLatchCPURound = 0;
	WriteAddressLatchEMRound = 0;
	WriteAddressLatchPPSRound = 0;
	WriteAddressLatchAERound = 0;
	RealBlockSize = 0;
	GuardThreshold = 0;

	DataCount = 0;
}

void CTeFifoMem::Clear()
{
	DummyWrite = 0;
	OverflowFlag = 0;
	ReadAddress = 0;
	WriteAddress = 0;
	WriteAddressRound = 0;
	CurReadAddress = 0;
	BlockSizeAdjust = 0;
	RealBlockSize = BlockSize;

	DataCount = 0;
}

void CTeFifoMem::SetRegValue(int Address, U32 Value)
{
	Address &= 0xff;

	switch (Address)
	{
	case ADDR_OFFSET_TE_FIFO_CONFIG:
		DummyWrite = EXTRACT_UINT(Value, 0, 1);
		TriggerSource = EXTRACT_UINT(Value, 8, 8);
		if (Value & 2)
		{
			FifoWaitTrigger = 1;
			FifoEnable = 0;
		}
		if (Value & 4)
			Clear();
		break;
	case ADDR_OFFSET_TE_FIFO_STATUS:
		// set 1 to clear overflow flag
		OverflowFlag &= ~(Value & 1);
		break;
	case ADDR_OFFSET_TE_FIFO_GUARD:
		FifoGuard = Value & (~0xffff00ff);	// FIFO guard always be multiple of 256, indicate number of samples can be input before FIFO overflow
		GuardThreshold = (int)FifoSize - (int)FifoGuard; // no protection needed, software gurantee FifoSize > FifoGuard
		break;
	case ADDR_OFFSET_TE_FIFO_READ_ADDR:
		// read address is not writable
		break;
	case ADDR_OFFSET_TE_FIFO_WRITE_ADDR:
		// write address is not writable
		break;
	case ADDR_OFFSET_TE_FIFO_BLOCK_SIZE:
		BlockSize = EXTRACT_UINT(Value, 0, 16);
		RealBlockSize = (int)BlockSize + BlockSizeAdjust;
		break;
	case ADDR_OFFSET_TE_FIFO_BLOCK_ADJ:
		BlockSizeAdjust = EXTRACT_INT(Value, 0, 8);
		RealBlockSize = (int)BlockSize + BlockSizeAdjust;
		break;
	default:
		break;
	}
}

U32 CTeFifoMem::GetRegValue(int Address)
{
	Address &= 0xff;

	switch (Address)
	{
	case ADDR_OFFSET_TE_FIFO_CONFIG:
		return (DummyWrite | (FifoWaitTrigger << 1) | (TriggerSource << 8));
	case ADDR_OFFSET_TE_FIFO_STATUS:
		return (OverflowFlag | (FifoEnable << 2));
	case ADDR_OFFSET_TE_FIFO_GUARD:
		return FifoGuard;
	case ADDR_OFFSET_TE_FIFO_READ_ADDR:
		return ReadAddress;
	case ADDR_OFFSET_TE_FIFO_WRITE_ADDR:
		return (WriteAddress << CLK_COUNT_WIDTH) | (WriteAddressRound << 20);
	case ADDR_OFFSET_TE_FIFO_BLOCK_SIZE:
		return BlockSize;
	case ADDR_OFFSET_TE_FIFO_BLOCK_ADJ:
		return BlockSizeAdjust;
	case ADDR_OFFSET_TE_FIFO_LWADDR_CPU:
		return (WriteAddressLatchCPU << CLK_COUNT_WIDTH) | (WriteAddressLatchCPURound << 20);
	case ADDR_OFFSET_TE_FIFO_LWADDR_EM:
		return (WriteAddressLatchEM << CLK_COUNT_WIDTH) | (WriteAddressLatchEMRound << 20);
	case ADDR_OFFSET_TE_FIFO_LWADDR_PPS:
		return (WriteAddressLatchPPS << CLK_COUNT_WIDTH) | (WriteAddressLatchPPSRound << 20);
	case ADDR_OFFSET_TE_FIFO_LWADDR_AE:
		return (WriteAddressLatchAE << CLK_COUNT_WIDTH) | (WriteAddressLatchAERound << 20);
	default:
		return 0;
	}
}

void CTeFifoMem::RewindPointer()
{
	CurReadAddress = ReadAddress;
}

// force CurReadAddress jump forward one block
void CTeFifoMem::SkipBlock()
{
	// skip read address one block forward
	ReadAddress += RealBlockSize;
	if (ReadAddress >= FifoSize)
		ReadAddress -= FifoSize;

	// force ReadAddress assigned by CurReadAddress
	CurReadAddress = ReadAddress;
	// data count decrease by one block
	DataCount -= BlockSize;
	// automatically force BlockSizeAdjust to be 0 and recalculate related wire
	BlockSizeAdjust = 0;
	RealBlockSize = (int)BlockSize + BlockSizeAdjust;
}

int CTeFifoMem::WriteData(complex_int Data)
{
	if (!FifoEnable)
		return 0;

	if (!DummyWrite)
		pBuffer[WriteAddress] = Data;

	WriteAddress ++;
	DataCount ++;
	if (WriteAddress == FifoSize)
	{
		WriteAddress = 0;
		WriteAddressRound ++;
		WriteAddressRound &= 0xfff;
	}
	
	// if overflow, set overflow flag
	if (DataCount > (int)FifoSize)
		OverflowFlag |= 1;
	// if exceed guard value, set guard warning flag, else clear
	if (DataCount >= GuardThreshold)
		OverflowFlag |= 2;
	else
		OverflowFlag &= ~2;

	// if data count reach ready threshold, trigger other channels
	if (DataCount == BlockSize)
	{
		if (TriggerCallback)
			TriggerCallback(FifoIndex);
	}

	return (DataCount >= (int)BlockSize) ? 1 : 0;	// return with whether there is at least one block of data
}

void CTeFifoMem::ReadData(int &ReadNumber, complex_int Data[])
{
	int i;

	if (!FifoEnable)
	{
		ReadNumber = 0;
		return;
	}

	ReadNumber = RealBlockSize;
	for (i = 0; i < ReadNumber; i ++)
	{
		Data[i] = pBuffer[CurReadAddress++];
	
		if (CurReadAddress == FifoSize)
			CurReadAddress = 0;
	}
}

void CTeFifoMem::LatchWriteAddress(int Source)
{
	switch (Source)
	{
	case 0:	// CPU
		WriteAddressLatchCPU = WriteAddress;
		WriteAddressLatchCPURound = WriteAddressRound;
		break;
	case 1:	// EM
		WriteAddressLatchEM = WriteAddress;
		WriteAddressLatchEMRound = WriteAddressRound;
		break;
	case 2:	// PPS
		WriteAddressLatchPPS = WriteAddress;
		WriteAddressLatchPPSRound = WriteAddressRound;
		break;
	case 3:	// AE
		WriteAddressLatchAE = WriteAddress;
		WriteAddressLatchAERound = WriteAddressRound;
		break;
	default:
		break;
	}
}

// drive external enable input
void CTeFifoMem::SetFifoEnable(int Enable)
{
	FifoEnableFromTe = Enable ? 1 : 0;
	FifoEnable = FifoEnableFromTe & (!FifoWaitTrigger);
}

void CTeFifoMem::SetTrigger(int SrcIndex)
{
	// if waiting for trigger, clear waiting flag and enable FIFO (if TE enabled)
	if (FifoWaitTrigger && (TriggerSource & (1 << SrcIndex)))
	{
		FifoWaitTrigger = 0;
		FifoEnable = FifoEnableFromTe;
	}
}
