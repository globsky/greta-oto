//----------------------------------------------------------------------
// TeFifoSim.cpp:
//   TE FIFO simulator class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <malloc.h>
#include "TeFifoSim.h"
#include "RegAddress.h"

CTeFifoSim::CTeFifoSim()
{
	Reset();
}

CTeFifoSim::~CTeFifoSim()
{
}

void CTeFifoSim::Reset()
{
	FifoGuard = 0;
	ReadAddress = 0;
	WriteAddress = 0;
	WriteAddressRound = 0;
	BlockSize = 0;
	WriteAddressLatchCPU = 0;
	WriteAddressLatchEM = 0;
	WriteAddressLatchPPS = 0;
	WriteAddressLatchAE = 0;
	WriteAddressLatchCPURound = 0;
	WriteAddressLatchEMRound = 0;
	WriteAddressLatchPPSRound = 0;
	WriteAddressLatchAERound = 0;
}

void CTeFifoSim::Clear()
{
	ReadAddress = 0;
	WriteAddress = 0;
	WriteAddressRound = 0;
}

void CTeFifoSim::SetRegValue(int Address, U32 Value)
{
	Address &= 0xff;

	switch (Address)
	{
	case ADDR_OFFSET_TE_FIFO_CONFIG:
		break;
	case ADDR_OFFSET_TE_FIFO_STATUS:
		break;
	case ADDR_OFFSET_TE_FIFO_GUARD:
		FifoGuard = Value & (~0xffff00ff);	// FIFO guard always be multiple of 256, indicate number of samples can be input before FIFO overflow
		break;
	case ADDR_OFFSET_TE_FIFO_READ_ADDR:
		// read address is not writable
		break;
	case ADDR_OFFSET_TE_FIFO_WRITE_ADDR:
		// write address is not writable
		break;
	case ADDR_OFFSET_TE_FIFO_BLOCK_SIZE:
		BlockSize = EXTRACT_UINT(Value, 0, 16);
		break;
	case ADDR_OFFSET_TE_FIFO_BLOCK_ADJ:
		break;
	default:
		break;
	}
}

U32 CTeFifoSim::GetRegValue(int Address)
{
	Address &= 0xff;

	switch (Address)
	{
	case ADDR_OFFSET_TE_FIFO_CONFIG:
		return 0;
	case ADDR_OFFSET_TE_FIFO_STATUS:
		return 0;
	case ADDR_OFFSET_TE_FIFO_GUARD:
		return FifoGuard;
	case ADDR_OFFSET_TE_FIFO_READ_ADDR:
		return ReadAddress;
	case ADDR_OFFSET_TE_FIFO_WRITE_ADDR:
		return (WriteAddress << (CLK_COUNT_WIDTH - 4)) | (WriteAddressRound << 16);
	case ADDR_OFFSET_TE_FIFO_BLOCK_SIZE:
		return BlockSize;
	case ADDR_OFFSET_TE_FIFO_BLOCK_ADJ:
		return 0;
	case ADDR_OFFSET_TE_FIFO_LWADDR_CPU:
		return (WriteAddressLatchCPU << CLK_COUNT_WIDTH) | (WriteAddressLatchCPURound << 20);
	case ADDR_OFFSET_TE_FIFO_LWADDR_EM:
		return (WriteAddressLatchEM << CLK_COUNT_WIDTH) | (WriteAddressLatchEMRound << 20);
	case ADDR_OFFSET_TE_FIFO_LWADDR_PPS:
		return (WriteAddressLatchPPS << CLK_COUNT_WIDTH) | (WriteAddressLatchPPSRound << 20);
	case ADDR_OFFSET_TE_FIFO_LWADDR_AE:
		return (WriteAddressLatchAE << (CLK_COUNT_WIDTH - 4)) | (WriteAddressLatchAERound << 16);
	default:
		return 0;
	}
}

void CTeFifoSim::StepOneBlock(int BlockSize)
{
	ReadAddress += BlockSize;
	if (ReadAddress >= FIFO_SIZE)
		ReadAddress -= FIFO_SIZE;
	WriteAddress += BlockSize;
	if (WriteAddress >= FIFO_SIZE)
	{
		WriteAddress -= FIFO_SIZE;
		WriteAddressRound ++;
		WriteAddressRound &= 0xffff;
	}
}

void CTeFifoSim::LatchWriteAddress(int Source)
{
	switch (Source)
	{
	case 0:	// CPU
		WriteAddressLatchCPU = WriteAddress;
		WriteAddressLatchCPURound = WriteAddressRound & 0xfff;
		break;
	case 1:	// EM
		WriteAddressLatchEM = WriteAddress;
		WriteAddressLatchEMRound = WriteAddressRound & 0xfff;
		break;
	case 2:	// PPS
		WriteAddressLatchPPS = WriteAddress;
		WriteAddressLatchPPSRound = WriteAddressRound & 0xfff;
		break;
	case 3:	// AE
		WriteAddressLatchAE = WriteAddress;
		WriteAddressLatchAERound = WriteAddressRound;
		break;
	default:
		break;
	}
}
