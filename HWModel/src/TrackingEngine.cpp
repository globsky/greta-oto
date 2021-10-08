//----------------------------------------------------------------------
// TrackingEngine.cpp:
//   Tracking engine class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <malloc.h>
#include <memory.h>
#include "CommonOps.h"
#include "RegAddress.h"
#include "TrackingEngine.h"

#define COH_OFFSET(ch_index, cor_index) ((ch_index << 5) + 24 + (cor_index >> 2))

CTrackingEngine::CTrackingEngine(CTeFifoMem *pTeFifo, unsigned int *MemCodeBuffer) : pTeFifo(pTeFifo)
{
	int i;

	for (i = 0; i < PHYSICAL_CHANNEL_NUMBER; i ++)
		Correlator[i] = new CCorrelator(PrnPolyLength, MemCodeBuffer);
	Reset();
	TEBuffer = (unsigned int *)malloc(TE_BUFFER_SIZE);
	memset(TEBuffer, 0, TE_BUFFER_SIZE);

	FifoData = (complex_int *)malloc(65536 * sizeof(complex_int));
}

CTrackingEngine::~CTrackingEngine()
{
	int i;
	free(TEBuffer);
	free(FifoData);
	for (i = 0; i < PHYSICAL_CHANNEL_NUMBER; i ++)
		delete Correlator[i];
}

void CTrackingEngine::Reset()
{
	int i;

	for (i = 0; i < PHYSICAL_CHANNEL_NUMBER; i ++)
		Correlator[i]->Reset();
	NoiseCalc.Reset();
	ChannelEnable = CohDataReady = 0;
	OverwriteProtectAddr = 0;
	OverwriteProtectValue = 0;
}

void CTrackingEngine::SetRegValue(int Address, U32 Value)
{
	Address &= 0xff;
	switch (Address)
	{
	case ADDR_OFFSET_TE_CHANNEL_ENABLE:
		ChannelEnable = Value;
		break;
	case ADDR_OFFSET_TE_COH_DATA_READY:
		CohDataReady = Value;
		break;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_CHANNEL:
		OverwriteProtectChannel = Value;
		break;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_ADDR:
		OverwriteProtectAddr = Value;
		break;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_VALUE:
		OverwriteProtectValue = Value;
		break;
	case ADDR_OFFSET_TE_POLYNOMIAL:
		PrnPolyLength[0] = Value;
		break;
	case ADDR_OFFSET_TE_CODE_LENGTH:
		PrnPolyLength[1] = Value;
		break;
	case ADDR_OFFSET_TE_POLYNOMIAL2:
		PrnPolyLength[2] = Value;
		break;
	case ADDR_OFFSET_TE_CODE_LENGTH2:
		PrnPolyLength[3] = Value;
		break;
	case ADDR_OFFSET_TE_NOISE_CONFIG:
		NoiseCalc.SetSmoothFactor(Value);
		break;
	case ADDR_OFFSET_TE_NOISE_FLOOR:
		NoiseCalc.SetNoise(Value);
		break;
	default:
		break;
	}
}

U32 CTrackingEngine::GetRegValue(int Address)
{
	Address &= 0xff;
	switch (Address)
	{
	case ADDR_OFFSET_TE_CHANNEL_ENABLE:
		return ChannelEnable;
	case ADDR_OFFSET_TE_COH_DATA_READY:
		return CohDataReady;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_CHANNEL:
		return OverwriteProtectChannel;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_ADDR:
		return OverwriteProtectAddr;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_VALUE:
		return OverwriteProtectValue;
	case ADDR_OFFSET_TE_POLYNOMIAL:
		return PrnPolyLength[0];
	case ADDR_OFFSET_TE_CODE_LENGTH:
		return PrnPolyLength[1];
	case ADDR_OFFSET_TE_POLYNOMIAL2:
		return PrnPolyLength[2];
	case ADDR_OFFSET_TE_CODE_LENGTH2:
		return PrnPolyLength[3];
	case ADDR_OFFSET_TE_NOISE_CONFIG:
		return NoiseCalc.SmoothFactor;
	case ADDR_OFFSET_TE_NOISE_FLOOR:
		return NoiseCalc.GetNoise();
	default:
		return 0;
	}
}

// process one system
// input parameter specify the system index
// return 1 if any correlator in any channel has data ready
int CTrackingEngine::ProcessData()
{
	unsigned int EnableMask;
	int i, j, TrackingChannelCount;
	int TrackingChannelIndex[PHYSICAL_CHANNEL_NUMBER];
	// Array length is 32 for DumpDataI, DumpDataQ and CohAddress
	// for RTL implementation, FIFO depth 16 is ok
	S16 DumpDataI[16], DumpDataQ[16];
	int CorIndex[16];
	int DumpCount;
	int ReadNumber;
	unsigned int CohData;
	S16 CohDataI, CohDataQ;
	int FirstRound = 1;

	// clear coherent data ready flag and overwrite protect flag
	CohDataReady = 0;
	OverwriteProtectChannel = 0;

	// if no channel enabled, send virtual read to FIFO
	if (ChannelEnable == 0)
	{
		pTeFifo->SkipBlock();
		return 0;
	}

	// loop for all enabled channel
	EnableMask = ChannelEnable;
	while (EnableMask)
	{
		if (FirstRound)
			Correlator[0]->NoiseCalc = &NoiseCalc;
		// find at most 4 active channel
		TrackingChannelCount = 0;
		while (EnableMask && TrackingChannelCount < PHYSICAL_CHANNEL_NUMBER)
		{
			TrackingChannelIndex[TrackingChannelCount] = FindLeastIndex(EnableMask);
			EnableMask &= ~(1 << TrackingChannelIndex[TrackingChannelCount]);
			TrackingChannelCount ++;
		}
		
		// read data from TE FIFO
		pTeFifo->ReadData(ReadNumber, FifoData);
//		for (i = 0; i < ReadNumber; i ++)
//			if (SetIndex == 1)
//				printf("%x\n", FifoData[i]);
		// process all physical channels
		for (i = 0; i < TrackingChannelCount; i ++)
		{
			// if any correlator reaches coherent value, set data ready flag
			if (Correlator[i]->Correlation(&TEBuffer[TrackingChannelIndex[i] << 5], ReadNumber, FifoData, DumpDataI, DumpDataQ, CorIndex, DumpCount))
				CohDataReady |= 1 << TrackingChannelIndex[i];
			for (j = 0; j < DumpCount; j ++)
			{
				// if overwrite protect bit is set, set corresponding flag bit and set address and value. do NOT accumulate
				if (CorIndex[j] & 2)
				{
					OverwriteProtectChannel |= 1 << TrackingChannelIndex[i];
					OverwriteProtectAddr = COH_OFFSET(TrackingChannelIndex[i], CorIndex[j]) << 2;
					OverwriteProtectValue = ((unsigned int)DumpDataI[j] << 16) | ((unsigned int)DumpDataQ[j] & 0xffff);
					continue;
				}

				// if this is first epoch of coherent data accumulation, clear stored value
				if (CorIndex[j] & 1)
					CohData = 0;
				else
					CohData = TEBuffer[COH_OFFSET(TrackingChannelIndex[i], CorIndex[j])];
				CohDataI = (S16)(CohData >> 16);
				CohDataQ = (S16)(CohData & 0xffff);
				CohDataI += DumpDataI[j];
				CohDataQ += DumpDataQ[j];
				CohData = ((unsigned int)CohDataI << 16) | ((unsigned int)CohDataQ & 0xffff);
				TEBuffer[COH_OFFSET(TrackingChannelIndex[i], CorIndex[j])] = CohData;
			}
		}
		// rewind FIFO read pointer
		pTeFifo->RewindPointer();
		FirstRound = 0;
		Correlator[0]->NoiseCalc = NULL;
	}
	pTeFifo->SkipBlock();

	return (CohDataReady != 0);
}

// find the index of bit 1 counting from LSB
// caller will ensure input argument data will not be 0
int CTrackingEngine::FindLeastIndex(unsigned int data)
{
	int index = 0;

	if (!(data & 0xffff))
	{
		data >>= 16;
		index += 16;
	}
	if (!(data & 0xff))
	{
		data >>= 8;
		index += 8;
	}
	if (!(data & 0xf))
	{
		data >>= 4;
		index += 4;
	}
	if (!(data & 0x3))
	{
		data >>= 2;
		index += 2;
	}
	if (!(data & 0x1))
		index += 1;

	return index;
}
