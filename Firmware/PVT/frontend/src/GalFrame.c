//----------------------------------------------------------------------
// GalFrame.c:
//   Galileo I/NAV frame sync and frame data decode
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "PlatformCtrl.h"
#include "DataTypes.h"
#include "GlobalVar.h"
#include "SupportPackage.h"

static int Distance[64], DistanceNew[64];
static unsigned long long Trace[64], TraceNew[64];
static int GalViterbiDecode(unsigned int SymbolBuffer[30], unsigned int DecodeResult[4]);
static void ViterbiDecodePair(unsigned int SymbolPair);
static void MergeBranches(const int StateArray[8], int DistanceSum);
static int FindMinIndex();

//*************** Galileo Frame sync process ****************
//* assume maximum epoch interval is 1024ms, which is 256 4bit symbols
// Parameters:
//   pChannelStatus: pointer to channel status structure
//   data_count: number of data in data stream
//   data: pointer to symbol array
//   FrameIndex: secondary code index at the epoch of last symbol
// Return value:
//   none
void GalFrameSync(PCHANNEL_STATUS pChannelStatus, int data_count, unsigned int *data, int FrameIndex)
{
	PGPS_FRAME_INFO pFrameInfo = (PGPS_FRAME_INFO)(pChannelStatus->FrameInfo);
	unsigned int SymbolWord = data[0], PageData[4];
	int SymbolIndex = 0, PosIndex;
	unsigned int *pBuffer;

	// meaning of FrameState:
	// -1: frame sync not completed
	// >=0: bit 0 reserved, bit 1 for check sync pattern again
	// if not frame sync, find sync pattern 0101100000 first
	if (pFrameInfo->FrameStatus < 0)
	{
		while (data_count > 0)
		{
			// move in sign bit of the symbol
			pFrameInfo->NavDataStream[0] = (pFrameInfo->NavDataStream[0] << 1) | (SymbolWord >> 31);
			SymbolWord <<= 4;
			data_count --;
			if (((++SymbolIndex) & 7) == 0)
				SymbolWord = data[SymbolIndex >> 3];
			pFrameInfo->NavBitNumber ++;
			if (pFrameInfo->NavBitNumber >= 10)
			{
				if ((pFrameInfo->NavDataStream[0] & 0x3ff) == 0x160)	// sync pattern match
				{
					if (((FrameIndex + 250 - data_count) % 25) == 10)	// align to NH boundary
					{
						// sync pattern found, change frame status to 2
						pFrameInfo->FrameStatus = 2;
						// force symbol number to 0
						pFrameInfo->NavBitNumber = 0;
						break;
					}
				}
				pFrameInfo->NavBitNumber = 9;	// discard oldest symbol
			}
		}
	}
	// put data in SubframeData
	if (pFrameInfo->FrameStatus >= 0)
	{
		pBuffer = &(pFrameInfo->SubframeData[0][0]);
		while (data_count > 0)
		{
			if (pFrameInfo->NavBitNumber < 0)	// sync pattern
			{
				pFrameInfo->NavDataStream[0] = (pFrameInfo->NavDataStream[0] << 1) | (SymbolWord >> 31);
				if ((pFrameInfo->FrameStatus & 2) && pFrameInfo->NavBitNumber == -1)	// need to double check sync pattern
				{
					pFrameInfo->NavDataStream[0] ^= 0x160;
					PosIndex = __builtin_popcount(pFrameInfo->NavDataStream[0] & 0x3ff);	// count number of symbols that sign does not match sync pattern
					if (PosIndex == 0)	// sync pattern double check passed
						pFrameInfo->FrameStatus &= ~2;	// clear check sync pattern flag
					else if (PosIndex >= 2)	// sync pattern double check failed
					{
						pFrameInfo->FrameStatus = -1;	// need to search sync pattern again
						break;
					}
					// otherwise wait to next sync pattern and check again
				}
			}
			else
			{
				PosIndex = (pFrameInfo->NavBitNumber & 7) * 4;
				pBuffer[pFrameInfo->NavBitNumber / 8] &= ~(0xf0000000 >> PosIndex);	// clear 4bit to put symbol
				pBuffer[pFrameInfo->NavBitNumber / 8] |= ((SymbolWord & 0xf0000000) >> PosIndex);	// put symbol (in 4MSB of DataStream) in
			}
			SymbolWord <<= 4;
			data_count --;
			if (((++SymbolIndex) & 7) == 0)
				SymbolWord = data[SymbolIndex >> 3];
			pFrameInfo->NavBitNumber ++;
			if (pFrameInfo->NavBitNumber == 240)	// one page completed
			{
				// do Viterbi decode on page data
				GalViterbiDecode(pBuffer, PageData);
				pFrameInfo->NavBitNumber = -10;	// skip first 10 symbols for next page as sync pattern
			}
		}
	}

	return;
}

//*************** Galileo Viterbi decode for one page ****************
//* assume input symbol is 4bit, totally 240 symbols placed from MSB with interleaving
// Parameters:
//   SymbolBuffer: array of input symbols
//   DecodeResult: decoded result, 120bits MSB first (with first 8bits filled with 0)
// Return value:
//   minimum distance
int GalViterbiDecode(unsigned int SymbolBuffer[30], unsigned int DecodeResult[4])
{
	unsigned int Symbols[30], DataWord = SymbolBuffer[0];	// store deinterleaved symbols
	int i, j, index = 0;
	int TotalMinDistance, MinState;

	memset(Distance, 1, sizeof(Distance));
	Distance[0] = 0;	// set Distance a big value except index 0 to ensure start state is 0
	// deinterleaving
	for (i = 0; i < 8; i ++)
	{
		for (j = 0; j < 30; j ++)
		{
			Symbols[j] = (Symbols[j] << 4) | (DataWord >> 28);
			DataWord <<= 4;
			if (((++index) & 7) == 0)
				DataWord = SymbolBuffer[index >> 3];
		}
	}

	for (i = 0; i < 30 * 4; i ++)
	{
		ViterbiDecodePair(Symbols[i>>2] >> (24 - (i & 3)*8));
		if (i >= 63 && ((i & 7) == 7))	// to reduce the number of comparision, do it every 8 bits
		{
			MinState = FindMinIndex();
			DecodeResult[(i - 56) / 32] = (DecodeResult[(i - 56) / 32] << 8) | ((unsigned int)(Trace[MinState] >> 56));
		}
	}
	MinState = FindMinIndex();
	DecodeResult[2] = (unsigned int)(Trace[MinState] >> 32);
	DecodeResult[3] = (unsigned int)(Trace[MinState]);
	TotalMinDistance = Distance[MinState];

	return 0;
}
static const int OutputTable[4][8] = {
	{ 3, 5, 11, 13, 16, 22, 24, 30, },	// output 00 if input 0
	{ 0, 6,  8, 14, 19, 21, 27, 29, },	// output 01 if input 0
	{ 2, 4, 10, 12, 17, 23, 25, 31, },	// output 10 if input 0
	{ 1, 7,  9, 15, 18, 20, 26, 28, },	// output 11 if input 0
};

//*************** Viterbi decoder to decode one pair of symbols ****************
//* assume input symbol is 4bit, first symbol in bit7~4, second symbol in bit3~0
// Parameters:
//   SymbolPair: input symbols
// Return value:
//   Minimum increased distance
void ViterbiDecodePair(unsigned int SymbolPair)
{
	int DistanceSum;
	int Symbol1, Symbol2;

	Symbol1 = (int)((SymbolPair >> 4) & 0xf);
	Symbol2 = (int)(SymbolPair & 0xf);
	DistanceSum = (Symbol1 ^ 0x7) + (Symbol2 ^ 0x7);	// distance for output 00
	MergeBranches(OutputTable[0], DistanceSum);
	MergeBranches(OutputTable[3], 30 - DistanceSum);
	DistanceSum = (Symbol1 ^ 0x7) + (Symbol2 ^ 0x8);	// distance for output 01
	MergeBranches(OutputTable[1], DistanceSum);
	MergeBranches(OutputTable[2], 30 - DistanceSum);

//	printf("Decode one pair completed with MinState=%d\n", MinState);
	// copy back distances and trace
	memcpy(Distance, DistanceNew, sizeof(Distance));
	memcpy(Trace, TraceNew, sizeof(Trace));
//	for (state = 0; state < 64; state ++)
//		printf("Distance = %8d Trace = %08x%08x\n", Distance[state], (unsigned int)(Trace[state] >> 32), (unsigned int)(Trace[state] & 0xffffffff));
}

//*************** Viterbi decoder to decode one pair of symbols ****************
//* assume input symbol is 4bit, first symbol in bit7~4, second symbol in bit3~0
// Parameters:
//   SymbolPair: input symbols
// Return value:
//   none
void MergeBranches(const int StateArray[8], int DistanceSum)
{
	int Distance00, Distance01, Distance10, Distance11;
	int DistanceCmp = 30 - DistanceSum;	// complement distance
	int i, state;

	for (i = 0; i < 8; i ++)
	{
		state = StateArray[i];
		Distance00 = Distance[state] + DistanceSum;	// distance for state with input 0
		Distance01 = Distance[state] + DistanceCmp;	// distance for state with input 1
		Distance10 = Distance[state+32] + DistanceCmp;	// distance for state+32 with input 0
		Distance11 = Distance[state+32] + DistanceSum;	// distance for state+32 with input 1

		// merge branch to state*2 by comparing Distance00 and Distance10, last bit of trace is 0
		if (Distance00 <= Distance10)	// from first branch (state)
		{
			TraceNew[state*2] = Trace[state] << 1;
			DistanceNew[state*2] = Distance00;
		}
		else	// from first branch (state+32)
		{
			TraceNew[state*2] = Trace[state+32] << 1;
			DistanceNew[state*2] = Distance10;
		}

		// merge branch to state*2+1 by comparing Distance01 and Distance11, last bit of trace is 1
		if (Distance01 <= Distance11)	// from first branch (state)
		{
			TraceNew[state*2+1] = (Trace[state] << 1) + 1;
			DistanceNew[state*2+1] = Distance01;
		}
		else	// from first branch (state+32)
		{
			TraceNew[state*2+1] = (Trace[state+32] << 1) + 1;
			DistanceNew[state*2+1] = Distance11;
		}
	}
}

//*************** Find state index with minimum distance ****************
//* assume input symbol is 4bit, first symbol in bit7~4, second symbol in bit3~0
// Parameters:
//   none
// Return value:
//   state with minimum distance
int FindMinIndex()
{
	int i, MinState = 0;
	int MinDistance = 32 * 250;	// maximum distance 32 * 250

	// calculate minimum distance
	for (i = 0; i < 64; i ++)
	{
		if (MinDistance > Distance[i])
		{
			MinDistance = Distance[i];
			MinState = i;
		}
	}

	return MinState;
}
