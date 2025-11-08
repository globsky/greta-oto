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
#include "TaskManager.h"
#include "GlobalVar.h"
#include "SupportPackage.h"

#define PAYLOAD_LENGTH 4	// 128bit page contents
#define PACKAGE_LENGTH (sizeof(SYMBOL_PACKAGE) + sizeof(unsigned int)*(PAYLOAD_LENGTH))	// 3 variables + 4 payload

extern U32 EphAlmMutex;

static int GalPageProc(PFRAME_INFO GalFrameInfo, PDATA_FOR_DECODE DataForDecode);
static int INavPageDecode(void* Param);
static int DecodeGalileoEphemeris(int svid, const unsigned int data[20]);

// for Viterbi decode
static int Distance[64], DistanceNew[64];
static unsigned long long Trace[64], TraceNew[64];
static int GalViterbiDecode(unsigned int SymbolBuffer[30], unsigned int DecodeResult[4]);
static void ViterbiDecodePair(unsigned int SymbolPair);
static void MergeBranches(const int StateArray[8], int DistanceSum);
static int FindMinIndex();

//*************** Galileo navigation data process ****************
//* Do Galileo navigation data process and page sync
//* meaning of FrameStatus:
//* -1: frame sync not completed
//* >=0: bit0 for even page valid, bit 1 for check sync pattern again (confirm page sync correct)
// Parameters:
//   pFrameInfo: pointer to frame info structure
//   DataForDecode: pointer to structure of symbols to be decoded
// Return value:
//   current millisecond count within current week (negative if unknown)
int GalNavDataProc(PFRAME_INFO pFrameInfo, PDATA_FOR_DECODE DataForDecode)
{
	int data_count = 8;	// DataStream contais 8 4bit data
	int SymbolCount = -1;
	U32 DataStream = DataForDecode->DataStream;
	unsigned int *pBuffer;
	int PosIndex;
	pFrameInfo->TimeTag = -1;	// reset decoded week number to invalid

	// meaning of FrameState:
	// -1: frame sync not completed
	// >=0: bit 0 reserved, bit 1 for check sync pattern again
	// if not frame sync, find sync pattern 0101100000 first
	if (pFrameInfo->FrameStatus < 0)
	{
		while (data_count > 0)
		{
			// move in sign bit of the symbol
			pFrameInfo->SymbolData[0] = (pFrameInfo->SymbolData[0] << 1) | (DataStream >> 31);
			DataStream <<= 4;
			data_count --;
			pFrameInfo->SymbolNumber ++;
			if (pFrameInfo->SymbolNumber >= 10)
			{
				if ((pFrameInfo->SymbolData[0] & 0x3ff) == 0x160)	// sync pattern match
				{
					if (((DataForDecode->SymbolIndex - data_count) % 25) == 10)	// check sync pattern align to NH boundary, add 25 to avoid negative value
					{
						// sync pattern found, change frame status to 2
						pFrameInfo->FrameStatus = 2;
						// force symbol number to 0
						pFrameInfo->SymbolNumber = 0;
						break;
					}
				}
				pFrameInfo->SymbolNumber = 9;	// discard oldest symbol
			}
		}
	}
	// put data in FrameData
	if (pFrameInfo->FrameStatus >= 0)
	{
		pBuffer = &(pFrameInfo->FrameData[0]);
		while (data_count > 0)
		{
			if (pFrameInfo->SymbolNumber < 0)	// sync pattern
			{
				pFrameInfo->SymbolData[0] = (pFrameInfo->SymbolData[0] << 1) | (DataStream >> 31);
				if ((pFrameInfo->FrameStatus & 2) && pFrameInfo->SymbolNumber == -1)	// need to double check sync pattern
				{
					pFrameInfo->SymbolData[0] ^= 0x160;
					PosIndex = __builtin_popcount(pFrameInfo->SymbolData[0] & 0x3ff);	// count number of symbols that sign does not match sync pattern
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
				PosIndex = (pFrameInfo->SymbolNumber & 7) * 4;
				pBuffer[pFrameInfo->SymbolNumber / 8] &= ~(0xf0000000 >> PosIndex);	// clear 4bit to put symbol
				pBuffer[pFrameInfo->SymbolNumber / 8] |= ((DataStream & 0xf0000000) >> PosIndex);	// put symbol (in 4MSB of DataStream) in
			}
			DataStream <<= 4;
			data_count --;
			pFrameInfo->SymbolNumber ++;
			if (pFrameInfo->SymbolNumber == 240)	// one page completed
			{
				// do Viterbi decode on page data
				GalViterbiDecode(pBuffer, pFrameInfo->FrameData);
				pFrameInfo->SymbolNumber = -10;	// skip first 10 symbols for next page as sync pattern
				SymbolCount = GalPageProc(pFrameInfo, DataForDecode) + data_count;
			}
		}
	}

	return SymbolCount * 4;
}

//*************** Do Galileo page part process ****************
// Parameters:
//   GalFrameInfo: pointer to frame info structure
//   PageData: 
//   DataForDecode: pointer to structure of symbols to be decoded
// Return value:
//   current symbols within week, <0 if unknown
int GalPageProc(PFRAME_INFO GalFrameInfo, PDATA_FOR_DECODE DataForDecode)
{
	unsigned int *PageData = GalFrameInfo->FrameData;
	unsigned int crc;
	unsigned int Package[PACKAGE_LENGTH];
	PSYMBOL_PACKAGE SymbolPackage = (PSYMBOL_PACKAGE)Package;
	int WordType;
	int tow = -1, wn = -1;

	// after decode, MSB of first DWORD is even/odd flag
	// each page part has 120 valid bit, leaving 8LSB of last DWORD unused
 	if (PageData[0] & 0x80000000)	// odd page
	{
		if (GalFrameInfo->FrameStatus & 1)	// even page valid
		{
			// put decoded odd page part (remove 6bit tail) into GalFrameInfo->SymbolData
			// starting bit17 of GalFrameInfo->SymbolData[5] until bit0 of GalFrameInfo->SymbolData[7] (totally 82bits)
			GalFrameInfo->SymbolData[5] |= PageData[0] >> 14;
			GalFrameInfo->SymbolData[6] = (PageData[0] << 18) | (PageData[1] >> 14);
			GalFrameInfo->SymbolData[7] = (PageData[1] << 18) | ((PageData[2] >> 6) & 0x3ffff);
			// extract 24bit CRC
			crc = ((PageData[2] << 18) & 0xfc0000) | (PageData[3] >> 14);
			GalFrameInfo->FrameStatus &= ~1;	// clear even page valid flag
			if (Crc24qEncode(&GalFrameInfo->SymbolData[1], 196) != crc)	// CRC check fail
				return -1;
			// send whole nominal page to decode
			SymbolPackage->ChannelState = DataForDecode->ChannelState;
			SymbolPackage->FrameInfo = GalFrameInfo;	// put pointer of FrameInfo in FrameIndex
			SymbolPackage->PayloadLength = 4;
			SymbolPackage->Symbols[0] = (GalFrameInfo->SymbolData[1] << 30) | (GalFrameInfo->SymbolData[2] >> 2);
			SymbolPackage->Symbols[1] = (GalFrameInfo->SymbolData[2] << 30) | (GalFrameInfo->SymbolData[3] >> 2);
			SymbolPackage->Symbols[2] = (GalFrameInfo->SymbolData[3] << 30) | (GalFrameInfo->SymbolData[4] >> 2);
			SymbolPackage->Symbols[3] = (GalFrameInfo->SymbolData[4] << 30) | ((GalFrameInfo->SymbolData[5] >> 2) & 0x3fff0000);
			SymbolPackage->Symbols[3] |= (GalFrameInfo->SymbolData[5] & 0xffff);
			AddToTask(TASK_POSTMEAS, INavPageDecode, SymbolPackage, PACKAGE_LENGTH);
			// decode week number and week millisecond
			WordType = SymbolPackage->Symbols[0] >> 26;
			if (WordType == 0)
			{
				tow = (SymbolPackage->Symbols[3] & 0xfffff) + 2;	// plus current decoded page
				wn = (SymbolPackage->Symbols[3] >> 20) & 0xfff;
			}
			else if (WordType == 5)
			{
				tow = (((SymbolPackage->Symbols[2] & 0x7ff) << 9) | (SymbolPackage->Symbols[3] >> 23)) + 2;	// plus current decoded page
				wn = (SymbolPackage->Symbols[2] >> 11) & 0xfff;
			}
			else if (WordType == 6)
			{
				tow = ((SymbolPackage->Symbols[3] >> 3) & 0xfffff) + 2;	// plus current decoded page
			}
			GalFrameInfo->TimeTag = wn;	// set decoded week number to TimeTag
		}
	}
	else	// even page
	{
		// put decoded even page part (remove 6bit tail) into GalFrameInfo->SymbolData
		// starting bit3 of GalFrameInfo->SymbolData[1] until bit18 of GalFrameInfo->SymbolData[5] (totally 114bits)
		GalFrameInfo->SymbolData[1] = PageData[0] >> 28;
		GalFrameInfo->SymbolData[2] = (PageData[0] << 4) | (PageData[1] >> 28);
		GalFrameInfo->SymbolData[3] = (PageData[1] << 4) | ((PageData[2] >> 20) & 0xf);
		GalFrameInfo->SymbolData[4] = (PageData[2] << 12) | (PageData[3] >> 20);
		GalFrameInfo->SymbolData[5] = (PageData[3] << 12) & 0xfffc0000;
		GalFrameInfo->FrameStatus |= 1;	// set even page valid flag
	}

	return tow * 250;	// 250 symbols per second
}

//*************** Task function to process one Galileo nominal page ****************
// Parameters:
//   Param: pointer to structure holding one page data
// Return value:
//   0
int INavPageDecode(void* Param)
{
	PSYMBOL_PACKAGE SymbolPackage = (PSYMBOL_PACKAGE)Param;
	int Svid = SymbolPackage->ChannelState->Svid;
	PFRAME_INFO FrameInfo = SymbolPackage->FrameInfo;
	int WordType = SymbolPackage->Symbols[0] >> 26;

	if (WordType >= 1 && WordType <= 5)	// Ephemeris/SISA/Clock/Iono/BGD
	{
		FrameInfo->FrameData[WordType * 4 + 26] = SymbolPackage->Symbols[0];
		FrameInfo->FrameData[WordType * 4 + 27] = SymbolPackage->Symbols[1];
		FrameInfo->FrameData[WordType * 4 + 28] = SymbolPackage->Symbols[2];
		FrameInfo->FrameData[WordType * 4 + 29] = SymbolPackage->Symbols[3];
		FrameInfo->FrameFlag |= (1 << (WordType - 1));
	}
	else if (WordType >= 7 && WordType <= 10)	// Almanac
	{
		// TODO: decode E1 almanac
	}

	if ((FrameInfo->FrameFlag & 0x1f) == 0x1f)	// bit0~4 set, WordType 1~5 completed
	{
		DecodeGalileoEphemeris(Svid, FrameInfo->FrameData + 30);
		FrameInfo->FrameFlag &= ~0x1f;
	}

	return 0;
}

int DecodeGalileoEphemeris(int svid, const unsigned int data[20])
{
	PGNSS_EPHEMERIS pEph = &g_GalileoEphemeris[svid-1];
	unsigned int iod;

	// check IOD identical and svid matches source signal
	if (svid != GET_UBITS(data[12], 10, 6))
		return 0;
	iod = GET_UBITS(data[12], 16, 10);
	if (pEph->flag == 1 && pEph->iodc == iod)	// do not decode repeat ephemeris
		return 1;
	if (iod != GET_UBITS(data[0], 16, 10) || iod != GET_UBITS(data[4], 16, 10) || iod != GET_UBITS(data[8], 16, 10))
		return 0;

	DEBUG_OUTPUT(OUTPUT_CONTROL(DATA_DECODE, INFO), "Decode ephemeris of E%02d\n", svid);
	MutexTake(EphAlmMutex);

	pEph->svid = svid - 1 + MIN_GAL_SAT_ID;
	pEph->health = 0;
	pEph->flag = 1;
	pEph->iodc = (unsigned short)iod;

	pEph->toe = GET_UBITS(data[0], 2, 14) * 60;
	pEph->M0 = ScaleDouble(((data[0] << 30) & 0xc0000000) | GET_UBITS(data[1], 2, 30), 31) * PI;
	pEph->ecc = ScaleDoubleU(((data[1] << 30) & 0xc0000000) | GET_UBITS(data[2], 2, 30), 33);
	pEph->sqrtA = ScaleDoubleU(((data[2] << 30) & 0xc0000000) | GET_UBITS(data[3], 2, 30), 19);
	pEph->omega0 = ScaleDouble(((data[4] << 16) & 0xffff0000) | GET_UBITS(data[5], 16, 16), 31) * PI;
	pEph->i0 = ScaleDouble(((data[5] << 16) & 0xffff0000) | GET_UBITS(data[6], 16, 16), 31) * PI;
	pEph->w = ScaleDouble(((data[6] << 16) & 0xffff0000) | GET_UBITS(data[7], 16, 16), 31) * PI;
	pEph->idot = ScaleDouble(GET_BITS(data[7], 2, 14), 43) * PI;
	pEph->omega_dot = ScaleDouble((GET_BITS(data[8], 0, 16) << 8) | GET_UBITS(data[9], 24, 8), 43) * PI;
	pEph->delta_n = ScaleDouble(GET_BITS(data[9], 8, 16), 43) * PI;
	pEph->cuc = ScaleDouble((GET_BITS(data[9], 0, 8) << 8) | GET_UBITS(data[10], 24, 8), 29);
	pEph->cus = ScaleDouble(GET_BITS(data[10], 8, 16), 29);
	pEph->crc = ScaleDouble((GET_BITS(data[10], 0, 8) << 8) | GET_UBITS(data[11], 24, 8), 5);
	pEph->crs = ScaleDouble(GET_BITS(data[11], 8, 16), 5);
	pEph->cic = ScaleDouble((GET_BITS(data[12], 0, 10) << 6) | GET_UBITS(data[13], 26, 6), 29);
	pEph->cis = ScaleDouble(GET_BITS(data[13], 10, 16), 29);
	pEph->toc = (((data[13] & 0x3ff) << 4) | GET_UBITS(data[14], 28, 4)) * 60;
	pEph->af0 = ScaleDouble((GET_BITS(data[14], 0, 28) << 3) | GET_UBITS(data[15], 29, 3), 34);
	pEph->af1 = ScaleDouble(GET_BITS(data[15], 8, 21), 46);
	pEph->af2 = ScaleDouble(GET_BITS(data[15], 2, 6), 59);
	pEph->tgd = ScaleDouble(GET_BITS(data[17], 7, 10), 32);
	pEph->tgd2 = ScaleDouble((GET_BITS(data[17], 0, 7) << 3) | GET_UBITS(data[18], 29, 3), 32);
	pEph->week = GET_UBITS(data[18], 11, 12);

	// calculate derived variables
	pEph->axis = pEph->sqrtA * pEph->sqrtA;
	pEph->n = WGS_SQRT_GM / (pEph->sqrtA * pEph->axis) + pEph->delta_n;
	pEph->root_ecc = sqrt(1.0 - pEph->ecc * pEph->ecc);
	pEph->omega_t = pEph->omega0 - WGS_OMEGDOTE * pEph->toe;
	pEph->omega_delta = pEph->omega_dot - WGS_OMEGDOTE;

	MutexGive(EphAlmMutex);
	return 1;
}

//*************** Galileo Viterbi decode for one page ****************
//* assume input symbol is 4bit, totally 240 symbols placed from MSB with interleaving
// Parameters:
//   SymbolBuffer: array of input symbols
//   DecodeResult: decoded result, 120bits MSB first (8LSB of DecodeResult[1] and 8MSB of DecodeResult[2] will overlap)
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
