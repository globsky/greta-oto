//----------------------------------------------------------------------
// BdsFrame.c:
//   BDS frame sync and frame data decode
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "PlatformCtrl.h"
#include "ChannelManager.h"
#include "DataTypes.h"
#include "TaskManager.h"
#include "GlobalVar.h"
#include "SupportPackage.h"

#define SUBFRAME2_LENGTH 18
#define SUBFRAME3_LENGTH 8
#define PAYLOAD_LENGTH (SUBFRAME2_LENGTH + SUBFRAME3_LENGTH)	// 18 DWORD for subframe2 and 8 DWORD for subframe3
#define PACKAGE_LENGTH (sizeof(SYMBOL_PACKAGE) + sizeof(unsigned int)*(PAYLOAD_LENGTH))	// 3 variables + 26 payload

extern U32 EphAlmMutex;

static void PutColumnData(unsigned int ColumnData[9], int ColumnIndex, U16 Subframe23Data[]);
static int BdsFrameProc(PFRAME_INFO BdsFrameInfo, PDATA_FOR_DECODE DataForDecode);
static int BdsFrameDecode(void* Param);
static int DecodeBdsEphemeris(int svid, const unsigned int data[SUBFRAME2_LENGTH]);
static int DecodeBdsMidiAlm(const unsigned int data[SUBFRAME3_LENGTH]);

//*************** BDS navigation data process ****************
//* do BDS CNAV1 frame process
//* meaning of FrameStatus:
//* -1: SymbolNumber not aligned with symbol position within frame
//* 0: current data in subframe1, SymbolNumber is number of symbols in subframe1
//* 1~48: current data in corresponding column of subframe2/3, SymbolNumber is number of symbols in current column
//* if SymbolNumber is negative, means discard symbols until SymbolNumber >= 0
// Parameters:
//   pFrameInfo: pointer to frame info structure
//   DataForDecode: pointer to structure of symbols to be decoded
// Return value:
//   current millisecond count within current week (negative if unknown)
int BdsNavDataProc(PFRAME_INFO pFrameInfo, PDATA_FOR_DECODE DataForDecode)
{
	int data_count = 4, SymbolCount = -1;	// DataStream contais 4 8bit symbol
	unsigned int Symbol;

	pFrameInfo->TimeTag = -1;	// reset decoded week number to invalid
	if (pFrameInfo->FrameStatus < 0)
	{
		pFrameInfo->FrameStatus = 0;
		pFrameInfo->SymbolNumber = (DataForDecode->SymbolIndex <= 4) ? (DataForDecode->SymbolIndex - 4) : (DataForDecode->SymbolIndex - (1800 + 4));
	}
	while (data_count > 0)
	{
		data_count -- ;
		if (pFrameInfo->SymbolNumber < 0)
		{
			pFrameInfo->SymbolNumber ++;
			DataForDecode->DataStream <<= 8;
			continue;
		}
		Symbol = (DataForDecode->DataStream >> 24);	// get one 8bit symbol
		DataForDecode->DataStream <<= 8;
		if (pFrameInfo->FrameStatus == 0)	// subframe1 data, put in first 3 DWORD of FrameData
		{
			if (pFrameInfo->SymbolNumber == 0)
				pFrameInfo->FrameData[0] = pFrameInfo->FrameData[1] = pFrameInfo->FrameData[2] = 0;
			pFrameInfo->FrameData[pFrameInfo->SymbolNumber/32] |= (((Symbol & 0x80) ? 1 : 0) << (31 - (pFrameInfo->SymbolNumber & 0x1f)));
			if (++pFrameInfo->SymbolNumber == 72)	// subframe1 completed
			{
				pFrameInfo->FrameStatus = 1;	// next to decode subframe2/3
				pFrameInfo->SymbolNumber = 0;
			}
		}
		else	// subframe2/3 data, first put in SymbolData, the deinterleave to FrameData
		{
			if (pFrameInfo->SymbolNumber == 0)
				memset(pFrameInfo->SymbolData, 0, sizeof(unsigned int) * 9);	// 9 DWORD x 4 = 36 symbols per column
			pFrameInfo->SymbolData[pFrameInfo->SymbolNumber/4] |= Symbol << (24 - ((pFrameInfo->SymbolNumber & 0x3) << 3));
			if (++pFrameInfo->SymbolNumber == 36)	// one column completed
			{
				pFrameInfo->SymbolNumber = 0;
				PutColumnData(pFrameInfo->SymbolData, pFrameInfo->FrameStatus - 1, (U16*)(pFrameInfo->FrameData + 3));
				if (++pFrameInfo->FrameStatus == 49)
				{
					pFrameInfo->FrameStatus = 0;	// next to decode subframe1
					if ((SymbolCount = BdsFrameProc(pFrameInfo, DataForDecode)) >= 0);
						SymbolCount += data_count;
				}
			}
		}
	}

	return SymbolCount * 10;
}

//*************** Put interleaved one column symbols into subframe2/3 buffer ****************
//* ColumnData has 36 8bit symbols, index 0 and MSB first
//* Subframe23Data has 1bit symbols, index 0 and MSB first, first 75 WORD for subframe2, next 33 WORD for subframe3
// Parameters:
//   ColumnData: Array holding symbols of one column
//   ColumnIndex: Index of current column
//   Subframe23Data: 16bit WORD array holding subframe2 and subframe3 data
// Return value:
//   none
void PutColumnData(unsigned int ColumnData[9], int ColumnIndex, U16 Subframe23Data[])
{
	int i, index = 0;
	U16 *pSubframe2, *pSubframe3, SymbolSign;

	pSubframe2 = Subframe23Data + ColumnIndex / 16;
	pSubframe3 = pSubframe2 + 75;

	for (i = 0; i < 12; i ++)
	{
		SymbolSign = (ColumnData[index / 4] & (0x80000000 >> ((index & 3) << 3))) ? 1 : 0; index ++;
		(*pSubframe2) = ((*pSubframe2) << 1) + SymbolSign; pSubframe2 += 3;
		SymbolSign = (ColumnData[index / 4] & (0x80000000 >> ((index & 3) << 3))) ? 1 : 0; index ++;
		(*pSubframe2) = ((*pSubframe2) << 1) + SymbolSign; pSubframe2 += 3;
		SymbolSign = (ColumnData[index / 4] & (0x80000000 >> ((index & 3) << 3))) ? 1 : 0; index ++;
		if (i == 11)
			(*pSubframe2) = ((*pSubframe2) << 1) + SymbolSign;
		else
			(*pSubframe3) = ((*pSubframe3) << 1) + SymbolSign;
		pSubframe3 += 3;
	}
}

//*************** Do BDS B-CNAV1 frame process ****************
// Parameters:
//   BdsFrameInfo: Pointer to BDS frame info structure
//   DataForDecode: pointer to structure of symbols to be decoded
// Return value:
//   current symbols within week, -1 if unknown
int BdsFrameProc(PFRAME_INFO BdsFrameInfo, PDATA_FOR_DECODE DataForDecode)
{
	int i, svid, soh, how, SymbolCount = -1;
	U16 *SubFrame2Data = (U16 *)(BdsFrameInfo->FrameData + 3);
	U16 *SubFrame3Data = SubFrame2Data + 75;
	unsigned int Data, Package[PACKAGE_LENGTH];
	PSYMBOL_PACKAGE SymbolPackage = (PSYMBOL_PACKAGE)Package;
	unsigned int *Symbols = SymbolPackage->Symbols;
	unsigned int crc;

	// decode subframe 1
	svid = (int)(BdsFrameInfo->FrameData[0] >> 26);	// TODO: BCH decode
	soh = (int)((BdsFrameInfo->FrameData[0] >> 3) & 0xff);	// TODO: BCH decode

	SymbolPackage->ChannelState = DataForDecode->ChannelState;
	SymbolPackage->FrameInfo = BdsFrameInfo;
	SymbolPackage->PayloadLength = 0;
	// put two 16bit DOWRD together to form 18 DWORD (576bits) for subframe2
 	for (i = 0; i < 18; i ++)
	{
		Data = (((unsigned int)(*SubFrame2Data)) << 16) + (*(SubFrame2Data + 1));
		(*Symbols ++) = Data;
		SubFrame2Data += 2;
	}
	crc = (((unsigned int)(*SubFrame2Data)) << 16) + (*(SubFrame2Data + 1));
	crc >>= 8;
	if (Crc24qEncode(SymbolPackage->Symbols, 576) == crc)	// CRC check successfully
		SymbolPackage->PayloadLength += SUBFRAME2_LENGTH;
	else
		Symbols = SymbolPackage->Symbols;	// discard subframe2
	// put two 16bit DOWRD together to form 8 DWORD (240bits plus 16 leading 0s) for subframe3
	(*Symbols ++) = (unsigned int)(*SubFrame3Data ++);
	for (i = 0; i < 7; i ++)
	{
		Data = (((unsigned int)(*SubFrame3Data)) << 16) + (*(SubFrame3Data + 1));
		(*Symbols ++) = Data;
		SubFrame3Data += 2;
	}
	crc = (((unsigned int)(*SubFrame3Data)) << 16) + (*(SubFrame3Data + 1));
	crc >>= 8;
	if (Crc24qEncode(Symbols - 8, 240) == crc)	// CRC check successfully
		SymbolPackage->PayloadLength += SUBFRAME3_LENGTH;
	if (SymbolPackage->PayloadLength != 0)
		AddToTask(TASK_POSTMEAS, BdsFrameDecode, SymbolPackage, PACKAGE_LENGTH);
	// decode week number and HOW
	if (SymbolPackage->PayloadLength >= SUBFRAME2_LENGTH)	// subframe2 decode OK
	{
		BdsFrameInfo->TimeTag = GET_UBITS(SymbolPackage->Symbols[0], 19, 13);	// set week number into TimeTag
		how = GET_UBITS(SymbolPackage->Symbols[0], 11, 8);
		SymbolCount = (how * 200 + soh + 1) * 1800;	// start of frame plus one whole frame length
	}

	return SymbolCount;
}

//*************** Task function to process one BDS CNAV1 frame ****************
// PayloadLength is either SUBFRAME2_LENGTH or SUBFRAME3_LENGTH or SUBFRAME2_LENGTH+SUBFRAME3_LENGTH
// indicating the contents has subframe2 or subframe3 or both
// Parameters:
//   Param: pointer to structure holding one subframe data
// Return value:
//   0
int BdsFrameDecode(void* Param)
{
	PSYMBOL_PACKAGE SymbolPackage = (PSYMBOL_PACKAGE)Param;
	int svid = SymbolPackage->ChannelState->Svid;
	int PageNumber = -1;
	unsigned int *Symbols = SymbolPackage->Symbols + SUBFRAME2_LENGTH;

	// decode subframe2 for ephemeris
	if (SymbolPackage->PayloadLength >= SUBFRAME2_LENGTH)
		DecodeBdsEphemeris(svid, SymbolPackage->Symbols);

	if (SymbolPackage->PayloadLength == SUBFRAME3_LENGTH)	// contents have only subframe3
		PageNumber = SymbolPackage->Symbols[0] >> 26;
	else if (SymbolPackage->PayloadLength == PAYLOAD_LENGTH)	// contents have both subframe2 and subframe3
		PageNumber = SymbolPackage->Symbols[SUBFRAME2_LENGTH] >> 26;

	if (PageNumber == 4)
		DecodeBdsMidiAlm(&SymbolPackage->Symbols[19]);
	return 0;
}

//*************** Decode BDS frame data to get ephemeris ****************
// Parameters:
//   svid: GPS SVID ranging from 1 to 32
//   data: Subframe2 data, totally 600bit
// Return value:
//   1 if decode success or 0 if decode fail
int DecodeBdsEphemeris(int svid, const unsigned int data[SUBFRAME2_LENGTH])
{
	PGNSS_EPHEMERIS pEph = &g_BdsEphemeris[svid-1];
	unsigned short iodc;
	int idata;
	unsigned int type;
	long long ilong;
	unsigned long long ulong;

	pEph->svid = svid - 1 + MIN_BDS_SAT_ID;
	iodc = GET_UBITS(data[0], 1, 10);
	if (pEph->flag == 1 && pEph->iodc == iodc)	// do not decode repeat ephemeris
		return 1;

	MutexTake(EphAlmMutex);

	pEph->health = 0;
	pEph->flag = 1;
	pEph->iode2 = GET_UBITS(data[1], 25, 7) | ((data[0] & 1) ? 128 : 0);
	pEph->week = GET_UBITS(data[0], 19, 13);
	pEph->iodc = iodc;

	// Ephemeris I:
	type = GET_UBITS(data[1], 12, 2);
	pEph->toe = (int)GET_UBITS(data[1], 14, 11) * 300;
	idata = (GET_BITS(data[1], 0, 12) << 14) | GET_UBITS(data[2], 18, 14);
	pEph->axis = ((type == 3) ? 27906100.0 : 42162200.0) + ScaleDouble(idata, 9);	// major-axis
	idata = (GET_BITS(data[2], 0, 18) << 7) | GET_UBITS(data[3], 25, 7);
	pEph->axis_dot = ScaleDouble(idata, 21);
	pEph->delta_n = ScaleDouble(GET_BITS(data[3], 8, 17), 44) * PI; // .44 * PI
	idata = (GET_BITS(data[3], 0, 8) << 15) | GET_UBITS(data[4], 15, 17);
//	pEph->delta_n_dot = ScaleDouble(idata, 58) * PI;	// .57 * PI * 1/2
	ilong = GET_BITS(data[4], 0, 17);
	ilong  = (ilong << 16) | GET_UBITS(data[5], 16, 16);
	pEph->M0 = ScaleDoubleLong(ilong, 32) * PI;	// .32 * PI
	ulong = GET_UBITS(data[5], 0, 16);
	ulong  = (ulong << 17) | GET_UBITS(data[6], 15, 17);
	pEph->ecc = ScaleDoubleULong(ulong, 34);	// .34
	ilong = GET_BITS(data[6], 0, 15);
	ilong  = (ilong << 18) | GET_UBITS(data[7], 14, 18);
	pEph->w = ScaleDoubleLong(ilong, 32) * PI;	// .32 * PI

	// Ephemeris II:
	ilong = GET_BITS(data[7], 0, 14);
	ilong  = (ilong << 19) | GET_UBITS(data[8], 13, 19);
	pEph->omega0 = ScaleDoubleLong(ilong, 32) * PI;	// .32 * PI
	ilong = GET_BITS(data[8], 0, 13);
	ilong  = (ilong << 20) | GET_UBITS(data[9], 12, 20);
	pEph->i0 = ScaleDoubleLong(ilong, 32) * PI;	// .32 * PI
	idata = (GET_BITS(data[9], 0, 12) << 7) | GET_UBITS(data[10], 25, 7);
	pEph->omega_dot = ScaleDouble(idata, 44) * PI;	// .44 * PI
	pEph->idot = ScaleDouble(GET_BITS(data[10], 10, 15), 44) * PI; // .44 * PI
	idata = (GET_BITS(data[10], 0, 10) << 6) | GET_UBITS(data[11], 26, 6);
	pEph->cis = ScaleDouble(idata, 30);	// .30
	pEph->cic = ScaleDouble(GET_BITS(data[11], 10, 16), 30); // .30
	idata = (GET_BITS(data[11], 0, 10) << 14) | GET_UBITS(data[12], 18, 14);
	pEph->crs = ScaleDouble(idata, 8);	// .8
	idata = (GET_BITS(data[12], 0, 18) << 6) | GET_UBITS(data[13], 26, 6);
	pEph->crc = ScaleDouble(idata, 8);	// .8
	pEph->cus = ScaleDouble(GET_BITS(data[13], 5, 21), 30); // .30
	idata = (GET_BITS(data[13], 0, 5) << 16) | GET_UBITS(data[14], 16, 16);
	pEph->cuc = ScaleDouble(idata, 30);	// .30


	// clock
	pEph->tgd = ScaleDouble(GET_BITS(data[17], 7, 12), 34); // .34
	pEph->toc = (int)GET_UBITS(data[14], 5, 11) * 300;
	idata = (GET_BITS(data[14], 0, 5) << 20) | GET_UBITS(data[15], 12, 20);
	pEph->af0 = ScaleDouble(idata, 34); // .34
	idata = (GET_BITS(data[15], 0, 12) << 10) | GET_UBITS(data[16], 22, 10);
	pEph->af1 = ScaleDouble(idata, 50); // .50
	pEph->af2 = ScaleDouble(GET_BITS(data[16], 11, 11), 66); // .66

	// calculate derived variables
	pEph->sqrtA = sqrt(pEph->axis);
	pEph->n = WGS_SQRT_GM / (pEph->sqrtA * pEph->axis) + pEph->delta_n;
	pEph->root_ecc = sqrt(1.0 - pEph->ecc * pEph->ecc);
	pEph->omega_t = pEph->omega0 - WGS_OMEGDOTE * pEph->toe;
	pEph->omega_delta = pEph->omega_dot - WGS_OMEGDOTE;
	return 1;
}

int DecodeBdsMidiAlm(const unsigned int data[SUBFRAME3_LENGTH])
{
	int svid, week;
	unsigned int type, toa;
	PMIDI_ALMANAC pAlm;
	int idata;

	svid = (int)GET_UBITS(data[1], 5, 6);
	pAlm = &g_BdsAlmanac[svid - 1];
	type = GET_UBITS(data[1], 3, 2);
	week = (int)((GET_UBITS(data[1], 0, 3) << 10) | GET_UBITS(data[2], 22, 10));
	toa = GET_UBITS(data[2], 14, 8) << 12;

	if (pAlm->week == week && pAlm->toa == toa)	// repeat almanac
		return svid;

	MutexTake(EphAlmMutex);

	pAlm->week = week;
	pAlm->toa = toa;
	pAlm->ecc = ScaleDoubleU(GET_UBITS(data[2], 3, 11), 16);
	idata = (GET_BITS(data[2], 0, 3) << 8) | GET_UBITS(data[3], 24, 8);
	pAlm->i0 = (((type == 1) ? 0 : 0.3) + ScaleDouble(idata, 14)) * PI;
	pAlm->sqrtA = ScaleDoubleU(GET_UBITS(data[3], 7, 17), 4);
	idata = (GET_BITS(data[3], 0, 7) << 9) | GET_UBITS(data[4], 23, 9);
	pAlm->omega0 = ScaleDouble(idata, 15) * PI;
	pAlm->omega_dot = ScaleDouble(GET_BITS(data[4], 12, 11), 33) * PI;
	idata = (GET_BITS(data[4], 0, 12) << 4) | GET_UBITS(data[5], 28, 4);
	pAlm->w = ScaleDouble(idata, 15) * PI;
	pAlm->M0 = ScaleDouble(GET_BITS(data[5], 12, 16), 15) * PI;
	pAlm->af0 = ScaleDouble(GET_BITS(data[5], 1, 11), 20);
	idata = (GET_BITS(data[5], 0, 1) << 9) | GET_UBITS(data[6], 23, 9);
	pAlm->af1 = ScaleDouble(idata, 37);
	pAlm->health = GET_UBITS(data[6], 21, 2);	// clock+B1C

	pAlm->flag = 1;

	// calculate derived variables
	pAlm->axis = pAlm->sqrtA * pAlm->sqrtA;
	pAlm->n = WGS_SQRT_GM / (pAlm->sqrtA * pAlm->axis);
	pAlm->root_ecc = sqrt(1.0 - pAlm->ecc * pAlm->ecc);
	pAlm->omega_t = pAlm->omega0 - WGS_OMEGDOTE * (pAlm->toa);
	pAlm->omega_delta = pAlm->omega_dot - WGS_OMEGDOTE;

	MutexGive(EphAlmMutex);
	return svid;
}
