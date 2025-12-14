//----------------------------------------------------------------------
// GpsFrame.c:
//   GPS frame sync and frame data decode
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "DataTypes.h"
#include "PlatformCtrl.h"
#include "TaskManager.h"
#include "ChannelManager.h"
#include "TimeManager.h"
#include "GlobalVar.h"
#include "SupportPackage.h"

#define MAX_GPS_TOW		100799
#define PAYLOAD_LENGTH 10
#define PACKAGE_LENGTH (sizeof(SYMBOL_PACKAGE) + sizeof(unsigned int)*(PAYLOAD_LENGTH))	// 3 variables + 10 payload

// word in data array with following order
#define WORD1  (data[9])
#define WORD2  (data[8])
#define WORD3  (data[7])
#define WORD4  (data[6])
#define WORD5  (data[5])
#define WORD6  (data[4])
#define WORD7  (data[3])
#define WORD8  (data[2])
#define WORD9  (data[1])
#define WORD10 (data[0])

typedef struct
{
	unsigned char toa;
	unsigned char health;
	unsigned short ecc;
	short delta_i;
	short omega_dot;
	signed short af0, af1;
	unsigned int sqrtA;
	int omega0;
	int w;
	int M0;
} RAW_ALMANAC, *PRAW_ALMANAC;

extern U32 EphAlmMutex;

static unsigned int RawAlmanacMask;
static RAW_ALMANAC RawAlmanac[32];
static unsigned int AlmValidMask = 0;
static unsigned int AlmHealthMask = 0;
static int AlmRefToa = -1, AlmRefWeek = -1;	// -1 means not valid

static void FillInBits(unsigned int* target, unsigned int* src, int number);
static int GetTowFromWord(unsigned int word);
static int GpsFrameDecode(void* Param);
static int DecodeGpsEphemeris(int svid, const unsigned int data[30]);
static int DecodeGpsAlmanac(int svid, const unsigned int data[10]);
//static int DecodeGpsHealthAS(const unsigned int data[10]);
static int DecodeGpsHealthWeek(const unsigned int data[10]);
static int ConvertAlmanac(int svid, int week);

extern BOOL GpsParityCheck(unsigned int word);

//*************** GPS data decode initialization ****************
// Parameters:
//   none
// Return value:
//   none
void GpsDecodeInit()
{
	RawAlmanacMask = 0;
}

//*************** GPS navigation data process ****************
//* do GPS LNAV frame sync and data collection
//* meaning of FrameStatus:
//* -1: frame sync not completed
//* 1~5: reserved for fast frame sync with current subframe id
//* 30: preamble found but frame sync not confirmed
//* 31~35: frame sync comfirmed and this value as recent decoded subframe id + 30
// Parameters:
//   pFrameInfo: pointer to frame info structure
//   DataForDecode: pointer to structure of symbols to be decoded
// Return value:
//   current millisecond count within current week (negative if unknown)
int GpsNavDataProc(PFRAME_INFO pFrameInfo, PDATA_FOR_DECODE DataForDecode)
{
	int fillin_count;
	int index, tow0, tow1, frame_id;
	int data_count = 32;	// DataStream contais 32bit data
	int BitCount = -1;
	U32 DataStream = DataForDecode->DataStream;
	PCHANNEL_STATE ChannelState = DataForDecode->ChannelState;
	unsigned int Package[PACKAGE_LENGTH];
	PSYMBOL_PACKAGE SymbolPackage = (PSYMBOL_PACKAGE)Package;

	// if not frame sync, find word sync by checking parity
	if (pFrameInfo->FrameStatus < 0)
	{
		while (data_count > 0)
		{
			// move bits in data0/data1 into data stream gather 32bit to do parity check
			fillin_count = 32 - (int)pFrameInfo->SymbolNumber;	// how many bit to complete 32bit
			if (fillin_count <= 0)
				fillin_count = 1;	// at least fill in one bit
			else if (fillin_count > data_count)
				fillin_count = data_count;	// at most fill in all bit
			// simplified shift, only consider last two 32bit without frame sync
			pFrameInfo->SymbolData[1] <<= fillin_count;
			pFrameInfo->SymbolData[1] |= (pFrameInfo->SymbolData[0] >> (30 - fillin_count));
			FillInBits(&pFrameInfo->SymbolData[0], &DataStream, fillin_count);
			data_count -= fillin_count;
			pFrameInfo->SymbolNumber += fillin_count;

			// check whether 32bit completed for word parity check
			if (pFrameInfo->SymbolNumber >= 32)
			{
				if (GpsParityCheck(pFrameInfo->SymbolData[0]))	// check parity of 32 bit word
				{
					/*// reserved for future fast frame sync
					if (EstimateTow >= 0 && (tow0 = GetTowFromWord(pFrameInfo->SymbolData[0])) >= 0)
					{
						if (tow0 == EstimateTow || tow0 == (EstimateTow + 1))	// estimate TOW is idential or within one subframe delay
						{
							pFrameInfo->tow = tow0;
							pFrameInfo->FrameStatus = 30;
							pFrameInfo->SymbolNumber = 62;
							// sync at HOW, fill in TLM, last two bit in HOW should be 0, so 1 means negative stream
							pFrameInfo->SymbolData[1] = (pFrameInfo->SymbolData[0] & 1) ? ~0x22c24838 : 0x22c24838;
							break;
						}
					}*/
					// check preamble, preamble include D29* and D30* are 10bit 0x8b or 0x374
					if ((pFrameInfo->SymbolData[0] >> 22) == 0x8b || (pFrameInfo->SymbolData[0] >> 22) == 0x374)
					{
						// frame sync with subframe id unknown
						pFrameInfo->FrameStatus = 30;
						// bits in SymbolData exceed 32 dropped, only keep TLM word in SymbolData[0]
						pFrameInfo->SymbolNumber = 32;
						break;
					}
				}
				// only maintain at most 2 words (plus D29* and D30* in previous word) when frame sync is not reached
				if (pFrameInfo->SymbolNumber > 62)
					pFrameInfo->SymbolNumber = 62;
			}
		}
	}

	// if frame sync done
	if (pFrameInfo->FrameStatus >= 30)
	{
		while (data_count > 0)
		{
			// first step: move in data
			// move bits in data0/data1 into data stream
			// if frame sync lost due to parity check fail etc., fill in all bits
			if (pFrameInfo->FrameStatus < 30)
				fillin_count = data_count;
			// if tow not get, calculate how many bit to complete 2 word
			else if (pFrameInfo->SymbolNumber < 62 && pFrameInfo->TimeTag < 0 && pFrameInfo->FrameStatus == 30)
				fillin_count = 62 - (int)pFrameInfo->SymbolNumber;
			// if frame id not get, calculate how many bit to complete 12 word
			else if (pFrameInfo->FrameStatus == 30)
				fillin_count = 362 - (int)pFrameInfo->SymbolNumber;
			else // calculate how many bit to complete 10 word
				fillin_count = 302 - (int)pFrameInfo->SymbolNumber;
			if (fillin_count > data_count)
				fillin_count = data_count;	// at most fill in all bits
			while (fillin_count >= 30)	// fill in whole word
			{
				for (index = 11; index > 0; index--)
					pFrameInfo->SymbolData[index] = pFrameInfo->SymbolData[index - 1];
				FillInBits(&pFrameInfo->SymbolData[0], &DataStream, 30);
				data_count -= 30;
				fillin_count -= 30;
				pFrameInfo->SymbolNumber += 30;
			}
			if (fillin_count > 0)	// fill remnant bits
			{
				for (index = 11; index > 0; index--)
				{
					pFrameInfo->SymbolData[index] <<= fillin_count;
					pFrameInfo->SymbolData[index] |= (pFrameInfo->SymbolData[index - 1] >> (30 - fillin_count));
				}
				FillInBits(&pFrameInfo->SymbolData[0], &DataStream, fillin_count);
			}
			data_count -= fillin_count;
			pFrameInfo->SymbolNumber += fillin_count;

			// second step: decode data bit
			// total 10 or 12 word get (current subframe and preamble and HOW word in next subframe)
			if (pFrameInfo->SymbolNumber == ((pFrameInfo->FrameStatus == 30) ? 362 : 302))
			{
				// unknown subframe id, check for next TLM and TOW continue
				if (pFrameInfo->FrameStatus == 30)
				{
					tow0 = pFrameInfo->TimeTag + 1;
					if (tow0 > MAX_GPS_TOW)
						tow0 = 0;
					tow1 = GetTowFromWord(pFrameInfo->SymbolData[0]);
					// preamble match and TOW continue, frame sync confirmed
					if (((pFrameInfo->SymbolData[1] >> 22) == 0x8b || (pFrameInfo->SymbolData[1] >> 22) == 0x374) && (tow1 == tow0))
					{
						// tow0 is the starting tow of next subframe
						pFrameInfo->TimeTag = tow0;
						// copy latest TLM to previous subframe TLM because first TLM may be missing when using TOW sync
						pFrameInfo->SymbolData[11] = pFrameInfo->SymbolData[1];
						// assign subframe id
						frame_id = (pFrameInfo->SymbolData[10] >> 8) & 0x7;
						if (pFrameInfo->SymbolData[10] & 0x40000000)	// contents of HOW XOR with D30
							frame_id ^= 7;
						pFrameInfo->FrameStatus = 30 + frame_id;
						// decode current subframe
						SymbolPackage->ChannelState = ChannelState;
						SymbolPackage->FrameInfo = pFrameInfo;
						SymbolPackage->PayloadLength = PAYLOAD_LENGTH;
						memcpy(SymbolPackage->Symbols, pFrameInfo->SymbolData + 2, sizeof(unsigned int) * PAYLOAD_LENGTH);
						AddToTask(TASK_POSTMEAS, GpsFrameDecode, SymbolPackage, PACKAGE_LENGTH);
						// drop current subframe
						pFrameInfo->SymbolNumber -= 300;
						BitCount = tow0 * 300 + pFrameInfo->SymbolNumber + data_count - 2;	// bit count of last decoded bit within the week
					}
					else // incorrect frame sync get, search for preamble again
					{
						// first try to find another preamble in following word
						for (index = 10; index >= 0; index--)
						{
							pFrameInfo->SymbolNumber -= 30;
							if ((pFrameInfo->SymbolData[index] >> 22) == 0x8b || (pFrameInfo->SymbolData[index] >> 22) == 0x374)
								break;
						}
						// if search fail or word has preamble failed to pass parity check, roll back to preamble search again
						if (index < 0 || !GpsParityCheck(pFrameInfo->SymbolData[index]))
						{
							pFrameInfo->FrameStatus = -1;
							pFrameInfo->SymbolNumber = 32;
							data_count = 0;
						}
						// TODO: if previouse frame sync using TLM/HOW has already use TOW to set receiver time, need to reset
						if (index > 0)	// found preamble again, set TOW
							pFrameInfo->TimeTag = GetTowFromWord(pFrameInfo->SymbolData[index - 1]);
						else
							pFrameInfo->TimeTag = -1;
					}
				}
				else if (pFrameInfo->FrameStatus > 30)	// already in frame sync, do data decode
				{
					// move tow to next subframe
					pFrameInfo->TimeTag ++;
					if (pFrameInfo->TimeTag > MAX_GPS_TOW)
						pFrameInfo->TimeTag = 0;
					// assign subframe id
					frame_id = (pFrameInfo->SymbolData[8] >> 8) & 0x7;
					if (pFrameInfo->SymbolData[8] & 0x40000000)	// contents of HOW XOR with D30
						frame_id ^= 7;
					pFrameInfo->FrameStatus = 30 + frame_id;
					// decode current subframe
					SymbolPackage->ChannelState = ChannelState;
					SymbolPackage->FrameInfo = pFrameInfo;
					SymbolPackage->PayloadLength = 10;
					memcpy(SymbolPackage->Symbols, pFrameInfo->SymbolData, sizeof(unsigned int) * PAYLOAD_LENGTH);
					AddToTask(TASK_POSTMEAS, GpsFrameDecode, SymbolPackage, PACKAGE_LENGTH);
					// drop current subframe
					pFrameInfo->SymbolNumber -= 300;
				}
				else
				{
					// should not go in here
//					DEBUG_MESSAGE(MSG_ERROR, ERR_INVALID_BRANCH);
				}
			}
			// TLM and HOW both get, calculate tow
			else if (pFrameInfo->SymbolNumber == 62 && pFrameInfo->FrameStatus == 30)
			{
				tow0 = GetTowFromWord(pFrameInfo->SymbolData[0]);
				/*if (tow0 < 0 || (EstimateTow >= 0 && !(tow0 == EstimateTow || tow0 == (EstimateTow + 1))))	// get TOW fail or not consistant with receiver time, drop frame sync
					pFrameInfo->FrameStatus = -1;
				else*/
				if (tow0 >= 0)
				{
					// set TOW
					pFrameInfo->TimeTag = tow0;
					// set polarity
					pFrameInfo->FrameFlag |= POLARITY_VALID;
					if ((pFrameInfo->SymbolData[0] & 0x3) == 0x0)
						pFrameInfo->FrameFlag &= ~NEGATIVE_STREAM;
					else
						pFrameInfo->FrameFlag |= NEGATIVE_STREAM;
					BitCount = tow0 * 300 + pFrameInfo->SymbolNumber + data_count - 2;	// bit count of last decoded bit within the week
				}
			}
		}
	}

	return BitCount * 20;
}

//*************** Shift bits from source to target ****************
//* source bits in src (MSB first)
//* data shifted in fill LSB of target, previous bits in target shift left
//* contents in src0|src1 also shifted left with MSBs move out
// Parameters:
//   target: pointer to target word
//   src: pointer to source data stream
//   number: number of bits to shift in
// Return value:
//   none
void FillInBits(unsigned int* target, unsigned int* src, int number)
{
	if (number == 32)
		*target = *src;
	else
	{
		*target <<= number;
		*target |= (*src >> (32 - number));
		*src <<= number;
	}
}

//*************** Get TOW from HOW word ****************
//* calculate TOW and return -1 under following condition
//*   last 2bit not equals 00 or 11
//*   parity check fail
//*   subframe number not in range 1~5
//*   TOW out of range
//* TOW will minus 1 because HOW holds start time of NEXT subframe
// Parameters:
//   word: HOW word
// Return value:
//   TOW value or -1
static int GetTowFromWord(unsigned int word)
{
	int tow;

	// check last 2bit of WORD2 to be all 0 or all 1
	if (((word & 0x3) == 1) || ((word & 0x3) == 2))
		return -1;

	// check parity
	if (!(GpsParityCheck(word)))
		return -1;

	// get data content
	if (word & 0x40000000)
		word ^= 0x3fffffff;

	// check subframe id within range 1~5
	tow = (word >> 8) & 0x7;
	if (tow < 1 || tow > 5)
		return -1;

	tow = (word >> 13) & 0x1ffff;
	// minus tow by one to get tow of current subframe
	if (tow == 0)
		tow = MAX_GPS_TOW;
	else if (tow > MAX_GPS_TOW)
		tow = -1;
	else
		tow --;

	return tow;
}

//*************** Task function to process one GPS subframe ****************
// Parameters:
//   Param: pointer to structure holding one subframe data
// Return value:
//   0
int GpsFrameDecode(void* Param)
{
	PSYMBOL_PACKAGE SymbolPackage = (PSYMBOL_PACKAGE)Param;
	PFRAME_INFO FrameInfo = SymbolPackage->FrameInfo;
	int Svid = SymbolPackage->ChannelState->Svid;
	int FrameID;
	unsigned int *data = SymbolPackage->Symbols;
	int i, Page, Week, StreamPolarity = 0;

	// restore contents d0~d29, put stream polarity in D29 indicate whether word contents identical to positive stream
	// StreamPolarity initialize with 0 means first word always has positive content
	for (i = 9; i >= 0; i --)
	{
		if (!GpsParityCheck((unsigned int)(data[i])))
			return 6;
		if (data[i] & 0x40000000)
			data[i] ^= 0x3fffffff;
		data[i] = (data[i] & 0x7fffffff) | StreamPolarity;
		StreamPolarity ^=(data[i] << 31);
	}

	FrameID = (data[8] >> 8) & 0x7;
	DEBUG_OUTPUT(OUTPUT_CONTROL(DATA_DECODE, INFO), "svid%2d decode subframe %d\n", Svid, FrameID);
	switch (FrameID)
	{
	case 1:
		memcpy(FrameInfo->FrameData, data, sizeof(unsigned int) * 10);
		FrameInfo->FrameFlag |= 1;
		Week = GET_UBITS(data[7], 20, 10);
		SetReceiverTime(SIGNAL_L1CA, Week + 2048, -1, 0);	// only set week number
		break;
	case 2:
		memcpy(FrameInfo->FrameData + 10, data, sizeof(unsigned int) * 10);
		FrameInfo->FrameFlag |= 2;
		break;
	case 3:
		memcpy(FrameInfo->FrameData + 20, data, sizeof(unsigned int) * 10);
		FrameInfo->FrameFlag |= 4;
		break;
	case 4:
	case 5:
		if (((data[7] >> 28) & 0x3) != 1)	// check DataID
			break;
		Page = (data[7] >> 22) & 0x3f;
		if (Page > 1 && Page <= 32)
			DecodeGpsAlmanac(Page, data);
		else if (Page == 56)
			;	// UTC & ionosphere
		else if (Page == 63)
			;	// DecodeGpsHealthAS(data);
		else if (Page == 51)
			DecodeGpsHealthWeek(data);
		break;
	}
	if ((FrameInfo->FrameFlag & 7) == 7)
	{
		DecodeGpsEphemeris(Svid, FrameInfo->FrameData);
		FrameInfo->FrameFlag &= ~7;
	}
	return 0;
}

//*************** Decode GPS ephemeris with subframe 1~3 ****************
// Parameters:
//   svid: GPS SVID ranging from 1 to 32
//   data: subframe data, each WORD in 30LSB of 32bit data content
// Return value:
//   1 if decode success or 0 if decode fail
int DecodeGpsEphemeris(int svid, const unsigned int data[30])
{
	PGNSS_EPHEMERIS pEph = &g_GpsEphemeris[svid-1];
	unsigned short iodc;
	unsigned char health;

	iodc = ((WORD3 << 2) & 0x300) | GET_UBITS(WORD8, 22, 8);
	if (pEph->flag == 1 && pEph->iodc == iodc)	// do not decode repeat ephemeris
		return 1;
	if (((iodc & 0xff) != GET_UBITS(data[17], 22, 8)) || ((iodc & 0xff) != GET_UBITS(data[20], 22, 8)))
		return 0;
	health = GET_UBITS(WORD3, 8, 6);
	if (health != 0)
		return 0;

	MutexTake(EphAlmMutex);

	// subframe 1:
	pEph->svid = svid - 1 + MIN_GPS_SAT_ID;
	pEph->health = health;
	pEph->flag = 1;
	pEph->iodc = iodc;
	pEph->week = 2048 + (int)GET_UBITS(WORD3, 20, 10);
	pEph->ura = GET_UBITS(WORD3, 14, 4);
	pEph->tgd = ScaleDouble(GET_BITS(WORD7, 6, 8), 31); // .31
	pEph->toc = (int)GET_UBITS(WORD8, 6, 16) * 16;
	pEph->af0 = ScaleDouble(GET_BITS(WORD10, 8, 22), 31); // .31
	pEph->af1 = ScaleDouble(GET_BITS(WORD9, 6, 16), 43); // .43
	pEph->af2 = ScaleDouble(GET_BITS(WORD9, 22, 8), 55); // .55

	// subframe 2:
	data += 10;
	pEph->iode2 = (unsigned char)(iodc & 0xff);
	pEph->crs = ScaleDouble(GET_BITS(WORD3, 6, 16), 5); // .5
	pEph->delta_n = ScaleDouble(GET_BITS(WORD4, 14, 16), 43) * PI; // .43 * PI
	pEph->M0 = ScaleDouble(((WORD4 << 18) & 0xff000000) | GET_UBITS(WORD5, 6, 24), 31) * PI; // .31 * PI
	pEph->cuc = ScaleDouble(GET_BITS(WORD6, 14, 16), 29); // .29
	pEph->ecc = ScaleDoubleU(((WORD6 << 18) & 0xff000000) | GET_UBITS(WORD7, 6, 24), 33); // .33
	pEph->cus = ScaleDouble(GET_BITS(WORD8, 14, 16), 29); // .29
	pEph->sqrtA = ScaleDoubleU(((WORD8 << 18) & 0xff000000) | GET_UBITS(WORD9, 6, 24), 19); // .19
	pEph->toe = (int)GET_UBITS(WORD10, 14, 16) * 16;

	// subframe 3:
	data += 10;
	pEph->iode3 = (unsigned char)(iodc & 0xff);
	pEph->cic = ScaleDouble(GET_BITS(WORD3, 14, 16), 29); // .29
	pEph->omega0 = ScaleDouble(((WORD3 << 18) & 0xff000000) | GET_UBITS(WORD4, 6, 24), 31) * PI; // .31 * PI
	pEph->cis = ScaleDouble(GET_BITS(WORD5, 14, 16), 29); // .29
	pEph->i0 = ScaleDouble(((WORD5 << 18) & 0xff000000) | GET_UBITS(WORD6, 6, 24), 31) * PI; // .31 * PI
	pEph->crc = ScaleDouble(GET_BITS(WORD7, 14, 16), 5); // .5
	pEph->w = ScaleDouble(((WORD7 << 18) & 0xff000000) | GET_UBITS(WORD8, 6, 24), 31) * PI; // .31 * PI
	pEph->omega_dot = ScaleDouble(GET_BITS(WORD9, 6, 24), 43) * PI; 
	pEph->idot = ScaleDouble(GET_BITS(WORD10, 8, 14), 43) * PI; // .43 * PI

	// calculate derived variables
	pEph->axis = pEph->sqrtA * pEph->sqrtA;
	pEph->n = WGS_SQRT_GM / (pEph->sqrtA * pEph->axis) + pEph->delta_n;
	pEph->root_ecc = sqrt(1.0 - pEph->ecc * pEph->ecc);
	pEph->omega_t = pEph->omega0 - WGS_OMEGDOTE * pEph->toe;
	pEph->omega_delta = pEph->omega_dot - WGS_OMEGDOTE;

	MutexGive(EphAlmMutex);
	return 1;
}

//*************** Decode subframe 4/5 to a fixed point GPS almanac structure  ****************
// Parameters:
//   svid: GPS SVID ranging from 1 to 32
//   data: subframe data, each WORD in 30LSB of 32bit data content
// Return value:
//   svid if decode success or 0 if decode fail
int DecodeGpsAlmanac(int svid, const unsigned int data[10])
{
	PRAW_ALMANAC pAlm = &RawAlmanac[svid - 1];
	unsigned char toa = GET_UBITS(WORD4, 22, 8);
	int omega0 = GET_BITS(WORD7, 6, 24);

	if (pAlm->toa == toa && pAlm->omega0 == omega0)	// same toa and omega0, repeat almanac from same or different satellite
		return 0;
	pAlm->toa = toa;
	pAlm->health = GET_UBITS(WORD5, 6, 8);
	pAlm->ecc = GET_UBITS(WORD3, 6, 16);
	pAlm->delta_i = GET_BITS(WORD4, 6, 16);
	pAlm->omega_dot = GET_BITS(WORD5, 14, 16);
	pAlm->sqrtA = GET_UBITS(WORD6, 6, 24);
	pAlm->omega0 = omega0;
	pAlm->w = GET_BITS(WORD8, 6, 24);
	pAlm->M0 = GET_BITS(WORD9, 6, 24);
	pAlm->af0 = (short)GET_BITS(WORD10, 22, 8) << 3;
	pAlm->af0 |= GET_UBITS(WORD10, 8, 3);
	pAlm->af1 = (short)GET_BITS(WORD10, 11, 11);
//	AlmRefWeek = g_ReceiverInfo.ReceiverTime->GpsWeekNumber;
//	AlmRefToa = pAlm->toa;

	if (AlmRefWeek >= 0 && AlmRefToa >= 0)	// has valid week and toa
	{
		if (AlmRefToa == pAlm->toa)
			ConvertAlmanac(svid, AlmRefWeek);	// update almanac if toa match
		else	// toa change, wait next matching page 51
		{
			AlmRefWeek = AlmRefToa = -1;
			AlmValidMask = (1 << (svid - 1));
		}
	}
	else	// set valid mask and wait next page 51
		AlmValidMask |= (1 << (svid - 1));

	return svid;
}

/*int DecodeGpsHealthAS(const unsigned int data[10])
{
	int i;

	AlmHealthMask = 0;	// reset all health flag
	// get MSB as overall health flag
	for (i = 23; i < 31; i ++)	// i = svid - 2
	{
		if ((data[7-i/4] & (1 << (29 - (i & 3) * 6))) == 0)
			AlmHealthMask |= (1 << (i + 1));
	}
	return 0;
}*/

//*************** Decode page 51 data to get health and week number  ****************
// Parameters:
//   data: subframe data holding page 51
// Return value:
//   svid if decode success or 0 if decode fail
int DecodeGpsHealthWeek(const unsigned int data[10])
{
	int i;
	int toa, week, receiver_week = g_ReceiverInfo.ReceiverTime->GpsWeekNumber;

	toa = GET_UBITS(WORD3, 14, 8);
	week = GET_UBITS(WORD3, 6, 8);

	// low 8bit week number match decoded value (allow +-1 for broadcast almanac of previous week or next week)
	if (week == (receiver_week & 0xff))
		week = receiver_week;
	else if (week == ((receiver_week - 1) & 0xff))
		week = receiver_week - 1;
	else if (week == ((receiver_week + 1) & 0xff))
		week = receiver_week + 1;
	else
	{
		AlmRefWeek = -1;	// invalidate reference week
		return 0;
	}
	AlmRefWeek = week;
	AlmRefToa = toa;

/*	// get MSB as overall health flag
	for (i = 0; i < 24; i ++)	// i = svid - 1
	{
		if ((data[6-i/4] & (1 << (29 - (i & 3) * 6))) == 0)
			AlmHealthMask |= (1 << i);
	}*/

	// convert all pending almanac
	for (i = 0; i < 32; i ++)
		if (AlmValidMask & (1 << i))
			ConvertAlmanac(i + 1, AlmRefWeek);
	AlmValidMask = 0;

	return 0;
}

//*************** Decode a fixed point almanac into global almanac structure ****************
// Parameters:
//   svid: GPS SVID ranging from 1 to 32
//   week: current GPS week
// Return value:
//   svid if decode success or 0 if decode fail
int ConvertAlmanac(int svid, int week)
{
	PRAW_ALMANAC pRawAlm = &RawAlmanac[svid - 1];
	PMIDI_ALMANAC pAlm = &g_GpsAlmanac[svid - 1];
//	KINEMATIC_INFO PosVel;

	if (pAlm->flag == 1 && (pAlm->toa >> 12) == pRawAlm->toa)	// same toa, skip update
		return 0;

	MutexTake(EphAlmMutex);

	pAlm->health = pRawAlm->health;
	pAlm->svid = (unsigned char)svid;
	pAlm->toa = (unsigned long)pRawAlm->toa << 12;
	pAlm->week = week;
	pAlm->M0 = ScaleDouble(pRawAlm->M0, 23) * PI;
	pAlm->ecc = ScaleDoubleU(pRawAlm->ecc, 21);
	pAlm->sqrtA = ScaleDoubleU(pRawAlm->sqrtA, 11);
	pAlm->omega0 = ScaleDouble(pRawAlm->omega0, 23) * PI;
	pAlm->i0 = (0.3 + ScaleDouble(pRawAlm->delta_i, 19)) * PI;
	pAlm->w = ScaleDouble(pRawAlm->w, 23) * PI;
	pAlm->omega_dot = ScaleDouble(pRawAlm->omega_dot, 38) * PI;
	pAlm->af0 = ScaleDouble(pRawAlm->af0, 20);
	pAlm->af1 = ScaleDouble(pRawAlm->af1, 38);

	pAlm->flag = 1;

	// calculate derived variables
	pAlm->axis = pAlm->sqrtA * pAlm->sqrtA;
	pAlm->n = WGS_SQRT_GM / (pAlm->sqrtA * pAlm->axis);
	pAlm->root_ecc = sqrt(1.0 - pAlm->ecc * pAlm->ecc);
	pAlm->omega_t = pAlm->omega0 - WGS_OMEGDOTE * (pAlm->toa);
	pAlm->omega_delta = pAlm->omega_dot - WGS_OMEGDOTE;
//	SatPosSpeedMidiAlm(g_ReceiverInfo.ReceiverTime->GpsWeekNumber, g_ReceiverInfo.ReceiverTime->GpsMsCount / 1000, pAlm, &PosVel);

	MutexGive(EphAlmMutex);
	return svid;
}
