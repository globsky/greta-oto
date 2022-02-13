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
#include "GlobalVar.h"
#include "SupportPackage.h"

//#define MSG_INFO 0
//#define MSG_WARNING 0
//#define MSG_ERROR 1
//#define DEBUG_OUTPUT(enable, ...) if(enable) printf(__VA_ARGS__)

#define TLM_WORD (data[9])
#define HOW_WORD (data[8])
#define WORD3  (data[7])
#define WORD4  (data[6])
#define WORD5  (data[5])
#define WORD6  (data[4])
#define WORD7  (data[3])
#define WORD8  (data[2])
#define WORD9  (data[1])
#define WORD10 (data[0])

static void FillInBits(unsigned int *target, unsigned int *src0, unsigned int *src1, int number);
static int GetTowFromWord(unsigned int word);
static int GpsFrameDecode(PCHANNEL_STATUS pChannelStatus, unsigned int *data);
static int DecodeGpsEphemeris(PGNSS_EPHEMERIS pEph, const unsigned int SubframeData[3][10]);

extern BOOL GpsParityCheck(unsigned int word);

//*************** GPS Frame sync process ****************
//* assume maximum epoch interval is 1280ms, which is 64bit
// Parameters:
//   pChannelStatus: pointer to channel status structure
//   data_count: number of data in data stream
//   data0: first 32bit data stream
//   data1: second 32bit data stream (data stream from MSB to LSB)
//   EstimateTow: estimate current TOW, -1 if invalid
// Return value:
//   none
void GpsFrameSync(PCHANNEL_STATUS pChannelStatus, int data_count, unsigned int data0, unsigned data1, int EstimateTow)
{
	int fillin_count;
	int index, tow0, tow1;
	PGPS_FRAME_INFO pFrameInfo = (PGPS_FRAME_INFO)(pChannelStatus->FrameInfo);

	// if not frame sync, find word sync by checking parity
	if (pFrameInfo->FrameStatus < 0)
	{
		while (data_count > 0)
		{
			// move bits in data0/data1 into data stream gather 32bit to do parity check
			fillin_count = 32 - (int)pFrameInfo->NavBitNumber;	// how many bit to complete 32bit
			if (fillin_count <= 0)
				fillin_count = 1;	// at least fill in one bit
			else if (fillin_count > data_count)
				fillin_count = data_count;	// at most fill in all bit
			// simplified shift, only consider last two 32bit without frame sync
			pFrameInfo->NavDataStream[1] <<= fillin_count;
			pFrameInfo->NavDataStream[1] |= (pFrameInfo->NavDataStream[0] >> (30 - fillin_count));
			FillInBits(&pFrameInfo->NavDataStream[0], &data0, &data1, fillin_count);
			data_count -= fillin_count;
			pFrameInfo->NavBitNumber += fillin_count;

			// check whether 32bit completed for word parity check
			if (pFrameInfo->NavBitNumber >= 32)
			{
				if (GpsParityCheck(pFrameInfo->NavDataStream[0]))	// check parity of 32 bit word
				{
					// check HOW word first
					if (EstimateTow >= 0 && (tow0 = GetTowFromWord(pFrameInfo->NavDataStream[0])) >= 0)
					{
						if (tow0 == EstimateTow || tow0 == (EstimateTow + 1))	// estimate TOW is idential or within one subframe delay
						{								
							pFrameInfo->tow = tow0;
							pFrameInfo->FrameStatus = 30;
							pFrameInfo->NavBitNumber = 62;
							// sync at HOW, fill in TLM, last two bit in HOW should be 0, so 1 means negative stream
							pFrameInfo->NavDataStream[1] = (pFrameInfo->NavDataStream[0] & 1) ? ~0x22c24838 : 0x22c24838;
							break;
						}
					}
					// check preamble, preamble include D29* and D30* are 10bit 0x8b or 0x374
					if ((pFrameInfo->NavDataStream[0] >> 22) == 0x8b || (pFrameInfo->NavDataStream[0] >> 22) == 0x374)
					{
						// frame sync with subframe id unknown
						pFrameInfo->FrameStatus = 30;
						// force drop bit exceed 32
						pFrameInfo->NavBitNumber = 32;
						break;
					}
				}
				// only maintain at most 2 words (plus D29* and D30* in previous word) when frame sync is not reached
				if (pFrameInfo->NavBitNumber > 62)
					pFrameInfo->NavBitNumber = 62;
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
			else if (pFrameInfo->NavBitNumber < 62 && pFrameInfo->tow < 0 && pFrameInfo->FrameStatus == 30) 
				fillin_count = 62 - (int)pFrameInfo->NavBitNumber;
			// if frame id not get, calculate how many bit to complete 12 word
			else if (pFrameInfo->FrameStatus == 30)
				fillin_count = 362 - (int)pFrameInfo->NavBitNumber;
			else // calculate how many bit to complete 10 word
				fillin_count = 302 - (int)pFrameInfo->NavBitNumber;
			if (fillin_count > data_count)
				fillin_count = data_count;	// at most fill in all bits
			while (fillin_count >= 30)	// fill in whole word
			{
				for (index = 11; index > 0; index --)
					pFrameInfo->NavDataStream[index] = pFrameInfo->NavDataStream[index-1];
				FillInBits(&pFrameInfo->NavDataStream[0], &data0, &data1, 30);
				data_count -= 30;
				fillin_count -= 30;
				pFrameInfo->NavBitNumber += 30;
			}
			if (fillin_count > 0)	// fill remnant bits
			{
				for (index = 11; index > 0; index --)
				{
					pFrameInfo->NavDataStream[index] <<= fillin_count;
					pFrameInfo->NavDataStream[index] |= (pFrameInfo->NavDataStream[index-1] >> (30 - fillin_count));
				}
				FillInBits(&pFrameInfo->NavDataStream[0], &data0, &data1, fillin_count);
			}
			data_count -= fillin_count;
			pFrameInfo->NavBitNumber += fillin_count;

			// second step: decode data bit
			// total 10 or 12 word get (current subframe and preamble and HOW word in next subframe)
			if (pFrameInfo->NavBitNumber == ((pFrameInfo->FrameStatus == 30) ? 362 : 302))
			{
				// unknown subframe id, check for next TLM and TOW continue
				if (pFrameInfo->FrameStatus == 30)
				{
					tow0 = pFrameInfo->tow + 1;
					if (tow0 > MAX_GPS_TOW)
						tow0 = 0;
					tow1 = GetTowFromWord(pFrameInfo->NavDataStream[0]);
					// preamble match and TOW continue, frame sync confirmed
					if (((pFrameInfo->NavDataStream[1] >> 22) == 0x8b || (pFrameInfo->NavDataStream[1] >> 22) == 0x374) && (tow1 == tow0))
					{
						// tow0 is the starting tow of next subframe
						pFrameInfo->tow = tow0;
//						DEBUG_OUTPUT(MSG_INFO, "Frame sync get for PRN%d, tow=%d\n", pChannelStatus->svid, pFrameInfo->tow);
						// copy latest TLM to previous subframe TLM because first TLM may be missing when using TOW sync
						pFrameInfo->NavDataStream[11] = pFrameInfo->NavDataStream[1];
						// decode current subframe
						pFrameInfo->FrameStatus = 30 + GpsFrameDecode(pChannelStatus, pFrameInfo->NavDataStream + 2);
						// drop current subframe
						pFrameInfo->NavBitNumber -= 300;
					}
					else // incorrect frame sync get, search for preamble again
					{
						// first try to find another preamble in following word
						for (index = 10; index >= 0; index --)
						{
							pFrameInfo->NavBitNumber -= 30;
							if ((pFrameInfo->NavDataStream[index] >> 22) == 0x8b ||	(pFrameInfo->NavDataStream[index] >> 22) == 0x374)
								break;
						}
						// if search fail or word has preamble failed to pass parity check, roll back to preamble search again
						if (index < 0 || !GpsParityCheck(pFrameInfo->NavDataStream[index]))
						{
							pFrameInfo->FrameStatus = -1;
							pFrameInfo->NavBitNumber = 32;
							data_count = 0;
						}
						// TODO: if previouse frame sync using TLM/HOW has already use TOW to set receiver time, need to reset
						if (index > 0)	// found preamble again, set TOW
							pFrameInfo->tow = GetTowFromWord(pFrameInfo->NavDataStream[index-1]);
						else
							pFrameInfo->tow = -1;
					}
				}
				else if (pFrameInfo->FrameStatus > 30)	// already in frame sync, do data decode
				{
					// move tow to next subframe
					pFrameInfo->tow ++;
					if (pFrameInfo->tow > MAX_GPS_TOW)
						pFrameInfo->tow = 0;
					// decode current subframe
					pFrameInfo->FrameStatus = 30 + GpsFrameDecode(pChannelStatus, pFrameInfo->NavDataStream);
					// drop current subframe
					pFrameInfo->NavBitNumber -= 300;
				}
				else
				{
					// should not go in here
//					DEBUG_MESSAGE(MSG_ERROR, ERR_INVALID_BRANCH);
				}
			}
			// TLM and HOW both get, calculate tow
			else if ( pFrameInfo->NavBitNumber == 62 &&  pFrameInfo->FrameStatus == 30)
			{
				tow0 = GetTowFromWord(pFrameInfo->NavDataStream[0]);
				if (tow0 < 0 || (EstimateTow >= 0 && !(tow0 == EstimateTow || tow0 == (EstimateTow + 1))))	// get TOW fail or not consistant with receiver time, drop frame sync
					pFrameInfo->FrameStatus = -1;
				else
				{
					// set TOW
					pFrameInfo->tow = tow0;
					// set polarity
					pFrameInfo->FrameFlag |= POLARITY_VALID;
					if ((pFrameInfo->NavDataStream[0] & 0x3) == 0x0)
						pFrameInfo->FrameFlag &= ~NEGATIVE_STREAM;
					else
						pFrameInfo->FrameFlag |= NEGATIVE_STREAM;
				}
			}
		}
	}
}

//*************** Shift bits from source to target ****************
//* source bits in src0|src1 (MSB in MSB of src0, LSB in LSB of src1)
//* data shifted in fill LSB of target, previous bits in target shift left
//* contents in src0|src1 also shifted left with MSBs move out
// Parameters:
//   target: pointer to target word
//   src0: pointer to source 0 word
//   src1: pointer to source 1 word
//   number: number of bits to shift in
// Return value:
//   none
void FillInBits(unsigned int *target, unsigned int *src0, unsigned int *src1, int number)
{
	if (number == 32)
	{
		*target = *src0;
		*src0 = *src1;
	}
	else
	{
		*target <<= number;
		*target |= (*src0 >> (32 - number));
		*src0 <<= number; *src0 |= (*src1 >> (32 - number));
		*src1 <<= number;
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

/*******************************************
* Decode one subframe
* return 1~5 for parity check error
* return 6 for subframe id decode error
* return 31~35 for decode success
*********************************************/
//*************** Decode GPS subframe ****************
//* for subframe 1/2/3, collect all three subframes and decode ephemeris
//* for subframe 4/5 decode almanac, UTC etc.
// Parameters:
//   pChannelStatus: pointer to channel status
//   data: 10 WORD navigation data stream
// Return value:
//   1~5 for subframe id successfully decoded, 6 for unknown subframe
#define SUBFRAME_VALID(pChannel, id) (pChannel->FrameFlag & SUBFRAME##id##_VALID)
int GpsFrameDecode(PCHANNEL_STATUS pChannelStatus, unsigned int *data)
{
	int frame_id = GET_UBITS(HOW_WORD, 8, 3);
	int i;
	int svid = pChannelStatus->svid;
	PGPS_FRAME_INFO pFrameInfo = (PGPS_FRAME_INFO)(pChannelStatus->FrameInfo);

	if (HOW_WORD & 0x40000000)
		frame_id ^= 0x7;

//	printf("SV%02d Frame%d decode\n", pChannelStatus->svid, frame_id);
	if ((TLM_WORD >> 22) == 0x374)
	{
		pFrameInfo->FrameFlag |= NEGATIVE_STREAM;
		pFrameInfo->FrameFlag |= POLARITY_VALID;
	}
	else if ((TLM_WORD >> 22) == 0x8b)
	{
		pFrameInfo->FrameFlag &= ~NEGATIVE_STREAM;
		pFrameInfo->FrameFlag |= POLARITY_VALID;
	}
	else	//preamble fail
		return 6;

	if (frame_id < 1 || frame_id > 5)
		return 6;

	// restore contents d1~d24, by XOR D30* with D1~D24
	for (i = 9; i >= 0; i --)
	{
		if (!GpsParityCheck((unsigned int)(data[i])))
			return 6;
		if (data[i] & 0x40000000)
			data[i] ^= 0x3fffffc0;
	}

	// if match existing ephemeris, for subframe 1/2/3, do not do data decode
	if (frame_id == 1)
	{
		pFrameInfo->iodc = ((data[7] << 2) & 0x300) | GET_UBITS(data[2], 22, 8);
		if (g_GpsEphemeris[svid-1].flag && pFrameInfo->iodc == g_GpsEphemeris[svid-1].iodc)
			return 1;
	}
	else if (frame_id == 2)
	{
		pFrameInfo->iode2 = GET_UBITS(data[7], 22, 8);
		if (g_GpsEphemeris[svid-1].flag && pFrameInfo->iode2 == g_GpsEphemeris[svid-1].iode2)
			return 2;
	}
	else if (frame_id == 3)
	{
		pFrameInfo->iode3 = GET_UBITS(data[0], 22, 8);
		if (g_GpsEphemeris[svid-1].flag && pFrameInfo->iode3 == g_GpsEphemeris[svid-1].iode3)
			return 3;
	}
	else // frame_id == 4 or 5
	{
		// decode almanac/UTC parameter etc. here
		return frame_id;
	}

	// fill in corresponding data array for subframe 1~3 and set flag
	if (frame_id <= 3)
	{
		pFrameInfo->FrameFlag |= (SUBFRAME1_VALID << (frame_id - 1));
		memcpy(pFrameInfo->SubframeData[frame_id-1], data, sizeof(unsigned int) * 10);
	}
	// if subframe 1 get, decode week number
	if (frame_id == 1 && (g_ReceiverInfo.PosFlag & GPS_WEEK_VALID) == 0)
	{
		g_ReceiverInfo.WeekNumber = 2048 + GET_UBITS(data[7], 20, 10);
		g_ReceiverInfo.PosFlag |= (GPS_WEEK_VALID);// | CALC_INVIEW_GPS);
	}
	
	// check if iodc, iode2 and iode3 are not identical, only leave recent subframe data
	if ((SUBFRAME_VALID(pFrameInfo, 1) && SUBFRAME_VALID(pFrameInfo, 2) && (pFrameInfo->iodc & 0xff) != pFrameInfo->iode2) ||
		(SUBFRAME_VALID(pFrameInfo, 1) && SUBFRAME_VALID(pFrameInfo, 3) && (pFrameInfo->iodc & 0xff) != pFrameInfo->iode3) ||
		(SUBFRAME_VALID(pFrameInfo, 2) && SUBFRAME_VALID(pFrameInfo, 3) && pFrameInfo->iode2 != pFrameInfo->iode3))
	{
		pFrameInfo->FrameFlag &= ~ALL_SUBFRAME_VALID;//clear 
		pFrameInfo->FrameFlag |= (SUBFRAME1_VALID << (frame_id - 1));
	}
	if ((pFrameInfo->FrameFlag & ALL_SUBFRAME_VALID) == ALL_SUBFRAME_VALID)
	{
		if (1)//pChannelStatus->cn0 > 3800 || EphemerisValid(&g_GpsEphemeris[svid-1], &g_GpsAlmanac[svid-1], pFrameInfo->SubframeData))	//check ephemeris valid
		{
			if (g_GpsEphemeris[svid-1].flag == 0 || pFrameInfo->iodc != g_GpsEphemeris[svid-1].iodc)
			{
				g_GpsEphemeris[svid-1].svid = svid;
				DecodeGpsEphemeris(&g_GpsEphemeris[svid-1], pFrameInfo->SubframeData);
			}
		}
		else
		{
			pChannelStatus->ChannelErrorFlag |= CHANNEL_ERR_XCORR;
		}

		pFrameInfo->FrameFlag &= ~ALL_SUBFRAME_VALID;	
	}

	return frame_id;
}

//*************** Decode GPS frame data to get ephemeris ****************
// Parameters:
//   pEph: pointer to ephemeris structure
//   SunframeData: array of 3x10 WORD of subframe1/2/3
// Return value:
//   1 if decode success, otherwise 0
int DecodeGpsEphemeris(PGNSS_EPHEMERIS pEph, const unsigned int SubframeData[3][10])
{
	const int *data;

	// subframe 1:
	data = (const int *)SubframeData[0];
	pEph->health = GET_UBITS(WORD3, 8, 6);
	if (pEph->health != 0)
	{
		pEph->flag = 0;
		return 0;
	}
	pEph->flag = 1;

	pEph->iodc = ((WORD3 << 2) & 0x300) | GET_UBITS(WORD8, 22, 8);
	pEph->week = 1024 + (int)GET_UBITS(WORD3, 20, 10);
	if (pEph->week < 500)	// week number for 2009/3/28 and later
		pEph->week += 1024;
	pEph->ura = GET_UBITS(WORD3, 14, 4);
	pEph->tgd = ScaleDouble(GET_BITS(WORD7, 6, 8), 31); // .31
	pEph->toc = (int)GET_UBITS(WORD8, 6, 16) * 16;
	pEph->af0 = ScaleDouble(GET_BITS(WORD10, 8, 22), 31); // .31
	pEph->af1 = ScaleDouble(GET_BITS(WORD9, 6, 16), 43); // .43
	pEph->af2 = ScaleDouble(GET_BITS(WORD9, 22, 8), 55); // .55

	// subframe 2:
	data = (int *)SubframeData[1];
	pEph->iode2 = GET_UBITS(WORD3, 22, 8);
	pEph->crs = ScaleDouble(GET_BITS(WORD3, 6, 16), 5); // .5
	pEph->delta_n = ScaleDouble(GET_BITS(WORD4, 14, 16), 43) * PI; // .43 * PI
	pEph->M0 = ScaleDouble(((WORD4 << 18) & 0xff000000) | GET_UBITS(WORD5, 6, 24), 31) * PI; // .31 * PI
	pEph->cuc = ScaleDouble(GET_BITS(WORD6, 14, 16), 29); // .29
	pEph->ecc = ScaleDoubleU(((WORD6 << 18) & 0xff000000) | GET_UBITS(WORD7, 6, 24), 33); // .33
	pEph->cus = ScaleDouble(GET_BITS(WORD8, 14, 16), 29); // .29
	pEph->sqrtA = ScaleDoubleU(((WORD8 << 18) & 0xff000000) | GET_UBITS(WORD9, 6, 24), 19); // .19
	pEph->toe = (int)GET_UBITS(WORD10, 14, 16) * 16;

	// subframe 3:
	data = (int *)SubframeData[2];
	pEph->iode3 = GET_UBITS(WORD10, 22, 8);
	pEph->cic = ScaleDouble(GET_BITS(WORD3, 14, 16), 29); // .29
	pEph->omega0 = ScaleDouble(((WORD3 << 18) & 0xff000000) | GET_UBITS(WORD4, 6, 24), 31) * PI; // .31 * PI
	pEph->cis = ScaleDouble(GET_BITS(WORD5, 14, 16), 29); // .29
	pEph->i0 = ScaleDouble(((WORD5 << 18) & 0xff000000) | GET_UBITS(WORD6, 6, 24), 31) * PI; // .31 * PI
	pEph->crc = ScaleDouble(GET_BITS(WORD7, 14, 16), 5); // .5
	pEph->w = ScaleDouble(((WORD7 << 18) & 0xff000000) | GET_UBITS(WORD8, 6, 24), 31) * PI; // .31 * PI
	pEph->omega_dot = ScaleDouble(GET_BITS(WORD9, 6, 24), 43) * PI; 
	pEph->idot = ScaleDouble(GET_BITS(WORD10, 8, 14), 43) * PI; // .43 * PI

	// calculate derived variables
	pEph->axis_dot = 0.;
	pEph->axis = pEph->sqrtA * pEph->sqrtA;
	pEph->n = WGS_SQRT_GM / (pEph->sqrtA * pEph->axis) + pEph->delta_n;
	pEph->root_ecc = sqrt(1.0 - pEph->ecc * pEph->ecc);
	pEph->omega_t = pEph->omega0 - WGS_OMEGDOTE * pEph->toe;
	pEph->omega_delta = pEph->omega_dot - WGS_OMEGDOTE;
	return 1;
}
